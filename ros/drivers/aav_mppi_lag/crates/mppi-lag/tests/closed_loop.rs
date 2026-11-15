//! Closed loop, no ROS: MPPI drives a pretend car around a circle and an S-curve, and
//! backwards: a straight reverse, a path with a cusp (forward, stop, reverse), and the
//! test lot's turn back (reverse, stop, forward), from a stop and arriving off the path;
//! and a path drawn off a lane's centre, kept inside the lane by `LaneCritic`.
//!
//! The pretend car is ego plus the lag model, stepped every 0.01 s (the truth). MPPI ticks
//! at 20 Hz, sees the car's pose, speed and measured actuators (as Gazebo gives them), and
//! plans with either the same lag model (lag-aware) or the lag-free one Nav2 assumes.
//! Each run prints its numbers: `cargo test -p mppi-lag --test closed_loop -- --nocapture`.

use mppi_lag::lag::Actuators;
use mppi_lag::{
    CarModel, Command, Critic, Driven, Edges, Ego, Follower, FollowerSettings, GlobalPath,
    IdealModel, Inversion, LagModel, Lane, LanePoint, Observation, Outcome, PathAngleMode, Pose,
    Settings, Tick,
};

const PERIOD: f64 = 0.05;
const PHYSICS: f64 = 0.01;

fn critics() -> Vec<Critic> {
    // nav2.yaml's list and its two changes from Nav2's defaults.
    mppi_lag::critics::NAMES
        .iter()
        .filter_map(|n| Critic::by_name(n))
        .map(|c| match c {
            Critic::GoalAngle(mut g) => {
                g.threshold = 2.5;
                Critic::GoalAngle(g)
            }
            Critic::PathAlign(mut p) => {
                p.use_path_orientations = true;
                Critic::PathAlign(p)
            }
            other => other,
        })
        .collect()
}

thread_local! {
    /// The steering's control cost weight for this test's runs (`Settings::steer_gamma`):
    /// None, gamma's (as nav2.yaml); the `ours` stack's 0.05 in the calm tests. Per test
    /// thread, so tests running side by side don't see each other's.
    static STEER_GAMMA: std::cell::Cell<Option<f64>> = const { std::cell::Cell::new(None) };
}

/// The `ours` stack's steering control cost (`mppi_lag_steer_gamma:=0.05`, TODO 168).
const CALM: f64 = 0.05;

fn settings(speed: f64) -> Settings {
    Settings {
        steer_gamma: STEER_GAMMA.with(std::cell::Cell::get),
        batch_size: 1000,
        vx_max: speed,
        seed: 7,
        // Fixed: each thread draws its own noise, so the core count changed the result
        // (16 on a Mac, 4 on CI).
        threads: 4,
        noise_correlation: std::env::var("CORR")
            .ok()
            .and_then(|c| c.parse().ok())
            .unwrap_or(Settings::default().noise_correlation),
        ..Settings::default()
    }
}

/// A path through `points` (x, y), headings along it, every ~0.05 m (as the planner's).
fn path_through(f: impl Fn(f64) -> (f64, f64), length: f64) -> Vec<Pose> {
    let n = (length / 0.05) as usize;
    let pts: Vec<(f64, f64)> = (0..=n).map(|i| f(i as f64 / n as f64)).collect();
    pts.iter()
        .enumerate()
        .map(|(i, &(x, y))| {
            let (a, b) = if i + 1 < pts.len() {
                (pts[i], pts[i + 1])
            } else {
                (pts[i - 1], pts[i])
            };
            Pose::new(x, y, (b.1 - a.1).atan2(b.0 - a.0))
        })
        .collect()
}

/// Three quarters of a circle of radius `r`, anticlockwise from (0, 0) facing east.
fn circle(r: f64) -> Vec<Pose> {
    let sweep = 1.5 * std::f64::consts::PI;
    path_through(
        |t| ((t * sweep).sin() * r, r - (t * sweep).cos() * r),
        sweep * r,
    )
}

/// An S: y = a sin(2 pi x / wavelength) for one and a half waves.
fn s_curve(a: f64, wavelength: f64) -> Vec<Pose> {
    let length = 1.5 * wavelength;
    path_through(
        |t| {
            let x = t * length;
            (x, a * (std::f64::consts::TAU * x / wavelength).sin())
        },
        length * 1.1,
    )
}

/// 10 m east, a quarter turn left of radius `r`, 10 m north.
fn corner(r: f64) -> Vec<Pose> {
    let arc = std::f64::consts::FRAC_PI_2 * r;
    let total = 20.0 + arc;
    path_through(
        |t| {
            let d = t * total;
            if d < 10.0 {
                (d, 0.0)
            } else if d < 10.0 + arc {
                let a = (d - 10.0) / r;
                (10.0 + r * a.sin(), r - r * a.cos())
            } else {
                (10.0 + r, r + d - 10.0 - arc)
            }
        },
        total,
    )
}

#[derive(Debug)]
struct Run {
    reached: bool,
    time: f64,
    mean_cross: f64,
    max_cross: f64,
    /// The car's speed every tick (m/s, - reversing).
    speeds: Vec<f64>,
    /// Where it was every tick.
    poses: Vec<Pose>,
    /// Where it stood once stopped after the follower said done (ego ramping it down).
    rest: Pose,
    /// How the follower ended the path (`None`: still going at the time limit).
    outcome: Option<Outcome>,
}

/// How a run may drive: forward only (nav2.yaml as it is), or reversing as
/// `navigation.launch.py reversing:=true` sets MPPI up: vx_min -max_reverse_speed,
/// PathAngleCritic mode 2, no PreferForwardCritic, and the path followed stretch by stretch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Way {
    Forward,
    Reversing,
}

const MAX_REVERSE_SPEED: f64 = 0.5;

fn drive<M: CarModel + Clone>(controller: M, truth: LagModel, poses: &[Pose], speed: f64) -> Run {
    drive_way(controller, truth, poses, speed, Way::Forward)
}

fn drive_way<M: CarModel + Clone>(
    controller: M,
    truth: LagModel,
    poses: &[Pose],
    speed: f64,
    way: Way,
) -> Run {
    drive_from(controller, truth, poses, speed, way, poses[0], 0.0)
}

/// As [`drive_way`], with the car starting at `start`, rolling at `rolling` m/s.
fn drive_from<M: CarModel + Clone>(
    controller: M,
    truth: LagModel,
    poses: &[Pose],
    speed: f64,
    way: Way,
    start: Pose,
    rolling: f64,
) -> Run {
    let ego = Ego::default();
    let plant = Driven { ego, car: truth };
    let (mppi, critics, follow) = set_up(way, settings(speed));
    let mut follower = Follower::new(controller, ego, mppi, critics, follow, Vec::new());
    let mut car = plant.at_rest(start, rolling);
    let path = GlobalPath::new(poses.to_vec()).expect("a path");
    let judge = path.clone();
    let mut command = Command::default();
    let mut crosses = Vec::new();
    let mut speeds = Vec::new();
    let mut trace = Vec::new();
    let mut t = 0.0;
    let mut first = true;
    while t < 120.0 {
        follower.observe(&Observation {
            pose: plant.pose(&car),
            speed: plant.speed(&car),
            steering: Some(plant.steering(&car)),
            throttle: Some(car.car.throttle),
            brake: Some(car.car.brake),
        });
        if first {
            follower.follow(path.clone(), false);
            first = false;
        }
        match follower.tick(t, None) {
            Some(Tick::Drive(report)) => command = report.command,
            Some(Tick::Done(outcome)) => {
                // Done: the follower stops sending; ego brings the car to rest.
                let stop = Command {
                    speed: 0.0,
                    ..command
                };
                for _ in 0..(8.0 / PHYSICS) as usize {
                    plant.step(&mut car, stop, PHYSICS);
                }
                let mut run = finish(outcome == Outcome::Succeeded, t, &crosses, speeds, trace);
                run.rest = plant.pose(&car);
                run.outcome = Some(outcome);
                return run;
            }
            None => {}
        }
        for _ in 0..(PERIOD / PHYSICS).round() as usize {
            plant.step(&mut car, command, PHYSICS);
        }
        t += PERIOD;
        speeds.push(plant.speed(&car));
        trace.push(plant.pose(&car));
        if t > 2.0 {
            crosses.push(judge.foot(&plant.pose(&car)).cross_track.abs());
        }
    }
    finish(false, t, &crosses, speeds, trace)
}

/// MPPI's settings, critics and the follower's for `way`, from forward-only `mppi`.
fn set_up(way: Way, mppi: Settings) -> (Settings, Vec<Critic>, FollowerSettings) {
    match way {
        Way::Forward => (mppi, critics(), FollowerSettings::default()),
        Way::Reversing => (
            Settings {
                vx_min: -MAX_REVERSE_SPEED,
                ..mppi
            },
            critics()
                .into_iter()
                .filter(|c| c.name() != "PreferForwardCritic")
                .map(|c| match c {
                    Critic::PathAngle(mut a) => {
                        a.mode = PathAngleMode::ConsiderFeasiblePathOrientations;
                        Critic::PathAngle(a)
                    }
                    other => other,
                })
                .collect(),
            FollowerSettings {
                inversion: Some(Inversion::default()),
                ..FollowerSettings::default()
            },
        ),
    }
}

fn finish(reached: bool, time: f64, crosses: &[f64], speeds: Vec<f64>, poses: Vec<Pose>) -> Run {
    let n = crosses.len().max(1) as f64;
    Run {
        reached,
        time,
        mean_cross: crosses.iter().sum::<f64>() / n,
        max_cross: crosses.iter().copied().fold(0.0, f64::max),
        rest: poses.last().copied().unwrap_or_default(),
        speeds,
        poses,
        outcome: None,
    }
}

/// `length` m straight back from (0, 0), the car facing east all the way (the planner's
/// poses face the car's way: on a reversing stretch they point back along the path).
fn straight_back(length: f64) -> Vec<Pose> {
    let n = (length / 0.05) as usize;
    (0..=n)
        .map(|i| Pose::new(-(i as f64) * 0.05, 0.0, 0.0))
        .collect()
}

/// A three-point turn's first two legs: `ahead` m east, a cusp, then backwards round a
/// quarter circle of radius `r`, steering left, to end facing south (the car backs up
/// to the north-west).
fn forward_then_back(ahead: f64, r: f64) -> Vec<Pose> {
    let n = (ahead / 0.05) as usize;
    let mut poses: Vec<Pose> = (0..=n)
        .map(|i| Pose::new(i as f64 * 0.05 * ahead / (n as f64 * 0.05), 0.0, 0.0))
        .collect();
    let arc = std::f64::consts::FRAC_PI_2 * r;
    let m = (arc / 0.05) as usize;
    poses.extend((1..=m).map(|i| {
        let phi = std::f64::consts::FRAC_PI_2 * i as f64 / m as f64;
        Pose::new(ahead - r * phi.sin(), r - r * phi.cos(), -phi)
    }));
    poses
}

fn ideal_of(truth: &LagModel) -> IdealModel {
    IdealModel {
        wheelbase: truth.wheelbase,
        max_steering_angle: truth.max_steering_angle,
        // Ego's ramps are already in front of it (Driven); the car itself is instant.
        accel: 100.0,
        decel: 100.0,
    }
}

/// The car with slow actuators: a sluggish linkage and dead times (a stand-in for a car
/// whose real numbers come in worse than vehicle.yaml's guesses).
fn slow_car() -> LagModel {
    LagModel {
        actuators: Actuators {
            steer_rate: 0.5,
            steer_lag: 0.3,
            steer_dead_time: 0.2,
            throttle_dead_time: 0.1,
            brake_dead_time: 0.1,
            ..Actuators::default()
        },
        ..LagModel::default()
    }
}

fn compare(name: &str, truth: LagModel, poses: &[Pose], speed: f64) -> (Run, Run) {
    let aware = drive(truth, truth, poses, speed);
    let free = drive(ideal_of(&truth), truth, poses, speed);
    println!(
        "{name} at {speed} m/s: lag-aware {:.3} m mean, {:.3} m worst, {:.1} s, reached {} | \
         lag-free {:.3} m mean, {:.3} m worst, {:.1} s, reached {}",
        aware.mean_cross,
        aware.max_cross,
        aware.time,
        aware.reached,
        free.mean_cross,
        free.max_cross,
        free.time,
        free.reached
    );
    (aware, free)
}

/// Much slower than vehicle.yaml's guesses: where the lag decides who stays on the path.
fn very_slow_car() -> LagModel {
    LagModel {
        actuators: Actuators {
            steer_rate: 0.3,
            steer_lag: 0.5,
            steer_dead_time: 0.3,
            throttle_dead_time: 0.2,
            brake_dead_time: 0.2,
            ..Actuators::default()
        },
        ..LagModel::default()
    }
}

/// Open loop: from the same state, predict 2.8 s of a weaving command with each model.
/// Returns the worst miss (m) of the lag model and of the lag-free one.
fn prediction_miss(truth: LagModel) -> (f64, f64) {
    let ego = Ego::default();
    let plant = Driven { ego, car: truth };
    let cmd = |t: f64| Command::new(1.5, 0.4 * (t * 1.5).sin());
    let mut car = plant.at_rest(Pose::default(), 0.0);
    let mut t = 0.0;
    for _ in 0..100 {
        for _ in 0..5 {
            plant.step(&mut car, cmd(t), PHYSICS);
        }
        t += PERIOD;
    }
    let mut lagged = car;
    let free_model = Driven {
        ego,
        car: ideal_of(&truth),
    };
    let mut free = free_model.at_rest(plant.pose(&car), plant.speed(&car));
    free.ego = car.ego;
    let mut worst = (0.0f64, 0.0f64);
    for _ in 0..56 {
        let c = cmd(t);
        for _ in 0..5 {
            plant.step(&mut car, c, PHYSICS);
        }
        plant.step(&mut lagged, c, PERIOD);
        free_model.step(&mut free, c, PERIOD);
        t += PERIOD;
        let p = plant.pose(&car);
        worst.0 = worst.0.max(p.distance(&plant.pose(&lagged)));
        worst.1 = worst.1.max(p.distance(&free_model.pose(&free)));
    }
    worst
}

/// Every case, printed: `cargo test --release -p mppi-lag --test closed_loop explore --
/// --ignored --nocapture` (CORR=0.0 for Nav2's white noise).
#[test]
#[ignore]
fn explore() {
    for (name, truth) in [
        ("vehicle.yaml", LagModel::default()),
        ("slow", slow_car()),
        ("very slow", very_slow_car()),
    ] {
        let (lag, free) = prediction_miss(truth);
        println!(
            "{name}: 2.8 s prediction misses by {lag:.3} m (lag model), {free:.3} m (lag-free)"
        );
        for speed in [1.0, 2.0, 3.0] {
            compare(&format!("{name}, circle r 5"), truth, &circle(5.0), speed);
            compare(
                &format!("{name}, S 2 x 16"),
                truth,
                &s_curve(2.0, 16.0),
                speed,
            );
            compare(&format!("{name}, corner r 4"), truth, &corner(4.0), speed);
        }
    }
}

#[test]
fn the_lag_model_predicts_the_car() {
    let (lag, free) = prediction_miss(slow_car());
    assert!(lag < 0.03, "{lag}");
    assert!(free > 0.3, "{free}");
}

#[test]
fn circle_at_vehicle_yaml_lags() {
    let (aware, free) = compare("circle r 5", LagModel::default(), &circle(5.0), 1.0);
    assert!(aware.reached && free.reached);
    assert!(aware.mean_cross < 0.05, "{aware:?}");
    assert!(aware.max_cross < 0.15, "{aware:?}");
}

#[test]
fn s_curve_at_vehicle_yaml_lags() {
    let (aware, free) = compare("S 2 x 16", LagModel::default(), &s_curve(2.0, 16.0), 2.0);
    assert!(aware.reached && free.reached);
    assert!(aware.mean_cross < 0.08, "{aware:?}");
    assert!(aware.max_cross < 0.2, "{aware:?}");
}

#[test]
fn very_slow_actuators_corner_lag_aware_holds_the_path() {
    // The lag-free one gets round a single corner too (0.04 m mean at 1-8 threads; it only
    // lost the path at 16): the S-curve below is where it fails. Here, aware is the closer.
    let (aware, free) = compare("corner r 4, very slow", very_slow_car(), &corner(4.0), 2.0);
    assert!(aware.reached, "{aware:?}");
    assert!(aware.mean_cross < 0.05, "{aware:?}");
    assert!(aware.max_cross < 0.2, "{aware:?}");
    assert!(aware.mean_cross < free.mean_cross, "{aware:?} vs {free:?}");
}

#[test]
fn very_slow_actuators_s_curve_only_lag_aware_holds_the_path() {
    let (aware, free) = compare(
        "S 2 x 16, very slow",
        very_slow_car(),
        &s_curve(2.0, 16.0),
        2.0,
    );
    assert!(aware.reached, "{aware:?}");
    assert!(aware.max_cross < 0.2, "{aware:?}");
    assert!(
        aware.mean_cross < 0.2 * free.mean_cross,
        "{aware:?} vs {free:?}"
    );
}

#[test]
fn straight_reverse() {
    let truth = LagModel::default();
    let run = drive_way(truth, truth, &straight_back(8.0), 1.0, Way::Reversing);
    let slowest = run.speeds.iter().copied().fold(0.0, f64::min);
    println!(
        "straight back 8 m: {:.3} m mean, {:.3} m worst, {:.1} s, reached {}, fastest back {slowest:.2} m/s",
        run.mean_cross, run.max_cross, run.time, run.reached
    );
    assert!(run.reached, "{:?}", run.poses.last());
    assert!(run.speeds.iter().all(|&v| v < 0.05), "never forward");
    assert!(
        slowest < -0.3 && slowest > -0.55,
        "backs up within max_reverse_speed: {slowest}"
    );
    assert!(
        run.mean_cross < 0.05 && run.max_cross < 0.15,
        "{:.3} {:.3}",
        run.mean_cross,
        run.max_cross
    );
}

#[test]
fn forward_stop_reverse_through_a_cusp() {
    // r 5 m, and the planner's own 3.6 m. Before the control cost knew the noise was
    // correlated, at r 4 m the car fell 0.6 m outside the bend after the cusp and
    // stopped short.
    for r in [5.0, 3.6] {
        forward_stop_reverse(r);
    }
}

fn forward_stop_reverse(r: f64) {
    let truth = LagModel::default();
    let run = drive_way(
        truth,
        truth,
        &forward_then_back(6.0, r),
        1.0,
        Way::Reversing,
    );
    let fastest = run.speeds.iter().copied().fold(0.0, f64::max);
    let slowest = run.speeds.iter().copied().fold(0.0, f64::min);
    println!(
        "6 m ahead, cusp, quarter circle r {r} back: {:.3} m mean, {:.3} m worst, {:.1} s, \
         reached {}, speeds {slowest:.2} .. {fastest:.2} m/s",
        run.mean_cross, run.max_cross, run.time, run.reached
    );
    assert!(run.reached, "{:?}", run.poses.last());
    assert!(fastest > 0.5, "went forward first: {fastest}");
    assert!(slowest < -0.3, "then backwards: {slowest}");
    // Forward first, then backwards: the speed changes sign once, through a stop.
    let first_back = run
        .speeds
        .iter()
        .position(|&v| v < -0.05)
        .expect("reversed");
    assert!(run.speeds[..first_back].iter().all(|&v| v > -0.05));
    assert!(
        run.speeds[first_back..].iter().all(|&v| v < 0.05),
        "no going forward again"
    );
    // Looser than going forward: the path bends at once after the cusp, and the car
    // starts from a stop with its wheels straight.
    assert!(
        run.mean_cross < 0.06 && run.max_cross < 0.15,
        "{:.3} {:.3}",
        run.mean_cross,
        run.max_cross
    );
}

/// The test lot's turn back (14 Nov), as our lattice planner draws it with reversing on:
/// from (35, 42) facing east, back round the planner's 3.572 m radius until facing
/// 63 degrees (a lattice heading), a cusp, then forward round the same radius to face
/// west, and 8 m on. A pose every 0.25 m, as the lattice's primitives give them (the
/// paths above have one every 0.05 m).
fn turn_back() -> Vec<Pose> {
    const R: f64 = 3.572;
    const STEP: f64 = 0.25;
    const CUSP_YAW: f64 = 1.1071;
    let pi = std::f64::consts::PI;
    let arc = |from: f64, to: f64| {
        let n = ((to - from) * R / STEP).ceil() as usize;
        (1..=n).map(move |i| from + (to - from) * i as f64 / n as f64)
    };
    // Backwards, steering right: the centre is on the car's right.
    let (bx, by) = (35.0, 42.0 - R);
    let mut poses = vec![Pose::new(35.0, 42.0, 0.0)];
    poses.extend(arc(0.0, CUSP_YAW).map(|a| Pose::new(bx - R * a.sin(), by + R * a.cos(), a)));
    // Forwards, steering left: the centre is on the car's left.
    let cusp = poses[poses.len() - 1];
    let (fx, fy) = (cusp.x - R * CUSP_YAW.sin(), cusp.y + R * CUSP_YAW.cos());
    poses.extend(arc(CUSP_YAW, pi).map(|a| Pose::new(fx + R * a.sin(), fy - R * a.cos(), a)));
    let turned = poses[poses.len() - 1];
    poses.extend((1..=32).map(|i| Pose::new(turned.x - f64::from(i) * STEP, turned.y, pi)));
    poses
}

/// The turn back from a stop on the path's start (TODO 174). Before the control cost
/// knew the noise was correlated, MPPI steered too little backwards, stopped beside the
/// cusp and gave up there (Stuck), as `mppi_lag` did in the simulator.
#[test]
fn turn_back_through_a_cusp() {
    let truth = LagModel::default();
    let run = drive_way(truth, truth, &turn_back(), 1.0, Way::Reversing);
    let backed = run.speeds.iter().copied().fold(0.0, f64::min);
    println!(
        "turn back: {:?} in {:.1} s, {:.3} m mean, {:.3} m worst, backed at {backed:.2} m/s",
        run.outcome, run.time, run.mean_cross, run.max_cross
    );
    assert_eq!(run.outcome, Some(Outcome::Succeeded), "{:?}", run.rest);
    assert!(run.time < 40.0, "{:.1} s", run.time);
    // Backwards first (never forward before the cusp), then forwards.
    let first_forward = run
        .speeds
        .iter()
        .position(|&v| v > 0.05)
        .expect("forward after the cusp");
    assert!(run.speeds[..first_forward].iter().any(|&v| v < -0.3));
    assert!(run.mean_cross < 0.3 && run.max_cross < 0.8, "{run:?}");
}

/// The same, with the car arriving as after the lot's leg before: still rolling on at
/// 0.5 m/s, 0.5 m and 11 degrees off the path's start. MPPI then stops beside the cusp,
/// outside the inversion tolerances (0.5 m) but within the goal's (1 m): the stretch
/// counts as done there, as the path would at its goal.
#[test]
fn turn_back_arriving_rolling_and_off_the_path() {
    let truth = LagModel::default();
    let poses = turn_back();
    let start = Pose::new(poses[0].x - 0.4, poses[0].y - 0.3, -0.2);
    let run = drive_from(truth, truth, &poses, 1.0, Way::Reversing, start, 0.5);
    println!(
        "turn back, arriving rolling: {:?} in {:.1} s, {:.3} m mean, {:.3} m worst",
        run.outcome, run.time, run.mean_cross, run.max_cross
    );
    assert_eq!(run.outcome, Some(Outcome::Succeeded), "{:?}", run.rest);
    assert!(run.time < 45.0, "{:.1} s", run.time);
}

/// The goal at speed (TODO 152): 80 m straight, asked for 2.5 m/s. The follower says done
/// within xy_goal_tolerance (1 m); the car must then stop near the goal, not run on past
/// it. Here the critics see nav2.yaml's 3 m of path, not the launch file's 8.4 m at
/// 2.5 m/s, and MPPI holds the car near 1.3 m/s (the wiggle tests below drive at 2.4), so
/// this is the goal at that speed, not the campus's 2.5 m/s (the simulator, 14 Nov: stood
/// 0.47 m short).
#[test]
fn the_goal_at_top_speed_stops_near_it() {
    const GOAL_X: f64 = 80.0;
    let poses = path_through(|u| (GOAL_X * u, 0.0), GOAL_X);
    let truth = LagModel::default();
    let run = drive(truth, truth, &poses, 2.5);
    let past = run.rest.x - GOAL_X;
    let top = run.speeds.iter().copied().fold(0.0, f64::max);
    let at_done = run.speeds.last().copied().unwrap_or(0.0);
    println!(
        "goal at 2.5 m/s: reached {} in {:.1} s, top {top:.2} m/s, {at_done:.2} m/s when done, \
         stood {past:+.2} m past the goal",
        run.reached, run.time
    );
    assert!(run.reached);
    assert!(past.abs() < 1.0, "stood {past:+.2} m from the goal");
}

/// The car's outline, padded (vehicle.yaml's 2 x 1 m body, 0.05 m padding).
const OUTLINE: [(f64, f64); 4] = [(1.85, 0.55), (1.85, -0.55), (-0.25, -0.55), (-0.25, 0.55)];

/// A lane along y = 0 from x = -5 to 45 m: its lines `soft` m and kerbs `hard` m either side.
fn lane(soft: f64, hard: f64) -> Lane {
    Lane::new((0..=100).map(|i| LanePoint {
        x: f64::from(i) * 0.5 - 5.0,
        y: 0.0,
        edges: Edges::from_numbers(hard, hard, soft, soft),
    }))
    .expect("a lane")
}

/// The furthest the outline goes left of y = 0 (m), at `pose`.
fn outline_left(pose: &Pose) -> f64 {
    let (s, c) = pose.yaw.sin_cos();
    OUTLINE
        .iter()
        .map(|&(x, y)| pose.y + s * x + c * y)
        .fold(f64::NEG_INFINITY, f64::max)
}

/// Follow `poses` with the car's outline and, if given, LaneCritic keeping to `lane`.
fn drive_in_lane(poses: &[Pose], lane: Option<Lane>) -> Run {
    let truth = LagModel::default();
    let ego = Ego::default();
    let plant = Driven { ego, car: truth };
    let mut critics = critics();
    if lane.is_some() {
        critics.extend(Critic::by_name("LaneCritic"));
    }
    let mut follower = Follower::new(
        truth,
        ego,
        settings(1.0),
        critics,
        FollowerSettings::default(),
        OUTLINE.to_vec(),
    );
    follower.set_lane(lane);
    let mut car = plant.at_rest(poses[0], 0.0);
    let path = GlobalPath::new(poses.to_vec()).expect("a path");
    let (mut command, mut t, mut first) = (Command::default(), 0.0, true);
    let mut trace = Vec::new();
    while t < 60.0 {
        follower.observe(&Observation {
            pose: plant.pose(&car),
            speed: plant.speed(&car),
            steering: Some(plant.steering(&car)),
            throttle: Some(car.car.throttle),
            brake: Some(car.car.brake),
        });
        if first {
            follower.follow(path.clone(), false);
            first = false;
        }
        match follower.tick(t, None) {
            Some(Tick::Drive(report)) => command = report.command,
            Some(Tick::Done(outcome)) => {
                let reached = outcome == Outcome::Succeeded;
                return finish(reached, t, &[], Vec::new(), trace);
            }
            None => {}
        }
        for _ in 0..(PERIOD / PHYSICS).round() as usize {
            plant.step(&mut car, command, PHYSICS);
        }
        t += PERIOD;
        trace.push(plant.pose(&car));
    }
    finish(false, t, &[], Vec::new(), trace)
}

/// The lane keeps the car in (TODO 169): a path drawn 1 m left of a 2.6 m lane's centre
/// (lines 1.3 m, kerbs 1.8 m either side), so following it would put the outline 0.25 m
/// over the line. With LaneCritic the car rides the line instead, never near the kerb;
/// without it, it follows the path over the line.
#[test]
fn a_path_off_centre_is_followed_inside_the_lane() {
    const SOFT: f64 = 1.3;
    const HARD: f64 = 1.8;
    // From the lane's centre, over to y = 1 within 6 m, then 24 m straight on.
    let poses = path_through(
        |u| {
            let x = 30.0 * u;
            let y = if x < 6.0 {
                0.5 - 0.5 * (std::f64::consts::PI * x / 6.0).cos()
            } else {
                1.0
            };
            (x, y)
        },
        30.0,
    );
    let free = drive_in_lane(&poses, None);
    let kept = drive_in_lane(&poses, Some(lane(SOFT, HARD)));
    let worst = |run: &Run| run.poses.iter().map(outline_left).fold(f64::MIN, f64::max);
    let settled = |run: &Run| {
        let tail = &run.poses[run.poses.len() * 2 / 3..];
        tail.iter().map(|p| p.y).sum::<f64>() / tail.len() as f64
    };
    println!(
        "path 1 m off centre, lines at {SOFT} m: without the lane the outline reaches {:.3} m \
         (settles at y {:.2}), with it {:.3} m (y {:.2}); reached {} / {}",
        worst(&free),
        settled(&free),
        worst(&kept),
        settled(&kept),
        free.reached,
        kept.reached
    );
    assert!(free.reached && kept.reached, "{free:?} {kept:?}");
    assert!(
        worst(&free) > SOFT + 0.15,
        "the path itself crosses the line"
    );
    assert!(
        worst(&kept) < SOFT + 0.1,
        "rides the line, at most a little over"
    );
    assert!(worst(&kept) < HARD - 0.3, "never near the kerb");
    // Still on the path's side of the lane: the critic doesn't drag it to the centre.
    assert!(settled(&kept) > 0.5, "{}", settled(&kept));
}

/// The campus tour's "round the bend" junction (14 Nov night, TODO 167), in the map frame:
/// our planner's path there, as the run saved it (about 1 m apart), from 75 m into the leg.
/// East along the road, a right turn of about 4.5 m radius into the junction, south, a
/// bend back west. In the simulator the car stood at (224.25, -160.90) for 70 s: not
/// mppi_lag, but the geofence (200 m round the start; the car stood 196.9 m out, the path
/// went on to 216 m) holding every program, forward and back.
const JUNCTION: [(f64, f64); 51] = [
    (211.82, -160.62),
    (212.71, -161.13),
    (213.68, -161.48),
    (214.69, -161.63),
    (215.69, -161.66),
    (216.70, -161.67),
    (217.70, -161.67),
    (218.70, -161.67),
    (219.70, -161.67),
    (220.70, -161.67),
    (221.70, -161.68),
    (222.71, -161.76),
    (223.70, -162.01),
    (224.59, -162.48),
    (225.33, -163.13),
    (225.91, -163.92),
    (226.37, -164.77),
    (226.80, -165.63),
    (227.23, -166.49),
    (227.66, -167.35),
    (228.09, -168.20),
    (228.52, -169.06),
    (228.93, -169.95),
    (229.26, -170.90),
    (229.45, -171.92),
    (229.63, -172.93),
    (229.96, -173.89),
    (230.38, -174.77),
    (230.80, -175.63),
    (231.23, -176.49),
    (231.66, -177.35),
    (232.09, -178.20),
    (232.52, -179.06),
    (232.93, -179.95),
    (233.26, -180.90),
    (233.41, -181.91),
    (233.42, -182.92),
    (233.31, -183.93),
    (233.00, -184.90),
    (232.49, -185.76),
    (231.80, -186.46),
    (230.98, -187.00),
    (230.12, -187.45),
    (229.27, -187.88),
    (228.41, -188.31),
    (227.55, -188.74),
    (226.70, -189.17),
    (225.84, -189.60),
    (224.98, -190.02),
    (224.12, -190.45),
    (223.27, -190.88),
];

/// `points` joined by straight lines, a pose every ~0.05 m (as the planner's).
fn polyline(points: &[(f64, f64)]) -> Vec<Pose> {
    let mut lengths = vec![0.0];
    for w in points.windows(2) {
        let step = (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1);
        lengths.push(lengths.last().copied().unwrap_or(0.0) + step);
    }
    let total = lengths.last().copied().unwrap_or(0.0);
    path_through(
        |u| {
            let d = u * total;
            let i = lengths
                .partition_point(|&l| l <= d)
                .clamp(1, points.len() - 1);
            let (a, b) = (points[i - 1], points[i]);
            let f = (d - lengths[i - 1]) / (lengths[i] - lengths[i - 1]).max(1e-9);
            (a.0 + f * (b.0 - a.0), a.1 + f * (b.1 - a.1))
        },
        total,
    )
}

/// mppi_lag takes the junction as the run had it: reversing on, asked for the campus's
/// 2.5 m/s, the car arriving rolling, 0.6 m left of the path and 15 degrees across it (the
/// sim's truth 3 s before the hold). It drives the turn and the bend to the end, without
/// backing up, close to the path.
#[test]
fn campus_junction_round_the_bend() {
    let truth = LagModel::default();
    let poses = polyline(&JUNCTION);
    let start = Pose::new(212.30, -160.70, -0.48);
    let run = drive_from(truth, truth, &poses, 2.5, Way::Reversing, start, 1.2);
    let backed = run.speeds.iter().filter(|&&v| v < -0.05).count() as f64 * PERIOD;
    println!(
        "campus junction: {:?} in {:.1} s, {:.3} m mean, {:.3} m worst, {backed:.1} s backing",
        run.outcome, run.time, run.mean_cross, run.max_cross
    );
    assert_eq!(run.outcome, Some(Outcome::Succeeded), "{:?}", run.rest);
    assert!(run.time < 60.0, "{:.1} s", run.time);
    assert!(run.max_cross < 0.3, "{:.3} m", run.max_cross);
    assert!(backed < 0.5, "backed for {backed:.1} s");
}

// The wiggle (TODO 168). On the campus road at 2.4 m/s mppi_lag's steering changed side
// about once a second and the car swung 0.2 m (the sim, 14 Nov). The cause: the steering
// noise (0.15 rad, sized for 1 m/s) spreads the 2.8 s futures v² as wide, so at 2.5 m/s
// the softmax rested on a handful of samples and each tick's average came out different.
// These runs drive as the launch file sets the node up: 2000 samples, and the path seen
// 2.8 s ahead at the top speed (the tests above see nav2.yaml's 3 m, so they never drive
// much faster than 1.2 m/s). `examples/wiggle.rs` prints every suspect.

/// How much a run wiggled, counted as `tools/drive_metrics.py` does.
#[derive(Debug)]
struct Wiggle {
    reached: bool,
    /// m: the mean distance off the path.
    mean_cross: f64,
    /// m: side to side about the car's own 4 s smoothed track (RMS), `swing_m`.
    swing: f64,
    /// m/s²: the sideways push less its 4 s average (RMS), `sway_mps2`.
    sway: f64,
    /// The wheels' angle changing side, a second (under 0.02 rad: straight).
    flips: f64,
    /// m/s: the mean speed.
    speed: f64,
    /// rad: the wheels' angle, RMS.
    steer_rms: f64,
}

/// s: drive_metrics.py's WEAVE_WIDTH.
const WEAVE: f64 = 4.0;

/// Each value's mean over the `half` values either side.
fn moving_average(v: &[f64], half: usize) -> Vec<f64> {
    (0..v.len())
        .map(|i| {
            let (a, b) = (i.saturating_sub(half), (i + half + 1).min(v.len()));
            v[a..b].iter().sum::<f64>() / (b - a) as f64
        })
        .collect()
}

/// RMS of `v` less its own `WEAVE` s moving average.
fn weave(v: &[f64]) -> f64 {
    let avg = moving_average(v, (WEAVE / PERIOD / 2.0) as usize);
    let n = v.len().max(1) as f64;
    (v.iter()
        .zip(&avg)
        .map(|(s, a)| (s - a).powi(2))
        .sum::<f64>()
        / n)
        .sqrt()
}

/// The side offset from the track's own 4 s smoothed track, weaved.
fn swing(track: &[Pose]) -> f64 {
    let half = (WEAVE / PERIOD / 2.0) as usize;
    let xs = moving_average(&track.iter().map(|p| p.x).collect::<Vec<_>>(), half);
    let ys = moving_average(&track.iter().map(|p| p.y).collect::<Vec<_>>(), half);
    let offsets: Vec<f64> = (1..track.len().saturating_sub(1))
        .filter_map(|i| {
            let (dx, dy) = (xs[i + 1] - xs[i - 1], ys[i + 1] - ys[i - 1]);
            let l = dx.hypot(dy);
            (l > 1e-6).then(|| ((track[i].y - ys[i]) * dx - (track[i].x - xs[i]) * dy) / l)
        })
        .collect();
    weave(&offsets)
}

/// Drive `poses` asking for `speed`, set up as navigation.launch.py sets the node up, with
/// `steer_std_speed` (0: the noise until 15 Nov). `sim_like`: the pose the follower sees
/// is 0.15 s old and off by an EKF-like error (AR(1), 8 cm and 1.2 degrees, 2 s, a new
/// value every 0.2 s), as the sim's saved campus runs had it.
fn wiggle(poses: &[Pose], speed: f64, way: Way, steer_std_speed: f64, sim_like: bool) -> Wiggle {
    let truth = LagModel::default();
    let ego = Ego::default();
    let plant = Driven { ego, car: truth };
    let (mppi, critics, follow) = set_up(
        way,
        Settings {
            batch_size: 2000,
            vx_max: speed,
            steer_std_speed,
            steer_gamma: STEER_GAMMA.with(std::cell::Cell::get),
            seed: 7,
            threads: 4,
            ..Settings::default()
        },
    );
    // vehicle.mppi_lag_params: 2.8 s at vx_max, and a fifth more.
    let follow = FollowerSettings {
        prune_distance: (2.8 * speed * 1.2).max(3.0),
        ..follow
    };
    let mut follower = Follower::new(truth, ego, mppi, critics, follow, Vec::new());
    let mut car = plant.at_rest(poses[0], 0.0);
    let path = GlobalPath::new(poses.to_vec()).expect("a path");
    let mut rng = mppi_lag::rng::Rng::new(99);
    let (mut error, mut next_fix) = ((0.0, 0.0, 0.0), 0.0);
    let mut seen: Vec<Pose> = Vec::new();
    let (mut crosses, mut steers, mut pushes, mut speeds) = (vec![], vec![], vec![], vec![]);
    let mut track = Vec::new();
    let (mut command, mut t, mut reached) = (Command::default(), 0.0, false);
    let mut yaw = poses[0].yaw;
    while t < 120.0 {
        seen.push(plant.pose(&car));
        let mut pose = seen[seen.len().saturating_sub(if sim_like { 4 } else { 1 })];
        if sim_like {
            if t >= next_fix {
                let a = (-0.2_f64 / 2.0).exp();
                let k = (1.0 - a * a).sqrt();
                error.0 = a * error.0 + k * 0.08 * rng.normal();
                error.1 = a * error.1 + k * 0.08 * rng.normal();
                error.2 = a * error.2 + k * 1.2_f64.to_radians() * rng.normal();
                next_fix += 0.2;
            }
            pose = Pose::new(pose.x + error.0, pose.y + error.1, pose.yaw + error.2);
        }
        follower.observe(&Observation {
            pose,
            speed: plant.speed(&car),
            steering: Some(plant.steering(&car)),
            throttle: Some(car.car.throttle),
            brake: Some(car.car.brake),
        });
        if t == 0.0 {
            follower.follow(path.clone(), false);
        }
        match follower.tick(t, None) {
            Some(Tick::Drive(report)) => command = report.command,
            Some(Tick::Done(outcome)) => {
                reached = outcome == Outcome::Succeeded;
                break;
            }
            None => {}
        }
        for _ in 0..(PERIOD / PHYSICS).round() as usize {
            plant.step(&mut car, command, PHYSICS);
        }
        t += PERIOD;
        let now = plant.pose(&car);
        let rate = mppi_lag::model::wrap(now.yaw - yaw) / PERIOD;
        yaw = now.yaw;
        let (foot, v) = (path.foot(&now), plant.speed(&car));
        // Once rolling, past the first 4 s and before the last 3 m.
        if t > 4.0 && foot.left > 3.0 && v.abs() > 0.2 {
            crosses.push(foot.cross_track.abs());
            steers.push(plant.steering(&car));
            pushes.push(v * rate);
            speeds.push(v.abs());
            track.push(now);
        }
    }
    let n = crosses.len().max(1) as f64;
    let sides: Vec<bool> = steers
        .iter()
        .filter(|s| s.abs() >= 0.02)
        .map(|s| *s > 0.0)
        .collect();
    Wiggle {
        reached,
        mean_cross: crosses.iter().sum::<f64>() / n,
        swing: swing(&track),
        sway: weave(&pushes),
        flips: sides.windows(2).filter(|w| w[0] != w[1]).count() as f64 / (n * PERIOD),
        speed: speeds.iter().sum::<f64>() / n,
        steer_rms: (steers.iter().map(|s| s * s).sum::<f64>() / n).sqrt(),
    }
}

/// 100 m straight on at the campus's 2.5 m/s: the wheels hold straight. Until 15 Nov
/// (the same noise at every speed) they changed side about twice a second.
#[test]
fn no_wiggle_on_a_straight_at_campus_speed() {
    let road = path_through(|u| (100.0 * u, 0.0), 100.0);
    let now = wiggle(&road, 2.5, Way::Forward, 1.0, false);
    let before = wiggle(&road, 2.5, Way::Forward, 0.0, false);
    println!("straight at 2.5 m/s: now {now:?}\n  before {before:?}");
    assert!(now.reached && now.speed > 2.2, "{now:?}");
    assert!(now.flips < 0.3, "{now:?}");
    assert!(now.swing < 0.01 && now.sway < 0.06, "{now:?}");
    assert!(now.mean_cross < 0.02, "{now:?}");
    // The cause, pinned: the old noise wiggles on the same road.
    assert!(
        before.flips > 1.0 && before.sway > 2.0 * now.sway,
        "{before:?}"
    );
}

/// The same with the pose as the sim gives it (late and noisy): the car still steers
/// less often and less far than before.
#[test]
fn less_wiggle_on_a_straight_with_a_sim_like_pose() {
    let road = path_through(|u| (100.0 * u, 0.0), 100.0);
    let now = wiggle(&road, 2.5, Way::Forward, 1.0, true);
    let before = wiggle(&road, 2.5, Way::Forward, 0.0, true);
    println!("straight, sim-like pose: now {now:?}\n  before {before:?}");
    assert!(now.reached, "{now:?}");
    assert!(now.flips < 0.8 && now.sway < 0.18, "{now:?}");
    assert!(now.swing < 0.05, "{now:?}");
    assert!(before.sway > 1.5 * now.sway, "{before:?}");
}

/// A gentle bend (radius 30 m) at 2.5 m/s: one steady turn, no side to side.
#[test]
fn no_wiggle_on_a_gentle_curve_at_campus_speed() {
    let r = 30.0;
    let bend = path_through(
        |u| ((80.0 * u / r).sin() * r, r - (80.0 * u / r).cos() * r),
        80.0,
    );
    let now = wiggle(&bend, 2.5, Way::Forward, 1.0, false);
    println!("curve r 30 m at 2.5 m/s: {now:?}");
    assert!(now.reached && now.speed > 2.2, "{now:?}");
    assert!(now.flips < 0.3 && now.sway < 0.06, "{now:?}");
    assert!(now.mean_cross < 0.02, "{now:?}");
}

/// Backing up 15 m (reversing:=true, 0.5 m/s): slower than steer_std_speed, so the noise
/// is as before. The car doesn't swing; the wheels dither about 1 degree round straight
/// (0.017 rad RMS, crossing 0.02 rad under once a second), too little to move it at 0.5 m/s.
#[test]
fn no_wiggle_backing_up() {
    let back = straight_back(15.0);
    let now = wiggle(&back, 1.0, Way::Reversing, 1.0, false);
    println!("15 m back: {now:?}");
    assert!(now.reached && now.speed > 0.4, "{now:?}");
    assert!(now.swing < 0.01 && now.sway < 0.01, "{now:?}");
    assert!(now.mean_cross < 0.02, "{now:?}");
    assert!(now.steer_rms < 0.03 && now.flips < 1.2, "{now:?}");
}

/// The `ours` stack's calm steering (`mppi_lag_steer_gamma:=0.05`, TODO 168): less
/// sway on a straight with a sim-like pose, at the lot's 1 m/s and the campus's 2.5 m/s,
/// as close to the path.
#[test]
fn calm_steering_sways_less_with_a_sim_like_pose() {
    let road = path_through(|u| (100.0 * u, 0.0), 100.0);
    for speed in [1.0, 2.5] {
        let nav2 = wiggle(&road, speed, Way::Forward, 1.0, true);
        STEER_GAMMA.with(|g| g.set(Some(CALM)));
        let calm = wiggle(&road, speed, Way::Forward, 1.0, true);
        STEER_GAMMA.with(|g| g.set(None));
        println!("{speed} m/s: calm {calm:?}\n  gamma's {nav2:?}");
        assert!(calm.reached && calm.speed > 0.9 * nav2.speed, "{calm:?}");
        assert!(calm.sway < 0.8 * nav2.sway, "{calm:?} against {nav2:?}");
        assert!(calm.mean_cross < nav2.mean_cross + 0.02, "{calm:?}");
    }
}

/// The calm steering eases back to gamma's as the car slows (`steer_gamma_speed`), so
/// the lot's turn back still goes through its cusp, from a stop and arriving rolling.
#[test]
fn calm_steering_still_turns_back_through_the_cusp() {
    STEER_GAMMA.with(|g| g.set(Some(CALM)));
    turn_back_through_a_cusp();
    turn_back_arriving_rolling_and_off_the_path();
    forward_stop_reverse_through_a_cusp();
}
