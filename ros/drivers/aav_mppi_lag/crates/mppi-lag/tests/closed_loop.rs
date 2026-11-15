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

fn settings(speed: f64) -> Settings {
    Settings {
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
    let (mppi, critics, follow) = match way {
        Way::Forward => (settings(speed), critics(), FollowerSettings::default()),
        Way::Reversing => (
            Settings {
                vx_min: -MAX_REVERSE_SPEED,
                ..settings(speed)
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
    };
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
/// it. The lag model's car tops out near 1.4 m/s at full throttle, so this is the goal at
/// its top speed, not the campus's 2.5 m/s (that needs the simulator; 14 Nov: stood
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
