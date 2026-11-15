//! Follow in odom, correct slowly (TODO 186), closed loop, no ROS.
//!
//! As the node runs: the follower sees the car in `odom` (the smooth local odometry), the
//! path comes in `map`, again every `replan` s (our navigator: every second, or kept up
//! to 10 s), and the global EKF's map -> odom is the car's pose in map, off by a GPS-like
//! error that changes 5 times a second, less the odometry. The old way moves each new path
//! into odom by the newest map -> odom; the new way by the held one (`MapCorrection`),
//! which slews toward the newest at a bounded rate and moves the running path with it.
//!
//! The pretend car is ego plus the lag model at 100 Hz, as in `closed_loop.rs`; MPPI is set
//! up as the launch file sets it (2000 samples, 2.8 s of path at the top speed). Scored on
//! the truth against the path in map, as `tools/drive_metrics.py` counts:
//!
//! | Column | What |
//! |---|---|
//! | off m | mean distance off the path |
//! | swing m | side to side about the car's own 4 s smoothed track (RMS) |
//! | sway m/s² | the sideways push less its 4 s average (RMS) |
//! | flips/s | the wheels' angle changing side (under 0.02 rad: straight) |
//!
//! Every case, printed: `cargo test --release -p mppi-lag --test map_correction explore --
//! --ignored --nocapture`.

use mppi_lag::map_correction::{into_map, into_odom};
use mppi_lag::rng::Rng;
use mppi_lag::{
    CarModel, Command, Critic, Driven, Ego, Follower, FollowerSettings, GlobalPath, LagModel,
    MapCorrection, Observation, Outcome, Pose, Settings, SlewLimits, Tick,
};

const PERIOD: f64 = 0.05;
const PHYSICS: f64 = 0.01;
/// s: drive_metrics.py's WEAVE_WIDTH.
const WEAVE: f64 = 4.0;
/// s between GPS fixes moving the global EKF.
const FIX: f64 = 0.2;

/// The global EKF's error on the car's pose in map: AR(1) in x, y and yaw (`tau` s), a new
/// value every 0.2 s.
#[derive(Debug, Clone, Copy)]
struct Noise {
    xy: f64,
    yaw: f64,
    tau: f64,
}

/// The sim's map -> odom, 14 Nov: about 0.09 m sideways from one fix to the next, as good as
/// independent (each fix 6.5 cm either way in x and y, so 9 cm between two; 0.5 degrees).
const JUMPY: Noise = Noise {
    xy: 0.065,
    yaw: 0.5 * std::f64::consts::PI / 180.0,
    tau: 0.05,
};

/// The wiggle example's EKF-like error: 8 cm and 1.2 degrees, wandering over 2 s.
const WANDER: Noise = Noise {
    xy: 0.08,
    yaw: 1.2 * std::f64::consts::PI / 180.0,
    tau: 2.0,
};

#[derive(Debug, Clone, Copy)]
struct Case {
    noise: Option<Noise>,
    /// s between new paths.
    replan: f64,
    limits: SlewLimits,
    /// m: the global EKF's pose starts this far left of the truth (a bias in map), and is
    /// corrected at `bias_until` s (a real offset the follower must take out).
    bias: f64,
    bias_until: f64,
    seed: u64,
}

impl Case {
    fn new(noise: Option<Noise>, replan: f64, limits: SlewLimits) -> Self {
        Self {
            noise,
            replan,
            limits,
            bias: 0.0,
            bias_until: 0.0,
            seed: 7,
        }
    }
}

#[derive(Debug, Default)]
struct Score {
    reached: bool,
    off: f64,
    swing: f64,
    sway: f64,
    flips: f64,
    speed: f64,
    /// s after the bias is corrected until the car stays within 0.1 m of the path.
    settle: f64,
    /// Every tick: (t, the truth's cross track).
    trace: Vec<(f64, f64)>,
}

fn critics() -> Vec<Critic> {
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

/// A path through `f(u)`, u in 0..1, a pose every ~0.25 m (as the lattice planner's).
fn path_through(f: impl Fn(f64) -> (f64, f64), length: f64) -> Vec<Pose> {
    let n = (length / 0.25) as usize;
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

fn straight(length: f64) -> Vec<Pose> {
    path_through(|u| (length * u, 0.0), length)
}

fn bend(r: f64, length: f64) -> Vec<Pose> {
    path_through(
        |u| ((length * u / r).sin() * r, r - (length * u / r).cos() * r),
        length,
    )
}

fn moving_average(v: &[f64], half: usize) -> Vec<f64> {
    (0..v.len())
        .map(|i| {
            let (a, b) = (i.saturating_sub(half), (i + half + 1).min(v.len()));
            v[a..b].iter().sum::<f64>() / (b - a) as f64
        })
        .collect()
}

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

/// Drive `poses` (map) at 2.5 m/s as `case` says.
fn run(case: &Case, poses: &[Pose]) -> Score {
    let speed = 2.5;
    let truth = LagModel::default();
    let ego = Ego::default();
    let plant = Driven { ego, car: truth };
    let mppi = Settings {
        batch_size: 2000,
        vx_max: speed,
        seed: case.seed,
        threads: 4,
        ..Settings::default()
    };
    let follow = FollowerSettings {
        prune_distance: (2.8 * speed * 1.2).max(3.0),
        ..FollowerSettings::default()
    };
    let mut follower = Follower::new(truth, ego, mppi, critics(), follow, Vec::new());
    // odom's true pose in map: odom started 30 m south-west and turned, so a turn of map
    // -> odom about odom's origin is not about the car.
    let odom_true = Pose::new(-30.0, -20.0, 0.3);
    let mut car = plant.at_rest(poses[0], 0.0);
    let judge = GlobalPath::new(poses.to_vec()).expect("a path");
    let mut correction = MapCorrection::new(case.limits);
    let mut rng = Rng::new(case.seed ^ 0x5eed);
    let (mut error, mut next_fix) = ((0.0, 0.0, 0.0), 0.0);
    let (mut next_path, mut driving) = (0.0, false);
    let mut score = Score::default();
    let (mut crosses, mut steers, mut pushes, mut speeds, mut track) =
        (vec![], vec![], vec![], vec![], vec![]);
    let (mut command, mut t) = (Command::default(), 0.0);
    let mut yaw = poses[0].yaw;
    while t < 150.0 {
        let in_map = plant.pose(&car);
        let in_odom = into_odom(&in_map, &odom_true);
        if let Some(n) = case.noise {
            if t >= next_fix {
                let a = (-FIX / n.tau).exp();
                let k = (1.0 - a * a).sqrt();
                error.0 = a * error.0 + k * n.xy * rng.normal();
                error.1 = a * error.1 + k * n.xy * rng.normal();
                error.2 = a * error.2 + k * n.yaw * rng.normal();
                next_fix += FIX;
            }
        }
        // The global EKF: the car in map, off by the error (and the bias, to its left).
        let bias = if t < case.bias_until { case.bias } else { 0.0 };
        let (s, c) = in_map.yaw.sin_cos();
        let believed = Pose::new(
            in_map.x + error.0 - s * bias,
            in_map.y + error.1 + c * bias,
            in_map.yaw + error.2,
        );
        // map -> odom = the believed pose less the odometry: odom's pose in map.
        let yaw_o = believed.yaw - in_odom.yaw;
        let (so, co) = yaw_o.sin_cos();
        let newest = Pose::new(
            believed.x - (co * in_odom.x - so * in_odom.y),
            believed.y - (so * in_odom.x + co * in_odom.y),
            yaw_o,
        );

        follower.observe(&Observation {
            pose: in_odom,
            speed: plant.speed(&car),
            steering: Some(plant.steering(&car)),
            throttle: Some(car.car.throttle),
            brake: Some(car.car.brake),
        });
        if driving {
            if let Some((_, shift)) = correction.update(newest, &in_odom, PERIOD) {
                follower.move_path(|p| shift.apply(p));
            }
        }
        if t >= next_path - 1e-9 {
            // The planner plans from where the global EKF believes the car is: the path
            // from its nearest pose on (the same line each time, only shorter).
            let from = poses
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.distance(&believed).total_cmp(&b.1.distance(&believed)))
                .map_or(0, |(i, _)| i)
                .min(poses.len().saturating_sub(2));
            let frame = correction.for_new_path(newest, driving);
            let moved: Vec<Pose> = poses[from..].iter().map(|p| into_odom(p, &frame)).collect();
            follower.follow(GlobalPath::new(moved).expect("a path"), driving);
            driving = true;
            next_path += case.replan;
        }
        match follower.tick(t, None) {
            Some(Tick::Drive(r)) => command = r.command,
            Some(Tick::Done(o)) => {
                score.reached = o == Outcome::Succeeded;
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
        let (foot, v) = (judge.foot(&now), plant.speed(&car));
        score.trace.push((t, foot.cross_track));
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
        .filter(|s: &&f64| s.abs() >= 0.02)
        .map(|s| *s > 0.0)
        .collect();
    score.off = crosses.iter().sum::<f64>() / n;
    score.swing = swing(&track);
    score.sway = weave(&pushes);
    score.flips = sides.windows(2).filter(|w| w[0] != w[1]).count() as f64 / (n * PERIOD);
    score.speed = speeds.iter().sum::<f64>() / n;
    if case.bias > 0.0 {
        let last_out = score
            .trace
            .iter()
            .filter(|(t, x)| *t >= case.bias_until && x.abs() > 0.1)
            .map(|(t, _)| *t)
            .fold(case.bias_until, f64::max);
        score.settle = last_out - case.bias_until;
    }
    score
}

/// The mean of `seeds` runs.
fn mean(case: Case, poses: &[Pose], seeds: u64) -> Score {
    let runs: Vec<Score> = (0..seeds)
        .map(|s| {
            run(
                &Case {
                    seed: 7 + s,
                    ..case
                },
                poses,
            )
        })
        .collect();
    let k = runs.len() as f64;
    let avg = |f: fn(&Score) -> f64| runs.iter().map(f).sum::<f64>() / k;
    Score {
        reached: runs.iter().all(|r| r.reached),
        off: avg(|s| s.off),
        swing: avg(|s| s.swing),
        sway: avg(|s| s.sway),
        flips: avg(|s| s.flips),
        speed: avg(|s| s.speed),
        settle: avg(|s| s.settle),
        trace: Vec::new(),
    }
}

fn limits(speed: f64, degrees: f64) -> SlewLimits {
    SlewLimits {
        speed,
        turn_rate: degrees.to_radians(),
        ..SlewLimits::default()
    }
}

fn row(name: &str, s: &Score) {
    println!(
        "| {name} | {} | {:.3} | {:.3} | {:.3} | {:.2} | {:.2} | {:.1} |",
        s.reached, s.off, s.swing, s.sway, s.flips, s.speed, s.settle
    );
}

/// Every case, a table.
#[test]
#[ignore]
fn explore() {
    let seeds = std::env::var("SEEDS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(2);
    println!("| case | reached | off m | swing m | sway m/s² | flips/s | m/s | settle s |");
    println!("|---|---|---|---|---|---|---|---|");
    let paths = [
        ("straight", straight(150.0)),
        ("bend r 30", bend(30.0, 120.0)),
    ];
    let rates: [(&str, SlewLimits); 4] = [
        ("old (newest each path)", SlewLimits::OFF),
        ("0.05 m/s, 0.5 deg/s", limits(0.05, 0.5)),
        ("0.1 m/s, 1 deg/s", limits(0.1, 1.0)),
        ("0.2 m/s, 2 deg/s", limits(0.2, 2.0)),
    ];
    for (path, poses) in &paths {
        row(
            &format!("{path}, truth"),
            &mean(Case::new(None, 1.0, SlewLimits::OFF), poses, seeds),
        );
        for (noise_name, noise) in [("jumpy", JUMPY), ("wander", WANDER)] {
            for replan in [1.0, 10.0] {
                for (name, l) in &rates {
                    let case = Case::new(Some(noise), replan, *l);
                    row(
                        &format!("{path}, {noise_name}, path every {replan} s, {name}"),
                        &mean(case, poses, seeds),
                    );
                }
            }
        }
        for replan in [1.0, 10.0] {
            for (name, l) in &rates {
                let case = Case {
                    bias: 0.5,
                    bias_until: 20.0,
                    ..Case::new(Some(JUMPY), replan, *l)
                };
                row(
                    &format!("{path}, 0.5 m step at 20 s, path every {replan} s, {name}"),
                    &mean(case, poses, seeds),
                );
            }
        }
    }
}

/// The sim's jumpy map -> odom, a new path every second, on a straight at 2.5 m/s: the
/// held map -> odom sways the car less and flips its wheels less than the newest.
#[test]
fn slow_correction_sways_less_on_a_jumpy_map() {
    let road = straight(150.0);
    let old = mean(Case::new(Some(JUMPY), 1.0, SlewLimits::OFF), &road, 2);
    let new = mean(Case::new(Some(JUMPY), 1.0, SlewLimits::default()), &road, 2);
    println!("jumpy map, straight: old {old:?}\n  new {new:?}");
    assert!(old.reached && new.reached);
    assert!(new.sway < 0.7 * old.sway, "{new:?} {old:?}");
    assert!(new.flips < old.flips, "{new:?} {old:?}");
    assert!(new.off < old.off + 0.02, "{new:?} {old:?}");
}

/// The same on a bend (radius 30 m).
#[test]
fn slow_correction_sways_less_on_a_bend() {
    let road = bend(30.0, 120.0);
    let old = mean(Case::new(Some(JUMPY), 1.0, SlewLimits::OFF), &road, 2);
    let new = mean(Case::new(Some(JUMPY), 1.0, SlewLimits::default()), &road, 2);
    println!("jumpy map, bend: old {old:?}\n  new {new:?}");
    assert!(old.reached && new.reached);
    assert!(new.sway < 0.8 * old.sway, "{new:?} {old:?}");
}

/// A real offset: the global pose 0.5 m off until 20 s, then right. The held map -> odom
/// takes it out at 0.1 m/s: the car is back within 0.1 m of the path inside 8 s.
#[test]
fn a_real_offset_is_still_taken_out() {
    let road = straight(150.0);
    let case = Case {
        bias: 0.5,
        bias_until: 20.0,
        ..Case::new(Some(JUMPY), 1.0, SlewLimits::default())
    };
    let s = run(&case, &road);
    let before: Vec<f64> = s
        .trace
        .iter()
        .filter(|(t, _)| (15.0..20.0).contains(t))
        .map(|(_, x)| *x)
        .collect();
    let was = before.iter().sum::<f64>() / before.len() as f64;
    println!(
        "0.5 m step: {:.3} m off before, settled {:.1} s after",
        was, s.settle
    );
    assert!(s.reached);
    // It followed the biased path: 0.5 m right of the true one.
    assert!((was + 0.5).abs() < 0.1, "{was}");
    assert!(s.settle < 8.0, "{:.1} s", s.settle);
}

/// A relocalisation (3 m) is taken at once, not slewed for 30 s.
#[test]
fn a_relocalisation_snaps() {
    let road = straight(150.0);
    let case = Case {
        bias: 3.0,
        bias_until: 20.0,
        ..Case::new(None, 1.0, SlewLimits::default())
    };
    let s = run(&case, &road);
    println!("3 m step: settled {:.1} s after", s.settle);
    assert!(s.reached);
    assert!(s.settle < 8.0, "{:.1} s", s.settle);
}

/// `into_map` is the inverse of `into_odom` (the shared helpers the node uses).
#[test]
fn into_map_undoes_into_odom() {
    let odom = Pose::new(-30.0, -20.0, 0.3);
    let p = Pose::new(5.0, 1.0, 0.2);
    let back = into_map(&into_odom(&p, &odom), &odom);
    assert!(p.distance(&back) < 1e-12);
}
