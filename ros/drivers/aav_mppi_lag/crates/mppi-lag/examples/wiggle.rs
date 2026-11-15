//! The wiggle, without ROS (TODO 168): which suspect makes the car swing side to side?
//!
//! ```sh
//! cargo run --release -p mppi-lag --example wiggle                 # every suspect, a table
//! cargo run --release -p mppi-lag --example wiggle -- path.csv     # a planned path too (x,y,yaw)
//! ONLY=after SPEED=1.0 REVERSE=1 cargo run --release -p mppi-lag --example wiggle
//! ```
//!
//! The car is ego plus the lag model (vehicle.yaml's guesses), stepped every 0.01 s; MPPI
//! ticks at 20 Hz with the node's 2000 samples, and sees the path as far as the launch
//! file lets it (`prune_distance`: 2.8 s at the top speed, and a fifth more: 8.4 m at
//! 2.5 m/s). Each row changes one suspect from "before" (the settings until 15 Nov:
//! the same steering noise at every speed); "after" is today's.
//!
//! | Column | What (as `tools/drive_metrics.py` counts it) |
//! |---|---|
//! | off m | mean distance off the path |
//! | swing m | side to side about the car's own 4 s smoothed track (RMS), `swing_m` |
//! | sway m/s² | the sideways push (speed x turn rate) less its 4 s average (RMS), `sway_mps2` |
//! | flips/s | the wheels' angle changing side (under 0.02 rad: straight) |
//! | m/s | the mean speed |
//!
//! Switches: `SPEED` (2.5 m/s), `SEEDS` (2), `ONLY` (rows whose name holds it), `REVERSE`
//! (also 15 m straight back at 0.5 m/s).

use mppi_lag::rng::Rng;
use mppi_lag::{
    Actuators, CarModel, Command, Critic, Driven, Ego, Follower, FollowerSettings, GlobalPath,
    LagModel, Observation, Outcome, PathAngleMode, Pose, Settings, SpeedLoop, Tick,
};

const PERIOD: f64 = 0.05;
const PHYSICS: f64 = 0.01;
/// rad: a smaller steering angle counts as straight.
const STRAIGHT: f64 = 0.02;
/// s: drive_metrics.py's WEAVE_WIDTH. Slower than this is drift, faster is wiggle.
const WEAVE: f64 = 4.0;
/// m between path poses, as the lattice planner's.
const STEP: f64 = 0.25;

/// A path through `f(t)`, t in 0..1, a pose every ~`STEP` m.
fn path_through(f: impl Fn(f64) -> (f64, f64), length: f64) -> Vec<Pose> {
    let n = (length / STEP) as usize;
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

/// An arc of radius `r`, `length` m, to the left.
fn curve(r: f64, length: f64) -> Vec<Pose> {
    path_through(
        |t| {
            let a = t * length / r;
            (r * a.sin(), r - r * a.cos())
        },
        length,
    )
}

/// 20 m east, a quarter turn left of radius `r`, 20 m north.
fn corner(r: f64) -> Vec<Pose> {
    let arc = std::f64::consts::FRAC_PI_2 * r;
    let total = 40.0 + arc;
    path_through(
        |t| {
            let d = t * total;
            if d < 20.0 {
                (d, 0.0)
            } else if d < 20.0 + arc {
                let a = (d - 20.0) / r;
                (20.0 + r * a.sin(), r - r * a.cos())
            } else {
                (20.0 + r, r + d - 20.0 - arc)
            }
        },
        total,
    )
}

/// Two waves of y = a sin(2 pi x / wavelength).
fn s_curve(a: f64, wavelength: f64) -> Vec<Pose> {
    let length = 2.0 * wavelength;
    path_through(
        |t| {
            let x = t * length;
            (x, a * (std::f64::consts::TAU * x / wavelength).sin())
        },
        length * 1.1,
    )
}

/// `length` m straight back, the car facing east.
fn straight_back(length: f64) -> Vec<Pose> {
    let n = (length / STEP) as usize;
    (0..=n)
        .map(|i| Pose::new(-(i as f64) * STEP, 0.0, 0.0))
        .collect()
}

/// A CSV path (x,y,yaw a line), thinned to a pose every ~`STEP` m.
fn read(path: &str) -> Vec<Pose> {
    let all: Vec<Pose> = std::fs::read_to_string(path)
        .expect("a CSV")
        .lines()
        .filter_map(|l| {
            let v: Vec<f64> = l.split(',').filter_map(|f| f.trim().parse().ok()).collect();
            (v.len() == 3).then(|| Pose::new(v[0], v[1], v[2]))
        })
        .collect();
    let mut kept: Vec<Pose> = Vec::new();
    for p in &all {
        if kept.last().is_none_or(|q| q.distance(p) >= STEP - 1e-6) {
            kept.push(*p);
        }
    }
    kept
}

/// What the follower sees of the pose: the truth plus an EKF-like error, AR(1) in x, y
/// and yaw, a new value every `every` s (a GPS fix moving map -> odom). The sim's saved
/// campus runs: belief 0.08-0.10 m sideways and 1.0-1.5 degrees off the truth (RMS).
#[derive(Clone, Copy)]
struct PoseNoise {
    xy: f64,
    yaw: f64,
    tau: f64,
    every: f64,
}

const EKF: PoseNoise = PoseNoise {
    xy: 0.08,
    yaw: 0.021,
    tau: 2.0,
    every: 0.2,
};

#[derive(Clone)]
struct Case {
    name: &'static str,
    speed: f64,
    mppi: Settings,
    truth: LagModel,
    model: LagModel,
    noise: Option<PoseNoise>,
    /// s: how old the pose is that the follower sees.
    pose_delay: f64,
    /// m of path the critics see; `None`: as the launch file stretches it.
    prune: Option<f64>,
    reversing: bool,
}

struct Score {
    reached: bool,
    off: f64,
    swing: f64,
    sway: f64,
    flips: f64,
    speed: f64,
}

/// Each value's mean over the `half` values either side.
fn moving_average(v: &[f64], half: usize) -> Vec<f64> {
    (0..v.len())
        .map(|i| {
            let a = i.saturating_sub(half);
            let b = (i + half + 1).min(v.len());
            v[a..b].iter().sum::<f64>() / (b - a) as f64
        })
        .collect()
}

/// RMS of `v` less its own `WEAVE` s moving average (drive_metrics.py's weave).
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

/// drive_metrics.py's swing: the car's side offset from its own 4 s smoothed track,
/// weaved. A path that zigzags counts too.
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

/// Changes of side in `v`, ignoring values under `dead`.
fn flips(v: &[f64], dead: f64) -> usize {
    let signs: Vec<bool> = v
        .iter()
        .filter(|x| x.abs() >= dead)
        .map(|x| *x > 0.0)
        .collect();
    signs.windows(2).filter(|w| w[0] != w[1]).count()
}

/// nav2.yaml's critics, set up as `navigation.launch.py` sets them.
fn critics(reversing: bool) -> Vec<Critic> {
    mppi_lag::critics::NAMES
        .iter()
        .filter_map(|n| Critic::by_name(n))
        .filter(|c| !(reversing && c.name() == "PreferForwardCritic"))
        .map(|c| match c {
            Critic::GoalAngle(mut g) => {
                g.threshold = 2.5;
                Critic::GoalAngle(g)
            }
            Critic::PathAlign(mut p) => {
                p.use_path_orientations = true;
                Critic::PathAlign(p)
            }
            Critic::PathAngle(mut a) if reversing => {
                a.mode = PathAngleMode::ConsiderFeasiblePathOrientations;
                Critic::PathAngle(a)
            }
            other => other,
        })
        .collect()
}

fn run(case: &Case, poses: &[Pose], seed: u64) -> Score {
    let ego = Ego::default();
    let plant = Driven {
        ego,
        car: case.truth,
    };
    let mppi = Settings {
        vx_max: if case.reversing { 1.0 } else { case.speed },
        vx_min: if case.reversing { -case.speed } else { 0.0 },
        seed,
        ..case.mppi.clone()
    };
    // vehicle.mppi_lag_params: nav2.yaml's 3 m, or 2.8 s at vx_max and a fifth more.
    let prune = case
        .prune
        .unwrap_or((56.0 * 0.05 * mppi.vx_max * 1.2).max(3.0));
    let mut follower = Follower::new(
        case.model,
        ego,
        mppi,
        critics(case.reversing),
        FollowerSettings {
            prune_distance: prune,
            ..FollowerSettings::default()
        },
        Vec::new(),
    );
    // Start 0.2 m left of the path: something to settle from.
    let p0 = poses[0];
    let start = Pose::new(p0.x - 0.2 * p0.yaw.sin(), p0.y + 0.2 * p0.yaw.cos(), p0.yaw);
    let mut car = plant.at_rest(start, 0.0);
    let path = GlobalPath::new(poses.to_vec()).expect("a path");
    let judge = path.clone();
    let mut rng = Rng::new(seed ^ 0x5eed);
    let (mut ex, mut ey, mut eyaw, mut next_fix) = (0.0, 0.0, 0.0, 0.0);
    let mut history: Vec<Pose> = Vec::new();
    let mut command = Command::default();
    let (mut offs, mut steers, mut pushes) = (Vec::new(), Vec::new(), Vec::new());
    let (mut track, mut speeds) = (Vec::new(), Vec::new());
    let (mut t, mut reached, mut first) = (0.0, false, true);
    let mut last_yaw = start.yaw;
    while t < 200.0 {
        history.push(plant.pose(&car));
        let delay = (case.pose_delay / PERIOD).round() as usize;
        let old = history[history.len().saturating_sub(1 + delay)];
        if let Some(n) = case.noise {
            if t >= next_fix {
                let a = (-n.every / n.tau).exp();
                let k = (1.0 - a * a).sqrt();
                ex = a * ex + k * n.xy * rng.normal();
                ey = a * ey + k * n.xy * rng.normal();
                eyaw = a * eyaw + k * n.yaw * rng.normal();
                next_fix += n.every;
            }
        }
        follower.observe(&Observation {
            pose: Pose::new(old.x + ex, old.y + ey, old.yaw + eyaw),
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
            Some(Tick::Drive(r)) => command = r.command,
            Some(Tick::Done(o)) => {
                reached = o == Outcome::Succeeded;
                break;
            }
            None => {}
        }
        for _ in 0..(PERIOD / PHYSICS).round() as usize {
            plant.step(&mut car, command, PHYSICS);
        }
        t += PERIOD;
        let now = plant.pose(&car);
        let rate = mppi_lag::model::wrap(now.yaw - last_yaw) / PERIOD;
        last_yaw = now.yaw;
        // Scored once rolling, past the first 4 s and before the last 3 m.
        let foot = judge.foot(&now);
        let v = plant.speed(&car);
        if t > 4.0 && foot.left > 3.0 && v.abs() > 0.2 {
            offs.push(foot.cross_track.abs());
            steers.push(plant.steering(&car));
            pushes.push(v * rate);
            speeds.push(v.abs());
            track.push(now);
        }
    }
    let n = offs.len().max(1) as f64;
    Score {
        reached,
        off: offs.iter().sum::<f64>() / n,
        swing: swing(&track),
        sway: weave(&pushes),
        flips: flips(&steers, STRAIGHT) as f64 / (n * PERIOD),
        speed: speeds.iter().sum::<f64>() / n,
    }
}

fn env(name: &str) -> Option<f64> {
    std::env::var(name).ok().and_then(|v| v.parse().ok())
}

fn main() {
    let seeds: u64 = env("SEEDS").map_or(2, |s| s as u64);
    let after = Settings {
        threads: 4,
        // STD_SPEED: try another steer_std_speed for "after".
        steer_std_speed: env("STD_SPEED").unwrap_or(Settings::default().steer_std_speed),
        ..Settings::default()
    };
    let before_mppi = Settings {
        steer_std_speed: 0.0,
        ..after.clone()
    };
    let before = Case {
        name: "before: truth pose",
        speed: env("SPEED").unwrap_or(2.5),
        mppi: before_mppi.clone(),
        truth: LagModel::default(),
        model: LagModel::default(),
        noise: None,
        pose_delay: 0.0,
        prune: None,
        reversing: false,
    };
    let slow_steering = Actuators {
        steer_rate: 0.5,
        steer_lag: 0.3,
        steer_dead_time: 0.1,
        ..Actuators::default()
    };
    let instant = Actuators {
        throttle_lag: 0.0,
        brake_lag: 0.0,
        steer_rate: 10.0,
        steer_lag: 0.0,
        ..Actuators::default()
    };
    let cases = [
        before.clone(),
        Case {
            name: "before: pose EKF-like (8 cm, 1.2 deg)",
            noise: Some(EKF),
            ..before.clone()
        },
        Case {
            name: "before: pose 0.15 s old",
            pose_delay: 0.15,
            ..before.clone()
        },
        Case {
            name: "before: steering 0.2 s dead time, unmodelled",
            truth: LagModel {
                actuators: Actuators {
                    steer_dead_time: 0.2,
                    ..Actuators::default()
                },
                ..LagModel::default()
            },
            ..before.clone()
        },
        Case {
            name: "before: steering slower than modelled",
            truth: LagModel {
                actuators: slow_steering,
                ..LagModel::default()
            },
            ..before.clone()
        },
        Case {
            name: "before: no lag at all (model knows)",
            truth: LagModel {
                actuators: instant,
                ..LagModel::default()
            },
            model: LagModel {
                actuators: instant,
                ..LagModel::default()
            },
            ..before.clone()
        },
        Case {
            name: "before: no speed overshoot (no integral)",
            truth: LagModel {
                speed_loop: SpeedLoop {
                    ki: 0.0,
                    ..SpeedLoop::default()
                },
                ..LagModel::default()
            },
            ..before.clone()
        },
        Case {
            name: "before: window nav2.yaml's 3 m (drives slower)",
            prune: Some(3.0),
            ..before.clone()
        },
        Case {
            name: "before: 8000 samples",
            mppi: Settings {
                batch_size: 8000,
                ..before_mppi.clone()
            },
            ..before.clone()
        },
        Case {
            name: "before: temperature 0.5",
            mppi: Settings {
                temperature: 0.5,
                ..before_mppi.clone()
            },
            ..before.clone()
        },
        Case {
            name: "before: white noise",
            mppi: Settings {
                noise_correlation: 0.0,
                ..before_mppi.clone()
            },
            ..before.clone()
        },
        Case {
            name: "before: sim-like (EKF, 0.15 s old)",
            noise: Some(EKF),
            pose_delay: 0.15,
            ..before.clone()
        },
        Case {
            name: "after: truth pose",
            mppi: after.clone(),
            ..before.clone()
        },
        Case {
            name: "after: sim-like (EKF, 0.15 s old)",
            mppi: after.clone(),
            noise: Some(EKF),
            pose_delay: 0.15,
            ..before.clone()
        },
    ];
    let mut paths: Vec<(String, Vec<Pose>, bool)> = vec![
        (
            "straight 100 m".into(),
            path_through(|t| (100.0 * t, 0.0), 100.0),
            false,
        ),
        ("curve r 30 m".into(), curve(30.0, 80.0), false),
        ("S 2 x 30 m".into(), s_curve(2.0, 30.0), false),
        ("corner r 5 m".into(), corner(5.0), false),
    ];
    paths.extend(std::env::args().skip(1).map(|f| {
        let poses = read(&f);
        (f, poses, false)
    }));
    if std::env::var("REVERSE").is_ok() {
        paths.push(("15 m back".into(), straight_back(15.0), true));
    }
    let only = std::env::var("ONLY").ok();
    println!("| path | case | reached | off m | swing m | sway m/s² | flips/s | m/s |");
    println!("|---|---|---|---|---|---|---|---|");
    for (name, poses, back) in &paths {
        for case in &cases {
            if only
                .as_ref()
                .is_some_and(|o| !case.name.contains(o.as_str()))
            {
                continue;
            }
            let case = Case {
                reversing: *back,
                speed: if *back { 0.5 } else { case.speed },
                ..case.clone()
            };
            let scores: Vec<Score> = (0..seeds).map(|s| run(&case, poses, 7 + s)).collect();
            let k = scores.len() as f64;
            let mean = |f: fn(&Score) -> f64| scores.iter().map(f).sum::<f64>() / k;
            println!(
                "| {name} | {} | {}/{} | {:.3} | {:.3} | {:.3} | {:.2} | {:.2} |",
                case.name,
                scores.iter().filter(|s| s.reached).count(),
                scores.len(),
                mean(|s| s.off),
                mean(|s| s.swing),
                mean(|s| s.sway),
                mean(|s| s.flips),
                mean(|s| s.speed),
            );
        }
    }
}
