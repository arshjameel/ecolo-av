//! Follow planned paths in closed loop, no ROS: how well `mppi_lag` keeps to each.
//!
//! ```sh
//! cargo run --release -p lattice-planner --example moves -- /tmp/moves   # (in aav_planner)
//! cargo run --release -p mppi-lag --example follow_plan -- /tmp/moves/*.csv
//! ```
//!
//! Each CSV is one path (x,y,yaw a line, as the planner's `moves` example writes them).
//! The car is ego plus the lag model (vehicle.yaml's guesses), stepped every 0.01 s; MPPI
//! ticks at 20 Hz, lag-aware, as `tests/closed_loop.rs` drives it; at 1 m/s (`SPEED`
//! sets another; `MODEL=ideal`: lag-free, as Nav2's MPPI), reversing set up as `reversing:=true` when the path has a cusp. Per path:
//! reached, time, metres off the path (mean, worst), and the steering: how often it swings
//! from left to right (flips a second) and how fast it moves on average (rad/s).

use mppi_lag::{
    CarModel, Command, Critic, Driven, Ego, Follower, FollowerSettings, GlobalPath, IdealModel,
    Inversion, LagModel, Observation, Outcome, PathAngleMode, Pose, Settings, Tick,
};

const PERIOD: f64 = 0.05;
const PHYSICS: f64 = 0.01;
const MAX_REVERSE_SPEED: f64 = 0.5;
/// Steering under this (rad) counts as straight for the flips.
const STRAIGHT: f64 = 0.02;

fn critics(reversing: bool) -> Vec<Critic> {
    // nav2.yaml's list and its changes from Nav2's defaults (as tests/closed_loop.rs).
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

fn read(path: &str) -> Vec<Pose> {
    std::fs::read_to_string(path)
        .expect("a CSV")
        .lines()
        .filter_map(|l| {
            let v: Vec<f64> = l.split(',').filter_map(|f| f.trim().parse().ok()).collect();
            (v.len() == 3).then(|| Pose::new(v[0], v[1], v[2]))
        })
        .collect()
}

fn has_cusp(poses: &[Pose]) -> bool {
    poses.windows(3).any(|w| {
        (w[1].x - w[0].x) * (w[2].x - w[1].x) + (w[1].y - w[0].y) * (w[2].y - w[1].y) < 0.0
    })
}

struct Run {
    outcome: Option<Outcome>,
    time: f64,
    mean_cross: f64,
    max_cross: f64,
    flips_per_s: f64,
    steer_speed: f64,
}

/// MODEL=ideal: MPPI plans with the lag-free model Nav2's MPPI assumes (the champion,
/// `tracker:=none`); else the lag model (`mppi_lag`).
fn drive(poses: &[Pose], speed: f64) -> Run {
    let truth = LagModel::default();
    if std::env::var("MODEL").is_ok_and(|m| m == "ideal") {
        let ideal = IdealModel {
            wheelbase: truth.wheelbase,
            max_steering_angle: truth.max_steering_angle,
            // Ego's ramps are already in front of it (Driven); the car itself is instant.
            accel: 100.0,
            decel: 100.0,
        };
        drive_with(ideal, truth, poses, speed)
    } else {
        drive_with(truth, truth, poses, speed)
    }
}

fn drive_with<M: CarModel + Clone>(
    controller: M,
    truth: LagModel,
    poses: &[Pose],
    speed: f64,
) -> Run {
    let reversing = has_cusp(poses);
    let ego = Ego::default();
    let plant = Driven { ego, car: truth };
    let mppi = Settings {
        batch_size: 1000,
        vx_max: speed,
        vx_min: if reversing { -MAX_REVERSE_SPEED } else { 0.0 },
        seed: 7,
        threads: 4,
        ..Settings::default()
    };
    let follow = FollowerSettings {
        inversion: reversing.then(Inversion::default),
        ..FollowerSettings::default()
    };
    let mut follower = Follower::new(
        controller,
        ego,
        mppi,
        critics(reversing),
        follow,
        Vec::new(),
    );
    let mut car = plant.at_rest(poses[0], 0.0);
    let path = GlobalPath::new(poses.to_vec()).expect("a path");
    let judge = path.clone();
    let mut command = Command::default();
    let (mut crosses, mut steers) = (Vec::new(), Vec::new());
    let mut t = 0.0;
    let mut outcome = None;
    follower.observe(&observe(&plant, &car));
    follower.follow(path, false);
    while t < 240.0 {
        follower.observe(&observe(&plant, &car));
        match follower.tick(t, None) {
            Some(Tick::Drive(report)) => command = report.command,
            Some(Tick::Done(o)) => {
                outcome = Some(o);
                break;
            }
            None => {}
        }
        for _ in 0..(PERIOD / PHYSICS).round() as usize {
            plant.step(&mut car, command, PHYSICS);
        }
        t += PERIOD;
        steers.push(plant.steering(&car));
        crosses.push(judge.foot(&plant.pose(&car)).cross_track.abs());
    }
    let signs: Vec<f64> = steers
        .iter()
        .filter(|s| s.abs() > STRAIGHT)
        .map(|s| s.signum())
        .collect();
    let flips = signs.windows(2).filter(|w| w[0] != w[1]).count();
    let moved: f64 = steers.windows(2).map(|w| (w[1] - w[0]).abs()).sum();
    let n = crosses.len().max(1) as f64;
    Run {
        outcome,
        time: t,
        mean_cross: crosses.iter().sum::<f64>() / n,
        max_cross: crosses.iter().copied().fold(0.0, f64::max),
        flips_per_s: flips as f64 / t.max(PERIOD),
        steer_speed: moved / t.max(PERIOD),
    }
}

fn observe<M: CarModel>(plant: &M, car: &M::State) -> Observation {
    Observation {
        pose: plant.pose(car),
        speed: plant.speed(car),
        steering: Some(plant.steering(car)),
        throttle: None,
        brake: None,
    }
}

fn main() {
    let speed: f64 = std::env::var("SPEED")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1.0);
    println!("| path | outcome | time s | mean off m | worst off m | steering flips /s | steering rad/s |");
    println!("|---|---|---|---|---|---|---|");
    for file in std::env::args().skip(1) {
        let poses = read(&file);
        if poses.len() < 2 {
            println!("| {file} | too short | | | | | |");
            continue;
        }
        let r = drive(&poses, speed);
        let name = std::path::Path::new(&file)
            .file_stem()
            .map_or(file.clone(), |s| s.to_string_lossy().into_owned());
        println!(
            "| {name} | {} | {:.1} | {:.3} | {:.3} | {:.2} | {:.3} |",
            r.outcome
                .map_or("time out".to_string(), |o| format!("{o:?}")),
            r.time,
            r.mean_cross,
            r.max_cross,
            r.flips_per_s,
            r.steer_speed
        );
    }
}
