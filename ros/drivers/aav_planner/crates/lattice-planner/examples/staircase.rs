//! The plan's staircase (TODO 168, 190): a straight road at an angle between the
//! lattice's headings, planned, and how much the plan itself swings.
//!
//! ```sh
//! cargo run --release -p lattice-planner --example staircase
//! W_DATA=0.02 W_SMOOTH=0.45 cargo run --release -p lattice-planner --example staircase
//! ```
//!
//! A 6.5 m road (walls each side, inflated as nav2.yaml's global costmap), 120 m long, at
//! each angle (and 240 m at 8°); the plan from one end to the other. Then the lot tour's
//! legs, real corners: the tightest turn and the dearest cell the path crosses. Per plan, driven at `SPEED` m/s
//! (2.2, the campus's): the plan's swing (its sideways offset from the road's centre
//! line, less its own 4 s average, RMS: `drive_metrics.swing`'s idea), its sway (the
//! sideways push, speed x turn rate, less its 4 s average, RMS: `drive_metrics.sway`), its
//! mean offset from the centre line, and its tightest turn. The smoother's weights come
//! from `W_DATA` and `W_SMOOTH` (Nav2's 0.2 and 0.3) and `ITERATIONS` (1000).

use lattice_planner::smoother::smooth;
use lattice_planner::{scenes, tightest_turn, Planner, Pose, Settings, SmootherSettings};

/// m: the road's length.
const LENGTH: f64 = 120.0;
/// s: drive_metrics.py's WEAVE_WIDTH.
const WEAVE: f64 = 4.0;
/// s between the driven plan's samples.
const DT: f64 = 0.05;

fn env(name: &str) -> Option<f64> {
    std::env::var(name).ok().and_then(|v| v.parse().ok())
}

/// Each value less its mean over `WEAVE` s round it, RMS.
fn weave(v: &[f64]) -> f64 {
    let half = (WEAVE / DT / 2.0) as usize;
    let n = v.len().max(1) as f64;
    let sum: f64 = (0..v.len())
        .map(|i| {
            let (a, b) = (i.saturating_sub(half), (i + half + 1).min(v.len()));
            let mean = v[a..b].iter().sum::<f64>() / (b - a) as f64;
            (v[i] - mean).powi(2)
        })
        .sum();
    (sum / n).sqrt()
}

/// The plan driven at `speed`: (swing m, sway m/s², mean offset m), offsets from the line
/// through (10, 10) at `angle`.
fn score(poses: &[Pose], angle: f64, speed: f64) -> (f64, f64, f64) {
    // Points every speed x DT metres along the plan.
    let mut along = vec![0.0];
    for w in poses.windows(2) {
        along.push(along.last().unwrap_or(&0.0) + (w[1].x - w[0].x).hypot(w[1].y - w[0].y));
    }
    let total = *along.last().unwrap_or(&0.0);
    let step = speed * DT;
    let mut pts = Vec::new();
    let (mut d, mut i) = (0.0, 0);
    while d < total {
        while i + 2 < along.len() && along[i + 1] < d {
            i += 1;
        }
        let f = (d - along[i]) / (along[i + 1] - along[i]).max(1e-9);
        let (a, b) = (poses[i], poses[i + 1]);
        pts.push((a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)));
        d += step;
    }
    let (c, s) = (angle.cos(), angle.sin());
    let offsets: Vec<f64> = pts
        .iter()
        .map(|(x, y)| -(x - 10.0) * s + (y - 10.0) * c)
        .collect();
    let yaws: Vec<f64> = pts
        .windows(2)
        .map(|w| (w[1].1 - w[0].1).atan2(w[1].0 - w[0].0))
        .collect();
    let push: Vec<f64> = yaws
        .windows(2)
        .map(|w| {
            let mut turn = w[1] - w[0];
            while turn > std::f64::consts::PI {
                turn -= std::f64::consts::TAU;
            }
            while turn < -std::f64::consts::PI {
                turn += std::f64::consts::TAU;
            }
            speed * turn / DT
        })
        .collect();
    // Away from the ends: the first and last 10 m join the start and goal.
    let cut = (10.0 / step) as usize;
    let mid = |v: &[f64]| -> Vec<f64> {
        if v.len() > 2 * cut {
            v[cut..v.len() - cut].to_vec()
        } else {
            v.to_vec()
        }
    };
    let (o, p) = (mid(&offsets), mid(&push));
    let mean = o.iter().map(|x| x.abs()).sum::<f64>() / o.len().max(1) as f64;
    (weave(&o), weave(&p), mean)
}

fn main() {
    let speed = env("SPEED").unwrap_or(2.2);
    let defaults = SmootherSettings::default();
    let smoother = SmootherSettings {
        w_data: env("W_DATA").unwrap_or(defaults.w_data),
        w_smooth: env("W_SMOOTH").unwrap_or(defaults.w_smooth),
        max_iterations: env("ITERATIONS").map_or(defaults.max_iterations, |n| n as usize),
        ..defaults
    };
    let planner = Planner::new(
        scenes::lattice(),
        Settings {
            smoother,
            ..Settings::default()
        },
        &scenes::outline(),
        Some(&scenes::inflation()),
    );
    println!(
        "smoother: w_data {}, w_smooth {}, {} iterations; driven at {speed} m/s",
        smoother.w_data, smoother.w_smooth, smoother.max_iterations
    );
    println!(
        "| road at | plan | swing m | sway m/s² | mean off m | tightest turn m | converged | ms |"
    );
    println!("|---|---|---|---|---|---|---|---|");
    for (length, degrees) in [0.0, 5.0, 8.0, 11.25, 15.0, 30.0, 40.0]
        .map(|d| (LENGTH, d))
        .into_iter()
        .chain([(2.0 * LENGTH, 8.0)])
    {
        let angle = f64::to_radians(degrees);
        let grid = scenes::angled_road(angle, length);
        let (c, s) = (angle.cos(), angle.sin());
        let start = Pose::new(10.0, 10.0, angle);
        let goal = Pose::new(10.0 + (length - 5.0) * c, 10.0 + (length - 5.0) * s, angle);
        let label = format!("{degrees}°, {length:.0} m");
        match planner.plan(&grid, &start, &goal, &|| false) {
            Ok(plan) => {
                // Did the smoother finish (and so join the true start and goal)?
                let mut again = plan.raw.clone();
                let done = smooth(&mut again, &grid, planner.turning_radius(), &smoother, 5.0);
                for (what, poses) in [("searched", &plan.raw), ("smoothed", &plan.poses)] {
                    let (swing, sway, off) = score(poses, angle, speed);
                    println!(
                        "| {label} | {what} | {swing:.3} | {sway:.3} | {off:.2} | {:.2} | {} | {:.0} |",
                        tightest_turn(poses, 1.0),
                        if what == "smoothed" { done.to_string() } else { "-".into() },
                        plan.total_time * 1e3
                    );
                }
            }
            Err(e) => println!("| {label} | {e} | | | | | | |"),
        }
    }
    // The lot tour: real corners, where a stronger smoother could cut too close.
    let lot = scenes::map("example_lot").expect("the example lot");
    println!("\n| lot leg | tightest turn m | converged | worst cost on the path | ms |");
    println!("|---|---|---|---|---|");
    let mut from = scenes::LOT_START;
    for (name, x, y, heading) in scenes::LOT_TOUR {
        let to = Pose::new(x, y, heading.to_radians());
        match planner.plan(&lot, &from, &to, &|| false) {
            Ok(plan) => {
                let mut again = plan.raw.clone();
                let done = smooth(&mut again, &lot, planner.turning_radius(), &smoother, 5.0);
                let worst = plan
                    .poses
                    .iter()
                    .filter_map(|p| lot.cell(p.x, p.y).map(|(cx, cy)| lot.get(cx, cy)))
                    .max()
                    .unwrap_or(0);
                println!(
                    "| {name} | {:.2} | {done} | {worst} | {:.0} |",
                    tightest_turn(&plan.poses, 1.0),
                    plan.total_time * 1e3
                );
            }
            Err(e) => println!("| {name} | {e} | | | |"),
        }
        from = to;
    }
}
