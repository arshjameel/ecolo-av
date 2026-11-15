//! Nav2's moves against the smooth ones (`lattice_moves:=smooth`), on the test lot's tour.
//!
//! ```sh
//! cargo run --release -p lattice-planner --example moves              # the table
//! cargo run --release -p lattice-planner --example moves -- /tmp/out  # and each path, CSV
//! ```
//!
//! Plans `measure controllers --tours lot`'s four legs on `example_lot` (the map Nav2 plans
//! on in the sim) with each primitives file, forward only and reversing, and prints per
//! leg: length, planning time, and how fast the path's curvature changes (1/m^2, over
//! 0.5 m windows: a straight that meets a full-lock arc shows about 0.56; the steering
//! keeps up with 0.31 at 1 m/s and 0.12 at 2.5 m/s, less the margin). The smoothed path
//! Each with the smoother on (nav2.yaml's `smooth_path: true`: Nav2's smoother, which
//! joins each stretch's ends to its start and goal with Dubins curves) and off. With a
//! folder, each path goes to `<folder>/<moves>-<on|off>-<way>-<leg>.csv` (x,y,yaw), for
//! mppi-lag's `follow_plan` example.

use std::f64::consts::PI;
use std::path::Path;

use lattice_planner::{cusps, length, scenes, Lattice, Planner, Pose, Settings};

const CONFIG: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../../aav_navigation/config"
);
/// The window curvature is measured over (m).
const WINDOW: f64 = 0.5;

/// Curvature (1/m) every `WINDOW` m along each stretch between cusps.
fn curvatures(poses: &[Pose]) -> Vec<Vec<f64>> {
    let mut stretches = Vec::new();
    let mut s = vec![0.0];
    let mut yaw = vec![poses[0].yaw];
    let mut flush = |s: &mut Vec<f64>, yaw: &mut Vec<f64>| {
        let mut ks = Vec::new();
        let mut at = 0.0;
        let total = *s.last().unwrap();
        let yaw_at = |d: f64| {
            let j = s.partition_point(|&x| x < d).clamp(1, s.len() - 1);
            let f = ((d - s[j - 1]) / (s[j] - s[j - 1]).max(1e-9)).clamp(0.0, 1.0);
            yaw[j - 1] + f * (yaw[j] - yaw[j - 1])
        };
        while at + WINDOW <= total {
            ks.push((yaw_at(at + WINDOW) - yaw_at(at)) / WINDOW);
            at += WINDOW;
        }
        stretches.push(ks);
    };
    for w in poses.windows(3) {
        let d = (w[1].x - w[0].x).hypot(w[1].y - w[0].y);
        let turn = (w[1].yaw - w[0].yaw + PI).rem_euclid(2.0 * PI) - PI;
        s.push(s.last().unwrap() + d);
        yaw.push(yaw.last().unwrap() + turn);
        let back = (w[1].x - w[0].x) * (w[2].x - w[1].x) + (w[1].y - w[0].y) * (w[2].y - w[1].y);
        if back < 0.0 {
            flush(&mut s, &mut yaw);
            s = vec![0.0];
            yaw = vec![w[1].yaw];
        }
    }
    flush(&mut s, &mut yaw);
    stretches
}

/// (the largest curvature change, 1/m^2; metres where it is over `limit`; turns from left
/// to right or back).
fn smoothness(poses: &[Pose], limit: f64) -> (f64, f64, usize) {
    let mut worst: f64 = 0.0;
    let mut over = 0.0;
    let mut flips = 0;
    for ks in curvatures(poses) {
        for w in ks.windows(2) {
            let rate = (w[1] - w[0]).abs() / WINDOW;
            worst = worst.max(rate);
            if rate > limit {
                over += WINDOW;
            }
        }
        // A flip: the sign changes between two bends of at least 0.02 1/m (50 m radius).
        let signs: Vec<f64> = ks
            .iter()
            .filter(|k| k.abs() > 0.02)
            .map(|k| k.signum())
            .collect();
        flips += signs.windows(2).filter(|w| w[0] != w[1]).count();
    }
    (worst, over, flips)
}

fn main() {
    let out = std::env::args().nth(1);
    let grid = scenes::map("example_lot").expect("the example lot");
    println!(
        "Curvature change: the largest over {WINDOW} m windows (1/m^2); 'over' = metres above \
         0.31 1/m^2, the most the steering (0.6 rad/s) keeps up with at 1 m/s"
    );
    println!();
    println!("| moves | reversing | leg | length m | cusps | plan ms | worst change | over m | left-right flips |");
    println!("|---|---|---|---|---|---|---|---|---|");
    // MOVES_FILE: another smooth file to try (tools/lattice_smooth.py --speed, elsewhere).
    let smooth_file = std::env::var("MOVES_FILE")
        .unwrap_or_else(|_| format!("{CONFIG}/lattice_ackermann_smooth.json"));
    for (moves, smoothed) in [
        ("nav2", true),
        ("nav2", false),
        ("smooth", true),
        ("smooth", false),
    ] {
        let file = match moves {
            "nav2" => format!("{CONFIG}/lattice_ackermann.json"),
            _ => smooth_file.clone(),
        };
        let text = std::fs::read_to_string(file).expect("the file");
        let label = if smoothed {
            format!("{moves}, smoother on")
        } else {
            format!("{moves}, smoother off")
        };
        let moves = &label;
        for reversing in [false, true] {
            let lattice = Lattice::parse(&text).expect("a good lattice file");
            let settings = Settings {
                allow_reverse: reversing,
                smooth_path: smoothed,
                ..Settings::default()
            };
            let planner = Planner::new(
                lattice,
                settings,
                &scenes::outline(),
                Some(&scenes::inflation()),
            );
            let mut from = scenes::LOT_START;
            let mut sums = (0.0, 0.0);
            for (n, (name, x, y, deg)) in scenes::LOT_TOUR.iter().enumerate() {
                let goal = Pose::new(*x, *y, deg.to_radians());
                // The best of 3 for the time.
                let mut best: Option<lattice_planner::Plan> = None;
                for _ in 0..3 {
                    match planner.plan(&grid, &from, &goal, &|| false) {
                        Ok(p) => {
                            if best.as_ref().is_none_or(|b| p.total_time < b.total_time) {
                                best = Some(p);
                            }
                        }
                        Err(e) => {
                            println!("| {moves} | {reversing} | {name} | {e} | | | | | |");
                            break;
                        }
                    }
                }
                if let Some(p) = best {
                    let (worst, over, flips) = smoothness(&p.poses, 0.31);
                    sums.0 += length(&p.poses);
                    sums.1 += p.total_time;
                    println!(
                        "| {moves} | {reversing} | {name} | {:.1} | {} | {:.0} | {worst:.2} | {over:.1} | {flips} |",
                        length(&p.poses),
                        cusps(&p.poses),
                        p.total_time * 1e3,
                    );
                    if let Some(dir) = &out {
                        let way = if reversing { "reversing" } else { "forward" };
                        let name = moves.replace(", smoother ", "-");
                        let path = Path::new(dir).join(format!("{name}-{way}-{n}.csv"));
                        let rows: String = p
                            .poses
                            .iter()
                            .map(|q| format!("{},{},{}\n", q.x, q.y, q.yaw))
                            .collect();
                        std::fs::write(path, rows).expect("write the CSV");
                    }
                }
                from = goal;
            }
            println!(
                "| {moves} | {reversing} | **whole tour** | {:.1} | | {:.0} | | | |",
                sums.0,
                sums.1 * 1e3
            );
        }
    }
}
