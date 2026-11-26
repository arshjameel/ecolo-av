//! The ablation: the same hostile drivers, with the filter's parts taken away one at a time.
//!
//! ```sh
//! cargo run --release -p safety-filter --example ablation            # the table
//! cargo run --release -p safety-filter --example ablation -- --csv   # one row per run
//! ```
//!
//! | Variant | Speed barrier | Lane barriers | Lyapunov term |
//! |---|---|---|---|
//! | open | no | no | no |
//! | speed | yes | no | no |
//! | speed+lane | yes | yes | no |
//! | full | yes | yes | yes |
//!
//! Every variant sees the same drivers and starts (a fixed seed): paired runs.

use ego_core::stop_law::StopCurve;
use safety_filter::sim::{bending_lane, run, Car, CarModel, Tick};
use safety_filter::{Command, Lane, SafetyFilter, Settings};

const HALF_LANE: f64 = 1.5;
const HALF_WIDTH: f64 = 0.5;
const RUNS: usize = 300;

/// xorshift64*: a tiny fixed-seed generator, so every variant gets the same drivers.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next()
    }
}

#[derive(Clone)]
struct Driver {
    pieces: Vec<(f64, f64)>,
    hold: f64,
}

fn settings(clf: bool, open: bool) -> Settings {
    Settings {
        curve: StopCurve::new(1.0, 2.0, 0.35).expect("curve"),
        margin: if open { 0.0 } else { 0.5 },
        max_offset: if open { 1e9 } else { 3.0 },
        alpha_speed: 1.0,
        look_ahead: 1.5,
        alpha_lane: 1.0,
        half_width: HALF_WIDTH,
        lane_margin: 0.2,
        clf_rate: 0.5,
        clf_weight: if clf { 1.0 } else { 0.0 },
        wheelbase: 1.561,
        max_steer: if open { 1.5 } else { 0.5 },
        max_speed: if open { 100.0 } else { 2.0 },
        max_reverse: 0.5,
    }
}

#[derive(Default)]
struct Score {
    past_end: usize,
    over_line: usize,
    worst_over: f64,
    trim_v: f64,
    trim_w: f64,
    mean_abs_offset: f64,
    progress: f64,
    ticks: usize,
}

fn main() {
    let csv = std::env::args().any(|a| a == "--csv");
    let mut rng = Rng(0x00C0_FFEE_D00D_F00D);
    let drivers: Vec<(Driver, Car)> = (0..RUNS)
        .map(|_| {
            let n = 1 + (rng.next() * 10.0) as usize;
            let pieces = (0..n)
                .map(|_| (rng.range(0.0, 3.0), rng.range(-1.5, 1.5)))
                .collect();
            let car = Car {
                x: 1.0,
                y: rng.range(-0.4, 0.4),
                yaw: rng.range(-0.3, 0.3),
                v: rng.range(0.0, 1.5),
                steer: 0.0,
            };
            (
                Driver {
                    pieces,
                    hold: rng.range(0.3, 4.0),
                },
                car,
            )
        })
        .collect();
    let lane = bending_lane(30.0, HALF_LANE, 15.0);
    let no_edges = Lane {
        left: vec![f64::NAN; lane.len()],
        right: vec![f64::NAN; lane.len()],
        ..lane.clone()
    };
    let open_lane = Lane {
        end: f64::INFINITY,
        ..no_edges.clone()
    };
    let variants: [(&str, Settings, &Lane); 4] = [
        ("open", settings(false, true), &open_lane),
        ("speed", settings(false, false), &no_edges),
        ("speed+lane", settings(false, false), &lane),
        ("full", settings(true, false), &lane),
    ];
    if csv {
        println!("variant,run,past_end_m,over_line_m,trim_v,trim_w,mean_abs_offset_m,progress_m");
    } else {
        println!(
            "| Variant | Runs past the end | Runs over a line | Worst over (m) | Mean |v| trimmed (m/s) | Mean |w| trimmed (rad/s) | Mean |offset| (m) | Mean progress (m) |"
        );
        println!("|---|---|---|---|---|---|---|---|");
    }
    for (name, s, used) in variants {
        let mut score = Score::default();
        for (k, (d, car)) in drivers.iter().enumerate() {
            let mut f = SafetyFilter::new(s.clone());
            let d2 = d.clone();
            let ticks: Vec<Tick> = run(
                &mut f,
                &CarModel::default(),
                used,
                *car,
                40.0,
                20.0,
                |t, _| {
                    let i = ((t / d2.hold) as usize).min(d2.pieces.len() - 1);
                    Command {
                        v: d2.pieces[i].0,
                        w: d2.pieces[i].1,
                    }
                },
            );
            let (mut past, mut over, mut tv, mut tw, mut off, mut prog) =
                (f64::NEG_INFINITY, f64::NEG_INFINITY, 0.0, 0.0, 0.0, 0.0_f64);
            for t in &ticks {
                // Score against the real lane, whatever the variant saw.
                if let Some(fr) = lane.frame(t.car.x, t.car.y, t.car.yaw) {
                    past = past.max(-fr.room);
                    over = over.max(fr.offset.abs() + HALF_WIDTH - HALF_LANE);
                    off += fr.offset.abs();
                    prog = prog.max(fr.along.min(lane.end));
                }
                tv += (t.report.input.v.min(s.max_speed) - t.report.output.v).abs();
                tw += (t.report.input.w - t.report.output.w).abs();
            }
            let n = ticks.len().max(1) as f64;
            if csv {
                println!(
                    "{name},{k},{past:.4},{over:.4},{:.4},{:.4},{:.4},{prog:.3}",
                    tv / n,
                    tw / n,
                    off / n
                );
            }
            score.past_end += usize::from(past > 0.0);
            score.over_line += usize::from(over > 0.0);
            score.worst_over = score.worst_over.max(over);
            score.trim_v += tv;
            score.trim_w += tw;
            score.mean_abs_offset += off;
            score.progress += prog;
            score.ticks += ticks.len();
        }
        if !csv {
            let t = score.ticks.max(1) as f64;
            println!(
                "| {name} | {}/{RUNS} | {}/{RUNS} | {:.2} | {:.3} | {:.3} | {:.3} | {:.1} |",
                score.past_end,
                score.over_line,
                score.worst_over,
                score.trim_v / t,
                score.trim_w / t,
                score.mean_abs_offset / t,
                score.progress / RUNS as f64
            );
        }
    }
}
