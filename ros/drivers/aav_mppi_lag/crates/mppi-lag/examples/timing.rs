//! How long one MPPI tick takes: `cargo run --release -p mppi-lag --example timing`.
//!
//! Nav2's sizes (2000 samples x 56 steps), the lag model, all eight critics and a 20 x 20 m
//! costmap at 0.1 m with a wall and its inflation, as the node runs them; and our
//! `LaneCritic` on a 40 m lane (the road ahead's, a point every 0.5 m), each
//! size without the lane (LaneCritic silent) and with it. Each line: the
//! threads, the samples, then the mean and worst tick over 200 ticks.

use std::time::Instant;

use mppi_lag::critics::{inflation_cost, NAMES, OWN};
use mppi_lag::{
    CarModel, Command, Costmap, Critic, Driven, Edges, Ego, GlobalPath, LagModel, Lane, LanePoint,
    Optimizer, Pose, Settings,
};

fn costmap() -> Costmap {
    // A wall 6 m ahead, inflated as nav2.yaml's local costmap does (3.0, inscribed 0.25).
    let (w, h, res) = (200usize, 200usize, 0.1);
    let data = (0..w * h)
        .map(|i| {
            let x = (i % w) as f64 * res - 10.0;
            let d = (x - 6.0).abs();
            if d > 2.0 {
                0
            } else {
                inflation_cost(d, 0.25, 3.0) as u8
            }
        })
        .collect();
    Costmap::new((-10.0, -10.0), res, w, h, data).expect("a costmap")
}

fn main() {
    let critics: Vec<Critic> = NAMES
        .iter()
        .chain(&OWN)
        .filter_map(|n| Critic::by_name(n))
        .collect();
    let model = Driven {
        ego: Ego::default(),
        car: LagModel::default(),
    };
    let poses: Vec<Pose> = (0..=200)
        .map(|i| {
            let x = f64::from(i) * 0.05;
            Pose::new(x, 0.5 * (x / 3.0).sin(), 0.0)
        })
        .collect();
    let map = costmap();
    let lane = Lane::new((0..=80).map(|i| LanePoint {
        x: f64::from(i) * 0.5,
        y: 0.0,
        edges: Edges::from_numbers(2.25, 2.25, 1.75, 1.75),
    }))
    .expect("a lane");
    let footprint = [(1.85, 0.55), (1.85, -0.55), (-0.25, -0.55), (-0.25, 0.55)];
    let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
    let runs = [(0, 2000), (0, 1000), (4, 2000), (1, 2000), (1, 1000)];
    for ((threads, batch), with_lane) in runs.into_iter().flat_map(|r| [(r, false), (r, true)]) {
        let settings = Settings {
            batch_size: batch,
            threads,
            ..Settings::default()
        };
        let mut opt = Optimizer::new(model, settings, critics.clone());
        let mut path = GlobalPath::new(poses.clone()).expect("a path");
        let state = model.at_rest(Pose::default(), 1.0);
        opt.reset(Command::new(1.0, 0.0));
        let goal = path.goal();
        let window = path
            .window(&Pose::default(), 10.0, 3.0, Some(&map))
            .to_vec();
        let scene = mppi_lag::Scene {
            path: &window,
            goal,
            costmap: Some(&map),
            footprint: &footprint,
            tracking_unknown: false,
            lane: with_lane.then_some(&lane),
        };
        let mut times = Vec::new();
        for _ in 0..200 {
            let start = Instant::now();
            let _ = opt.evaluate(&state, &scene);
            times.push(start.elapsed().as_secs_f64() * 1000.0);
        }
        let mean = times.iter().sum::<f64>() / times.len() as f64;
        let worst = times.iter().copied().fold(0.0, f64::max);
        let used = if threads == 0 { cores } else { threads };
        let lane = if with_lane { "with a lane" } else { "no lane" };
        println!("{used:2} threads, {batch} samples x 56 steps, {lane:11}: {mean:6.2} ms mean, {worst:6.2} ms worst");
    }
}
