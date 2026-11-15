//! One MPPI tick, measured properly (TODO 203): `cargo bench -p mppi-lag`.
//!
//! The timing example's scene (a 20 x 20 m costmap with a wall and its inflation, a gentle
//! S of a path, our `LaneCritic` on a 40 m lane), at the node's 2000 samples and the 1000
//! that TODO 152 proposes, with and without the lane, on every core. criterion warms up,
//! repeats until the spread is small, and says whether a change made it slower.

use std::hint::black_box;

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use mppi_lag::critics::{inflation_cost, NAMES, OWN};
use mppi_lag::{
    CarModel, Command, Costmap, Critic, Driven, Edges, Ego, GlobalPath, LagModel, Lane, LanePoint,
    Optimizer, Pose, Scene, Settings,
};

const FOOTPRINT: [(f64, f64); 4] = [(1.85, 0.55), (1.85, -0.55), (-0.25, -0.55), (-0.25, 0.55)];

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

fn tick(c: &mut Criterion) {
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
    let mut path = GlobalPath::new(poses).expect("a path");
    let goal = path.goal();
    let window = path
        .window(&Pose::default(), 10.0, 3.0, Some(&map))
        .to_vec();
    let state = model.at_rest(Pose::default(), 1.0);

    let mut group = c.benchmark_group("mppi tick");
    group.sample_size(30);
    for batch in [1000, 2000] {
        for with_lane in [false, true] {
            let settings = Settings {
                batch_size: batch,
                ..Settings::default()
            };
            let mut opt = Optimizer::new(model, settings, critics.clone());
            opt.reset(Command::new(1.0, 0.0));
            let scene = Scene {
                path: &window,
                goal,
                costmap: Some(&map),
                footprint: &FOOTPRINT,
                tracking_unknown: false,
                lane: with_lane.then_some(&lane),
            };
            let name = if with_lane { "lane" } else { "no lane" };
            group.bench_with_input(BenchmarkId::new(name, batch), &scene, |b, scene| {
                b.iter(|| black_box(opt.evaluate(&state, scene)))
            });
        }
    }
    group.finish();
}

criterion_group!(benches, tick);
criterion_main!(benches);
