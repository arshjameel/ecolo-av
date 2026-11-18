//! Planning, measured properly (TODO 203): `cargo bench -p lattice-planner`.
//!
//! The test lot tour's legs (`example_lot`, in git), forward only and reversing, the State
//! Lattice and the Hybrid-A*; and the campus tour's legs when `pixi run campus` has made
//! its map (else they are left out, and it says so).

use std::hint::black_box;

use costmap::Grid;
use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use lattice_planner::{scenes, Planner, Pose};

/// A planner's name, and how to make it (reversing or not).
type Make = (&'static str, fn(bool) -> Planner);

/// Each leg of a tour, from `start`, as (name, from, goal).
fn legs(start: Pose, tour: &[(&'static str, f64, f64, f64)]) -> Vec<(&'static str, Pose, Pose)> {
    let mut from = start;
    tour.iter()
        .map(|&(name, x, y, deg)| {
            let goal = Pose::new(x, y, deg.to_radians());
            let leg = (name, from, goal);
            from = goal;
            leg
        })
        .collect()
}

fn tour(c: &mut Criterion, group: &str, grid: &Grid, legs: &[(&'static str, Pose, Pose)]) {
    let mut group = c.benchmark_group(group);
    group.sample_size(10);
    let planners: [Make; 2] = [
        ("lattice", scenes::planner),
        ("hybrid", scenes::hybrid_planner),
    ];
    for (kind, make) in planners {
        for reversing in [false, true] {
            let planner = make(reversing);
            let way = if reversing { "reversing" } else { "forward" };
            for (name, from, goal) in legs {
                let id = BenchmarkId::new(format!("{kind} {way}"), name);
                group.bench_function(id, |b| {
                    b.iter(|| black_box(planner.plan(grid, from, goal, &|| false)))
                });
            }
        }
    }
    group.finish();
}

fn plan(c: &mut Criterion) {
    let lot = scenes::map("example_lot").expect("the lot's map, in git");
    tour(c, "lot", &lot, &legs(scenes::LOT_START, &scenes::LOT_TOUR));
    match scenes::campus() {
        Ok(campus) => tour(
            c,
            "campus",
            &campus,
            &legs(scenes::CAMPUS_START, &scenes::CAMPUS_TOUR),
        ),
        Err(why) => eprintln!("campus left out ({why}): pixi run campus makes its map"),
    }
}

criterion_group!(benches, plan);
criterion_main!(benches);
