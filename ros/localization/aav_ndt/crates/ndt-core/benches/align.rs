//! One NDT match, measured properly (TODO 203): `cargo bench -p ndt-core`.
//!
//! The matching test's yard (tests/common): a map made by driving round it, a scan from a
//! known pose, filtered as the node does, and a guess 0.5 m and 4° off; the match at the
//! node's two cube sizes (2 m then 1 m), and building the cubes from the map.

use std::hint::black_box;

use criterion::{criterion_group, criterion_main, Criterion};
use ndt_core::{CubeMap, MapSettings, MatchSettings, Matcher, Pose2, ScanFilter};

#[path = "../tests/common/mod.rs"]
mod common;

use common::{map_from_route, route, Rng, World};

fn align(c: &mut Criterion) {
    let world = World::yard();
    let mut rng = Rng::new(1);
    let corners = [
        (-14.0, -8.0),
        (14.0, -8.0),
        (14.0, 10.0),
        (-14.0, 10.0),
        (-14.0, -8.0),
    ];
    let map = map_from_route(&world, &route(&corners, 2.0), &mut rng);
    let truth = Pose2::new(1.0, 2.0, 0.4);
    let (_, mount) = World::mount();
    let scan = ScanFilter::default().apply(
        world
            .scan(&truth, 1800, 0.03, &mut rng)
            .iter()
            .map(|p| mount.to_base_link(p)),
    );
    let guess = Pose2::new(truth.x + 0.4, truth.y - 0.3, truth.yaw + 4f64.to_radians());
    let ndt = Matcher::new(
        &map,
        &[2.0, 1.0],
        MapSettings::with_cube,
        MatchSettings::default(),
    )
    .expect("the map has cubes");

    c.bench_function("ndt match, yard, 0.5 m and 4 degrees off", |b| {
        b.iter(|| black_box(ndt.align(&scan, guess)))
    });
    c.bench_function("ndt cubes, yard map, 1 m", |b| {
        b.iter(|| black_box(CubeMap::build(&map, &MapSettings::with_cube(1.0))))
    });
}

criterion_group!(benches, align);
criterion_main!(benches);
