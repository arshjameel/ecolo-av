//! Costmap updates, measured properly (TODO 203): `cargo bench -p costmap`.
//!
//! The timing example's cases: the local costmap (20 x 20 m at 0.1 m, a lidar layer, a
//! 720-ray scan with a wall and posts) moving along; the global one on the campus map's
//! size (6400 x 6400 cells at 0.25 m), steady, and with a lidar layer (the scan's area).

use std::f64::consts::{PI, TAU};
use std::hint::black_box;

use costmap::obstacle::scan_points;
use costmap::{footprint, Costmap, Observation, ObstacleLayer, Pose2, Settings, Source, StaticMap};
use criterion::{criterion_group, criterion_main, Criterion};

/// A wall 4 m to the left, posts every 30 degrees ahead at 6 m, inf elsewhere.
fn scan() -> Vec<[f64; 3]> {
    let n = 720;
    let step = TAU / n as f64;
    let ranges: Vec<f32> = (0..n)
        .map(|i| {
            let a = -PI + i as f64 * step;
            if a.sin() > 0.3 {
                (4.0 / a.sin()) as f32
            } else if a.cos() > 0.5 && (i % 60) < 4 {
                6.0
            } else {
                f32::INFINITY
            }
        })
        .collect();
    scan_points(-PI, step, 0.2, 30.0, &ranges, true)
}

fn lidar() -> Source {
    Source {
        clearing: true,
        min_obstacle_height: -0.5,
        max_obstacle_height: 0.5,
        obstacle_max_range: 12.0,
        raytrace_max_range: 15.0,
        inf_is_valid: true,
        ..Source::named("lidar")
    }
}

/// The scan seen from `pose` (x only), in the costmap's frame.
fn seen(points: &[[f64; 3]], pose: &Pose2) -> Observation {
    Observation {
        origin: [pose.x, pose.y, 0.0],
        points: points.iter().map(|&[x, y, z]| [x + pose.x, y, z]).collect(),
    }
}

fn update(c: &mut Criterion) {
    let car = footprint::padded(
        &footprint::parse("[[1.8, 0.5], [1.8, -0.5], [-0.2, -0.5], [-0.2, 0.5]]")
            .expect("a footprint"),
        0.05,
    );
    let points = scan();

    let local_settings = Settings {
        rolling: true,
        width: 20.0,
        height: 20.0,
        resolution: 0.1,
        track_unknown: false,
        footprint: car.clone(),
        inflation: Some((2.0, 3.0)),
        static_layer: true,
    };
    let probe = Costmap::new(local_settings.clone(), Vec::new());
    let layer = ObstacleLayer::new("lidar_layer", vec![lidar()], probe.grid(), false);
    let mut local = Costmap::new(local_settings, vec![layer]);
    let mut k = 0u32;
    c.bench_function("local, 200 x 200, a scan, moving", |b| {
        b.iter(|| {
            k += 1;
            let pose = Pose2::new(f64::from(k % 500) * 0.02, 0.0, 0.0);
            let obs = seen(&points, &pose);
            local.update(&pose, None, &[vec![vec![&obs]]]);
            black_box(local.grid().data[0])
        })
    });

    // A campus-sized map: walls every 200 rows and 300 columns.
    let (w, h) = (6400usize, 6400usize);
    let values: Vec<i8> = (0..w * h)
        .map(|i| {
            if (i / w) % 200 == 0 || (i % w) % 300 == 0 {
                100
            } else {
                0
            }
        })
        .collect();
    let map =
        StaticMap::from_occupancy((-800.0, -800.0), 0.25, w, h, &values, true).expect("a map");
    let global_settings = Settings {
        rolling: false,
        width: 0.0,
        height: 0.0,
        resolution: 0.25,
        track_unknown: true,
        footprint: car,
        inflation: Some((3.0, 1.0)),
        static_layer: true,
    };
    let mut group = c.benchmark_group("global, 6400 x 6400");
    group.sample_size(20);
    let mut global = Costmap::new(global_settings.clone(), Vec::new());
    global.set_map(map.clone());
    global.update(&Pose2::IDENTITY, None, &[]);
    group.bench_function("steady, nothing changed", |b| {
        b.iter(|| global.update(&Pose2::IDENTITY, None, &[]))
    });

    let probe = Costmap::new(global_settings.clone(), Vec::new());
    let layer = ObstacleLayer::new("lidar_layer", vec![lidar()], probe.grid(), true);
    let mut sensed = Costmap::new(global_settings, vec![layer]);
    sensed.set_map(map);
    sensed.update(&Pose2::IDENTITY, None, &[]);
    let mut k = 0u32;
    group.bench_function("a lidar layer, the scan's area", |b| {
        b.iter(|| {
            k += 1;
            let pose = Pose2::new(f64::from(k % 100) * 0.5, 0.0, 0.0);
            let obs = seen(&points, &pose);
            sensed.update(&pose, None, &[vec![vec![&obs]]]);
        })
    });
    group.finish();
}

criterion_group!(benches, update);
criterion_main!(benches);
