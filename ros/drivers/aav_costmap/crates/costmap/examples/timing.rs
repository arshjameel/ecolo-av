//! How long an update takes: `cargo run --release -p costmap --example timing`.
//!
//! The local costmap (20 m x 20 m at 0.1 m, the lidar layer, a 720-ray scan with a wall
//! and posts in it), and the global one on the campus map's size (6400 x 6400 cells at
//! 0.25 m): its first update, the steady ones after, and with a lidar layer on it.

use std::time::Instant;

use costmap::obstacle::scan_points;
use costmap::{footprint, Costmap, Observation, ObstacleLayer, Pose2, Settings, Source, StaticMap};

fn main() {
    let car = footprint::padded(
        &footprint::parse("[[1.8, 0.5], [1.8, -0.5], [-0.2, -0.5], [-0.2, 0.5]]").unwrap(),
        0.05,
    );
    let settings = Settings {
        rolling: true,
        width: 20.0,
        height: 20.0,
        resolution: 0.1,
        track_unknown: false,
        footprint: car.clone(),
        inflation: Some((2.0, 3.0)),
        static_layer: true,
    };
    let probe = Costmap::new(settings.clone(), Vec::new());
    let source = Source {
        clearing: true,
        min_obstacle_height: -0.5,
        max_obstacle_height: 0.5,
        obstacle_max_range: 12.0,
        raytrace_max_range: 15.0,
        inf_is_valid: true,
        ..Source::named("lidar")
    };
    let lidar = source.clone();
    let layer = ObstacleLayer::new("lidar_layer", vec![source], probe.grid(), false);
    let mut local = Costmap::new(settings, vec![layer]);
    // A wall 4 m to the left, posts every 30 degrees ahead at 6 m, inf elsewhere.
    let n = 720;
    let step = std::f64::consts::TAU / n as f64;
    let ranges: Vec<f32> = (0..n)
        .map(|i| {
            let a = -std::f64::consts::PI + i as f64 * step;
            if a.sin() > 0.3 {
                (4.0 / a.sin()) as f32
            } else if a.cos() > 0.5 && (i % 60) < 4 {
                6.0
            } else {
                f32::INFINITY
            }
        })
        .collect();
    let points = scan_points(-std::f64::consts::PI, step, 0.2, 30.0, &ranges, true);
    let runs = 200;
    let t = Instant::now();
    for k in 0..runs {
        let pose = Pose2::new(k as f64 * 0.02, 0.0, 0.0);
        let obs = Observation {
            origin: [pose.x, pose.y, 0.0],
            points: points.iter().map(|&[x, y, z]| [x + pose.x, y, z]).collect(),
        };
        local.update(&pose, None, &[vec![vec![&obs]]]);
    }
    let local_ms = t.elapsed().as_secs_f64() * 1e3 / runs as f64;
    println!("local, 200 x 200 cells, a 720-ray scan: {local_ms:.2} ms an update");

    // The campus map (maps/campus: 6400 x 6400 cells at 0.25 m, 1.6 km square), with
    // nav2.yaml's global costmap: the static layer and a 3.0 m, 1.0 inflation.
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
    let map = StaticMap::from_occupancy((-800.0, -800.0), 0.25, w, h, &values, true).unwrap();
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
    let mut global = Costmap::new(global_settings.clone(), Vec::new());
    global.set_map(map.clone());
    let t = Instant::now();
    global.update(&Pose2::IDENTITY, None, &[]);
    let whole_ms = t.elapsed().as_secs_f64() * 1e3;
    println!("global, 6400 x 6400 cells (campus), the map's update (every cell): {whole_ms:.0} ms");
    let runs = 20;
    let t = Instant::now();
    for _ in 0..runs {
        global.update(&Pose2::IDENTITY, None, &[]);
    }
    let steady_ms = t.elapsed().as_secs_f64() * 1e3 / runs as f64;
    println!("global, campus, each update after (nothing changed): {steady_ms:.3} ms");

    // The same with a lidar layer on it: each update redoes the scan's area only.
    let probe = Costmap::new(global_settings.clone(), Vec::new());
    let layer = ObstacleLayer::new("lidar_layer", vec![lidar], probe.grid(), true);
    let mut sensed = Costmap::new(global_settings, vec![layer]);
    sensed.set_map(map);
    sensed.update(&Pose2::IDENTITY, None, &[]);
    let t = Instant::now();
    for k in 0..runs {
        let pose = Pose2::new(k as f64 * 0.5, 0.0, 0.0);
        let obs = Observation {
            origin: [pose.x, pose.y, 0.0],
            points: points.iter().map(|&[x, y, z]| [x + pose.x, y, z]).collect(),
        };
        sensed.update(&pose, None, &[vec![vec![&obs]]]);
    }
    let area_ms = t.elapsed().as_secs_f64() * 1e3 / runs as f64;
    println!("global, campus, with a lidar layer (the scan's area): {area_ms:.2} ms an update");

    // What the node does to publish it: the raw costs copied, the 0-100 grid made.
    let t = Instant::now();
    let raw = global.grid().data.clone();
    let grid: Vec<i8> = raw.iter().map(|&c| costmap::cost::occupancy(c)).collect();
    let publish_ms = t.elapsed().as_secs_f64() * 1e3;
    println!(
        "global, campus, the two messages' data made: {publish_ms:.0} ms ({} MB each)",
        grid.len() / 1_000_000
    );
}
