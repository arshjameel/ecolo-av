//! How long one cloud takes: the mount, then the cubes (and the steps), ms per cloud.
//!
//! ```sh
//! python3 ros/perception/aav_voxels_rs/test/capture_python_vectors.py --bench /tmp/cloud.txt
//! cargo run --release -p voxels --example timing -- /tmp/cloud.txt
//! ```
//!
//! The first line times the Python on the same cloud; this, the Rust. With no file, the
//! recorded test clouds (smaller: every 10th step of the turn).

#[path = "../tests/vectors/mod.rs"]
mod vectors;

use std::time::Instant;

use voxels::{cloud, cubes, steps, usable, Point, StepShape};

const RUNS: u32 = 200;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let text = match std::env::args().nth(1) {
        Some(path) => std::fs::read_to_string(path)?,
        None => include_str!("../tests/python_vectors.txt").to_string(),
    };
    let (shape, clouds) = vectors::read(&text);
    for c in &clouds {
        let run = || {
            let base: Vec<Point> = c.lidar.iter().map(|p| c.mount.to_base(p)).collect();
            cubes(&base, &shape)
        };
        let found = run(); // warm up
        let start = Instant::now();
        for _ in 0..RUNS {
            std::hint::black_box(run());
        }
        let ms = start.elapsed().as_secs_f64() * 1e3 / f64::from(RUNS);
        // The kerbs' step finder, as the node runs it with steps:=true.
        let base: Vec<Point> = c.lidar.iter().map(|p| c.mount.to_base(p)).collect();
        let good = usable(&base, &shape);
        let start = Instant::now();
        for _ in 0..RUNS {
            std::hint::black_box(steps(&good, &StepShape::default()));
        }
        let steps_ms = start.elapsed().as_secs_f64() * 1e3 / f64::from(RUNS);
        // And the bytes either side, as the node does them.
        let bytes = cloud::write_xyz32(&c.lidar);
        let start = Instant::now();
        for _ in 0..RUNS {
            let view = cloud::CloudView {
                fields: cloud::XYZ32_FIELDS.to_vec(),
                width: u32::try_from(c.lidar.len())?,
                height: 1,
                point_step: cloud::XYZ32_STEP,
                row_step: cloud::XYZ32_STEP * u32::try_from(c.lidar.len())?,
                is_bigendian: false,
                data: &bytes,
            };
            let points = cloud::read_xyz(&view)?;
            std::hint::black_box(cloud::write_xyz32(&found.obstacles));
            std::hint::black_box(cloud::write_xyz32(&found.ground));
            std::hint::black_box(points);
        }
        let io_ms = start.elapsed().as_secs_f64() * 1e3 / f64::from(RUNS);
        println!(
            "{}: {} points, {} cubes. Rust: to_base + cubes {ms:.2} ms per cloud \
             (+ {io_ms:.2} ms reading and writing the bytes; + {steps_ms:.2} ms the steps), \
             mean of {RUNS}",
            c.name,
            c.lidar.len(),
            found.obstacles.len()
        );
    }
    Ok(())
}
