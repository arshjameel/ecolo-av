//! How long one 1280 x 720 picture takes to undistort, and to shrink into a preview.
//!
//!   cargo run --release -p stream-core --example undistort_time [calibration.ini]

use std::path::Path;
use std::time::Instant;

use stream_core::{lens, preview};

fn main() -> Result<(), stream_core::Error> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "../aav_sensors/config/cameras/left.ini".to_string());
    let lens = lens::scaled(&lens::load(Path::new(&path))?, 1280, 720)?;
    let undistorter = lens::Undistorter::new(&lens, 0.0)?;
    let picture: Vec<u8> = (0..1280 * 720 * 3).map(|i| (i % 251) as u8).collect();
    let runs = 50;
    let t = Instant::now();
    for _ in 0..runs {
        std::hint::black_box(undistorter.apply(&picture)?);
    }
    let undistort = t.elapsed().as_secs_f64() * 1000.0 / f64::from(runs);
    let t = Instant::now();
    for _ in 0..runs {
        std::hint::black_box(preview::jpeg(&picture, 1280, 720, 320)?);
    }
    let jpeg = t.elapsed().as_secs_f64() * 1000.0 / f64::from(runs);
    println!("undistort {undistort:.2} ms, preview {jpeg:.2} ms a picture");
    Ok(())
}
