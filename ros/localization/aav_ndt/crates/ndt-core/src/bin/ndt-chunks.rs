//! ndt-chunks: cut an NDT map (.pcd) into one .pcd per 64 m chunk (TODO 175, step 4),
//! and say how long the whole map and the 3 x 3 round a point take to build.
//!
//! ```text
//! cargo run --release --bin ndt-chunks -- maps/ndt/campus.pcd maps/campus/chunks/ndt \
//!     -900 -900 [x y]
//! ```
//!
//! The grid's origin must be the occupancy chunks' (chunks.yaml), so both line up.

use std::fs::File;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use map_chunks::{Grid, Point as FlatPoint};
use ndt_core::chunks::{write_chunks, ChunkedMap};
use ndt_core::{pcd, CubeMap, MapSettings};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let numbers: Vec<f64> = args.iter().skip(2).filter_map(|s| s.parse().ok()).collect();
    let (Some(map), Some(out), [ox, oy, rest @ ..]) = (args.first(), args.get(1), &numbers[..])
    else {
        eprintln!("usage: ndt-chunks <map.pcd> <out folder> <origin x> <origin y> [x y]");
        return ExitCode::from(2);
    };
    let at = match rest {
        [x, y, ..] => FlatPoint::new(*x, *y),
        _ => FlatPoint::new(0.0, 0.0),
    };
    match run(map, out, FlatPoint::new(*ox, *oy), at) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("ndt-chunks: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(
    map: &str,
    out: &str,
    origin: FlatPoint,
    at: FlatPoint,
) -> Result<(), Box<dyn std::error::Error>> {
    let settings = MapSettings::with_cube(1.0);
    let grid = Grid::new(origin, 64.0)?;
    let points = pcd::read(File::open(map)?)?;
    let start = Instant::now();
    let whole = CubeMap::build(&points, &settings)?;
    let whole_s = start.elapsed().as_secs_f64();
    let out = PathBuf::from(out);
    let written = write_chunks(&out, &grid, &points)?;
    let start = Instant::now();
    let mut chunked = ChunkedMap::new(&out, grid, settings)?;
    chunked.update(at, &[], 0.0)?;
    let near_s = start.elapsed().as_secs_f64();
    let near = chunked.map().map_or(0, CubeMap::len);
    println!(
        "{} points, {written} chunks in {}\n\
         whole map: {} cubes, built in {whole_s:.2} s\n\
         3 x 3 round ({:.0}, {:.0}): {near} cubes ({:.1} %), loaded and built in {near_s:.2} s",
        points.len(),
        out.display(),
        whole.len(),
        at.x,
        at.y,
        100.0 * near as f64 / whole.len().max(1) as f64
    );
    Ok(())
}
