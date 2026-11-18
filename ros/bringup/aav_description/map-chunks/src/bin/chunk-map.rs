//! chunk-map: cut a map server's map into fixed chunks on disk (TODO 175, step 2).
//!
//! ```text
//! cargo run --release --bin chunk-map -- maps/campus/campus.yaml maps/campus/chunks [64]
//! ```
//!
//! The chunks start at the map's own origin; 64 m unless a size is given.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use map_chunks::files::{read_map, write_chunks};
use map_chunks::{Grid, Point};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(yaml), Some(out)) = (args.first(), args.get(1)) else {
        eprintln!("usage: chunk-map <map.yaml> <out folder> [chunk size m, 64]");
        return ExitCode::from(2);
    };
    let size = match args.get(2).map(|s| s.parse::<f64>()) {
        None => 64.0,
        Some(Ok(size)) => size,
        Some(Err(_)) => {
            eprintln!("the chunk size must be a number of metres");
            return ExitCode::from(2);
        }
    };
    let start = Instant::now();
    let result = read_map(&PathBuf::from(yaml)).and_then(|map| {
        let origin = Point::new(map.yaml.origin[0], map.yaml.origin[1]);
        let grid = Grid::new(origin, size).map_err(map_chunks::files::FileError::Grid)?;
        write_chunks(&grid, &map.whole(), &PathBuf::from(out))
    });
    match result {
        Ok((fine, coarse)) => {
            println!(
                "{fine} fine and {coarse} coarse chunks of {size} m in {out}, {:.1} s",
                start.elapsed().as_secs_f64()
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("chunk-map: {e}");
            ExitCode::FAILURE
        }
    }
}
