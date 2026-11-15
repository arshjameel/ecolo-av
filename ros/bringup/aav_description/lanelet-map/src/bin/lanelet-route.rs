//! lanelet-route: read a Lanelet2 map and route by lanes between two map points.
//!
//! ```text
//! cargo run --release --bin lanelet-route -- maps/campus/campus.osm 49 -71 400 30
//! ```
//!
//! Points from the map's `local_x`, `local_y` (as `tools/campus_lanelet.py` writes them).

use std::process::ExitCode;
use std::time::Instant;

use lanelet_map::{LaneletMap, Origin};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let numbers: Vec<f64> = args.iter().skip(1).filter_map(|s| s.parse().ok()).collect();
    let (Some(path), [x0, y0, x1, y1]) = (args.first(), &numbers[..]) else {
        eprintln!("usage: lanelet-route <map.osm> <from x> <from y> <to x> <to y>");
        return ExitCode::from(2);
    };
    let started = Instant::now();
    let map = match std::fs::read_to_string(path)
        .map_err(|e| e.to_string())
        .and_then(|text| LaneletMap::from_osm(&text, Origin::default()).map_err(|e| e.to_string()))
    {
        Ok(map) => map,
        Err(e) => {
            eprintln!("lanelet-route: {path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let read = started.elapsed().as_secs_f64();
    let (Some(from), Some(to)) = (map.nearest(*x0, *y0), map.nearest(*x1, *y1)) else {
        eprintln!("lanelet-route: the map has no lanelets");
        return ExitCode::FAILURE;
    };
    let started = Instant::now();
    let route = map.route(from, to);
    let ms = started.elapsed().as_secs_f64() * 1000.0;
    println!("{} lanelets, read in {read:.2} s", map.lanelets().count());
    match route {
        Some(route) => {
            let length: f64 = route
                .iter()
                .filter_map(|id| map.lanelet(*id))
                .map(|l| l.length())
                .sum();
            println!(
                "route {from} -> {to}: {} lanelets, {length:.0} m, {} stop lines, found in {ms:.2} ms",
                route.len(),
                map.stops_on(&route).len()
            );
            ExitCode::SUCCESS
        }
        None => {
            println!(
                "no way by lanes from {from} to {to} ({ms:.2} ms): {from} has {} following, {} beside",
                map.following(from).len(),
                map.beside(from).len()
            );
            ExitCode::FAILURE
        }
    }
}
