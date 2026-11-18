//! The global costmap from map chunks, timed on a real map (TODO 175, 195), each step on
//! one core and on all of them (rayon):
//!
//! ```sh
//! cargo run --release -p costmap --example chunks -- <chunks folder> [<map.yaml>] [x y]...
//! cargo run --release -p costmap --example chunks -- ../../../maps/campus/chunks \
//!     ../../../maps/campus/campus.yaml
//! ```
//!
//! For the chunks round the car at each (x, y) given (the campus tour's starts by
//! default): one chunk read, one stitched, the first window (read, stitch, inflate), the
//! window moving a chunk; and with `<map.yaml>`, the whole map worked out, as today.

use std::path::Path;
use std::time::Instant;

use costmap::chunked::stitch;
use costmap::{footprint, ChunkSource, Costmap, Pose2, Settings, StaticMap};
use map_chunks::files::read_map;

/// nav2.yaml's global costmap: 0.25 m, unknown kept, 3 m inflation at 1.0.
fn settings() -> Settings {
    let outline = footprint::parse("[[1.8, 0.5], [1.8, -0.5], [-0.2, -0.5], [-0.2, 0.5]]");
    Settings {
        rolling: false,
        width: 0.0,
        height: 0.0,
        resolution: 0.25,
        track_unknown: true,
        footprint: footprint::padded(&outline.expect("an outline"), 0.05),
        inflation: Some((3.0, 1.0)),
        static_layer: true,
    }
}

fn ms(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}

/// The median of `runs` timings of `f`, ms.
fn median(runs: usize, mut f: impl FnMut() -> f64) -> f64 {
    let mut t: Vec<f64> = (0..runs).map(|_| f()).collect();
    t.sort_by(f64::total_cmp);
    t[t.len() / 2]
}

fn one_pass(source: &ChunkSource, cars: &[(f64, f64)]) {
    let runs = 5;
    let key = source
        .grid
        .key_at(map_chunks::Point::new(cars[0].0, cars[0].1));
    let read = median(20, || {
        let t = Instant::now();
        let _ = source.load(key, true).expect("a chunk");
        ms(t)
    });
    let window = source.window(cars[0], &[]);
    let side = source.side();
    let tile = source.load(key, true).expect("a chunk");
    let mut grid = source.grid_for(&window, 255);
    let stitched = median(20, || {
        let t = Instant::now();
        stitch(&mut grid, &window, key, side, &tile);
        ms(t)
    });
    println!("  one chunk: read {read:.2} ms, stitched {stitched:.3} ms");
    for &car in cars {
        let (mut first_read, mut first_build) = (Vec::new(), Vec::new());
        let mut moved = Vec::new();
        let mut held = 0;
        for _ in 0..runs {
            let mut c = Costmap::new(settings(), Vec::new());
            c.set_chunks(source.clone()).expect("chunks");
            let t = Instant::now();
            c.follow_chunks(car).expect("the chunks");
            first_read.push(ms(t));
            let t = Instant::now();
            c.update(&Pose2::new(car.0, car.1, 0.0), None, &[]);
            first_build.push(ms(t));
            held = c.chunk_window().map_or(0, |(_, n)| n);
            // One chunk east: read the new column, redo what changed.
            let next = (car.0 + 64.0, car.1);
            let t = Instant::now();
            c.follow_chunks(next).expect("the chunks");
            c.update(&Pose2::new(next.0, next.1, 0.0), None, &[]);
            moved.push(ms(t));
        }
        let med = |v: &mut Vec<f64>| {
            v.sort_by(f64::total_cmp);
            v[v.len() / 2]
        };
        let (r, b, m) = (med(&mut first_read), med(&mut first_build), med(&mut moved));
        println!(
            "  car at ({:.0}, {:.0}): {held} chunks; read and stitched {r:.1} ms, inflated \
             {b:.1} ms ({:.2} ms a chunk), full {:.1} ms; a chunk east {m:.1} ms",
            car.0,
            car.1,
            b / held.max(1) as f64,
            r + b
        );
    }
}

fn whole(yaml: &Path, check: Option<(&ChunkSource, (f64, f64))>) {
    let t = Instant::now();
    let map = read_map(yaml).expect("the map");
    let read = ms(t);
    let w = map.whole();
    let stat = StaticMap::from_occupancy(
        (w.origin.x, w.origin.y),
        w.resolution,
        w.width,
        w.height,
        w.data,
        true,
    )
    .expect("the map's size and data agree");
    let build = median(3, || {
        let mut c = Costmap::new(settings(), Vec::new());
        c.set_map(stat.clone());
        let t = Instant::now();
        c.update(&Pose2::IDENTITY, None, &[]);
        ms(t)
    });
    println!(
        "  whole map {} x {} ({:.1} M cells): read {read:.0} ms, worked out {build:.0} ms",
        w.width,
        w.height,
        (w.width * w.height) as f64 / 1e6
    );
    // The same cells: the chunks' window against the whole map, away from its edge.
    if let Some((source, car)) = check {
        let mut whole = Costmap::new(settings(), Vec::new());
        whole.set_map(stat);
        whole.update(&Pose2::IDENTITY, None, &[]);
        let mut c = Costmap::new(settings(), Vec::new());
        c.set_chunks(source.clone()).expect("chunks");
        c.follow_chunks(car).expect("the chunks");
        c.update(&Pose2::new(car.0, car.1, 0.0), None, &[]);
        let (g, all) = (c.grid(), whole.grid());
        let dx = ((g.origin_x - all.origin_x) / 0.25).round() as usize;
        let dy = ((g.origin_y - all.origin_y) / 0.25).round() as usize;
        let edge = 14;
        let differ = (edge..g.height - edge)
            .flat_map(|y| (edge..g.width - edge).map(move |x| (x, y)))
            .filter(|&(x, y)| g.get(x, y) != all.get(x + dx, y + dy))
            .count();
        println!("  the chunks' window against the whole map: {differ} cells differ");
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(dir) = args.first() else {
        eprintln!("usage: chunks <chunks folder> [<map.yaml>] [x y]...");
        std::process::exit(2);
    };
    let yaml = args.get(1).filter(|a| a.ends_with(".yaml"));
    let numbers: Vec<f64> = args
        .iter()
        .skip(if yaml.is_some() { 2 } else { 1 })
        .filter_map(|a| a.parse().ok())
        .collect();
    let mut cars: Vec<(f64, f64)> = numbers
        .as_chunks::<2>()
        .0
        .iter()
        .map(|p| (p[0], p[1]))
        .collect();
    if cars.is_empty() {
        // The campus tour's starts (tools/controller_compare.py): P4, and two legs on.
        cars = vec![(-10.0, 10.0), (73.0, -38.0), (165.0, -111.0)];
    }
    let source = ChunkSource::open(Path::new(dir), 100.0, 1).expect("chunks.yaml");
    let cores = std::thread::available_parallelism().map_or(1, usize::from);
    for threads in [1, cores] {
        println!("{threads} thread(s):");
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("a thread pool");
        pool.install(|| {
            one_pass(&source, &cars);
            if let Some(y) = yaml {
                whole(Path::new(y), (threads == 1).then_some((&source, cars[0])));
            }
        });
    }
}
