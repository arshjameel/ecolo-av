//! The global costmap from map chunks (TODO 175) against the same costmap from the whole
//! map: the same cells wherever the window holds the walls that reach them.

use std::path::{Path, PathBuf};

use costmap::{footprint, ChunkSource, Costmap, Grid, Pose2, Settings, StaticMap};
use map_chunks::files::write_chunks;
use map_chunks::{Grid as ChunkGrid, Point, WholeMap};

/// m: the map's corner, and its side (5 x 5 chunks of 64 m).
const CORNER: f64 = -160.0;
const SIDE_M: f64 = 320.0;
const RES: f64 = 0.25;

fn settings() -> Settings {
    let outline = footprint::parse("[[1.8, 0.5], [1.8, -0.5], [-0.2, -0.5], [-0.2, 0.5]]");
    Settings {
        rolling: false,
        width: 0.0,
        height: 0.0,
        resolution: RES,
        track_unknown: true,
        footprint: footprint::padded(&outline.unwrap(), 0.05),
        inflation: Some((3.0, 1.0)),
        static_layer: true,
    }
}

/// A 320 m map: free, with walls and blocks scattered (a fixed pseudo-random draw, some
/// across chunk edges), and an unknown corner.
fn map_cells() -> (usize, Vec<i8>) {
    let n = (SIDE_M / RES) as usize;
    let mut cells = vec![0i8; n * n];
    let mut seed = 12345u64;
    let mut next = |m: usize| {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((seed >> 33) as usize) % m
    };
    for _ in 0..300 {
        let (x, y) = (next(n), next(n));
        let (w, h) = (1 + next(40), 1 + next(40));
        for row in y..(y + h).min(n) {
            for col in x..(x + w).min(n) {
                cells[row * n + col] = 100;
            }
        }
    }
    for row in 0..100 {
        for col in 0..100 {
            cells[row * n + col] = -1;
        }
    }
    (n, cells)
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("costmap-chunks-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// The whole map's costmap, and the chunk folder of the same map.
fn both(name: &str) -> (Costmap, PathBuf) {
    let (n, cells) = map_cells();
    let dir = scratch(name);
    let grid = ChunkGrid::new(Point::new(CORNER, CORNER), 64.0).unwrap();
    let whole = WholeMap {
        origin: Point::new(CORNER, CORNER),
        resolution: RES,
        width: n,
        height: n,
        data: &cells,
    };
    write_chunks(&grid, &whole, &dir).unwrap();
    let mut c = Costmap::new(settings(), Vec::new());
    c.set_map(StaticMap::from_occupancy((CORNER, CORNER), RES, n, n, &cells, true).unwrap());
    c.update(&Pose2::IDENTITY, None, &[]);
    (c, dir)
}

fn chunked(dir: &Path, margin: i32) -> Costmap {
    let mut c = Costmap::new(settings(), Vec::new());
    c.set_chunks(ChunkSource::open(dir, 100.0, margin).unwrap())
        .unwrap();
    c
}

/// Cells of `part` that differ from `whole` at the same place, leaving out a band of
/// `edge` cells round `part` (where walls off the window would reach).
fn differences(whole: &Grid, part: &Grid, edge: usize) -> usize {
    let dx = ((part.origin_x - whole.origin_x) / RES).round() as i64;
    let dy = ((part.origin_y - whole.origin_y) / RES).round() as i64;
    let mut bad = 0;
    for y in edge..part.height.saturating_sub(edge) {
        for x in edge..part.width.saturating_sub(edge) {
            let (wx, wy) = (x as i64 + dx, y as i64 + dy);
            if wx < 0 || wy < 0 || wx >= whole.width as i64 || wy >= whole.height as i64 {
                continue;
            }
            if part.get(x, y) != whole.get(wx as usize, wy as usize) {
                bad += 1;
            }
        }
    }
    bad
}

#[test]
fn a_window_over_the_whole_map_is_the_whole_maps_costmap() {
    let (whole, dir) = both("whole");
    // The car in the middle chunk, two chunks round: all 5 x 5.
    let mut c = chunked(&dir, 1);
    let moved = c
        .follow_chunks((0.0, 0.0))
        .unwrap()
        .expect("a first window");
    assert_eq!(moved.changes.loaded.len(), 25);
    c.update(&Pose2::IDENTITY, None, &[]);
    assert_eq!(c.grid().width, whole.grid().width);
    assert_eq!(c.grid(), whole.grid());
}

#[test]
fn driving_keeps_the_cells_and_redoes_only_what_the_move_changed() {
    let (whole, dir) = both("driving");
    let reach = (3.0 / RES) as usize + 1;
    let mut c = chunked(&dir, 0);
    let route: Vec<(f64, f64)> = (0..=40)
        .map(|k| (-150.0 + 7.5 * f64::from(k), -20.0))
        .collect();
    c.set_ahead(&route);
    let (mut moves, mut redone, mut held_total) = (0, 0, 0);
    for k in 0..30 {
        let car = (-150.0 + 10.0 * f64::from(k), -20.0);
        let moved = c.follow_chunks(car).unwrap();
        c.update(&Pose2::new(car.0, car.1, 0.0), None, &[]);
        if let Some(m) = moved {
            moves += 1;
            let (window, held) = c.chunk_window().unwrap();
            assert_eq!(window, m.window);
            assert_eq!(held, window.across() * window.up());
            if moves > 1 {
                redone += m.redo;
                held_total += held;
            }
        }
        // Against the whole map, away from the window's edge...
        assert_eq!(differences(whole.grid(), c.grid(), reach), 0, "step {k}");
        // ...and cell for cell against a costmap that read this window afresh.
        let mut fresh = chunked(&dir, 0);
        fresh.set_ahead(&route);
        fresh.follow_chunks(car).unwrap();
        fresh.update(&Pose2::new(car.0, car.1, 0.0), None, &[]);
        assert_eq!(c.grid(), fresh.grid(), "step {k}");
    }
    assert!(moves >= 4, "the window moved {moves} times");
    assert!(
        redone < held_total,
        "{redone} of {held_total} chunks redone"
    );
}

#[test]
fn the_route_ahead_widens_the_window() {
    let (_, dir) = both("ahead");
    let mut c = chunked(&dir, 0);
    c.follow_chunks((-130.0, -130.0)).unwrap();
    let (alone, _) = c.chunk_window().unwrap();
    assert_eq!((alone.across(), alone.up()), (3, 3));
    c.set_ahead(&[(-130.0, -130.0), (100.0, -130.0)]);
    c.follow_chunks((-130.0, -130.0)).unwrap();
    let (ahead, _) = c.chunk_window().unwrap();
    assert!(ahead.across() > 3, "{ahead:?}");
}

#[test]
fn a_fixed_costmap_needs_the_fine_cells_a_rolling_one_any() {
    let (_, dir) = both("checks");
    let source = ChunkSource::open(&dir, 100.0, 0).unwrap();
    let mut coarse = Costmap::new(
        Settings {
            resolution: 1.0,
            ..settings()
        },
        Vec::new(),
    );
    assert!(coarse.set_chunks(source.clone()).is_err());
    // The local costmap (TODO 175): its own cells, stamped from the chunks round the car.
    let mut rolling = Costmap::new(
        Settings {
            rolling: true,
            width: 20.0,
            height: 20.0,
            resolution: 0.1,
            ..settings()
        },
        Vec::new(),
    );
    assert!(rolling.set_chunks(source).is_ok());
    assert!(ChunkSource::open(&dir.join("nowhere"), 100.0, 0).is_err());
}
