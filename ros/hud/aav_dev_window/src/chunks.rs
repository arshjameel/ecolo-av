//! The map's chunks (`--chunks`, a folder `chunk-map` wrote): which ones the car has
//! loaded, their outlines and labels, and each one's saved map as a grid to paint.
//! No Bevy and no egui: `panels.rs` only draws what this returns (TODO 175, step 7).

use std::{collections::BTreeMap, path::Path};

use map_chunks::{files, ChunkKey, Detail, Grid as ChunkGrid, OccupancyChunk};
use map_core::{
    geom::{Point, Pose},
    grid::{Bounds, Grid, GridInfo},
};

/// How far along the route the coarse chunks reach (m): the design's C3.
pub const AHEAD: f64 = 100.0;

/// One loaded chunk, ready to draw.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkBox {
    /// Which chunk.
    pub key: ChunkKey,
    /// 0.25 m (round the car) or 1 m (along the route).
    pub detail: Detail,
    /// Its square, closed: five corners on the map.
    pub outline: [Point; 5],
    /// "(i, j) fine" or "(i, j) coarse".
    pub label: String,
    /// The chunk the car is in.
    pub here: bool,
}

/// The chunks `map_chunks::wanted` loads for the car here on this route, in key order.
pub fn loaded(grid: &ChunkGrid, car: Point, route: &[Point]) -> Vec<ChunkBox> {
    let at = |p: Point| map_chunks::Point::new(p.x, p.y);
    let route: Vec<_> = route.iter().map(|p| at(*p)).collect();
    let here = grid.key_at(at(car));
    map_chunks::wanted(grid, at(car), &route, AHEAD)
        .into_iter()
        .map(|(key, detail)| {
            let c = grid.corner(key);
            let s = grid.size();
            let p = |dx: f64, dy: f64| Point::new(c.x + dx, c.y + dy);
            ChunkBox {
                key,
                detail,
                outline: [p(0.0, 0.0), p(s, 0.0), p(s, s), p(0.0, s), p(0.0, 0.0)],
                label: format!("({}, {}) {}", key.i, key.j, detail_word(detail)),
                here: key == here,
            }
        })
        .collect()
}

/// The word for a detail, as the labels say it.
pub fn detail_word(detail: Detail) -> &'static str {
    match detail {
        Detail::Fine => "fine",
        Detail::Coarse => "coarse",
    }
}

/// The smallest box round every chunk; `None` for none.
pub fn bounds(boxes: &[ChunkBox]) -> Option<Bounds> {
    Bounds::around(boxes.iter().flat_map(|b| b.outline))
}

/// One chunk's saved map as a grid on the map frame, for the map panel's painter.
pub fn to_grid(grid: &ChunkGrid, chunk: &OccupancyChunk) -> Option<Grid> {
    let corner = grid.corner(chunk.key);
    let side = u32::try_from(chunk.side()).ok()?;
    let info = GridInfo {
        width: side,
        height: side,
        resolution: chunk.detail.cell(),
        origin: Pose::new(corner.x, corner.y, 0.0),
    };
    Grid::new(info, "map".to_owned(), chunk.cells().to_vec()).ok()
}

/// The saved maps of the chunks read so far, each read once: `None` where a chunk has no
/// file (nothing known there) or a bad one.
#[derive(Default)]
pub struct SavedMaps {
    read: BTreeMap<(ChunkKey, Detail), Option<Grid>>,
    /// The files that could not be read, for the console.
    pub errors: Vec<String>,
}

impl SavedMaps {
    /// The saved map of each loaded chunk, reading those not read before from `folder`.
    pub fn of<'a>(
        &'a mut self,
        folder: &Path,
        grid: &ChunkGrid,
        boxes: &[ChunkBox],
    ) -> Vec<(ChunkKey, Detail, &'a Grid)> {
        for b in boxes {
            self.read.entry((b.key, b.detail)).or_insert_with(|| {
                match files::read_chunk(folder, grid, b.key, b.detail) {
                    Ok(chunk) => chunk.and_then(|c| to_grid(grid, &c)),
                    Err(e) => {
                        self.errors.push(e.to_string());
                        None
                    }
                }
            });
        }
        boxes
            .iter()
            .filter_map(|b| {
                let grid = self.read.get(&(b.key, b.detail))?.as_ref()?;
                Some((b.key, b.detail, grid))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use map_core::grid::Cell;

    fn grid() -> ChunkGrid {
        ChunkGrid::new(map_chunks::Point::new(0.0, 0.0), 64.0).expect("a grid")
    }

    #[test]
    fn standing_still_loads_the_three_by_three_fine() {
        let boxes = loaded(&grid(), Point::new(100.0, 100.0), &[]);
        assert_eq!(boxes.len(), 9);
        assert!(boxes.iter().all(|b| b.detail == Detail::Fine));
        let here: Vec<_> = boxes.iter().filter(|b| b.here).collect();
        assert_eq!(here.len(), 1);
        assert_eq!(here[0].key, ChunkKey::new(1, 1));
        assert_eq!(here[0].label, "(1, 1) fine");
    }

    #[test]
    fn a_route_adds_coarse_chunks_only_100_m_ahead() {
        let route = [Point::new(100.0, 100.0), Point::new(600.0, 100.0)];
        let boxes = loaded(&grid(), Point::new(100.0, 100.0), &route);
        let coarse: Vec<_> = boxes
            .iter()
            .filter(|b| b.detail == Detail::Coarse)
            .map(|b| b.key)
            .collect();
        // 100 m from x = 100 ends at x = 200: chunk 3 (192-256) is the only new one.
        assert_eq!(coarse, vec![ChunkKey::new(3, 1)]);
        assert!(coarse.iter().all(|k| k.j == 1));
        assert!(boxes.iter().any(|b| b.label == "(3, 1) coarse"));
    }

    #[test]
    fn an_outline_is_its_square_closed() {
        let boxes = loaded(&grid(), Point::new(10.0, 10.0), &[]);
        let b = boxes.iter().find(|b| b.here).expect("the car's chunk");
        assert_eq!(b.outline[0], b.outline[4]);
        assert_eq!(b.outline[2], Point::new(64.0, 64.0));
        let b = bounds(&boxes).expect("bounds");
        assert_eq!(
            (b.min, b.max),
            (Point::new(-64.0, -64.0), Point::new(128.0, 128.0))
        );
        assert_eq!(bounds(&[]), None);
    }

    #[test]
    fn a_chunk_becomes_a_grid_at_its_corner() {
        let g = grid();
        let mut chunk = OccupancyChunk::unknown(&g, ChunkKey::new(2, -1), Detail::Coarse);
        assert!(chunk.mark(&g, map_chunks::Point::new(130.5, -63.5), 100));
        let painted = to_grid(&g, &chunk).expect("a grid");
        assert_eq!(painted.info.width, 64);
        assert_eq!(painted.info.resolution, 1.0);
        assert_eq!(painted.info.origin.position, Point::new(128.0, -64.0));
        assert_eq!(painted.cell(2, 0), Some(Cell::Occupancy(100)));
        assert_eq!(painted.cell(22, 43), Some(Cell::Unknown));
    }

    #[test]
    fn saved_maps_read_each_chunk_once_and_skip_missing_files() {
        let dir = std::env::temp_dir().join(format!("dev-window-chunks-{}", std::process::id()));
        let g = grid();
        let mut chunk = OccupancyChunk::unknown(&g, ChunkKey::new(0, 0), Detail::Fine);
        chunk.mark(&g, map_chunks::Point::new(1.0, 1.0), 100);
        std::fs::create_dir_all(files::detail_dir(&dir, Detail::Fine)).expect("a folder");
        files::write_chunk(&dir, &chunk).expect("a chunk file");
        let boxes = loaded(&g, Point::new(10.0, 10.0), &[]);
        let mut saved = SavedMaps::default();
        let maps = saved.of(&dir, &g, &boxes);
        assert_eq!(maps.len(), 1, "only (0, 0) has a file");
        assert_eq!(maps[0].2.cell(4, 4), Some(Cell::Occupancy(100)));
        assert_eq!(saved.read.len(), 9);
        assert!(saved.errors.is_empty());
        let _ = std::fs::remove_dir_all(dir);
    }
}
