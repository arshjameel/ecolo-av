//! The global costmap from map chunks (TODO 175, step 3 of the three levels): only the
//! chunks round the car and along its route are in memory, not the whole map.
//!
//! ```text
//!  car + route ─▶ map_chunks::wanted ─▶ the rectangle round them (+ `margin` chunks)
//!                                        │ Store::update: each new chunk read on its own core
//!                                        ▼
//!   ┌────┬────┬────┬────┐   the window: one grid, its corner on a chunk's corner
//!   │kept│kept│new │new │   kept chunks keep their cells; a chunk is worked out again
//!   ├────┼────┼────┼────┤   (static map, layers, inflation) only if it is new or a
//!   │kept│CAR │new │new │   neighbour came or went: each on its own core (rayon)
//!   └────┴────┴────┴────┘
//! ```
//!
//! [`Window`] is the maths (which chunks, which to redo); [`ChunkSource`] reads the files.
//! The costmap itself ([`crate::Costmap::set_chunks`], [`crate::Costmap::follow_chunks`])
//! puts them together.

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

use map_chunks::files::{read_chunk, read_grid, FileError};
use map_chunks::{wanted, ChunkKey, Detail, Grid as ChunkGrid, Point};

use crate::cost::{FREE, LETHAL, NO_INFORMATION};
use crate::grid::{Bounds, Grid};
use crate::static_layer::LETHAL_THRESHOLD;

/// A rectangle of chunks, `i0..=i1` by `j0..=j1`: what the global costmap holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Window {
    /// The first chunk along x.
    pub i0: i32,
    /// The first chunk along y.
    pub j0: i32,
    /// The last chunk along x.
    pub i1: i32,
    /// The last chunk along y.
    pub j1: i32,
}

impl Window {
    /// The smallest rectangle holding every key, grown by `margin` chunks each side;
    /// `None` with no keys.
    #[must_use]
    pub fn around<'a>(keys: impl IntoIterator<Item = &'a ChunkKey>, margin: i32) -> Option<Self> {
        keys.into_iter()
            .map(|k| Self {
                i0: k.i,
                j0: k.j,
                i1: k.i,
                j1: k.j,
            })
            .reduce(|a, b| Self {
                i0: a.i0.min(b.i0),
                j0: a.j0.min(b.j0),
                i1: a.i1.max(b.i1),
                j1: a.j1.max(b.j1),
            })
            .map(|w| Self {
                i0: w.i0 - margin,
                j0: w.j0 - margin,
                i1: w.i1 + margin,
                j1: w.j1 + margin,
            })
    }

    /// Does it hold chunk `k`?
    #[must_use]
    pub fn contains(&self, k: ChunkKey) -> bool {
        (self.i0..=self.i1).contains(&k.i) && (self.j0..=self.j1).contains(&k.j)
    }

    /// Chunks along x.
    #[must_use]
    pub fn across(&self) -> usize {
        (self.i1 - self.i0 + 1).max(0) as usize
    }

    /// Chunks along y.
    #[must_use]
    pub fn up(&self) -> usize {
        (self.j1 - self.j0 + 1).max(0) as usize
    }

    /// Every chunk in it, row by row.
    pub fn keys(&self) -> impl Iterator<Item = ChunkKey> + '_ {
        (self.j0..=self.j1).flat_map(move |j| (self.i0..=self.i1).map(move |i| ChunkKey::new(i, j)))
    }

    /// Chunk `k`'s cells in a grid on this window, `side` cells a chunk.
    #[must_use]
    pub fn cells_of(&self, k: ChunkKey, side: usize) -> Bounds {
        let x0 = (k.i - self.i0) as usize * side;
        let y0 = (k.j - self.j0) as usize * side;
        Bounds {
            x0,
            y0,
            x1: x0 + side,
            y1: y0 + side,
        }
    }

    /// The chunks of `self` (the new window) whose cells must be worked out again after
    /// `old`: a chunk's costs depend only on its own cells and its eight neighbours' (the
    /// inflation reaches less than a chunk), so a chunk held before whose neighbours were
    /// all held before or are all still away keeps its cells. `None`: every chunk.
    #[must_use]
    pub fn to_redo(&self, old: Option<&Window>) -> Vec<ChunkKey> {
        let Some(old) = old else {
            return self.keys().collect();
        };
        self.keys()
            .filter(|&k| {
                !old.contains(k) || k.around(1).any(|n| old.contains(n) != self.contains(n))
            })
            .collect()
    }
}

/// Why the chunks can't be used.
#[derive(Debug)]
pub enum ChunkError {
    /// A chunk file, or `chunks.yaml`, couldn't be read.
    File(FileError),
    /// The costmap's cells aren't the chunk files' fine cells.
    Resolution {
        /// The costmap's (m).
        costmap: f64,
        /// The fine chunks' (m).
        chunks: f64,
    },
    /// A rolling costmap moves on its own: chunks are for the fixed (global) one.
    Rolling,
}

impl fmt::Display for ChunkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ChunkError::File(e) => write!(f, "{e}"),
            ChunkError::Resolution { costmap, chunks } => write!(
                f,
                "the costmap's cells are {costmap} m, the fine chunks' {chunks} m: they must match"
            ),
            ChunkError::Rolling => write!(f, "a rolling costmap can't be made of chunks"),
        }
    }
}

impl std::error::Error for ChunkError {}

impl From<FileError> for ChunkError {
    fn from(e: FileError) -> Self {
        ChunkError::File(e)
    }
}

/// Where the chunks are, and how much of them to hold.
#[derive(Debug, Clone)]
pub struct ChunkSource {
    /// The folder `chunk-map` wrote: `chunks.yaml`, `fine/i_j.pgm`.
    pub dir: PathBuf,
    /// The chunk grid, from `chunks.yaml`.
    pub grid: ChunkGrid,
    /// m of the route ahead whose chunks are held (C3: 100 m).
    pub ahead: f64,
    /// Chunks added round the rectangle of those wanted: room for the planner to go
    /// round a block that leaves the route's own chunks.
    pub margin: i32,
}

impl ChunkSource {
    /// The chunks in `dir` (its `chunks.yaml` read).
    ///
    /// # Errors
    ///
    /// [`ChunkError::File`]: no `chunks.yaml`, or a bad one.
    pub fn open(dir: &Path, ahead: f64, margin: i32) -> Result<Self, ChunkError> {
        Ok(Self {
            dir: dir.to_path_buf(),
            grid: read_grid(dir)?,
            ahead,
            margin: margin.max(0),
        })
    }

    /// Cells along a chunk's side, at the fine detail.
    #[must_use]
    pub fn side(&self) -> usize {
        self.grid.cells_per_side(Detail::Fine)
    }

    /// The window for the car at `car` on `route` (map frame).
    #[must_use]
    pub fn window(&self, car: (f64, f64), route: &[(f64, f64)]) -> Window {
        let route: Vec<Point> = route.iter().map(|&(x, y)| Point::new(x, y)).collect();
        let need: BTreeMap<ChunkKey, Detail> =
            wanted(&self.grid, Point::new(car.0, car.1), &route, self.ahead);
        // wanted always holds the 3 x 3 round the car, so never None.
        Window::around(need.keys(), self.margin).unwrap_or(Window {
            i0: 0,
            j0: 0,
            i1: 0,
            j1: 0,
        })
    }

    /// One chunk's static costs, as the static layer reads a map (lethal at 100, else
    /// free; unknown kept with `track_unknown`). A chunk with no file is all unknown.
    ///
    /// # Errors
    ///
    /// [`FileError`]: the file is there but can't be read.
    pub fn load(&self, key: ChunkKey, track_unknown: bool) -> Result<Vec<u8>, FileError> {
        let side = self.side();
        let unknown = if track_unknown { NO_INFORMATION } else { FREE };
        Ok(
            match read_chunk(&self.dir, &self.grid, key, Detail::Fine)? {
                None => vec![unknown; side * side],
                Some(chunk) => chunk
                    .cells()
                    .iter()
                    .map(|&v| match v {
                        -1 => unknown,
                        v if v >= LETHAL_THRESHOLD => LETHAL,
                        _ => FREE,
                    })
                    .collect(),
            },
        )
    }

    /// The grid a window covers (every cell `fill`), its corner on the first chunk's.
    #[must_use]
    pub fn grid_for(&self, w: &Window, fill: u8) -> Grid {
        let corner = self.grid.corner(ChunkKey::new(w.i0, w.j0));
        let side = self.side();
        Grid::new(
            (corner.x, corner.y),
            Detail::Fine.cell(),
            w.across() * side,
            w.up() * side,
            fill,
        )
    }
}

/// Put chunk `key`'s cells (`side` x `side`, row by row) into `into`, a grid on `w`.
pub fn stitch(into: &mut Grid, w: &Window, key: ChunkKey, side: usize, cells: &[u8]) {
    let b = w.cells_of(key, side);
    for (row, src) in cells.chunks_exact(side).enumerate() {
        let to = (b.y0 + row) * into.width + b.x0;
        into.data[to..to + side].copy_from_slice(src);
    }
}

/// Copy the cells `from` and `into` share (both on the same lattice of cells): what a
/// window keeps when it moves.
pub fn copy_shared(into: &mut Grid, from: &Grid, area: Bounds) {
    let res = into.resolution;
    let dx = ((from.origin_x - into.origin_x) / res).round() as i64;
    let dy = ((from.origin_y - into.origin_y) / res).round() as i64;
    for y in area.y0..area.y1 {
        let fy = y as i64 - dy;
        if fy < 0 || fy >= from.height as i64 {
            continue;
        }
        let (fx0, fx1) = (area.x0 as i64 - dx, area.x1 as i64 - dx);
        let (a, b) = (fx0.max(0), fx1.min(from.width as i64));
        if a >= b {
            continue;
        }
        let to = y * into.width + (a + dx) as usize;
        let src = fy as usize * from.width + a as usize;
        let n = (b - a) as usize;
        into.data[to..to + n].copy_from_slice(&from.data[src..src + n]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn w(i0: i32, j0: i32, i1: i32, j1: i32) -> Window {
        Window { i0, j0, i1, j1 }
    }

    #[test]
    fn the_window_is_the_rectangle_round_the_wanted_chunks() {
        let keys = [ChunkKey::new(3, 4), ChunkKey::new(5, 2)];
        assert_eq!(Window::around(&keys, 0), Some(w(3, 2, 5, 4)));
        assert_eq!(Window::around(&keys, 1), Some(w(2, 1, 6, 5)));
        assert_eq!(Window::around(&[], 1), None);
        let win = w(2, 1, 6, 5);
        assert_eq!((win.across(), win.up()), (5, 5));
        assert_eq!(win.keys().count(), 25);
        assert_eq!(
            win.cells_of(ChunkKey::new(3, 2), 256),
            Bounds {
                x0: 256,
                y0: 256,
                x1: 512,
                y1: 512
            }
        );
    }

    #[test]
    fn only_new_chunks_and_their_neighbours_are_redone() {
        let old = w(0, 0, 2, 2);
        // Nothing moved: nothing to redo.
        assert!(old.to_redo(Some(&old)).is_empty());
        // No old window: all of it.
        assert_eq!(old.to_redo(None).len(), 9);
        // One chunk east: the new column, and the column beside it (its east neighbours
        // came), and the old west column's neighbours went: the middle column too.
        let new = w(1, 0, 3, 2);
        let redo = new.to_redo(Some(&old));
        assert!(redo.contains(&ChunkKey::new(3, 1)));
        assert!(redo.contains(&ChunkKey::new(2, 1)));
        assert!(redo.contains(&ChunkKey::new(1, 1)));
        // Grown east only: the old west column keeps its cells.
        let grown = w(0, 0, 3, 2);
        let redo = grown.to_redo(Some(&old));
        assert!(!redo.contains(&ChunkKey::new(0, 1)));
        assert!(!redo.contains(&ChunkKey::new(1, 1)));
        assert!(redo.contains(&ChunkKey::new(2, 1)));
        assert_eq!(redo.len(), 6);
    }

    #[test]
    fn shared_cells_follow_the_ground() {
        let mut from = Grid::new((0.0, 0.0), 1.0, 4, 4, 0);
        from.set(3, 3, 9);
        from.set(0, 0, 7);
        let mut into = Grid::new((2.0, 2.0), 1.0, 4, 4, 1);
        let all = Bounds {
            x0: 0,
            y0: 0,
            x1: 4,
            y1: 4,
        };
        copy_shared(&mut into, &from, all);
        assert_eq!(into.get(1, 1), 9);
        assert_eq!(into.get(0, 0), 0);
        assert_eq!(into.get(2, 2), 1, "off the old grid: left as it was");
        assert!(!into.data.contains(&7));
    }
}
