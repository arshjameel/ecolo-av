//! The map cut into fixed squares (chunks), the base of the three map levels
//! (`docs/designs/2026-11-15_three-levels.md`, TODO 175). No ROS here.
//!
//! ```text
//!  the map's origin
//!       ▼
//!       ┌──────┬──────┬──────┐
//!       │(0,2) │(1,2) │(2,2) │   a chunk is (i, j): 64 m squares from the origin,
//!       ├──────┼──────┼──────┤   i along x, j along y (REP-103)
//!       │(0,1) │(1,1) │(2,1) │
//!       ├──────┼──────┼──────┤
//!       │(0,0) │(1,0) │(2,0) │
//!       └──────┴──────┴──────┘
//! ```
//!
//! - [`Grid`]: where a point's chunk is (one division, no tree).
//! - [`wanted`]: the chunks the car needs: the 3 x 3 round it at [`Detail::Fine`], and those
//!   the route crosses in the next `ahead` metres at [`Detail::Coarse`].
//! - [`Store`]: the loaded chunks; [`Store::update`] loads the new ones and drops the rest.
//! - [`OccupancyChunk`] and [`cut`]: one chunk's occupancy, and a whole map cut into them.
//! - [`files`]: a map server's map in, one `.pgm` per chunk out, and back.
//!
//! Units: metres, in the map frame.
//!
//! ```
//! use map_chunks::{wanted, Detail, Grid, Point};
//!
//! let grid = Grid::new(Point::new(-900.0, -900.0), 64.0).unwrap();
//! let car = Point::new(10.0, 10.0);
//! let route = [car, Point::new(200.0, 10.0)];
//! let need = wanted(&grid, car, &route, 100.0);
//! assert_eq!(need[&grid.key_at(car)], Detail::Fine);
//! assert!(need.len() <= 15);
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use rayon::prelude::*;

pub mod files;

/// A point on the map (m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    fn distance(self, other: Point) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}

/// One chunk: `i` squares along x from the map's origin, `j` along y.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ChunkKey {
    pub i: i32,
    pub j: i32,
}

impl ChunkKey {
    pub const fn new(i: i32, j: i32) -> Self {
        Self { i, j }
    }

    /// The chunks at most `reach` squares away, this one included, in order.
    pub fn around(self, reach: i32) -> impl Iterator<Item = ChunkKey> {
        (-reach..=reach).flat_map(move |di| {
            (-reach..=reach).map(move |dj| ChunkKey::new(self.i + di, self.j + dj))
        })
    }
}

impl fmt::Display for ChunkKey {
    /// `i_j`, as chunk files are named (C4).
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}_{}", self.i, self.j)
    }
}

/// How fine a chunk's cells are (C2: 1 m far, 0.25 m near).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Detail {
    /// 1 m cells: chunks along the route, away from the car.
    Coarse,
    /// 0.25 m cells: the chunks round the car.
    Fine,
}

impl Detail {
    /// One cell's edge (m).
    pub const fn cell(self) -> f64 {
        match self {
            Detail::Coarse => 1.0,
            Detail::Fine => 0.25,
        }
    }
}

/// Why a grid can't be made.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GridError {
    /// The chunk size isn't a positive whole number of metres, so 1 m cells wouldn't fit.
    BadSize(f64),
    /// The origin isn't a finite point.
    BadOrigin,
}

impl fmt::Display for GridError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GridError::BadSize(size) => {
                write!(
                    f,
                    "a chunk must be a positive whole number of metres, not {size}"
                )
            }
            GridError::BadOrigin => write!(f, "the map's origin must be a finite point"),
        }
    }
}

impl std::error::Error for GridError {}

/// The fixed squares: where they start and how big each is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grid {
    origin: Point,
    size: f64,
}

impl Grid {
    /// Squares of `size` m from `origin` (the map's own origin, so chunks line up with it).
    pub fn new(origin: Point, size: f64) -> Result<Self, GridError> {
        if !(origin.x.is_finite() && origin.y.is_finite()) {
            return Err(GridError::BadOrigin);
        }
        if !(size.is_finite() && size >= 1.0 && size.fract() == 0.0) {
            return Err(GridError::BadSize(size));
        }
        Ok(Self { origin, size })
    }

    pub fn origin(&self) -> Point {
        self.origin
    }

    /// One chunk's edge (m).
    pub fn size(&self) -> f64 {
        self.size
    }

    /// The chunk holding `p`. A point on an edge belongs to the chunk above and right of it.
    pub fn key_at(&self, p: Point) -> ChunkKey {
        ChunkKey::new(
            ((p.x - self.origin.x) / self.size).floor() as i32,
            ((p.y - self.origin.y) / self.size).floor() as i32,
        )
    }

    /// A chunk's lower-left corner.
    pub fn corner(&self, key: ChunkKey) -> Point {
        Point::new(
            self.origin.x + f64::from(key.i) * self.size,
            self.origin.y + f64::from(key.j) * self.size,
        )
    }

    /// How many cells of `detail` along one side of a chunk.
    pub fn cells_per_side(&self, detail: Detail) -> usize {
        (self.size / detail.cell()).round() as usize
    }
}

/// The chunks the car needs now, and at what detail.
///
/// The 3 x 3 round the car at [`Detail::Fine`] (a 64 m chunk: at least 64 m all round, past
/// the lidar's 30 m); then, from the route's point nearest the car, every chunk the route
/// crosses in the next `ahead` m at [`Detail::Coarse`] (C3: 100 m).
pub fn wanted(grid: &Grid, car: Point, route: &[Point], ahead: f64) -> BTreeMap<ChunkKey, Detail> {
    let mut need: BTreeMap<ChunkKey, Detail> = grid
        .key_at(car)
        .around(1)
        .map(|key| (key, Detail::Fine))
        .collect();
    for key in route_chunks(grid, route, nearest_index(route, car), ahead) {
        need.entry(key).or_insert(Detail::Coarse);
    }
    need
}

/// The index of the route's point nearest `p` (0 for an empty route).
fn nearest_index(route: &[Point], p: Point) -> usize {
    route
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.distance(p).total_cmp(&b.distance(p)))
        .map_or(0, |(index, _)| index)
}

/// The chunks the route crosses from `route[start]` for `ahead` m, sampled every quarter
/// chunk (a chunk can't be crossed between two samples except at a corner, which the
/// 3 x 3 or the next sample covers).
fn route_chunks(grid: &Grid, route: &[Point], start: usize, ahead: f64) -> BTreeSet<ChunkKey> {
    let step = grid.size() / 4.0;
    let mut keys = BTreeSet::new();
    let mut left = ahead;
    if let Some(&first) = route.get(start) {
        keys.insert(grid.key_at(first));
    }
    for pair in route.get(start..).unwrap_or_default().windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let length = a.distance(b);
        let run = length.min(left);
        let samples = (run / step).ceil() as usize;
        keys.extend((1..=samples).map(|n| {
            let t = (n as f64 * step).min(run) / length.max(f64::EPSILON);
            grid.key_at(Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)))
        }));
        left -= run;
        if left <= 0.0 {
            break;
        }
    }
    keys
}

/// What one [`Store::update`] did.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Changes {
    /// Newly loaded, or loaded again at another detail.
    pub loaded: Vec<ChunkKey>,
    /// Dropped: no longer wanted.
    pub dropped: Vec<ChunkKey>,
}

/// The loaded chunks, each with its detail and its layers `T`.
#[derive(Debug, Clone)]
pub struct Store<T> {
    chunks: BTreeMap<ChunkKey, (Detail, T)>,
}

impl<T> Default for Store<T> {
    fn default() -> Self {
        Self {
            chunks: BTreeMap::new(),
        }
    }
}

impl<T> Store<T> {
    pub fn new() -> Self {
        Self::default()
    }

    /// Load what `want` holds and isn't loaded at that detail; drop what it doesn't hold.
    ///
    /// All the loads run first, each chunk on its own core (rayon; Ben, 15 Nov: "processing
    /// each block could be done in threads"): a new chunk reached loads 3 to 15 at once.
    /// If one fails, its error comes back and nothing changes.
    pub fn update<E: Send>(
        &mut self,
        want: &BTreeMap<ChunkKey, Detail>,
        load: impl Fn(ChunkKey, Detail) -> Result<T, E> + Sync,
    ) -> Result<Changes, E>
    where
        T: Send,
    {
        let needed: Vec<(ChunkKey, Detail)> = want
            .iter()
            .filter(|(key, detail)| self.chunks.get(key).map(|(have, _)| have) != Some(detail))
            .map(|(&key, &detail)| (key, detail))
            .collect();
        let fresh = needed
            .into_par_iter()
            .map(|(key, detail)| load(key, detail).map(|layers| (key, detail, layers)))
            .collect::<Result<Vec<_>, E>>()?;
        let dropped: Vec<ChunkKey> = self
            .chunks
            .keys()
            .filter(|key| !want.contains_key(key))
            .copied()
            .collect();
        for key in &dropped {
            self.chunks.remove(key);
        }
        let loaded = fresh.iter().map(|(key, _, _)| *key).collect();
        for (key, detail, layers) in fresh {
            self.chunks.insert(key, (detail, layers));
        }
        Ok(Changes { loaded, dropped })
    }

    /// A loaded chunk's detail and layers.
    pub fn get(&self, key: ChunkKey) -> Option<(Detail, &T)> {
        self.chunks
            .get(&key)
            .map(|(detail, layers)| (*detail, layers))
    }

    /// The loaded chunks, in order.
    pub fn loaded(&self) -> impl Iterator<Item = (ChunkKey, Detail)> + '_ {
        self.chunks.iter().map(|(key, (detail, _))| (*key, *detail))
    }

    pub fn len(&self) -> usize {
        self.chunks.len()
    }

    pub fn is_empty(&self) -> bool {
        self.chunks.is_empty()
    }
}

/// A cell no sensor or map has seen: ROS's `OccupancyGrid` -1.
pub const UNKNOWN: i8 = -1;

/// One chunk's occupancy, as ROS's `OccupancyGrid` counts it: -1 unknown, 0 free to 100
/// taken. Row by row from the chunk's lower-left corner, x fastest.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OccupancyChunk {
    pub key: ChunkKey,
    pub detail: Detail,
    side: usize,
    cells: Vec<i8>,
}

impl OccupancyChunk {
    /// A chunk with every cell unknown.
    pub fn unknown(grid: &Grid, key: ChunkKey, detail: Detail) -> Self {
        let side = grid.cells_per_side(detail);
        Self {
            key,
            detail,
            side,
            cells: vec![UNKNOWN; side * side],
        }
    }

    /// A chunk from its cells, row by row from the lower left; `None` if there aren't
    /// exactly side x side of them.
    pub fn from_cells(grid: &Grid, key: ChunkKey, detail: Detail, cells: Vec<i8>) -> Option<Self> {
        let side = grid.cells_per_side(detail);
        (cells.len() == side * side).then_some(Self {
            key,
            detail,
            side,
            cells,
        })
    }

    /// Cells along one side.
    pub fn side(&self) -> usize {
        self.side
    }

    pub fn cells(&self) -> &[i8] {
        &self.cells
    }

    /// The index of the cell under `p`, if `p` is in this chunk.
    fn index(&self, grid: &Grid, p: Point) -> Option<usize> {
        let corner = grid.corner(self.key);
        let cell = self.detail.cell();
        let (cx, cy) = ((p.x - corner.x) / cell, (p.y - corner.y) / cell);
        let inside = |c: f64| (0.0..self.side as f64).contains(&c);
        (inside(cx) && inside(cy)).then(|| cy as usize * self.side + cx as usize)
    }

    /// The cell under `p`, if `p` is in this chunk.
    pub fn at(&self, grid: &Grid, p: Point) -> Option<i8> {
        self.index(grid, p).map(|index| self.cells[index])
    }

    /// Mark the cell under `p` with `value`, keeping the worse of it and what's there
    /// (unknown counts as nothing yet). False if `p` isn't in this chunk.
    pub fn mark(&mut self, grid: &Grid, p: Point, value: i8) -> bool {
        match self.index(grid, p) {
            Some(index) => {
                self.cells[index] = worse(self.cells[index], value);
                true
            }
            None => false,
        }
    }

    /// True if no cell is known.
    pub fn is_unknown(&self) -> bool {
        self.cells.iter().all(|&cell| cell == UNKNOWN)
    }
}

/// The worse of two cells: the higher cost; unknown only if both are.
fn worse(a: i8, b: i8) -> i8 {
    match (a, b) {
        (UNKNOWN, other) | (other, UNKNOWN) => other,
        (a, b) => a.max(b),
    }
}

/// A whole occupancy map, as a ROS `OccupancyGrid` or a map server's `.pgm` holds it.
#[derive(Debug, Clone, Copy)]
pub struct WholeMap<'a> {
    /// The lower-left corner of cell (0, 0).
    pub origin: Point,
    /// One cell's edge (m).
    pub resolution: f64,
    pub width: usize,
    pub height: usize,
    /// Row by row from the lower left, x fastest: `width * height` cells.
    pub data: &'a [i8],
}

/// Cut a whole map into chunks at `detail`, each chunk cell the worst of the map's cells
/// whose centres fall in it. Chunks with nothing known are left out.
///
/// The map's cells should be no bigger than `detail`'s, or some chunk cells stay unknown.
pub fn cut(grid: &Grid, map: &WholeMap<'_>, detail: Detail) -> BTreeMap<ChunkKey, OccupancyChunk> {
    let mut chunks: BTreeMap<ChunkKey, OccupancyChunk> = BTreeMap::new();
    for (index, &value) in map.data.iter().enumerate().take(map.width * map.height) {
        if value == UNKNOWN {
            continue;
        }
        let (col, row) = (index % map.width, index / map.width);
        let centre = Point::new(
            map.origin.x + (col as f64 + 0.5) * map.resolution,
            map.origin.y + (row as f64 + 0.5) * map.resolution,
        );
        let key = grid.key_at(centre);
        chunks
            .entry(key)
            .or_insert_with(|| OccupancyChunk::unknown(grid, key, detail))
            .mark(grid, centre, value);
    }
    chunks
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    fn campus() -> Grid {
        Grid::new(Point::new(-900.0, -900.0), 64.0).unwrap()
    }

    #[test]
    fn a_grid_needs_whole_metres_and_a_finite_origin() {
        let zero = Point::new(0.0, 0.0);
        assert_eq!(Grid::new(zero, 0.5), Err(GridError::BadSize(0.5)));
        assert_eq!(Grid::new(zero, 64.5), Err(GridError::BadSize(64.5)));
        assert_eq!(
            Grid::new(Point::new(f64::NAN, 0.0), 64.0),
            Err(GridError::BadOrigin)
        );
        assert!(Grid::new(zero, 64.0).is_ok());
    }

    #[test]
    fn a_point_finds_its_chunk_both_sides_of_the_origin() {
        let grid = campus();
        assert_eq!(grid.key_at(Point::new(-900.0, -900.0)), ChunkKey::new(0, 0));
        assert_eq!(grid.key_at(Point::new(-836.0, -900.0)), ChunkKey::new(1, 0));
        assert_eq!(
            grid.key_at(Point::new(-900.1, -836.1)),
            ChunkKey::new(-1, 0)
        );
        assert_eq!(grid.corner(ChunkKey::new(1, 2)), Point::new(-836.0, -772.0));
        assert_eq!(grid.cells_per_side(Detail::Fine), 256);
        assert_eq!(grid.cells_per_side(Detail::Coarse), 64);
        assert_eq!(ChunkKey::new(-3, 14).to_string(), "-3_14");
    }

    #[test]
    fn parked_the_car_needs_the_three_by_three_fine() {
        let grid = campus();
        let car = Point::new(5.0, 5.0);
        let need = wanted(&grid, car, &[], 100.0);
        assert_eq!(need.len(), 9);
        assert!(need.values().all(|&detail| detail == Detail::Fine));
        assert!(need.contains_key(&grid.key_at(Point::new(5.0 + 64.0, 5.0 - 64.0))));
    }

    #[test]
    fn a_long_straight_route_loads_100_m_ahead_and_no_more() {
        // The car near its chunk's east edge (that chunk runs from x = -4 to 60), so 100 m
        // ahead (to x = 160) reaches one chunk past the 3 x 3 (which ends at x = 124).
        let grid = campus();
        let car = Point::new(58.0, 4.0);
        let route: Vec<Point> = (0..=100)
            .map(|n| Point::new(4.0 + 8.0 * n as f64, 4.0))
            .collect();
        let need = wanted(&grid, car, &route, 100.0);
        let past = grid.key_at(Point::new(150.0, 4.0));
        assert_eq!(need[&grid.key_at(car)], Detail::Fine);
        assert_eq!(need[&past], Detail::Coarse);
        assert!(!need.contains_key(&grid.key_at(Point::new(200.0, 4.0))));
        assert_eq!(need.len(), 10, "{need:?}");
    }

    #[test]
    fn halfway_along_the_route_it_looks_ahead_from_the_car_not_the_start() {
        let grid = campus();
        let route: Vec<Point> = (0..=100).map(|n| Point::new(8.0 * n as f64, 4.0)).collect();
        let car = Point::new(400.0, 4.0);
        let need = wanted(&grid, car, &route, 100.0);
        assert!(!need.contains_key(&grid.key_at(Point::new(0.0, 4.0))));
        assert!(need.contains_key(&grid.key_at(Point::new(499.0, 4.0))));
    }

    #[test]
    fn a_campus_route_with_turns_needs_about_fifteen_chunks() {
        // East 60 m, north 60 m, east again: an L and a jog, as on the campus loop.
        let grid = campus();
        let route = [
            Point::new(0.0, 0.0),
            Point::new(60.0, 0.0),
            Point::new(60.0, 60.0),
            Point::new(300.0, 60.0),
        ];
        for car in route.iter().take(3) {
            let need = wanted(&grid, *car, &route, 100.0);
            assert!((9..=15).contains(&need.len()), "{car:?}: {}", need.len());
        }
    }

    #[test]
    fn the_store_loads_what_is_new_and_drops_what_is_left_behind() {
        let grid = campus();
        let mut store: Store<u32> = Store::new();
        let loads = AtomicU32::new(0);
        let count = |_: ChunkKey, _: Detail| -> Result<u32, ()> {
            Ok(loads.fetch_add(1, Ordering::Relaxed) + 1)
        };
        let first = store
            .update(&wanted(&grid, Point::new(5.0, 5.0), &[], 0.0), count)
            .unwrap();
        assert_eq!((first.loaded.len(), first.dropped.len()), (9, 0));
        // One chunk east: three columns new, three dropped, six kept as they were.
        let second = store
            .update(&wanted(&grid, Point::new(69.0, 5.0), &[], 0.0), count)
            .unwrap();
        assert_eq!((second.loaded.len(), second.dropped.len()), (3, 3));
        assert_eq!(store.len(), 9);
        assert_eq!(loads.into_inner(), 12);
    }

    #[test]
    fn a_chunk_loads_again_when_its_detail_changes() {
        let grid = campus();
        let key = grid.key_at(Point::new(5.0, 5.0));
        let mut store: Store<Detail> = Store::new();
        let coarse = BTreeMap::from([(key, Detail::Coarse)]);
        let fine = BTreeMap::from([(key, Detail::Fine)]);
        let echo = |_: ChunkKey, detail: Detail| -> Result<Detail, ()> { Ok(detail) };
        store.update(&coarse, echo).unwrap();
        let changes = store.update(&fine, echo).unwrap();
        assert_eq!(changes.loaded, vec![key]);
        assert_eq!(store.get(key), Some((Detail::Fine, &Detail::Fine)));
    }

    #[test]
    fn a_failed_load_changes_nothing() {
        let grid = campus();
        let mut store: Store<()> = Store::new();
        let ok = |_: ChunkKey, _: Detail| -> Result<(), &str> { Ok(()) };
        store
            .update(&wanted(&grid, Point::new(5.0, 5.0), &[], 0.0), ok)
            .unwrap();
        let before: Vec<_> = store.loaded().collect();
        let fail = |_: ChunkKey, _: Detail| -> Result<(), &str> { Err("no file") };
        let err = store.update(&wanted(&grid, Point::new(500.0, 5.0), &[], 0.0), fail);
        assert_eq!(err, Err("no file"));
        assert_eq!(store.loaded().collect::<Vec<_>>(), before);
    }

    #[test]
    fn a_chunk_keeps_the_worse_cell_and_knows_its_edges() {
        let grid = campus();
        let key = ChunkKey::new(0, 0);
        let mut chunk = OccupancyChunk::unknown(&grid, key, Detail::Coarse);
        let p = Point::new(-899.5, -899.5);
        assert!(chunk.is_unknown());
        assert!(chunk.mark(&grid, p, 0));
        assert!(chunk.mark(&grid, p, 100));
        assert!(chunk.mark(&grid, p, 30));
        assert_eq!(chunk.at(&grid, p), Some(100));
        assert_eq!(chunk.at(&grid, Point::new(-836.0, -899.5)), None);
        assert!(!chunk.mark(&grid, Point::new(-901.0, -899.5), 100));
        assert_eq!(chunk.at(&grid, Point::new(-836.01, -836.01)), Some(UNKNOWN));
    }

    #[test]
    fn cutting_a_map_keeps_every_wall_and_leaves_out_empty_chunks() {
        // A 200 x 200 m map at 0.25 m from (-100, -100): unknown, a free square, one wall.
        let (side, res) = (800, 0.25);
        let mut data = vec![UNKNOWN; side * side];
        for row in 380..420 {
            for col in 380..420 {
                data[row * side + col] = 0;
            }
        }
        data[400 * side + 410] = 100; // at (2.6, 0.1)
        let map = WholeMap {
            origin: Point::new(-100.0, -100.0),
            resolution: res,
            width: side,
            height: side,
            data: &data,
        };
        let grid = Grid::new(Point::new(-128.0, -128.0), 64.0).unwrap();
        let wall = Point::new(-100.0 + 410.5 * res, -100.0 + 400.5 * res);
        for detail in [Detail::Fine, Detail::Coarse] {
            let chunks = cut(&grid, &map, detail);
            // The free square straddles four chunks' corner (x = y = 0): only those four.
            assert_eq!(chunks.len(), 4, "{detail:?}");
            let holding = &chunks[&grid.key_at(wall)];
            assert_eq!(holding.at(&grid, wall), Some(100));
            assert_eq!(holding.at(&grid, Point::new(0.1, 0.1)), Some(0));
            assert!(chunks.values().all(|chunk| !chunk.is_unknown()));
        }
    }
}
