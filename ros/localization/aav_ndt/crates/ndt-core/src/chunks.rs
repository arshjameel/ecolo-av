//! The NDT map in fixed chunks: only the cubes near the car are built and held (TODO 175,
//! step 4; `docs/designs/2026-11-15_three-levels.md`).
//!
//! ```text
//!  campus.pcd ─▶ split ─▶ write_chunks ─▶ <dir>/i_j.pcd, one per 64 m chunk
//!
//!  car + route ─▶ ChunkedMap::update ─▶ loads the chunks wanted, drops the rest
//!                                    ─▶ map(): one CubeMap of the loaded chunks
//! ```
//!
//! A chunk's edges must fall on cube edges (its size and the grid's origin whole numbers
//! of cubes), so no cube is split between two chunks and the joined map is the same as
//! one built from all their points.

use std::collections::BTreeMap;
use std::fmt;
use std::fs::{self, File};
use std::io;
use std::path::{Path, PathBuf};

use map_chunks::{wanted, Changes, ChunkKey, Detail, Grid, Point as FlatPoint, Store};

use crate::map::{CubeMap, MapError, MapSettings};
use crate::pcd::{self, PcdError};
use crate::Point;

/// Why the chunked map couldn't be made or loaded.
#[derive(Debug)]
pub enum ChunkError {
    /// The chunk size or the grid's origin isn't a whole number of cubes.
    Misaligned {
        /// The chunk's side (m).
        chunk: f64,
        /// The cube's side (m).
        cube: f64,
    },
    /// A file couldn't be read or written.
    Io(PathBuf, io::Error),
    /// A chunk's file isn't a PCD this reads.
    Pcd(PathBuf, PcdError),
    /// The chunks' cubes couldn't be made.
    Map(MapError),
}

impl fmt::Display for ChunkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Misaligned { chunk, cube } => write!(
                f,
                "{chunk} m chunks don't fall on {cube} m cube edges: cubes would be split"
            ),
            Self::Io(path, e) => write!(f, "{}: {e}", path.display()),
            Self::Pcd(path, e) => write!(f, "{}: {e}", path.display()),
            Self::Map(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for ChunkError {}

/// The map's points, by the chunk each falls in.
#[must_use]
pub fn split(points: &[Point], grid: &Grid) -> BTreeMap<ChunkKey, Vec<Point>> {
    let mut chunks: BTreeMap<ChunkKey, Vec<Point>> = BTreeMap::new();
    for p in points.iter().filter(|p| p.iter().all(|v| v.is_finite())) {
        chunks
            .entry(grid.key_at(FlatPoint::new(p.x, p.y)))
            .or_default()
            .push(*p);
    }
    chunks
}

/// The file of one chunk's points.
#[must_use]
pub fn chunk_path(dir: &Path, key: ChunkKey) -> PathBuf {
    dir.join(format!("{key}.pcd"))
}

/// Write each chunk's points as `<dir>/<i_j>.pcd`. Returns how many chunks were written.
///
/// # Errors
///
/// [`ChunkError::Io`] if a file can't be written.
pub fn write_chunks(dir: &Path, grid: &Grid, points: &[Point]) -> Result<usize, ChunkError> {
    fs::create_dir_all(dir).map_err(|e| ChunkError::Io(dir.to_path_buf(), e))?;
    let chunks = split(points, grid);
    for (key, inside) in &chunks {
        let path = chunk_path(dir, *key);
        let file = File::create(&path).map_err(|e| ChunkError::Io(path.clone(), e))?;
        pcd::write(io::BufWriter::new(file), inside).map_err(|e| ChunkError::Io(path, e))?;
    }
    Ok(chunks.len())
}

/// Only the chunks near the car, built into one map.
#[derive(Debug)]
pub struct ChunkedMap {
    dir: PathBuf,
    grid: Grid,
    settings: MapSettings,
    /// Each loaded chunk's cubes; `None` for a chunk with no file or too few points.
    store: Store<Option<CubeMap>>,
    joined: Option<CubeMap>,
}

impl ChunkedMap {
    /// A map that loads `<dir>/<i_j>.pcd` on `grid`, with cubes made by `settings`.
    ///
    /// # Errors
    ///
    /// [`ChunkError::Misaligned`] if the chunks aren't a whole number of cubes.
    pub fn new(dir: &Path, grid: Grid, settings: MapSettings) -> Result<Self, ChunkError> {
        let whole = |v: f64| (v / settings.cube - (v / settings.cube).round()).abs() < 1e-9;
        let origin = grid.origin();
        if !(whole(grid.size()) && whole(origin.x) && whole(origin.y)) {
            return Err(ChunkError::Misaligned {
                chunk: grid.size(),
                cube: settings.cube,
            });
        }
        Ok(Self {
            dir: dir.to_path_buf(),
            grid,
            settings,
            store: Store::new(),
            joined: None,
        })
    }

    /// Load the chunks the car needs (the 3 x 3 round it and the route's next `ahead` m)
    /// and drop the rest; the joined map is rebuilt only when something changed.
    ///
    /// # Errors
    ///
    /// [`ChunkError`]: a chunk's file can't be read, or isn't a good PCD, or its cubes can't be
    /// made.
    pub fn update(
        &mut self,
        car: FlatPoint,
        route: &[FlatPoint],
        ahead: f64,
    ) -> Result<Changes, ChunkError> {
        // NDT keeps one detail: every wanted chunk at full.
        let want: BTreeMap<ChunkKey, Detail> = wanted(&self.grid, car, route, ahead)
            .into_keys()
            .map(|key| (key, Detail::Fine))
            .collect();
        let (dir, settings) = (&self.dir, &self.settings);
        let changes = self
            .store
            .update(&want, |key, _| load_chunk(dir, key, settings))?;
        if !(changes.loaded.is_empty() && changes.dropped.is_empty()) {
            let parts: Vec<&CubeMap> = self
                .store
                .loaded()
                .filter_map(|(key, _)| self.store.get(key).and_then(|(_, part)| part.as_ref()))
                .collect();
            self.joined = CubeMap::join(parts);
        }
        Ok(changes)
    }

    /// The cubes of the loaded chunks; `None` before the first update or with none near.
    #[must_use]
    pub fn map(&self) -> Option<&CubeMap> {
        self.joined.as_ref()
    }

    /// The loaded chunks.
    pub fn loaded(&self) -> impl Iterator<Item = ChunkKey> + '_ {
        self.store.loaded().map(|(key, _)| key)
    }
}

/// The points of the chunks near the car, for a matcher with several cube sizes.
///
/// The node rebuilds its `Matcher` (`Matcher::new_or_empty`) from `points()` whenever
/// `update` says the loaded chunks changed: about once a chunk (64 m) driven.
#[derive(Debug)]
pub struct ChunkedPoints {
    dir: PathBuf,
    grid: Grid,
    store: Store<Vec<Point>>,
}

impl ChunkedPoints {
    /// The points of `<dir>/<i_j>.pcd` on `grid`, none loaded yet.
    #[must_use]
    pub fn new(dir: &Path, grid: Grid) -> Self {
        Self {
            dir: dir.to_path_buf(),
            grid,
            store: Store::new(),
        }
    }

    /// Load the 3 x 3 chunks round `car`; true if they changed.
    ///
    /// # Errors
    ///
    /// [`ChunkError::Io`] or [`ChunkError::Pcd`] for a chunk's file.
    pub fn update(&mut self, car: FlatPoint) -> Result<bool, ChunkError> {
        let want: BTreeMap<ChunkKey, Detail> = wanted(&self.grid, car, &[], 0.0)
            .into_keys()
            .map(|key| (key, Detail::Fine))
            .collect();
        let dir = &self.dir;
        let changes = self.store.update(&want, |key, _| read_points(dir, key))?;
        Ok(!(changes.loaded.is_empty() && changes.dropped.is_empty()))
    }

    /// Every loaded chunk's points.
    #[must_use]
    pub fn points(&self) -> Vec<Point> {
        self.store
            .loaded()
            .filter_map(|(key, _)| self.store.get(key))
            .flat_map(|(_, points)| points.iter().copied())
            .collect()
    }

    /// The loaded chunks.
    pub fn loaded(&self) -> impl Iterator<Item = ChunkKey> + '_ {
        self.store.loaded().map(|(key, _)| key)
    }
}

/// One chunk's points: none if it has no file.
fn read_points(dir: &Path, key: ChunkKey) -> Result<Vec<Point>, ChunkError> {
    let path = chunk_path(dir, key);
    match File::open(&path) {
        Ok(file) => pcd::read(file).map_err(|e| ChunkError::Pcd(path, e)),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(Vec::new()),
        Err(e) => Err(ChunkError::Io(path, e)),
    }
}

/// One chunk's cubes: `None` if it has no file or too few points for a cube.
fn load_chunk(
    dir: &Path,
    key: ChunkKey,
    settings: &MapSettings,
) -> Result<Option<CubeMap>, ChunkError> {
    let path = chunk_path(dir, key);
    let file = match File::open(&path) {
        Ok(file) => file,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(ChunkError::Io(path, e)),
    };
    let points = pcd::read(file).map_err(|e| ChunkError::Pcd(path, e))?;
    match CubeMap::build(&points, settings) {
        Ok(map) => Ok(Some(map)),
        Err(MapError::NoCubes { .. }) => Ok(None),
        Err(e) => Err(ChunkError::Map(e)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("ndt-chunks-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        dir
    }

    /// Walls every 16 m along x from -200 to 200, 3 m tall, points every 0.25 m.
    fn street() -> Vec<Point> {
        let mut points = Vec::new();
        for wall in -12..=12 {
            let x = f64::from(wall) * 16.0 + 0.3;
            for j in 0..160 {
                for k in 0..12 {
                    let (y, z) = (-20.0 + f64::from(j) * 0.25, f64::from(k) * 0.25);
                    points.push(Point::new(x, y, z));
                }
            }
        }
        points
    }

    fn grid() -> Grid {
        Grid::new(FlatPoint::new(-256.0, -256.0), 64.0).unwrap()
    }

    #[test]
    fn near_the_car_the_chunks_give_the_same_cubes_as_the_whole_map() {
        let dir = scratch("same");
        let points = street();
        let settings = MapSettings::with_cube(1.0);
        assert!(write_chunks(&dir, &grid(), &points).unwrap() >= 7);
        let mut chunked = ChunkedMap::new(&dir, grid(), settings).unwrap();
        let car = FlatPoint::new(1.0, 1.0);
        let changes = chunked.update(car, &[], 0.0).unwrap();
        assert_eq!(changes.loaded.len(), 9);
        // The whole map, cut to the same 3 x 3 (x and y from -64 to 128), cube for cube.
        let near: Vec<Point> = points
            .iter()
            .filter(|p| (-64.0..128.0).contains(&p.x) && (-64.0..128.0).contains(&p.y))
            .copied()
            .collect();
        let whole = CubeMap::build(&near, &settings).unwrap();
        let joined = chunked.map().unwrap();
        assert_eq!(joined.len(), whole.len());
        for p in &near {
            assert_eq!(
                joined.get(p).map(|c| c.points),
                whole.get(p).map(|c| c.points)
            );
        }
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn far_walls_are_not_held() {
        let dir = scratch("far");
        let points = street();
        write_chunks(&dir, &grid(), &points).unwrap();
        let mut chunked = ChunkedMap::new(&dir, grid(), MapSettings::with_cube(1.0)).unwrap();
        chunked.update(FlatPoint::new(1.0, 1.0), &[], 0.0).unwrap();
        let far = Point::new(-191.7, 0.0, 1.0); // the wall at x = -191.7
        assert!(chunked.map().unwrap().get(&far).is_none());
        // Driving there loads it, and drops the chunks left behind.
        let changes = chunked
            .update(FlatPoint::new(-190.0, 1.0), &[], 0.0)
            .unwrap();
        assert!(!changes.dropped.is_empty());
        assert!(chunked.map().unwrap().get(&far).is_some());
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn chunks_must_fall_on_cube_edges() {
        let odd = Grid::new(FlatPoint::new(-256.5, -256.0), 64.0).unwrap();
        let err = ChunkedMap::new(Path::new("."), odd, MapSettings::with_cube(1.0)).unwrap_err();
        assert!(matches!(err, ChunkError::Misaligned { .. }), "{err}");
        assert!(ChunkedMap::new(Path::new("."), grid(), MapSettings::with_cube(2.0)).is_ok());
    }

    #[test]
    fn the_points_near_the_car_feed_a_matcher_that_finds_the_wall() {
        use crate::{MatchSettings, Matcher};
        let dir = scratch("points");
        write_chunks(&dir, &grid(), &street()).unwrap();
        let mut near = ChunkedPoints::new(&dir, grid());
        assert!(near.update(FlatPoint::new(1.0, 1.0)).unwrap());
        assert!(!near.update(FlatPoint::new(2.0, 1.0)).unwrap()); // the same chunk
        let points = near.points();
        assert!(points.iter().all(|p| (-64.0..128.0).contains(&p.x)));
        let matcher = Matcher::new_or_empty(
            &points,
            &[2.0, 1.0],
            MapSettings::with_cube,
            MatchSettings::default(),
        )
        .unwrap();
        assert!(matcher.levels().all(|(_, curves)| curves > 0));
        // Far from every wall: empty levels, not an error.
        let empty = Matcher::new_or_empty(
            &[],
            &[2.0, 1.0],
            MapSettings::with_cube,
            MatchSettings::default(),
        )
        .unwrap();
        assert!(empty.levels().all(|(_, curves)| curves == 0));
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn no_file_near_is_no_map_not_an_error() {
        let dir = scratch("empty");
        let mut chunked = ChunkedMap::new(&dir, grid(), MapSettings::with_cube(1.0)).unwrap();
        assert!(chunked.update(FlatPoint::new(0.0, 0.0), &[], 0.0).is_ok());
        assert!(chunked.map().is_none());
    }
}
