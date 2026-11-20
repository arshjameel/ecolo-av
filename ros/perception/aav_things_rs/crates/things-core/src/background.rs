//! The saved map as the background: cubes on it are the walls again, not new things.
//!
//! Ben, 14 Nov: "NDT can give us the stationary things ... then all we need is occupancy
//! graph to track poepole". Step 5 of the three levels
//! (`docs/designs/2026-11-15_three-levels.md`):
//!
//! ```text
//! live cubes ──► on a taken cell of the saved map (± slack for pose error)?
//!                   yes ─► the wall again: dropped before grouping
//!                   no  ─► a new occupancy: grouped and tracked as before
//! ```
//!
//! | From | What is "taken" | Made by |
//! |---|---|---|
//! | `/map` (the occupancy grid) | every wall cell | [`Background::from_grid`] |
//! | `/map`, low cubes only (`below`) | the same cells, but only cubes under `below` m drop | [`new_only`] |
//! | the NDT map's chunks | where the saved 3D map has points over `min_z` | [`ChunkBackground`] |
//!
//! Only the tracker uses this. The stop zone and the costmaps see every point, always.

use std::{
    error::Error,
    fmt, fs, io,
    path::{Path, PathBuf},
};

use map_chunks::{ChunkKey, Grid};
use rayon::prelude::*;

use crate::Pose2;

/// ROS OccupancyGrid: this or more is a wall (map_server's `occupied_thresh`).
pub const TAKEN: i8 = 65;

/// The saved map's taken cells, grown by the slack: `true` where a cube is the map.
#[derive(Clone, Debug, PartialEq)]
pub struct Background {
    origin: [f64; 2],
    resolution: f64,
    width: usize,
    height: usize,
    /// Row by row from `origin`'s row (y), `width` a row.
    taken: Vec<bool>,
}

impl Background {
    /// One from a ROS OccupancyGrid's cells (row by row), each wall grown by `slack` m.
    /// `None` if `data` doesn't hold `width` × `height` cells.
    #[must_use]
    pub fn from_grid(
        data: &[i8],
        width: usize,
        height: usize,
        resolution: f64,
        origin: [f64; 2],
        slack: f64,
    ) -> Option<Self> {
        if data.len() != width.checked_mul(height)? {
            return None;
        }
        let taken = data.iter().map(|&c| c >= TAKEN).collect();
        Some(Self::grown(origin, resolution, width, height, taken, slack))
    }

    /// One `side` m square from `origin`: taken where a point (x, y) falls.
    #[must_use]
    pub fn from_points(
        xy: &[[f64; 2]],
        origin: [f64; 2],
        side: f64,
        resolution: f64,
        slack: f64,
    ) -> Self {
        let n = (side / resolution).round_ties_even().max(0.0) as usize;
        let mut taken = vec![false; n * n];
        for p in xy {
            if let Some(i) = cell(p, origin, resolution, n, n) {
                taken[i] = true;
            }
        }
        Self::grown(origin, resolution, n, n, taken, slack)
    }

    fn grown(
        origin: [f64; 2],
        resolution: f64,
        width: usize,
        height: usize,
        taken: Vec<bool>,
        slack: f64,
    ) -> Self {
        let reach = (slack / resolution - 1e-9).ceil().max(0.0) as usize;
        if reach == 0 || width == 0 || height == 0 {
            return Self {
                origin,
                resolution,
                width,
                height,
                taken,
            };
        }
        // A square of `reach` cells round each taken cell: along the rows, then the
        // columns. Two grids at a time: the campus's /map is 6400 x 6400.
        // Each row on its own core (TODO 195): a row of `rows` from its own row of `taken`,
        // a row of `grown` from the rows of `rows` within `reach` of it.
        let mut rows = vec![false; taken.len()];
        taken
            .par_chunks(width)
            .zip(rows.par_chunks_mut(width))
            .for_each(|(row, out)| {
                for (col, _) in row.iter().enumerate().filter(|(_, &t)| t) {
                    let (a, b) = (col.saturating_sub(reach), (col + reach).min(width - 1));
                    out[a..=b].fill(true);
                }
            });
        drop(taken);
        let mut grown = vec![false; rows.len()];
        grown
            .par_chunks_mut(width)
            .enumerate()
            .for_each(|(r, out)| {
                let (a, b) = (r.saturating_sub(reach), (r + reach).min(height - 1));
                for near in rows[a * width..(b + 1) * width].chunks(width) {
                    out.iter_mut().zip(near).for_each(|(o, &n)| *o |= n);
                }
            });
        Self {
            origin,
            resolution,
            width,
            height,
            taken: grown,
        }
    }

    /// Whether (x, y), in the map frame, is on a (grown) wall. Off the map: never.
    #[must_use]
    pub fn on_map(&self, x: f64, y: f64) -> bool {
        cell(
            &[x, y],
            self.origin,
            self.resolution,
            self.width,
            self.height,
        )
        .is_some_and(|i| self.taken[i])
    }
}

/// The index of the cell under `p`, if it is on a `width` × `height` grid.
fn cell(
    p: &[f64; 2],
    origin: [f64; 2],
    resolution: f64,
    width: usize,
    height: usize,
) -> Option<usize> {
    let col = ((p[0] - origin[0]) / resolution).floor();
    let row = ((p[1] - origin[1]) / resolution).floor();
    let inside = col >= 0.0 && row >= 0.0 && col < width as f64 && row < height as f64;
    inside.then(|| row as usize * width + col as usize)
}

/// The cubes (x, y, z in the car's frame) not on the saved map.
///
/// `pose`: the cubes' frame in the map's frame. `below` (m, the cubes' z): if given, only
/// cubes lower than it may be dropped; a taller one is kept even on the map, so a person on
/// the grass beside the road keeps their body (`/map` marks the grass).
#[must_use]
pub fn new_only(
    points: &[[f64; 3]],
    pose: Pose2,
    background: &Background,
    below: Option<f64>,
) -> Vec<[f64; 3]> {
    let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
    points
        .iter()
        .copied()
        .filter(|p| {
            let x = pose.x + c * p[0] - s * p[1];
            let y = pose.y + s * p[0] + c * p[1];
            let drop = background.on_map(x, y) && below.is_none_or(|b| p[2] < b);
            !drop
        })
        .collect()
}

/// A chunk folder that can't be read.
#[derive(Debug)]
pub enum ChunkError {
    /// The grid (`chunks.yaml` beside the folder).
    Grid(map_chunks::files::FileError),
    /// A chunk's PCD.
    Pcd(PathBuf, ndt_core::pcd::PcdError),
    /// A chunk's file.
    Io(PathBuf, io::Error),
}

impl fmt::Display for ChunkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Grid(e) => write!(f, "the chunk grid: {e}"),
            Self::Pcd(path, e) => write!(f, "{}: {e}", path.display()),
            Self::Io(path, e) => write!(f, "{}: {e}", path.display()),
        }
    }
}

impl Error for ChunkError {}

/// The NDT map's chunks as the background: the 3 × 3 round the car, read again as it
/// crosses into another chunk (TODO 175 step 5). Points lower than `min_z` (m, the map
/// frame) are the ground and taken as nothing.
#[derive(Debug)]
pub struct ChunkBackground {
    folder: PathBuf,
    grid: Grid,
    resolution: f64,
    slack: f64,
    min_z: f64,
    here: Option<ChunkKey>,
    background: Option<Background>,
    /// Times the 3 × 3 was read.
    pub loads: u32,
}

impl ChunkBackground {
    /// The chunks in `folder` (such as `maps/campus/chunks/ndt`), on the grid in the
    /// `chunks.yaml` beside it (`maps/campus/chunks/chunks.yaml`).
    ///
    /// # Errors
    ///
    /// [`ChunkError::Grid`]: no `chunks.yaml`, or a bad one.
    pub fn open(folder: &Path, slack: f64, min_z: f64) -> Result<Self, ChunkError> {
        let parent = folder
            .canonicalize()
            .ok()
            .and_then(|f| f.parent().map(Path::to_path_buf))
            .unwrap_or_else(|| folder.join(".."));
        let grid = map_chunks::files::read_grid(&parent).map_err(ChunkError::Grid)?;
        Ok(Self::new(folder, grid, 0.25, slack, min_z))
    }

    /// The chunks in `folder`, cut on `grid`, as `resolution` m cells.
    #[must_use]
    pub fn new(folder: &Path, grid: Grid, resolution: f64, slack: f64, min_z: f64) -> Self {
        Self {
            folder: folder.to_path_buf(),
            grid,
            resolution,
            slack,
            min_z,
            here: None,
            background: None,
            loads: 0,
        }
    }

    /// The chunk the 3 × 3 is centred on, once read.
    #[must_use]
    pub fn here(&self) -> Option<ChunkKey> {
        self.here
    }

    /// The background round (x, y), the chunks read again in a new chunk. A chunk with no
    /// file is empty.
    ///
    /// # Errors
    ///
    /// [`ChunkError`]: a chunk's file is there but can't be read.
    pub fn update(&mut self, x: f64, y: f64) -> Result<&Background, ChunkError> {
        let key = self.grid.key_at(map_chunks::Point::new(x, y));
        if self.here != Some(key) || self.background.is_none() {
            // The 3 × 3 chunks, each read on its own core (TODO 195), kept in order.
            let keys: Vec<(i32, i32)> = (key.i - 1..=key.i + 1)
                .flat_map(|i| (key.j - 1..=key.j + 1).map(move |j| (i, j)))
                .collect();
            let min_z = self.min_z;
            let read = |&(i, j): &(i32, i32)| -> Result<Vec<[f64; 2]>, ChunkError> {
                let path = self.folder.join(format!("{i}_{j}.pcd"));
                if !path.exists() {
                    return Ok(Vec::new());
                }
                let file = fs::File::open(&path).map_err(|e| ChunkError::Io(path.clone(), e))?;
                let points =
                    ndt_core::pcd::read(file).map_err(|e| ChunkError::Pcd(path.clone(), e))?;
                Ok(points
                    .iter()
                    .filter(|p| p.z >= min_z)
                    .map(|p| [p.x, p.y])
                    .collect())
            };
            let xy: Vec<[f64; 2]> = keys
                .par_iter()
                .map(read)
                .collect::<Result<Vec<_>, _>>()?
                .concat();
            let corner = self.grid.corner(ChunkKey::new(key.i - 1, key.j - 1));
            self.background = Some(Background::from_points(
                &xy,
                [corner.x, corner.y],
                3.0 * self.grid.size(),
                self.resolution,
                self.slack,
            ));
            self.here = Some(key);
            self.loads += 1;
        }
        // Set just above.
        self.background
            .as_ref()
            .ok_or_else(|| ChunkError::Io(self.folder.clone(), io::Error::other("no background")))
    }
}
