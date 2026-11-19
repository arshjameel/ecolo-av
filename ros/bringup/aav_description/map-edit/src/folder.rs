//! A map folder as `map-drive` writes it, read into memory (`docs/maps-from-drives.md`).
//!
//! | File | Needed | Read as |
//! |---|---|---|
//! | `ndt.pcd` | yes | NDT's points, the map frame |
//! | `map2d.yaml` + `.pgm` | yes | the 2D map: free, occupied, unknown |
//! | `ndt.yaml` | no | kept whole: where the map is placed (`placed`, its datum) |
//! | `poses.tum` | no | the drive's path |
//! | `labels.yaml` | no | labels from an earlier edit |

use std::fmt;
use std::fs::{self, File};
use std::io;
use std::path::{Path, PathBuf};

use serde_yaml_ng::{Mapping, Value};

use crate::geom::{Align, Point};
use crate::labels::{Label, LabelsFile};

/// One of NDT's points (x, y, z m).
pub use ndt_core::Point as Point3;

/// Why a map folder couldn't be read.
#[derive(Debug)]
pub enum LoadError {
    /// A file couldn't be read.
    Io(PathBuf, io::Error),
    /// A file isn't what it should be.
    Bad(PathBuf, String),
}

impl fmt::Display for LoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(path, e) => write!(f, "{}: {e}", path.display()),
            Self::Bad(path, why) => write!(f, "{}: {why}", path.display()),
        }
    }
}

impl std::error::Error for LoadError {}

/// One cell of the 2D map, as map_server's trinary mode reads it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cell {
    /// The lidar's rays crossed it.
    Free,
    /// Something stands there.
    Occupied,
    /// Nothing is known.
    Unknown,
}

impl Cell {
    /// From ROS's count: -1 unknown, 0..100.
    #[must_use]
    pub fn from_ros(value: i8) -> Self {
        match value {
            v if v < 0 => Self::Unknown,
            v if v >= 50 => Self::Occupied,
            _ => Self::Free,
        }
    }

    /// ROS's count: -1, 0 or 100.
    #[must_use]
    pub fn to_ros(self) -> i8 {
        match self {
            Self::Free => 0,
            Self::Occupied => 100,
            Self::Unknown => map_chunks::UNKNOWN,
        }
    }

    /// The trinary `.pgm`'s grey: free 254, occupied 0, unknown 128
    /// (as map-drive writes it: half-grey reads unknown between the thresholds 0.25 and 0.65).
    #[must_use]
    pub fn pixel(self) -> u8 {
        match self {
            Self::Free => 254,
            Self::Occupied => 0,
            Self::Unknown => 128,
        }
    }
}

/// The 2D map: square cells, row by row from the lower left, x fastest (as ROS has them).
#[derive(Debug, Clone, PartialEq)]
pub struct Grid2d {
    /// The lower-left corner of cell (0, 0).
    pub origin: Point,
    /// One cell's side (m).
    pub resolution: f64,
    /// Cells across.
    pub width: usize,
    /// Cells up.
    pub height: usize,
    /// `width * height` cells.
    pub cells: Vec<Cell>,
}

impl Grid2d {
    /// The middle of cell `index`.
    #[must_use]
    pub fn centre(&self, index: usize) -> Point {
        let (col, row) = (index % self.width.max(1), index / self.width.max(1));
        Point::new(
            self.origin.x + (col as f64 + 0.5) * self.resolution,
            self.origin.y + (row as f64 + 0.5) * self.resolution,
        )
    }

    /// The cell under `p`, if it is on the map.
    #[must_use]
    pub fn index_at(&self, p: Point) -> Option<usize> {
        let cx = ((p.x - self.origin.x) / self.resolution).floor();
        let cy = ((p.y - self.origin.y) / self.resolution).floor();
        let inside = |c: f64, n: usize| c >= 0.0 && c < n as f64;
        (inside(cx, self.width) && inside(cy, self.height))
            .then(|| cy as usize * self.width + cx as usize)
    }

    /// The map's box: (lower left, upper right).
    #[must_use]
    pub fn bounds(&self) -> (Point, Point) {
        (
            self.origin,
            Point::new(
                self.origin.x + self.width as f64 * self.resolution,
                self.origin.y + self.height as f64 * self.resolution,
            ),
        )
    }

    /// The cells whose middles may be in the box `lo`..`hi`: their columns and rows.
    #[must_use]
    pub fn span(&self, lo: Point, hi: Point) -> (std::ops::Range<usize>, std::ops::Range<usize>) {
        let range = |a: f64, b: f64, start: f64, n: usize| {
            let first = ((a - start) / self.resolution - 0.5).floor().max(0.0) as usize;
            let last = ((((b - start) / self.resolution) - 0.5).ceil() + 1.0).max(0.0) as usize;
            first.min(n)..last.min(n)
        };
        (
            range(lo.x, hi.x, self.origin.x, self.width),
            range(lo.y, hi.y, self.origin.y, self.height),
        )
    }

    /// The map after `align` moves it, on new cells of the same size. With no move, the
    /// same map. Otherwise its origin is on whole metres (NDT's chunks need their edges on
    /// its 1 m cubes), and each new cell takes the old cell under its middle.
    #[must_use]
    pub fn aligned(&self, align: &Align) -> Self {
        if align.is_identity() {
            return self.clone();
        }
        let (origin, width, height) = self.aligned_frame(align);
        let mut out = Self {
            origin,
            resolution: self.resolution,
            width,
            height,
            cells: vec![Cell::Unknown; width * height],
        };
        for index in 0..out.cells.len() {
            let from = align.invert(out.centre(index));
            if let Some(at) = self.index_at(from) {
                out.cells[index] = self.cells[at];
            }
        }
        out
    }

    /// Where [`Grid2d::aligned`] puts the moved map: its origin (whole metres, or the
    /// same origin with no move), cells across and up. Cheap: no cells are made.
    #[must_use]
    pub fn aligned_frame(&self, align: &Align) -> (Point, usize, usize) {
        if align.is_identity() {
            return (self.origin, self.width, self.height);
        }
        let (lo, hi) = self.bounds();
        let corners =
            [lo, hi, Point::new(lo.x, hi.y), Point::new(hi.x, lo.y)].map(|p| align.apply(p));
        let (min, max) = crate::geom::bounds_of(corners).unwrap_or((lo, hi));
        let origin = Point::new(min.x.floor(), min.y.floor());
        let r = self.resolution;
        let width = ((max.x - origin.x) / r).ceil().max(1.0) as usize;
        let height = ((max.y - origin.y) / r).ceil().max(1.0) as usize;
        (origin, width, height)
    }

    /// The trinary `.pgm`'s pixels, top row first.
    #[must_use]
    pub fn pgm_pixels(&self) -> Vec<u8> {
        self.cells
            .chunks(self.width.max(1))
            .rev()
            .flatten()
            .map(|c| c.pixel())
            .collect()
    }
}

/// One pose of the drive's path, as a TUM line: `t x y z qx qy qz qw`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TumPose {
    /// Seconds.
    pub t: f64,
    /// Where (m).
    pub xyz: [f64; 3],
    /// Which way: the quaternion x, y, z, w.
    pub q: [f64; 4],
}

impl TumPose {
    /// The heading (rad), for a flat pose.
    #[must_use]
    pub fn yaw(&self) -> f64 {
        let [x, y, z, w] = self.q;
        (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
    }

    /// The pose after `align` moves the map: its place moved, its quaternion turned
    /// about z.
    #[must_use]
    pub fn aligned(&self, align: &Align) -> Self {
        let p = align.apply(Point::new(self.xyz[0], self.xyz[1]));
        let (s, c) = (align.yaw_rad / 2.0).sin_cos();
        let [x, y, z, w] = self.q;
        Self {
            t: self.t,
            xyz: [p.x, p.y, self.xyz[2]],
            q: [c * x - s * y, c * y + s * x, c * z + s * w, c * w - s * z],
        }
    }

    /// The TUM line.
    #[must_use]
    pub fn line(&self) -> String {
        let [x, y, z] = self.xyz;
        let [qx, qy, qz, qw] = self.q;
        format!(
            "{:.6} {x:.4} {y:.4} {z:.4} {qx:.6} {qy:.6} {qz:.6} {qw:.6}",
            self.t
        )
    }
}

/// A map folder, read.
#[derive(Debug, Clone)]
pub struct MapFolder {
    /// Where it was read from.
    pub dir: PathBuf,
    /// NDT's points.
    pub points: Vec<Point3>,
    /// `ndt.yaml`, whole (empty if there is none).
    pub ndt_yaml: Mapping,
    /// The 2D map.
    pub grid: Grid2d,
    /// `map2d.yaml`, whole: its thresholds and `placed` are kept on save.
    pub grid_yaml: Mapping,
    /// The drive's path (`poses.tum`), if there is one.
    pub path: Vec<TumPose>,
    /// Labels from an earlier edit (`labels.yaml`).
    pub labels: Vec<Label>,
}

impl MapFolder {
    /// Reads a map folder.
    ///
    /// # Errors
    ///
    /// [`LoadError`]: a needed file is missing, or a file isn't what it should be.
    pub fn load(dir: &Path) -> Result<Self, LoadError> {
        let pcd = dir.join("ndt.pcd");
        let file = File::open(&pcd).map_err(|e| LoadError::Io(pcd.clone(), e))?;
        let points = ndt_core::pcd::read(file).map_err(|e| LoadError::Bad(pcd, e.to_string()))?;

        let grid_path = dir.join("map2d.yaml");
        let loaded = map_chunks::files::read_map(&grid_path)
            .map_err(|e| LoadError::Bad(grid_path.clone(), e.to_string()))?;
        let grid = Grid2d {
            origin: Point::new(loaded.yaml.origin[0], loaded.yaml.origin[1]),
            resolution: loaded.yaml.resolution,
            width: loaded.width,
            height: loaded.height,
            cells: loaded.data.iter().map(|&v| Cell::from_ros(v)).collect(),
        };
        let grid_yaml = read_yaml(&grid_path)?.unwrap_or_default();
        let ndt_yaml = read_yaml(&dir.join("ndt.yaml"))?.unwrap_or_default();

        let tum = dir.join("poses.tum");
        let path = match fs::read_to_string(&tum) {
            Ok(text) => read_tum(&text).map_err(|why| LoadError::Bad(tum, why))?,
            Err(e) if e.kind() == io::ErrorKind::NotFound => Vec::new(),
            Err(e) => return Err(LoadError::Io(tum, e)),
        };

        let labels_path = dir.join("labels.yaml");
        let labels = match fs::read_to_string(&labels_path) {
            Ok(text) => {
                serde_yaml_ng::from_str::<LabelsFile>(&text)
                    .map_err(|e| LoadError::Bad(labels_path, e.to_string()))?
                    .labels
            }
            Err(e) if e.kind() == io::ErrorKind::NotFound => Vec::new(),
            Err(e) => return Err(LoadError::Io(labels_path, e)),
        };

        Ok(Self {
            dir: dir.to_path_buf(),
            points,
            ndt_yaml,
            grid,
            grid_yaml,
            path,
            labels,
        })
    }

    /// The datum (latitude, longitude, degrees) the map frame starts at, if it was placed
    /// on the Earth (`placed: datum:` in `ndt.yaml` or `map2d.yaml`).
    #[must_use]
    pub fn datum(&self) -> Option<[f64; 2]> {
        [&self.ndt_yaml, &self.grid_yaml]
            .into_iter()
            .find_map(|yaml| {
                let datum = yaml.get("placed")?.get("datum")?.as_sequence()?;
                let lat = datum.first()?.as_f64()?;
                let lon = datum.get(1)?.as_f64()?;
                Some([lat, lon])
            })
    }

    /// How the map was placed (`gps`, `start`, `local`), if it says.
    #[must_use]
    pub fn placed_by(&self) -> Option<String> {
        self.ndt_yaml
            .get("placed")
            .and_then(|p| p.get("method"))
            .and_then(Value::as_str)
            .map(str::to_owned)
    }
}

fn read_yaml(path: &Path) -> Result<Option<Mapping>, LoadError> {
    match fs::read_to_string(path) {
        Ok(text) => serde_yaml_ng::from_str::<Mapping>(&text)
            .map(Some)
            .map_err(|e| LoadError::Bad(path.to_path_buf(), e.to_string())),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(LoadError::Io(path.to_path_buf(), e)),
    }
}

/// Reads TUM lines; `#` comments and blank lines are skipped.
///
/// # Errors
///
/// Which line isn't eight numbers.
pub fn read_tum(text: &str) -> Result<Vec<TumPose>, String> {
    text.lines()
        .enumerate()
        .map(|(n, line)| (n, line.trim()))
        .filter(|(_, line)| !line.is_empty() && !line.starts_with('#'))
        .map(|(n, line)| {
            let v: Vec<f64> = line
                .split_whitespace()
                .map(str::parse)
                .collect::<Result<_, _>>()
                .map_err(|_| format!("line {}: not numbers", n + 1))?;
            match v[..] {
                [t, x, y, z, qx, qy, qz, qw] => Ok(TumPose {
                    t,
                    xyz: [x, y, z],
                    q: [qx, qy, qz, qw],
                }),
                _ => Err(format!("line {}: {} numbers, not 8", n + 1, v.len())),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid() -> Grid2d {
        // 4 x 2 cells of 0.5 m from (-1, 0): occupied in the lower right.
        let mut cells = vec![Cell::Free; 8];
        cells[3] = Cell::Occupied;
        cells[4] = Cell::Unknown;
        Grid2d {
            origin: Point::new(-1.0, 0.0),
            resolution: 0.5,
            width: 4,
            height: 2,
            cells,
        }
    }

    #[test]
    fn cells_and_places() {
        let g = grid();
        assert_eq!(g.index_at(Point::new(0.9, 0.1)), Some(3));
        assert_eq!(g.index_at(Point::new(1.1, 0.1)), None);
        let c = g.centre(3);
        assert!((c.x - 0.75).abs() < 1e-12 && (c.y - 0.25).abs() < 1e-12);
        // The .pgm is top row first: the occupied cell is in its second row.
        assert_eq!(g.pgm_pixels()[4 + 3], 0);
        assert_eq!(g.pgm_pixels()[0], 128);
    }

    #[test]
    fn an_aligned_grid_moves_its_cells() {
        let g = grid();
        let moved = g.aligned(&Align::new(10.25, 0.0, 0.0));
        assert_eq!(moved.origin, Point::new(9.0, 0.0));
        let at = moved.index_at(Point::new(10.8, 0.25)).expect("on the map");
        assert_eq!(moved.cells[at], Cell::Occupied);
        assert_eq!(g.aligned(&Align::IDENTITY), g);
        // A half turn: the occupied cell goes to the other side of (0, 0).
        let turned = g.aligned(&Align::new(0.0, 0.0, std::f64::consts::PI));
        let at = turned
            .index_at(Point::new(-0.75, -0.25))
            .expect("on the map");
        assert_eq!(turned.cells[at], Cell::Occupied);
    }

    #[test]
    fn tum_lines_and_turns() {
        let poses = read_tum("# t x y z qx qy qz qw\n1.0 2 3 0 0 0 0 1\n\n").expect("read");
        assert_eq!(poses.len(), 1);
        let turned = poses[0].aligned(&Align::new(0.0, 0.0, 0.5));
        assert!((turned.yaw() - 0.5).abs() < 1e-9);
        assert!(read_tum("1 2 3").is_err());
        assert_eq!(read_tum(&poses[0].line()).expect("again"), poses);
    }
}
