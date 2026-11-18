//! Nav2's occupancy grid (`/map`, nav_msgs/OccupancyGrid), and its pixels for the screen.
//!
//! The message's cells run row by row from the grid's origin corner: x first, then y.
//! A picture runs from its top row down, so [`Grid::to_rgba`] flips it: north is up.

use thiserror::Error;

use crate::geom::{Metres, Point, Pose};

/// Where the grid lies, and how big a cell is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridInfo {
    /// Cells across (x).
    pub width: u32,
    /// Cells up (y).
    pub height: u32,
    /// Metres per cell.
    pub resolution: f64,
    /// Where cell (0, 0)'s outer corner is on the map, and how the grid is turned.
    pub origin: Pose,
}

impl GridInfo {
    /// The grid's size in metres: (across, up).
    #[must_use]
    pub fn size(&self) -> (f64, f64) {
        (
            f64::from(self.width) * self.resolution,
            f64::from(self.height) * self.resolution,
        )
    }

    /// The four corners on the map, counter-clockwise from the origin's.
    #[must_use]
    pub fn corners(&self) -> [Point; 4] {
        let (w, h) = self.size();
        [
            Point::new(0.0, 0.0),
            Point::new(w, 0.0),
            Point::new(w, h),
            Point::new(0.0, h),
        ]
        .map(|p| self.origin.transform(p))
    }

    /// The smallest box, on the map's axes, that holds the whole grid.
    #[must_use]
    pub fn bounds(&self) -> Bounds {
        Bounds::around(self.corners()).unwrap_or_default()
    }

    /// A map point in the grid's own frame: metres from cell (0, 0)'s corner.
    fn local(&self, p: Point) -> Point {
        (p - self.origin.position).rotated(-self.origin.yaw)
    }

    /// Whether a map point is on the grid (edges included).
    #[must_use]
    pub fn contains(&self, p: Point) -> bool {
        self.distance_outside(p).0 == 0.0
    }

    /// How far a map point is from the grid: 0 on it, else to its nearest edge.
    #[must_use]
    pub fn distance_outside(&self, p: Point) -> Metres {
        let q = self.local(p);
        let (w, h) = self.size();
        let dx = (-q.x).max(q.x - w).max(0.0);
        let dy = (-q.y).max(q.y - h).max(0.0);
        Metres(dx.hypot(dy))
    }
}

/// A box on the map's axes.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Bounds {
    /// The lower-left corner.
    pub min: Point,
    /// The upper-right corner.
    pub max: Point,
}

impl Bounds {
    /// The box around some points; None if there are none.
    pub fn around(points: impl IntoIterator<Item = Point>) -> Option<Self> {
        points.into_iter().fold(None, |b, p| {
            Some(match b {
                None => Bounds { min: p, max: p },
                Some(b) => Bounds {
                    min: Point::new(b.min.x.min(p.x), b.min.y.min(p.y)),
                    max: Point::new(b.max.x.max(p.x), b.max.y.max(p.y)),
                },
            })
        })
    }

    /// The middle of the box.
    #[must_use]
    pub fn centre(&self) -> Point {
        (self.min + self.max) * 0.5
    }

    /// (across, up), in metres.
    #[must_use]
    pub fn size(&self) -> (f64, f64) {
        (self.max.x - self.min.x, self.max.y - self.min.y)
    }
}

/// What one cell says.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cell {
    /// Nobody knows (-1, or any number outside 0..=100).
    Unknown,
    /// How sure the map is that something is there: 0 free, 100 occupied.
    Occupancy(u8),
}

impl From<i8> for Cell {
    fn from(value: i8) -> Self {
        match u8::try_from(value) {
            Ok(p) if p <= 100 => Self::Occupancy(p),
            _ => Self::Unknown,
        }
    }
}

/// The colours a grid is drawn in, as RGBA.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Palette {
    /// A free cell.
    pub free: [u8; 4],
    /// An occupied cell.
    pub occupied: [u8; 4],
    /// An unknown cell.
    pub unknown: [u8; 4],
}

impl Palette {
    /// One cell's colour: free and occupied mix by the cell's occupancy.
    #[must_use]
    pub fn colour(&self, cell: Cell) -> [u8; 4] {
        match cell {
            Cell::Unknown => self.unknown,
            Cell::Occupancy(p) => {
                let k = u16::from(p);
                std::array::from_fn(|i| {
                    let (a, b) = (u16::from(self.free[i]), u16::from(self.occupied[i]));
                    // a + (b - a) * k / 100, in whole numbers; it stays within 0..=255.
                    u8::try_from((a * (100 - k) + b * k) / 100).unwrap_or(u8::MAX)
                })
            }
        }
    }
}

/// Why a grid can't be drawn.
#[derive(Debug, Error, PartialEq)]
pub enum GridError {
    /// The cell count doesn't match the size.
    #[error("the map has {got} cells, but {width} x {height} needs {expected}")]
    WrongSize {
        /// Cells across.
        width: u32,
        /// Cells down.
        height: u32,
        /// Cells the size needs.
        expected: usize,
        /// Cells the message has.
        got: usize,
    },
    /// The resolution (m) isn't positive.
    #[error("the map's resolution must be positive, not {0} m")]
    BadResolution(f64),
    /// The map has no cells.
    #[error("the map is empty (0 x 0 cells)")]
    Empty,
}

/// A whole occupancy grid.
#[derive(Debug, Clone, PartialEq)]
pub struct Grid {
    /// Its size, resolution and origin.
    pub info: GridInfo,
    /// The frame it's in: `map`, normally.
    pub frame: String,
    cells: Vec<i8>,
}

impl Grid {
    /// Checks the message: the cell count must match its size.
    ///
    /// # Errors
    ///
    /// [`GridError`] if the map is empty, its resolution isn't positive, or the cell count doesn't match.
    pub fn new(info: GridInfo, frame: String, cells: Vec<i8>) -> Result<Self, GridError> {
        if !(info.resolution.is_finite() && info.resolution > 0.0) {
            return Err(GridError::BadResolution(info.resolution));
        }
        if info.width == 0 || info.height == 0 {
            return Err(GridError::Empty);
        }
        let expected = info.width as usize * info.height as usize;
        if cells.len() != expected {
            return Err(GridError::WrongSize {
                width: info.width,
                height: info.height,
                expected,
                got: cells.len(),
            });
        }
        Ok(Self { info, frame, cells })
    }

    /// The cell at (column, row), counted from the origin corner.
    #[must_use]
    pub fn cell(&self, column: u32, row: u32) -> Option<Cell> {
        if column >= self.info.width || row >= self.info.height {
            return None;
        }
        let i = row as usize * self.info.width as usize + column as usize;
        self.cells.get(i).map(|&v| Cell::from(v))
    }

    /// The picture: width x height RGBA pixels, top row first, the grid's highest row
    /// (north, when the grid isn't turned) at the top.
    #[must_use]
    pub fn to_rgba(&self, palette: &Palette) -> Vec<u8> {
        let width = self.info.width as usize;
        self.cells
            .chunks_exact(width)
            .rev()
            .flatten()
            .flat_map(|&v| palette.colour(Cell::from(v)))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PALETTE: Palette = Palette {
        free: [200, 200, 200, 255],
        occupied: [0, 0, 0, 255],
        unknown: [100, 100, 100, 255],
    };

    fn info(width: u32, height: u32) -> GridInfo {
        GridInfo {
            width,
            height,
            resolution: 0.5,
            origin: Pose::new(-1.0, -2.0, 0.0),
        }
    }

    #[test]
    fn cells_read_as_free_occupied_or_unknown() {
        assert_eq!(Cell::from(-1), Cell::Unknown);
        assert_eq!(Cell::from(0), Cell::Occupancy(0));
        assert_eq!(Cell::from(100), Cell::Occupancy(100));
        assert_eq!(Cell::from(101), Cell::Unknown);
        assert_eq!(PALETTE.colour(Cell::Occupancy(0)), PALETTE.free);
        assert_eq!(PALETTE.colour(Cell::Occupancy(100)), PALETTE.occupied);
        assert_eq!(PALETTE.colour(Cell::Occupancy(50)), [100, 100, 100, 255]);
        assert_eq!(PALETTE.colour(Cell::Unknown), PALETTE.unknown);
    }

    #[test]
    fn the_picture_has_north_at_the_top() {
        // 2 x 2: bottom row (row 0) free, unknown; top row (row 1) occupied, free.
        let g = Grid::new(info(2, 2), "map".into(), vec![0, -1, 100, 0]).expect("valid");
        assert_eq!(g.cell(0, 1), Some(Cell::Occupancy(100)));
        assert_eq!(g.cell(2, 0), None);
        let px = g.to_rgba(&PALETTE);
        assert_eq!(px.len(), 2 * 2 * 4);
        let pixel = |i: usize| &px[i * 4..i * 4 + 4];
        assert_eq!(pixel(0), PALETTE.occupied); // top left = row 1, column 0
        assert_eq!(pixel(1), PALETTE.free);
        assert_eq!(pixel(2), PALETTE.free); // bottom left = row 0, column 0
        assert_eq!(pixel(3), PALETTE.unknown);
    }

    #[test]
    fn bad_grids_are_refused() {
        assert!(matches!(
            Grid::new(info(2, 2), "map".into(), vec![0; 3]),
            Err(GridError::WrongSize { expected: 4, .. })
        ));
        assert_eq!(
            Grid::new(info(0, 0), "map".into(), vec![]),
            Err(GridError::Empty)
        );
        let mut bad = info(1, 1);
        bad.resolution = 0.0;
        assert_eq!(
            Grid::new(bad, "map".into(), vec![0]),
            Err(GridError::BadResolution(0.0))
        );
    }

    #[test]
    fn corners_and_bounds() {
        let i = info(4, 2); // 2 m x 1 m, from (-1, -2)
        assert_eq!(i.size(), (2.0, 1.0));
        let b = i.bounds();
        assert_eq!(b.min, Point::new(-1.0, -2.0));
        assert_eq!(b.max, Point::new(1.0, -1.0));
        assert_eq!(b.centre(), Point::new(0.0, -1.5));
        assert_eq!(Bounds::around([]), None);
    }

    #[test]
    fn a_point_on_or_off_a_turned_grid() {
        // 10 m x 4 m, from (2, 0), turned 90 degrees: it covers x -2..2, y 0..10.
        let info = GridInfo {
            width: 10,
            height: 4,
            resolution: 1.0,
            origin: Pose::new(2.0, 0.0, std::f64::consts::FRAC_PI_2),
        };
        assert!(info.contains(Point::new(0.0, 5.0)));
        assert!(info.contains(Point::new(2.0, 0.0)));
        assert!(!info.contains(Point::new(3.0, 5.0)));
        let d = info.distance_outside(Point::new(0.0, 13.0)).0;
        assert!((d - 3.0).abs() < 1e-9, "{d}");
        let d = info.distance_outside(Point::new(5.0, -4.0)).0;
        assert!((d - 5.0).abs() < 1e-9, "{d}");
    }
}
