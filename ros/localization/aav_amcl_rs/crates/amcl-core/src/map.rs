//! The 2D map: which cells are free, occupied or unknown; how far each place is from the
//! nearest occupied cell (the likelihood field's table); and where a lost particle may go.

use std::fmt;

use crate::{
    pose::{Metres, Pose2},
    rng::Rng,
};

/// One cell of the map, as nav2_amcl reads an `OccupancyGrid`: 0 free, 100 occupied, any
/// other value (-1, or a grey in between) unknown.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cell {
    /// Nothing there: a particle may stand here.
    Free,
    /// A wall, a kerb, a car: a beam ends here.
    Occupied,
    /// Never seen, or not sure.
    Unknown,
}

impl Cell {
    /// The cell for an `OccupancyGrid` value.
    #[must_use]
    pub fn from_occupancy(value: i8) -> Self {
        match value {
            0 => Self::Free,
            100 => Self::Occupied,
            _ => Self::Unknown,
        }
    }
}

/// Why a map was refused.
#[derive(Clone, Debug, PartialEq)]
pub enum MapError {
    /// Width or height 0.
    Empty,
    /// The cell size is not a positive number.
    BadResolution(f64),
    /// `width * height` cells were expected, `got` came.
    SizeMismatch {
        /// width * height.
        expected: usize,
        /// The data's length.
        got: usize,
    },
}

impl fmt::Display for MapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(f, "the map has no cells"),
            Self::BadResolution(r) => write!(f, "the map's resolution {r} m is not above 0"),
            Self::SizeMismatch { expected, got } => {
                write!(f, "the map has {got} cells, its size says {expected}")
            }
        }
    }
}

impl std::error::Error for MapError {}

/// An occupancy grid: cells in rows from the origin, x along a row.
#[derive(Clone, Debug)]
pub struct OccupancyMap {
    width: usize,
    height: usize,
    resolution: f64,
    /// The grid's corner (cell (0, 0)'s outer corner) in the map frame.
    origin: Pose2,
    cells: Vec<Cell>,
}

impl OccupancyMap {
    /// A map of `width` x `height` cells of `resolution` metres, cell (0, 0)'s corner at
    /// `origin`, `cells` row by row.
    ///
    /// # Errors
    /// [`MapError`] when it is empty, its resolution is not above 0, or the cells don't
    /// fill it.
    pub fn new(
        width: usize,
        height: usize,
        resolution: Metres,
        origin: Pose2,
        cells: Vec<Cell>,
    ) -> Result<Self, MapError> {
        if width == 0 || height == 0 {
            return Err(MapError::Empty);
        }
        if !(resolution.0 > 0.0 && resolution.0.is_finite()) {
            return Err(MapError::BadResolution(resolution.0));
        }
        if cells.len() != width * height {
            return Err(MapError::SizeMismatch {
                expected: width * height,
                got: cells.len(),
            });
        }
        Ok(Self {
            width,
            height,
            resolution: resolution.0,
            origin,
            cells,
        })
    }

    /// A map from an `OccupancyGrid`'s values (see [`Cell::from_occupancy`]).
    ///
    /// # Errors
    /// As [`OccupancyMap::new`].
    pub fn from_occupancy(
        width: usize,
        height: usize,
        resolution: Metres,
        origin: Pose2,
        data: &[i8],
    ) -> Result<Self, MapError> {
        let cells = data.iter().map(|&v| Cell::from_occupancy(v)).collect();
        Self::new(width, height, resolution, origin, cells)
    }

    /// Cells along x.
    #[must_use]
    pub fn width(&self) -> usize {
        self.width
    }

    /// Cells along y.
    #[must_use]
    pub fn height(&self) -> usize {
        self.height
    }

    /// A cell's size, m.
    #[must_use]
    pub fn resolution(&self) -> Metres {
        Metres(self.resolution)
    }

    /// The cell at column `i`, row `j`.
    #[must_use]
    pub fn cell(&self, i: usize, j: usize) -> Option<Cell> {
        (i < self.width && j < self.height).then(|| self.cells[j * self.width + i])
    }

    /// The cell a map-frame point falls in, or None off the map.
    #[must_use]
    pub fn cell_of(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let (gx, gy) = self.origin.inverse().apply(x, y);
        let i = (gx / self.resolution).floor();
        let j = (gy / self.resolution).floor();
        let inside = i >= 0.0 && j >= 0.0 && i < self.width as f64 && j < self.height as f64;
        inside.then_some((i as usize, j as usize))
    }

    /// A cell's centre, in the map frame.
    #[must_use]
    pub fn centre(&self, i: usize, j: usize) -> (f64, f64) {
        self.origin.apply(
            (i as f64 + 0.5) * self.resolution,
            (j as f64 + 0.5) * self.resolution,
        )
    }
}

/// How far each cell is from the nearest occupied one, up to a cap: the likelihood field's
/// table (nav2_amcl's `map_update_cspace`). Exact Euclidean distances between cell
/// centres (Felzenszwalb and Huttenlocher's distance transform), not nav2's queue.
#[derive(Clone, Debug)]
pub struct DistanceField {
    width: usize,
    height: usize,
    resolution: f64,
    to_grid: Pose2,
    /// m, row by row; at most `max`.
    distance: Vec<f32>,
    max: f64,
}

impl DistanceField {
    /// The field of `map`, capped at `max` (laser_likelihood_max_dist).
    #[must_use]
    pub fn new(map: &OccupancyMap, max: Metres) -> Self {
        // Squared distances, in cells: 0 on an occupied cell, "far" elsewhere.
        const FAR: f64 = 1e20;
        let (w, h) = (map.width, map.height);
        let mut squared: Vec<f64> = map
            .cells
            .iter()
            .map(|&c| if c == Cell::Occupied { 0.0 } else { FAR })
            .collect();
        let mut line = Vec::with_capacity(w.max(h));
        let mut out = vec![0.0; w.max(h)];
        for i in 0..w {
            line.clear();
            line.extend((0..h).map(|j| squared[j * w + i]));
            transform_1d(&line, &mut out[..h]);
            for (j, d) in out[..h].iter().enumerate() {
                squared[j * w + i] = *d;
            }
        }
        for row in squared.chunks_mut(w) {
            line.clear();
            line.extend_from_slice(row);
            transform_1d(&line, &mut out[..w]);
            row.copy_from_slice(&out[..w]);
        }
        let max = max.0.max(0.0);
        let distance = squared
            .iter()
            .map(|d2| (d2.sqrt() * map.resolution).min(max) as f32)
            .collect();
        Self {
            width: w,
            height: h,
            resolution: map.resolution,
            to_grid: map.origin.inverse(),
            distance,
            max,
        }
    }

    /// The distance from the map-frame point (`x`, `y`) to the nearest occupied cell, m;
    /// the cap off the map.
    #[must_use]
    pub fn distance(&self, x: f64, y: f64) -> f64 {
        let (gx, gy) = self.to_grid.apply(x, y);
        let (i, j) = (
            (gx / self.resolution).floor(),
            (gy / self.resolution).floor(),
        );
        if i >= 0.0 && j >= 0.0 && i < self.width as f64 && j < self.height as f64 {
            f64::from(self.distance[j as usize * self.width + i as usize])
        } else {
            self.max
        }
    }

    /// The cap, m.
    #[must_use]
    pub fn max(&self) -> Metres {
        Metres(self.max)
    }
}

/// The 1D squared distance transform (Felzenszwalb and Huttenlocher 2012): for each `q`,
/// the least `f[p] + (q - p)²`.
fn transform_1d(f: &[f64], out: &mut [f64]) {
    let n = f.len();
    if n == 0 {
        return;
    }
    // The parabolas in the lower envelope (`v`) and where each starts to be lowest (`z`).
    let mut v = vec![0usize; n];
    let mut z = vec![0.0f64; n + 1];
    let mut k = 0usize;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;
    let meet = |q: usize, p: usize| -> f64 {
        let (qf, pf) = (q as f64, p as f64);
        ((f[q] + qf * qf) - (f[p] + pf * pf)) / (2.0 * qf - 2.0 * pf)
    };
    for q in 1..n {
        let mut s = meet(q, v[k]);
        while s <= z[k] {
            // The newest parabola hides the one before it everywhere it matters.
            k -= 1;
            s = meet(q, v[k]);
        }
        k += 1;
        v[k] = q;
        z[k] = s;
        z[k + 1] = f64::INFINITY;
    }
    k = 0;
    for (q, d) in out.iter_mut().enumerate() {
        while z[k + 1] < q as f64 {
            k += 1;
        }
        let dq = q as f64 - v[k] as f64;
        *d = dq * dq + f[v[k]];
    }
}

/// The free cells, where a random particle may be put (nav2_amcl's `uniformPoseGenerator`).
#[derive(Clone, Debug)]
pub struct FreeSpace {
    /// Cell corners in the grid's frame, m.
    corners: Vec<(f64, f64)>,
    resolution: f64,
    origin: Pose2,
}

impl FreeSpace {
    /// The free cells of `map` (none: [`FreeSpace::sample`] gives None).
    #[must_use]
    pub fn new(map: &OccupancyMap) -> Self {
        let corners = map
            .cells
            .iter()
            .enumerate()
            .filter(|(_, c)| **c == Cell::Free)
            .map(|(k, _)| {
                let (i, j) = (k % map.width, k / map.width);
                (i as f64 * map.resolution, j as f64 * map.resolution)
            })
            .collect();
        Self {
            corners,
            resolution: map.resolution,
            origin: map.origin,
        }
    }

    /// How many free cells there are.
    #[must_use]
    pub fn len(&self) -> usize {
        self.corners.len()
    }

    /// No free cells.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.corners.is_empty()
    }

    /// A random pose: somewhere in a random free cell, facing any way.
    pub fn sample(&self, rng: &mut Rng) -> Option<Pose2> {
        if self.corners.is_empty() {
            return None;
        }
        let (cx, cy) = self.corners[rng.below(self.corners.len())];
        let (x, y) = self.origin.apply(
            cx + rng.uniform() * self.resolution,
            cy + rng.uniform() * self.resolution,
        );
        let yaw = crate::pose::wrap(std::f64::consts::TAU * rng.uniform());
        Some(Pose2::new(x, y, yaw))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn walled(n: usize) -> OccupancyMap {
        let cells = (0..n * n)
            .map(|k| {
                let (i, j) = (k % n, k / n);
                if i == 0 || j == 0 || i == n - 1 || j == n - 1 {
                    Cell::Occupied
                } else {
                    Cell::Free
                }
            })
            .collect();
        OccupancyMap::new(n, n, Metres(0.1), Pose2::new(-1.0, -1.0, 0.0), cells)
            .expect("a good map")
    }

    #[test]
    fn refuses_bad_maps() {
        let origin = Pose2::default();
        assert_eq!(
            OccupancyMap::new(0, 3, Metres(0.1), origin, vec![]).unwrap_err(),
            MapError::Empty
        );
        assert!(matches!(
            OccupancyMap::new(2, 2, Metres(0.0), origin, vec![Cell::Free; 4]),
            Err(MapError::BadResolution(_))
        ));
        assert!(matches!(
            OccupancyMap::new(2, 2, Metres(0.1), origin, vec![Cell::Free; 3]),
            Err(MapError::SizeMismatch {
                expected: 4,
                got: 3
            })
        ));
    }

    #[test]
    fn reads_occupancy_values_as_nav2_does() {
        assert_eq!(Cell::from_occupancy(0), Cell::Free);
        assert_eq!(Cell::from_occupancy(100), Cell::Occupied);
        assert_eq!(Cell::from_occupancy(-1), Cell::Unknown);
        assert_eq!(Cell::from_occupancy(50), Cell::Unknown);
    }

    #[test]
    fn distances_are_euclidean_and_capped() {
        let map = walled(21); // walls at -0.95 and +0.95 m (cell centres)
        let field = DistanceField::new(&map, Metres(0.5));
        // On a wall: 0. One cell in: 0.1. The middle: 0.9 from each wall, capped at 0.5.
        let (wx, wy) = map.centre(0, 10);
        assert!(field.distance(wx, wy).abs() < 1e-6);
        let (x1, y1) = map.centre(1, 10);
        assert!((field.distance(x1, y1) - 0.1).abs() < 1e-6);
        let (x3, y3) = map.centre(3, 3); // 3 cells from two walls: 0.3 m
        assert!((field.distance(x3, y3) - 0.3).abs() < 1e-6);
        assert!((field.distance(0.05, 0.05) - 0.5).abs() < 1e-6);
        assert!(
            (field.distance(50.0, 0.0) - 0.5).abs() < 1e-6,
            "off the map: the cap"
        );
    }

    #[test]
    fn a_diagonal_distance_is_the_hypotenuse() {
        let mut cells = vec![Cell::Free; 100];
        cells[0] = Cell::Occupied;
        let map = OccupancyMap::new(10, 10, Metres(1.0), Pose2::default(), cells).expect("map");
        let field = DistanceField::new(&map, Metres(100.0));
        let (x, y) = map.centre(3, 4);
        assert!((field.distance(x, y) - 5.0).abs() < 1e-6);
    }

    #[test]
    fn a_turned_origin_moves_the_cells() {
        let cells = vec![Cell::Free; 4];
        let map = OccupancyMap::new(
            2,
            2,
            Metres(1.0),
            Pose2::new(10.0, 0.0, std::f64::consts::FRAC_PI_2),
            cells,
        )
        .expect("map");
        // Cell (1, 0) is 1.5 m along the grid's x, which points up the map's y.
        let (x, y) = map.centre(1, 0);
        assert!((x - 9.5).abs() < 1e-9 && (y - 1.5).abs() < 1e-9);
        assert_eq!(map.cell_of(x, y), Some((1, 0)));
    }

    #[test]
    fn random_poses_land_on_free_cells() {
        let map = walled(21);
        let free = FreeSpace::new(&map);
        assert_eq!(free.len(), 19 * 19);
        let mut rng = Rng::new(9);
        for _ in 0..1000 {
            let p = free.sample(&mut rng).expect("free cells");
            let (i, j) = map.cell_of(p.x, p.y).expect("on the map");
            assert_eq!(map.cell(i, j), Some(Cell::Free));
        }
        let none = OccupancyMap::new(1, 1, Metres(1.0), Pose2::default(), vec![Cell::Unknown])
            .expect("map");
        assert!(FreeSpace::new(&none).sample(&mut rng).is_none());
    }
}
