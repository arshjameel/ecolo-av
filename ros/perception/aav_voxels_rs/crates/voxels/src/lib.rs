//! The roof lidar's cloud as coarse cubes: what stands in the car's way. No ROS here.
//!
//! A port of `aav_voxels`' `cubes.py` (Ben, 30 Oct: "reducing the navigation area down to
//! cubes. Less precise but this is a car not a needle"). Given one cloud's points in
//! `base_link` ([`Mount::to_base`]), [`cubes`] returns:
//!
//! | Out | What |
//! |---|---|
//! | obstacle cubes | one point for every cube ([`CubeShape::size`], 0.25 m) holding a point between `min_height` and `max_height` above the road: of its points, the nearest to `base_link`, so the stop zone measures the real face |
//! | ground squares | the centre of every flat square where the lidar saw the road: free space, which the costmap clears its old marks with |
//!
//! The road's height comes from the cloud itself ([`fit_ground`]): a plane through the
//! lowest points, so a car leaning in a turn doesn't see the road rising as an obstacle.
//! Points on the car itself ([`Body`]) never count. The same answers as the Python, checked
//! on its recorded clouds (`tests/python_vectors.rs`). The design:
//! `docs/designs/2026-10-30_voxels.md`.
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod cloud;
pub mod memory;
mod mount;
pub mod patchwork;
pub mod steps;

use std::{cmp::Ordering, error::Error, fmt};

pub use memory::{Blind, Pose2, StepMemory, LOWEST_BEAM};
pub use mount::Mount;
pub use steps::{kerbs_only, steps, StepError, StepShape};

/// A point in `base_link` (m), or in the lidar's frame before [`Mount::to_base`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point {
    /// m, + forward.
    pub x: f64,
    /// m, + left.
    pub y: f64,
    /// m, + up.
    pub z: f64,
}

impl Point {
    /// The point (`x`, `y`, `z`).
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// The distance from `base_link`, flat (m).
    #[must_use]
    pub fn flat(&self) -> f64 {
        self.x.hypot(self.y)
    }
}

/// The car itself in `base_link`, from above (m): points inside are its own roof, frame
/// and cameras, at any height. `aav_lidar`'s `scan.body_box` works it out from
/// `vehicle.yaml`; the launch file passes it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Body {
    /// x from (behind the rear bumper).
    pub back: f64,
    /// x to (ahead of the front bumper).
    pub front: f64,
    /// Half the width, margins included.
    pub half_width: f64,
}

impl Body {
    /// Whether `p` is on the car.
    #[must_use]
    pub fn contains(&self, p: &Point) -> bool {
        p.x >= self.back && p.x <= self.front && p.y.abs() <= self.half_width
    }
}

/// Which points count, and how big a cube is. Metres and radians, in `base_link`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CubeShape {
    /// One cube's edge.
    pub size: f64,
    /// Above the road: lower is the road (`vehicle.obstacle_heights`).
    pub min_height: f64,
    /// Above the road: higher is what the car passes under.
    pub max_height: f64,
    /// Flat from `base_link`: further is too sparse to matter.
    pub range_max: f64,
    /// The car itself.
    pub body: Body,
    /// The lowest point of each such square is a road candidate.
    pub ground_cell: f64,
    /// Flat from `base_link`: candidates from here...
    pub ground_near: f64,
    /// ...to here.
    pub ground_far: f64,
    /// Fewer candidates that agree: keep the road flat.
    pub ground_points: usize,
    /// A body leans about 3 degrees at most on its springs: a steeper fit is a wrong one.
    pub max_tilt: f64,
    /// The road's height under `base_link`, at most.
    pub max_offset: f64,
    /// The root-mean-square of the road's points off the plane, at most.
    pub max_spread: f64,
    /// Either side of the plane: the road, for clearing.
    pub ground_band: f64,
}

impl Default for CubeShape {
    /// `cubes.py`'s numbers.
    fn default() -> Self {
        Self {
            size: 0.25,
            min_height: 0.3,
            max_height: 2.0,
            range_max: 30.0,
            body: Body::default(),
            ground_cell: 2.0,
            ground_near: 2.0,
            ground_far: 20.0,
            ground_points: 20,
            max_tilt: 4.0_f64.to_radians(),
            max_offset: 0.2,
            max_spread: 0.05,
            ground_band: 0.15,
        }
    }
}

/// Why a [`CubeShape`] can't cut a cloud.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ShapeError {
    /// The cube's edge isn't a length above 0.
    Size(f64),
    /// min_height isn't below max_height.
    Heights {
        /// min_height (m).
        min: f64,
        /// max_height (m).
        max: f64,
    },
    /// range_max isn't a distance above 0.
    Range(f64),
}

impl fmt::Display for ShapeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Size(s) => write!(f, "size {s} m: a cube needs an edge above 0"),
            Self::Heights { min, max } => {
                write!(f, "min_height {min} m must be below max_height {max} m")
            }
            Self::Range(r) => write!(f, "range_max {r} m must be above 0"),
        }
    }
}

impl Error for ShapeError {}

impl CubeShape {
    /// The shape, if it can cut a cloud.
    ///
    /// # Errors
    ///
    /// [`ShapeError::Size`] or [`ShapeError::Heights`].
    pub fn checked(self) -> Result<Self, ShapeError> {
        if !(self.size.is_finite() && self.size > 0.0) {
            return Err(ShapeError::Size(self.size));
        }
        if self.min_height.partial_cmp(&self.max_height) != Some(Ordering::Less) {
            return Err(ShapeError::Heights {
                min: self.min_height,
                max: self.max_height,
            });
        }
        if self.range_max.partial_cmp(&0.0) != Some(Ordering::Greater) {
            return Err(ShapeError::Range(self.range_max));
        }
        Ok(self)
    }
}

/// A plane z = a x + b y + c in `base_link` (m).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Plane {
    /// The slope along x.
    pub a: f64,
    /// The slope along y.
    pub b: f64,
    /// The height at the origin (m).
    pub c: f64,
}

impl Plane {
    /// The plane's height under (x, y).
    #[must_use]
    pub fn height(&self, x: f64, y: f64) -> f64 {
        // As cubes.py: (a x + b y) + c, so the two agree to the last bit.
        self.a * x + self.b * y + self.c
    }
}

/// The road under the car, as one cloud measured it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Road {
    /// A plane the checks believed, through this many candidates.
    Fitted {
        /// The plane.
        plane: Plane,
        /// The road points it was fitted through.
        candidates: usize,
    },
    /// No fit believed (too few candidates, too steep, too far off, too spread): the road
    /// is taken as flat, z = 0, as `lidar_scan` takes it.
    Flat {
        /// The road points there were.
        candidates: usize,
    },
}

impl Default for Road {
    fn default() -> Self {
        Self::Flat { candidates: 0 }
    }
}

impl Road {
    /// The road as a plane: z = 0 when flat.
    #[must_use]
    pub fn plane(&self) -> Plane {
        match self {
            Self::Fitted { plane, .. } => *plane,
            Self::Flat { .. } => Plane::default(),
        }
    }

    /// A plane was believed.
    #[must_use]
    pub fn is_fitted(&self) -> bool {
        matches!(self, Self::Fitted { .. })
    }

    /// The candidates the last fit used (or found, if too few).
    #[must_use]
    pub fn candidates(&self) -> usize {
        match self {
            Self::Fitted { candidates, .. } | Self::Flat { candidates } => *candidates,
        }
    }

    /// The road's height in `base_link` under (x, y).
    #[must_use]
    pub fn height(&self, x: f64, y: f64) -> f64 {
        self.plane().height(x, y)
    }

    /// The body's lean on the road, side to side (rad, REP-103: + = right side down).
    /// Leaning right, the road on the right looks higher: it rises as y falls.
    #[must_use]
    pub fn roll(&self) -> f64 {
        -self.plane().b.atan()
    }

    /// The body's lean on the road, front to back (rad, REP-103: + = nose down).
    #[must_use]
    pub fn pitch(&self) -> f64 {
        self.plane().a.atan()
    }
}

/// One cloud's cubes, in `base_link`, and the road they were measured from.
#[derive(Clone, Debug, PartialEq)]
pub struct Cubes {
    /// Each obstacle cube's point nearest to `base_link`.
    pub obstacles: Vec<Point>,
    /// The centre of each flat square of road seen, at the road's height.
    pub ground: Vec<Point>,
    /// The road's plane, or flat.
    pub road: Road,
    /// Finite points, off the car, within `range_max`.
    pub points: usize,
}

/// Of the points sharing each key, the one with the least `by`; sorted by key. Ties go
/// to the first in `points`' order, as `cubes.py`'s stable sort gives them.
fn least_per_key<K: Ord + Copy>(
    points: &[Point],
    key: impl Fn(&Point) -> K,
    by: impl Fn(&Point) -> f64,
) -> Vec<Point> {
    let mut order: Vec<(K, f64, usize)> = points
        .iter()
        .enumerate()
        .map(|(i, p)| (key(p), by(p), i))
        .collect();
    // `by` is finite (the caller dropped the rest); -0.0 and 0.0 tie, as in numpy.
    order.sort_unstable_by(|a, b| {
        a.0.cmp(&b.0)
            .then(a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
            .then(a.2.cmp(&b.2))
    });
    order.dedup_by_key(|entry| entry.0);
    order.into_iter().map(|(_, _, i)| points[i]).collect()
}

/// Which `size` square (x, y) holds `p`: floored, not truncated, below zero.
fn square_of(p: &Point, size: f64) -> [i64; 2] {
    [(p.x / size).floor() as i64, (p.y / size).floor() as i64]
}

/// Which `size` cube holds `p`.
fn cube_of(p: &Point, size: f64) -> [i64; 3] {
    let [x, y] = square_of(p, size);
    [x, y, (p.z / size).floor() as i64]
}

/// The lowest of the points in each `cell` x `cell` square, one per square.
#[must_use]
pub fn lowest_per_cell(points: &[Point], cell: f64) -> Vec<Point> {
    least_per_key(points, |p| square_of(p, cell), |p| p.z)
}

/// Each `size` cube's point nearest to `base_link` (flat).
pub fn nearest_per_cube(points: &[Point], size: f64) -> Vec<Point> {
    least_per_key(points, |p| cube_of(p, size), Point::flat)
}

/// The centre of every `size` cube holding one of the points, each once, sorted.
#[must_use]
pub fn cube_centres(points: &[Point], size: f64) -> Vec<Point> {
    let mut keys: Vec<[i64; 3]> = points.iter().map(|p| cube_of(p, size)).collect();
    keys.sort_unstable();
    keys.dedup();
    let centre = |k: i64| (k as f64 + 0.5) * size;
    keys.into_iter()
        .map(|[x, y, z]| Point::new(centre(x), centre(y), centre(z)))
        .collect()
}

/// The centre of every flat `size` square holding one of the points, each once, sorted;
/// z = 0, for the caller to set.
#[must_use]
pub fn square_centres(points: &[Point], size: f64) -> Vec<Point> {
    let mut keys: Vec<[i64; 2]> = points.iter().map(|p| square_of(p, size)).collect();
    keys.sort_unstable();
    keys.dedup();
    let centre = |k: i64| (k as f64 + 0.5) * size;
    keys.into_iter()
        .map(|[x, y]| Point::new(centre(x), centre(y), 0.0))
        .collect()
}

/// The `q` percentile (0 to 100) of the values, as `numpy.percentile` gives it (linear
/// interpolation between the two nearest ranks); `None` if there are none. Sorts them.
pub fn percentile(values: &mut [f64], q: f64) -> Option<f64> {
    let last = values.len().checked_sub(1)?;
    values.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
    let index = last as f64 * (q / 100.0);
    let below = index.floor();
    let gamma = index - below;
    let low = values[(below as usize).min(last)];
    let high = values[(below as usize + 1).min(last)];
    // numpy's _lerp: from the nearer end, so the answer stays between the two.
    let span = high - low;
    Some(if gamma >= 0.5 {
        high - span * (1.0 - gamma)
    } else {
        low + span * gamma
    })
}

/// The least-squares plane z = a x + b y + c through the points: the normal equations,
/// about the points' mean (better conditioned). `None` if they lie on one line.
pub fn fit_plane<'a>(points: impl Iterator<Item = &'a Point> + Clone) -> Option<Plane> {
    let (n, sx, sy, sz) = points
        .clone()
        .fold((0.0, 0.0, 0.0, 0.0), |(n, x, y, z), p| {
            (n + 1.0, x + p.x, y + p.y, z + p.z)
        });
    if n < 3.0 {
        return None;
    }
    let (mx, my, mz) = (sx / n, sy / n, sz / n);
    let [xx, xy, yy, xz, yz] = points.fold([0.0; 5], |[xx, xy, yy, xz, yz], p| {
        let (dx, dy, dz) = (p.x - mx, p.y - my, p.z - mz);
        [
            xx + dx * dx,
            xy + dx * dy,
            yy + dy * dy,
            xz + dx * dz,
            yz + dy * dz,
        ]
    });
    let det = xx * yy - xy * xy;
    // On one line (or nearly): no plane. NaN sums count as that too.
    let floor = 1e-12 * (xx * yy).max(f64::MIN_POSITIVE);
    if det.abs().partial_cmp(&floor) != Some(Ordering::Greater) {
        return None;
    }
    let a = (xz * yy - yz * xy) / det;
    let b = (yz * xx - xz * xy) / det;
    Some(Plane {
        a,
        b,
        c: mz - a * mx - b * my,
    })
}

/// The road under the car, fitted to finite points in `base_link`.
///
/// The lowest point of each `ground_cell` square between `ground_near` and `ground_far`
/// is a candidate: the road, or the bottom of a wall where the lidar saw no road. A plane
/// through the lower three quarters of them, then again through those within 0.3 m of it,
/// then within 0.1 m: the walls drop out. A fit with too few candidates, steeper than
/// `max_tilt`, further than `max_offset` from `base_link`, or whose road points spread more
/// than `max_spread` off it (a crest, a crowned road: not one plane), is not believed.
#[must_use]
pub fn fit_ground(points: &[Point], shape: &CubeShape) -> Road {
    let near: Vec<Point> = points
        .iter()
        .filter(|p| (shape.ground_near..=shape.ground_far).contains(&p.flat()))
        .copied()
        .collect();
    let candidates = lowest_per_cell(&near, shape.ground_cell);
    let too_few = Road::Flat {
        candidates: candidates.len(),
    };
    if candidates.len() < shape.ground_points {
        return too_few;
    }
    let mut heights: Vec<f64> = candidates.iter().map(|p| p.z).collect();
    let Some(lower) = percentile(&mut heights, 75.0) else {
        return too_few;
    };
    // The first plane through the lowest candidates: the road lies under everything else.
    let mut keep: Vec<bool> = candidates.iter().map(|p| p.z <= lower).collect();
    let mut fit = (Plane::default(), 0);
    for tolerance in [Some(0.3), Some(0.1), None] {
        let n = keep.iter().filter(|k| **k).count();
        let kept = candidates
            .iter()
            .zip(&keep)
            .filter(|(_, k)| **k)
            .map(|(p, _)| p);
        let Some(plane) = fit_plane(kept).filter(|_| n >= shape.ground_points) else {
            return Road::Flat { candidates: n };
        };
        if let Some(tolerance) = tolerance {
            keep = candidates
                .iter()
                .map(|p| (p.z - plane.height(p.x, p.y)).abs() <= tolerance)
                .collect();
        }
        fit = (plane, n);
    }
    let (plane, n) = fit;
    let tilt = plane.a.hypot(plane.b).atan();
    let (squares, inliers) = candidates.iter().zip(&keep).filter(|(_, k)| **k).fold(
        (0.0, 0.0),
        |(sum, count), (p, _)| {
            let off = p.z - plane.height(p.x, p.y);
            (sum + off * off, count + 1.0)
        },
    );
    let spread = (squares / inliers).sqrt();
    if tilt > shape.max_tilt || plane.c.abs() > shape.max_offset || spread > shape.max_spread {
        return Road::Flat { candidates: n };
    }
    Road::Fitted {
        plane,
        candidates: n,
    }
}

/// The points that count: finite, within `range_max`, off the car.
#[must_use]
pub fn usable(points: &[Point], shape: &CubeShape) -> Vec<Point> {
    points
        .iter()
        .filter(|p| {
            let flat = p.flat();
            flat.is_finite()
                && p.z.is_finite()
                && flat <= shape.range_max
                && !shape.body.contains(p)
        })
        .copied()
        .collect()
}

/// One cloud's obstacle cubes and ground squares from its points in `base_link`, the
/// road fitted to them.
///
/// Points that are NaN or inf (rays that hit nothing), on the car itself or further than
/// `range_max` don't count.
#[must_use]
pub fn cubes(points: &[Point], shape: &CubeShape) -> Cubes {
    let good = usable(points, shape);
    let road = fit_ground(&good, shape);
    cut(good, shape, road)
}

/// As [`cubes`], measured from this `road` instead of a fit: `Road::default()` is the
/// flat cut, as `lidar_scan`'s.
#[must_use]
pub fn cubes_on(points: &[Point], shape: &CubeShape, road: Road) -> Cubes {
    cut(usable(points, shape), shape, road)
}

fn cut(good: Vec<Point>, shape: &CubeShape, road: Road) -> Cubes {
    let plane = road.plane();
    let above = |p: &Point| p.z - plane.height(p.x, p.y);
    let high: Vec<Point> = good
        .iter()
        .filter(|p| (shape.min_height..=shape.max_height).contains(&above(p)))
        .copied()
        .collect();
    let on_road: Vec<Point> = good
        .iter()
        .filter(|p| above(p).abs() <= shape.ground_band)
        .copied()
        .collect();
    let ground = square_centres(&on_road, shape.size)
        .into_iter()
        .map(|c| Point {
            z: plane.height(c.x, c.y),
            ..c
        })
        .collect();
    Cubes {
        obstacles: nearest_per_cube(&high, shape.size),
        ground,
        road,
        points: good.len(),
    }
}

#[cfg(test)]
mod tests;
