//! The roof lidar's cloud, flattened into a 2D scan around `base_link`. No ROS here.
//!
//! A port of `aav_lidar`'s `scan.py` (and `ground.py`, the body-roll sums). The node hands
//! [`flatten`] the cloud's points in `base_link` ([`voxels::Mount::to_base`]); it returns,
//! for each direction round the car, the distance to the nearest point between two heights
//! above the road: what the car would hit.
//!
//! ```text
//!   max_height ─────────────  above: what the car passes under (a branch)
//!        ■  counts            between: an obstacle, its flat distance from base_link
//!   min_height ─────────────  below: the road
//! ```
//!
//! | Module | Holds | From |
//! |---|---|---|
//! | this one | [`ScanShape`], [`body_box`], [`flatten`] | `scan.py` |
//! | [`ground`] | [`ground::ground_hit`], [`ground::worst_ground_height`], [`ground::max_safe_roll`] | `ground.py` |
//!
//! The same answers as the Python, checked on recorded clouds (`tests/vectors.rs`).
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod ground;

use std::{error::Error, f64::consts::PI, fmt};

pub use voxels::{cloud, Body, Mount, Point};

/// m behind and beside the body's outline that is still the car: the rear camera's
/// housing, the mirrors.
pub const SELF_MARGIN: f64 = 0.3;
/// m ahead of the front bumper, only: the front cameras' housings at the roof's edge.
pub const FRONT_MARGIN: f64 = 0.1;

/// Which points count, and the scan's directions. Metres and radians, in `base_link`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScanShape {
    /// m above the road: lower is the road.
    pub min_height: f64,
    /// m above the road: higher is what the car passes under.
    pub max_height: f64,
    /// rad: the first direction.
    pub angle_min: f64,
    /// rad: the last direction (with `angle_min`, sets how many there are).
    pub angle_max: f64,
    /// rad between two directions.
    pub angle_increment: f64,
    /// m from `base_link`: nearer doesn't count.
    pub range_min: f64,
    /// m from `base_link`: farther doesn't count.
    pub range_max: f64,
    /// The car itself: its points never count.
    pub body: Body,
}

impl Default for ScanShape {
    /// `scan.py`'s defaults: 720 directions all the way round, 0.3 to 2.0 m up.
    fn default() -> Self {
        Self {
            min_height: 0.3,
            max_height: 2.0,
            angle_min: -PI,
            angle_max: PI,
            angle_increment: 0.5_f64.to_radians(),
            range_min: 0.2,
            range_max: 30.0,
            body: Body::default(),
        }
    }
}

/// A [`ScanShape`] that makes no scan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ShapeError {
    /// The step between directions is not above 0.
    Increment(f64),
    /// No direction between `angle_min` and `angle_max`.
    NoDirections,
    /// A number is NaN or infinite.
    NotFinite,
}

impl fmt::Display for ShapeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Increment(v) => write!(f, "angle_increment {v}: needs to be above 0"),
            Self::NoDirections => write!(f, "angle_max - angle_min holds no direction"),
            Self::NotFinite => write!(f, "a scan setting is NaN or infinite"),
        }
    }
}

impl Error for ShapeError {}

impl ScanShape {
    /// How many directions the scan has (Python's `round`: halves to even).
    #[must_use]
    pub fn count(&self) -> usize {
        let n = ((self.angle_max - self.angle_min) / self.angle_increment).round_ties_even();
        if n.is_finite() && n > 0.0 {
            n as usize
        } else {
            0
        }
    }

    /// The shape, if it makes a scan.
    ///
    /// # Errors
    ///
    /// [`ShapeError`]: a step of 0 or less, no direction, or a NaN.
    pub fn checked(self) -> Result<Self, ShapeError> {
        let numbers = [
            self.min_height,
            self.max_height,
            self.angle_min,
            self.angle_max,
            self.angle_increment,
            self.range_min,
            self.range_max,
        ];
        if !numbers.iter().all(|v| v.is_finite()) {
            return Err(ShapeError::NotFinite);
        }
        if self.angle_increment <= 0.0 {
            return Err(ShapeError::Increment(self.angle_increment));
        }
        if self.count() == 0 {
            return Err(ShapeError::NoDirections);
        }
        Ok(self)
    }

    /// Whether the directions go all the way round (-π and π are then one).
    fn full_circle(&self) -> bool {
        self.angle_max - self.angle_min >= 2.0 * PI - 1e-6
    }
}

/// The car itself from its outline's corners (x, y in `base_link`, m), plus the margins:
/// points inside it are its own roof, frame and cameras, at any height.
#[must_use]
pub fn body_box(corners: &[[f64; 2]]) -> Body {
    let fold = |init: f64, pick: fn(f64, f64) -> f64, of: fn(&[f64; 2]) -> f64| {
        corners.iter().map(of).fold(init, pick)
    };
    Body {
        back: fold(f64::INFINITY, f64::min, |p| p[0]) - SELF_MARGIN,
        front: fold(f64::NEG_INFINITY, f64::max, |p| p[0]) + FRONT_MARGIN,
        half_width: fold(f64::NEG_INFINITY, f64::max, |p| p[1].abs()) + SELF_MARGIN,
    }
}

/// The scan's ranges (m) from points in `base_link`: `inf` where nothing is.
///
/// Each range is the nearest point in its direction between the two heights, measured
/// flat from `base_link`. Points that are NaN or infinite (rays that hit nothing) don't
/// count, nor do points on the car itself. Each point goes to its nearest direction
/// (range `i` lies at `angle_min + i * increment`); all the way round, the last half step
/// is the first direction again.
#[must_use]
pub fn flatten(points: &[Point], shape: &ScanShape) -> Vec<f64> {
    let count = shape.count();
    let mut ranges = vec![f64::INFINITY; count];
    let full = shape.full_circle();
    for p in points {
        let flat = p.flat();
        let keep = flat.is_finite()
            && p.z.is_finite()
            && p.z >= shape.min_height
            && p.z <= shape.max_height
            && flat >= shape.range_min
            && flat <= shape.range_max
            && !shape.body.contains(p);
        if !keep {
            continue;
        }
        let step = ((p.y.atan2(p.x) - shape.angle_min) / shape.angle_increment).round_ties_even();
        // As numpy's astype(int) and %: a whole number, wrapped to 0..count all round.
        let mut index = step as i64;
        if full {
            index = index.rem_euclid(count as i64);
        }
        if let Some(r) = usize::try_from(index).ok().and_then(|i| ranges.get_mut(i)) {
            if flat < *r {
                *r = flat;
            }
        }
    }
    ranges
}

#[cfg(test)]
mod tests;
