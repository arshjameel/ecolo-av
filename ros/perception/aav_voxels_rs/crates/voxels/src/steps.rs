//! Kerbs: sudden steps in the ground, which the cubes' 0.3 m floor misses (TODO 119).
//!
//! A point is on a step when a point within [`StepShape::radius`] of it, flat, lies more
//! than [`StepShape::min_step`] lower (and not more than [`StepShape::max_step`]: what
//! hangs over the road, a branch or a sign, isn't a step). Only the high side is kept: the
//! kerb's top or face, where the costmap must mark.
//!
//! ```text
//!   ring k+1 ─────────╮                         the lidar, 1.52 m up
//!   ring k   ──────╮  │
//!                  │  ●  kerb top, h            ● and ○ within 0.3 m, flat,
//!   road ──────────○──┘                         ● more than 0.10 m higher: a step
//! ```
//!
//! Why two points and not one ring: every point of one ring lies on the same cone, so its
//! height is fixed by its range (z = H - r tan e). Along one ring a kerb looks like a ramp
//! as steep as the beam, never a step. A step shows only between rings: a lower ring on
//! the road just before the kerb and the next ring on its face or top, within 0.3 m. Each
//! ring pair sees it in its own band of distances ([`StepShape`]'s docs: the numbers).
//!
//! What it must not see, tested on clouds cast like the roof lidar's (`steps_tests.rs`):
//! slopes and ramps (10 degrees rises 0.05 m in 0.3 m), the road's camber, a 7.5 cm speed
//! bump, grass beside the road. A step is any rise steeper than about 18 degrees over 0.3 m
//! and higher than 0.10 m: a kerb, a high speed bump, a wall's foot, a drop off the road's
//! edge (its high side is the road).
//!
//! Lone points don't count: a step needs [`StepShape::min_points`] in one
//! [`StepShape::cell`] square, so one noisy return never marks the costmap.

use std::{collections::HashMap, error::Error, fmt};

use crate::Point;

/// How a step is found. Metres, in `base_link`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepShape {
    /// Flat, the farthest apart the two sides of a step may be (Ben, 16 Nov: "> 0.08 m over
    /// < 0.3 m horizontally").
    pub radius: f64,
    /// The rise that is a step: more than this. 0.10 m: a 7.5 cm speed bump and grass pass,
    /// a 0.15 m kerb doesn't.
    pub min_step: f64,
    /// The most a step rises: higher is something hanging over the road.
    pub max_step: f64,
    /// Flat from `base_link`: further, the rings are too far apart to find a step.
    pub range_max: f64,
    /// The square the step's points are counted in, and each published once.
    pub cell: f64,
    /// Fewer step points in a square: noise.
    pub min_points: usize,
}

impl Default for StepShape {
    fn default() -> Self {
        Self {
            radius: 0.3,
            min_step: 0.10,
            max_step: 0.5,
            range_max: 12.0,
            cell: 0.1,
            min_points: 2,
        }
    }
}

/// Why a [`StepShape`] can't find steps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StepError {
    /// radius, cell or range_max isn't a length above 0.
    Length(&'static str, f64),
    /// min_step isn't between 0 and max_step.
    Heights {
        /// min_step (m).
        min: f64,
        /// max_step (m).
        max: f64,
    },
}

impl fmt::Display for StepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Length(name, v) => write!(f, "{name} {v} m: needs a length above 0"),
            Self::Heights { min, max } => {
                write!(
                    f,
                    "step_min {min} m must be above 0 and below max_step {max} m"
                )
            }
        }
    }
}

impl Error for StepError {}

impl StepShape {
    /// The shape, if it can find steps.
    ///
    /// # Errors
    ///
    /// [`StepError::Length`] or [`StepError::Heights`].
    pub fn checked(self) -> Result<Self, StepError> {
        for (name, v) in [
            ("step_radius", self.radius),
            ("step cell", self.cell),
            ("step_range", self.range_max),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return Err(StepError::Length(name, v));
            }
        }
        if !(self.min_step > 0.0 && self.min_step < self.max_step) {
            return Err(StepError::Heights {
                min: self.min_step,
                max: self.max_step,
            });
        }
        Ok(self)
    }
}

fn square(p: &Point, size: f64) -> [i64; 2] {
    [(p.x / size).floor() as i64, (p.y / size).floor() as i64]
}

/// The points on a step's high side, in `base_link`: of each [`StepShape::cell`] square
/// holding at least [`StepShape::min_points`] of them, the one nearest `base_link`, sorted
/// by square. `points`: finite, off the car (as [`crate::usable`] gives them).
#[must_use]
pub fn steps(points: &[Point], shape: &StepShape) -> Vec<Point> {
    let near: Vec<&Point> = points
        .iter()
        .filter(|p| p.flat() <= shape.range_max && p.z.is_finite())
        .collect();
    // Squares as wide as the radius: a point's neighbours are in its square and the 8 round it.
    let mut lowest: HashMap<[i64; 2], f64> = HashMap::new();
    for p in &near {
        let key = square(p, shape.radius);
        let low = lowest.entry(key).or_insert(f64::INFINITY);
        *low = low.min(p.z);
    }
    // The points again, by square, to look for the lowest within the radius.
    let mut members: HashMap<[i64; 2], Vec<&Point>> = HashMap::new();
    for p in &near {
        members.entry(square(p, shape.radius)).or_default().push(p);
    }
    let r2 = shape.radius * shape.radius;
    let high: Vec<Point> = near
        .iter()
        .filter(|p| {
            let [i, j] = square(p, shape.radius);
            // Quick: no square round it low enough, no step.
            let floor = (-1..=1)
                .flat_map(|di| (-1..=1).map(move |dj| [i + di, j + dj]))
                .filter_map(|k| lowest.get(&k))
                .fold(f64::INFINITY, |a, b| a.min(*b));
            if p.z - floor <= shape.min_step {
                return false;
            }
            let low = (-1..=1)
                .flat_map(|di| (-1..=1).map(move |dj| [i + di, j + dj]))
                .filter_map(|k| members.get(&k))
                .flatten()
                .filter(|q| (q.x - p.x).powi(2) + (q.y - p.y).powi(2) <= r2)
                .fold(f64::INFINITY, |a, q| a.min(q.z));
            let rise = p.z - low;
            rise > shape.min_step && rise <= shape.max_step
        })
        .map(|p| **p)
        .collect();
    let mut by_cell: HashMap<[i64; 2], (usize, Point)> = HashMap::new();
    for p in high {
        let entry = by_cell.entry(square(&p, shape.cell)).or_insert((0, p));
        entry.0 += 1;
        if p.flat() < entry.1.flat() {
            entry.1 = p;
        }
    }
    let mut kept: Vec<([i64; 2], Point)> = by_cell
        .into_iter()
        .filter(|(_, (n, _))| *n >= shape.min_points)
        .map(|(k, (_, p))| (k, p))
        .collect();
    kept.sort_unstable_by_key(|(k, _)| *k);
    kept.into_iter().map(|(_, p)| p).collect()
}

/// The steps that are kerbs: those not at the foot of something taller than any kerb.
///
/// A vertical face makes steps too: where the road meets it, and between its own rings
/// (a post 4 m out is hit 0.37 m and 0.53 m up, 0.16 m apart). A person, a post, a car's
/// side or a wall stands higher than [`StepShape::max_step`] over the road, a kerb never
/// does. So a step within [`StepShape::radius`] (flat) of a point higher than that and no
/// higher than `ceiling` (what the car passes under: a sign's board isn't a wall) belongs
/// to the tall thing, not the road. The lidar sees the tall thing at any range (it is a
/// cube: `max_step` is over the cubes' floor), and it may walk away: the step memory must
/// not keep its foot (TODO 189: a walker crossing under every beam left a kerb across the
/// lane that the car stood before for ever).
///
/// `found`: [`steps`]' answer; `points`: the same cloud's points (as [`crate::usable`] gives
/// them); `road`: the cloud's road ([`crate::Cubes::road`]). The order of `found` is kept.
#[must_use]
pub fn kerbs_only(
    found: Vec<Point>,
    points: &[Point],
    road: &crate::Road,
    shape: &StepShape,
    ceiling: f64,
) -> Vec<Point> {
    let plane = road.plane();
    let mut tall: HashMap<[i64; 2], Vec<&Point>> = HashMap::new();
    for p in points {
        let above = p.z - plane.height(p.x, p.y);
        if above > shape.max_step && above <= ceiling {
            tall.entry(square(p, shape.radius)).or_default().push(p);
        }
    }
    if tall.is_empty() {
        return found;
    }
    let r2 = shape.radius * shape.radius;
    found
        .into_iter()
        .filter(|p| {
            let [i, j] = square(p, shape.radius);
            !(-1..=1)
                .flat_map(|di| (-1..=1).map(move |dj| [i + di, j + dj]))
                .filter_map(|k| tall.get(&k))
                .flatten()
                .any(|q| (q.x - p.x).powi(2) + (q.y - p.y).powi(2) <= r2)
        })
        .collect()
}

#[cfg(test)]
#[path = "steps_tests.rs"]
mod tests;
