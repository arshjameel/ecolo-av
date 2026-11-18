//! Where the car is against the line to B, and how to steer onto it (`line.py`).
//!
//! Map frame (x east, y north), metres and radians; the pose is `base_link`'s.
//!
//! ```text
//!   turn: pure pursuit to B           line: pure pursuit on A -> B
//!   car >---.                    A ------------x-------------> B
//!            `-.                     carrot: lookahead m ahead on the line
//! ```
//!
//! The corridor ([`Fence`]) stops a drive that leaves it, whatever the reason.

use std::cmp::Ordering;
use std::f64::consts::PI;
use std::fmt;

use crate::{fmt_g, hypot, py_max, py_min};

/// A map point (x, y), m.
pub type Point = (f64, f64);

/// The car's pose on the map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// m east.
    pub x: f64,
    /// m north.
    pub y: f64,
    /// rad, anticlockwise from east.
    pub yaw: f64,
}

/// The angle in [-π, π), as Python's `(a + pi) % (2 pi) - pi`.
#[must_use]
pub fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

/// A map point in the car's frame: x ahead, y to the left.
#[must_use]
pub fn to_car(pose: &Pose, point: Point) -> Point {
    let (dx, dy) = (point.0 - pose.x, point.1 - pose.y);
    let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
    (c * dx + s * dy, -s * dx + c * dy)
}

/// How far B is off the car's nose (rad, + to the left).
#[must_use]
pub fn bearing_error(pose: &Pose, target: Point) -> f64 {
    let (ahead, left) = to_car(pose, target);
    left.atan2(ahead)
}

/// Numbers that make no car or no line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LineError {
    /// The wheelbase must be above 0, the steering limit in (0, π/2).
    Car,
    /// A and B are the same point.
    SamePoint,
    /// The fence's number (its name) must be above 0.
    Fence(&'static str, f64),
}

impl fmt::Display for LineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LineError::Car => {
                f.write_str("wheelbase must be above 0 and max_steering in (0, pi/2)")
            }
            LineError::SamePoint => f.write_str("A and B are the same point"),
            LineError::Fence(name, v) => {
                write!(f, "{name} must be above 0 (got {})", crate::py_repr(*v))
            }
        }
    }
}

impl std::error::Error for LineError {}

/// The tightest turning radius (m) of `base_link`, at full lock.
///
/// # Errors
///
/// [`LineError::Car`] for a wheelbase not above 0 or a steering limit outside (0, π/2).
pub fn min_radius(wheelbase: f64, max_steering: f64) -> Result<f64, LineError> {
    if wheelbase <= 0.0 || !(max_steering > 0.0 && max_steering < PI / 2.0) {
        return Err(LineError::Car);
    }
    Ok(wheelbase / max_steering.tan())
}

/// True if a forward-only car can drive to `target` without reversing: not inside either
/// of its tightest circles, `margin` m more.
#[must_use]
pub fn reachable(pose: &Pose, target: Point, radius: f64, margin: f64) -> bool {
    [1.0, -1.0].into_iter().all(|side: f64| {
        let cx = pose.x - side * radius * pose.yaw.sin();
        let cy = pose.y + side * radius * pose.yaw.cos();
        // Not "<": a NaN distance counts as reachable, as in the Python.
        hypot(target.0 - cx, target.1 - cy).partial_cmp(&(radius + margin)) != Some(Ordering::Less)
    })
}

/// The straight line from A to B.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line {
    /// Where it starts.
    pub a: Point,
    /// B.
    pub b: Point,
}

impl Line {
    /// Its length (m).
    #[must_use]
    pub fn length(&self) -> f64 {
        hypot(self.b.0 - self.a.0, self.b.1 - self.a.1)
    }

    /// The unit vector from A to B.
    ///
    /// # Errors
    ///
    /// [`LineError::SamePoint`] when A is B.
    pub fn unit(&self) -> Result<Point, LineError> {
        let n = self.length();
        if n <= 0.0 {
            return Err(LineError::SamePoint);
        }
        Ok(((self.b.0 - self.a.0) / n, (self.b.1 - self.a.1) / n))
    }

    /// (metres left to B along the line, metres left (+) of the line).
    ///
    /// # Errors
    ///
    /// [`LineError::SamePoint`] when A is B.
    pub fn progress(&self, p: Point) -> Result<(f64, f64), LineError> {
        let (ux, uy) = self.unit()?;
        let (dx, dy) = (p.0 - self.b.0, p.1 - self.b.1);
        Ok((-(dx * ux + dy * uy), -dx * uy + dy * ux))
    }

    /// The point on the line `lookahead` m from p, ahead of it (past B is fine); off the
    /// line by more than the lookahead, the nearest point on it.
    ///
    /// # Errors
    ///
    /// [`LineError::SamePoint`] when A is B.
    pub fn carrot(&self, p: Point, lookahead: f64) -> Result<Point, LineError> {
        let (ux, uy) = self.unit()?;
        let (left, cross) = self.progress(p)?;
        let mut along = self.length() - left;
        if cross.abs() < lookahead {
            along += (lookahead * lookahead - cross * cross).sqrt();
        }
        Ok((self.a.0 + along * ux, self.a.1 + along * uy))
    }
}

/// Why the car is outside a drive's corridor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Breach {
    /// Further than `max_off_line` from the line (m off it, left of it?, the limit).
    OffLine(f64, bool, f64),
    /// Further than `max_overrun` past B (m past, the limit).
    Overrun(f64, f64),
    /// Further from the start than the drive could need (m, the limit).
    Far(f64, f64),
}

impl fmt::Display for Breach {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Breach::OffLine(off, left, limit) => write!(
                f,
                "outside the fence: {off:.1} m {} of the line to B, over max_off_line {} m",
                if left { "left" } else { "right" },
                fmt_g(limit)
            ),
            Breach::Overrun(past, limit) => write!(
                f,
                "outside the fence: {past:.1} m past B, over max_overrun {} m",
                fmt_g(limit)
            ),
            Breach::Far(far, reach) => write!(
                f,
                "outside the fence: {far:.1} m from the start, over {reach:.1} m (the distance \
                 to B, or a turn round, plus max_overrun)"
            ),
        }
    }
}

/// Where a drive to B may go (`line.Fence`). Distances in m.
///
/// While it turns towards B: only the circle round the start, which a turn round (up to
/// 2 R) stays inside. Once lined up, also `max_off_line` either side of the line it
/// follows, ending `max_overrun` past B.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fence {
    start: Point,
    target: Point,
    turn_radius: f64,
    max_off_line: f64,
    max_overrun: f64,
}

impl Fence {
    /// The corridor of a drive from `start` to `target`.
    ///
    /// # Errors
    ///
    /// [`LineError::Fence`] for a radius or limit not above 0.
    pub fn new(
        start: Point,
        target: Point,
        turn_radius: f64,
        max_off_line: f64,
        max_overrun: f64,
    ) -> Result<Self, LineError> {
        for (name, value) in [
            ("turn_radius", turn_radius),
            ("max_off_line", max_off_line),
            ("max_overrun", max_overrun),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return Err(LineError::Fence(name, value));
            }
        }
        Ok(Fence {
            start,
            target,
            turn_radius,
            max_off_line,
            max_overrun,
        })
    }

    /// How far from the start the car may ever be.
    #[must_use]
    pub fn reach(&self) -> f64 {
        let ab = hypot(self.target.0 - self.start.0, self.target.1 - self.start.1);
        py_max(ab, 2.0 * self.turn_radius) + self.max_overrun
    }

    /// Why the car at p is outside the fence, or None. `line`: the one it follows.
    #[must_use]
    pub fn breach(&self, p: Point, line: Option<&Line>) -> Option<Breach> {
        if let Some((left, cross)) = line.and_then(|l| l.progress(p).ok()) {
            if cross.abs() > self.max_off_line {
                return Some(Breach::OffLine(cross.abs(), cross > 0.0, self.max_off_line));
            }
            if -left > self.max_overrun {
                return Some(Breach::Overrun(-left, self.max_overrun));
            }
        }
        let far = hypot(p.0 - self.start.0, p.1 - self.start.1);
        let reach = self.reach();
        (far > reach).then_some(Breach::Far(far, reach))
    }
}

/// The curvature (1/m, + left) of the arc from the car through `point`: pure pursuit,
/// k = 2 y / (x² + y²) in the car's frame, clamped to full lock. A point behind the car
/// gets full lock towards its side (left if dead behind).
#[must_use]
pub fn pursuit_curvature(pose: &Pose, point: Point, max_curvature: f64) -> f64 {
    let (ahead, left) = to_car(pose, point);
    if ahead <= 0.0 {
        return if left >= 0.0 {
            max_curvature
        } else {
            -max_curvature
        };
    }
    let d2 = ahead * ahead + left * left;
    let k = if d2 > 0.0 { 2.0 * left / d2 } else { 0.0 };
    py_max(-max_curvature, py_min(max_curvature, k))
}

/// The fastest speed for this curvature with at most `lateral_accel` sideways.
#[must_use]
pub fn turn_speed_limit(curvature: f64, lateral_accel: f64) -> f64 {
    let k = curvature.abs();
    if k > 1e-6 {
        (lateral_accel / k).sqrt()
    } else {
        f64::INFINITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn progress_and_carrot_on_a_line_east() {
        let line = Line {
            a: (0.0, 0.0),
            b: (10.0, 0.0),
        };
        assert_eq!(line.progress((4.0, 1.0)).unwrap(), (6.0, 1.0));
        assert_eq!(line.carrot((4.0, 0.0), 2.0).unwrap(), (6.0, 0.0));
        assert!(Line {
            a: (1.0, 1.0),
            b: (1.0, 1.0)
        }
        .unit()
        .is_err());
    }

    #[test]
    fn the_fence_holds_the_line_and_the_end() {
        let line = Line {
            a: (0.0, 0.0),
            b: (10.0, 0.0),
        };
        let fence = Fence::new((0.0, 0.0), (10.0, 0.0), 3.0, 3.0, 2.0).unwrap();
        assert_eq!(fence.breach((5.0, 1.0), Some(&line)), None);
        assert!(matches!(
            fence.breach((5.0, -4.0), Some(&line)),
            Some(Breach::OffLine(..))
        ));
        assert!(matches!(
            fence.breach((13.0, 0.0), Some(&line)),
            Some(Breach::Overrun(..))
        ));
        assert!(matches!(
            fence.breach((13.0, 0.0), None),
            Some(Breach::Far(..))
        ));
    }

    #[test]
    fn pursuit_clamps_and_turns_round() {
        let pose = Pose {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        };
        assert_eq!(pursuit_curvature(&pose, (-5.0, 0.0), 0.3), 0.3);
        assert_eq!(pursuit_curvature(&pose, (-5.0, -1.0), 0.3), -0.3);
        assert_eq!(pursuit_curvature(&pose, (5.0, 0.0), 0.3), 0.0);
        assert!(turn_speed_limit(0.0, 1.5).is_infinite());
        assert!(!reachable(&pose, (0.0, 3.0), 3.0, 0.5));
        assert!(min_radius(1.57, 2.0).is_err());
    }
}
