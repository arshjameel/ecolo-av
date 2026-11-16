//! A planned path as the tracker sees it: where the car is along it, and how it bends.
//! A port of `aav_track/path.py` (its `Path`, `Foot` and `wrap`).
//!
//! ```text
//!    path:  p0 ---- p1 ---- p2 ---- p3 ---- ... ---- end
//!                      ^ the car's foot on the path: s (m from p0), cross (+ = car left of it)
//!                      |
//!                     car
//! ```
//!
//! `project` searches forward from the last foot only (`window` m), so a path that loops
//! back past itself can't make the car jump ahead. Past the ends the path carries on
//! straight, so a point beyond it still has a line.

use std::f64::consts::PI;

/// The angle in [-pi, pi), as Python's `(a + pi) % (2 pi) - pi`.
#[must_use]
pub fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

/// Where a point is against the path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Foot {
    /// m along the path from its start, to the point's foot on it.
    pub s: f64,
    /// m, + = the point is left of the path.
    pub cross: f64,
    /// rad: the path's direction at the foot.
    pub heading: f64,
    /// The segment the foot is on.
    pub index: usize,
}

/// Why a path can't be made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathError {
    /// No poses at all.
    Empty,
    /// A point that is not a number.
    NotANumber,
}

impl std::fmt::Display for PathError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            PathError::Empty => "an empty path",
            PathError::NotANumber => "a path point is not a number",
        })
    }
}

impl std::error::Error for PathError {}

/// A polyline with its arc length; the last pose's yaw is the goal's heading.
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    points: Vec<(f64, f64)>,
    s: Vec<f64>,
    headings: Vec<f64>,
    goal_yaw: f64,
    /// m either side over which the curvature is measured.
    bend: f64,
}

impl Path {
    /// The planner's (x, y, yaw) poses; repeated points are dropped. Curvature over
    /// +-0.5 m, as path.py's default.
    ///
    /// # Errors
    ///
    /// [`PathError::Empty`] for no poses; [`PathError::NotANumber`] for a NaN.
    pub fn new(poses: &[(f64, f64, f64)]) -> Result<Self, PathError> {
        Self::with_bend(poses, 0.5)
    }

    /// As [`Path::new`], with the curvature measured over +-`bend` m.
    ///
    /// # Errors
    ///
    /// As [`Path::new`].
    pub fn with_bend(poses: &[(f64, f64, f64)], bend: f64) -> Result<Self, PathError> {
        let &(_, _, goal_yaw) = poses.last().ok_or(PathError::Empty)?;
        let mut points: Vec<(f64, f64)> = Vec::with_capacity(poses.len());
        for &(x, y, _) in poses {
            if !(x.is_finite() && y.is_finite()) {
                return Err(PathError::NotANumber);
            }
            let far = points
                .last()
                .is_none_or(|&(px, py)| (x - px).hypot(y - py) > 1e-6);
            if far {
                points.push((x, y));
            }
        }
        if let [(x, y)] = points[..] {
            // One point: a short line along the goal's heading, so there is a direction.
            points.push((x + 0.01 * goal_yaw.cos(), y + 0.01 * goal_yaw.sin()));
        }
        let mut s = vec![0.0];
        let mut headings = Vec::with_capacity(points.len() - 1);
        for w in points.windows(2) {
            let ((x0, y0), (x1, y1)) = (w[0], w[1]);
            s.push(s[s.len() - 1] + (x1 - x0).hypot(y1 - y0));
            headings.push((y1 - y0).atan2(x1 - x0));
        }
        Ok(Path {
            points,
            s,
            headings,
            goal_yaw,
            bend,
        })
    }

    /// m from the start to the end.
    #[must_use]
    pub fn length(&self) -> f64 {
        self.s[self.s.len() - 1]
    }

    /// The last point.
    #[must_use]
    pub fn end(&self) -> (f64, f64) {
        self.points[self.points.len() - 1]
    }

    /// The goal's heading (the last pose's yaw).
    #[must_use]
    pub fn goal_yaw(&self) -> f64 {
        self.goal_yaw
    }

    /// The points, repeated ones dropped.
    #[must_use]
    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    /// The same path with every pose moved by `f` (x, y, yaw), as the held map -> odom moves.
    #[must_use]
    pub fn moved(&self, f: impl Fn(f64, f64, f64) -> (f64, f64, f64)) -> Self {
        let n = self.points.len();
        let poses: Vec<(f64, f64, f64)> = self
            .points
            .iter()
            .enumerate()
            .map(|(i, &(x, y))| {
                let yaw = if i + 1 == n {
                    self.goal_yaw
                } else {
                    self.headings[i]
                };
                f(x, y, yaw)
            })
            .collect();
        let mut path = Self::with_bend(&poses, self.bend).unwrap_or_else(|_| self.clone());
        path.goal_yaw = f(self.end().0, self.end().1, self.goal_yaw).2;
        path
    }

    /// The segment holding arc length `s` (the first or last past the ends).
    fn segment(&self, s: f64) -> usize {
        // bisect_right(self.s, s) - 1, within the segments.
        let i = self.s.partition_point(|&a| a <= s);
        i.saturating_sub(1).min(self.headings.len() - 1)
    }

    /// The point `s` m along; before the start or past the end, straight on.
    #[must_use]
    pub fn point_at(&self, s: f64) -> (f64, f64) {
        let i = self.segment(s);
        let (x0, y0) = self.points[i];
        let h = self.headings[i];
        let d = s - self.s[i];
        (x0 + d * h.cos(), y0 + d * h.sin())
    }

    /// The path's direction at `s`.
    #[must_use]
    pub fn heading_at(&self, s: f64) -> f64 {
        self.headings[self.segment(s)]
    }

    /// The curvature at `s` (1/m, + left): the turn over +-bend m.
    #[must_use]
    pub fn curvature_at(&self, s: f64) -> f64 {
        let a = (s - self.bend).max(0.0);
        let b = (s + self.bend).min(self.length());
        if b - a < 1e-6 {
            return 0.0;
        }
        wrap(self.heading_at(b - 1e-9) - self.heading_at(a)) / (b - a)
    }

    /// Where `p` is against the path, searching segments from `hint` on, `window` m. Past
    /// the last segment, the line straight on (so `s` may exceed the length).
    #[must_use]
    pub fn project(&self, p: (f64, f64), hint: usize, window: f64) -> Foot {
        let last = self.headings.len() - 1;
        let hint = hint.min(last);
        let mut best: Option<(f64, usize, f64)> = None;
        for i in (hint..=last).take_while(|&i| self.s[i] - self.s[hint] <= window) {
            let (x0, y0) = self.points[i];
            let (uy, ux) = self.headings[i].sin_cos();
            let seg = self.s[i + 1] - self.s[i];
            let t = (p.0 - x0) * ux + (p.1 - y0) * uy;
            let lo = if i == 0 { f64::NEG_INFINITY } else { 0.0 };
            let hi = if i == last { f64::INFINITY } else { seg };
            let t = t.max(lo).min(hi);
            let d2 = (p.0 - x0 - t * ux).powi(2) + (p.1 - y0 - t * uy).powi(2);
            if best.is_none_or(|(b, _, _)| d2 < b - 1e-12) {
                best = Some((d2, i, t));
            }
        }
        let (_, i, t) = best.unwrap_or((0.0, hint, 0.0));
        let (x0, y0) = self.points[i];
        let h = self.headings[i];
        Foot {
            s: self.s[i] + t,
            cross: -(p.0 - x0) * h.sin() + (p.1 - y0) * h.cos(),
            heading: h,
            index: i,
        }
    }

    /// m from `s` to the end (negative past it).
    #[must_use]
    pub fn left(&self, s: f64) -> f64 {
        self.length() - s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight(n: usize) -> Vec<(f64, f64, f64)> {
        (0..=n).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect()
    }

    #[test]
    fn wrap_is_pythons() {
        assert!((wrap(3.0 * PI / 2.0) + PI / 2.0).abs() < 1e-12);
        assert_eq!(wrap(PI), -PI);
        assert!((wrap(-0.1) + 0.1).abs() < 1e-15);
    }

    #[test]
    fn the_foot_on_a_line_and_past_its_ends() {
        let path = Path::new(&straight(100)).unwrap();
        assert!((path.length() - 10.0).abs() < 1e-9);
        let f = path.project((3.0, 0.5), 0, 10.0);
        assert!((f.s - 3.0).abs() < 1e-9 && (f.cross - 0.5).abs() < 1e-12);
        let f = path.project((-1.0, -0.2), 0, 10.0);
        assert!((f.s + 1.0).abs() < 1e-9 && (f.cross + 0.2).abs() < 1e-12);
        let f = path.project((12.0, 0.0), 90, 10.0);
        assert!((f.s - 12.0).abs() < 1e-9);
    }

    #[test]
    fn a_circle_bends_by_one_over_its_radius() {
        let r = 5.0;
        let poses: Vec<_> = (0..=300)
            .map(|i| {
                let a = i as f64 * 0.005;
                (r * a.sin(), r - r * a.cos(), a)
            })
            .collect();
        let path = Path::new(&poses).unwrap();
        assert!((path.curvature_at(3.0) - 1.0 / r).abs() < 1e-3);
    }

    #[test]
    fn bad_paths_say_why() {
        assert_eq!(Path::new(&[]), Err(PathError::Empty));
        assert_eq!(
            Path::new(&[(0.0, f64::NAN, 0.0)]),
            Err(PathError::NotANumber)
        );
        let one = Path::new(&[(1.0, 2.0, 0.0)]).unwrap();
        assert!((one.length() - 0.01).abs() < 1e-12);
    }
}
