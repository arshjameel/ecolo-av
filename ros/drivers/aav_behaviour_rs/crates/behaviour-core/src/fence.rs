//! The geofence's outline and how deep a point is inside it (`aav_safety/geofence.py`'s
//! `Polygon`): the behaviour node's fence stop is where the plan's depth falls below
//! `near_depth`.

use std::fmt;

use crate::paths::Point;
use crate::{hypot, py_max, py_min, py_sum};

/// A fence with corners (x, y), in order round it, either way. Not self-crossing.
#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    corners: Vec<Point>,
}

/// Corners that make no fence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FenceError {
    /// Fewer than 3 corners, or one not finite.
    Corners,
    /// m²: so small it is a line.
    Area(f64),
}

impl fmt::Display for FenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FenceError::Corners => f.write_str("a polygon needs at least 3 finite corners"),
            FenceError::Area(a) => write!(f, "the polygon is {a:.2} m^2: is it a line?"),
        }
    }
}

impl std::error::Error for FenceError {}

impl Polygon {
    /// A fence of these corners.
    ///
    /// # Errors
    ///
    /// [`FenceError`] for fewer than 3 finite corners, or an area under 1 m².
    pub fn new(corners: Vec<Point>) -> Result<Self, FenceError> {
        if corners.len() < 3 || !corners.iter().all(|p| p.0.is_finite() && p.1.is_finite()) {
            return Err(FenceError::Corners);
        }
        let n = corners.len();
        let area = py_sum((0..n).map(|i| {
            let ((x0, y0), (x1, y1)) = (corners[i], corners[(i + 1) % n]);
            x0 * y1 - x1 * y0
        })) / 2.0;
        if area.abs() < 1.0 {
            return Err(FenceError::Area(area.abs()));
        }
        Ok(Polygon { corners })
    }

    fn edges(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        let n = self.corners.len();
        (0..n).map(move |i| (self.corners[i], self.corners[(i + 1) % n]))
    }

    /// Whether (x, y) is inside.
    #[must_use]
    pub fn contains(&self, x: f64, y: f64) -> bool {
        let mut inside = false;
        for ((x0, y0), (x1, y1)) in self.edges() {
            if (y0 > y) != (y1 > y) && x < x0 + (y - y0) * (x1 - x0) / (y1 - y0) {
                inside = !inside;
            }
        }
        inside
    }

    /// How far (x, y) is inside the fence (m); negative outside.
    #[must_use]
    pub fn depth(&self, x: f64, y: f64) -> f64 {
        let nearest = self
            .edges()
            .map(|(a, b)| to_segment(x, y, a, b))
            .reduce(py_min)
            .unwrap_or(f64::INFINITY);
        if self.contains(x, y) {
            nearest
        } else {
            -nearest
        }
    }
}

fn to_segment(x: f64, y: f64, a: Point, b: Point) -> f64 {
    let ((ax, ay), (bx, by)) = (a, b);
    let (dx, dy) = (bx - ax, by - ay);
    let length2 = dx * dx + dy * dy;
    let t = if length2 == 0.0 {
        0.0
    } else {
        py_max(0.0, py_min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
    };
    hypot(x - (ax + t * dx), y - (ay + t * dy))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn depth_is_positive_inside_and_negative_outside() {
        let square =
            Polygon::new(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]).unwrap();
        assert_eq!(square.depth(2.0, 5.0), 2.0);
        assert_eq!(square.depth(12.0, 5.0), -2.0);
        assert!(Polygon::new(vec![(0.0, 0.0), (1.0, 0.0)]).is_err());
        assert!(matches!(
            Polygon::new(vec![(0.0, 0.0), (10.0, 0.0), (20.0, 0.01)]),
            Err(FenceError::Area(_))
        ));
    }
}
