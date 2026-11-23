//! The car's body from above, and a turned grid's corners.

use map_core::geom::{Point, Pose};

/// The body's size, from `vehicle.yaml` (`body:`), metres. `base_link` is under the
/// rear axle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    /// Bumper to bumper.
    pub length: f64,
    pub width: f64,
    /// The rear bumper, behind the rear axle.
    pub rear_overhang: f64,
}

impl Body {
    /// The body's four corners on the map, closed: rear right, front right, front
    /// left, rear left, rear right.
    #[must_use]
    pub fn outline(&self, car: Pose) -> [Point; 5] {
        let (back, front, half) = (
            -self.rear_overhang,
            self.length - self.rear_overhang,
            self.width / 2.0,
        );
        [
            (back, -half),
            (front, -half),
            (front, half),
            (back, half),
            (back, -half),
        ]
        .map(|(x, y)| from_pose(car, x, y))
    }

    /// The front bumper's middle on the map.
    #[must_use]
    pub fn nose(&self, car: Pose) -> Point {
        from_pose(car, self.length - self.rear_overhang, 0.0)
    }
}

/// A point `x` ahead and `y` left of a pose.
#[must_use]
pub fn from_pose(pose: Pose, x: f64, y: f64) -> Point {
    let (sin, cos) = pose.yaw.sin_cos();
    Point::new(
        pose.position.x + x * cos - y * sin,
        pose.position.y + x * sin + y * cos,
    )
}

/// A grid's four corners in its own frame, from its origin's pose and its size in
/// metres: cell (0, 0)'s corner, then along x, the far corner, then along y. A rolling
/// costmap in odom is turned on the map: the panel draws its picture on these.
#[must_use]
pub fn quad(origin: Pose, across: f64, up: f64) -> [Point; 4] {
    [(0.0, 0.0), (across, 0.0), (across, up), (0.0, up)].map(|(x, y)| from_pose(origin, x, y))
}

#[cfg(test)]
mod tests {
    use super::*;

    const BODY: Body = Body {
        length: 2.0,
        width: 1.0,
        rear_overhang: 0.2,
    };

    #[test]
    fn the_body_sits_on_the_rear_axle() {
        let line = BODY.outline(Pose::new(10.0, 0.0, 0.0));
        assert_eq!(line[0], Point::new(9.8, -0.5));
        assert_eq!(line[2], Point::new(11.8, 0.5));
        assert_eq!(line[0], line[4]);
        assert_eq!(BODY.nose(Pose::new(10.0, 0.0, 0.0)), Point::new(11.8, 0.0));
    }

    #[test]
    fn the_body_turns_with_the_car() {
        let line = BODY.outline(Pose::new(0.0, 0.0, std::f64::consts::FRAC_PI_2));
        // Facing north: the front left corner is north-west.
        assert!((line[2].x + 0.5).abs() < 1e-9 && (line[2].y - 1.8).abs() < 1e-9);
    }

    #[test]
    fn a_grids_corners_follow_its_origin() {
        let corners = quad(Pose::new(-10.0, -10.0, 0.0), 20.0, 20.0);
        assert_eq!(corners[2], Point::new(10.0, 10.0));
        let turned = quad(Pose::new(0.0, 0.0, std::f64::consts::FRAC_PI_2), 20.0, 10.0);
        assert!((turned[1].x).abs() < 1e-9 && (turned[1].y - 20.0).abs() < 1e-9);
        assert!((turned[3].x + 10.0).abs() < 1e-9 && turned[3].y.abs() < 1e-9);
    }
}
