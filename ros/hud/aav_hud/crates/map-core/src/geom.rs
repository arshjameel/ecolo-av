//! Points and poses on the map: metres in the map frame (x east, y north, REP-103/105),
//! angles in radians, counter-clockwise from east.

use std::ops::{Add, Mul, Sub};

/// A distance, in metres.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Metres(pub f64);

/// A point on the map, in metres.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
    /// m, east.
    pub x: f64,
    /// m, north.
    pub y: f64,
}

impl Point {
    /// A point from x and y (m).
    #[must_use]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// The straight-line distance to another point.
    #[must_use]
    pub fn distance(self, other: Point) -> Metres {
        Metres((other - self).length())
    }

    /// The length of this point seen as a vector from the origin.
    #[must_use]
    pub fn length(self) -> f64 {
        self.x.hypot(self.y)
    }

    /// The dot product, as vectors.
    #[must_use]
    pub fn dot(self, other: Point) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Turned `yaw` radians counter-clockwise about the origin.
    #[must_use]
    pub fn rotated(self, yaw: f64) -> Point {
        let (s, c) = yaw.sin_cos();
        Point::new(c * self.x - s * self.y, s * self.x + c * self.y)
    }

    /// True if both numbers are finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

impl Add for Point {
    type Output = Point;
    fn add(self, o: Point) -> Point {
        Point::new(self.x + o.x, self.y + o.y)
    }
}

impl Sub for Point {
    type Output = Point;
    fn sub(self, o: Point) -> Point {
        Point::new(self.x - o.x, self.y - o.y)
    }
}

impl Mul<f64> for Point {
    type Output = Point;
    fn mul(self, k: f64) -> Point {
        Point::new(self.x * k, self.y * k)
    }
}

/// A position and a heading on the map.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose {
    /// Where it is.
    pub position: Point,
    /// Radians, counter-clockwise from the map's x axis (east).
    pub yaw: f64,
}

impl Pose {
    /// A pose from x, y (m) and yaw (rad).
    #[must_use]
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self {
            position: Point::new(x, y),
            yaw,
        }
    }

    /// From a ROS pose: its position and its orientation quaternion. Only the heading is
    /// kept: the map is flat.
    #[must_use]
    pub fn from_ros(x: f64, y: f64, q: Quaternion) -> Self {
        Self {
            position: Point::new(x, y),
            yaw: q.yaw(),
        }
    }

    /// A point given in this pose's own frame (x forward, y left), on the map.
    #[must_use]
    pub fn transform(self, local: Point) -> Point {
        self.position + local.rotated(self.yaw)
    }
}

/// A ROS orientation quaternion (x, y, z, w).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    /// The x part.
    pub x: f64,
    /// The y part.
    pub y: f64,
    /// The z part.
    pub z: f64,
    /// The w part.
    pub w: f64,
}

impl Quaternion {
    /// The heading about z (yaw), radians. A zero quaternion (all fields 0, an unset
    /// message) comes out facing east.
    #[must_use]
    pub fn yaw(self) -> f64 {
        let Self { x, y, z, w } = self;
        let siny = 2.0 * (w * z + x * y);
        let cosy = 1.0 - 2.0 * (y * y + z * z);
        siny.atan2(cosy)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn yaw_from_a_quaternion() {
        let half = FRAC_PI_2 / 2.0;
        let north = Quaternion {
            x: 0.0,
            y: 0.0,
            z: half.sin(),
            w: half.cos(),
        };
        assert!(close(north.yaw(), FRAC_PI_2));
        let unset = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        assert_eq!(unset.yaw(), 0.0);
    }

    #[test]
    fn a_pose_moves_its_own_points_onto_the_map() {
        // Facing north at (10, 5): one metre forward is one metre north.
        let p = Pose::new(10.0, 5.0, FRAC_PI_2);
        let ahead = p.transform(Point::new(1.0, 0.0));
        assert!(close(ahead.x, 10.0) && close(ahead.y, 6.0));
        let left = p.transform(Point::new(0.0, 1.0));
        assert!(close(left.x, 9.0) && close(left.y, 5.0));
    }

    #[test]
    fn distance() {
        assert_eq!(
            Point::new(0.0, 0.0).distance(Point::new(3.0, 4.0)),
            Metres(5.0)
        );
    }
}
