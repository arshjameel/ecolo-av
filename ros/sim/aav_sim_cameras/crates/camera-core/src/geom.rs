//! Points, rotations and poses in ROS's axes (REP-103: x forward, y left, z up).

use std::ops::{Add, Mul, Neg, Sub};

/// A point or a direction, in metres.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[must_use]
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[must_use]
    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// The same direction, one long; `None` for the zero vector.
    #[must_use]
    pub fn normalized(self) -> Option<Self> {
        let length = self.length();
        (length > f64::EPSILON).then(|| self * (1.0 / length))
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k, self.z * k)
    }
}

/// A rotation, as its 3 x 3 matrix (rows).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rotation {
    rows: [[f64; 3]; 3],
}

impl Rotation {
    pub const IDENTITY: Self = Self {
        rows: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    /// URDF's and SDF's roll, pitch, yaw (radians): about fixed x, then y, then z.
    #[must_use]
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        Self {
            rows: [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
        }
    }

    /// A turn about z, the only one a car on flat ground makes.
    #[must_use]
    pub fn from_yaw(yaw: f64) -> Self {
        Self::from_rpy(0.0, 0.0, yaw)
    }

    /// From a quaternion (x, y, z, w), as in geometry_msgs. Normalised first.
    #[must_use]
    pub fn from_quaternion(x: f64, y: f64, z: f64, w: f64) -> Self {
        let n = (x * x + y * y + z * z + w * w).sqrt();
        if n < f64::EPSILON {
            return Self::IDENTITY;
        }
        let (x, y, z, w) = (x / n, y / n, z / n, w / n);
        Self {
            rows: [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
        }
    }

    /// Built from where the local x, y and z axes point.
    #[must_use]
    pub fn from_axes(x: Vec3, y: Vec3, z: Vec3) -> Self {
        Self {
            rows: [[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]],
        }
    }

    /// The matrix, row by row.
    #[must_use]
    pub fn rows(&self) -> [[f64; 3]; 3] {
        self.rows
    }

    /// Where the local axis `i` (0 = x, 1 = y, 2 = z) points.
    #[must_use]
    pub fn axis(&self, i: usize) -> Vec3 {
        Vec3::new(self.rows[0][i], self.rows[1][i], self.rows[2][i])
    }

    #[must_use]
    pub fn apply(&self, v: Vec3) -> Vec3 {
        let r = &self.rows;
        Vec3::new(
            r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z,
            r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z,
            r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z,
        )
    }

    #[must_use]
    pub fn inverse(&self) -> Self {
        let r = &self.rows;
        Self {
            rows: [
                [r[0][0], r[1][0], r[2][0]],
                [r[0][1], r[1][1], r[2][1]],
                [r[0][2], r[1][2], r[2][2]],
            ],
        }
    }

    /// This rotation after `inner`: `self * inner`.
    #[must_use]
    pub fn then(&self, inner: &Self) -> Self {
        Self::from_axes(
            self.apply(inner.axis(0)),
            self.apply(inner.axis(1)),
            self.apply(inner.axis(2)),
        )
    }

    /// As a quaternion (x, y, z, w).
    #[must_use]
    pub fn to_quaternion(&self) -> [f64; 4] {
        let m = &self.rows;
        let trace = m[0][0] + m[1][1] + m[2][2];
        if trace > 0.0 {
            let s = (trace + 1.0).sqrt() * 2.0;
            [
                (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s,
                0.25 * s,
            ]
        } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
            let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
            [
                0.25 * s,
                (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s,
                (m[2][1] - m[1][2]) / s,
            ]
        } else if m[1][1] > m[2][2] {
            let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
            [
                (m[0][1] + m[1][0]) / s,
                0.25 * s,
                (m[1][2] + m[2][1]) / s,
                (m[0][2] - m[2][0]) / s,
            ]
        } else {
            let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
            [
                (m[0][2] + m[2][0]) / s,
                (m[1][2] + m[2][1]) / s,
                0.25 * s,
                (m[1][0] - m[0][1]) / s,
            ]
        }
    }
}

/// Where a frame sits in its parent: `parent_point = rotation * child_point + translation`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub translation: Vec3,
    pub rotation: Rotation,
}

impl Default for Pose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Pose {
    pub const IDENTITY: Self = Self {
        translation: Vec3::ZERO,
        rotation: Rotation::IDENTITY,
    };

    #[must_use]
    pub fn new(translation: Vec3, rotation: Rotation) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    /// URDF's `xyz` and `rpy`, as `vehicle.yaml` gives a sensor's mount.
    #[must_use]
    pub fn from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> Self {
        Self::new(
            Vec3::new(xyz[0], xyz[1], xyz[2]),
            Rotation::from_rpy(rpy[0], rpy[1], rpy[2]),
        )
    }

    /// A car on flat ground: x, y (m) and its heading (rad, 0 = east in map, + = left).
    #[must_use]
    pub fn planar(x: f64, y: f64, yaw: f64) -> Self {
        Self::new(Vec3::new(x, y, 0.0), Rotation::from_yaw(yaw))
    }

    /// A point in this frame, in the parent.
    #[must_use]
    pub fn apply(&self, p: Vec3) -> Vec3 {
        self.rotation.apply(p) + self.translation
    }

    /// `self * child`: the child frame's pose in this frame's parent.
    #[must_use]
    pub fn then(&self, child: &Self) -> Self {
        Self::new(
            self.apply(child.translation),
            self.rotation.then(&child.rotation),
        )
    }

    #[must_use]
    pub fn inverse(&self) -> Self {
        let r = self.rotation.inverse();
        Self::new(-r.apply(self.translation), r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-9
    }

    #[test]
    fn yaw_turns_x_to_y() {
        let r = Rotation::from_yaw(std::f64::consts::FRAC_PI_2);
        assert!(near(
            r.apply(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.0, 1.0, 0.0)
        ));
    }

    #[test]
    fn positive_pitch_looks_down() {
        let r = Rotation::from_rpy(0.0, 0.3, 0.0);
        assert!(r.apply(Vec3::new(1.0, 0.0, 0.0)).z < 0.0);
    }

    #[test]
    fn quaternion_round_trip() {
        let r = Rotation::from_rpy(0.1, -0.2, 2.9);
        let [x, y, z, w] = r.to_quaternion();
        let back = Rotation::from_quaternion(x, y, z, w);
        for i in 0..3 {
            assert!(near(r.axis(i), back.axis(i)));
        }
    }

    #[test]
    fn pose_inverse_undoes() {
        let p = Pose::from_xyz_rpy([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]);
        let q = Vec3::new(-4.0, 5.0, 0.5);
        assert!(near(p.inverse().apply(p.apply(q)), q));
        assert!(near(p.then(&p.inverse()).translation, Vec3::ZERO));
    }
}
