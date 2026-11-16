//! Small 3D maths: vectors, rotations and poses. Metres and radians; x forward, y left, z up.

use std::ops::{Add, Mul, Neg, Sub};

/// A point or a direction, in metres.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3::new(0.0, 0.0, 0.0);

    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[must_use]
    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    #[must_use]
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    #[must_use]
    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    #[must_use]
    pub fn min(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }

    #[must_use]
    pub fn max(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }

    /// The component along an axis: 0 x, 1 y, 2 z.
    #[must_use]
    pub fn axis(self, axis: usize) -> f64 {
        match axis {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, k: f64) -> Vec3 {
        Vec3::new(self.x * k, self.y * k, self.z * k)
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        self * -1.0
    }
}

/// A rotation, as a 3 x 3 matrix (rows).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rot3 {
    pub m: [[f64; 3]; 3],
}

impl Default for Rot3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Rot3 {
    pub const IDENTITY: Rot3 = Rot3 {
        m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    /// Roll about x, then pitch about y, then yaw about z (fixed axes), as SDF and URDF.
    #[must_use]
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        Rot3 {
            m: [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
        }
    }

    #[must_use]
    pub fn from_yaw(yaw: f64) -> Self {
        Self::from_rpy(0.0, 0.0, yaw)
    }

    #[must_use]
    pub fn apply(&self, v: Vec3) -> Vec3 {
        let m = &self.m;
        Vec3::new(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
        )
    }

    #[must_use]
    pub fn then(&self, inner: &Rot3) -> Rot3 {
        let a = &self.m;
        let b = &inner.m;
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (0..3).map(|k| a[i][k] * b[k][j]).sum();
            }
        }
        Rot3 { m }
    }

    #[must_use]
    pub fn transpose(&self) -> Rot3 {
        let a = &self.m;
        Rot3 {
            m: [
                [a[0][0], a[1][0], a[2][0]],
                [a[0][1], a[1][1], a[2][1]],
                [a[0][2], a[1][2], a[2][2]],
            ],
        }
    }

    /// Roll, pitch and yaw (rad), as `from_rpy` takes them.
    #[must_use]
    pub fn rpy(&self) -> (f64, f64, f64) {
        let m = &self.m;
        let pitch = (-m[2][0]).clamp(-1.0, 1.0).asin();
        (m[2][1].atan2(m[2][2]), pitch, m[1][0].atan2(m[0][0]))
    }

    /// The rotation as a unit quaternion: (x, y, z, w).
    #[must_use]
    pub fn quaternion(&self) -> [f64; 4] {
        let m = &self.m;
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

/// Where something is and how it's turned: the child frame in its parent's.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose {
    pub position: Vec3,
    pub rotation: Rot3,
}

impl Pose {
    pub const IDENTITY: Pose = Pose {
        position: Vec3::ZERO,
        rotation: Rot3::IDENTITY,
    };

    #[must_use]
    pub fn new(position: Vec3, rotation: Rot3) -> Self {
        Self { position, rotation }
    }

    /// x y z roll pitch yaw, as SDF's `<pose>` and vehicle.yaml's xyz and rpy.
    #[must_use]
    pub fn from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> Self {
        Self::new(
            Vec3::new(xyz[0], xyz[1], xyz[2]),
            Rot3::from_rpy(rpy[0], rpy[1], rpy[2]),
        )
    }

    /// A point in this frame, in the parent's.
    #[must_use]
    pub fn apply(&self, p: Vec3) -> Vec3 {
        self.rotation.apply(p) + self.position
    }

    /// This pose, then `inner` inside it: parent <- self <- inner.
    #[must_use]
    pub fn then(&self, inner: &Pose) -> Pose {
        Pose::new(
            self.apply(inner.position),
            self.rotation.then(&inner.rotation),
        )
    }

    #[must_use]
    pub fn inverse(&self) -> Pose {
        let r = self.rotation.transpose();
        Pose::new(-r.apply(self.position), r)
    }
}

/// Wrap an angle into (-pi, pi].
#[must_use]
pub fn wrap(angle: f64) -> f64 {
    let a = angle.rem_euclid(std::f64::consts::TAU);
    if a > std::f64::consts::PI {
        a - std::f64::consts::TAU
    } else {
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-9
    }

    #[test]
    fn yaw_turns_x_into_y() {
        let r = Rot3::from_yaw(std::f64::consts::FRAC_PI_2);
        assert!(close(
            r.apply(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.0, 1.0, 0.0)
        ));
    }

    #[test]
    fn a_pose_and_its_inverse_cancel() {
        let p = Pose::from_xyz_rpy([1.0, 2.0, 3.0], [0.1, -0.2, 0.3]);
        let q = Vec3::new(-4.0, 5.0, 0.5);
        assert!(close(p.inverse().apply(p.apply(q)), q));
    }

    #[test]
    fn a_yaw_quaternion() {
        let q = Rot3::from_yaw(1.0).quaternion();
        assert!((q[2] - 0.5f64.sin()).abs() < 1e-12 && (q[3] - 0.5f64.cos()).abs() < 1e-12);
    }

    #[test]
    fn wrap_keeps_angles_in_range() {
        assert!((wrap(3.0 * std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-12);
        assert!((wrap(-0.5) + 0.5).abs() < 1e-12);
    }
}
