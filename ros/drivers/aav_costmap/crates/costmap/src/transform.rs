//! Poses on the ground (x, y, yaw) and 3D transforms (a sensor's mount).

/// A pose on the ground: where a frame's origin is, and which way its x axis points.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2 {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad, from the parent's x axis, positive to the left.
    pub yaw: f64,
}

impl Pose2 {
    /// A pose.
    #[must_use]
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    /// The pose that changes nothing.
    pub const IDENTITY: Self = Self::new(0.0, 0.0, 0.0);

    /// A point in this pose's frame, in the parent's.
    #[must_use]
    pub fn apply(&self, (x, y): (f64, f64)) -> (f64, f64) {
        let (s, c) = self.yaw.sin_cos();
        (self.x + c * x - s * y, self.y + s * x + c * y)
    }

    /// `self` then `child`: the child's pose in this pose's parent.
    #[must_use]
    pub fn then(&self, child: &Pose2) -> Pose2 {
        let (x, y) = self.apply((child.x, child.y));
        Pose2::new(x, y, self.yaw + child.yaw)
    }

    /// The parent's pose in this frame.
    #[must_use]
    pub fn inverse(&self) -> Pose2 {
        let (s, c) = self.yaw.sin_cos();
        Pose2::new(-c * self.x - s * self.y, s * self.x - c * self.y, -self.yaw)
    }

    /// Between `self` (t = 0) and `other` (t = 1); the yaw the short way round.
    #[must_use]
    pub fn lerp(&self, other: &Pose2, t: f64) -> Pose2 {
        let turn = (other.yaw - self.yaw)
            .sin()
            .atan2((other.yaw - self.yaw).cos());
        Pose2::new(
            self.x + t * (other.x - self.x),
            self.y + t * (other.y - self.y),
            self.yaw + t * turn,
        )
    }
}

/// A rigid transform in 3D: a rotation (unit quaternion x, y, z, w), then a translation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform3 {
    /// m.
    pub translation: [f64; 3],
    /// x, y, z, w.
    pub rotation: [f64; 4],
}

impl Transform3 {
    /// The transform that changes nothing.
    pub const IDENTITY: Self = Self {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };

    /// A pose on the ground, at z = 0.
    #[must_use]
    pub fn from_pose2(p: &Pose2) -> Self {
        let (s, c) = (p.yaw / 2.0).sin_cos();
        Self {
            translation: [p.x, p.y, 0.0],
            rotation: [0.0, 0.0, s, c],
        }
    }

    /// A point in the child frame, in the parent's.
    #[must_use]
    pub fn apply(&self, v: [f64; 3]) -> [f64; 3] {
        let [qx, qy, qz, w] = self.rotation;
        let q = [qx, qy, qz];
        // v + 2w (q x v) + 2 q x (q x v)
        let t = cross(q, v).map(|c| 2.0 * c);
        let u = cross(q, t);
        let [x, y, z] = self.translation;
        [
            x + v[0] + w * t[0] + u[0],
            y + v[1] + w * t[1] + u[1],
            z + v[2] + w * t[2] + u[2],
        ]
    }

    /// `self` then `child`: the child's transform in this one's parent.
    #[must_use]
    pub fn then(&self, child: &Transform3) -> Transform3 {
        let [ax, ay, az, aw] = self.rotation;
        let [bx, by, bz, bw] = child.rotation;
        Transform3 {
            translation: self.apply(child.translation),
            rotation: [
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ],
        }
    }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn near(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    #[test]
    fn a_quarter_turn_moves_x_to_y() {
        let p = Pose2::new(1.0, 2.0, FRAC_PI_2);
        let (x, y) = p.apply((1.0, 0.0));
        assert!((x - 1.0).abs() < 1e-12 && (y - 3.0).abs() < 1e-12);
        let t = Transform3::from_pose2(&p);
        assert!(near(t.apply([1.0, 0.0, 0.5]), [1.0, 3.0, 0.5]));
    }

    #[test]
    fn inverse_and_then_undo_each_other() {
        let p = Pose2::new(3.0, -1.0, 0.7);
        let back = p.then(&p.inverse());
        assert!(back.x.abs() < 1e-12 && back.y.abs() < 1e-12 && back.yaw.abs() < 1e-12);
    }

    #[test]
    fn transforms_chain_as_poses_do() {
        let a = Pose2::new(1.0, 2.0, 0.3);
        let b = Pose2::new(-0.5, 0.25, -1.1);
        let whole = Transform3::from_pose2(&a).then(&Transform3::from_pose2(&b));
        let ab = a.then(&b);
        let (x, y) = ab.apply((2.0, 1.0));
        assert!(near(whole.apply([2.0, 1.0, 0.0]), [x, y, 0.0]));
    }

    #[test]
    fn lerp_goes_the_short_way_round() {
        let a = Pose2::new(0.0, 0.0, 3.0);
        let b = Pose2::new(2.0, 0.0, -3.0);
        let m = a.lerp(&b, 0.5);
        assert!((m.x - 1.0).abs() < 1e-12);
        assert!(
            (m.yaw.cos() + 1.0).abs() < 0.01,
            "near pi, not 0: {}",
            m.yaw
        );
    }
}
