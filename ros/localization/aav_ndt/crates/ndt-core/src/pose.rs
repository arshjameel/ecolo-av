//! Poses on a flat lot (x, y, yaw), and where the lidar sits on the car.
//!
//! REP-103: metres and radians, x forward, y left, z up, yaw counter-clockwise from x.

use nalgebra::{Isometry3, Translation3, UnitQuaternion};

use crate::Point;

/// Where something is on a flat lot: x and y in metres, yaw in radians. Height, roll and
/// pitch are the map's (0: `base_link` on the ground).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2 {
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    /// `self ∘ other`: `other`, as seen from `self`, put into `self`'s frame.
    pub fn compose(&self, other: &Pose2) -> Pose2 {
        let (s, c) = self.yaw.sin_cos();
        Pose2 {
            x: self.x + c * other.x - s * other.y,
            y: self.y + s * other.x + c * other.y,
            yaw: wrap(self.yaw + other.yaw),
        }
    }

    /// The pose that undoes this one: `self ∘ self.inverse()` is no move.
    pub fn inverse(&self) -> Pose2 {
        let (s, c) = self.yaw.sin_cos();
        Pose2 {
            x: -(c * self.x + s * self.y),
            y: s * self.x - c * self.y,
            yaw: wrap(-self.yaw),
        }
    }

    /// Where `other` is, seen from `self`: `self⁻¹ ∘ other`.
    pub fn between(&self, other: &Pose2) -> Pose2 {
        self.inverse().compose(other)
    }

    /// A point in this pose's frame, moved into the frame the pose is in. Height is kept.
    pub fn apply(&self, p: &Point) -> Point {
        let (s, c) = self.yaw.sin_cos();
        Point::new(self.x + c * p.x - s * p.y, self.y + s * p.x + c * p.y, p.z)
    }

    /// Part way from `self` to `other`: `t` = 0 is `self`, 1 is `other`. Position on a
    /// straight line, yaw the short way round.
    pub fn lerp(&self, other: &Pose2, t: f64) -> Pose2 {
        Pose2 {
            x: self.x + t * (other.x - self.x),
            y: self.y + t * (other.y - self.y),
            yaw: wrap(self.yaw + t * wrap(other.yaw - self.yaw)),
        }
    }

    /// How far apart two poses are: (metres, radians, the turn's size).
    pub fn distance(&self, other: &Pose2) -> (f64, f64) {
        (
            (other.x - self.x).hypot(other.y - self.y),
            wrap(other.yaw - self.yaw).abs(),
        )
    }

    /// The same pose with every number finite.
    pub fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.yaw.is_finite()
    }
}

/// An angle in (-π, π].
pub fn wrap(angle: f64) -> f64 {
    let a = angle.rem_euclid(std::f64::consts::TAU);
    if a > std::f64::consts::PI {
        a - std::f64::consts::TAU
    } else {
        a
    }
}

/// Where the lidar sits on the car: `base_link → lidar_link` (`vehicle.yaml`'s
/// `lidar_link`, passed by the launch file).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mount(Isometry3<f64>);

impl Mount {
    /// From `vehicle.yaml`'s `xyz` (m) and `rpy` (rad: roll about x, then pitch about y,
    /// then yaw about z, as URDF).
    pub fn from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> Self {
        let [x, y, z] = xyz;
        let [roll, pitch, yaw] = rpy;
        Self(Isometry3::from_parts(
            Translation3::new(x, y, z),
            UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        ))
    }

    /// A lidar point, moved into `base_link`.
    pub fn to_base_link(&self, p: &Point) -> Point {
        self.0.transform_point(p)
    }
}

impl Default for Mount {
    fn default() -> Self {
        Self(Isometry3::identity())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn close(a: &Pose2, b: &Pose2) -> bool {
        let (m, r) = a.distance(b);
        m < 1e-12 && r < 1e-12
    }

    #[test]
    fn a_pose_and_its_inverse_cancel() {
        let p = Pose2::new(3.0, -2.0, 0.7);
        assert!(close(&p.compose(&p.inverse()), &Pose2::default()));
        assert!(close(&p.inverse().compose(&p), &Pose2::default()));
    }

    #[test]
    fn composing_moves_in_the_first_frame() {
        let facing_north = Pose2::new(1.0, 1.0, PI / 2.0);
        let ahead = Pose2::new(2.0, 0.0, 0.0);
        assert!(close(
            &facing_north.compose(&ahead),
            &Pose2::new(1.0, 3.0, PI / 2.0)
        ));
    }

    #[test]
    fn between_undoes_compose() {
        let a = Pose2::new(1.0, 2.0, 0.3);
        let b = Pose2::new(-4.0, 0.5, -2.9);
        assert!(close(&a.compose(&a.between(&b)), &b));
    }

    #[test]
    fn lerp_turns_the_short_way() {
        let a = Pose2::new(0.0, 0.0, 3.0);
        let b = Pose2::new(2.0, 0.0, -3.0);
        let mid = a.lerp(&b, 0.5);
        assert!((mid.x - 1.0).abs() < 1e-12);
        assert!((mid.yaw.abs() - PI).abs() < 1e-9);
    }

    #[test]
    fn wrap_keeps_angles_in_one_turn() {
        assert!((wrap(3.0 * PI) - PI).abs() < 1e-12);
        assert!((wrap(-0.5) + 0.5).abs() < 1e-12);
    }

    #[test]
    fn the_mount_lifts_and_moves_points() {
        let mount = Mount::from_xyz_rpy([0.9, 0.0, 1.8], [0.0, 0.0, PI / 2.0]);
        let p = mount.to_base_link(&Point::new(1.0, 0.0, 0.0));
        assert!((p - Point::new(0.9, 1.0, 1.8)).norm() < 1e-12);
    }
}
