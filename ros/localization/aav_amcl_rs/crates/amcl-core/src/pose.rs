//! Poses on a flat map (x, y, yaw), and the units the settings are in.
//!
//! REP-103: metres and radians, x forward, y left, yaw counter-clockwise from x.

use std::f64::consts::{PI, TAU};

/// A length, in metres.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Metres(pub f64);

/// An angle, in radians.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Radians(pub f64);

/// Where something is on a flat map: x and y in metres, yaw in radians.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2 {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad, + left of x.
    pub yaw: f64,
}

impl Pose2 {
    /// The pose (`x`, `y`, `yaw`).
    #[must_use]
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    /// `self ∘ other`: `other`, given in `self`'s frame, put into the frame `self` is in.
    #[must_use]
    pub fn compose(&self, other: &Pose2) -> Pose2 {
        let (x, y) = self.apply(other.x, other.y);
        Pose2::new(x, y, wrap(self.yaw + other.yaw))
    }

    /// The pose that undoes this one.
    #[must_use]
    pub fn inverse(&self) -> Pose2 {
        let (s, c) = self.yaw.sin_cos();
        Pose2::new(
            -(c * self.x + s * self.y),
            s * self.x - c * self.y,
            wrap(-self.yaw),
        )
    }

    /// Where `other` is, seen from `self`: `self⁻¹ ∘ other`.
    #[must_use]
    pub fn between(&self, other: &Pose2) -> Pose2 {
        self.inverse().compose(other)
    }

    /// A point given in this pose's frame, in the frame the pose is in.
    #[must_use]
    pub fn apply(&self, x: f64, y: f64) -> (f64, f64) {
        let (s, c) = self.yaw.sin_cos();
        (self.x + c * x - s * y, self.y + s * x + c * y)
    }

    /// Part way from `self` to `other`: `t` = 0 is `self`, 1 is `other`; yaw the short way.
    #[must_use]
    pub fn lerp(&self, other: &Pose2, t: f64) -> Pose2 {
        Pose2::new(
            self.x + t * (other.x - self.x),
            self.y + t * (other.y - self.y),
            wrap(self.yaw + t * wrap(other.yaw - self.yaw)),
        )
    }

    /// How far apart two poses are: (metres, radians, the turn's size).
    #[must_use]
    pub fn distance(&self, other: &Pose2) -> (Metres, Radians) {
        (
            Metres((other.x - self.x).hypot(other.y - self.y)),
            Radians(wrap(other.yaw - self.yaw).abs()),
        )
    }
}

/// An angle in (-π, π].
#[must_use]
pub fn wrap(a: f64) -> f64 {
    let a = (a + PI).rem_euclid(TAU) - PI;
    if a <= -PI {
        a + TAU
    } else {
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_keeps_angles_in_range() {
        assert!((wrap(3.0 * PI) - PI).abs() < 1e-12);
        assert!((wrap(-PI) - PI).abs() < 1e-12);
        assert!((wrap(0.5) - 0.5).abs() < 1e-12);
        assert!((wrap(-7.0) - (-7.0 + TAU)).abs() < 1e-12);
    }

    #[test]
    fn between_undoes_compose() {
        let a = Pose2::new(1.0, -2.0, 0.7);
        let b = Pose2::new(0.5, 0.3, -0.2);
        let c = a.compose(&b);
        let back = a.between(&c);
        assert!((back.x - b.x).abs() < 1e-12 && (back.y - b.y).abs() < 1e-12);
        assert!((back.yaw - b.yaw).abs() < 1e-12);
    }
}
