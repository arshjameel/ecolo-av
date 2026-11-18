//! The odometry motion model: each particle moves as the odometry did, plus noise.
//!
//! nav2_amcl's `DifferentialMotionModel` (*Probabilistic Robotics*, table 5.6,
//! `sample_motion_model_odometry`): the move is a turn (rot1), a straight line (trans) and
//! a turn (rot2), each with its own noise. No sideways slip: a car can't slide, so the
//! omni model's alpha5 has no use here.

use crate::{
    filter::Particle,
    pose::{wrap, Pose2},
    rng::Rng,
};

/// nav2's alpha1-4: how wrong odometry may be. Each noise's standard deviation is the
/// square root of a sum of alpha times a squared move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DifferentialMotion {
    /// Heading noise from turning (rad² per rad²).
    pub alpha1: f64,
    /// Heading noise from driving (rad² per m²).
    pub alpha2: f64,
    /// Distance noise from driving (m² per m²).
    pub alpha3: f64,
    /// Distance noise from turning (m² per rad²).
    pub alpha4: f64,
}

impl Default for DifferentialMotion {
    /// nav2's defaults, 0.2 each.
    fn default() -> Self {
        Self {
            alpha1: 0.2,
            alpha2: 0.2,
            alpha3: 0.2,
            alpha4: 0.2,
        }
    }
}

/// One odometry move, split as the model does.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Move {
    /// The turn before driving, rad.
    pub rot1: f64,
    /// The distance driven, m (never negative: reversing is a half turn in rot1).
    pub trans: f64,
    /// The turn after, rad.
    pub rot2: f64,
}

impl Move {
    /// The move from odometry pose `from` to `to` (both in the odometry's frame).
    #[must_use]
    pub fn between(from: &Pose2, to: &Pose2) -> Self {
        let (dx, dy) = (to.x - from.x, to.y - from.y);
        let trans = dx.hypot(dy);
        // No bearing between two poses this close (a turn on the spot, or parked).
        let rot1 = if trans < 0.01 {
            0.0
        } else {
            wrap(dy.atan2(dx) - from.yaw)
        };
        let rot2 = wrap(wrap(to.yaw - from.yaw) - rot1);
        Self { rot1, trans, rot2 }
    }
}

/// A turn's size for the noise, forward and backward alike: reversing has rot1 near π,
/// which is no turn at all.
fn turn_size(rot: f64) -> f64 {
    rot.abs().min(wrap(rot - std::f64::consts::PI).abs())
}

impl DifferentialMotion {
    /// Moves every particle as odometry moved from `from` to `to`, each with its own noise.
    pub fn apply(&self, particles: &mut [Particle], from: &Pose2, to: &Pose2, rng: &mut Rng) {
        let m = Move::between(from, to);
        let (r1, r2) = (turn_size(m.rot1), turn_size(m.rot2));
        let t2 = m.trans * m.trans;
        let sd_rot1 = (self.alpha1 * r1 * r1 + self.alpha2 * t2).sqrt();
        let sd_trans = (self.alpha3 * t2 + self.alpha4 * (r1 * r1 + r2 * r2)).sqrt();
        let sd_rot2 = (self.alpha1 * r2 * r2 + self.alpha2 * t2).sqrt();
        for p in particles {
            let rot1 = wrap(m.rot1 - sd_rot1 * rng.normal());
            let trans = m.trans - sd_trans * rng.normal();
            let rot2 = wrap(m.rot2 - sd_rot2 * rng.normal());
            let heading = p.pose.yaw + rot1;
            p.pose.x += trans * heading.cos();
            p.pose.y += trans * heading.sin();
            p.pose.yaw = wrap(heading + rot2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_move_splits_into_turn_drive_turn() {
        let m = Move::between(&Pose2::new(0.0, 0.0, 0.0), &Pose2::new(1.0, 1.0, 0.5));
        assert!((m.rot1 - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
        assert!((m.trans - 2f64.sqrt()).abs() < 1e-12);
        assert!((m.rot2 - (0.5 - std::f64::consts::FRAC_PI_4)).abs() < 1e-12);
    }

    #[test]
    fn reversing_is_no_turn_for_the_noise() {
        let m = Move::between(&Pose2::new(0.0, 0.0, 0.0), &Pose2::new(-1.0, 0.0, 0.0));
        assert!((m.rot1.abs() - std::f64::consts::PI).abs() < 1e-12);
        assert!(turn_size(m.rot1) < 1e-12 && turn_size(m.rot2) < 1e-12);
    }

    #[test]
    fn no_noise_means_the_odometry_exactly() {
        let model = DifferentialMotion {
            alpha1: 0.0,
            alpha2: 0.0,
            alpha3: 0.0,
            alpha4: 0.0,
        };
        let mut ps = vec![Particle::new(
            Pose2::new(5.0, 5.0, std::f64::consts::FRAC_PI_2),
            1.0,
        )];
        let mut rng = Rng::new(1);
        // Odometry: 1 m ahead and a 0.3 rad turn left; the particle faces +y, so it
        // goes 1 m up the map.
        model.apply(
            &mut ps,
            &Pose2::new(0.0, 0.0, 0.0),
            &Pose2::new(1.0, 0.0, 0.3),
            &mut rng,
        );
        let p = ps[0].pose;
        assert!(
            (p.x - 5.0).abs() < 1e-12 && (p.y - 6.0).abs() < 1e-12,
            "{p:?}"
        );
        assert!((p.yaw - (std::f64::consts::FRAC_PI_2 + 0.3)).abs() < 1e-12);
    }
}
