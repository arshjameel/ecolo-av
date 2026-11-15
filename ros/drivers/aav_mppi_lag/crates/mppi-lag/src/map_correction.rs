//! Follow in odom, correct slowly (TODO 186): a complementary filter on map -> odom.
//!
//! The follower drives in `odom` on the smooth local odometry (wheels and IMU: no jumps).
//! The path comes in `map`, and the global EKF's map -> odom jumps with every GPS fix
//! (the simulator: about 0.09 m sideways, 5 times a second). Moving each new path into
//! odom with the newest map -> odom moves the path under the car by that much: a fake
//! cross-track error the follower steers at.
//!
//! So the follower keeps its own map -> odom, the **held** one, and moves it toward the
//! newest at a bounded rate: the fast part of the pose comes from odometry, the slow part
//! from the map (Higgins 1975's complementary filter, with a rate limit as the low pass).
//!
//! ```text
//!   newest map -> odom ──(jumps 0.09 m at 5 Hz)──┐
//!                                                ▼
//!   held map -> odom ◄── at most `speed` m/s and `turn_rate` rad/s, measured at the car
//!        │               snaps at once past `snap_distance` or `snap_angle` (relocalised)
//!        ▼
//!   path (map) ──► path (odom) ──► the follower; the path moves as the held one moves
//! ```
//!
//! The gap is measured **at the car** (where the car is in map by the held transform,
//! against the newest), not at odom's origin: a turn of map -> odom about an origin 100 m
//! away would otherwise read as a 100 m x angle step.
//!
//! [`SlewLimits::OFF`] (speed or turn rate 0): the old way, each new path moved into odom
//! by the newest map -> odom, nothing between.

use crate::model::{wrap, Pose};

/// A pose in map, given odom's pose in map (the map -> odom transform), into odom.
pub fn into_odom(p: &Pose, odom_in_map: &Pose) -> Pose {
    let (s, c) = odom_in_map.yaw.sin_cos();
    let (dx, dy) = (p.x - odom_in_map.x, p.y - odom_in_map.y);
    Pose::new(
        c * dx + s * dy,
        -s * dx + c * dy,
        wrap(p.yaw - odom_in_map.yaw),
    )
}

/// A pose in odom, given odom's pose in map, into map.
pub fn into_map(p: &Pose, odom_in_map: &Pose) -> Pose {
    let (s, c) = odom_in_map.yaw.sin_cos();
    Pose::new(
        odom_in_map.x + c * p.x - s * p.y,
        odom_in_map.y + s * p.x + c * p.y,
        wrap(p.yaw + odom_in_map.yaw),
    )
}

/// How fast the held map -> odom may move, and when it jumps instead.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlewLimits {
    /// m/s at the car (`map_correction_speed`). 0: off.
    pub speed: f64,
    /// rad/s (`map_correction_turn_rate`). 0: off.
    pub turn_rate: f64,
    /// m: a bigger gap is a relocalisation, taken at once (`map_correction_snap_distance`).
    pub snap_distance: f64,
    /// rad: likewise (`map_correction_snap_angle`).
    pub snap_angle: f64,
}

impl SlewLimits {
    /// The old way: each new path by the newest map -> odom.
    pub const OFF: Self = Self {
        speed: 0.0,
        turn_rate: 0.0,
        snap_distance: 2.0,
        snap_angle: 10.0 * std::f64::consts::PI / 180.0,
    };

    /// True if the held map -> odom slews (both rates over 0).
    pub fn on(&self) -> bool {
        self.speed > 0.0 && self.turn_rate > 0.0
    }
}

impl Default for SlewLimits {
    /// 0.1 m/s and 1 degree/s; snap past 2 m or 10 degrees (README: why these).
    fn default() -> Self {
        Self {
            speed: 0.1,
            turn_rate: 1.0_f64.to_radians(),
            ..Self::OFF
        }
    }
}

/// How the held map -> odom moved on one update.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Correction {
    /// The first map -> odom, or no path running: taken as it is.
    Taken,
    /// Moved toward the newest, within the limits (or already there).
    Slewed,
    /// The gap was over a snap limit: taken at once. The gap at the car (m, rad).
    Snapped {
        /// m.
        distance: f64,
        /// rad.
        angle: f64,
    },
}

/// The held map -> odom moved: from one odom pose in map to another. A pose in odom under
/// the old one is the same place in map as [`Shift::apply`] of it under the new one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Shift {
    /// odom's pose in map before.
    pub from: Pose,
    /// odom's pose in map after.
    pub to: Pose,
}

impl Shift {
    /// Where a pose in odom (placed by `from`) lies in odom by `to`.
    pub fn apply(&self, p: &Pose) -> Pose {
        into_odom(&into_map(p, &self.from), &self.to)
    }
}

/// The held map -> odom: odom's pose in map, moved toward the newest within [`SlewLimits`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapCorrection {
    limits: SlewLimits,
    held: Option<Pose>,
}

impl MapCorrection {
    /// Nothing held yet.
    pub fn new(limits: SlewLimits) -> Self {
        Self { limits, held: None }
    }

    /// The limits.
    pub fn limits(&self) -> SlewLimits {
        self.limits
    }

    /// odom's pose in map as held (`None` before the first).
    pub fn held(&self) -> Option<Pose> {
        self.held
    }

    /// The map -> odom to move a new path into odom with. With the limits off, or no path
    /// running (`driving` false), or nothing held yet: the newest, which is then held.
    /// Otherwise the held one: a new path lands where the last one was moved to.
    pub fn for_new_path(&mut self, newest: Pose, driving: bool) -> Pose {
        match self.held {
            Some(held) if driving && self.limits.on() => held,
            _ => {
                self.held = Some(newest);
                newest
            }
        }
    }

    /// One step of `dt` s toward `newest`, with the car at `car` in odom. Returns how it
    /// moved and the shift to move the running path by (`None`: it didn't move). Does
    /// nothing with the limits off.
    pub fn update(&mut self, newest: Pose, car: &Pose, dt: f64) -> Option<(Correction, Shift)> {
        if !self.limits.on() {
            return None;
        }
        let Some(held) = self.held else {
            self.held = Some(newest);
            return None;
        };
        let at_held = into_map(car, &held);
        let at_newest = into_map(car, &newest);
        let (dx, dy) = (at_newest.x - at_held.x, at_newest.y - at_held.y);
        let distance = dx.hypot(dy);
        let angle = wrap(newest.yaw - held.yaw);
        let finite = [distance, angle, dt].iter().all(|v| v.is_finite());
        if !finite {
            return None;
        }
        let (correction, to) =
            if distance > self.limits.snap_distance || angle.abs() > self.limits.snap_angle {
                (Correction::Snapped { distance, angle }, newest)
            } else {
                let dt = dt.max(0.0);
                let step = self.limits.speed * dt;
                let f = if distance > step {
                    step / distance
                } else {
                    1.0
                };
                let turn = self.limits.turn_rate * dt;
                // Where the car is in map by the new held one: moved toward the newest.
                let car_map = Pose::new(
                    at_held.x + f * dx,
                    at_held.y + f * dy,
                    at_held.yaw + angle.clamp(-turn, turn),
                );
                // odom's pose in map that puts the car (in odom) there.
                let yaw = wrap(car_map.yaw - car.yaw);
                let (s, c) = yaw.sin_cos();
                let to = Pose::new(
                    car_map.x - (c * car.x - s * car.y),
                    car_map.y - (s * car.x + c * car.y),
                    yaw,
                );
                (Correction::Slewed, to)
            };
        self.held = Some(to);
        (to != held).then_some((correction, Shift { from: held, to }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    fn close(a: &Pose, b: &Pose) -> bool {
        (a.x - b.x).abs() < EPS && (a.y - b.y).abs() < EPS && wrap(a.yaw - b.yaw).abs() < EPS
    }

    #[test]
    fn into_odom_and_back() {
        let odom = Pose::new(3.0, -2.0, 0.7);
        let p = Pose::new(10.0, 4.0, -2.5);
        assert!(close(&into_map(&into_odom(&p, &odom), &odom), &p));
        // odom's origin in map is (3, -2).
        assert!(close(
            &into_odom(&Pose::new(3.0, -2.0, 0.7), &odom),
            &Pose::default()
        ));
    }

    #[test]
    fn off_takes_the_newest_for_each_path_and_never_moves() {
        let mut m = MapCorrection::new(SlewLimits::OFF);
        let a = Pose::new(0.0, 0.1, 0.0);
        let b = Pose::new(0.0, -0.1, 0.0);
        assert_eq!(m.for_new_path(a, true), a);
        assert_eq!(m.update(b, &Pose::default(), 0.05), None);
        assert_eq!(m.for_new_path(b, true), b);
    }

    #[test]
    fn a_new_path_while_driving_lands_by_the_held_one() {
        let mut m = MapCorrection::new(SlewLimits::default());
        let a = Pose::new(0.0, 0.0, 0.0);
        assert_eq!(m.for_new_path(a, false), a);
        let jumped = Pose::new(0.0, 0.09, 0.0);
        assert_eq!(m.for_new_path(jumped, true), a, "no jump under the car");
        // Not driving: take the newest at once.
        assert_eq!(m.for_new_path(jumped, false), jumped);
    }

    #[test]
    fn a_sideways_step_is_taken_at_the_speed_limit() {
        let mut m = MapCorrection::new(SlewLimits::default());
        m.for_new_path(Pose::default(), false);
        let newest = Pose::new(0.0, 0.5, 0.0);
        let car = Pose::new(40.0, 0.0, 0.0);
        let mut t: f64 = 0.0;
        while m.held().is_some_and(|h| (h.y - 0.5).abs() > 1e-9) {
            let (c, _) = m.update(newest, &car, 0.05).expect("it moves");
            assert_eq!(c, Correction::Slewed);
            t += 0.05;
            assert!(t < 10.0);
        }
        // 0.5 m at 0.1 m/s.
        assert!((t - 5.0).abs() < 0.051, "{t}");
        assert_eq!(m.update(newest, &car, 0.05), None, "there: nothing moves");
    }

    #[test]
    fn a_turn_is_slewed_about_the_car_not_odoms_origin() {
        let mut m = MapCorrection::new(SlewLimits::default());
        m.for_new_path(Pose::default(), false);
        // The newest says odom turned 2 degrees about the car, 100 m from odom's origin.
        let car = Pose::new(100.0, 0.0, 0.0);
        let angle = 2.0_f64.to_radians();
        let newest = Pose::new(100.0 - 100.0 * angle.cos(), -100.0 * angle.sin(), angle);
        assert!(close(
            &into_map(&car, &newest),
            &Pose::new(100.0, 0.0, angle)
        ));
        let (c, shift) = m.update(newest, &car, 0.1).expect("it moves");
        assert_eq!(
            c,
            Correction::Slewed,
            "3.5 m at odom's origin, 0 at the car"
        );
        let held = m.held().expect("held");
        let at = into_map(&car, &held);
        assert!((at.x - 100.0).abs() < 1e-9 && at.y.abs() < 1e-9, "{at:?}");
        assert!((held.yaw - 0.1_f64.to_radians()).abs() < 1e-12);
        // The path under the car turns about the car: the car's own place stays put.
        assert!(close(
            &shift.apply(&car),
            &Pose::new(100.0, 0.0, -0.1_f64.to_radians())
        ));
    }

    #[test]
    fn a_relocalisation_is_taken_at_once() {
        let mut m = MapCorrection::new(SlewLimits::default());
        m.for_new_path(Pose::default(), false);
        let far = Pose::new(3.0, 0.0, 0.0);
        let (c, shift) = m.update(far, &Pose::default(), 0.05).expect("it moves");
        assert!(matches!(c, Correction::Snapped { distance, .. } if (distance - 3.0).abs() < EPS));
        assert_eq!(shift.to, far);
        let turned = Pose::new(3.0, 0.0, 0.3);
        let (c, _) = m.update(turned, &Pose::default(), 0.05).expect("it moves");
        assert!(matches!(c, Correction::Snapped { angle, .. } if (angle - 0.3).abs() < EPS));
    }

    #[test]
    fn noise_is_held_to_the_rate() {
        // Jumps of 0.09 m either side, 5 times a second: the held one moves 2 cm at most
        // between fixes, and stays near the middle.
        let mut m = MapCorrection::new(SlewLimits::default());
        m.for_new_path(Pose::default(), false);
        let car = Pose::new(5.0, 0.0, 0.0);
        let mut worst: f64 = 0.0;
        for i in 0..200 {
            let side = if (i / 4) % 2 == 0 { 0.045 } else { -0.045 };
            let before = m.held().expect("held");
            m.update(Pose::new(0.0, side, 0.0), &car, 0.05);
            let after = m.held().expect("held");
            assert!(before.distance(&after) <= 0.1 * 0.05 + 1e-12);
            worst = worst.max(after.y.abs());
        }
        assert!(worst < 0.03, "{worst}");
    }

    #[test]
    fn a_bad_number_changes_nothing() {
        let mut m = MapCorrection::new(SlewLimits::default());
        m.for_new_path(Pose::default(), false);
        assert_eq!(
            m.update(Pose::new(f64::NAN, 0.0, 0.0), &Pose::default(), 0.05),
            None
        );
        assert_eq!(m.held(), Some(Pose::default()));
    }

    #[test]
    fn a_shift_moves_odom_poses_to_the_same_place_in_map() {
        let shift = Shift {
            from: Pose::new(1.0, 2.0, 0.1),
            to: Pose::new(1.1, 1.9, 0.12),
        };
        let p = Pose::new(7.0, -3.0, 1.0);
        assert!(close(
            &into_map(&shift.apply(&p), &shift.to),
            &into_map(&p, &shift.from)
        ));
    }
}
