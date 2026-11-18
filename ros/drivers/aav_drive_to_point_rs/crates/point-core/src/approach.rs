//! The speed for driving to B: speed up gently, hold, then slow down by the distance left
//! (`approach.py`).
//!
//! ```text
//!  speed       hold (cruise)
//!    ^      +-------------+
//!    |     /               \   v = sqrt(2 a (d - lead v))
//!    |    /  up at accel    \
//!    +---/-------------------\---> distance left to B
//!       A                     B: stop (d <= stop_band)
//! ```
//!
//! Slowing follows the distance left, not the time, so a late brake corrects itself. With
//! a stopping law ([`StopCurve`], `stop_law:=hybrid`) the slowing is its `cap(d)` instead.

use std::fmt;

use ego_core::stop_law::StopCurve;

use crate::{py_max, py_min, py_repr};

/// Of the measured braking: docs/tests.md 5.2.
pub const BRAKE_FRACTION: f64 = 0.5;

/// The feedback's phase (`aav_interfaces/action/DriveToPoint`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Turning towards B, before the line.
    Turn,
    /// Speeding up.
    Accelerate,
    /// At the cruise speed.
    Hold,
    /// Slowing for B.
    Decelerate,
    /// At B (or past it): zero.
    Stopping,
}

impl Phase {
    /// The number in the action's feedback (`PHASE_TURN` 0 ... `PHASE_STOPPING` 4).
    #[must_use]
    pub fn code(self) -> u8 {
        match self {
            Phase::Turn => 0,
            Phase::Accelerate => 1,
            Phase::Hold => 2,
            Phase::Decelerate => 3,
            Phase::Stopping => 4,
        }
    }
}

/// A plan's numbers refused.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ApproachError {
    /// `max_brake` must be finite and above 0.
    MaxBrake(f64),
    /// The braking fraction must be in (0, 1].
    Fraction(f64),
    /// The cruise speed, acceleration or deceleration (its name) must be above 0.
    NotPositive(&'static str, f64),
    /// lead, stop_band and creep must not be negative; ahead above 0.
    Margins,
}

impl fmt::Display for ApproachError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ApproachError::MaxBrake(v) => {
                write!(f, "max_brake must be above 0 m/s^2 (got {})", py_repr(*v))
            }
            ApproachError::Fraction(v) => {
                write!(
                    f,
                    "the braking fraction must be in (0, 1] (got {})",
                    py_repr(*v)
                )
            }
            ApproachError::NotPositive(name, v) => {
                write!(f, "{name} must be above 0 (got {})", py_repr(*v))
            }
            ApproachError::Margins => {
                f.write_str("lead, stop_band and creep must not be negative; ahead above 0")
            }
        }
    }
}

impl std::error::Error for ApproachError {}

/// The deceleration to plan with (m/s²): a fraction of full braking.
///
/// # Errors
///
/// [`ApproachError`] for a `max_brake` not above 0, or a fraction outside (0, 1].
pub fn planned_deceleration(max_brake: f64, fraction: f64) -> Result<f64, ApproachError> {
    if !(max_brake.is_finite() && max_brake > 0.0) {
        return Err(ApproachError::MaxBrake(max_brake));
    }
    if !(fraction > 0.0 && fraction <= 1.0) {
        return Err(ApproachError::Fraction(fraction));
    }
    Ok(max_brake * fraction)
}

/// The metres it takes to stop from `speed` at `deceleration`.
#[must_use]
pub fn stopping_distance(speed: f64, deceleration: f64) -> f64 {
    speed * speed / (2.0 * deceleration)
}

/// One drive's speed plan. Speeds m/s, accelerations m/s², distances m, times s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Approach {
    /// The speed to hold.
    pub cruise: f64,
    /// How fast to speed up (gently).
    pub accel: f64,
    /// How fast to slow down: half the measured braking.
    pub decel: f64,
    /// The car's delay in braking: start slowing this much sooner.
    pub lead: f64,
    /// This close to B (or past it), command zero.
    pub stop_band: f64,
    /// Never command more than this above the measured speed.
    pub ahead: f64,
    /// The least it asks for short of B (a speed loop's deadband).
    pub creep: f64,
    /// The stopping law instead of sqrt(2 a (d - lead v)).
    pub curve: Option<StopCurve>,
}

impl Approach {
    /// A plan, its numbers checked; `ahead` 0.5 m/s, as the Python's default.
    ///
    /// # Errors
    ///
    /// [`ApproachError`] for a cruise, accel or decel not above 0, or a negative margin.
    pub fn new(
        cruise: f64,
        accel: f64,
        decel: f64,
        lead: f64,
        stop_band: f64,
        creep: f64,
        curve: Option<StopCurve>,
    ) -> Result<Self, ApproachError> {
        let plan = Approach {
            cruise,
            accel,
            decel,
            lead,
            stop_band,
            ahead: 0.5,
            creep,
            curve,
        };
        for (name, value) in [("cruise", cruise), ("accel", accel), ("decel", decel)] {
            if !(value.is_finite() && value > 0.0) {
                return Err(ApproachError::NotPositive(name, value));
            }
        }
        if lead < 0.0 || stop_band < 0.0 || plan.ahead <= 0.0 || creep < 0.0 {
            return Err(ApproachError::Margins);
        }
        Ok(plan)
    }

    /// The fastest speed from which slowing at `decel` (or the curve) stops at B.
    #[must_use]
    pub fn braking_speed(&self, left: f64, measured: f64) -> f64 {
        if let Some(curve) = &self.curve {
            return curve.cap(left);
        }
        let room = left - py_max(measured, 0.0) * self.lead;
        if room > 0.0 {
            (2.0 * self.decel * room).sqrt()
        } else {
            0.0
        }
    }

    /// (the speed to command, the phase).
    ///
    /// `previous`: the last speed commanded. `left`: metres to B (negative once past it).
    /// `measured`: the car's speed. `limit`: a cap for this tick (a tight turn).
    #[must_use]
    pub fn step(
        &self,
        previous: f64,
        dt: f64,
        left: f64,
        measured: f64,
        limit: f64,
    ) -> (f64, Phase) {
        if left <= self.stop_band {
            return (0.0, Phase::Stopping);
        }
        let up = py_min(
            previous + self.accel * py_max(dt, 0.0),
            py_max(measured, 0.0) + self.ahead,
        );
        let brake = self.braking_speed(left, measured);
        let cruise = py_min(self.cruise, limit);
        let mut speed = py_min(py_min(up, cruise), brake);
        let phase = if brake <= py_min(up, cruise) {
            // Short of B but nearly stopped: keep rolling slowly, or it would stop before
            // B and stay there.
            speed = py_max(speed, py_min(self.creep, self.reach(left)));
            Phase::Decelerate
        } else if cruise <= up {
            Phase::Hold
        } else {
            Phase::Accelerate
        };
        (py_max(speed, 0.0), phase)
    }

    /// The speed that stops exactly at B with no lead: the creep's own limit.
    #[must_use]
    pub fn reach(&self, left: f64) -> f64 {
        if left <= 0.0 {
            return 0.0;
        }
        if let Some(curve) = &self.curve {
            return curve.cap(left);
        }
        (2.0 * self.decel * left).sqrt()
    }

    /// The seconds a perfect car takes for `distance` m (for a time limit).
    #[must_use]
    pub fn ideal_time(&self, distance: f64) -> f64 {
        let up = stopping_distance(self.cruise, self.accel);
        let down = stopping_distance(self.cruise, self.decel);
        if up + down >= distance {
            // A triangle: never reaches cruise.
            let peak =
                (2.0 * distance * self.accel * self.decel / (self.accel + self.decel)).sqrt();
            return peak / self.accel + peak / self.decel;
        }
        self.cruise / self.accel + self.cruise / self.decel + (distance - up - down) / self.cruise
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan() -> Approach {
        Approach::new(1.0, 0.5, 1.5, 0.0, 0.05, 0.1, None).unwrap()
    }

    #[test]
    fn it_speeds_up_holds_and_stops() {
        let p = plan();
        assert_eq!(
            p.step(0.0, 0.1, 50.0, 0.0, f64::INFINITY).1,
            Phase::Accelerate
        );
        assert_eq!(p.step(1.0, 0.1, 50.0, 1.0, f64::INFINITY).1, Phase::Hold);
        assert_eq!(
            p.step(1.0, 0.1, 0.2, 1.0, f64::INFINITY).1,
            Phase::Decelerate
        );
        assert_eq!(
            p.step(1.0, 0.1, 0.0, 1.0, f64::INFINITY),
            (0.0, Phase::Stopping)
        );
    }

    #[test]
    fn bad_numbers_are_refused() {
        assert!(planned_deceleration(0.0, 0.5).is_err());
        assert!(planned_deceleration(3.0, 1.5).is_err());
        assert!(Approach::new(0.0, 0.5, 1.5, 0.0, 0.05, 0.1, None).is_err());
        assert!(Approach::new(1.0, 0.5, 1.5, -1.0, 0.05, 0.1, None).is_err());
    }
}
