//! How the car stops: one curve, S(v), for every stop (`aav_description/stop_law.py`).
//!
//! ```text
//! S(v) = v·τ + v²/(2a)        τ = ρ + a/(2j)
//! v_cap(d) = a·(√(τ² + 2d/a) − τ)     the speed that stops in d metres
//! ```
//!
//! Only what ego and its log use: the curve, [`StopCurve::from_car_model`] and
//! [`StopCurve::from_numbers`].

use std::fmt;

use car_model::{ActuatorConfig, SpeedLoopConfig};
use spine_core::speed_loop::SpeedLaw;

use crate::widen;

/// s: the input's age when a node acts on it, and one tick of each node on the way.
pub const INPUT_DELAY: f64 = 0.15;

/// One stopping law. Speeds m/s, distances m, `a` m/s², `j` m/s³, `rho` s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StopCurve {
    a: f64,
    j: f64,
    rho: f64,
}

/// Why a curve's numbers were refused.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CurveError {
    /// `a` must be finite and above 0.
    Braking(f64),
    /// `j` must be finite and above 0.
    Jerk(f64),
    /// `rho` must be finite and not negative.
    Delay(f64),
}

impl fmt::Display for CurveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CurveError::Braking(a) => write!(f, "a must be a finite number above 0 (got {a})"),
            CurveError::Jerk(j) => write!(f, "j must be a finite number above 0 (got {j})"),
            CurveError::Delay(r) => {
                write!(f, "rho must be a finite number, not negative (got {r})")
            }
        }
    }
}

impl std::error::Error for CurveError {}

/// Half the softer of ego's deceleration and the brake's (m/s²).
pub fn planned_braking(ego_max_deceleration: f64, max_brake: f64) -> f64 {
    ego_max_deceleration.min(max_brake) / 2.0
}

/// How long the speed loop lags a set speed falling at `a` (s); 0 with no loop.
pub fn loop_lag(speed_loop: Option<&SpeedLoopConfig>, a: f64, max_brake: f64) -> f64 {
    match speed_loop {
        None => 0.0,
        Some(c) => match c.law {
            SpeedLaw::Lqi(q) => 1.0 / widen(q.k_speed),
            SpeedLaw::Pi => (widen(c.coast_band) + a / (max_brake * widen(c.brake_gain))) / a,
        },
    }
}

impl StopCurve {
    /// A curve, its numbers checked.
    pub fn new(a: f64, j: f64, rho: f64) -> Result<Self, CurveError> {
        if !(a.is_finite() && a > 0.0) {
            return Err(CurveError::Braking(a));
        }
        if !(j.is_finite() && j > 0.0) {
            return Err(CurveError::Jerk(j));
        }
        if !(rho.is_finite() && rho >= 0.0) {
            return Err(CurveError::Delay(rho));
        }
        Ok(StopCurve { a, j, rho })
    }

    /// The curve for this car: the car model's brake and speed loop, ego's limits.
    pub fn from_car_model(
        cfg: &ActuatorConfig,
        ego_max_deceleration: f64,
        max_jerk: f64,
        input_delay: f64,
        speed_loop: Option<&SpeedLoopConfig>,
    ) -> Result<Self, CurveError> {
        let a = planned_braking(ego_max_deceleration, cfg.max_brake);
        StopCurve::new(
            a,
            max_jerk,
            input_delay
                + cfg.brake_dead_time
                + cfg.brake_lag
                + loop_lag(speed_loop, a, cfg.max_brake),
        )
    }

    /// From loose numbers (the brake's lag 0.2 s and no dead time, as the Python's
    /// defaults; the spine's default loop), as `from_car_model`.
    pub fn from_numbers(
        max_brake: f64,
        ego_max_deceleration: f64,
        max_jerk: f64,
    ) -> Result<Self, CurveError> {
        let cfg = ActuatorConfig {
            max_brake,
            brake_lag: 0.2,
            brake_dead_time: 0.0,
            ..ActuatorConfig::default()
        };
        StopCurve::from_car_model(
            &cfg,
            ego_max_deceleration,
            max_jerk,
            INPUT_DELAY,
            Some(&SpeedLoopConfig::default()),
        )
    }

    pub fn a(&self) -> f64 {
        self.a
    }

    pub fn j(&self) -> f64 {
        self.j
    }

    pub fn rho(&self) -> f64 {
        self.rho
    }

    /// τ = ρ + a/(2j): the delay plus the easing in of the brake (s).
    pub fn tau(&self) -> f64 {
        self.rho + self.a / (2.0 * self.j)
    }

    /// S(v): how far the car needs to stop from `speed` (m).
    pub fn distance(&self, speed: f64) -> f64 {
        let v = speed.abs();
        if !v.is_finite() {
            return f64::INFINITY;
        }
        v * self.tau() + v * v / (2.0 * self.a)
    }

    /// v_cap: the fastest speed that still stops in `left` m; 0 if none.
    pub fn cap(&self, left: f64) -> f64 {
        if !left.is_finite() {
            return if left > 0.0 { f64::INFINITY } else { 0.0 };
        }
        if left <= 0.0 {
            return 0.0;
        }
        let t = self.tau();
        self.a * ((t * t + 2.0 * left / self.a).sqrt() - t)
    }

    /// One line for the log, as the Python's `describe()`.
    pub fn describe(&self) -> String {
        format!(
            "the stopping law: a {:.2} m/s^2, j {:.2} m/s^3, rho {:.2} s (tau {:.2} s)",
            self.a,
            self.j,
            self.rho,
            self.tau()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cap_stops_in_the_room_given() {
        let c = StopCurve::new(1.0, 2.0, 0.35).unwrap();
        let v = c.cap(3.0);
        assert!((c.distance(v) - 3.0).abs() < 1e-12);
        assert_eq!(c.cap(-1.0), 0.0);
        assert_eq!(c.cap(f64::INFINITY), f64::INFINITY);
    }

    #[test]
    fn bad_numbers_are_refused() {
        assert_eq!(StopCurve::new(0.0, 1.0, 0.0), Err(CurveError::Braking(0.0)));
        assert_eq!(StopCurve::new(1.0, -1.0, 0.0), Err(CurveError::Jerk(-1.0)));
        assert_eq!(StopCurve::new(1.0, 1.0, -0.1), Err(CurveError::Delay(-0.1)));
    }
}
