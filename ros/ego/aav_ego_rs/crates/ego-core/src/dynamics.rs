//! The car as a body with weight: what its mass, its load and its shape allow
//! (`dynamics.py`).
//!
//! | Changes with the load | Doesn't |
//! |---|---|
//! | speeding up: a = F / m | the speed a turn allows (sliding and tipping cancel the mass) |
//! | braking, the energy in a crash | |
//!
//! ego never brakes less hard because the car is heavy, and never steers less sharply in
//! a turn: it slows down instead.

use crate::fmt_g;
use crate::stop_law::StopCurve;

/// m/s².
pub const G: f64 = 9.81;
/// ego stays at half the sliding and tipping limits: their inputs are guesses.
pub const MARGIN: f64 = 2.0;
/// s from an obstacle seen to the brakes biting.
pub const REACTION: f64 = 0.4;
/// kg: the payload a running ego accepts (`ros2 param set /ego payload`).
pub const PAYLOAD_RANGE: (f64, f64) = (0.0, 300.0);

/// The body: `vehicle.yaml`'s numbers and ego.yaml's payload and comfort limit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    /// kg, the empty car.
    pub mass: f64,
    /// kg aboard. May change while running.
    pub payload: f64,
    /// m between the left and right wheels' centres.
    pub track: f64,
    /// m, the centre of mass above the ground.
    pub cg_height: f64,
    /// The most the tyres can push, in g, before they slide.
    pub tyre_grip: f64,
    /// m/s², the empty car at full throttle.
    pub full_throttle_accel: f64,
    /// m/s², the empty car at full brake.
    pub full_brake_decel: f64,
    /// m/s² sideways in a turn: comfort.
    pub max_lateral_accel: f64,
}

impl Default for Body {
    fn default() -> Self {
        Body {
            mass: 181.0,
            payload: 0.0,
            track: 0.77,
            cg_height: 0.55,
            tyre_grip: 0.7,
            full_throttle_accel: 1.5,
            full_brake_decel: 3.0,
            max_lateral_accel: 2.0,
        }
    }
}

impl Body {
    /// The car and its load (kg).
    #[must_use]
    pub fn total_mass(&self) -> f64 {
        self.mass + self.payload
    }

    /// The empty car's share of the total mass.
    #[must_use]
    pub fn load_share(&self) -> f64 {
        self.mass / self.total_mass()
    }

    /// The hardest (m/s²) the loaded car can speed up: its motor, then its tyres.
    #[must_use]
    pub fn speed_up_limit(&self) -> f64 {
        (self.full_throttle_accel * self.load_share()).min(self.tyre_grip * G)
    }

    /// The hardest (m/s²) the loaded car can brake: its brakes, then its tyres.
    #[must_use]
    pub fn braking_limit(&self) -> f64 {
        (self.full_brake_decel * self.load_share()).min(self.tyre_grip * G)
    }

    /// m/s² sideways at which the tyres slide.
    #[must_use]
    pub fn sliding_accel(&self) -> f64 {
        self.tyre_grip * G
    }

    /// m/s² sideways at which the inside wheels lift.
    #[must_use]
    pub fn tipping_accel(&self) -> f64 {
        G * (self.track / 2.0) / self.cg_height
    }

    /// m/s² sideways ego allows: comfort, or half of slide or tip.
    #[must_use]
    pub fn lateral_limit(&self) -> f64 {
        self.max_lateral_accel
            .min(self.sliding_accel() / MARGIN)
            .min(self.tipping_accel() / MARGIN)
    }

    /// The fastest (m/s) a turn of this curvature (1/m) allows; infinite when straight.
    #[must_use]
    pub fn curve_speed(&self, curvature: f64) -> f64 {
        let k = curvature.abs();
        if k < 1e-9 {
            f64::INFINITY
        } else {
            (self.lateral_limit() / k).sqrt()
        }
    }

    /// The distance (m) to stop from `speed`: the reaction time, then braking at `decel`.
    #[must_use]
    pub fn stopping_distance(&self, speed: f64, decel: f64) -> f64 {
        speed.abs() * REACTION + speed * speed / (2.0 * decel)
    }

    /// The energy of motion (J), the load included.
    #[must_use]
    pub fn energy(&self, speed: f64) -> f64 {
        0.5 * self.total_mass() * speed * speed
    }
}

/// ego's limits as `describe` needs them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Described {
    /// m.
    pub wheelbase: f64,
    /// rad.
    pub max_steering_angle: f64,
    /// m/s.
    pub max_speed: f64,
    /// m/s².
    pub max_acceleration: f64,
    /// m/s².
    pub max_deceleration: f64,
    /// m/s³.
    pub max_jerk: f64,
}

/// A line for ego's start-up log.
#[derive(Debug, Clone, PartialEq)]
pub enum LogLine {
    /// Logged at INFO.
    Info(String),
    /// Logged at WARN.
    Warning(String),
}

/// Short lines for ego's log: what this body allows with ego's limits (`dynamics.describe`).
#[must_use]
pub fn describe(body: &Body, lim: &Described) -> Vec<LogLine> {
    let jerk = if lim.max_jerk != 0.0 {
        lim.max_jerk
    } else {
        2.0
    };
    let plans = StopCurve::from_numbers(body.braking_limit(), lim.max_deceleration, jerk)
        .map(|c| format!("{:.2} m ({})", c.distance(lim.max_speed), c.describe()))
        .unwrap_or_else(|e| format!("nothing: {e}"));
    let speed_up = lim.max_acceleration.min(body.speed_up_limit());
    let brake = lim.max_deceleration.min(body.braking_limit());
    let lock = lim.wheelbase / lim.max_steering_angle.tan();
    let mut lines = vec![
        LogLine::Info(format!(
            "body: {:.0} kg + {:.0} kg aboard",
            body.mass, body.payload
        )),
        LogLine::Info(format!(
            "speeds up at {speed_up:.2} m/s^2 at most (the motor could give {:.2})",
            body.speed_up_limit()
        )),
        LogLine::Info(format!(
            "stops from {} m/s in {:.2} m, braking at {brake:.2} m/s^2 after {} s; plans {plans}",
            fmt_g(lim.max_speed),
            body.stopping_distance(lim.max_speed, brake),
            fmt_g(REACTION),
        )),
        LogLine::Info(format!(
            "turns: at most {:.1} m/s^2 sideways (slides at {:.1}, tips at {:.1}); full lock \
             ({lock:.2} m) at up to {:.1} m/s",
            body.lateral_limit(),
            body.sliding_accel(),
            body.tipping_accel(),
            body.curve_speed(1.0 / lock)
        )),
        LogLine::Info(format!(
            "carries {:.0} J at {} m/s",
            body.energy(lim.max_speed),
            fmt_g(lim.max_speed)
        )),
    ];
    if body.braking_limit() < lim.max_deceleration {
        lines.push(LogLine::Warning(format!(
            "with {:.0} kg aboard the brakes give only {:.2} m/s^2, less than \
             max_deceleration {}: stops take longer than ego plans",
            body.payload,
            body.braking_limit(),
            fmt_g(lim.max_deceleration)
        )));
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_heavier_car_speeds_up_and_brakes_less_but_turns_the_same() {
        let empty = Body::default();
        let loaded = Body {
            payload: 150.0,
            ..empty
        };
        assert!(loaded.speed_up_limit() < empty.speed_up_limit());
        assert!(loaded.braking_limit() < empty.braking_limit());
        assert_eq!(loaded.curve_speed(0.5), empty.curve_speed(0.5));
        assert_eq!(empty.curve_speed(0.0), f64::INFINITY);
    }

    #[test]
    fn describe_warns_when_the_brakes_are_weak() {
        let lim = Described {
            wheelbase: 1.561,
            max_steering_angle: 0.5,
            max_speed: 1.0,
            max_acceleration: 1.0,
            max_deceleration: 2.0,
            max_jerk: 2.0,
        };
        let heavy = Body {
            payload: 300.0,
            ..Body::default()
        };
        assert!(describe(&heavy, &lim)
            .iter()
            .any(|l| matches!(l, LogLine::Warning(_))));
        assert!(describe(&Body::default(), &lim)
            .iter()
            .all(|l| matches!(l, LogLine::Info(_))));
    }
}
