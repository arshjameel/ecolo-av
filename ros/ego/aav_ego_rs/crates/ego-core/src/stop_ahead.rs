//! The stop point: where the car stops if it brakes fully now (`stop_ahead.py`).
//!
//! ```text
//! the twin's state now ──▶ car model, full brake, the wheels held where they point
//!                      ──▶ step until the speed is 0 ──▶ where, how far, how long
//! ```

use car_model::{ActuatorConfig, CarState, Command, Drive, ModelError};

use crate::hypot;

/// m/s: slower counts as stopped.
pub const STOPPED: f64 = 0.01;
/// s: a stop that takes longer is not found.
pub const MAX_TIME: f64 = 30.0;

/// Where a full brake now stops the car, in `base_link`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StopPoint {
    /// m, + forward.
    pub x: f64,
    /// m, + left.
    pub y: f64,
    /// rad, + left.
    pub yaw: f64,
    /// m along the way there.
    pub distance: f64,
    /// s from now.
    pub time: f64,
    /// `false`: still moving after `max_time`.
    pub found: bool,
}

/// Where the car stops if it brakes fully now, in `base_link` (the state's x, y, yaw are
/// ignored). The throttle let go, the brake full, the steering held at the motor's
/// setpoint. Nothing passed in changes.
///
/// # Errors
///
/// [`ModelError::NonPositiveDt`] for `dt` not above 0.
pub fn stop_point(
    cfg: &ActuatorConfig,
    state: &CarState,
    dt: f64,
    max_time: f64,
) -> Result<StopPoint, ModelError> {
    if dt <= 0.0 || dt.is_nan() {
        return Err(ModelError::NonPositiveDt(dt));
    }
    let mut s = CarState {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
        ..state.clone()
    };
    let brake = Command {
        drive: Drive::Pedals {
            throttle: 0.0,
            brake: 1.0,
            reverse: s.v < 0.0,
        },
        steer: s.steer_setpoint,
    };
    let (mut distance, mut t) = (0.0, 0.0);
    while s.v.abs() >= STOPPED {
        if t >= max_time {
            return Ok(StopPoint {
                x: s.x,
                y: s.y,
                yaw: s.yaw,
                distance,
                time: t,
                found: false,
            });
        }
        let next = cfg.step(&s, brake, dt, None)?;
        distance += hypot(next.x - s.x, next.y - s.y);
        s = next;
        t += dt;
    }
    Ok(StopPoint {
        x: s.x,
        y: s.y,
        yaw: s.yaw,
        distance,
        time: t,
        found: true,
    })
}

/// The design's S(v) = v tau + v² / (2a): m to stop from v m/s.
#[must_use]
pub fn s_of_v(v: f64, tau: f64, a: f64) -> f64 {
    let v = v.abs();
    v * tau + v * v / (2.0 * a)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parked_stops_where_it_is_and_moving_stops_ahead() {
        let cfg = ActuatorConfig::default();
        let parked = stop_point(&cfg, &CarState::default(), 0.02, MAX_TIME).unwrap();
        assert_eq!((parked.x, parked.distance, parked.found), (0.0, 0.0, true));
        let moving = CarState {
            v: 1.0,
            ..CarState::default()
        };
        let p = stop_point(&cfg, &moving, 0.02, MAX_TIME).unwrap();
        assert!(p.found && p.x > 0.1 && p.x < 0.6, "{p:?}");
        let backwards = CarState {
            v: -1.0,
            ..CarState::default()
        };
        assert!(stop_point(&cfg, &backwards, 0.02, MAX_TIME).unwrap().x < 0.0);
    }

    #[test]
    fn no_brake_is_not_found() {
        let cfg = ActuatorConfig {
            max_brake: 0.0,
            drag: 0.0,
            ..ActuatorConfig::default()
        };
        let moving = CarState {
            v: 1.0,
            ..CarState::default()
        };
        assert!(!stop_point(&cfg, &moving, 0.1, 2.0).unwrap().found);
    }
}
