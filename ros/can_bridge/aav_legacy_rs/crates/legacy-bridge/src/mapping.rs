//! Ego's command to the frame's three numbers (`legacy_protocol.py`'s `LegacyMapping`,
//! `to_legacy`, `from_pedals`, `stop_command`).
//!
//! The frame carries steering, speed and accel, each -1..1. The 2025-26 firmware turns them
//! into Jrk and DAC targets (legacy-link's `Frame::targets`).

/// Scale negative steering by this, so both sides get the firmware's 1550 counts per unit.
pub const NEGATIVE_STEERING_GAIN: f64 = 1550.0 / 1950.0;
/// The brake Jrk's target, released (the firmware's `JRK_MIN`).
pub const BRAKE_RELEASED: f64 = 400.0;
/// The brake Jrk's target, full on (the firmware's `JRK_MAX`).
pub const BRAKE_FULL: f64 = 3500.0;
/// The firmware ignores a speed number below this (it ramps the throttle to 0).
pub const THROTTLE_MIN: f64 = 0.03;
/// What `to_legacy` sends for a creep: just above the firmware's dead band.
const CREEP: f64 = 0.031;

/// How ego's metres and radians become the frame's -1..1 (the bridge's parameters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mapping {
    /// rad at full lock.
    pub max_steering_angle: f64,
    /// -1 if +1 steers right on this car.
    pub steering_sign: f64,
    /// m/s the car reaches at speed = 1.0.
    pub full_throttle_speed: f64,
    /// m/s below which ego's command means stand still.
    pub stop_speed: f64,
    /// Brake (0..1) while standing still.
    pub hold_brake: f64,
    /// Brake (0..1) when commands go stale.
    pub stop_brake: f64,
}

impl Default for Mapping {
    fn default() -> Self {
        Mapping {
            max_steering_angle: 0.5,
            steering_sign: 1.0,
            full_throttle_speed: 3.0,
            stop_speed: 0.05,
            hold_brake: 0.3,
            stop_brake: 1.0,
        }
    }
}

/// One frame's three numbers, before the frame clamps them to -1..1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Values {
    /// -1..1 of full lock; + as the ESP32 takes it (`steering_sign`).
    pub steering: f64,
    /// -1..1 of full throttle; its sign sets the direction.
    pub speed: f64,
    /// Below -0.05: a brake demand.
    pub accel: f64,
}

/// `x` within -1..1, exactly as Python's `max(-1.0, min(1.0, x))`: NaN becomes 1.0.
#[must_use]
pub fn clamp1(x: f64) -> f64 {
    // min(1.0, nan) keeps 1.0 (nan < 1.0 is false), and max(-1.0, 1.0) is 1.0. The same
    // comparisons, in the same order.
    let low = if x < 1.0 { x } else { 1.0 };
    if low > -1.0 {
        low
    } else {
        -1.0
    }
}

/// The frame's steering number, -1..1, for an angle in rad (+ left).
#[must_use]
pub fn steering_value(steering: f64, m: &Mapping) -> f64 {
    let s = clamp1(m.steering_sign * steering / m.max_steering_angle);
    if s < 0.0 {
        s * NEGATIVE_STEERING_GAIN
    } else {
        s
    }
}

/// The frame's numbers for an ego command in m/s and rad: the 2025-26 open-loop mapping.
#[must_use]
pub fn to_legacy(speed: f64, steering: f64, m: &Mapping) -> Values {
    let s = steering_value(steering, m);
    if speed.abs() < m.stop_speed {
        return Values {
            steering: s,
            speed: 0.0,
            accel: -m.hold_brake.abs(),
        };
    }
    let mut v = clamp1(speed / m.full_throttle_speed);
    if v.abs() < CREEP {
        // The firmware ignores |v| < 0.03; creep instead.
        v = CREEP.copysign(v);
    }
    Values {
        steering: s,
        speed: v,
        accel: 0.0,
    }
}

/// The frame's numbers for a speed loop's pedals (0..1 each), forward only.
///
/// The throttle is the speed number itself (the firmware's DAC is that times its cap, open
/// loop); below [`THROTTLE_MIN`] the firmware would ignore it, so 0. The brake spreads over
/// the brake Jrk's travel, so any brake at all lands past the firmware's -0.05 threshold.
#[must_use]
pub fn from_pedals(steering: f64, throttle: f64, brake: f64, m: &Mapping) -> Values {
    let s = steering_value(steering, m);
    if brake > 0.0 {
        let travel = BRAKE_RELEASED + brake.min(1.0) * (BRAKE_FULL - BRAKE_RELEASED);
        return Values {
            steering: s,
            speed: 0.0,
            accel: -travel / BRAKE_FULL,
        };
    }
    Values {
        steering: s,
        speed: if throttle >= THROTTLE_MIN {
            throttle
        } else {
            0.0
        },
        accel: 0.0,
    }
}

/// A stale-command stop: no throttle, the brake on, the steering held.
#[must_use]
pub fn stop_command(m: &Mapping, steering_hold: f64) -> Values {
    Values {
        steering: clamp1(steering_hold),
        speed: 0.0,
        accel: -m.stop_brake.abs(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamp1_keeps_pythons_nan_answer() {
        assert_eq!(clamp1(f64::NAN), 1.0);
        assert_eq!(clamp1(2.0), 1.0);
        assert_eq!(clamp1(-2.0), -1.0);
        assert_eq!(clamp1(0.25), 0.25);
    }

    #[test]
    fn a_crawl_becomes_a_creep_the_firmware_takes() {
        let v = to_legacy(0.06, 0.0, &Mapping::default());
        assert_eq!(v.speed, CREEP);
        let v = to_legacy(-0.06, 0.0, &Mapping::default());
        assert_eq!(v.speed, -CREEP);
    }
}
