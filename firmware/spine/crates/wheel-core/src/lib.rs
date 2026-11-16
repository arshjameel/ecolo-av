//! The wheel nodes' logic: Hall sensor pulses in, the WHEEL_SPEED CAN frame out.
//!
//! A wheel node (an ESP32-CAN-X2, one per front wheel) counts the holes of the brake
//! rotor as they pass a Hall sensor, and sends WHEEL_SPEED_L (0x200) or WHEEL_SPEED_R
//! (0x201) every 10 ms. See `firmware/can/aav.dbc` and `docs/can-plan.md`.
//!
//! ```text
//!  Hall sensor edge ──► WheelNode::on_pulse(now_us)
//!  every 10 ms      ──► WheelNode::frame(now_us) ──► [u8; 8] onto the CAN bus
//! ```
//!
//! Plain `no_std` Rust with no hardware access, like `spine-core`: the board crate owns
//! the pin, the clock and the CAN driver, and this crate is tested with `cargo test` on
//! a laptop. The settings (holes, radius) come from the board crate: calibration belongs
//! in the firmware, not in ROS.
//!
//! Times are a free-running microsecond clock that may wrap (every 71 minutes).
#![no_std]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

use core::f32::consts::TAU;

use spine_core::encoder::SpeedEstimator;
use spine_core::protocol::{Wheel, WheelSpeed};

/// How often a wheel node sends its frame (ms): 100 Hz, from firmware/can/aav.dbc.
pub const FRAME_PERIOD_MS: u32 = 10;

/// One wheel's settings, measured on the car.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelConfig {
    /// Holes in the brake rotor: pulses per turn of the wheel.
    pub holes_per_rev: u16,
    /// Rolling radius of the tyre (m): half of vehicle.yaml's wheel_diameter, then
    /// calibrated by driving a known distance and counting pulses.
    pub wheel_radius: f32,
    /// With no pulse for this long, the wheel counts as stopped (ms). It also sets the
    /// slowest speed seen: one hole's travel in this time.
    pub zero_speed_timeout_ms: u32,
    /// Pulses closer together than this are noise, and ignored (µs). 0 keeps them all.
    pub min_pulse_gap_us: u32,
}

/// A setting that can't work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// `holes_per_rev` is 0.
    NoHoles,
    /// `wheel_radius` is not a positive number.
    BadRadius,
    /// `zero_speed_timeout_ms` is 0, or too long for the microsecond clock.
    BadTimeout,
}

impl WheelConfig {
    /// How far the car goes from one pulse to the next (m).
    #[must_use]
    pub fn metres_per_pulse(&self) -> f32 {
        TAU * self.wheel_radius / f32::from(self.holes_per_rev)
    }

    /// The slowest speed that still reads as moving (m/s).
    #[must_use]
    pub fn slowest_speed(&self) -> f32 {
        self.metres_per_pulse() * 1000.0 / self.zero_speed_timeout_ms as f32
    }

    fn check(&self) -> Result<(), ConfigError> {
        if self.holes_per_rev == 0 {
            return Err(ConfigError::NoHoles);
        }
        if !(self.wheel_radius.is_finite() && self.wheel_radius > 0.0) {
            return Err(ConfigError::BadRadius);
        }
        // Half the clock's range, so a wrapped difference is never mistaken for a short one.
        if self.zero_speed_timeout_ms == 0 || self.zero_speed_timeout_ms > u32::MAX / 2000 {
            return Err(ConfigError::BadTimeout);
        }
        Ok(())
    }
}

/// Which way the wheel turns, when the board can tell (a second sensor, or the gear).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// The car rolls forward.
    Forward,
    /// The car rolls backward.
    Reverse,
}

/// One wheel node: counts pulses, works out the speed, and builds its frame.
#[derive(Debug, Clone, Copy)]
pub struct WheelNode {
    wheel: Wheel,
    cfg: WheelConfig,
    estimator: SpeedEstimator,
    pulses: u16,
    last_pulse_us: Option<u32>,
    counter: u8,
}

impl WheelNode {
    /// A node for `wheel`, with no pulse yet.
    ///
    /// # Errors
    ///
    /// The config's error, if it can't give a speed (see [`ConfigError`]).
    pub fn new(wheel: Wheel, cfg: WheelConfig) -> Result<Self, ConfigError> {
        cfg.check()?;
        Ok(WheelNode {
            wheel,
            cfg,
            estimator: SpeedEstimator::new(
                cfg.metres_per_pulse(),
                cfg.zero_speed_timeout_ms * 1000,
            ),
            pulses: 0,
            last_pulse_us: None,
            counter: 0,
        })
    }

    /// Which wheel this node is.
    #[must_use]
    pub fn wheel(&self) -> Wheel {
        self.wheel
    }

    /// The config it was made with.
    #[must_use]
    pub fn config(&self) -> WheelConfig {
        self.cfg
    }

    /// One edge from the Hall sensor, at `now_us`. Returns false if it was ignored as
    /// noise (closer than `min_pulse_gap_us` to the previous one).
    pub fn on_pulse(&mut self, now_us: u32) -> bool {
        if let Some(last) = self.last_pulse_us {
            if now_us.wrapping_sub(last) < self.cfg.min_pulse_gap_us {
                return false;
            }
        }
        self.last_pulse_us = Some(now_us);
        self.pulses = self.pulses.wrapping_add(1);
        self.estimator.on_count(now_us, 1);
        true
    }

    /// Pulses so far; wraps at 65536, as in the frame.
    #[must_use]
    pub fn pulse_count(&self) -> u16 {
        self.pulses
    }

    /// Speed of the tyre over the ground at `now_us` (m/s), always 0 or more: one Hall
    /// sensor can't tell which way the wheel turns.
    #[must_use]
    pub fn speed(&self, now_us: u32) -> f32 {
        self.estimator.speed(now_us)
    }

    /// The next WHEEL_SPEED frame, at `now_us`; send it every [`FRAME_PERIOD_MS`].
    /// `direction`: `None` with only the Hall sensor, and the receiver takes the sign
    /// from the drive direction.
    pub fn frame(&mut self, now_us: u32, direction: Option<Direction>) -> [u8; 8] {
        self.counter = (self.counter + 1) & 0x0F;
        let speed = self.speed(now_us);
        WheelSpeed {
            wheel: self.wheel,
            wheel_speed: match direction {
                Some(Direction::Reverse) => -speed,
                _ => speed,
            },
            pulse_count: self.pulses,
            direction_known: direction.is_some(),
            counter: self.counter,
        }
        .encode()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 8 holes and a 0.15 m radius: 0.1178 m a pulse.
    const CFG: WheelConfig = WheelConfig {
        holes_per_rev: 8,
        wheel_radius: 0.15,
        zero_speed_timeout_ms: 500,
        min_pulse_gap_us: 1000,
    };

    fn node() -> WheelNode {
        match WheelNode::new(Wheel::Left, CFG) {
            Ok(n) => n,
            Err(e) => panic!("{e:?}"),
        }
    }

    /// Pulses for `speed` m/s from 0 to `until_us`; returns the node.
    fn drive(speed: f32, until_us: u32) -> WheelNode {
        let mut n = node();
        let period_us = (CFG.metres_per_pulse() / speed * 1e6) as u32;
        (0..until_us).step_by(period_us as usize).for_each(|t| {
            n.on_pulse(t);
        });
        n
    }

    #[test]
    fn distance_per_pulse() {
        assert!((CFG.metres_per_pulse() - 0.117_81).abs() < 1e-4);
        assert!((CFG.slowest_speed() - 0.2356).abs() < 1e-3);
    }

    #[test]
    fn bad_settings_are_refused() {
        let with = |f: fn(&mut WheelConfig)| {
            let mut c = CFG;
            f(&mut c);
            WheelNode::new(Wheel::Right, c).err()
        };
        assert_eq!(with(|c| c.holes_per_rev = 0), Some(ConfigError::NoHoles));
        assert_eq!(with(|c| c.wheel_radius = 0.0), Some(ConfigError::BadRadius));
        assert_eq!(
            with(|c| c.wheel_radius = f32::NAN),
            Some(ConfigError::BadRadius)
        );
        assert_eq!(
            with(|c| c.zero_speed_timeout_ms = 0),
            Some(ConfigError::BadTimeout)
        );
        assert_eq!(with(|_| {}), None);
    }

    #[test]
    fn stopped_until_it_moves() {
        let mut n = node();
        assert_eq!(n.speed(0), 0.0);
        n.on_pulse(0);
        assert_eq!(n.speed(10_000), 0.0, "one pulse is not a speed yet");
    }

    #[test]
    fn steady_speed_is_measured() {
        for v in [0.5_f32, 2.0, 8.0] {
            let n = drive(v, 2_000_000);
            let got = n.speed(n.last_pulse_us.unwrap_or(0));
            assert!((got - v).abs() / v < 0.01, "{v} m/s read as {got}");
        }
    }

    #[test]
    fn slows_between_pulses_then_stops_after_the_timeout() {
        let n = drive(2.0, 1_000_000);
        let last = n.last_pulse_us.unwrap_or(0);
        assert!(n.speed(last + 200_000) < 0.6, "200 ms with no pulse");
        assert_eq!(n.speed(last + 501_000), 0.0, "past the 500 ms timeout");
    }

    #[test]
    fn noise_closer_than_the_gap_is_ignored() {
        let mut n = node();
        assert!(n.on_pulse(0));
        assert!(!n.on_pulse(300));
        assert!(n.on_pulse(60_000));
        assert_eq!(n.pulse_count(), 2);
    }

    #[test]
    fn survives_the_clock_wrap() {
        let mut n = node();
        let start = u32::MAX - 30_000;
        n.on_pulse(start);
        n.on_pulse(start.wrapping_add(58_905)); // 2 m/s
        let v = n.speed(start.wrapping_add(58_905));
        assert!((v - 2.0).abs() < 0.01, "{v}");
    }

    #[test]
    fn frames_decode_with_the_shared_protocol() {
        let mut n = drive(2.0, 1_000_000);
        let now = n.last_pulse_us.unwrap_or(0);
        let pulses = n.pulse_count();
        let first = n.frame(now, None);
        let back = WheelSpeed::decode(Wheel::Left, &first);
        let Ok(back) = back else { panic!("{back:?}") };
        assert!((back.wheel_speed - 2.0).abs() < 0.02);
        assert_eq!(back.pulse_count, pulses);
        assert!(!back.direction_known);
        assert_eq!(back.counter, 1);

        let second = WheelSpeed::decode(Wheel::Left, &n.frame(now, Some(Direction::Reverse)));
        let Ok(second) = second else {
            panic!("{second:?}")
        };
        assert!(second.wheel_speed < -1.9);
        assert!(second.direction_known);
        assert_eq!(second.counter, 2);
    }

    #[test]
    fn the_counter_wraps_at_16() {
        let mut n = node();
        let counters: [u8; 17] = core::array::from_fn(|_| {
            let f = n.frame(0, None);
            f[6] >> 4
        });
        assert_eq!(counters[14], 15);
        assert_eq!(counters[15], 0);
        assert_eq!(counters[16], 1);
    }
}
