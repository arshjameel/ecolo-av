//! Today's numbers: the I2C addresses, and how the spine's units become device units.
//!
//! | Spine | Device | Today |
//! |---|---|---|
//! | steering, rad (positive left) | steering Jrk target | 1950 centre, 3100 a radian, 400 to 3500 |
//! | brake, 0 to 1 | brake Jrk target | 400 released to 3500 full |
//! | throttle, 0 to 1 | DAC code | 0 to 700 of 1023, rising at most 600 a second |
//! | reverse | expander pin P00 | high forward, low reverse |
//!
//! The numbers come from the 2025-26 firmware. Where they are unconfirmed, or
//! disagree with each other or with `docs/firmware.md`, the C++ value is kept and
//! marked TODO with the design doc's question number.

use ibus::RcMap;
use pca9555::Pin;

/// Everything about today's car that the board-independent code needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Calibration {
    pub addresses: Addresses,
    pub steering: Steering,
    pub brake: Brake,
    pub throttle: Throttle,
    pub direction: Direction,
    pub wheel: Wheel,
    /// Which RC channel does what.
    pub rc: RcMap,
    /// No good iBUS frame for this long, and the RC counts as lost. Frames come
    /// every 7 ms. TODO(Q7): the C++ had no timeout at all.
    pub rc_timeout_ms: u32,
    /// The hardware e-stop trips at once, and must read closed this long to
    /// release. TODO(Q6): the C++ had no e-stop input.
    pub estop_release_ms: u32,
}

/// The I2C addresses (`hardware_configs.h`: STEER_ID, BRAKE_ID, DAC_ADDR; the
/// expander at the PCA95x5 library's default).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Addresses {
    pub steering: u8,
    pub brake: u8,
    pub dac: u8,
    pub expander: u8,
}

/// Steering: radians to steering Jrk targets, and back.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Steering {
    /// The target with the wheels straight.
    pub centre: u16,
    /// Target counts per radian. Positive: steering left raises the target.
    pub counts_per_rad: f32,
    /// The targets at full lock: never sent beyond these.
    pub min: u16,
    pub max: u16,
}

/// The brake: 0 (released) to 1 (full) into brake Jrk targets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Brake {
    pub released: u16,
    pub full: u16,
}

/// The throttle: 0 to 1 into DAC codes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Throttle {
    /// The code for full throttle. The DAC never gets more.
    pub full_code: u16,
    /// The fastest the code may rise, in codes a second. It may fall at once.
    pub rise_per_s: u16,
}

/// The direction output on the I/O expander.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Direction {
    /// The pin wired to the motor controller's direction input.
    pub pin: Pin,
    /// True if the pin is high for forward.
    pub forward_high: bool,
    /// The levels for the other 15 pins. They are all outputs, as in the C++.
    pub other_pins: u16,
}

/// The wheel encoder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wheel {
    pub metres_per_count: f32,
    /// With no count for this long, the car counts as stopped.
    pub timeout_ms: u32,
}

impl Calibration {
    /// Today's car, from the 2025-26 firmware.
    pub const TODAY: Calibration = Calibration {
        addresses: Addresses {
            steering: 0x05,
            brake: 0x04,
            dac: 0x0C,
            expander: 0x20,
        },
        // TODO(Q2): steering calibration unconfirmed. The C++ disagrees with itself:
        // it steers between 400 and 3500 around 1950 (setSteer() clamps to JRK_MIN
        // and JRK_MAX), but hardware_configs.h also says centre 500, range 0 to 1550,
        // and rc_handler.cpp says "JRK 0-1550". docs/firmware.md: "about 0-3500".
        // Is the sensor wired to the Jrk, and is its feedback scaled to the target?
        // Measure them: docs/actuator-tests.md, "Find the steering actuator's ends and
        // centre" (the hardware panel's limp and jog, or Pololu's utility), then
        // `steering_limits` gives this block.
        steering: Steering {
            centre: 1950,
            // TODO(Q1): 1550 counts to full lock, taken as 0.5 rad (vehicle.yaml
            // max_steering_angle, itself a guess). TODO(Q2): the sign: the C++ sends
            // its +1 to 3500, and whether +1 is left is unconfirmed.
            counts_per_rad: 3100.0,
            min: 400,
            max: 3500,
        },
        // TODO(Q3): brake travel unconfirmed. The C++ releases at 400 and brakes
        // fully at 3500 (JRK_MIN, JRK_MAX); its ROS path used |accel| * 3500.
        brake: Brake {
            released: 400,
            full: 3500,
        },
        // TODO(Q4): the throttle cap (DAC_MAX_CAP, "~68% of 1023") and the ramp.
        // The C++ stepped up 6 codes a loop (ACCEL_STEP); its loop rate is unknown,
        // so 600 a second assumes about 100 loops a second.
        throttle: Throttle {
            full_code: 700,
            rise_per_s: 600,
        },
        // TODO(Q4): high forward, low reverse (rc_handler.cpp, ros2_handler.cpp).
        // TODO(Q13): what the other 15 expander pins drive is unknown; the C++
        // makes them all outputs, high.
        direction: Direction {
            pin: Pin::P00,
            forward_high: true,
            other_pins: 0xFFFF,
        },
        // TODO(Q5): which wheel, and how many counts a turn. 8 spokes and a 0.30 m
        // wheel (vehicle.yaml, both TODO): pi * 0.30 / 8 = 0.1178 m a count.
        wheel: Wheel {
            metres_per_count: 0.1178,
            timeout_ms: 1000,
        },
        rc: RcMap::TODAY,
        rc_timeout_ms: 100,
        estop_release_ms: 50,
    };
}

impl Default for Calibration {
    fn default() -> Self {
        Self::TODAY
    }
}

impl Steering {
    /// The Jrk target for an angle (rad, positive left), inside the limits.
    /// Not a number gives the centre.
    pub fn target(&self, angle: f32) -> u16 {
        let target = if angle.is_finite() {
            f32::from(self.centre) + angle * self.counts_per_rad
        } else {
            f32::from(self.centre)
        };
        round_within(target, self.min, self.max)
    }

    /// The angle (rad) for a reading in target units, such as the Jrk's scaled
    /// feedback.
    pub fn angle(&self, target: u16) -> f32 {
        (f32::from(target) - f32::from(self.centre)) / self.counts_per_rad
    }
}

impl Brake {
    /// The Jrk target for 0 (released) to 1 (full). Not a number gives full.
    pub fn target(&self, brake: f32) -> u16 {
        let b = if brake.is_nan() {
            1.0
        } else {
            brake.clamp(0.0, 1.0)
        };
        let (released, full) = (f32::from(self.released), f32::from(self.full));
        let (lo, hi) = (self.released.min(self.full), self.released.max(self.full));
        round_within(released + b * (full - released), lo, hi)
    }
}

impl Brake {
    /// The fraction (0 to 1) for a target: the inverse of [`Brake::target`].
    pub fn fraction(&self, target: u16) -> f32 {
        let span = f32::from(self.full) - f32::from(self.released);
        if span == 0.0 {
            return 1.0;
        }
        ((f32::from(target) - f32::from(self.released)) / span).clamp(0.0, 1.0)
    }
}

impl Throttle {
    /// The fraction (0 to 1) for a DAC code: the inverse of [`Throttle::code`].
    pub fn fraction(&self, code: u16) -> f32 {
        if self.full_code == 0 {
            return 0.0;
        }
        (f32::from(code) / f32::from(self.full_code)).clamp(0.0, 1.0)
    }

    /// The DAC code for 0 to 1. Not a number gives 0.
    pub fn code(&self, throttle: f32) -> u16 {
        if throttle > 0.0 {
            round_within(throttle * f32::from(self.full_code), 0, self.full_code)
        } else {
            0
        }
    }

    /// How far the code may rise in `dt_ms`.
    pub fn max_rise(&self, dt_ms: u32) -> u16 {
        let rise = u32::from(self.rise_per_s) * dt_ms / 1000;
        rise.min(u32::from(u16::MAX)) as u16
    }
}

impl Direction {
    /// All sixteen expander levels, for forward or reverse.
    pub fn levels(&self, reverse: bool) -> u16 {
        if self.forward_high != reverse {
            self.other_pins | self.pin.mask()
        } else {
            self.other_pins & !self.pin.mask()
        }
    }
}

/// Round to the nearest whole number, kept between `lo` and `hi`.
fn round_within(x: f32, lo: u16, hi: u16) -> u16 {
    let x = x.clamp(f32::from(lo), f32::from(hi));
    (x + 0.5) as u16 // x >= 0 here, and `as` saturates
}

#[cfg(test)]
mod tests {
    use super::*;

    const CAL: Calibration = Calibration::TODAY;

    #[test]
    fn steering_targets() {
        let s = CAL.steering;
        assert_eq!(s.target(0.0), 1950);
        assert_eq!(s.target(0.5), 3500); // full left
        assert_eq!(s.target(-0.5), 400); // full right
        assert_eq!(s.target(0.1), 2260);
        assert_eq!(s.target(2.0), 3500); // never past full lock
        assert_eq!(s.target(-2.0), 400);
        assert_eq!(s.target(f32::NAN), 1950);
        assert_eq!(s.target(f32::INFINITY), 1950);
    }

    #[test]
    fn steering_angle_reverses_target() {
        let s = CAL.steering;
        assert_eq!(s.angle(1950), 0.0);
        assert!((s.angle(3500) - 0.5).abs() < 1e-6);
        assert!((s.angle(400) + 0.5).abs() < 1e-6);
        for angle in [-0.4, -0.1, 0.0, 0.25] {
            assert!((s.angle(s.target(angle)) - angle).abs() < 1.0 / 3100.0);
        }
    }

    #[test]
    fn brake_targets() {
        let b = CAL.brake;
        assert_eq!(b.target(0.0), 400);
        assert_eq!(b.target(1.0), 3500);
        assert_eq!(b.target(0.5), 1950);
        assert_eq!(b.target(-1.0), 400);
        assert_eq!(b.target(3.0), 3500);
        assert_eq!(b.target(f32::NAN), 3500);
    }

    #[test]
    fn a_brake_that_pulls_the_other_way() {
        let b = Brake {
            released: 3500,
            full: 400,
        };
        assert_eq!(b.target(0.0), 3500);
        assert_eq!(b.target(1.0), 400);
        assert_eq!(b.target(0.5), 1950);
    }

    #[test]
    fn throttle_codes() {
        let t = CAL.throttle;
        assert_eq!(t.code(0.0), 0);
        assert_eq!(t.code(1.0), 700);
        assert_eq!(t.code(0.5), 350);
        assert_eq!(t.code(2.0), 700); // the cap
        assert_eq!(t.code(-0.5), 0);
        assert_eq!(t.code(f32::NAN), 0);
    }

    #[test]
    fn fractions_give_the_same_targets_back() {
        let (b, t) = (CAL.brake, CAL.throttle);
        for target in 400..=3500 {
            assert_eq!(b.target(b.fraction(target)), target);
        }
        for code in 0..=700 {
            assert_eq!(t.code(t.fraction(code)), code);
        }
        let s = CAL.steering;
        for target in 400..=3500 {
            assert_eq!(s.target(s.angle(target)), target);
        }
        assert_eq!((b.fraction(0), b.fraction(9999)), (0.0, 1.0));
        assert_eq!(t.fraction(1023), 1.0);
    }

    #[test]
    fn throttle_rise() {
        let t = CAL.throttle;
        assert_eq!(t.max_rise(10), 6); // the C++'s ACCEL_STEP, at 100 loops a second
        assert_eq!(t.max_rise(0), 0);
        assert_eq!(t.max_rise(1000), 600);
    }

    #[test]
    fn direction_levels() {
        let d = CAL.direction;
        assert_eq!(d.levels(false), 0xFFFF); // forward: P00 high
        assert_eq!(d.levels(true), 0xFFFE); // reverse: P00 low
        let low_forward = Direction {
            forward_high: false,
            ..d
        };
        assert_eq!(low_forward.levels(false), 0xFFFE);
    }
}
