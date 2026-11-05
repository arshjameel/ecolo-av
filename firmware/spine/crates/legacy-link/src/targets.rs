//! What the C++ did with a frame, in device units: the Jrk targets and the DAC code.

use crate::frame::Frame;

/// The steering Jrk's target for straight ahead (`JRK_CENTER`).
pub const STEER_CENTRE: u16 = 1950;
/// The Jrk limits; `setSteer()` and `setBrake()` both clamp to them (`JRK_MIN`, `JRK_MAX`).
pub(crate) const JRK_MIN: u16 = 400;
pub(crate) const JRK_MAX: u16 = 3500;
/// The brake Jrk's targets: released, and full (`JRK_MIN`, `JRK_MAX`).
pub const BRAKE_RELEASED: u16 = JRK_MIN;
pub const BRAKE_FULL: u16 = JRK_MAX;
/// The throttle DAC's top code (`DAC_MAX_CAP`, of 1023).
pub const DAC_CAP: u16 = 700;
/// Below this |speed|, no throttle (`fabsf(speed) < 0.03f`).
pub const SPEED_DEADBAND: f32 = 0.03;
/// Below this accel, the brake (`accel < -0.05f`).
pub const BRAKE_BELOW: f32 = -0.05;

/// The C++'s three outputs for one frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Targets {
    /// The steering Jrk's target, 400..3500.
    pub steering: u16,
    /// The brake Jrk's target, 400 (released)..3500 (full).
    pub brake: u16,
    pub throttle: Throttle,
}

/// The throttle DAC, for one frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Throttle {
    /// A brake demand: the DAC to 0 at once (`writeDAC(0)`, no ramp). The direction
    /// pin is left as it was.
    Cut,
    /// |speed| under 3 %: ramp the DAC down to 0 (`writeDACRamped(0)`). The direction
    /// pin is left as it was.
    Idle,
    /// Ramp the DAC to `code`, in this direction (`ext_digitalWrite(DIR_PIN, ...)`).
    Drive { code: u16, reverse: bool },
}

impl Frame {
    /// What the C++ sent to the devices for this frame (`ros2_handler.cpp`). The frame
    /// should have passed [`Frame::check`]; values outside -1..1 are clamped here.
    ///
    /// The arithmetic is the C++'s: `f32`, then truncated to a whole number.
    pub fn targets(&self) -> Targets {
        let steering = self.steering.clamp(-1.0, 1.0);
        let speed = self.speed.clamp(-1.0, 1.0);
        let accel = self.accel.clamp(-1.0, 1.0);

        // Steering: 1950 counts per unit below 0, 1550 above (the Jrk clips at 400).
        let centre = f32::from(STEER_CENTRE);
        let per_unit = if steering <= 0.0 {
            centre
        } else {
            f32::from(JRK_MAX - STEER_CENTRE)
        };
        let steering = truncate(centre + steering * per_unit).clamp(JRK_MIN, JRK_MAX);

        // A brake demand: clamp(|accel| * 3500, 400, 3500), and the throttle cut.
        if accel < BRAKE_BELOW {
            return Targets {
                steering,
                brake: truncate(accel.abs() * f32::from(JRK_MAX)).clamp(JRK_MIN, JRK_MAX),
                throttle: Throttle::Cut,
            };
        }
        let throttle = if speed.abs() < SPEED_DEADBAND {
            Throttle::Idle
        } else {
            Throttle::Drive {
                code: truncate(speed.abs() * f32::from(DAC_CAP)).min(DAC_CAP),
                reverse: speed < 0.0,
            }
        };
        Targets {
            steering,
            brake: BRAKE_RELEASED,
            throttle,
        }
    }
}

/// C's `(uint16_t)x` for 0 <= x < 65536: drop the fraction.
fn truncate(x: f32) -> u16 {
    x as u16 // saturates; NaN gives 0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(steering: f32, speed: f32, accel: f32) -> Targets {
        Frame {
            steering,
            speed,
            accel,
        }
        .targets()
    }

    #[test]
    fn steering_has_two_gains() {
        assert_eq!(t(0.0, 0.0, 0.0).steering, 1950);
        assert_eq!(t(1.0, 0.0, 0.0).steering, 3500);
        assert_eq!(t(0.5, 0.0, 0.0).steering, 2725);
        assert_eq!(t(-0.5, 0.0, 0.0).steering, 975);
        assert_eq!(t(-1.0, 0.0, 0.0).steering, 400); // 0, clipped by setSteer()
    }

    #[test]
    fn a_brake_demand_cuts_the_throttle() {
        let x = t(0.2, 0.8, -1.0);
        assert_eq!((x.brake, x.throttle), (3500, Throttle::Cut));
        assert_eq!(t(0.0, 0.0, -0.3).brake, 1050);
        assert_eq!(t(0.0, 0.0, -0.06).brake, 400); // 210, clamped up to "released"
        assert_eq!(t(0.0, 0.0, -0.05).throttle, Throttle::Idle); // not below -0.05
    }

    #[test]
    fn speed_is_an_open_loop_throttle() {
        let drive = |code, reverse| Throttle::Drive { code, reverse };
        assert_eq!(t(0.0, 0.5, 0.0).throttle, drive(350, false));
        assert_eq!(t(0.0, -1.0, 0.0).throttle, drive(700, true));
        assert_eq!(t(0.0, 0.029, 0.0).throttle, Throttle::Idle);
        assert_eq!(t(0.0, 0.5, 0.0).brake, 400);
    }
}
