//! What the C++ did with a frame, in device units: the Jrk targets and the DAC code.

use crate::frame::Frame;

/// The steering Jrk's target for straight ahead (`JRK_CENTER`).
pub const STEER_CENTRE: u16 = 1950;
/// The Jrk limits; `setSteer()` and `setBrake()` both clamp to them (`JRK_MIN`, `JRK_MAX`).
pub(crate) const JRK_MIN: u16 = 400;
pub(crate) const JRK_MAX: u16 = 3500;
/// The brake Jrk's targets: released, and full (`JRK_MIN`, `JRK_MAX`).
pub const BRAKE_RELEASED: u16 = JRK_MIN;
/// The brake Jrk's target for full on.
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
    /// What the DAC and the direction pin do.
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
    Drive {
        /// The DAC code to ramp to.
        code: u16,
        /// The direction pin: true for reverse.
        reverse: bool,
    },
}

impl Frame {
    /// What the C++ sent to the devices for this frame (`ros2_handler.cpp`). The frame
    /// should have passed [`Frame::check`]; values outside -1..1 are clamped here.
    ///
    /// The arithmetic is the C++'s: `f32`, then truncated to a whole number.
    #[must_use]
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

/// The C++'s DAC ramp (`writeDACRamped`, `hardware_configs.h`): each call moved the code at
/// most this far up, or down. On the Motor-Pi's link it was called once a frame, so at
/// 50 frames a second the code rose at most 300 a second and fell at most 600.
pub const ACCEL_STEP: u16 = 6;
/// The C++'s DAC ramp's step down, a call.
pub const DECEL_STEP: u16 = 12;

/// How far the steering's stick or frame value turns right for each count it turns left:
/// the C++ steered the negative side with `JRK_CENTER` counts per unit (1950) and the
/// positive side with `JRK_MAX - JRK_CENTER` (1550), so a right turn reached the Jrk's 400
/// limit at 80 % of the stick. [`Frame::targets`] does the same; the RC can, with this.
pub const STEER_RIGHT_PER_LEFT: f32 = STEER_CENTRE as f32 / (JRK_MAX - STEER_CENTRE) as f32;

/// The C++'s ramp, one frame at a time: [`ACCEL_STEP`] up, [`DECEL_STEP`] down, to at
/// most [`DAC_CAP`]. A brake demand (`Throttle::Cut`) cut it to 0 at once (`writeDAC(0)`,
/// `current_dac = 0`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DacRamp {
    code: u16,
}

impl DacRamp {
    /// A ramp at code 0.
    #[must_use]
    pub const fn new() -> Self {
        DacRamp { code: 0 }
    }

    /// One step toward `target`; returns the code the DAC gets.
    pub fn toward(&mut self, target: u16) -> u16 {
        let target = target.min(DAC_CAP);
        self.code = if self.code < target {
            (self.code + ACCEL_STEP).min(target)
        } else {
            self.code.saturating_sub(DECEL_STEP).max(target)
        };
        self.code
    }

    /// Straight to 0.
    pub fn cut(&mut self) {
        self.code = 0;
    }

    /// The code the ramp is at.
    #[must_use]
    pub const fn code(&self) -> u16 {
        self.code
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
    fn the_ramp_rises_6_and_falls_12_a_frame_as_write_dac_ramped() {
        let mut r = DacRamp::new();
        assert_eq!(r.toward(350), 6);
        assert_eq!(r.toward(350), 12);
        let up: u16 = (0..100).map(|_| r.toward(350)).last().unwrap();
        assert_eq!(up, 350);
        assert_eq!(r.toward(0), 338);
        assert_eq!(r.toward(330), 330); // never past its target
        assert_eq!(r.toward(9999), 336); // one step at a time
        let top: u16 = (0..200).map(|_| r.toward(9999)).last().unwrap();
        assert_eq!(top, DAC_CAP);
        r.cut();
        assert_eq!(r.code(), 0);
        assert_eq!(r.toward(0), 0);
    }

    #[test]
    fn right_turns_have_the_bigger_gain() {
        // The same ratio as the frame's two gains.
        let left = f32::from(t(1.0, 0.0, 0.0).steering - STEER_CENTRE);
        let right = f32::from(STEER_CENTRE - t(-0.5, 0.0, 0.0).steering) / 0.5;
        assert!((right / left - STEER_RIGHT_PER_LEFT).abs() < 1e-3);
        assert!((STEER_RIGHT_PER_LEFT - 1950.0 / 1550.0).abs() < 1e-6);
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
