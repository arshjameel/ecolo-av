//! Stand-in devices: an I2C bus with a healthy Jrk, DAC and expander on it, for a board
//! with nothing wired, such as the ESP32 in QEMU (`docs/esp32-bench.md`, "Before the
//! board: the emulator").
//!
//! The real drivers (`jrk-g2`, `ad5311`, `pca9555`) and [`Actuators`](crate::Actuators)
//! talk to it byte for byte, as they would to the car's devices. It answers every
//! transfer, keeps what each device was last told, and reports no errors, so the spine
//! can drive. [`StandInBus::devices`] says what the car's devices would be doing now.
//!
//! | Address (today's) | Device | What it keeps | What it answers |
//! |---|---|---|---|
//! | steering (0x05) | Jrk G2 | the target | analog feedback; scaled feedback = the target; no errors |
//! | brake (0x04) | Jrk G2 | the target | the same |
//! | DAC (0x0C) | AD5311 | the code | the code |
//! | expander (0x20) | PCA9555 | the output levels | 0 |
//! | any other | none | nothing | no acknowledge |
//!
//! Not for the car: on the car, a device that doesn't answer must fault.

use crate::calibration::{Addresses, Calibration};
use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation};
use jrk_g2::command::{offset, GET_RAM_SETTINGS, GET_VARIABLE_16, SET_TARGET};

/// The PCA9555's command byte for its output port pair (SCPS131J, 8.5.2.2).
const EXPANDER_OUTPUT: u8 = 0x02;
/// The Jrk's one-byte read of its scaled feedback (0xA7).
const SCALED_FEEDBACK: u8 = GET_VARIABLE_16 + offset::SCALED_FEEDBACK;

/// What the devices were last told, in their own units. `None`: never written.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DeviceTargets {
    /// The steering Jrk's target (400..3500; 1950 is straight ahead).
    pub steering: Option<u16>,
    /// The brake Jrk's target (400 released, 3500 full).
    pub brake: Option<u16>,
    /// The throttle DAC's code (0..700 today).
    pub dac: Option<u16>,
    /// The expander's 16 output levels (the direction is one of them).
    pub expander: Option<u16>,
}

/// The stand-in bus. Give each driver its own handle on it, as with the real bus
/// (`embedded-hal-bus`'s `RefCellDevice`).
#[derive(Debug, Clone)]
pub struct StandInBus {
    addresses: Addresses,
    targets: DeviceTargets,
}

impl StandInBus {
    /// Devices at the calibration's addresses.
    #[must_use]
    pub fn new(cal: &Calibration) -> Self {
        StandInBus {
            addresses: cal.addresses,
            targets: DeviceTargets::default(),
        }
    }

    /// What each device was last told.
    #[must_use]
    pub fn devices(&self) -> DeviceTargets {
        self.targets
    }

    /// One write, as the device at `address` takes it.
    fn take_write(&mut self, address: u8, bytes: &[u8]) {
        let a = self.addresses;
        let t = &mut self.targets;
        if address == a.steering {
            jrk_write(&mut t.steering, bytes);
        } else if address == a.brake {
            jrk_write(&mut t.brake, bytes);
        } else if address == a.dac {
            if let [high, low] = *bytes {
                t.dac = Some(ad5311::decode([high, low]).code);
            }
        } else if address == a.expander {
            // The output port's pair: command 0x02, then port 0 and port 1.
            if let [EXPANDER_OUTPUT, port0, port1] = *bytes {
                t.expander = Some(u16::from_le_bytes([port0, port1]));
            }
        }
    }

    /// The bytes a read answers, after `command` was written.
    fn answer(&self, address: u8, command: Option<u8>) -> [u8; 2] {
        let a = self.addresses;
        let t = self.targets;
        if address == a.dac {
            // The AD5311 sends back its two bytes, as written.
            return ad5311::encode(t.dac.unwrap_or(0), ad5311::Mode::Normal);
        }
        let jrk_target = if address == a.steering {
            t.steering
        } else if address == a.brake {
            t.brake
        } else {
            None
        };
        match command {
            // Scaled feedback: where the motor is. The stand-in is always there.
            Some(SCALED_FEEDBACK) => jrk_target.unwrap_or(0).to_le_bytes(),
            // Its only setting asked for, the feedback mode: 1, analog.
            Some(GET_RAM_SETTINGS) => [1, 0],
            // Error flags, and everything else: 0, no errors.
            _ => [0, 0],
        }
    }

    fn present(&self, address: u8) -> bool {
        let a = self.addresses;
        [a.steering, a.brake, a.dac, a.expander].contains(&address)
    }
}

/// A Jrk's set-target command; any other leaves the target as it was.
fn jrk_write(target: &mut Option<u16>, bytes: &[u8]) {
    if let [first, high] = *bytes {
        if first & 0xE0 == SET_TARGET {
            *target = Some(u16::from(first & 0x1F) | u16::from(high & 0x7F) << 5);
        }
    }
}

impl ErrorType for StandInBus {
    type Error = ErrorKind;
}

impl I2c for StandInBus {
    fn transaction(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
        if !self.present(address) {
            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
        }
        let mut command = None;
        for op in ops {
            match op {
                Operation::Write(bytes) => {
                    command = bytes.first().copied();
                    self.take_write(address, bytes);
                }
                Operation::Read(buf) => {
                    let answer = self.answer(address, command);
                    let padded = answer.iter().chain(core::iter::repeat(&0));
                    for (b, v) in buf.iter_mut().zip(padded) {
                        *b = *v;
                    }
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::actuators::{Actuators, Devices};
    use core::cell::RefCell;
    use spine_core::protocol::Mode;
    use spine_core::Outputs;

    fn outputs(throttle: f32, brake: f32) -> Outputs {
        Outputs {
            mode: Mode::Computer,
            throttle,
            brake,
            reverse: false,
            steering_target: 0.0,
            armed: true,
            estop_active: false,
            faults: 0,
        }
    }

    /// One handle on the shared bus, as embedded-hal-bus's `RefCellDevice` gives.
    struct Handle<'a>(&'a RefCell<StandInBus>);

    impl ErrorType for Handle<'_> {
        type Error = ErrorKind;
    }

    impl I2c for Handle<'_> {
        fn transaction(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), ErrorKind> {
            self.0.borrow_mut().transaction(address, ops)
        }
    }

    #[test]
    fn the_real_drivers_drive_it_without_faults() {
        let cal = Calibration::TODAY;
        let bus = RefCell::new(StandInBus::new(&cal));
        let devices = Devices::new(Handle(&bus), Handle(&bus), Handle(&bus), Handle(&bus), &cal);
        let mut act = Actuators::new(devices, cal);
        assert_eq!(act.init(), 0);
        let after_init = bus.borrow().devices();
        assert_eq!(after_init.brake, Some(3500));
        assert_eq!(after_init.dac, Some(0));
        assert_eq!(after_init.steering, Some(1950));
        assert_eq!(after_init.expander, Some(0xFFFF)); // forward: P00 high

        // Drive for a second: the brake lets go, the DAC rises at 600 codes a second.
        for ms in (0..=1000).step_by(10) {
            assert_eq!(act.apply(&outputs(0.5, 0.0), ms), 0);
            assert_eq!(act.read().faults, 0);
        }
        let driving = bus.borrow().devices();
        assert_eq!(driving.brake, Some(400));
        assert_eq!(driving.dac, Some(350));

        // A stop: the DAC to 0 at once, the brake full.
        assert_eq!(act.apply(&outputs(0.0, 1.0), 1010), 0);
        let stopped = bus.borrow().devices();
        assert_eq!((stopped.dac, stopped.brake), (Some(0), Some(3500)));
    }

    #[test]
    fn the_jrk_reports_its_target_as_its_feedback() {
        let cal = Calibration::TODAY;
        let mut bus = StandInBus::new(&cal);
        let mut jrk = jrk_g2::JrkG2::new(&mut bus, cal.addresses.steering);
        jrk.set_target(2725).unwrap();
        assert_eq!(jrk.scaled_feedback().unwrap(), 2725);
        assert_eq!(jrk.feedback_mode().unwrap(), jrk_g2::FeedbackMode::Analog);
        assert_eq!(jrk.take_error_flags().unwrap().0, 0);
    }

    #[test]
    fn nobody_else_answers() {
        let mut bus = StandInBus::new(&Calibration::TODAY);
        assert!(bus.write(0x48, &[0]).is_err());
        assert_eq!(bus.devices(), DeviceTargets::default());
    }
}
