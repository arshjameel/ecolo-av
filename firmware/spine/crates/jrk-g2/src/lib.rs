//! The Pololu Jrk G2 motor controller, over I2C.
//!
//! A Jrk drives one motor to a target (0 to 4095), using its own feedback sensor.
//! Today's car has two: steering at address 0x05 and the brake at 0x04. This driver
//! sends the commands the 2025-26 firmware sent, and reads the feedback and the
//! error flags.
//!
//! Plain `no_std` Rust against `embedded-hal` 1.0: it runs on any board, and in
//! `cargo test` with a mock bus. The bytes come from Pololu's *Jrk G2 Motor
//! Controller User's Guide* (pololu.com/docs/0J73); see [`command`].
//!
//! Before it works, from the user's guide:
//! - Set the Jrk's input mode to "Serial / I²C / USB", or it ignores targets
//!   (section 11, "Set target").
//! - The Jrk stretches the I2C clock, so the board's I2C must allow that
//!   (section 13, "Clock stretching").
//! - Above 100 kHz, leave the bus quiet for 100 µs after each read (section 13).
//!   The 2025-26 firmware ran at the ESP32's default 100 kHz.
//!
//! ```
//! use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
//! use jrk_g2::JrkG2;
//!
//! let bus = Mock::new(&[Transaction::write(0x05, vec![0xDE, 0x3C])]);
//! let mut steering = JrkG2::new(bus, 0x05);
//! steering.set_target(1950).unwrap(); // the steering's centre on today's car
//! steering.release().done();
//! ```
#![no_std]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod command;
mod flags;

pub use command::MAX_TARGET;
pub use flags::ErrorFlags;

use command::{offset, setting};
use embedded_hal::i2c::I2c;

/// A new Jrk's I2C address (section 13). Today's car uses 0x05 and 0x04.
pub const DEFAULT_ADDRESS: u8 = 0x0B;

/// Where the Jrk's feedback comes from: its "Feedback mode" setting (section 6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeedbackMode {
    /// No sensor: open-loop speed control. The feedback reads 0, and means nothing.
    None,
    /// A voltage on the FBA pin, from a potentiometer, say: a position.
    Analog,
    /// Pulses on the FBT pin: a speed.
    Frequency,
    /// A value this driver doesn't know.
    Other(u8),
}

impl FeedbackMode {
    /// The mode for the setting's byte.
    #[must_use]
    pub fn from_byte(byte: u8) -> FeedbackMode {
        match byte {
            0 => FeedbackMode::None,
            1 => FeedbackMode::Analog,
            2 => FeedbackMode::Frequency,
            other => FeedbackMode::Other(other),
        }
    }

    /// True if the scaled feedback says where the motor is: analog feedback only.
    #[must_use]
    pub fn is_position(self) -> bool {
        self == FeedbackMode::Analog
    }
}

/// One Jrk G2 on an I2C bus.
#[derive(Debug)]
pub struct JrkG2<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C: I2c> JrkG2<I2C> {
    /// `address` is the Jrk's 7-bit I2C address: its "Device number" setting.
    pub fn new(i2c: I2C, address: u8) -> Self {
        JrkG2 { i2c, address }
    }

    /// The bus address it was made with.
    #[must_use]
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Give the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Set the target, 0 to 4095. With analog feedback it is a position. Above 4095
    /// it is sent as 4095. It also clears "awaiting command" and restarts the Jrk's
    /// serial timeout (section 11).
    ///
    /// # Errors
    ///
    /// The bus's error, if the transfer fails.
    pub fn set_target(&mut self, target: u16) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &command::set_target(target))
    }

    /// Stop the motor. The Jrk then waits for a new target ("awaiting command").
    ///
    /// # Errors
    ///
    /// The bus's error, if the transfer fails.
    pub fn stop_motor(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[command::STOP_MOTOR])
    }

    /// The target the Jrk is working to.
    ///
    /// # Errors
    ///
    /// The bus's error, if the transfer fails.
    pub fn target(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(command::GET_VARIABLE_16 + offset::TARGET)
    }

    /// The raw feedback reading: with analog feedback, 0 to about 4092 for 0 V to 5 V
    /// on the FBA pin (section 10).
    ///
    /// # Errors
    ///
    /// The bus's error, if the transfer fails.
    pub fn feedback(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(command::GET_VARIABLE_16 + offset::FEEDBACK)
    }

    /// The feedback after the Jrk's own scaling: in the same units as the target,
    /// when the Jrk's feedback is calibrated (section 10).
    ///
    /// # Errors
    ///
    /// The bus's error, if the transfer fails.
    pub fn scaled_feedback(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(command::GET_VARIABLE_16 + offset::SCALED_FEEDBACK)
    }

    /// Where the feedback comes from: the "Feedback mode" setting the Jrk runs with.
    /// The 2024-25 firmware read the steering's position as the scaled feedback
    /// (`getScaledFeedback()`); that is a position only when this is
    /// [`FeedbackMode::Analog`].
    ///
    /// # Errors
    ///
    /// The bus's error, if the transfer fails.
    pub fn feedback_mode(&mut self) -> Result<FeedbackMode, I2C::Error> {
        let mut buf = [0; 1];
        let cmd = [command::GET_RAM_SETTINGS, setting::FEEDBACK_MODE];
        self.i2c.write_read(self.address, &cmd, &mut buf)?;
        Ok(FeedbackMode::from_byte(buf[0]))
    }

    /// What is stopping the motor now ("Error flags halting"). Clears nothing.
    ///
    /// # Errors
    ///
    /// The bus's error, if the transfer fails.
    pub fn error_flags(&mut self) -> Result<ErrorFlags, I2C::Error> {
        let mut buf = [0; 2];
        let cmd = [command::GET_VARIABLES, offset::ERROR_FLAGS_HALTING];
        self.i2c.write_read(self.address, &cmd, &mut buf)?;
        Ok(ErrorFlags(u16::from_le_bytes(buf)))
    }

    /// What is stopping the motor now, and clear the latched errors (0xB3). This is
    /// what the 2025-26 firmware's `getErrorFlagsHalting()` did after every target.
    /// An error whose cause is still there is set again at once.
    ///
    /// # Errors
    ///
    /// The bus's error, if the transfer fails.
    pub fn take_error_flags(&mut self) -> Result<ErrorFlags, I2C::Error> {
        self.read_u16(command::TAKE_ERRORS_HALTING).map(ErrorFlags)
    }

    /// Every error since the last call, enabled or not; then clear them (0xB5).
    ///
    /// # Errors
    ///
    /// The bus's error, if the transfer fails.
    pub fn take_errors_occurred(&mut self) -> Result<ErrorFlags, I2C::Error> {
        self.read_u16(command::TAKE_ERRORS_OCCURRED).map(ErrorFlags)
    }

    /// Check that the Jrk answers, by reading its target. Has no side effects.
    ///
    /// # Errors
    ///
    /// The bus's error: no Jrk answers at the address, say.
    pub fn ping(&mut self) -> Result<(), I2C::Error> {
        self.target().map(|_| ())
    }

    /// A one-byte "get variable" command, then two bytes back, low byte first.
    fn read_u16(&mut self, cmd: u8) -> Result<u16, I2C::Error> {
        let mut buf = [0; 2];
        self.i2c.write_read(self.address, &[cmd], &mut buf)?;
        Ok(u16::from_le_bytes(buf))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::vec;

    const STEERING: u8 = 0x05;
    const BRAKE: u8 = 0x04;

    /// Run `f` against a Jrk at `address` on a mock bus expecting `expected`.
    fn with_jrk(address: u8, expected: &[T], f: impl FnOnce(&mut JrkG2<Mock>)) {
        let mut jrk = JrkG2::new(Mock::new(expected), address);
        f(&mut jrk);
        jrk.release().done();
    }

    #[test]
    fn set_target_writes_two_bytes() {
        with_jrk(STEERING, &[T::write(STEERING, vec![0xDE, 0x3C])], |j| {
            j.set_target(1950).unwrap();
        });
        with_jrk(BRAKE, &[T::write(BRAKE, vec![0xCC, 0x6D])], |j| {
            j.set_target(3500).unwrap();
        });
    }

    #[test]
    fn a_target_above_4095_is_sent_as_4095() {
        with_jrk(BRAKE, &[T::write(BRAKE, vec![0xDF, 0x7F])], |j| {
            j.set_target(5000).unwrap();
        });
    }

    #[test]
    fn stop_motor_is_one_byte() {
        with_jrk(BRAKE, &[T::write(BRAKE, vec![0xFF])], |j| {
            j.stop_motor().unwrap()
        });
    }

    #[test]
    fn reads_are_little_endian() {
        let expected = [
            T::write_read(STEERING, vec![0xA3], vec![0x9E, 0x07]),
            T::write_read(STEERING, vec![0xA5], vec![0x00, 0x08]),
            T::write_read(STEERING, vec![0xA7], vec![0xFF, 0x0F]),
        ];
        with_jrk(STEERING, &expected, |j| {
            assert_eq!(j.target().unwrap(), 1950);
            assert_eq!(j.feedback().unwrap(), 2048);
            assert_eq!(j.scaled_feedback().unwrap(), 4095);
        });
    }

    #[test]
    fn error_flags_with_and_without_clearing() {
        let expected = [
            // Get variables at offset 0x12: no side effects.
            T::write_read(BRAKE, vec![0xE5, 0x12], vec![0x21, 0x00]),
            // 0xB3 reads and clears; 0xB5 the same for "occurred".
            T::write_read(BRAKE, vec![0xB3], vec![0x01, 0x00]),
            T::write_read(BRAKE, vec![0xB5], vec![0x00, 0x20]),
        ];
        with_jrk(BRAKE, &expected, |j| {
            let halting = j.error_flags().unwrap();
            assert!(halting.contains(ErrorFlags::FEEDBACK_DISCONNECT));
            assert!(halting.is_fault());
            assert!(!j.take_error_flags().unwrap().is_fault()); // only "awaiting command"
            assert_eq!(
                j.take_errors_occurred().unwrap(),
                ErrorFlags(ErrorFlags::HARD_OVERCURRENT)
            );
        });
    }

    #[test]
    fn feedback_mode_reads_one_ram_setting() {
        let expected = [
            T::write_read(STEERING, vec![0xEA, 0x18], vec![0x01]),
            T::write_read(STEERING, vec![0xEA, 0x18], vec![0x00]),
            T::write_read(STEERING, vec![0xEA, 0x18], vec![0x07]),
        ];
        with_jrk(STEERING, &expected, |j| {
            assert_eq!(j.feedback_mode().unwrap(), FeedbackMode::Analog);
            assert_eq!(j.feedback_mode().unwrap(), FeedbackMode::None);
            assert_eq!(j.feedback_mode().unwrap(), FeedbackMode::Other(7));
        });
        assert!(FeedbackMode::Analog.is_position());
        assert!(!FeedbackMode::None.is_position());
        assert!(!FeedbackMode::Frequency.is_position());
    }

    #[test]
    fn ping_reads_the_target() {
        with_jrk(
            STEERING,
            &[T::write_read(STEERING, vec![0xA3], vec![0, 0])],
            |j| {
                j.ping().unwrap();
            },
        );
    }

    #[test]
    fn bus_errors_come_back() {
        let expected = [
            T::write(STEERING, vec![0xDE, 0x3C]).with_error(ErrorKind::Other),
            T::write_read(STEERING, vec![0xA7], vec![0, 0]).with_error(ErrorKind::Other),
        ];
        with_jrk(STEERING, &expected, |j| {
            assert_eq!(j.set_target(1950), Err(ErrorKind::Other));
            assert_eq!(j.scaled_feedback(), Err(ErrorKind::Other));
        });
    }
}
