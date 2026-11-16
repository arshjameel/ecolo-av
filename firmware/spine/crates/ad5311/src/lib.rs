//! The throttle DAC: an Analog Devices AD5311, a 10-bit DAC on I2C.
//!
//! Its output voltage goes to the motor controller's throttle input: code 0 is 0 V,
//! code 1023 is almost its supply voltage. Today's car caps the code at 700.
//!
//! Plain `no_std` Rust against `embedded-hal` 1.0. From the *AD5301/AD5311/AD5321*
//! data sheet, Rev. C: "Serial interface" (the address), "Input shift register"
//! (Figure 27), "Read operation" and "Power-down modes" (Table 6).
//!
//! # Which chip
//!
//! The 2025-26 firmware never names it. Three clues fit the AD5311:
//!
//! | Clue | The C++ | The AD5311 |
//! |---|---|---|
//! | Address | 0x0C | `000110` then the A0 pin: 0x0C with A0 low |
//! | Its own label | "DAC (A0=GND)" | the address pin is called A0 |
//! | The two bytes | `(v >> 6) & 0x0F`, `(v << 2) & 0xFC` | `x x PD1 PD0 D9..D6`, `D5..D0 x x` |
//!
//! TI's DAC101C081 takes the same two bytes, but answers at 0x0C only with its ADR0
//! pin floating; tied to ground it would be 0x0D. Either way this driver works.
//! TODO(Q13): read the part number off today's board.
//!
//! ```
//! use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
//!
//! let bus = Mock::new(&[Transaction::write(0x0C, vec![0x0A, 0xF0])]);
//! let mut dac = ad5311::Ad5311::with_limit(bus, 0x0C, 700);
//! assert_eq!(dac.set_code(1023).unwrap(), 700); // never above the limit
//! dac.release().done();
//! ```
#![no_std]

use embedded_hal::i2c::I2c;

/// The address with the A0 pin low (today's car), and high.
pub const ADDRESS_A0_LOW: u8 = 0x0C;
pub const ADDRESS_A0_HIGH: u8 = 0x0D;

/// Full scale: 10 bits.
pub const MAX_CODE: u16 = 1023;

/// What the output does: bits PD1 and PD0 (Table 6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// On: the output follows the code.
    Normal = 0,
    /// Off, the output pulled to ground through 1 kΩ.
    PullDown1k = 1,
    /// Off, pulled to ground through 100 kΩ.
    PullDown100k = 2,
    /// Off, the output left floating.
    Floating = 3,
}

impl Mode {
    fn from_bits(bits: u8) -> Mode {
        match bits & 0b11 {
            0 => Mode::Normal,
            1 => Mode::PullDown1k,
            2 => Mode::PullDown100k,
            _ => Mode::Floating,
        }
    }
}

/// What the DAC holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Register {
    pub code: u16,
    pub mode: Mode,
}

/// The two bytes for a code and a mode (Figure 27): `x x PD1 PD0 D9 D8 D7 D6`, then
/// `D5 D4 D3 D2 D1 D0 x x`, with the x bits sent as 0. Above 1023 the code is sent
/// as 1023.
#[must_use]
pub fn encode(code: u16, mode: Mode) -> [u8; 2] {
    let code = code.min(MAX_CODE);
    [(mode as u8) << 4 | (code >> 6) as u8, (code << 2) as u8]
}

/// Read the two bytes the DAC sends back ("Read operation"): the same layout.
#[must_use]
pub fn decode(bytes: [u8; 2]) -> Register {
    let code = u16::from(bytes[0] & 0x0F) << 6 | u16::from(bytes[1]) >> 2;
    Register {
        code,
        mode: Mode::from_bits(bytes[0] >> 4),
    }
}

/// The DAC on an I2C bus, with a hard limit on the code it will write.
#[derive(Debug)]
pub struct Ad5311<I2C> {
    i2c: I2C,
    address: u8,
    limit: u16,
}

impl<I2C: I2c> Ad5311<I2C> {
    /// No limit below full scale.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self::with_limit(i2c, address, MAX_CODE)
    }

    /// Never write a code above `limit`. Today's throttle stops at 700.
    pub fn with_limit(i2c: I2C, address: u8, limit: u16) -> Self {
        Ad5311 {
            i2c,
            address,
            limit: limit.min(MAX_CODE),
        }
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    pub fn limit(&self) -> u16 {
        self.limit
    }

    /// Give the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Set the output. Above the limit it writes the limit. Returns the code written.
    pub fn set_code(&mut self, code: u16) -> Result<u16, I2C::Error> {
        let code = code.min(self.limit);
        self.i2c.write(self.address, &encode(code, Mode::Normal))?;
        Ok(code)
    }

    /// Turn the output off in one of the off modes, with code 0.
    /// `Mode::Normal` just sets 0 V.
    pub fn power_down(&mut self, mode: Mode) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &encode(0, mode))
    }

    /// Read back the code and the mode. Also checks that the DAC answers.
    pub fn read(&mut self) -> Result<Register, I2C::Error> {
        let mut buf = [0; 2];
        self.i2c.read(self.address, &mut buf)?;
        Ok(decode(buf))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::vec;

    /// The 2025-26 firmware's writeDAC(), byte for byte (dac.cpp).
    fn cpp_write_dac(value: u16) -> [u8; 2] {
        let value = value & 0x3FF;
        [((value >> 6) & 0x0F) as u8, ((value << 2) & 0xFC) as u8]
    }

    #[test]
    fn the_same_bytes_as_the_cpp_for_every_code() {
        for code in 0..=MAX_CODE {
            assert_eq!(
                encode(code, Mode::Normal),
                cpp_write_dac(code),
                "code {code}"
            );
        }
    }

    #[test]
    fn known_codes() {
        assert_eq!(encode(0, Mode::Normal), [0x00, 0x00]);
        assert_eq!(encode(1, Mode::Normal), [0x00, 0x04]);
        assert_eq!(encode(700, Mode::Normal), [0x0A, 0xF0]); // today's cap
        assert_eq!(encode(1023, Mode::Normal), [0x0F, 0xFC]);
    }

    #[test]
    fn above_full_scale_is_full_scale_not_a_wrap() {
        // The C++ masked with 0x3FF, so 1024 would have become 0.
        assert_eq!(encode(1024, Mode::Normal), [0x0F, 0xFC]);
    }

    #[test]
    fn the_mode_bits() {
        assert_eq!(encode(0, Mode::PullDown1k), [0x10, 0x00]);
        assert_eq!(encode(0, Mode::PullDown100k), [0x20, 0x00]);
        assert_eq!(encode(0, Mode::Floating), [0x30, 0x00]);
    }

    #[test]
    fn decode_reverses_encode_and_ignores_the_x_bits() {
        for code in [0, 1, 511, 700, 1023] {
            for mode in [Mode::Normal, Mode::PullDown1k, Mode::Floating] {
                assert_eq!(decode(encode(code, mode)), Register { code, mode });
            }
        }
        assert_eq!(decode([0xCA, 0xF3]).code, 700);
    }

    #[test]
    fn set_code_stops_at_the_limit() {
        let bus = Mock::new(&[
            T::write(0x0C, vec![0x0A, 0xF0]),
            T::write(0x0C, vec![0x00, 0x00]),
        ]);
        let mut dac = Ad5311::with_limit(bus, ADDRESS_A0_LOW, 700);
        assert_eq!(dac.set_code(1023).unwrap(), 700);
        assert_eq!(dac.set_code(0).unwrap(), 0);
        dac.release().done();
    }

    #[test]
    fn a_limit_above_full_scale_is_full_scale() {
        let dac = Ad5311::with_limit(Mock::new(&[]), ADDRESS_A0_LOW, 5000);
        assert_eq!(dac.limit(), MAX_CODE);
        dac.release().done();
    }

    #[test]
    fn read_and_power_down() {
        let bus = Mock::new(&[
            T::read(0x0C, vec![0x0A, 0xF0]),
            T::write(0x0C, vec![0x10, 0x00]),
        ]);
        let mut dac = Ad5311::new(bus, ADDRESS_A0_LOW);
        let r = dac.read().unwrap();
        assert_eq!((r.code, r.mode), (700, Mode::Normal));
        dac.power_down(Mode::PullDown1k).unwrap();
        dac.release().done();
    }

    #[test]
    fn bus_errors_come_back() {
        let bus = Mock::new(&[T::write(0x0C, vec![0x00, 0x00]).with_error(ErrorKind::Other)]);
        let mut dac = Ad5311::new(bus, ADDRESS_A0_LOW);
        assert_eq!(dac.set_code(0), Err(ErrorKind::Other));
        dac.release().done();
    }
}
