//! The PCA9555 16-bit I/O expander, on I2C.
//!
//! Sixteen pins in two ports: P00 to P07 (port 0) and P10 to P17 (port 1). Each pin
//! is an input or an output. On today's car the expander sits at 0x20, and pin P00
//! is the drive direction: high forward, low reverse.
//!
//! Plain `no_std` Rust against `embedded-hal` 1.0. From TI's PCA9555 data sheet
//! (SCPS131J), section 8.5.2, "Register Map": the command bytes (8.5.2.2) and the
//! registers (8.5.2.3). Registers come in pairs: after the command byte, the first
//! data byte goes to port 0 and the second to port 1.
//!
//! In every 16-bit value here, bit n is pin n: P00 is bit 0, P17 is bit 15.
//!
//! ```
//! use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
//! use pca9555::{Pca9555, Pin};
//!
//! let bus = Mock::new(&[Transaction::write(0x20, vec![0x02, 0xFE, 0xFF])]);
//! let mut expander = Pca9555::new(bus, 0x20);
//! expander.set_pin(Pin::P00, false).unwrap(); // reverse, on today's car
//! expander.release().done();
//! ```
#![no_std]

use embedded_hal::i2c::I2c;

/// The address with A2, A1 and A0 all low (today's car). The address is
/// `0100 A2 A1 A0`: 0x20 to 0x27.
pub const BASE_ADDRESS: u8 = 0x20;

/// The command bytes (section 8.5.2.2). Each names the port 0 register of a pair.
mod command {
    pub const INPUT: u8 = 0x00;
    pub const OUTPUT: u8 = 0x02;
    pub const POLARITY: u8 = 0x04;
    pub const CONFIGURATION: u8 = 0x06;
}

/// One of the sixteen pins.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Pin {
    P00,
    P01,
    P02,
    P03,
    P04,
    P05,
    P06,
    P07,
    P10,
    P11,
    P12,
    P13,
    P14,
    P15,
    P16,
    P17,
}

impl Pin {
    /// This pin's bit in a 16-bit value.
    #[must_use]
    pub const fn mask(self) -> u16 {
        1 << self as u8
    }
}

/// The expander on an I2C bus.
#[derive(Debug)]
pub struct Pca9555<I2C> {
    i2c: I2C,
    address: u8,
    outputs: u16,
}

impl<I2C: I2c> Pca9555<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Pca9555 {
            i2c,
            address,
            outputs: 0xFFFF, // the chip's own power-up value
        }
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    /// Give the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Set all sixteen output levels: 1 is high. Pins that are inputs ignore them.
    pub fn set_outputs(&mut self, levels: u16) -> Result<(), I2C::Error> {
        self.outputs = levels;
        self.write_pair(command::OUTPUT, levels)
    }

    /// Set one output, and keep the others as last asked for.
    pub fn set_pin(&mut self, pin: Pin, high: bool) -> Result<(), I2C::Error> {
        let levels = if high {
            self.outputs | pin.mask()
        } else {
            self.outputs & !pin.mask()
        };
        self.set_outputs(levels)
    }

    /// The output levels last asked for, even if that write failed.
    pub fn outputs(&self) -> u16 {
        self.outputs
    }

    /// Choose the inputs: 1 makes a pin an input, 0 an output. At power-up every
    /// pin is an input.
    pub fn set_inputs(&mut self, inputs: u16) -> Result<(), I2C::Error> {
        self.write_pair(command::CONFIGURATION, inputs)
    }

    /// Invert how these inputs read: 1 inverts. At power-up nothing is inverted.
    pub fn set_polarity(&mut self, inverted: u16) -> Result<(), I2C::Error> {
        self.write_pair(command::POLARITY, inverted)
    }

    /// Read the level on all sixteen pins, inputs and outputs.
    pub fn read_inputs(&mut self) -> Result<u16, I2C::Error> {
        let mut buf = [0; 2];
        self.i2c
            .write_read(self.address, &[command::INPUT], &mut buf)?;
        Ok(u16::from_le_bytes(buf))
    }

    /// The command byte, then port 0, then port 1.
    fn write_pair(&mut self, command: u8, value: u16) -> Result<(), I2C::Error> {
        let [port0, port1] = value.to_le_bytes();
        self.i2c.write(self.address, &[command, port0, port1])
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};
    use std::vec;

    #[test]
    fn pin_masks() {
        assert_eq!(Pin::P00.mask(), 0x0001);
        assert_eq!(Pin::P07.mask(), 0x0080);
        assert_eq!(Pin::P10.mask(), 0x0100);
        assert_eq!(Pin::P17.mask(), 0x8000);
    }

    #[test]
    fn the_cpp_setup_bytes() {
        // main.cpp setup(): polarity(ORIGINAL_ALL), direction(OUT_ALL), write(H_ALL).
        let bus = Mock::new(&[
            T::write(0x20, vec![0x04, 0x00, 0x00]),
            T::write(0x20, vec![0x06, 0x00, 0x00]),
            T::write(0x20, vec![0x02, 0xFF, 0xFF]),
        ]);
        let mut x = Pca9555::new(bus, BASE_ADDRESS);
        x.set_polarity(0x0000).unwrap();
        x.set_inputs(0x0000).unwrap();
        x.set_outputs(0xFFFF).unwrap();
        x.release().done();
    }

    #[test]
    fn set_pin_keeps_the_other_pins() {
        let bus = Mock::new(&[
            T::write(0x20, vec![0x02, 0xFE, 0xFF]),
            T::write(0x20, vec![0x02, 0xFE, 0x7F]),
            T::write(0x20, vec![0x02, 0xFF, 0x7F]),
        ]);
        let mut x = Pca9555::new(bus, BASE_ADDRESS);
        x.set_pin(Pin::P00, false).unwrap();
        x.set_pin(Pin::P17, false).unwrap();
        x.set_pin(Pin::P00, true).unwrap();
        assert_eq!(x.outputs(), 0x7FFF);
        x.release().done();
    }

    #[test]
    fn read_inputs_is_port_0_then_port_1() {
        let bus = Mock::new(&[T::write_read(0x20, vec![0x00], vec![0x34, 0x12])]);
        let mut x = Pca9555::new(bus, BASE_ADDRESS);
        assert_eq!(x.read_inputs().unwrap(), 0x1234);
        x.release().done();
    }

    #[test]
    fn a_failed_write_still_remembers_the_levels_asked_for() {
        let bus = Mock::new(&[
            T::write(0x20, vec![0x02, 0xFE, 0xFF]).with_error(ErrorKind::Other),
            T::write(0x20, vec![0x02, 0xFE, 0xFF]),
        ]);
        let mut x = Pca9555::new(bus, BASE_ADDRESS);
        assert_eq!(x.set_pin(Pin::P00, false), Err(ErrorKind::Other));
        assert_eq!(x.outputs(), 0xFFFE);
        x.set_outputs(x.outputs()).unwrap(); // a retry sends the same levels
        x.release().done();
    }
}
