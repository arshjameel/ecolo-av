//! A character LCD with an HD44780 controller, on six GPIO pins in 4-bit mode.
//!
//! The common 16x2 and 20x4 displays. Write only: the R/W pin is tied to ground, so the
//! driver never reads the busy flag, and waits the datasheet's longest times instead.
//!
//! ```text
//!   RS --- register select: low for a command, high for a character
//!   E  --- enable: the display reads D4-D7 as E falls
//!   D4-D7  one nibble at a time, the high one first
//! ```
//!
//! Timings from the Hitachi HD44780U datasheet (ADE-207-272): commands take 37 us, clear
//! and home 1.52 ms; E must stay high 450 ns. Plain `no_std` Rust against
//! `embedded-hal` 1.0, so it runs on any board.
#![no_std]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

/// The six pins. One type for all, as a HAL's plain output pin.
#[derive(Debug)]
pub struct Pins<P> {
    /// Register select: low for a command, high for a character.
    pub rs: P,
    /// Enable: the display latches a nibble as it falls.
    pub e: P,
    /// Data bit 4 (the nibble's lowest).
    pub d4: P,
    /// Data bit 5.
    pub d5: P,
    /// Data bit 6.
    pub d6: P,
    /// Data bit 7 (the nibble's highest).
    pub d7: P,
}

/// How many lines the display has: it decides the controller's line mode and the
/// address of each line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lines {
    /// 16x2, 20x2, 40x2.
    Two,
    /// 20x4: lines 2 and 3 continue lines 0 and 1 in the controller's memory.
    Four {
        /// Characters on a line.
        columns: u8,
    },
}

/// The cursor and blinking, for [`Hd44780::set_display`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cursor {
    /// No cursor.
    Hidden,
    /// An underline under the next character.
    Underline,
    /// An underline and a blinking block.
    Blinking,
}

/// Commands (datasheet, Table 6).
mod cmd {
    pub const CLEAR: u8 = 0x01;
    pub const HOME: u8 = 0x02;
    /// Entry mode: move right after each character, no shift.
    pub const ENTRY_RIGHT: u8 = 0x06;
    pub const DISPLAY: u8 = 0x08;
    pub const DISPLAY_ON: u8 = 0x04;
    pub const CURSOR_ON: u8 = 0x02;
    pub const BLINK_ON: u8 = 0x01;
    /// Function set: 4-bit, 5x8 dots; add `TWO_LINES`.
    pub const FUNCTION_4BIT: u8 = 0x20;
    pub const TWO_LINES: u8 = 0x08;
    pub const SET_DDRAM: u8 = 0x80;
}

const E_PULSE_NS: u32 = 1_000;
const COMMAND_US: u32 = 50;
const CLEAR_US: u32 = 2_000;

/// One HD44780 display.
#[derive(Debug)]
pub struct Hd44780<P, D> {
    pins: Pins<P>,
    delay: D,
    lines: Lines,
}

impl<P: OutputPin, D: DelayNs> Hd44780<P, D> {
    /// Takes the pins; [`init`](Self::init) must run before anything shows.
    pub fn new(pins: Pins<P>, delay: D, lines: Lines) -> Self {
        Hd44780 { pins, delay, lines }
    }

    /// Wakes the controller into 4-bit mode from any state (datasheet, Figure 24), then
    /// clears it, with the display on and no cursor.
    ///
    /// # Errors
    ///
    /// The pin's error, if setting a pin fails.
    pub fn init(&mut self) -> Result<(), P::Error> {
        self.pins.rs.set_low()?;
        self.pins.e.set_low()?;
        self.delay.delay_ms(50); // after power-up, Vcc above 2.7 V for 40 ms
        for wait_us in [4_500, 150, 150] {
            self.nibble(0x3)?; // 8-bit mode, three times: a known state
            self.delay.delay_us(wait_us);
        }
        self.nibble(0x2)?; // now 4-bit mode
        self.delay.delay_us(COMMAND_US);
        self.command(cmd::FUNCTION_4BIT | cmd::TWO_LINES)?;
        self.set_display(true, Cursor::Hidden)?;
        self.command(cmd::ENTRY_RIGHT)?;
        self.clear()
    }

    /// Blank the display, and put the cursor at the top left.
    ///
    /// # Errors
    ///
    /// The pin's error, if setting a pin fails.
    pub fn clear(&mut self) -> Result<(), P::Error> {
        self.command(cmd::CLEAR)?;
        self.delay.delay_us(CLEAR_US);
        Ok(())
    }

    /// The cursor to the top left, the text unchanged.
    ///
    /// # Errors
    ///
    /// The pin's error, if setting a pin fails.
    pub fn home(&mut self) -> Result<(), P::Error> {
        self.command(cmd::HOME)?;
        self.delay.delay_us(CLEAR_US);
        Ok(())
    }

    /// The display on or off (the text is kept), and the cursor.
    ///
    /// # Errors
    ///
    /// The pin's error, if setting a pin fails.
    pub fn set_display(&mut self, on: bool, cursor: Cursor) -> Result<(), P::Error> {
        let mut c = cmd::DISPLAY;
        if on {
            c |= cmd::DISPLAY_ON;
        }
        c |= match cursor {
            Cursor::Hidden => 0,
            Cursor::Underline => cmd::CURSOR_ON,
            Cursor::Blinking => cmd::CURSOR_ON | cmd::BLINK_ON,
        };
        self.command(c)
    }

    /// Move the cursor to `column` of `line`, both from 0.
    ///
    /// # Errors
    ///
    /// The pin's error, if setting a pin fails.
    pub fn set_cursor(&mut self, column: u8, line: u8) -> Result<(), P::Error> {
        let start = match (self.lines, line) {
            (_, 0) => 0x00,
            (_, 1) => 0x40,
            (Lines::Four { columns }, 2) => columns,
            (Lines::Four { columns }, _) => 0x40 + columns,
            (Lines::Two, _) => 0x40,
        };
        self.command(cmd::SET_DDRAM | (start + column))
    }

    /// Characters from the cursor on: ASCII shows as itself; the display's own font maps
    /// the rest.
    ///
    /// # Errors
    ///
    /// The pin's error, if setting a pin fails.
    pub fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), P::Error> {
        bytes.iter().try_for_each(|&b| self.write(true, b))
    }

    /// A whole line, from its first column.
    ///
    /// # Errors
    ///
    /// The pin's error, if setting a pin fails.
    pub fn write_line(&mut self, line: u8, bytes: &[u8]) -> Result<(), P::Error> {
        self.set_cursor(0, line)?;
        self.write_bytes(bytes)
    }

    /// Give the pins back.
    pub fn release(self) -> (Pins<P>, D) {
        (self.pins, self.delay)
    }

    fn command(&mut self, c: u8) -> Result<(), P::Error> {
        self.write(false, c)
    }

    /// One byte: the high nibble, then the low one.
    fn write(&mut self, character: bool, byte: u8) -> Result<(), P::Error> {
        self.pins.rs.set_state(character.into())?;
        self.nibble(byte >> 4)?;
        self.nibble(byte & 0x0F)?;
        self.delay.delay_us(COMMAND_US);
        Ok(())
    }

    /// D4-D7 from the low four bits, then a pulse on E.
    fn nibble(&mut self, n: u8) -> Result<(), P::Error> {
        let bit = |i: u8| ((n >> i) & 1 == 1).into();
        self.pins.d4.set_state(bit(0))?;
        self.pins.d5.set_state(bit(1))?;
        self.pins.d6.set_state(bit(2))?;
        self.pins.d7.set_state(bit(3))?;
        self.pins.e.set_high()?;
        self.delay.delay_ns(E_PULSE_NS);
        self.pins.e.set_low()?;
        self.delay.delay_ns(E_PULSE_NS);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::cell::RefCell;
    use core::convert::Infallible;
    use std::rc::Rc;
    use std::vec::Vec;

    /// What the display saw: each (RS, nibble) latched on E's falling edge, and the time.
    #[derive(Default)]
    struct Bus {
        levels: u8,
        latched: Vec<(bool, u8)>,
        waited_ns: u64,
    }

    /// A pin that sets one bit of the shared bus; bit 1 is E.
    struct Pin {
        bit: u8,
        bus: Rc<RefCell<Bus>>,
    }

    const RS: u8 = 0;
    const E: u8 = 1;

    impl embedded_hal::digital::ErrorType for Pin {
        type Error = Infallible;
    }

    impl OutputPin for Pin {
        fn set_low(&mut self) -> Result<(), Infallible> {
            let mut bus = self.bus.borrow_mut();
            let was = bus.levels;
            bus.levels &= !(1 << self.bit);
            if self.bit == E && was & (1 << E) != 0 {
                let rs = bus.levels & (1 << RS) != 0;
                let nibble = (bus.levels >> 2) & 0x0F;
                bus.latched.push((rs, nibble));
            }
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Infallible> {
            self.bus.borrow_mut().levels |= 1 << self.bit;
            Ok(())
        }
    }

    struct Clock(Rc<RefCell<Bus>>);

    impl DelayNs for Clock {
        fn delay_ns(&mut self, ns: u32) {
            self.0.borrow_mut().waited_ns += u64::from(ns);
        }
    }

    fn display(lines: Lines) -> (Hd44780<Pin, Clock>, Rc<RefCell<Bus>>) {
        let bus = Rc::new(RefCell::new(Bus::default()));
        let pin = |bit| Pin {
            bit,
            bus: bus.clone(),
        };
        let pins = Pins {
            rs: pin(RS),
            e: pin(E),
            d4: pin(2),
            d5: pin(3),
            d6: pin(4),
            d7: pin(5),
        };
        (Hd44780::new(pins, Clock(bus.clone()), lines), bus)
    }

    /// The bytes the display got, put back together from nibbles.
    fn bytes(nibbles: &[(bool, u8)]) -> Vec<(bool, u8)> {
        nibbles
            .chunks(2)
            .map(|pair| (pair[0].0, (pair[0].1 << 4) | pair[1].1))
            .collect()
    }

    #[test]
    fn init_wakes_it_into_four_bit_mode() {
        let (mut lcd, bus) = display(Lines::Two);
        lcd.init().unwrap();
        let bus = bus.borrow();
        let wake: Vec<_> = bus.latched[..4].to_vec();
        assert_eq!(wake, [(false, 3), (false, 3), (false, 3), (false, 2)]);
        // Then: 4-bit and two lines, display on with no cursor, left to right, clear.
        assert_eq!(
            bytes(&bus.latched[4..]),
            [(false, 0x28), (false, 0x0C), (false, 0x06), (false, 0x01)]
        );
        assert!(bus.waited_ns >= 50_000_000 + 4_800_000); // power-up and the wake-up waits
    }

    #[test]
    fn a_line_goes_to_its_address() {
        let (mut lcd, bus) = display(Lines::Two);
        lcd.write_line(1, b"Hi").unwrap();
        assert_eq!(
            bytes(&bus.borrow().latched),
            [(false, 0xC0), (true, b'H'), (true, b'i')]
        );
    }

    #[test]
    fn four_line_displays_interleave() {
        let (mut lcd, bus) = display(Lines::Four { columns: 20 });
        for line in 0..4 {
            lcd.set_cursor(1, line).unwrap();
        }
        let addresses: Vec<u8> = bytes(&bus.borrow().latched).iter().map(|b| b.1).collect();
        assert_eq!(addresses, [0x81, 0xC1, 0x95, 0xD5]);
    }

    #[test]
    fn cursor_modes() {
        let (mut lcd, bus) = display(Lines::Two);
        lcd.set_display(true, Cursor::Blinking).unwrap();
        lcd.set_display(false, Cursor::Underline).unwrap();
        assert_eq!(bytes(&bus.borrow().latched), [(false, 0x0F), (false, 0x0A)]);
    }

    #[test]
    fn clear_waits_for_the_display() {
        let (mut lcd, bus) = display(Lines::Two);
        lcd.clear().unwrap();
        assert!(bus.borrow().waited_ns >= 1_520_000);
    }
}
