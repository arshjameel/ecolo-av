//! What a board crate provides: the parts that differ from chip to chip.
//!
//! A future ESP32, Pico 2 or STM32 crate implements [`Board`], and hands over its
//! I2C bus and the RC receiver's serial port. Everything else is in this crate and
//! is the same on every board.
//!
//! | Board part | Why it differs |
//! |---|---|
//! | the clock | each chip has its own timers |
//! | the hardware e-stop input | which pin, and whether it reads high or low when the loop is open |
//! | the wheel encoder counter | an interrupt, or the chip's own counter: PIO on a Pico 2, PCNT on an ESP32, a timer on an STM32 |
//! | the CAN controller | built into an ESP32 (TWAI) or an STM32; a Pico 2 needs a separate chip, such as an MCP2515 |
//!
//! The I2C bus and the serial port are not here: every chip's HAL already implements
//! `embedded-hal`'s `I2c` and `embedded-io`'s `Read`.

/// A classic CAN frame with an 11-bit ID: all the spine uses (`firmware/can/aav.dbc`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    id: u16,
    len: u8,
    data: [u8; 8],
}

impl CanFrame {
    /// The highest 11-bit ID.
    pub const MAX_ID: u16 = 0x7FF;

    /// None if the ID needs more than 11 bits, or there are more than 8 bytes.
    #[must_use]
    pub fn new(id: u16, data: &[u8]) -> Option<Self> {
        if id > Self::MAX_ID || data.len() > 8 {
            return None;
        }
        let mut frame = CanFrame {
            id,
            len: data.len() as u8,
            data: [0; 8],
        };
        frame.data[..data.len()].copy_from_slice(data);
        Some(frame)
    }

    #[must_use]
    pub fn id(&self) -> u16 {
        self.id
    }

    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data[..usize::from(self.len)]
    }
}

/// The wheel encoder's counter, and when it last changed (from `wheel-encoder`).
pub use wheel_encoder::WheelCount;

/// The board-specific parts.
pub trait Board {
    /// Milliseconds since power-up. Free-running: it may wrap.
    fn now_ms(&mut self) -> u32;

    /// The hardware e-stop, read raw: true while the loop is open (tripped).
    /// `SpineBoard` debounces it: it trips at once, and must stay closed to release.
    fn estop_open(&mut self) -> bool;

    /// The wheel encoder's counter.
    fn wheel(&mut self) -> WheelCount;

    /// The next CAN frame received, if any. Filtering by ID is optional: frames for
    /// someone else are ignored.
    fn can_receive(&mut self) -> Option<CanFrame>;

    /// Send a CAN frame. False if it could not go: bus off, or the queue is full.
    fn can_send(&mut self, frame: &CanFrame) -> bool;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_can_frame_keeps_its_length() {
        let f = CanFrame::new(0x181, &[1, 2, 3]).unwrap();
        assert_eq!((f.id(), f.data()), (0x181, &[1, 2, 3][..]));
        assert_eq!(CanFrame::new(0x010, &[]).unwrap().data(), &[] as &[u8]);
    }

    #[test]
    fn a_can_frame_has_limits() {
        assert!(CanFrame::new(0x7FF, &[0; 8]).is_some());
        assert!(CanFrame::new(0x800, &[]).is_none()); // needs 12 bits
        assert!(CanFrame::new(0x100, &[0; 9]).is_none());
    }
}
