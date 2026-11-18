//! The Jrk's error bits.
//!
//! The same bits are used by "Error flags halting" (what is stopping the motor now)
//! and "Error flags occurred" (what happened since the last read). From the user's
//! guide, section 7.7, "Error handling".

/// A set of Jrk error bits.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ErrorFlags(pub u16);

impl ErrorFlags {
    /// At power-up, and after "Stop motor": the Jrk waits for a target. Not a fault.
    pub const AWAITING_COMMAND: u16 = 1 << 0;
    /// The motor supply (VIN) is too low.
    pub const NO_POWER: u16 = 1 << 1;
    /// The motor driver shut down (too hot, too much current, or low voltage).
    pub const MOTOR_DRIVER: u16 = 1 << 2;
    /// RC input mode only: the RC signal is bad.
    pub const INPUT_INVALID: u16 = 1 << 3;
    /// Analog or RC input mode only: the input is disconnected.
    pub const INPUT_DISCONNECT: u16 = 1 << 4;
    /// The feedback sensor reads outside its error limits: a loose wire, say.
    pub const FEEDBACK_DISCONNECT: u16 = 1 << 5;
    /// The current went over the soft limit.
    pub const SOFT_OVERCURRENT: u16 = 1 << 6;
    /// Serial: a stop bit was missing (a framing error).
    pub const SERIAL_SIGNAL: u16 = 1 << 7;
    /// Serial: a byte came before the last was read.
    pub const SERIAL_OVERRUN: u16 = 1 << 8;
    /// Serial: the receive buffer filled up.
    pub const SERIAL_RX_BUFFER_FULL: u16 = 1 << 9;
    /// Serial: a command's CRC was wrong.
    pub const SERIAL_CRC: u16 = 1 << 10;
    /// A badly formed command arrived, over serial or I2C.
    pub const SERIAL_PROTOCOL: u16 = 1 << 11;
    /// No command for longer than the Jrk's "serial timeout" setting.
    pub const SERIAL_TIMEOUT: u16 = 1 << 12;
    /// The motor driver's own hard current limit tripped.
    pub const HARD_OVERCURRENT: u16 = 1 << 13;

    /// True if every bit in `bits` is set.
    #[must_use]
    pub fn contains(self, bits: u16) -> bool {
        self.0 & bits == bits
    }

    /// True if anything other than "awaiting command" is set: something is wrong
    /// with the motor, its power or its sensor.
    #[must_use]
    pub fn is_fault(self) -> bool {
        self.0 & !Self::AWAITING_COMMAND != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn awaiting_command_is_not_a_fault() {
        assert!(!ErrorFlags(0).is_fault());
        assert!(!ErrorFlags(ErrorFlags::AWAITING_COMMAND).is_fault());
    }

    #[test]
    fn anything_else_is() {
        let flags = ErrorFlags(ErrorFlags::AWAITING_COMMAND | ErrorFlags::FEEDBACK_DISCONNECT);
        assert!(flags.is_fault());
        assert!(flags.contains(ErrorFlags::FEEDBACK_DISCONNECT));
        assert!(!flags.contains(ErrorFlags::NO_POWER));
        assert!(ErrorFlags(ErrorFlags::HARD_OVERCURRENT).is_fault());
    }
}
