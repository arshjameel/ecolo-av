//! The command bytes and variable offsets, from Pololu's *Jrk G2 Motor Controller
//! User's Guide* (pololu.com/docs/0J73):
//! - section 13, "I²C command encoding": how each command is put on the bus;
//! - section 11, "Command reference": what each command does;
//! - section 10, "Variable reference": where each variable sits.

/// "Set target (high resolution)": 0xC0 plus the target's low 5 bits, then its high
/// 7 bits (section 13).
pub const SET_TARGET: u8 = 0xC0;
/// "Stop motor", also called "motor off" (section 13).
pub const STOP_MOTOR: u8 = 0xFF;
/// "Get variables": then the offset; read up to 15 bytes back. No side effects
/// (section 13).
pub const GET_VARIABLES: u8 = 0xE5;
/// One-byte "get variable" commands: 0xA1 + offset reads a 16-bit variable
/// (section 13, "Get variables (one-byte commands)"). Some of them clear the
/// variable as a side effect: see [`TAKE_ERRORS_HALTING`].
pub const GET_VARIABLE_16: u8 = 0xA1;
/// "Get RAM settings": then the offset; read up to 15 bytes back. The settings the Jrk
/// is running with (section 13; Pololu's Arduino library, `JrkG2::getRAMSettings`).
pub const GET_RAM_SETTINGS: u8 = 0xEA;

/// The highest target the Jrk takes (section 11, "Set target").
pub const MAX_TARGET: u16 = 4095;

/// Where each variable sits (section 10).
pub mod offset {
    pub const TARGET: u8 = 0x02;
    pub const FEEDBACK: u8 = 0x04;
    pub const SCALED_FEEDBACK: u8 = 0x06;
    pub const ERROR_FLAGS_HALTING: u8 = 0x12;
    pub const ERROR_FLAGS_OCCURRED: u8 = 0x14;
}

/// Where each setting sits (section 6, "Setting reference"; Pololu's Arduino library,
/// `SettingOffset`).
pub mod setting {
    /// One byte: 0 none, 1 analog, 2 frequency.
    pub const FEEDBACK_MODE: u8 = 0x18;
}

/// 0xB3: read "Error flags halting", and clear its latched bits (section 13).
pub const TAKE_ERRORS_HALTING: u8 = GET_VARIABLE_16 + offset::ERROR_FLAGS_HALTING;
/// 0xB5: read "Error flags occurred", and clear it (section 13).
pub const TAKE_ERRORS_OCCURRED: u8 = GET_VARIABLE_16 + offset::ERROR_FLAGS_OCCURRED;

/// The two bytes of a "Set target" command. Above 4095 the target is sent as
/// 4095, as Pololu's Arduino library does.
#[must_use]
pub fn set_target(target: u16) -> [u8; 2] {
    let target = target.min(MAX_TARGET);
    [
        SET_TARGET | (target & 0x1F) as u8,
        (target >> 5) as u8 & 0x7F,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_target_matches_the_users_guide_example() {
        // Section 13: a target of 3229 (0b110010011101) is 0b11011101, 0b01100100.
        assert_eq!(set_target(3229), [0b1101_1101, 0b0110_0100]);
    }

    #[test]
    fn set_target_covers_the_whole_range() {
        assert_eq!(set_target(0), [0xC0, 0x00]);
        assert_eq!(set_target(31), [0xDF, 0x00]);
        assert_eq!(set_target(32), [0xC0, 0x01]);
        assert_eq!(set_target(4095), [0xDF, 0x7F]);
        assert_eq!(set_target(u16::MAX), [0xDF, 0x7F]);
    }

    #[test]
    fn todays_targets() {
        assert_eq!(set_target(400), [0xD0, 0x0C]); // brake released
        assert_eq!(set_target(1950), [0xDE, 0x3C]); // steering centre
        assert_eq!(set_target(3500), [0xCC, 0x6D]); // brake full
    }

    #[test]
    fn one_byte_reads_follow_the_guides_pattern() {
        // Section 13's table: 0xA7 is "Scaled feedback", 0xB3 and 0xB5 the error flags.
        assert_eq!(GET_VARIABLE_16 + offset::SCALED_FEEDBACK, 0xA7);
        assert_eq!(TAKE_ERRORS_HALTING, 0xB3);
        assert_eq!(TAKE_ERRORS_OCCURRED, 0xB5);
    }
}
