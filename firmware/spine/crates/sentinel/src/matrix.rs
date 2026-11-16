//! The 8x8 LED matrix (a MAX7219): one picture at a time.

use crate::telemetry::{DriveMode, DriveState, ErrorCode, Telemetry};

/// The pictures, drawn by the 2025-26 team.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pattern {
    /// Before the first packet.
    Smile,
    /// The e-stop.
    Cross,
    /// The link to the Pi is lost.
    Exclaim,
    /// Autonomous: an "A".
    Auto,
    /// RC: an "R".
    Rc,
    /// An arrow up.
    Forward,
    /// An arrow down.
    Reverse,
    /// A filled circle: stopped or braking.
    Stop,
}

impl Pattern {
    /// Each row, top first; bit 7 is the left column.
    #[must_use]
    pub fn rows(self) -> [u8; 8] {
        match self {
            Pattern::Smile => [
                0b0011_1100,
                0b0100_0010,
                0b1010_0101,
                0b1000_0001,
                0b1010_0101,
                0b1001_1001,
                0b0100_0010,
                0b0011_1100,
            ],
            Pattern::Cross => [
                0b1000_0001,
                0b0100_0010,
                0b0010_0100,
                0b0001_1000,
                0b0001_1000,
                0b0010_0100,
                0b0100_0010,
                0b1000_0001,
            ],
            Pattern::Exclaim => [
                0b0001_1000,
                0b0001_1000,
                0b0001_1000,
                0b0001_1000,
                0b0000_0000,
                0b0000_0000,
                0b0001_1000,
                0b0001_1000,
            ],
            Pattern::Auto => [
                0b0001_1000,
                0b0010_0100,
                0b0100_0010,
                0b1111_1110,
                0b1000_0010,
                0b1000_0010,
                0b1000_0010,
                0b0000_0000,
            ],
            Pattern::Rc => [
                0b1111_1100,
                0b1000_0010,
                0b1000_0010,
                0b1111_1100,
                0b1001_0000,
                0b1000_1000,
                0b1000_0100,
                0b0000_0000,
            ],
            Pattern::Forward => [
                0b0001_1000,
                0b0011_1100,
                0b0111_1110,
                0b1111_1111,
                0b0001_1000,
                0b0001_1000,
                0b0001_1000,
                0b0000_0000,
            ],
            Pattern::Reverse => [
                0b0000_0000,
                0b0001_1000,
                0b0001_1000,
                0b0001_1000,
                0b1111_1111,
                0b0111_1110,
                0b0011_1100,
                0b0001_1000,
            ],
            Pattern::Stop => [
                0b0000_0000,
                0b0011_1100,
                0b0111_1110,
                0b0111_1110,
                0b0111_1110,
                0b0111_1110,
                0b0011_1100,
                0b0000_0000,
            ],
        }
    }

    /// What to show now. Normally the drive mode and the direction take turns, 0.8 s
    /// each; the e-stop and a lost link hold their own picture instead.
    #[must_use]
    pub fn now(telemetry: Option<&Telemetry>, comms_lost: bool, now_ms: u32) -> Pattern {
        let Some(t) = telemetry else {
            return Pattern::Smile;
        };
        if comms_lost {
            return Pattern::Exclaim;
        }
        if t.error == ErrorCode::EStop {
            return Pattern::Cross;
        }
        if (now_ms / SWAP_MS).is_multiple_of(2) {
            match t.drive_mode {
                DriveMode::Autonomous => Pattern::Auto,
                DriveMode::Rc => Pattern::Rc,
            }
        } else {
            match t.drive_state {
                DriveState::Forward => Pattern::Forward,
                DriveState::Reverse => Pattern::Reverse,
                DriveState::Stop | DriveState::Brake => Pattern::Stop,
            }
        }
    }
}

/// The mode and the direction take turns this often.
pub const SWAP_MS: u32 = 800;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::tests::sample;

    #[test]
    fn mode_and_direction_take_turns() {
        let t = sample(); // autonomous, forward
        assert_eq!(Pattern::now(Some(&t), false, 0), Pattern::Auto);
        assert_eq!(Pattern::now(Some(&t), false, 799), Pattern::Auto);
        assert_eq!(Pattern::now(Some(&t), false, 800), Pattern::Forward);
        assert_eq!(Pattern::now(Some(&t), false, 1600), Pattern::Auto);
    }

    #[test]
    fn braking_shows_stop() {
        let t = Telemetry {
            drive_state: DriveState::Brake,
            drive_mode: DriveMode::Rc,
            ..sample()
        };
        assert_eq!(Pattern::now(Some(&t), false, 0), Pattern::Rc);
        assert_eq!(Pattern::now(Some(&t), false, 800), Pattern::Stop);
    }

    #[test]
    fn alarms_hold_their_picture() {
        let estop = Telemetry {
            error: ErrorCode::EStop,
            ..sample()
        };
        for ms in [0, 800] {
            assert_eq!(Pattern::now(Some(&estop), false, ms), Pattern::Cross);
            assert_eq!(Pattern::now(Some(&estop), true, ms), Pattern::Exclaim);
        }
        assert_eq!(Pattern::now(None, false, 0), Pattern::Smile);
    }

    #[test]
    fn the_cross_is_symmetric() {
        let rows = Pattern::Cross.rows();
        assert!(rows.iter().all(|r| r.reverse_bits() == *r));
    }
}
