//! Version 2 of the frame (TODO 50): the same three values, with a sync byte, a version,
//! the Orin's counter, two flags and a CRC-8.
//!
//! ```text
//!  byte  0     1        2..5      6..9    10..13  14       15     16
//!        0xA5  version  steering  speed   accel   counter  flags  CRC-8 of bytes 1..15
//! ```
//!
//! The counter is the Orin's heartbeat: its output mux adds one for every command it
//! passes on, and the Motor-Pi only copies it. A counter that stops changing means the
//! Orin (or the network) stopped, however many frames the Motor-Pi still sends.

use crate::frame::{Frame, FrameError};

/// The first byte of every frame: 1010 0101, so a parser can find the next frame.
pub const SYNC: u8 = 0xA5;
/// This layout's version (byte 1).
pub const VERSION: u8 = 2;
/// Bytes in a version 2 frame.
pub const FRAME_V2_LEN: usize = 17;

/// Byte 15: what the Motor-Pi says about the command. Bits 2 to 7 must be 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Flags {
    /// Bit 0: the Motor-Pi has a fresh command from the Orin and no e-stop.
    pub armed: bool,
    /// Bit 1: `/safety/estop` is on. The ESP32 stops at once, whatever the counter.
    pub estop: bool,
}

impl Flags {
    const ARMED: u8 = 1 << 0;
    const ESTOP: u8 = 1 << 1;

    /// The flags as byte 15.
    pub fn to_byte(self) -> u8 {
        let bit = |on: bool, mask: u8| if on { mask } else { 0 };
        bit(self.armed, Self::ARMED) | bit(self.estop, Self::ESTOP)
    }

    /// The flags in byte 15, if no unknown bit is set.
    pub fn from_byte(byte: u8) -> Result<Flags, FrameError> {
        if byte & !(Self::ARMED | Self::ESTOP) != 0 {
            return Err(FrameError::Flags);
        }
        Ok(Flags {
            armed: byte & Self::ARMED != 0,
            estop: byte & Self::ESTOP != 0,
        })
    }
}

/// One version 2 frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameV2 {
    /// Steering, speed and accel, as in version 1.
    pub values: Frame,
    /// The Orin's heartbeat counter, copied by the Motor-Pi. Wraps from 255 to 0.
    pub counter: u8,
    pub flags: Flags,
}

impl FrameV2 {
    /// The 17 bytes, as `legacy_protocol.pack_frame_v2` sends them (without its clamp).
    pub fn encode(&self) -> [u8; FRAME_V2_LEN] {
        let mut bytes = [0; FRAME_V2_LEN];
        bytes[0] = SYNC;
        bytes[1] = VERSION;
        let values = [self.values.steering, self.values.speed, self.values.accel];
        for (chunk, value) in bytes[2..14].as_chunks_mut::<4>().0.iter_mut().zip(values) {
            *chunk = value.to_le_bytes();
        }
        bytes[14] = self.counter;
        bytes[15] = self.flags.to_byte();
        bytes[16] = crc8(&bytes[1..16]);
        bytes
    }

    /// A frame from 17 bytes: the sync byte, then the CRC, then the version and the
    /// flags. The values are not checked ([`Frame::check`] does that).
    pub fn decode(bytes: &[u8; FRAME_V2_LEN]) -> Result<FrameV2, FrameError> {
        if bytes[0] != SYNC {
            return Err(FrameError::Sync);
        }
        if crc8(&bytes[1..16]) != bytes[16] {
            return Err(FrameError::Checksum);
        }
        if bytes[1] != VERSION {
            return Err(FrameError::Version);
        }
        let flags = Flags::from_byte(bytes[15])?;
        let value = |at: usize| {
            let mut four = [0; 4];
            four.copy_from_slice(&bytes[at..at + 4]);
            f32::from_le_bytes(four)
        };
        Ok(FrameV2 {
            values: Frame {
                steering: value(2),
                speed: value(6),
                accel: value(10),
            },
            counter: bytes[14],
            flags,
        })
    }
}

/// CRC-8/SAE-J1850: polynomial 0x1D, start 0xFF, not reflected, final XOR 0xFF. The same
/// CRC as every CAN frame in `firmware/can/aav.dbc`. Its check value: 0x4B for
/// "123456789".
pub fn crc8(bytes: &[u8]) -> u8 {
    let step = |crc: u8| {
        if crc & 0x80 != 0 {
            (crc << 1) ^ 0x1D
        } else {
            crc << 1
        }
    };
    let crc = bytes
        .iter()
        .fold(0xFF, |crc, &b| (0..8).fold(crc ^ b, |c, _| step(c)));
    crc ^ 0xFF
}

/// The heartbeat rule: is this frame's counter new?
///
/// A counter is new when it differs from the frame before. The first frame after a boot
/// only sets the counter: it proves nothing yet. Frames come at 50 Hz and the Orin's
/// commands at 50 Hz, on clocks of their own, so a counter seen twice in a row, or one
/// that skips a value, is normal. Only "unchanged for 100 ms" means the Orin stopped:
/// the spine's command watchdog times that.
#[derive(Debug, Clone, Copy, Default)]
pub struct Heartbeat {
    last: Option<u8>,
}

impl Heartbeat {
    pub const fn new() -> Self {
        Heartbeat { last: None }
    }

    /// Note a frame's counter; true if it changed.
    pub fn fresh(&mut self, counter: u8) -> bool {
        let changed = self.last.is_some_and(|last| last != counter);
        self.last = Some(counter);
        changed
    }

    /// The last counter seen, if any.
    pub fn last(&self) -> Option<u8> {
        self.last
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(counter: u8, armed: bool, estop: bool) -> FrameV2 {
        FrameV2 {
            values: Frame {
                steering: 0.5,
                speed: -0.25,
                accel: 0.0,
            },
            counter,
            flags: Flags { armed, estop },
        }
    }

    #[test]
    fn crc_check_value() {
        assert_eq!(crc8(b"123456789"), 0x4B);
    }

    #[test]
    fn round_trip() {
        let f = frame(200, true, false);
        let bytes = f.encode();
        assert_eq!(
            (bytes[0], bytes[1], bytes[14], bytes[15]),
            (0xA5, 2, 200, 1)
        );
        assert_eq!(FrameV2::decode(&bytes), Ok(f));
        let f = frame(0, false, true);
        assert_eq!(f.encode()[15], 2);
        assert_eq!(FrameV2::decode(&f.encode()), Ok(f));
    }

    #[test]
    fn every_single_bit_error_is_caught() {
        let bytes = frame(7, true, false).encode();
        for bit in 8..FRAME_V2_LEN * 8 {
            let mut bad = bytes;
            bad[bit / 8] ^= 0x80 >> (bit % 8);
            assert!(FrameV2::decode(&bad).is_err(), "bit {bit}");
        }
    }

    #[test]
    fn a_wrong_sync_version_or_flag_is_turned_down() {
        let mut bad = frame(1, true, false).encode();
        bad[0] = 0x5A;
        assert_eq!(FrameV2::decode(&bad), Err(FrameError::Sync));

        let mut bad = frame(1, true, false).encode();
        bad[1] = 3;
        bad[16] = crc8(&bad[1..16]);
        assert_eq!(FrameV2::decode(&bad), Err(FrameError::Version));

        let mut bad = frame(1, true, false).encode();
        bad[15] |= 0x04;
        bad[16] = crc8(&bad[1..16]);
        assert_eq!(FrameV2::decode(&bad), Err(FrameError::Flags));
    }

    #[test]
    fn the_heartbeat_wants_a_change() {
        let mut h = Heartbeat::new();
        assert!(!h.fresh(5)); // the first frame proves nothing
        assert!(!h.fresh(5));
        assert!(h.fresh(6));
        assert!(h.fresh(8)); // a skipped value is fine
        assert!(!h.fresh(8));
        assert!(h.fresh(255));
        assert!(h.fresh(0)); // the wrap
        assert_eq!(h.last(), Some(0));
    }
}
