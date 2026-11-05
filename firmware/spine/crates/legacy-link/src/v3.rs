//! Version 3 of the frame (1 Nov): version 2, and a request for the steering actuator's
//! calibration (limp, or a jog target in counts).
//!
//! ```text
//!  byte  0     1   2..5      6..9   10..13  14       15     16        17..18        19
//!        0xA5  3   steering  speed  accel   counter  flags  cal mode  jog target    CRC-8 of 1..18
//! ```
//!
//! Byte 16: 0 off (drive), 1 limp, 2 jog. Bytes 17..18: the jog target, little-endian,
//! 0 to 4095; 0 unless the mode is jog. Anything else is [`FrameError::Field`].
//!
//! Why a new version, not flags in version 2: the jog target needs two more bytes, and a
//! firmware that only knows version 2 turns a version 3 frame down whole (the watchdog
//! then brakes), rather than misreading it. [`ParserV3`](crate::ParserV3) takes both
//! versions, so a version 2 sender keeps working.

use crate::frame::{Frame, FrameError};
use crate::steer_cal::{CalRequest, COUNTS_MAX};
use crate::v2::{crc8, Flags, FrameV2, SYNC};

/// This layout's version (byte 1).
pub const VERSION_3: u8 = 3;
/// Bytes in a version 3 frame.
pub const FRAME_V3_LEN: usize = 20;

/// One version 3 frame; or a version 2 frame read as one, with no calibration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameV3 {
    /// Steering, speed and accel, as in version 1.
    pub values: Frame,
    /// The Orin's heartbeat counter, copied by the Motor-Pi.
    pub counter: u8,
    pub flags: Flags,
    /// What the steering actuator should do: drive, limp or jog.
    pub calibration: CalRequest,
}

impl From<FrameV2> for FrameV3 {
    fn from(f: FrameV2) -> Self {
        FrameV3 {
            values: f.values,
            counter: f.counter,
            flags: f.flags,
            calibration: CalRequest::Off,
        }
    }
}

impl FrameV3 {
    /// The 20 bytes, as `legacy_protocol.pack_frame_v3` sends them (without its clamp).
    pub fn encode(&self) -> [u8; FRAME_V3_LEN] {
        let mut bytes = [0; FRAME_V3_LEN];
        bytes[0] = SYNC;
        bytes[1] = VERSION_3;
        let values = [self.values.steering, self.values.speed, self.values.accel];
        for (chunk, value) in bytes[2..14].chunks_exact_mut(4).zip(values) {
            chunk.copy_from_slice(&value.to_le_bytes());
        }
        bytes[14] = self.counter;
        bytes[15] = self.flags.to_byte();
        let (mode, target) = match self.calibration {
            CalRequest::Off => (0, 0),
            CalRequest::Limp => (1, 0),
            CalRequest::Jog(target) => (2, target),
        };
        bytes[16] = mode;
        bytes[17..19].copy_from_slice(&target.to_le_bytes());
        bytes[19] = crc8(&bytes[1..19]);
        bytes
    }

    /// A frame from 20 bytes: the sync byte, then the CRC, then the version, the flags
    /// and the calibration. The values are not checked ([`Frame::check`] does that).
    pub fn decode(bytes: &[u8; FRAME_V3_LEN]) -> Result<FrameV3, FrameError> {
        if bytes[0] != SYNC {
            return Err(FrameError::Sync);
        }
        if crc8(&bytes[1..19]) != bytes[19] {
            return Err(FrameError::Checksum);
        }
        if bytes[1] != VERSION_3 {
            return Err(FrameError::Version);
        }
        let flags = Flags::from_byte(bytes[15])?;
        let target = u16::from_le_bytes([bytes[17], bytes[18]]);
        let calibration = match (bytes[16], target) {
            (0, 0) => CalRequest::Off,
            (1, 0) => CalRequest::Limp,
            (2, t) if t <= COUNTS_MAX => CalRequest::Jog(t),
            _ => return Err(FrameError::Field),
        };
        let value = |at: usize| {
            let mut four = [0; 4];
            four.copy_from_slice(&bytes[at..at + 4]);
            f32::from_le_bytes(four)
        };
        Ok(FrameV3 {
            values: Frame {
                steering: value(2),
                speed: value(6),
                accel: value(10),
            },
            counter: bytes[14],
            flags,
            calibration,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(calibration: CalRequest) -> FrameV3 {
        FrameV3 {
            values: Frame {
                steering: 0.0,
                speed: 0.0,
                accel: -1.0,
            },
            counter: 12,
            flags: Flags {
                armed: false,
                estop: false,
            },
            calibration,
        }
    }

    #[test]
    fn round_trip() {
        for cal in [CalRequest::Off, CalRequest::Limp, CalRequest::Jog(2048)] {
            let f = frame(cal);
            let b = f.encode();
            assert_eq!((b[0], b[1], b[14]), (0xA5, 3, 12));
            assert_eq!(FrameV3::decode(&b), Ok(f));
        }
        let b = frame(CalRequest::Jog(0x0123)).encode();
        assert_eq!((b[16], b[17], b[18]), (2, 0x23, 0x01));
    }

    #[test]
    fn every_single_bit_error_is_caught() {
        let bytes = frame(CalRequest::Jog(1000)).encode();
        for bit in 8..FRAME_V3_LEN * 8 {
            let mut bad = bytes;
            bad[bit / 8] ^= 0x80 >> (bit % 8);
            assert!(FrameV3::decode(&bad).is_err(), "bit {bit}");
        }
    }

    #[test]
    fn an_unknown_mode_or_target_is_turned_down() {
        let fix = |mut b: [u8; FRAME_V3_LEN]| {
            b[19] = crc8(&b[1..19]);
            b
        };
        let mut b = frame(CalRequest::Off).encode();
        b[16] = 3;
        assert_eq!(FrameV3::decode(&fix(b)), Err(FrameError::Field));
        // A target with limp, or a target past 4095.
        let mut b = frame(CalRequest::Limp).encode();
        b[17] = 1;
        assert_eq!(FrameV3::decode(&fix(b)), Err(FrameError::Field));
        let mut b = frame(CalRequest::Jog(0)).encode();
        b[17..19].copy_from_slice(&4096u16.to_le_bytes());
        assert_eq!(FrameV3::decode(&fix(b)), Err(FrameError::Field));
        let mut b = frame(CalRequest::Off).encode();
        b[1] = 2;
        assert_eq!(FrameV3::decode(&fix(b)), Err(FrameError::Version));
    }

    #[test]
    fn a_version_2_frame_reads_as_no_calibration() {
        let v2 = FrameV2 {
            values: frame(CalRequest::Off).values,
            counter: 12,
            flags: Flags::default(),
        };
        assert_eq!(FrameV3::from(v2), frame(CalRequest::Off));
    }
}
