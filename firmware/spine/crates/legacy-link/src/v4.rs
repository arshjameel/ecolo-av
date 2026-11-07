//! Version 4 of the frame (T3.6, 6 Nov): version 3, and the car's wheel speed, so the ESP32
//! can enforce the speed limit itself: it has no wheel sensor of its own.
//!
//! ```text
//!  byte  0     1   2..18            19..22       23
//!        0xA5  4   as in version 3  wheel speed  CRC-8 of 1..22
//! ```
//!
//! Bytes 19..22: m/s, an `f32`, little-endian, positive forward. NaN: the sender doesn't
//! know the speed; any other value that isn't finite is [`FrameError::Field`].
//!
//! Why a new version: as with version 3, a firmware that doesn't know it turns the frame
//! down whole (the watchdog then brakes), rather than misreading it.
//! [`ParserV4`](crate::ParserV4) takes versions 2 and 3 too, with no speed.

use crate::frame::FrameError;
use crate::v2::{crc8, SYNC};
use crate::v3::{FrameV3, FRAME_V3_LEN, VERSION_3};

/// This layout's version (byte 1).
pub const VERSION_4: u8 = 4;
/// Bytes in a version 4 frame.
pub const FRAME_V4_LEN: usize = 24;

/// The car's speed, as a frame carries it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WheelSpeed {
    /// A frame of an earlier version: this sender has no speed to send at all.
    NotSent,
    /// A version 4 frame whose sender doesn't know the speed right now.
    Unknown,
    /// m/s, positive forward.
    Known(f32),
}

/// One version 4 frame; or a version 2 or 3 frame read as one, with no speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameV4 {
    /// What version 3 carries: the command, the heartbeat, the calibration.
    pub command: FrameV3,
    pub wheel_speed: WheelSpeed,
}

impl From<FrameV3> for FrameV4 {
    fn from(command: FrameV3) -> Self {
        FrameV4 {
            command,
            wheel_speed: WheelSpeed::NotSent,
        }
    }
}

impl FrameV4 {
    /// The 24 bytes.
    pub fn encode(&self) -> [u8; FRAME_V4_LEN] {
        let mut bytes = [0; FRAME_V4_LEN];
        bytes[..FRAME_V3_LEN - 1].copy_from_slice(&self.command.encode()[..FRAME_V3_LEN - 1]);
        bytes[1] = VERSION_4;
        let speed = match self.wheel_speed {
            WheelSpeed::Known(speed) => speed,
            WheelSpeed::Unknown | WheelSpeed::NotSent => f32::NAN,
        };
        bytes[19..23].copy_from_slice(&speed.to_le_bytes());
        bytes[23] = crc8(&bytes[1..23]);
        bytes
    }

    /// A frame from 24 bytes: the sync byte, then the CRC, then the rest, as version 3.
    pub fn decode(bytes: &[u8; FRAME_V4_LEN]) -> Result<FrameV4, FrameError> {
        if bytes[0] != SYNC {
            return Err(FrameError::Sync);
        }
        if crc8(&bytes[1..23]) != bytes[23] {
            return Err(FrameError::Checksum);
        }
        if bytes[1] != VERSION_4 {
            return Err(FrameError::Version);
        }
        let speed = f32::from_le_bytes([bytes[19], bytes[20], bytes[21], bytes[22]]);
        if speed.is_infinite() {
            return Err(FrameError::Field);
        }
        // Version 3's own rules for the fields they share, with its CRC worked out here.
        let mut v3 = [0; FRAME_V3_LEN];
        v3[..FRAME_V3_LEN - 1].copy_from_slice(&bytes[..FRAME_V3_LEN - 1]);
        v3[1] = VERSION_3;
        v3[FRAME_V3_LEN - 1] = crc8(&v3[1..FRAME_V3_LEN - 1]);
        Ok(FrameV4 {
            command: FrameV3::decode(&v3)?,
            wheel_speed: if speed.is_nan() {
                WheelSpeed::Unknown
            } else {
                WheelSpeed::Known(speed)
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::Frame;
    use crate::steer_cal::CalRequest;
    use crate::v2::Flags;

    fn frame(wheel_speed: WheelSpeed) -> FrameV4 {
        FrameV4 {
            command: FrameV3 {
                values: Frame {
                    steering: 0.1,
                    speed: 0.2,
                    accel: -0.3,
                },
                counter: 12,
                flags: Flags {
                    armed: true,
                    estop: false,
                },
                calibration: CalRequest::Jog(2048),
            },
            wheel_speed,
        }
    }

    #[test]
    fn round_trip() {
        for speed in [
            WheelSpeed::Known(0.75),
            WheelSpeed::Known(-0.25),
            WheelSpeed::Known(0.0),
            WheelSpeed::Unknown,
        ] {
            let f = frame(speed);
            let b = f.encode();
            assert_eq!((b[0], b[1], b[14]), (0xA5, 4, 12));
            assert_eq!(FrameV4::decode(&b), Ok(f));
        }
    }

    #[test]
    fn the_speed_is_little_endian_after_the_version_3_fields() {
        let b = frame(WheelSpeed::Known(1.5)).encode();
        assert_eq!(b[19..23], 1.5f32.to_le_bytes());
    }

    #[test]
    fn every_single_bit_error_is_caught() {
        let bytes = frame(WheelSpeed::Known(0.75)).encode();
        for bit in 8..FRAME_V4_LEN * 8 {
            let mut bad = bytes;
            bad[bit / 8] ^= 0x80 >> (bit % 8);
            assert!(FrameV4::decode(&bad).is_err(), "bit {bit}");
        }
    }

    #[test]
    fn an_infinite_speed_or_a_bad_field_is_turned_down() {
        let fix = |mut b: [u8; FRAME_V4_LEN]| {
            b[23] = crc8(&b[1..23]);
            b
        };
        let mut b = frame(WheelSpeed::Unknown).encode();
        b[19..23].copy_from_slice(&f32::INFINITY.to_le_bytes());
        assert_eq!(FrameV4::decode(&fix(b)), Err(FrameError::Field));
        let mut b = frame(WheelSpeed::Unknown).encode();
        b[16] = 3;
        assert_eq!(FrameV4::decode(&fix(b)), Err(FrameError::Field));
        let mut b = frame(WheelSpeed::Unknown).encode();
        b[1] = 3;
        assert_eq!(FrameV4::decode(&fix(b)), Err(FrameError::Version));
    }

    #[test]
    fn an_earlier_frame_reads_as_no_speed() {
        let f = FrameV4::from(frame(WheelSpeed::Known(1.0)).command);
        assert_eq!(f.wheel_speed, WheelSpeed::NotSent);
    }
}
