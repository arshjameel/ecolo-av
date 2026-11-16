//! One frame's values: steering, speed and accel, each -1..1; and the version 1 frame.

/// The link's speed: 115200 baud, 8N1 (`main.cpp`: `Serial1.begin(115200, ...)`).
pub const BAUD: u32 = 115_200;

/// How many frames a second the Motor-Pi sends: `legacy_uart_bridge`'s `rate_hz`
/// (`ros/can_bridge/aav_legacy/config/legacy.yaml`; `tools/tests/test_esp32_settings.py`
/// checks the two match). The board only reports it: what it acts on is the 100 ms
/// command timeout, which lets four frames in a row go missing.
pub const FRAME_HZ: u32 = 50;

/// Bytes in a version 1 frame. There is no header: the checksum is the only way to find
/// a frame.
pub const FRAME_LEN: usize = 13;

/// One frame's values, as the Motor-Pi sent them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    /// -1..1 of full lock. +1 is the Jrk's 3500 (which way that turns: Q2).
    pub steering: f32,
    /// -1..1 of the throttle cap; the sign is the direction.
    pub speed: f32,
    /// -1..1; below -0.05 it is a brake demand, and cuts the throttle.
    pub accel: f32,
}

/// Why a frame was turned down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Version 1: the last byte isn't the sum of the other twelve. Version 2: the CRC-8
    /// doesn't match.
    Checksum,
    /// A value is outside -1..1, or not a number. The C++ then ramped the throttle to 0.
    OutOfRange,
    /// Version 2: a byte outside a frame that isn't the sync byte (0xA5): noise, or a
    /// frame's tail after a bad CRC.
    Sync,
    /// Version 2: the CRC matches, but the version byte isn't 2.
    Version,
    /// Version 2: a flag bit this firmware doesn't know is set.
    Flags,
    /// The uplink: a mode or a switch position outside the known ones.
    Field,
}

impl Frame {
    /// The 13 bytes, as `legacy_protocol.pack_frame` sends them (without its clamp).
    #[must_use]
    pub fn encode(&self) -> [u8; FRAME_LEN] {
        let mut bytes = [0; FRAME_LEN];
        let values = [self.steering, self.speed, self.accel];
        let (chunks, _checksum) = bytes.as_chunks_mut::<4>();
        for (chunk, value) in chunks.iter_mut().zip(values) {
            *chunk = value.to_le_bytes();
        }
        bytes[12] = checksum(&bytes[..12]);
        bytes
    }

    /// A frame from 13 bytes, if its checksum matches. The values are not checked.
    pub fn decode(bytes: &[u8; FRAME_LEN]) -> Result<Frame, FrameError> {
        if checksum(&bytes[..12]) != bytes[12] {
            return Err(FrameError::Checksum);
        }
        let value = |at: usize| {
            let mut four = [0; 4];
            four.copy_from_slice(&bytes[at..at + 4]);
            f32::from_le_bytes(four)
        };
        Ok(Frame {
            steering: value(0),
            speed: value(4),
            accel: value(8),
        })
    }

    /// The frame, if every value is within -1..1 (not a number fails too).
    pub fn check(self) -> Result<Frame, FrameError> {
        let ok = |v: f32| v.abs() <= 1.0; // false for NaN, as fabsf(NaN) <= 1.0f is
        if ok(self.steering) && ok(self.speed) && ok(self.accel) {
            Ok(self)
        } else {
            Err(FrameError::OutOfRange)
        }
    }
}

/// The sum of the bytes, modulo 256.
fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |sum, &b| sum.wrapping_add(b))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(steering: f32, speed: f32, accel: f32) -> Frame {
        Frame {
            steering,
            speed,
            accel,
        }
    }

    #[test]
    fn round_trip() {
        let frame = f(0.5, -0.25, 0.0);
        let bytes = frame.encode();
        let sum: u32 = bytes[..12].iter().map(|&b| u32::from(b)).sum();
        assert_eq!(bytes[12], sum as u8);
        assert_eq!(Frame::decode(&bytes), Ok(frame));
    }

    #[test]
    fn a_bad_checksum_is_turned_down() {
        let mut bytes = f(0.1, 0.2, 0.0).encode();
        bytes[0] ^= 1;
        assert_eq!(Frame::decode(&bytes), Err(FrameError::Checksum));
    }

    #[test]
    fn values_outside_one_are_turned_down() {
        assert!(f(1.0, -1.0, 1.0).check().is_ok());
        assert_eq!(f(1.01, 0.0, 0.0).check(), Err(FrameError::OutOfRange));
        assert_eq!(f(0.0, -2.0, 0.0).check(), Err(FrameError::OutOfRange));
        assert_eq!(f(0.0, 0.0, f32::NAN).check(), Err(FrameError::OutOfRange));
        assert_eq!(
            f(0.0, f32::INFINITY, 0.0).check(),
            Err(FrameError::OutOfRange)
        );
    }
}
