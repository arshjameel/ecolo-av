//! One iBUS servo frame: 32 bytes, every 7 ms or so.
//!
//! | Bytes | Holds |
//! |---|---|
//! | 0, 1 | `0x20 0x40`: the length (32), then the command (servo data) |
//! | 2 to 29 | 14 channels, each a little-endian `u16` |
//! | 30, 31 | the checksum, little-endian: 0xFFFF minus the sum of bytes 0 to 29 |

/// The receiver's serial speed: 115200 baud, 8 data bits, no parity, 1 stop bit.
pub const BAUD: u32 = 115_200;
/// Bytes in one servo frame.
pub const FRAME_LEN: usize = 32;
/// Channels in one servo frame.
pub const CHANNELS: usize = 14;
/// The first two bytes of every servo frame.
pub const HEADER: [u8; 2] = [0x20, 0x40];

/// The 14 channels of one frame, in microseconds: sticks and switches give 1000 to
/// 2000. Index 0 is the transmitter's CH1, as in the 2025-26 firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channels(pub [u16; CHANNELS]);

impl Channels {
    /// The channel at a 0-based index, or None past the 14th.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<u16> {
        self.0.get(index).copied()
    }
}

/// 0xFFFF minus the sum of the bytes (the first 30 of a frame).
#[must_use]
pub fn checksum(bytes: &[u8]) -> u16 {
    bytes
        .iter()
        .fold(0xFFFF_u16, |sum, &b| sum.wrapping_sub(u16::from(b)))
}

/// The channels of a whole frame, or None if its header or checksum is wrong.
#[must_use]
pub fn decode(frame: &[u8; FRAME_LEN]) -> Option<Channels> {
    let sent = u16::from_le_bytes([frame[30], frame[31]]);
    if frame[..2] != HEADER || checksum(&frame[..30]) != sent {
        return None;
    }
    let mut ch = [0; CHANNELS];
    for (i, c) in ch.iter_mut().enumerate() {
        *c = u16::from_le_bytes([frame[2 + 2 * i], frame[3 + 2 * i]]);
    }
    Some(Channels(ch))
}

/// The frame a receiver would send for these channels: for tests and bench tools.
#[must_use]
pub fn encode(channels: &Channels) -> [u8; FRAME_LEN] {
    let mut frame = [0; FRAME_LEN];
    frame[..2].copy_from_slice(&HEADER);
    for (i, c) in channels.0.iter().enumerate() {
        frame[2 + 2 * i..4 + 2 * i].copy_from_slice(&c.to_le_bytes());
    }
    let sum = checksum(&frame[..30]);
    frame[30..].copy_from_slice(&sum.to_le_bytes());
    frame
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksum_of_a_centred_frame() {
        // 14 channels at 1500 (0x05DC): 0x20 + 0x40 + 14 * (0xDC + 0x05) = 3246.
        let frame = encode(&Channels([1500; CHANNELS]));
        assert_eq!(&frame[..4], &[0x20, 0x40, 0xDC, 0x05]);
        assert_eq!(&frame[30..], &(0xFFFF_u16 - 3246).to_le_bytes());
        assert_eq!(&frame[30..], &[0x51, 0xF3]);
    }

    #[test]
    fn checksum_the_way_the_cpp_library_does_it() {
        // IBusBM::loop(): start at 0xFFFF minus the length byte, then subtract each byte.
        let frame = encode(&Channels([
            1000, 2000, 1234, 1500, 1999, 1001, 1500, 0, 0, 0, 0, 0, 0, 65535,
        ]));
        let mut sum: u16 = 0xFFFF - 0x20;
        for &b in &frame[1..30] {
            sum = sum.wrapping_sub(u16::from(b));
        }
        assert_eq!(sum, u16::from_le_bytes([frame[30], frame[31]]));
    }

    #[test]
    fn decode_reverses_encode() {
        let ch = Channels([
            1000, 2000, 1500, 1234, 1999, 1001, 1500, 1500, 1500, 1500, 0, 1, 2, 3,
        ]);
        assert_eq!(decode(&encode(&ch)), Some(ch));
        assert_eq!(ch.get(0), Some(1000));
        assert_eq!(ch.get(13), Some(3));
        assert_eq!(ch.get(14), None);
    }

    #[test]
    fn decode_rejects_a_bad_header_or_checksum() {
        let good = encode(&Channels([1500; CHANNELS]));
        let mut bad = good;
        bad[0] = 0x21;
        assert_eq!(decode(&bad), None);
        let mut bad = good;
        bad[10] ^= 0x01;
        assert_eq!(decode(&bad), None);
        let mut bad = good;
        bad[31] ^= 0x80;
        assert_eq!(decode(&bad), None);
    }
}
