//! Find frames in the byte stream, one byte at a time.
//!
//! The stream has no gaps the parser can see, and may start mid-frame or carry
//! noise. So the parser waits for the header `0x20 0x40`, collects 32 bytes and
//! checks the checksum. After a bad frame it carries on from the next `0x20 0x40`
//! inside the bytes it already has, so a good frame that starts there isn't lost.

use crate::frame::{decode, Channels, FRAME_LEN, HEADER};

/// Finds whole, checked frames in a stream of serial bytes, one byte at a time.
#[derive(Debug, Clone)]
pub struct Parser {
    buf: [u8; FRAME_LEN],
    len: usize,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    /// An empty parser, waiting for a header.
    #[must_use]
    pub const fn new() -> Self {
        Parser {
            buf: [0; FRAME_LEN],
            len: 0,
        }
    }

    /// Feed one byte. Returns the channels when it completes a good frame.
    pub fn push(&mut self, byte: u8) -> Option<Channels> {
        match self.len {
            // Waiting for 0x20.
            0 => {
                if byte == HEADER[0] {
                    self.buf[0] = byte;
                    self.len = 1;
                }
            }
            // Have 0x20, want 0x40. Another 0x20 may be the real start: keep it.
            1 => {
                if byte == HEADER[1] {
                    self.buf[1] = byte;
                    self.len = 2;
                } else if byte != HEADER[0] {
                    self.len = 0;
                }
            }
            _ => {
                self.buf[self.len] = byte;
                self.len += 1;
                if self.len == FRAME_LEN {
                    if let Some(channels) = decode(&self.buf) {
                        self.len = 0;
                        return Some(channels);
                    }
                    self.resync();
                }
            }
        }
        None
    }

    /// After a bad frame, keep the bytes from the next possible start: `0x20 0x40`,
    /// or a `0x20` in the last place.
    fn resync(&mut self) {
        let start = (1..FRAME_LEN).find(|&i| {
            self.buf[i] == HEADER[0] && (i + 1 == FRAME_LEN || self.buf[i + 1] == HEADER[1])
        });
        match start {
            Some(i) => {
                self.buf.copy_within(i.., 0);
                self.len = FRAME_LEN - i;
            }
            None => self.len = 0,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::frame::{encode, CHANNELS};
    use std::vec::Vec;

    fn frame(first: u16) -> [u8; FRAME_LEN] {
        let mut ch = [1500; CHANNELS];
        ch[0] = first;
        encode(&Channels(ch))
    }

    /// Feed bytes; collect channel 0 of each frame found.
    fn run(p: &mut Parser, bytes: &[u8]) -> Vec<u16> {
        bytes
            .iter()
            .filter_map(|&b| p.push(b))
            .map(|c| c.0[0])
            .collect()
    }

    #[test]
    fn back_to_back_frames() {
        let mut bytes = Vec::new();
        for v in [1000, 1100, 1200] {
            bytes.extend_from_slice(&frame(v));
        }
        assert_eq!(run(&mut Parser::new(), &bytes), [1000, 1100, 1200]);
    }

    #[test]
    fn noise_before_a_frame() {
        let mut bytes = Vec::from([0x00, 0xFF, 0x20, 0x13, 0x40, 0x20, 0x20]);
        bytes.extend_from_slice(&frame(1234));
        assert_eq!(run(&mut Parser::new(), &bytes), [1234]);
    }

    #[test]
    fn starting_mid_frame() {
        let mut bytes = Vec::from(&frame(1000)[13..]);
        bytes.extend_from_slice(&frame(1111));
        assert_eq!(run(&mut Parser::new(), &bytes), [1111]);
    }

    #[test]
    fn a_corrupt_frame_is_dropped_and_the_next_one_kept() {
        let mut bad = frame(1000);
        bad[7] ^= 0x10;
        let mut bytes = Vec::from(bad);
        bytes.extend_from_slice(&frame(1222));
        assert_eq!(run(&mut Parser::new(), &bytes), [1222]);
    }

    #[test]
    fn a_frame_that_starts_inside_a_cut_off_one() {
        // A frame cut off after 20 bytes, then a whole frame: the parser has taken 12
        // bytes of the good frame into the bad one, and must find it there.
        let mut bytes = Vec::from(&frame(1000)[..20]);
        bytes.extend_from_slice(&frame(1333));
        bytes.extend_from_slice(&frame(1444));
        assert_eq!(run(&mut Parser::new(), &bytes), [1333, 1444]);
    }

    #[test]
    fn a_header_lookalike_inside_the_data() {
        // Channel values whose bytes read 0x20 0x40 (0x4020 = 16416).
        let mut ch = [0x4020; CHANNELS];
        ch[0] = 1555;
        let mut bytes = Vec::from([0x20, 0x40, 0x00]);
        bytes.extend_from_slice(&encode(&Channels(ch)));
        assert_eq!(run(&mut Parser::new(), &bytes), [1555]);
    }

    #[test]
    fn many_garbage_bytes_never_make_a_frame() {
        let mut p = Parser::new();
        let mut x: u32 = 12345;
        for _ in 0..100_000 {
            x = x.wrapping_mul(1_103_515_245).wrapping_add(12345); // a simple pseudo-random stream
            assert_eq!(p.push((x >> 16) as u8), None);
        }
        assert_eq!(run(&mut p, &frame(1666)), [1666]);
    }
}
