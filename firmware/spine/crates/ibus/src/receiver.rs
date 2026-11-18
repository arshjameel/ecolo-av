//! The receiver as the spine sees it: the latest good frame, or nothing once the
//! receiver has been silent too long.
//!
//! The 2025-26 firmware had no timeout: if frames stopped, it kept the last channel
//! values, and with them the last throttle. Here a silent receiver reads as `None`.
//! That covers a cut wire or a dead receiver. If the radio link drops, the receiver
//! itself decides what it sends: its failsafe values, set on the transmitter (Q7).

use crate::frame::Channels;
use crate::parser::Parser;

/// The receiver as a whole: bytes in, the latest good channels out, and whether
/// they are too old to trust.
#[derive(Debug, Clone)]
pub struct Receiver {
    parser: Parser,
    timeout_ms: u32,
    latest: Option<(Channels, u32)>,
    frames: u32,
}

impl Receiver {
    /// `timeout_ms`: no good frame for longer than this, and the receiver is silent.
    /// Frames come about every 7 ms.
    #[must_use]
    pub const fn new(timeout_ms: u32) -> Self {
        Receiver {
            parser: Parser::new(),
            timeout_ms,
            latest: None,
            frames: 0,
        }
    }

    /// Feed one byte, read at `now_ms` (a free-running clock that may wrap).
    /// Returns true if it completed a good frame.
    pub fn push(&mut self, byte: u8, now_ms: u32) -> bool {
        match self.parser.push(byte) {
            Some(channels) => {
                self.latest = Some((channels, now_ms));
                self.frames = self.frames.wrapping_add(1);
                true
            }
            None => false,
        }
    }

    /// The latest channels, or None before the first frame and once the receiver
    /// has been silent for longer than the timeout.
    #[must_use]
    pub fn channels(&self, now_ms: u32) -> Option<Channels> {
        let (channels, at) = self.latest?;
        (now_ms.wrapping_sub(at) <= self.timeout_ms).then_some(channels)
    }

    /// Good frames so far (wraps).
    #[must_use]
    pub fn frames(&self) -> u32 {
        self.frames
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{encode, CHANNELS};

    fn feed(r: &mut Receiver, first: u16, now_ms: u32) -> bool {
        let mut ch = [1500; CHANNELS];
        ch[0] = first;
        let mut done = false;
        for byte in encode(&Channels(ch)) {
            done |= r.push(byte, now_ms);
        }
        done
    }

    #[test]
    fn nothing_before_the_first_frame() {
        assert_eq!(Receiver::new(100).channels(0), None);
    }

    #[test]
    fn the_latest_frame_until_the_timeout() {
        let mut r = Receiver::new(100);
        assert!(feed(&mut r, 1000, 1000));
        assert!(feed(&mut r, 1200, 1007));
        assert_eq!(r.channels(1107).map(|c| c.0[0]), Some(1200)); // 100 ms: still fresh
        assert_eq!(r.channels(1108), None); // 101 ms: silent
        assert_eq!(r.frames(), 2);
    }

    #[test]
    fn a_bad_frame_does_not_refresh_it() {
        let mut r = Receiver::new(100);
        feed(&mut r, 1000, 0);
        let mut bad = encode(&Channels([1500; CHANNELS]));
        bad[5] ^= 1;
        for &b in &bad {
            r.push(b, 90);
        }
        assert_eq!(r.channels(150), None);
    }

    #[test]
    fn survives_clock_wrap() {
        let mut r = Receiver::new(100);
        feed(&mut r, 1000, u32::MAX - 10);
        assert!(r.channels(50).is_some()); // 61 ms later
        assert!(r.channels(100).is_none()); // 111 ms later
    }
}
