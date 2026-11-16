//! The wheel encoder's UDP packet to the Motor Pi.
//!
//! Byte for byte the 2025-26 one (`Wheel_Encoders`, `wiznet.ino`, `loop()`): 4 bytes, a
//! big-endian signed 32-bit count of spokes since the previous packet, about every 100 ms.
//! Positive is forward. `aav_legacy`'s `encoder_counts()` reads it on the Motor Pi, and
//! both sides check the same examples: `firmware/udp/wheel_encoder_vectors.txt`.

use crate::spokes::WheelCount;

/// Spokes counted since the previous packet: positive forward, negative in reverse.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SpokeDelta(pub i32);

/// Why a packet was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketError {
    /// A packet is exactly 4 bytes; this one had this many.
    Length(usize),
}

/// Which way the wheel turned in one packet's time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
    Stopped,
}

impl SpokeDelta {
    /// The packet's length in bytes.
    pub const LEN: usize = 4;

    /// The packet: big-endian, as the 2025-26 encoder sent it.
    #[must_use]
    pub fn to_bytes(self) -> [u8; Self::LEN] {
        self.0.to_be_bytes()
    }

    /// Read a packet, as the Motor Pi does.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, PacketError> {
        let four: [u8; Self::LEN] = bytes
            .try_into()
            .map_err(|_| PacketError::Length(bytes.len()))?;
        Ok(SpokeDelta(i32::from_be_bytes(four)))
    }

    #[must_use]
    pub fn direction(self) -> Direction {
        match self.0 {
            0 => Direction::Stopped,
            n if n > 0 => Direction::Forward,
            _ => Direction::Reverse,
        }
    }

    /// The wheel's turns a minute, for the logs only: `aav_legacy` works out the speed.
    #[must_use]
    pub fn rpm(self, spokes_per_rev: u16, interval_ms: u32) -> f32 {
        if spokes_per_rev == 0 || interval_ms == 0 {
            return 0.0;
        }
        let revs = self.0.unsigned_abs() as f32 / f32::from(spokes_per_rev);
        revs * 60_000.0 / interval_ms as f32
    }
}

/// Turns the running counter into one delta a packet.
///
/// The 2025-26 firmware set its counter back to 0 at each packet, with the interrupts off.
/// Here the counter only runs on, and the reporter remembers what it last sent, so no
/// spoke is lost between the read and the reset, and the counter's wrap does no harm.
#[derive(Debug, Clone, Copy, Default)]
pub struct Reporter {
    sent: i32,
}

impl Reporter {
    /// Starts from the counter as it is now.
    #[must_use]
    pub const fn new(now: WheelCount) -> Self {
        Reporter { sent: now.count }
    }

    /// The spokes since the previous report.
    pub fn report(&mut self, now: WheelCount) -> SpokeDelta {
        let delta = now.count.wrapping_sub(self.sent);
        self.sent = now.count;
        SpokeDelta(delta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn count(count: i32) -> WheelCount {
        WheelCount { count, at_ms: 0 }
    }

    #[test]
    fn packets_are_big_endian() {
        assert_eq!(SpokeDelta(1).to_bytes(), [0, 0, 0, 1]);
        assert_eq!(SpokeDelta(-3).to_bytes(), [0xFF, 0xFF, 0xFF, 0xFD]);
        assert_eq!(SpokeDelta(0x0102_0304).to_bytes(), [1, 2, 3, 4]);
    }

    #[test]
    fn packets_round_trip() {
        for n in [0, 1, -1, 7, -42, i32::MAX, i32::MIN] {
            let delta = SpokeDelta(n);
            assert_eq!(SpokeDelta::from_bytes(&delta.to_bytes()), Ok(delta));
        }
    }

    #[test]
    fn only_four_bytes_make_a_packet() {
        assert_eq!(SpokeDelta::from_bytes(&[0, 1]), Err(PacketError::Length(2)));
        assert_eq!(SpokeDelta::from_bytes(&[0; 5]), Err(PacketError::Length(5)));
    }

    #[test]
    fn the_reporter_sends_each_spoke_once() {
        let mut r = Reporter::new(count(10));
        assert_eq!(r.report(count(13)), SpokeDelta(3));
        assert_eq!(r.report(count(13)), SpokeDelta(0));
        assert_eq!(r.report(count(11)), SpokeDelta(-2));
    }

    #[test]
    fn the_reporter_survives_the_counter_wrapping() {
        let mut r = Reporter::new(count(i32::MAX - 1));
        assert_eq!(r.report(count(i32::MIN)), SpokeDelta(2));
    }

    #[test]
    fn direction_and_rpm_for_the_logs() {
        assert_eq!(SpokeDelta(0).direction(), Direction::Stopped);
        assert_eq!(SpokeDelta(2).direction(), Direction::Forward);
        assert_eq!(SpokeDelta(-2).direction(), Direction::Reverse);
        // 2 of 8 spokes in 100 ms: a quarter turn, 150 turns a minute.
        assert_eq!(SpokeDelta(-2).rpm(8, 100), 150.0);
        assert_eq!(SpokeDelta(5).rpm(0, 100), 0.0);
    }
}
