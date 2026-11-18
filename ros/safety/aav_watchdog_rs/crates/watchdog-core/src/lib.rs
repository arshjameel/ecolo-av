//! The sensor watchdog's logic, with no ROS: the Rust twin of `aav_safety`'s `watchdog.py`
//! and `wheel_check.py`. **A safety lock**: it must give the Python's answers exactly.
//!
//! The node says what arrived and when; this crate says, for each input, OK or what is
//! wrong, and whether the automatic drivers must stop (twist_mux's sensors lock).
//!
//! | State | Means | Latched? |
//! |---|---|---|
//! | NEVER | nothing yet | |
//! | STALE | nothing good for `max_age` s (a GPS with no fix counts as nothing) | |
//! | NAN | NaN or inf in the newest message | |
//! | FROZEN | the same values for `frozen_after` s (a camera only while the car moves; a heartbeat whose count stopped) | |
//! | JUMP | two GPS fixes further apart than `max_jump_speed` allows, plus `jump_margin` | yes |
//! | DISAGREE | the wheels against the IMU and the GPS ([`wheels`]) | yes |
//! | STUCK | an actuator more than `follow_tolerance` off its command for `follow_persist` s | yes |
//! | OVER | over ego's speed by `over_margin` for `over_warn` s, not slowing: a warning | |
//! | OVERSPEED | the same for `over_persist` s | yes |
//! | LOST | a `latch` input went wrong after its start-up | yes |
//!
//! | Module | The Python it ports |
//! |---|---|
//! | [`feed`] | `sensor_watchdog.py`: what the node does with each message and tick, and its `/diagnostics` |
//! | [`watch`] | `watchdog.py`: `Rule`, `Watch`, `Watchdog`, `ground_distance` |
//! | [`wheels`] | `wheel_check.py`: `CrossCheck`, `WheelCheck`, `trapezoid`, `fitted_speed` |
//!
//! The same answers as the Python: `tests/vectors.rs` reads `vectors.json`, written by
//! `gen_vectors.py` from the Python modules. Times are seconds on the node's clock.

// The Rust API Guidelines, kept: every public item documented, its errors too, and a pure
// function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod feed;
pub mod watch;
pub mod wheels;

pub use ego_core::{fmt_g, hypot};

/// Python 3.12's `sum()` of floats: Neumaier's compensated sum, as CPython adds them.
#[must_use]
pub fn py_sum(values: impl IntoIterator<Item = f64>) -> f64 {
    let (mut total, mut c) = (0.0_f64, 0.0_f64);
    for x in values {
        let t = total + x;
        if total.abs() >= x.abs() {
            c += (total - t) + x;
        } else {
            c += (x - t) + total;
        }
        total = t;
    }
    if c != 0.0 && c.is_finite() {
        total += c;
    }
    total
}

/// Python's `max(a, b)`: `a`, unless `b` is greater.
#[must_use]
pub fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python's `min(a, b)`: `a`, unless `b` is smaller.
#[must_use]
pub fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// zlib's CRC-32 (IEEE 802.3, as Python's `zlib.crc32`) of these bytes.
#[must_use]
pub fn crc32(bytes: impl IntoIterator<Item = u8>) -> u32 {
    let mut crc = !0_u32;
    for b in bytes {
        crc ^= u32::from(b);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// A camera picture's digest, as the Python node takes it: the CRC-32 of every 997th byte
/// (`zlib.crc32(data[::997])`). A frozen camera repeats its whole picture: a sample is enough.
#[must_use]
pub fn picture_digest(data: &[u8]) -> u32 {
    crc32(data.iter().step_by(997).copied())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_is_zlibs() {
        assert_eq!(crc32(*b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32([]), 0);
        let data: Vec<u8> = (0..5000_u32).map(|i| (i % 251) as u8).collect();
        // data[0], data[997], ... data[4985]: 6 bytes.
        let sample: Vec<u8> = data.iter().step_by(997).copied().collect();
        assert_eq!(sample.len(), 6);
        assert_eq!(picture_digest(&data), crc32(sample));
    }

    #[test]
    fn the_sum_is_compensated_as_pythons() {
        assert_eq!(py_sum([1e100, 1.0, -1e100, 1.0]), 2.0);
        assert_eq!(py_sum([0.1; 10]), 1.0);
    }
}
