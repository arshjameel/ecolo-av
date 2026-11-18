//! The sensor mux's and the output mux's logic, with no ROS: the Rust twin of
//! aav_io_mux's `provenance.py` (the rules), `heartbeat.py` (the Orin's proof of life)
//! and `muxes.py` (what each node does with each message and each tick).
//!
//! `tests/vectors.rs` checks every step against the Python's answers (`vectors.json`,
//! from `gen_vectors.py`), as text. Times are seconds on a steady clock.

pub mod muxes;
pub mod provenance;

/// The output mux's heartbeat counter: +1 for each command passed on, in a byte:
/// 1, 2 ... 255, 0, 1 ... (aav_io_mux `heartbeat.Counter`). The Motor Pi copies it into
/// each serial frame; the ESP32 brakes when it hasn't changed for 100 ms.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counter(u8);

impl Counter {
    /// Count one command passed on; the new counter.
    pub fn beat(&mut self) -> u8 {
        self.0 = self.0.wrapping_add(1);
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::Counter;

    #[test]
    fn the_counter_wraps_in_a_byte() {
        let mut c = Counter::default();
        let beats: Vec<u8> = (0..257).map(|_| c.beat()).collect();
        assert_eq!(&beats[..3], &[1, 2, 3]);
        assert_eq!(&beats[254..], &[255, 0, 1]);
    }
}
