//! The goal leash (TODO 116): a goal's owner that goes quiet loses its goals. The Rust
//! twin of `aav_safety`'s `leash.py`.
//!
//! ```text
//! beats (1, 2, 3...) ─► armed ─► silent over `timeout` s ─► trip: cancel the goals
//! beat 0 (goodbye: the owner closed cleanly, cancelling its own) ─► disarmed
//! ```
//!
//! A trip is a cancel, so the car slows to a stop the driver's own way: not an e-stop.
//! It re-arms on the next beat.

/// The beat that says goodbye.
pub const GOODBYE: u64 = 0;

/// Watches one owner's beats. `timeout` s of silence while armed trips it once.
#[derive(Debug, Clone, PartialEq)]
pub struct Leash {
    timeout: f64,
    last: Option<f64>,
}

impl Leash {
    /// A leash that trips after `timeout` s of silence (as `leash.py`, any number).
    #[must_use]
    pub fn new(timeout: f64) -> Self {
        Self {
            timeout,
            last: None,
        }
    }

    /// The seconds of silence it allows.
    #[must_use]
    pub fn timeout(&self) -> f64 {
        self.timeout
    }

    /// Whether an owner is beating.
    #[must_use]
    pub fn armed(&self) -> bool {
        self.last.is_some()
    }

    /// Take a beat: a count arms it, [`GOODBYE`] disarms it (nothing cancelled).
    pub fn beat(&mut self, value: u64, now: f64) {
        self.last = if value == GOODBYE { None } else { Some(now) };
    }

    /// True once, the moment an armed owner has been silent over `timeout` s.
    pub fn check(&mut self, now: f64) -> bool {
        match self.last {
            Some(last)
                if now - last > self.timeout || (now - last).is_nan() || self.timeout.is_nan() =>
            {
                self.last = None;
                true
            }
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trips_once_then_rearms() {
        let mut l = Leash::new(1.0);
        assert!(!l.check(10.0));
        l.beat(1, 0.0);
        assert!(!l.check(1.0));
        assert!(l.check(1.01));
        assert!(!l.check(2.0));
        l.beat(2, 3.0);
        l.beat(GOODBYE, 3.1);
        assert!(!l.check(9.0));
    }
}
