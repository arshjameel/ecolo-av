//! The stop hold: once the stop zone stops the car, keep it stopped until the way stays
//! clear. The Rust twin of `aav_safety`'s `hold.py`.
//!
//! The stop zone decides afresh on every command, so an obstacle at its box's edge
//! flickers it between stop and go, and each go lets the car creep on. [`StopHold`] is
//! locked from the moment the zone says STOP, and lets go only once it has said something
//! else for `clear_time` s without a break.

use std::fmt;

/// A `clear_time` that is not a finite number of seconds >= 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BadClearTime(pub f64);

impl fmt::Display for BadClearTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("clear_time must be a number of seconds >= 0")
    }
}

impl std::error::Error for BadClearTime {}

/// Locked from a stop until `clear_time` s of no stop.
#[derive(Debug, Clone, PartialEq)]
pub struct StopHold {
    clear_time: f64,
    stopped: bool,
    /// When the last stop ended; `None` while stopped.
    clear_since: Option<f64>,
}

impl StopHold {
    /// A hold that lets go after `clear_time` s clear.
    ///
    /// # Errors
    /// [`BadClearTime`] unless `clear_time` is finite and >= 0.
    pub fn new(clear_time: f64) -> Result<Self, BadClearTime> {
        // As hold.py: NaN and infinity refused.
        if clear_time.is_nan() || clear_time < 0.0 || clear_time.is_infinite() {
            return Err(BadClearTime(clear_time));
        }
        Ok(Self {
            clear_time,
            stopped: false,
            clear_since: None,
        })
    }

    /// The seconds clear it waits for.
    #[must_use]
    pub fn clear_time(&self) -> f64 {
        self.clear_time
    }

    /// Take the stop zone's state at `now`: `stop` while it stops the car.
    pub fn update(&mut self, now: f64, stop: bool) {
        if stop {
            self.stopped = true;
            self.clear_since = None;
        } else if self.stopped && self.clear_since.is_none() {
            self.clear_since = Some(now);
        }
    }

    /// Whether the hold is on at `now`.
    pub fn locked(&mut self, now: f64) -> bool {
        if !self.stopped {
            return false;
        }
        if let Some(since) = self.clear_since {
            if now - since >= self.clear_time {
                self.stopped = false;
                self.clear_since = None;
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn holds_until_clear_long_enough() {
        let mut h = StopHold::new(1.0).expect("ok");
        assert!(!h.locked(0.0));
        h.update(0.0, true);
        assert!(h.locked(0.5));
        h.update(0.6, false);
        assert!(h.locked(1.5));
        h.update(1.55, true);
        h.update(1.7, false);
        assert!(h.locked(2.6));
        assert!(!h.locked(2.7));
    }

    #[test]
    fn refuses_bad_times() {
        assert!(StopHold::new(f64::NAN).is_err());
        assert!(StopHold::new(f64::INFINITY).is_err());
        assert!(StopHold::new(-0.1).is_err());
        assert!(StopHold::new(0.0).is_ok());
    }
}
