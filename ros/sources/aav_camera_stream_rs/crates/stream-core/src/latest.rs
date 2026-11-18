//! The newest frame from the reader thread to the worker, dropping old ones; and the
//! numbers the node reports (`latest.py`).
//!
//! The decoder must be read as fast as frames come, or they queue up in the pipe and the
//! socket and every frame arrives later than the one before. The worker may be slower, so
//! it takes only the newest frame; the ones it never saw count as dropped.

use std::collections::VecDeque;
use std::sync::{Condvar, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use crate::round_even;

struct Slot<T> {
    item: Option<T>,
    put_count: u64,
    dropped: u64,
}

/// A one-frame slot: [`Latest::put`] replaces, [`Latest::take`] waits for one newer than
/// the last taken.
pub struct Latest<T> {
    slot: Mutex<Slot<T>>,
    cond: Condvar,
}

impl<T> Default for Latest<T> {
    fn default() -> Self {
        Self {
            slot: Mutex::new(Slot {
                item: None,
                put_count: 0,
                dropped: 0,
            }),
            cond: Condvar::new(),
        }
    }
}

impl<T> Latest<T> {
    fn lock(&self) -> MutexGuard<'_, Slot<T>> {
        // A panic elsewhere leaves the numbers usable.
        self.slot.lock().unwrap_or_else(PoisonError::into_inner)
    }

    pub fn put(&self, item: T) {
        let mut slot = self.lock();
        if slot.item.is_some() {
            slot.dropped += 1; // replaced before anyone took it
        }
        slot.item = Some(item);
        slot.put_count += 1;
        self.cond.notify_all();
    }

    /// The newest item not taken yet, or None after `timeout`.
    pub fn take(&self, timeout: Duration) -> Option<T> {
        let slot = self.lock();
        let (mut slot, _) = self
            .cond
            .wait_timeout_while(slot, timeout, |s| s.item.is_none())
            .unwrap_or_else(PoisonError::into_inner);
        slot.item.take()
    }

    /// Items replaced before they were taken.
    pub fn dropped(&self) -> u64 {
        self.lock().dropped
    }

    pub fn put_count(&self) -> u64 {
        self.lock().put_count
    }
}

/// Events per second over the last `window` s, on a clock the caller gives (s).
#[derive(Debug, Clone)]
pub struct Rate {
    window: f64,
    times: VecDeque<f64>,
}

impl Default for Rate {
    fn default() -> Self {
        Self::new(5.0)
    }
}

impl Rate {
    pub fn new(window: f64) -> Self {
        Self {
            window,
            times: VecDeque::new(),
        }
    }

    pub fn tick(&mut self, now: f64) {
        self.times.push_back(now);
        self.trim(now);
    }

    pub fn hz(&mut self, now: f64) -> f64 {
        self.trim(now);
        match (self.times.front(), self.times.back()) {
            (Some(first), Some(last)) if self.times.len() >= 2 => {
                let span = last - first;
                if span > 0.0 {
                    (self.times.len() - 1) as f64 / span
                } else {
                    0.0
                }
            }
            _ => 0.0,
        }
    }

    fn trim(&mut self, now: f64) {
        while self.times.front().is_some_and(|&t| t < now - self.window) {
            self.times.pop_front();
        }
    }
}

/// The median, the 95th percentile and the worst of the delays kept (s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Summary {
    pub n: usize,
    pub median: f64,
    pub p95: f64,
    pub max: f64,
}

/// Delays in s, kept to the last `keep`.
#[derive(Debug, Clone)]
pub struct Delays {
    keep: usize,
    values: VecDeque<f64>,
    count: u64,
}

impl Default for Delays {
    fn default() -> Self {
        Self::new(600)
    }
}

impl Delays {
    pub fn new(keep: usize) -> Self {
        Self {
            keep,
            values: VecDeque::new(),
            count: 0,
        }
    }

    pub fn add(&mut self, seconds: f64) {
        self.values.push_back(seconds);
        self.count += 1;
        while self.values.len() > self.keep {
            self.values.pop_front();
        }
    }

    /// All delays ever added.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// None with no values yet.
    pub fn summary(&self) -> Option<Summary> {
        let mut ordered: Vec<f64> = self.values.iter().copied().collect();
        if ordered.is_empty() {
            return None;
        }
        ordered.sort_by(f64::total_cmp);
        let n = ordered.len();
        let median = if n % 2 == 1 {
            ordered[n / 2]
        } else {
            (ordered[n / 2 - 1] + ordered[n / 2]) / 2.0
        };
        let i = (round_even(0.95 * (n - 1) as f64) as usize).min(n - 1);
        Some(Summary {
            n,
            median,
            p95: ordered[i],
            max: ordered[n - 1],
        })
    }
}

/// A frame's stamp on the node's clock: its arrival (wall clock) minus the delay.
///
/// The node's clock is the wall clock on the car, and simulated time in a simulator: the
/// arrival moves over by the difference between the two clocks now. Never before 0.
pub fn stamp_ns(arrival_wall_ns: i64, now_clock_ns: i64, now_wall_ns: i64, delay_s: f64) -> i64 {
    let offset = now_clock_ns - now_wall_ns;
    (arrival_wall_ns + offset - round_even(delay_s * 1e9) as i64).max(0)
}

/// How long the reader waits before starting the decoder again: a stream that ran a
/// while starts again quickly (`retry`); one that never came, slower, doubling up to
/// `retry_max` (`reader.py`).
pub fn next_pause(pause: f64, got_frames: bool, retry: f64, retry_max: f64) -> f64 {
    if got_frames {
        retry
    } else {
        retry_max.min(pause * 2.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_newest_is_taken_and_the_rest_count_as_dropped() {
        let latest = Latest::default();
        latest.put(1);
        latest.put(2);
        latest.put(3);
        assert_eq!(latest.take(Duration::ZERO), Some(3));
        assert_eq!(latest.take(Duration::from_millis(10)), None);
        assert_eq!((latest.dropped(), latest.put_count()), (2, 3));
    }

    #[test]
    fn take_waits_for_a_put_from_another_thread() {
        let latest = std::sync::Arc::new(Latest::default());
        let other = std::sync::Arc::clone(&latest);
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(20));
            other.put(7);
        });
        assert_eq!(latest.take(Duration::from_secs(5)), Some(7));
        t.join().ok();
    }

    #[test]
    fn the_pause_grows_only_while_nothing_comes() {
        assert_eq!(next_pause(1.0, false, 1.0, 5.0), 2.0);
        assert_eq!(next_pause(4.0, false, 1.0, 5.0), 5.0);
        assert_eq!(next_pause(5.0, true, 1.0, 5.0), 1.0);
    }
}
