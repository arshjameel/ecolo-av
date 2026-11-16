//! Two spoke sensors, A and B, read about every millisecond.

use aav_inputs::{Edge, Lockout};

/// The wheel encoder's counter, and when it last changed.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct WheelCount {
    /// Counts since power-up: up going forward, down in reverse. It may wrap.
    pub count: i32,
    /// The millisecond clock at the latest count.
    pub at_ms: u32,
}

/// Two spoke sensors, A and B, for a board that reads them itself, about every millisecond.
///
/// The rule of the 2025-26 encoder Pico 2 (`Wheel_Encoders`, `wiznet.ino`,
/// `sensor_callback()`): one count on A's rising edge, and backwards if B rose in the 20 ms
/// before and is still high. Each sensor is cleaned by an `aav_inputs::Lockout` of 10 ms, the
/// 2025-26 debounce, so a flicker at the edge of a spoke counts once.
///
/// TODO(Q5): which wheel, how far apart the sensors are, and which way is forward. Turning
/// slowly backwards, with B more than 20 ms ahead of A, counts as forward, as it did before.
#[derive(Debug, Clone, Copy)]
pub struct SpokeSensors {
    a: Lockout,
    b: Lockout,
    b_rose_ms: Option<u32>,
    direction_window_ms: u32,
    count: WheelCount,
}

impl SpokeSensors {
    /// The 2025-26 numbers: 10 ms of lockout on each sensor, a 20 ms window for B.
    pub const TODAY: SpokeSensors = SpokeSensors::new(10, 20);

    #[must_use]
    pub const fn new(lockout_ms: u32, direction_window_ms: u32) -> Self {
        SpokeSensors {
            a: Lockout::new(false, lockout_ms),
            b: Lockout::new(false, lockout_ms),
            b_rose_ms: None,
            direction_window_ms,
            count: WheelCount { count: 0, at_ms: 0 },
        }
    }

    /// Feed both sensors' raw levels, read at `now_ms`. Returns the counter.
    pub fn update(&mut self, a: bool, b: bool, now_ms: u32) -> WheelCount {
        if self.b.update(b, now_ms) == Some(Edge::Rising) {
            self.b_rose_ms = Some(now_ms);
        }
        if self.a.update(a, now_ms) == Some(Edge::Rising) {
            let b_ahead = self.b.level()
                && self
                    .b_rose_ms
                    .is_some_and(|t| now_ms.wrapping_sub(t) < self.direction_window_ms);
            self.count = WheelCount {
                count: self.count.count.wrapping_add(if b_ahead { -1 } else { 1 }),
                at_ms: now_ms,
            };
        }
        self.count
    }

    /// The counter, and the clock at its latest count.
    #[must_use]
    pub fn count(&self) -> WheelCount {
        self.count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(count: i32, at_ms: u32) -> WheelCount {
        WheelCount { count, at_ms }
    }

    /// One spoke every 40 ms for 400 ms, sampled every millisecond. `a` and `b` say when, in
    /// each 40 ms, each sensor sees the spoke.
    fn spokes(a: core::ops::Range<u32>, b: core::ops::Range<u32>) -> Option<WheelCount> {
        let mut sensors = SpokeSensors::TODAY;
        (0..400)
            .map(|ms| sensors.update(a.contains(&(ms % 40)), b.contains(&(ms % 40)), ms))
            .last()
    }

    #[test]
    fn spokes_forward_count_up() {
        // A sees each spoke first: ten spokes, the last at 360 ms.
        assert_eq!(spokes(0..15, 10..25), Some(at(10, 360)));
    }

    #[test]
    fn spokes_backward_count_down() {
        // B sees each spoke 10 ms before A.
        assert_eq!(spokes(10..25, 0..15), Some(at(-10, 370)));
    }

    #[test]
    fn a_flicker_counts_once() {
        // A flickers for 4 ms at the edge of each spoke.
        let mut sensors = SpokeSensors::TODAY;
        let a = |ms: u32| matches!(ms % 40, 0 | 2 | 4..=15);
        let last = (0..400).map(|ms| sensors.update(a(ms), false, ms)).last();
        assert_eq!(last, Some(at(10, 360)));
    }

    #[test]
    fn b_ahead_by_more_than_the_window_is_forward() {
        // B rose 30 ms before A and is still high: the 2025-26 rule says forward.
        let mut sensors = SpokeSensors::TODAY;
        for ms in 0..30 {
            sensors.update(false, true, ms);
        }
        assert_eq!(sensors.update(true, true, 30), at(1, 30));
    }

    #[test]
    fn starts_at_zero() {
        assert_eq!(SpokeSensors::TODAY.count(), WheelCount::default());
    }
}
