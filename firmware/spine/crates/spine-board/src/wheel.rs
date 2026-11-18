//! The car's speed, from the wheel encoder's counter.
//!
//! `spine_core::encoder::SpeedEstimator` works from the time between counts, so it
//! stays smooth at walking pace. It wants to hear about every count. The board gives
//! a counter and the time of its latest count instead; when several counts arrive
//! between two ticks, they are spread evenly over the time since the last one.

use crate::board::WheelCount;
use spine_core::encoder::SpeedEstimator;
/// Two spoke sensors, counted the 2025-26 way: in `wheel-encoder`, which the stand-alone
/// wheel encoder Pico 2 shares.
pub use wheel_encoder::SpokeSensors;

/// The car's speed from the wheel encoder's counter.
#[derive(Debug, Clone, Copy)]
pub struct WheelSpeed {
    estimator: SpeedEstimator,
    last: Option<WheelCount>,
}

impl WheelSpeed {
    /// `metres_per_count` a count; with no count for `timeout_ms`, the car is stopped.
    #[must_use]
    pub fn new(metres_per_count: f32, timeout_ms: u32) -> Self {
        WheelSpeed {
            estimator: SpeedEstimator::new(metres_per_count, timeout_ms.saturating_mul(1000)),
            last: None,
        }
    }

    /// Feed the counter as read at `now_ms`. Returns the speed in m/s, positive
    /// forward.
    pub fn update(&mut self, wheel: WheelCount, now_ms: u32) -> f32 {
        if let Some(last) = self.last {
            let counts = wheel.count.wrapping_sub(last.count);
            if counts != 0 {
                let step = if counts > 0 { 1 } else { -1 };
                let n = counts.unsigned_abs();
                let at_us = us(wheel.at_ms);
                if n > 1 {
                    // The count before the latest one, one average period earlier.
                    let period_us = us(wheel.at_ms.wrapping_sub(last.at_ms)) / n;
                    self.estimator.on_count(at_us.wrapping_sub(period_us), step);
                }
                self.estimator.on_count(at_us, step);
            }
        }
        self.last = Some(wheel);
        self.estimator.speed(us(now_ms))
    }
}

/// Milliseconds to the estimator's microseconds. Both wrap, and only differences
/// matter, so the wrap is harmless.
fn us(ms: u32) -> u32 {
    ms.wrapping_mul(1000)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 0.1 m a count, stopped after 1 s without a count.
    fn wheel() -> WheelSpeed {
        WheelSpeed::new(0.1, 1000)
    }

    fn at(count: i32, at_ms: u32) -> WheelCount {
        WheelCount { count, at_ms }
    }

    #[test]
    fn stopped_until_it_moves() {
        let mut w = wheel();
        assert_eq!(w.update(at(0, 0), 0), 0.0);
        assert_eq!(w.update(at(0, 0), 500), 0.0);
    }

    #[test]
    fn one_count_a_tick() {
        // A count every 50 ms at 0.1 m a count: 2 m/s.
        let mut w = wheel();
        w.update(at(0, 0), 0);
        let mut v = 0.0;
        for i in 1..=5 {
            v = w.update(at(i, i as u32 * 50), i as u32 * 50);
        }
        assert!((v - 2.0).abs() < 1e-4, "{v}");
    }

    #[test]
    fn several_counts_a_tick() {
        // 5 counts every 10 ms: 50 m/s, far too fast for the car, but simple maths.
        let mut w = wheel();
        w.update(at(0, 0), 0);
        w.update(at(5, 10), 10);
        let v = w.update(at(10, 20), 20);
        assert!((v - 50.0).abs() < 1e-3, "{v}");
    }

    #[test]
    fn reverse_is_negative() {
        let mut w = wheel();
        w.update(at(100, 0), 0);
        w.update(at(99, 100), 100);
        let v = w.update(at(98, 200), 200);
        assert!((v + 1.0).abs() < 1e-4, "{v}");
    }

    #[test]
    fn slows_then_stops_without_counts() {
        let mut w = wheel();
        w.update(at(0, 0), 0);
        w.update(at(1, 100), 100);
        w.update(at(2, 200), 200); // 1 m/s
        assert!(w.update(at(2, 200), 400) < 0.6); // 200 ms since the last count
        assert_eq!(w.update(at(2, 200), 1300), 0.0); // over the 1 s timeout
    }

    #[test]
    fn spokes_feed_the_speed() {
        // A spoke every 40 ms at 0.1178 m a spoke: 2.945 m/s.
        let mut sensors = SpokeSensors::TODAY;
        let mut speed = WheelSpeed::new(0.1178, 1000);
        let mut v = 0.0;
        for ms in 0..400 {
            let count = sensors.update(ms % 40 < 15, (10..25).contains(&(ms % 40)), ms);
            if ms % 10 == 0 {
                v = speed.update(count, ms); // the spine's tick reads it every 10 ms
            }
        }
        assert!((v - 2.945).abs() < 0.01, "{v}");
    }

    #[test]
    fn survives_the_counter_wrapping() {
        let mut w = wheel();
        w.update(at(i32::MAX - 1, 0), 0);
        w.update(at(i32::MAX, 100), 100);
        let v = w.update(at(i32::MIN, 200), 200); // one more count forward
        assert!((v - 1.0).abs() < 1e-4, "{v}");
    }
}
