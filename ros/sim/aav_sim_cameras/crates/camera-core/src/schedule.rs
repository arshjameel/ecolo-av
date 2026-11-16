//! When each camera takes its next picture, on the simulator's clock.
//!
//! As Gazebo's sensors: a camera at 10 Hz is due at 0.0, 0.1, 0.2 s... of sim time. A
//! tick that skips several due times takes one picture, not a burst. A clock that goes
//! back (the simulation restarted) starts over.

use std::time::Duration;

use crate::rig::Hz;

/// One camera's timer.
#[derive(Clone, Debug)]
pub struct Timer {
    period: Duration,
    next: Option<Duration>,
    last: Option<Duration>,
}

impl Timer {
    /// A timer for this rate, due at its first look.
    #[must_use]
    pub fn new(rate: Hz) -> Self {
        Self {
            period: rate.period(),
            next: None,
            last: None,
        }
    }

    /// Is a picture due at `now`? If so, the timer moves on to the next whole period.
    pub fn due(&mut self, now: Duration) -> bool {
        if self.last.is_some_and(|last| now < last) {
            self.next = None;
        }
        self.last = Some(now);
        match self.next {
            Some(next) if now < next => false,
            _ => {
                let period = self.period.as_nanos().max(1);
                let k = now.as_nanos() / period + 1;
                self.next = Some(Duration::from_nanos((k * period) as u64));
                true
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(v: u64) -> Duration {
        Duration::from_millis(v)
    }

    #[test]
    fn ten_hz_on_a_fifty_hz_clock() {
        let mut t = Timer::new(Hz::new(10.0).unwrap());
        let shots: Vec<u64> = (0..50).map(|i| i * 20).filter(|&m| t.due(ms(m))).collect();
        assert_eq!(shots, [0, 100, 200, 300, 400, 500, 600, 700, 800, 900]);
    }

    #[test]
    fn a_gap_gives_one_picture_not_a_burst() {
        let mut t = Timer::new(Hz::new(10.0).unwrap());
        assert!(t.due(ms(0)));
        assert!(t.due(ms(550)));
        assert!(!t.due(ms(560)));
        assert!(t.due(ms(600)));
    }

    #[test]
    fn restarts_when_the_clock_goes_back() {
        let mut t = Timer::new(Hz::new(10.0).unwrap());
        assert!(t.due(ms(5000)));
        assert!(t.due(ms(10)));
        assert!(!t.due(ms(50)));
    }
}
