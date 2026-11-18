//! How well the tick keeps its time: the longest gap between ticks, and how many came late.
//!
//! The baseline for the timing test (`boards/esp32-umod/README.md`): the same numbers with
//! no radio and with one. It only compares two clock readings, so it costs almost nothing.

/// A tick later than this counts as an overrun: 10 ms past when it was due.
pub const LATE_US: u32 = 10_000;

/// The longest gap between ticks since boot, and the ticks that came late.
///
/// With a 10 ms tick, a gap over 20 ms is a tick more than 10 ms late
/// (`firmware/status/README.md`: `tick`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TickTiming {
    period_us: u32,
    last_us: Option<u64>,
    worst_gap_us: u32,
    overruns: u32,
}

impl TickTiming {
    /// `period_us`: how often a tick is due.
    #[must_use]
    pub const fn new(period_us: u32) -> Self {
        TickTiming {
            period_us,
            last_us: None,
            worst_gap_us: 0,
            overruns: 0,
        }
    }

    /// One tick began at `now_us`, on a clock that only counts up.
    pub fn tick(&mut self, now_us: u64) {
        if let Some(last) = self.last_us {
            let gap = u32::try_from(now_us.saturating_sub(last)).unwrap_or(u32::MAX);
            self.worst_gap_us = self.worst_gap_us.max(gap);
            if gap > self.period_us.saturating_add(LATE_US) {
                self.overruns = self.overruns.saturating_add(1);
            }
        }
        self.last_us = Some(now_us);
    }

    /// The longest gap between two ticks since boot, in microseconds; 0 before the second.
    #[must_use]
    pub const fn worst_gap_us(&self) -> u32 {
        self.worst_gap_us
    }

    /// How many ticks came more than [`LATE_US`] late.
    #[must_use]
    pub const fn overruns(&self) -> u32 {
        self.overruns
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_is_known_before_the_second_tick() {
        let mut t = TickTiming::new(10_000);
        assert_eq!((t.worst_gap_us(), t.overruns()), (0, 0));
        t.tick(5_000_000);
        assert_eq!((t.worst_gap_us(), t.overruns()), (0, 0));
    }

    #[test]
    fn keeps_the_longest_gap() {
        let mut t = TickTiming::new(10_000);
        for now in [0, 10_000, 20_840, 30_000, 40_000] {
            t.tick(now);
        }
        assert_eq!(t.worst_gap_us(), 10_840);
        assert_eq!(t.overruns(), 0, "the contract's example: 10840 is not late");
    }

    #[test]
    fn a_tick_over_10_ms_late_is_an_overrun() {
        let mut t = TickTiming::new(10_000);
        t.tick(0);
        t.tick(20_000); // exactly 10 ms late: not over
        assert_eq!(t.overruns(), 0);
        t.tick(40_001);
        assert_eq!(t.overruns(), 1);
        assert_eq!(t.worst_gap_us(), 20_001);
    }

    #[test]
    fn a_clock_going_backwards_or_a_huge_gap_doesnt_panic() {
        let mut t = TickTiming::new(10_000);
        t.tick(100);
        t.tick(50);
        assert_eq!(t.worst_gap_us(), 0);
        t.tick(u64::MAX);
        assert_eq!(t.worst_gap_us(), u32::MAX);
        assert_eq!(t.overruns(), 1);
    }
}
