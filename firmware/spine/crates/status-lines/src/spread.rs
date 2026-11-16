//! The lowest and highest device values between two status lines.

use crate::Levels;

/// Follows `steer_counts`, `brake_counts` and `dac` every tick, so a spike shorter than
/// the time between two status lines still shows in the line's `low` and `high`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Spread {
    range: Option<(Levels, Levels)>,
}

impl Spread {
    #[must_use]
    pub const fn new() -> Self {
        Spread { range: None }
    }

    /// One tick's values.
    pub fn add(&mut self, now: Levels) {
        let (low, high) = self.range.get_or_insert((now, now));
        *low = Levels {
            steer_counts: low.steer_counts.min(now.steer_counts),
            brake_counts: low.brake_counts.min(now.brake_counts),
            dac: low.dac.min(now.dac),
        };
        *high = Levels {
            steer_counts: high.steer_counts.max(now.steer_counts),
            brake_counts: high.brake_counts.max(now.brake_counts),
            dac: high.dac.max(now.dac),
        };
    }

    /// The lowest and highest since the last `take`, then start again. With no tick in
    /// between, both are `now`.
    pub fn take(&mut self, now: Levels) -> (Levels, Levels) {
        self.range.take().unwrap_or((now, now))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn levels(steer_counts: u16, brake_counts: u16, dac: u16) -> Levels {
        Levels {
            steer_counts,
            brake_counts,
            dac,
        }
    }

    #[test]
    fn a_short_spike_shows() {
        let mut s = Spread::new();
        s.add(levels(1750, 400, 204));
        s.add(levels(1760, 400, 300));
        s.add(levels(1755, 400, 210));
        let (low, high) = s.take(levels(1755, 400, 210));
        assert_eq!(low, levels(1750, 400, 204));
        assert_eq!(high, levels(1760, 400, 300));
    }

    #[test]
    fn take_starts_again_and_an_empty_spread_is_now() {
        let mut s = Spread::new();
        s.add(levels(1, 2, 3));
        s.take(levels(0, 0, 0));
        let now = levels(1950, 3500, 0);
        assert_eq!(s.take(now), (now, now));
    }
}
