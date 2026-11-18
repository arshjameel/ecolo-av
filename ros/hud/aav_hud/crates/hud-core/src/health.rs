//! Is each topic alive? When it was last heard, and OK / STALE / NEVER.
//!
//! Times are passed in (`now`), never read from the clock here, so the tests can say
//! exactly what time it is.

use std::{
    fmt,
    time::{Duration, Instant},
};

/// What the screen says about one topic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Health {
    /// Heard within the limit; how long ago.
    Ok(Duration),
    /// Heard once, but not within the limit; how long ago.
    Stale(Duration),
    /// Never heard since the HUD started.
    Never,
}

impl Health {
    /// The word on the screen.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            Self::Ok(_) => "OK",
            Self::Stale(_) => "STALE",
            Self::Never => "NEVER",
        }
    }

    /// How long ago the topic was heard, if ever.
    #[must_use]
    pub fn age(self) -> Option<Duration> {
        match self {
            Self::Ok(age) | Self::Stale(age) => Some(age),
            Self::Never => None,
        }
    }

    /// Only Ok counts as live.
    #[must_use]
    pub fn is_ok(self) -> bool {
        matches!(self, Self::Ok(_))
    }
}

impl fmt::Display for Health {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.word())
    }
}

/// When one topic was last heard.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LastSeen(Option<Instant>);

impl LastSeen {
    /// A message arrived at `now`.
    pub fn heard(&mut self, now: Instant) {
        self.0 = Some(now);
    }

    /// OK, STALE or NEVER at `now`, with `stale_after` as the limit.
    #[must_use]
    pub fn health(self, now: Instant, stale_after: Duration) -> Health {
        match self.0 {
            None => Health::Never,
            Some(at) => {
                let age = now.saturating_duration_since(at);
                if age <= stale_after {
                    Health::Ok(age)
                } else {
                    Health::Stale(age)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn never_then_ok_then_stale() {
        let start = Instant::now();
        let limit = Duration::from_secs(1);
        let mut seen = LastSeen::default();
        assert_eq!(seen.health(start, limit), Health::Never);
        seen.heard(start);
        let later = start + Duration::from_millis(300);
        assert_eq!(
            seen.health(later, limit),
            Health::Ok(Duration::from_millis(300))
        );
        let much_later = start + Duration::from_millis(1500);
        let h = seen.health(much_later, limit);
        assert_eq!(h, Health::Stale(Duration::from_millis(1500)));
        assert_eq!(h.word(), "STALE");
        assert!(!h.is_ok());
    }

    #[test]
    fn a_clock_going_backwards_is_age_zero() {
        let start = Instant::now();
        let mut seen = LastSeen::default();
        seen.heard(start + Duration::from_secs(5));
        assert_eq!(
            seen.health(start, Duration::from_secs(1)),
            Health::Ok(Duration::ZERO)
        );
    }
}
