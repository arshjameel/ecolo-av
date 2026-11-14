//! How old an input is: Nav2's controller waits `costmap_update_timeout` for a current
//! costmap, then fails the path (CONTROLLER_TIMED_OUT). The node keeps the costmap's
//! arrival time here and checks it each tick. The same rule as `aav_costmap`'s
//! `costmap::age` (this crate takes no dependencies). Times are seconds on one clock.

use std::fmt;

/// When an input last arrived.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Arrival {
    at: Option<f64>,
}

/// An input too old to act on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stale {
    /// s waited: since it last arrived, or since the wait began if that is later.
    pub waited: f64,
    /// s allowed.
    pub limit: f64,
    /// It never arrived.
    pub never: bool,
}

impl fmt::Display for Stale {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.never {
            write!(
                f,
                "none in {:.2} s (limit {:.2} s)",
                self.waited, self.limit
            )
        } else {
            write!(f, "{:.2} s old (limit {:.2} s)", self.waited, self.limit)
        }
    }
}

impl Arrival {
    /// Nothing has arrived yet.
    pub const fn new() -> Self {
        Self { at: None }
    }

    /// It arrived at `now`.
    pub fn mark(&mut self, now: f64) {
        self.at = Some(now);
    }

    /// s since it last arrived; `None` if it never did.
    pub fn age(&self, now: f64) -> Option<f64> {
        self.at.map(|at| now - at)
    }

    /// Nav2's wait for a current input, for a path that began at `since`: counted from
    /// the last arrival or from `since`, whichever is later, so a path gets `limit` s for
    /// its first costmap. `Err` once that is over `limit`.
    pub fn check(&self, now: f64, since: f64, limit: f64) -> Result<(), Stale> {
        let from = self.at.map_or(since, |at| at.max(since));
        let waited = now - from;
        if waited <= limit {
            Ok(())
        } else {
            Err(Stale {
                waited,
                limit,
                never: self.at.is_none(),
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_path_gets_the_limit_then_fails_on_a_stale_costmap() {
        let mut a = Arrival::new();
        assert_eq!(a.age(1.0), None);
        // No costmap yet: the path's first `limit` s are allowed.
        assert!(a.check(10.5, 10.0, 0.8).is_ok());
        let e = a.check(10.9, 10.0, 0.8).unwrap_err();
        assert!(e.never, "{e}");
        // Costmaps every 0.5 s: fine.
        for i in 0..10 {
            let t = 11.0 + 0.5 * f64::from(i);
            a.mark(t);
            assert!(a.check(t + 0.49, 10.0, 0.8).is_ok());
        }
        // They stop: the path fails once the last is over the limit old.
        assert!(a.check(15.5 + 0.79, 10.0, 0.8).is_ok());
        let e = a.check(15.5 + 0.81, 10.0, 0.8).unwrap_err();
        assert!(!e.never);
        assert!((e.waited - 0.81).abs() < 1e-9, "{e}");
        assert_eq!(a.age(16.0), Some(0.5));
    }
}
