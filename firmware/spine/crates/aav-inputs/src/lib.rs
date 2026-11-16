//! Clean digital inputs for every board: debounced switches and buttons, and edges
//! with a lockout for counting.
//!
//! A switch or a relay contact bounces for a few milliseconds when it moves, and an IR
//! sensor flickers at the edge of a spoke. Read raw, one press looks like several. These
//! filters turn a raw level into clean changes. Like `spine-core`, this is plain `no_std`
//! Rust with no hardware access, so the same code runs on a board and in `cargo test` on
//! a laptop. The board code reads the pin and the clock, and calls `update`.
//!
//! Times use a free-running millisecond counter that may wrap.
#![no_std]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

/// A change of the cleaned-up level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Edge {
    /// Low to high.
    Rising,
    /// High to low.
    Falling,
}

impl Edge {
    fn to(level: bool) -> Edge {
        if level {
            Edge::Rising
        } else {
            Edge::Falling
        }
    }
}

/// For switches and buttons: the output takes a new level only after the raw input has
/// held it, without a break, for long enough.
///
/// Each direction has its own time, so a safety input can act at once and recover
/// slowly. An active-high e-stop that trips on the first sample but must stay released
/// for 50 ms before it counts, starting tripped so that a reboot is safe:
///
/// ```
/// let mut estop = aav_inputs::Debouncer::with_times(true, 0, 50);
/// assert!(estop.level());
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Debouncer {
    to_high_ms: u32,
    to_low_ms: u32,
    level: bool,
    /// Since when the raw input has differed from `level`, if it does.
    since_ms: Option<u32>,
}

impl Debouncer {
    /// The same time in both directions.
    #[must_use]
    pub const fn new(initial: bool, stable_ms: u32) -> Self {
        Debouncer::with_times(initial, stable_ms, stable_ms)
    }

    /// `to_high_ms`: how long the input must stay high before the output goes high;
    /// `to_low_ms` the same for low. 0 means at once.
    #[must_use]
    pub const fn with_times(initial: bool, to_high_ms: u32, to_low_ms: u32) -> Self {
        Debouncer {
            to_high_ms,
            to_low_ms,
            level: initial,
            since_ms: None,
        }
    }

    /// Feed the raw level read at `now_ms`. Returns the change, if the output changed.
    pub fn update(&mut self, raw: bool, now_ms: u32) -> Option<Edge> {
        if raw == self.level {
            self.since_ms = None; // a bounce, over before it counted
            return None;
        }
        let since = *self.since_ms.get_or_insert(now_ms);
        let needed = if raw { self.to_high_ms } else { self.to_low_ms };
        if now_ms.wrapping_sub(since) < needed {
            return None;
        }
        self.level = raw;
        self.since_ms = None;
        Some(Edge::to(raw))
    }

    /// The cleaned-up level.
    #[must_use]
    pub fn level(&self) -> bool {
        self.level
    }
}

/// For counting fast signals, such as a wheel-encoder sensor: reacts to a change at once,
/// then ignores the input for `lockout_ms`. Unlike a [`Debouncer`] it adds no delay, but
/// a real change must take longer than `lockout_ms` (a spoke passing the sensor). The
/// 2025-26 encoder firmware used the same idea, with 10 ms.
#[derive(Debug, Clone, Copy)]
pub struct Lockout {
    lockout_ms: u32,
    level: bool,
    changed_ms: Option<u32>,
}

impl Lockout {
    /// Starts at `initial`, free to change at once.
    #[must_use]
    pub const fn new(initial: bool, lockout_ms: u32) -> Self {
        Lockout {
            lockout_ms,
            level: initial,
            changed_ms: None,
        }
    }

    /// Feed the raw level read at `now_ms`. Returns the change, if one counts.
    pub fn update(&mut self, raw: bool, now_ms: u32) -> Option<Edge> {
        if raw == self.level {
            return None;
        }
        if let Some(t) = self.changed_ms {
            if now_ms.wrapping_sub(t) < self.lockout_ms {
                return None; // still bouncing from the last change
            }
        }
        self.level = raw;
        self.changed_ms = Some(now_ms);
        Some(Edge::to(raw))
    }

    /// The level after the last change that counted.
    #[must_use]
    pub fn level(&self) -> bool {
        self.level
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feed a list of (time, raw level) samples; collect the changes and their times.
    fn run<F: FnMut(bool, u32) -> Option<Edge>>(
        mut f: F,
        trace: &[(u32, bool)],
    ) -> [(u32, Option<Edge>); 16] {
        let mut out = [(0, None); 16];
        for (i, &(t, raw)) in trace.iter().enumerate() {
            out[i] = (t, f(raw, t));
        }
        out
    }

    fn changes(out: &[(u32, Option<Edge>)]) -> usize {
        out.iter().filter(|(_, e)| e.is_some()).count()
    }

    #[test]
    fn starts_at_the_initial_level() {
        assert!(Debouncer::new(true, 20).level());
        assert!(!Debouncer::new(false, 20).level());
    }

    #[test]
    fn a_bouncing_press_counts_once() {
        let mut d = Debouncer::new(false, 20);
        // A press that bounces for 6 ms, then holds; then a release that bounces.
        let trace = [
            (0, true),
            (2, false),
            (4, true),
            (6, false),
            (8, true),
            (20, true),
            (28, true),
            (100, false),
            (101, true),
            (103, false),
            (130, false),
        ];
        let out = run(|r, t| d.update(r, t), &trace);
        assert_eq!(changes(&out), 2);
        assert_eq!(out[6], (28, Some(Edge::Rising))); // 20 ms after the last bounce at 8
        assert_eq!(out[10], (130, Some(Edge::Falling)));
        assert!(!d.level());
    }

    #[test]
    fn a_glitch_shorter_than_the_time_is_ignored() {
        let mut d = Debouncer::new(false, 20);
        assert_eq!(d.update(true, 0), None);
        assert_eq!(d.update(true, 19), None);
        assert_eq!(d.update(false, 19), None); // back before 20 ms
        assert_eq!(d.update(true, 30), None); // the timer starts again here
        assert_eq!(d.update(true, 49), None);
        assert_eq!(d.update(true, 50), Some(Edge::Rising));
    }

    #[test]
    fn a_safety_input_trips_at_once_and_recovers_slowly() {
        let mut estop = Debouncer::with_times(false, 0, 50);
        assert_eq!(estop.update(true, 1000), Some(Edge::Rising)); // first sample: tripped
        assert_eq!(estop.update(false, 1010), None);
        assert_eq!(estop.update(true, 1020), None); // a bounce keeps it tripped
        assert_eq!(estop.update(false, 1030), None);
        assert_eq!(estop.update(false, 1079), None);
        assert_eq!(estop.update(false, 1080), Some(Edge::Falling)); // 50 ms clear
    }

    #[test]
    fn debouncer_survives_counter_wrap() {
        let mut d = Debouncer::new(false, 20);
        assert_eq!(d.update(true, u32::MAX - 5), None);
        assert_eq!(d.update(true, 13), None); // 19 ms later, after the wrap
        assert_eq!(d.update(true, 14), Some(Edge::Rising)); // 20 ms
    }

    #[test]
    fn lockout_counts_each_spoke_once() {
        let mut s = Lockout::new(false, 10);
        // Two spokes: each edge flickers for 3 ms. Rising at 0 and 50, falling at 25 and 75.
        let trace = [
            (0, true),
            (1, false),
            (2, true),
            (25, false),
            (26, true),
            (27, false),
            (50, true),
            (52, false),
            (53, true),
            (75, false),
            (77, true),
            (78, false),
        ];
        let out = run(|r, t| s.update(r, t), &trace);
        let rising = out.iter().filter(|(_, e)| *e == Some(Edge::Rising)).count();
        let falling = out
            .iter()
            .filter(|(_, e)| *e == Some(Edge::Falling))
            .count();
        assert_eq!((rising, falling), (2, 2));
        assert_eq!(out[0], (0, Some(Edge::Rising))); // no delay: the first edge counts at once
    }

    #[test]
    fn lockout_accepts_the_next_change_after_the_lockout() {
        let mut s = Lockout::new(false, 10);
        assert_eq!(s.update(true, 100), Some(Edge::Rising));
        assert_eq!(s.update(false, 109), None);
        assert_eq!(s.update(false, 110), Some(Edge::Falling));
    }

    #[test]
    fn lockout_survives_counter_wrap() {
        let mut s = Lockout::new(false, 10);
        assert_eq!(s.update(true, u32::MAX - 2), Some(Edge::Rising));
        assert_eq!(s.update(false, 5), None); // 8 ms later
        assert_eq!(s.update(false, 7), Some(Edge::Falling)); // 10 ms later
    }
}
