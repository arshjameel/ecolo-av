//! Is the Pi link working? The ESP32's own verdict, from the link's counts.
//!
//! For the bench and the status lines only: it never drives anything. The line between
//! "OK" and not is the spine's own rule: a new command within `cmd_timeout_ms` (100 ms).
//!
//! | Verdict | Seen | Means |
//! |---|---|---|
//! | [`LinkVerdict::NoFramesYet`] | no good frame since boot | the Pi, its bridge or the cable is not there yet |
//! | [`LinkVerdict::Ok`] | a new command within the timeout | the Pi and the Orin are both sending |
//! | [`LinkVerdict::Stale`] | frames came, then stopped | the Pi's bridge stopped, or the cable came out |
//! | [`LinkVerdict::CounterFrozen`] | frames still come, but the counter doesn't move | the Pi is alive, the Orin's output mux isn't |
//! | [`LinkVerdict::BadFrames`] | bad frames or stray bytes in the last [`BAD_WINDOW_MS`] | a bad cable, the wrong baud, or the wrong `frame_version` |
//!
//! A version 1 frame (`link-v1`) has no counter: each good frame counts as a new command,
//! so it is never [`LinkVerdict::CounterFrozen`].

use crate::link::LinkStats;

/// Bad frames or stray bytes this recent make the verdict [`LinkVerdict::BadFrames`].
pub const BAD_WINDOW_MS: u32 = 3_000;

/// The link's state, judged on the ESP32.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkVerdict {
    /// No good frame since boot, and nothing bad either.
    NoFramesYet,
    /// A new command (a new counter) within the timeout.
    Ok,
    /// Good frames came, but none for `age_ms`, more than the timeout.
    Stale {
        /// How old the last good frame is.
        age_ms: u32,
    },
    /// Frames come, but the counter hasn't changed for `age_ms`: the Orin is not running.
    CounterFrozen {
        /// How long the counter has stood still.
        age_ms: u32,
    },
    /// Bad frames or stray bytes in the last [`BAD_WINDOW_MS`]: counts since boot.
    BadFrames {
        /// Frames turned down since boot.
        bad: u32,
        /// Stray bytes since boot.
        skipped: u32,
    },
}

impl LinkVerdict {
    /// Its name in a line: `no_frames`, `ok`, `stale`, `counter_frozen`, `bad_frames`.
    #[must_use]
    pub const fn name(&self) -> &'static str {
        match self {
            LinkVerdict::NoFramesYet => "no_frames",
            LinkVerdict::Ok => "ok",
            LinkVerdict::Stale { .. } => "stale",
            LinkVerdict::CounterFrozen { .. } => "counter_frozen",
            LinkVerdict::BadFrames { .. } => "bad_frames",
        }
    }

    /// True only for [`LinkVerdict::Ok`].
    #[must_use]
    pub const fn is_ok(&self) -> bool {
        matches!(self, LinkVerdict::Ok)
    }
}

/// One tick's verdict, with what it was judged from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkReport {
    /// The verdict.
    pub verdict: LinkVerdict,
    /// How old the last good frame is (a new counter or not); `None` before the first.
    pub age_ms: Option<u32>,
    /// `Some(true)`: the counter moved within the timeout. `Some(false)`: frames come
    /// but it doesn't. `None`: no frames within the timeout, or no counter (version 1).
    pub counter_moving: Option<bool>,
    /// The link's counts since boot.
    pub stats: LinkStats,
}

/// Follows the link's counts, tick by tick, and judges them.
#[derive(Debug, Clone, Copy)]
pub struct LinkHealth {
    timeout_ms: u32,
    last: LinkStats,
    /// When the last good frame came, the last new command, the last bad frame or byte.
    frame_ms: Option<u32>,
    fresh_ms: Option<u32>,
    bad_ms: Option<u32>,
}

impl LinkHealth {
    /// `timeout_ms`: the spine's command timeout (`SpineConfig::cmd_timeout_ms`).
    #[must_use]
    pub const fn new(timeout_ms: u32) -> Self {
        LinkHealth {
            timeout_ms,
            last: LinkStats {
                frames: 0,
                bad: 0,
                skipped: 0,
                repeats: 0,
                counter: None,
                uplinks: 0,
                uplink_errors: 0,
            },
            frame_ms: None,
            fresh_ms: None,
            bad_ms: None,
        }
    }

    /// Each tick: the link's counts now (`LegacyLink::stats`), and the tick's clock (ms,
    /// it may wrap). Returns the verdict.
    pub fn update(&mut self, now_ms: u32, stats: &LinkStats) -> LinkReport {
        let was = self.last;
        if stats.frames != was.frames {
            self.frame_ms = Some(now_ms);
        }
        // A new command: a good frame that wasn't a repeat of the counter before it.
        let fresh = |s: &LinkStats| s.frames.wrapping_sub(s.repeats);
        if fresh(stats) != fresh(&was) {
            self.fresh_ms = Some(now_ms);
        }
        if stats.bad != was.bad || stats.skipped != was.skipped {
            self.bad_ms = Some(now_ms);
        }
        self.last = *stats;
        self.report(now_ms)
    }

    /// The verdict at `now_ms`, from what [`update`](Self::update) saw last.
    #[must_use]
    pub fn report(&self, now_ms: u32) -> LinkReport {
        let age = |at: Option<u32>| at.map(|t| now_ms.wrapping_sub(t));
        let within = |at: Option<u32>| age(at).is_some_and(|a| a <= self.timeout_ms);
        let s = self.last;
        let frame_age = age(self.frame_ms);
        let fresh = within(self.fresh_ms);
        let frames_now = within(self.frame_ms);
        let bad_recently = age(self.bad_ms).is_some_and(|a| a <= BAD_WINDOW_MS);
        let has_counter = s.counter.is_some();
        let verdict = if bad_recently {
            LinkVerdict::BadFrames {
                bad: s.bad,
                skipped: s.skipped,
            }
        } else if fresh {
            LinkVerdict::Ok
        } else if frames_now && has_counter {
            LinkVerdict::CounterFrozen {
                age_ms: age(self.fresh_ms).unwrap_or(u32::MAX),
            }
        } else {
            match frame_age {
                None => LinkVerdict::NoFramesYet,
                Some(age_ms) => LinkVerdict::Stale { age_ms },
            }
        };
        let counter_moving = (has_counter && frames_now).then_some(fresh);
        LinkReport {
            verdict,
            age_ms: frame_age,
            counter_moving,
            stats: s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TIMEOUT: u32 = 100;

    /// A link that has taken `frames` good frames, `repeats` of them with an old counter.
    fn stats(frames: u32, repeats: u32, counter: Option<u8>) -> LinkStats {
        LinkStats {
            frames,
            repeats,
            counter,
            ..LinkStats::default()
        }
    }

    /// Ticks every 10 ms from `from` to `to`, a new counter every 20 ms (50 Hz).
    fn drive(h: &mut LinkHealth, s: &mut LinkStats, from: u32, to: u32) -> LinkReport {
        let mut last = h.report(from);
        for now in (from..to).step_by(10) {
            if now % 20 == 0 {
                s.frames += 1;
                s.counter = Some(s.counter.map_or(0, |c| c.wrapping_add(1)));
            }
            last = h.update(now, s);
        }
        last
    }

    #[test]
    fn nothing_yet_is_no_frames() {
        let mut h = LinkHealth::new(TIMEOUT);
        let r = h.update(5_000, &LinkStats::default());
        assert_eq!(r.verdict, LinkVerdict::NoFramesYet);
        assert_eq!((r.age_ms, r.counter_moving), (None, None));
        assert_eq!(r.verdict.name(), "no_frames");
    }

    #[test]
    fn a_moving_counter_is_ok() {
        let mut h = LinkHealth::new(TIMEOUT);
        let mut s = LinkStats::default();
        let r = drive(&mut h, &mut s, 0, 1_000);
        assert_eq!(r.verdict, LinkVerdict::Ok);
        assert!(r.verdict.is_ok());
        assert_eq!(r.counter_moving, Some(true));
        assert_eq!(r.age_ms, Some(10));
        assert_eq!(r.stats.frames, 50);
    }

    #[test]
    fn frames_that_stop_go_stale_after_the_timeout_not_before() {
        let mut h = LinkHealth::new(TIMEOUT);
        let mut s = LinkStats::default();
        drive(&mut h, &mut s, 0, 1_000); // the last frame at 980
        assert_eq!(
            h.update(1_080, &s).verdict,
            LinkVerdict::Ok,
            "100 ms: still OK"
        );
        let r = h.update(1_090, &s);
        assert_eq!(r.verdict, LinkVerdict::Stale { age_ms: 110 });
        assert_eq!(r.verdict.name(), "stale");
        assert_eq!(r.counter_moving, None, "no frames: nothing to tell");
    }

    #[test]
    fn frames_with_a_frozen_counter_say_counter_frozen() {
        let mut h = LinkHealth::new(TIMEOUT);
        let mut s = LinkStats::default();
        drive(&mut h, &mut s, 0, 1_000); // the last new counter at 980
        let mut r = h.report(1_000);
        for now in (1_000..1_500).step_by(20) {
            s.frames += 1;
            s.repeats += 1; // the Motor-Pi copies the last counter
            r = h.update(now, &s);
        }
        assert_eq!(r.verdict, LinkVerdict::CounterFrozen { age_ms: 500 });
        assert_eq!(r.verdict.name(), "counter_frozen");
        assert_eq!(r.counter_moving, Some(false));
        assert_eq!(r.age_ms, Some(0), "frames still come");
    }

    #[test]
    fn bad_frames_in_the_last_seconds_say_bad_frames_then_clear() {
        let mut h = LinkHealth::new(TIMEOUT);
        let mut s = LinkStats::default();
        drive(&mut h, &mut s, 0, 1_000);
        s.bad = 2;
        s.skipped = 17;
        let r = h.update(1_000, &s);
        assert_eq!(
            r.verdict,
            LinkVerdict::BadFrames {
                bad: 2,
                skipped: 17
            }
        );
        assert_eq!(r.verdict.name(), "bad_frames");
        // Good frames again, nothing bad: OK once the window has passed.
        assert!(matches!(
            drive(&mut h, &mut s, 1_000, 1_000 + BAD_WINDOW_MS).verdict,
            LinkVerdict::BadFrames { .. }
        ));
        let r = drive(&mut h, &mut s, 1_000 + BAD_WINDOW_MS, 2_000 + BAD_WINDOW_MS);
        assert_eq!(r.verdict, LinkVerdict::Ok);
    }

    #[test]
    fn only_stray_bytes_and_no_frame_is_bad_frames_not_no_frames() {
        // The Pi sends version 1 frames to a version 2 build: no frame ever passes.
        let mut h = LinkHealth::new(TIMEOUT);
        let r = h.update(
            200,
            &LinkStats {
                skipped: 130,
                ..LinkStats::default()
            },
        );
        assert_eq!(
            r.verdict,
            LinkVerdict::BadFrames {
                bad: 0,
                skipped: 130
            }
        );
        assert_eq!(r.age_ms, None);
    }

    #[test]
    fn version_1_frames_have_no_counter_and_are_never_frozen() {
        let mut h = LinkHealth::new(TIMEOUT);
        let mut r = h.report(0);
        for (i, now) in (0..500).step_by(20).enumerate() {
            r = h.update(now, &stats(i as u32 + 1, 0, None));
        }
        assert_eq!(r.verdict, LinkVerdict::Ok);
        assert_eq!(r.counter_moving, None);
        let r = h.update(700, &r.stats);
        assert!(matches!(r.verdict, LinkVerdict::Stale { .. }));
    }

    #[test]
    fn the_clock_wrapping_changes_nothing() {
        let mut h = LinkHealth::new(TIMEOUT);
        let start = u32::MAX - 15;
        h.update(start, &stats(1, 0, Some(1)));
        let r = h.update(start.wrapping_add(50), &stats(1, 0, Some(1)));
        assert_eq!(r.verdict, LinkVerdict::Ok);
        assert_eq!(r.age_ms, Some(50));
        let r = h.update(start.wrapping_add(150), &stats(1, 0, Some(1)));
        assert_eq!(r.verdict, LinkVerdict::Stale { age_ms: 150 });
    }
}
