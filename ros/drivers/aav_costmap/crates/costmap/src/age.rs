//! How old an input is: Nav2's "is it current" checks, for our nodes.
//!
//! Nav2's controller waits `costmap_update_timeout` (0.3 s) for a current costmap, then
//! fails the path; its planner waits 1 s. Its costmap drops a scan whose odometry it
//! can't find within `transform_tolerance` (0.3 s). Ours keep each input's arrival time
//! ([`Arrival`]), and the odometry by stamp ([`PoseHistory`]), and check them the same way.
//! Times are seconds on any one clock (the node's: sim time in a sim).

use std::collections::VecDeque;
use std::fmt;

use crate::Pose2;

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
    #[must_use]
    pub const fn new() -> Self {
        Self { at: None }
    }

    /// It arrived at `now`. A clock that went back (a sim restarted) takes the new time.
    pub fn mark(&mut self, now: f64) {
        self.at = Some(now);
    }

    /// When it last arrived.
    #[must_use]
    pub fn at(&self) -> Option<f64> {
        self.at
    }

    /// s since it last arrived; `None` if it never did.
    #[must_use]
    pub fn age(&self, now: f64) -> Option<f64> {
        self.at.map(|at| now - at)
    }

    /// Fresh enough to act on: arrived within `limit` s of `now`.
    #[must_use]
    pub fn fresh(&self, now: f64, limit: f64) -> bool {
        self.age(now).is_some_and(|age| age <= limit)
    }

    /// Nav2's wait for a current input, for a job that began at `since`: the time waited
    /// counts from its last arrival or from `since`, whichever is later, so a job gets
    /// `limit` s for its first input. `Err` once that is over `limit`.
    ///
    /// # Errors
    ///
    /// [`Stale`], with how long it waited, once that is over `limit`.
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

/// A pose by stamp (base_link in odom), kept for a while, to place each scan where the
/// car was when it was taken.
#[derive(Debug, Clone, Default)]
pub struct PoseHistory {
    /// Oldest first.
    poses: VecDeque<(f64, Pose2)>,
    /// s kept.
    kept: f64,
}

impl PoseHistory {
    /// Keeps `kept` s of poses.
    #[must_use]
    pub fn new(kept: f64) -> Self {
        Self {
            poses: VecDeque::new(),
            kept,
        }
    }

    /// A pose at stamp `t`. A stamp older than the newest (a sim restarted) starts again.
    pub fn push(&mut self, t: f64, pose: Pose2) {
        if self.poses.back().is_some_and(|(last, _)| t < *last) {
            self.poses.clear();
        }
        self.poses.push_back((t, pose));
        while self
            .poses
            .front()
            .is_some_and(|(first, _)| t - first > self.kept)
        {
            self.poses.pop_front();
        }
    }

    /// The newest pose and its stamp.
    #[must_use]
    pub fn latest(&self) -> Option<(f64, Pose2)> {
        self.poses.back().copied()
    }

    /// Forget every pose.
    pub fn clear(&mut self) {
        self.poses.clear();
    }

    /// The pose at stamp `t`: between the two nearest, or the nearest end if `t` is
    /// within `tolerance` s past it (as tf2 with Nav2's `transform_tolerance`). `None`
    /// further out: no pose is known for that time.
    #[must_use]
    pub fn at(&self, t: f64, tolerance: f64) -> Option<Pose2> {
        let (first, last) = (self.poses.front()?, self.poses.back()?);
        if t <= first.0 {
            return (first.0 - t <= tolerance).then_some(first.1);
        }
        if t >= last.0 {
            return (t - last.0 <= tolerance).then_some(last.1);
        }
        let i = self.poses.partition_point(|(s, _)| *s <= t);
        let (a, b) = (self.poses.get(i - 1)?, self.poses.get(i)?);
        let k = if b.0 > a.0 {
            (t - a.0) / (b.0 - a.0)
        } else {
            0.0
        };
        Some(a.1.lerp(&b.1, k))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arrival_ages_and_freshness() {
        let mut a = Arrival::new();
        assert_eq!(a.age(5.0), None);
        assert!(!a.fresh(5.0, 1.0));
        a.mark(10.0);
        assert_eq!(a.age(10.5), Some(0.5));
        assert!(a.fresh(10.29, 0.3));
        assert!(!a.fresh(10.31, 0.3));
    }

    #[test]
    fn a_job_gets_the_limit_for_its_first_input() {
        let mut a = Arrival::new();
        // Never arrived: the wait counts from the job's start.
        assert!(a.check(100.2, 100.0, 0.3).is_ok());
        let e = a.check(100.5, 100.0, 0.3).unwrap_err();
        assert!(e.never, "{e}");
        assert!((e.waited - 0.5).abs() < 1e-9);
        // Arrived long before the job: the job still gets its limit, then fails.
        a.mark(50.0);
        assert!(a.check(100.2, 100.0, 0.3).is_ok());
        assert!(a.check(100.4, 100.0, 0.3).is_err());
        // Arriving during the job: counted from the arrival.
        a.mark(100.4);
        assert!(a.check(100.6, 100.0, 0.3).is_ok());
        let e = a.check(101.0, 100.0, 0.3).unwrap_err();
        assert!(!e.never);
        assert!((e.waited - 0.6).abs() < 1e-9, "{e}");
    }

    #[test]
    fn poses_between_stamps_and_not_far_past_the_ends() {
        let mut h = PoseHistory::new(2.0);
        h.push(10.0, Pose2::new(0.0, 0.0, 0.0));
        h.push(10.1, Pose2::new(1.0, 0.0, 0.0));
        let mid = h.at(10.05, 0.3).unwrap();
        assert!((mid.x - 0.5).abs() < 1e-9);
        // Within the tolerance past the newest: the newest.
        assert_eq!(h.at(10.3, 0.3), Some(Pose2::new(1.0, 0.0, 0.0)));
        // Odometry stopped: a scan 0.5 s newer has no pose.
        assert_eq!(h.at(10.6, 0.3), None);
        assert_eq!(h.at(9.5, 0.3), None);
        assert_eq!(h.latest().map(|(t, _)| t), Some(10.1));
    }

    #[test]
    fn history_keeps_its_window_and_restarts_on_a_clock_going_back() {
        let mut h = PoseHistory::new(2.0);
        for i in 0..50 {
            h.push(f64::from(i) * 0.1, Pose2::IDENTITY);
        }
        assert_eq!(h.at(1.0, 0.3), None, "older than 2 s: gone");
        assert!(h.at(3.0, 0.0).is_some());
        h.push(0.5, Pose2::new(7.0, 0.0, 0.0));
        assert_eq!(h.latest(), Some((0.5, Pose2::new(7.0, 0.0, 0.0))));
        assert_eq!(h.at(4.0, 0.3), None);
    }
}
