//! The odometry at a scan's stamp: a short buffer of the local EKF's poses, interpolated.
//!
//! nav2_amcl looks up TF's `odom -> base_link` at the scan's stamp; the node has no TF
//! library, so it keeps the same transform from `ekf_local`'s odometry (the one that
//! publishes that TF) and asks here.

use std::{collections::VecDeque, time::Duration};

use crate::pose::Pose2;

/// The odometry's recent poses, oldest first.
#[derive(Clone, Debug)]
pub struct OdomBuffer {
    poses: VecDeque<(Duration, Pose2)>,
    keep: Duration,
}

impl OdomBuffer {
    /// A buffer that keeps `keep` of history.
    #[must_use]
    pub fn new(keep: Duration) -> Self {
        Self {
            poses: VecDeque::new(),
            keep,
        }
    }

    /// Adds the odometry's pose at `stamp`. A stamp from before the newest (a replay or a
    /// sim restarted) starts the buffer again.
    pub fn push(&mut self, stamp: Duration, pose: Pose2) {
        if self.poses.back().is_some_and(|(t, _)| stamp < *t) {
            self.poses.clear();
        }
        self.poses.push_back((stamp, pose));
        while self
            .poses
            .front()
            .is_some_and(|(t, _)| stamp.saturating_sub(*t) > self.keep)
        {
            self.poses.pop_front();
        }
    }

    /// The newest pose and its stamp.
    #[must_use]
    pub fn latest(&self) -> Option<(Duration, Pose2)> {
        self.poses.back().copied()
    }

    /// The pose at `stamp`: between the two around it, or the newest when `stamp` is at
    /// most `tolerance` after it. None before the buffer, or too far after it.
    #[must_use]
    pub fn at(&self, stamp: Duration, tolerance: Duration) -> Option<Pose2> {
        let (newest_t, newest) = self.latest()?;
        if stamp >= newest_t {
            return (stamp - newest_t <= tolerance).then_some(newest);
        }
        let after = self.poses.iter().position(|(t, _)| *t >= stamp)?;
        let (t1, p1) = self.poses[after];
        if after == 0 {
            return (t1 - stamp <= tolerance).then_some(p1);
        }
        let (t0, p0) = self.poses[after - 1];
        let span = (t1 - t0).as_secs_f64();
        let t = if span > 0.0 {
            (stamp - t0).as_secs_f64() / span
        } else {
            1.0
        };
        Some(p0.lerp(&p1, t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: Duration = Duration::from_millis(200);

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn interpolates_between_poses() {
        let mut odom = OdomBuffer::new(Duration::from_secs(2));
        odom.push(ms(1000), Pose2::new(0.0, 0.0, 0.0));
        odom.push(ms(1100), Pose2::new(1.0, 0.0, 0.2));
        let p = odom.at(ms(1050), TOL).expect("inside");
        assert!((p.x - 0.5).abs() < 1e-12 && (p.yaw - 0.1).abs() < 1e-12);
        assert_eq!(odom.at(ms(1250), TOL), Some(Pose2::new(1.0, 0.0, 0.2)));
        assert_eq!(odom.at(ms(1400), TOL), None, "too far after");
        assert_eq!(odom.at(ms(500), TOL), None, "before the buffer");
    }

    #[test]
    fn forgets_old_poses_and_starts_again_on_a_time_jump() {
        let mut odom = OdomBuffer::new(Duration::from_secs(1));
        for i in 0..30 {
            odom.push(ms(100 * i), Pose2::new(i as f64, 0.0, 0.0));
        }
        assert_eq!(odom.at(ms(1000), TOL), None, "kept 1 s only");
        odom.push(ms(10), Pose2::default());
        assert_eq!(odom.latest(), Some((ms(10), Pose2::default())));
        assert_eq!(odom.at(ms(2900), TOL), None);
    }
}
