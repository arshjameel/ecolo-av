//! The local EKF's recent poses, to place a cloud with the pose **at its stamp**.
//!
//! A cloud arrives late (0.9 MB, then parsed): at 3 m/s a 50 ms mismatch moves walls
//! 15 cm, and a turn doubles them far away. So the last second of poses is kept, and the
//! pose at the cloud's stamp is interpolated between the two around it.

use std::{collections::VecDeque, time::Duration};

use map_core::geom::Pose;
use thiserror::Error;

use crate::frames::Stamp;

/// Why there is no pose at a stamp.
#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum Lookup {
    #[error("no local EKF pose yet")]
    Empty,
    /// Older than the oldest pose kept: drop the cloud.
    #[error("the cloud is {0:.2} s older than the poses kept")]
    TooOld(f64),
    /// Newer than the newest pose: wait for the next one.
    #[error("the cloud is {0:.2} s newer than the newest pose")]
    TooNew(f64),
}

/// Poses by stamp, oldest first, within a window.
#[derive(Debug, Clone)]
pub struct PoseBuffer {
    window: Duration,
    poses: VecDeque<(Stamp, Pose)>,
}

impl PoseBuffer {
    pub fn new(window: Duration) -> Self {
        Self {
            window,
            poses: VecDeque::new(),
        }
    }

    /// A pose at `stamp`. One older than the newest (out of order) is ignored; a stamp
    /// far back in time (Gazebo restarted) starts the buffer again.
    pub fn push(&mut self, stamp: Stamp, pose: Pose) {
        if !(pose.position.is_finite() && pose.yaw.is_finite()) {
            return;
        }
        if let Some(&(newest, _)) = self.poses.back() {
            if stamp <= newest {
                if newest.seconds_since(stamp) > self.window.as_secs_f64() {
                    self.poses.clear();
                } else {
                    return;
                }
            }
        }
        self.poses.push_back((stamp, pose));
        let window = self.window.as_secs_f64();
        while let Some(&(oldest, _)) = self.poses.front() {
            if stamp.seconds_since(oldest) > window {
                self.poses.pop_front();
            } else {
                break;
            }
        }
    }

    /// The newest pose.
    pub fn latest(&self) -> Option<(Stamp, Pose)> {
        self.poses.back().copied()
    }

    /// How many poses are kept.
    pub fn len(&self) -> usize {
        self.poses.len()
    }

    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }

    /// The pose at `stamp`: interpolated between the two around it (the heading the
    /// short way round), or exactly a kept one.
    pub fn at(&self, stamp: Stamp) -> Result<Pose, Lookup> {
        let (&(oldest, first), &(newest, last)) = match (self.poses.front(), self.poses.back()) {
            (Some(a), Some(b)) => (a, b),
            _ => return Err(Lookup::Empty),
        };
        if stamp < oldest {
            return Err(Lookup::TooOld(oldest.seconds_since(stamp)));
        }
        if stamp > newest {
            return Err(Lookup::TooNew(stamp.seconds_since(newest)));
        }
        if stamp == oldest {
            return Ok(first);
        }
        if stamp == newest {
            return Ok(last);
        }
        // The first pose at or after the stamp; the one before it is before the stamp.
        let after = self.poses.partition_point(|&(s, _)| s < stamp);
        let (s1, p1) = self.poses[after];
        let (s0, p0) = self.poses[after - 1];
        let k = stamp.seconds_since(s0) / s1.seconds_since(s0);
        Ok(lerp(p0, p1, k))
    }
}

/// Between `a` (k = 0) and `b` (k = 1); the heading turns the short way.
fn lerp(a: Pose, b: Pose, k: f64) -> Pose {
    let turn = (b.yaw - a.yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    Pose {
        position: a.position + (b.position - a.position) * k,
        yaw: a.yaw + turn * k,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn s(ms: i64) -> Stamp {
        Stamp(ms * 1_000_000)
    }

    fn buffer() -> PoseBuffer {
        let mut b = PoseBuffer::new(Duration::from_secs(1));
        b.push(s(1000), Pose::new(0.0, 0.0, 0.0));
        b.push(s(1100), Pose::new(1.0, 0.0, 0.2));
        b
    }

    #[test]
    fn a_stamp_between_two_poses_is_interpolated() {
        let p = buffer().at(s(1025)).expect("inside");
        assert!((p.position.x - 0.25).abs() < 1e-9);
        assert!((p.yaw - 0.05).abs() < 1e-9);
        assert_eq!(
            buffer().at(s(1100)).expect("the end"),
            Pose::new(1.0, 0.0, 0.2)
        );
    }

    #[test]
    fn outside_the_poses_says_which_way() {
        let b = buffer();
        assert!(matches!(b.at(s(900)), Err(Lookup::TooOld(_))));
        assert!(matches!(b.at(s(1200)), Err(Lookup::TooNew(_))));
        assert_eq!(
            PoseBuffer::new(Duration::from_secs(1)).at(s(0)),
            Err(Lookup::Empty)
        );
    }

    #[test]
    fn the_heading_turns_the_short_way_past_pi() {
        let mut b = PoseBuffer::new(Duration::from_secs(1));
        b.push(s(0), Pose::new(0.0, 0.0, PI - 0.1));
        b.push(s(100), Pose::new(0.0, 0.0, -PI + 0.1));
        let mid = b.at(s(50)).expect("inside");
        assert!((mid.yaw.cos() + 1.0).abs() < 1e-9, "{}", mid.yaw);
    }

    #[test]
    fn old_poses_fall_out_and_out_of_order_ones_are_ignored() {
        let mut b = buffer();
        b.push(s(1050), Pose::new(9.0, 9.0, 0.0));
        assert_eq!(b.len(), 2);
        b.push(s(2150), Pose::new(2.0, 0.0, 0.0));
        assert_eq!(b.len(), 1, "both older than 1 s");
        assert_eq!(b.latest().map(|(t, _)| t), Some(s(2150)));
        // Gazebo restarted: time went back past the window.
        b.push(s(10), Pose::new(0.0, 0.0, 0.0));
        assert_eq!(b.latest().map(|(t, _)| t), Some(s(10)));
        b.push(s(20), Pose::new(f64::NAN, 0.0, 0.0));
        assert_eq!(b.len(), 1);
    }
}
