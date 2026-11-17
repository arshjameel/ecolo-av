//! The kerbs, remembered: the roof lidar loses a kerb about 4 m before the bumper.
//!
//! Its lowest beam meets the road 5.7 m ahead of the lidar, so a kerb nearer than that is
//! under every beam. The stop zone reaches only 1 m past the bumper: without a memory it
//! never sees a kerb at all (sim-check --kerb, 16 Nov: the car drove 0.24 m into one).
//! So each step point is kept in `odom`, where a kerb stays put, for [`StepMemory::keep`]
//! seconds, and every cloud sends the ones still in reach back in `base_link`.
//!
//! ```text
//!   seen 8 m out          kept in odom             sent back in base_link
//!   ▲ step  ─────────▶   ▪ (x, y, z) at t    ──▶   each cloud, while younger than keep
//! ```

use std::collections::HashMap;

use crate::Point;

/// Where `base_link` is in `odom`: metres and radians.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2 {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad, + left.
    pub yaw: f64,
}

impl Pose2 {
    /// A point in `base_link` into `odom`.
    #[must_use]
    pub fn to_odom(&self, p: &Point) -> Point {
        let (s, c) = self.yaw.sin_cos();
        Point::new(self.x + c * p.x - s * p.y, self.y + s * p.x + c * p.y, p.z)
    }

    /// A point in `odom` into `base_link`.
    #[must_use]
    pub fn to_base(&self, p: &Point) -> Point {
        let (s, c) = self.yaw.sin_cos();
        let (dx, dy) = (p.x - self.x, p.y - self.y);
        Point::new(c * dx + s * dy, -s * dx + c * dy, p.z)
    }
}

/// The step points of the last [`StepMemory::keep`] seconds, one per `cell` square in
/// `odom` (the newest).
#[derive(Clone, Debug)]
pub struct StepMemory {
    /// s: a point older than this is forgotten.
    pub keep: f64,
    /// m, flat from `base_link`: further is forgotten too.
    pub range: f64,
    /// m: one point per square this size.
    pub cell: f64,
    held: HashMap<[i64; 2], (f64, Point)>,
}

impl StepMemory {
    /// An empty memory.
    #[must_use]
    pub fn new(keep: f64, range: f64, cell: f64) -> Self {
        Self {
            keep,
            range,
            cell,
            held: HashMap::new(),
        }
    }

    /// The points held now.
    #[must_use]
    pub fn len(&self) -> usize {
        self.held.len()
    }

    /// Nothing held.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.held.is_empty()
    }

    /// Forget everything (odometry jumped, or went back in time).
    pub fn clear(&mut self) {
        self.held.clear();
    }

    /// Keep this cloud's steps (`found`, in `base_link`, the car at `pose` at `stamp` s),
    /// forget the old and the far, and return every point held, in `base_link`, sorted.
    pub fn update(&mut self, stamp: f64, pose: Pose2, found: &[Point]) -> Vec<Point> {
        for p in found {
            let o = pose.to_odom(p);
            let key = [
                (o.x / self.cell).floor() as i64,
                (o.y / self.cell).floor() as i64,
            ];
            self.held.insert(key, (stamp, o));
        }
        let (keep, range) = (self.keep, self.range);
        self.held.retain(|_, (t, o)| {
            let age = stamp - *t;
            (0.0..=keep).contains(&age) && pose.to_base(o).flat() <= range
        });
        let mut out: Vec<([i64; 2], Point)> = self
            .held
            .iter()
            .map(|(k, (_, o))| (*k, pose.to_base(o)))
            .collect();
        out.sort_unstable_by_key(|(k, _)| *k);
        out.into_iter().map(|(_, p)| p).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: &Point, b: &Point) -> bool {
        (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && (a.z - b.z).abs() < 1e-9
    }

    #[test]
    fn a_pose_turns_both_ways() {
        let pose = Pose2 {
            x: 3.0,
            y: -2.0,
            yaw: 0.7,
        };
        let p = Point::new(4.0, 1.5, 0.2);
        assert!(close(&pose.to_base(&pose.to_odom(&p)), &p));
        let ahead = Pose2 {
            x: 1.0,
            y: 0.0,
            yaw: std::f64::consts::FRAC_PI_2,
        }
        .to_odom(&Point::new(2.0, 0.0, 0.0));
        assert!(close(&ahead, &Point::new(1.0, 2.0, 0.0)));
    }

    #[test]
    fn a_kerb_seen_far_stays_as_the_car_drives_up_to_it() {
        let mut m = StepMemory::new(10.0, 12.0, 0.1);
        let kerb = [Point::new(8.0, 0.0, 0.15), Point::new(8.0, 0.5, 0.15)];
        let start = Pose2::default();
        assert_eq!(m.update(0.0, start, &kerb).len(), 2);
        // 6.5 m on, 5 s later, the lidar sees nothing: the kerb is 1.5 m ahead.
        let on = Pose2 { x: 6.5, ..start };
        let held = m.update(5.0, on, &[]);
        assert_eq!(held.len(), 2);
        assert!(close(&held[0], &Point::new(1.5, 0.0, 0.15)));
    }

    #[test]
    fn old_and_far_points_are_forgotten() {
        let mut m = StepMemory::new(10.0, 12.0, 0.1);
        m.update(0.0, Pose2::default(), &[Point::new(5.0, 0.0, 0.2)]);
        assert_eq!(m.update(10.0, Pose2::default(), &[]).len(), 1);
        assert!(m.update(10.5, Pose2::default(), &[]).is_empty());
        m.update(20.0, Pose2::default(), &[Point::new(5.0, 0.0, 0.2)]);
        let behind = Pose2 {
            x: 20.0,
            ..Pose2::default()
        };
        assert!(m.update(21.0, behind, &[]).is_empty());
        // Time going back (a bag looped, the sim restarted): the old ones are gone.
        m.update(30.0, Pose2::default(), &[Point::new(5.0, 0.0, 0.2)]);
        assert!(m.update(1.0, Pose2::default(), &[]).is_empty());
    }

    #[test]
    fn one_point_per_square_the_newest() {
        let mut m = StepMemory::new(10.0, 12.0, 0.1);
        m.update(0.0, Pose2::default(), &[Point::new(5.01, 0.01, 0.1)]);
        let held = m.update(1.0, Pose2::default(), &[Point::new(5.05, 0.05, 0.3)]);
        assert_eq!(held.len(), 1);
        assert!((held[0].z - 0.3).abs() < 1e-12);
    }
}
