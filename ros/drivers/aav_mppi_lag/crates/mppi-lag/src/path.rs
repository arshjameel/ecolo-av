//! The planner's path, and the piece of it MPPI looks at each tick: Nav2's PathHandler.
//!
//! Each tick: find the closest pose within `max_search` m of path from where the car was
//! last, drop everything before it (Nav2 prunes the same), and hand the critics the poses
//! from there to `prune_distance` m along (Nav2's "transformed plan"). The goal is the
//! path's last pose.
//!
//! Reversing (Nav2's `enforce_path_inversion`, [`GlobalPath::with_inversion`]): a path
//! that changes direction (a cusp: two steps that point more than 90 degrees apart) is
//! followed one stretch at a time. The search and the window stop at the cusp; once the
//! car is within the tolerances of it, the next stretch starts. The planner's poses face
//! the way the car faces, so on a reversing stretch they point back along the path.

use crate::costmap::Costmap;
use crate::model::{wrap, Pose};

/// How close to a cusp the car must come before the path's next stretch starts (Nav2's
/// `inversion_xy_tolerance`, `inversion_yaw_tolerance`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inversion {
    /// m.
    pub xy: f64,
    /// rad.
    pub yaw: f64,
}

impl Default for Inversion {
    /// Our nav2.yaml's (Nav2's 0.2 m is under the GPS's error).
    fn default() -> Self {
        Self { xy: 0.5, yaw: 0.6 }
    }
}

/// A path in the odom frame, and how far along it the car has got.
#[derive(Debug, Clone, PartialEq)]
pub struct GlobalPath {
    poses: Vec<Pose>,
    start: usize,
    /// One past the last pose of the stretch being followed (`poses.len()`: no cusp ahead).
    end: usize,
    inversion: Option<Inversion>,
}

/// One past the pose where the path first changes direction after `from`, or
/// `poses.len()` if it never does (Nav2's findFirstPathInversion).
fn first_cusp(poses: &[Pose], from: usize) -> usize {
    (from + 1..poses.len().saturating_sub(1))
        .find(|&i| {
            let (a, b, c) = (poses[i - 1], poses[i], poses[i + 1]);
            (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y) < 0.0
        })
        .map_or(poses.len(), |i| i + 1)
}

/// Where the car is beside the path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Foot {
    /// m, + left of the path.
    pub cross_track: f64,
    /// rad: the car's heading minus the path's.
    pub heading_error: f64,
    /// m along the path to its end.
    pub left: f64,
}

/// The index of the first pose after `distance` m along `poses` from `from`; `poses.len()`
/// if the path ends first (Nav2's first_after_integrated_distance).
fn first_after(poses: &[Pose], from: usize, distance: f64) -> usize {
    let mut along = 0.0;
    for i in from + 1..poses.len() {
        along += poses[i].distance(&poses[i - 1]);
        if along > distance {
            return i;
        }
    }
    poses.len()
}

impl GlobalPath {
    /// A path of `poses` (odom frame). `None` if it is empty or not all numbers.
    pub fn new(poses: Vec<Pose>) -> Option<Self> {
        let finite = poses
            .iter()
            .all(|p| p.x.is_finite() && p.y.is_finite() && p.yaw.is_finite());
        (finite && !poses.is_empty()).then_some(Self {
            end: poses.len(),
            poses,
            start: 0,
            inversion: None,
        })
    }

    /// The same path, followed one stretch at a time where it changes direction (Nav2's
    /// `enforce_path_inversion`).
    pub fn with_inversion(mut self, tolerance: Inversion) -> Self {
        self.inversion = Some(tolerance);
        self.end = first_cusp(&self.poses, self.start);
        self
    }

    /// The last pose: the goal.
    pub fn goal(&self) -> Pose {
        self.poses[self.poses.len() - 1]
    }

    /// The end of the stretch being followed: the next cusp, or the goal.
    pub fn stretch_end(&self) -> Pose {
        self.poses[self.end - 1]
    }

    /// True while a change of direction lies ahead.
    pub fn cusp_ahead(&self) -> bool {
        self.end < self.poses.len()
    }

    /// True if a cusp lies ahead and `pose` is within `tolerance` of it.
    pub fn near_cusp(&self, pose: &Pose, tolerance: Inversion) -> bool {
        let cusp = self.stretch_end();
        self.cusp_ahead()
            && pose.distance(&cusp) <= tolerance.xy
            && wrap(cusp.yaw - pose.yaw).abs() <= tolerance.yaw
    }

    /// Start the stretch after the next cusp (nothing if no cusp lies ahead).
    pub fn next_stretch(&mut self) {
        if self.cusp_ahead() {
            self.start = self.end;
            self.end = first_cusp(&self.poses, self.start);
        }
    }

    /// Every pose not yet pruned.
    pub fn remaining(&self) -> &[Pose] {
        &self.poses[self.start..]
    }

    /// Prune to the closest pose to `pose` (searching `max_search` m ahead), and return the
    /// next `prune_distance` m of path; with a costmap, only the part inside it. Neither
    /// passes the next cusp. Within the tolerances of that cusp, the next stretch starts
    /// (from the next tick, as Nav2's).
    pub fn window(
        &mut self,
        pose: &Pose,
        max_search: f64,
        prune_distance: f64,
        costmap: Option<&Costmap>,
    ) -> &[Pose] {
        let stretch = &self.poses[..self.end];
        let bound = first_after(stretch, self.start, max_search);
        let closest = (self.start..bound.max(self.start + 1))
            .min_by(|&a, &b| {
                pose.distance(&self.poses[a])
                    .total_cmp(&pose.distance(&self.poses[b]))
            })
            .unwrap_or(self.start);
        let end = first_after(stretch, closest, prune_distance);
        self.start = closest;
        if let Some(tolerance) = self.inversion {
            if self.near_cusp(pose, tolerance) {
                self.next_stretch();
            }
        }
        let mut window = &self.poses[closest..end];
        if let Some(map) = costmap {
            let inside = window
                .iter()
                .position(|p| map.cell(p.x, p.y).is_none())
                .unwrap_or(window.len());
            window = &window[..inside];
        }
        window
    }

    /// Where `pose` is beside the path, from the pruned start on.
    pub fn foot(&self, pose: &Pose) -> Foot {
        let poses = self.remaining();
        if poses.len() < 2 {
            let end = poses[0];
            return Foot {
                cross_track: 0.0,
                heading_error: wrap(pose.yaw - end.yaw),
                left: pose.distance(&end),
            };
        }
        let mut best = (f64::INFINITY, 0.0, 0.0, 0.0); // (distance², cross, heading, along)
        let mut along = 0.0;
        let mut total = 0.0;
        for pair in poses.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let length = dx.hypot(dy);
            if length > 1e-9 {
                let t = (((pose.x - a.x) * dx + (pose.y - a.y) * dy) / (length * length))
                    .clamp(0.0, 1.0);
                let (fx, fy) = (a.x + t * dx, a.y + t * dy);
                let d2 = (pose.x - fx).powi(2) + (pose.y - fy).powi(2);
                if d2 < best.0 {
                    let heading = dy.atan2(dx);
                    let cross = (dx * (pose.y - a.y) - dy * (pose.x - a.x)) / length;
                    best = (d2, cross, wrap(pose.yaw - heading), along + t * length);
                }
            }
            along += length;
            total = along;
        }
        Foot {
            cross_track: best.1,
            heading_error: best.2,
            left: (total - best.3).max(0.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(n: usize, step: f64) -> Vec<Pose> {
        (0..n)
            .map(|i| Pose::new(i as f64 * step, 0.0, 0.0))
            .collect()
    }

    #[test]
    fn empty_or_nan_paths_are_refused() {
        assert!(GlobalPath::new(vec![]).is_none());
        assert!(GlobalPath::new(vec![Pose::new(f64::NAN, 0.0, 0.0)]).is_none());
    }

    #[test]
    fn window_prunes_behind_and_stops_after_prune_distance() {
        let mut path = GlobalPath::new(line(201, 0.1)).expect("a path");
        let window = path
            .window(&Pose::new(5.02, 0.3, 0.0), 10.0, 3.0, None)
            .to_vec();
        assert!((window[0].x - 5.0).abs() < 1e-9);
        // First pose after 3 m along is excluded: 5.0 .. 8.0 inclusive.
        assert!((window[window.len() - 1].x - 8.0).abs() < 1e-9);
        assert_eq!(path.remaining()[0].x, window[0].x);
    }

    #[test]
    fn search_is_limited_ahead() {
        // The car is near the far end, but the search only looks 1 m ahead of the start.
        let mut path = GlobalPath::new(line(201, 0.1)).expect("a path");
        let window = path.window(&Pose::new(19.0, 0.0, 0.0), 1.0, 3.0, None);
        assert!((window[0].x - 1.0).abs() < 1e-9, "{:?}", window[0]);
    }

    /// 3 m east, a cusp at (3, 0), then 2 m back west, drifting 1 m left: the reversing
    /// stretch faces east (the car backs up).
    fn three_point() -> Vec<Pose> {
        let mut poses = line(31, 0.1);
        poses.extend(
            (1..=20).map(|i| Pose::new(3.0 - f64::from(i) * 0.1, 0.05 * f64::from(i), 0.0)),
        );
        poses
    }

    #[test]
    fn first_cusp_is_where_the_path_turns_back() {
        let poses = three_point();
        assert_eq!(first_cusp(&poses, 0), 31, "one past (3, 0)");
        assert_eq!(first_cusp(&poses, 31), poses.len());
        assert_eq!(first_cusp(&line(50, 0.1), 0), 50);
    }

    #[test]
    fn the_window_stops_at_the_cusp_until_the_car_is_there() {
        let mut path = GlobalPath::new(three_point())
            .expect("a path")
            .with_inversion(Inversion::default());
        assert!(path.cusp_ahead());
        assert_eq!(path.stretch_end(), Pose::new(3.0, 0.0, 0.0));
        let window = path
            .window(&Pose::new(1.0, 0.0, 0.0), 10.0, 5.0, None)
            .to_vec();
        assert!(
            (window[window.len() - 1].x - 3.0).abs() < 1e-9,
            "not past the cusp"
        );
        // Near the cusp but turned too far: not yet.
        path.window(&Pose::new(2.8, 0.0, 1.0), 10.0, 5.0, None);
        assert!(path.cusp_ahead());
        // There: the next stretch, backwards, from the next tick on.
        path.window(&Pose::new(2.8, 0.0, 0.1), 10.0, 5.0, None);
        assert!(!path.cusp_ahead());
        assert_eq!(path.stretch_end(), path.goal());
        let back = path.window(&Pose::new(2.9, 0.05, 0.0), 10.0, 5.0, None);
        assert!(back[0].x < 3.0 && back[0].y > 0.0, "{:?}", back[0]);
        // Without inversion, the whole path is one stretch (as before).
        let plain = GlobalPath::new(three_point()).expect("a path");
        assert!(!plain.cusp_ahead());
    }

    #[test]
    fn near_cusp_and_next_stretch() {
        let mut path = GlobalPath::new(three_point())
            .expect("a path")
            .with_inversion(Inversion::default());
        let wide = Inversion { xy: 1.0, yaw: 0.75 };
        let beside = Pose::new(3.0, 0.7, 0.2);
        assert!(!path.near_cusp(&beside, Inversion::default()), "0.7 m off");
        assert!(path.near_cusp(&beside, wide));
        assert!(!path.near_cusp(&Pose::new(3.0, 0.7, 1.0), wide), "turned");
        path.next_stretch();
        assert!(!path.cusp_ahead());
        assert!(!path.near_cusp(&beside, wide), "no cusp left");
        path.next_stretch();
        assert_eq!(
            path.stretch_end(),
            path.goal(),
            "nothing past the last stretch"
        );
    }

    #[test]
    fn foot_measures_cross_track_left_positive() {
        let path = GlobalPath::new(line(101, 0.1)).expect("a path");
        let foot = path.foot(&Pose::new(4.0, 0.5, 0.1));
        assert!((foot.cross_track - 0.5).abs() < 1e-9);
        assert!((foot.heading_error - 0.1).abs() < 1e-9);
        assert!((foot.left - 6.0).abs() < 1e-9);
        assert!((path.foot(&Pose::new(4.0, -0.2, 0.0)).cross_track + 0.2).abs() < 1e-9);
    }
}
