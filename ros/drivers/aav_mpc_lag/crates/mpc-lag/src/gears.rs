//! Forward and back: a path cut at its cusps, and a backwards stretch driven by the
//! forward MPC, mirrored. A port of `aav_track/gears.py` (TODO 174, 213).
//!
//! The planner's poses face the way the car faces, so on a backwards stretch they point
//! back along the path: a segment against its pose's yaw is driven backwards.
//!
//! ```text
//!    forward  ─────────►  cusp  ◄───────── backwards  cusp  ─────────► forward
//!    stretch 0                  stretch 1                   stretch 2
//! ```
//!
//! The MPC only ever drives forwards. A backwards stretch is given to it **mirrored**:
//! driving backwards at v with steering d, base_link (the rear axle) moves as a car
//! facing the other way moving forwards at |v| with steering -d. The same bicycle, so
//! the answer is the real car's with both signs turned back.
//!
//! | Real car, backwards | The MPC sees |
//! |---|---|
//! | yaw | yaw + pi |
//! | v < 0 | -v > 0 |
//! | steering d | -d |
//! | poses (x, y, yaw) | (x, y, yaw + pi) |
//!
//! [`Gears`] keeps which stretch the car is on. The next is taken at the cusp: within
//! `reach` m of the stretch's end and nearly still; or standing `wait` s within `near` m
//! of it (a lock held it short). Each stretch ends at its cusp, so the MPC's own speed
//! limit (the stop at the end) brings the car to a stop there: the gear changes only at
//! a standstill, as ego and the spine want.

use std::f64::consts::PI;

use crate::path::{wrap, Path, PathError};
use crate::tracker::Observed;

/// A planner's pose: (x, y, yaw), the yaw the way the car faces.
pub type Pose = (f64, f64, f64);

/// Which way the car drives along a stretch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// The way its poses face.
    Forward,
    /// Against its poses' yaw: in reverse.
    Backward,
}

impl Direction {
    /// +1 forward, -1 backward: a speed or a steering along this gear is this times it.
    #[must_use]
    pub fn sign(self) -> f64 {
        match self {
            Direction::Forward => 1.0,
            Direction::Backward => -1.0,
        }
    }
}

/// Each segment's direction: backward if it runs against its first pose's yaw. A repeated
/// point keeps the direction before it.
#[must_use]
pub fn directions(poses: &[Pose]) -> Vec<Direction> {
    let mut out: Vec<Direction> = Vec::with_capacity(poses.len().saturating_sub(1));
    for w in poses.windows(2) {
        let ((x0, y0, yaw0), (x1, y1, _)) = (w[0], w[1]);
        let (dx, dy) = (x1 - x0, y1 - y0);
        let d = if dx.hypot(dy) < 1e-6 {
            out.last().copied().unwrap_or(Direction::Forward)
        } else if (dy.atan2(dx) - yaw0).cos() >= 0.0 {
            Direction::Forward
        } else {
            Direction::Backward
        };
        out.push(d);
    }
    out
}

/// One direction's run of poses (as planned).
#[derive(Debug, Clone, PartialEq)]
pub struct Stretch {
    poses: Vec<Pose>,
    direction: Direction,
}

impl Stretch {
    /// Its poses, as planned.
    #[must_use]
    pub fn poses(&self) -> &[Pose] {
        &self.poses
    }

    /// Which way it is driven.
    #[must_use]
    pub fn direction(&self) -> Direction {
        self.direction
    }

    /// m from its first pose to its last.
    #[must_use]
    pub fn length(&self) -> f64 {
        self.poses
            .windows(2)
            .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
            .sum()
    }

    /// The path the MPC follows: as planned, or mirrored if backward.
    ///
    /// # Errors
    ///
    /// As [`Path::new`].
    pub fn path(&self) -> Result<Path, PathError> {
        match self.direction {
            Direction::Forward => Path::new(&self.poses),
            Direction::Backward => {
                let turned: Vec<Pose> = self
                    .poses
                    .iter()
                    .map(|&(x, y, yaw)| (x, y, wrap(yaw + PI)))
                    .collect();
                Path::new(&turned)
            }
        }
    }
}

/// The path cut at each change of direction; the cusp's pose ends one stretch and starts
/// the next. Fewer than two poses: one forward stretch.
#[must_use]
pub fn stretches(poses: &[Pose]) -> Vec<Stretch> {
    let dirs = directions(poses);
    let Some(&last) = dirs.last() else {
        return vec![Stretch {
            poses: poses.to_vec(),
            direction: Direction::Forward,
        }];
    };
    let mut out = Vec::new();
    let mut start = 0;
    for (i, w) in dirs.windows(2).enumerate() {
        if w[0] != w[1] {
            out.push(Stretch {
                poses: poses[start..=i + 1].to_vec(),
                direction: w[0],
            });
            start = i + 1;
        }
    }
    out.push(Stretch {
        poses: poses[start..].to_vec(),
        direction: last,
    });
    out
}

/// The car as the MPC sees it on a stretch driven this way: turned round if backward.
#[must_use]
pub fn mirror(seen: &Observed, direction: Direction) -> Observed {
    match direction {
        Direction::Forward => *seen,
        Direction::Backward => Observed {
            yaw: wrap(seen.yaw + PI),
            v: -seen.v,
            steer: -seen.steer,
            ..*seen
        },
    }
}

/// When a cusp counts as reached (gears.py's defaults).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CuspRules {
    /// m from a stretch's end: the cusp is reached.
    pub reach: f64,
    /// m/s: slow enough to change gear: standing (gears.py takes 0.15; mppi_lag 0.05).
    pub still: f64,
    /// m: standing this close for `wait` s will do.
    pub near: f64,
    /// s.
    pub wait: f64,
}

impl Default for CuspRules {
    fn default() -> Self {
        CuspRules {
            reach: 0.3,
            still: 0.05,
            near: 1.5,
            wait: 2.0,
        }
    }
}

/// m/s: slower than this the car counts as standing.
const STANDING: f64 = 0.05;

/// Which stretch the car is on, and when to take the next.
#[derive(Debug, Clone, PartialEq)]
pub struct Gears {
    parts: Vec<Stretch>,
    rules: CuspRules,
    index: usize,
    /// When the car began standing near the cusp (s), while it still stands.
    standing: Option<f64>,
}

impl Gears {
    /// The path cut at its cusps, the car on its first stretch.
    #[must_use]
    pub fn new(poses: &[Pose], rules: CuspRules) -> Self {
        Gears {
            parts: stretches(poses),
            rules,
            index: 0,
            standing: None,
        }
    }

    /// The stretch the car is on.
    #[must_use]
    pub fn current(&self) -> &Stretch {
        &self.parts[self.index]
    }

    /// Every stretch.
    #[must_use]
    pub fn parts(&self) -> &[Stretch] {
        &self.parts
    }

    /// Which stretch (from 0).
    #[must_use]
    pub fn index(&self) -> usize {
        self.index
    }

    /// On the last stretch: only now may the goal be checked.
    #[must_use]
    pub fn is_last(&self) -> bool {
        self.index + 1 == self.parts.len()
    }

    /// The way the car drives now.
    #[must_use]
    pub fn direction(&self) -> Direction {
        self.current().direction
    }

    /// Some stretch is driven backwards.
    #[must_use]
    pub fn any_backward(&self) -> bool {
        self.parts
            .iter()
            .any(|p| p.direction == Direction::Backward)
    }

    /// The whole path's last pose (the goal), as planned.
    #[must_use]
    pub fn goal(&self) -> Option<Pose> {
        self.parts.last().and_then(|p| p.poses.last().copied())
    }

    /// m of the stretches after this one.
    #[must_use]
    pub fn left_after(&self) -> f64 {
        self.parts[self.index + 1..]
            .iter()
            .map(Stretch::length)
            .sum()
    }

    /// Take the next stretch if this one is done: `left` m to its end, the car at `v` m/s,
    /// at `now` s. True when the stretch changed (the MPC needs the new path).
    pub fn update(&mut self, left: f64, v: f64, now: f64) -> bool {
        if self.is_last() {
            return false;
        }
        let r = self.rules;
        if v.abs() <= STANDING && left <= r.near {
            self.standing.get_or_insert(now);
        } else {
            self.standing = None;
        }
        let at_cusp = left <= r.reach && v.abs() <= r.still;
        let waited = self.standing.is_some_and(|since| now - since >= r.wait);
        if !(at_cusp || waited) {
            return false;
        }
        self.index += 1;
        self.standing = None;
        true
    }

    /// Every pose moved by `f` (x, y, yaw), as the held map -> odom moves.
    pub fn move_poses(&mut self, f: impl Fn(f64, f64, f64) -> Pose) {
        for part in &mut self.parts {
            for p in &mut part.poses {
                *p = f(p.0, p.1, p.2);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use Direction::{Backward, Forward};

    /// Forward 4 m along x, then backwards 3 m (the car still facing +x).
    fn forward_then_back() -> Vec<Pose> {
        let mut p: Vec<Pose> = (0..=40).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
        p.extend((1..=30).map(|i| (4.0 - i as f64 * 0.1, 0.0, 0.0)));
        p
    }

    #[test]
    fn a_segment_against_its_pose_is_backwards() {
        assert_eq!(
            directions(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]),
            [Forward, Forward]
        );
        assert_eq!(
            directions(&[(0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (-2.0, 0.0, 0.0)]),
            [Backward, Backward]
        );
        // A repeated point keeps the direction before it.
        assert_eq!(
            directions(&[(0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)]),
            [Backward, Backward]
        );
    }

    #[test]
    fn the_path_is_cut_at_each_cusp() {
        let parts = stretches(&forward_then_back());
        let dirs: Vec<_> = parts.iter().map(Stretch::direction).collect();
        assert_eq!(dirs, [Forward, Backward]);
        // The cusp's pose ends one stretch and starts the next.
        assert_eq!(parts[0].poses().last(), parts[1].poses().first());
        assert!((parts[0].length() - 4.0).abs() < 1e-9);
        assert_eq!(stretches(&[(0.0, 0.0, 0.0)]).len(), 1);
        let three = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.5, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
        ];
        let dirs: Vec<_> = stretches(&three).iter().map(Stretch::direction).collect();
        assert_eq!(dirs, [Forward, Backward, Forward]);
    }

    #[test]
    fn a_forward_path_is_one_stretch_unchanged() {
        let poses: Vec<Pose> = (0..=20).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
        let gears = Gears::new(&poses, CuspRules::default());
        assert!(gears.is_last() && !gears.any_backward());
        assert_eq!(gears.current().path(), Path::new(&poses));
    }

    #[test]
    fn a_backwards_stretch_is_mirrored_for_the_mpc() {
        let back: Vec<Pose> = (0..=20).map(|i| (-(i as f64) * 0.1, 0.0, 0.0)).collect();
        let part = &stretches(&back)[0];
        assert_eq!(part.direction(), Backward);
        let path = part.path().expect("a path");
        // Mirrored: the goal faces the way the path runs (-x), as a forward car would.
        assert!((path.goal_yaw().abs() - PI).abs() < 1e-9);
        let seen = Observed {
            x: 1.0,
            y: 2.0,
            yaw: 0.0,
            v: -0.4,
            steer: 0.2,
        };
        let m = mirror(&seen, Backward);
        assert_eq!((m.x, m.y, m.v, m.steer), (1.0, 2.0, 0.4, -0.2));
        assert!((m.yaw + PI).abs() < 1e-12);
        assert_eq!(mirror(&seen, Forward), seen);
    }

    #[test]
    fn the_next_stretch_waits_for_the_cusp() {
        let mut gears = Gears::new(&forward_then_back(), CuspRules::default());
        assert!((gears.left_after() - 3.0).abs() < 1e-9);
        assert!(!gears.update(2.0, 1.0, 0.0));
        assert!(!gears.update(0.2, 0.5, 0.1)); // too fast to change gear
        assert!(!gears.update(0.2, 0.1, 0.15)); // still rolling: gears.py's 0.15 m/s would
        assert!(gears.update(0.2, 0.04, 0.2));
        assert_eq!(gears.direction(), Backward);
        assert!(gears.is_last() && gears.left_after() == 0.0);
        assert!(!gears.update(0.0, 0.0, 0.3)); // the last: the goal decides
    }

    #[test]
    fn held_short_of_a_cusp_it_changes_gear_after_standing() {
        let mut gears = Gears::new(&forward_then_back(), CuspRules::default());
        assert!(!gears.update(1.0, 0.0, 0.0));
        assert!(!gears.update(1.0, 0.0, 1.9));
        assert!(gears.update(1.0, 0.0, 2.0));
        // Too far short, it never does: the progress check gives up instead.
        let mut far = Gears::new(&forward_then_back(), CuspRules::default());
        assert!(!(0..20).any(|t| far.update(3.0, 0.0, f64::from(t))));
    }

    #[test]
    fn moving_the_poses_moves_the_goal() {
        let mut gears = Gears::new(&forward_then_back(), CuspRules::default());
        gears.move_poses(|x, y, yaw| (x + 1.0, y, yaw));
        assert_eq!(gears.goal(), Some((2.0, 0.0, 0.0)));
    }
}
