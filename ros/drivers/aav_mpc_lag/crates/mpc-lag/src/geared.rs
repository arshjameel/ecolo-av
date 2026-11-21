//! The MPC forward and back: a whole path, its stretches one at a time (TODO 213).
//! What `aav_track/track_node.py` does around a tracker since TODO 174, here without ROS
//! so the node only moves messages.
//!
//! ```text
//! path ─▶ Gears (cut at cusps) ─▶ this stretch, mirrored if backward ─▶ MpcLag
//! car  ─▶ mirror ──────────────────────────────────────────────────────▶ MpcLag ─▶ (steer, v)
//!                                                     signs turned back ◀─┘
//! ```
//!
//! | Step | What |
//! |---|---|
//! | [`Geared::follow`] | the path cut at its cusps; one that reverses is refused when the car may not |
//! | [`Geared::advance`] | at the cusp (stopped there by the stretch's own end), the next stretch; the MPC turned round |
//! | [`Geared::step`] | the MPC on the car as it sees it; the command with the real signs, backwards at `reverse_speed` at most; held at 0 within `reach` of a cusp, so the car stands there for the change of gear |
//!
//! A path with no cusp is one forward stretch: the MPC gets it as it is, and every
//! command is the same as without this.

use crate::gears::{mirror, CuspRules, Direction, Gears, Pose};
use crate::path::PathError;
use crate::tracker::{Command, MpcLag, Observed, TrackError};

/// Why a path can't be followed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FollowError {
    /// The path itself: empty, or not numbers.
    Path(PathError),
    /// The path reverses, and the car may not (`max_reverse_speed` 0).
    MayNotReverse,
}

impl std::fmt::Display for FollowError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FollowError::Path(e) => e.fmt(f),
            FollowError::MayNotReverse => {
                f.write_str("the path reverses, and this car may not (reversing:=false)")
            }
        }
    }
}

impl std::error::Error for FollowError {}

impl From<PathError> for FollowError {
    fn from(e: PathError) -> Self {
        FollowError::Path(e)
    }
}

/// The MPC with gears: a path that changes direction, one stretch at a time.
#[derive(Debug, Clone)]
pub struct Geared {
    mpc: MpcLag,
    gears: Option<Gears>,
    rules: CuspRules,
    /// m/s: the MPC's cruise forwards (its settings' when made).
    forward: f64,
    /// m/s: the most backwards (0: never).
    reverse: f64,
}

impl Geared {
    /// The MPC, allowed to back up at `reverse_speed` m/s at most (0: forward only).
    #[must_use]
    pub fn new(mpc: MpcLag, reverse_speed: f64, rules: CuspRules) -> Self {
        Geared {
            forward: mpc.settings().cruise,
            mpc,
            gears: None,
            rules,
            reverse: reverse_speed.max(0.0),
        }
    }

    /// The MPC inside.
    #[must_use]
    pub fn mpc(&self) -> &MpcLag {
        &self.mpc
    }

    /// The stretches, once a path is given.
    #[must_use]
    pub fn gears(&self) -> Option<&Gears> {
        self.gears.as_ref()
    }

    /// Forget the MPC's estimate and plan ([`MpcLag::forget`]).
    pub fn forget(&mut self) {
        self.mpc.forget();
    }

    /// Follow a new path (the planner's poses), from its first stretch. The estimate
    /// carries on.
    ///
    /// # Errors
    ///
    /// [`FollowError::Path`] for a path that can't be made; [`FollowError::MayNotReverse`]
    /// for one that reverses with `reverse_speed` 0.
    pub fn follow(&mut self, poses: &[Pose]) -> Result<(), FollowError> {
        if poses
            .iter()
            .any(|&(x, y, yaw)| !(x.is_finite() && y.is_finite() && yaw.is_finite()))
        {
            return Err(FollowError::Path(PathError::NotANumber));
        }
        let gears = Gears::new(poses, self.rules);
        if gears.any_backward() && self.reverse <= 0.0 {
            return Err(FollowError::MayNotReverse);
        }
        let path = gears.current().path()?;
        let turned = self
            .gears
            .as_ref()
            .is_some_and(|g| g.direction() != gears.direction());
        if turned {
            // The newest path starts the other way from the one running: the estimate too.
            self.mpc.turn_round();
        }
        self.set_cruise(gears.direction());
        self.mpc.reset(path);
        self.gears = Some(gears);
        Ok(())
    }

    fn set_cruise(&mut self, direction: Direction) {
        let cruise = match direction {
            Direction::Forward => self.forward,
            Direction::Backward => self.reverse.min(self.forward),
        };
        self.mpc.set_cruise(cruise);
    }

    /// Take the next stretch if the car is at the cusp: at (x, y), `v` m/s (signed), at
    /// `now` s. True when it changed (a new stretch: the progress check starts again).
    pub fn advance(&mut self, x: f64, y: f64, v: f64, now: f64) -> bool {
        let Some(left) = self.mpc.left_from(x, y) else {
            return false;
        };
        let Some(gears) = self.gears.as_mut() else {
            return false;
        };
        let before = gears.direction();
        if !gears.update(left, v, now) {
            return false;
        }
        let after = gears.direction();
        match gears.current().path() {
            Ok(path) => {
                if after != before {
                    self.mpc.turn_round();
                }
                self.set_cruise(after);
                self.mpc.reset(path);
                true
            }
            // Can't happen: every stretch has two poses or more, all numbers.
            Err(_) => false,
        }
    }

    /// On the last stretch (or none): only then is the goal checked.
    #[must_use]
    pub fn on_last(&self) -> bool {
        self.gears.as_ref().is_none_or(Gears::is_last)
    }

    /// The goal: the whole path's last pose.
    #[must_use]
    pub fn goal(&self) -> Option<Pose> {
        self.gears.as_ref().and_then(Gears::goal)
    }

    /// The way the car drives now.
    #[must_use]
    pub fn direction(&self) -> Direction {
        self.gears
            .as_ref()
            .map_or(Direction::Forward, Gears::direction)
    }

    /// m from (x, y) to the goal: the rest of this stretch, and every one after.
    #[must_use]
    pub fn left(&self, x: f64, y: f64) -> f64 {
        let here = self.mpc.left_from(x, y).unwrap_or(0.0).max(0.0);
        here + self.gears.as_ref().map_or(0.0, Gears::left_after)
    }

    /// Move the path, every stretch (the held map -> odom moved), keeping the place on it.
    pub fn move_path(&mut self, f: impl Fn(f64, f64, f64) -> Pose) {
        if let Some(g) = self.gears.as_mut() {
            g.move_poses(&f);
        }
        // A rigid move: it commutes with the mirroring (yaw + pi).
        self.mpc.move_path(f);
    }

    /// One tick: the steering and set speed to send now, with the real car's signs (the
    /// speed negative backwards). `seen` is the car as measured; `dt` s since the last tick.
    /// The report's foot is on the stretch as the MPC sees it.
    ///
    /// # Errors
    ///
    /// As [`MpcLag::step`].
    pub fn step(&mut self, seen: &Observed, dt: f64) -> Result<Command, TrackError> {
        let direction = self.direction();
        // At a cusp not yet taken: stand for the change of gear.
        let at_cusp = !self.on_last()
            && self
                .mpc
                .left_from(seen.x, seen.y)
                .is_some_and(|left| left <= self.rules.reach);
        self.mpc.hold(at_cusp);
        let cmd = self.mpc.step(&mirror(seen, direction), dt)?;
        let sign = direction.sign();
        Ok(Command {
            steer: sign * cmd.steer,
            speed: sign * cmd.speed,
            report: cmd.report,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tracker::{Limits, Settings};

    const LIMITS: Limits = Limits {
        wheelbase: 1.57,
        max_steer: 0.5,
        max_steer_rate: 0.6,
    };

    fn geared(reverse: f64) -> Geared {
        let mpc = MpcLag::new(LIMITS, Default::default(), Settings::default()).expect("an MPC");
        Geared::new(mpc, reverse, CuspRules::default())
    }

    fn forward_then_back() -> Vec<Pose> {
        let mut p: Vec<Pose> = (0..=40).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
        p.extend((1..=30).map(|i| (4.0 - i as f64 * 0.1, 0.0, 0.0)));
        p
    }

    #[test]
    fn a_path_that_reverses_is_refused_forward_only() {
        let mut g = geared(0.0);
        assert_eq!(
            g.follow(&forward_then_back()),
            Err(FollowError::MayNotReverse)
        );
        assert_eq!(g.follow(&[]), Err(FollowError::Path(PathError::Empty)));
        let nan = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.5, f64::NAN, 0.0)];
        assert_eq!(
            g.follow(&nan),
            Err(FollowError::Path(PathError::NotANumber))
        );
        let mut g = geared(0.5);
        assert_eq!(g.follow(&forward_then_back()), Ok(()));
        assert!(!g.on_last());
        assert!((g.left(0.0, 0.0) - 7.0).abs() < 1e-9);
    }

    #[test]
    fn backwards_it_asks_a_negative_speed_within_the_reverse_limit() {
        let mut g = geared(0.5);
        let back: Vec<Pose> = (0..=40).map(|i| (-(i as f64) * 0.1, 0.0, 0.0)).collect();
        g.follow(&back).expect("a path");
        assert_eq!(g.direction(), Direction::Backward);
        let mut seen = Observed::default();
        for _ in 0..40 {
            let cmd = g.step(&seen, 0.05).expect("a command");
            assert!((-0.5 - 1e-9..=0.0).contains(&cmd.speed), "{}", cmd.speed);
            assert!(cmd.steer.abs() < 1e-3);
            seen.v = cmd.speed;
            seen.x += cmd.speed * 0.05;
        }
        assert!(seen.x < -0.3, "backed up: {}", seen.x);
    }

    #[test]
    fn a_forward_path_gives_the_mpcs_own_commands() {
        let line: Vec<Pose> = (0..=100).map(|i| (i as f64 * 0.1, 0.0, 0.0)).collect();
        let mut g = geared(0.5);
        g.follow(&line).expect("a path");
        let mut mpc = MpcLag::new(LIMITS, Default::default(), Settings::default()).expect("MPC");
        mpc.reset(crate::Path::new(&line).expect("a path"));
        let seen = Observed {
            y: 0.3,
            ..Observed::default()
        };
        for _ in 0..5 {
            let a = g.step(&seen, 0.05).expect("a command");
            let b = mpc.step(&seen, 0.05).expect("a command");
            assert_eq!((a.steer, a.speed), (b.steer, b.speed));
        }
    }
}
