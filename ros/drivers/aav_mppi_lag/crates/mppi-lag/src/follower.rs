//! Following one path: Nav2's goal and progress checks, the state estimate, and MPPI.
//!
//! Everything the node does each tick, without ROS:
//!
//! ```text
//! observe(odometry, measured actuators) ─> estimate ─┐   (ego's output + the car's state)
//! follow(path) ──────────────────────────────────────┼─> tick() ─> Drive(command) or Done
//!                                                    └── the estimate steps on with the
//!                                                        command sent: the next tick's
//!                                                        lags and dead times start there
//! ```
//!
//! | Check | Nav2's plugin | Rule |
//! |---|---|---|
//! | [`GoalCheck`] | `SimpleGoalChecker` (stateful) | within xy_goal_tolerance of the end, then the heading |
//! | [`ProgressCheck`] | `PoseProgressChecker` | moved or turned enough in time, else stuck; starts again at each new stretch (a cusp) |
//! | cusp reached | `enforce_path_inversion` | within the inversion tolerances; or (ours) stopped within the goal's |

use crate::costmap::Costmap;
use crate::critics::Critic;
use crate::ego::{Driven, DrivenState, Ego};
use crate::lane::Lane;
use crate::model::{wrap, CarModel, Command, Observation, Pose};
use crate::optimizer::{NoValidControl, Optimizer, Plan, Scene, Settings};
use crate::path::{Foot, GlobalPath, Inversion};

/// m/s: slower than this, the car counts as stopped (at a cusp).
const STOPPED: f64 = 0.05;

/// Nav2's SimpleGoalChecker, stateful: once within xy, only the heading is left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GoalCheck {
    /// m.
    pub xy: f64,
    /// rad.
    pub yaw: f64,
    reached_xy: bool,
}

impl GoalCheck {
    /// A check with these tolerances.
    #[must_use]
    pub fn new(xy: f64, yaw: f64) -> Self {
        Self {
            xy,
            yaw,
            reached_xy: false,
        }
    }

    /// Start again (a new goal).
    pub fn reset(&mut self) {
        self.reached_xy = false;
    }

    /// True once `pose` is at `goal`.
    pub fn done(&mut self, pose: &Pose, goal: &Pose) -> bool {
        if !self.reached_xy {
            if pose.distance(goal) > self.xy {
                return false;
            }
            self.reached_xy = true;
        }
        wrap(goal.yaw - pose.yaw).abs() <= self.yaw
    }
}

/// Nav2's PoseProgressChecker: moved `radius` m or turned `angle` rad within `time` s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProgressCheck {
    /// m.
    pub radius: f64,
    /// rad.
    pub angle: f64,
    /// s.
    pub time: f64,
    base: Option<(Pose, f64)>,
}

impl ProgressCheck {
    /// A check with these limits.
    #[must_use]
    pub fn new(radius: f64, angle: f64, time: f64) -> Self {
        Self {
            radius,
            angle,
            time,
            base: None,
        }
    }

    /// Start again.
    pub fn reset(&mut self) {
        self.base = None;
    }

    /// True once the car has neither moved nor turned enough for `time` s.
    pub fn stuck(&mut self, pose: &Pose, now: f64) -> bool {
        let Some((base, since)) = self.base else {
            self.base = Some((*pose, now));
            return false;
        };
        if pose.distance(&base) >= self.radius || wrap(pose.yaw - base.yaw).abs() >= self.angle {
            self.base = Some((*pose, now));
            return false;
        }
        now - since > self.time
    }
}

/// How a path ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// At the goal, heading within tolerance.
    Succeeded,
    /// No progress in time (Nav2: FAILED_TO_MAKE_PROGRESS).
    Stuck,
    /// Every future collides (Nav2: NO_VALID_CONTROL).
    NoValidControl,
    /// No part of the path ahead is inside the costmap (Nav2: INVALID_PATH).
    PathOutsideCostmap,
    /// The node hasn't seen the car yet.
    NoEstimate,
}

/// One tick's command and what went into it.
#[derive(Debug, Clone, PartialEq)]
pub struct Report {
    /// Send this.
    pub command: Command,
    /// Where the car is beside the path.
    pub foot: Foot,
    /// MPPI's tick.
    pub plan: Plan,
    /// The model's steering estimate before this tick (rad).
    pub steering: f64,
    /// The model's speed estimate before this tick (m/s).
    pub speed: f64,
}

/// What a tick decided.
#[derive(Debug, Clone, PartialEq)]
pub enum Tick {
    /// Keep going: send the command.
    Drive(Box<Report>),
    /// The path is over: send a stop.
    Done(Outcome),
}

/// The follower's own settings (the rest is MPPI's [`Settings`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowerSettings {
    /// m: how far ahead along the path to look for the car (Nav2's
    /// max_robot_pose_search_dist; default half the local costmap, 10 m).
    pub max_search: f64,
    /// m of path the critics see (prune_distance).
    pub prune_distance: f64,
    /// m and rad: the goal check.
    pub xy_goal_tolerance: f64,
    /// rad.
    pub yaw_goal_tolerance: f64,
    /// m, rad, s: the progress check.
    pub progress_radius: f64,
    /// rad.
    pub progress_angle: f64,
    /// s.
    pub progress_time: f64,
    /// Unknown costmap cells are drivable.
    pub tracking_unknown: bool,
    /// Follow a path that changes direction one stretch at a time, stopping at each cusp
    /// (Nav2's `enforce_path_inversion`; on when the car may reverse). `None`: one stretch.
    pub inversion: Option<Inversion>,
    /// m: stopped this close to a cusp for `cusp_wait` s, the stretch is done (ours,
    /// TODO 174: a lock may hold the car short of it).
    pub cusp_reach: f64,
    /// s: how long the car must stand there first.
    pub cusp_wait: f64,
}

impl Default for FollowerSettings {
    /// Our nav2.yaml's.
    fn default() -> Self {
        Self {
            max_search: 10.0,
            prune_distance: 3.0,
            xy_goal_tolerance: 1.0,
            yaw_goal_tolerance: 0.75,
            progress_radius: 0.5,
            progress_angle: 0.5,
            progress_time: 10.0,
            tracking_unknown: false,
            inversion: None,
            cusp_reach: 2.0,
            cusp_wait: 3.0,
        }
    }
}

/// Follows one path at a time with MPPI over ego and the car model `M`.
#[derive(Debug, Clone)]
pub struct Follower<M: CarModel> {
    optimizer: Optimizer<Driven<M>>,
    settings: FollowerSettings,
    footprint: Vec<(f64, f64)>,
    path: Option<GlobalPath>,
    /// The road ahead's lane, in odom, while the node has a fresh one.
    lane: Option<Lane>,
    estimate: Option<DrivenState<M::State>>,
    last: Command,
    goal: GoalCheck,
    progress: ProgressCheck,
    /// The stretch followed last tick ([`GlobalPath::stretch`]; `None`: a new path).
    stretch: Option<usize>,
    /// When the car last stood still with a cusp ahead (s), while it still stands.
    standing: Option<f64>,
}

impl<M: CarModel + Clone> Follower<M> {
    /// A follower: MPPI over ego and `model`, these critics, and the car's padded outline.
    pub fn new(
        model: M,
        ego: Ego,
        mppi: Settings,
        critics: Vec<Critic>,
        settings: FollowerSettings,
        footprint: Vec<(f64, f64)>,
    ) -> Self {
        Self {
            optimizer: Optimizer::new(Driven { ego, car: model }, mppi, critics),
            footprint,
            path: None,
            lane: None,
            estimate: None,
            last: Command::default(),
            goal: GoalCheck::new(settings.xy_goal_tolerance, settings.yaw_goal_tolerance),
            progress: ProgressCheck::new(
                settings.progress_radius,
                settings.progress_angle,
                settings.progress_time,
            ),
            stretch: None,
            standing: None,
            settings,
        }
    }

    /// The car model.
    pub fn model(&self) -> &M {
        &self.optimizer.model().car
    }

    /// True if the model knows the actuators' lag.
    pub fn lag_aware(&self) -> bool {
        self.optimizer.model().lag_aware()
    }

    /// The current path, if one runs.
    pub fn path(&self) -> Option<&GlobalPath> {
        self.path.as_ref()
    }

    /// The state estimate, once the car has been seen.
    pub fn estimate(&self) -> Option<&DrivenState<M::State>> {
        self.estimate.as_ref()
    }

    /// Follow `path`. `replacing`: a replan of the path already running, so the checks
    /// and MPPI's plan carry on (Nav2 keeps both across replans).
    pub fn follow(&mut self, path: GlobalPath, replacing: bool) {
        if !replacing || self.path.is_none() {
            // Taking over: ego's output starts from the car as it is.
            if let Some(state) = self.estimate.as_mut() {
                self.optimizer.model().resync(state);
                self.last = state.ego;
            }
            self.goal.reset();
            self.progress.reset();
            self.optimizer.reset(self.last);
        }
        self.stretch = None;
        self.standing = None;
        self.path = Some(match self.settings.inversion {
            Some(tolerance) => path.with_inversion(tolerance),
            None => path,
        });
    }

    /// Move the running path by `f` (the held map -> odom moved); its place on it, the
    /// checks and MPPI's plan carry on.
    pub fn move_path(&mut self, f: impl Fn(&Pose) -> Pose) {
        if let Some(path) = self.path.as_mut() {
            path.move_by(f);
        }
    }

    /// The lane to keep to (the road ahead's, in odom), or `None`: none known, or stale.
    /// Only `LaneCritic` reads it.
    pub fn set_lane(&mut self, lane: Option<Lane>) {
        self.lane = lane;
    }

    /// The behaviour planner holds the car for something moving near: standing is not a
    /// lack of progress, so the progress check starts again (Ben, 16 Nov: "we should
    /// likely just stop").
    pub fn held(&mut self) {
        self.progress.reset();
    }

    /// The lane it keeps to, if any.
    pub fn lane(&self) -> Option<&Lane> {
        self.lane.as_ref()
    }

    /// Stop following (the path ended or was cancelled). The next command is a stop.
    pub fn stop(&mut self) {
        self.path = None;
        self.last = Command::new(0.0, self.last.steer);
    }

    /// Take in what was measured.
    pub fn observe(&mut self, seen: &Observation) {
        let model = self.optimizer.model();
        match self.estimate.as_mut() {
            Some(state) => model.observe(state, seen),
            None => {
                let mut state = model.at_rest(seen.pose, seen.speed);
                model.observe(&mut state, seen);
                self.estimate = Some(state);
            }
        }
    }

    /// One tick at time `now` (s): check the goal and progress, then plan. Without a path,
    /// the estimate steps on with the car held (ego's stop).
    pub fn tick(&mut self, now: f64, costmap: Option<&Costmap>) -> Option<Tick> {
        let dt = self.optimizer.settings().model_dt;
        let Some(state) = self.estimate.clone() else {
            return self
                .path
                .is_some()
                .then_some(Tick::Done(Outcome::NoEstimate));
        };
        let model = self.optimizer.model().clone();
        let Some(path) = self.path.as_mut() else {
            if let Some(s) = self.estimate.as_mut() {
                model.step(s, self.last, dt);
            }
            return None;
        };
        let pose = model.pose(&state);
        // Not while a cusp lies ahead: a three-point turn may pass near its own goal.
        if !path.cusp_ahead() && self.goal.done(&pose, &path.goal()) {
            return Some(Tick::Done(Outcome::Succeeded));
        }
        if self.progress.stuck(&pose, now) {
            return Some(Tick::Done(Outcome::Stuck));
        }
        let s = self.settings;
        // Stopped short of a cusp, but within the goal's tolerances of it: the stretch is
        // done, as the path would be at its goal. MPPI slows to a stop near a stretch's end
        // as near the goal, and may stand just outside the inversion tolerances, where
        // nothing would move it on (ours; Nav2 waits for the inversion tolerances).
        let at_goal = Inversion {
            xy: s.xy_goal_tolerance,
            yaw: s.yaw_goal_tolerance,
        };
        // Held short of a cusp: standing `cusp_wait` s within `cusp_reach` m of it, the
        // stretch is done too. A lock holds the car there, not the follower: the stop zone
        // keeps the bumper 1 m from a wall the plan's cusp comes closer to (a narrow street,
        // TODO 174), and nothing else would move it on.
        let stopped = model.speed(&state).abs() < STOPPED;
        self.standing = (stopped && path.cusp_ahead()).then(|| self.standing.unwrap_or(now));
        let held = Inversion {
            xy: s.cusp_reach,
            yaw: s.yaw_goal_tolerance,
        };
        let waited = self
            .standing
            .is_some_and(|since| now - since >= s.cusp_wait);
        if s.inversion.is_some()
            && stopped
            && (path.near_cusp(&pose, at_goal) || (waited && path.near_cusp(&pose, held)))
        {
            path.next_stretch();
            self.standing = None;
        }
        // The goal the critics see: the next cusp while one lies ahead, so the car stops
        // there as it would at the goal (Nav2's critics see only the final goal, and stop
        // at a cusp only because the path ends there).
        let goal = path.stretch_end();
        let window = path
            .window(&pose, s.max_search, s.prune_distance, costmap)
            .to_vec();
        // A new stretch (this tick or from the window's own check): the car stops and
        // changes gear there, which is not a lack of progress.
        // By index: the path's poses move a little most ticks (map_correction), and a
        // check by pose started the progress check again every tick, so it never fired.
        let stretch = path.stretch();
        if self.stretch.is_some_and(|last| last != stretch) {
            self.progress.reset();
        }
        self.stretch = Some(stretch);
        if window.is_empty() {
            return Some(Tick::Done(Outcome::PathOutsideCostmap));
        }
        let foot = path.foot(&pose);
        let scene = Scene {
            path: &window,
            goal,
            costmap,
            footprint: &self.footprint,
            tracking_unknown: s.tracking_unknown,
            lane: self.lane.as_ref(),
        };
        let plan = match self.optimizer.evaluate(&state, &scene) {
            Ok(plan) => plan,
            Err(NoValidControl) => return Some(Tick::Done(Outcome::NoValidControl)),
        };
        self.last = plan.command;
        if let Some(est) = self.estimate.as_mut() {
            model.step(est, plan.command, dt);
        }
        Some(Tick::Drive(Box::new(Report {
            command: plan.command,
            foot,
            steering: model.steering(&state),
            speed: model.speed(&state),
            plan,
        })))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn goal_check_is_stateful() {
        let mut g = GoalCheck::new(1.0, 0.5);
        let goal = Pose::new(10.0, 0.0, 0.0);
        assert!(!g.done(&Pose::new(8.0, 0.0, 0.0), &goal));
        assert!(
            !g.done(&Pose::new(9.5, 0.0, 1.0), &goal),
            "there, heading off"
        );
        // Drifted past the circle, heading now right: still done (stateful).
        assert!(g.done(&Pose::new(11.5, 0.0, 0.1), &goal));
        g.reset();
        assert!(!g.done(&Pose::new(11.5, 0.0, 0.1), &goal));
    }

    #[test]
    fn progress_check_times_out_unless_moving_or_turning() {
        let mut p = ProgressCheck::new(0.5, 0.5, 10.0);
        assert!(!p.stuck(&Pose::default(), 0.0));
        assert!(!p.stuck(&Pose::new(0.2, 0.0, 0.0), 9.0));
        assert!(p.stuck(&Pose::new(0.3, 0.0, 0.0), 10.5));
        p.reset();
        assert!(!p.stuck(&Pose::default(), 20.0));
        assert!(!p.stuck(&Pose::new(0.0, 0.0, 0.6), 29.0), "a turn counts");
        assert!(!p.stuck(&Pose::new(0.0, 0.0, 0.6), 38.0));
    }
}
