//! Our own navigator: what Nav2's bt_navigator and waypoint_follower do for the car, as a
//! plain state machine. No ROS: the node (`navigator-node`) feeds it events and carries
//! out the commands it returns.
//!
//! ```text
//! goal ─▶ plan ─▶ follow ─▶ replan every second (keep the path on the final approach)
//!           │        │
//!           ▼        ▼  a failure Nav2 would recover from
//!   clear global  clear local, follow again
//!           └────────┴─▶ recoveries in turn: clear both costmaps, wait, back up
//!                        (up to 6), then give up
//! ```
//!
//! It follows our behaviour trees (`aav_navigation/behavior_trees/*_car*.xml`), with a
//! few changes, each in PROVENANCE.md:
//! - a failed replan while a good path is being followed doesn't stop the car: it tries
//!   again next period, and only gives up after `replan_failures` in a row;
//! - through-poses drops a pose once the car has passed it along the path, not only
//!   within 1 m of it (a car cutting a bend by 1.5 m never came back for it);
//! - waypoints (and GPS waypoints) run as one through-poses goal by default: no stop
//!   and no turn to face at each one (TODO 140's bends). `WaypointMode::StopAtEach` is
//!   Nav2's way;
//! - the final approach keeps its path on the last leg of a through-poses goal too.
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod call;
mod machine;
pub mod path;

pub use machine::{Navigator, Progress};

/// A pose in the map frame: metres, radians (REP-103: x forward, yaw counter-clockwise).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad, counter-clockwise from x.
    pub yaw: f64,
}

impl Pose {
    /// The pose (`x`, `y`, `yaw`).
    #[must_use]
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    /// Straight-line distance, m.
    #[must_use]
    pub fn distance(&self, other: &Pose) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }

    /// `local`, a pose in this pose's own frame, put in the frame this pose is in
    /// (`self ∘ local`). With `self` = map -> odom, an odom pose becomes a map pose; with
    /// `self` = the car in map, a base_link pose (1 m ahead, say) becomes a map pose.
    #[must_use]
    pub fn compose(&self, local: &Pose) -> Pose {
        let (s, c) = self.yaw.sin_cos();
        let yaw = self.yaw + local.yaw;
        Pose::new(
            self.x + c * local.x - s * local.y,
            self.y + s * local.x + c * local.y,
            yaw.sin().atan2(yaw.cos()),
        )
    }
}

/// What a goal asks for.
#[derive(Debug, Clone, PartialEq)]
pub enum Mission {
    /// `navigate_to_pose`: drive to one pose.
    ToPose(Pose),
    /// `navigate_through_poses`: one path through each pose in turn.
    ThroughPoses(Vec<Pose>),
    /// `follow_waypoints`, and `follow_gps_waypoints` once on the map: as
    /// [`Settings::waypoints`] says.
    Waypoints(Vec<Pose>),
}

/// How waypoints are driven.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaypointMode {
    /// One through-poses goal: the car passes each waypoint without stopping (ours).
    PassThrough,
    /// Nav2's waypoint_follower: a to-pose goal per waypoint, stopping at each.
    StopAtEach,
}

/// Backing up, one of the recoveries (only where the car may reverse).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BackUp {
    /// m.
    pub distance: f64,
    /// m/s, positive.
    pub speed: f64,
}

/// The navigator's numbers. The defaults are our behaviour trees' and nav2.yaml's.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// s between plans for a to-pose goal (the trees' RateController, 1 Hz).
    pub replan_period: f64,
    /// s between plans for through-poses (0.333 Hz).
    pub through_replan_period: f64,
    /// m: within this of the goal the path is kept, not replanned (vehicle.final_approach).
    pub final_approach: f64,
    /// m: ... as long as the path still ends this close to the goal ...
    pub path_end_tolerance: f64,
    /// s: ... and is younger than this.
    pub path_max_age: f64,
    /// s: elsewhere, a to-pose goal keeps a path that still ends at the goal and nothing
    /// blocks (IsPathValid) while it is younger than this (the trees' KeepAValidPath, 10 s;
    /// TODO 149, 168). Planning every second gave the follower a path a few cm to one side
    /// each time. 0: plan every `replan_period`, as until 15 Nov.
    pub keep_path_age: f64,
    /// Recoveries before giving up (the outer RecoveryNode's retries).
    pub retries: u32,
    /// s: the Wait recovery.
    pub wait: f64,
    /// The BackUp recovery; None: forward only (the car's default).
    pub backup: Option<BackUp>,
    /// m: a through-pose this close to the car is passed (RemovePassedGoals' radius).
    pub pass_radius: f64,
    /// m: a through-pose counts as on the path within this (to be passed along it).
    pub on_path: f64,
    /// How waypoints are driven.
    pub waypoints: WaypointMode,
    /// StopAtEach: a waypoint that fails ends the goal (waypoint_follower's own flag).
    pub stop_on_failure: bool,
    /// Failed replans in a row tolerated while a path is followed (ours; Nav2's tree: 0).
    pub replan_failures: u32,
    /// s: a plan not answered by then fails as [`PlanError::Timeout`] (twice nav2.yaml's
    /// `max_planning_time`, 5 s). A planner that died mid-plan can't hang the goal.
    pub plan_timeout: f64,
    /// s: a follower silent this long (no feedback, no result) fails as
    /// [`FollowError::ControllerTimedOut`], and the goal ends. Followers send feedback
    /// every tick (20 Hz); 4 s is Nav2's `bond_timeout`, after which its lifecycle
    /// manager takes a dead server's whole stack down.
    pub follow_timeout: f64,
    /// s: a costmap clear not answered by then counts as failed. A back up gets this on
    /// top of its own time allowance ([`BackUp::time_allowance`]).
    pub recovery_timeout: f64,
}

impl BackUp {
    /// s: behavior_server's `time_allowance` for this back up: 10 s plus the drive.
    #[must_use]
    pub fn time_allowance(&self) -> f64 {
        10.0 + self.distance.abs() / self.speed.abs().max(0.05)
    }
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            replan_period: 1.0,
            through_replan_period: 3.0,
            final_approach: 8.0,
            path_end_tolerance: 0.5,
            path_max_age: 20.0,
            keep_path_age: 10.0,
            retries: 6,
            wait: 5.0,
            backup: None,
            pass_radius: 1.0,
            on_path: 2.0,
            waypoints: WaypointMode::PassThrough,
            stop_on_failure: false,
            replan_failures: 3,
            plan_timeout: 10.0,
            follow_timeout: 4.0,
            recovery_timeout: 5.0,
        }
    }
}

/// A request's number: an answer carries it back, so a late answer is known as late.
pub type Token = u64;

/// Why the planner failed: Nav2's ComputePathToPose error codes (200s), the same for
/// ComputePathThroughPoses (300s, see [`PlanError::code`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanError {
    /// Nav2's UNKNOWN.
    Unknown,
    /// No planner of that name.
    InvalidPlanner,
    /// The car's pose couldn't be looked up.
    TfError,
    /// The start is off the costmap.
    StartOutsideMap,
    /// The goal is off the costmap.
    GoalOutsideMap,
    /// The car at the start hits something.
    StartOccupied,
    /// The car at the goal would hit something.
    GoalOccupied,
    /// The planner ran out of time, or (ours) never answered within `plan_timeout`.
    Timeout,
    /// The search found no way.
    NoValidPath,
    /// Through-poses only.
    NoViapointsGiven,
}

impl PlanError {
    const ALL: [Self; 10] = [
        Self::Unknown,
        Self::InvalidPlanner,
        Self::TfError,
        Self::StartOutsideMap,
        Self::GoalOutsideMap,
        Self::StartOccupied,
        Self::GoalOccupied,
        Self::Timeout,
        Self::NoValidPath,
        Self::NoViapointsGiven,
    ];

    /// From a planner's `error_code`, to-pose (200s) or through-poses (300s). Any other
    /// number (0 too, from a planner that failed without saying why) is `Unknown`.
    #[must_use]
    pub fn from_code(code: u16) -> Self {
        Self::ALL
            .into_iter()
            .find(|&e| {
                e.code(true) == code || (e != Self::NoViapointsGiven && e.code(false) == code)
            })
            .unwrap_or(Self::Unknown)
    }

    /// Nav2's number: ComputePathToPose's, or ComputePathThroughPoses' with `through`.
    #[must_use]
    pub fn code(self, through: bool) -> u16 {
        let offset = match self {
            Self::Unknown => 0,
            Self::InvalidPlanner => 1,
            Self::TfError => 2,
            Self::StartOutsideMap => 3,
            Self::GoalOutsideMap => 4,
            Self::StartOccupied => 5,
            Self::GoalOccupied => 6,
            Self::Timeout => 7,
            Self::NoValidPath => 8,
            Self::NoViapointsGiven => 9,
        };
        if through {
            300 + offset
        } else {
            200 + offset
        }
    }

    /// WouldAPlannerRecoveryHelp: unknown, no valid path, timeout; and ours: the start
    /// occupied. On the campus the lidar's marks beside a lot exit made the car's own spot
    /// "occupied" (8 Nov); clearing the global costmap forgets them, and the lidar marks
    /// again only what it still sees.
    #[must_use]
    pub fn recovery_helps(self) -> bool {
        matches!(
            self,
            Self::Unknown | Self::StartOccupied | Self::Timeout | Self::NoValidPath
        )
    }
}

/// Why the follower failed: Nav2's FollowPath error codes (100s).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FollowError {
    /// Nav2's UNKNOWN.
    Unknown,
    /// No follower of that name.
    InvalidController,
    /// The car's pose couldn't be looked up.
    TfError,
    /// The path is empty or bad.
    InvalidPath,
    /// The follower waited too long.
    PatienceExceeded,
    /// The car stopped getting nearer the goal.
    FailedToMakeProgress,
    /// Every command the follower tried hits something.
    NoValidControl,
    /// The follower's costmap went stale, or (ours) the follower fell silent for
    /// `follow_timeout`.
    ControllerTimedOut,
}

impl FollowError {
    const ALL: [Self; 8] = [
        Self::Unknown,
        Self::InvalidController,
        Self::TfError,
        Self::InvalidPath,
        Self::PatienceExceeded,
        Self::FailedToMakeProgress,
        Self::NoValidControl,
        Self::ControllerTimedOut,
    ];

    /// From a follower's `error_code`; any other number (0 too) is `Unknown`.
    #[must_use]
    pub fn from_code(code: u16) -> Self {
        Self::ALL
            .into_iter()
            .find(|e| e.code() == code)
            .unwrap_or(Self::Unknown)
    }

    /// Nav2's number.
    #[must_use]
    pub fn code(self) -> u16 {
        match self {
            Self::Unknown => 100,
            Self::InvalidController => 101,
            Self::TfError => 102,
            Self::InvalidPath => 103,
            Self::PatienceExceeded => 104,
            Self::FailedToMakeProgress => 105,
            Self::NoValidControl => 106,
            Self::ControllerTimedOut => 107,
        }
    }

    /// WouldAControllerRecoveryHelp: unknown, patience exceeded, no progress, no valid
    /// control. A follower timed out is not on it: as in Nav2, the goal ends.
    #[must_use]
    pub fn recovery_helps(self) -> bool {
        matches!(
            self,
            Self::Unknown
                | Self::PatienceExceeded
                | Self::FailedToMakeProgress
                | Self::NoValidControl
        )
    }
}

/// Why a step failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Failure {
    /// ComputePathToPose, or ComputePathThroughPoses (`through`).
    Plan {
        /// Nav2's code.
        error: PlanError,
        /// ComputePathThroughPoses, not ComputePathToPose.
        through: bool,
        /// The planner's own words.
        message: String,
    },
    /// FollowPath.
    Follow {
        /// Nav2's code.
        error: FollowError,
        /// The follower's own words.
        message: String,
    },
}

impl Failure {
    /// Nav2's error code.
    #[must_use]
    pub fn code(&self) -> u16 {
        match self {
            Self::Plan { error, through, .. } => error.code(*through),
            Self::Follow { error, .. } => error.code(),
        }
    }

    /// The planner's or the follower's own words.
    #[must_use]
    pub fn message(&self) -> &str {
        match self {
            Self::Plan { message, .. } | Self::Follow { message, .. } => message,
        }
    }

    /// WouldAPlannerRecoveryHelp ([`PlanError::recovery_helps`]).
    #[must_use]
    pub fn planner_recovery_helps(&self) -> bool {
        matches!(self, Self::Plan { error, .. } if error.recovery_helps())
    }

    /// WouldAControllerRecoveryHelp ([`FollowError::recovery_helps`]).
    #[must_use]
    pub fn controller_recovery_helps(&self) -> bool {
        matches!(self, Self::Follow { error, .. } if error.recovery_helps())
    }

    /// Either: the outer recovery runs only then.
    #[must_use]
    pub fn recovery_helps(&self) -> bool {
        self.planner_recovery_helps() || self.controller_recovery_helps()
    }
}

impl std::fmt::Display for Failure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let who = match self {
            Self::Plan { .. } => "the planner",
            Self::Follow { .. } => "the follower",
        };
        let (code, message) = (self.code(), self.message());
        if message.is_empty() {
            write!(f, "{who} failed (error {code})")
        } else {
            write!(f, "{who} failed (error {code}): {message}")
        }
    }
}

/// Why a goal was given up.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Abort {
    /// A newer goal took its place (Nav2's preemption).
    Replaced,
    /// The goal itself can't be driven: no poses, a pose not all numbers, another frame.
    InvalidGoal(String),
    /// A step failed, and no recovery was left or none could help.
    Failed(Failure),
}

impl Abort {
    /// Nav2's error code for the goal's result: the failed step's, else 0.
    #[must_use]
    pub fn code(&self) -> u16 {
        match self {
            Self::Failed(failure) => failure.code(),
            Self::Replaced | Self::InvalidGoal(_) => 0,
        }
    }
}

impl std::fmt::Display for Abort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Replaced => f.write_str("replaced by a newer goal"),
            Self::InvalidGoal(why) => f.write_str(why),
            Self::Failed(failure) => failure.fmt(f),
        }
    }
}

/// How a goal ended.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// There (waypoints: `missed` holds those not reached, by index).
    Succeeded {
        /// The waypoints not reached, by index.
        missed: Vec<usize>,
    },
    /// Given up, and why.
    Aborted {
        /// Why.
        reason: Abort,
        /// The waypoints not reached, by index.
        missed: Vec<usize>,
    },
    /// Cancelled by the caller.
    Cancelled,
}

/// What the node must do.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Ask the planner for a path from the car: one goal, ComputePathToPose; more,
    /// ComputePathThroughPoses. Answer with [`Navigator::planned`].
    Plan {
        /// Hand it back with the answer.
        token: Token,
        /// The goals, in order.
        goals: Vec<Pose>,
    },
    /// Send the follower this path (FollowPath; it replaces the one before without a
    /// stop). Pass on each feedback with [`Navigator::follower_heard`]; answer with
    /// [`Navigator::followed`].
    Follow {
        /// Hand it back with the answer.
        token: Token,
        /// The path to follow.
        path: Vec<Pose>,
    },
    /// Cancel the follower's goal: the car stops.
    StopFollowing,
    /// Clear costmaps. Answer with [`Navigator::recovered`].
    Clear {
        /// Hand it back with the answer.
        token: Token,
        /// Clear the local costmap.
        local: bool,
        /// Clear the global costmap.
        global: bool,
    },
    /// Back up. Answer with [`Navigator::recovered`].
    BackUp {
        /// Hand it back with the answer.
        token: Token,
        /// m.
        distance: f64,
        /// m/s.
        speed: f64,
    },
    /// The goal `id` ended.
    Finished {
        /// The goal's number.
        id: u64,
        /// How it ended.
        outcome: Outcome,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn near(a: &Pose, b: &Pose) -> bool {
        a.distance(b) < 1e-9 && (a.yaw - b.yaw).sin().abs() < 1e-9
    }

    #[test]
    fn compose_puts_a_local_pose_in_the_outer_frame() {
        // A car at (2, 1) facing +y; 1 m ahead of it is (2, 2), 1 m to its left (1, 1).
        let car = Pose::new(2.0, 1.0, FRAC_PI_2);
        assert!(near(
            &car.compose(&Pose::new(1.0, 0.0, 0.0)),
            &Pose::new(2.0, 2.0, FRAC_PI_2)
        ));
        assert!(near(
            &car.compose(&Pose::new(0.0, 1.0, 0.0)),
            &Pose::new(1.0, 1.0, FRAC_PI_2)
        ));
        // The identity changes nothing; the yaw stays within (-pi, pi].
        let p = Pose::new(3.0, -4.0, 0.5);
        assert!(near(&Pose::default().compose(&p), &p));
        let turned = Pose::new(0.0, 0.0, PI - 0.1).compose(&Pose::new(0.0, 0.0, 0.3));
        assert!(turned.yaw.abs() <= PI && (turned.yaw - (0.2 - PI)).abs() < 1e-9);
    }
}
