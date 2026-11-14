//! The goal state machine: one goal at a time.

use crate::path;
use crate::{
    Abort, Command, Failure, FollowError, Mission, Outcome, PlanError, Pose, Settings, Token,
    WaypointMode,
};

/// How the active goal is driven.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// One pose (navigate_to_pose).
    ToPose,
    /// One path through the poses left (navigate_through_poses; waypoints, PassThrough).
    Through,
    /// A to-pose goal per waypoint (waypoints, StopAtEach).
    EachInTurn,
}

/// One of the outer recoveries, taken in turn (the trees' RoundRobin).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Recovery {
    ClearBoth,
    Wait,
    BackUp,
}

/// Each stage but `Driving` and `Waiting` waits for an answer (its token), until its
/// deadline: after it, the answer counts as a failure.
#[derive(Debug, Clone, PartialEq)]
enum Stage {
    /// Planning and following.
    Driving,
    /// Clearing the global costmap before planning once more (the planner's own recovery).
    /// `cause`: the plan's failure, the goal's if the clear fails too.
    ClearingForPlan {
        token: Token,
        deadline: f64,
        cause: Failure,
    },
    /// Clearing the local costmap before following once more (the follower's own).
    ClearingForFollow { token: Token, deadline: f64 },
    /// An outer recovery that answers: clearing both costmaps, or backing up.
    Recovering { token: Token, deadline: f64 },
    /// The Wait recovery.
    Waiting { until: f64 },
}

/// A plan asked for and not answered yet.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Asked {
    token: Token,
    /// When it was asked.
    at: f64,
    /// ComputePathThroughPoses (more than one pose), for its error codes.
    through: bool,
}

/// The follower's goal now.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Following {
    token: Token,
    /// When the follower last said anything (feedback), or was first sent a path.
    heard: f64,
}

/// The path being followed, when it was made, and whether the planner's IsPathValid
/// last said something blocks it.
#[derive(Debug, Clone)]
struct Held {
    poses: Vec<Pose>,
    made: f64,
    blocked: bool,
}

#[derive(Debug, Clone)]
struct Active {
    id: u64,
    kind: Kind,
    /// Through: the poses still ahead. ToPose and EachInTurn: every pose given. Never empty.
    targets: Vec<Pose>,
    /// EachInTurn: the waypoint driven to now. Through: how many have been passed.
    index: usize,
    total: usize,
    missed: Vec<usize>,
    started: f64,
    stage: Stage,
    path: Option<Held>,
    plan: Option<Asked>,
    last_plan: Option<f64>,
    plan_retry_used: bool,
    replans_failed: u32,
    following: Option<Following>,
    follow_retry_used: bool,
    recoveries: u32,
    next_recovery: usize,
}

impl Active {
    /// The poses the planner must reach next.
    fn goals(&self) -> Vec<Pose> {
        match self.kind {
            Kind::Through => self.targets.clone(),
            Kind::ToPose => self.targets.first().copied().into_iter().collect(),
            Kind::EachInTurn => self.targets.get(self.index).copied().into_iter().collect(),
        }
    }

    /// Where this leg ends.
    fn end(&self) -> Option<Pose> {
        match self.kind {
            Kind::EachInTurn => self.targets.get(self.index).copied(),
            _ => self.targets.last().copied(),
        }
    }

    /// A fresh start for the next waypoint, or after an outer recovery.
    fn reset_attempt(&mut self) {
        self.stage = Stage::Driving;
        self.path = None;
        self.plan = None;
        self.last_plan = None;
        self.plan_retry_used = false;
        self.replans_failed = 0;
        self.following = None;
        self.follow_retry_used = false;
    }
}

/// How far along a goal is, for its feedback.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Progress {
    /// Goal number, as given to [`Navigator::start`].
    pub id: u64,
    /// m along the path to its end (the straight line with no path yet).
    pub distance_remaining: f64,
    pub recoveries: u32,
    /// Waypoints: the one driven to now (EachInTurn), or how many passed (PassThrough).
    pub waypoint: usize,
    /// Poses still ahead (through-poses' feedback).
    pub poses_remaining: usize,
    /// When it started (the clock given to [`Navigator::start`]).
    pub started: f64,
}

/// The next request number.
fn next(tokens: &mut Token) -> Token {
    *tokens += 1;
    *tokens
}

/// Nav2's navigator for the car, as a state machine. Every method returns what the node
/// must do now; times are seconds on any one clock.
#[derive(Debug)]
pub struct Navigator {
    settings: Settings,
    active: Option<Active>,
    tokens: Token,
}

impl Navigator {
    pub fn new(settings: Settings) -> Self {
        Self {
            settings,
            active: None,
            tokens: 0,
        }
    }

    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The goal now, if any.
    pub fn goal(&self) -> Option<u64> {
        self.active.as_ref().map(|a| a.id)
    }

    /// A new goal. One already running ends as aborted ("replaced"), as Nav2 does when a
    /// goal is preempted; the follower keeps driving until the new path arrives.
    pub fn start(&mut self, id: u64, mission: Mission, now: f64) -> Vec<Command> {
        let mut out = Vec::new();
        if let Some(old) = self.active.take() {
            out.push(Command::Finished {
                id: old.id,
                outcome: Outcome::Aborted {
                    reason: Abort::Replaced,
                    missed: Vec::new(),
                },
            });
        }
        let (kind, targets) = match mission {
            Mission::ToPose(p) => (Kind::ToPose, vec![p]),
            Mission::ThroughPoses(poses) => (Kind::Through, poses),
            Mission::Waypoints(poses) => match self.settings.waypoints {
                WaypointMode::PassThrough => (Kind::Through, poses),
                WaypointMode::StopAtEach => (Kind::EachInTurn, poses),
            },
        };
        if targets.is_empty() || targets.iter().any(|p| !finite(p)) {
            out.push(Command::StopFollowing);
            out.push(Command::Finished {
                id,
                outcome: Outcome::Aborted {
                    reason: Abort::InvalidGoal(
                        "no poses, or a pose that is not all numbers".into(),
                    ),
                    missed: Vec::new(),
                },
            });
            return out;
        }
        let total = targets.len();
        self.active = Some(Active {
            id,
            kind,
            targets,
            index: 0,
            total,
            missed: Vec::new(),
            started: now,
            stage: Stage::Driving,
            path: None,
            plan: None,
            last_plan: None,
            plan_retry_used: false,
            replans_failed: 0,
            following: None,
            follow_retry_used: false,
            recoveries: 0,
            next_recovery: 0,
        });
        out
    }

    /// The goal `id` is cancelled: the car stops.
    pub fn cancel(&mut self, id: u64) -> Vec<Command> {
        if self.goal() != Some(id) {
            return Vec::new();
        }
        self.active = None;
        vec![
            Command::StopFollowing,
            Command::Finished {
                id,
                outcome: Outcome::Cancelled,
            },
        ]
    }

    /// The clock moved on; `car` is where the car is on the map, if known. Also where the
    /// time limits fire: a planner, follower or recovery that never answers fails here.
    pub fn tick(&mut self, now: f64, car: Option<&Pose>) -> Vec<Command> {
        let Some(a) = self.active.as_mut() else {
            return Vec::new();
        };
        match a.stage {
            Stage::Driving => {}
            Stage::Waiting { until } if now >= until => a.reset_attempt(),
            Stage::Waiting { .. } => return Vec::new(),
            Stage::ClearingForPlan {
                token, deadline, ..
            }
            | Stage::ClearingForFollow { token, deadline }
            | Stage::Recovering { token, deadline } => {
                return if now >= deadline {
                    self.recovered(token, false, now)
                } else {
                    Vec::new()
                };
            }
        }
        if let Some(f) = a.following {
            if now - f.heard >= self.settings.follow_timeout {
                let why = format!(
                    "no word from the follower in {:.1} s",
                    self.settings.follow_timeout
                );
                return self.followed(f.token, Err((FollowError::ControllerTimedOut, why)), now);
            }
        }
        if a.kind == Kind::Through {
            if let Some(car) = car {
                drop_passed(a, car, &self.settings);
            }
        }
        if let Some(asked) = a.plan {
            if now - asked.at >= self.settings.plan_timeout {
                let why = format!(
                    "no answer from the planner in {:.1} s",
                    self.settings.plan_timeout
                );
                return self.planned(asked.token, Err((PlanError::Timeout, why)), now);
            }
            return Vec::new();
        }
        let period = if a.kind == Kind::Through {
            self.settings.through_replan_period
        } else {
            self.settings.replan_period
        };
        let due = a.last_plan.is_none_or(|t| now - t >= period);
        if !due || keep_path(a, car, now, &self.settings) {
            return Vec::new();
        }
        let goals = a.goals();
        let token = next(&mut self.tokens);
        a.plan = Some(Asked {
            token,
            at: now,
            through: goals.len() > 1,
        });
        a.last_plan = Some(now);
        vec![Command::Plan { token, goals }]
    }

    /// The path to ask IsPathValid about, while the car is on the final approach (where
    /// the path is kept, not replanned): its `made` time (to name it in the answer) and its
    /// poses. `None` elsewhere: the second's replan already checks the way.
    pub fn path_to_check(&self, now: f64, car: Option<&Pose>) -> Option<(f64, &[Pose])> {
        let a = self.active.as_ref()?;
        if !on_final_approach(a, car, now, &self.settings) {
            return None;
        }
        a.path.as_ref().map(|h| (h.made, h.poses.as_slice()))
    }

    /// IsPathValid's answer about the path made at `made`: a blocked path isn't kept, so
    /// the next tick plans again (the trees' `IsPathValid` in the final approach's
    /// sequence). An answer about an older path is ignored.
    pub fn path_checked(&mut self, made: f64, valid: bool) {
        if let Some(h) = self.active.as_mut().and_then(|a| a.path.as_mut()) {
            if h.made == made {
                h.blocked = !valid;
            }
        }
    }

    /// The planner's answer to [`Command::Plan`]: a path, or Nav2's error and why.
    pub fn planned(
        &mut self,
        token: Token,
        answer: Result<Vec<Pose>, (PlanError, String)>,
        now: f64,
    ) -> Vec<Command> {
        let Some(a) = self.active.as_mut() else {
            return Vec::new();
        };
        let Some(asked) = a.plan.filter(|p| p.token == token) else {
            return Vec::new(); // late: a newer plan, goal or recovery took its place
        };
        a.plan = None;
        let answer = match answer {
            Ok(p) if p.is_empty() => Err((
                PlanError::NoValidPath,
                "the planner sent an empty path".to_string(),
            )),
            other => other,
        };
        match answer {
            Ok(poses) => {
                a.plan_retry_used = false;
                a.replans_failed = 0;
                a.path = Some(Held {
                    poses: poses.clone(),
                    made: now,
                    blocked: false,
                });
                let token = next(&mut self.tokens);
                // A new path for a follower already driving: it hasn't spoken yet.
                let heard = a.following.map_or(now, |f| f.heard);
                a.following = Some(Following { token, heard });
                vec![Command::Follow { token, path: poses }]
            }
            Err((error, message)) => {
                let failure = Failure::Plan {
                    error,
                    through: asked.through,
                    message,
                };
                // Ours: a good path is still being followed; try again next period.
                if a.following.is_some() && a.replans_failed < self.settings.replan_failures {
                    a.replans_failed += 1;
                    return Vec::new();
                }
                if !a.plan_retry_used && failure.planner_recovery_helps() {
                    a.plan_retry_used = true;
                    let token = next(&mut self.tokens);
                    a.stage = Stage::ClearingForPlan {
                        token,
                        deadline: now + self.settings.recovery_timeout,
                        cause: failure,
                    };
                    return vec![Command::Clear {
                        token,
                        local: false,
                        global: true,
                    }];
                }
                self.attempt_failed(failure, now)
            }
        }
    }

    /// The follower said something about the goal it drives (FollowPath feedback): it is
    /// alive, and `follow_timeout` starts again.
    pub fn follower_heard(&mut self, now: f64) {
        if let Some(f) = self.active.as_mut().and_then(|a| a.following.as_mut()) {
            f.heard = f.heard.max(now);
        }
    }

    /// The follower's answer to [`Command::Follow`]: there, or Nav2's error and why.
    pub fn followed(
        &mut self,
        token: Token,
        answer: Result<(), (FollowError, String)>,
        now: f64,
    ) -> Vec<Command> {
        let Some(a) = self.active.as_mut() else {
            return Vec::new();
        };
        if a.following.is_none_or(|f| f.token != token) {
            return Vec::new(); // replaced by a newer path, or stopped on purpose
        }
        a.following = None;
        match answer {
            Ok(()) => self.arrived(),
            Err((error, message)) => {
                let failure = Failure::Follow { error, message };
                if !a.follow_retry_used && failure.controller_recovery_helps() && a.path.is_some() {
                    // The path to follow again is kept in `path`.
                    a.follow_retry_used = true;
                    a.plan = None; // a plan answered meanwhile would only race this
                    let token = next(&mut self.tokens);
                    a.stage = Stage::ClearingForFollow {
                        token,
                        deadline: now + self.settings.recovery_timeout,
                    };
                    return vec![Command::Clear {
                        token,
                        local: true,
                        global: false,
                    }];
                }
                self.attempt_failed(failure, now)
            }
        }
    }

    /// The answer to [`Command::Clear`] or [`Command::BackUp`]. A recovery that failed
    /// still counts; the goal carries on (the next recovery is another kind).
    pub fn recovered(&mut self, token: Token, ok: bool, now: f64) -> Vec<Command> {
        let Some(a) = self.active.as_mut() else {
            return Vec::new();
        };
        match &a.stage {
            Stage::ClearingForPlan {
                token: t, cause, ..
            } if *t == token => {
                let cause = cause.clone();
                a.stage = Stage::Driving;
                if !ok {
                    // Nav2's tree: the clear failing fails the attempt, with the plan's code.
                    return self.attempt_failed(cause, now);
                }
                a.last_plan = None; // plan again at once
                self.tick(now, None)
            }
            Stage::ClearingForFollow { token: t, .. } if *t == token => {
                a.stage = Stage::Driving;
                let Some(held) = a.path.as_ref() else {
                    return Vec::new();
                };
                let path = held.poses.clone();
                let token = next(&mut self.tokens);
                a.following = Some(Following { token, heard: now });
                vec![Command::Follow { token, path }]
            }
            Stage::Recovering { token: t, .. } if *t == token => {
                a.reset_attempt();
                self.tick(now, None)
            }
            _ => Vec::new(),
        }
    }

    /// How far along the goal is.
    pub fn progress(&self, car: Option<&Pose>) -> Option<Progress> {
        let a = self.active.as_ref()?;
        let distance_remaining = match (&a.path, car, a.end()) {
            (Some(h), Some(c), _) => path::remaining(&h.poses, c),
            (None, Some(c), Some(end)) => c.distance(&end),
            _ => f64::NAN,
        };
        Some(Progress {
            id: a.id,
            distance_remaining,
            recoveries: a.recoveries,
            waypoint: a.index,
            poses_remaining: match a.kind {
                Kind::Through => a.targets.len(),
                Kind::ToPose => 1,
                Kind::EachInTurn => a.total - a.index,
            },
            started: a.started,
        })
    }

    /// The follower says the car is at the end of its path.
    fn arrived(&mut self) -> Vec<Command> {
        let Some(a) = self.active.as_mut() else {
            return Vec::new();
        };
        if a.kind == Kind::EachInTurn && a.index + 1 < a.total {
            a.index += 1;
            a.recoveries = 0;
            a.next_recovery = 0;
            a.reset_attempt();
            return Vec::new();
        }
        let Some(a) = self.active.take() else {
            return Vec::new();
        };
        vec![Command::Finished {
            id: a.id,
            outcome: Outcome::Succeeded { missed: a.missed },
        }]
    }

    /// The plan-and-follow attempt failed: the next recovery, or give up.
    fn attempt_failed(&mut self, failure: Failure, now: f64) -> Vec<Command> {
        let mut out = vec![Command::StopFollowing];
        let Some(a) = self.active.as_mut() else {
            return out;
        };
        a.following = None;
        a.plan = None;
        if !failure.recovery_helps() || a.recoveries >= self.settings.retries {
            out.extend(self.leg_failed(failure));
            return out;
        }
        a.recoveries += 1;
        let mut turns = vec![Recovery::ClearBoth, Recovery::Wait];
        if self.settings.backup.is_some() {
            turns.push(Recovery::BackUp);
        }
        let pick = turns[a.next_recovery % turns.len()];
        a.next_recovery += 1;
        let deadline = now + self.settings.recovery_timeout;
        match (pick, self.settings.backup) {
            (Recovery::ClearBoth, _) => {
                let token = next(&mut self.tokens);
                a.stage = Stage::Recovering { token, deadline };
                out.push(Command::Clear {
                    token,
                    local: true,
                    global: true,
                });
            }
            (Recovery::BackUp, Some(b)) => {
                let token = next(&mut self.tokens);
                a.stage = Stage::Recovering {
                    token,
                    deadline: deadline + b.time_allowance(),
                };
                out.push(Command::BackUp {
                    token,
                    distance: b.distance,
                    speed: b.speed,
                });
            }
            _ => {
                a.stage = Stage::Waiting {
                    until: now + self.settings.wait,
                };
            }
        }
        out
    }

    /// No more recoveries for this leg: the goal fails, or (waypoints, each in turn, not
    /// stop_on_failure) the waypoint is missed and the next one starts.
    fn leg_failed(&mut self, failure: Failure) -> Vec<Command> {
        let Some(a) = self.active.as_mut() else {
            return Vec::new();
        };
        if a.kind == Kind::EachInTurn && !self.settings.stop_on_failure && a.index + 1 < a.total {
            a.missed.push(a.index);
            a.index += 1;
            a.recoveries = 0;
            a.next_recovery = 0;
            a.reset_attempt();
            return Vec::new();
        }
        let Some(a) = self.active.take() else {
            return Vec::new();
        };
        let mut missed = a.missed;
        match a.kind {
            Kind::EachInTurn => missed.push(a.index),
            Kind::Through => missed.extend(a.index..a.total),
            Kind::ToPose => {}
        }
        vec![Command::Finished {
            id: a.id,
            outcome: Outcome::Aborted {
                reason: Abort::Failed(failure),
                missed,
            },
        }]
    }
}

fn finite(p: &Pose) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.yaw.is_finite()
}

/// Through-poses: drop the poses the car has passed (never the last one).
fn drop_passed(a: &mut Active, car: &Pose, settings: &Settings) {
    while a.targets.len() > 1 {
        let next = a.targets[0];
        let near = next.distance(car) <= settings.pass_radius;
        let along = a
            .path
            .as_ref()
            .is_some_and(|h| path::passed_along(&h.poses, &next, car, settings.on_path));
        if !(near || along) {
            break;
        }
        a.targets.remove(0);
        a.index += 1;
    }
}

/// The final approach: near the goal, keep a path that still ends there and is young
/// enough, instead of replanning (TODO 17: replans there made the car circle the goal).
fn keep_path(a: &Active, car: Option<&Pose>, now: f64, settings: &Settings) -> bool {
    on_final_approach(a, car, now, settings) && a.path.as_ref().is_some_and(|h| !h.blocked)
}

/// The trees' final approach, but for IsPathValid: the last leg, near the goal, a path
/// that still ends there and is young enough to keep.
fn on_final_approach(a: &Active, car: Option<&Pose>, now: f64, settings: &Settings) -> bool {
    let (Some(car), Some(held), Some(end)) = (car, &a.path, a.end()) else {
        return false;
    };
    let Some(path_end) = held.poses.last() else {
        return false;
    };
    let last_leg = match a.kind {
        Kind::Through => a.targets.len() == 1,
        _ => true,
    };
    last_leg
        && car.distance(&end) <= settings.final_approach
        && path_end.distance(&end) <= settings.path_end_tolerance
        && now - held.made < settings.path_max_age
}
