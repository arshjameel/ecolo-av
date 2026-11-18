//! One drive to B, from the plan to the result (`run.py`): every tick's stops and limits,
//! and the end.
//!
//! A tick, in order (the first that fires ends it):
//!
//! | Check | Then |
//! |---|---|
//! | stopping or cancelled already | a zero command; finished after [`ZEROS`] of them |
//! | a cancel is asked for | cancelled, nothing sent this tick |
//! | [`rules::stop_reason`] | stop |
//! | further than `max_path`, longer than `max_time`, faster than the cruise allows | stop |
//! | outside the corridor ([`Fence`]), unless standing still at B | stop |
//! | asked to move but standing still for `stuck_time` | stop |
//! | otherwise | the drive's command; at B and still for `still_time`: finished |

use std::f64::consts::PI;
use std::fmt;

use ego_core::car_model::{ActuatorConfig, SpeedLoopConfig};
use ego_core::stop_law::{CurveError, StopCurve, INPUT_DELAY};

use crate::approach::{planned_deceleration, Approach, ApproachError, Phase};
use crate::drive::{Command, Drive, Steering};
use crate::line::{Breach, Fence, LineError, Point, Pose};
use crate::rules::{self, Limits, Now, Refusal, Stop};
use crate::{fmt_g, hypot, py_max, py_min};

/// Zero commands at the end, so twist_mux and ego see the stop.
pub const ZEROS: u32 = 5;
/// Ticks between feedback messages.
pub const FEEDBACK_EVERY: u64 = 5;
/// m/s: asked for at least this, the car should move ([`rules::not_moving`]).
const ASKED: f64 = 0.2;

/// The point driver's numbers for a drive (config/drive_to_point.yaml, vehicle.yaml).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// m/s, when the goal gives 0.
    pub cruise_speed: f64,
    /// m/s², when the goal gives 0.
    pub acceleration: f64,
    /// m/s²: vehicle.yaml's full braking.
    pub max_brake: f64,
    /// Plan with this much of `max_brake`.
    pub brake_fraction: f64,
    /// m/s²: 0 = max_brake × brake_fraction.
    pub deceleration: f64,
    /// s: start slowing this much sooner, times the speed.
    pub brake_lead: f64,
    /// m: this close to B, command zero.
    pub stop_band: f64,
    /// m/s: the least it asks for short of B.
    pub creep: f64,
    /// m: pure pursuit's carrot, at least.
    pub lookahead_min: f64,
    /// s of driving: the carrot moves out with speed.
    pub lookahead_time: f64,
    /// m/s²: slow down in tight turns.
    pub lateral_accel: f64,
    /// m/s: standing still.
    pub still_speed: f64,
    /// s standing still though asked to move: stop.
    pub stuck_time: f64,
    /// s standing still at B before it reports.
    pub still_time: f64,
    /// m either side of the line A -> B.
    pub max_off_line: f64,
    /// m past B along the line.
    pub max_overrun: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            cruise_speed: 1.0,
            acceleration: 0.5,
            max_brake: 3.0,
            brake_fraction: 0.5,
            deceleration: 0.0,
            brake_lead: 0.8,
            stop_band: 0.05,
            creep: 0.1,
            lookahead_min: 2.0,
            lookahead_time: 1.0,
            lateral_accel: 1.5,
            still_speed: 0.05,
            stuck_time: 5.0,
            still_time: 1.0,
            max_off_line: 3.0,
            max_overrun: 2.0,
        }
    }
}

/// Ego's caps, as its parameters answered (None: it didn't say).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Caps {
    /// m/s.
    pub max_speed: Option<f64>,
    /// m/s².
    pub max_acceleration: Option<f64>,
    /// m/s².
    pub max_deceleration: Option<f64>,
    /// m/s³.
    pub max_jerk: Option<f64>,
}

/// Why a drive can't be planned or started.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlanError {
    /// A rule refused it.
    Refused(Refusal),
    /// The plan's numbers (a bad settings file, an odd ego).
    Approach(ApproachError),
    /// The stopping law's numbers.
    Curve(CurveError),
    /// The corridor's numbers.
    Line(LineError),
}

impl fmt::Display for PlanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PlanError::Refused(r) => write!(f, "{r}"),
            PlanError::Approach(e) => write!(f, "error: {e}"),
            PlanError::Curve(e) => write!(f, "error: {e}"),
            PlanError::Line(e) => write!(f, "error: {e}"),
        }
    }
}

impl std::error::Error for PlanError {}

/// The Approach for a goal's numbers (0 = default) and ego's caps. `car`: the car model
/// for the stopping law (`stop_law:=hybrid`), or None for approach's own law.
///
/// # Errors
///
/// [`PlanError`]: a goal number refused, or numbers that make no plan.
pub fn plan(
    cruise: f64,
    accel: f64,
    decel: f64,
    caps: &Caps,
    s: &Settings,
    car: Option<&ActuatorConfig>,
) -> Result<Approach, PlanError> {
    if let Some(why) = rules::check_numbers(cruise, accel, decel) {
        return Err(PlanError::Refused(why));
    }
    let or = |v: f64, d: f64| if v != 0.0 { v } else { d };
    let cruise = or(cruise, s.cruise_speed);
    let accel = or(accel, s.acceleration);
    let decel = if decel != 0.0 {
        decel
    } else if s.deceleration != 0.0 {
        s.deceleration
    } else {
        planned_deceleration(s.max_brake, s.brake_fraction).map_err(PlanError::Approach)?
    };
    // Ego's caps: it would cut a faster command anyway, and the plan must know.
    // Without ego, the real car's 1 m/s (ego.yaml).
    let cruise = py_min(cruise, caps.max_speed.unwrap_or(1.0));
    let accel = py_min(accel, caps.max_acceleration.unwrap_or(accel));
    let decel = py_min(decel, caps.max_deceleration.unwrap_or(decel));
    let curve = car
        .map(|cfg| {
            StopCurve::from_car_model(
                cfg,
                caps.max_deceleration.unwrap_or(2.0),
                caps.max_jerk.unwrap_or(2.0),
                INPUT_DELAY,
                Some(&SpeedLoopConfig::default()),
            )
        })
        .transpose()
        .map_err(PlanError::Curve)?;
    Approach::new(
        cruise,
        accel,
        decel,
        s.brake_lead,
        s.stop_band,
        py_min(s.creep, cruise),
        curve,
    )
    .map_err(PlanError::Approach)
}

/// Why a running drive stopped.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Halt {
    /// A rule: the e-stop, a lock, stale inputs, another driver.
    Rule(Stop),
    /// Drove this far (m), over the limit (m).
    Path(f64, f64),
    /// Ran this long (s), over the limit (s).
    Time(f64, f64),
    /// Measured this speed (m/s), over the cruise (m/s) allows.
    Speed(f64, f64),
    /// Left the corridor.
    Corridor(Breach),
    /// Stood still this long (s) though asked to move.
    Stuck(f64),
}

impl fmt::Display for Halt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Halt::Rule(r) => write!(f, "{r}"),
            Halt::Path(p, max) => write!(f, "drove {p:.0} m, over the limit of {max:.0} m"),
            Halt::Time(t, max) => write!(f, "ran {t:.0} s, over the limit of {max:.0} s"),
            Halt::Speed(v, cruise) => write!(
                f,
                "speed {v:.2} m/s, over the cruise speed {} m/s",
                fmt_g(cruise)
            ),
            Halt::Corridor(b) => write!(f, "{b}"),
            Halt::Stuck(t) => write!(
                f,
                "the car stood still for {} s though asked to move: something holds it",
                fmt_g(t)
            ),
        }
    }
}

/// What one tick says to do.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Tick {
    /// (speed m/s, curvature 1/m) to publish.
    pub send: Option<(f64, f64)>,
    /// The drive stops now, and why (log it).
    pub stopped: Option<Halt>,
    /// Send feedback with this command (and the measured speed).
    pub feedback: Option<Command>,
    /// The drive is over: end the goal.
    pub finished: bool,
}

/// How a goal ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// At B, within the tolerance.
    Succeeded,
    /// Stopped by a rule or a limit, or too far from B.
    Aborted,
    /// A cancel was asked for.
    Cancelled,
}

/// How a drive ended: the goal's result.
#[derive(Debug, Clone, PartialEq)]
pub struct Outcome {
    /// Succeeded, aborted or cancelled.
    pub status: Status,
    /// At B.
    pub success: bool,
    /// The result's words.
    pub message: String,
    /// m from B, straight.
    pub left: f64,
    /// m: how close counts as at B.
    pub tolerance: f64,
    /// m, from the speed.
    pub travelled: f64,
    /// s.
    pub duration: f64,
}

/// One drive: the Drive, its corridor and limits, and what happened so far.
#[derive(Debug, Clone, PartialEq)]
pub struct Run {
    target: Point,
    s: Settings,
    limits: Limits,
    drive: Drive,
    corridor: Fence,
    distance: f64,
    max_path: f64,
    max_time: f64,
    t0: f64,
    last: f64,
    ticks: u64,
    zeros: u32,
    still: Option<f64>,
    stuck: Option<f64>,
    path: f64,
    stop: Option<Halt>,
    cancel: bool,
    cmd: Option<Command>,
    done: bool,
}

impl Run {
    /// A drive from `start` at time `t0` (s, the node's clock); `radius`: the tightest turn.
    ///
    /// # Errors
    ///
    /// [`LineError`] for a radius or corridor number not above 0.
    pub fn new(
        target: Point,
        approach: Approach,
        s: Settings,
        limits: Limits,
        radius: f64,
        start: &Pose,
        t0: f64,
    ) -> Result<Self, LineError> {
        let steering = Steering {
            max_curvature: 1.0 / radius,
            lookahead_min: s.lookahead_min,
            lookahead_time: s.lookahead_time,
            lateral_accel: s.lateral_accel,
            ..Steering::new(1.0 / radius)
        };
        // The drive's own corridor (TODO 187), apart from the site's geofence and Nav2.
        let corridor = Fence::new(
            (start.x, start.y),
            target,
            radius,
            s.max_off_line,
            s.max_overrun,
        )?;
        let distance = hypot(target.0 - start.x, target.1 - start.y);
        Ok(Run {
            target,
            s,
            limits,
            drive: Drive::new(target, approach, steering, start),
            corridor,
            distance,
            // A turn round costs up to a circle.
            max_path: 2.0 * distance + 2.0 * PI * radius + 10.0,
            max_time: 2.0 * approach.ideal_time(distance + PI * radius) + 30.0,
            t0,
            last: t0,
            ticks: 0,
            zeros: 0,
            still: None,
            stuck: None,
            path: 0.0,
            stop: None,
            cancel: false,
            cmd: None,
            done: false,
        })
    }

    /// m from the start to B.
    #[must_use]
    pub fn distance(&self) -> f64 {
        self.distance
    }

    /// The plan.
    #[must_use]
    pub fn approach(&self) -> &Approach {
        self.drive.approach()
    }

    /// The limit on the distance driven (m).
    #[must_use]
    pub fn max_path(&self) -> f64 {
        self.max_path
    }

    /// The limit on the drive's time (s).
    #[must_use]
    pub fn max_time(&self) -> f64 {
        self.max_time
    }

    fn halt(&mut self, why: Halt) -> Tick {
        if self.stop.is_none() && !self.cancel {
            self.stop = Some(why);
            return Tick {
                stopped: Some(why),
                ..Tick::default()
            };
        }
        Tick::default()
    }

    fn finish(&mut self) -> bool {
        !std::mem::replace(&mut self.done, true)
    }

    /// One tick at time `t` (s, the node's clock): what to send. `now`: what the driver
    /// knows. `pose`, `measured`: the global EKF's pose and speed. `cancel_asked`: the
    /// goal's cancel was asked for.
    pub fn tick(
        &mut self,
        t: f64,
        now: &Now,
        pose: &Pose,
        measured: f64,
        cancel_asked: bool,
    ) -> Tick {
        let s = self.s;
        if self.stop.is_some() || self.cancel {
            self.zeros += 1;
            let finished = self.zeros >= ZEROS && self.finish();
            return Tick {
                send: Some((0.0, 0.0)),
                finished,
                ..Tick::default()
            };
        }
        if cancel_asked {
            self.cancel = true;
            return Tick::default();
        }
        let dt = py_min(py_max(t - self.last, 0.0), 0.5);
        self.last = t;
        if let Some(why) = rules::stop_reason(now, &self.limits) {
            return self.halt(Halt::Rule(why));
        }
        // The distance driven, from the speed: the EKF's position jitters with the GPS.
        self.path += measured.abs() * dt;
        if self.path > self.max_path {
            return self.halt(Halt::Path(self.path, self.max_path));
        }
        if t - self.t0 > self.max_time {
            return self.halt(Halt::Time(t - self.t0, self.max_time));
        }
        let cruise = self.drive.approach().cruise;
        if measured.abs() > cruise * 1.2 + 0.3 {
            return self.halt(Halt::Speed(measured, cruise));
        }
        // The corridor, until the car stands still at B: a GPS jump there shouldn't turn a
        // good stop into an abort.
        let standing_at_b = self
            .cmd
            .is_some_and(|c| c.phase == Phase::Stopping && measured.abs() < s.still_speed);
        if !standing_at_b {
            if let Some(why) = self.corridor.breach((pose.x, pose.y), self.drive.line()) {
                return self.halt(Halt::Corridor(why));
            }
        }
        let cmd = self.drive.tick(pose, measured, dt);
        self.cmd = Some(cmd);
        // Asked to move, standing still: something holds the car. Stop asking.
        if rules::not_moving(cmd.speed, measured, s.still_speed, ASKED) {
            let since = *self.stuck.get_or_insert(t);
            if t - since > s.stuck_time {
                return self.halt(Halt::Stuck(s.stuck_time));
            }
        } else {
            self.stuck = None;
        }
        let mut finished = false;
        if cmd.phase == Phase::Stopping {
            if measured.abs() < s.still_speed {
                let since = *self.still.get_or_insert(t);
                finished = t - since >= s.still_time && self.finish();
            } else {
                self.still = None;
            }
        }
        let feedback = self.ticks.is_multiple_of(FEEDBACK_EVERY).then_some(cmd);
        self.ticks += 1;
        Tick {
            send: Some((cmd.speed, cmd.curvature)),
            stopped: None,
            feedback,
            finished,
        }
    }

    /// The result, with the car at `pose` at time `t`. `fix_status`: the last fix's
    /// NavSatStatus; `require_gps` false counts as RTK.
    #[must_use]
    pub fn outcome(
        &self,
        pose: &Pose,
        fix_status: Option<i8>,
        require_gps: bool,
        t: f64,
    ) -> Outcome {
        let left = hypot(self.target.0 - pose.x, self.target.1 - pose.y);
        let tolerance = rules::tolerance(if require_gps {
            fix_status
        } else {
            Some(rules::STATUS_GBAS_FIX)
        });
        let (status, success, message) = if self.cancel {
            (
                Status::Cancelled,
                false,
                format!("cancelled; stopped {left:.2} m from B"),
            )
        } else if let Some(why) = self.stop {
            (
                Status::Aborted,
                false,
                format!("stopped {left:.2} m from B: {why}"),
            )
        } else {
            let success = left <= tolerance;
            (
                if success {
                    Status::Succeeded
                } else {
                    Status::Aborted
                },
                success,
                format!(
                    "stopped {left:.2} m from B (tolerance {} m){}",
                    fmt_g(tolerance),
                    if success { "" } else { ": too far" }
                ),
            )
        };
        Outcome {
            status,
            success,
            message,
            left,
            tolerance,
            travelled: self.path,
            duration: t - self.t0,
        }
    }
}
