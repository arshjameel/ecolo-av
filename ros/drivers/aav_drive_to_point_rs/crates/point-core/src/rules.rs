//! The point driver's rules: when a drive may start, when it must stop, how close is close
//! (`rules.py`). Every rule errs towards "this is the real car".
//!
//! | Rule | Function |
//! |---|---|
//! | A goal's numbers are sane | [`check_numbers`] |
//! | May start: no e-stop or lock, a fresh position and fix, confirmed on REAL | [`refuse_start`] |
//! | B is near enough, and a forward-only car can reach it | [`refuse_target`] |
//! | B is inside the geofence | [`refuse_outside`] |
//! | Stop now: e-stop, lock or silent watchdog, stale position or fix, another driver | [`stop_reason`] |
//! | The car doesn't move though asked to | [`not_moving`] |
//! | How close counts as at B: 1 m with RTK, 2 m without | [`tolerance`] |
//!
//! `/system/source`'s kinds follow the actuator tests' rules (`aav_actuator_tests`
//! `safety.py`'s `source_kind`), so the two agree on what REAL means.

use std::fmt;

use crate::line::{reachable, Point, Pose};
use crate::{fmt_g, hypot, py_repr};

/// sensor_msgs/NavSatStatus: a fix with ground-based augmentation (RTK).
pub const STATUS_GBAS_FIX: i8 = 2;
/// m: docs/tests.md 5.2, with an RTK fix.
pub const RTK_TOLERANCE: f64 = 1.0;
/// m: without.
pub const GPS_TOLERANCE: f64 = 2.0;
/// What a person types to start a drive on the real car.
pub const CONFIRM_WORD: &str = "drive";

/// aav_interfaces/SensorSource's values.
const SOURCE_REAL: u8 = 0;
const SOURCE_BENCH: u8 = 2;
const SOURCE_REPLAY: u8 = 3;

/// What `/system/source` says we drive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// The real car.
    Real,
    /// A simulator.
    Sim,
    /// Bench mode.
    Bench,
    /// A recording.
    Replay,
    /// Nobody announced: treated as the real car.
    Unknown,
}

impl Kind {
    /// Its word, as the Python's `Kind.value`.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            Kind::Real => "REAL",
            Kind::Sim => "SIM",
            Kind::Bench => "BENCH",
            Kind::Replay => "REPLAY",
            Kind::Unknown => "UNKNOWN",
        }
    }
}

/// Merge `/system/source`'s announcements: every entry's source value, from everyone.
/// Anything REAL makes it REAL, even beside a simulator; then REPLAY, BENCH, else SIM.
#[must_use]
pub fn source_kind(values: impl IntoIterator<Item = u8>) -> Kind {
    let (mut any, mut real, mut replay, mut bench) = (false, false, false, false);
    for v in values {
        any = true;
        real |= v == SOURCE_REAL;
        replay |= v == SOURCE_REPLAY;
        bench |= v == SOURCE_BENCH;
    }
    match (any, real, replay, bench) {
        (false, ..) => Kind::Unknown,
        (_, true, ..) => Kind::Real,
        (_, _, true, _) => Kind::Replay,
        (_, _, _, true) => Kind::Bench,
        _ => Kind::Sim,
    }
}

/// Whether twist_mux's output (speed, turn rate) is none of our recent commands: true means
/// a higher-priority driver (the joystick, the keyboard) is driving. 1e-3 apart counts.
#[must_use]
pub fn taken_over(ours: impl IntoIterator<Item = (f64, f64)>, theirs: (f64, f64)) -> bool {
    !ours
        .into_iter()
        .any(|(v, w)| (v - theirs.0).abs() <= 1e-3 && (w - theirs.1).abs() <= 1e-3)
}

/// The driver's own limits (config/drive_to_point.yaml).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// m: refuse a B further than this.
    pub max_distance: f64,
    /// s without the global EKF: stop.
    pub pose_timeout: f64,
    /// s without a GPS fix: stop.
    pub gps_timeout: f64,
    /// false only where no GPS runs at all.
    pub require_gps: bool,
    /// s without the watchdog's lock (a heartbeat): stop; 0 where no watchdog runs.
    pub watchdog_timeout: f64,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            max_distance: 200.0,
            pose_timeout: 0.5,
            gps_timeout: 2.0,
            require_gps: true,
            watchdog_timeout: 1.0,
        }
    }
}

/// What the driver knows, this moment. Ages in s; None = never heard.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Now {
    /// Since the last global EKF pose.
    pub pose_age: Option<f64>,
    /// Since the last GPS fix with a fix.
    pub fix_age: Option<f64>,
    /// `/safety/estop` says true.
    pub estop: bool,
    /// The sensor watchdog's lock says true.
    pub locked: bool,
    /// What `/system/source` says we drive.
    pub kind: Kind,
    /// twist_mux passes someone else's commands.
    pub taken_over: bool,
    /// Since the watchdog's lock last arrived.
    pub lock_age: Option<f64>,
}

/// Why the car must stop now ([`stop_reason`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Stop {
    /// `/safety/estop` is on.
    Estop,
    /// The watchdog's lock is on.
    Locked,
    /// The watchdog is silent (s), or never heard (None).
    WatchdogSilent(Option<f64>),
    /// Nothing on `/localization/odom/global`.
    NoPosition,
    /// The position is this old (s).
    StalePosition(f64),
    /// No GPS fix.
    NoFix,
    /// The fix is this old (s).
    StaleFix(f64),
    /// Someone else is driving.
    TakenOver,
}

impl fmt::Display for Stop {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Stop::Estop => f.write_str("the e-stop is on (/safety/estop)"),
            Stop::Locked => f.write_str(
                "the sensor watchdog's lock is on: a sensor is stale, NaN, frozen or jumped",
            ),
            Stop::WatchdogSilent(age) => {
                let heard = match age {
                    None => "never heard".to_string(),
                    Some(a) => format!("silent for {a:.1} s"),
                };
                write!(
                    f,
                    "the sensor watchdog's lock is on: the watchdog is {heard}, so twist_mux \
                     holds every driver below 30"
                )
            }
            Stop::NoPosition => f.write_str("no position: nothing on /localization/odom/global"),
            Stop::StalePosition(a) => write!(f, "the position is stale: {a:.1} s old"),
            Stop::NoFix => f.write_str("no GPS fix on /sensors/gps/fix"),
            Stop::StaleFix(a) => write!(f, "the GPS fix is stale: {a:.1} s old"),
            Stop::TakenOver => {
                f.write_str("someone else is driving (twist_mux passes another driver)")
            }
        }
    }
}

/// Why a goal, or its B, is refused.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Refusal {
    /// A goal's number (its name) is negative or not a number.
    Number(&'static str, f64),
    /// A recording plays.
    Replay,
    /// REAL (true) or nobody announcing (false), and no person confirmed.
    NotConfirmed(bool),
    /// The car must stop.
    Stop(Stop),
    /// B is not a number.
    NotANumber,
    /// B is this far (m), over the limit (m).
    TooFar(f64, f64),
    /// The car is at B already (m).
    AtB(f64),
    /// B is inside the car's tightest turn (its radius, m).
    InsideTurn(f64),
    /// B is this far (m) outside the geofence.
    OutsideGeofence(f64),
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Refusal::Number(name, v) => write!(
                f,
                "{name} must be 0 (the default) or above (got {})",
                py_repr(v)
            ),
            Refusal::Replay => f.write_str(
                "/system/source says REPLAY: a recording plays; there is nothing to drive",
            ),
            Refusal::NotConfirmed(real) => write!(
                f,
                "{}. A person must confirm: pixi run drive-to-point asks you to type \
                 \"{CONFIRM_WORD}\"",
                if real {
                    "/system/source says REAL"
                } else {
                    "nobody announced /system/source: treated as the real car"
                }
            ),
            Refusal::Stop(why) => write!(f, "{why}"),
            Refusal::NotANumber => f.write_str("B is not a number"),
            Refusal::TooFar(d, limit) => write!(
                f,
                "B is {d:.0} m away, over the limit of {} m (drive_to_point.yaml max_distance)",
                fmt_g(limit)
            ),
            Refusal::AtB(d) => write!(f, "the car is at B already ({d:.2} m)"),
            Refusal::InsideTurn(r) => write!(
                f,
                "B is inside the car's tightest turn ({r:.1} m radius): a car that can't \
                 reverse would circle it. Drive further first, or use Nav2"
            ),
            Refusal::OutsideGeofence(d) => write!(
                f,
                "B is {d:.1} m outside the geofence (/safety/geofence/fence): the geofence \
                 would stop the car before it, on the way"
            ),
        }
    }
}

/// Why a goal's speed, acceleration or deceleration is refused (0 = default), or None.
#[must_use]
pub fn check_numbers(cruise: f64, accel: f64, decel: f64) -> Option<Refusal> {
    [
        ("cruise_speed", cruise),
        ("acceleration", accel),
        ("deceleration", decel),
    ]
    .into_iter()
    .find(|&(_, v)| !v.is_finite() || v < 0.0)
    .map(|(name, v)| Refusal::Number(name, v))
}

/// Why a drive may not start, or None.
#[must_use]
pub fn refuse_start(now: &Now, limits: &Limits, confirmed: bool) -> Option<Refusal> {
    match now.kind {
        Kind::Replay => Some(Refusal::Replay),
        Kind::Real | Kind::Unknown if !confirmed => {
            Some(Refusal::NotConfirmed(now.kind == Kind::Real))
        }
        _ => stop_reason(now, limits).map(Refusal::Stop),
    }
}

/// Why the car must stop now, or None.
#[must_use]
pub fn stop_reason(now: &Now, limits: &Limits) -> Option<Stop> {
    if now.estop {
        return Some(Stop::Estop);
    }
    if now.locked {
        return Some(Stop::Locked);
    }
    if limits.watchdog_timeout > 0.0 && now.lock_age.is_none_or(|a| a > limits.watchdog_timeout) {
        return Some(Stop::WatchdogSilent(now.lock_age));
    }
    match now.pose_age {
        None => return Some(Stop::NoPosition),
        Some(a) if a > limits.pose_timeout => return Some(Stop::StalePosition(a)),
        Some(_) => {}
    }
    if limits.require_gps {
        match now.fix_age {
            None => return Some(Stop::NoFix),
            Some(a) if a > limits.gps_timeout => return Some(Stop::StaleFix(a)),
            Some(_) => {}
        }
    }
    now.taken_over.then_some(Stop::TakenOver)
}

/// Why B is refused (too far, here already, inside the turning circle), or None.
#[must_use]
pub fn refuse_target(pose: &Pose, target: Point, limits: &Limits, radius: f64) -> Option<Refusal> {
    if !(target.0.is_finite() && target.1.is_finite()) {
        return Some(Refusal::NotANumber);
    }
    let distance = hypot(target.0 - pose.x, target.1 - pose.y);
    if distance > limits.max_distance {
        return Some(Refusal::TooFar(distance, limits.max_distance));
    }
    if distance < 0.5 {
        return Some(Refusal::AtB(distance));
    }
    (!reachable(pose, target, radius, 0.5)).then_some(Refusal::InsideTurn(radius))
}

/// Why B is refused for the geofence, or None. `depth`: how far B is inside
/// `/safety/geofence/fence` (m, negative outside), or None with no fence heard (the
/// geofence's lock still holds the drive).
#[must_use]
pub fn refuse_outside(depth: Option<f64>) -> Option<Refusal> {
    depth
        .filter(|&d| d < 0.0)
        .map(|d| Refusal::OutsideGeofence(-d))
}

/// True if the car stands still though asked for at least `asked` m/s (0.2 in the node).
#[must_use]
pub fn not_moving(commanded: f64, measured: f64, still: f64, asked: f64) -> bool {
    commanded >= asked && measured.abs() < still
}

/// How close to B counts as there: 1 m with an RTK fix (GBAS), 2 m without.
#[must_use]
pub fn tolerance(fix_status: Option<i8>) -> f64 {
    match fix_status {
        Some(s) if s >= STATUS_GBAS_FIX => RTK_TOLERANCE,
        _ => GPS_TOLERANCE,
    }
}

/// True if the person typed the confirmation word exactly (spaces aside).
#[must_use]
pub fn confirmed(typed: Option<&str>) -> bool {
    typed.is_some_and(|t| t.trim() == CONFIRM_WORD)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn now() -> Now {
        Now {
            pose_age: Some(0.1),
            fix_age: Some(0.1),
            estop: false,
            locked: false,
            kind: Kind::Sim,
            taken_over: false,
            lock_age: Some(0.1),
        }
    }

    #[test]
    fn a_fresh_sim_starts_and_the_real_car_asks() {
        let limits = Limits::default();
        assert_eq!(refuse_start(&now(), &limits, false), None);
        let real = Now {
            kind: Kind::Real,
            ..now()
        };
        assert_eq!(
            refuse_start(&real, &limits, false),
            Some(Refusal::NotConfirmed(true))
        );
        assert_eq!(refuse_start(&real, &limits, true), None);
    }

    #[test]
    fn sources_merge_as_the_actuator_tests_do() {
        assert_eq!(source_kind([]), Kind::Unknown);
        assert_eq!(source_kind([1, 0]), Kind::Real);
        assert_eq!(source_kind([1, 3, 2]), Kind::Replay);
        assert_eq!(source_kind([1, 7]), Kind::Sim);
        assert!(confirmed(Some(" drive\n")));
        assert!(!confirmed(Some("Drive")));
    }
}
