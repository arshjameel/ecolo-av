//! The boot self-test's logic, with no ROS (TODO 255). Ben, 19 Nov: "Make sure we have
//! control over everything with a basic boot self test."
//!
//! ```text
//!  the sensor watchdog's line ─┐
//!  each lock's line ───────────┤
//!  the stop zone (graph) ──────┼─▶ SelfTest ─▶ Verdict: READY, or NOT READY and why
//!  the spine's status ─────────┤                 (spine_bridge refuses /spine/arm until READY)
//!  stamped sensors (clocks) ───┘
//! ```
//!
//! | Check | Passes when |
//! |---|---|
//! | `sensors` | the sensor watchdog spoke within `line_timeout` and says OK or WARN, not STOP: every sensor it watches is at its rate (its own rules) |
//! | `actuators` | `status`: `/vehicle/status` within `status_timeout`, with no driver fault (steering, brake, throttle, CAN); `link`: the UART bridge writes frames (it can't hear the ESP32 without the uplink); `none`: not checked |
//! | each lock | `watchdog`, `geofence`, `output_gate`: its line within `line_timeout` (alive; whatever it says, it holds what it must); `stop_zone`: a publisher on its state (it speaks only on commands) |
//! | each clock | a stamped message from that source within `line_timeout`, its stamp within `max_clock_offset` of this computer's clock |
//!
//! READY once every check has passed for `settle` s without a break; any failure is NOT
//! READY at once. It only reads: it never moves anything, and never stands between a
//! person and the e-stop.

#![warn(missing_docs)]

use std::fmt;
use std::str::FromStr;

/// The fault bits that mean an actuator isn't answering (`VehicleStatus.FAULT_*`):
/// the steering and brake drivers, the throttle DAC, the CAN bus.
pub const DRIVER_FAULTS: u16 = 8 | 16 | 32 | 128;

/// The locks it can watch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Lock {
    /// The sensor watchdog (`/safety/sensor_watchdog/state`).
    Watchdog,
    /// The geofence (`/safety/geofence/state`).
    Geofence,
    /// The output mux, the safety gate (`/safety/output_mux/state`).
    OutputGate,
    /// The stop zone: speaks only on commands, so alive means it publishes its state.
    StopZone,
}

impl Lock {
    /// Every lock, in the order the verdict lists them.
    pub const ALL: [Lock; 4] = [
        Lock::Watchdog,
        Lock::Geofence,
        Lock::OutputGate,
        Lock::StopZone,
    ];

    /// Its name in the `locks` parameter and the verdict.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Lock::Watchdog => "watchdog",
            Lock::Geofence => "geofence",
            Lock::OutputGate => "output_gate",
            Lock::StopZone => "stop_zone",
        }
    }
}

/// How the actuators are checked (the `actuators` parameter).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Actuators {
    /// `/vehicle/status`, fresh, no driver fault: the CAN spine (or the ESP32's uplink).
    #[default]
    Status,
    /// The UART bridge writes frames (`/control/car/link_frame`): alive, but it can't hear
    /// the ESP32 (today's car, the uplink off).
    Link,
    /// Not checked.
    Unchecked,
}

/// The clocks it can compare, by the stamped topic that carries them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ClockSource {
    /// `/sensors/imu/data`: the localization Pi.
    Imu,
    /// `/sensors/gps/fix`: the localization Pi.
    Gps,
    /// `/vehicle/odom`: the wheels' computer.
    Odom,
}

impl ClockSource {
    /// Every source, in the order the verdict lists them.
    pub const ALL: [ClockSource; 3] = [ClockSource::Imu, ClockSource::Gps, ClockSource::Odom];

    /// Its name in the `clocks` parameter and the verdict.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            ClockSource::Imu => "imu",
            ClockSource::Gps => "gps",
            ClockSource::Odom => "odom",
        }
    }
}

/// A word the parameters don't know.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownName {
    what: &'static str,
    name: String,
    known: String,
}

impl fmt::Display for UnknownName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} '{}': use {}", self.what, self.name, self.known)
    }
}

impl std::error::Error for UnknownName {}

impl FromStr for Lock {
    type Err = UnknownName;

    fn from_str(s: &str) -> Result<Self, UnknownName> {
        Lock::ALL
            .into_iter()
            .find(|l| l.name() == s)
            .ok_or_else(|| UnknownName {
                what: "lock",
                name: s.into(),
                known: names(Lock::ALL.map(Lock::name)),
            })
    }
}

impl FromStr for ClockSource {
    type Err = UnknownName;

    fn from_str(s: &str) -> Result<Self, UnknownName> {
        ClockSource::ALL
            .into_iter()
            .find(|c| c.name() == s)
            .ok_or_else(|| UnknownName {
                what: "clock",
                name: s.into(),
                known: names(ClockSource::ALL.map(ClockSource::name)),
            })
    }
}

impl FromStr for Actuators {
    type Err = UnknownName;

    fn from_str(s: &str) -> Result<Self, UnknownName> {
        match s {
            "status" => Ok(Actuators::Status),
            "link" => Ok(Actuators::Link),
            "none" => Ok(Actuators::Unchecked),
            other => Err(UnknownName {
                what: "actuators",
                name: other.into(),
                known: "status, link or none".into(),
            }),
        }
    }
}

fn names<const N: usize>(all: [&str; N]) -> String {
    all.join(", ")
}

/// What it checks, and its limits (seconds).
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// How the actuators are checked.
    pub actuators: Actuators,
    /// The locks that must be alive.
    pub locks: Vec<Lock>,
    /// The clocks that must agree with this computer's.
    pub clocks: Vec<ClockSource>,
    /// s every check must pass, without a break, before READY.
    pub settle: f64,
    /// s: an older line (or stamped message) counts as silence.
    pub line_timeout: f64,
    /// s: an older `/vehicle/status` (or link frame) counts as silence.
    pub status_timeout: f64,
    /// s: the most a stamp may differ from this computer's clock.
    pub max_clock_offset: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            actuators: Actuators::Status,
            locks: vec![Lock::Watchdog, Lock::Geofence, Lock::OutputGate],
            clocks: Vec::new(),
            settle: 2.0,
            line_timeout: 1.0,
            status_timeout: 0.5,
            max_clock_offset: 0.5,
        }
    }
}

/// One check's result: its name, and why it fails (None: it passes).
#[derive(Debug, Clone, PartialEq)]
pub struct Outcome {
    /// `sensors`, `actuators`, a lock's name, or `clock <source>`.
    pub name: String,
    /// Why it fails; None if it passes.
    pub why: Option<String>,
}

impl Outcome {
    /// True if it passes.
    #[must_use]
    pub fn ok(&self) -> bool {
        self.why.is_none()
    }
}

/// The answer: READY or not, and each check.
#[derive(Debug, Clone, PartialEq)]
pub struct Verdict {
    /// Every check passed for `settle` s.
    pub ready: bool,
    /// Every check, in order: sensors, actuators, the locks, the clocks.
    pub outcomes: Vec<Outcome>,
    /// s every check has passed so far (0 while one fails).
    pub passing_for: f64,
}

impl Verdict {
    /// One line for people: `READY`, or `NOT READY - ` and each failing check and why.
    #[must_use]
    pub fn line(&self) -> String {
        if self.ready {
            return "READY".into();
        }
        let failing: Vec<String> = self
            .outcomes
            .iter()
            .filter_map(|o| o.why.as_ref().map(|why| format!("{} ({why})", o.name)))
            .collect();
        if failing.is_empty() {
            format!(
                "NOT READY - all passing for {:.1} s, settling",
                self.passing_for
            )
        } else {
            format!("NOT READY - {}", failing.join("; "))
        }
    }
}

/// The last of something heard, and when (s, this computer's steady clock).
type Heard<T> = Option<(T, f64)>;

/// The self-test's state: feed it what it hears, ask for the verdict.
#[derive(Debug, Clone)]
pub struct SelfTest {
    cfg: Config,
    watchdog: Heard<String>,
    locks: Vec<(Lock, Heard<()>)>,
    stop_zone_publishers: usize,
    status: Heard<u16>,
    link: Heard<()>,
    clocks: Vec<(ClockSource, Heard<f64>)>,
    passing_since: Option<f64>,
}

impl SelfTest {
    /// A self-test that has heard nothing: NOT READY.
    #[must_use]
    pub fn new(cfg: Config) -> Self {
        SelfTest {
            locks: cfg.locks.iter().map(|&l| (l, None)).collect(),
            clocks: cfg.clocks.iter().map(|&c| (c, None)).collect(),
            cfg,
            watchdog: None,
            stop_zone_publishers: 0,
            status: None,
            link: None,
            passing_since: None,
        }
    }

    /// What it checks.
    #[must_use]
    pub fn config(&self) -> &Config {
        &self.cfg
    }

    /// The sensor watchdog's line (`/safety/sensor_watchdog/state`), heard at `now`. It is
    /// also the watchdog lock's sign of life.
    pub fn watchdog_line(&mut self, line: &str, now: f64) {
        self.watchdog = Some((line.into(), now));
        self.lock_heard(Lock::Watchdog, now);
    }

    /// A lock's line, heard at `now`: it is alive.
    pub fn lock_heard(&mut self, lock: Lock, now: f64) {
        for (l, heard) in &mut self.locks {
            if *l == lock {
                *heard = Some(((), now));
            }
        }
    }

    /// How many publishers the stop zone's state has (the ROS graph).
    pub fn stop_zone_publishers(&mut self, count: usize) {
        self.stop_zone_publishers = count;
    }

    /// `/vehicle/status`'s fault bits, heard at `now`.
    pub fn vehicle_status(&mut self, faults: u16, now: f64) {
        self.status = Some((faults, now));
    }

    /// A link frame from the UART bridge, heard at `now`.
    pub fn link_frame(&mut self, now: f64) {
        self.link = Some(((), now));
    }

    /// A stamped message from `source`: its stamp (s, the sender's clock) heard at `now`
    /// (s, this computer's clock; the same epoch).
    pub fn stamp(&mut self, source: ClockSource, stamp: f64, now: f64) {
        for (c, heard) in &mut self.clocks {
            if *c == source {
                *heard = Some((now - stamp, now));
            }
        }
    }

    /// The verdict at `now`: each check, and READY once all have passed for `settle` s.
    pub fn verdict(&mut self, now: f64) -> Verdict {
        let mut outcomes = vec![
            Outcome {
                name: "sensors".into(),
                why: self.sensors(now),
            },
            Outcome {
                name: "actuators".into(),
                why: self.actuators(now),
            },
        ];
        outcomes.extend(self.locks.iter().map(|&(lock, heard)| Outcome {
            name: lock.name().into(),
            why: self.lock(lock, heard, now),
        }));
        outcomes.extend(self.clocks.iter().map(|&(source, heard)| Outcome {
            name: format!("clock {}", source.name()),
            why: self.clock(heard, now),
        }));
        let passing = outcomes.iter().all(Outcome::ok);
        if !passing {
            self.passing_since = None;
        } else if self.passing_since.is_none() {
            self.passing_since = Some(now);
        }
        let passing_for = self.passing_since.map_or(0.0, |since| now - since);
        Verdict {
            ready: passing && passing_for >= self.cfg.settle,
            outcomes,
            passing_for,
        }
    }

    fn silent(&self, at: Option<f64>, timeout: f64, now: f64, what: &str) -> Option<String> {
        match at {
            None => Some(format!("no {what} yet")),
            Some(at) if now - at > timeout => Some(format!("{what} quiet for {:.1} s", now - at)),
            Some(_) => None,
        }
    }

    fn sensors(&self, now: f64) -> Option<String> {
        let at = self.watchdog.as_ref().map(|(_, at)| *at);
        if let Some(why) = self.silent(
            at,
            self.cfg.line_timeout,
            now,
            "word from the sensor watchdog",
        ) {
            return Some(why);
        }
        let line = self.watchdog.as_ref().map_or("", |(line, _)| line.as_str());
        (line.starts_with("STOP") || line.is_empty()).then(|| line.to_string())
    }

    fn actuators(&self, now: f64) -> Option<String> {
        match self.cfg.actuators {
            Actuators::Unchecked => None,
            Actuators::Link => self.silent(
                self.link.map(|(_, at)| at),
                self.cfg.status_timeout,
                now,
                "frame written to the ESP32",
            ),
            Actuators::Status => {
                if let Some(why) = self.silent(
                    self.status.map(|(_, at)| at),
                    self.cfg.status_timeout,
                    now,
                    "/vehicle/status",
                ) {
                    return Some(why);
                }
                let faults = self.status.map_or(0, |(f, _)| f) & DRIVER_FAULTS;
                (faults != 0)
                    .then(|| format!("driver faults 0x{faults:03X}: {}", fault_names(faults)))
            }
        }
    }

    fn lock(&self, lock: Lock, heard: Heard<()>, now: f64) -> Option<String> {
        if lock == Lock::StopZone {
            return (self.stop_zone_publishers == 0).then(|| "not running".to_string());
        }
        self.silent(
            heard.map(|(_, at)| at),
            self.cfg.line_timeout,
            now,
            "word from it",
        )
    }

    fn clock(&self, heard: Heard<f64>, now: f64) -> Option<String> {
        if let Some(why) = self.silent(
            heard.map(|(_, at)| at),
            self.cfg.line_timeout,
            now,
            "stamped message",
        ) {
            return Some(why);
        }
        let offset = heard.map_or(0.0, |(offset, _)| offset);
        (offset.is_nan() || offset.abs() > self.cfg.max_clock_offset).then(|| {
            format!(
                "its stamps are {offset:+.2} s from this clock (more than {:.2} s): chrony?",
                self.cfg.max_clock_offset
            )
        })
    }
}

/// The driver faults' names, as `VehicleStatus` says them.
fn fault_names(faults: u16) -> String {
    [
        (8, "steering driver"),
        (16, "brake driver"),
        (32, "throttle DAC"),
        (128, "CAN bus"),
    ]
    .iter()
    .filter(|(bit, _)| faults & bit != 0)
    .map(|(_, name)| *name)
    .collect::<Vec<_>>()
    .join(", ")
}
