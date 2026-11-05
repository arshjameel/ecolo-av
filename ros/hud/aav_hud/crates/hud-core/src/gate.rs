//! May the car move? One answer for the safety drivers: GO, WAIT or STOP, with one
//! colour, one icon and the one main reason.
//!
//! | What says no | Heard on | What it stops |
//! |---|---|---|
//! | the car's e-stop | `/vehicle/status` (`estop_active`) | everything, until a laptop clears it |
//! | the output mux's gate (aav_io_mux) | `/safety/output_mux/state` | every command to the car, a person's too |
//! | the sensor watchdog (aav_safety) | `/safety/sensor_watchdog/state` | Nav2 and the AI slot; people still drive |
//!
//! The first in [`verdict`]'s order is the reason: what a person must clear first, then
//! what clears by itself. Both states are one line of text for people; [`Gate::parse`]
//! and [`Sensors::parse`] read them, and anything they don't know is shown as it is.

use std::time::Duration;

use crate::health::Health;

/// The output mux says its state twice a second: this long without it, it's silent.
pub const GATE_QUIET: Duration = Duration::from_secs(2);

/// The colour and the icon.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Light {
    /// Green, a tick: the car may move.
    Go,
    /// Amber, a pause sign: shut for now, opens by itself.
    Wait,
    /// Red, a stop sign: shut until something is fixed, or a person clears it.
    Stop,
    /// Grey, a question mark: no gate heard.
    Unknown,
}

/// What the panel says.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Verdict {
    pub light: Light,
    /// One or two short words, big: GO, WAIT, STOP, AUTO STOP, NO GATE.
    pub headline: &'static str,
    /// The one main reason, a few words: "E-STOP ON", "GPS is SIM", "GPS STALE".
    pub reason: String,
    /// What a person can do, smaller: "clear: pixi run clear gate".
    pub hint: &'static str,
}

impl Verdict {
    fn new(
        light: Light,
        headline: &'static str,
        reason: impl Into<String>,
        hint: &'static str,
    ) -> Self {
        Self {
            light,
            headline,
            reason: reason.into(),
            hint,
        }
    }
}

/// The output mux's state (`OPEN - car: every source REAL`, `WAIT - ...`, `STOP - ...`),
/// each with its first reason, made short.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Gate {
    /// Passing ego's commands on: "all REAL".
    Open(String),
    /// Shut, not latched: it opens by itself.
    Wait(String),
    /// Shut and latched, until a person calls `/safety/output_mux/clear`.
    Stop(String),
    /// A line this HUD doesn't know: shown as it is.
    Other(String),
}

impl Gate {
    /// Reads the output mux's line (aav_io_mux `provenance.Gate.state`).
    pub fn parse(text: &str) -> Self {
        if let Some(rest) = text.strip_prefix("OPEN - ") {
            let what = rest
                .rsplit_once("every source ")
                .map_or_else(|| "open".to_owned(), |(_, kind)| format!("all {kind}"));
            Self::Open(what)
        } else if let Some(rest) = text.strip_prefix("WAIT - ") {
            // "car: why; why"
            let why = rest.split_once(": ").map_or(rest, |(_, why)| why);
            Self::Wait(short_wait(first(why)))
        } else if let Some(rest) = text.strip_prefix("STOP - ") {
            // "target car needs REAL - why; why - latched: /safety/output_mux/clear"
            let rest = rest.split_once(" - ").map_or(rest, |(_, r)| r);
            let why = rest.rsplit_once(" - latched").map_or(rest, |(why, _)| why);
            Self::Stop(short_problem(first(why)))
        } else {
            Self::Other(text.to_owned())
        }
    }
}

/// The sensor watchdog's line (`OK`, `WARN: ...`, `STOP: gps STALE (...); ...`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Sensors {
    /// Nothing stops the drivers (a warning may be on: the camera's banner shows it).
    Drive,
    /// Nav2 and the AI slot are held: the first input that stops them, and its state.
    Held { input: String, state: String },
}

impl Sensors {
    /// Reads the watchdog's line (aav_safety `watchdog.Watchdog.summary`: the inputs that
    /// stop come first).
    pub fn parse(text: &str) -> Self {
        let Some(rest) = text.strip_prefix("STOP: ") else {
            return Self::Drive;
        };
        let entry = first(rest);
        let entry = entry.split_once(" (").map_or(entry, |(e, _)| e);
        let (input, state) = entry.rsplit_once(' ').unwrap_or((entry, ""));
        Self::Held {
            input: input.to_owned(),
            state: state.to_owned(),
        }
    }

    /// Whether only a person clears it (`/safety/sensor_watchdog/clear`): a GPS jump,
    /// wheels that disagreed, a HUD that dropped out.
    pub fn is_latched(&self) -> bool {
        matches!(self, Self::Held { state, .. } if ["JUMP", "DISAGREE", "LOST"].contains(&state.as_str()))
    }
}

/// The answer, from what was heard. `estop_on`: a fresh `/vehicle/status` says the
/// e-stop is on. `gate` and `watchdog`: each topic's health, and its last line.
///
/// The order: the e-stop; the gate latched or silent (nothing reaches the car); the
/// watchdog holding Nav2; the gate waiting; no gate heard; then GO.
pub fn verdict(
    estop_on: bool,
    gate: (Health, Option<&str>),
    watchdog: (Health, Option<&str>),
) -> Verdict {
    use Light::{Go, Stop, Unknown, Wait};
    if estop_on {
        return Verdict::new(Stop, "STOP", "E-STOP ON", "clear: pixi run clear estop");
    }
    let gate_state = match gate {
        (Health::Ok(_), Some(text)) => Some(Gate::parse(text)),
        _ => None,
    };
    if let Some(Gate::Stop(why)) = &gate_state {
        return Verdict::new(Stop, "STOP", why.as_str(), "clear: pixi run clear gate");
    }
    if let Health::Stale(_) = gate.0 {
        return Verdict::new(Stop, "STOP", "gate silent", "nothing reaches the car");
    }
    match watchdog {
        (Health::Stale(_), _) => {
            return Verdict::new(
                Stop,
                "AUTO STOP",
                "watchdog silent",
                "people can still drive",
            );
        }
        (Health::Ok(_), Some(text)) => {
            let sensors = Sensors::parse(text);
            if let Sensors::Held { input, state } = &sensors {
                let hint = if sensors.is_latched() {
                    "clear: pixi run clear watchdog"
                } else {
                    "people can still drive"
                };
                let reason = format!("{} {state}", name(input));
                return Verdict::new(Stop, "AUTO STOP", reason.trim_end(), hint);
            }
        }
        _ => {}
    }
    match gate_state {
        Some(Gate::Wait(why)) => Verdict::new(Wait, "WAIT", why, "opens by itself"),
        Some(Gate::Other(text)) => Verdict::new(Wait, "WAIT", text, ""),
        Some(Gate::Open(what)) => Verdict::new(Go, "GO", what, "the car may move"),
        Some(Gate::Stop(_)) | None => Verdict::new(
            Unknown,
            "NO GATE",
            "no output mux heard",
            "nothing reaches the car",
        ),
    }
}

/// The first of several reasons, joined by "; ".
fn first(reasons: &str) -> &str {
    reasons.split("; ").next().unwrap_or(reasons).trim()
}

/// An input's or a sensor's name, for the screen: GPS, WHEELS, CAMERA FRONT LEFT.
fn name(raw: &str) -> String {
    match raw {
        "odom" | "odometry" => "WHEELS".to_owned(),
        other => other.replace('_', " ").to_uppercase(),
    }
}

/// One of the output mux's problems (aav_io_mux `provenance`), in a few words.
fn short_problem(problem: &str) -> String {
    let before_colon = || name(problem.split_once(':').map_or(problem, |(n, _)| n));
    if let Some(rest) = problem.strip_prefix("the sensor mux checks for ") {
        let kind = rest.split(',').next().unwrap_or(rest);
        format!("sensor mux wants {kind}")
    } else if problem == "the sensor mux says no" {
        "sensor mux says no".to_owned()
    } else if problem.contains(" publishers on ") {
        format!("2 sources: {}", before_colon())
    } else if problem.contains("nobody announces") {
        format!("unannounced: {}", before_colon())
    } else if let Some((sensor, rest)) = problem.split_once(" is ") {
        let kind = rest.split(' ').next().unwrap_or(rest);
        format!("{} is {kind}", name(sensor))
    } else {
        problem.to_owned()
    }
}

/// One of the output mux's reasons to wait, in a few words.
fn short_wait(wait: &str) -> String {
    if wait.starts_with("waiting for the sensor mux") {
        "no sensor mux yet".to_owned()
    } else if wait.starts_with("the sensor mux is quiet") {
        "sensor mux quiet".to_owned()
    } else if let Some(doubt) = wait.strip_prefix("unsure: ") {
        format!(
            "unsure: {}",
            name(doubt.split_once(':').map_or(doubt, |(n, _)| n))
        )
    } else if wait.contains("settling") {
        "settling".to_owned()
    } else {
        wait.to_owned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OK: Health = Health::Ok(Duration::ZERO);
    const STALE: Health = Health::Stale(Duration::from_secs(3));
    const OPEN: &str = "OPEN - car: every source REAL";

    // The output mux's lines, as aav_io_mux's own tests have them (test_provenance.py).
    #[test]
    fn the_gates_lines_read() {
        assert_eq!(Gate::parse(OPEN), Gate::Open("all REAL".into()));
        assert_eq!(
            Gate::parse("OPEN - gazebo: every source SIM"),
            Gate::Open("all SIM".into())
        );
        let stop = "STOP - target car needs REAL - camera_front_left is SIM \
                    (gazebo_adapter) - latched: /safety/output_mux/clear";
        assert_eq!(
            Gate::parse(stop),
            Gate::Stop("CAMERA FRONT LEFT is SIM".into())
        );
        let two = "STOP - target car needs REAL - gps is SIM (gazebo_adapter); imu is SIM \
                   (gazebo_imu) - latched: /safety/output_mux/clear";
        assert_eq!(Gate::parse(two), Gate::Stop("GPS is SIM".into()));
        let wrong = "STOP - target car needs REAL - the sensor mux checks for SIM, target \
                     car needs REAL - latched: /safety/output_mux/clear";
        assert_eq!(
            Gate::parse(wrong),
            Gate::Stop("sensor mux wants SIM".into())
        );
        let pubs = "STOP - target car needs REAL - imu: 2 publishers on /sensors/imu/data_raw \
                    (a, b): two sources at once - latched: /safety/output_mux/clear";
        assert_eq!(Gate::parse(pubs), Gate::Stop("2 sources: IMU".into()));
        assert_eq!(
            Gate::parse("WAIT - car: waiting for the sensor mux (/system/sensor_mux/state)"),
            Gate::Wait("no sensor mux yet".into())
        );
        assert_eq!(
            Gate::parse("WAIT - car: the sensor mux is quiet (3.4 s)"),
            Gate::Wait("sensor mux quiet".into())
        );
        assert_eq!(
            Gate::parse("WAIT - turtle: unsure: gps: /sensors/gps/fix from ublox, which nobody announces on /system/source: a real driver?"),
            Gate::Wait("unsure: GPS".into())
        );
        assert_eq!(
            Gate::parse("WAIT - car: all REAL, settling for 2 s"),
            Gate::Wait("settling".into())
        );
        // As the live output mux said it next to the turtle (24 Oct), latched by a node
        // announcing a REAL GPS.
        let live = "STOP - target turtle needs SIM - gps is REAL (fake_gps) - latched: \
                    /safety/output_mux/clear";
        assert_eq!(Gate::parse(live), Gate::Stop("GPS is REAL".into()));
        assert_eq!(Gate::parse("hello"), Gate::Other("hello".into()));
    }

    #[test]
    fn the_watchdogs_lines_read() {
        assert_eq!(Sensors::parse("OK"), Sensors::Drive);
        assert_eq!(
            Sensors::parse("WARN: radar NEVER (nothing yet)"),
            Sensors::Drive
        );
        let held = Sensors::parse(
            "STOP: gps STALE (nothing for 2.3 s (more than 2 s)); radar NEVER (nothing yet)",
        );
        assert_eq!(
            held,
            Sensors::Held {
                input: "gps".into(),
                state: "STALE".into()
            }
        );
        assert!(!held.is_latched());
        let lost = Sensors::parse("STOP: hud LOST (went STALE 12 s ago, OK now: clear it)");
        assert!(lost.is_latched());
    }

    #[test]
    fn go_only_when_the_gate_is_open_and_nothing_else_says_no() {
        let v = verdict(false, (OK, Some(OPEN)), (OK, Some("OK")));
        assert_eq!(
            (v.light, v.headline, v.reason.as_str()),
            (Light::Go, "GO", "all REAL")
        );
        // A warning doesn't hold the drivers: still GO.
        let v = verdict(false, (OK, Some(OPEN)), (OK, Some("WARN: radar NEVER")));
        assert_eq!(v.light, Light::Go);
        // No watchdog in this run (the turtle): the gate decides.
        let v = verdict(false, (OK, Some(OPEN)), (Health::Never, None));
        assert_eq!(v.light, Light::Go);
    }

    #[test]
    fn the_estop_comes_first() {
        let v = verdict(true, (OK, Some(OPEN)), (OK, Some("STOP: gps STALE (x)")));
        assert_eq!((v.light, v.reason.as_str()), (Light::Stop, "E-STOP ON"));
        assert!(v.hint.contains("clear estop"));
    }

    #[test]
    fn a_latched_gate_comes_before_the_watchdog() {
        let stop = "STOP - target car needs REAL - gps is SIM (gazebo_adapter) - latched: x";
        let v = verdict(false, (OK, Some(stop)), (OK, Some("STOP: imu STALE (x)")));
        assert_eq!((v.light, v.headline), (Light::Stop, "STOP"));
        assert_eq!(v.reason, "GPS is SIM");
        assert!(v.hint.contains("clear gate"));
    }

    #[test]
    fn a_silent_gate_lets_nothing_through() {
        let v = verdict(false, (STALE, Some(OPEN)), (OK, Some("OK")));
        assert_eq!((v.light, v.reason.as_str()), (Light::Stop, "gate silent"));
    }

    #[test]
    fn the_watchdog_holds_the_automatic_drivers() {
        let v = verdict(
            false,
            (OK, Some(OPEN)),
            (OK, Some("STOP: gps STALE (x); radar NEVER")),
        );
        assert_eq!((v.light, v.headline), (Light::Stop, "AUTO STOP"));
        assert_eq!(v.reason, "GPS STALE");
        assert_eq!(v.hint, "people can still drive");
        let v = verdict(
            false,
            (OK, Some(OPEN)),
            (OK, Some("STOP: hud LOST (went STALE)")),
        );
        assert_eq!(v.reason, "HUD LOST");
        assert!(v.hint.contains("clear watchdog"));
        let v = verdict(
            false,
            (OK, Some(OPEN)),
            (OK, Some("STOP: odom DISAGREE (x)")),
        );
        assert_eq!(v.reason, "WHEELS DISAGREE");
        let v = verdict(false, (OK, Some(OPEN)), (STALE, Some("OK")));
        assert_eq!(
            (v.headline, v.reason.as_str()),
            ("AUTO STOP", "watchdog silent")
        );
    }

    #[test]
    fn a_waiting_gate_opens_by_itself() {
        let wait = "WAIT - car: the sensor mux is quiet (3.4 s)";
        let v = verdict(false, (OK, Some(wait)), (OK, Some("OK")));
        assert_eq!((v.light, v.headline), (Light::Wait, "WAIT"));
        assert_eq!(v.reason, "sensor mux quiet");
    }

    #[test]
    fn no_gate_heard_is_not_go() {
        let v = verdict(false, (Health::Never, None), (OK, Some("OK")));
        assert_eq!((v.light, v.headline), (Light::Unknown, "NO GATE"));
        // An unknown line from a newer output mux: shown as it is, never GO.
        let v = verdict(false, (OK, Some("MAYBE - car")), (OK, Some("OK")));
        assert_eq!((v.light, v.reason.as_str()), (Light::Wait, "MAYBE - car"));
    }
}
