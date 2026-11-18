//! Where the data comes from, and whether a target may act on it. No ROS here: the Rust
//! twin of aav_io_mux's `provenance.py`, rule for rule, word for word.
//!
//! | Rule | Where |
//! |---|---|
//! | The sources and targets, and each target's topic and source | [`Kind`], [`Target`], [`SENSORS`] |
//! | Forget an announcement once its announcer has gone; doubt one not heard yet | [`Announcers`] |
//! | Any announcement that isn't the source this run asked for | [`foreign`] |
//! | Each sensor: one publisher, from the source asked for | [`check_sensors`] |
//! | What the ROS graph suggests counts once it lasts; a missing announcement after the start-up grace | [`Hold`] |
//! | Pass the target's commands on, wait, or latch a stop | [`Gate`] |
//!
//! Every rule errs towards "this is the real car, and something is wrong": an unknown
//! source number is never REAL, and a silent sensor mux closes the gate. Evidence (an
//! announcement of another source) latches a stop; doubt (a silent sensor mux, a graph
//! finding still young) only holds the gate shut. Times are seconds on a steady clock.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::fmt;
use std::str::FromStr;

use ego_core::fmt_g;

/// Where a sensor's data comes from: `aav_interfaces/SensorSource`'s values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    /// A real device on the real car (0).
    Real,
    /// A simulator: Ravensim, turtlesim, CARLA (1).
    Sim,
    /// The bench's stand-ins (2).
    Bench,
    /// A recording, played back (3).
    Replay,
    /// A number newer than this code: never REAL.
    Unknown(u8),
}

impl Kind {
    /// The message's byte as a kind.
    #[must_use]
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0 => Self::Real,
            1 => Self::Sim,
            2 => Self::Bench,
            3 => Self::Replay,
            n => Self::Unknown(n),
        }
    }

    /// The kind as the message's byte.
    #[must_use]
    pub fn byte(self) -> u8 {
        match self {
            Self::Real => 0,
            Self::Sim => 1,
            Self::Bench => 2,
            Self::Replay => 3,
            Self::Unknown(n) => n,
        }
    }

    /// `source:=real|sim|bench|replay` on the sensor mux.
    ///
    /// # Errors
    /// [`ChoiceError::Source`] for any other name.
    pub fn from_source_name(name: &str) -> Result<Self, ChoiceError> {
        SOURCES
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, k)| *k)
            .ok_or_else(|| ChoiceError::Source(name.to_string()))
    }
}

impl fmt::Display for Kind {
    /// REAL, SIM, BENCH, REPLAY; a newer number as UNKNOWN(n).
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Real => f.write_str("REAL"),
            Self::Sim => f.write_str("SIM"),
            Self::Bench => f.write_str("BENCH"),
            Self::Replay => f.write_str("REPLAY"),
            Self::Unknown(n) => write!(f, "UNKNOWN({n})"),
        }
    }
}

/// `source:=` names, in the Python's order.
pub const SOURCES: [(&str, Kind); 4] = [
    ("real", Kind::Real),
    ("sim", Kind::Sim),
    ("bench", Kind::Bench),
    ("replay", Kind::Replay),
];

/// `target:=` on the output mux: who plays the car.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Target {
    /// The real car: its data must be REAL.
    Car,
    /// Ravensim.
    Sim,
    /// turtlesim.
    Turtle,
    /// CARLA.
    Carla,
    /// The bench's spine_sim.
    Bench,
}

impl Target {
    /// Every target, in the Python's order (`provenance.TARGETS`).
    pub const ALL: [Target; 5] = [
        Target::Car,
        Target::Sim,
        Target::Turtle,
        Target::Carla,
        Target::Bench,
    ];

    /// Its name, as `target:=` takes it.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Car => "car",
            Self::Sim => "sim",
            Self::Turtle => "turtle",
            Self::Carla => "carla",
            Self::Bench => "bench",
        }
    }

    /// The source its data must come from.
    #[must_use]
    pub fn source(self) -> Kind {
        match self {
            Self::Car => Kind::Real,
            Self::Sim | Self::Turtle | Self::Carla => Kind::Sim,
            Self::Bench => Kind::Bench,
        }
    }

    /// The topic the output mux sends this target's commands on.
    #[must_use]
    pub fn command_topic(self) -> String {
        format!("/control/{}/actuator_cmd", self.as_str())
    }

    /// The heartbeat's topic, beside the command topic.
    #[must_use]
    pub fn heartbeat_topic(self) -> String {
        format!("/control/{}/heartbeat", self.as_str())
    }
}

impl fmt::Display for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl FromStr for Target {
    type Err = ChoiceError;

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|t| t.as_str() == name)
            .ok_or_else(|| ChoiceError::Target(name.to_string()))
    }
}

/// A launch's choice the muxes refuse, in the Python's words.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChoiceError {
    /// `source:=` isn't real, sim, bench or replay.
    Source(String),
    /// `target:=` isn't car, sim, turtle, carla or bench.
    Target(String),
    /// Screens for the real car: a test-only rule.
    ScreensForCar(Target),
}

impl fmt::Display for ChoiceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Source(s) => {
                let names: Vec<&str> = SOURCES.iter().map(|(n, _)| *n).collect();
                write!(f, "source:={s}: use one of {}", names.join(", "))
            }
            Self::Target(t) => {
                let names: Vec<&str> = Target::ALL.iter().map(|t| t.as_str()).collect();
                write!(f, "target:={t}: use one of {}", names.join(", "))
            }
            Self::ScreensForCar(t) => write!(f, "target:={t} takes no screens: a test-only rule"),
        }
    }
}

impl std::error::Error for ChoiceError {}

/// The sensor topics the sensor mux watches: the raw data, as each driver or stand-in
/// publishes it. The derived topics (/sensors/imu/data, the EKFs) follow from these.
pub const SENSORS: [(&str, &str); 11] = [
    ("imu", "/sensors/imu/data_raw"),
    ("gps", "/sensors/gps/fix"),
    ("camera_front_left", "/sensors/camera/front_left/image_raw"),
    (
        "camera_front_right",
        "/sensors/camera/front_right/image_raw",
    ),
    ("camera_rear", "/sensors/camera/rear/image_raw"),
    ("radar", "/sensors/radar/points"),
    ("lidar", "/sensors/lidar/points"),
    ("odometry", "/vehicle/odom"),
    ("wheels", "/vehicle/wheels"),
    ("vehicle", "/vehicle/status"),
    ("battery", "/vehicle/battery"),
];

/// One sensor in one /system/source announcement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Entry {
    /// The node that says so.
    pub announcer: String,
    /// imu, gps, camera_front_left, actuators...
    pub name: String,
    /// Its main topic; empty if it has none.
    pub topic: String,
    /// Where it comes from.
    pub kind: Kind,
}

#[derive(Debug, Clone)]
struct Heard {
    announcer: String,
    at: f64,
    entries: Vec<Entry>,
}

/// /system/source as heard, against the nodes the ROS graph shows publishing it.
///
/// | Announcer | Its entries |
/// |---|---|
/// | heard, and in the graph | count |
/// | heard, not in the graph yet | count, for up to `grace` s |
/// | heard, seen in the graph, now gone | forgotten |
/// | in the graph, not heard yet | a doubt ([`Announcers::unheard`]) |
#[derive(Debug, Clone)]
pub struct Announcers {
    /// s an announcement counts before its announcer shows in the graph.
    pub grace: f64,
    /// In the order first heard (a Python dict's order).
    heard: Vec<Heard>,
    /// Heard, then seen in the graph.
    seen: HashSet<String>,
}

impl Default for Announcers {
    fn default() -> Self {
        Self {
            grace: 10.0,
            heard: Vec::new(),
            seen: HashSet::new(),
        }
    }
}

impl Announcers {
    /// Keep an announcement, heard at `now`.
    pub fn hear(&mut self, announcer: &str, entries: Vec<Entry>, now: f64) {
        match self.heard.iter_mut().find(|h| h.announcer == announcer) {
            Some(h) => {
                h.at = now;
                h.entries = entries;
            }
            None => self.heard.push(Heard {
                announcer: announcer.to_string(),
                at: now,
                entries,
            }),
        }
        self.seen.remove(announcer);
    }

    /// Every entry that counts now; `alive`: the nodes publishing /system/source.
    pub fn entries(&mut self, alive: &HashSet<String>, now: f64) -> Vec<Entry> {
        for h in &self.heard {
            if alive.contains(&h.announcer) {
                self.seen.insert(h.announcer.clone());
            }
        }
        let seen = &self.seen;
        let grace = self.grace;
        self.heard
            .iter()
            .filter(|h| {
                alive.contains(&h.announcer) || (!seen.contains(&h.announcer) && now - h.at < grace)
            })
            .flat_map(|h| h.entries.iter().cloned())
            .collect()
    }

    /// One doubt per node publishing /system/source whose announcement hasn't come.
    #[must_use]
    pub fn unheard(&self, alive: &HashSet<String>) -> Vec<String> {
        let mut names: Vec<&String> = alive
            .iter()
            .filter(|a| !self.heard.iter().any(|h| &h.announcer == *a))
            .collect();
        names.sort();
        names
            .into_iter()
            .map(|a| format!("{a} publishes /system/source, its announcement not heard yet"))
            .collect()
    }
}

/// One line per announced entry whose source isn't the one this run asked for.
#[must_use]
pub fn foreign(entries: &[Entry], expected: Kind) -> Vec<String> {
    entries
        .iter()
        .filter(|e| e.kind != expected)
        .map(|e| format!("{} is {} ({})", e.name, e.kind, e.announcer))
        .collect()
}

/// One watched sensor topic, as found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sensor {
    pub name: String,
    pub topic: String,
    /// The source found: announced, or REAL if unannounced.
    pub kind: Kind,
    pub publishers: Vec<String>,
}

/// The sensor mux's verdict.
///
/// `problems`: certain, at once. `doubts`: what the ROS graph suggests, until [`Hold`]
/// turns a lasting one into a problem. `pending`: the doubts a late announcement would
/// settle; they get the start-up grace as well.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Check {
    /// What `source:=` asked for.
    pub source: Kind,
    /// No problems and no doubts.
    pub ok: bool,
    pub problems: Vec<String>,
    pub doubts: Vec<String>,
    /// The watched topics that have a publisher.
    pub sensors: Vec<Sensor>,
    pub pending: Vec<String>,
}

/// Check that each watched sensor topic comes from exactly one source: `expected`.
///
/// `publishers`: topic -> the nodes publishing it now; a topic nobody publishes is
/// absent, not a fault. A publisher nobody announced counts as REAL. `unheard`:
/// [`Announcers::unheard`], doubts like the rest.
#[must_use]
pub fn check_sensors(
    expected: Kind,
    entries: &[Entry],
    publishers: &HashMap<String, Vec<String>>,
    watched: &[(&str, &str)],
    unheard: &[String],
) -> Check {
    let problems = foreign(entries, expected);
    let mut doubts = unheard.to_vec();
    let mut pending = unheard.to_vec();
    let mut sensors = Vec::new();
    for &(name, topic) in watched {
        let pubs: Vec<String> = publishers
            .get(topic)
            .map(|p| p.iter().cloned().collect::<BTreeSet<_>>())
            .unwrap_or_default()
            .into_iter()
            .collect();
        if pubs.is_empty() {
            continue;
        }
        if pubs.len() > 1 {
            doubts.push(format!(
                "{name}: {} publishers on {topic} ({}): two sources at once",
                pubs.len(),
                pubs.join(", ")
            ));
        }
        let kinds: HashSet<Kind> = entries
            .iter()
            .filter(|e| e.topic == topic)
            .map(|e| e.kind)
            .collect();
        let kind = if kinds.is_empty() {
            if expected != Kind::Real {
                let doubt = format!(
                    "{name}: {topic} from {}, which nobody announces on /system/source: \
                     a real driver?",
                    pubs.join(", ")
                );
                doubts.push(doubt.clone());
                pending.push(doubt);
            }
            Kind::Real
        } else {
            // One that isn't the source asked for is already a problem (foreign, above).
            kinds
                .iter()
                .copied()
                .filter(|k| *k != expected)
                .min_by_key(|k| k.byte())
                .unwrap_or(expected)
        };
        sensors.push(Sensor {
            name: name.to_string(),
            topic: topic.to_string(),
            kind,
            publishers: pubs,
        });
    }
    Check {
        source: expected,
        ok: problems.is_empty() && doubts.is_empty(),
        problems,
        doubts,
        sensors,
        pending,
    }
}

/// Turn a doubt into a problem once it has lasted `hold` s (the sensor mux's memory).
///
/// A pending doubt also waits for `grace_until`: at start-up, on a busy computer,
/// discovery can show a simulator's sensors 10 s before the node announcing them.
#[derive(Debug, Clone)]
pub struct Hold {
    pub hold: f64,
    pub grace_until: f64,
    since: HashMap<String, f64>,
}

impl Hold {
    #[must_use]
    pub fn new(hold: f64, grace_until: f64) -> Self {
        Self {
            hold,
            grace_until,
            since: HashMap::new(),
        }
    }

    /// Whether a pending doubt is being given the start-up grace now.
    #[must_use]
    pub fn in_grace(&self, now: f64, check: &Check) -> bool {
        !check.pending.is_empty() && now < self.grace_until
    }

    /// The check with each doubt that has lasted long enough as a problem.
    pub fn judge(&mut self, now: f64, check: Check) -> Check {
        self.since = check
            .doubts
            .iter()
            .map(|d| (d.clone(), self.since.get(d).copied().unwrap_or(now)))
            .collect();
        let lasting: Vec<String> = check
            .doubts
            .iter()
            .filter(|d| {
                now - self.since[*d] >= self.hold
                    && (!check.pending.contains(d) || now >= self.grace_until)
            })
            .cloned()
            .collect();
        let doubts: Vec<String> = check
            .doubts
            .iter()
            .filter(|d| !lasting.contains(d))
            .cloned()
            .collect();
        let mut problems = check.problems;
        problems.extend(lasting);
        Check {
            ok: problems.is_empty() && doubts.is_empty(),
            problems,
            doubts,
            ..check
        }
    }
}

/// The sensor mux's last verdict, as the output mux heard it.
#[derive(Debug, Clone, PartialEq)]
pub struct Report {
    /// s, when it arrived.
    pub time: f64,
    pub source: Kind,
    pub ok: bool,
    pub problems: Vec<String>,
    pub doubts: Vec<String>,
}

/// What a person's clear did: the message either way.
pub type Cleared = Result<String, String>;

/// The output mux's rule: pass the target's commands on, or latch a stop.
///
/// Open only when the sensor mux, heard within `stale_after` s, says ok for the
/// target's source; nothing on /system/source says otherwise; and both have held for
/// `settle` s. Evidence latches a stop until [`Gate::clear`]; doubt only waits.
/// `screens` (a test only, never the car): screens that must be alive.
#[derive(Debug, Clone)]
pub struct Gate {
    pub target: Target,
    pub expected: Kind,
    /// s of all clear before commands flow.
    pub settle: f64,
    /// s: a quieter sensor mux closes the gate (waits).
    pub stale_after: f64,
    pub report: Option<Report>,
    pub entries: Vec<Entry>,
    /// [`Announcers::unheard`]: waits.
    pub unheard: Vec<String>,
    /// Why it stopped, while latched.
    pub latched: Vec<String>,
    pub clear_since: Option<f64>,
    pub screens: Vec<String>,
    /// s: 10 beats a second, so a quiet second is no glitch.
    pub screen_timeout: f64,
    beats: HashMap<String, f64>,
}

impl Gate {
    /// # Errors
    /// [`ChoiceError::ScreensForCar`]: a car target takes no screens.
    pub fn new(
        target: Target,
        settle: f64,
        stale_after: f64,
        screens: Vec<String>,
    ) -> Result<Self, ChoiceError> {
        let expected = target.source();
        if !screens.is_empty() && expected == Kind::Real {
            return Err(ChoiceError::ScreensForCar(target));
        }
        Ok(Self {
            target,
            expected,
            settle,
            stale_after,
            report: None,
            entries: Vec::new(),
            unheard: Vec::new(),
            latched: Vec::new(),
            clear_since: None,
            screens,
            screen_timeout: 1.0,
            beats: HashMap::new(),
        })
    }

    /// Note that a screen is alive.
    pub fn beat(&mut self, screen: &str, now: f64) {
        self.beats.insert(screen.to_string(), now);
    }

    /// (problems, waits) now: what latches a stop, and what only holds it shut.
    #[must_use]
    pub fn judge(&self, now: f64) -> (Vec<String>, Vec<String>) {
        let Some(report) = &self.report else {
            return (
                Vec::new(),
                vec!["waiting for the sensor mux (/system/sensor_mux/state)".to_string()],
            );
        };
        let (mut problems, mut waits) = (Vec::new(), Vec::new());
        let age = now - report.time;
        if age > self.stale_after {
            waits.push(format!("the sensor mux is quiet ({age:.1} s)"));
        }
        if report.source != self.expected {
            problems.push(format!(
                "the sensor mux checks for {}, target {} needs {}",
                report.source, self.target, self.expected
            ));
        } else if !report.ok {
            problems.extend(report.problems.iter().cloned());
            waits.extend(report.doubts.iter().map(|d| format!("unsure: {d}")));
            if report.problems.is_empty() && report.doubts.is_empty() {
                problems.push("the sensor mux says no".to_string());
            }
        }
        problems.extend(foreign(&self.entries, self.expected));
        waits.extend(self.unheard.iter().cloned());
        for screen in &self.screens {
            match self.beats.get(screen) {
                None => waits.push(format!("waiting for the {screen} screen")),
                Some(heard) if now - heard > self.screen_timeout => waits.push(format!(
                    "the {screen} screen is quiet ({:.1} s)",
                    now - heard
                )),
                Some(_) => {}
            }
        }
        (problems, waits)
    }

    /// Re-judge, after a report, an announcement, a command or a tick.
    pub fn update(&mut self, now: f64) {
        if !self.latched.is_empty() {
            return;
        }
        let (problems, waits) = self.judge(now);
        if !problems.is_empty() {
            self.latched = problems;
            self.clear_since = None;
        } else if !waits.is_empty() {
            self.clear_since = None;
        } else if self.clear_since.is_none() {
            self.clear_since = Some(now);
        }
    }

    /// Whether a command arriving now goes on to the target.
    pub fn is_open(&mut self, now: f64) -> bool {
        self.update(now);
        self.latched.is_empty() && self.clear_since.is_some_and(|t| now - t >= self.settle)
    }

    /// Unlatch, if the evidence has gone: a person's reset. Commands flow after settle.
    ///
    /// # Errors
    /// The evidence is still there: why, and nothing changes.
    pub fn clear(&mut self, now: f64) -> Cleared {
        let (problems, waits) = self.judge(now);
        if !problems.is_empty() {
            return Err(format!("still wrong: {}", problems.join("; ")));
        }
        let was = std::mem::take(&mut self.latched);
        self.clear_since = if waits.is_empty() { Some(now) } else { None };
        let then = if waits.is_empty() {
            format!("in {} s", fmt_g(self.settle))
        } else {
            format!("once {} passes", waits.join("; "))
        };
        Ok(if was.is_empty() {
            "nothing was latched".to_string()
        } else {
            format!("cleared ({}): commands flow {then}", was.join("; "))
        })
    }

    /// One line for people: OPEN, WAIT or STOP, the target, and why.
    pub fn state(&mut self, now: f64) -> String {
        let want = self.expected;
        if self.is_open(now) {
            return format!("OPEN - {}: every source {want}", self.target);
        }
        if !self.latched.is_empty() {
            return format!(
                "STOP - target {} needs {want} - {} - latched: /safety/output_mux/clear",
                self.target,
                self.latched.join("; ")
            );
        }
        let (_, waits) = self.judge(now);
        if waits.is_empty() {
            format!(
                "WAIT - {}: all {want}, settling for {} s",
                self.target,
                fmt_g(self.settle)
            )
        } else {
            format!("WAIT - {}: {}", self.target, waits.join("; "))
        }
    }
}
