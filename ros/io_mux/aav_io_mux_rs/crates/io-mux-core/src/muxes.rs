//! The two muxes' node logic, without ROS (aav_io_mux's `muxes.py`): what each does with
//! each message and each tick. The rclrs nodes only turn messages into calls here, and
//! the answers back into messages and log lines.
//!
//! `alive` is the set of nodes the ROS graph shows publishing /system/source now.

use std::collections::{HashMap, HashSet};

use ego_core::fmt_g;

use crate::provenance::{
    check_sensors, Announcers, Check, ChoiceError, Cleared, Entry, Gate, Hold, Kind, Report,
    Target, SENSORS,
};
use crate::Counter;

/// How loud a log line is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Error,
    Warning,
    Info,
}

impl Level {
    /// As the Python names it: error, warning, info.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Error => "error",
            Self::Warning => "warning",
            Self::Info => "info",
        }
    }
}

/// One line to log.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Log {
    pub level: Level,
    pub text: String,
}

impl Log {
    fn new(level: Level, text: String) -> Self {
        Self { level, text }
    }
}

/// The output mux's `screens` parameter: comma-separated names, blanks dropped.
#[must_use]
pub fn screen_names(text: &str) -> Vec<String> {
    text.split(',')
        .map(str::trim)
        .filter(|n| !n.is_empty())
        .map(String::from)
        .collect()
}

/// sensor_mux's work: the announcements heard, and the verdict each tick.
#[derive(Debug, Clone)]
pub struct SensorMux {
    expected: Kind,
    ready_at: f64,
    hold: Hold,
    announcers: Announcers,
    last_verdict: Option<(Vec<String>, Vec<String>, bool)>,
}

impl SensorMux {
    /// `source`: `source:=`; `now`: the start; `startup`, `hold`, `grace`: s, as the
    /// parameters.
    ///
    /// # Errors
    /// An unknown `source:=`.
    pub fn new(
        source: &str,
        now: f64,
        startup: f64,
        hold: f64,
        grace: f64,
    ) -> Result<Self, ChoiceError> {
        Ok(Self {
            expected: Kind::from_source_name(source)?,
            ready_at: now + startup,
            hold: Hold::new(hold, now + grace),
            announcers: Announcers::default(),
            last_verdict: None,
        })
    }

    /// The source every sensor must come from.
    #[must_use]
    pub fn expected(&self) -> Kind {
        self.expected
    }

    /// Before this, ticks say nothing: discovery settles.
    #[must_use]
    pub fn ready_at(&self) -> f64 {
        self.ready_at
    }

    /// One /system/source message: its announcer and its entries.
    pub fn hear(&mut self, announcer: &str, entries: Vec<Entry>, now: f64) {
        self.announcers.hear(announcer, entries, now);
    }

    /// The check to publish this tick, and what to log; `None` while discovery settles.
    /// `publishers`: each watched sensor topic -> the nodes publishing it now.
    pub fn tick(
        &mut self,
        now: f64,
        alive: &HashSet<String>,
        publishers: &HashMap<String, Vec<String>>,
    ) -> Option<(Check, Option<Log>)> {
        if now < self.ready_at {
            return None;
        }
        let entries = self.announcers.entries(alive, now);
        let unheard = self.announcers.unheard(alive);
        let found = check_sensors(self.expected, &entries, publishers, &SENSORS, &unheard);
        let grace = self.hold.in_grace(now, &found);
        let check = self.hold.judge(now, found);
        let verdict = (check.problems.clone(), check.doubts.clone(), grace);
        if self.last_verdict.as_ref() == Some(&verdict) {
            return Some((check, None));
        }
        self.last_verdict = Some(verdict);
        let want = self.expected;
        let log = if !check.problems.is_empty() {
            Log::new(
                Level::Error,
                format!("NOT {want}: {}", check.problems.join("; ")),
            )
        } else if !check.doubts.is_empty() && grace {
            let left = self.hold.grace_until - now;
            Log::new(
                Level::Warning,
                format!(
                    "unsure, waiting for /system/source (the output mux holds WAIT; a missing \
                     announcement counts in {left:.0} s): {}",
                    check.doubts.join("; ")
                ),
            )
        } else if !check.doubts.is_empty() {
            Log::new(
                Level::Warning,
                format!(
                    "unsure (for {} s): {}",
                    fmt_g(self.hold.hold),
                    check.doubts.join("; ")
                ),
            )
        } else {
            let found: Vec<String> = check
                .sensors
                .iter()
                .map(|s| format!("{} {}", s.name, s.kind))
                .collect();
            let found = if found.is_empty() {
                "no sensors yet".to_string()
            } else {
                found.join(", ")
            };
            Log::new(Level::Info, format!("all {want}: {found}"))
        };
        Some((check, Some(log)))
    }
}

/// A state line to publish on /safety/output_mux/state, and what to log with it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Published {
    pub line: String,
    pub log: Option<Log>,
}

/// output_mux's work: the gate, the announcements, the heartbeat, the state line.
#[derive(Debug, Clone)]
pub struct OutputMux {
    gate: Gate,
    announcers: Announcers,
    counter: Counter,
    last_state: Option<String>,
}

impl OutputMux {
    /// # Errors
    /// An unknown `target:=`, or screens for the car.
    pub fn new(
        target: &str,
        settle: f64,
        stale_after: f64,
        screens: Vec<String>,
    ) -> Result<Self, ChoiceError> {
        Ok(Self {
            gate: Gate::new(target.parse::<Target>()?, settle, stale_after, screens)?,
            announcers: Announcers::default(),
            counter: Counter::default(),
            last_state: None,
        })
    }

    /// The gate itself: its target, source and screens.
    #[must_use]
    pub fn gate(&self) -> &Gate {
        &self.gate
    }

    /// Re-judge: the live announcements, any not heard yet, the sensor mux's age.
    pub fn tick(&mut self, now: f64, alive: &HashSet<String>) {
        self.gate.entries = self.announcers.entries(alive, now);
        self.gate.unheard = self.announcers.unheard(alive);
        self.gate.update(now);
    }

    /// One /system/sensor_mux/state message.
    pub fn check(
        &mut self,
        now: f64,
        alive: &HashSet<String>,
        source: Kind,
        ok: bool,
        problems: Vec<String>,
        doubts: Vec<String>,
    ) -> Option<Published> {
        self.gate.report = Some(Report {
            time: now,
            source,
            ok,
            problems,
            doubts,
        });
        self.tick(now, alive);
        self.publish_state(now, true)
    }

    /// One /system/source message.
    pub fn source(
        &mut self,
        now: f64,
        alive: &HashSet<String>,
        announcer: &str,
        entries: Vec<Entry>,
    ) -> Option<Published> {
        self.announcers.hear(announcer, entries, now);
        self.tick(now, alive);
        self.publish_state(now, true)
    }

    /// One beat from a screen in `screens`.
    pub fn beat(&mut self, screen: &str, now: f64) {
        self.gate.beat(screen, now);
    }

    /// The 2 Hz timer: re-judge, and say the state.
    pub fn timer(&mut self, now: f64, alive: &HashSet<String>) -> Published {
        self.tick(now, alive);
        self.state_now(now)
    }

    /// One of ego's commands: the heartbeat's counter if it goes on, else `None`.
    pub fn command(&mut self, now: f64) -> Option<u8> {
        self.gate.is_open(now).then(|| self.counter.beat())
    }

    /// A person's /safety/output_mux/clear: its answer, and the state to publish.
    pub fn clear(&mut self, now: f64, alive: &HashSet<String>) -> (Cleared, Option<Published>) {
        self.tick(now, alive);
        let cleared = self.gate.clear(now);
        (cleared, self.publish_state(now, true))
    }

    /// The state to publish now, or `None`: only a change, if `changes_only`.
    pub fn publish_state(&mut self, now: f64, changes_only: bool) -> Option<Published> {
        let line = self.gate.state(now);
        if changes_only && self.last_state.as_deref() == Some(line.as_str()) {
            return None;
        }
        Some(self.said(line))
    }

    fn state_now(&mut self, now: f64) -> Published {
        let line = self.gate.state(now);
        self.said(line)
    }

    fn said(&mut self, line: String) -> Published {
        let log = (self.last_state.as_deref() != Some(line.as_str())).then(|| {
            let level = if line.starts_with("STOP") {
                Level::Error
            } else {
                Level::Info
            };
            Log::new(level, line.clone())
        });
        if log.is_some() {
            self.last_state = Some(line.clone());
        }
        Published { line, log }
    }
}
