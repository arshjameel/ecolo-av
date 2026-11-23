//! A test session's report, as `python3 tools/ladder.py session <rung> --json` prints it:
//! each step with its result, the `vehicle.yaml` lines it produced, its key numbers and one
//! graph (commanded against measured, over time); a hallway session's fits with their error
//! bars. The window never opens a report file: the tool reads them (`tools/ladder_report.py`).
//!
//! Also here, because it is about a session: which step a run is on, from its own lines.

use std::fmt;

use serde::Deserialize;

/// How a step ended, in `car-test`'s own words.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Deserialize)]
pub enum Status {
    #[serde(rename = "PASS")]
    Pass,
    #[serde(rename = "FAIL")]
    Fail,
    #[serde(rename = "SKIPPED")]
    Skipped,
    #[serde(rename = "NOT RUN")]
    NotRun,
}

impl Status {
    /// The word on the screen: colour is never the only signal.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Fail => "FAIL",
            Self::Skipped => "SKIPPED",
            Self::NotRun => "NOT RUN",
        }
    }
}

/// One line of a test's own table: a thing it measured.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Number {
    pub what: String,
    pub value: String,
    pub unit: String,
    /// `vehicle.yaml`'s value today, or empty.
    pub now: String,
    pub note: String,
}

impl Number {
    /// `1.054 m`, or the value alone when it has no unit.
    #[must_use]
    pub fn measured(&self) -> String {
        match self.unit.as_str() {
            "" => self.value.clone(),
            unit => format!("{} {unit}", self.value),
        }
    }
}

/// What a graph's lines are.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum GraphKind {
    Speed,
    Steering,
}

/// A range on one axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Span {
    pub low: f64,
    pub high: f64,
}

impl Span {
    /// The range of some values, or `None` when there is none that is finite.
    fn of(values: impl Iterator<Item = f64>) -> Option<Self> {
        values
            .filter(|v| v.is_finite())
            .fold(None, |span: Option<Self>, v| {
                Some(span.map_or(Self { low: v, high: v }, |s| Self {
                    low: s.low.min(v),
                    high: s.high.max(v),
                }))
            })
    }

    #[must_use]
    pub fn width(self) -> f64 {
        self.high - self.low
    }

    /// Where a value sits in the range, 0 at `low` and 1 at `high`.
    #[must_use]
    pub fn share(self, value: f64) -> f64 {
        if self.width() > 0.0 {
            (value - self.low) / self.width()
        } else {
            0.5
        }
    }

    /// Round numbers across the range, about `wanted` of them: 0, 0.5, 1, 1.5...
    #[must_use]
    pub fn ticks(self, wanted: usize) -> Vec<f64> {
        let width = self.width();
        if width.is_nan() || width <= 0.0 || wanted == 0 {
            return vec![self.low];
        }
        #[allow(clippy::cast_precision_loss)]
        let rough = width / wanted as f64;
        let power = 10f64.powf(rough.log10().floor());
        let step = [1.0, 2.0, 5.0, 10.0]
            .into_iter()
            .map(|m| m * power)
            .find(|&s| s >= rough)
            .unwrap_or(10.0 * power);
        let first = (self.low / step).ceil();
        let last = (self.high / step).floor();
        #[allow(clippy::cast_possible_truncation)]
        let count = (last - first).max(0.0) as u32;
        (0..=count)
            .map(|i| (first + f64::from(i)) * step)
            // -0 would print as "-0".
            .map(|v| if v == 0.0 { 0.0 } else { v })
            .collect()
    }
}

/// A tick's number as an axis shows it: `0.5`, `10`, `-0.25`; never `0.30000000000000004`.
#[must_use]
pub fn tick_label(value: f64) -> String {
    let text = format!("{value:.3}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    match text {
        "-0" | "" => "0".into(),
        other => other.into(),
    }
}

/// One step's graph: what was asked for and what was measured, over the run's time.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Graph {
    pub kind: GraphKind,
    /// `m/s` or `rad`.
    pub unit: String,
    /// `speed` or `steering angle`.
    pub what: String,
    /// Seconds from the start of the run.
    pub t: Vec<f64>,
    pub commanded: Vec<f64>,
    pub measured: Vec<f64>,
}

impl Graph {
    /// The time axis's label.
    pub const TIME_LABEL: &'static str = "time (s)";
    /// The legend, in words: the two lines differ by their dashes, not only their colour.
    pub const COMMANDED: &'static str = "commanded (dashed)";
    pub const MEASURED: &'static str = "measured (solid)";

    /// The value axis's label: `speed (m/s)`.
    #[must_use]
    pub fn value_label(&self) -> String {
        format!("{} ({})", self.what, self.unit)
    }

    /// The time the lines cover.
    #[must_use]
    pub fn time(&self) -> Option<Span> {
        Span::of(self.t.iter().copied())
    }

    /// The values both lines cover, zero always in view, with a little room above and below.
    #[must_use]
    pub fn values(&self) -> Option<Span> {
        let all = self.commanded.iter().chain(&self.measured).copied();
        let span = Span::of(all.chain(std::iter::once(0.0)))?;
        let room = (span.width() * 0.08).max(0.01);
        Some(Span {
            low: if span.low < 0.0 { span.low - room } else { 0.0 },
            high: span.high + room,
        })
    }

    /// One line's points as (time, value), for the lines that have both.
    pub fn points<'a>(&'a self, line: &'a [f64]) -> impl Iterator<Item = (f64, f64)> + 'a {
        self.t.iter().copied().zip(line.iter().copied())
    }

    /// The largest gap between the two lines, with when: a number to read beside the graph.
    #[must_use]
    pub fn widest_gap(&self) -> Option<(f64, f64)> {
        self.t
            .iter()
            .zip(self.commanded.iter().zip(&self.measured))
            .map(|(&t, (&asked, &got))| (t, (asked - got).abs()))
            .filter(|(_, gap)| gap.is_finite())
            .max_by(|a, b| a.1.total_cmp(&b.1))
    }
}

/// One step of a session.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct StepRow {
    pub name: String,
    pub number: String,
    pub status: Status,
    /// What happened, in the runner's words.
    pub detail: String,
    /// The `vehicle.yaml` lines it produced: `max_brake: 3.127`.
    pub yaml: Vec<String>,
    pub numbers: Vec<Number>,
    pub graph: Option<Graph>,
}

/// One fitted number of a hallway session, with its error bar.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Fit {
    pub name: String,
    pub unit: String,
    /// The mean over the passes; `None`: no pass gave it.
    pub value: Option<f64>,
    /// The passes' spread: the error bar.
    pub sd: Option<f64>,
    pub n: u32,
    pub trusted: bool,
    /// Why it is not trusted, or empty.
    pub why: String,
    /// `vehicle.yaml`'s value today.
    pub now: Option<f64>,
}

impl Fit {
    /// `2.989 ± 0.093 m/s^2 (n 3)`, or `-` when no pass gave it.
    #[must_use]
    pub fn measured(&self) -> String {
        match self.value {
            Some(value) => format!(
                "{value:.3} ± {:.3} {} (n {})",
                self.sd.unwrap_or(0.0),
                self.unit,
                self.n
            ),
            None => "-".into(),
        }
    }

    /// `trusted`, or `NOT TRUSTED: why`.
    #[must_use]
    pub fn trust(&self) -> String {
        match (self.trusted, self.why.as_str()) {
            (true, _) => "trusted".into(),
            (false, "") => "NOT TRUSTED".into(),
            (false, why) => format!("NOT TRUSTED: {why}"),
        }
    }
}

/// How many steps ended each way.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
pub struct Counts {
    #[serde(rename = "PASS", default)]
    pub pass: u32,
    #[serde(rename = "FAIL", default)]
    pub fail: u32,
    #[serde(rename = "SKIPPED", default)]
    pub skipped: u32,
    #[serde(rename = "NOT RUN", default)]
    pub not_run: u32,
}

impl fmt::Display for Counts {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} PASS, {} FAIL, {} SKIPPED, {} NOT RUN",
            self.pass, self.fail, self.skipped, self.not_run
        )
    }
}

/// A rung's newest simulator session.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Report {
    pub rung: String,
    /// The session's folder; empty when there is none yet.
    pub folder: String,
    /// False: there is no session to show, and `why` says so.
    pub found: bool,
    pub why: String,
    pub started: String,
    #[serde(rename = "where")]
    pub location: String,
    pub counts: Counts,
    /// This session is the one the ladder's newest simulator result names. False: a single
    /// test, or a run by hand: it did not move the ladder.
    pub recorded: bool,
    pub steps: Vec<StepRow>,
    pub fits: Vec<Fit>,
}

/// The tool's output was not a session.
#[derive(Debug, thiserror::Error)]
#[error("the ladder tool's session does not read: {0}")]
pub struct ParseError(#[from] serde_json::Error);

impl Report {
    /// Read `tools/ladder.py session <rung> --json`'s output.
    ///
    /// # Errors
    /// When the text is not the tool's JSON.
    pub fn parse(json: &str) -> Result<Self, ParseError> {
        Ok(serde_json::from_str(json)?)
    }

    #[must_use]
    pub fn step(&self, name: &str) -> Option<&StepRow> {
        self.steps.iter().find(|s| s.name == name)
    }

    /// The step the window opens on: the first that failed, else the first with a graph,
    /// else the first.
    #[must_use]
    pub fn first_to_read(&self) -> Option<&StepRow> {
        self.steps
            .iter()
            .find(|s| s.status == Status::Fail)
            .or_else(|| self.steps.iter().find(|s| s.graph.is_some()))
            .or_else(|| self.steps.first())
    }

    /// One line on what this session is to the ladder.
    #[must_use]
    pub fn standing(&self) -> &'static str {
        if self.recorded {
            "This session is the one the ladder recorded."
        } else {
            "Not recorded on the ladder: a single test, or a run from a terminal."
        }
    }
}

/// Which step a run is on, from its own lines: the runner prints `=== Step 3b: steering
/// rate (3.11) ===` (or `=== Pass 2: hall-brake, facing there ===`) as each one starts.
pub fn current_step<'a>(lines: impl DoubleEndedIterator<Item = &'a str>) -> Option<&'a str> {
    lines.rev().find_map(|line| {
        line.trim()
            .strip_prefix("=== ")?
            .strip_suffix(" ===")
            .map(str::trim)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `tools/ladder.py session hallway --json` after a real run in Ravensim (22 Nov:
    /// `car-test --hallway --sim --attach --kinds brake --repeats 1`).
    const REAL: &str = include_str!("../tests/session.json");

    fn real() -> Report {
        Report::parse(REAL).expect("the tool's session reads")
    }

    #[test]
    fn the_real_tools_session_reads() {
        let report = real();
        assert_eq!(report.rung, "hallway");
        assert!(report.found && !report.recorded);
        assert_eq!(report.counts.pass, 1);
        assert_eq!(
            report.counts.to_string(),
            "1 PASS, 0 FAIL, 0 SKIPPED, 0 NOT RUN"
        );
        let step = &report.steps[0];
        assert_eq!(step.status, Status::Pass);
        assert!(step.yaml.iter().any(|line| line.starts_with("max_brake:")));
        assert!(step.numbers[0].what.contains("distance"));
        assert!(step.numbers[0].measured().ends_with(" m"));
        assert_eq!(report.first_to_read().map(|s| &s.name), Some(&step.name));
    }

    #[test]
    fn its_graph_has_both_lines_and_axes_in_words() {
        let report = real();
        let graph = report.steps[0]
            .graph
            .as_ref()
            .expect("a brake pass has a log");
        assert_eq!(graph.kind, GraphKind::Speed);
        assert_eq!(graph.value_label(), "speed (m/s)");
        assert_eq!(graph.t.len(), graph.commanded.len());
        assert_eq!(graph.t.len(), graph.measured.len());
        let time = graph.time().expect("it has points");
        assert!(time.low < 0.1 && time.high > 10.0);
        let values = graph.values().expect("it has points");
        // Asked 1 m/s: zero is in view and there is room above the top.
        assert!(values.low == 0.0 && values.high > 1.0 && values.high < 1.5);
        let (when, gap) = graph.widest_gap().expect("it has points");
        assert!(gap > 0.1 && when > 0.0);
    }

    #[test]
    fn a_fit_says_its_error_bar_and_its_trust() {
        let report = real();
        let fit = report
            .fits
            .iter()
            .find(|f| f.name == "max_brake")
            .expect("a brake pass fits max_brake");
        assert!(fit.measured().contains(" ± ") && fit.measured().ends_with("(n 1)"));
        assert!(fit.trust().starts_with("NOT TRUSTED: one run"));
        let trusted = Fit {
            trusted: true,
            sd: Some(0.093),
            value: Some(2.989),
            n: 3,
            ..fit.clone()
        };
        assert_eq!(trusted.measured(), "2.989 ± 0.093 m/s^2 (n 3)");
        assert_eq!(trusted.trust(), "trusted");
        let none = Fit {
            value: None,
            ..fit.clone()
        };
        assert_eq!(none.measured(), "-");
    }

    #[test]
    fn ticks_are_round_numbers_inside_the_range() {
        let span = Span {
            low: 0.0,
            high: 1.24,
        };
        assert_eq!(span.ticks(5), [0.0, 0.5, 1.0]);
        let time = Span {
            low: 0.0,
            high: 21.9,
        };
        assert_eq!(time.ticks(5), [0.0, 5.0, 10.0, 15.0, 20.0]);
        let steer = Span {
            low: -0.54,
            high: 0.54,
        };
        assert_eq!(steer.ticks(4), [-0.5, 0.0, 0.5]);
        let flat = Span {
            low: 2.0,
            high: 2.0,
        };
        assert_eq!(flat.ticks(5), [2.0]);
        assert!((span.share(0.62) - 0.5).abs() < 1e-9);
        assert!((flat.share(2.0) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn a_tick_reads_as_a_person_writes_it() {
        assert_eq!(tick_label(0.1 + 0.2), "0.3");
        assert_eq!(tick_label(10.0), "10");
        assert_eq!(tick_label(-0.25), "-0.25");
        assert_eq!(tick_label(0.0), "0");
        assert_eq!(tick_label(-0.0), "0");
    }

    #[test]
    fn a_failed_step_is_read_first() {
        let mut report = real();
        let mut failed = report.steps[0].clone();
        failed.name = "02-hall-brake-there".into();
        failed.status = Status::Fail;
        failed.graph = None;
        report.steps.push(failed);
        assert_eq!(
            report.first_to_read().map(|s| s.name.as_str()),
            Some("02-hall-brake-there")
        );
        assert!(report.step("no-such-step").is_none());
    }

    #[test]
    fn no_session_yet_reads_too() {
        let none = r#"{"folder": "", "found": false, "why": "no session yet: run it once",
            "started": "", "where": "", "counts": {}, "steps": [], "fits": [],
            "rung": "lot", "recorded": false}"#;
        let report = Report::parse(none).expect("it reads");
        assert!(!report.found);
        assert_eq!(report.counts, Counts::default());
        assert!(report.first_to_read().is_none());
        assert!(Report::parse("Traceback (most recent call last):").is_err());
    }

    #[test]
    fn the_running_step_is_the_runners_last_heading() {
        let lines = [
            "car-test --stands: ...",
            "=== Step 1: the preflight (actuator-test preflight --stands) ===",
            "Step 1: PASS: every line PASS",
            "",
            "=== Step 3b: steering rate (3.11) ===",
            "Running: 7 steps. Ctrl-C stops the car.",
        ];
        assert_eq!(
            current_step(lines.iter().copied()),
            Some("Step 3b: steering rate (3.11)")
        );
        let passes = ["=== Pass 2: hall-brake, facing there ==="];
        assert_eq!(
            current_step(passes.iter().copied()),
            Some("Pass 2: hall-brake, facing there")
        );
        assert_eq!(current_step(["Waiting for Ravensim"].into_iter()), None);
    }
}
