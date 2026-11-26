//! A rung's answer, as `python3 tools/ladder.py session <rung> --json` carries it under
//! `answer`: one verdict line, a few graphs each with its numbers, and the faults tried
//! on purpose. Every number is the tool's (`tools/rung_answer.py`), read from the run's
//! own files and already in words: nothing is computed here.

use serde::Deserialize;

use crate::report::{Lines, Status};

/// How the run answered the rung's question.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum VerdictState {
    #[serde(rename = "PASS")]
    Pass,
    #[serde(rename = "FAIL")]
    Fail,
    /// Some steps did not run.
    #[serde(rename = "NOT FINISHED")]
    NotFinished,
    /// One of the rung's tests alone: it does not count.
    #[serde(rename = "ONE TEST ONLY")]
    OneTestOnly,
    /// There is no run to judge.
    #[serde(rename = "")]
    None,
}

/// The verdict: its kind, and one line of words that begins with it (`PASS: the wheels...`).
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Verdict {
    pub state: VerdictState,
    pub words: String,
}

/// One graph of the answer, with the numbers that belong beside it.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct AnswerGraph {
    pub id: String,
    pub title: String,
    /// The part of the rung's question this graph answers.
    pub asks: String,
    /// Which of the run's files the lines come from.
    pub from: String,
    pub x_label: String,
    /// The value axis, with its unit: `wheel speed (m/s)`.
    pub y_label: String,
    pub unit: String,
    /// The legend, in words: `asked for (dashed line)`.
    pub asked_words: String,
    pub measured_words: String,
    pub t: Vec<f64>,
    pub asked: Vec<f64>,
    pub measured: Vec<f64>,
    /// The numbers beside the graph: a table, already in words with their units.
    pub columns: Vec<String>,
    pub rows: Vec<Vec<String>>,
    /// Single numbers: what, and its value.
    pub facts: Vec<Vec<String>>,
    /// How the numbers were made, and what could not be measured.
    pub notes: Vec<String>,
    /// The run's own result for the steps behind this graph.
    pub judged: String,
    /// Why there is no graph (`not measured: ...`), or empty.
    pub missing: String,
    /// The car's side of this graph: `no car run yet`, until a car run fills it.
    pub car: String,
}

impl AnswerGraph {
    /// The two lines, for drawing; `None` when the run did not measure this.
    #[must_use]
    pub fn lines(&self) -> Option<Lines<'_>> {
        (self.missing.is_empty() && self.t.len() >= 2).then_some(Lines {
            t: &self.t,
            asked: &self.asked,
            measured: &self.measured,
        })
    }
}

/// One fault tried on purpose.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct FaultRow {
    pub name: String,
    pub number: String,
    /// The fault, in words: `the GPS is lost`.
    pub fault: String,
    /// The run's own result for the step.
    pub status: Status,
    /// `yes`, `NO`, `not tried`, or `not measured`.
    pub stopped: String,
    /// How long the car took to stand still: `0.33 s`, or `not measured`, or empty.
    pub seconds: String,
    /// The time allowed: `2 s`, or empty.
    pub limit: String,
    /// What the clock counts from; or why it was not tried.
    pub from: String,
    pub car: String,
}

impl FaultRow {
    /// The row's cells, in the table's order.
    #[must_use]
    pub fn cells(&self) -> [String; 5] {
        [
            format!("{} {}", self.number, self.fault),
            self.stopped.clone(),
            self.seconds.clone(),
            self.limit.clone(),
            self.from.clone(),
        ]
    }
}

/// The faults tried on purpose: did each stop the car, and in how long.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Faults {
    pub columns: Vec<String>,
    pub rows: Vec<FaultRow>,
    pub missing: String,
    pub car: String,
}

/// A rung's answer from one run.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Answer {
    pub rung: String,
    pub number: String,
    pub title: String,
    pub question: String,
    /// The pass mark the verdict was judged against.
    pub passes: String,
    /// The final report's chapter it feeds.
    pub chapter: String,
    /// False: there is no run yet, and `why` says so.
    pub found: bool,
    pub why: String,
    pub folder: String,
    pub started: String,
    /// `the simulator` or `the car`.
    pub place: String,
    pub commit: String,
    pub command: String,
    pub verdict: Verdict,
    pub graphs: Vec<AnswerGraph>,
    pub faults: Option<Faults>,
    /// The car's side of the whole answer: `no car run yet`.
    pub car: String,
    /// The saved report page for this run, or empty: none is saved yet.
    pub report_file: String,
}

impl Answer {
    /// The heading over the simulator's column and the car's: this run's place first.
    #[must_use]
    pub fn places(&self) -> [&'static str; 2] {
        if self.place == "the car" {
            ["On the car", "In the simulator"]
        } else {
            ["In the simulator", "On the car"]
        }
    }

    /// What the "Open the report" button says it will do.
    #[must_use]
    pub fn report_words(&self) -> &'static str {
        if self.report_file.is_empty() {
            "No page is saved for this run yet: the button writes it, then opens it."
        } else {
            "One page, saved with the run: the question, the verdict, the graphs, the numbers."
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::report::Report;

    use super::*;

    fn stands() -> Answer {
        Report::parse(include_str!("../tests/session-stands.json"))
            .expect("the tool's JSON reads")
            .answer
            .expect("rung 1 has an answer")
    }

    #[test]
    fn the_real_answer_reads_as_a_verdict_three_graphs_and_the_faults() {
        let answer = stands();
        assert!(answer.found);
        assert_eq!(answer.number, "1");
        assert_eq!(answer.verdict.state, VerdictState::Pass);
        assert!(answer.verdict.words.starts_with("PASS: "));
        let ids: Vec<_> = answer.graphs.iter().map(|g| g.id.as_str()).collect();
        assert_eq!(ids, ["wheels", "steering", "brake"]);
        for graph in &answer.graphs {
            let lines = graph.lines().expect("each graph has its lines");
            assert!(lines.time().is_some_and(|t| t.width() > 5.0));
            assert!(graph.y_label.contains('('), "{}: a unit", graph.y_label);
            assert!(graph.asked_words.contains("dashed"));
            assert!(graph.rows.iter().all(|r| r.len() == graph.columns.len()));
            assert_eq!(graph.car, "no car run yet");
        }
        let faults = answer.faults.expect("the faults were tried");
        assert!(faults
            .rows
            .iter()
            .any(|r| r.stopped == "yes" && r.seconds.ends_with(" s")));
        assert!(faults.rows.iter().any(|r| r.stopped == "not tried"));
        assert!(faults
            .rows
            .iter()
            .all(|r| r.cells().len() == faults.columns.len()));
    }

    #[test]
    fn the_cars_side_is_a_slot_that_says_no_car_run_yet() {
        let answer = stands();
        assert_eq!(answer.places(), ["In the simulator", "On the car"]);
        assert_eq!(answer.car, "no car run yet");
    }

    #[test]
    fn a_graph_the_run_did_not_measure_has_no_lines_and_says_why() {
        let mut graph = stands().graphs.remove(0);
        graph.missing = "not measured: this run did not include the step wheel-steps".into();
        assert!(graph.lines().is_none());
    }

    #[test]
    fn no_run_yet_reads_too() {
        let none = r#"{"rung":"stands","number":"1","title":"t","question":"q","passes":"p",
            "chapter":"c","found":false,"why":"No run yet","folder":"","started":"","place":"",
            "where":"","commit":"","command":"","verdict":{"state":"","words":""},"graphs":[],
            "faults":null,"steps":[],"car":"no car run yet","report_file":""}"#;
        let answer: Answer = serde_json::from_str(none).expect("it reads");
        assert_eq!(answer.verdict.state, VerdictState::None);
        assert!(!answer.found && answer.faults.is_none());
    }
}
