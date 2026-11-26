//! What `python3 tools/ladder.py --json` prints, as types.

use serde::Deserialize;

/// Where a rung is run. The two are never mixed: a pass in one says nothing of the other.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Place {
    /// Ravensim, or a laptop: the window may start these.
    Sim,
    /// The real car: only a person starts these, from a terminal.
    Car,
}

impl Place {
    pub const ALL: [Self; 2] = [Self::Sim, Self::Car];

    /// The column's heading.
    #[must_use]
    pub fn heading(self) -> &'static str {
        match self {
            Self::Sim => "Simulator",
            Self::Car => "The car",
        }
    }
}

/// A rung's newest result in one place.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Deserialize)]
pub enum State {
    #[serde(rename = "pass")]
    Pass,
    #[serde(rename = "fail")]
    Fail,
    #[serde(rename = "not run")]
    NotRun,
    /// The rung is listed and not built yet: it cannot run, pass or fail.
    #[serde(rename = "not built")]
    NotBuilt,
}

impl State {
    /// The word on the screen: colour is never the only signal.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Fail => "FAIL",
            Self::NotRun => "not run",
            Self::NotBuilt => "not built yet",
        }
    }
}

/// One rung in one place: its newest result, its command, and what keeps it shut.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Side {
    pub state: State,
    /// `YYYY-MM-DD HH:MM`, or empty when it has not run.
    pub when: String,
    /// The repository's short commit at the result; `+` at its end: files were changed.
    pub commit: String,
    /// The run's folder of files, or empty.
    pub report: String,
    /// The run's report page (the question, the verdict, the graphs), or empty.
    #[serde(default)]
    pub report_file: String,
    /// Why a pass was recorded on a shut rung, or empty.
    #[serde(rename = "override")]
    pub override_reason: String,
    /// The command that runs it there, or empty: it is done by hand.
    pub command: String,
    /// What keeps it shut, a line each; empty when it is open.
    pub blocked_by: Vec<String>,
}

impl Side {
    /// Shut: the tool refuses to run it, and takes a pass only with an override.
    #[must_use]
    pub fn is_shut(&self) -> bool {
        !self.blocked_by.is_empty()
    }

    /// The result's day (`YYYY-MM-DD`), or `None` when it has not run.
    #[must_use]
    pub fn day(&self) -> Option<&str> {
        self.when.split_whitespace().next()
    }
}

/// One rung of the ladder: something the car can do, and the question that proves it.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Rung {
    /// Its short name on the tool's command line: `stands`.
    pub id: String,
    /// `1` to `9`, or `S`: the sensors, beside every rung.
    pub number: String,
    pub title: String,
    /// THE question it answers, one sentence.
    pub question: String,
    /// The graphs that answer it, in words.
    pub graphs: Vec<String>,
    /// The pass mark.
    pub passes: String,
    /// The final report's chapter it feeds.
    pub chapter: String,
    /// False: listed only. It is never run, recorded, passed or failed.
    pub built: bool,
    /// What exists today, for a rung not built; or empty.
    #[serde(default)]
    pub today: String,
    /// The page about it.
    pub how: String,
    /// What a person does after its run, as on the car; or empty.
    #[serde(default)]
    pub after: String,
    /// Its tests one at a time, in the simulator; empty: it has none.
    #[serde(default)]
    pub tests: Vec<SingleTest>,
    pub sim: Side,
    pub car: Side,
}

/// A check to run before you start: it moves no car, and it never shuts a rung.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Check {
    pub id: String,
    /// What it is, in words: `the code's own tests`.
    pub title: String,
    /// Its newest result and its command.
    #[serde(flatten)]
    pub side: Side,
}

/// One of a rung's tests, run alone in the simulator. The names are the runner's own
/// (`car-test`'s steps); the tool builds the command. There is none for the car.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct SingleTest {
    pub name: String,
    /// Its number on the testing plan's card: `3b`, `B3`, `4.3`.
    pub number: String,
    pub title: String,
    /// The command the tool runs for it: shown, never run by the window itself.
    pub command: String,
}

impl Rung {
    #[must_use]
    pub fn test(&self, name: &str) -> Option<&SingleTest> {
        self.tests.iter().find(|t| t.name == name)
    }

    /// `Rung 1: On stands: accelerate, brake, turn`.
    #[must_use]
    pub fn heading(&self) -> String {
        format!("Rung {}: {}", self.number, self.title)
    }

    /// The sensors' rung: it runs beside every rung, not above one.
    #[must_use]
    pub fn is_beside(&self) -> bool {
        self.number == "S"
    }

    #[must_use]
    pub fn side(&self, place: Place) -> &Side {
        match place {
            Place::Sim => &self.sim,
            Place::Car => &self.car,
        }
    }
}

/// The whole ladder, bottom rung first, as the tool prints it.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Ladder {
    /// The progress file the tool reads and writes. The window never opens it.
    pub file: String,
    pub rungs: Vec<Rung>,
    /// The checks to run before you start.
    #[serde(default)]
    pub checks: Vec<Check>,
    /// A line about older results that are kept and not shown, or empty.
    #[serde(default)]
    pub note: String,
}

/// The tool's output was not the ladder.
#[derive(Debug, thiserror::Error)]
#[error("the ladder tool's JSON does not read: {0}")]
pub struct ParseError(#[from] serde_json::Error);

impl Ladder {
    /// Read `tools/ladder.py --json`'s output.
    ///
    /// # Errors
    /// When the text is not the tool's JSON: a field missing, or a state it does not have.
    pub fn parse(json: &str) -> Result<Self, ParseError> {
        Ok(serde_json::from_str(json)?)
    }

    /// The rungs as the window lists them: the top rung first, then the sensors' rung,
    /// which stands beside the ladder.
    pub fn top_first(&self) -> impl Iterator<Item = &Rung> {
        let climbed = self.rungs.iter().rev().filter(|r| !r.is_beside());
        climbed.chain(self.rungs.iter().filter(|r| r.is_beside()))
    }

    #[must_use]
    pub fn rung(&self, id: &str) -> Option<&Rung> {
        self.rungs.iter().find(|r| r.id == id)
    }

    /// The lowest built rung whose newest result in the simulator is not a pass: the row
    /// the window opens on (the bottom rung when all have passed). Only a place to start
    /// reading: the tool decides what is open.
    #[must_use]
    pub fn first_not_passed(&self) -> Option<&Rung> {
        self.rungs
            .iter()
            .find(|r| r.built && r.sim.state != State::Pass)
            .or_else(|| self.rungs.first())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `tools/ladder.py --json` on a progress file with three results from before 23 Nov:
    /// code passed, stack failed, hallway passed (`tests/ladder.json`;
    /// `tools/tests/test_ladder.py` checks the tool still prints these fields).
    const REAL: &str = include_str!("../tests/ladder.json");

    #[test]
    fn the_real_tools_json_reads_as_the_plans_ten_rungs() -> Result<(), ParseError> {
        let ladder = Ladder::parse(REAL)?;
        let numbers: Vec<_> = ladder.rungs.iter().map(|r| r.number.as_str()).collect();
        assert_eq!(numbers, ["1", "2", "3", "4", "5", "6", "7", "8", "9", "S"]);
        assert!(ladder.file.ends_with("ladder.yaml"));
        let stands = &ladder.rungs[0];
        assert_eq!(stands.id, "stands");
        assert_eq!(
            stands.heading(),
            "Rung 1: On stands: accelerate, brake, turn"
        );
        assert!(stands.question.ends_with('?'));
        assert_eq!(stands.graphs.len(), 4);
        assert_eq!(stands.chapter, "The car and its model");
        assert!(stands.built && stands.sim.command.contains("--stands --sim"));
        Ok(())
    }

    #[test]
    fn a_rung_not_built_is_neither_passed_nor_failed_and_has_no_command() -> Result<(), ParseError>
    {
        let ladder = Ladder::parse(REAL)?;
        for rung in ladder.rungs.iter().filter(|r| !r.built) {
            for place in Place::ALL {
                assert_eq!(rung.side(place).state, State::NotBuilt, "{}", rung.id);
                assert!(rung.side(place).command.is_empty(), "{}", rung.id);
            }
            assert!(!rung.today.is_empty(), "{} says what exists today", rung.id);
        }
        assert_eq!(ladder.rungs.iter().filter(|r| r.built).count(), 1);
        assert_eq!(State::NotBuilt.word(), "not built yet");
        Ok(())
    }

    #[test]
    fn the_checks_and_the_note_on_older_results_read() -> Result<(), ParseError> {
        let ladder = Ladder::parse(REAL)?;
        let ids: Vec<_> = ladder.checks.iter().map(|c| c.id.as_str()).collect();
        assert_eq!(ids, ["code", "stack", "bench"]);
        assert_eq!(ladder.checks[0].side.state, State::Pass);
        assert_eq!(ladder.checks[0].side.day(), Some("2026-11-22"));
        assert_eq!(ladder.checks[0].side.command, "pixi run bash tools/test.sh");
        assert_eq!(ladder.checks[1].side.state, State::Fail);
        assert_eq!(ladder.checks[2].side.state, State::NotRun);
        assert!(ladder.note.contains("hallway (1)"));
        Ok(())
    }

    #[test]
    fn a_car_side_is_shut_until_its_simulator_run_has_passed() -> Result<(), ParseError> {
        let ladder = Ladder::parse(REAL)?;
        let stands = ladder.rung("stands").ok_or_else(missing)?;
        assert_eq!(stands.sim.state, State::NotRun);
        assert_eq!(stands.sim.day(), None);
        assert!(!stands.sim.is_shut(), "the checks never shut a rung");
        assert!(stands.car.is_shut());
        assert!(stands
            .car
            .blocked_by
            .iter()
            .any(|line| line.contains("has not passed in the simulator")));
        Ok(())
    }

    #[test]
    fn the_window_lists_the_top_rung_first_and_opens_on_the_work() -> Result<(), ParseError> {
        let mut ladder = Ladder::parse(REAL)?;
        let listed: Vec<_> = ladder.top_first().map(|r| r.number.as_str()).collect();
        assert_eq!(listed, ["9", "8", "7", "6", "5", "4", "3", "2", "1", "S"]);
        assert_eq!(
            ladder.first_not_passed().map(|r| r.id.as_str()),
            Some("stands")
        );
        // Every built rung passed: it still opens on a rung, never on one not built.
        ladder.rungs[0].sim.state = State::Pass;
        assert_eq!(
            ladder.first_not_passed().map(|r| r.id.as_str()),
            Some("stands")
        );
        Ok(())
    }

    #[test]
    fn a_rungs_single_tests_read_and_every_one_is_the_simulators() -> Result<(), ParseError> {
        let ladder = Ladder::parse(REAL)?;
        let stands = ladder.rung("stands").ok_or_else(missing)?;
        assert_eq!(
            stands.tests.first().map(|t| t.name.as_str()),
            Some("preflight")
        );
        assert!(stands.test("estop").is_some() && stands.test("cable").is_none());
        assert!(stands.after.contains("pixi run clear estop --sim"));
        for test in ladder.rungs.iter().flat_map(|r| &r.tests) {
            assert!(test.command.contains(" --sim "), "{}", test.command);
            assert!(test.command.ends_with(&test.name), "{}", test.command);
        }
        assert!(ladder.rungs[1].tests.is_empty());
        Ok(())
    }

    #[test]
    fn a_state_the_tool_does_not_have_is_an_error_not_a_guess() {
        let odd = REAL.replacen("\"pass\"", "\"maybe\"", 1);
        assert!(Ladder::parse(&odd).is_err());
        assert!(Ladder::parse("Traceback (most recent call last):").is_err());
    }

    fn missing() -> ParseError {
        ParseError(serde::de::Error::custom("the rung is missing"))
    }
}
