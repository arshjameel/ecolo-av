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
}

impl State {
    /// The word on the screen: colour is never the only signal.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Fail => "FAIL",
            Self::NotRun => "not run",
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
    /// The report folder, or empty.
    pub report: String,
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

/// One rung of the ladder.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Rung {
    pub id: String,
    pub title: String,
    /// Where it happens: laptop, Ravensim, bench, stands...
    #[serde(rename = "where")]
    pub location: String,
    pub measures: String,
    pub passes: String,
    /// The page that says how to do it by hand.
    pub how: String,
    /// What a person does after its session, as on the car; or empty.
    #[serde(default)]
    pub after: String,
    /// Its tests one at a time, in the simulator; empty: it has none.
    #[serde(default)]
    pub tests: Vec<SingleTest>,
    pub sim: Side,
    pub car: Side,
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

    /// The rungs as the window lists them: the top rung first.
    pub fn top_first(&self) -> impl Iterator<Item = &Rung> {
        self.rungs.iter().rev()
    }

    #[must_use]
    pub fn rung(&self, id: &str) -> Option<&Rung> {
        self.rungs.iter().find(|r| r.id == id)
    }

    /// The lowest rung whose newest result in the simulator is not a pass: the row the
    /// window opens on. Only a place to start reading: the tool decides what is open.
    #[must_use]
    pub fn first_not_passed(&self) -> Option<&Rung> {
        self.rungs.iter().find(|r| r.sim.state != State::Pass)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `tools/ladder.py --json` on an empty progress file, then two results
    /// (`tests/ladder.json`; `tools/tests/test_ladder.py` checks the tool still prints
    /// these fields).
    const REAL: &str = include_str!("../tests/ladder.json");

    #[test]
    fn the_real_tools_json_reads() -> Result<(), ParseError> {
        let ladder = Ladder::parse(REAL)?;
        let ids: Vec<_> = ladder.rungs.iter().map(|r| r.id.as_str()).collect();
        assert_eq!(
            ids,
            ["code", "stack", "bench", "stands", "hallway", "lot", "route"]
        );
        assert!(ladder.file.ends_with("ladder.yaml"));
        let code = &ladder.rungs[0];
        assert_eq!(code.location, "laptop");
        assert_eq!(code.sim.state, State::Pass);
        assert_eq!(code.sim.day(), Some("2026-11-22"));
        assert!(!code.sim.commit.is_empty());
        assert_eq!(code.sim.command, "pixi run bash tools/test.sh");
        assert!(!code.sim.is_shut());
        Ok(())
    }

    #[test]
    fn a_fail_a_shut_rung_and_one_never_run_read() -> Result<(), ParseError> {
        let ladder = Ladder::parse(REAL)?;
        let stack = ladder.rung("stack").ok_or_else(missing)?;
        assert_eq!(stack.sim.state, State::Fail);
        let stands = ladder.rung("stands").ok_or_else(missing)?;
        assert_eq!(stands.sim.state, State::NotRun);
        assert_eq!(stands.sim.day(), None);
        assert!(stands.sim.is_shut());
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
        let ladder = Ladder::parse(REAL)?;
        assert_eq!(
            ladder.top_first().next().map(|r| r.id.as_str()),
            Some("route")
        );
        assert_eq!(
            ladder.first_not_passed().map(|r| r.id.as_str()),
            Some("stack")
        );
        Ok(())
    }

    #[test]
    fn a_rungs_single_tests_read_and_every_one_is_the_simulators() -> Result<(), ParseError> {
        let ladder = Ladder::parse(REAL)?;
        assert!(ladder.rung("code").ok_or_else(missing)?.tests.is_empty());
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
        let brake = ladder.rung("lot").ok_or_else(missing)?.test("brake");
        assert_eq!(
            brake.map(|t| t.command.as_str()),
            Some("pixi run car-test --lot --sim --attach --only brake")
        );
        Ok(())
    }

    #[test]
    fn an_older_tool_without_single_tests_still_reads() {
        let old = REAL
            .replace("\"tests\":", "\"tests_then\":")
            .replace("\"after\":", "\"after_then\":");
        let ladder = Ladder::parse(&old).expect("the fields are optional");
        assert!(ladder
            .rungs
            .iter()
            .all(|r| r.tests.is_empty() && r.after.is_empty()));
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
