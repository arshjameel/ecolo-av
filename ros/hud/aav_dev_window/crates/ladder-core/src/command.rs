//! The commands the window gives `tools/ladder.py`: the table, a run in the simulator, a
//! result recorded by hand. There is no command here that runs a car test.

use std::{path::Path, process::Command};

use crate::ladder::{Check, Place, Rung, SingleTest};

/// The interpreter: the one on the path (`pixi run ladder-window` puts pixi's first).
pub const PROGRAM: &str = "python3";
/// The tool, from the repository's root.
pub const TOOL: &str = "tools/ladder.py";

/// Which of a rung's simulator sessions to show.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Which {
    /// The newest run, a single test included.
    #[default]
    Newest,
    /// The session the ladder's newest result names.
    Recorded,
}

impl Which {
    pub const ALL: [Self; 2] = [Self::Newest, Self::Recorded];

    /// The choice's words on the screen.
    #[must_use]
    pub fn words(self) -> &'static str {
        match self {
            Self::Newest => "the newest run",
            Self::Recorded => "the run that counted on the ladder",
        }
    }
}

/// A result a person records.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Outcome {
    #[default]
    Pass,
    Fail,
}

impl Outcome {
    /// The tool's own word for it.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            Self::Pass => "pass",
            Self::Fail => "fail",
        }
    }
}

/// The "Record by hand" form.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordForm {
    pub outcome: Outcome,
    pub place: Place,
    pub note: String,
    /// Why a shut rung is passed anyway. The tool asks for it; the window does not judge.
    pub override_reason: String,
}

impl Default for RecordForm {
    fn default() -> Self {
        Self {
            outcome: Outcome::Pass,
            place: Place::Sim,
            note: String::new(),
            override_reason: String::new(),
        }
    }
}

impl RecordForm {
    /// Would the tool ask for an override? Only for the form's hint: the tool decides.
    #[must_use]
    pub fn override_expected(&self, rung: &Rung) -> bool {
        self.outcome == Outcome::Pass && rung.side(self.place).is_shut()
    }
}

/// One call of `tools/ladder.py`: its arguments, after the interpreter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ToolCommand {
    args: Vec<String>,
}

impl ToolCommand {
    /// The whole ladder, as JSON.
    #[must_use]
    pub fn table() -> Self {
        Self::new(["--json"])
    }

    /// Run a rung's simulator twin; the tool records the result. Only
    /// [`crate::show::run_button`] makes one, after its checks. The tool's `run` has no
    /// car side at all.
    pub(crate) fn run_in_simulator(rung: &Rung) -> Self {
        Self::new(["run", &rung.id])
    }

    /// Run one of a rung's tests alone, in the simulator; the tool records nothing. Only
    /// [`crate::show::test_button`] makes one. The tool's `run --only` has no car side.
    pub(crate) fn run_one_in_simulator(rung: &Rung, test: &SingleTest) -> Self {
        Self::new(["run", &rung.id, "--only", &test.name])
    }

    /// Run a check that moves no car (the code's own tests, `sim-check`, `bench-check`);
    /// the tool records it. Only [`crate::show::check_button`] makes one.
    pub(crate) fn run_check(check: &Check) -> Self {
        Self::new(["run", &check.id])
    }

    /// Open a run's report page in the browser; the tool writes the page first when the
    /// run has none (a run from a terminal). It runs no test.
    #[must_use]
    pub fn open_report(rung: &Rung, which: Which) -> Self {
        let mut command = Self::new(["report", &rung.id, "--open"]);
        if which == Which::Recorded {
            command.args.push("--recorded".into());
        }
        command
    }

    /// One of a rung's simulator sessions, as JSON: the tool reads the report's files.
    #[must_use]
    pub fn session(rung: &Rung, which: Which) -> Self {
        let mut command = Self::new(["session", &rung.id, "--json"]);
        if which == Which::Recorded {
            command.args.push("--recorded".into());
        }
        command
    }

    /// Record a result by hand. It runs nothing: `--car` only says where a person ran it.
    #[must_use]
    pub fn record(rung: &Rung, form: &RecordForm) -> Self {
        let mut command = Self::new(["record", &rung.id, form.outcome.word()]);
        if form.place == Place::Car {
            command.args.push("--car".into());
        }
        // `--name=value`: a note that starts with a dash stays a note.
        for (name, value) in [
            ("--note", &form.note),
            ("--override", &form.override_reason),
        ] {
            let value = value.trim();
            if !value.is_empty() {
                command.args.push(format!("{name}={value}"));
            }
        }
        command
    }

    fn new<'a>(args: impl IntoIterator<Item = &'a str>) -> Self {
        Self {
            args: std::iter::once(TOOL)
                .chain(args)
                .map(String::from)
                .collect(),
        }
    }

    /// The arguments after the interpreter, the tool's path first.
    #[must_use]
    pub fn args(&self) -> &[String] {
        &self.args
    }

    /// As a person would read it in a terminal.
    #[must_use]
    pub fn line(&self) -> String {
        format!("{PROGRAM} {}", self.args.join(" "))
    }

    /// The process to start, in the repository's root. Python prints each line as it comes.
    #[must_use]
    pub fn process(&self, root: &Path) -> Command {
        let mut process = Command::new(PROGRAM);
        process
            .args(&self.args)
            .current_dir(root)
            .env("PYTHONUNBUFFERED", "1");
        process
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pretend;

    fn rung(id: &str) -> Rung {
        pretend::ladder()
            .rungs
            .into_iter()
            .find(|r| r.id == id)
            .unwrap_or_else(|| panic!("the pretend ladder has no {id}"))
    }

    #[test]
    fn the_table_is_the_tools_json() {
        assert_eq!(ToolCommand::table().args(), ["tools/ladder.py", "--json"]);
        assert_eq!(
            ToolCommand::table().line(),
            "python3 tools/ladder.py --json"
        );
    }

    #[test]
    fn a_session_is_asked_of_the_tool_and_a_single_test_is_its_run_only() {
        let stands = rung("stands");
        assert_eq!(
            ToolCommand::session(&stands, Which::Newest).args(),
            ["tools/ladder.py", "session", "stands", "--json"]
        );
        assert_eq!(
            ToolCommand::session(&stands, Which::Recorded).args(),
            [
                "tools/ladder.py",
                "session",
                "stands",
                "--json",
                "--recorded"
            ]
        );
        let estop = stands.test("estop").expect("stands has the e-stop");
        assert_eq!(
            ToolCommand::run_one_in_simulator(&stands, estop).args(),
            ["tools/ladder.py", "run", "stands", "--only", "estop"]
        );
    }

    #[test]
    fn the_report_is_opened_by_the_tool_and_a_check_is_its_run() {
        let stands = rung("stands");
        assert_eq!(
            ToolCommand::open_report(&stands, Which::Newest).args(),
            ["tools/ladder.py", "report", "stands", "--open"]
        );
        assert_eq!(
            ToolCommand::open_report(&stands, Which::Recorded).args(),
            [
                "tools/ladder.py",
                "report",
                "stands",
                "--open",
                "--recorded"
            ]
        );
        let ladder = pretend::ladder();
        assert_eq!(
            ToolCommand::run_check(&ladder.checks[0]).args(),
            ["tools/ladder.py", "run", "code"]
        );
    }

    #[test]
    fn a_plain_record_in_the_simulator() {
        let command = ToolCommand::record(&rung("stands"), &RecordForm::default());
        assert_eq!(
            command.args(),
            ["tools/ladder.py", "record", "stands", "pass"]
        );
    }

    #[test]
    fn a_car_record_with_a_note_and_an_override() {
        let form = RecordForm {
            outcome: Outcome::Fail,
            place: Place::Car,
            note: "  0.3 m/s, on stands ".into(),
            override_reason: "--the simulator was run last week".into(),
        };
        assert_eq!(
            ToolCommand::record(&rung("stands"), &form).args(),
            [
                "tools/ladder.py",
                "record",
                "stands",
                "fail",
                "--car",
                "--note=0.3 m/s, on stands",
                "--override=--the simulator was run last week",
            ]
        );
    }

    #[test]
    fn blank_fields_are_left_out() {
        let form = RecordForm {
            note: "   ".into(),
            ..RecordForm::default()
        };
        let command = ToolCommand::record(&rung("stands"), &form);
        assert!(command.args().iter().all(|a| !a.starts_with("--")));
    }

    #[test]
    fn the_form_expects_an_override_only_for_a_pass_on_a_shut_side() {
        let mut stands = rung("stands");
        stands.car.blocked_by = vec!["rung 1 (stands) has not passed in the simulator".into()];
        let mut form = RecordForm {
            place: Place::Car,
            ..RecordForm::default()
        };
        assert!(form.override_expected(&stands));
        form.outcome = Outcome::Fail;
        assert!(!form.override_expected(&stands));
        assert!(!RecordForm::default().override_expected(&stands));
    }

    #[test]
    fn the_process_starts_in_the_repository_root() {
        let process = ToolCommand::table().process(Path::new("/somewhere"));
        assert_eq!(process.get_program(), "python3");
        assert_eq!(process.get_current_dir(), Some(Path::new("/somewhere")));
    }
}
