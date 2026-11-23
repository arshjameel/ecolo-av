//! What the window shows for a rung: each cell's words, whether "Run in the simulator" is
//! on (and why not), and the car's side, which is only ever text.

use std::fmt;

use crate::{
    command::ToolCommand,
    jacks::Jacks,
    ladder::{Rung, Side, SingleTest, State},
};

/// The sentence under the car's command. A safety rule: the window never moves the car.
pub const CAR_RULE: &str = "A person starts a car test from the terminal.";

/// One cell of the ladder: a rung in a place.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cell<'a> {
    pub state: State,
    /// Shut and not passed, as the tool's own table marks it.
    pub shut: bool,
    pub day: Option<&'a str>,
    /// What keeps it shut, a line each: for the hover and the detail.
    pub why_shut: &'a [String],
}

impl<'a> Cell<'a> {
    #[must_use]
    pub fn of(side: &'a Side) -> Self {
        Self {
            state: side.state,
            shut: side.state != State::Pass && side.is_shut(),
            day: side.day(),
            why_shut: &side.blocked_by,
        }
    }

    /// The cell in words: `PASS 2026-11-22`, `FAIL 2026-11-22, shut`, `not run, shut`.
    #[must_use]
    pub fn words(&self) -> String {
        let mut words = self.state.word().to_owned();
        if let Some(day) = self.day {
            words.push(' ');
            words.push_str(day);
        }
        if self.shut {
            words.push_str(", shut");
        }
        words
    }
}

/// Why "Run in the simulator" is off.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WhyNot {
    /// One run at a time: this rung is running.
    Running { rung: String },
    /// The tool would refuse: the rungs under it have not passed.
    Shut { blocked_by: Vec<String> },
    /// No one command runs it: a person does it, by the page.
    ByHand { how: String },
    /// `--pretend`: nothing runs.
    Pretend,
}

impl fmt::Display for WhyNot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Running { rung } => write!(f, "{rung} is running: one run at a time"),
            Self::Shut { blocked_by } => write!(f, "shut: {}", blocked_by.join("; ")),
            Self::ByHand { how } => {
                write!(f, "no one command runs it: by hand ({how}), then record it")
            }
            Self::Pretend => write!(f, "pretend data: nothing runs"),
        }
    }
}

/// The "Run in the simulator" button.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunButton {
    /// On: pressing it starts this command.
    Ready(ToolCommand),
    Off(WhyNot),
}

/// Decide the run button for a rung. `running`: the rung a run is busy with, if any.
///
/// It takes no place: the only run there is, is the simulator's.
#[must_use]
pub fn run_button(rung: &Rung, running: Option<&str>) -> RunButton {
    if let Some(busy) = running {
        return RunButton::Off(WhyNot::Running { rung: busy.into() });
    }
    if rung.sim.is_shut() {
        return RunButton::Off(WhyNot::Shut {
            blocked_by: rung.sim.blocked_by.clone(),
        });
    }
    if rung.sim.command.trim().is_empty() {
        return RunButton::Off(WhyNot::ByHand {
            how: rung.how.clone(),
        });
    }
    RunButton::Ready(ToolCommand::run_in_simulator(rung))
}

/// The Run button beside one of a rung's single tests. A single test is never shut: it
/// records nothing, so it cannot move the ladder; one run at a time still holds.
///
/// Like [`run_button`] it takes no place: the tool's `run --only` is the simulator's.
#[must_use]
pub fn test_button(rung: &Rung, test: &SingleTest, running: Option<&str>) -> RunButton {
    match running {
        Some(busy) => RunButton::Off(WhyNot::Running { rung: busy.into() }),
        None => RunButton::Ready(ToolCommand::run_one_in_simulator(rung, test)),
    }
}

/// Where a simulator run will happen: in the simulator that is open, or in one of its own.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunsIn {
    /// A simulator runs on this computer and the command attaches: the car you see is
    /// the one tested.
    Visible,
    /// The command attaches, but no simulator is open: it starts its own, with no picture.
    OwnNoneOpen,
    /// The command always starts its own (the unit tests, `sim-check`, `bench-check`).
    Own,
}

impl RunsIn {
    /// Decide from the command the tool will run and what the jacks poll last heard.
    #[must_use]
    pub fn of(command: &str, jacks: &Jacks) -> Self {
        let attaches = command.split_whitespace().any(|word| word == "--attach");
        let open = matches!(jacks, Jacks::OnGround | Jacks::OnStands | Jacks::Silent);
        match (attaches, open) {
            (true, true) => Self::Visible,
            (true, false) => Self::OwnNoneOpen,
            (false, _) => Self::Own,
        }
    }
}

impl fmt::Display for RunsIn {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Visible => {
                "Runs in the simulator that is open on this computer: watch the car there."
            }
            Self::OwnNoneOpen => {
                "No simulator is open: the run starts its own, with no picture. \
                 To watch it: pixi run dry-run."
            }
            Self::Own => "Starts what it needs itself, with no picture.",
        })
    }
}

/// The car's side of the detail: words to read and copy, never a button that runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarSide<'a> {
    /// The command a person types in a terminal.
    Command(&'a str),
    /// No one command: the page that says how.
    ByHand { how: &'a str },
}

impl<'a> CarSide<'a> {
    #[must_use]
    pub fn of(rung: &'a Rung) -> Self {
        match rung.car.command.trim() {
            "" => Self::ByHand { how: &rung.how },
            command => Self::Command(command),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ladder::{Ladder, Place},
        pretend,
    };

    #[test]
    fn a_single_test_runs_even_on_a_shut_rung_but_one_run_at_a_time() {
        let ladder = real();
        let stands = rung(&ladder, "stands");
        assert!(stands.sim.is_shut());
        let sweep = stands.test("steer-sweep").expect("stands has steer-sweep");
        let RunButton::Ready(command) = test_button(stands, sweep, None) else {
            panic!("a single test is never shut");
        };
        assert_eq!(
            command.args(),
            ["tools/ladder.py", "run", "stands", "--only", "steer-sweep"]
        );
        assert_eq!(
            test_button(stands, sweep, Some("hallway")),
            RunButton::Off(WhyNot::Running {
                rung: "hallway".into()
            })
        );
    }

    #[test]
    fn a_run_says_whether_it_will_be_seen() {
        let attach = "pixi run car-test --stands --sim --attach --no-manual";
        assert_eq!(RunsIn::of(attach, &Jacks::OnGround), RunsIn::Visible);
        assert_eq!(RunsIn::of(attach, &Jacks::OnStands), RunsIn::Visible);
        let none = Jacks::NoSimulator("no simulator heard".into());
        assert_eq!(RunsIn::of(attach, &none), RunsIn::OwnNoneOpen);
        assert_eq!(RunsIn::of(attach, &Jacks::Unknown), RunsIn::OwnNoneOpen);
        assert_eq!(
            RunsIn::of("pixi run sim-check", &Jacks::OnGround),
            RunsIn::Own
        );
        assert!(RunsIn::OwnNoneOpen.to_string().contains("pixi run dry-run"));
        // "--attached" or a path with the word in it is not the flag.
        assert_eq!(RunsIn::of("tool --attached", &Jacks::OnGround), RunsIn::Own);
    }

    fn real() -> Ladder {
        Ladder::parse(include_str!("../tests/ladder.json")).expect("the tool's JSON reads")
    }

    fn rung<'a>(ladder: &'a Ladder, id: &str) -> &'a Rung {
        ladder.rung(id).expect("the rung is there")
    }

    #[test]
    fn every_cell_says_its_state_in_words() {
        let ladder = real();
        assert_eq!(
            Cell::of(&rung(&ladder, "code").sim).words(),
            "PASS 2026-11-22"
        );
        assert_eq!(
            Cell::of(&rung(&ladder, "stack").sim).words(),
            "FAIL 2026-11-22"
        );
        assert_eq!(
            Cell::of(&rung(&ladder, "bench").sim).words(),
            "not run, shut"
        );
    }

    #[test]
    fn a_shut_cell_carries_why() {
        let ladder = real();
        let cell = Cell::of(&rung(&ladder, "stands").car);
        assert!(cell.shut);
        assert_eq!(cell.why_shut.len(), 2);
    }

    #[test]
    fn a_passed_rung_is_not_marked_shut_as_in_the_tools_table() {
        let mut ladder = real();
        let stands = &mut ladder.rungs[3];
        stands.sim.state = State::Pass;
        assert!(stands.sim.is_shut());
        assert!(!Cell::of(&stands.sim).shut);
    }

    #[test]
    fn an_open_rung_with_a_command_can_run() {
        let ladder = real();
        let RunButton::Ready(command) = run_button(rung(&ladder, "stack"), None) else {
            panic!("stack is open and has a command");
        };
        assert_eq!(command.args(), ["tools/ladder.py", "run", "stack"]);
    }

    #[test]
    fn a_shut_rung_cannot_run_and_says_why() {
        let ladder = real();
        let RunButton::Off(why) = run_button(rung(&ladder, "stands"), None) else {
            panic!("stands is shut");
        };
        assert_eq!(why.to_string(), "shut: bench has not passed (sim)");
    }

    #[test]
    fn a_rung_done_by_hand_cannot_run() {
        let mut ladder = real();
        let lot = &mut ladder.rungs[5];
        lot.sim.blocked_by.clear();
        lot.sim.command.clear();
        assert_eq!(
            run_button(lot, None),
            RunButton::Off(WhyNot::ByHand {
                how: "docs/actuator-tests.md".into()
            })
        );
    }

    #[test]
    fn one_run_at_a_time() {
        let ladder = real();
        assert_eq!(
            run_button(rung(&ladder, "stack"), Some("code")),
            RunButton::Off(WhyNot::Running {
                rung: "code".into()
            })
        );
    }

    /// The safety rule. Whatever the ladder says, a run is `run <rung>`, which the tool
    /// runs in the simulator only; the car's command never reaches a command line.
    #[test]
    fn the_car_side_never_makes_a_run_command() {
        for mut ladder in [real(), pretend::ladder()] {
            for rung in &mut ladder.rungs {
                // Open everything, and give every car side a command: still no car run.
                rung.sim.blocked_by.clear();
                rung.car.blocked_by.clear();
                rung.car.command = format!("pixi run car-test --{}", rung.id);
                rung.car.state = State::NotRun;
                if let RunButton::Ready(command) = run_button(rung, None) {
                    assert_eq!(command.args(), ["tools/ladder.py", "run", rung.id.as_str()]);
                    assert!(command.args().iter().all(|a| !a.contains("car")));
                }
                assert_eq!(
                    CarSide::of(rung),
                    CarSide::Command(rung.car.command.as_str())
                );
            }
        }
    }

    #[test]
    fn the_car_side_is_its_command_or_the_page() {
        let ladder = real();
        assert_eq!(
            CarSide::of(rung(&ladder, "stands")),
            CarSide::Command("pixi run car-test --stands")
        );
        assert_eq!(
            CarSide::of(rung(&ladder, "lot")),
            CarSide::ByHand {
                how: "docs/actuator-tests.md"
            }
        );
    }

    #[test]
    fn both_columns_have_a_heading() {
        assert_eq!(Place::ALL.map(Place::heading), ["Simulator", "The car"]);
    }
}
