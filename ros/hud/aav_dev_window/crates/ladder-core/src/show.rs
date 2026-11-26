//! What the window shows for a rung: each cell's words, whether "Run in the simulator" is
//! on (and why not), and the car's side, which is only ever text.

use std::fmt;

use crate::{
    command::ToolCommand,
    jacks::Jacks,
    ladder::{Check, Rung, Side, SingleTest, State},
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
            shut: matches!(side.state, State::Fail | State::NotRun) && side.is_shut(),
            day: side.day(),
            why_shut: &side.blocked_by,
        }
    }

    /// The cell in words: `PASS 23 Nov`, `FAIL 23 Nov, shut`, `not run, shut`,
    /// `not built yet`.
    #[must_use]
    pub fn words(&self) -> String {
        let mut words = self.state.word().to_owned();
        if let Some(day) = self.day {
            words.push(' ');
            words.push_str(&short_day(day));
        }
        if self.shut {
            words.push_str(", shut");
        }
        words
    }
}

/// A day as a person says it: `2026-11-23` is `23 Nov`. Anything else is left as it is.
#[must_use]
pub fn short_day(day: &str) -> String {
    const MONTHS: [&str; 12] = [
        "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
    ];
    let mut parts = day.split('-');
    let (Some(_year), Some(month), Some(date), None) =
        (parts.next(), parts.next(), parts.next(), parts.next())
    else {
        return day.to_owned();
    };
    let name = month
        .parse::<usize>()
        .ok()
        .and_then(|m| MONTHS.get(m.wrapping_sub(1)));
    match (name, date.parse::<u8>()) {
        (Some(name), Ok(date)) => format!("{date} {name}"),
        _ => day.to_owned(),
    }
}

/// A check before you start, as one phrase: `the code's own tests: passed 23 Nov`,
/// `...: FAILED 23 Nov`, `...: not run`.
#[must_use]
pub fn check_words(check: &Check) -> String {
    let result = match check.side.state {
        State::Pass => "passed",
        State::Fail => "FAILED",
        State::NotRun | State::NotBuilt => "not run",
    };
    match check.side.day() {
        Some(day) if check.side.state != State::NotRun => {
            format!("{}: {result} {}", check.title, short_day(day))
        }
        _ => format!("{}: {result}", check.title),
    }
}

/// The Run button beside a check. A check is never shut, and shuts nothing; one run at a
/// time still holds.
#[must_use]
pub fn check_button(check: &Check, running: Option<&str>) -> RunButton {
    match running {
        Some(busy) => RunButton::Off(WhyNot::Running { rung: busy.into() }),
        None => RunButton::Ready(ToolCommand::run_check(check)),
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
    /// The rung is listed, and not built yet. `today`: what exists today.
    NotBuilt { today: String },
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
            Self::NotBuilt { today } => write!(f, "not built yet. Today: {today}"),
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
    if !rung.built {
        return RunButton::Off(WhyNot::NotBuilt {
            today: rung.today.clone(),
        });
    }
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
                "It runs in the simulator that is open on this computer: watch the car in \
                 its picture. The wheels spin and steer; the car stays where it is."
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

    fn real() -> Ladder {
        Ladder::parse(include_str!("../tests/ladder.json")).expect("the tool's JSON reads")
    }

    fn rung<'a>(ladder: &'a Ladder, id: &str) -> &'a Rung {
        ladder.rung(id).expect("the rung is there")
    }

    fn shut(ladder: &mut Ladder) -> &mut Rung {
        let stands = &mut ladder.rungs[0];
        stands.sim.blocked_by = vec!["rung 0 (below) has not passed (sim)".into()];
        stands
    }

    #[test]
    fn a_single_test_runs_even_on_a_shut_rung_but_one_run_at_a_time() {
        let mut ladder = real();
        let stands = shut(&mut ladder);
        let sweep = stands.test("steer-sweep").expect("stands has steer-sweep");
        let RunButton::Ready(command) = test_button(stands, sweep, None) else {
            panic!("a single test is never shut");
        };
        assert_eq!(
            command.args(),
            ["tools/ladder.py", "run", "stands", "--only", "steer-sweep"]
        );
        assert_eq!(
            test_button(stands, sweep, Some("code")),
            RunButton::Off(WhyNot::Running {
                rung: "code".into()
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
        assert!(RunsIn::Visible
            .to_string()
            .contains("the car stays where it is"));
        // "--attached" or a path with the word in it is not the flag.
        assert_eq!(RunsIn::of("tool --attached", &Jacks::OnGround), RunsIn::Own);
    }

    #[test]
    fn every_cell_says_its_state_in_words() {
        let mut ladder = real();
        assert_eq!(Cell::of(&rung(&ladder, "stands").sim).words(), "not run");
        assert_eq!(
            Cell::of(&rung(&ladder, "stands").car).words(),
            "not run, shut"
        );
        assert_eq!(
            Cell::of(&rung(&ladder, "speed").sim).words(),
            "not built yet"
        );
        assert_eq!(Cell::of(&ladder.checks[0].side).words(), "PASS 22 Nov");
        assert_eq!(Cell::of(&ladder.checks[1].side).words(), "FAIL 22 Nov");
        let stands = shut(&mut ladder);
        stands.sim.state = State::Fail;
        stands.sim.when = "2026-11-23 18:00".into();
        assert_eq!(Cell::of(&stands.sim).words(), "FAIL 23 Nov, shut");
    }

    #[test]
    fn a_day_is_said_as_a_person_says_it() {
        assert_eq!(short_day("2026-11-23"), "23 Nov");
        assert_eq!(short_day("2026-01-05"), "5 Jan");
        assert_eq!(short_day("2026-13-05"), "2026-13-05");
        assert_eq!(short_day("yesterday"), "yesterday");
    }

    #[test]
    fn the_checks_are_one_phrase_each_and_always_runnable() {
        let ladder = real();
        let words: Vec<_> = ladder.checks.iter().map(check_words).collect();
        assert_eq!(
            words,
            [
                "the code's own tests: passed 22 Nov",
                "the whole stack drives in the simulator: FAILED 22 Nov",
                "the bench stand-in: the real boards, no car: not run",
            ]
        );
        let RunButton::Ready(command) = check_button(&ladder.checks[1], None) else {
            panic!("a failed check can run again");
        };
        assert_eq!(command.args(), ["tools/ladder.py", "run", "stack"]);
        assert!(matches!(
            check_button(&ladder.checks[1], Some("stands")),
            RunButton::Off(WhyNot::Running { .. })
        ));
    }

    #[test]
    fn a_failed_check_never_shuts_a_rung() {
        let ladder = real();
        assert_eq!(ladder.checks[1].side.state, State::Fail);
        assert!(matches!(
            run_button(rung(&ladder, "stands"), None),
            RunButton::Ready(_)
        ));
    }

    #[test]
    fn a_shut_cell_carries_why() {
        let ladder = real();
        let cell = Cell::of(&rung(&ladder, "stands").car);
        assert!(cell.shut);
        assert_eq!(cell.why_shut.len(), 1);
    }

    #[test]
    fn a_passed_rung_is_not_marked_shut_as_in_the_tools_table() {
        let mut ladder = real();
        let stands = shut(&mut ladder);
        stands.sim.state = State::Pass;
        assert!(stands.sim.is_shut());
        assert!(!Cell::of(&stands.sim).shut);
    }

    #[test]
    fn an_open_rung_with_a_command_can_run() {
        let ladder = real();
        let RunButton::Ready(command) = run_button(rung(&ladder, "stands"), None) else {
            panic!("rung 1 is open and has a command");
        };
        assert_eq!(command.args(), ["tools/ladder.py", "run", "stands"]);
    }

    #[test]
    fn a_shut_rung_cannot_run_and_says_why() {
        let mut ladder = real();
        let RunButton::Off(why) = run_button(shut(&mut ladder), None) else {
            panic!("it is shut");
        };
        assert_eq!(why.to_string(), "shut: rung 0 (below) has not passed (sim)");
    }

    /// A rung not built is never runnable, whatever else is true of it.
    #[test]
    fn a_rung_not_built_cannot_run_even_with_a_command() {
        let mut ladder = real();
        for rung in ladder.rungs.iter_mut().filter(|r| !r.built) {
            rung.sim.command = "pixi run something".into();
            let RunButton::Off(WhyNot::NotBuilt { today }) = run_button(rung, None) else {
                panic!("{} is not built", rung.id);
            };
            assert!(!today.is_empty());
            assert!(!Cell::of(&rung.sim).shut, "not built is not 'shut'");
        }
    }

    #[test]
    fn a_rung_done_by_hand_cannot_run() {
        let mut ladder = real();
        let stands = &mut ladder.rungs[0];
        stands.sim.command.clear();
        assert_eq!(
            run_button(stands, None),
            RunButton::Off(WhyNot::ByHand {
                how: "docs/actuator-tests.md".into()
            })
        );
    }

    #[test]
    fn one_run_at_a_time() {
        let ladder = real();
        assert_eq!(
            run_button(rung(&ladder, "stands"), Some("code")),
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
                // Build and open everything, and give every car side a command: still
                // no car run.
                rung.built = true;
                rung.sim.blocked_by.clear();
                rung.car.blocked_by.clear();
                rung.sim.command = format!("pixi run car-test --{} --sim", rung.id);
                rung.car.command = format!("pixi run car-test --{}", rung.id);
                rung.car.state = State::NotRun;
                let RunButton::Ready(command) = run_button(rung, None) else {
                    panic!("{} is open", rung.id);
                };
                assert_eq!(command.args(), ["tools/ladder.py", "run", rung.id.as_str()]);
                assert!(command.args().iter().all(|a| !a.contains("car")));
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
            CarSide::of(rung(&ladder, "speed")),
            CarSide::ByHand {
                how: "docs/wrap-up.md"
            }
        );
    }

    #[test]
    fn both_columns_have_a_heading() {
        assert_eq!(Place::ALL.map(Place::heading), ["Simulator", "The car"]);
    }
}
