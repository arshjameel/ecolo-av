//! Sample data for `--pretend`: the ladder part-way up, with every kind of cell in it, so
//! the window can be shown with no tool and no progress file. Nothing runs from it.

use crate::{
    ladder::{Ladder, Rung, Side, SingleTest, State},
    report::Report,
};

/// The pretend ladder: the real rungs' names, made-up results.
#[must_use]
pub fn ladder() -> Ladder {
    let bench_shut = ["stack has not passed (sim)"];
    Ladder {
        file: "(pretend data: no file)".into(),
        rungs: vec![
            rung(
                ("code", "The code against the maths", "laptop"),
                "nothing; every unit test, the pretend car",
                "tools/test.sh exits 0",
                "docs/proving-it.md",
                done(
                    State::Pass,
                    "2026-11-20 09:12",
                    "pixi run bash tools/test.sh",
                ),
                done(State::Pass, "2026-11-20 09:12", ""),
            ),
            rung(
                ("stack", "The whole stack drives in Ravensim", "Ravensim"),
                "the graph, a drive, a stop",
                "every line of sim-check PASS",
                "docs/tests.md",
                done(State::Fail, "2026-11-21 14:40", "pixi run sim-check"),
                done(State::Fail, "2026-11-21 14:40", ""),
            ),
            rung(
                ("bench", "The bench, the real boards and no car", "bench"),
                "messages, timing, the locks",
                "every line of bench-check PASS; on the car, every box of After the install",
                "docs/after-install.md",
                shut("pixi run bench-check", &bench_shut),
                shut(
                    "",
                    &[
                        "stack has not passed (car)",
                        "bench has not passed in the simulator",
                    ],
                ),
            ),
            rung(
                (
                    "stands",
                    "Stands, wheels off the ground (phase A)",
                    "stands",
                ),
                "steering: angle, rate, centre; the wheels follow a speed; faults on purpose",
                "car-test --stands, every step PASS",
                "docs/actuator-tests.md",
                Side {
                    report: "~/aav-tests/2026-11-19-stands-sim".into(),
                    override_reason: "shown at the review before the bench was free".into(),
                    blocked_by: vec!["bench has not passed (sim)".into()],
                    ..done(
                        State::Pass,
                        "2026-11-19 16:05",
                        "pixi run car-test --stands --sim --attach --no-manual",
                    )
                },
                shut(
                    "pixi run car-test --stands",
                    &["bench has not passed (car)"],
                ),
            ),
            rung(
                (
                    "hallway",
                    "Hallway, the transfer functions (phase B)",
                    "hallway",
                ),
                "speed up, coast, brake: dead time, lag, stopping distance, with error bars",
                "the model's speed within 10 % of the car's, its stop within 0.2 m",
                "docs/actuator-tests.md",
                shut(
                    "pixi run car-test --hallway --sim --attach --no-manual",
                    &[],
                ),
                shut(
                    "pixi run car-test --hallway",
                    &[
                        "stands has not passed (car)",
                        "hallway has not passed in the simulator",
                    ],
                ),
            ),
            rung(
                ("lot", "The lot, the same at 2 to 3 m/s", "lot"),
                "speed-up, brake, circles and a straight line, at speed",
                "the hallway's pass rule, at the higher speeds",
                "docs/actuator-tests.md",
                shut(
                    "pixi run car-test --lot --sim --attach",
                    &["hallway has not passed (sim)"],
                ),
                shut(
                    "",
                    &[
                        "hallway has not passed (car)",
                        "lot has not passed in the simulator",
                    ],
                ),
            ),
            rung(
                ("route", "A route, the car driving itself", "route"),
                "the scorecard: time, metres off the plan, sway, holds",
                "the route done at low speed, no takeover needed for safety",
                "docs/league.md",
                shut(
                    "pixi run measure controllers --tours lot",
                    &["lot has not passed (sim)"],
                ),
                shut(
                    "",
                    &[
                        "lot has not passed (car)",
                        "route has not passed in the simulator",
                    ],
                ),
            ),
        ],
    }
}

fn rung(
    (id, title, location): (&str, &str, &str),
    measures: &str,
    passes: &str,
    how: &str,
    sim: Side,
    car: Side,
) -> Rung {
    Rung {
        id: id.into(),
        title: title.into(),
        location: location.into(),
        measures: measures.into(),
        passes: passes.into(),
        how: how.into(),
        after: match id {
            "stands" => "The last step sets the e-stop, and it stays set, as on the car: a \
                         person clears it (pixi run clear estop --sim)."
                .into(),
            _ => String::new(),
        },
        tests: tests(id),
        sim,
        car,
    }
}

/// A few of each rung's single tests, with the tool's own commands.
fn tests(id: &str) -> Vec<SingleTest> {
    let (names, command): (&[(&str, &str, &str)], &str) = match id {
        "stands" => (
            &[
                ("preflight", "1", "the preflight"),
                ("steer-sweep", "3a", "steering sweep (3.10)"),
                ("steer-rate", "3b", "steering rate (3.11)"),
                ("wheel-steps", "4", "throttle on free wheels (3.14)"),
                ("estop", "5h", "the e-stop"),
            ],
            "pixi run car-test --stands --sim --attach --no-manual --only",
        ),
        "hallway" => (
            &[
                ("coast", "B2", "hallway: stand still, speed up, coast"),
                ("speed-up", "B1", "hallway: stand still, speed up, brake"),
                ("brake", "B3", "hallway: stand still, speed up, brake hard"),
                ("steer", "B4", "hallway: steer left, right, left"),
            ],
            "pixi run car-test --hallway --sim --attach --no-manual --repeats 1 --kinds",
        ),
        "lot" => (
            &[
                ("speed-up", "4.1", "speed up: time to 1, 2, 3 m/s"),
                ("brake", "4.3", "brake: stopping distance"),
                ("circles", "4.5", "circles: the real wheel angle"),
            ],
            "pixi run car-test --lot --sim --attach --only",
        ),
        _ => (&[], ""),
    };
    names
        .iter()
        .map(|&(name, number, title)| SingleTest {
            name: name.into(),
            number: number.into(),
            title: title.into(),
            command: format!("{command} {name}"),
        })
        .collect()
}

/// A pretend session for a rung: two real ones from Ravensim (22 Nov), kept as the tool
/// printed them, so the window's table and graph can be shown with no tool.
#[must_use]
pub fn report(rung: &str) -> Option<Report> {
    let json = match rung {
        "stands" => include_str!("../tests/session-stands.json"),
        "hallway" => include_str!("../tests/session.json"),
        _ => return None,
    };
    Report::parse(json).ok()
}

/// A side with a result.
fn done(state: State, when: &str, command: &str) -> Side {
    Side {
        state,
        when: when.into(),
        commit: "a8365b6e".into(),
        report: String::new(),
        override_reason: String::new(),
        command: command.into(),
        blocked_by: Vec::new(),
    }
}

/// A side never run, and what keeps it shut.
fn shut(command: &str, blocked_by: &[&str]) -> Side {
    Side {
        state: State::NotRun,
        when: String::new(),
        commit: String::new(),
        report: String::new(),
        override_reason: String::new(),
        command: command.into(),
        blocked_by: blocked_by.iter().map(|&line| line.into()).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::show::{run_button, Cell, RunButton};

    #[test]
    fn the_pretend_ladder_has_the_real_rungs() {
        let real = Ladder::parse(include_str!("../tests/ladder.json")).expect("it reads");
        let ids = |l: &Ladder| l.rungs.iter().map(|r| r.id.clone()).collect::<Vec<_>>();
        assert_eq!(ids(&ladder()), ids(&real));
    }

    #[test]
    fn the_pretend_sessions_read_and_have_a_table_a_graph_and_fits() {
        let stands = report("stands").expect("the stands session reads");
        assert!(stands.recorded && stands.steps.len() > 10);
        assert!(stands.steps.iter().any(|s| s.graph.is_some()));
        assert!(stands.steps.iter().any(|s| !s.yaml.is_empty()));
        let hallway = report("hallway").expect("the hallway session reads");
        assert!(!hallway.fits.is_empty());
        assert!(report("code").is_none());
    }

    #[test]
    fn the_pretend_single_tests_are_the_simulators() {
        let ladder = ladder();
        let tests: Vec<_> = ladder.rungs.iter().flat_map(|r| &r.tests).collect();
        assert!(tests.len() > 8);
        assert!(tests.iter().all(|t| t.command.contains(" --sim ")));
    }

    #[test]
    fn it_shows_every_kind_of_cell() {
        let ladder = ladder();
        let words: Vec<_> = ladder
            .rungs
            .iter()
            .map(|r| Cell::of(&r.sim).words())
            .collect();
        assert!(words.iter().any(|w| w.starts_with("PASS")));
        assert!(words.iter().any(|w| w.starts_with("FAIL")));
        assert!(words.iter().any(|w| w == "not run"));
        assert!(words.iter().any(|w| w == "not run, shut"));
        assert!(ladder
            .rungs
            .iter()
            .any(|r| !r.sim.override_reason.is_empty()));
    }

    #[test]
    fn it_has_a_rung_that_could_run_and_one_that_could_not() {
        let ladder = ladder();
        let ready = |r: &Rung| matches!(run_button(r, None), RunButton::Ready(_));
        assert!(ladder.rungs.iter().any(ready));
        assert!(!ladder.rungs.iter().all(ready));
    }
}
