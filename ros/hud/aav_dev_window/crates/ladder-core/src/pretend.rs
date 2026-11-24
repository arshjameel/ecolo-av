//! Sample data for `--pretend`: the ladder with rung 1 passed in the simulator, so the
//! window and a rung's answer can be shown with no tool and no progress file. Nothing
//! runs from it.

use crate::{
    ladder::{Ladder, State},
    report::Report,
};

/// The tool's own table, kept as it printed it (`tests/ladder.json`).
const LADDER: &str = include_str!("../tests/ladder.json");
/// A real run of rung 1 in Ravensim (23 Nov), kept as the tool printed it.
const STANDS: &str = include_str!("../tests/session-stands.json");

/// The pretend ladder: the real rungs and checks; rung 1's simulator side made a pass,
/// so the car's side is open and not run. (The JSON is compiled in, and a test reads it:
/// an empty ladder would mean a broken build, not a broken computer.)
#[must_use]
pub fn ladder() -> Ladder {
    let mut ladder = Ladder::parse(LADDER).unwrap_or_else(|_| Ladder {
        file: String::new(),
        rungs: Vec::new(),
        checks: Vec::new(),
        note: String::new(),
    });
    ladder.file = "(pretend data: no file)".into();
    if let Some(stands) = ladder.rungs.iter_mut().find(|r| r.id == "stands") {
        stands.sim.state = State::Pass;
        stands.sim.when = "2026-11-23 18:04".into();
        stands.sim.commit = "69aa3122".into();
        stands.sim.report = "~/aav-tests/2026-11-23-180055-stands-sim".into();
        stands.car.blocked_by.clear();
    }
    ladder
}

/// A pretend run for a rung: rung 1's, with its answer; the others have none.
#[must_use]
pub fn report(rung: &str) -> Option<Report> {
    match rung {
        "stands" => Report::parse(STANDS).ok(),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::show::{run_button, Cell, RunButton, WhyNot};

    #[test]
    fn the_pretend_ladder_has_the_real_rungs_and_checks() {
        let ladder = ladder();
        assert_eq!(ladder.rungs.len(), 10);
        assert_eq!(ladder.checks.len(), 3);
        assert_eq!(
            ladder.first_not_passed().map(|r| r.id.as_str()),
            Some("stands")
        );
    }

    #[test]
    fn the_pretend_run_reads_and_has_an_answer_with_graphs() {
        let stands = report("stands").expect("the stands run reads");
        assert!(stands.found && stands.steps.len() > 10);
        let answer = stands.answer.as_ref().expect("rung 1 has an answer");
        assert!(answer.found && answer.graphs.len() == 3);
        assert!(answer.graphs.iter().all(|g| g.missing.is_empty()));
        assert!(report("speed").is_none());
    }

    #[test]
    fn it_shows_every_kind_of_cell() {
        let ladder = ladder();
        let words: Vec<String> = ladder
            .rungs
            .iter()
            .flat_map(|r| [Cell::of(&r.sim).words(), Cell::of(&r.car).words()])
            .chain(ladder.checks.iter().map(|c| Cell::of(&c.side).words()))
            .collect();
        for kind in ["PASS", "FAIL", "not run", "not built yet"] {
            assert!(words.iter().any(|w| w.starts_with(kind)), "no {kind} cell");
        }
    }

    #[test]
    fn a_built_rung_could_run_and_one_not_built_could_not() {
        let ladder = ladder();
        assert!(matches!(
            run_button(&ladder.rungs[0], None),
            RunButton::Ready(_)
        ));
        assert!(matches!(
            run_button(&ladder.rungs[1], None),
            RunButton::Off(WhyNot::NotBuilt { .. })
        ));
    }
}
