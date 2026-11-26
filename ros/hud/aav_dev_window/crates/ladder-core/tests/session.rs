//! The session against a stand-in tool (`fake_ladder.py`, as `tools/ladder.py` in a
//! temporary root): what the window asks the tool, and what it shows of the answers.
//! Needs `python3` on the path, as the window does.
#![cfg(unix)]

use std::{
    fs,
    path::{Path, PathBuf},
    thread,
    time::{Duration, Instant},
};

use ladder_core::{session::READ_EVERY, Outcome, Place, RunButton, Session, Source, State, WhyNot};

/// A repository root with only the stand-in tool in it; removed when dropped.
struct Root(PathBuf);

impl Root {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("ladder-core-{}-{name}", std::process::id()));
        let here = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests");
        fs::create_dir_all(dir.join("tools")).expect("a temporary folder");
        fs::copy(here.join("fake_ladder.py"), dir.join("tools/ladder.py")).expect("the tool");
        fs::copy(here.join("ladder.json"), dir.join("ladder.json")).expect("the ladder");
        Self(dir)
    }

    fn session(&self) -> Session {
        let mut session = Session::new(Source::Tool {
            root: self.0.clone(),
        });
        assert!(
            tick_until(&mut session, |s| s.ladder().is_some()),
            "the ladder never came: {:?}",
            session.problem()
        );
        session
    }

    /// Every call the tool got, its arguments joined by `|`.
    /// How many times the whole ladder was asked for.
    fn tables(&self) -> usize {
        self.calls().iter().filter(|c| *c == "--json").count()
    }

    fn calls(&self) -> Vec<String> {
        fs::read_to_string(self.0.join("calls.log"))
            .unwrap_or_default()
            .lines()
            .map(String::from)
            .collect()
    }
}

impl Drop for Root {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

/// Tick as the window does, until `done` or ten seconds.
fn tick_until(session: &mut Session, done: impl Fn(&Session) -> bool) -> bool {
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(10) {
        session.tick(Instant::now());
        if done(session) {
            return true;
        }
        thread::sleep(Duration::from_millis(20));
    }
    false
}

#[test]
fn it_reads_the_ladder_from_the_tool_and_again_every_two_seconds() {
    let root = Root::new("read");
    let mut session = root.session();
    assert_eq!(session.selected().map(|r| r.id.as_str()), Some("stands"));
    assert_eq!(root.tables(), 1);

    // Not before it is due; once it is, exactly one more.
    let later = Instant::now() + READ_EVERY / 2;
    session.tick(later);
    thread::sleep(Duration::from_millis(300));
    assert_eq!(root.tables(), 1);
    session.tick(Instant::now() + READ_EVERY);
    assert!(tick_until(&mut session, |_| root.tables() >= 2));
    assert_eq!(root.tables(), 2);
}

#[test]
fn a_tool_that_fails_is_a_problem_in_words_and_the_old_ladder_stays() {
    let root = Root::new("broken");
    let mut session = root.session();
    fs::write(
        root.0.join("tools/ladder.py"),
        "raise SystemExit('no yaml here')\n",
    )
    .expect("the tool is replaced");
    session.tick(Instant::now() + READ_EVERY);
    assert!(tick_until(&mut session, |s| s.problem().is_some()));
    assert!(session
        .problem()
        .is_some_and(|why| why.contains("no yaml here")));
    assert_eq!(session.ladder().map(|l| l.rungs.len()), Some(10));
}

#[test]
fn a_run_shows_its_lines_ends_and_the_ladder_is_read_again() {
    let root = Root::new("run");
    let mut session = root.session();
    assert!(matches!(session.run_button(), Some(RunButton::Ready(_))));
    session.press_run();
    assert_eq!(session.running().map(|r| r.rung), Some("stands"));

    assert!(tick_until(&mut session, |s| s.running().is_none()));
    assert!(tick_until(&mut session, |s| s.console().len() == 3));
    let console: Vec<_> = session.console().collect();
    assert_eq!(
        console,
        [
            "$ python3 tools/ladder.py run stands",
            "Running stands in the simulator",
            "Recorded: stands pass (sim) at fake",
        ]
    );
    assert!(session.last_run().is_some_and(|n| n.good));
    assert!(tick_until(&mut session, |_| root.tables() >= 2));
    let calls = root.calls();
    let run = calls.iter().position(|c| c == "run|stands");
    assert!(run.is_some_and(|at| calls[at..].iter().any(|c| c == "--json")));
}

#[test]
fn a_check_runs_like_a_rung_and_one_run_at_a_time_and_stop_ends_it() {
    let root = Root::new("stop");
    let mut session = root.session();
    assert!(matches!(
        session.check_button("stack"),
        Some(RunButton::Ready(_))
    ));
    session.press_run_check("stack");
    assert!(tick_until(&mut session, |s| s.console().len() == 2));

    // While it runs, no rung's button is on, and a second press starts nothing.
    assert_eq!(
        session.run_button(),
        Some(RunButton::Off(WhyNot::Running {
            rung: "stack".into()
        }))
    );
    session.press_run();
    session.press_run_check("code");
    assert_eq!(session.running().map(|r| r.rung), Some("stack"));

    session.press_stop();
    assert!(session.running().is_some_and(|r| r.stop_asked));
    assert!(tick_until(&mut session, |s| s.running().is_none()));
    let notice = session.last_run().expect("how it ended");
    assert!(!notice.good);
    assert_eq!(notice.text, "stack: stopped by you; nothing was recorded");
    assert_eq!(
        root.calls().iter().filter(|c| c.starts_with("run")).count(),
        1
    );
}

#[test]
fn a_rung_not_built_never_reaches_the_tools_run() {
    let root = Root::new("not-built");
    let mut session = root.session();
    session.select("speed");
    assert!(matches!(
        session.run_button(),
        Some(RunButton::Off(WhyNot::NotBuilt { .. }))
    ));
    session.press_run();
    assert!(session.running().is_none());
    assert!(root.calls().iter().all(|c| !c.starts_with("run")));
}

#[test]
fn open_the_report_asks_the_tool_and_shows_where_the_page_is() {
    let root = Root::new("open");
    let mut session = root.session();
    session.press_open_report();
    assert!(tick_until(&mut session, |s| s.opened().is_some()));
    let opened = session.opened().expect("the tool answered");
    assert!(opened.good, "{}", opened.text);
    assert_eq!(opened.text, "/tmp/fake/rung-1-report.html");
    assert!(root.calls().contains(&"report|stands|--open".to_owned()));
}

#[test]
fn a_record_goes_to_the_tool_and_its_refusal_is_shown_word_for_word() {
    let root = Root::new("record");
    let mut session = root.session();
    session.select("stands");
    assert_eq!(session.selected().map(|r| r.sim.state), Some(State::NotRun));
    session.form.outcome = Outcome::Pass;
    session.form.note = "by hand".into();
    session.press_record();
    assert!(session.is_recording());
    assert!(tick_until(&mut session, |s| s.recorded().is_some()));
    let refusal = session.recorded().expect("the tool answered");
    assert!(!refusal.good);
    assert_eq!(
        refusal.text,
        "Shut: rung 1 (stands) has not passed in the simulator. A pass here needs --override \"why\"."
    );
    assert_eq!(session.form.note, "by hand", "a refused form is kept");

    session.form.place = Place::Car;
    session.form.override_reason = "the simulator was run last week".into();
    session.press_record();
    assert!(tick_until(&mut session, |s| s.recorded().is_some()));
    let done = session.recorded().expect("the tool answered");
    assert!(done.good, "{}", done.text);
    assert_eq!(done.text, "Recorded: stands pass at fake");
    assert!(session.form.note.is_empty() && session.form.override_reason.is_empty());

    let records: Vec<_> = root
        .calls()
        .into_iter()
        .filter(|c| c.starts_with("record"))
        .collect();
    assert_eq!(
        records,
        [
            "record|stands|pass|--note=by hand",
            "record|stands|pass|--car|--note=by hand|--override=the simulator was run last week",
        ]
    );
}
