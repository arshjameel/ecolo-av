//! The window's state, without the window: the ladder read from the tool every 2 s and
//! after every action, one run at a time, and the "Record by hand" form.
//!
//! It never opens the progress file and holds no rule: it asks `tools/ladder.py`, and
//! shows what the tool answers, its refusals word for word.

use std::{
    collections::VecDeque,
    path::PathBuf,
    process::ExitStatus,
    sync::mpsc::{Receiver, TryRecvError},
    time::{Duration, Instant},
};

use crate::{
    command::{RecordForm, ToolCommand, Which},
    jacks::{self, Ask, Jacks},
    ladder::{Ladder, Rung},
    pretend,
    process::{self, Answer, Running, Signal},
    report::{self, Report, StepRow},
    research::{Watch, WatchButton},
    show::{self, RunButton, RunsIn, WhyNot},
};

/// How often the ladder is read again.
pub const READ_EVERY: Duration = Duration::from_secs(2);
/// The console keeps this many of a run's last lines.
pub const CONSOLE_LINES: usize = 200;

/// Where the ladder comes from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Source {
    /// `tools/ladder.py`, in this repository.
    Tool { root: PathBuf },
    /// Built-in sample data: nothing runs, nothing is recorded.
    Pretend,
}

/// One line the window tells the person, good news or bad.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Notice {
    pub good: bool,
    pub text: String,
}

impl Notice {
    fn good(text: impl Into<String>) -> Self {
        Self {
            good: true,
            text: text.into(),
        }
    }

    fn bad(text: impl Into<String>) -> Self {
        Self {
            good: false,
            text: text.into(),
        }
    }
}

/// The run in progress, for the screen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunningNow<'a> {
    pub rung: &'a str,
    /// The single test it runs, or `None`: the whole rung.
    pub test: Option<&'a str>,
    /// The step it is on, from its own lines: `Step 3b: steering rate (3.11)`.
    pub step: Option<&'a str>,
    /// Stop was pressed: the next press does not ask.
    pub stop_asked: bool,
}

#[derive(Debug)]
struct Run {
    rung: String,
    /// The single test, or `None`: the whole rung, which the tool records.
    test: Option<String>,
    child: Running,
    stop_asked: bool,
    /// A research watch, not the ladder's: nothing is recorded.
    research: bool,
}

type Pending = Receiver<Result<Answer, String>>;

/// Everything the window shows and does.
#[derive(Debug)]
pub struct Session {
    source: Source,
    ladder: Option<Ladder>,
    problem: Option<String>,
    selected: Option<String>,
    /// The "Record by hand" form, edited in place by the window.
    pub form: RecordForm,
    run: Option<Run>,
    /// The run that just ended: its last lines may still be on their way.
    ended: Option<Running>,
    console: VecDeque<String>,
    last_run: Option<Notice>,
    recording: Option<Pending>,
    recorded: Option<Notice>,
    reading: Option<Pending>,
    read_at: Option<Instant>,
    read_now: bool,
    /// The simulated car: on stands or on the ground, as `tools/jacks.py` last said.
    jacks: Jacks,
    /// The jacks call in progress, and what it asked.
    jacks_asking: Option<(Ask, Pending)>,
    /// When the last jacks answer came: the next status is asked `jacks::ASK_EVERY` later.
    jacks_at: Option<Instant>,
    jacks_said: Option<Notice>,
    /// The selected rung's newest simulator session, as the tool last read it.
    report: Option<Report>,
    /// The session call in progress, and the rung it asked about.
    reporting: Option<(String, Pending)>,
    /// The session shown is stale: the rung changed, or a run ended.
    report_now: bool,
    /// The step whose numbers and graph are shown.
    step: Option<String>,
    /// Which session is shown: the newest run, or the one the ladder recorded.
    which: Which,
    /// The "Open the report" call in progress, and what the tool last said to one.
    opening: Option<Pending>,
    opened: Option<Notice>,
}

impl Session {
    #[must_use]
    pub fn new(source: Source) -> Self {
        let ladder = (source == Source::Pretend).then(pretend::ladder);
        let mut session = Self {
            source,
            ladder,
            problem: None,
            selected: None,
            form: RecordForm::default(),
            run: None,
            ended: None,
            console: VecDeque::new(),
            last_run: None,
            recording: None,
            recorded: None,
            reading: None,
            read_at: None,
            read_now: true,
            jacks: Jacks::Unknown,
            jacks_asking: None,
            jacks_at: None,
            jacks_said: None,
            report: None,
            reporting: None,
            report_now: true,
            step: None,
            which: Which::default(),
            opening: None,
            opened: None,
        };
        if session.source == Source::Pretend {
            session.jacks = Jacks::Pretend;
        }
        session.keep_selection();
        session
    }

    /// Once a frame: take what the children have said, and read the ladder when it is due.
    pub fn tick(&mut self, now: Instant) {
        self.follow_run();
        self.take_record();
        self.take_ladder();
        self.take_jacks(now);
        self.take_report();
        self.take_opened();
        let Source::Tool { root } = &self.source else {
            if self.report_now {
                self.report_now = false;
                self.report = self.selected.as_deref().and_then(pretend::report);
                self.keep_step();
            }
            return;
        };
        // Not before the ladder is read: until then no rung is known to have sessions.
        if self.report_now && self.reporting.is_none() && self.ladder.is_some() {
            self.report_now = false;
            // Only a rung with tests of its own has sessions to read.
            match self.selected().filter(|rung| !rung.tests.is_empty()) {
                Some(rung) => {
                    let asked = process::ask(ToolCommand::session(rung, self.which).process(root));
                    self.reporting = Some((rung.id.clone(), asked));
                }
                None => self.report = None,
            }
        }
        let jacks_due = self
            .jacks_at
            .is_none_or(|at| now.saturating_duration_since(at) >= jacks::ASK_EVERY);
        if self.jacks_asking.is_none() && jacks_due {
            self.jacks_asking = Some((Ask::Status, process::ask(Ask::Status.process(root))));
        }
        let due = self
            .read_at
            .is_none_or(|at| now.saturating_duration_since(at) >= READ_EVERY);
        if self.reading.is_none() && (self.read_now || due) {
            self.reading = Some(process::ask(ToolCommand::table().process(root)));
            self.read_at = Some(now);
            self.read_now = false;
        }
    }

    #[must_use]
    pub fn source(&self) -> &Source {
        &self.source
    }

    /// The ladder as last read; `None` until the tool has answered once.
    #[must_use]
    pub fn ladder(&self) -> Option<&Ladder> {
        self.ladder.as_ref()
    }

    /// Why the last read failed. The ladder shown is then the one before.
    #[must_use]
    pub fn problem(&self) -> Option<&str> {
        self.problem.as_deref()
    }

    #[must_use]
    pub fn selected(&self) -> Option<&Rung> {
        self.ladder.as_ref()?.rung(self.selected.as_deref()?)
    }

    pub fn select(&mut self, id: &str) {
        if self.selected.as_deref() != Some(id) {
            self.selected = Some(id.to_owned());
            self.recorded = None;
            self.opened = None;
            self.report = None;
            self.step = None;
            self.report_now = true;
        }
    }

    /// The selected rung's newest simulator session; `None`: not read yet, or the rung
    /// has no sessions (the unit tests, `sim-check`).
    #[must_use]
    pub fn report(&self) -> Option<&Report> {
        self.report
            .as_ref()
            .filter(|report| Some(report.rung.as_str()) == self.selected.as_deref())
    }

    /// The step whose numbers and graph are shown.
    #[must_use]
    pub fn step(&self) -> Option<&StepRow> {
        self.report()?.step(self.step.as_deref()?)
    }

    /// Which session is shown.
    #[must_use]
    pub fn which(&self) -> Which {
        self.which
    }

    /// Show the newest run, or the session the ladder recorded. A run that ends shows
    /// the newest again: it is what was just done.
    pub fn show(&mut self, which: Which) {
        if self.which != which {
            self.which = which;
            self.step = None;
            self.report_now = true;
        }
    }

    /// A row of the session's table was clicked.
    pub fn select_step(&mut self, name: &str) {
        self.step = Some(name.to_owned());
    }

    /// Where the selected rung's simulator run would happen: in the simulator that is
    /// open, or in one of its own.
    #[must_use]
    pub fn runs_in(&self) -> Option<RunsIn> {
        let rung = self.selected()?;
        (!rung.sim.command.trim().is_empty()).then(|| RunsIn::of(&rung.sim.command, &self.jacks))
    }

    #[must_use]
    pub fn running(&self) -> Option<RunningNow<'_>> {
        self.run.as_ref().map(|run| RunningNow {
            rung: &run.rung,
            test: run.test.as_deref(),
            step: report::current_step(self.console.iter().map(String::as_str)),
            stop_asked: run.stop_asked,
        })
    }

    /// The run's last lines, oldest first.
    pub fn console(&self) -> impl ExactSizeIterator<Item = &str> {
        self.console.iter().map(String::as_str)
    }

    /// How the last run ended.
    #[must_use]
    pub fn last_run(&self) -> Option<&Notice> {
        self.last_run.as_ref()
    }

    /// What the tool said to the last "Record": its line, or its refusal.
    #[must_use]
    pub fn recorded(&self) -> Option<&Notice> {
        self.recorded.as_ref()
    }

    #[must_use]
    pub fn is_recording(&self) -> bool {
        self.recording.is_some()
    }

    /// The "Run in the simulator" button for the selected rung.
    #[must_use]
    pub fn run_button(&self) -> Option<RunButton> {
        let rung = self.selected()?;
        Some(match self.source {
            Source::Pretend => RunButton::Off(WhyNot::Pretend),
            Source::Tool { .. } => {
                show::run_button(rung, self.run.as_ref().map(|r| r.rung.as_str()))
            }
        })
    }

    /// "Run in the simulator" was pressed. Nothing starts unless the button is on.
    pub fn press_run(&mut self) {
        if let Some(RunButton::Ready(command)) = self.run_button() {
            self.start(&command, None);
        }
    }

    /// The Run button beside one of the selected rung's single tests.
    #[must_use]
    pub fn test_button(&self, test: &str) -> Option<RunButton> {
        let rung = self.selected()?;
        let test = rung.test(test)?;
        Some(match self.source {
            Source::Pretend => RunButton::Off(WhyNot::Pretend),
            Source::Tool { .. } => {
                show::test_button(rung, test, self.run.as_ref().map(|r| r.rung.as_str()))
            }
        })
    }

    /// A single test's Run was pressed. Nothing starts unless its button is on. The tool
    /// runs it in the simulator and records nothing.
    pub fn press_run_test(&mut self, test: &str) {
        if let Some(RunButton::Ready(command)) = self.test_button(test) {
            self.start(&command, Some(test));
        }
    }

    /// The Run button beside a check (before you start). It moves no car.
    #[must_use]
    pub fn check_button(&self, id: &str) -> Option<RunButton> {
        let check = self.ladder.as_ref()?.checks.iter().find(|c| c.id == id)?;
        Some(match self.source {
            Source::Pretend => RunButton::Off(WhyNot::Pretend),
            Source::Tool { .. } => {
                show::check_button(check, self.run.as_ref().map(|r| r.rung.as_str()))
            }
        })
    }

    /// A check's Run was pressed. Nothing starts unless its button is on; the tool
    /// records its result, which opens and shuts no rung.
    pub fn press_run_check(&mut self, id: &str) {
        if let Some(RunButton::Ready(command)) = self.check_button(id) {
            self.start_named(id.to_owned(), &command, None);
        }
    }

    /// "Open the report" was pressed: the tool opens the run's page in the browser, and
    /// writes it first when the run has none. It runs no test.
    pub fn press_open_report(&mut self) {
        let (Source::Tool { root }, Some(rung)) = (&self.source, self.selected()) else {
            self.opened = Some(Notice::bad("pretend data: there is no page to open"));
            return;
        };
        if self.opening.is_none() {
            let command = ToolCommand::open_report(rung, self.which);
            self.opening = Some(process::ask(command.process(root)));
            self.opened = None;
        }
    }

    /// What the tool said to the last "Open the report": where the page is, or why not.
    #[must_use]
    pub fn opened(&self) -> Option<&Notice> {
        self.opened.as_ref()
    }

    fn take_opened(&mut self) {
        let Some(answer) = take(&mut self.opening) else {
            return;
        };
        self.report_now = true; // the page may have just been written: show where it is
        self.opened = Some(match answer {
            Ok(answer) if answer.succeeded() => Notice::good(answer.stdout.trim()),
            Ok(answer) => Notice::bad(first_words(&answer.stderr, &answer.stdout)),
            Err(why) => Notice::bad(why),
        });
    }

    fn start(&mut self, command: &ToolCommand, test: Option<&str>) {
        if let Some(rung) = self.selected().map(|rung| rung.id.clone()) {
            self.start_named(rung, command, test);
        }
    }

    fn start_named(&mut self, rung: String, command: &ToolCommand, test: Option<&str>) {
        let Source::Tool { root } = &self.source else {
            return;
        };
        let process = command.process(root);
        self.console.clear();
        self.say(format!("$ {}", command.line()));
        self.ended = None;
        match Running::start(process) {
            Ok(child) => {
                self.last_run = None;
                self.run = Some(Run {
                    rung,
                    test: test.map(str::to_owned),
                    child,
                    stop_asked: false,
                    research: false,
                });
            }
            Err(why) => {
                self.last_run = Some(Notice::bad(format!("{rung} did not start: {why}")));
            }
        }
    }

    /// A research watch's Run button (`research`): off with pretend data and while
    /// anything runs. It is not the ladder's: no rung need be chosen, none is shut.
    #[must_use]
    pub fn watch_button(&self, watch: Watch) -> WatchButton {
        match (&self.source, &self.run) {
            (Source::Pretend, _) => WatchButton::Off(WhyNot::Pretend),
            (_, Some(run)) => WatchButton::Off(WhyNot::Running {
                rung: run.rung.clone(),
            }),
            (Source::Tool { .. }, None) => WatchButton::Ready(watch.command()),
        }
    }

    /// A research watch's Run was pressed. Nothing starts unless its button is on; the
    /// ladder's tool is not called, so nothing is recorded.
    pub fn press_watch(&mut self, watch: Watch) {
        let (Source::Tool { root }, WatchButton::Ready(command)) =
            (&self.source, self.watch_button(watch))
        else {
            return;
        };
        let process = command.process(root);
        self.console.clear();
        self.say(format!("$ {}", command.line()));
        self.ended = None;
        match Running::start(process) {
            Ok(child) => {
                self.last_run = None;
                self.run = Some(Run {
                    rung: watch.name(),
                    test: None,
                    child,
                    stop_asked: false,
                    research: true,
                });
            }
            Err(why) => {
                self.last_run = Some(Notice::bad(format!(
                    "{} did not start: {why}",
                    watch.name()
                )));
            }
        }
    }

    /// "Stop" was pressed: the run's whole process group is asked to end; pressed again,
    /// it is killed.
    pub fn press_stop(&mut self) {
        let Some(run) = &mut self.run else {
            return;
        };
        let signal = if run.stop_asked {
            Signal::Kill
        } else {
            Signal::Terminate
        };
        let sent = run.child.stop(signal);
        run.stop_asked = true;
        match sent {
            Ok(()) => self.say(match signal {
                Signal::Terminate => "(stop asked)",
                Signal::Kill => "(killed)",
            }),
            Err(why) => self.say(format!("(the stop was not sent: {why})")),
        }
    }

    /// The simulated car: on stands, on the ground, or why nobody can say.
    #[must_use]
    pub fn jacks(&self) -> &Jacks {
        &self.jacks
    }

    /// What the jacks button would ask, if it is on: a simulator runs on this computer,
    /// and no jacks request is on its way.
    #[must_use]
    pub fn jacks_button(&self) -> Option<Ask> {
        let busy = matches!(self.jacks_asking, Some((Ask::Up | Ask::Down, _)));
        self.jacks.button().filter(|_| !busy)
    }

    /// What the simulator said to the last press: done, or its refusal word for word.
    #[must_use]
    pub fn jacks_said(&self) -> Option<&Notice> {
        self.jacks_said.as_ref()
    }

    /// The jacks button was pressed. Nothing is asked unless the button is on; the tool
    /// and the simulator still decide (a car whose wheels spin is not lowered).
    pub fn press_jacks(&mut self) {
        let (Some(ask), Source::Tool { root }) = (self.jacks_button(), &self.source) else {
            return;
        };
        // A status on its way is dropped: its answer would be older than this request's.
        self.jacks_asking = Some((ask, process::ask(ask.process(root))));
        self.jacks_said = None;
    }

    /// "Record" was pressed: the form goes to the tool, which may refuse.
    pub fn press_record(&mut self) {
        let Some(rung) = self.selected() else {
            return;
        };
        let command = ToolCommand::record(rung, &self.form);
        match &self.source {
            Source::Pretend => {
                self.recorded = Some(Notice::bad("pretend data: nothing is recorded"));
            }
            Source::Tool { root } if self.recording.is_none() => {
                self.recording = Some(process::ask(command.process(root)));
                self.recorded = None;
            }
            Source::Tool { .. } => {}
        }
    }

    fn say(&mut self, line: impl Into<String>) {
        self.console.push_back(line.into());
        while self.console.len() > CONSOLE_LINES {
            self.console.pop_front();
        }
    }

    fn follow_run(&mut self) {
        let mut lines = Vec::new();
        if let Some(ended) = &mut self.ended {
            lines.extend(ended.new_lines());
        }
        let mut status = None;
        if let Some(run) = &mut self.run {
            lines.extend(run.child.new_lines());
            status = match run.child.ended() {
                Ok(status) => status.map(Ok),
                Err(why) => Some(Err(why.to_string())),
            };
        }
        for line in lines {
            self.say(line);
        }
        if let Some(status) = status {
            if let Some(run) = self.run.take() {
                self.last_run = Some(run_notice(&run, status));
                self.ended = Some(run.child);
                self.read_now = true;
                self.report_now = true;
                self.which = Which::Newest;
            }
        }
    }

    fn take_record(&mut self) {
        let Some(answer) = take(&mut self.recording) else {
            return;
        };
        self.read_now = true;
        self.recorded = Some(match answer {
            Ok(answer) if answer.succeeded() => {
                self.form.note.clear();
                self.form.override_reason.clear();
                Notice::good(answer.stdout.trim())
            }
            // The tool's own refusal, word for word.
            Ok(answer) => Notice::bad(first_words(&answer.stderr, &answer.stdout)),
            Err(why) => Notice::bad(why),
        });
    }

    fn take_jacks(&mut self, now: Instant) {
        let Some((ask, pending)) = self.jacks_asking.take() else {
            return;
        };
        let mut pending = Some(pending);
        let Some(answer) = take(&mut pending) else {
            self.jacks_asking = pending.map(|p| (ask, p));
            return;
        };
        self.jacks_at = Some(now);
        let read = answer.and_then(|a| match jacks::Answer::parse(&a.stdout) {
            Ok(read) => Ok(read),
            Err(why) => Err(first_words(&a.stderr, &why)),
        });
        match read {
            Ok(read) => {
                self.jacks = Jacks::of(&read);
                if ask != Ask::Status {
                    self.jacks_said = Some(Notice {
                        good: read.ok,
                        text: read.message,
                    });
                }
            }
            Err(why) => {
                self.jacks = Jacks::NoSimulator(last_line(&why).to_owned());
                if ask != Ask::Status {
                    self.jacks_said = Some(Notice::bad(why));
                }
            }
        }
    }

    fn take_report(&mut self) {
        let Some((rung, pending)) = self.reporting.take() else {
            return;
        };
        let mut pending = Some(pending);
        let Some(answer) = take(&mut pending) else {
            self.reporting = pending.map(|p| (rung, p));
            return;
        };
        if self.selected.as_deref() != Some(rung.as_str()) {
            return; // another rung was chosen meanwhile: its own call is due
        }
        // A session that does not read leaves the last one shown; the ladder's own
        // problem line covers a tool that is broken.
        if let Some(report) = answer
            .ok()
            .filter(Answer::succeeded)
            .and_then(|a| Report::parse(&a.stdout).ok())
        {
            self.report = Some(report);
            self.keep_step();
        }
    }

    /// Keep the step chosen while the session has it; else open on the one to read first.
    fn keep_step(&mut self) {
        if self.step().is_none() {
            self.step = self
                .report()
                .and_then(Report::first_to_read)
                .map(|step| step.name.clone());
        }
    }

    fn take_ladder(&mut self) {
        let Some(answer) = take(&mut self.reading) else {
            return;
        };
        let read = answer.and_then(|answer| {
            if answer.succeeded() {
                Ladder::parse(&answer.stdout).map_err(|why| why.to_string())
            } else {
                Err(format!(
                    "{} failed: {}",
                    ToolCommand::table().line(),
                    last_line(&answer.stderr)
                ))
            }
        });
        match read {
            Ok(ladder) => {
                self.ladder = Some(ladder);
                self.problem = None;
                self.keep_selection();
            }
            Err(why) => self.problem = Some(why),
        }
    }

    /// Keep the row chosen while it exists; else open on the lowest rung not passed.
    fn keep_selection(&mut self) {
        let Some(ladder) = &self.ladder else {
            return;
        };
        if self.selected().is_none() {
            self.selected = ladder
                .first_not_passed()
                .or_else(|| ladder.top_first().next())
                .map(|rung| rung.id.clone());
            self.report_now = true;
        }
    }
}

/// The answer, if it has come; a sender that died counts as an answer too.
fn take(pending: &mut Option<Pending>) -> Option<Result<Answer, String>> {
    let answer = match pending.as_ref()?.try_recv() {
        Ok(answer) => answer,
        Err(TryRecvError::Empty) => return None,
        Err(TryRecvError::Disconnected) => Err("the call ended without an answer".into()),
    };
    *pending = None;
    Some(answer)
}

fn run_notice(run: &Run, status: Result<ExitStatus, String>) -> Notice {
    let rung = &run.rung;
    let what = match &run.test {
        Some(test) => format!("{rung}, {test} alone"),
        None => rung.clone(),
    };
    let kept = if run.research {
        "research, outside the capstone's scope: not on the ladder, nothing recorded"
    } else if run.test.is_some() {
        "a single test: not recorded on the ladder"
    } else {
        "the tool recorded it"
    };
    match status.map(|s| s.code()) {
        Ok(Some(0)) => Notice::good(format!("{what}: finished, and passed; {kept}")),
        Ok(Some(code)) => Notice::bad(format!(
            "{what}: finished, and did not pass (exit code {code}); {kept}; see the answer \
             and the console"
        )),
        Ok(None) if run.stop_asked => {
            Notice::bad(format!("{what}: stopped by you; nothing was recorded"))
        }
        Ok(None) => Notice::bad(format!("{what}: a signal ended the run")),
        Err(why) => Notice::bad(format!("{what}: the run was lost: {why}")),
    }
}

fn first_words(stderr: &str, stdout: &str) -> String {
    match (stderr.trim(), stdout.trim()) {
        ("", "") => "the tool refused, and said nothing".into(),
        ("", out) => out.into(),
        (err, _) => err.into(),
    }
}

fn last_line(text: &str) -> &str {
    text.lines()
        .rev()
        .map(str::trim)
        .find(|line| !line.is_empty())
        .unwrap_or("it said nothing")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{command::Outcome, ladder::Place};

    #[test]
    fn pretend_data_is_there_at_once_and_opens_on_the_work() {
        let session = Session::new(Source::Pretend);
        assert_eq!(session.ladder().map(|l| l.rungs.len()), Some(10));
        assert_eq!(session.selected().map(|r| r.id.as_str()), Some("stands"));
        assert!(session.problem().is_none());
    }

    #[test]
    fn pretend_data_shows_a_session_with_a_step_to_read() {
        let mut session = Session::new(Source::Pretend);
        session.select("stands");
        assert!(session.report().is_none(), "not read until the next tick");
        session.tick(Instant::now());
        let report = session.report().expect("the pretend stands session");
        assert_eq!(report.rung, "stands");
        let step = session.step().expect("a step is chosen");
        assert!(step.graph.is_some(), "it opens on a step with a graph");
        session.select_step("estop");
        assert_eq!(session.step().map(|s| s.name.as_str()), Some("estop"));
        // Another rung: its own session, or none; never the last rung's.
        session.select("speed");
        session.tick(Instant::now());
        assert!(session.report().is_none() && session.step().is_none());
    }

    #[test]
    fn pretend_data_runs_no_research_watch() {
        let mut session = Session::new(Source::Pretend);
        for watch in Watch::ALL {
            assert_eq!(
                session.watch_button(watch),
                WatchButton::Off(WhyNot::Pretend)
            );
            session.press_watch(watch);
        }
        assert!(session.running().is_none());
    }

    #[test]
    fn pretend_data_runs_no_single_test() {
        let mut session = Session::new(Source::Pretend);
        session.select("stands");
        assert_eq!(
            session.test_button("steer-rate"),
            Some(RunButton::Off(WhyNot::Pretend))
        );
        assert_eq!(session.test_button("no-such-test"), None);
        session.press_run_test("steer-rate");
        assert!(session.running().is_none());
        assert_eq!(session.runs_in(), Some(RunsIn::OwnNoneOpen));
    }

    #[test]
    fn pretend_data_runs_nothing_and_records_nothing() {
        let mut session = Session::new(Source::Pretend);
        assert_eq!(session.run_button(), Some(RunButton::Off(WhyNot::Pretend)));
        session.press_run();
        assert!(session.running().is_none());
        assert_eq!(session.console().len(), 0);

        session.form.outcome = Outcome::Pass;
        session.form.place = Place::Car;
        session.press_record();
        assert!(!session.is_recording());
        assert_eq!(
            session.recorded().map(|n| (n.good, n.text.as_str())),
            Some((false, "pretend data: nothing is recorded"))
        );
        session.tick(Instant::now());
        assert!(session.problem().is_none());
    }

    #[test]
    fn pretend_data_runs_no_check_and_opens_no_page() {
        let mut session = Session::new(Source::Pretend);
        assert_eq!(
            session.check_button("code"),
            Some(RunButton::Off(WhyNot::Pretend))
        );
        assert_eq!(session.check_button("no-such-check"), None);
        session.press_run_check("code");
        assert!(session.running().is_none());
        session.press_open_report();
        assert!(session.opened().is_some_and(|n| !n.good));
    }

    #[test]
    fn pretend_data_never_asks_for_the_jacks() {
        let mut session = Session::new(Source::Pretend);
        assert_eq!(session.jacks(), &Jacks::Pretend);
        assert_eq!(session.jacks_button(), None);
        session.press_jacks();
        session.tick(Instant::now());
        assert!(session.jacks_asking.is_none() && session.jacks_said().is_none());
    }

    #[test]
    fn the_jacks_button_is_off_until_a_simulator_answers_and_while_a_request_is_out() {
        let mut session = Session::new(Source::Tool {
            root: PathBuf::from("/nowhere"),
        });
        assert_eq!(session.jacks(), &Jacks::Unknown);
        assert_eq!(session.jacks_button(), None);
        session.press_jacks();
        assert!(
            session.jacks_asking.is_none(),
            "nothing is asked while it is off"
        );

        session.jacks = Jacks::OnStands;
        assert_eq!(session.jacks_button(), Some(Ask::Down));
        let (_tx, rx) = std::sync::mpsc::channel();
        session.jacks_asking = Some((Ask::Down, rx));
        assert_eq!(session.jacks_button(), None, "one request at a time");
        // A status poll on its way does not grey the button.
        let (_tx, rx) = std::sync::mpsc::channel();
        session.jacks_asking = Some((Ask::Status, rx));
        assert_eq!(session.jacks_button(), Some(Ask::Down));
    }

    #[test]
    fn the_simulators_refusal_is_shown_word_for_word_and_the_car_stays_up() {
        let mut session = Session::new(Source::Tool {
            root: PathBuf::from("/nowhere"),
        });
        let (tx, rx) = std::sync::mpsc::channel();
        session.jacks_asking = Some((Ask::Down, rx));
        let stdout = r#"{"verb": "down", "simulator": true, "on_stands": true, "ok": false,
            "message": "refused, the car stays on stands: the wheels still spin at +0.31 m/s"}"#
            .replace('\n', " ");
        tx.send(Ok(Answer {
            code: Some(3),
            stdout,
            stderr: String::new(),
        }))
        .expect("the session listens");
        session.take_jacks(Instant::now());
        assert_eq!(session.jacks(), &Jacks::OnStands);
        let said = session.jacks_said().expect("its words");
        assert!(!said.good && said.text.contains("the wheels still spin"));
    }

    #[test]
    fn a_tool_that_cannot_answer_turns_the_switch_off_with_its_last_line() {
        let mut session = Session::new(Source::Tool {
            root: PathBuf::from("/nowhere"),
        });
        let (tx, rx) = std::sync::mpsc::channel();
        session.jacks_asking = Some((Ask::Status, rx));
        tx.send(Ok(Answer {
            code: Some(1),
            stdout: String::new(),
            stderr: "Not built yet: run 'pixi run build' first.\n".into(),
        }))
        .expect("the session listens");
        session.take_jacks(Instant::now());
        assert_eq!(
            session.jacks(),
            &Jacks::NoSimulator("Not built yet: run 'pixi run build' first.".into())
        );
        assert_eq!(session.jacks_button(), None);
    }

    #[test]
    fn choosing_another_rung_clears_the_last_records_words() {
        let mut session = Session::new(Source::Pretend);
        session.press_record();
        session.select("stands");
        assert!(session.recorded().is_some(), "the same rung: kept");
        session.select("speed");
        assert!(session.recorded().is_none());
        assert_eq!(session.selected().map(|r| r.id.as_str()), Some("speed"));
    }

    #[test]
    fn the_refusal_is_the_tools_stderr_then_its_stdout() {
        assert_eq!(first_words("Shut: x.\n", "ignored"), "Shut: x.");
        assert_eq!(first_words("", "said here\n"), "said here");
        assert_eq!(first_words(" ", ""), "the tool refused, and said nothing");
        assert_eq!(
            last_line("Traceback\n  File\nModuleNotFoundError: No module named 'yaml'\n\n"),
            "ModuleNotFoundError: No module named 'yaml'"
        );
    }
}
