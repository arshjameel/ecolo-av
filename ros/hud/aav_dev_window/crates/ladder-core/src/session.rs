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
    command::{RecordForm, ToolCommand},
    ladder::{Ladder, Rung},
    pretend,
    process::{self, Answer, Running, Signal},
    show::{self, RunButton, WhyNot},
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
    /// Stop was pressed: the next press does not ask.
    pub stop_asked: bool,
}

#[derive(Debug)]
struct Run {
    rung: String,
    child: Running,
    stop_asked: bool,
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
        };
        session.keep_selection();
        session
    }

    /// Once a frame: take what the children have said, and read the ladder when it is due.
    pub fn tick(&mut self, now: Instant) {
        self.follow_run();
        self.take_record();
        self.take_ladder();
        let Source::Tool { root } = &self.source else {
            return;
        };
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
        }
    }

    #[must_use]
    pub fn running(&self) -> Option<RunningNow<'_>> {
        self.run.as_ref().map(|run| RunningNow {
            rung: &run.rung,
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
        let (Some(RunButton::Ready(command)), Source::Tool { root }, Some(rung)) =
            (self.run_button(), &self.source, self.selected())
        else {
            return;
        };
        let rung = rung.id.clone();
        let process = command.process(root);
        self.console.clear();
        self.say(format!("$ {}", command.line()));
        self.ended = None;
        match Running::start(process) {
            Ok(child) => {
                self.last_run = None;
                self.run = Some(Run {
                    rung,
                    child,
                    stop_asked: false,
                });
            }
            Err(why) => {
                self.last_run = Some(Notice::bad(format!("{rung} did not start: {why}")));
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
    match status.map(|s| s.code()) {
        Ok(Some(0)) => Notice::good(format!(
            "{rung}: the run ended with 0; the tool recorded it"
        )),
        Ok(Some(code)) => Notice::bad(format!(
            "{rung}: the run ended with {code}; see the console and the ladder"
        )),
        Ok(None) if run.stop_asked => {
            Notice::bad(format!("{rung}: stopped by you; nothing was recorded"))
        }
        Ok(None) => Notice::bad(format!("{rung}: a signal ended the run")),
        Err(why) => Notice::bad(format!("{rung}: the run was lost: {why}")),
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
        assert_eq!(session.ladder().map(|l| l.rungs.len()), Some(7));
        assert_eq!(session.selected().map(|r| r.id.as_str()), Some("stack"));
        assert!(session.problem().is_none());
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
    fn choosing_another_rung_clears_the_last_records_words() {
        let mut session = Session::new(Source::Pretend);
        session.press_record();
        session.select("stack");
        assert!(session.recorded().is_some(), "the same rung: kept");
        session.select("code");
        assert!(session.recorded().is_none());
        assert_eq!(session.selected().map(|r| r.id.as_str()), Some("code"));
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
