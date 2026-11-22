//! The tool as a child process: a long run whose lines are read as they come and which is
//! stopped by its process group, and a short call whose whole answer comes back at once.
//! Neither blocks the window: threads read, the window polls.

use std::{
    io::{self, BufRead, BufReader, Read},
    process::{Child, Command, ExitStatus, Stdio},
    sync::mpsc::{self, Receiver, Sender},
    thread,
};

/// A short call's whole answer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Answer {
    /// The exit code; `None`: a signal ended it.
    pub code: Option<i32>,
    pub stdout: String,
    pub stderr: String,
}

impl Answer {
    #[must_use]
    pub fn succeeded(&self) -> bool {
        self.code == Some(0)
    }
}

/// Start a short call on its own thread. The answer arrives once; `Err`: it did not start.
#[must_use]
pub fn ask(mut process: Command) -> Receiver<Result<Answer, String>> {
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        let answer = process
            .stdin(Stdio::null())
            .output()
            .map(|out| Answer {
                code: out.status.code(),
                stdout: String::from_utf8_lossy(&out.stdout).into_owned(),
                stderr: String::from_utf8_lossy(&out.stderr).into_owned(),
            })
            .map_err(|why| format!("{:?} did not start: {why}", process.get_program()));
        // The window may have closed: then nobody is waiting.
        let _ = tx.send(answer);
    });
    rx
}

/// How hard to stop a run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Signal {
    /// Ask: the child and everything it started may tidy up.
    Terminate,
    /// Do not ask.
    Kill,
}

/// A run in progress: the child, in a process group of its own.
#[derive(Debug)]
pub struct Running {
    child: Child,
    lines: Receiver<String>,
}

impl Running {
    /// Start it, its output piped here, as the leader of a new process group, so a stop
    /// reaches everything it starts (the tool's own child, a simulator) and nothing else.
    ///
    /// # Errors
    /// When the program cannot be started.
    pub fn start(mut process: Command) -> io::Result<Self> {
        process
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(unix)]
        std::os::unix::process::CommandExt::process_group(&mut process, 0);
        let mut child = process.spawn()?;
        let (tx, lines) = mpsc::channel();
        if let Some(out) = child.stdout.take() {
            read_lines(out, tx.clone());
        }
        if let Some(err) = child.stderr.take() {
            read_lines(err, tx);
        }
        Ok(Self { child, lines })
    }

    /// The lines printed since the last call, both streams.
    pub fn new_lines(&mut self) -> impl Iterator<Item = String> + '_ {
        self.lines.try_iter()
    }

    /// Has it ended? `None`: still running.
    ///
    /// # Errors
    /// When the system cannot say.
    pub fn ended(&mut self) -> io::Result<Option<ExitStatus>> {
        self.child.try_wait()
    }

    /// Signal the whole process group.
    ///
    /// # Errors
    /// When the signal could not be sent (the group has already gone, mostly).
    pub fn stop(&mut self, signal: Signal) -> io::Result<()> {
        #[cfg(unix)]
        {
            let name = match signal {
                Signal::Terminate => "-TERM",
                Signal::Kill => "-KILL",
            };
            // The group's id is the child's: it was started as its leader.
            let group = format!("-{}", self.child.id());
            let sent = Command::new("kill")
                .args([name, "--", &group])
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .status()?;
            if sent.success() {
                Ok(())
            } else {
                Err(io::Error::other(format!("kill {name} {group}: {sent}")))
            }
        }
        #[cfg(not(unix))]
        {
            let _ = signal;
            self.child.kill()
        }
    }
}

impl Drop for Running {
    /// A window that closes takes its run with it: no simulator is left behind.
    fn drop(&mut self) {
        if matches!(self.child.try_wait(), Ok(None)) {
            let _ = self.stop(Signal::Terminate);
        }
    }
}

fn read_lines(stream: impl Read + Send + 'static, tx: Sender<String>) {
    thread::spawn(move || {
        let lines = BufReader::new(stream).split(b'\n');
        for line in lines.map_while(Result::ok) {
            let text = String::from_utf8_lossy(&line).trim_end().to_owned();
            if tx.send(text).is_err() {
                break;
            }
        }
    });
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    fn sh(script: &str) -> Command {
        let mut command = Command::new("sh");
        command.args(["-c", script]);
        command
    }

    /// Poll until `done` says so, or five seconds.
    fn wait_for(mut done: impl FnMut() -> bool) -> bool {
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            if done() {
                return true;
            }
            thread::sleep(Duration::from_millis(20));
        }
        false
    }

    fn alive(pid: &str) -> bool {
        Command::new("kill")
            .args(["-0", pid])
            .stderr(Stdio::null())
            .status()
            .is_ok_and(|s| s.success())
    }

    #[test]
    fn a_short_call_gives_its_code_and_both_streams() {
        let answer = ask(sh("echo out; echo why >&2; exit 2"))
            .recv_timeout(Duration::from_secs(5))
            .expect("an answer")
            .expect("sh starts");
        assert_eq!(answer.code, Some(2));
        assert!(!answer.succeeded());
        assert_eq!(answer.stdout, "out\n");
        assert_eq!(answer.stderr, "why\n");
    }

    #[test]
    fn a_program_that_is_not_there_is_an_error_in_words() {
        let answer = ask(Command::new("/no/such/program"))
            .recv_timeout(Duration::from_secs(5))
            .expect("an answer");
        assert!(answer.is_err_and(|why| why.contains("did not start")));
        assert!(Running::start(Command::new("/no/such/program")).is_err());
    }

    #[test]
    fn a_runs_lines_come_as_it_prints_them_and_it_ends() {
        let mut run = Running::start(sh("echo one; echo two >&2")).expect("sh starts");
        let mut lines = Vec::new();
        assert!(wait_for(|| {
            lines.extend(run.new_lines());
            lines.len() == 2
        }));
        lines.sort();
        assert_eq!(lines, ["one", "two"]);
        let mut status = None;
        assert!(wait_for(|| {
            status = run.ended().ok().flatten();
            status.is_some()
        }));
        assert!(status.is_some_and(|s| s.success()));
    }

    #[test]
    fn a_stop_ends_the_child_and_what_it_started() {
        // The shell starts a grandchild, says its pid, and waits on it.
        let mut run = Running::start(sh("sleep 60 & echo $!; wait")).expect("sh starts");
        let mut grandchild = None;
        assert!(wait_for(|| {
            grandchild = grandchild.take().or_else(|| run.new_lines().next());
            grandchild.is_some()
        }));
        let grandchild = grandchild.expect("the pid was printed");
        assert!(alive(&grandchild));

        run.stop(Signal::Terminate).expect("the group is there");
        let mut status = None;
        assert!(wait_for(|| {
            status = run.ended().ok().flatten();
            status.is_some()
        }));
        assert!(
            status.is_some_and(|s| s.code().is_none()),
            "a signal ended it"
        );
        assert!(wait_for(|| !alive(&grandchild)), "the grandchild went too");
    }

    #[test]
    fn dropping_a_run_stops_it() {
        let mut run = Running::start(sh("sleep 60 & echo $!; wait")).expect("sh starts");
        let mut grandchild = None;
        assert!(wait_for(|| {
            grandchild = grandchild.take().or_else(|| run.new_lines().next());
            grandchild.is_some()
        }));
        let grandchild = grandchild.expect("the pid was printed");
        drop(run);
        assert!(wait_for(|| !alive(&grandchild)));
    }
}
