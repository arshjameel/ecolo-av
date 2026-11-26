//! The research watch: the barrier comparison, started from the tests screen.
//!
//! Outside the capstone's scope: Ben's MASc research (`docs/barriers.md`). It is not a
//! rung, has no place in the ladder's JSON and never touches the saved progress: the
//! window starts `pixi run barrier-watch`'s program, which opens a simulator of its own
//! on a ROS domain of its own, on this computer only. There is no car side.

use std::{path::Path, process::Command};

/// The words over these entries on the screen.
pub const HEADING: &str = "Outside the capstone's scope: Ben's MASc research";
/// What a reader must know before pressing one.
pub const RULE: &str = "Simulator only. Not a rung of the ladder: nothing is recorded, and \
                        no progress changes. Each opens a Ravensim of its own and its \
                        navigation view.";

/// The wrapper that keeps ROS on this computer, from the repository's root.
const LOCAL: [&str; 2] = ["tools/ros.sh", "--local"];
/// The tool, as `pixi run barrier-watch` runs it.
const TOOL: [&str; 2] = ["python3", "tools/barrier_watch.py"];
/// The speed the comparison is driven at (m/s), and how long its window stays (s).
const SPEED: &str = "3";
const KEEP: &str = "30";

/// One drive past the ghost, by the follower's mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Watch {
    /// The follower alone.
    Plain,
    /// A hard distance limit.
    Step,
    /// The barrier.
    Barrier,
}

impl Watch {
    pub const ALL: [Self; 3] = [Self::Plain, Self::Step, Self::Barrier];

    /// The tool's word for the mode.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            Self::Plain => "plain",
            Self::Step => "step",
            Self::Barrier => "barrier",
        }
    }

    /// What the mode is, in a line.
    #[must_use]
    pub fn says(self) -> &'static str {
        match self {
            Self::Plain => "the follower alone: it drives into the keep-out circle",
            Self::Step => "a hard distance limit: the rule with gamma 1",
            Self::Barrier => "the barrier: it slows early and goes round",
        }
    }

    /// The run's name in the console and the notices.
    #[must_use]
    pub fn name(self) -> String {
        format!("research: barrier-watch {}", self.word())
    }

    /// The command that runs it.
    #[must_use]
    pub fn command(self) -> WatchCommand {
        WatchCommand {
            args: LOCAL[1..]
                .iter()
                .chain(&TOOL)
                .copied()
                .chain([self.word(), "--speed", SPEED, "--keep", KEEP])
                .map(String::from)
                .collect(),
        }
    }
}

/// One call of the research tool: `tools/ros.sh` and its arguments.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WatchCommand {
    args: Vec<String>,
}

impl WatchCommand {
    /// As a person would read it in a terminal.
    #[must_use]
    pub fn line(&self) -> String {
        format!("{} {}", LOCAL[0], self.args.join(" "))
    }

    /// The process to start, in the repository's root.
    #[must_use]
    pub fn process(&self, root: &Path) -> Command {
        let mut process = Command::new(LOCAL[0]);
        process
            .args(&self.args)
            .current_dir(root)
            .env("PYTHONUNBUFFERED", "1");
        process
    }
}

/// A research entry's Run button.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WatchButton {
    /// On: pressing it starts this command.
    Ready(WatchCommand),
    /// Off, and why.
    Off(crate::show::WhyNot),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_watch_stays_on_this_computer_and_names_its_mode() {
        for watch in Watch::ALL {
            let line = watch.command().line();
            assert!(
                line.starts_with("tools/ros.sh --local python3 tools/barrier_watch.py "),
                "{line}"
            );
            assert!(
                line.contains(&format!(" {} --speed 3", watch.word())),
                "{line}"
            );
            assert!(watch.name().starts_with("research: "));
        }
    }

    #[test]
    fn no_watch_goes_through_the_ladders_tool() {
        for watch in Watch::ALL {
            let line = watch.command().line();
            assert!(
                !line.contains("ladder.py") && !line.contains("record"),
                "{line}"
            );
            assert!(!line.contains("--rviz"), "{line}");
        }
        assert!(HEADING.contains("Outside the capstone's scope"));
        assert!(RULE.contains("Simulator only"));
    }
}
