//! "The simulated car: on stands / on the ground": the window's switch for Ravensim's jacks.
//!
//! The window has no ROS: it runs the same tool a person does, `pixi run jacks`
//! (`tools/jacks.py`, through `tools/ros.sh --local`, so only this computer is ever asked),
//! and shows its answer. The tool refuses anything but a simulator, and the simulator
//! refuses to lower a car whose wheels spin: neither rule is repeated here. Every command
//! this module builds carries `--sim`; there is none for a real car.

use std::{fmt, path::Path, process::Command, time::Duration};

use serde::Deserialize;

/// The wrapper that keeps ROS on this computer, from the repository's root.
pub const PROGRAM: &str = "tools/ros.sh";
/// The tool, from the repository's root.
pub const TOOL: &str = "tools/jacks.py";
/// How often the state is asked again (after the last answer came).
pub const ASK_EVERY: Duration = Duration::from_secs(5);

/// What to ask the tool.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ask {
    /// Is there a simulator, and where is its car?
    Status,
    /// Put the car on stands.
    Up,
    /// Put it back on the ground.
    Down,
}

impl Ask {
    fn verb(self) -> &'static str {
        match self {
            Self::Status => "status",
            Self::Up => "up",
            Self::Down => "down",
        }
    }

    /// The tool's arguments, after [`PROGRAM`]. Always `--local`, always `--sim`.
    #[must_use]
    pub fn args(self) -> Vec<&'static str> {
        let mut args = vec!["--local", "python3", TOOL, self.verb(), "--sim", "--json"];
        if self == Self::Status {
            // Nobody waits on a poll: a missing simulator is known in 2 s.
            args.extend(["--wait", "2"]);
        }
        args
    }

    /// As a person would type it.
    #[must_use]
    pub fn line(self) -> String {
        format!("pixi run jacks {} --sim", self.verb())
    }

    /// The process to start, in the repository's root.
    #[must_use]
    pub fn process(self, root: &Path) -> Command {
        let mut process = Command::new(PROGRAM);
        process.args(self.args()).current_dir(root);
        process
    }
}

/// The tool's `--json` line.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Answer {
    /// A simulator, and only a simulator, was heard on this computer.
    pub simulator: bool,
    /// Where its car is; `None`: it has not said.
    pub on_stands: Option<bool>,
    /// The request was done (a status always is).
    pub ok: bool,
    /// The simulator's own words, or why the tool refused.
    pub message: String,
}

impl Answer {
    /// Read the tool's output: its last line that is JSON (ROS may print above it).
    ///
    /// # Errors
    ///
    /// The tool's own last words, when none of its lines is an answer.
    pub fn parse(stdout: &str) -> Result<Self, String> {
        stdout
            .lines()
            .rev()
            .find_map(|line| serde_json::from_str(line.trim()).ok())
            .ok_or_else(|| match stdout.trim().lines().last() {
                Some(last) => format!("the jacks tool said: {last}"),
                None => "the jacks tool said nothing".into(),
            })
    }
}

/// Where the simulated car is, as far as the window knows.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum Jacks {
    /// Not asked yet.
    #[default]
    Unknown,
    /// `--pretend`: nothing is asked.
    Pretend,
    /// No simulator on this computer, or something that is not one: the tool's words.
    NoSimulator(String),
    /// A simulator that has not said where its car is (one without jacks).
    Silent,
    OnGround,
    OnStands,
}

impl Jacks {
    #[must_use]
    pub fn of(answer: &Answer) -> Self {
        match (answer.simulator, answer.on_stands) {
            (false, _) => Self::NoSimulator(answer.message.clone()),
            (true, None) => Self::Silent,
            (true, Some(true)) => Self::OnStands,
            (true, Some(false)) => Self::OnGround,
        }
    }

    /// What the one button would ask, if it is on: only with a simulator running here,
    /// and only the other state.
    #[must_use]
    pub fn button(&self) -> Option<Ask> {
        match self {
            Self::OnGround => Some(Ask::Up),
            Self::OnStands => Some(Ask::Down),
            Self::Unknown | Self::Pretend | Self::NoSimulator(_) | Self::Silent => None,
        }
    }

    /// The button's words, whether it is on or not.
    #[must_use]
    pub fn button_words(&self) -> &'static str {
        match self {
            Self::OnStands => "Lower it to the ground",
            _ => "Put it on stands",
        }
    }
}

impl fmt::Display for Jacks {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unknown => f.write_str("asking..."),
            Self::Pretend => f.write_str("pretend data: nothing is asked"),
            Self::NoSimulator(why) => write!(f, "off: {why}"),
            Self::Silent => f.write_str("off: this simulator has no jacks"),
            Self::OnGround => f.write_str("on the ground"),
            Self::OnStands => f.write_str("ON STANDS: the wheels spin, the car stays put"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn answer(simulator: bool, on_stands: Option<bool>) -> Answer {
        Answer {
            simulator,
            on_stands,
            ok: true,
            message: "why".into(),
        }
    }

    #[test]
    fn every_command_is_local_and_for_the_simulator_only() {
        for ask in [Ask::Status, Ask::Up, Ask::Down] {
            let args = ask.args();
            assert_eq!(&args[..3], ["--local", "python3", "tools/jacks.py"]);
            assert!(
                args.contains(&"--sim") && args.contains(&"--json"),
                "{args:?}"
            );
            let process = ask.process(Path::new("/repo"));
            assert_eq!(process.get_program(), "tools/ros.sh");
            assert_eq!(process.get_current_dir(), Some(Path::new("/repo")));
        }
        assert_eq!(Ask::Up.args()[3], "up");
        assert_eq!(Ask::Down.line(), "pixi run jacks down --sim");
    }

    #[test]
    fn the_tools_json_is_read_under_whatever_ros_printed() {
        let out = "[INFO] something from ROS\n{\"verb\": \"down\", \"simulator\": true, \
                   \"on_stands\": true, \"ok\": false, \"message\": \"refused: the wheels \
                   still spin\"}\n";
        let read = Answer::parse(out).expect("an answer");
        assert!(read.simulator && !read.ok);
        assert_eq!(read.on_stands, Some(true));
        assert!(read.message.contains("still spin"));
        assert_eq!(
            Answer::parse("Not built yet: run 'pixi run build' first.\n"),
            Err("the jacks tool said: Not built yet: run 'pixi run build' first.".into())
        );
        assert_eq!(Answer::parse(""), Err("the jacks tool said nothing".into()));
    }

    #[test]
    fn the_button_is_on_only_with_a_simulator_and_asks_for_the_other_state() {
        assert_eq!(
            Jacks::of(&answer(true, Some(false))).button(),
            Some(Ask::Up)
        );
        assert_eq!(
            Jacks::of(&answer(true, Some(true))).button(),
            Some(Ask::Down)
        );
        // No simulator, a real car, the bench: the tool says why, and the button is off.
        let none = Jacks::of(&answer(false, None));
        assert_eq!(none, Jacks::NoSimulator("why".into()));
        assert_eq!(none.button(), None);
        assert_eq!(Jacks::of(&answer(false, Some(true))).button(), None);
        assert_eq!(Jacks::of(&answer(true, None)).button(), None);
        assert_eq!(Jacks::Unknown.button(), None);
        assert_eq!(Jacks::Pretend.button(), None);
    }

    #[test]
    fn the_state_is_said_in_words() {
        assert_eq!(Jacks::OnGround.to_string(), "on the ground");
        assert!(Jacks::OnStands.to_string().starts_with("ON STANDS"));
        assert_eq!(
            Jacks::NoSimulator("no simulator heard".into()).to_string(),
            "off: no simulator heard"
        );
        assert_eq!(Jacks::OnStands.button_words(), "Lower it to the ground");
        assert_eq!(Jacks::OnGround.button_words(), "Put it on stands");
    }
}
