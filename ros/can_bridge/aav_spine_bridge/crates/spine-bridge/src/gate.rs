//! The safety gate, as the bridge hears it: the output mux's one-line state
//! (/safety/output_mux/state, aav_io_mux). The bridge refuses to arm unless the gate is
//! OPEN for its own target, and was heard recently (Ben, 24 Oct).
//!
//! Its times are a steady clock's ([`Instant`]), never the node's: in the simulator the node's
//! clock is simulated, and a paused simulation must not keep an old OPEN fresh. The output
//! mux keeps its own time on a steady clock too.

use std::{
    fmt,
    str::FromStr,
    time::{Duration, Instant},
};

/// Every refusal starts with this: `arm_when_open` (aav_spine) retries only these.
pub const GATE_SHUT: &str = "the safety gate is shut";

/// Who plays the car: the output mux's `target:=` this bridge reads commands from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Target {
    /// The real car (`source:=real`): /control/car/actuator_cmd.
    Car,
    /// The bench (`source:=bench`): /control/bench/actuator_cmd.
    Bench,
    /// Gazebo's spine profile (`source:=sim`): /control/sim/actuator_cmd.
    Sim,
}

impl Target {
    /// The output mux's name for it, as its state line says it.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Target::Car => "car",
            Target::Bench => "bench",
            Target::Sim => "sim",
        }
    }
}

impl fmt::Display for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A target this bridge can't play.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownTarget(pub String);

impl fmt::Display for UnknownTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "target {}: use car, bench or sim", self.0)
    }
}

impl std::error::Error for UnknownTarget {}

impl FromStr for Target {
    type Err = UnknownTarget;

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        match name {
            "car" => Ok(Target::Car),
            "bench" => Ok(Target::Bench),
            "sim" => Ok(Target::Sim),
            other => Err(UnknownTarget(other.into())),
        }
    }
}

/// Why the gate counts as shut. Each says what to wait for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GateShut {
    /// Nothing heard from the output mux yet.
    NotHeard,
    /// The output mux's last word is older than the timeout: it may have stopped.
    Quiet(Duration),
    /// The output mux says WAIT or STOP: its line, with the reason.
    Shut(String),
    /// An output mux for another target says OPEN: not this bridge's gate.
    OtherTarget {
        /// The mux's line.
        line: String,
        /// The target it is open for.
        target: Target,
    },
}

impl fmt::Display for GateShut {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{GATE_SHUT}: ")?;
        match self {
            GateShut::NotHeard => f.write_str(
                "no word from the output mux yet (/safety/output_mux/state); \
                 arm once it says OPEN",
            ),
            GateShut::Quiet(age) => write!(
                f,
                "the output mux has been quiet for {:.1} s; arm once it says OPEN",
                age.as_secs_f64()
            ),
            GateShut::Shut(line) => write!(f, "{line}; arm once it says OPEN"),
            GateShut::OtherTarget { line, target } => write!(
                f,
                "the output mux is for another target ({line}); this bridge needs target {target}"
            ),
        }
    }
}

impl std::error::Error for GateShut {}

/// The output mux's last state line, and when it came.
#[derive(Debug, Clone)]
pub struct OutputGate {
    target: Target,
    timeout: Duration,
    last: Option<(String, Instant)>,
}

impl OutputGate {
    /// A gate not heard from yet. `timeout`: an older line counts as silence (the mux
    /// says its state twice a second).
    #[must_use]
    pub fn new(target: Target, timeout: Duration) -> Self {
        Self {
            target,
            timeout,
            last: None,
        }
    }

    /// One line from /safety/output_mux/state, heard at `at`.
    pub fn heard(&mut self, line: &str, at: Instant) {
        self.last = Some((line.into(), at));
    }

    /// Ok if the output mux said OPEN for this bridge's target within the timeout.
    ///
    /// # Errors
    ///
    /// [`GateShut`]: why not.
    pub fn check(&self, now: Instant) -> Result<(), GateShut> {
        let Some((line, at)) = &self.last else {
            return Err(GateShut::NotHeard);
        };
        let age = now.saturating_duration_since(*at);
        if age > self.timeout {
            return Err(GateShut::Quiet(age));
        }
        // "OPEN - car: every source REAL"
        let Some(rest) = line.strip_prefix("OPEN - ") else {
            return Err(GateShut::Shut(line.clone()));
        };
        match rest.split_once(':') {
            Some((target, _)) if target == self.target.name() => Ok(()),
            _ => Err(GateShut::OtherTarget {
                line: line.clone(),
                target: self.target,
            }),
        }
    }
}

/// `arm_when_open`: ask `/spine/arm` again? Only while the gate alone refused (a message
/// starting with [`GATE_SHUT`]), for `give_up` (`output_gate.py`'s `try_again`).
///
/// An e-stop's refusal is final: a person pressed it, and a launch must not keep arming
/// behind them.
#[must_use]
pub fn try_again(success: bool, message: &str, waited: Duration, give_up: Duration) -> bool {
    !success && message.starts_with(GATE_SHUT) && waited < give_up
}
