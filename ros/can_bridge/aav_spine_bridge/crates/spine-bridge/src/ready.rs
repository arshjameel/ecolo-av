//! The boot self-test, as the bridge hears it (TODO 255): `/safety/self_test/status`
//! (aav_self_test_rs). With `require_ready` (the car), the bridge refuses to arm until it
//! says READY (level OK) within `ready_timeout`. Silence refuses too: a dead self-test
//! can't vouch for anything. It never disarms and never touches the e-stop.

use std::fmt;
use std::time::{Duration, Instant};

/// Every refusal starts with this (`output_gate.py`'s `NOT_READY`, the same words).
pub const NOT_READY: &str = "the boot self-test says NOT READY";

/// Why the self-test doesn't let the bridge arm.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NotReady {
    /// Nothing heard from it yet.
    NotHeard,
    /// Nothing heard for this long.
    Quiet(Duration),
    /// It said NOT READY, with this line.
    Says(String),
}

impl fmt::Display for NotReady {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{NOT_READY}: ")?;
        match self {
            NotReady::NotHeard => f.write_str(
                "nothing heard from it yet (/safety/self_test/status); arm once it says READY",
            ),
            NotReady::Quiet(age) => write!(
                f,
                "it has been quiet for {:.1} s; arm once it says READY",
                age.as_secs_f64()
            ),
            NotReady::Says(line) => write!(f, "{line}"),
        }
    }
}

impl std::error::Error for NotReady {}

/// The self-test's last word, and when it came.
#[derive(Debug, Clone)]
pub struct ReadyGate {
    timeout: Duration,
    last: Option<(bool, String, Instant)>,
}

impl ReadyGate {
    /// A gate not heard from yet. `timeout`: an older word counts as silence (the
    /// self-test speaks twice a second).
    #[must_use]
    pub fn new(timeout: Duration) -> Self {
        Self {
            timeout,
            last: None,
        }
    }

    /// One status from /safety/self_test/status, heard at `at`: `ready` if its level is
    /// OK, and its message.
    pub fn heard(&mut self, ready: bool, message: &str, at: Instant) {
        self.last = Some((ready, message.into(), at));
    }

    /// Ok if the self-test said READY within the timeout.
    ///
    /// # Errors
    ///
    /// [`NotReady`]: why not.
    pub fn check(&self, now: Instant) -> Result<(), NotReady> {
        let Some((ready, line, at)) = &self.last else {
            return Err(NotReady::NotHeard);
        };
        let age = now.saturating_duration_since(*at);
        if age > self.timeout {
            return Err(NotReady::Quiet(age));
        }
        if *ready {
            Ok(())
        } else {
            Err(NotReady::Says(line.clone()))
        }
    }
}
