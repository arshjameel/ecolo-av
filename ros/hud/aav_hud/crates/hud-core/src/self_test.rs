//! The boot self-test's answer (`/safety/self_test/status`, aav_self_test_rs), turned into
//! what the HUD shows: a small green READY, or a stop banner with why not.
//!
//! | Heard | Shown |
//! |---|---|
//! | never (Ravensim, the bench, the turtle run none) | nothing |
//! | level OK | `READY`, small and green |
//! | any other level | `NOT READY: <why>`, a stop banner |
//! | heard, then quiet for [`QUIET`] | `SELF-TEST SILENT`, a stop banner (spine_bridge won't arm) |

use std::time::Duration;

use crate::health::Health;

/// The self-test speaks twice a second; spine_bridge counts a READY older than this as
/// silence, and so does the screen.
pub const QUIET: Duration = Duration::from_secs(2);

/// diagnostic_msgs/DiagnosticStatus's OK level: the self-test's READY.
const LEVEL_OK: u8 = 0;

/// The self-test's last message, as the screen needs it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report {
    /// Level OK: every check passed for its settling time.
    pub ready: bool,
    /// The line for people: `READY`, or `NOT READY - ` and each failing check and why.
    pub message: String,
}

impl Report {
    /// From the message's level and text.
    #[must_use]
    pub fn from_status(level: u8, message: &str) -> Self {
        Self {
            ready: level == LEVEL_OK,
            message: message.into(),
        }
    }
}

/// What the screen shows for the self-test.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Readiness {
    /// Every check passed: a small green READY.
    Ready,
    /// A stop banner: `NOT READY: <why>`, or `SELF-TEST SILENT`.
    NotReady(String),
}

/// What to show, from the topic's health and the last report; None when never heard.
#[must_use]
pub fn readiness(health: Health, last: Option<&Report>) -> Option<Readiness> {
    match (health, last) {
        (Health::Never, _) | (Health::Ok(_), None) => None,
        (Health::Stale(_), _) => Some(Readiness::NotReady("SELF-TEST SILENT".into())),
        (Health::Ok(_), Some(r)) if r.ready => Some(Readiness::Ready),
        (Health::Ok(_), Some(r)) => {
            let why = r
                .message
                .strip_prefix("NOT READY")
                .unwrap_or(&r.message)
                .trim_start_matches([' ', '-', ':'])
                .trim();
            Some(Readiness::NotReady(if why.is_empty() {
                "NOT READY".into()
            } else {
                format!("NOT READY: {why}")
            }))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OK: Health = Health::Ok(Duration::ZERO);

    #[test]
    fn never_heard_shows_nothing() {
        assert_eq!(readiness(Health::Never, None), None);
        assert_eq!(readiness(OK, None), None);
    }

    #[test]
    fn ready_is_small_and_green() {
        let r = Report::from_status(0, "READY");
        assert_eq!(readiness(OK, Some(&r)), Some(Readiness::Ready));
    }

    #[test]
    fn not_ready_says_why() {
        let r = Report::from_status(
            2,
            "NOT READY - sensors (STOP: gps STALE (nothing for 2.3 s))",
        );
        assert_eq!(
            readiness(OK, Some(&r)),
            Some(Readiness::NotReady(
                "NOT READY: sensors (STOP: gps STALE (nothing for 2.3 s))".into()
            ))
        );
        let bare = Report::from_status(2, "");
        assert_eq!(
            readiness(OK, Some(&bare)),
            Some(Readiness::NotReady("NOT READY".into()))
        );
    }

    #[test]
    fn the_level_decides_not_the_words() {
        // A WARN level with READY's words is still not READY.
        let r = Report::from_status(1, "READY");
        assert_eq!(
            readiness(OK, Some(&r)),
            Some(Readiness::NotReady("NOT READY: READY".into()))
        );
    }

    #[test]
    fn quiet_after_speaking_is_silent() {
        let r = Report::from_status(0, "READY");
        assert_eq!(
            readiness(Health::Stale(Duration::from_secs(3)), Some(&r)),
            Some(Readiness::NotReady("SELF-TEST SILENT".into()))
        );
    }
}
