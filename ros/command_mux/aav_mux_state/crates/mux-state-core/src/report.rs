//! The answer in words: one line for people, and key-value rows for screens.
//!
//! The node sends both in one `diagnostic_msgs/DiagnosticStatus` on
//! `/control/command_mux/state`: `message` is [`Choice::summary`], `values` are
//! [`Choice::values`]. The rows, each always there:
//!
//! | Key | Value |
//! |---|---|
//! | `driver` | its name (`keyboard`, `nav`...), or `none` |
//! | `priority` | its priority, or `0` |
//! | `topic` | its topic, or empty |
//! | `age_s` | seconds since its last command, 2 decimals, or empty |
//! | `locks` | the locks on, highest first, `name priority` and ` silent` if silence locked it, joined by `, `; or `none` |
//! | `lock_priority` | the highest of them, or `0` |
//! | `held` | fresh inputs a lock holds, joined by `, `; or `none` |

use crate::mux::{Choice, LockOn, Why};

/// How the line should look.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// Someone drives, or nobody wants to.
    Ok,
    /// Nobody drives, though someone is sending: a lock holds them.
    Warn,
}

impl Level {
    /// `diagnostic_msgs/DiagnosticStatus`'s byte: OK 0, WARN 1.
    #[must_use]
    pub fn byte(self) -> u8 {
        match self {
            Self::Ok => 0,
            Self::Warn => 1,
        }
    }
}

fn lock_words(l: &LockOn) -> String {
    match l.why {
        Why::Set => format!("{} {}", l.name, l.priority),
        Why::Silent => format!("{} {} silent", l.name, l.priority),
    }
}

fn list(items: impl Iterator<Item = String>) -> String {
    let v: Vec<String> = items.collect();
    if v.is_empty() {
        "none".into()
    } else {
        v.join(", ")
    }
}

impl Choice {
    /// OK, or WARN while a lock holds someone and nobody drives.
    #[must_use]
    pub fn level(&self) -> Level {
        if self.driver.is_none() && !self.held.is_empty() {
            Level::Warn
        } else {
            Level::Ok
        }
    }

    /// One line: `keyboard (90)`, `keyboard (90); held: nav by sensors 30 silent`,
    /// `nobody; held: nav by sensors 30 silent`, `nobody sending`.
    #[must_use]
    pub fn summary(&self) -> String {
        let who = match &self.driver {
            Some(d) => format!("{} ({})", d.name, d.priority),
            None if self.held.is_empty() => "nobody sending".into(),
            None => "nobody".into(),
        };
        if self.held.is_empty() {
            return who;
        }
        let by = self.locks.first().map(lock_words).unwrap_or_default();
        format!("{who}; held: {} by {by}", self.held.join(", "))
    }

    /// The rows (the table above), in that order.
    #[must_use]
    pub fn values(&self) -> Vec<(&'static str, String)> {
        let d = self.driver.as_ref();
        vec![
            (
                "driver",
                d.map_or_else(|| "none".into(), |d| d.name.clone()),
            ),
            ("priority", d.map_or(0, |d| d.priority).to_string()),
            ("topic", d.map(|d| d.topic.clone()).unwrap_or_default()),
            (
                "age_s",
                d.and_then(|d| d.age)
                    .map(|a| format!("{a:.2}"))
                    .unwrap_or_default(),
            ),
            ("locks", list(self.locks.iter().map(lock_words))),
            ("lock_priority", self.lock_priority.to_string()),
            ("held", list(self.held.iter().cloned())),
        ]
    }

    /// What, if it changes, is news worth sending at once: the driver, the locks, the
    /// held. Not the age, which changes every tick.
    #[must_use]
    pub fn key(&self) -> String {
        format!(
            "{}|{}|{}",
            self.driver.as_ref().map_or("", |d| d.name.as_str()),
            list(self.locks.iter().map(lock_words)),
            self.held.join(",")
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mux::Driver;

    fn keyboard() -> Driver {
        Driver {
            name: "keyboard".into(),
            topic: "/drivers/keyboard/cmd_vel".into(),
            priority: 90,
            age: Some(0.123),
        }
    }

    fn sensors() -> LockOn {
        LockOn {
            name: "sensors".into(),
            priority: 30,
            why: Why::Silent,
        }
    }

    #[test]
    fn someone_driving() {
        let c = Choice {
            driver: Some(keyboard()),
            locks: vec![],
            lock_priority: 0,
            held: vec![],
        };
        assert_eq!(c.summary(), "keyboard (90)");
        assert_eq!(c.level(), Level::Ok);
        let v = c.values();
        let keys: Vec<&str> = v.iter().map(|(k, _)| *k).collect();
        assert_eq!(
            keys,
            [
                "driver",
                "priority",
                "topic",
                "age_s",
                "locks",
                "lock_priority",
                "held"
            ]
        );
        assert_eq!(v[0].1, "keyboard");
        assert_eq!(v[3].1, "0.12");
        assert_eq!(v[4].1, "none");
        assert_eq!(v[6].1, "none");
    }

    #[test]
    fn nobody_and_held() {
        let c = Choice {
            driver: None,
            locks: vec![sensors()],
            lock_priority: 30,
            held: vec!["nav".into(), "ai".into()],
        };
        assert_eq!(c.summary(), "nobody; held: nav, ai by sensors 30 silent");
        assert_eq!(c.level(), Level::Warn);
        assert_eq!(c.level().byte(), 1);
        let v = c.values();
        assert_eq!(v[0].1, "none");
        assert_eq!(v[1].1, "0");
        assert_eq!(v[3].1, "");
        assert_eq!(v[4].1, "sensors 30 silent");
        assert_eq!(v[6].1, "nav, ai");
        assert_eq!(c.key(), "|sensors 30 silent|nav,ai");
    }

    #[test]
    fn nobody_sending() {
        let c = Choice {
            driver: None,
            locks: vec![],
            lock_priority: 0,
            held: vec![],
        };
        assert_eq!(c.summary(), "nobody sending");
        assert_eq!(c.level(), Level::Ok);
    }

    #[test]
    fn a_driver_with_a_held_one_below() {
        let c = Choice {
            driver: Some(keyboard()),
            locks: vec![sensors()],
            lock_priority: 30,
            held: vec!["nav".into()],
        };
        assert_eq!(c.summary(), "keyboard (90); held: nav by sensors 30 silent");
        assert_eq!(c.level(), Level::Ok);
    }
}
