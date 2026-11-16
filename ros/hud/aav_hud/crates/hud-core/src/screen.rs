//! Which monitor a window goes on, for a computer with two screens (the car's dash: the
//! HUD on the small one, the map on the big one).
//!
//! A monitor is chosen by its **name**, as the windowing system calls it (on X11 the
//! output's name, such as `HDMI-1`, the same as `xrandr --listmonitors`), or by its
//! **number** in the list (`0` is the first). `hud --list-monitors` prints both.
//!
//! No guessing: a name must match exactly, and a missing monitor is an error that lists
//! what is there. A window on the wrong screen is worse than no window.

use std::{fmt, str::FromStr};

use thiserror::Error;

/// The monitor asked for, from `--monitor`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Monitor {
    /// Its place in the list, from 0.
    Number(usize),
    /// Its name, exactly as listed.
    Name(String),
}

/// One monitor, as the windowing system reports it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MonitorInfo {
    pub name: String,
    /// Its size in screen coordinates.
    pub width: i32,
    pub height: i32,
    /// Its top-left corner on the desktop.
    pub x: i32,
    pub y: i32,
}

#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum MonitorError {
    #[error("--monitor needs a name or a number, such as HDMI-1 or 0")]
    Empty,
    #[error("no monitor {wanted}: this computer has {}", seen_list(.seen))]
    NotFound { wanted: String, seen: Vec<String> },
    #[error("{count} monitors are called {name:?}: give its number instead ({})", seen_list(.seen))]
    Ambiguous {
        name: String,
        count: usize,
        seen: Vec<String>,
    },
}

/// `0 "HDMI-1", 1 "DSI-1"`, or `no monitor at all`.
fn seen_list(seen: &[String]) -> String {
    if seen.is_empty() {
        return "no monitor at all".to_owned();
    }
    seen.iter()
        .enumerate()
        .map(|(i, name)| format!("{i} {name:?}"))
        .collect::<Vec<_>>()
        .join(", ")
}

impl FromStr for Monitor {
    type Err = MonitorError;

    /// Digits are a number; anything else is a name (spaces at the ends dropped).
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim();
        if s.is_empty() {
            return Err(MonitorError::Empty);
        }
        if s.bytes().all(|b| b.is_ascii_digit()) {
            // Too many digits for a usize is still a number: just not one that exists.
            return Ok(Self::Number(s.parse().unwrap_or(usize::MAX)));
        }
        Ok(Self::Name(s.to_owned()))
    }
}

impl fmt::Display for Monitor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Number(n) => write!(f, "number {n}"),
            Self::Name(name) => write!(f, "{name:?}"),
        }
    }
}

impl Monitor {
    /// This monitor's place in `names`, the list the windowing system gave.
    pub fn find<S: AsRef<str>>(&self, names: &[S]) -> Result<usize, MonitorError> {
        let seen = || names.iter().map(|n| n.as_ref().to_owned()).collect();
        match self {
            Self::Number(n) if *n < names.len() => Ok(*n),
            Self::Number(_) => Err(MonitorError::NotFound {
                wanted: self.to_string(),
                seen: seen(),
            }),
            Self::Name(name) => {
                let mut hits = names.iter().enumerate().filter(|(_, n)| n.as_ref() == name);
                match (hits.next(), hits.count()) {
                    (Some((i, _)), 0) => Ok(i),
                    (Some(_), more) => Err(MonitorError::Ambiguous {
                        name: name.clone(),
                        count: more + 1,
                        seen: seen(),
                    }),
                    (None, _) => Err(MonitorError::NotFound {
                        wanted: self.to_string(),
                        seen: seen(),
                    }),
                }
            }
        }
    }
}

/// What `--list-monitors` prints: one line each, the number, the name, the size and where.
#[must_use]
pub fn listing(monitors: &[MonitorInfo]) -> String {
    if monitors.is_empty() {
        return "no monitor found (is DISPLAY set? is the screen on?)\n".to_owned();
    }
    let mut out = String::from("number  name                      size         at\n");
    for (i, m) in monitors.iter().enumerate() {
        out.push_str(&format!(
            "{i:<7} {:<25} {:<12} {},{}\n",
            format!("{:?}", m.name),
            format!("{}x{}", m.width, m.height),
            m.x,
            m.y
        ));
    }
    out.push_str("use either: --monitor HDMI-1 (the name, exactly) or --monitor 0 (the number)\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const DASH: [&str; 2] = ["HDMI-1", "HDMI-2"];

    #[test]
    fn digits_are_a_number_anything_else_a_name() {
        assert_eq!("0".parse(), Ok(Monitor::Number(0)));
        assert_eq!(" 1 ".parse(), Ok(Monitor::Number(1)));
        assert_eq!("HDMI-1".parse(), Ok(Monitor::Name("HDMI-1".into())));
        assert_eq!(
            "Built-in Retina Display".parse(),
            Ok(Monitor::Name("Built-in Retina Display".into()))
        );
        assert_eq!("".parse::<Monitor>(), Err(MonitorError::Empty));
        assert_eq!("  ".parse::<Monitor>(), Err(MonitorError::Empty));
        assert_eq!(
            "99999999999999999999999".parse(),
            Ok(Monitor::Number(usize::MAX))
        );
    }

    #[test]
    fn finds_by_name_exactly() {
        let m: Monitor = "HDMI-2".parse().expect("a name");
        assert_eq!(m.find(&DASH), Ok(1));
        // No guessing: case and near misses don't count.
        assert!("hdmi-2"
            .parse::<Monitor>()
            .expect("a name")
            .find(&DASH)
            .is_err());
        assert!("HDMI"
            .parse::<Monitor>()
            .expect("a name")
            .find(&DASH)
            .is_err());
    }

    #[test]
    fn finds_by_number() {
        assert_eq!(Monitor::Number(0).find(&DASH), Ok(0));
        assert_eq!(Monitor::Number(1).find(&DASH), Ok(1));
        assert!(Monitor::Number(2).find(&DASH).is_err());
        assert!(Monitor::Number(0).find::<&str>(&[]).is_err());
    }

    #[test]
    fn a_missing_monitor_says_what_is_there() {
        let err = Monitor::Name("DSI-1".into())
            .find(&DASH)
            .expect_err("not there");
        assert_eq!(
            err.to_string(),
            r#"no monitor "DSI-1": this computer has 0 "HDMI-1", 1 "HDMI-2""#
        );
        let err = Monitor::Number(3).find::<&str>(&[]).expect_err("none");
        assert_eq!(
            err.to_string(),
            "no monitor number 3: this computer has no monitor at all"
        );
    }

    #[test]
    fn two_monitors_with_one_name_need_a_number() {
        // Two of the same screen, on a system that names them by model.
        let names = ["Innoview", "Innoview"];
        let err = Monitor::Name("Innoview".into())
            .find(&names)
            .expect_err("ambiguous");
        assert!(matches!(err, MonitorError::Ambiguous { count: 2, .. }));
        assert!(err.to_string().contains("give its number"));
        assert_eq!(Monitor::Number(1).find(&names), Ok(1));
    }

    #[test]
    fn the_listing_has_a_line_each() {
        let list = listing(&[
            MonitorInfo {
                name: "DSI-1".into(),
                width: 800,
                height: 480,
                x: 0,
                y: 0,
            },
            MonitorInfo {
                name: "HDMI-1".into(),
                width: 1920,
                height: 1080,
                x: 800,
                y: 0,
            },
        ]);
        let lines: Vec<&str> = list.lines().collect();
        assert_eq!(lines.len(), 4, "{list}");
        assert!(lines[1].starts_with("0 ") && lines[1].contains(r#""DSI-1""#));
        assert!(lines[1].contains("800x480"));
        assert!(lines[2].contains("1920x1080") && lines[2].ends_with("800,0"));
        assert!(listing(&[]).starts_with("no monitor found"));
    }
}
