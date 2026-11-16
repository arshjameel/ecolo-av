//! Where the data comes from: the real car, a simulator, the bench or a recording
//! (`/system/source`, aav_interfaces/DataSource).
//!
//! Several nodes announce, each its own list (Gazebo, the bench's stand-ins...). The
//! newest list from each announcer is kept; together they say what the HUD shows: a big
//! banner when anything isn't real, and a tag beside each topic it knows about.

use std::{collections::BTreeMap, fmt};

/// Where one sensor's data comes from (aav_interfaces/SensorSource's constants).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Origin {
    /// A real device on the real car.
    Real,
    /// A simulator: Gazebo, turtlesim.
    Sim,
    /// The bench's stand-ins.
    Bench,
    /// A recording, played back.
    Replay,
    /// A number this HUD doesn't know: newer than the HUD. Never taken for real.
    Unknown(u8),
}

impl From<u8> for Origin {
    fn from(code: u8) -> Self {
        match code {
            0 => Self::Real,
            1 => Self::Sim,
            2 => Self::Bench,
            3 => Self::Replay,
            other => Self::Unknown(other),
        }
    }
}

impl Origin {
    /// The short tag beside a topic.
    #[must_use]
    pub fn tag(self) -> &'static str {
        match self {
            Self::Real => "REAL",
            Self::Sim => "SIM",
            Self::Bench => "BENCH",
            Self::Replay => "REPLAY",
            Self::Unknown(_) => "?",
        }
    }

    /// The banner's word; None for real data, which needs no banner.
    #[must_use]
    pub fn banner(self) -> Option<&'static str> {
        match self {
            Self::Real => None,
            Self::Sim => Some("SIMULATED"),
            Self::Bench => Some("BENCH"),
            Self::Replay => Some("REPLAY"),
            Self::Unknown(_) => Some("NOT REAL?"),
        }
    }

    /// True for real data.
    #[must_use]
    pub fn is_real(self) -> bool {
        self == Self::Real
    }
}

impl fmt::Display for Origin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.tag())
    }
}

/// One sensor in an announcement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Entry {
    /// What it is: imu, gps, camera_front_left...
    pub name: String,
    /// Its main topic; empty if it has none.
    pub topic: String,
    /// Where its data comes from.
    pub origin: Origin,
}

/// One node's message on `/system/source`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Announcement {
    /// The node that says so.
    pub announcer: String,
    /// Each sensor it speaks for.
    pub entries: Vec<Entry>,
}

/// Every announcement heard, the newest per announcer.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Origins {
    by_announcer: BTreeMap<String, Vec<Entry>>,
}

impl Origins {
    /// Takes in one announcement: it replaces that announcer's last one.
    pub fn apply(&mut self, a: Announcement) {
        self.by_announcer.insert(a.announcer, a.entries);
    }

    /// Every entry heard, from every announcer.
    pub fn entries(&self) -> impl Iterator<Item = &Entry> {
        self.by_announcer.values().flatten()
    }

    /// The banner's text: each kind of data that isn't real, in a fixed order
    /// ("SIMULATED", "BENCH + REPLAY"...). None when everything heard is real, or nothing
    /// has been heard.
    #[must_use]
    pub fn banner(&self) -> Option<String> {
        let mut kinds: Vec<Origin> = self
            .entries()
            .map(|e| e.origin)
            .filter(|o| !o.is_real())
            .collect();
        kinds.sort();
        kinds.dedup();
        let words: Vec<&str> = kinds.iter().filter_map(|o| o.banner()).collect();
        (!words.is_empty()).then(|| words.join(" + "))
    }

    /// Where one topic's data comes from, if anyone said. If two announcers disagree,
    /// the less real one wins: a banner too many is safer than one too few.
    #[must_use]
    pub fn of_topic(&self, topic: &str) -> Option<Origin> {
        self.entries()
            .filter(|e| !e.topic.is_empty() && e.topic == topic)
            .map(|e| e.origin)
            .max()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn says(announcer: &str, list: &[(&str, &str, u8)]) -> Announcement {
        Announcement {
            announcer: announcer.into(),
            entries: list
                .iter()
                .map(|&(name, topic, code)| Entry {
                    name: name.into(),
                    topic: topic.into(),
                    origin: Origin::from(code),
                })
                .collect(),
        }
    }

    #[test]
    fn codes_match_the_message() {
        assert_eq!(Origin::from(0), Origin::Real);
        assert_eq!(Origin::from(1), Origin::Sim);
        assert_eq!(Origin::from(2), Origin::Bench);
        assert_eq!(Origin::from(3), Origin::Replay);
        assert_eq!(Origin::from(9), Origin::Unknown(9));
    }

    #[test]
    fn nothing_heard_means_no_banner() {
        assert_eq!(Origins::default().banner(), None);
    }

    #[test]
    fn the_real_car_has_no_banner() {
        let mut o = Origins::default();
        o.apply(says("spine_bridge", &[("vehicle", "/vehicle/status", 0)]));
        assert_eq!(o.banner(), None);
        assert_eq!(o.of_topic("/vehicle/status"), Some(Origin::Real));
    }

    #[test]
    fn gazebo_says_simulated() {
        let mut o = Origins::default();
        o.apply(says(
            "gazebo_adapter",
            &[
                ("imu", "/sensors/imu/data_raw", 1),
                ("gps", "/sensors/gps/fix", 1),
            ],
        ));
        assert_eq!(o.banner().as_deref(), Some("SIMULATED"));
        assert_eq!(o.of_topic("/sensors/gps/fix"), Some(Origin::Sim));
        assert_eq!(o.of_topic("/vehicle/battery"), None);
    }

    #[test]
    fn announcers_merge_and_a_new_list_replaces_the_old() {
        let mut o = Origins::default();
        o.apply(says("spine_bridge", &[("vehicle", "/vehicle/status", 2)]));
        o.apply(says("mock_gps", &[("gps", "/sensors/gps/fix", 2)]));
        o.apply(says(
            "replay_source",
            &[("imu", "/sensors/imu/data_raw", 3)],
        ));
        assert_eq!(o.banner().as_deref(), Some("BENCH + REPLAY"));
        o.apply(says("replay_source", &[]));
        assert_eq!(o.banner().as_deref(), Some("BENCH"));
        assert_eq!(o.entries().count(), 2);
    }

    #[test]
    fn a_disagreement_shows_the_less_real_one() {
        let mut o = Origins::default();
        o.apply(says("a", &[("gps", "/sensors/gps/fix", 0)]));
        o.apply(says("b", &[("gps", "/sensors/gps/fix", 1)]));
        assert_eq!(o.of_topic("/sensors/gps/fix"), Some(Origin::Sim));
        assert_eq!(o.banner().as_deref(), Some("SIMULATED"));
    }

    #[test]
    fn an_unknown_code_is_never_taken_for_real() {
        let mut o = Origins::default();
        o.apply(says("future", &[("x", "", 7)]));
        assert_eq!(o.banner().as_deref(), Some("NOT REAL?"));
        assert_eq!(o.of_topic(""), None);
    }
}
