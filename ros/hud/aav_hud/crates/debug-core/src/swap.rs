//! Keys 1-4: swap the trial tracker (aav_track) while it drives, in the simulator only.
//!
//! The debug HUD never touches an actuator (the design's D3); it may only retune a
//! **simulated** tracker, through the track node's own parameter service. The track
//! node refuses too, unless it was started with `live_swap:=true` (sim.launch.py).

use hud_core::origin::{Origin, Origins};
use thiserror::Error;

/// The trackers, in key order: 1 pure pursuit, 2 Stanley, 3 LQR, 4 MPC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tracker {
    PurePursuit,
    Stanley,
    Lqr,
    Mpc,
}

impl Tracker {
    pub const ALL: [Tracker; 4] = [
        Tracker::PurePursuit,
        Tracker::Stanley,
        Tracker::Lqr,
        Tracker::Mpc,
    ];

    /// aav_track's name for it (its `tracker` parameter).
    pub fn name(self) -> &'static str {
        match self {
            Self::PurePursuit => "pure_pursuit",
            Self::Stanley => "stanley",
            Self::Lqr => "lqr",
            Self::Mpc => "mpc",
        }
    }

    /// The tracker on key `digit` (1 to 4).
    pub fn on_key(digit: u8) -> Option<Self> {
        Self::ALL.get(usize::from(digit.checked_sub(1)?)).copied()
    }
}

/// Why a swap isn't asked for.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum Refused {
    #[error("no /system/source yet: swaps only in the simulator")]
    NoSource,
    #[error("{0} data: trackers swap only in the simulator")]
    NotSim(&'static str),
}

/// Whether a swap may be asked for: only when `/system/source` has been heard and every
/// sensor it lists is simulated.
pub fn allowed(origins: &Origins) -> Result<(), Refused> {
    let mut entries = origins.entries().peekable();
    if entries.peek().is_none() {
        return Err(Refused::NoSource);
    }
    match entries.map(|e| e.origin).find(|&o| o != Origin::Sim) {
        Some(other) => Err(Refused::NotSim(other.tag())),
        None => Ok(()),
    }
}

/// The track node's answer to a swap, for the panel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reply {
    /// The tracker asked for.
    pub asked: &'static str,
    /// Ok, or the reason it was refused (by this window or by the node).
    pub outcome: Result<(), String>,
}

impl Reply {
    /// The panel's line.
    pub fn line(&self) -> String {
        match &self.outcome {
            Ok(()) => format!("swapped to {}", self.asked),
            Err(why) => format!("{} refused: {why}", self.asked),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hud_core::origin::{Announcement, Entry};

    fn origins(list: &[Origin]) -> Origins {
        let mut o = Origins::default();
        o.apply(Announcement {
            announcer: "gazebo".into(),
            entries: list
                .iter()
                .enumerate()
                .map(|(i, &origin)| Entry {
                    name: format!("s{i}"),
                    topic: format!("/t{i}"),
                    origin,
                })
                .collect(),
        });
        o
    }

    #[test]
    fn keys_pick_trackers() {
        assert_eq!(Tracker::on_key(1), Some(Tracker::PurePursuit));
        assert_eq!(Tracker::on_key(4).map(Tracker::name), Some("mpc"));
        assert_eq!(Tracker::on_key(0), None);
        assert_eq!(Tracker::on_key(5), None);
    }

    #[test]
    fn only_the_simulator_swaps() {
        assert_eq!(allowed(&Origins::default()), Err(Refused::NoSource));
        assert_eq!(allowed(&origins(&[Origin::Sim, Origin::Sim])), Ok(()));
        assert_eq!(
            allowed(&origins(&[Origin::Sim, Origin::Real])),
            Err(Refused::NotSim("REAL"))
        );
        assert!(allowed(&origins(&[Origin::Bench])).is_err());
    }

    #[test]
    fn replies_read_plainly() {
        let ok = Reply {
            asked: "lqr",
            outcome: Ok(()),
        };
        assert_eq!(ok.line(), "swapped to lqr");
        let no = Reply {
            asked: "mpc",
            outcome: Err("live_swap is off".into()),
        };
        assert_eq!(no.line(), "mpc refused: live_swap is off");
    }
}
