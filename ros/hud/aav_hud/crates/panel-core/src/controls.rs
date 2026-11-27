//! Who may do what, where: the hardware page's controls by its host and the source
//! (Ben, 26 Nov, option A: "Watch-only on the car: calibration stays laptop-only").
//!
//! | Host | Source | Controls |
//! |---|---|---|
//! | `hw_panel` (a laptop) | any | [`Controls::Full`]: the page's own rules apply, as before |
//! | the map (the dash screen) | SIM, BENCH, REPLAY | [`Controls::Full`] |
//! | the map (the dash screen) | REAL, or not heard yet | [`Controls::WatchOnly`]: numbers and graphs; E-STOP only |
//!
//! Watch-only, [`Action`] by action:
//!
//! | Action | Watch-only |
//! |---|---|
//! | [`Action::Estop`] | allowed: it only ever sends true |
//! | [`Action::StopOwnRun`] | allowed: stopping is always allowed, and the tab only stops a run it started |
//! | [`Action::Drive`] | off |
//! | [`Action::Calibrate`] | off |
//! | [`Action::RunLadder`] | off |
//! | [`Action::Jacks`] | off |
//!
//! "Full" does not mean "on": the gate, the calibration page's checks and ladder-core
//! still decide each button. This only takes buttons away.

use std::fmt;

use crate::rules::Kind;

/// Which window the hardware page is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Host {
    /// `hw_panel`: the page is the whole program (a laptop).
    Standalone,
    /// The map window (the car's dash screen): a MAP (F2) button goes back to the map.
    Map,
}

/// Something a person can do on the hardware page.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// The DRIVE tab's buttons: speed, steering, CENTRE, STOP, hold-to-drive GO.
    Drive,
    /// The STEERING CAL tab's buttons: limp, hold, steps, marks, the check, SAVE.
    Calibrate,
    /// The LADDER tab's RUN.
    RunLadder,
    /// The LADDER tab's STOP, of a run this tab started.
    StopOwnRun,
    /// The LADDER tab's jacks button (the simulated car).
    Jacks,
    /// The E-STOP: only ever sets it.
    Estop,
}

impl Action {
    /// Every action, for tests and tables.
    pub const ALL: [Self; 6] = [
        Self::Drive,
        Self::Calibrate,
        Self::RunLadder,
        Self::StopOwnRun,
        Self::Jacks,
        Self::Estop,
    ];
}

/// What the page lets a person do, here and now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Controls {
    /// Every control, under its own rules.
    Full,
    /// The dash screen on the real car: it only shows; E-STOP and stopping its own run stay.
    WatchOnly,
}

/// Why a button is off on the dash screen, in words for the screen.
pub const WATCH_ONLY: &str = "watch-only on the car's screen: calibrate and drive from a laptop";

impl Controls {
    /// The controls for a page in `host`, with `/system/source` saying `kind`.
    /// Not heard yet ([`Kind::Unknown`]) counts as the real car.
    #[must_use]
    pub fn for_host(host: Host, kind: Kind) -> Self {
        match (host, kind) {
            (Host::Map, Kind::Real | Kind::Unknown) => Self::WatchOnly,
            (Host::Map, Kind::Sim | Kind::Bench | Kind::Replay) | (Host::Standalone, _) => {
                Self::Full
            }
        }
    }

    /// Whether `action` may be done here.
    #[must_use]
    pub fn allows(self, action: Action) -> bool {
        match self {
            Self::Full => true,
            Self::WatchOnly => matches!(action, Action::Estop | Action::StopOwnRun),
        }
    }

    /// Why `action` is off here, if it is.
    #[must_use]
    pub fn why_not(self, action: Action) -> Option<WatchOnly> {
        (!self.allows(action)).then_some(WatchOnly)
    }

    /// Whether the page only shows.
    #[must_use]
    pub fn is_watch_only(self) -> bool {
        self == Self::WatchOnly
    }
}

/// The reason a control is off: the dash screen on the real car.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WatchOnly;

impl fmt::Display for WatchOnly {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(WATCH_ONLY)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KINDS: [Kind; 5] = [
        Kind::Real,
        Kind::Sim,
        Kind::Bench,
        Kind::Replay,
        Kind::Unknown,
    ];

    #[test]
    fn hw_panel_keeps_every_control_whatever_the_source() {
        for kind in KINDS {
            let c = Controls::for_host(Host::Standalone, kind);
            assert_eq!(c, Controls::Full, "{kind}");
            assert!(Action::ALL.into_iter().all(|a| c.allows(a)), "{kind}");
        }
    }

    #[test]
    fn the_map_on_sim_bench_or_replay_keeps_every_control() {
        for kind in [Kind::Sim, Kind::Bench, Kind::Replay] {
            let c = Controls::for_host(Host::Map, kind);
            assert_eq!(c, Controls::Full, "{kind}");
            assert!(Action::ALL.into_iter().all(|a| c.why_not(a).is_none()));
        }
    }

    #[test]
    fn the_map_on_the_real_car_or_unheard_only_watches() {
        for kind in [Kind::Real, Kind::Unknown] {
            let c = Controls::for_host(Host::Map, kind);
            assert_eq!(c, Controls::WatchOnly, "{kind}");
            assert!(c.is_watch_only());
            for a in [
                Action::Drive,
                Action::Calibrate,
                Action::RunLadder,
                Action::Jacks,
            ] {
                assert!(!c.allows(a), "{kind}: {a:?}");
                assert_eq!(c.why_not(a), Some(WatchOnly));
            }
            assert!(c.allows(Action::Estop), "stopping is always allowed");
            assert!(c.allows(Action::StopOwnRun), "stopping is always allowed");
        }
    }

    #[test]
    fn the_reason_says_where_to_go_instead() {
        let s = WatchOnly.to_string();
        assert!(s.contains("watch-only") && s.contains("laptop"), "{s}");
    }
}
