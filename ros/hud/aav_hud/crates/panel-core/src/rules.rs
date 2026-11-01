//! The actuator tests' safety rules, ported from `aav_actuator_tests/safety.py` so the
//! panel and the tests agree on what REAL means, when the stands may drive, when the
//! speed can't be seen, and when someone else is driving. The tests below are
//! `test_safety.py`'s own vectors.
//!
//! | Python | Here |
//! |---|---|
//! | `source_kind` | [`source_kind`] |
//! | `preflight_refusal` | [`preflight_refusal`] |
//! | `unseen` | [`unseen`] |
//! | `taken_over` | [`taken_over`] |
//!
//! One difference, towards safety: a source number this program doesn't know
//! ([`Origin::Unknown`]) counts as REAL here; `safety.py` would call it SIM.

use std::fmt;

use hud_core::origin::Origin;

use crate::vehicle::SpeedWheels;

/// m/s: never faster with the wheels off the ground (docs/tests.md stage 3).
pub const STANDS_CAP: f64 = 0.3;
/// s of a command above [`UNSEEN_SPEED`] with the measured speed below it.
pub const UNSEEN_TIME: f64 = 4.0;
/// m/s: the spine's own standstill threshold.
pub const UNSEEN_SPEED: f64 = 0.05;
/// How close a command must be to ours to be ours (m/s, rad/s).
pub const TAKEN_OVER_TOLERANCE: f64 = 1e-3;

/// What `/system/source` says we're driving.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Real,
    Sim,
    Bench,
    Replay,
    /// Nobody announced: treated as the real car.
    Unknown,
}

impl Kind {
    pub fn word(self) -> &'static str {
        match self {
            Self::Real => "REAL",
            Self::Sim => "SIM",
            Self::Bench => "BENCH",
            Self::Replay => "REPLAY",
            Self::Unknown => "UNKNOWN",
        }
    }

    /// The real car, or as good as (nobody said otherwise).
    pub fn is_real(self) -> bool {
        matches!(self, Self::Real | Self::Unknown)
    }
}

impl fmt::Display for Kind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.word())
    }
}

/// Merges every announcer's entries on `/system/source`. Anything REAL makes it REAL,
/// even beside a simulator: one real actuator is enough.
pub fn source_kind(origins: impl IntoIterator<Item = Origin>) -> Kind {
    let mut seen = [false; 4];
    let mut any = false;
    for o in origins {
        any = true;
        let i = match o {
            Origin::Real | Origin::Unknown(_) => 0,
            Origin::Replay => 1,
            Origin::Bench => 2,
            Origin::Sim => 3,
        };
        seen[i] = true;
    }
    match seen {
        _ if !any => Kind::Unknown,
        [true, ..] => Kind::Real,
        [_, true, ..] => Kind::Replay,
        [_, _, true, _] => Kind::Bench,
        _ => Kind::Sim,
    }
}

/// Why the preflight (and the panel, on stands) may not drive the real car.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Refusal {
    /// `/vehicle/odom` measures wheels the motor doesn't drive (TODO 49b).
    NotDrivenWheels(Option<SpeedWheels>),
    /// The 2025-26 C++ ESP32: no command timeout.
    NoTimeoutLock,
    /// The legacy bridge's speed loop chases a speed it can't reach on stands.
    SpeedLoopPi,
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotDrivenWheels(w) => write!(
                f,
                "/vehicle/odom measures the {} wheels (vehicle.yaml vehicle.speed_wheels), \
                 but the car drives the rear ones: on stands the speed reads 0 while the \
                 motor spins, so no speed cap can work. Run --no-drive (the sensors only) \
                 until a driven-wheel speed exists (TODO 49b, the hardware lead)",
                w.as_ref().map_or("unknown", SpeedWheels::word)
            ),
            Self::NoTimeoutLock => f.write_str(
                "the legacy bridge's no-timeout lock is ON: the C++ ESP32 has no command \
                 timeout, so a lost link leaves the throttle on. Flash the Rust firmware \
                 (car-day session 6) and pass test 3.9 first",
            ),
            Self::SpeedLoopPi => f.write_str(
                "the legacy bridge runs speed_loop.mode pi: on stands its loop chases a \
                 speed it can't reach and raises the throttle. Restart it with \
                 speed_loop.mode open",
            ),
        }
    }
}

/// Whether the preflight may drive the real car (`Ok`), or why not.
///
/// `drive`: false for `--no-drive` (nothing moves, so nothing refused). `speed_wheels`:
/// vehicle.yaml's. `no_timeout_lock`: the legacy bridge's lock (None: not heard).
/// `speed_loop_mode`: the legacy bridge's `speed_loop.mode` (None: not heard).
pub fn preflight_refusal(
    kind: Kind,
    drive: bool,
    speed_wheels: Option<&SpeedWheels>,
    no_timeout_lock: Option<bool>,
    speed_loop_mode: Option<&str>,
) -> Result<(), Refusal> {
    if !drive || !kind.is_real() {
        return Ok(());
    }
    if !speed_wheels.is_some_and(SpeedWheels::are_driven) {
        return Err(Refusal::NotDrivenWheels(speed_wheels.cloned()));
    }
    if no_timeout_lock == Some(true) {
        return Err(Refusal::NoTimeoutLock);
    }
    if speed_loop_mode == Some("pi") {
        return Err(Refusal::SpeedLoopPi);
    }
    Ok(())
}

/// One moment of driving: seconds, the command and the measured speed (m/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    pub t: f64,
    pub commanded: f64,
    pub measured: f64,
}

impl From<(f64, f64, f64)> for Sample {
    fn from((t, commanded, measured): (f64, f64, f64)) -> Self {
        Self {
            t,
            commanded,
            measured,
        }
    }
}

/// The wheels are told to turn but the speed sensor sees nothing (see [`unseen`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Unseen;

impl fmt::Display for Unseen {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "told to drive for {UNSEEN_TIME} s, but the measured speed stays under \
             {UNSEEN_SPEED} m/s: the speed can't be seen (front wheel sensors on stands? \
             a dead sensor?) or the car isn't moving (the spine not armed?), so the speed \
             cap can't work"
        )
    }
}

/// Whether to stop because the speed can't be seen: [`UNSEEN_TIME`] of commands above
/// [`UNSEEN_SPEED`] while the measured speed stays below it. `history`: oldest first.
///
/// On a rear-wheel-drive car whose wheel sensors are on the front, on stands, this is
/// what happens: the motor turns the rear wheels at a speed nobody measures.
pub fn unseen(history: &[Sample]) -> Option<Unseen> {
    let (first, last) = (history.first()?, history.last()?);
    if last.t - first.t < UNSEEN_TIME {
        return None;
    }
    history
        .iter()
        .filter(|s| last.t - s.t <= UNSEEN_TIME)
        .all(|s| s.commanded.abs() > UNSEEN_SPEED && s.measured.abs() < UNSEEN_SPEED)
        .then_some(Unseen)
}

/// Whether twist_mux's output (`theirs`: speed, turn rate) is none of `ours`: a
/// higher-priority driver (the joystick, the keyboard) is driving.
pub fn taken_over(ours: impl IntoIterator<Item = (f64, f64)>, theirs: (f64, f64)) -> bool {
    !ours.into_iter().any(|(v, w)| {
        (v - theirs.0).abs() <= TAKEN_OVER_TOLERANCE && (w - theirs.1).abs() <= TAKEN_OVER_TOLERANCE
    })
}

#[cfg(test)]
mod tests {
    //! `ros/drivers/aav_actuator_tests/test/test_safety.py`'s vectors, one for one.
    use super::*;

    const REAL: Origin = Origin::Real;
    const SIM: Origin = Origin::Sim;
    const BENCH: Origin = Origin::Bench;
    const REPLAY: Origin = Origin::Replay;

    /// `safety.source_kind({announcer: [values]})`: every announcer's entries together.
    fn kind(announcements: &[&[Origin]]) -> Kind {
        source_kind(announcements.iter().flat_map(|a| a.iter().copied()))
    }

    #[test]
    fn source_kind_errs_towards_real() {
        assert_eq!(kind(&[]), Kind::Unknown);
        assert_eq!(kind(&[&[SIM, SIM]]), Kind::Sim);
        assert_eq!(kind(&[&[BENCH], &[BENCH]]), Kind::Bench);
        assert_eq!(kind(&[&[REAL]]), Kind::Real);
        // One real actuator beside a simulator is the real car.
        assert_eq!(kind(&[&[SIM], &[REAL]]), Kind::Real);
        assert_eq!(kind(&[&[REPLAY]]), Kind::Replay);
        // Here only: a number we don't know is the real car.
        assert_eq!(kind(&[&[SIM], &[Origin::Unknown(9)]]), Kind::Real);
    }

    #[test]
    fn taken_over_vectors() {
        let ours = [(0.5, 0.0), (0.5, 0.1)];
        assert!(!taken_over(ours, (0.5, 0.1)));
        assert!(taken_over(ours, (0.0, 0.0))); // the joystick's deadman: stop
        assert!(taken_over([], (0.5, 0.0)));
    }

    fn samples(n: u32, cmd: f64, measured: f64) -> Vec<Sample> {
        (0..n)
            .map(|t| (f64::from(t) / 10.0, cmd, measured).into())
            .collect()
    }

    #[test]
    fn unseen_speed_stops_after_four_seconds() {
        let spinning = samples(45, 0.3, 0.0);
        assert_eq!(unseen(&spinning[..35]), None); // 3.4 s: not yet (Hall clicks)
        let why = unseen(&spinning).expect("4.4 s unseen");
        assert!(why.to_string().contains("can't be seen"));
        assert_eq!(unseen(&samples(25, 0.3, 0.2)), None); // moving
        assert_eq!(unseen(&samples(25, 0.0, 0.0)), None); // parked
        assert_eq!(unseen(&[]), None);
    }

    #[test]
    fn the_preflight_refuses_the_real_car_without_a_driven_wheel_speed() {
        let (front, rear) = (SpeedWheels::Front, SpeedWheels::Rear);
        let why = preflight_refusal(Kind::Real, true, Some(&front), Some(false), Some("open"))
            .expect_err("front wheels");
        let why = why.to_string();
        assert!(why.contains("49b") && why.contains("--no-drive"), "{why}");
        assert!(preflight_refusal(Kind::Unknown, true, None, None, None).is_err());
        assert_eq!(
            preflight_refusal(Kind::Real, false, Some(&front), Some(true), Some("pi")),
            Ok(())
        );
        assert_eq!(
            preflight_refusal(Kind::Sim, true, Some(&front), None, None),
            Ok(())
        );
        let lock = preflight_refusal(Kind::Real, true, Some(&rear), Some(true), Some("open"));
        assert!(lock.expect_err("lock").to_string().contains("no-timeout"));
        let pi = preflight_refusal(Kind::Real, true, Some(&rear), Some(false), Some("pi"));
        assert!(pi.expect_err("pi").to_string().contains("pi"));
        assert_eq!(
            preflight_refusal(Kind::Real, true, Some(&rear), Some(false), Some("open")),
            Ok(())
        );
    }
}
