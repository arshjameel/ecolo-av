//! The lock-to-lock check: once the limits are set, jog slowly from the centre to each
//! limit and back, and stop at once if the steering actuator stops following.
//!
//! ```text
//!   right limit        centre         left limit       (counts; left may be up or down)
//!        |<-- 4 ---------- | ---------- 2 -->|
//!        |--------- 5 ---->|<---- 3 ---------|          1: from where it is, to the centre
//! ```
//!
//! Each tick it asks for one jog target, moving at [`SweepSettings::rate`] (slower than
//! the firmware's own 300 counts a second, so the firmware's target is ours). It only ever
//! asks for targets between the limits: it won't start with the wheels outside them.
//!
//! | It stops (and the page holds where the wheels are) when | [`Stop`] |
//! |---|---|
//! | the position is more than `max_error` from the target | `Resistance` |
//! | the target moves but the position hasn't, by `min_move`, for `stall_after` | `NoMovement` |
//! | at a leg's end, the position isn't within `arrive_within` after `settle` | `NotArrived` |
//! | no position for `lost_after` | `LostPosition` |
//! | the firmware isn't jogging (after `grace`): refused, holding, returning, off | `Firmware` |
//! | the page says so: a person, an e-stop, the car moving | `Aborted` |
//!
//! The checks start `grace` into each leg, so the actuator has time to start.

use std::{
    fmt,
    time::{Duration, Instant},
};

use crate::{calibration::FirmwareState, steering_limits::Limits};

/// The check's numbers (config/hw_panel.yaml).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweepSettings {
    /// Counts a second the target moves.
    pub rate: f64,
    /// Counts: further from the target than this is resistance.
    pub max_error: u16,
    /// Counts: less movement than this is no movement.
    pub min_move: u16,
    /// No movement for this long, while the target moves, stops it.
    pub stall_after: Duration,
    /// Counts: close enough to a leg's end.
    pub arrive_within: u16,
    /// The longest wait at a leg's end.
    pub settle: Duration,
    /// No position for this long stops it.
    pub lost_after: Duration,
    /// The checks start this long into each leg.
    pub grace: Duration,
}

impl Default for SweepSettings {
    fn default() -> Self {
        Self {
            rate: 100.0,
            max_error: 150,
            min_move: 5,
            stall_after: Duration::from_millis(600),
            arrive_within: 20,
            settle: Duration::from_secs(2),
            lost_after: Duration::from_millis(300),
            grace: Duration::from_millis(500),
        }
    }
}

/// One leg of the check.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Leg {
    ToCentre,
    ToLeft,
    BackFromLeft,
    ToRight,
    BackFromRight,
}

impl Leg {
    pub const ALL: [Leg; 5] = [
        Leg::ToCentre,
        Leg::ToLeft,
        Leg::BackFromLeft,
        Leg::ToRight,
        Leg::BackFromRight,
    ];

    fn next(self) -> Option<Leg> {
        let i = Leg::ALL.iter().position(|&l| l == self)?;
        Leg::ALL.get(i + 1).copied()
    }
}

impl fmt::Display for Leg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Leg::ToCentre => "to the centre",
            Leg::ToLeft => "centre to the left limit",
            Leg::BackFromLeft => "left limit back to the centre",
            Leg::ToRight => "centre to the right limit",
            Leg::BackFromRight => "right limit back to the centre",
        })
    }
}

/// Why the check stopped.
#[derive(Debug, Clone, PartialEq)]
pub enum Stop {
    Resistance { error: u16 },
    NoMovement { for_s: f64 },
    NotArrived { error: u16 },
    LostPosition,
    Firmware(FirmwareState),
    Aborted(String),
}

impl fmt::Display for Stop {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Stop::Resistance { error } => write!(
                f,
                "resistance: the position fell {error} counts behind the target"
            ),
            Stop::NoMovement { for_s } => write!(
                f,
                "resistance: no movement for {for_s:.1} s while the target moved"
            ),
            Stop::NotArrived { error } => {
                write!(f, "didn't reach the leg's end: {error} counts short")
            }
            Stop::LostPosition => f.write_str("the position was lost"),
            Stop::Firmware(s) => write!(f, "the firmware isn't jogging: {s}"),
            Stop::Aborted(why) => f.write_str(why),
        }
    }
}

/// How it ended.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// Every leg passed: the largest error seen on each (counts).
    Passed { max_error: Vec<(Leg, u16)> },
    Stopped {
        leg: Leg,
        why: Stop,
        /// Where the wheels were, and the target then (counts).
        position: u16,
        target: u16,
    },
}

impl fmt::Display for Outcome {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Outcome::Passed { max_error } => {
                let worst = max_error.iter().map(|&(_, e)| e).max().unwrap_or(0);
                write!(
                    f,
                    "PASSED: lock to lock and back, the position within {worst} counts of the \
                     target"
                )
            }
            Outcome::Stopped {
                leg,
                why,
                position,
                target,
            } => write!(
                f,
                "STOPPED ({leg}): {why}; at {position} counts, target {target}. Holding there"
            ),
        }
    }
}

/// Why the check can't start.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CantStart {
    NoPosition,
    /// The wheels are outside the limits: the check only asks for targets inside them.
    Outside {
        position: u16,
        lo: u16,
        hi: u16,
    },
}

impl fmt::Display for CantStart {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CantStart::NoPosition => f.write_str("no position from the steering actuator"),
            CantStart::Outside { position, lo, hi } => write!(
                f,
                "the wheels ({position}) are outside the limits ({lo} to {hi}): step them \
                 inside first"
            ),
        }
    }
}

/// Where a leg is.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// The target moves from `from` to the leg's end.
    Moving,
    /// The target is at the leg's end; waiting for the wheels, since then.
    Settling(Instant),
}

/// The check, one tick at a time.
#[derive(Debug, Clone, PartialEq)]
pub struct Sweep {
    settings: SweepSettings,
    centre: u16,
    left: u16,
    right: u16,
    lo: u16,
    hi: u16,
    leg: Leg,
    from: f64,
    leg_started: Instant,
    phase: Phase,
    target: f64,
    /// The position when it last moved by `min_move`, and when.
    moved: (u16, Instant),
    /// The last position, and when it came.
    seen: (u16, Instant),
    max_error: Vec<(Leg, u16)>,
    outcome: Option<Outcome>,
}

impl Sweep {
    /// Starts the check with the wheels at `position`.
    pub fn start(
        now: Instant,
        position: Option<u16>,
        limits: &Limits,
        settings: SweepSettings,
    ) -> Result<Self, CantStart> {
        let position = position.ok_or(CantStart::NoPosition)?;
        if !(limits.lo..=limits.hi).contains(&position) {
            return Err(CantStart::Outside {
                position,
                lo: limits.lo,
                hi: limits.hi,
            });
        }
        Ok(Self {
            settings,
            centre: limits.centre,
            left: limits.left_limit(),
            right: limits.right_limit(),
            lo: limits.lo,
            hi: limits.hi,
            leg: Leg::ToCentre,
            from: f64::from(position),
            leg_started: now,
            phase: Phase::Moving,
            target: f64::from(position),
            moved: (position, now),
            seen: (position, now),
            max_error: vec![(Leg::ToCentre, 0)],
            outcome: None,
        })
    }

    /// The leg it is on.
    pub fn leg(&self) -> Leg {
        self.leg
    }

    /// How it ended, once it has.
    pub fn outcome(&self) -> Option<&Outcome> {
        self.outcome.as_ref()
    }

    /// The target now (counts), always between the limits.
    pub fn target(&self) -> u16 {
        // Between lo and hi by construction; clamped again, so it can never leave them.
        (self.target.round() as u16).clamp(self.lo, self.hi)
    }

    fn leg_end(&self) -> u16 {
        match self.leg {
            Leg::ToCentre | Leg::BackFromLeft | Leg::BackFromRight => self.centre,
            Leg::ToLeft => self.left,
            Leg::ToRight => self.right,
        }
    }

    /// Ends it now, holding where the wheels are.
    pub fn abort(&mut self, why: impl Into<String>) {
        self.stop(Stop::Aborted(why.into()));
    }

    fn stop(&mut self, why: Stop) {
        if self.outcome.is_none() {
            self.outcome = Some(Outcome::Stopped {
                leg: self.leg,
                why,
                position: self.seen.0,
                target: self.target(),
            });
        }
    }

    /// One tick: the position (None: not seen this tick) and the firmware's state. Returns
    /// the jog target to ask for, or None once it has ended (see [`Sweep::outcome`]).
    pub fn step(
        &mut self,
        now: Instant,
        position: Option<u16>,
        firmware: FirmwareState,
    ) -> Option<u16> {
        if self.outcome.is_some() {
            return None;
        }
        let s = self.settings;
        let in_grace = now.saturating_duration_since(self.leg_started) < s.grace;

        // The position, and whether it moved.
        match position {
            Some(p) => {
                self.seen = (p, now);
                if p.abs_diff(self.moved.0) >= s.min_move {
                    self.moved = (p, now);
                }
            }
            None if now.saturating_duration_since(self.seen.1) > s.lost_after => {
                self.stop(Stop::LostPosition);
                return None;
            }
            None => {}
        }
        let p = self.seen.0;

        // The firmware must be jogging: refused at once, anything else after the grace.
        match firmware {
            FirmwareState::Jog => {}
            FirmwareState::Refused(_) => {
                self.stop(Stop::Firmware(firmware));
                return None;
            }
            _ if !in_grace => {
                self.stop(Stop::Firmware(firmware));
                return None;
            }
            _ => {}
        }

        // The target moves, then waits at the leg's end.
        let end = f64::from(self.leg_end());
        if self.phase == Phase::Moving {
            let t = now
                .saturating_duration_since(self.leg_started)
                .as_secs_f64();
            let travel = s.rate * t;
            let towards = end - self.from;
            if travel >= towards.abs() {
                self.target = end;
                self.phase = Phase::Settling(now);
            } else {
                self.target = self.from + towards.signum() * travel;
            }
        }

        let error = (f64::from(p) - self.target).abs().round() as u16;
        if let Some((_, worst)) = self.max_error.last_mut() {
            *worst = (*worst).max(error);
        }
        if !in_grace {
            if error > s.max_error {
                self.stop(Stop::Resistance { error });
                return None;
            }
            let still = now.saturating_duration_since(self.moved.1);
            if self.phase == Phase::Moving && still >= s.stall_after {
                self.stop(Stop::NoMovement {
                    for_s: still.as_secs_f64(),
                });
                return None;
            }
        }
        if let Phase::Settling(since) = self.phase {
            let short = p.abs_diff(self.leg_end());
            if short <= s.arrive_within {
                match self.leg.next() {
                    Some(next) => {
                        self.leg = next;
                        self.from = self.target;
                        self.leg_started = now;
                        self.phase = Phase::Moving;
                        self.moved = (p, now);
                        self.max_error.push((next, 0));
                    }
                    None => {
                        self.outcome = Some(Outcome::Passed {
                            max_error: self.max_error.clone(),
                        });
                        return None;
                    }
                }
            } else if now.saturating_duration_since(since) > s.settle {
                self.stop(Stop::NotArrived { error: short });
                return None;
            }
        }
        Some(self.target())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::Refusal;
    use crate::steering_limits::{compute, Marks};

    const TICK: Duration = Duration::from_millis(20);

    fn limits() -> Limits {
        // Left raises: left limit 3346, right limit 624, centre 1985.
        compute(&Marks {
            left: 3420,
            centre: 1985,
            right: 610,
            left_deg: None,
            right_deg: None,
        })
        .expect("limits")
    }

    /// A pretend steering actuator: follows the target at up to `speed` counts a second,
    /// but can't pass `wall` (counts) going up, if there is one.
    struct Actuator {
        at: f64,
        speed: f64,
        wall: Option<f64>,
        lag: f64,
    }

    impl Actuator {
        fn follow(&mut self, target: u16, dt: f64) {
            let want = f64::from(target) - self.lag * (f64::from(target) - self.at).signum();
            let step = (want - self.at).clamp(-self.speed * dt, self.speed * dt);
            self.at += step;
            if let Some(w) = self.wall {
                self.at = self.at.min(w);
            }
        }
        fn position(&self) -> u16 {
            self.at.round() as u16
        }
    }

    /// Runs it to the end (at most 5 minutes); returns the outcome and every target asked.
    fn run(sweep: &mut Sweep, act: &mut Actuator, t0: Instant) -> (Outcome, Vec<u16>) {
        let mut asked = Vec::new();
        let mut now = t0;
        for _ in 0..15_000 {
            now += TICK;
            match sweep.step(now, Some(act.position()), FirmwareState::Jog) {
                Some(target) => {
                    asked.push(target);
                    act.follow(target, TICK.as_secs_f64());
                }
                None => break,
            }
        }
        (sweep.outcome().cloned().expect("it ended"), asked)
    }

    #[test]
    fn a_free_actuator_passes_lock_to_lock_inside_the_limits() {
        let l = limits();
        let t = Instant::now();
        let mut sweep = Sweep::start(t, Some(1500), &l, SweepSettings::default()).expect("start");
        let mut act = Actuator {
            at: 1500.0,
            speed: 300.0,
            wall: None,
            lag: 5.0,
        };
        let (outcome, asked) = run(&mut sweep, &mut act, t);
        assert!(matches!(outcome, Outcome::Passed { .. }), "{outcome}");
        assert!(asked.iter().all(|&a| (l.lo..=l.hi).contains(&a)));
        assert_eq!(asked.iter().max(), Some(&l.left_limit()));
        assert_eq!(asked.iter().min(), Some(&l.right_limit()));
        if let Outcome::Passed { max_error } = outcome {
            assert_eq!(max_error.len(), 5);
        }
    }

    #[test]
    fn a_bind_stops_it_and_says_where() {
        let l = limits();
        let t = Instant::now();
        let mut sweep = Sweep::start(t, Some(1985), &l, SweepSettings::default()).expect("start");
        // Binds at 2800, on the way to the left limit (3346).
        let mut act = Actuator {
            at: 1985.0,
            speed: 300.0,
            wall: Some(2800.0),
            lag: 0.0,
        };
        let (outcome, asked) = run(&mut sweep, &mut act, t);
        match outcome {
            Outcome::Stopped {
                leg,
                why,
                position,
                target,
            } => {
                assert_eq!(leg, Leg::ToLeft);
                assert!(
                    matches!(why, Stop::NoMovement { .. } | Stop::Resistance { .. }),
                    "{why}"
                );
                assert_eq!(position, 2800);
                // It stopped soon: the target never got far past the bind.
                assert!(target < 2800 + 150 + 10, "{target}");
            }
            other => panic!("{other}"),
        }
        assert!(asked.iter().all(|&a| a <= l.hi));
    }

    #[test]
    fn a_slow_actuator_is_resistance() {
        let l = limits();
        let t = Instant::now();
        let mut sweep = Sweep::start(t, Some(1985), &l, SweepSettings::default()).expect("start");
        // Moves, but at only 40 counts a second: it falls behind the target.
        let mut act = Actuator {
            at: 1985.0,
            speed: 40.0,
            wall: None,
            lag: 0.0,
        };
        let (outcome, _) = run(&mut sweep, &mut act, t);
        assert!(
            matches!(
                outcome,
                Outcome::Stopped {
                    why: Stop::Resistance { .. },
                    ..
                }
            ),
            "{outcome}"
        );
    }

    #[test]
    fn the_firmware_refusing_or_the_position_lost_stops_it() {
        let l = limits();
        let t = Instant::now();
        let s = SweepSettings::default();
        let mut sweep = Sweep::start(t, Some(1985), &l, s).expect("start");
        let refused = FirmwareState::Refused(Refusal::NotBraked);
        assert_eq!(sweep.step(t + TICK, Some(1985), refused), None);
        assert!(matches!(
            sweep.outcome(),
            Some(Outcome::Stopped {
                why: Stop::Firmware(_),
                ..
            })
        ));

        // Still LIMP during the grace is fine; after it, it stops.
        let mut sweep = Sweep::start(t, Some(1985), &l, s).expect("start");
        assert!(sweep
            .step(t + TICK, Some(1985), FirmwareState::Limp)
            .is_some());
        assert_eq!(
            sweep.step(t + s.grace + TICK, Some(1985), FirmwareState::Limp),
            None
        );

        let mut sweep = Sweep::start(t, Some(1985), &l, s).expect("start");
        assert!(sweep.step(t + TICK, None, FirmwareState::Jog).is_some());
        assert_eq!(
            sweep.step(t + s.lost_after + 2 * TICK, None, FirmwareState::Jog),
            None
        );
        assert!(matches!(
            sweep.outcome(),
            Some(Outcome::Stopped {
                why: Stop::LostPosition,
                ..
            })
        ));
    }

    #[test]
    fn it_wont_start_outside_the_limits_and_a_person_can_stop_it() {
        let l = limits();
        let t = Instant::now();
        let s = SweepSettings::default();
        assert_eq!(
            Sweep::start(t, Some(3400), &l, s),
            Err(CantStart::Outside {
                position: 3400,
                lo: l.lo,
                hi: l.hi
            })
        );
        assert_eq!(Sweep::start(t, None, &l, s), Err(CantStart::NoPosition));
        let mut sweep = Sweep::start(t, Some(2000), &l, s).expect("start");
        sweep.abort("a person pressed HOLD");
        assert_eq!(sweep.step(t + TICK, Some(2000), FirmwareState::Jog), None);
        let text = sweep.outcome().expect("ended").to_string();
        assert!(text.contains("a person pressed HOLD") && text.contains("Holding"));
    }
}
