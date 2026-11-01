//! Where the speed buttons may drive (docs/designs/2026-10-29_two-screens.md, "Where it
//! may drive"), from `/system/source`, `--stands` and the preflight's own rules.
//!
//! | Source | Speed buttons |
//! |---|---|
//! | SIM, BENCH | on, capped at `sim_cap` (1.0 m/s); `stands_cap` with `--stands` |
//! | REAL with `--stands` | the preflight's refusal ([`preflight_refusal`]): off until a driven-wheel speed exists (TODO 49b), the Rust ESP32 and `speed_loop.mode open`; then `stands_cap` (0.3 m/s), and [`crate::rules::unseen`]'s 4 s rule |
//! | REAL on the ground, nothing announced, REPLAY | off |

use std::fmt;

use crate::{
    rules::{preflight_refusal, Kind, Refusal},
    vehicle::SpeedWheels,
};

/// Whether the speed buttons may drive, and how fast.
#[derive(Debug, Clone, PartialEq)]
pub enum Gate {
    On {
        /// m/s: the most the buttons ask for.
        cap: f64,
        /// The real car on stands: stop when the speed can't be seen (4 s).
        watch_unseen: bool,
    },
    Off(Off),
}

/// Why the speed buttons are off.
#[derive(Debug, Clone, PartialEq)]
pub enum Off {
    /// `/system/source` says REPLAY: a recording plays.
    Replay,
    /// Nobody announced `/system/source`: treated as the real car.
    Unannounced,
    /// The real car with its wheels on the ground: not in step 1 (T5).
    OnTheGround,
    /// The real car on stands, refused by the preflight's rules.
    Refused(Refusal),
}

impl fmt::Display for Off {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Replay => f.write_str("/system/source says REPLAY: nothing to drive"),
            Self::Unannounced => f.write_str(
                "nobody announced /system/source: treated as the real car, so the speed \
                 buttons stay off",
            ),
            Self::OnTheGround => f.write_str(
                "the real car on the ground: the buttons drive only Gazebo, the bench, \
                 and the car on stands (--stands) once TODO 49b is answered",
            ),
            Self::Refused(r) => write!(f, "on stands: {r}"),
        }
    }
}

/// What the gate needs to know about the car.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Situation<'a> {
    pub kind: Kind,
    /// `--stands`: the wheels are off the ground.
    pub stands: bool,
    /// vehicle.yaml's `speed_wheels`.
    pub speed_wheels: Option<&'a SpeedWheels>,
    /// The legacy bridge's no-timeout lock (None: not heard).
    pub no_timeout_lock: Option<bool>,
    /// The legacy bridge's `speed_loop.mode`, if known (the panel doesn't read it yet).
    pub speed_loop_mode: Option<&'a str>,
}

/// The caps, from the settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Caps {
    /// m/s in Gazebo and on the bench.
    pub sim: f64,
    /// m/s on stands.
    pub stands: f64,
}

/// Where the speed buttons may drive now.
pub fn gate(s: &Situation<'_>, caps: Caps) -> Gate {
    let on = |cap: f64, watch_unseen| Gate::On {
        cap: if s.stands { cap.min(caps.stands) } else { cap },
        watch_unseen,
    };
    match s.kind {
        Kind::Sim | Kind::Bench => on(caps.sim, false),
        Kind::Replay => Gate::Off(Off::Replay),
        Kind::Unknown => Gate::Off(Off::Unannounced),
        Kind::Real if !s.stands => Gate::Off(Off::OnTheGround),
        Kind::Real => match preflight_refusal(
            s.kind,
            true,
            s.speed_wheels,
            s.no_timeout_lock,
            s.speed_loop_mode,
        ) {
            Ok(()) => on(caps.stands, true),
            Err(r) => Gate::Off(Off::Refused(r)),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CAPS: Caps = Caps {
        sim: 1.0,
        stands: 0.3,
    };

    fn situation(kind: Kind, stands: bool, wheels: Option<&SpeedWheels>) -> Situation<'_> {
        Situation {
            kind,
            stands,
            speed_wheels: wheels,
            no_timeout_lock: Some(false),
            speed_loop_mode: None,
        }
    }

    #[test]
    fn sim_and_bench_drive_at_one_metre_a_second() {
        for kind in [Kind::Sim, Kind::Bench] {
            let on = Gate::On {
                cap: 1.0,
                watch_unseen: false,
            };
            assert_eq!(gate(&situation(kind, false, None), CAPS), on);
            let stands = gate(&situation(kind, true, None), CAPS);
            assert!(matches!(stands, Gate::On { cap, .. } if cap == 0.3));
        }
    }

    #[test]
    fn the_real_car_is_off_on_the_ground_and_unannounced() {
        let rear = SpeedWheels::Rear;
        assert_eq!(
            gate(&situation(Kind::Real, false, Some(&rear)), CAPS),
            Gate::Off(Off::OnTheGround)
        );
        assert_eq!(
            gate(&situation(Kind::Unknown, true, Some(&rear)), CAPS),
            Gate::Off(Off::Unannounced)
        );
        assert_eq!(
            gate(&situation(Kind::Replay, false, None), CAPS),
            Gate::Off(Off::Replay)
        );
    }

    #[test]
    fn stands_copy_the_preflights_refusal() {
        // Today's car: the wheel sensors are on the front (TODO 49b).
        let front = SpeedWheels::Front;
        let off = gate(&situation(Kind::Real, true, Some(&front)), CAPS);
        let Gate::Off(why) = off else {
            panic!("front wheels must be refused: {off:?}")
        };
        assert!(why.to_string().contains("49b"), "{why}");
        // A rear-wheel speed and the Rust ESP32: 0.3 m/s, watching for an unseen speed.
        let rear = SpeedWheels::Rear;
        assert_eq!(
            gate(&situation(Kind::Real, true, Some(&rear)), CAPS),
            Gate::On {
                cap: 0.3,
                watch_unseen: true
            }
        );
        let mut locked = situation(Kind::Real, true, Some(&rear));
        locked.no_timeout_lock = Some(true);
        assert!(matches!(
            gate(&locked, CAPS),
            Gate::Off(Off::Refused(Refusal::NoTimeoutLock))
        ));
    }
}
