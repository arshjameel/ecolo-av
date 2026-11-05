//! Finding the steering actuator's ends and centre (version 3 frames): its rules.
//!
//! A person on the hardware panel asks for one of two modes, through the Orin, in every
//! version 3 frame ([`CalRequest`]):
//!
//! | Mode | The steering actuator |
//! |---|---|
//! | [`CalRequest::Limp`] | its motor off (the Jrk's "stop motor"): a person turns the wheels by hand, and the uplink reports where they are |
//! | [`CalRequest::Jog`] | goes to a target in counts, slowly ([`CAL_RATE`] counts a second), never outside the software limits |
//!
//! [`SteerCal`] decides, each tick, what the steering actuator gets ([`SteeringDrive`]).
//! Its rules:
//!
//! | Rule | Why |
//! |---|---|
//! | only while the frame asks for no speed and a brake, and the spine's throttle is 0 and its brake on | the car must stand still, braked |
//! | not with an e-stop, nor while the RC or a test drives | they have the steering |
//! | not without a position from the steering actuator (analog feedback) | limp and jog need it, and so does leaving |
//! | a jog target is clamped to the software limits, and moves at [`CAL_RATE`] | the steering actuator's own (Jrk) limits are the last line, not the first |
//! | leaving holds where the wheels are (target = position), then moves back to the spine's target at [`CAL_RATE`] | the wheels never snap |
//! | no fresh frame asking for it for [`CAL_TIMEOUT_MS`]: leave, and hold where the wheels are until fresh frames come again | the Orin's heartbeat is gone |
//!
//! The link keeps the throttle at 0 for any frame that asks for a mode, refused or not;
//! the board keeps the DAC at 0 whenever this isn't [`SteeringDrive::Spine`].

/// The highest position or target a Jrk G2 takes: 12 bits.
pub const COUNTS_MAX: u16 = 4095;
/// How fast a jog target, or the way back, moves: counts a second (the whole software
/// range, 400 to 3500, in about 10 s).
pub const CAL_RATE: f32 = 300.0;
/// No fresh frame asking for a mode for this long, and the mode ends: the spine's own
/// command timeout.
pub const CAL_TIMEOUT_MS: u32 = 100;
/// The longest step the rate is applied over, so a late tick can't jump.
const MAX_STEP_MS: u32 = 100;

/// What a version 3 frame asks of the steering actuator (bytes 16 to 18).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CalRequest {
    /// Steer as the frame's steering value says: normal driving.
    #[default]
    Off,
    /// The motor off, for turning the wheels by hand.
    Limp,
    /// Go to this target (counts, 0 to 4095), slowly; clamped to the software limits.
    Jog(u16),
}

/// Why a mode was refused (or ended).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    /// The frame asks for speed: its speed value isn't within the dead band.
    Moving,
    /// The frame asks for no brake, or the spine's brake is off or its throttle on.
    NotBraked,
    /// The steering actuator gives no position: its feedback isn't analog, or the read failed.
    NoFeedback,
    /// The hardware e-stop, or the frame's e-stop flag.
    Estop,
    /// The RC (direct) or a test mode drives.
    OtherDriver,
}

/// What the steering calibration is doing, for the uplink (byte 38).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CalState {
    /// Normal driving.
    #[default]
    Off,
    /// The motor is off.
    Limp,
    /// Moving to, or at, a jog target.
    Jog,
    /// A mode ended without fresh frames: holding where the wheels were.
    Hold,
    /// A mode ended: moving back to the spine's target, slowly.
    Return,
    /// A mode was asked for and refused; the steering does what it did before.
    Refused(Refusal),
}

impl CalState {
    /// The state as the uplink's byte 38.
    pub fn to_byte(self) -> u8 {
        match self {
            CalState::Off => 0,
            CalState::Limp => 1,
            CalState::Jog => 2,
            CalState::Hold => 3,
            CalState::Return => 4,
            CalState::Refused(Refusal::Moving) => 5,
            CalState::Refused(Refusal::NotBraked) => 6,
            CalState::Refused(Refusal::NoFeedback) => 7,
            CalState::Refused(Refusal::Estop) => 8,
            CalState::Refused(Refusal::OtherDriver) => 9,
        }
    }

    /// The state in byte 38, if it is a known one.
    pub fn from_byte(byte: u8) -> Option<CalState> {
        Some(match byte {
            0 => CalState::Off,
            1 => CalState::Limp,
            2 => CalState::Jog,
            3 => CalState::Hold,
            4 => CalState::Return,
            5 => CalState::Refused(Refusal::Moving),
            6 => CalState::Refused(Refusal::NotBraked),
            7 => CalState::Refused(Refusal::NoFeedback),
            8 => CalState::Refused(Refusal::Estop),
            9 => CalState::Refused(Refusal::OtherDriver),
            _ => return None,
        })
    }
}

/// What the steering actuator gets this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SteeringDrive {
    /// The spine's steering target, as always.
    Spine,
    /// Stop its motor (the Jrk's "stop motor" command): it goes limp.
    MotorOff,
    /// This target, in counts, already inside the software limits.
    Target(u16),
}

/// One fresh frame's request (a version 3 frame whose counter changed), and whether that
/// frame stands the car still: no speed, and a brake.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ask {
    pub request: CalRequest,
    /// The frame's speed value is within the dead band (|speed| < 0.03).
    pub speed_zero: bool,
    /// The frame asks for the brake (accel below -0.05).
    pub braked: bool,
}

/// What the rest of the spine says, this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Situation {
    pub now_ms: u32,
    /// The steering actuator's position (counts), if it has one this tick.
    pub feedback: Option<u16>,
    /// The spine's steering target, in counts.
    pub spine_target: u16,
    /// The spine's outputs: throttle 0 and the brake on.
    pub stopped: bool,
    /// The hardware e-stop, or the frame's e-stop flag.
    pub estop: bool,
    /// The RC (direct) or a test mode drives.
    pub other_driver: bool,
}

/// Where the steering is, inside the mode.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Motion {
    Normal,
    /// The motor off; the last position seen.
    Limp(Option<u16>),
    Jog(f32),
    Hold(f32),
    Return(f32),
}

/// The steering calibration's rules, one tick at a time (the module's table).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteerCal {
    min: u16,
    max: u16,
    motion: Motion,
    /// The latest fresh request, and when it came.
    last: Option<(Ask, u32)>,
    refusal: Option<Refusal>,
    last_ms: Option<u32>,
}

impl SteerCal {
    /// The rules, with the software limits (counts): a target never leaves them.
    pub const fn new(min: u16, max: u16) -> Self {
        SteerCal {
            min,
            max,
            motion: Motion::Normal,
            last: None,
            refusal: None,
            last_ms: None,
        }
    }

    /// A fresh frame's request (its counter changed), at `now_ms`.
    pub fn take(&mut self, ask: Ask, now_ms: u32) {
        self.last = Some((ask, now_ms));
    }

    /// This tick's decision: what the steering actuator gets.
    pub fn step(&mut self, s: &Situation) -> SteeringDrive {
        let dt_ms = self
            .last_ms
            .map_or(0, |t| s.now_ms.wrapping_sub(t).min(MAX_STEP_MS));
        self.last_ms = Some(s.now_ms);
        let step = CAL_RATE * dt_ms as f32 / 1000.0;

        let heard = self
            .last
            .filter(|&(_, at)| s.now_ms.wrapping_sub(at) <= CAL_TIMEOUT_MS)
            .map(|(ask, _)| ask);
        self.refusal = None;
        match heard.filter(|ask| ask.request != CalRequest::Off) {
            Some(ask) => match refusal(&ask, s) {
                Some(why) => {
                    self.refusal = Some(why);
                    self.leave(s, true);
                }
                None => self.enter(ask.request, s, step),
            },
            None => self.leave(s, heard.is_some()),
        }
        self.drive(s, step)
    }

    /// What it is doing, for the uplink.
    pub fn state(&self) -> CalState {
        if let Some(why) = self.refusal {
            return CalState::Refused(why);
        }
        match self.motion {
            Motion::Normal => CalState::Off,
            Motion::Limp(_) => CalState::Limp,
            Motion::Jog(_) => CalState::Jog,
            Motion::Hold(_) => CalState::Hold,
            Motion::Return(_) => CalState::Return,
        }
    }

    fn clamp(&self, counts: f32) -> f32 {
        counts.clamp(f32::from(self.min), f32::from(self.max))
    }

    /// Start, or carry on with, a mode the rules allow. `s.feedback` is known here.
    fn enter(&mut self, request: CalRequest, s: &Situation, step: f32) {
        let feedback = s.feedback.map(f32::from);
        self.motion = match (request, self.motion) {
            (CalRequest::Limp, _) => Motion::Limp(s.feedback),
            (CalRequest::Jog(target), from) => {
                let start = match from {
                    Motion::Jog(at) | Motion::Hold(at) | Motion::Return(at) => at,
                    Motion::Limp(_) | Motion::Normal => self.clamp(feedback.unwrap_or(0.0)),
                };
                Motion::Jog(slew(start, self.clamp(f32::from(target)), step))
            }
            (CalRequest::Off, motion) => motion,
        };
    }

    /// End a mode, if one runs: hold where the wheels are; with fresh frames (`heard`),
    /// move back from there.
    fn leave(&mut self, s: &Situation, heard: bool) {
        let at = match self.motion {
            Motion::Normal => return,
            Motion::Limp(seen) => match s.feedback.or(seen) {
                Some(counts) => self.clamp(f32::from(counts)),
                // Never a position: nothing to hold.
                None => {
                    self.motion = Motion::Normal;
                    return;
                }
            },
            Motion::Jog(at) | Motion::Hold(at) | Motion::Return(at) => at,
        };
        self.motion = if heard {
            Motion::Return(at)
        } else {
            Motion::Hold(at)
        };
    }

    fn drive(&mut self, s: &Situation, step: f32) -> SteeringDrive {
        match self.motion {
            Motion::Normal => SteeringDrive::Spine,
            Motion::Limp(_) => SteeringDrive::MotorOff,
            Motion::Jog(at) | Motion::Hold(at) => SteeringDrive::Target(round(at)),
            Motion::Return(at) => {
                let goal = self.clamp(f32::from(s.spine_target));
                let at = slew(at, goal, step);
                if at == goal {
                    self.motion = Motion::Normal;
                } else {
                    self.motion = Motion::Return(at);
                }
                SteeringDrive::Target(round(at))
            }
        }
    }
}

/// Why the rules turn this request down now, if they do.
fn refusal(ask: &Ask, s: &Situation) -> Option<Refusal> {
    if s.estop {
        Some(Refusal::Estop)
    } else if s.other_driver {
        Some(Refusal::OtherDriver)
    } else if !ask.speed_zero {
        Some(Refusal::Moving)
    } else if !ask.braked || !s.stopped {
        Some(Refusal::NotBraked)
    } else if s.feedback.is_none() {
        Some(Refusal::NoFeedback)
    } else {
        None
    }
}

/// From `at` toward `goal`, at most `step`.
fn slew(at: f32, goal: f32, step: f32) -> f32 {
    at + (goal - at).clamp(-step, step)
}

/// The nearest whole count; `x` is inside the limits here.
fn round(x: f32) -> u16 {
    (x + 0.5) as u16 // x >= 0, and `as` saturates
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    const STANDING: Ask = Ask {
        request: CalRequest::Limp,
        speed_zero: true,
        braked: true,
    };

    fn situation(now_ms: u32, feedback: Option<u16>) -> Situation {
        Situation {
            now_ms,
            feedback,
            spine_target: 1950,
            stopped: true,
            estop: false,
            other_driver: false,
        }
    }

    fn ask(request: CalRequest) -> Ask {
        Ask {
            request,
            ..STANDING
        }
    }

    /// `n` ticks 10 ms apart from `*now`, a fresh frame with `request` each; the drives.
    fn run(
        cal: &mut SteerCal,
        now: &mut u32,
        n: usize,
        request: Option<CalRequest>,
        s: Situation,
    ) -> Vec<SteeringDrive> {
        (0..n)
            .map(|_| {
                *now += 10;
                if let Some(r) = request {
                    cal.take(ask(r), *now);
                }
                cal.step(&Situation { now_ms: *now, ..s })
            })
            .collect()
    }

    #[test]
    fn off_leaves_the_steering_to_the_spine() {
        let mut cal = SteerCal::new(400, 3500);
        let mut now = 0;
        let drives = run(
            &mut cal,
            &mut now,
            5,
            Some(CalRequest::Off),
            situation(0, Some(2000)),
        );
        assert!(drives.iter().all(|&d| d == SteeringDrive::Spine));
        assert_eq!(cal.state(), CalState::Off);
    }

    #[test]
    fn limp_stops_the_motor_and_leaving_holds_the_position() {
        let mut cal = SteerCal::new(400, 3500);
        let mut now = 0;
        let s = situation(0, Some(2100));
        let drives = run(&mut cal, &mut now, 3, Some(CalRequest::Limp), s);
        assert_eq!(drives, [SteeringDrive::MotorOff; 3]);
        assert_eq!(cal.state(), CalState::Limp);
        // A person turns the wheels to 2900, then the panel says Off.
        let s = situation(0, Some(2900));
        run(&mut cal, &mut now, 1, Some(CalRequest::Limp), s);
        let first = run(&mut cal, &mut now, 1, Some(CalRequest::Off), s);
        // The first target is where the wheels are: no snap. Then 3 counts a tick back.
        assert_eq!(first, [SteeringDrive::Target(2897)]);
        assert_eq!(cal.state(), CalState::Return);
        let back = run(&mut cal, &mut now, 400, Some(CalRequest::Off), s);
        assert!(back.windows(2).all(|w| match (w[0], w[1]) {
            (SteeringDrive::Target(a), SteeringDrive::Target(b)) => a - b <= 3,
            _ => true,
        }));
        assert_eq!(back.last(), Some(&SteeringDrive::Spine));
        assert_eq!(cal.state(), CalState::Off);
    }

    #[test]
    fn jog_starts_where_the_wheels_are_and_moves_slowly() {
        let mut cal = SteerCal::new(400, 3500);
        let mut now = 0;
        let s = situation(0, Some(2000));
        let drives = run(&mut cal, &mut now, 10, Some(CalRequest::Jog(2100)), s);
        // The first tick only sets the clock: 2000. Then 3 counts each 10 ms.
        assert_eq!(drives[0], SteeringDrive::Target(2000));
        assert_eq!(drives[1], SteeringDrive::Target(2003));
        assert_eq!(drives[9], SteeringDrive::Target(2027));
        let drives = run(&mut cal, &mut now, 100, Some(CalRequest::Jog(2100)), s);
        assert_eq!(drives.last(), Some(&SteeringDrive::Target(2100)));
        assert_eq!(cal.state(), CalState::Jog);
    }

    #[test]
    fn jog_targets_stay_inside_the_software_limits() {
        let mut cal = SteerCal::new(400, 3500);
        let mut now = 0;
        let s = situation(0, Some(3450));
        let drives = run(&mut cal, &mut now, 100, Some(CalRequest::Jog(4095)), s);
        assert_eq!(drives.last(), Some(&SteeringDrive::Target(3500)));
        let drives = run(&mut cal, &mut now, 2000, Some(CalRequest::Jog(0)), s);
        assert_eq!(drives.last(), Some(&SteeringDrive::Target(400)));
        assert!(drives.iter().all(|d| match d {
            SteeringDrive::Target(t) => (400..=3500).contains(t),
            _ => false,
        }));
    }

    #[test]
    fn a_position_past_the_limits_is_held_at_the_limit() {
        // Turned by hand past the software limit, then Off: the hold starts at the limit.
        let mut cal = SteerCal::new(400, 3500);
        let mut now = 0;
        let s = situation(0, Some(3700));
        run(&mut cal, &mut now, 2, Some(CalRequest::Limp), s);
        let first = run(&mut cal, &mut now, 1, Some(CalRequest::Off), s);
        assert_eq!(first, [SteeringDrive::Target(3497)]);
    }

    #[test]
    fn refused_unless_standing_braked_and_with_a_position() {
        let cases = [
            (
                ask(CalRequest::Limp),
                situation(0, None),
                Refusal::NoFeedback,
            ),
            (
                Ask {
                    speed_zero: false,
                    ..STANDING
                },
                situation(0, Some(2000)),
                Refusal::Moving,
            ),
            (
                Ask {
                    braked: false,
                    ..STANDING
                },
                situation(0, Some(2000)),
                Refusal::NotBraked,
            ),
            (
                STANDING,
                Situation {
                    stopped: false,
                    ..situation(0, Some(2000))
                },
                Refusal::NotBraked,
            ),
            (
                STANDING,
                Situation {
                    estop: true,
                    ..situation(0, Some(2000))
                },
                Refusal::Estop,
            ),
            (
                STANDING,
                Situation {
                    other_driver: true,
                    ..situation(0, Some(2000))
                },
                Refusal::OtherDriver,
            ),
        ];
        for (a, s, why) in cases {
            let mut cal = SteerCal::new(400, 3500);
            cal.take(a, 10);
            let drive = cal.step(&Situation { now_ms: 10, ..s });
            assert_eq!(drive, SteeringDrive::Spine, "{why:?}");
            assert_eq!(cal.state(), CalState::Refused(why));
        }
    }

    #[test]
    fn a_rule_broken_mid_mode_ends_it_without_a_snap() {
        let mut cal = SteerCal::new(400, 3500);
        let mut now = 0;
        let s = situation(0, Some(2600));
        run(&mut cal, &mut now, 3, Some(CalRequest::Limp), s);
        // The e-stop: the motor comes back on, holding 2600, then moves back slowly.
        let estop = Situation { estop: true, ..s };
        let drives = run(&mut cal, &mut now, 2, Some(CalRequest::Limp), estop);
        assert_eq!(
            drives,
            [SteeringDrive::Target(2597), SteeringDrive::Target(2594)]
        );
        assert_eq!(cal.state(), CalState::Refused(Refusal::Estop));
    }

    #[test]
    fn losing_the_heartbeat_ends_the_mode_and_holds() {
        let mut cal = SteerCal::new(400, 3500);
        let mut now = 0;
        let s = situation(0, Some(2600));
        run(&mut cal, &mut now, 3, Some(CalRequest::Limp), s);
        // No fresh frame: still limp for 100 ms, then hold at the last position.
        let quiet = run(&mut cal, &mut now, 15, None, s);
        assert_eq!(quiet[..10], [SteeringDrive::MotorOff; 10]);
        assert_eq!(quiet[10..], [SteeringDrive::Target(2600); 5]);
        assert_eq!(cal.state(), CalState::Hold);
        // Still held, however long; fresh frames (Off) then move it back slowly.
        let quiet = run(&mut cal, &mut now, 100, None, s);
        assert!(quiet.iter().all(|&d| d == SteeringDrive::Target(2600)));
        let back = run(&mut cal, &mut now, 1, Some(CalRequest::Off), s);
        assert_eq!(back, [SteeringDrive::Target(2597)]);
        assert_eq!(cal.state(), CalState::Return);
    }

    #[test]
    fn jog_after_limp_starts_from_the_position() {
        let mut cal = SteerCal::new(400, 3500);
        let mut now = 0;
        run(
            &mut cal,
            &mut now,
            3,
            Some(CalRequest::Limp),
            situation(0, Some(1200)),
        );
        let drives = run(
            &mut cal,
            &mut now,
            1,
            Some(CalRequest::Jog(1300)),
            situation(0, Some(1200)),
        );
        assert_eq!(drives, [SteeringDrive::Target(1203)]);
    }

    #[test]
    fn the_state_byte_round_trips() {
        for byte in 0..=9 {
            assert_eq!(CalState::from_byte(byte).map(CalState::to_byte), Some(byte));
        }
        assert_eq!(CalState::from_byte(10), None);
    }
}
