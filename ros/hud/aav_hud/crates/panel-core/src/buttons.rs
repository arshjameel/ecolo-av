//! The buttons: what a press asks for, and what goes to twist_mux each frame
//! (docs/designs/2026-10-29_two-screens.md, "Screen 1").
//!
//! | Rule | Here |
//! |---|---|
//! | Speed +/− set the asked speed in steps (0.1 m/s), up to the gate's cap; never reverse | [`Buttons::press`] |
//! | Hold to drive: the car moves only while GO (Space, or the GO button) is held | [`Buttons::set_go`], [`Buttons::command`] |
//! | Let go: zeros for half a second, then nothing (twist_mux hands back to the next driver) | [`STOP_TAIL`] |
//! | ◀ ▶: 0.05 rad a press, sent as a turn rate ω = v · tan δ / L | [`Command::from_steering`] |
//! | STOP: the asked speed 0 at once | [`Press::Stop`] |
//! | CENTRE: the wheels straight at a creep, then speed 0 (they don't turn at a standstill) | [`Press::Centre`] |
//! | An e-stop, a takeover (twist_mux passing someone else's command for 0.3 s), a source change, the speed unseen for 4 s on stands: everything zeroed, and GO must be let go and pressed again | [`Buttons::zero`] |
//! | Speed buttons off: nothing is sent at all | [`crate::gate`] |
//!
//! Publishing only while the window draws is the ROS side's job (hud-core's
//! `Heartbeat`): this only says what to send.

use std::{
    collections::VecDeque,
    fmt,
    time::{Duration, Instant},
};

use crate::{
    gate::Gate,
    rules::{self, Sample, Unseen},
};

/// After GO is let go, zeros go out this long, so the car stops at once.
pub const STOP_TAIL: Duration = Duration::from_millis(500);
/// twist_mux passing someone else's command this long is a takeover (as the tests').
pub const TAKEOVER_AFTER: Duration = Duration::from_millis(300);
/// Our commands remembered, to tell twist_mux's output from someone else's.
pub const OURS_FOR: Duration = Duration::from_millis(500);
/// CENTRE gives up creeping after this long, and stops.
pub const CENTRE_FOR: Duration = Duration::from_secs(3);
/// CENTRE is done when the seen steering is this close to straight, radians.
pub const CENTRED: f64 = 0.02;
/// Seconds of (command, measured) kept for the unseen rule.
const HISTORY_FOR: f64 = rules::UNSEEN_TIME + 1.0;

/// A button (or its key).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Press {
    /// Speed + (key +, or Up).
    SpeedUp,
    /// Speed − (key −, or Down).
    SpeedDown,
    /// ◀ (key Left).
    Left,
    /// ▶ (key Right).
    Right,
    /// CENTRE (key C).
    Centre,
    /// STOP (key S): the asked speed 0.
    Stop,
}

/// Why the buttons were last zeroed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Zeroed {
    Estop,
    /// Someone else drives: twist_mux passed another command.
    TakenOver,
    /// `/system/source` changed.
    SourceChanged,
    /// The speed buttons went off (the gate).
    GateOff,
    /// On stands: told to drive, the speed never seen.
    Unseen(Unseen),
}

impl fmt::Display for Zeroed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Estop => f.write_str("the e-stop"),
            Self::TakenOver => f.write_str("someone else drove (twist_mux passed another command)"),
            Self::SourceChanged => f.write_str("/system/source changed"),
            Self::GateOff => f.write_str("the speed buttons went off"),
            Self::Unseen(u) => write!(f, "{u}"),
        }
    }
}

/// What the buttons need of the car and the settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Steps {
    /// m/s a press.
    pub speed: f64,
    /// rad a press.
    pub steer: f64,
    /// m/s while CENTRE straightens the wheels.
    pub creep: f64,
    /// rad, either way: vehicle.yaml's `max_steering_angle`.
    pub max_steer: f64,
    /// m: vehicle.yaml's `wheelbase`.
    pub wheelbase: f64,
}

/// One command for twist_mux: forward speed and turn rate, base_link (REP-103).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// m/s, forward.
    pub speed: f64,
    /// rad/s, + = left.
    pub yaw_rate: f64,
}

impl Command {
    pub const ZERO: Self = Self {
        speed: 0.0,
        yaw_rate: 0.0,
    };

    /// `speed` with the front wheels at `steer` radians: the bicycle model,
    /// ω = v · tan δ / L. ego turns it back into the same angle.
    pub fn from_steering(speed: f64, steer: f64, wheelbase: f64) -> Self {
        Self {
            speed,
            yaw_rate: speed * steer.tan() / wheelbase,
        }
    }

    fn pair(self) -> (f64, f64) {
        (self.speed, self.yaw_rate)
    }
}

/// What the world says this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct World<'a> {
    pub gate: &'a Gate,
    /// `/safety/estop` true, or `/vehicle/status` says the e-stop is on.
    pub estop: bool,
    /// The steering the car reports, rad (`/vehicle/steering` measured).
    pub seen_steer: Option<f64>,
    /// The measured speed, m/s (`/vehicle/status`).
    pub seen_speed: Option<f64>,
}

/// The buttons' state.
#[derive(Debug, Clone)]
pub struct Buttons {
    steps: Steps,
    asked_speed: f64,
    asked_steer: f64,
    go: bool,
    /// After a zero, GO must be let go before it drives again.
    needs_release: bool,
    centring: Option<Instant>,
    stop_until: Option<Instant>,
    ours: VecDeque<(Instant, Command)>,
    foreign_since: Option<Instant>,
    history: VecDeque<(Instant, Sample)>,
    last_zero: Option<Zeroed>,
}

impl Buttons {
    pub fn new(steps: Steps) -> Self {
        Self {
            steps,
            asked_speed: 0.0,
            asked_steer: 0.0,
            go: false,
            needs_release: false,
            centring: None,
            stop_until: None,
            ours: VecDeque::new(),
            foreign_since: None,
            history: VecDeque::new(),
            last_zero: None,
        }
    }

    /// m/s the buttons ask for while GO is held.
    pub fn asked_speed(&self) -> f64 {
        self.asked_speed
    }

    /// rad the buttons ask for, + = left.
    pub fn asked_steer(&self) -> f64 {
        self.asked_steer
    }

    /// GO held, and not waiting for it to be let go after a zero.
    pub fn is_driving(&self) -> bool {
        self.go && !self.needs_release
    }

    /// GO is held but must be let go first (after a zero).
    pub fn needs_release(&self) -> bool {
        self.go && self.needs_release
    }

    /// CENTRE is straightening the wheels.
    pub fn is_centring(&self) -> bool {
        self.centring.is_some()
    }

    /// Why the buttons were last zeroed, if they were.
    pub fn last_zero(&self) -> Option<&Zeroed> {
        self.last_zero.as_ref()
    }

    /// A press at `now`. With the speed buttons off, only STOP and CENTRE do anything.
    pub fn press(&mut self, p: Press, now: Instant, gate: &Gate) {
        let cap = match gate {
            Gate::On { cap, .. } => *cap,
            Gate::Off(_) => {
                if matches!(p, Press::Stop | Press::Centre) {
                    self.asked_speed = 0.0;
                    self.asked_steer = 0.0;
                    self.centring = None;
                }
                return;
            }
        };
        let (step, turn, max) = (self.steps.speed, self.steps.steer, self.steps.max_steer);
        match p {
            Press::SpeedUp => self.asked_speed = snap(self.asked_speed + step, step).min(cap),
            Press::SpeedDown => self.asked_speed = snap(self.asked_speed - step, step).max(0.0),
            Press::Left => self.asked_steer = snap(self.asked_steer + turn, turn).min(max),
            Press::Right => self.asked_steer = snap(self.asked_steer - turn, turn).max(-max),
            Press::Stop => {
                self.asked_speed = 0.0;
                self.centring = None;
            }
            Press::Centre => {
                self.asked_steer = 0.0;
                if self.is_driving() {
                    self.centring = Some(now);
                } else {
                    self.asked_speed = 0.0;
                }
            }
        }
        if p != Press::Centre {
            // Any other press takes over from CENTRE.
            self.centring = None;
        }
        self.last_zero = None;
    }

    /// GO is held (Space, or the button) or not, at `now`.
    pub fn set_go(&mut self, held: bool, now: Instant) {
        if self.go && !held {
            if !self.needs_release {
                self.stop_until = Some(now + STOP_TAIL);
            }
            self.needs_release = false;
            self.centring = None;
        }
        self.go = held;
    }

    /// Everything to 0; if GO is held, it must be let go and pressed again.
    pub fn zero(&mut self, why: Zeroed) {
        self.asked_speed = 0.0;
        self.asked_steer = 0.0;
        self.centring = None;
        self.stop_until = None;
        self.needs_release = self.go;
        self.foreign_since = None;
        self.history.clear();
        self.last_zero = Some(why);
    }

    /// twist_mux's output (`/control/cmd_vel`) arrived at `now`. While we drive, someone
    /// else's command for [`TAKEOVER_AFTER`] zeroes us.
    pub fn observe_mux(&mut self, theirs: Command, now: Instant) {
        self.forget(now);
        let sending = self
            .ours
            .back()
            .is_some_and(|(at, _)| now.saturating_duration_since(*at) < TAKEOVER_AFTER);
        if !sending {
            self.foreign_since = None;
            return;
        }
        if rules::taken_over(self.ours.iter().map(|(_, c)| c.pair()), theirs.pair()) {
            let since = *self.foreign_since.get_or_insert(now);
            if now.saturating_duration_since(since) >= TAKEOVER_AFTER {
                self.zero(Zeroed::TakenOver);
            }
        } else {
            self.foreign_since = None;
        }
    }

    /// What to send now, if anything. Call once a frame.
    pub fn command(&mut self, now: Instant, world: &World<'_>) -> Option<Command> {
        self.forget(now);
        if world.estop {
            if self.last_zero != Some(Zeroed::Estop) || !self.needs_release {
                self.zero(Zeroed::Estop);
            }
            return None;
        }
        let (cap, watch_unseen) = match world.gate {
            Gate::On { cap, watch_unseen } => (*cap, *watch_unseen),
            Gate::Off(_) => {
                if self.asked_speed != 0.0 || self.is_driving() || self.stop_until.is_some() {
                    self.zero(Zeroed::GateOff);
                }
                return None;
            }
        };
        self.asked_speed = self.asked_speed.min(cap);
        let command = if self.is_driving() {
            let centring = self.centring.filter(|&at| {
                let straight = world.seen_steer.is_some_and(|s| s.abs() < CENTRED);
                !straight && now.saturating_duration_since(at) < CENTRE_FOR
            });
            if self.centring.is_some() && centring.is_none() {
                // Straight (or gave up): CENTRE's second half, speed 0.
                self.asked_speed = 0.0;
            }
            self.centring = centring;
            if centring.is_some() {
                Command::from_steering(self.steps.creep.min(cap), 0.0, self.steps.wheelbase)
            } else {
                Command::from_steering(self.asked_speed, self.asked_steer, self.steps.wheelbase)
            }
        } else if self.stop_until.is_some_and(|until| now < until) {
            Command::ZERO
        } else {
            self.stop_until = None;
            self.history.clear();
            return None;
        };
        if watch_unseen {
            if let Some(why) = self.unseen(now, command.speed, world.seen_speed) {
                self.zero(Zeroed::Unseen(why));
                return None;
            }
        }
        self.ours.push_back((now, command));
        Some(command)
    }

    /// The unseen rule, over the last few seconds of commands and measured speeds.
    fn unseen(&mut self, now: Instant, commanded: f64, measured: Option<f64>) -> Option<Unseen> {
        let first = self.history.front().map_or(now, |(at, _)| *at);
        let t = now.saturating_duration_since(first).as_secs_f64();
        self.history.push_back((
            now,
            Sample {
                t,
                commanded,
                // No reading counts as none seen.
                measured: measured.unwrap_or(0.0),
            },
        ));
        while self
            .history
            .front()
            .is_some_and(|(at, _)| now.saturating_duration_since(*at).as_secs_f64() > HISTORY_FOR)
        {
            self.history.pop_front();
        }
        // Times from the oldest kept, as `unseen` wants them.
        let base = self.history.front().map_or(0.0, |(_, s)| s.t);
        let samples: Vec<Sample> = self
            .history
            .iter()
            .map(|(_, s)| Sample {
                t: s.t - base,
                ..*s
            })
            .collect();
        rules::unseen(&samples)
    }

    fn forget(&mut self, now: Instant) {
        while self
            .ours
            .front()
            .is_some_and(|(at, _)| now.saturating_duration_since(*at) > OURS_FOR)
        {
            self.ours.pop_front();
        }
    }
}

/// `v` to the nearest whole number of `step`s, so ten presses of 0.1 make 1.0, not
/// 0.9999999.
fn snap(v: f64, step: f64) -> f64 {
    (v / step).round() * step
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gate::Off;

    const STEPS: Steps = Steps {
        speed: 0.1,
        steer: 0.05,
        creep: 0.1,
        max_steer: 0.5,
        wheelbase: 1.57,
    };
    const SIM: Gate = Gate::On {
        cap: 1.0,
        watch_unseen: false,
    };

    fn world(gate: &Gate) -> World<'_> {
        World {
            gate,
            estop: false,
            seen_steer: Some(0.0),
            seen_speed: Some(0.0),
        }
    }

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn a_press_is_one_step_and_the_cap_holds() {
        let t = Instant::now();
        let mut b = Buttons::new(STEPS);
        b.press(Press::SpeedUp, t, &SIM);
        assert_eq!(b.asked_speed(), 0.1);
        for _ in 0..20 {
            b.press(Press::SpeedUp, t, &SIM);
        }
        assert_eq!(b.asked_speed(), 1.0);
        b.press(Press::SpeedDown, t, &SIM);
        assert!((b.asked_speed() - 0.9).abs() < 1e-12);
        for _ in 0..20 {
            b.press(Press::SpeedDown, t, &SIM);
        }
        assert_eq!(b.asked_speed(), 0.0, "never reverse");
        for _ in 0..20 {
            b.press(Press::Left, t, &SIM);
        }
        assert_eq!(b.asked_steer(), 0.5);
        // A lower cap (stands) clamps what was asked.
        let stands = Gate::On {
            cap: 0.3,
            watch_unseen: false,
        };
        for _ in 0..5 {
            b.press(Press::SpeedUp, t, &SIM);
        }
        b.set_go(true, t);
        let c = b.command(t, &world(&stands)).expect("driving");
        assert!((c.speed - 0.3).abs() < 1e-12);
    }

    #[test]
    fn it_drives_only_while_go_is_held_then_stops() {
        let t = Instant::now();
        let mut b = Buttons::new(STEPS);
        for _ in 0..3 {
            b.press(Press::SpeedUp, t, &SIM);
        }
        // Asked 0.3 m/s, GO not held: nothing sent.
        assert_eq!(b.command(t, &world(&SIM)), None);
        b.set_go(true, t);
        let c = b.command(t, &world(&SIM)).expect("held");
        assert!((c.speed - 0.3).abs() < 1e-12 && c.yaw_rate == 0.0);
        // Let go: zeros for half a second, then silence.
        b.set_go(false, t + ms(100));
        assert_eq!(b.command(t + ms(200), &world(&SIM)), Some(Command::ZERO));
        assert_eq!(b.command(t + ms(700), &world(&SIM)), None);
        // The asked speed stays for the next hold.
        assert!((b.asked_speed() - 0.3).abs() < 1e-12);
    }

    #[test]
    fn steering_is_a_turn_rate_by_the_bicycle_model() {
        let t = Instant::now();
        let mut b = Buttons::new(STEPS);
        b.press(Press::SpeedUp, t, &SIM);
        b.press(Press::Left, t, &SIM);
        b.press(Press::Left, t, &SIM);
        b.set_go(true, t);
        let c = b.command(t, &world(&SIM)).expect("held");
        assert!((c.yaw_rate - 0.1 * 0.1_f64.tan() / 1.57).abs() < 1e-12);
        assert!(c.yaw_rate > 0.0, "left is +");
        // At a standstill there is no turn rate to send.
        assert_eq!(Command::from_steering(0.0, 0.3, 1.57).yaw_rate, 0.0);
    }

    #[test]
    fn centre_creeps_straight_then_stops() {
        let t = Instant::now();
        let mut b = Buttons::new(STEPS);
        for _ in 0..5 {
            b.press(Press::SpeedUp, t, &SIM);
        }
        b.press(Press::Right, t, &SIM);
        b.set_go(true, t);
        b.press(Press::Centre, t, &SIM);
        let turned = World {
            seen_steer: Some(-0.05),
            ..world(&SIM)
        };
        let c = b.command(t, &turned).expect("creeping");
        assert_eq!((c.speed, c.yaw_rate), (0.1, 0.0));
        assert!(b.is_centring());
        // Straight: speed 0.
        let c = b.command(t + ms(500), &world(&SIM)).expect("held");
        assert_eq!(c, Command::ZERO);
        assert_eq!((b.asked_speed(), b.asked_steer()), (0.0, 0.0));
        // CENTRE with GO let go: everything 0 at once, nothing sent.
        let mut b = Buttons::new(STEPS);
        b.press(Press::SpeedUp, t, &SIM);
        b.press(Press::Left, t, &SIM);
        b.press(Press::Centre, t, &SIM);
        assert_eq!((b.asked_speed(), b.asked_steer()), (0.0, 0.0));
        assert_eq!(b.command(t, &world(&SIM)), None);
    }

    #[test]
    fn centre_gives_up_after_three_seconds() {
        let t = Instant::now();
        let mut b = Buttons::new(STEPS);
        b.press(Press::SpeedUp, t, &SIM);
        b.set_go(true, t);
        b.press(Press::Centre, t, &SIM);
        let stuck = World {
            seen_steer: Some(0.2),
            ..world(&SIM)
        };
        assert_eq!(b.command(t + ms(2900), &stuck).map(|c| c.speed), Some(0.1));
        assert_eq!(b.command(t + ms(3100), &stuck), Some(Command::ZERO));
    }

    /// Asked 0.3 m/s, GO held, sending.
    fn driving(t: Instant) -> Buttons {
        let mut b = Buttons::new(STEPS);
        for _ in 0..3 {
            b.press(Press::SpeedUp, t, &SIM);
        }
        b.set_go(true, t);
        assert!(b.command(t, &world(&SIM)).is_some());
        b
    }

    /// After a zero: nothing until GO is let go and pressed again, and a fresh Speed +.
    fn needs_fresh_presses(b: &mut Buttons, t: Instant) {
        assert_eq!((b.asked_speed(), b.asked_steer()), (0.0, 0.0));
        assert_eq!(b.command(t, &world(&SIM)), None, "GO still held");
        assert!(b.needs_release());
        b.set_go(false, t);
        assert_eq!(
            b.command(t, &world(&SIM)),
            None,
            "no stop tail after a zero"
        );
        b.set_go(true, t);
        assert_eq!(
            b.command(t, &world(&SIM)),
            Some(Command::ZERO),
            "speed 0 until +"
        );
    }

    #[test]
    fn an_estop_zeroes_everything() {
        let t = Instant::now();
        let mut b = driving(t);
        let stopped = World {
            estop: true,
            ..world(&SIM)
        };
        assert_eq!(b.command(t + ms(10), &stopped), None);
        assert_eq!(b.last_zero(), Some(&Zeroed::Estop));
        // Still on: still nothing, even after a release and a press.
        b.set_go(false, t + ms(20));
        b.set_go(true, t + ms(30));
        b.press(Press::SpeedUp, t + ms(30), &SIM);
        assert_eq!(b.command(t + ms(40), &stopped), None);
        // Cleared (by a person, elsewhere): fresh presses again.
        needs_fresh_presses(&mut b, t + ms(50));
    }

    #[test]
    fn a_takeover_zeroes_everything() {
        let t = Instant::now();
        let mut b = driving(t);
        let ours = Command::from_steering(0.3, 0.0, STEPS.wheelbase);
        // twist_mux passes ours: fine.
        b.observe_mux(ours, t + ms(5));
        // The joystick's deadman: 0, 0 from twist_mux while we still send 0.3.
        for i in 1..=4 {
            let at = t + ms(100 * i);
            assert!(b.command(at, &world(&SIM)).is_some());
            b.observe_mux(Command::ZERO, at + ms(5));
        }
        assert_eq!(b.last_zero(), Some(&Zeroed::TakenOver));
        // The joystick let go: the old speed doesn't come back.
        needs_fresh_presses(&mut b, t + ms(600));
    }

    #[test]
    fn a_takeover_at_the_real_rates() {
        // Frames at 30 Hz, the keyboard's zeros through twist_mux at 10 Hz.
        let t = Instant::now();
        let mut b = driving(t);
        let mut zeroed_at = None;
        for frame in 1..60u64 {
            let at = t + ms(33 * frame);
            if frame % 3 == 0 {
                b.observe_mux(Command::ZERO, at);
            }
            if b.last_zero() == Some(&Zeroed::TakenOver) {
                zeroed_at = Some(frame);
                break;
            }
            b.command(at, &world(&SIM));
        }
        assert!(zeroed_at.is_some_and(|f| f <= 15), "{zeroed_at:?}");
    }

    #[test]
    fn a_zero_with_go_let_go_needs_no_release() {
        let t = Instant::now();
        let mut b = Buttons::new(STEPS);
        b.zero(Zeroed::SourceChanged);
        // Pressed afterwards, GO and Speed + drive at once.
        b.press(Press::SpeedUp, t, &SIM);
        b.set_go(true, t);
        assert!(!b.needs_release());
        let c = b.command(t, &world(&SIM)).expect("driving");
        assert!((c.speed - 0.1).abs() < 1e-12);
    }

    #[test]
    fn a_short_mismatch_is_not_a_takeover() {
        let t = Instant::now();
        let mut b = driving(t);
        b.observe_mux(Command::ZERO, t + ms(10));
        b.observe_mux(Command::ZERO, t + ms(200));
        b.observe_mux(Command::from_steering(0.3, 0.0, 1.57), t + ms(250));
        b.observe_mux(Command::ZERO, t + ms(260));
        assert!(b.command(t + ms(270), &world(&SIM)).is_some());
        assert_eq!(b.last_zero(), None);
    }

    #[test]
    fn a_source_change_and_the_gate_going_off_zero_everything() {
        let t = Instant::now();
        let mut b = driving(t);
        b.zero(Zeroed::SourceChanged);
        needs_fresh_presses(&mut b, t);

        let mut b = driving(t);
        let off = Gate::Off(Off::Unannounced);
        assert_eq!(b.command(t, &world(&off)), None);
        assert_eq!(b.last_zero(), Some(&Zeroed::GateOff));
        // Off: speed presses do nothing.
        b.press(Press::SpeedUp, t, &off);
        assert_eq!(b.asked_speed(), 0.0);
        needs_fresh_presses(&mut b, t);
    }

    #[test]
    fn on_stands_an_unseen_speed_stops_after_four_seconds() {
        let stands = Gate::On {
            cap: 0.3,
            watch_unseen: true,
        };
        let t = Instant::now();
        let mut b = Buttons::new(STEPS);
        b.press(Press::SpeedUp, t, &stands);
        b.set_go(true, t);
        let still = World {
            seen_speed: Some(0.0),
            ..world(&stands)
        };
        let mut stopped_at = None;
        for i in 0..60 {
            let at = t + ms(100 * i);
            if b.command(at, &still).is_none() {
                stopped_at = Some(i);
                break;
            }
        }
        assert_eq!(stopped_at, Some(40), "4.0 s, as safety.py's unseen");
        assert!(matches!(b.last_zero(), Some(Zeroed::Unseen(_))));
        // Moving: never stopped.
        let mut b = Buttons::new(STEPS);
        b.press(Press::SpeedUp, t, &stands);
        b.set_go(true, t);
        let moving = World {
            seen_speed: Some(0.1),
            ..world(&stands)
        };
        assert!((0..60).all(|i| b.command(t + ms(100 * i), &moving).is_some()));
    }
}
