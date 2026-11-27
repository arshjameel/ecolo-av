//! The spine's decisions, one call per tick (about 100 Hz).
//!
//! Its states are the five regions in [`crate::fsm`]. Priorities, highest first:
//! 1. Any e-stop (hardware input or ESTOP_CMD): throttle off, full brake.
//! 2. The RC transmitter, while its mode switch is on manual (RC direct).
//! 3. The hardware test mode, once entered: one actuator at a time.
//! 4. The computer, while armed and sending fresh DRIVE_CMD frames.
//! 5. Otherwise idle: actuators idle.
//!
//! The transmitter's mode switch ([`RcSwitch`], TODO 48) has three positions: normal
//! (the computer drives), RC via ROS (the computer drives, from the sticks the board sends
//! up to it) and manual (RC direct). The RC signal must stay in RC via ROS and manual, and
//! in normal too on a board with a receiver ([`SpineConfig::rc_fitted`]). A lost RC signal
//! is a hard stop (fault bit `RC_SIGNAL_LOST`), and latches, like a timeout, so the
//! computer must stop, then drive again.
//!
//! A driver fault (steering, brake, throttle or encoder) stops the car in any mode.
//!
//! The stops, with the speed as one more input ([`SpeedReport`]):
//!
//! | What happens | The spine |
//! |---|---|
//! | An e-stop, a driver fault, the RC's right stick pulled down | a hard stop |
//! | A lost input: commands, RC or wheel speed (once a link has sent one) | a hard stop, latched |
//! | Past `max_speed + overspeed_margin` | throttle off and `overspeed_brake` until under `max_speed`; still over it after [`OVERSPEED_WINDOW_MS`](crate::fsm::OVERSPEED_WINDOW_MS): a hard stop, latched |
//!
//! The RC has no arming, so after any stop it holds the car braked ([`RcHold`]) until the
//! stop is gone, a second has passed, and the left stick has left the zone it was in.
//!
//! The computer's commands come in two kinds, with the same watchdog and arming:
//!
//! | Kind | From | The spine |
//! |---|---|---|
//! | [`DriveCmd`]: a speed and a steering angle | CAN (`firmware/can/aav.dbc`) | runs the speed loop |
//! | [`DirectCmd`]: throttle, brake and steering as they are | the 2025-26 UART link (`legacy-link`) | passes them on, inside its limits |
//!
//! With no wheel speed on the board ([`SpeedFeedback::None`]), the speed loop runs open
//! loop: throttle in a straight line from the speed asked for, from where the wheels start
//! to turn.
//!
//! The computer can't drive until it sends a frame with `armed = 0` and then
//! `armed = 1`: at power-up, after an e-stop, after a driver fault has cleared, and
//! after the command stream stopped while it was driving. A glitch or a reboot never
//! restarts the car by itself.

use crate::fsm::{
    Authority, Confidence, Latch, Lifecycle, Motion, Overspeed, Pedal, RcHold, RcOwner, RightStick,
    Safety, State, TestRequest, TestStage, RC_ACCEL, RC_BRAKE_MS, RC_STOPPED_MS, STANDSTILL_SPEED,
    TEST_MAX_SPEED, TEST_MAX_THROTTLE,
};
use crate::limits::slew;
use crate::protocol::{counter_step, DriveCmd, EstopCmd, Mode, VehicleFeedback, VehicleStatus};
use crate::speed_loop::{open_loop_throttle, SpeedLoop, SpeedLoopConfig};
use crate::watchdog::Watchdog;

/// Fault bits, same values as aav_interfaces/VehicleStatus FAULT_*.
pub mod faults {
    /// No valid DRIVE_CMD for 100 ms.
    pub const CMD_TIMEOUT: u16 = 1;
    /// A frame failed its CRC.
    pub const BAD_CHECKSUM: u16 = 2;
    /// A frame repeated or skipped its counter.
    pub const BAD_COUNTER: u16 = 4;
    /// The steering Jrk isn't answering, or is in error.
    pub const STEERING_DRIVER: u16 = 8;
    /// The brake Jrk isn't answering, or is in error.
    pub const BRAKE_DRIVER: u16 = 16;
    /// The throttle DAC isn't answering.
    pub const THROTTLE_DAC: u16 = 32;
    /// No wheel ticks while the car should be moving.
    pub const WHEEL_ENCODER: u16 = 64;
    /// CAN error passive, or bus off.
    pub const CAN_BUS: u16 = 128;
    /// The RC receiver is silent, or sends the transmitter's failsafe, while it counts.
    pub const RC_SIGNAL_LOST: u16 = 256;
    /// The measured speed is over the firmware's limit.
    pub const OVERSPEED: u16 = 512;

    /// Faults that stop the car and latch it, like a command timeout.
    pub const STOP: u16 = STEERING_DRIVER | BRAKE_DRIVER | THROTTLE_DAC | WHEEL_ENCODER;

    /// Every bit, with its name as `VehicleStatus.FAULT_*` has it, lowest first.
    pub const NAMES: [(u16, &str); 10] = [
        (CMD_TIMEOUT, "CMD_TIMEOUT"),
        (BAD_CHECKSUM, "BAD_CHECKSUM"),
        (BAD_COUNTER, "BAD_COUNTER"),
        (STEERING_DRIVER, "STEERING_DRIVER"),
        (BRAKE_DRIVER, "BRAKE_DRIVER"),
        (THROTTLE_DAC, "THROTTLE_DAC"),
        (WHEEL_ENCODER, "WHEEL_ENCODER"),
        (CAN_BUS, "CAN_BUS"),
        (RC_SIGNAL_LOST, "RC_SIGNAL_LOST"),
        (OVERSPEED, "OVERSPEED"),
    ];

    /// The names of the bits set in `bits`, lowest first. A bit with no name is skipped.
    pub fn names(bits: u16) -> impl Iterator<Item = &'static str> {
        NAMES
            .into_iter()
            .filter(move |(bit, _)| bits & bit != 0)
            .map(|(_, name)| name)
    }
}

// Three choices still open with the team (docs/firmware.md, "Still to decide"). Each is one
// constant, set to what the docs propose. `SpineConfig::default()` uses them, so the board
// program and the bench both follow a change here.

/// The steering on an e-stop, and on every other stop the spine makes by itself: a driver
/// fault, a lost RC signal, a command timeout, overspeed. Proposed: hold it where it is.
/// The 2025-26 firmware centred it (`jrkSafeStop()`).
pub const ESTOP_STEERING: EstopSteering = EstopSteering::Hold;

/// The brake while nobody drives, from boot on (0 to 1). Proposed: 0, so the brake lets go
/// once the e-stop input clears. The 2025-26 firmware kept it full on (1.0).
pub const IDLE_BRAKE: f32 = 0.0;

/// The fastest the car may go forward (m/s), whoever drives, the RC transmitter too. The
/// same as `ros/ego/aav_ego/config/ego.yaml`'s `max_speed` (Arsh, 6 Nov, T3.6): raise them
/// together, per the test plan. test_vehicle.py checks they match.
pub const MAX_SPEED: f32 = 1.0;

// Copies of ros/bringup/aav_description/config/vehicle.yaml, the one place the car's numbers live:
// the firmware can't read it. ros/bringup/aav_description/test/test_vehicle.py checks the copies.

/// The virtual centre wheel's steering limit, either way (rad): vehicle.yaml
/// `max_steering_angle`. TODO(Q1): measure on the car.
pub const MAX_STEERING_ANGLE: f32 = 0.5;

/// The fastest the car may go backwards (m/s), whoever drives, the RC transmitter too:
/// vehicle.yaml `max_reverse_speed`. Slow on purpose: nothing senses what is behind the car.
pub const MAX_REVERSE_SPEED: f32 = 0.5;

/// What the steering does when the spine stops the car by itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EstopSteering {
    /// Hold the wheels where they are.
    Hold,
    /// Bring the wheels back to straight, at the steering rate limit.
    Centre,
}

/// Where the speed loop gets the car's speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpeedFeedback {
    /// `Inputs::measured_speed`, from a wheel encoder: closed loop.
    Wheel,
    /// No speed on this board (today's ESP32 control board: the encoder talks to the
    /// Motor-Pi). Open loop: throttle from the start (`start_throttle` forward,
    /// `start_throttle_reverse` back) up to full throttle at `full_throttle_speed`
    /// ([`open_loop_throttle`]). The overspeed check works only while the link sends a
    /// speed ([`SpeedReport`]), and the stop brake is the hold brake.
    None {
        /// The speed (m/s) the car reaches at full throttle. TODO(Q4): measure.
        full_throttle_speed: f32,
        /// The least throttle (0..1) that turns the wheels forward: any forward speed
        /// asked for gets at least this. 0: throttle in proportion to the speed, the
        /// 2025-26 way.
        start_throttle: f32,
        /// The same in reverse, where the car may need more to start.
        start_throttle_reverse: f32,
    },
}

/// Everything the spine runs with: the limits, the timeouts, the team's choices.
#[derive(Debug, Clone, Copy)]
pub struct SpineConfig {
    /// No valid DRIVE_CMD for this long while driving = stop (ms).
    pub cmd_timeout_ms: u32,
    /// The firmware's own speed caps (m/s), applied after ego's.
    pub max_speed: f32,
    /// The reverse cap (m/s).
    pub max_reverse_speed: f32,
    /// Steering range (rad), from the car's measurements.
    pub max_steering_angle: f32,
    /// Acceleration and steering-rate caps, also used when a frame sends 0.
    pub max_accel: f32,
    /// Slowing down is capped separately, and harder than ego's 2.0 m/s^2, so the
    /// firmware never lags behind a stop that ego asked for.
    pub max_decel: f32,
    /// The steering-rate cap (rad/s).
    pub max_steer_rate: f32,
    /// Measured speed this far over the cap trips the overspeed rule (m/s). Above the
    /// speed loop's overshoot of about 15%, so normal driving at the cap doesn't trip it.
    pub overspeed_margin: f32,
    /// The brake while coming back under the cap (0..1). GUESS: tune in test 4.3.
    pub overspeed_brake: f32,
    /// No speed from the link for this long, once it has sent one = a hard stop (ms).
    pub speed_timeout_ms: u32,
    /// The board has an RC receiver (Arsh, 21 Nov): then the RC signal must stay in every
    /// position of the mode switch, normal too, and losing it is a hard stop. `false` (the
    /// default) for a spine with no receiver: the bench, the simulators, the Pico 2. The
    /// ESP32 on the car sets it.
    pub rc_fitted: bool,
    /// Brake for stops, for the RC's hard brake, and in manual mode (0..1).
    pub stop_brake: f32,
    /// The brake while nobody drives (0..1): [`IDLE_BRAKE`].
    pub idle_brake: f32,
    /// The steering on a stop: [`ESTOP_STEERING`].
    pub estop_steering: EstopSteering,
    /// The closed speed loop's law and gains.
    pub speed_loop: SpeedLoopConfig,
    /// The wheel speed, or none: [`SpeedFeedback`].
    pub speed_feedback: SpeedFeedback,
}

impl Default for SpineConfig {
    fn default() -> Self {
        SpineConfig {
            cmd_timeout_ms: 100,
            max_speed: MAX_SPEED,
            max_reverse_speed: MAX_REVERSE_SPEED,
            max_steering_angle: MAX_STEERING_ANGLE,
            max_accel: 1.5,
            max_decel: 2.5,
            max_steer_rate: 0.6,
            overspeed_margin: 0.25,
            overspeed_brake: 0.5,
            speed_timeout_ms: 300,
            rc_fitted: false,
            stop_brake: 1.0,
            idle_brake: IDLE_BRAKE,
            estop_steering: ESTOP_STEERING,
            speed_loop: SpeedLoopConfig::default(),
            speed_feedback: SpeedFeedback::Wheel,
        }
    }
}

impl SpineConfig {
    /// The most throttle (0..1) an open-loop command (the UART link's [`DirectCmd`]) may
    /// have, forward or in reverse, with no wheel speed on the board: the throttle that the
    /// open loop says gives `max_speed` (or `max_reverse_speed`). `None` with a wheel speed:
    /// then the throttle is cut while the car is at or over the limit instead.
    ///
    /// The spine caps each command with this; the board's settings line reports it.
    #[must_use]
    pub fn open_loop_throttle_cap(&self, reverse: bool) -> Option<f32> {
        let limit = if reverse {
            self.max_reverse_speed
        } else {
            self.max_speed
        };
        match self.speed_feedback {
            SpeedFeedback::None { .. } => Some(self.open_loop_cap(limit, reverse)),
            SpeedFeedback::Wheel => None,
        }
    }

    /// The open loop's throttle at `limit`, forward or in reverse, 0..1; 0 with no map
    /// from throttle to speed.
    fn open_loop_cap(&self, limit: f32, reverse: bool) -> f32 {
        match self.speed_feedback {
            SpeedFeedback::None {
                full_throttle_speed,
                start_throttle,
                start_throttle_reverse,
            } => {
                let start = if reverse {
                    start_throttle_reverse
                } else {
                    start_throttle
                };
                open_loop_throttle(limit, full_throttle_speed, start)
            }
            SpeedFeedback::Wheel => 0.0,
        }
    }
}

/// Where the transmitter's mode switch is (TODO 48).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RcSwitch {
    /// Normal: the computer drives. With a receiver fitted, its signal must stay.
    #[default]
    Computer,
    /// A person drives with the sticks, through the computer: the board sends them up,
    /// a ROS driver turns them into commands, and the computer's frames drive.
    ViaRos,
    /// Manual: a person drives with the sticks, straight from this board (RC direct). The
    /// computer's commands count for nothing.
    Direct,
}

/// The RC receiver, already scaled by the board crate (`ibus::RcMap`).
///
/// The default is the mode switch on normal, both sticks in the middle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcInput {
    /// The transmitter's mode switch.
    pub switch: RcSwitch,
    /// The left stick, -1..1, positive forward: 0 in its middle zone, then from 0 at the
    /// zone's edge to 1 at the stick's end. Only its sign drives the car (the RC's speeds
    /// are fixed); the board sends it up as it is.
    pub throttle: f32,
    /// -1..1, positive left.
    pub steering: f32,
    /// The left stick is in its middle zone (20 % each way): throttle off, the motor's
    /// own braking.
    pub middle: bool,
    /// The left stick is past 88 % either way: asking for the other direction while the
    /// car still moves, a hard brake.
    pub full: bool,
    /// The right stick, up and down.
    pub right: RightStick,
}

impl Default for RcInput {
    fn default() -> Self {
        RcInput {
            switch: RcSwitch::Computer,
            throttle: 0.0,
            steering: 0.0,
            middle: true,
            full: false,
            right: RightStick::Centre,
        }
    }
}

impl RcInput {
    /// Which way the left stick asks to go.
    #[must_use]
    pub fn pedal(&self) -> Pedal {
        if self.middle {
            Pedal::Middle
        } else if self.throttle < 0.0 {
            Pedal::Reverse
        } else {
            Pedal::Forward
        }
    }
}

/// An open-loop command: the actuators' demands as they are, with no speed loop. The
/// 2025-26 UART link carries these (`legacy-link`): it has no speed, and no arming bit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectCmd {
    /// 0..1, to the throttle. Ignored while `brake` is above 0.
    pub throttle: f32,
    /// The direction for the throttle.
    pub reverse: bool,
    /// 0..1, to the brake.
    pub brake: f32,
    /// rad, positive left.
    pub steering_angle: f32,
    /// This command is a stop: no throttle. A stop is the first half of the arming edge,
    /// like `armed = 0` in a DRIVE_CMD; any command after it is the second half. Unlike
    /// `armed = 0`, a stop still drives the actuators: its brake is applied.
    pub stop: bool,
}

/// The computer's latest command, of either kind.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Command {
    Speed(DriveCmd),
    Direct(DirectCmd),
}

impl Command {
    fn armed(&self) -> bool {
        match self {
            Command::Speed(cmd) => cmd.armed,
            // Every command of this kind counts as armed: the link has no arming bit.
            // Only its stops make the edge (below).
            Command::Direct(_) => true,
        }
    }
}

/// What the RC asks for this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
enum RcDrive {
    /// Drive at this speed (m/s, positive forward), through the speed loop.
    Go(f32),
    /// No throttle: this brake (0..1), the direction output as it was.
    Pedals {
        /// 0..1, to the brake.
        brake: f32,
        /// The direction output.
        reverse: bool,
    },
}

/// The car's speed, as a link sends it for a board that can't measure it: an input to the
/// overspeed and lost-speed rules only. The throttle stays open loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpeedReport {
    /// m/s, positive forward.
    Known(f32),
    /// The sender is there, and doesn't know the speed.
    Unknown,
}

/// Everything the board read this tick.
#[derive(Debug, Clone, Copy, Default)]
pub struct Inputs {
    /// Free-running milliseconds; may wrap.
    pub now_ms: u32,
    /// A valid DRIVE_CMD received since the last tick.
    pub drive_cmd: Option<DriveCmd>,
    /// A valid open-loop command received since the last tick (the UART link).
    pub direct_cmd: Option<DirectCmd>,
    /// A valid ESTOP_CMD received since the last tick.
    pub estop_cmd: Option<EstopCmd>,
    /// A hardware test mode request received since the last tick. Each one also keeps
    /// the test mode fresh: none for `cmd_timeout_ms` leaves it.
    pub test_cmd: Option<TestRequest>,
    /// A frame for the spine failed its length or checksum since the last tick.
    pub bad_frame: bool,
    /// The hardware e-stop loop is open.
    pub hw_estop: bool,
    /// None when the RC receiver has been silent too long.
    pub rc: Option<RcInput>,
    /// The board's wheel speed (m/s), used with [`SpeedFeedback::Wheel`].
    pub measured_speed: f32,
    /// The speed a link sent since the last tick. Used only with [`SpeedFeedback::None`].
    /// A link that has never sent one leaves the overspeed and lost-speed rules off.
    pub speed_report: Option<SpeedReport>,
    /// The steering's position (rad, positive left).
    pub measured_steering: f32,
    /// Driver faults found by the board: STEERING_DRIVER, BRAKE_DRIVER,
    /// THROTTLE_DAC, WHEEL_ENCODER, CAN_BUS.
    pub driver_faults: u16,
}

/// What the board writes to the hardware this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outputs {
    /// Who drives.
    pub mode: Mode,
    /// 0..1, to the throttle DAC.
    pub throttle: f32,
    /// 0..1, to the brake actuator.
    pub brake: f32,
    /// Direction output to the motor controller.
    pub reverse: bool,
    /// rad, positive left; the board converts it to a Jrk G2 target.
    pub steering_target: f32,
    /// The spine is armed.
    pub armed: bool,
    /// An e-stop is on.
    pub estop_active: bool,
    /// The fault bits ([`faults`]).
    pub faults: u16,
}

/// The spine's whole state: feed it inputs, and each tick gives the outputs.
#[derive(Debug)]
pub struct Spine {
    cfg: SpineConfig,
    watchdog: Watchdog,
    speed_loop: SpeedLoop,
    last_cmd: Option<Command>,
    last_counter: Option<u8>,
    sw_estop: bool,
    /// The five regions (`crate::fsm`); `latch` is what Safety remembers.
    lifecycle: Lifecycle,
    safety: Safety,
    latch: Latch,
    authority: Authority,
    motion: Motion,
    /// The test mode's own watchdog, its latest request, and the first half of its entry.
    test_watchdog: Watchdog,
    test_request: Option<TestRequest>,
    seen_test_exit: bool,
    /// The mode switch when the RC was last heard: Computer at boot.
    rc_switch: RcSwitch,
    /// The RC stays braked after any stop, at power-up and after the mode switch moves,
    /// until a person moves the left stick on purpose ([`RcHold`]).
    rc_hold: Option<RcHold>,
    /// Which way the RC last drove (true: reverse); `None` once the car is known to be
    /// stopped, and the RC may go either way.
    rc_reverse: Option<bool>,
    /// Since when the car has been under [`STANDSTILL_SPEED`], with a speed known (ms).
    still_since_ms: Option<u32>,
    /// Since when the RC has held the hard brake asking for the other direction (ms).
    hard_brake_since_ms: Option<u32>,
    /// Normal mode: does the RC or the computer have the car? The RC's at boot.
    owner: RcOwner,
    /// The speed from the link: the latest, whether one ever came, its own watchdog.
    reported_speed: f32,
    speed_seen: bool,
    speed_watchdog: Watchdog,
    /// This tick's speed, and whether it is known. Nothing else reads the speed.
    speed: f32,
    speed_known: bool,
    overspeed: Overspeed,
    speed_setpoint: f32,
    steering_setpoint: f32,
    last_ms: Option<u32>,
    frame_faults: u16,
    feedback_counter: u8,
    status_counter: u8,
}

impl Spine {
    /// A spine just booted, running with `cfg`.
    #[must_use]
    pub fn new(cfg: SpineConfig) -> Self {
        Spine {
            cfg,
            watchdog: Watchdog::new(cfg.cmd_timeout_ms),
            speed_loop: SpeedLoop::new(cfg.speed_loop),
            last_cmd: None,
            last_counter: None,
            sw_estop: false,
            lifecycle: Lifecycle::Boot,
            safety: Safety::Latched,
            latch: Latch::default(), // boot latched: the computer must disarm, then arm
            authority: Authority::Idle,
            motion: Motion::StandstillHold,
            test_watchdog: Watchdog::new(cfg.cmd_timeout_ms),
            test_request: None,
            seen_test_exit: false,
            rc_switch: RcSwitch::Computer,
            rc_hold: Some(RcHold::start(0, None)),
            rc_reverse: None,
            still_since_ms: None,
            hard_brake_since_ms: None,
            owner: RcOwner::RC,
            reported_speed: 0.0,
            speed_seen: false,
            speed_watchdog: Watchdog::new(cfg.speed_timeout_ms),
            speed: 0.0,
            speed_known: matches!(cfg.speed_feedback, SpeedFeedback::Wheel),
            overspeed: Overspeed::Clear,
            speed_setpoint: 0.0,
            steering_setpoint: 0.0,
            last_ms: None,
            frame_faults: 0,
            feedback_counter: 0,
            status_counter: 0,
        }
    }

    /// One tick: decide, cut the throttle if the car is coming back under its limit, then
    /// note which way the car moves (the Motion region).
    pub fn step(&mut self, i: &Inputs) -> Outputs {
        let mut out = self.decide(i);
        if self.overspeed.is_slowing() {
            self.speed_loop.reset();
            self.speed_setpoint = 0.0;
            out.throttle = 0.0;
            out.brake = out.brake.max(self.cfg.overspeed_brake);
        }
        self.motion = Motion::observe(self.speed, out.throttle, out.reverse);
        out
    }

    /// All five regions, as the latest tick left them.
    #[must_use]
    pub fn state(&self) -> State {
        State {
            lifecycle: self.lifecycle,
            safety: self.safety,
            authority: self.authority,
            motion: self.motion,
            confidence: self.confidence(),
        }
    }

    /// [5] Confidence: whether this tick's speed is known.
    fn confidence(&self) -> Confidence {
        if self.speed_known {
            Confidence::Full
        } else {
            Confidence::Degraded
        }
    }

    /// This tick's speed from the board or the link, and whether a link's speed is lost.
    /// A link that has never sent one has nothing to lose.
    fn update_speed(&mut self, i: &Inputs) -> bool {
        if let SpeedFeedback::Wheel = self.cfg.speed_feedback {
            self.speed = i.measured_speed;
            self.speed_known = true;
            return false;
        }
        if let Some(report) = i.speed_report {
            self.speed_seen = true;
            if let SpeedReport::Known(speed) = report {
                if speed.is_finite() {
                    self.reported_speed = speed;
                    self.speed_watchdog.feed(i.now_ms);
                }
            }
        }
        self.speed_known = self.speed_seen && !self.speed_watchdog.expired(i.now_ms);
        self.speed = if self.speed_known {
            self.reported_speed
        } else {
            i.measured_speed
        };
        self.speed_seen && !self.speed_known
    }

    /// The test request the spine carries out: the latest, unless this confidence refuses
    /// it ([`TestRequest::allowed`]), when it holds still instead.
    fn allowed_test_request(&self) -> Option<TestRequest> {
        let confidence = self.confidence();
        self.test_request.map(|r| {
            if r.allowed(confidence) {
                r
            } else {
                TestRequest::Hold
            }
        })
    }

    fn decide(&mut self, i: &Inputs) -> Outputs {
        let c = self.cfg;
        let dt = match self.last_ms {
            Some(t) => (i.now_ms.wrapping_sub(t) as f32 / 1000.0).min(0.1),
            None => 0.0,
        };
        self.last_ms = Some(i.now_ms);
        if i.bad_frame {
            self.frame_faults |= faults::BAD_CHECKSUM;
        }

        // New frames. A repeated counter means a stuck sender: it doesn't
        // count as a fresh command.
        if let Some(e) = i.estop_cmd {
            self.sw_estop = e.estop_request;
        }
        if let Some(cmd) = i.drive_cmd {
            if counter_step(self.last_counter, cmd.counter) == 0 {
                self.frame_faults |= faults::BAD_COUNTER;
            } else {
                self.watchdog.feed(i.now_ms);
                self.last_cmd = Some(Command::Speed(cmd));
                if !cmd.armed {
                    self.latch.seen_disarm = true;
                }
            }
            self.last_counter = Some(cmd.counter);
        }
        // The UART link has no counter: its checksum is all it has.
        if let Some(cmd) = i.direct_cmd {
            self.watchdog.feed(i.now_ms);
            self.last_cmd = Some(Command::Direct(cmd));
            if cmd.stop {
                self.latch.seen_disarm = true;
            }
        }
        // The test mode's requests: each one keeps it fresh.
        if let Some(req) = i.test_cmd {
            self.test_watchdog.feed(i.now_ms);
            self.test_request = Some(req);
            if req == TestRequest::Exit {
                self.seen_test_exit = true;
            }
        }
        let fresh = !self.watchdog.expired(i.now_ms);
        let armed_request = fresh && self.last_cmd.is_some_and(|cmd| cmd.armed());
        let test_fresh = self.test_request.is_some() && !self.test_watchdog.expired(i.now_ms);
        let speed_lost = self.update_speed(i);

        // [2] Safety: the latches.
        let estop = i.hw_estop || self.sw_estop;
        if estop {
            self.latch.trip();
        }
        // The RC. A new mode switch position: the left stick may rest anywhere, so the RC
        // holds the car until it moves (Arsh, 21 Nov).
        let pedal = i.rc.map(|rc| rc.pedal());
        let rc_stop = i.rc.is_some_and(|rc| rc.right == RightStick::Stop);
        let mut switch_moved = false;
        if let Some(rc) = i.rc {
            switch_moved = rc.switch != self.rc_switch;
            self.rc_switch = rc.switch;
        }
        let rc_lost = i.rc.is_none() && (self.rc_switch != RcSwitch::Computer || c.rc_fitted);
        if rc_lost {
            self.latch.trip();
        }
        let driver_fault = i.driver_faults & faults::STOP != 0;
        if driver_fault {
            self.latch.trip_fault();
        }
        if speed_lost {
            self.latch.trip_speed_lost();
        }
        // Too fast: come back under the limit in time, or a hard stop.
        let limit = if self.speed >= 0.0 {
            c.max_speed
        } else {
            c.max_reverse_speed
        };
        let over = self.speed.abs() - limit;
        self.overspeed = if self.speed_known {
            self.overspeed
                .next(over > c.overspeed_margin, over > 0.0, i.now_ms)
        } else {
            Overspeed::Clear
        };
        let overspeed_expired = self.overspeed.expired(i.now_ms);
        if overspeed_expired {
            self.latch.trip_overspeed();
        }
        // Any stop holds the RC until a person moves the left stick ([`RcHold`]): an e-stop
        // or a driver fault clearing with the stick still pushed must not drive the car
        // again by itself. So does the right stick pulled down, the RC's own stop. The RC
        // has the car after each (normal mode's owner).
        if estop
            || driver_fault
            || rc_lost
            || speed_lost
            || overspeed_expired
            || rc_stop
            || switch_moved
        {
            self.rc_hold = Some(RcHold::start(i.now_ms, pedal));
            self.owner = RcOwner::RC;
        } else if self.overspeed == Overspeed::Clear {
            self.rc_hold = self.rc_hold.and_then(|h| h.next(i.now_ms, pedal));
        }
        // Like the computer's disarm, the test mode's Exit counts only after the latest
        // e-stop, fault or lost RC.
        if estop || driver_fault || rc_lost || speed_lost {
            self.seen_test_exit = false;
        }
        // The computer went silent while armed: a timeout, whoever drove meanwhile (RC
        // direct may have taken over), so it must re-arm after a crash or a reboot.
        let computer_armed = self.authority == Authority::Drive
            || (!self.latch.latched && self.last_cmd.is_some_and(|cmd| cmd.armed()));
        if computer_armed && !fresh && !estop {
            self.latch.trip_timeout();
        }

        // [1] Lifecycle: nothing is armed until the self-test passes.
        self.lifecycle = self.lifecycle.next(!driver_fault);
        let running = self.lifecycle.is_running();
        self.latch.try_release(
            estop || driver_fault || speed_lost || self.overspeed.is_slowing(),
            armed_request && running,
        );

        // [3] Normal mode's owner: the right stick takes the car and gives it back
        // (RcOwner). The RC outranks the computer, decided here so it works with the
        // computer dead.
        let rc_direct = self.rc_switch == RcSwitch::Direct;
        let normal = self.rc_switch == RcSwitch::Computer && i.rc.is_some();
        if normal {
            if let Some(rc) = i.rc {
                self.owner = self.owner.next(i.now_ms, rc.right);
            }
        }
        let rc_owns = normal && self.owner.is_rc();
        // The RC has the car but asks nothing: held, or the left stick in its middle. The
        // test mode may start, and beats it.
        let rc_idle =
            rc_owns && !rc_stop && (self.rc_hold.is_some() || pedal == Some(Pedal::Middle));
        let rc_has_car = rc_direct || (rc_owns && !rc_idle);

        // [3] The test mode. Leave it on anything unsafe, a stale request, or an Exit;
        // leaving always latches, so the computer must re-arm before it drives.
        let mut in_test = self.authority.in_test();
        if in_test {
            let asked_out = self.test_request == Some(TestRequest::Exit);
            let unsafe_now = estop
                || driver_fault
                || rc_lost
                || speed_lost
                || rc_has_car
                || self.speed.abs() > TEST_MAX_SPEED;
            if !test_fresh && !unsafe_now {
                self.latch.trip_timeout();
                in_test = false;
            } else if unsafe_now || asked_out {
                self.latch.trip();
                in_test = false;
            }
        } else if i.test_cmd == Some(TestRequest::Enter)
            && self.seen_test_exit
            && running
            && !estop
            && !driver_fault
            && !rc_lost
            && !speed_lost
            && !rc_has_car
            && self.authority != Authority::Drive
            // Nor on the tick the computer's arm releases the latch.
            && (!armed_request || self.latch.latched)
            && self.speed.abs() < crate::fsm::STANDSTILL_SPEED
        {
            // Exit, then Enter, while nothing stops the car and nobody drives it.
            in_test = true;
            self.seen_test_exit = false;
        }

        // [3] Authority.
        self.authority = if rc_has_car {
            Authority::Rc
        } else if in_test {
            Authority::Test(
                self.allowed_test_request()
                    .map_or(TestStage::Armed, |r| r.stage()),
            )
        } else if rc_idle {
            Authority::Rc
        } else if armed_request && !self.latch.latched {
            Authority::Drive
        } else {
            Authority::Idle
        };
        self.safety = Safety::of(
            estop,
            driver_fault,
            rc_lost || speed_lost,
            self.overspeed.is_slowing(),
            &self.latch,
        );

        let mut fault_bits = i.driver_faults | self.frame_faults;
        if self.latch.timeout {
            fault_bits |= faults::CMD_TIMEOUT;
        }
        if rc_lost {
            fault_bits |= faults::RC_SIGNAL_LOST;
        }
        if self.overspeed.is_slowing() || self.latch.overspeed {
            fault_bits |= faults::OVERSPEED;
        }
        if speed_lost || self.latch.speed_lost {
            fault_bits |= faults::WHEEL_ENCODER;
        }

        // The right stick's stop while the RC doesn't drive (RC via ROS): the RC's own
        // brake, below, covers the rest, with the steering stick still steering.
        let stop = estop || driver_fault || rc_lost || speed_lost || (rc_stop && !rc_has_car);
        let latched_stop =
            self.latch.timeout || self.latch.fault || self.latch.speed_lost || self.latch.overspeed;
        let out = |throttle, brake, reverse, steering, s: &Self| Outputs {
            mode: s.authority.mode(),
            throttle,
            brake,
            reverse,
            steering_target: steering,
            armed: armed_request && !s.latch.latched,
            estop_active: estop || latched_stop,
            faults: fault_bits,
        };

        if self.authority != Authority::Rc {
            self.rc_reverse = None;
            self.still_since_ms = None;
            self.hard_brake_since_ms = None;
        }
        if stop || (self.authority == Authority::Idle && latched_stop) {
            self.rc_reverse = None;
            self.speed_setpoint = 0.0;
            self.speed_loop.reset();
            if c.estop_steering == EstopSteering::Centre {
                self.steering_setpoint = slew(self.steering_setpoint, 0.0, c.max_steer_rate, dt);
            }
            return out(0.0, c.stop_brake, false, self.steering_setpoint, self);
        }

        // Targets for this mode, inside the firmware's own limits.
        let (target_speed, target_steering, accel, steer_rate) = match self.authority {
            Authority::Test(_) => return self.test(dt, &out),
            Authority::Idle => {
                self.speed_setpoint = 0.0;
                self.speed_loop.reset();
                return out(0.0, c.idle_brake, false, self.steering_setpoint, self);
            }
            Authority::Rc => match self.rc(i, dt) {
                RcDrive::Go(target) => (
                    target,
                    i.rc.map_or(0.0, |rc| rc.steering) * c.max_steering_angle,
                    RC_ACCEL.min(c.max_accel),
                    c.max_steer_rate,
                ),
                RcDrive::Pedals { brake, reverse } => {
                    self.speed_loop.reset();
                    return out(0.0, brake, reverse, self.steering_setpoint, self);
                }
            },
            Authority::Drive => {
                let cmd = match self.last_cmd {
                    Some(Command::Speed(cmd)) => cmd,
                    Some(Command::Direct(cmd)) => return self.direct(&cmd, dt, &out),
                    None => DriveCmd {
                        target_speed: 0.0,
                        steering_angle: self.steering_setpoint,
                        accel_limit: 0.0,
                        steer_rate_limit: 0.0,
                        armed: false,
                        counter: 0,
                    },
                };
                let accel = if cmd.accel_limit > 0.0 {
                    cmd.accel_limit.min(c.max_accel)
                } else {
                    c.max_accel
                };
                let rate = if cmd.steer_rate_limit > 0.0 {
                    cmd.steer_rate_limit.min(c.max_steer_rate)
                } else {
                    c.max_steer_rate
                };
                (cmd.target_speed, cmd.steering_angle, accel, rate)
            }
        };
        let target_speed = target_speed.clamp(-c.max_reverse_speed, c.max_speed);
        let target_steering = target_steering.clamp(-c.max_steering_angle, c.max_steering_angle);
        self.steering_setpoint = slew(self.steering_setpoint, target_steering, steer_rate, dt);
        // Slowing down (toward zero, or through it) uses the separate, harder cap.
        let slowing = target_speed.abs() < self.speed_setpoint.abs()
            || target_speed * self.speed_setpoint < 0.0;
        let rate = if slowing { c.max_decel } else { accel };
        self.speed_setpoint = slew(self.speed_setpoint, target_speed, rate, dt);
        let drive = match c.speed_feedback {
            SpeedFeedback::Wheel => {
                self.speed_loop
                    .update(self.speed_setpoint, i.measured_speed, dt)
            }
            SpeedFeedback::None {
                full_throttle_speed,
                start_throttle,
                start_throttle_reverse,
            } => {
                let start = if self.speed_setpoint < 0.0 {
                    start_throttle_reverse
                } else {
                    start_throttle
                };
                self.speed_loop
                    .open_loop(self.speed_setpoint, full_throttle_speed, start)
            }
        };
        out(
            drive.throttle,
            drive.brake,
            drive.reverse,
            self.steering_setpoint,
            self,
        )
    }

    /// The RC driving (Arsh and Ben, 21 Nov): the left stick says which way, at a fixed
    /// speed; its middle lets the motor slow the car; the right stick down brakes hard.
    ///
    /// | The left stick | Car going the same way, or known stopped | Car going the other way |
    /// |---|---|---|
    /// | Up or back | drive at `max_speed` (`max_reverse_speed` back), ramped at [`RC_ACCEL`] | throttle off; past 88 % the hard brake too. Then the new way, once stopped |
    /// | Middle | throttle off | throttle off |
    ///
    /// Stopped: under [`STANDSTILL_SPEED`] for [`RC_STOPPED_MS`], with a speed known.
    /// With none, the hard brake held for [`RC_BRAKE_MS`] counts instead.
    fn rc(&mut self, i: &Inputs, dt: f32) -> RcDrive {
        let c = self.cfg;
        let now = i.now_ms;
        let rc = i.rc.unwrap_or(RcInput {
            right: RightStick::Stop,
            ..RcInput::default()
        });
        let steering = rc.steering.clamp(-1.0, 1.0) * c.max_steering_angle;
        let steering = steering.clamp(-c.max_steering_angle, c.max_steering_angle);
        // Held, or the right stick down: the stop brake, the steering stick still steers.
        if rc.right == RightStick::Stop || self.rc_hold.is_some() {
            self.rc_reverse = None;
            self.hard_brake_since_ms = None;
            self.speed_setpoint = 0.0;
            self.steering_setpoint = slew(self.steering_setpoint, steering, c.max_steer_rate, dt);
            return RcDrive::Pedals {
                brake: c.stop_brake,
                reverse: false,
            };
        }
        let still = self.speed_known && self.speed.abs() < STANDSTILL_SPEED;
        self.still_since_ms = if still {
            Some(self.still_since_ms.unwrap_or(now))
        } else {
            None
        };
        if self
            .still_since_ms
            .is_some_and(|t| now.wrapping_sub(t) >= RC_STOPPED_MS)
        {
            self.rc_reverse = None;
        }
        let want = match rc.pedal() {
            Pedal::Middle => None,
            Pedal::Forward => Some(false),
            Pedal::Reverse => Some(true),
        };
        let coasting = |s: &mut Self| {
            s.steering_setpoint = slew(s.steering_setpoint, steering, c.max_steer_rate, dt);
            // The ramp starts again from the speed the car has, when it is known.
            let v = if s.speed_known { s.speed } else { 0.0 };
            s.speed_setpoint = match s.rc_reverse {
                Some(false) => v.clamp(0.0, c.max_speed),
                Some(true) => v.clamp(-c.max_reverse_speed, 0.0),
                None => 0.0,
            };
        };
        match want {
            None => {
                self.hard_brake_since_ms = None;
                coasting(self);
                RcDrive::Pedals {
                    brake: 0.0,
                    reverse: self.rc_reverse.unwrap_or(false),
                }
            }
            Some(reverse) if self.rc_reverse.is_some_and(|r| r != reverse) => {
                self.hard_brake_since_ms = if rc.full {
                    Some(self.hard_brake_since_ms.unwrap_or(now))
                } else {
                    None
                };
                let braked_long = !self.speed_known
                    && self
                        .hard_brake_since_ms
                        .is_some_and(|t| now.wrapping_sub(t) >= RC_BRAKE_MS);
                let was = self.rc_reverse.unwrap_or(false);
                coasting(self);
                if braked_long {
                    // Stopped, as far as it can tell: the new way from the next tick.
                    self.rc_reverse = None;
                    self.speed_setpoint = 0.0;
                }
                RcDrive::Pedals {
                    brake: if rc.full { c.stop_brake } else { 0.0 },
                    reverse: was,
                }
            }
            Some(reverse) => {
                self.hard_brake_since_ms = None;
                self.rc_reverse = Some(reverse);
                RcDrive::Go(if reverse {
                    -c.max_reverse_speed
                } else {
                    c.max_speed
                })
            }
        }
    }

    /// An open-loop command, in computer mode: steering at the steering rate limit, the
    /// throttle and the brake as asked (never both), inside the firmware's limits.
    ///
    /// The throttle is capped so the car stays under `max_speed`, or `max_reverse_speed`
    /// in reverse: with no wheel speed on the board, at the throttle that open loop would
    /// give that speed; with one, cut while the car is at or over it.
    fn direct(
        &mut self,
        cmd: &DirectCmd,
        dt: f32,
        out: &impl Fn(f32, f32, bool, f32, &Self) -> Outputs,
    ) -> Outputs {
        let c = self.cfg;
        self.speed_setpoint = 0.0;
        self.speed_loop.reset();
        let target = cmd
            .steering_angle
            .clamp(-c.max_steering_angle, c.max_steering_angle);
        let target = if target.is_nan() {
            self.steering_setpoint
        } else {
            target
        };
        self.steering_setpoint = slew(self.steering_setpoint, target, c.max_steer_rate, dt);
        let brake = if cmd.brake.is_nan() {
            1.0
        } else {
            cmd.brake.clamp(0.0, 1.0)
        };
        let throttle = if cmd.stop || brake > 0.0 || cmd.throttle.is_nan() {
            0.0
        } else {
            cmd.throttle.clamp(0.0, 1.0)
        };
        let limit = if cmd.reverse {
            c.max_reverse_speed
        } else {
            c.max_speed
        };
        let throttle = throttle.min(self.throttle_cap(limit, cmd.reverse));
        out(throttle, brake, cmd.reverse, self.steering_setpoint, self)
    }

    /// The hardware test mode: the latest request moves one actuator, inside the limits.
    /// Otherwise the car holds still: throttle 0, the stop brake, the steering where it is.
    /// The throttle is forward only, capped at [`TEST_MAX_THROTTLE`], with the brake off,
    /// and only with a wheel speed ([`Confidence::Full`]).
    fn test(&mut self, dt: f32, out: &impl Fn(f32, f32, bool, f32, &Self) -> Outputs) -> Outputs {
        let c = self.cfg;
        self.speed_setpoint = 0.0;
        self.speed_loop.reset();
        let mut throttle = 0.0;
        let mut brake = c.stop_brake;
        let mut steering = self.steering_setpoint;
        match self.allowed_test_request() {
            Some(TestRequest::Steering(a)) if !a.is_nan() => {
                steering = a.clamp(-c.max_steering_angle, c.max_steering_angle);
            }
            Some(TestRequest::Brake(b)) if !b.is_nan() => brake = b.clamp(0.0, 1.0),
            Some(TestRequest::Throttle(t)) if !t.is_nan() && t > 0.0 => {
                throttle = t.min(TEST_MAX_THROTTLE);
                brake = 0.0;
            }
            _ => {}
        }
        self.steering_setpoint = slew(self.steering_setpoint, steering, c.max_steer_rate, dt);
        out(throttle, brake, false, self.steering_setpoint, self)
    }

    /// The most throttle an open-loop command may have to stay under `limit` (0..1).
    fn throttle_cap(&self, limit: f32, reverse: bool) -> f32 {
        match self.cfg.speed_feedback {
            SpeedFeedback::None { .. } => self.cfg.open_loop_cap(limit, reverse),
            SpeedFeedback::Wheel if self.speed.abs() >= limit => 0.0,
            SpeedFeedback::Wheel => 1.0,
        }
    }

    /// VEHICLE_FEEDBACK frame (send at 50 Hz).
    pub fn feedback_frame(
        &mut self,
        measured_speed: f32,
        measured_steering: f32,
        wheel_ticks: u16,
    ) -> [u8; 8] {
        self.feedback_counter = (self.feedback_counter + 1) & 0x0F;
        VehicleFeedback {
            measured_speed,
            measured_steering_angle: measured_steering,
            wheel_ticks,
            counter: self.feedback_counter,
        }
        .encode()
    }

    /// VEHICLE_STATUS frame (send at 10 Hz). Reports frame faults seen since
    /// the previous status frame, then clears them.
    pub fn status_frame(&mut self, out: &Outputs) -> [u8; 8] {
        self.status_counter = (self.status_counter + 1) & 0x0F;
        let frame = VehicleStatus {
            mode: out.mode,
            armed: out.armed,
            estop_active: out.estop_active,
            throttle: out.throttle * 100.0,
            brake: out.brake * 100.0,
            faults: out.faults | self.frame_faults,
            counter: self.status_counter,
        }
        .encode();
        self.frame_faults = 0;
        frame
    }

    /// Who drives now.
    #[must_use]
    pub fn mode(&self) -> Mode {
        self.authority.mode()
    }

    /// The rate-limited speed the speed loop is chasing (m/s).
    #[must_use]
    pub fn speed_setpoint(&self) -> f32 {
        self.speed_setpoint
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    struct Car {
        spine: Spine,
        now: u32,
        counter: u8,
    }

    impl Car {
        fn new() -> Self {
            Car::with_config(SpineConfig::default())
        }

        fn with_config(cfg: SpineConfig) -> Self {
            Car {
                spine: Spine::new(cfg),
                now: 1000,
                counter: 0,
            }
        }

        fn cmd(&mut self, speed: f32, steering: f32, armed: bool) -> DriveCmd {
            self.counter = (self.counter + 1) & 0x0F;
            DriveCmd {
                target_speed: speed,
                steering_angle: steering,
                accel_limit: 0.0,
                steer_rate_limit: 0.0,
                armed,
                counter: self.counter,
            }
        }

        fn tick(&mut self, mut i: Inputs) -> Outputs {
            self.now += 10;
            i.now_ms = self.now;
            self.spine.step(&i)
        }

        /// A person clears the RC's hold (at power-up, after a stop, after the mode switch
        /// moves): a second with the left stick in its middle, the switch at `switch`. A
        /// stick that was elsewhere has then moved; one that was in the middle clears as
        /// soon as it leaves it.
        fn clear_rc(&mut self, switch: RcSwitch) {
            for _ in 0..101 {
                self.tick(Inputs {
                    rc: Some(RcInput {
                        switch,
                        throttle: 0.0,
                        steering: 0.0,
                        middle: true,
                        ..RcInput::default()
                    }),
                    ..Default::default()
                });
            }
        }

        /// The computer's arming edge: `armed = 0`, then the caller sends `armed = 1`.
        fn arm(&mut self) {
            self.drive(0.0, 0.0, false, 2);
        }

        fn drive(&mut self, speed: f32, steering: f32, armed: bool, ticks: u32) -> Outputs {
            let mut last = None;
            for _ in 0..ticks {
                let cmd = self.cmd(speed, steering, armed);
                last = Some(self.tick(Inputs {
                    drive_cmd: Some(cmd),
                    ..Default::default()
                }));
            }
            last.unwrap()
        }
    }

    #[test]
    fn idle_without_commands() {
        let mut car = Car::new();
        let out = car.tick(Inputs::default());
        assert_eq!(out.mode, Mode::Manual);
        assert_eq!(out.throttle, 0.0);
        assert_eq!(out.faults & faults::CMD_TIMEOUT, 0);
    }

    #[test]
    fn boots_latched_until_disarm_then_arm() {
        let mut car = Car::new();
        // Armed frames straight after a reboot must not drive...
        let out = car.drive(1.0, 0.0, true, 20);
        assert_eq!(out.mode, Mode::Manual);
        assert_eq!(out.throttle, 0.0);
        // ...until the computer disarms, then arms.
        car.arm();
        assert_eq!(car.drive(1.0, 0.0, true, 5).mode, Mode::Computer);
    }

    #[test]
    fn armed_commands_drive() {
        let mut car = Car::new();
        car.arm();
        let out = car.drive(1.0, 0.0, true, 20);
        assert_eq!(out.mode, Mode::Computer);
        assert!(out.armed);
        assert!(out.throttle > 0.0);
        assert_eq!(out.brake, 0.0);
    }

    #[test]
    fn unarmed_commands_do_not_drive() {
        let mut car = Car::new();
        let out = car.drive(1.0, 0.0, false, 20);
        assert_eq!(out.mode, Mode::Manual);
        assert_eq!(out.throttle, 0.0);
    }

    #[test]
    fn lost_commands_brake_and_latch() {
        let mut car = Car::new();
        car.arm();
        car.drive(1.0, 0.0, true, 20);
        let mut out = car.tick(Inputs::default());
        for _ in 0..25 {
            out = car.tick(Inputs::default()); // 250 ms of silence
        }
        assert_eq!(out.brake, 1.0);
        assert_eq!(out.throttle, 0.0);
        assert_ne!(out.faults & faults::CMD_TIMEOUT, 0);
        // Fresh armed commands alone don't restart the car...
        let out = car.drive(1.0, 0.0, true, 5);
        assert_eq!(out.throttle, 0.0);
        // ...a disarm then an arm does.
        car.drive(0.0, 0.0, false, 2);
        let out = car.drive(1.0, 0.0, true, 5);
        assert_eq!(out.mode, Mode::Computer);
        assert_eq!(out.faults & faults::CMD_TIMEOUT, 0);
    }

    #[test]
    fn software_estop_brakes_and_needs_rearm() {
        let mut car = Car::new();
        car.arm();
        car.drive(1.0, 0.0, true, 10);
        let cmd = car.cmd(1.0, 0.0, true);
        let out = car.tick(Inputs {
            drive_cmd: Some(cmd),
            estop_cmd: Some(EstopCmd {
                estop_request: true,
                counter: 1,
            }),
            ..Default::default()
        });
        assert!(out.estop_active);
        assert_eq!(out.brake, 1.0);
        let cmd = car.cmd(1.0, 0.0, true);
        let out = car.tick(Inputs {
            drive_cmd: Some(cmd),
            estop_cmd: Some(EstopCmd {
                estop_request: false,
                counter: 2,
            }),
            ..Default::default()
        });
        assert_ne!(out.mode, Mode::Computer);
        car.drive(0.0, 0.0, false, 2);
        assert_eq!(car.drive(1.0, 0.0, true, 3).mode, Mode::Computer);
    }

    #[test]
    fn hardware_estop_beats_rc() {
        let mut car = Car::new();
        let rc = RcInput {
            switch: RcSwitch::Direct,
            throttle: 1.0,
            steering: 0.0,
            middle: false,
            ..RcInput::default()
        };
        let out = car.tick(Inputs {
            rc: Some(rc),
            hw_estop: true,
            ..Default::default()
        });
        assert_eq!(out.throttle, 0.0);
        assert_eq!(out.brake, 1.0);
    }

    #[test]
    fn rc_beats_computer() {
        let mut car = Car::new();
        car.arm();
        car.drive(1.0, 0.0, true, 10);
        let rc = RcInput {
            switch: RcSwitch::Direct,
            throttle: 0.0,
            steering: 1.0,
            middle: false,
            ..RcInput::default()
        };
        let cmd = car.cmd(1.0, 0.0, true);
        let out = car.tick(Inputs {
            drive_cmd: Some(cmd),
            rc: Some(rc),
            ..Default::default()
        });
        assert_eq!(out.mode, Mode::Rc);
    }

    #[test]
    fn rc_signal_loss_brakes() {
        let mut car = Car::new();
        let rc = RcInput {
            switch: RcSwitch::Direct,
            throttle: 0.5,
            steering: 0.0,
            middle: false,
            ..RcInput::default()
        };
        car.tick(Inputs {
            rc: Some(rc),
            ..Default::default()
        });
        let out = car.tick(Inputs::default());
        assert_eq!(out.brake, 1.0);
        assert_ne!(out.faults & faults::RC_SIGNAL_LOST, 0);
    }

    // --- RC via ROS (TODO 48) ---

    fn sticks(switch: RcSwitch) -> RcInput {
        RcInput {
            switch,
            throttle: 1.0,
            steering: 1.0,
            middle: false,
            ..RcInput::default()
        }
    }

    /// The left stick resting in its brake band: in normal mode, the computer may drive.
    fn band(switch: RcSwitch) -> RcInput {
        RcInput {
            switch,
            throttle: 0.0,
            steering: 0.0,
            middle: true,
            ..RcInput::default()
        }
    }

    impl Car {
        /// `ticks` of the computer's command, with the RC as given.
        fn drive_with_rc(
            &mut self,
            speed: f32,
            armed: bool,
            rc: Option<RcInput>,
            n: u32,
        ) -> Outputs {
            let mut last = None;
            for _ in 0..n {
                let cmd = self.cmd(speed, 0.0, armed);
                last = Some(self.tick(Inputs {
                    drive_cmd: Some(cmd),
                    rc,
                    ..Default::default()
                }));
            }
            last.unwrap()
        }
    }

    #[test]
    fn rc_via_ros_lets_the_computer_drive_and_ignores_the_sticks() {
        let mut car = Car::new();
        let rc = Some(sticks(RcSwitch::ViaRos));
        car.drive_with_rc(0.0, false, rc, 2);
        let out = car.drive_with_rc(0.0, true, rc, 100);
        // Full stick, but the computer says stand still: it stands still.
        assert_eq!((out.mode, out.throttle), (Mode::Computer, 0.0));
        assert!(out.steering_target.abs() < 1e-6);
        let out = car.drive_with_rc(1.0, true, rc, 100);
        assert_eq!(out.mode, Mode::Computer);
        assert!(out.throttle > 0.0);
        assert_eq!(out.faults & faults::RC_SIGNAL_LOST, 0);
    }

    #[test]
    fn rc_via_ros_brakes_and_latches_when_the_rc_is_lost() {
        let mut car = Car::new();
        let rc = Some(sticks(RcSwitch::ViaRos));
        car.drive_with_rc(0.0, false, rc, 2);
        car.drive_with_rc(1.0, true, rc, 50);
        // The receiver goes silent; the computer's commands still come.
        let out = car.drive_with_rc(1.0, true, None, 1);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_ne!(out.faults & faults::RC_SIGNAL_LOST, 0);
        assert_ne!(out.mode, Mode::Computer);
        let out = car.drive_with_rc(1.0, true, None, 20);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        // The RC comes back: no drive until the computer stops, then drives again.
        let out = car.drive_with_rc(1.0, true, rc, 20);
        assert_eq!((out.mode, out.throttle), (Mode::Manual, 0.0));
        assert_eq!(out.faults & faults::RC_SIGNAL_LOST, 0);
        car.drive_with_rc(0.0, false, rc, 2);
        assert_eq!(car.drive_with_rc(1.0, true, rc, 5).mode, Mode::Computer);
    }

    #[test]
    fn rc_lost_on_the_computer_position_changes_nothing() {
        let mut car = Car::new();
        let rc = Some(band(RcSwitch::Computer));
        car.drive_with_rc(0.0, false, rc, 2);
        car.drive_with_rc(1.0, true, normal(RightStick::Give), 51);
        assert_eq!(car.drive_with_rc(1.0, true, rc, 50).mode, Mode::Computer);
        let out = car.drive_with_rc(1.0, true, None, 20);
        assert_eq!(out.mode, Mode::Computer);
        assert_eq!(out.faults & faults::RC_SIGNAL_LOST, 0);
    }

    #[test]
    fn with_a_receiver_fitted_a_lost_rc_stops_the_computer_too() {
        // Arsh, 21 Nov: in normal mode the RC must stay, whether the computer drives or
        // not.
        let mut car = Car::with_config(SpineConfig {
            rc_fitted: true,
            ..SpineConfig::default()
        });
        let rc = normal(RightStick::Centre);
        // At power-up the RC has the car, until the right stick gives it.
        car.drive_with_rc(0.0, false, rc, 2);
        car.drive_with_rc(1.0, true, normal(RightStick::Give), 51);
        let out = car.drive_with_rc(1.0, true, rc, 100);
        assert_eq!(out.mode, Mode::Computer);
        assert!(out.throttle > 0.0);
        // The receiver goes silent (or sends the failsafe): a hard stop, latched.
        let out = car.drive_with_rc(1.0, true, None, 1);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_ne!(out.faults & faults::RC_SIGNAL_LOST, 0);
        // The RC back: it has the car, braked. Given back, the computer must still re-arm.
        let out = car.drive_with_rc(1.0, true, rc, 120);
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Give), 51);
        assert_eq!((out.mode, out.throttle), (Mode::Manual, 0.0));
        car.drive_with_rc(0.0, false, rc, 2);
        assert_eq!(car.drive_with_rc(1.0, true, rc, 5).mode, Mode::Computer);
        // No transmitter at all, from boot: never drives.
        let mut car = Car::with_config(SpineConfig {
            rc_fitted: true,
            ..SpineConfig::default()
        });
        car.drive_with_rc(0.0, false, None, 2);
        let out = car.drive_with_rc(1.0, true, None, 50);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_ne!(out.faults & faults::RC_SIGNAL_LOST, 0);
    }

    // --- Normal mode: the right stick takes the car and gives it back (Arsh and Ben, 21 Nov) ---

    /// The RC with the left stick at `pedal` (0.5 either way; `full`: at its end) and the
    /// right stick at `right`.
    fn rc_with(switch: RcSwitch, pedal: Pedal, full: bool, right: RightStick) -> RcInput {
        let throttle = match pedal {
            Pedal::Forward => 0.5,
            Pedal::Middle => 0.0,
            Pedal::Reverse => -0.5,
        };
        RcInput {
            switch,
            throttle,
            steering: 0.0,
            middle: pedal == Pedal::Middle,
            full,
            right,
        }
    }

    /// Normal mode, the left stick in its middle, the right stick at `right`.
    fn normal(right: RightStick) -> Option<RcInput> {
        Some(rc_with(RcSwitch::Computer, Pedal::Middle, false, right))
    }

    /// A spine with a receiver, as the ESP32 sets it up.
    fn with_receiver() -> Car {
        Car::with_config(SpineConfig {
            rc_fitted: true,
            idle_brake: 1.0,
            ..SpineConfig::default()
        })
    }

    /// A spine with a receiver, the car given to the computer, driving at 1 m/s.
    fn computer_driving() -> Car {
        let mut car = with_receiver();
        car.drive_with_rc(0.0, false, normal(RightStick::Centre), 2);
        car.drive_with_rc(1.0, true, normal(RightStick::Give), 51);
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Centre), 50);
        assert_eq!(out.mode, Mode::Computer);
        assert!(out.throttle > 0.0);
        car
    }

    #[test]
    fn at_power_up_the_rc_has_the_car_until_the_right_stick_gives_it() {
        let mut car = with_receiver();
        car.drive_with_rc(0.0, false, normal(RightStick::Centre), 2);
        // The computer armed and asking: the RC keeps the car, braked.
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Centre), 300);
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        // Up for just under half a second: still the RC's. Then the computer's.
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Give), 50);
        assert_eq!(out.mode, Mode::Rc);
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Give), 1);
        assert_eq!(out.mode, Mode::Computer);
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Centre), 20);
        assert!(out.throttle > 0.0);
    }

    #[test]
    fn the_right_stick_down_takes_the_car_from_the_computer_at_once() {
        let mut car = computer_driving();
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Stop), 1);
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Stop), 50);
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        // Let go, the left stick still in its middle: braked, whatever the computer asks.
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Centre), 300);
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        // The left stick up: the RC drives.
        let up = Some(rc_with(
            RcSwitch::Computer,
            Pedal::Forward,
            false,
            RightStick::Centre,
        ));
        let out = car.drive_with_rc(1.0, true, up, 20);
        assert_eq!(out.mode, Mode::Rc);
        assert!(out.throttle > 0.0 && out.brake == 0.0, "{out:?}");
    }

    #[test]
    fn the_left_stick_alone_never_takes_the_car_from_the_computer() {
        let mut car = computer_driving();
        for pedal in [Pedal::Forward, Pedal::Reverse] {
            let rc = Some(rc_with(RcSwitch::Computer, pedal, true, RightStick::Centre));
            let out = car.drive_with_rc(1.0, true, rc, 100);
            assert_eq!(out.mode, Mode::Computer, "{pedal:?}");
            assert!(out.throttle > 0.0, "{pedal:?}");
        }
    }

    #[test]
    fn letting_go_early_starts_the_give_again() {
        let mut car = computer_driving();
        car.drive_with_rc(1.0, true, normal(RightStick::Stop), 1);
        car.drive_with_rc(1.0, true, normal(RightStick::Centre), 10);
        car.drive_with_rc(1.0, true, normal(RightStick::Give), 40);
        car.drive_with_rc(1.0, true, normal(RightStick::Centre), 1);
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Give), 50);
        assert_eq!(out.mode, Mode::Rc);
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Give), 1);
        assert_eq!(out.mode, Mode::Computer);
    }

    #[test]
    fn manual_ignores_the_give() {
        let mut car = with_receiver();
        car.clear_rc(RcSwitch::Direct);
        let give = Some(rc_with(
            RcSwitch::Direct,
            Pedal::Middle,
            false,
            RightStick::Give,
        ));
        car.drive_with_rc(0.0, false, give, 2);
        let out = car.drive_with_rc(1.0, true, give, 200);
        assert_eq!((out.mode, out.throttle), (Mode::Rc, 0.0));
    }

    #[test]
    fn rc_via_ros_the_right_stick_still_brakes_hard() {
        let mut car = Car::new();
        let via = |right| Some(rc_with(RcSwitch::ViaRos, Pedal::Middle, false, right));
        car.drive_with_rc(0.0, false, via(RightStick::Centre), 2);
        let out = car.drive_with_rc(1.0, true, via(RightStick::Centre), 50);
        assert!(out.throttle > 0.0);
        let out = car.drive_with_rc(1.0, true, via(RightStick::Stop), 1);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        // Let go: the computer drives on (RC via ROS is its, from the sticks it is sent).
        let out = car.drive_with_rc(1.0, true, via(RightStick::Centre), 50);
        assert_eq!(out.mode, Mode::Computer);
        assert!(out.throttle > 0.0);
    }

    #[test]
    fn manual_refuses_the_computer() {
        let mut car = with_receiver();
        car.clear_rc(RcSwitch::Direct);
        let b = Some(band(RcSwitch::Direct));
        car.drive_with_rc(0.0, false, b, 2);
        let out = car.drive_with_rc(1.0, true, b, 300);
        assert_eq!((out.mode, out.throttle), (Mode::Rc, 0.0));
    }

    #[test]
    fn rc_direct_from_via_ros_takes_over_but_waits_for_the_left_stick() {
        let mut car = Car::new();
        let via_ros = Some(sticks(RcSwitch::ViaRos));
        car.drive_with_rc(0.0, false, via_ros, 2);
        car.drive_with_rc(0.0, true, via_ros, 20);
        // Switched to manual with the stick pushed: RC has the car, braked.
        let out = car.drive_with_rc(0.0, true, Some(sticks(RcSwitch::Direct)), 30);
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        // The stick to the middle and out again: the sticks drive.
        car.clear_rc(RcSwitch::Direct);
        let out = car.drive_with_rc(0.0, true, Some(sticks(RcSwitch::Direct)), 30);
        assert_eq!(out.mode, Mode::Rc);
        assert!(out.throttle > 0.0, "the sticks drive");
    }

    #[test]
    fn rc_direct_waits_for_the_left_stick_after_power_up() {
        let mut car = Car::new();
        let pushed = Some(sticks(RcSwitch::Direct));
        let out = car.drive_with_rc(0.0, false, pushed, 150);
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        car.clear_rc(RcSwitch::Direct);
        assert!(car.drive_with_rc(0.0, false, pushed, 30).throttle > 0.0);
    }

    #[test]
    fn moving_the_mode_switch_holds_rc_direct_again() {
        let mut car = Car::new();
        car.clear_rc(RcSwitch::Direct);
        let pushed = Some(sticks(RcSwitch::Direct));
        assert!(car.drive_with_rc(0.0, false, pushed, 30).throttle > 0.0);
        // To normal and back, the stick still pushed: held until it moves.
        car.drive_with_rc(0.0, false, Some(band(RcSwitch::Computer)), 2);
        let out = car.drive_with_rc(0.0, false, pushed, 300);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
    }

    #[test]
    fn speed_setpoint_ramps() {
        let mut car = Car::new();
        car.arm();
        car.drive(3.0, 0.0, true, 11); // 100 ms of ramp at 1.5 m/s^2
        let sp = car.spine.speed_setpoint();
        assert!(sp > 0.1 && sp < 0.2, "{sp}");
    }

    #[test]
    fn reverse_is_capped_for_the_computer_and_the_rc() {
        // The computer asks for 2 m/s backwards: it gets the reverse cap.
        let mut car = Car::new();
        car.arm();
        car.drive(-2.0, 0.2, true, 300);
        assert!((car.spine.speed_setpoint() + MAX_REVERSE_SPEED).abs() < 1e-6);
        // Full reverse on the RC stick: the same cap.
        let mut car = Car::new();
        car.clear_rc(RcSwitch::Direct);
        let rc = RcInput {
            switch: RcSwitch::Direct,
            throttle: -1.0,
            steering: 0.0,
            middle: false,
            ..RcInput::default()
        };
        for _ in 0..300 {
            car.tick(Inputs {
                rc: Some(rc),
                ..Default::default()
            });
        }
        assert!((car.spine.speed_setpoint() + MAX_REVERSE_SPEED).abs() < 1e-6);
        assert_eq!(SpineConfig::default().max_reverse_speed, MAX_REVERSE_SPEED);
    }

    #[test]
    fn steering_is_clamped() {
        let mut car = Car::new();
        car.arm();
        let out = car.drive(0.5, 2.0, true, 300);
        assert!((out.steering_target - SpineConfig::default().max_steering_angle).abs() < 1e-6);
    }

    #[test]
    fn slows_down_faster_than_ego() {
        let mut car = Car::new();
        car.arm();
        car.drive(1.0, 0.0, true, 100); // up to 1 m/s
        assert!((car.spine.speed_setpoint() - 1.0).abs() < 1e-3);
        // ego stops at 2.0 m/s^2, so from 1 m/s it reaches 0 in 0.5 s. At 2.5 m/s^2
        // the firmware is already there after 0.4 s.
        car.drive(0.0, 0.0, true, 40);
        assert!(
            car.spine.speed_setpoint().abs() < 1e-3,
            "{}",
            car.spine.speed_setpoint()
        );
    }

    #[test]
    fn driver_fault_stops_and_needs_rearm() {
        let mut car = Car::new();
        car.arm();
        car.drive(1.0, 0.0, true, 20);
        let cmd = car.cmd(1.0, 0.0, true);
        let out = car.tick(Inputs {
            drive_cmd: Some(cmd),
            driver_faults: faults::WHEEL_ENCODER,
            ..Default::default()
        });
        assert_eq!(out.throttle, 0.0);
        assert_eq!(out.brake, 1.0);
        assert_ne!(out.faults & faults::WHEEL_ENCODER, 0);
        // The fault clears, but the car stays stopped until a fresh arm.
        let out = car.drive(1.0, 0.0, true, 5);
        assert_eq!(out.throttle, 0.0);
        assert!(out.estop_active);
        car.arm();
        assert_eq!(car.drive(1.0, 0.0, true, 5).mode, Mode::Computer);
    }

    #[test]
    fn driver_fault_stops_rc_too() {
        let mut car = Car::new();
        let rc = RcInput {
            switch: RcSwitch::Direct,
            throttle: 1.0,
            steering: 0.0,
            middle: false,
            ..RcInput::default()
        };
        let out = car.tick(Inputs {
            rc: Some(rc),
            driver_faults: faults::THROTTLE_DAC,
            ..Default::default()
        });
        assert_eq!(out.throttle, 0.0);
        assert_eq!(out.brake, 1.0);
    }

    #[test]
    fn repeated_counter_is_a_fault_and_not_fresh() {
        let mut car = Car::new();
        let cmd = car.cmd(1.0, 0.0, true);
        car.tick(Inputs {
            drive_cmd: Some(cmd),
            ..Default::default()
        });
        let out = car.tick(Inputs {
            drive_cmd: Some(cmd),
            ..Default::default()
        });
        assert_ne!(out.faults & faults::BAD_COUNTER, 0);
    }

    #[test]
    fn the_open_choices_are_what_the_docs_propose() {
        // docs/firmware.md, "Still to decide". Change them only with the team.
        assert_eq!(ESTOP_STEERING, EstopSteering::Hold);
        assert_eq!(IDLE_BRAKE, 0.0);
        let c = SpineConfig::default();
        assert_eq!(c.estop_steering, ESTOP_STEERING);
        assert_eq!(c.idle_brake, IDLE_BRAKE);
        // No receiver unless the board says so: the bench and the simulators have none.
        assert!(!c.rc_fitted);
    }

    /// Steer to 0.3 rad, then hold the hardware e-stop for `ticks`.
    fn steering_on_an_estop(cfg: SpineConfig, ticks: usize) -> std::vec::Vec<f32> {
        let mut car = Car::with_config(cfg);
        car.arm();
        let out = car.drive(0.0, 0.3, true, 100);
        assert!((out.steering_target - 0.3).abs() < 1e-6);
        let estop = Inputs {
            hw_estop: true,
            ..Default::default()
        };
        (0..ticks)
            .map(|_| car.tick(estop).steering_target)
            .collect()
    }

    #[test]
    fn an_estop_holds_the_steering() {
        let steering = steering_on_an_estop(SpineConfig::default(), 100);
        assert!(
            steering.iter().all(|s| (s - 0.3).abs() < 1e-6),
            "{steering:?}"
        );
    }

    #[test]
    fn an_estop_can_centre_the_steering_instead() {
        let cfg = SpineConfig {
            estop_steering: EstopSteering::Centre,
            ..SpineConfig::default()
        };
        let steering = steering_on_an_estop(cfg, 100);
        // Back at the steering rate limit, 0.6 rad/s: 6 mrad a tick, straight after 0.5 s.
        assert!((steering[0] - 0.294).abs() < 1e-4, "{}", steering[0]);
        assert!(steering.windows(2).all(|w| w[1] <= w[0]));
        assert_eq!(steering[60..], [0.0; 40]);
    }

    #[test]
    fn the_idle_brake_after_boot() {
        // Nobody drives straight after boot: the brake is released...
        let out = Car::new().tick(Inputs::default());
        assert_eq!((out.mode, out.brake), (Mode::Manual, 0.0));
        // ...or full on, as the 2025-26 firmware kept it.
        let cfg = SpineConfig {
            idle_brake: 1.0,
            ..SpineConfig::default()
        };
        assert_eq!(Car::with_config(cfg).tick(Inputs::default()).brake, 1.0);
    }

    #[test]
    fn the_right_stick_down_is_the_full_brake_at_once_and_still_steers() {
        // The RC drives; the right stick pulled down while turning left, the car still
        // rolling at 1 m/s: the full brake from the first tick.
        let mut car = Car::new();
        car.clear_rc(RcSwitch::Direct);
        let inputs = Inputs {
            rc: Some(RcInput {
                steering: 1.0,
                ..rc_with(RcSwitch::Direct, Pedal::Forward, false, RightStick::Stop)
            }),
            measured_speed: 1.0,
            ..Default::default()
        };
        let first = car.tick(inputs);
        assert_eq!(
            (first.mode, first.throttle, first.brake),
            (Mode::Rc, 0.0, 1.0)
        );
        let out = (0..10).map(|_| car.tick(inputs)).last().unwrap();
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        assert!(out.steering_target > 0.0, "the stick still steers");
    }

    #[test]
    fn status_frame_round_trips() {
        let mut car = Car::new();
        car.arm();
        let out = car.drive(1.0, 0.1, true, 5);
        let frame = car.spine.status_frame(&out);
        let back = VehicleStatus::decode(&frame).unwrap();
        assert_eq!(back.mode, Mode::Computer);
        assert!(back.armed);
    }

    // --- Open-loop commands (the 2025-26 UART link) and no wheel speed ---

    fn direct(throttle: f32, brake: f32, steering: f32) -> DirectCmd {
        DirectCmd {
            throttle,
            reverse: false,
            brake,
            steering_angle: steering,
            stop: brake > 0.0,
        }
    }

    impl Car {
        fn send_direct(&mut self, cmd: DirectCmd, ticks: u32) -> Outputs {
            let mut last = None;
            for _ in 0..ticks {
                last = Some(self.tick(Inputs {
                    direct_cmd: Some(cmd),
                    ..Default::default()
                }));
            }
            last.unwrap()
        }
    }

    #[test]
    fn direct_commands_need_a_stop_first() {
        let mut car = Car::new();
        // A drive frame straight after boot does nothing...
        let out = car.send_direct(direct(0.5, 0.0, 0.0), 5);
        assert_eq!((out.mode, out.throttle), (Mode::Manual, 0.0));
        // ...until a stop frame (the arming edge's first half), then a drive frame.
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        let out = car.send_direct(direct(0.5, 0.0, 0.0), 1);
        assert_eq!(out.mode, Mode::Computer);
        assert_eq!((out.throttle, out.brake), (0.5, 0.0));
    }

    #[test]
    fn direct_commands_pass_through_inside_limits() {
        let mut car = Car::new();
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        let out = car.send_direct(direct(3.0, 0.0, 0.0), 1);
        assert_eq!(out.throttle, 1.0); // clamped to 0..1
                                       // The brake wins over the throttle, and the stop frame's brake passes as it is.
        let out = car.send_direct(direct(0.5, 0.4, 0.0), 1);
        assert_eq!((out.throttle, out.brake), (0.0, 0.4));
        assert_eq!(out.mode, Mode::Computer); // a stop frame still counts as fresh
                                              // Reverse passes through.
        let mut cmd = direct(0.2, 0.0, 0.0);
        cmd.reverse = true;
        let out = car.send_direct(cmd, 1);
        assert!(out.reverse && out.throttle == 0.2);
    }

    #[test]
    fn direct_reverse_stays_under_the_reverse_speed() {
        // No wheel speed: capped at the throttle open loop gives MAX_REVERSE_SPEED.
        let mut car = Car::with_config(SpineConfig {
            speed_feedback: SpeedFeedback::None {
                full_throttle_speed: 3.0,
                start_throttle: 0.0,
                start_throttle_reverse: 0.0,
            },
            ..SpineConfig::default()
        });
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        let mut cmd = direct(1.0, 0.0, 0.0);
        cmd.reverse = true;
        let out = car.send_direct(cmd, 1);
        assert!(out.reverse);
        assert!((out.throttle - MAX_REVERSE_SPEED / 3.0).abs() < 1e-6);
        // Forward: capped the same way, at the throttle that gives MAX_SPEED.
        let out = car.send_direct(direct(1.0, 0.0, 0.0), 1);
        assert!((out.throttle - MAX_SPEED / 3.0).abs() < 1e-6);

        // With a wheel speed: cut once the car reverses at the limit.
        let mut car = Car::new();
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        let out = car.tick(Inputs {
            direct_cmd: Some(cmd),
            measured_speed: -MAX_REVERSE_SPEED,
            ..Default::default()
        });
        assert_eq!(out.throttle, 0.0);
    }

    #[test]
    fn direct_steering_is_rate_limited_and_capped() {
        let mut car = Car::new();
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        let out = car.send_direct(direct(0.0, 0.0, 2.0), 1);
        let cfg = SpineConfig::default();
        assert!(out.steering_target <= cfg.max_steer_rate * 0.01 + 1e-6);
        let out = car.send_direct(direct(0.0, 0.0, 2.0), 300);
        assert_eq!(out.steering_target, cfg.max_steering_angle);
    }

    #[test]
    fn direct_commands_time_out_into_a_full_stop() {
        let mut car = Car::new();
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        car.send_direct(direct(0.5, 0.0, 0.0), 1);
        let mut out = car.tick(Inputs::default());
        for _ in 0..25 {
            out = car.tick(Inputs::default());
        }
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_ne!(out.faults & faults::CMD_TIMEOUT, 0);
        // Drive frames alone don't restart it: a stop frame must come first.
        let out = car.send_direct(direct(0.5, 0.0, 0.0), 3);
        assert_eq!(out.throttle, 0.0);
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        assert_eq!(car.send_direct(direct(0.5, 0.0, 0.0), 1).throttle, 0.5);
    }

    #[test]
    fn rc_and_estop_beat_direct_commands() {
        let mut car = Car::new();
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        car.send_direct(direct(0.5, 0.0, 0.0), 1);
        let out = car.tick(Inputs {
            direct_cmd: Some(direct(0.5, 0.0, 0.0)),
            hw_estop: true,
            ..Default::default()
        });
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        let mut car = Car::new();
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        let out = car.tick(Inputs {
            direct_cmd: Some(direct(0.5, 0.0, 0.0)),
            rc: Some(RcInput {
                switch: RcSwitch::Direct,
                throttle: 0.0,
                steering: 0.0,
                middle: false,
                ..RcInput::default()
            }),
            ..Default::default()
        });
        assert_eq!((out.mode, out.throttle), (Mode::Rc, 0.0));
    }

    fn open_loop_config() -> SpineConfig {
        SpineConfig {
            speed_feedback: SpeedFeedback::None {
                full_throttle_speed: 3.0,
                start_throttle: 0.0,
                start_throttle_reverse: 0.0,
            },
            ..SpineConfig::default()
        }
    }

    #[test]
    fn the_open_loop_throttle_cap_is_the_limit_over_full_throttle_speed() {
        let c = open_loop_config();
        assert_eq!(c.open_loop_throttle_cap(false), Some(MAX_SPEED / 3.0));
        assert_eq!(
            c.open_loop_throttle_cap(true),
            Some(MAX_REVERSE_SPEED / 3.0)
        );
        assert_eq!(SpineConfig::default().open_loop_throttle_cap(false), None);
        let no_map = SpineConfig {
            speed_feedback: SpeedFeedback::None {
                full_throttle_speed: 0.0,
                start_throttle: 0.0,
                start_throttle_reverse: 0.0,
            },
            ..c
        };
        assert_eq!(no_map.open_loop_throttle_cap(false), Some(0.0));
        // The spine caps a full-throttle direct command at exactly this.
        let mut spine = Spine::new(c);
        let cmd = |stop| DirectCmd {
            throttle: 1.0,
            reverse: false,
            brake: 0.0,
            steering_angle: 0.0,
            stop,
        };
        for (t, stop) in [(0, true), (10, false)] {
            spine.step(&Inputs {
                now_ms: t,
                direct_cmd: Some(cmd(stop)),
                ..Inputs::default()
            });
        }
        let out = spine.step(&Inputs {
            now_ms: 20,
            direct_cmd: Some(cmd(false)),
            ..Inputs::default()
        });
        assert_eq!(out.throttle, c.open_loop_throttle_cap(false).unwrap());
    }

    #[test]
    fn fault_names_are_the_bits_set_lowest_first() {
        let names: std::vec::Vec<_> =
            faults::names(faults::RC_SIGNAL_LOST | faults::CMD_TIMEOUT | 0x8000).collect();
        assert_eq!(names, ["CMD_TIMEOUT", "RC_SIGNAL_LOST"]);
        assert_eq!(faults::names(0).count(), 0);
        let all = faults::NAMES.iter().fold(0, |bits, (bit, _)| bits | bit);
        assert_eq!(all, 0x03FF, "every bit has one name");
    }

    #[test]
    fn without_a_wheel_speed_the_throttle_follows_the_setpoint() {
        let mut car = Car::with_config(open_loop_config());
        car.arm();
        // 0.75 m/s of 3.0: the setpoint ramps at 1.5 m/s^2, so after 0.5 s it is there.
        let out = car.drive(0.75, 0.0, true, 200);
        assert_eq!(out.mode, Mode::Computer);
        assert!((out.throttle - 0.25).abs() < 1e-3, "{}", out.throttle);
        assert_eq!(out.brake, 0.0);
        // Back to zero: the throttle falls with the setpoint, then the hold brake.
        let out = car.drive(0.0, 0.0, true, 200);
        assert_eq!(out.throttle, 0.0);
        assert_eq!(out.brake, SpeedLoopConfig::default().hold_brake);
    }

    #[test]
    fn without_a_wheel_speed_there_is_no_overspeed() {
        let mut car = Car::with_config(open_loop_config());
        let out = car.tick(Inputs {
            measured_speed: 20.0,
            ..Default::default()
        });
        assert_eq!(out.faults & faults::OVERSPEED, 0);
    }

    // The state machine (`crate::fsm`): the self-test, the test mode, the reported state.

    impl Car {
        /// One test request a tick, for `ticks` ticks.
        fn test(&mut self, req: TestRequest, ticks: u32) -> Outputs {
            let mut last = None;
            for _ in 0..ticks {
                last = Some(self.tick(Inputs {
                    test_cmd: Some(req),
                    ..Default::default()
                }));
            }
            last.unwrap()
        }

        /// Exit, then Enter: into the test mode.
        fn enter_test(&mut self) -> Outputs {
            self.test(TestRequest::Exit, 2);
            self.test(TestRequest::Enter, 1)
        }

        fn state(&self) -> State {
            self.spine.state()
        }
    }

    #[test]
    fn the_regions_after_boot() {
        let mut car = Car::new();
        assert_eq!(car.state().lifecycle, Lifecycle::Boot);
        car.tick(Inputs::default());
        let s = car.state();
        assert_eq!(s.lifecycle, Lifecycle::SelfTest);
        assert_eq!(s.safety, Safety::Latched);
        assert_eq!(s.authority, Authority::Idle);
        assert_eq!(s.motion, Motion::StandstillHold);
        assert_eq!(s.confidence, Confidence::Full);
        car.tick(Inputs::default());
        assert_eq!(car.state().lifecycle, Lifecycle::Running);
    }

    #[test]
    fn the_self_test_waits_for_the_devices_and_nothing_arms_before_it() {
        let mut car = Car::new();
        let missing = Inputs {
            driver_faults: faults::STEERING_DRIVER | faults::BRAKE_DRIVER,
            ..Default::default()
        };
        for _ in 0..5 {
            car.tick(missing);
        }
        assert_eq!(car.state().lifecycle, Lifecycle::SelfTest);
        assert_eq!(car.state().safety, Safety::Faulted);
        // The devices answer: the self-test passes, then the computer can arm.
        car.arm();
        assert_eq!(car.state().lifecycle, Lifecycle::Running);
        let out = car.drive(1.0, 0.0, true, 5);
        assert_eq!(out.mode, Mode::Computer);
        assert_eq!(car.state().safety, Safety::Ok);
    }

    #[test]
    fn open_loop_is_reported_as_degraded() {
        let mut car = Car::with_config(open_loop_config());
        car.tick(Inputs::default());
        assert_eq!(car.state().confidence, Confidence::Degraded);
    }

    #[test]
    fn motion_follows_the_drive() {
        let mut car = Car::new();
        car.arm();
        car.drive(1.0, 0.0, true, 50);
        assert_eq!(car.state().motion, Motion::Forward);
    }

    #[test]
    fn the_test_mode_needs_exit_then_enter_and_holds_still() {
        let mut car = Car::new();
        // Enter alone does nothing.
        let out = car.test(TestRequest::Enter, 3);
        assert_eq!(out.mode, Mode::Manual);
        let out = car.enter_test();
        assert_eq!(out.mode, Mode::Test);
        assert_eq!(car.state().authority, Authority::Test(TestStage::Armed));
        assert_eq!(out.throttle, 0.0);
        assert_eq!(out.brake, SpineConfig::default().stop_brake);
        assert!(!out.armed);
    }

    #[test]
    fn the_test_mode_moves_one_actuator_at_a_time() {
        let mut car = Car::new();
        car.enter_test();

        // Steering: rate limited toward the target, the brake stays on.
        let out = car.test(TestRequest::Steering(0.3), 100);
        assert_eq!(car.state().authority, Authority::Test(TestStage::Active));
        assert!(
            (out.steering_target - 0.3).abs() < 1e-4,
            "{}",
            out.steering_target
        );
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        // Past the limit: clamped.
        let out = car.test(TestRequest::Steering(2.0), 100);
        assert!((out.steering_target - MAX_STEERING_ANGLE).abs() < 1e-4);

        // Brake: to the position asked for, no throttle.
        let out = car.test(TestRequest::Brake(0.4), 1);
        assert_eq!((out.throttle, out.brake), (0.0, 0.4));

        // Throttle: capped, forward, brake off.
        let out = car.test(TestRequest::Throttle(0.9), 1);
        assert_eq!(out.throttle, TEST_MAX_THROTTLE);
        assert_eq!(out.brake, 0.0);
        assert!(!out.reverse);

        // Hold: back to Armed, holding still, the steering where it is.
        let out = car.test(TestRequest::Hold, 1);
        assert_eq!(car.state().authority, Authority::Test(TestStage::Armed));
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert!((out.steering_target - MAX_STEERING_ANGLE).abs() < 1e-4);
    }

    #[test]
    fn a_stale_test_request_leaves_the_test_mode_and_latches() {
        let mut car = Car::new();
        car.enter_test();
        car.test(TestRequest::Throttle(0.1), 5);
        // 100 ms with no request: out, braked, latched with a command timeout.
        let mut out = car.tick(Inputs::default());
        for _ in 0..11 {
            out = car.tick(Inputs::default());
        }
        assert_eq!(out.mode, Mode::Manual);
        assert_eq!(out.throttle, 0.0);
        assert_eq!(out.brake, SpineConfig::default().stop_brake);
        assert_ne!(out.faults & faults::CMD_TIMEOUT, 0);
        assert_eq!(car.state().safety, Safety::Latched);
        // The computer must disarm, then arm, as after any timeout.
        assert_eq!(car.drive(1.0, 0.0, true, 3).mode, Mode::Manual);
        car.arm();
        assert_eq!(car.drive(1.0, 0.0, true, 3).mode, Mode::Computer);
    }

    #[test]
    fn the_test_mode_leaves_on_an_estop_a_fault_rc_or_speed() {
        let unsafe_inputs = [
            Inputs {
                hw_estop: true,
                ..Default::default()
            },
            Inputs {
                driver_faults: faults::BRAKE_DRIVER,
                ..Default::default()
            },
            Inputs {
                measured_speed: TEST_MAX_SPEED + 0.1,
                ..Default::default()
            },
        ];
        for bad in unsafe_inputs {
            let mut car = Car::new();
            car.enter_test();
            car.test(TestRequest::Throttle(0.1), 3);
            let out = car.tick(Inputs {
                test_cmd: Some(TestRequest::Throttle(0.1)),
                ..bad
            });
            assert_ne!(out.mode, Mode::Test, "{bad:?}");
            assert_eq!(out.throttle, 0.0, "{bad:?}");
            assert!(car.spine.latch.latched, "{bad:?}");
        }

        // RC direct: the person on the transmitter takes over at once, and the computer
        // stays latched for later.
        let mut car = Car::new();
        car.enter_test();
        let out = car.tick(Inputs {
            test_cmd: Some(TestRequest::Throttle(0.1)),
            rc: Some(sticks(RcSwitch::Direct)),
            ..Default::default()
        });
        assert_eq!(out.mode, Mode::Rc);
        assert!(car.spine.latch.latched);
    }

    #[test]
    fn an_exit_request_leaves_the_test_mode_latched() {
        let mut car = Car::new();
        car.enter_test();
        let out = car.test(TestRequest::Exit, 1);
        assert_eq!(out.mode, Mode::Manual);
        assert_eq!(car.state().safety, Safety::Latched);
        assert_eq!(out.faults & faults::CMD_TIMEOUT, 0);
    }

    #[test]
    fn no_test_mode_while_the_computer_drives_or_the_car_moves() {
        let mut car = Car::new();
        car.arm();
        car.drive(1.0, 0.0, true, 5);
        car.test(TestRequest::Exit, 1);
        let cmd = car.cmd(1.0, 0.0, true);
        let out = car.tick(Inputs {
            drive_cmd: Some(cmd),
            test_cmd: Some(TestRequest::Enter),
            ..Default::default()
        });
        assert_eq!(out.mode, Mode::Computer);

        let mut car = Car::new();
        car.test(TestRequest::Exit, 2);
        let out = car.tick(Inputs {
            test_cmd: Some(TestRequest::Enter),
            measured_speed: 0.3,
            ..Default::default()
        });
        assert_eq!(out.mode, Mode::Manual);
    }

    #[test]
    fn drive_commands_do_not_move_the_car_in_the_test_mode() {
        let mut car = Car::new();
        car.arm();
        car.enter_test();
        for _ in 0..5 {
            let cmd = car.cmd(2.0, 0.4, true);
            let out = car.tick(Inputs {
                drive_cmd: Some(cmd),
                test_cmd: Some(TestRequest::Hold),
                ..Default::default()
            });
            assert_eq!(out.mode, Mode::Test);
            assert_eq!(out.throttle, 0.0);
        }
    }

    // The boot self-test, with each fault it must catch injected.

    /// The faults that stop the car: each one alone must hold the self-test.
    const STOP_FAULTS: [u16; 4] = [
        faults::STEERING_DRIVER,
        faults::BRAKE_DRIVER,
        faults::THROTTLE_DAC,
        faults::WHEEL_ENCODER,
    ];

    #[test]
    fn the_self_test_catches_each_stop_fault() {
        for fault in STOP_FAULTS {
            let mut car = Car::new();
            let faulty = Inputs {
                driver_faults: fault,
                ..Default::default()
            };
            for _ in 0..5 {
                let out = car.tick(faulty);
                assert_eq!((out.throttle, out.brake), (0.0, 1.0), "{fault}");
                assert_ne!(out.faults & fault, 0, "{fault}");
            }
            assert_eq!(car.state().lifecycle, Lifecycle::SelfTest, "{fault}");
            assert_eq!(car.state().safety, Safety::Faulted, "{fault}");
            // The computer's arming edge, while the device is still faulty: no drive.
            for armed in [false, false, true, true, true] {
                let cmd = car.cmd(1.0, 0.0, armed);
                let out = car.tick(Inputs {
                    drive_cmd: Some(cmd),
                    ..faulty
                });
                assert_ne!(out.mode, Mode::Computer, "{fault}");
                assert_eq!(out.throttle, 0.0, "{fault}");
            }
            // Nor the test mode.
            for req in [TestRequest::Exit, TestRequest::Exit, TestRequest::Enter] {
                car.tick(Inputs {
                    test_cmd: Some(req),
                    ..faulty
                });
            }
            assert!(!car.state().authority.in_test(), "{fault}");
            assert_eq!(car.state().lifecycle, Lifecycle::SelfTest, "{fault}");
        }
    }

    #[test]
    fn the_self_test_passes_once_the_fault_clears_and_still_needs_an_arm() {
        for fault in STOP_FAULTS {
            let mut car = Car::new();
            for _ in 0..3 {
                car.tick(Inputs {
                    driver_faults: fault,
                    ..Default::default()
                });
            }
            car.tick(Inputs::default());
            assert_eq!(car.state().lifecycle, Lifecycle::Running, "{fault}");
            // Armed commands alone don't drive: the fault latched.
            assert_eq!(car.drive(1.0, 0.0, true, 3).mode, Mode::Manual, "{fault}");
            car.arm();
            assert_eq!(car.drive(1.0, 0.0, true, 3).mode, Mode::Computer, "{fault}");
        }
    }

    #[test]
    fn a_fault_on_the_first_tick_alone_still_latches() {
        // The Boot tick moves to SelfTest whatever it sees, but the fault still latches.
        let mut car = Car::new();
        car.tick(Inputs {
            driver_faults: faults::BRAKE_DRIVER,
            ..Default::default()
        });
        assert!(car.spine.latch.fault);
        car.tick(Inputs::default());
        assert_eq!(car.state().lifecycle, Lifecycle::Running);
        assert_eq!(car.drive(1.0, 0.0, true, 3).mode, Mode::Manual);
    }

    #[test]
    fn faults_that_do_not_stop_the_car_do_not_hold_the_self_test() {
        // CAN_BUS and the RC are not devices that stop the car (`faults::STOP`).
        let mut car = Car::new();
        for _ in 0..2 {
            car.tick(Inputs {
                driver_faults: faults::CAN_BUS,
                ..Default::default()
            });
        }
        assert_eq!(car.state().lifecycle, Lifecycle::Running);
    }

    #[test]
    fn a_fault_after_the_self_test_is_the_safety_regions_not_a_new_test() {
        let mut car = Car::new();
        car.arm();
        car.drive(1.0, 0.0, true, 10);
        let out = car.tick(Inputs {
            driver_faults: faults::THROTTLE_DAC,
            ..Default::default()
        });
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_eq!(car.state().lifecycle, Lifecycle::Running);
        assert_eq!(car.state().safety, Safety::Faulted);
    }

    #[test]
    fn without_a_wheel_speed_the_test_mode_refuses_the_throttle() {
        // Today's board (esp32-umod) has no wheel speed: the "faster than
        // TEST_MAX_SPEED" exit can never fire there, so no throttle test either.
        let mut car = Car::with_config(open_loop_config());
        car.tick(Inputs::default());
        car.enter_test();
        let out = car.test(TestRequest::Throttle(0.1), 5);
        assert_eq!(out.mode, Mode::Test);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_eq!(car.state().authority, Authority::Test(TestStage::Armed));
        // The brake and the steering still work.
        let out = car.test(TestRequest::Brake(0.4), 1);
        assert_eq!((out.throttle, out.brake), (0.0, 0.4));
        let out = car.test(TestRequest::Steering(0.3), 100);
        assert!((out.steering_target - 0.3).abs() < 1e-4);
        assert_eq!(car.state().authority, Authority::Test(TestStage::Active));
    }

    // Bugs found in review of the state machine, each with the test that showed it.

    #[test]
    fn no_test_mode_on_the_tick_the_computer_arms() {
        // The entry check looked at last tick's authority (Idle), so an Enter on the very
        // tick the computer's arm released the latch went into the test mode, armed.
        let mut car = Car::new();
        car.test(TestRequest::Exit, 1);
        car.drive(0.0, 0.0, false, 2);
        let cmd = car.cmd(1.0, 0.0, true);
        let out = car.tick(Inputs {
            drive_cmd: Some(cmd),
            test_cmd: Some(TestRequest::Enter),
            ..Default::default()
        });
        assert_eq!(out.mode, Mode::Computer);
        assert!(!car.state().authority.in_test());
    }

    #[test]
    fn an_exit_before_an_estop_does_not_count_for_the_entry_after_it() {
        // Like the computer's disarm, the first half of the entry must come after the
        // latest e-stop or fault: an old Exit must not complete a new Enter.
        let unsafe_inputs = [
            Inputs {
                hw_estop: true,
                ..Default::default()
            },
            Inputs {
                driver_faults: faults::STEERING_DRIVER,
                ..Default::default()
            },
        ];
        for bad in unsafe_inputs {
            let mut car = Car::new();
            car.tick(Inputs::default());
            car.test(TestRequest::Exit, 1);
            for _ in 0..3 {
                car.tick(bad);
            }
            car.tick(Inputs::default());
            let out = car.test(TestRequest::Enter, 1);
            assert_eq!(out.mode, Mode::Manual, "{bad:?}");
            // A fresh Exit, then Enter, still works.
            assert_eq!(car.enter_test().mode, Mode::Test, "{bad:?}");
        }
    }

    #[test]
    fn the_computer_stopping_while_rc_direct_drives_still_latches() {
        // The timeout latched only while the computer drove. Armed, then overridden by RC
        // direct, then silent, it came back on the computer position without a re-arm.
        let mut car = Car::new();
        let computer = Some(band(RcSwitch::Computer));
        car.drive_with_rc(0.0, false, computer, 2);
        car.drive_with_rc(1.0, true, normal(RightStick::Give), 51);
        assert_eq!(
            car.drive_with_rc(1.0, true, computer, 10).mode,
            Mode::Computer
        );
        // The person takes over; the computer's stream stops (a crash, a reboot).
        let direct = RcInput {
            throttle: 0.0,
            ..sticks(RcSwitch::Direct)
        };
        for _ in 0..30 {
            car.tick(Inputs {
                rc: Some(direct),
                ..Default::default()
            });
        }
        // Back on the computer position; the computer sends armed commands again. The RC
        // has the car (the mode switch moved); given back, the computer is latched.
        let out = car.drive_with_rc(1.0, true, computer, 5);
        assert_eq!(out.throttle, 0.0);
        assert_ne!(out.faults & faults::CMD_TIMEOUT, 0);
        let out = car.drive_with_rc(1.0, true, computer, 100);
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        let out = car.drive_with_rc(1.0, true, normal(RightStick::Give), 51);
        assert_eq!((out.mode, out.throttle), (Mode::Manual, 0.0));
        car.drive_with_rc(0.0, false, computer, 2);
        assert_eq!(
            car.drive_with_rc(1.0, true, computer, 5).mode,
            Mode::Computer
        );
    }

    // --- The stop rules (T3.6, Arsh, 6 Nov) ---

    #[test]
    fn the_shared_limit_caps_the_computer_and_the_rc() {
        assert_eq!(SpineConfig::default().max_speed, MAX_SPEED);
        let mut car = Car::with_config(open_loop_config());
        car.arm();
        let out = car.drive(5.0, 0.0, true, 300);
        assert!(
            (out.throttle - MAX_SPEED / 3.0).abs() < 1e-3,
            "{}",
            out.throttle
        );
        let mut car = Car::with_config(open_loop_config());
        car.clear_rc(RcSwitch::Direct);
        let rc = RcInput {
            switch: RcSwitch::Direct,
            throttle: 1.0,
            steering: 0.0,
            middle: false,
            ..RcInput::default()
        };
        let out = (0..300)
            .map(|_| {
                car.tick(Inputs {
                    rc: Some(rc),
                    ..Default::default()
                })
            })
            .last()
            .unwrap();
        assert!(
            (out.throttle - MAX_SPEED / 3.0).abs() < 1e-3,
            "{}",
            out.throttle
        );
    }

    fn direct_with_speed(car: &mut Car, throttle: f32, report: SpeedReport) -> Outputs {
        car.tick(Inputs {
            direct_cmd: Some(direct(throttle, 0.0, 0.2)),
            speed_report: Some(report),
            ..Default::default()
        })
    }

    /// Open loop, armed and driving, with the link sending a speed of 0.5 m/s.
    fn driving_car() -> Car {
        let mut car = Car::with_config(open_loop_config());
        car.tick(Inputs {
            direct_cmd: Some(direct(0.0, 1.0, 0.0)),
            speed_report: Some(SpeedReport::Known(0.0)),
            ..Default::default()
        });
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(0.5));
        assert_eq!((out.mode, out.throttle), (Mode::Computer, 0.2));
        car
    }

    #[test]
    fn a_link_that_never_sends_a_speed_leaves_the_rules_off() {
        let mut car = Car::with_config(open_loop_config());
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        let out = car.send_direct(direct(0.2, 0.0, 0.0), 100);
        assert_eq!((out.throttle, out.brake), (0.2, 0.0));
        assert_eq!(out.faults & (faults::WHEEL_ENCODER | faults::OVERSPEED), 0);
        assert_eq!(car.spine.state().confidence, Confidence::Degraded);
        assert_eq!(car.spine.state().safety, Safety::Ok);
    }

    #[test]
    fn a_speed_from_the_link_gives_full_confidence_without_closing_the_loop() {
        let mut car = driving_car();
        assert_eq!(car.spine.state().confidence, Confidence::Full);
        // Still open loop: the throttle is what was asked for, whatever the speed says.
        let out = direct_with_speed(&mut car, 0.25, SpeedReport::Known(0.0));
        assert_eq!(out.throttle, 0.25);
    }

    #[test]
    fn the_overshoot_at_the_limit_does_not_trip() {
        let mut car = driving_car();
        for _ in 0..500 {
            let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(1.15));
            assert_eq!(out.throttle, 0.2);
            assert_eq!(out.faults & faults::OVERSPEED, 0);
        }
    }

    #[test]
    fn overspeed_cuts_the_throttle_and_brakes_until_under_the_limit_then_drives_on() {
        let mut car = driving_car();
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(1.3));
        assert_eq!((out.throttle, out.brake), (0.0, 0.5));
        assert_ne!(out.faults & faults::OVERSPEED, 0);
        assert_eq!(car.spine.state().safety, Safety::Overspeed);
        assert!(out.steering_target > 0.0, "the driver still steers");
        // Under the trip but still over the limit: keep slowing.
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(1.1));
        assert_eq!((out.throttle, out.brake), (0.0, 0.5));
        // Under the limit: it aimed for the limit, not a stop, and no re-arm is needed.
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(0.95));
        assert_eq!(
            (out.mode, out.throttle, out.brake),
            (Mode::Computer, 0.2, 0.0)
        );
        assert_eq!(out.faults & faults::OVERSPEED, 0);
    }

    fn hold_speed_past_the_window(car: &mut Car, speed: f32) -> Outputs {
        let ticks = crate::fsm::OVERSPEED_WINDOW_MS / 10;
        for _ in 0..ticks {
            let out = direct_with_speed(car, 0.2, SpeedReport::Known(speed));
            assert_eq!(
                out.brake, 0.5,
                "only the overspeed brake, inside the window"
            );
        }
        direct_with_speed(car, 0.2, SpeedReport::Known(speed))
    }

    #[test]
    fn an_overspeed_that_holds_steady_for_two_seconds_is_a_hard_stop_and_latches() {
        let mut car = driving_car();
        let out = hold_speed_past_the_window(&mut car, 1.3);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_ne!(out.faults & faults::OVERSPEED, 0);
        assert!(out.estop_active);
        // Back under the limit, it stays stopped until a deliberate re-arm.
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(0.0));
        assert_eq!(
            (out.mode, out.throttle, out.brake),
            (Mode::Manual, 0.0, 1.0)
        );
        assert_eq!(car.spine.state().safety, Safety::Latched);
        car.tick(Inputs {
            direct_cmd: Some(direct(0.0, 1.0, 0.0)),
            speed_report: Some(SpeedReport::Known(0.0)),
            ..Default::default()
        });
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(0.0));
        assert_eq!((out.mode, out.throttle), (Mode::Computer, 0.2));
        assert_eq!(out.faults & faults::OVERSPEED, 0);
    }

    #[test]
    fn an_overspeed_still_speeding_up_is_a_hard_stop_too() {
        let mut car = driving_car();
        for tick in 0..=(crate::fsm::OVERSPEED_WINDOW_MS / 10) {
            direct_with_speed(&mut car, 0.2, SpeedReport::Known(1.3 + tick as f32 * 0.01));
        }
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(5.0));
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
    }

    #[test]
    fn overspeed_counts_in_reverse_at_the_reverse_limit() {
        let mut car = driving_car();
        // 0.5 m/s backwards is the limit; past 0.75 trips it.
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(-0.7));
        assert_eq!(out.faults & faults::OVERSPEED, 0);
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(-0.8));
        assert_ne!(out.faults & faults::OVERSPEED, 0);
        assert_eq!(out.throttle, 0.0);
    }

    #[test]
    fn overspeed_works_with_a_wheel_speed_on_the_board_too() {
        let mut car = Car::new();
        car.arm();
        car.drive(0.5, 0.0, true, 10);
        let cmd = car.cmd(0.5, 0.0, true);
        let out = car.tick(Inputs {
            drive_cmd: Some(cmd),
            measured_speed: 1.3,
            ..Default::default()
        });
        assert_eq!((out.throttle, out.brake), (0.0, 0.5));
    }

    #[test]
    fn a_lost_speed_is_a_hard_stop_and_latches() {
        let mut car = driving_car();
        // 300 ms is allowed; more is a hard stop, even with fresh commands.
        for _ in 0..30 {
            let out = car.send_direct(direct(0.2, 0.0, 0.0), 1);
            assert_eq!(out.throttle, 0.2);
        }
        let out = car.send_direct(direct(0.2, 0.0, 0.0), 1);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_ne!(out.faults & faults::WHEEL_ENCODER, 0);
        assert_eq!(car.spine.state().safety, Safety::Degraded);
        assert_eq!(car.spine.state().confidence, Confidence::Degraded);
        // The speed back: still stopped until a deliberate re-arm.
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(0.0));
        assert_eq!((out.mode, out.throttle), (Mode::Manual, 0.0));
        assert_eq!(out.brake, 1.0);
        car.tick(Inputs {
            direct_cmd: Some(direct(0.0, 1.0, 0.0)),
            speed_report: Some(SpeedReport::Known(0.0)),
            ..Default::default()
        });
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(0.0));
        assert_eq!((out.mode, out.throttle), (Mode::Computer, 0.2));
        assert_eq!(out.faults & faults::WHEEL_ENCODER, 0);
    }

    #[test]
    fn no_re_arm_while_the_speed_is_still_lost() {
        let mut car = driving_car();
        car.send_direct(direct(0.2, 0.0, 0.0), 40);
        car.send_direct(direct(0.0, 1.0, 0.0), 1);
        let out = car.send_direct(direct(0.2, 0.0, 0.0), 1);
        assert_eq!(
            (out.mode, out.throttle, out.brake),
            (Mode::Manual, 0.0, 1.0)
        );
    }

    #[test]
    fn a_stopped_car_reporting_zero_is_not_a_lost_speed() {
        let mut car = driving_car();
        for _ in 0..500 {
            let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(0.0));
            assert_eq!(out.faults & faults::WHEEL_ENCODER, 0);
            assert_eq!(out.throttle, 0.2);
        }
    }

    #[test]
    fn a_sender_that_does_not_know_the_speed_counts_as_lost() {
        let mut car = driving_car();
        for _ in 0..31 {
            direct_with_speed(&mut car, 0.2, SpeedReport::Unknown);
        }
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Unknown);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        let out = direct_with_speed(&mut car, 0.2, SpeedReport::Known(f32::NAN));
        assert_eq!((out.throttle, out.brake), (0.0, 1.0), "NaN is no speed");
    }

    fn rc_direct(pedal: Pedal, full: bool) -> RcInput {
        rc_with(RcSwitch::Direct, pedal, full, RightStick::Centre)
    }

    /// `n` ticks of RC direct, the board's wheel speed at `speed`.
    fn rc_ticks(car: &mut Car, rc: RcInput, speed: f32, n: u32) -> Outputs {
        (0..n)
            .map(|_| {
                car.tick(Inputs {
                    rc: Some(rc),
                    measured_speed: speed,
                    ..Default::default()
                })
            })
            .last()
            .unwrap()
    }

    /// The ESP32's open loop (Arsh, 27 Nov): the wheels start at 475 of the 700 cap
    /// forward, 525 in reverse.
    fn start_config() -> SpineConfig {
        SpineConfig {
            speed_feedback: SpeedFeedback::None {
                full_throttle_speed: 3.0,
                start_throttle: 475.0 / 700.0,
                start_throttle_reverse: 525.0 / 700.0,
            },
            rc_fitted: true,
            ..SpineConfig::default()
        }
    }

    #[test]
    fn with_no_wheel_speed_the_rc_never_asks_for_a_throttle_that_cannot_turn_the_wheels() {
        let (start, start_reverse) = (475.0 / 700.0, 525.0 / 700.0);
        let mut car = Car::with_config(start_config());
        car.clear_rc(RcSwitch::Direct);
        // The whole ramp, forward: either no throttle (the stop at its start) or at least
        // the start, rising to 1.0 m/s's throttle, 550 of 700.
        let mut last = 0.0;
        for _ in 0..300 {
            let out = rc_ticks(&mut car, rc_direct(Pedal::Forward, true), 0.0, 1);
            assert!(
                out.throttle == 0.0 || out.throttle >= start,
                "{}",
                out.throttle
            );
            assert!(out.throttle >= last, "the ramp only rises");
            last = out.throttle;
        }
        assert!((last * 700.0 - 550.0).abs() < 0.5, "{}", last * 700.0);
        // Back, after the stop brake held a second: the reverse ramp, from its own start
        // to 0.5 m/s's throttle, 554 of 700.
        rc_ticks(&mut car, rc_direct(Pedal::Reverse, true), 0.0, 120);
        let mut last = 0.0;
        for _ in 0..300 {
            let out = rc_ticks(&mut car, rc_direct(Pedal::Reverse, true), 0.0, 1);
            assert!(
                out.throttle == 0.0 || (out.reverse && out.throttle >= start_reverse),
                "{out:?}"
            );
            last = out.throttle;
        }
        assert!((last * 700.0 - 554.2).abs() < 0.5, "{}", last * 700.0);
    }

    #[test]
    fn the_open_loop_cap_starts_where_the_wheels_turn_too() {
        let c = start_config();
        let cap = |reverse| c.open_loop_throttle_cap(reverse).unwrap() * 700.0;
        assert!((cap(false) - 550.0).abs() < 0.5, "{}", cap(false));
        assert!((cap(true) - 554.2).abs() < 0.5, "{}", cap(true));
    }

    #[test]
    fn the_rc_drives_at_its_fixed_speed_ramped_at_half_a_metre_per_second_squared() {
        let mut car = Car::new();
        car.clear_rc(RcSwitch::Direct);
        // Just out of the middle: the same speed as at the end, ramped.
        let out = rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 100);
        let sp = car.spine.speed_setpoint();
        assert!((sp - 0.5).abs() < 0.02, "{sp}");
        assert!(out.throttle > 0.0);
        rc_ticks(&mut car, rc_direct(Pedal::Forward, true), 0.0, 120);
        assert!((car.spine.speed_setpoint() - MAX_SPEED).abs() < 1e-6);
        assert_eq!(RC_ACCEL, 0.5);
    }

    #[test]
    fn the_middle_is_throttle_off_with_no_brake_and_still_steers() {
        let mut car = Car::new();
        car.clear_rc(RcSwitch::Direct);
        rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 200);
        let middle = RcInput {
            steering: 1.0,
            ..rc_direct(Pedal::Middle, false)
        };
        for _ in 0..100 {
            let out = rc_ticks(&mut car, middle, 0.8, 1);
            assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 0.0));
        }
        assert!(rc_ticks(&mut car, middle, 0.8, 1).steering_target > 0.0);
    }

    #[test]
    fn the_other_way_while_moving_lets_the_motor_slow_it_and_brakes_hard_past_88() {
        let mut car = Car::new();
        car.clear_rc(RcSwitch::Direct);
        rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.8, 200);
        // Pulled back, short of the end, the car still rolling forward: throttle off only.
        let out = rc_ticks(&mut car, rc_direct(Pedal::Reverse, false), 0.8, 100);
        assert_eq!((out.throttle, out.brake, out.reverse), (0.0, 0.0, false));
        // To the end: the hard brake too.
        let out = rc_ticks(&mut car, rc_direct(Pedal::Reverse, true), 0.6, 1);
        assert_eq!((out.throttle, out.brake, out.reverse), (0.0, 1.0, false));
        // Stopped, but not yet for half a second: still braked.
        let out = rc_ticks(&mut car, rc_direct(Pedal::Reverse, true), 0.0, 50);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        // Then it reverses.
        let out = rc_ticks(&mut car, rc_direct(Pedal::Reverse, true), 0.0, 50);
        assert!(out.reverse && out.throttle > 0.0, "{out:?}");
    }

    #[test]
    fn with_no_speed_only_the_hard_brake_held_a_second_changes_direction() {
        let mut car = Car::with_config(open_loop_config());
        car.clear_rc(RcSwitch::Direct);
        rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 100);
        // Pulled back short of the end: throttle off, and it never reverses.
        let out = rc_ticks(&mut car, rc_direct(Pedal::Reverse, false), 0.0, 300);
        assert_eq!((out.throttle, out.brake, out.reverse), (0.0, 0.0, false));
        // To the end: the hard brake for a second, then reverse.
        for n in 0..101 {
            let out = rc_ticks(&mut car, rc_direct(Pedal::Reverse, true), 0.0, 1);
            assert_eq!((out.throttle, out.brake), (0.0, 1.0), "tick {n}");
        }
        let out = rc_ticks(&mut car, rc_direct(Pedal::Reverse, true), 0.0, 20);
        assert!(out.reverse && out.throttle > 0.0, "{out:?}");
    }

    #[test]
    fn the_same_way_again_drives_at_once() {
        let mut car = Car::with_config(open_loop_config());
        car.clear_rc(RcSwitch::Direct);
        rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 100);
        let out = rc_ticks(&mut car, rc_direct(Pedal::Middle, false), 0.0, 30);
        assert_eq!((out.throttle, out.brake), (0.0, 0.0));
        let out = rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 20);
        assert!(out.throttle > 0.0 && !out.reverse, "{out:?}");
    }

    #[test]
    fn a_car_known_to_be_stopped_goes_either_way_from_the_middle() {
        let mut car = Car::new();
        car.clear_rc(RcSwitch::Direct);
        rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.5, 100);
        rc_ticks(&mut car, rc_direct(Pedal::Middle, false), 0.0, 50);
        let out = rc_ticks(&mut car, rc_direct(Pedal::Reverse, false), 0.0, 30);
        assert!(out.reverse && out.throttle > 0.0, "{out:?}");
    }

    #[test]
    fn after_the_right_stick_stop_the_car_stays_braked_until_the_left_stick_moves() {
        let mut car = Car::with_config(open_loop_config());
        car.clear_rc(RcSwitch::Direct);
        rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 50);
        let stop = RcInput {
            right: RightStick::Stop,
            ..rc_direct(Pedal::Forward, false)
        };
        let out = rc_ticks(&mut car, stop, 0.0, 30);
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        // Let go with the left stick still up: braked, however long.
        let out = rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 300);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        // To the middle: the brake lets go (throttle still off); then up drives.
        let out = rc_ticks(&mut car, rc_direct(Pedal::Middle, false), 0.0, 1);
        assert_eq!((out.throttle, out.brake), (0.0, 0.0));
        let out = rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 20);
        assert!(out.throttle > 0.0);
    }

    #[test]
    fn moving_the_left_stick_inside_the_second_still_waits_for_the_second() {
        let mut car = Car::with_config(open_loop_config());
        car.clear_rc(RcSwitch::Direct);
        rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 50);
        let stop = RcInput {
            right: RightStick::Stop,
            ..rc_direct(Pedal::Forward, false)
        };
        rc_ticks(&mut car, stop, 0.0, 5);
        // To the middle and straight back up: braked until a second after the stop.
        rc_ticks(&mut car, rc_direct(Pedal::Middle, false), 0.0, 1);
        let out = rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 98);
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        let out = rc_ticks(&mut car, rc_direct(Pedal::Forward, false), 0.0, 20);
        assert!(out.throttle > 0.0, "{out:?}");
    }

    #[test]
    fn rc_direct_stays_stopped_after_a_lost_signal_until_a_person_moves_the_stick() {
        let mut car = Car::with_config(open_loop_config());
        car.clear_rc(RcSwitch::Direct);
        let moving = Inputs {
            rc: Some(rc_direct(Pedal::Forward, false)),
            ..Default::default()
        };
        for _ in 0..20 {
            car.tick(moving);
        }
        assert!(car.tick(moving).throttle > 0.0);
        let out = car.tick(Inputs::default());
        assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        assert_eq!(car.spine.state().safety, Safety::Degraded);
        // The signal back, the stick still pushed: stays stopped.
        for _ in 0..150 {
            let out = car.tick(moving);
            assert_eq!((out.throttle, out.brake), (0.0, 1.0));
        }
        // The left stick to its middle: cleared. The stick drives again.
        car.tick(Inputs {
            rc: Some(rc_direct(Pedal::Middle, false)),
            ..Default::default()
        });
        for _ in 0..20 {
            car.tick(moving);
        }
        assert!(car.tick(moving).throttle > 0.0);
    }

    #[test]
    fn rc_direct_waits_for_a_person_after_an_estop_or_a_driver_fault_clears() {
        // The stick still pushed when the stop goes away must not drive the car again
        // by itself: the same rule as a lost signal (RcHold).
        let moving = Inputs {
            rc: Some(rc_direct(Pedal::Forward, false)),
            ..Default::default()
        };
        for stop in [
            Inputs {
                hw_estop: true,
                ..moving
            },
            Inputs {
                driver_faults: faults::BRAKE_DRIVER,
                ..moving
            },
        ] {
            let mut car = Car::with_config(open_loop_config());
            car.clear_rc(RcSwitch::Direct);
            for _ in 0..20 {
                car.tick(moving);
            }
            assert!(car.tick(moving).throttle > 0.0);
            let out = car.tick(stop);
            assert_eq!(out.throttle, 0.0);
            // The stop gone, the stick still pushed: stays stopped.
            for _ in 0..150 {
                let out = car.tick(moving);
                assert_eq!((out.throttle, out.brake), (0.0, 1.0), "{stop:?}");
            }
            // The left stick to its middle: cleared, it drives again.
            car.tick(Inputs {
                rc: Some(rc_direct(Pedal::Middle, false)),
                ..Default::default()
            });
            for _ in 0..20 {
                car.tick(moving);
            }
            assert!(car.tick(moving).throttle > 0.0, "{stop:?}");
        }
    }

    #[test]
    fn a_lost_speed_stops_rc_direct_too() {
        let mut car = Car::with_config(open_loop_config());
        car.clear_rc(RcSwitch::Direct);
        let inputs = |speed| Inputs {
            rc: Some(rc_direct(Pedal::Forward, false)),
            speed_report: speed,
            ..Default::default()
        };
        for _ in 0..20 {
            car.tick(inputs(Some(SpeedReport::Known(0.2))));
        }
        assert!(car.tick(inputs(Some(SpeedReport::Known(0.2)))).throttle > 0.0);
        for _ in 0..30 {
            car.tick(inputs(None));
        }
        let out = car.tick(inputs(None));
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 1.0));
        assert_ne!(out.faults & faults::WHEEL_ENCODER, 0);
        // The speed back: a person must still move the stick.
        let out = car.tick(inputs(Some(SpeedReport::Known(0.0))));
        assert_eq!(out.brake, 1.0);
    }

    #[test]
    fn overspeed_applies_to_rc_direct() {
        let mut car = Car::with_config(open_loop_config());
        car.clear_rc(RcSwitch::Direct);
        let inputs = |speed| Inputs {
            rc: Some(rc_direct(Pedal::Forward, true)),
            speed_report: Some(SpeedReport::Known(speed)),
            ..Default::default()
        };
        for _ in 0..20 {
            car.tick(inputs(0.5));
        }
        assert!(
            car.tick(inputs(0.5)).throttle > 0.0,
            "the stick moved: the RC drives"
        );
        let out = car.tick(inputs(1.3));
        assert_eq!((out.mode, out.throttle, out.brake), (Mode::Rc, 0.0, 0.5));
        for _ in 0..200 {
            car.tick(inputs(1.3));
        }
        assert_eq!(
            car.tick(inputs(0.0)).brake,
            1.0,
            "held until a person clears it"
        );
    }

    #[test]
    fn a_speed_from_the_link_lets_a_throttle_test_run_and_its_speed_exit_fire() {
        let mut car = Car::with_config(open_loop_config());
        let tick = |car: &mut Car, req, speed| {
            car.tick(Inputs {
                test_cmd: Some(req),
                speed_report: Some(SpeedReport::Known(speed)),
                ..Default::default()
            })
        };
        tick(&mut car, TestRequest::Exit, 0.0);
        tick(&mut car, TestRequest::Enter, 0.0);
        let out = tick(&mut car, TestRequest::Throttle(0.1), 0.0);
        assert_eq!((out.mode, out.throttle), (Mode::Test, 0.1));
        let out = tick(&mut car, TestRequest::Throttle(0.1), 0.6);
        assert_ne!(out.mode, Mode::Test, "faster than TEST_MAX_SPEED leaves it");
        assert_eq!(out.throttle, 0.0);
    }
}
