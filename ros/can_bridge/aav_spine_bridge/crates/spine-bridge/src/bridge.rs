//! The bridge's state: what it sends the spine, and what it makes of what it hears.

use std::{
    fmt,
    str::FromStr,
    time::{Duration, Instant},
};

use spine_core::protocol::{
    decode_report, DriveCmd, EstopCmd, FrameError, Mode, Report, VehicleFeedback, VehicleStatus,
};

use crate::gate::{GateShut, OutputGate, Target};
use crate::ready::{NotReady, ReadyGate};
use crate::wheels::{rear_axle_speed, Direction, FrontWheels, Reading};

/// Armed=0 frames sent before armed=1: the spine arms only on that edge.
const DISARM_FRAMES: u8 = 3;

/// The car's shape, in metres (aav_description's vehicle.yaml).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geometry {
    /// Front axle to rear axle.
    pub wheelbase: f64,
    /// Between the front wheels' centres.
    pub track: f64,
}

impl Geometry {
    /// The yaw rate (rad/s) of base_link at `speed` (m/s) and a centre steering angle.
    #[must_use]
    pub fn yaw_rate(&self, speed: f64, steering_angle: f64) -> f64 {
        speed * steering_angle.tan() / self.wheelbase
    }

    /// The left and right front wheels' angles (rad) for a centre steering angle
    /// (Ackermann).
    #[must_use]
    pub fn wheel_angles(&self, steering_angle: f64) -> [f64; 2] {
        if steering_angle.abs() < 1e-6 {
            return [0.0, 0.0];
        }
        let radius = self.wheelbase / steering_angle.tan(); // signed, positive turning left
        [
            (self.wheelbase / (radius - self.track / 2.0)).atan(),
            (self.wheelbase / (radius + self.track / 2.0)).atan(),
        ]
    }
}

/// What the bridge does on its bus (the `role` parameter).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Role {
    /// The CAN spine drives the car: the bridge sends DRIVE_CMD and ESTOP_CMD, and reads
    /// every frame back.
    #[default]
    Spine,
    /// The car is driven another way (today: the ESP32, through the Motor Pi's UART). The
    /// bridge only listens, and only to the wheel nodes: it never sends a frame, and it
    /// ignores VEHICLE_FEEDBACK and VEHICLE_STATUS, which only a spine sends.
    Sensors,
}

impl Role {
    /// The names the `role` parameter takes.
    pub const NAMES: [&'static str; 2] = ["spine", "sensors"];

    /// The bridge sends frames on the bus: DRIVE_CMD and ESTOP_CMD.
    #[must_use]
    pub fn sends(self) -> bool {
        self == Role::Spine
    }
}

/// A `role` parameter that is neither `spine` nor `sensors`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownRole(pub String);

impl fmt::Display for UnknownRole {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "role:={}: use one of {:?}", self.0, Role::NAMES)
    }
}

impl std::error::Error for UnknownRole {}

impl FromStr for Role {
    type Err = UnknownRole;

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        match name {
            "spine" => Ok(Role::Spine),
            "sensors" => Ok(Role::Sensors),
            other => Err(UnknownRole(other.into())),
        }
    }
}

impl fmt::Display for Role {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Role::Spine => "spine",
            Role::Sensors => "sensors",
        })
    }
}

/// The bridge's settings: its ROS parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// The whole spine, or only the wheel sensors.
    pub role: Role,
    /// The car's shape.
    pub geometry: Geometry,
    /// Stop sending DRIVE_CMD when ego's last command is older.
    pub cmd_timeout: Duration,
    /// A wheel node quiet for longer is stale.
    pub wheel_timeout: Duration,
    /// The output mux whose gate must be OPEN to arm: the one this bridge's commands come
    /// from.
    pub target: Target,
    /// An older /safety/output_mux/state counts as silence: arming is refused.
    pub gate_timeout: Duration,
}

/// ego's command (ackermann_msgs/AckermannDrive): m/s, rad (+ left), m/s², rad/s.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Command {
    /// m/s, negative in reverse.
    pub speed: f32,
    /// rad, + left.
    pub steering_angle: f32,
    /// m/s²; 0: the spine's default.
    pub acceleration: f32,
    /// rad/s; 0: the spine's default.
    pub steering_angle_velocity: f32,
}

/// What to do at a DRIVE_CMD tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Drive {
    /// ego has never sent a command: send nothing.
    NoCommand,
    /// ego's command is too old: send nothing, and the spine's watchdog brakes the car.
    Stale,
    /// Send this frame.
    Send(DriveCmd),
}

/// The answer to /spine/arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arming {
    /// The spine was asked to arm.
    Armed,
    /// The spine was asked to disarm.
    Disarmed,
}

impl fmt::Display for Arming {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Arming::Armed => "armed",
            Arming::Disarmed => "disarmed",
        })
    }
}

/// Why /spine/arm refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ArmError {
    /// The e-stop is on: clear it first.
    EstopActive,
    /// The output mux isn't OPEN for this bridge's target, or has gone quiet (Ben, 24 Oct):
    /// armed now, the spine would only latch again, with no commands coming.
    GateShut(GateShut),
    /// The boot self-test hasn't said READY (TODO 255; only with
    /// [`Bridge::with_ready_gate`], the car).
    NotReady(NotReady),
}

impl fmt::Display for ArmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ArmError::EstopActive => f.write_str("e-stop is active; clear /safety/estop first"),
            ArmError::GateShut(shut) => shut.fmt(f),
            ArmError::NotReady(why) => why.fmt(f),
        }
    }
}

impl std::error::Error for ArmError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ArmError::EstopActive => None,
            ArmError::GateShut(shut) => Some(shut),
            ArmError::NotReady(why) => Some(why),
        }
    }
}

impl From<NotReady> for ArmError {
    fn from(why: NotReady) -> Self {
        ArmError::NotReady(why)
    }
}

impl From<GateShut> for ArmError {
    fn from(shut: GateShut) -> Self {
        ArmError::GateShut(shut)
    }
}

/// A received frame that failed its checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BadFrame {
    /// Its CAN ID.
    pub id: u16,
    /// What was wrong.
    pub error: FrameError,
    /// Bad frames since the bridge started, this one included.
    pub so_far: u64,
}

impl fmt::Display for BadFrame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "CAN frame 0x{:03X}: {} ({} so far)",
            self.id, self.error, self.so_far
        )
    }
}

impl std::error::Error for BadFrame {}

/// The car's motion, from VEHICLE_FEEDBACK and the wheel nodes: /vehicle/odom,
/// /vehicle/joint_states and /vehicle/steering.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Motion {
    /// base_link's speed (m/s): the front wheels' while both are fresh, else the spine's.
    pub speed: f64,
    /// rad/s, from the speed and the measured steering angle.
    pub yaw_rate: f64,
    /// The measured centre steering angle (rad, + left).
    pub steering_angle: f64,
    /// The left and right front wheels' angles (rad).
    pub wheel_angles: [f64; 2],
    /// The steering angle last asked for, if ego has sent anything.
    pub steering_command: Option<f32>,
}

/// The spine's state, from VEHICLE_STATUS: /vehicle/status, throttle and brake.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Status {
    /// Who drives.
    pub mode: Mode,
    /// The spine is armed.
    pub armed: bool,
    /// An e-stop is on.
    pub estop_active: bool,
    /// 0..1 of full throttle.
    pub throttle: f32,
    /// 0..1 of full brake.
    pub brake: f32,
    /// VehicleStatus.FAULT_* bits.
    pub faults: u16,
    /// From the last VEHICLE_FEEDBACK, if any: m/s and rad.
    pub speed: Option<f32>,
    /// The last VEHICLE_FEEDBACK's steering (rad, + left), if any.
    pub steering_angle: Option<f32>,
}

/// What a received frame gives the ROS side to publish.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Update {
    /// From VEHICLE_FEEDBACK.
    Motion(Motion),
    /// From VEHICLE_STATUS.
    Status(Status),
    /// Both front wheels, from WHEEL_SPEED_L and _R.
    Wheels(Reading),
}

/// A 4-bit rolling counter: 1 for the first frame, then up by one per frame.
#[derive(Debug, Clone, Copy, Default)]
struct Counter(u8);

impl Counter {
    fn next(&mut self) -> u8 {
        self.0 = (self.0 + 1) & 0x0F;
        self.0
    }
}

/// spine_bridge without ROS: times are durations since the node clock's zero.
#[derive(Debug, Clone)]
pub struct Bridge {
    settings: Settings,
    command: Option<(Command, Duration)>,
    armed: bool,
    /// armed=0 frames still to send before arming.
    disarm_frames: u8,
    estop: bool,
    gate: OutputGate,
    ready: Option<ReadyGate>,
    drive_counter: Counter,
    estop_counter: Counter,
    feedback: Option<VehicleFeedback>,
    wheels: FrontWheels,
    bad_frames: u64,
}

impl Bridge {
    /// A bridge with these settings, not armed, nothing heard.
    #[must_use]
    pub fn new(settings: Settings) -> Self {
        Self {
            settings,
            command: None,
            armed: false,
            disarm_frames: 0,
            estop: false,
            gate: OutputGate::new(settings.target, settings.gate_timeout),
            ready: None,
            drive_counter: Counter::default(),
            estop_counter: Counter::default(),
            feedback: None,
            wheels: FrontWheels::new(settings.wheel_timeout),
            bad_frames: 0,
        }
    }

    // Orin -> spine

    /// ego's latest command (from the output mux), received at `now`.
    pub fn command(&mut self, command: Command, now: Duration) {
        self.command = Some((command, now));
    }

    /// /safety/estop. Returns the frame to send at once when the request changed.
    pub fn set_estop(&mut self, estop: bool) -> Option<EstopCmd> {
        let changed = estop != self.estop;
        self.estop = estop;
        if estop {
            self.armed = false;
        }
        changed.then(|| self.estop_frame())
    }

    /// The same bridge, refusing to arm until the boot self-test says READY within
    /// `timeout` (TODO 255; `require_ready`, the car). Disarming and the e-stop never wait.
    #[must_use]
    pub fn with_ready_gate(mut self, timeout: Duration) -> Self {
        self.ready = Some(ReadyGate::new(timeout));
        self
    }

    /// /safety/self_test/status: `ready` if its level is OK, its message, heard at `at`
    /// (a steady clock). Ignored without [`Bridge::with_ready_gate`].
    pub fn self_test(&mut self, ready: bool, message: &str, at: Instant) {
        if let Some(gate) = self.ready.as_mut() {
            gate.heard(ready, message, at);
        }
    }

    /// /safety/output_mux/state: the output mux's line, heard at `at` (a steady clock).
    pub fn output_mux_state(&mut self, line: &str, at: Instant) {
        self.gate.heard(line, at);
    }

    /// /spine/arm, at `now` (a steady clock, as [`Bridge::output_mux_state`]'s). Refused
    /// during an e-stop, and while the safety gate is shut: the output mux not OPEN for
    /// this bridge's target, or not heard for `gate_timeout`. Disarming always works.
    ///
    /// Arming always sends a few armed=0 frames first: the spine arms only on that edge,
    /// and it also clears latches the bridge can't see (a hardware e-stop, a command
    /// timeout, a driver fault, a spine reboot).
    ///
    /// # Errors
    ///
    /// [`ArmError::EstopActive`] while the e-stop is on; [`ArmError::GateShut`] while
    /// the output mux isn't open for this bridge; [`ArmError::NotReady`] while the boot
    /// self-test hasn't said READY (with [`Bridge::with_ready_gate`]).
    pub fn arm(&mut self, arm: bool, now: Instant) -> Result<Arming, ArmError> {
        if arm && self.estop {
            return Err(ArmError::EstopActive);
        }
        if arm {
            self.gate.check(now)?;
            if let Some(ready) = &self.ready {
                ready.check(now)?;
            }
        }
        self.armed = arm;
        if arm {
            self.disarm_frames = DISARM_FRAMES;
            Ok(Arming::Armed)
        } else {
            Ok(Arming::Disarmed)
        }
    }

    /// The DRIVE_CMD tick (50 Hz): a frame only while ego's command is fresh, and never in
    /// [`Role::Sensors`] (its node has no tick; this is the second lock).
    pub fn drive(&mut self, now: Duration) -> Drive {
        let Some((cmd, time)) = self.command.filter(|_| self.settings.role.sends()) else {
            return Drive::NoCommand;
        };
        if now.saturating_sub(time) > self.settings.cmd_timeout {
            return Drive::Stale;
        }
        let armed = self.armed && self.disarm_frames == 0;
        self.disarm_frames = self.disarm_frames.saturating_sub(1);
        Drive::Send(DriveCmd {
            target_speed: cmd.speed,
            steering_angle: cmd.steering_angle,
            accel_limit: cmd.acceleration.abs(),
            steer_rate_limit: cmd.steering_angle_velocity.abs(),
            armed,
            counter: self.drive_counter.next(),
        })
    }

    /// The ESTOP_CMD heartbeat (10 Hz, and at once on a change).
    pub fn estop_frame(&mut self) -> EstopCmd {
        EstopCmd {
            estop_request: self.estop,
            counter: self.estop_counter.next(),
        }
    }

    // spine -> Orin

    /// One frame from the bus (/can/rx), at `now`. Frames for other IDs are not ours:
    /// `Ok(None)`, as is a wheel frame that waits for its partner. In [`Role::Sensors`]
    /// only the wheel nodes' frames count: a spine's are `Ok(None)` too.
    ///
    /// # Errors
    ///
    /// [`BadFrame`] for a frame of ours that fails its length or CRC.
    pub fn receive(
        &mut self,
        id: u16,
        data: &[u8],
        now: Duration,
    ) -> Result<Option<Update>, BadFrame> {
        let report = match decode_report(id, data) {
            Ok(report) => report,
            Err(FrameError::UnknownId) => return Ok(None),
            Err(error) => {
                self.bad_frames += 1;
                return Err(BadFrame {
                    id,
                    error,
                    so_far: self.bad_frames,
                });
            }
        };
        Ok(match report {
            Report::Feedback(_) | Report::Status(_) if !self.settings.role.sends() => None,
            Report::Feedback(feedback) => Some(Update::Motion(self.motion(feedback, now))),
            Report::Status(status) => Some(Update::Status(self.status(status))),
            Report::WheelSpeed(frame) => self
                .wheels
                .update(frame, now)
                .then(|| Update::Wheels(self.wheels.reading(now, self.direction()))),
        })
    }

    /// Which way the car is going, for a wheel that sends only a magnitude.
    fn direction(&self) -> Direction {
        Direction::of_drive(
            self.feedback.map(|f| f.measured_speed),
            self.command.map(|(c, _)| c.speed),
        )
    }

    fn motion(&mut self, feedback: VehicleFeedback, now: Duration) -> Motion {
        self.feedback = Some(feedback);
        let steer = f64::from(feedback.measured_steering_angle);
        let speed = match self.wheels.reading(now, self.direction()).mean() {
            Some(front) => rear_axle_speed(front, steer),
            None => f64::from(feedback.measured_speed),
        };
        let geometry = self.settings.geometry;
        Motion {
            speed,
            yaw_rate: geometry.yaw_rate(speed, steer),
            steering_angle: steer,
            wheel_angles: geometry.wheel_angles(steer),
            steering_command: self.command.map(|(c, _)| c.steering_angle),
        }
    }

    fn status(&self, status: VehicleStatus) -> Status {
        Status {
            mode: status.mode,
            armed: status.armed,
            estop_active: status.estop_active,
            throttle: status.throttle / 100.0,
            brake: status.brake / 100.0,
            faults: status.faults,
            speed: self.feedback.map(|f| f.measured_speed),
            steering_angle: self.feedback.map(|f| f.measured_steering_angle),
        }
    }

    /// Bad frames since the start.
    #[must_use]
    pub fn bad_frames(&self) -> u64 {
        self.bad_frames
    }
}
