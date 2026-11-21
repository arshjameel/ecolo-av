//! spine_bridge: the ROS side of the CAN spine, in Rust.
//!
//! Subscribes  control/<target>/actuator_cmd  ackermann_msgs/AckermannDriveStamped: ego's
//!                                   command, from the output mux (aav_io_mux): control/car/
//!                                   for source real, control/bench/ for bench,
//!                                   control/sim/ for sim. Never /ego/actuator_cmd
//!                                   itself: on the car, only an output mux that found
//!                                   every source REAL reaches this node
//!             safety/estop          std_msgs/Bool
//!             safety/output_mux/state  std_msgs/String: the safety gate, for /spine/arm
//!             safety/self_test/status  diagnostic_msgs/DiagnosticStatus: the boot
//!                                   self-test, for /spine/arm (only with require_ready)
//!             can/rx               can_msgs/Frame  (ros2_socketcan's receiver on the car;
//!                                   spine_sim on the bench and in Ravensim)
//! Publishes   can/tx                can_msgs/Frame  (ros2_socketcan's sender)
//!             vehicle/odom          nav_msgs/Odometry  (twist only, in base_link)
//!             vehicle/wheels        aav_interfaces/WheelSpeeds  (the front wheel nodes,
//!                                   ~100 Hz; NaN for a stale wheel and for the rear two)
//!             vehicle/status        aav_interfaces/VehicleStatus
//!             vehicle/joint_states  sensor_msgs/JointState  (front steering joints, for
//!                                   RViz; joint_state_publisher merges it into /joint_states)
//!             vehicle/steering      aav_interfaces/ActuatorState  (command: the last sent;
//!                                   measured: VEHICLE_FEEDBACK)
//!             vehicle/throttle, vehicle/brake  aav_interfaces/ActuatorState  (measured, from
//!                                   VEHICLE_STATUS; command NaN: the spine's speed loop
//!                                   decides)
//!             system/source         aav_interfaces/DataSource, latched: REAL on the car,
//!                                   BENCH on the bench, SIM in Ravensim's spine profile (the
//!                                   `source` parameter)
//! Service     spine/arm             std_srvs/SetBool: refused during an e-stop, and while
//!                                   the safety gate is shut: the output mux not OPEN for
//!                                   this bridge's target, or quiet for gate_timeout s
//!                                   (Ben, 24 Oct); with require_ready, also until the
//!                                   boot self-test says READY within ready_timeout s
//!                                   (TODO 255). Disarming always works
//! Parameters  wheelbase, track (m), cmd_timeout, wheel_timeout, gate_timeout (s),
//!             require_ready (false; the car: true), ready_timeout (2.0 s),
//!             drive_rate_hz, estop_rate_hz, speed_variance, yaw_rate_variance,
//!             steering_joints, source, role; aav_spine's config/spine.yaml
//!
//! role:=spine (the default) is all of the above. role:=sensors is for a car driven
//! another way (today's: the ESP32, through the Motor Pi's UART, docs/can-plan.md): the
//! node only reads the wheel nodes' frames on can/rx, and has
//!   no can/tx publisher, no safety/estop or safety/output_mux/state subscription, no
//!   spine/arm service, and none
//!   of vehicle/status, joint_states, steering, throttle, brake;
//!   vehicle/odom from the wheels alone (their mean while both are fresh; the yaw rate
//!   unmeasured: the steering angle isn't on the bus), and vehicle/wheels;
//!   system/source: only odometry and wheels, never the actuators.
//! It still reads control/<target>/actuator_cmd, only to sign a wheel that sends a
//! magnitude (one Hall sensor can't tell which way it turns).
//!
//! The same node as aav_spine's spine_bridge.py, topic for topic. The frames are
//! firmware/can/aav.dbc, packed by the firmware's own code (spine-core), and the logic is
//! the `spine-bridge` crate, with no ROS. /vehicle/odom's speed is the front wheel nodes'
//! mean while both are fresh, otherwise the spine's measured speed. DRIVE_CMD goes out at
//! 50 Hz only while ego's command is fresh: if ego (or the output mux) stops, the bridge
//! stops sending and the spine's own watchdog brakes the car. Where it differs from the Python, on purpose:
//! it ignores extended-ID frames (another device's, never ours), and its parameters are
//! read-only (it reads them once, as the Python did).

use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{anyhow, bail, Context as _, Result};
use rclrs::*;
use ros_env::aav_interfaces::msg::{
    ActuatorState, DataSource, SensorSource, VehicleStatus, WheelSpeeds,
};
use ros_env::ackermann_msgs::msg::AckermannDriveStamped;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::can_msgs::msg::Frame;
use ros_env::diagnostic_msgs::msg::DiagnosticStatus;
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::JointState;
use ros_env::std_msgs::msg::{Bool, Header, String as StringMsg};
use ros_env::std_srvs::srv::{SetBool, SetBool_Request, SetBool_Response};
use spine_bridge::{
    wheels, Bridge, Command, Drive, Geometry, Motion, Reading, Role, Settings, Status, Target,
    Update, DRIVE_CMD_ID, ESTOP_CMD_ID,
};

/// What this node publishes, for /system/source, in role spine.
const ANNOUNCED: [(&str, &str); 6] = [
    ("vehicle", "/vehicle/status"),
    ("odometry", "/vehicle/odom"),
    ("steering", "/vehicle/steering"),
    ("throttle", "/vehicle/throttle"),
    ("brake", "/vehicle/brake"),
    ("wheels", "/vehicle/wheels"),
];
/// In role sensors: the wheels only. No actuators.
const ANNOUNCED_SENSORS: [(&str, &str); 2] =
    [("odometry", "/vehicle/odom"), ("wheels", "/vehicle/wheels")];
/// How often a repeated warning may be logged.
const WARN_EVERY: Duration = Duration::from_secs(2);
/// Covariance of what isn't measured: z, roll and pitch, and the whole pose.
const UNKNOWN: f64 = 1e3;
/// (m/s)²: a car doesn't slide sideways.
const SIDEWAYS_VARIANCE: f64 = 1e-4;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("spine_bridge")?;

    let params = Params::declare(&node)?;
    let source = Source::parse(&params.source.get())?;
    let role: Role = params.role.get().parse()?;
    let settings = Settings {
        role,
        geometry: Geometry {
            wheelbase: params.wheelbase.get(),
            track: params.track.get(),
        },
        cmd_timeout: seconds("cmd_timeout", params.cmd_timeout.get())?,
        wheel_timeout: seconds("wheel_timeout", params.wheel_timeout.get())?,
        target: source.target(),
        gate_timeout: seconds("gate_timeout", params.gate_timeout.get())?,
    };
    let drive_period = period("drive_rate_hz", params.drive_rate_hz.get())?;
    let estop_period = period("estop_rate_hz", params.estop_rate_hz.get())?;

    let bridge = if params.require_ready.get() {
        log_info!(
            node.logger(),
            "require_ready: /spine/arm waits for the boot self-test's READY"
        );
        Bridge::new(settings).with_ready_gate(seconds("ready_timeout", params.ready_timeout.get())?)
    } else {
        Bridge::new(settings)
    };
    let state = State {
        bridge,
        clock: node.get_clock(),
        logger: node.logger().clone(),
        variances: [params.speed_variance.get(), params.yaw_rate_variance.get()],
        joints: params
            .steering_joints
            .get()
            .iter()
            .map(|j| j.to_string())
            .collect(),
        out: Outputs {
            odom: node.create_publisher("vehicle/odom")?,
            wheels: node.create_publisher("vehicle/wheels")?,
            // In role sensors, none of these exist: nothing to put on the bus, and no
            // topic the actuators would own.
            spine: if role.sends() {
                Some(SpineOutputs {
                    can: node.create_publisher("can/tx".keep_last(50))?,
                    status: node.create_publisher("vehicle/status")?,
                    joints: node.create_publisher("vehicle/joint_states")?,
                    steering: node.create_publisher("vehicle/steering")?,
                    throttle: node.create_publisher("vehicle/throttle")?,
                    brake: node.create_publisher("vehicle/brake")?,
                })
            } else {
                None
            },
        },
    };
    let announced: &[(&str, &str)] = if role.sends() {
        &ANNOUNCED
    } else {
        &ANNOUNCED_SENSORS
    };

    // Said once, latched: where this node's data comes from.
    let source_pub = node.create_publisher::<DataSource>(
        "system/source".keep_last(1).reliable().transient_local(),
    )?;
    source_pub.publish(DataSource {
        header: Header {
            stamp: state.stamp(),
            frame_id: String::new(),
        },
        announcer: node.name(),
        sensors: announced
            .iter()
            .map(|(name, topic)| SensorSource {
                name: (*name).into(),
                topic: (*topic).into(),
                source: source.kind(),
            })
            .collect(),
    })?;

    // One worker: its callbacks take turns with the state, as the Python's did.
    let worker = node.create_worker(state);
    // In role sensors, only to sign a wheel that sends a magnitude.
    let _command = worker.create_subscription(
        source.commands(),
        |s: &mut State, msg: AckermannDriveStamped| {
            let now = s.now();
            s.bridge.command(
                Command {
                    speed: msg.drive.speed,
                    steering_angle: msg.drive.steering_angle,
                    acceleration: msg.drive.acceleration,
                    steering_angle_velocity: msg.drive.steering_angle_velocity,
                },
                now,
            );
        },
    )?;
    let _frames = worker
        .create_subscription("can/rx".keep_last(100), |s: &mut State, frame: Frame| {
            s.on_frame(&frame)
        })?;
    let _drive_side = if role.sends() {
        Some(DriveSide::create(&worker, drive_period, estop_period)?)
    } else {
        log_info!(
            node.logger(),
            "role sensors: reading the wheel nodes only; nothing is sent on CAN"
        );
        None
    };

    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// Role spine only: what drives the car. Kept alive as long as the node.
struct DriveSide {
    _estop: WorkerSubscription<Bool, State>,
    _gate: WorkerSubscription<StringMsg, State>,
    _ready: WorkerSubscription<DiagnosticStatus, State>,
    _arm: WorkerService<SetBool, State>,
    _drive: WorkerTimer<State>,
    _heartbeat: WorkerTimer<State>,
}

impl DriveSide {
    /// /safety/estop, /safety/output_mux/state, /spine/arm, and the DRIVE_CMD and
    /// ESTOP_CMD timers.
    fn create(
        worker: &Worker<State>,
        drive_period: Duration,
        estop_period: Duration,
    ) -> Result<Self> {
        let _estop = worker.create_subscription("safety/estop", |s: &mut State, msg: Bool| {
            if let Some(frame) = s.bridge.set_estop(msg.data) {
                s.send(ESTOP_CMD_ID, &frame.encode());
            }
        })?;
        // The safety gate, on a steady clock: a paused simulation can't keep an old OPEN.
        let _gate = worker.create_subscription(
            "safety/output_mux/state",
            |s: &mut State, msg: StringMsg| s.bridge.output_mux_state(&msg.data, Instant::now()),
        )?;
        // The boot self-test (TODO 255), on the same steady clock. Heard always; it counts
        // only with require_ready (Bridge::with_ready_gate).
        let _ready = worker.create_subscription(
            "safety/self_test/status",
            |s: &mut State, msg: DiagnosticStatus| {
                s.bridge.self_test(
                    msg.level == DiagnosticStatus::OK,
                    &msg.message,
                    Instant::now(),
                );
            },
        )?;
        let _arm = worker.create_service::<SetBool, _>(
            "spine/arm",
            |s: &mut State, request: SetBool_Request| {
                let (success, message) = match s.bridge.arm(request.data, Instant::now()) {
                    Ok(arming) => {
                        log_info!(&s.logger, "{arming}");
                        (true, arming.to_string())
                    }
                    Err(refused) => {
                        log_warn!(&s.logger, "arm refused: {refused}");
                        (false, refused.to_string())
                    }
                };
                SetBool_Response { success, message }
            },
        )?;
        let _drive = worker.create_timer_repeating(drive_period.node_time(), |s: &mut State| {
            let now = s.now();
            match s.bridge.drive(now) {
                Drive::Send(cmd) => s.send(DRIVE_CMD_ID, &cmd.encode()),
                Drive::Stale => log_warn!(
                    s.logger.throttle(WARN_EVERY),
                    "no fresh command from ego and the output mux; the spine will brake"
                ),
                Drive::NoCommand => {}
            }
        })?;
        let _heartbeat =
            worker.create_timer_repeating(estop_period.node_time(), |s: &mut State| {
                let frame = s.bridge.estop_frame();
                s.send(ESTOP_CMD_ID, &frame.encode());
            })?;
        Ok(Self {
            _estop,
            _gate,
            _ready,
            _arm,
            _drive,
            _heartbeat,
        })
    }
}

/// The node's parameters: aav_spine's config/spine.yaml, and the car's wheelbase and track
/// from vehicle.yaml (the launch files pass them).
///
/// Keep this alive as long as the node: rclrs undeclares a parameter when its handle is
/// dropped, and then `ros2 param` can't see it. Read-only, because they are read once.
struct Params {
    wheelbase: ReadOnlyParameter<f64>,
    track: ReadOnlyParameter<f64>,
    cmd_timeout: ReadOnlyParameter<f64>,
    wheel_timeout: ReadOnlyParameter<f64>,
    gate_timeout: ReadOnlyParameter<f64>,
    require_ready: ReadOnlyParameter<bool>,
    ready_timeout: ReadOnlyParameter<f64>,
    drive_rate_hz: ReadOnlyParameter<f64>,
    estop_rate_hz: ReadOnlyParameter<f64>,
    speed_variance: ReadOnlyParameter<f64>,
    yaw_rate_variance: ReadOnlyParameter<f64>,
    steering_joints: ReadOnlyParameter<Arc<[Arc<str>]>>,
    source: ReadOnlyParameter<Arc<str>>,
    role: ReadOnlyParameter<Arc<str>>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        let number = |name: &str, default: f64| -> Result<ReadOnlyParameter<f64>> {
            Ok(node.declare_parameter(name).default(default).read_only()?)
        };
        let joints: Arc<[Arc<str>]> = Arc::from([
            "front_left_steering_joint".into(),
            "front_right_steering_joint".into(),
        ]);
        Ok(Self {
            wheelbase: number("wheelbase", 1.57)?,        // m
            track: number("track", 0.77)?,                // m
            cmd_timeout: number("cmd_timeout", 0.1)?,     // s
            wheel_timeout: number("wheel_timeout", 0.1)?, // s
            gate_timeout: number("gate_timeout", 1.5)?,   // s
            require_ready: node
                .declare_parameter("require_ready")
                .default(false)
                .read_only()?,
            ready_timeout: number("ready_timeout", 2.0)?, // s
            drive_rate_hz: number("drive_rate_hz", 50.0)?, // Hz
            estop_rate_hz: number("estop_rate_hz", 10.0)?, // Hz
            speed_variance: number("speed_variance", 0.01)?, // (m/s)²
            yaw_rate_variance: number("yaw_rate_variance", 0.02)?, // (rad/s)²
            steering_joints: node
                .declare_parameter("steering_joints")
                .default(joints)
                .read_only()?,
            source: node
                .declare_parameter("source")
                .default("real".into())
                .read_only()?,
            role: node
                .declare_parameter("role")
                .default("spine".into())
                .read_only()?,
        })
    }
}

fn seconds(name: &str, value: f64) -> Result<Duration> {
    Duration::try_from_secs_f64(value).with_context(|| format!("{name}:={value} (seconds)"))
}

fn period(name: &str, hz: f64) -> Result<Duration> {
    if hz.is_nan() || hz <= 0.0 {
        bail!("{name}:={hz}: a rate must be above 0 Hz");
    }
    seconds(name, 1.0 / hz)
}

/// Where the data under the bridge comes from (the `source` parameter).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    /// The real spine board.
    Real,
    /// The bench's spine_sim.
    Bench,
    /// spine_sim on Ravensim's car.
    Sim,
}

impl Source {
    fn parse(name: &str) -> Result<Self> {
        match name {
            "real" => Ok(Source::Real),
            "bench" => Ok(Source::Bench),
            "sim" => Ok(Source::Sim),
            other => Err(anyhow!(
                "source:={other}: use one of ['bench', 'real', 'sim']"
            )),
        }
    }

    /// Where ego's commands come from: the output mux's topic for the target this bridge
    /// plays (aav_io_mux provenance.TARGETS). The real car's only from target car.
    fn commands(self) -> &'static str {
        match self {
            Source::Real => "control/car/actuator_cmd",
            Source::Bench => "control/bench/actuator_cmd",
            Source::Sim => "control/sim/actuator_cmd",
        }
    }

    /// The output mux whose gate must be OPEN to arm: the target in [`Source::commands`].
    fn target(self) -> Target {
        match self {
            Source::Real => Target::Car,
            Source::Bench => Target::Bench,
            Source::Sim => Target::Sim,
        }
    }

    fn kind(self) -> u8 {
        match self {
            Source::Real => SensorSource::REAL,
            Source::Bench => SensorSource::BENCH,
            Source::Sim => SensorSource::SIM,
        }
    }
}

struct Outputs {
    odom: Publisher<Odometry>,
    wheels: Publisher<WheelSpeeds>,
    /// Role spine only; `None` in role sensors.
    spine: Option<SpineOutputs>,
}

/// The bus, and the topics only a spine fills.
struct SpineOutputs {
    can: Publisher<Frame>,
    status: Publisher<VehicleStatus>,
    joints: Publisher<JointState>,
    steering: Publisher<ActuatorState>,
    throttle: Publisher<ActuatorState>,
    brake: Publisher<ActuatorState>,
}

/// The worker's payload: the bridge and what it publishes with.
struct State {
    bridge: Bridge,
    clock: Clock,
    logger: Logger,
    /// /vehicle/odom's speed and yaw-rate variances.
    variances: [f64; 2],
    joints: Vec<String>,
    out: Outputs,
}

impl State {
    /// The node's clock (the simulator's, with use_sim_time), since its zero.
    fn now(&self) -> Duration {
        Duration::from_nanos(u64::try_from(self.clock.now().nsec).unwrap_or(0))
    }

    fn stamp(&self) -> Time {
        self.clock.now().to_ros_msg().unwrap_or_default()
    }

    fn header(&self, frame_id: &str) -> Header {
        Header {
            stamp: self.stamp(),
            frame_id: frame_id.into(),
        }
    }

    /// Puts one frame on /can/tx. In role sensors there is no /can/tx: nothing is sent.
    fn send(&self, id: u16, data: &[u8]) {
        let Some(spine) = &self.out.spine else {
            return;
        };
        let mut frame = Frame {
            header: self.header(""),
            id: id.into(),
            dlc: u8::try_from(data.len()).unwrap_or(8),
            ..Frame::default()
        };
        frame.data.iter_mut().zip(data).for_each(|(d, b)| *d = *b);
        if let Err(e) = spine.can.publish(frame) {
            log_warn!(self.logger.throttle(WARN_EVERY), "/can/tx: {e}");
        }
    }

    /// One frame from /can/rx.
    fn on_frame(&mut self, frame: &Frame) {
        if frame.is_error || frame.is_rtr || frame.is_extended {
            return;
        }
        let Ok(id) = u16::try_from(frame.id) else {
            return;
        };
        let len = usize::from(frame.dlc).min(frame.data.len());
        let now = self.now();
        match self.bridge.receive(id, &frame.data[..len], now) {
            Ok(Some(update)) => {
                if let Err(e) = self.publish(update) {
                    log_warn!(self.logger.throttle(WARN_EVERY), "publishing: {e}");
                }
            }
            Ok(None) => {}
            Err(bad) => log_warn!(self.logger.throttle(WARN_EVERY), "{bad}"),
        }
    }

    fn publish(&self, update: Update) -> Result<(), RclrsError> {
        match update {
            Update::Motion(m) => self.publish_motion(&m),
            Update::Status(s) => self.publish_status(&s),
            Update::Wheels(r) => self.publish_wheels(&r),
        }
    }

    /// /vehicle/odom: base_link's speed, and its yaw rate if it is known.
    fn publish_odom(
        &self,
        stamp: Time,
        speed: f64,
        yaw_rate: Option<f64>,
    ) -> Result<(), RclrsError> {
        let [speed_variance, yaw_rate_variance] = self.variances;
        let mut odom = Odometry {
            header: Header {
                stamp,
                frame_id: "odom".into(),
            },
            child_frame_id: "base_link".into(),
            ..Odometry::default()
        };
        odom.twist.twist.linear.x = speed;
        odom.twist.twist.angular.z = yaw_rate.unwrap_or(0.0);
        // vx, vy, vz, roll, pitch and yaw rates, on the diagonal.
        let mut twist_diagonal = [
            speed_variance,
            SIDEWAYS_VARIANCE,
            UNKNOWN,
            UNKNOWN,
            UNKNOWN,
            yaw_rate.map_or(UNKNOWN, |_| yaw_rate_variance),
        ];
        // Wheels that read nothing: the car stands, exactly (TODO 191, as aav_description's
        // standstill_covariance): its speed, sideways speed and turn rate all known zero.
        if wheels::is_standstill(speed) {
            for i in [0, 1, 5] {
                twist_diagonal[i] = wheels::STANDSTILL_VARIANCE;
            }
        }
        for (i, variance) in twist_diagonal.into_iter().enumerate() {
            odom.twist.covariance[i * 7] = variance;
        }
        odom.pose
            .covariance
            .iter_mut()
            .step_by(7)
            .for_each(|c| *c = UNKNOWN);
        self.out.odom.publish(odom)
    }

    fn publish_motion(&self, m: &Motion) -> Result<(), RclrsError> {
        let stamp = self.stamp();
        self.publish_odom(stamp.clone(), m.speed, Some(m.yaw_rate))?;
        let Some(spine) = &self.out.spine else {
            return Ok(());
        };
        spine.joints.publish(JointState {
            header: Header {
                stamp: stamp.clone(),
                frame_id: String::new(),
            },
            name: self.joints.clone(),
            position: m.wheel_angles.to_vec(),
            ..JointState::default()
        })?;

        spine.steering.publish(actuator(
            stamp,
            m.steering_command.unwrap_or(f32::NAN),
            m.steering_angle as f32,
        ))
    }

    fn publish_status(&self, s: &Status) -> Result<(), RclrsError> {
        let Some(spine) = &self.out.spine else {
            return Ok(());
        };
        let header = self.header("base_link");
        let stamp = header.stamp.clone();
        spine.status.publish(VehicleStatus {
            header,
            mode: s.mode as u8,
            armed: s.armed,
            estop_active: s.estop_active,
            speed: s.speed.unwrap_or(0.0),
            steering_angle: s.steering_angle.unwrap_or(0.0),
            throttle: s.throttle,
            brake: s.brake,
            faults: s.faults,
        })?;
        spine
            .throttle
            .publish(actuator(stamp.clone(), f32::NAN, s.throttle))?;
        spine.brake.publish(actuator(stamp, f32::NAN, s.brake))
    }

    fn publish_wheels(&self, r: &Reading) -> Result<(), RclrsError> {
        let nan_unless_fresh =
            |side: &spine_bridge::Side| side.fresh_speed().map_or(f32::NAN, |speed| speed as f32);
        let header = self.header("base_link");
        let stamp = header.stamp.clone();
        self.out.wheels.publish(WheelSpeeds {
            header,
            front_left: nan_unless_fresh(&r.left),
            front_right: nan_unless_fresh(&r.right),
            rear_left: f32::NAN,
            rear_right: f32::NAN,
            direction_measured: r.direction_measured,
        })?;
        // Role sensors: no spine sends VEHICLE_FEEDBACK, so the wheels alone give
        // /vehicle/odom, while both are fresh (else nothing: the watchdog sees it stop).
        // No steering angle on this bus: the front mean as it is, and no yaw rate (the
        // EKF takes it from the IMU).
        match (&self.out.spine, r.mean()) {
            (None, Some(speed)) => self.publish_odom(stamp, speed, None),
            _ => Ok(()),
        }
    }
}

fn actuator(stamp: Time, command: f32, measured: f32) -> ActuatorState {
    ActuatorState {
        header: Header {
            stamp,
            frame_id: "base_link".into(),
        },
        command,
        measured,
    }
}
