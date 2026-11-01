//! The ROS side of the hardware panel: the node `hw_panel`, spun by rclrs on its own
//! thread, as the HUD's.
//!
//! Callbacks turn each message into a panel_core `Update`, sent over a channel. The
//! other way, the window hands over two things: the frame count and the buttons' latest
//! command. A 10 ms timer on the ROS thread does the sending:
//!
//! - the buttons' command, through hud-core's `Heartbeat`: at most 10 a second, and only
//!   while the frame count rises, so a frozen window sends nothing and twist_mux (0.5 s)
//!   and ego (0.3 s) stop the car;
//! - the E-STOP's burst (hud-core's `EstopButton`): true 5 times, never false.
//!
//! QoS:
//! - the lidar, the radar and the camera: best effort, keep last 1 (only counted);
//! - `/system/source` and the no-timeout lock: reliable, transient local, depth 1
//!   (latched);
//! - `/safety/estop`, both ways: reliable, depth 10, as the backends;
//! - the buttons' command and `/control/cmd_vel`: reliable, depth 10 (twist_mux's);
//! - everything else: best effort (SensorDataQoS), which takes reliable publishers too.
//!
//! The command's stamp is left at 0: twist_mux and ego go by arrival, and ego warns only
//! about a stamp that is set and far from its clock (sim time in Gazebo).

use std::{
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender, TrySendError},
        Arc, Mutex, PoisonError,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use anyhow::{anyhow, Context as _, Result};
use hud_core::{
    estop::EstopButton,
    gps::{Fix, RawFix},
    heartbeat::{Frames, Heartbeat},
    origin::{Announcement, Entry, Origin},
    state::Battery,
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Faults, Mode, VehicleStatus},
};
use panel_core::{
    buttons::Command,
    config::{Number, PanelConfig, Topic, NODE_NAME},
    state::{Actuator, ImuReading, Row, Update, Wheels},
};
use rclrs::*;
use ros_env::aav_interfaces::msg::{
    ActuatorState, DataSource, VehicleStatus as VehicleStatusMsg, WheelSpeeds,
};
use ros_env::geometry_msgs::msg::{Twist, TwistStamped, Vector3};
use ros_env::sensor_msgs::msg::{BatteryState, Image, Imu, NavSatFix, PointCloud2};
use ros_env::std_msgs::msg::{Bool, Header};

/// How many updates may wait for the screen. More are dropped: the screen is stuck.
const QUEUE: usize = 512;
/// How often the ROS thread sends.
const TICK: Duration = Duration::from_millis(10);

/// What the window shares with the ROS thread.
struct Shared {
    /// The frames the window has drawn.
    frames: AtomicU64,
    /// The buttons' latest command (None: send nothing).
    command: Mutex<Option<Command>>,
    estop_presses: AtomicU64,
    stopped: AtomicBool,
}

/// The node, seen from the screen.
pub struct RosSource {
    updates: Receiver<Update>,
    shared: Arc<Shared>,
    executor: Arc<ExecutorCommands>,
    thread: Option<JoinHandle<()>>,
}

/// What the ROS thread hands back once the node is up.
type Ready = Result<(PanelConfig, Arc<ExecutorCommands>)>;

impl RosSource {
    /// Starts the node on its own thread and waits until it's up. `args` is the whole
    /// command line: ROS reads what follows `--ros-args`.
    pub fn start(args: Vec<String>, config: PanelConfig) -> Result<(Self, PanelConfig)> {
        let (ready_tx, ready_rx) = mpsc::channel::<Ready>();
        let (update_tx, updates) = mpsc::sync_channel(QUEUE);
        let shared = Arc::new(Shared {
            frames: AtomicU64::new(0),
            command: Mutex::new(None),
            estop_presses: AtomicU64::new(0),
            stopped: AtomicBool::new(false),
        });
        let thread = {
            let shared = Arc::clone(&shared);
            thread::Builder::new().name("ros".into()).spawn(move || {
                let mut ready = Some(ready_tx);
                if let Err(e) = spin(args, config, update_tx, &shared, &mut ready) {
                    match ready.take() {
                        Some(r) => {
                            let _ = r.send(Err(e));
                        }
                        None => eprintln!("hw_panel: the ROS node stopped: {e:#}"),
                    }
                }
                shared.stopped.store(true, Ordering::SeqCst);
            })?
        };
        let (config, executor) = ready_rx
            .recv()
            .map_err(|_| anyhow!("the ROS thread ended before the node was up"))?
            .context("starting the ROS node (is ROS sourced? try tools/ros.sh, or --demo)")?;
        {
            let shared = Arc::clone(&shared);
            let executor = Arc::clone(&executor);
            ctrlc::set_handler(move || {
                shared.stopped.store(true, Ordering::SeqCst);
                executor.halt_spinning();
            })?;
        }
        let source = Self {
            updates,
            shared,
            executor,
            thread: Some(thread),
        };
        Ok((source, config))
    }
}

impl crate::source::DataSource for RosSource {
    fn label(&self) -> &'static str {
        "ROS"
    }

    fn drain(&mut self, _now: Instant) -> Vec<Update> {
        self.updates.try_iter().collect()
    }

    fn press_estop(&mut self) {
        self.shared.estop_presses.fetch_add(1, Ordering::SeqCst);
    }

    fn frame_drawn(&mut self, command: Option<Command>) {
        *self
            .shared
            .command
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = command;
        self.shared.frames.fetch_add(1, Ordering::Relaxed);
    }

    fn has_stopped(&self) -> bool {
        self.shared.stopped.load(Ordering::SeqCst)
    }
}

impl Drop for RosSource {
    fn drop(&mut self) {
        self.executor.halt_spinning();
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// The ROS thread: makes the node, says it's ready, then spins until halted.
fn spin(
    args: Vec<String>,
    config: PanelConfig,
    tx: SyncSender<Update>,
    shared: &Arc<Shared>,
    ready: &mut Option<mpsc::Sender<Ready>>,
) -> Result<()> {
    let context = Context::new(args, InitOptions::default())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME)?;
    let (_params, config) = Params::declare(&node, config)?;
    let topic = |t: Topic| config.topic(t).to_owned();
    let latched = QoSProfile::topics_default()
        .reliable()
        .transient_local()
        .keep_last(1);
    let reliable10 = QoSProfile::topics_default().reliable().keep_last(10);
    let newest = QoSProfile::sensor_data_default().keep_last(1);

    let _vehicle =
        node.create_subscription(topic(Topic::VehicleStatus).as_str().sensor_data_qos(), {
            let tx = tx.clone();
            move |m: VehicleStatusMsg| send(&tx, Update::Vehicle(vehicle(&m)))
        })?;
    let actuator = |t: Topic, wrap: fn(Actuator) -> Update| {
        let tx = tx.clone();
        node.create_subscription(
            topic(t).as_str().sensor_data_qos(),
            move |m: ActuatorState| {
                send(
                    &tx,
                    wrap(Actuator {
                        command: m.command,
                        measured: m.measured,
                    }),
                )
            },
        )
    };
    let _steering = actuator(Topic::Steering, Update::Steering)?;
    let _throttle = actuator(Topic::Throttle, Update::Throttle)?;
    let _brake = actuator(Topic::Brake, Update::Brake)?;
    let _wheels = node.create_subscription(topic(Topic::Wheels).as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: WheelSpeeds| {
            send(
                &tx,
                Update::Wheels(Wheels {
                    front_left: m.front_left,
                    front_right: m.front_right,
                    rear_left: m.rear_left,
                    rear_right: m.rear_right,
                }),
            )
        }
    })?;
    let _imu = node.create_subscription(topic(Topic::Imu).as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: Imu| {
            let (a, g) = (&m.linear_acceleration, &m.angular_velocity);
            send(
                &tx,
                Update::Imu(ImuReading {
                    accel: [a.x, a.y, a.z],
                    gyro: [g.x, g.y, g.z],
                }),
            )
        }
    })?;
    let _gps = node.create_subscription(topic(Topic::Gps).as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: NavSatFix| send(&tx, Update::Gps(fix(&m)))
    })?;
    let _lidar = node.create_subscription(topic(Topic::Lidar).as_str().qos(newest), {
        let tx = tx.clone();
        move |_: PointCloud2| send(&tx, Update::Counted(Row::Lidar))
    })?;
    let _radar = node.create_subscription(topic(Topic::Radar).as_str().qos(newest), {
        let tx = tx.clone();
        move |_: PointCloud2| send(&tx, Update::Counted(Row::Radar))
    })?;
    let _camera = node.create_subscription(topic(Topic::Camera).as_str().qos(newest), {
        let tx = tx.clone();
        move |_: Box<Image>| send(&tx, Update::Counted(Row::Camera))
    })?;
    let _battery = node.create_subscription(topic(Topic::Battery).as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: BatteryState| {
            let b = Battery::from_message(m.voltage, m.percentage, m.current);
            send(&tx, Update::Battery(b))
        }
    })?;
    let _source = node.create_subscription(topic(Topic::Source).as_str().qos(latched), {
        let tx = tx.clone();
        move |m: DataSource| send(&tx, Update::Source(announcement(&m)))
    })?;
    let _estop_in = node.create_subscription(topic(Topic::Estop).as_str().qos(reliable10), {
        let tx = tx.clone();
        move |m: Bool| send(&tx, Update::Estop(m.data))
    })?;
    let _control = node.create_subscription(topic(Topic::Control).as_str().qos(reliable10), {
        let tx = tx.clone();
        move |m: TwistStamped| {
            let c = Command {
                speed: m.twist.linear.x,
                yaw_rate: m.twist.angular.z,
            };
            send(&tx, Update::Mux(c))
        }
    })?;
    let _geofence =
        node.create_subscription(topic(Topic::GeofenceLock).as_str().sensor_data_qos(), {
            let tx = tx.clone();
            move |m: Bool| send(&tx, Update::GeofenceLock(m.data))
        })?;
    let _no_timeout =
        node.create_subscription(topic(Topic::NoTimeoutLock).as_str().qos(latched), {
            let tx = tx.clone();
            move |m: Bool| send(&tx, Update::NoTimeoutLock(m.data))
        })?;

    let estop_pub: Publisher<Bool> =
        node.create_publisher(topic(Topic::Estop).as_str().qos(reliable10))?;
    let buttons_pub: Publisher<TwistStamped> =
        node.create_publisher(topic(Topic::Buttons).as_str().qos(reliable10))?;
    let _timer = node.create_timer_repeating(TICK, {
        let node = Arc::clone(&node);
        let shared = Arc::clone(shared);
        let mut heartbeat = Heartbeat::default();
        let mut button = EstopButton::default();
        let mut presses_seen = 0;
        let mut failed = false;
        move || {
            let now = Instant::now();
            // The buttons: only while the window draws.
            let drawn = Frames(shared.frames.load(Ordering::Relaxed));
            if heartbeat.poll(now, drawn).is_some() {
                let command = *shared
                    .command
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner);
                if let Some(c) = command {
                    match buttons_pub.publish(twist(c)) {
                        Ok(()) => failed = false,
                        Err(e) if !failed => {
                            failed = true;
                            log_error!(node.logger(), "the buttons' command wasn't published: {e}");
                        }
                        Err(_) => {}
                    }
                }
            }
            // The E-STOP: true 5 times, never false.
            let presses = shared.estop_presses.load(Ordering::SeqCst);
            if presses != presses_seen {
                presses_seen = presses;
                log_warn!(
                    node.logger(),
                    "e-stop pressed on the hardware panel: publishing true"
                );
                button.press(now);
            }
            if button.poll(now).is_some() {
                let sent = match estop_pub.publish(Bool { data: true }) {
                    Ok(()) => Some(estop_pub.get_subscription_count().unwrap_or(0)),
                    Err(e) => {
                        log_error!(node.logger(), "the e-stop wasn't published: {e}");
                        None
                    }
                };
                send(&tx, Update::EstopSent(sent));
            }
        }
    })?;

    if let Some(r) = ready.take() {
        r.send(Ok((config, Arc::clone(executor.commands()))))
            .map_err(|_| anyhow!("the window closed while the node started"))?;
    }
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// Sends one update to the screen; drops it if the screen is far behind.
fn send(tx: &SyncSender<Update>, update: Update) {
    match tx.try_send(update) {
        Ok(()) | Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {}
    }
}

/// The buttons' command as twist_mux takes it: base_link, stamp 0 (see the module's notes).
fn twist(c: Command) -> TwistStamped {
    TwistStamped {
        header: Header {
            frame_id: "base_link".into(),
            ..Default::default()
        },
        twist: Twist {
            linear: Vector3 {
                x: c.speed,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: c.yaw_rate,
            },
        },
    }
}

fn vehicle(m: &VehicleStatusMsg) -> VehicleStatus {
    VehicleStatus {
        mode: Mode::from(m.mode),
        armed: m.armed,
        estop_active: m.estop_active,
        speed: MetresPerSecond(m.speed),
        steering_angle: Radians(m.steering_angle),
        throttle: Fraction::new(m.throttle),
        brake: Fraction::new(m.brake),
        faults: Faults(m.faults),
    }
}

fn fix(m: &NavSatFix) -> Fix {
    Fix::from(RawFix {
        status: m.status.status,
        latitude: m.latitude,
        longitude: m.longitude,
        altitude: m.altitude,
        position_covariance: m.position_covariance,
        position_covariance_type: m.position_covariance_type,
    })
}

fn announcement(m: &DataSource) -> Announcement {
    Announcement {
        announcer: m.announcer.clone(),
        entries: m
            .sensors
            .iter()
            .map(|e| Entry {
                name: e.name.clone(),
                topic: e.topic.clone(),
                origin: Origin::from(e.source),
            })
            .collect(),
    }
}

/// The node's parameters: each topic's name, and the numbers. Read once.
struct Params {
    _names: Vec<ReadOnlyParameter<Arc<str>>>,
    _numbers: Vec<ReadOnlyParameter<f64>>,
}

impl Params {
    /// Declares them, with `config` (config/hw_panel.yaml) as the defaults, and returns
    /// the settings as ROS has them (a --params-file or -p wins).
    fn declare(node: &Node, mut config: PanelConfig) -> Result<(Self, PanelConfig)> {
        let mut names = Vec::new();
        for t in Topic::ALL {
            let p = node
                .declare_parameter::<Arc<str>>(t.parameter())
                .default(config.topic(t).into())
                .read_only()?;
            config.set_topic(t, p.get().to_string());
            names.push(p);
        }
        let mut numbers = Vec::new();
        for n in Number::ALL {
            let p = node
                .declare_parameter(n.parameter())
                .default(config.number(n))
                .read_only()?;
            config.set_number(n, p.get());
            numbers.push(p);
        }
        config.validate()?;
        let params = Self {
            _names: names,
            _numbers: numbers,
        };
        Ok((params, config))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::DataSource as _;
    use std::sync::atomic::AtomicUsize;

    /// The real node, as twist_mux would hear it: no command before the window draws;
    /// the buttons' command at about 10 Hz while it draws; none once it stops drawing,
    /// even with a command still held; and an E-STOP press is five trues, never false.
    /// Needs ROS sourced; runs on this computer only, on ROS_DOMAIN_ID (95 unless set),
    /// with topics of its own.
    #[test]
    fn sends_only_while_the_window_draws_and_the_estop_is_five_trues() -> Result<()> {
        if std::env::var_os("ROS_DOMAIN_ID").is_none() {
            std::env::set_var("ROS_DOMAIN_ID", "95");
        }
        std::env::set_var("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST");
        let prefix = format!("/hw_panel_test_{}", std::process::id());
        let (buttons, estop) = (format!("{prefix}/buttons"), format!("{prefix}/estop"));
        let mut config = PanelConfig::default();
        config.set_topic(Topic::Buttons, buttons.clone());
        config.set_topic(Topic::Estop, estop.clone());
        let (mut panel, _) = RosSource::start(vec!["hw_panel".into()], config)?;

        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let listener = executor.create_node("hw_panel_test_listener")?;
        let commands = Arc::new(Mutex::new(Vec::<f64>::new()));
        let _sub = listener.create_subscription(buttons.as_str().reliable().keep_last(10), {
            let commands = Arc::clone(&commands);
            move |m: TwistStamped| {
                commands
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .push(m.twist.linear.x)
            }
        })?;
        let (trues, falses) = (Arc::new(AtomicUsize::new(0)), Arc::new(AtomicUsize::new(0)));
        let _estop_sub =
            listener.create_subscription(estop.as_str().reliable().keep_last(10), {
                let (trues, falses) = (Arc::clone(&trues), Arc::clone(&falses));
                move |m: Bool| {
                    let n = if m.data { &trues } else { &falses };
                    n.fetch_add(1, Ordering::SeqCst);
                }
            })?;
        let count = || {
            commands
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .len()
        };
        let spin = |executor: &mut Executor, ms| {
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(ms)));
        };

        spin(&mut executor, 1500);
        assert_eq!(count(), 0, "a command before the first frame");

        // Draw at about 30 fps for 2 s, holding 0.3 m/s.
        let held = Command::from_steering(0.3, 0.0, 1.57);
        let end = Instant::now() + Duration::from_secs(2);
        while Instant::now() < end {
            panel.frame_drawn(Some(held));
            spin(&mut executor, 33);
        }
        let drawing = count();
        assert!((15..=21).contains(&drawing), "{drawing} commands in 2 s");
        assert!(commands
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .iter()
            .all(|&v| (v - 0.3).abs() < 1e-9));

        // The window freezes with the command still held: nothing more goes out.
        spin(&mut executor, 1000);
        assert!(
            count() <= drawing + 1,
            "{} after the window froze",
            count() - drawing
        );

        panel.press_estop();
        let deadline = Instant::now() + Duration::from_secs(3);
        while trues.load(Ordering::SeqCst) < 5 && Instant::now() < deadline {
            spin(&mut executor, 100);
        }
        spin(&mut executor, 1000);
        assert_eq!(trues.load(Ordering::SeqCst), 5);
        assert_eq!(falses.load(Ordering::SeqCst), 0);
        Ok(())
    }
}
