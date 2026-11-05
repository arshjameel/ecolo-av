//! The ROS side of the HUD: the node `hud`, spun by rclrs on its own thread.
//!
//! raylib must own the main thread (macOS draws only from there), so the node lives on
//! a thread called "ros". Its callbacks turn each message into a hud_core `Update` and
//! send it over a channel; the screen drains the channel every frame. The e-stop button
//! goes the other way, as a command; a 10 ms timer on the ROS thread sends the burst of
//! `true`s (hud_core::estop), so a slow frame can't delay it.
//!
//! The same timer sends the heartbeat (hud_core::heartbeat): the window counts the frames
//! it draws, and a beat goes out only while that count rises. So a beat proves both the
//! screen and the timer that sends the e-stop are running.
//!
//! QoS, as aav_hud's (docs/hud.md):
//! - small messages: best effort (SensorDataQoS), which takes reliable and best-effort
//!   publishers alike (ublox publishes /sensors/gps/fix best effort);
//! - pictures: reliable, depth 1; best effort loses most big messages over WiFi;
//! - the e-stop: reliable, depth 10, what spine_bridge and legacy_uart_bridge use;
//! - the heartbeat: reliable, depth 10, as the sensor watchdog subscribes;
//! - the watchdog's and the output mux's one-line states: reliable, depth 1;
//! - `/system/source`: reliable, transient local, depth 1, so the HUD gets each
//!   announcer's one message even when it starts later.

use std::{
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender, TrySendError},
        Arc,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use anyhow::{anyhow, Context as _, Result};
use hud_core::{
    config::{HudConfig, Topic, NODE_NAME},
    estop::EstopButton,
    gps::{Fix, RawFix},
    heartbeat::{Frames, Heartbeat},
    origin::{Announcement, Entry, Origin},
    picture::Picture,
    state::{Battery, EstopReport, Update},
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Faults, Mode, VehicleStatus},
};
use rclrs::*;
use ros_env::aav_interfaces::msg::{DataSource, VehicleStatus as VehicleStatusMsg};
use ros_env::sensor_msgs::msg::{BatteryState, Image, NavSatFix};
use ros_env::std_msgs::msg::{Bool, String as StringMsg, UInt64};

/// How many updates may wait for the screen. More are dropped: the screen is stuck,
/// and news that old is no use.
const QUEUE: usize = 256;
/// How often the ROS thread checks the e-stop button.
const ESTOP_TICK: Duration = Duration::from_millis(10);

/// What the screen asks the ROS thread to do.
enum Command {
    PressEstop,
}

/// The node, seen from the screen.
pub struct RosSource {
    updates: Receiver<Update>,
    commands: SyncSender<Command>,
    /// The frames the window has drawn: the heartbeat's count.
    frames: Arc<AtomicU64>,
    stopped: Arc<AtomicBool>,
    executor: Arc<ExecutorCommands>,
    thread: Option<JoinHandle<()>>,
}

/// What the ROS thread hands back once the node is up.
type Ready = Result<(HudConfig, Arc<ExecutorCommands>)>;

impl RosSource {
    /// Starts the node on its own thread and waits until it's up. `args` is the whole
    /// command line: ROS reads what follows `--ros-args`. Returns the settings with any
    /// ROS parameters applied.
    pub fn start(args: Vec<String>, config: HudConfig) -> Result<(Self, HudConfig)> {
        let (ready_tx, ready_rx) = mpsc::channel::<Ready>();
        let (update_tx, updates) = mpsc::sync_channel(QUEUE);
        let (commands, command_rx) = mpsc::sync_channel(16);
        let stopped = Arc::new(AtomicBool::new(false));
        let frames = Arc::new(AtomicU64::new(0));
        let thread = {
            let stopped = Arc::clone(&stopped);
            let frames = Arc::clone(&frames);
            thread::Builder::new().name("ros".into()).spawn(move || {
                let mut ready = Some(ready_tx);
                let links = Links {
                    updates: update_tx,
                    commands: command_rx,
                    frames,
                };
                if let Err(e) = spin(args, config, links, &mut ready) {
                    match ready.take() {
                        Some(tx) => {
                            let _ = tx.send(Err(e));
                        }
                        None => eprintln!("hud: the ROS node stopped: {e:#}"),
                    }
                }
                stopped.store(true, Ordering::SeqCst);
            })?
        };
        let (config, executor) = ready_rx
            .recv()
            .map_err(|_| anyhow!("the ROS thread ended before the node was up"))?
            .context("starting the ROS node (is ROS sourced? try tools/ros.sh, or --demo)")?;

        // Ctrl-C, or launch stopping us: stop spinning; the window sees it and closes.
        {
            let stopped = Arc::clone(&stopped);
            let executor = Arc::clone(&executor);
            ctrlc::set_handler(move || {
                stopped.store(true, Ordering::SeqCst);
                executor.halt_spinning();
            })?;
        }
        let source = Self {
            updates,
            commands,
            frames,
            stopped,
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
        if self.commands.try_send(Command::PressEstop).is_err() {
            eprintln!("hud: the e-stop press didn't reach the ROS thread");
        }
    }

    fn frame_drawn(&mut self) {
        self.frames.fetch_add(1, Ordering::Relaxed);
    }

    fn has_stopped(&self) -> bool {
        self.stopped.load(Ordering::SeqCst)
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

/// What joins the ROS thread to the window.
struct Links {
    /// News for the screen.
    updates: SyncSender<Update>,
    /// The screen's requests (the e-stop button).
    commands: Receiver<Command>,
    /// The frames the window has drawn, for the heartbeat.
    frames: Arc<AtomicU64>,
}

/// The ROS thread: makes the node, says it's ready, then spins until halted.
fn spin(
    args: Vec<String>,
    config: HudConfig,
    links: Links,
    ready: &mut Option<mpsc::Sender<Ready>>,
) -> Result<()> {
    let Links {
        updates: tx,
        commands,
        frames,
    } = links;
    let context = Context::new(args, InitOptions::default())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME)?;
    // Keep the parameters alive with the node: rclrs undeclares one when it's dropped.
    let (_params, config) = Params::declare(&node, config)?;

    let _vehicle =
        node.create_subscription(config.topic(Topic::VehicleStatus).sensor_data_qos(), {
            let tx = tx.clone();
            move |m: VehicleStatusMsg| send(&tx, Update::Vehicle(vehicle(&m)))
        })?;
    let _gps = node.create_subscription(config.topic(Topic::Gps).sensor_data_qos(), {
        let tx = tx.clone();
        move |m: NavSatFix| send(&tx, Update::Gps(fix(&m)))
    })?;
    let _gps_filtered =
        node.create_subscription(config.topic(Topic::GpsFiltered).sensor_data_qos(), {
            let tx = tx.clone();
            move |m: NavSatFix| send(&tx, Update::GpsFiltered(fix(&m)))
        })?;
    let _stop = node.create_subscription(config.topic(Topic::StopSign).sensor_data_qos(), {
        let tx = tx.clone();
        move |m: Bool| send(&tx, Update::StopSign(m.data))
    })?;
    let _battery = node.create_subscription(config.topic(Topic::Battery).sensor_data_qos(), {
        let tx = tx.clone();
        move |m: BatteryState| {
            let b = Battery::from_message(m.voltage, m.percentage, m.current);
            send(&tx, Update::Battery(b))
        }
    })?;
    let pictures = QoSProfile::topics_default().reliable().keep_last(1);
    let _camera = node.create_subscription(config.topic(Topic::Camera).qos(pictures), {
        let tx = tx.clone();
        move |m: Box<Image>| {
            let update = Picture::from_image(&m.encoding, m.width, m.height, m.step, &m.data)
                .map_or_else(|e| Update::BadPicture(e.to_string()), Update::Picture);
            send(&tx, update)
        }
    })?;

    let _watchdog = node.create_subscription(
        config
            .topic(Topic::Watchdog)
            .qos(QoSProfile::topics_default().reliable().keep_last(1)),
        {
            let tx = tx.clone();
            move |m: StringMsg| send(&tx, Update::Watchdog(m.data))
        },
    )?;
    let _output_mux = node.create_subscription(
        config
            .topic(Topic::OutputMux)
            .qos(QoSProfile::topics_default().reliable().keep_last(1)),
        {
            let tx = tx.clone();
            move |m: StringMsg| send(&tx, Update::OutputMux(m.data))
        },
    )?;

    let latched = QoSProfile::topics_default()
        .reliable()
        .transient_local()
        .keep_last(1);
    let _source = node.create_subscription(config.topic(Topic::Source).qos(latched), {
        let tx = tx.clone();
        move |m: DataSource| send(&tx, Update::Source(announcement(&m)))
    })?;

    let estop_qos = QoSProfile::topics_default().reliable().keep_last(10);
    let estop_pub: Publisher<Bool> =
        node.create_publisher(config.topic(Topic::Estop).qos(estop_qos))?;
    let heartbeat_qos = QoSProfile::topics_default().reliable().keep_last(10);
    let heartbeat_pub: Publisher<UInt64> =
        node.create_publisher(config.topic(Topic::Heartbeat).qos(heartbeat_qos))?;
    // One timer for both: a beat says the e-stop's sender runs too.
    let _estop_timer = node.create_timer_repeating(ESTOP_TICK, {
        let node = Arc::clone(&node);
        let mut button = EstopButton::default();
        let mut heartbeat = Heartbeat::default();
        let mut beat_failed = false;
        move || {
            let drawn = Frames(frames.load(Ordering::Relaxed));
            if let Some(Frames(count)) = heartbeat.poll(Instant::now(), drawn) {
                match heartbeat_pub.publish(UInt64 { data: count }) {
                    Ok(()) => beat_failed = false,
                    // Once per run of failures, not 10 times a second.
                    Err(e) if !beat_failed => {
                        beat_failed = true;
                        log_error!(node.logger(), "the heartbeat wasn't published: {e}");
                    }
                    Err(_) => {}
                }
            }
            while let Ok(Command::PressEstop) = commands.try_recv() {
                log_warn!(node.logger(), "e-stop pressed on the HUD: publishing true");
                button.press(Instant::now());
            }
            if button.poll(Instant::now()).is_none() {
                return;
            }
            let report = match estop_pub.publish(Bool { data: true }) {
                Ok(()) => {
                    let subscribers = estop_pub.get_subscription_count().unwrap_or(0);
                    if subscribers == 0 {
                        log_error!(node.logger(), "nothing subscribes to the e-stop topic");
                    }
                    EstopReport::Sent { subscribers }
                }
                Err(e) => {
                    log_error!(node.logger(), "the e-stop wasn't published: {e}");
                    EstopReport::Failed
                }
            };
            send(&tx, Update::Estop(report));
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

/// The node's parameters: each topic's name, stale_after, max_speed and the theme. Read once, at
/// start-up.
struct Params {
    _topics: Vec<ReadOnlyParameter<Arc<str>>>,
    _stale_after: ReadOnlyParameter<f64>,
    _max_speed: ReadOnlyParameter<f64>,
    _theme: ReadOnlyParameter<Arc<str>>,
}

impl Params {
    /// Declares them, with `config` (config/hud.yaml) as the defaults, and returns the
    /// settings as ROS has them (a --params-file or -p wins).
    fn declare(node: &Node, mut config: HudConfig) -> Result<(Self, HudConfig)> {
        let mut topics = Vec::new();
        for t in Topic::LISTENED.into_iter().chain(Topic::UNLISTED) {
            let p = node
                .declare_parameter::<Arc<str>>(t.parameter())
                .default(config.topic(t).into())
                .read_only()?;
            config.set_topic(t, p.get().to_string());
            topics.push(p);
        }
        let stale_after = node
            .declare_parameter("stale_after")
            .default(config.stale_after)
            .read_only()?;
        config.stale_after = stale_after.get();
        let max_speed = node
            .declare_parameter("max_speed")
            .default(config.max_speed)
            .read_only()?;
        config.max_speed = max_speed.get();
        let theme = node
            .declare_parameter::<Arc<str>>("theme")
            .default(config.theme.as_str().into())
            .read_only()?;
        config.theme = theme.get().parse()?;
        config.validate()?;
        let params = Self {
            _topics: topics,
            _stale_after: stale_after,
            _max_speed: max_speed,
            _theme: theme,
        };
        Ok((params, config))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// hud_core's numbers must match the message's constants.
    #[test]
    fn constants_match_the_message() {
        assert_eq!(Mode::from(VehicleStatusMsg::MODE_MANUAL), Mode::Manual);
        assert_eq!(Mode::from(VehicleStatusMsg::MODE_RC), Mode::Rc);
        assert_eq!(Mode::from(VehicleStatusMsg::MODE_COMPUTER), Mode::Computer);
        let bits = [
            VehicleStatusMsg::FAULT_CMD_TIMEOUT,
            VehicleStatusMsg::FAULT_BAD_CHECKSUM,
            VehicleStatusMsg::FAULT_BAD_COUNTER,
            VehicleStatusMsg::FAULT_STEERING_DRIVER,
            VehicleStatusMsg::FAULT_BRAKE_DRIVER,
            VehicleStatusMsg::FAULT_THROTTLE_DAC,
            VehicleStatusMsg::FAULT_WHEEL_ENCODER,
            VehicleStatusMsg::FAULT_CAN_BUS,
            VehicleStatusMsg::FAULT_RC_SIGNAL_LOST,
            VehicleStatusMsg::FAULT_OVERSPEED,
        ];
        let ours: Vec<u16> = hud_core::vehicle::Fault::ALL
            .iter()
            .map(|f| f.bit())
            .collect();
        assert_eq!(ours, bits);
    }

    /// hud_core's origins must match the message's constants.
    #[test]
    fn origins_match_the_message() {
        use ros_env::aav_interfaces::msg::SensorSource;
        assert_eq!(Origin::from(SensorSource::REAL), Origin::Real);
        assert_eq!(Origin::from(SensorSource::SIM), Origin::Sim);
        assert_eq!(Origin::from(SensorSource::BENCH), Origin::Bench);
        assert_eq!(Origin::from(SensorSource::REPLAY), Origin::Replay);
    }

    /// The real node, as another node hears it: beats 10 times a second while the window
    /// draws, with a rising count, and none before or after; and one press of the button
    /// is five trues, with the window frozen too, each reported to the screen as sent to a
    /// subscriber. Then the output mux's line reaches the screen, as the GO / STOP panel
    /// reads it. One test: the Ctrl-C handler can be set once per process. Needs ROS
    /// sourced; runs on this computer only, on ROS_DOMAIN_ID (93 unless set), with topics
    /// of its own.
    #[test]
    fn beats_while_the_window_draws_and_a_press_is_five_trues() -> Result<()> {
        use crate::source::DataSource;
        use std::sync::{atomic::AtomicUsize, Mutex};

        if std::env::var_os("ROS_DOMAIN_ID").is_none() {
            std::env::set_var("ROS_DOMAIN_ID", "93");
        }
        std::env::set_var("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST");
        let prefix = format!("/hud_test_{}", std::process::id());
        let (estop, heartbeat) = (format!("{prefix}/estop"), format!("{prefix}/heartbeat"));
        let mut config = HudConfig::default();
        config.set_topic(Topic::Estop, estop.clone());
        config.set_topic(Topic::Heartbeat, heartbeat.clone());
        let gate = format!("{prefix}/output_mux_state");
        config.set_topic(Topic::OutputMux, gate.clone());
        let (mut hud, _) = RosSource::start(vec!["hud".into()], config)?;

        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let listener = executor.create_node("hud_test_listener")?;
        let trues = Arc::new(AtomicUsize::new(0));
        let falses = Arc::new(AtomicUsize::new(0));
        let _estop_sub =
            listener.create_subscription(estop.as_str().reliable().keep_last(10), {
                let (trues, falses) = (Arc::clone(&trues), Arc::clone(&falses));
                move |m: Bool| {
                    let n = if m.data { &trues } else { &falses };
                    n.fetch_add(1, Ordering::SeqCst);
                }
            })?;
        let beats = Arc::new(Mutex::new(Vec::<u64>::new()));
        let _beat_sub =
            listener.create_subscription(heartbeat.as_str().reliable().keep_last(10), {
                let beats = Arc::clone(&beats);
                move |m: UInt64| beats.lock().expect("not poisoned").push(m.data)
            })?;
        let count = || beats.lock().expect("not poisoned").len();
        let spin = |executor: &mut Executor, ms| {
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(ms)));
        };

        // Discovery, and nothing drawn yet: no beats, however long the node runs.
        spin(&mut executor, 1500);
        assert_eq!(count(), 0, "beats before the first frame");

        // Draw at about 30 frames a second for 2 s.
        let end = Instant::now() + Duration::from_secs(2);
        while Instant::now() < end {
            hud.frame_drawn();
            spin(&mut executor, 33);
        }
        let drawing = count();
        assert!((15..=21).contains(&drawing), "{drawing} beats in 2 s");
        {
            let got = beats.lock().expect("not poisoned");
            assert!(got.windows(2).all(|w| w[1] > w[0]), "{got:?}");
        }

        // The window stops drawing; the button still works.
        hud.press_estop();
        let deadline = Instant::now() + Duration::from_secs(3);
        while trues.load(Ordering::SeqCst) < 5 && Instant::now() < deadline {
            spin(&mut executor, 100);
        }
        // A little longer: a sixth would be a bug, and so would more beats.
        spin(&mut executor, 1000);
        assert_eq!(trues.load(Ordering::SeqCst), 5);
        assert_eq!(falses.load(Ordering::SeqCst), 0);
        // At most one beat already on its way when the window stopped.
        assert!(
            count() <= drawing + 1,
            "{} beats after the window stopped",
            count() - drawing
        );
        let reports: Vec<Update> = hud
            .drain(Instant::now())
            .into_iter()
            .filter(|u| matches!(u, Update::Estop(_)))
            .collect();
        assert_eq!(reports.len(), 5);
        assert!(reports.iter().all(
            |u| matches!(u, Update::Estop(EstopReport::Sent { subscribers }) if *subscribers >= 1)
        ));

        // The output mux says it latched: the screen gets its line.
        let gate_pub: Publisher<StringMsg> =
            listener.create_publisher(gate.as_str().reliable().keep_last(10))?;
        let line = "STOP - target car needs REAL - gps is SIM (gazebo_adapter) - latched: x";
        let mut heard = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(5);
        while heard.is_empty() && Instant::now() < deadline {
            gate_pub.publish(StringMsg { data: line.into() })?;
            spin(&mut executor, 200);
            heard.extend(
                hud.drain(Instant::now())
                    .into_iter()
                    .filter(|u| matches!(u, Update::OutputMux(_))),
            );
        }
        assert!(matches!(heard.first(), Some(Update::OutputMux(l)) if l == line));
        Ok(())
    }
}
