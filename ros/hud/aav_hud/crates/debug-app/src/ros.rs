//! The ROS side of the debug window: the node `debug_hud`, spun by rclrs on its own
//! thread, and a second thread that turns clouds into cubes.
//!
//! As the HUD's: raylib must own the main thread, so the node lives on a thread called
//! "ros", and its callbacks turn each message into a debug_core `Update`, sent over a
//! channel. The lidar's clouds never go through that channel:
//!
//! ```text
//! lidar callback --newest-only slot--> "cubes" thread --newest-only slot--> the screen
//!                 (one cloud at most)  (frames, pose at   (one scan at most)
//!                                       the stamp, cubes)
//! ```
//!
//! The local EKF's callback fills the pose buffer the cubes thread reads; `/tf_static`
//! fills the static frames. Both are small and cloned under their lock, so the cubes
//! thread never holds a lock while it parses a cloud.
//!
//! QoS:
//! - the lidar: reliable, keep last 1 (as Gazebo's bridge and velodyne send it; a slow
//!   window gets the newest cloud, not a backlog);
//! - `/tf`: reliable, depth 100; `/tf_static`: reliable, transient local, depth 100, so
//!   a late window still gets robot_state_publisher's frames;
//! - `/system/source`: reliable, transient local, depth 1 (latched), as the HUD;
//! - the plan: reliable, keep last 1, as the map window;
//! - AMCL's and NDT's poses: reliable (as they publish);
//! - AMCL's particles: best effort, as AMCL publishes them;
//! - everything else: best effort (SensorDataQoS), which takes reliable publishers too.
//!
//! Keys 1-4 call aav_track's own parameter service (`track_parameters_service`,
//! `/track/set_parameters`) from the window's thread: the screen has already checked
//! `/system/source` says SIM, and the track node refuses unless `live_swap` is on. The
//! answer comes back as an `Update::SwapReply`.

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver, SyncSender, TrySendError},
        Arc, Condvar, Mutex, PoisonError,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use anyhow::{anyhow, Context as _, Result};
use debug_core::{
    cloud::{CloudView, Field},
    config::{DebugConfig, Frame, Number, Topic, NODE_NAME},
    cubes::{CubeScan, CubeSettings},
    frames::{frame_name, Stamp, StaticFrames, Transform3},
    ndt::NdtStatus,
    poses::PoseBuffer,
    scan::{self, ScanError},
    state::{Localizer, Source, Update},
    swap::{Reply, Tracker},
    track::TrackStatus,
};
use hud_core::{
    origin::{Announcement, Entry, Origin},
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Faults, Mode, VehicleStatus},
};
use map_core::geom::{Point, Pose, Quaternion};
use rclrs::*;
use ros_env::aav_interfaces::msg::{DataSource, VehicleStatus as VehicleStatusMsg, WheelSpeeds};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::DiagnosticStatus;
use ros_env::geometry_msgs::msg::{
    Pose as PoseMsg, PoseWithCovarianceStamped, Quaternion as QuaternionMsg,
};
use ros_env::nav2_msgs::msg::ParticleCloud;
use ros_env::nav_msgs::msg::{Odometry, Path};
use ros_env::rcl_interfaces::msg::{Parameter, ParameterType, ParameterValue};
use ros_env::rcl_interfaces::srv::{SetParameters, SetParameters_Request, SetParameters_Response};
use ros_env::sensor_msgs::msg::{Imu, NavSatFix, PointCloud2};
use ros_env::tf2_msgs::msg::TFMessage;

/// How many updates may wait for the screen. More are dropped: the screen is stuck.
const QUEUE: usize = 512;
/// How long the cubes thread waits for the pose at a cloud's stamp to arrive.
const POSE_WAIT: Duration = Duration::from_millis(300);
const POSE_RETRY: Duration = Duration::from_millis(10);

/// A newest-only slot: putting replaces what is there.
struct Slot<T> {
    value: Mutex<Option<T>>,
    ready: Condvar,
}

impl<T> Slot<T> {
    fn new() -> Self {
        Self {
            value: Mutex::new(None),
            ready: Condvar::new(),
        }
    }

    fn put(&self, v: T) {
        *self.value.lock().unwrap_or_else(PoisonError::into_inner) = Some(v);
        self.ready.notify_one();
    }

    fn take(&self) -> Option<T> {
        self.value
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take()
    }

    fn is_empty(&self) -> bool {
        self.value
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .is_none()
    }

    /// Waits up to `timeout` for a value, then takes it.
    fn wait(&self, timeout: Duration) -> Option<T> {
        let guard = self.value.lock().unwrap_or_else(PoisonError::into_inner);
        let (mut guard, _) = self
            .ready
            .wait_timeout_while(guard, timeout, |v| v.is_none())
            .unwrap_or_else(PoisonError::into_inner);
        guard.take()
    }
}

/// What the cubes thread shares with the node.
struct Shared {
    clouds: Slot<PointCloud2>,
    scans: Slot<CubeScan>,
    poses: Mutex<PoseBuffer>,
    statics: Mutex<StaticFrames>,
    stopped: AtomicBool,
}

/// The node, seen from the screen.
pub struct RosSource {
    updates: Receiver<Update>,
    /// For the swap's answer, which arrives on the ROS thread.
    tx: SyncSender<Update>,
    track_parameters: Client<SetParameters>,
    shared: Arc<Shared>,
    executor: Arc<ExecutorCommands>,
    threads: Vec<JoinHandle<()>>,
}

/// What the ROS thread hands back once the node is up.
type Ready = Result<(DebugConfig, Arc<ExecutorCommands>, Client<SetParameters>)>;

impl RosSource {
    /// Starts the node and the cubes thread, and waits until the node is up. `args` is
    /// the whole command line: ROS reads what follows `--ros-args`.
    pub fn start(args: Vec<String>, config: DebugConfig) -> Result<(Self, DebugConfig)> {
        let (ready_tx, ready_rx) = mpsc::channel::<Ready>();
        let (update_tx, updates) = mpsc::sync_channel(QUEUE);
        let shared = Arc::new(Shared {
            clouds: Slot::new(),
            scans: Slot::new(),
            poses: Mutex::new(PoseBuffer::new(config.pose_window())),
            statics: Mutex::new(StaticFrames::default()),
            stopped: AtomicBool::new(false),
        });
        let ros = {
            let shared = Arc::clone(&shared);
            let tx = update_tx.clone();
            thread::Builder::new().name("ros".into()).spawn(move || {
                let mut ready = Some(ready_tx);
                if let Err(e) = spin(args, config, tx, &shared, &mut ready) {
                    match ready.take() {
                        Some(r) => {
                            let _ = r.send(Err(e));
                        }
                        None => eprintln!("debug_hud: the ROS node stopped: {e:#}"),
                    }
                }
                shared.stopped.store(true, Ordering::SeqCst);
                shared.clouds.ready.notify_all();
            })?
        };
        let (config, executor, track_parameters) = ready_rx
            .recv()
            .map_err(|_| anyhow!("the ROS thread ended before the node was up"))?
            .context("starting the ROS node (is ROS sourced? try tools/ros.sh, or --demo)")?;
        let cubes = {
            let shared = Arc::clone(&shared);
            let base = config.frame(Frame::Base).to_owned();
            let settings = config.cube_settings();
            let tx = update_tx.clone();
            thread::Builder::new()
                .name("cubes".into())
                .spawn(move || make_cubes(&shared, &base, &settings, &tx))?
        };

        // Ctrl-C, or launch stopping us: stop spinning; the window sees it and closes.
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
            tx: update_tx,
            track_parameters,
            shared,
            executor,
            threads: vec![ros, cubes],
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

    fn take_scan(&mut self) -> Option<CubeScan> {
        self.shared.scans.take()
    }

    fn swap_tracker(&mut self, tracker: Tracker) -> Result<()> {
        if !self.track_parameters.service_is_ready()? {
            anyhow::bail!("no track node (start the sim with tracker:=pure_pursuit)");
        }
        let tx = self.tx.clone();
        let asked = tracker.name();
        // The promise needn't be kept (rclrs): the callback runs when the answer comes.
        let promise = self.track_parameters.call_then(
            &tracker_request(asked),
            move |response: SetParameters_Response| send(&tx, swap_reply(asked, &response)),
        )?;
        drop(promise);
        Ok(())
    }

    fn has_stopped(&self) -> bool {
        self.shared.stopped.load(Ordering::SeqCst)
    }
}

impl Drop for RosSource {
    fn drop(&mut self) {
        self.shared.stopped.store(true, Ordering::SeqCst);
        self.shared.clouds.ready.notify_all();
        self.executor.halt_spinning();
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

/// The cubes thread: the newest cloud, placed with the pose at its stamp, into cubes.
fn make_cubes(shared: &Shared, base: &str, settings: &CubeSettings, tx: &SyncSender<Update>) {
    while !shared.stopped.load(Ordering::SeqCst) {
        let Some(cloud) = shared.clouds.wait(Duration::from_millis(100)) else {
            continue;
        };
        let waited = Instant::now();
        loop {
            // Small copies, so no lock is held while the cloud is read.
            let statics = shared
                .statics
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone();
            let poses = shared
                .poses
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone();
            match place(&cloud, base, &statics, &poses, settings) {
                Ok(scan) => {
                    shared.scans.put(scan);
                    break;
                }
                // The pose at its stamp may still be on its way; a newer cloud wins.
                Err(e)
                    if e.is_too_new()
                        && waited.elapsed() < POSE_WAIT
                        && shared.clouds.is_empty()
                        && !shared.stopped.load(Ordering::SeqCst) =>
                {
                    thread::sleep(POSE_RETRY);
                }
                Err(e) => {
                    send(tx, Update::ScanProblem(e.to_string()));
                    break;
                }
            }
        }
    }
}

fn place(
    m: &PointCloud2,
    base: &str,
    statics: &StaticFrames,
    poses: &PoseBuffer,
    settings: &CubeSettings,
) -> Result<CubeScan, ScanError> {
    let view = CloudView {
        fields: m
            .fields
            .iter()
            .map(|f| Field {
                name: &f.name,
                offset: f.offset,
                datatype: f.datatype,
                count: f.count,
            })
            .collect(),
        width: m.width,
        height: m.height,
        point_step: m.point_step,
        row_step: m.row_step,
        is_bigendian: m.is_bigendian,
        data: &m.data,
    };
    scan::place(
        &view,
        &m.header.frame_id,
        stamp(&m.header.stamp),
        base,
        statics,
        poses,
        settings,
    )
}

/// The ROS thread: makes the node, says it's ready, then spins until halted.
fn spin(
    args: Vec<String>,
    config: DebugConfig,
    tx: SyncSender<Update>,
    shared: &Arc<Shared>,
    ready: &mut Option<mpsc::Sender<Ready>>,
) -> Result<()> {
    let context = Context::new(args, InitOptions::default())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME)?;
    // Keep the parameters alive with the node: rclrs undeclares one when it's dropped.
    let (_params, config) = Params::declare(&node, config)?;
    let topic = |t: Topic| config.topic(t).to_owned();

    let lidar_qos = QoSProfile::topics_default().reliable().keep_last(1);
    let _lidar = node.create_subscription(topic(Topic::Lidar).as_str().qos(lidar_qos), {
        let shared = Arc::clone(shared);
        move |m: PointCloud2| shared.clouds.put(m)
    })?;

    // The local EKF: the car, and the poses the cubes are placed with.
    let _local = node.create_subscription(topic(Topic::LocalOdom).as_str().sensor_data_qos(), {
        let tx = tx.clone();
        let shared = Arc::clone(shared);
        move |m: Odometry| {
            let p = pose(&m.pose.pose);
            shared
                .poses
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .push(stamp(&m.header.stamp), p);
            send(&tx, odometry(Localizer::Local, &m));
        }
    })?;
    let _global =
        node.create_subscription(topic(Topic::GlobalOdom).as_str().sensor_data_qos(), {
            let tx = tx.clone();
            move |m: Odometry| send(&tx, odometry(Localizer::Global, &m))
        })?;
    let _truth = node.create_subscription(topic(Topic::Truth).as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: Odometry| send(&tx, odometry(Localizer::Truth, &m))
    })?;
    let _amcl = node.create_subscription(topic(Topic::Amcl).as_str().reliable(), {
        let tx = tx.clone();
        move |m: PoseWithCovarianceStamped| {
            send(
                &tx,
                Update::Pose {
                    who: Localizer::Amcl,
                    pose: pose(&m.pose.pose),
                    frame: m.header.frame_id,
                    speed: None,
                },
            )
        }
    })?;
    let _ndt = node.create_subscription(topic(Topic::NdtPose).as_str().reliable(), {
        let tx = tx.clone();
        move |m: PoseWithCovarianceStamped| {
            send(
                &tx,
                Update::Pose {
                    who: Localizer::Ndt,
                    pose: pose(&m.pose.pose),
                    frame: m.header.frame_id,
                    speed: None,
                },
            )
        }
    })?;
    let _ndt_status =
        node.create_subscription(topic(Topic::NdtStatus).as_str().sensor_data_qos(), {
            let tx = tx.clone();
            move |m: DiagnosticStatus| send(&tx, Update::Ndt(ndt_status(&m)))
        })?;
    let _particles =
        node.create_subscription(topic(Topic::AmclParticles).as_str().sensor_data_qos(), {
            let tx = tx.clone();
            move |m: ParticleCloud| {
                let poses = m.particles.iter().map(|p| pose(&p.pose)).collect();
                send(
                    &tx,
                    Update::Particles {
                        poses,
                        frame: m.header.frame_id,
                    },
                )
            }
        })?;
    let plans = QoSProfile::topics_default().reliable().keep_last(1);
    let _plan = node.create_subscription(topic(Topic::Plan).as_str().qos(plans), {
        let tx = tx.clone();
        move |m: Path| {
            let points = m
                .poses
                .iter()
                .map(|p| Point::new(p.pose.position.x, p.pose.position.y))
                .collect();
            send(
                &tx,
                Update::Plan {
                    points,
                    frame: m.header.frame_id,
                },
            )
        }
    })?;

    // Frames.
    let (map, odom) = (
        config.frame(Frame::Map).to_owned(),
        config.frame(Frame::Odom).to_owned(),
    );
    let tf_qos = QoSProfile::topics_default().reliable().keep_last(100);
    let _tf = node.create_subscription(topic(Topic::Tf).as_str().qos(tf_qos), {
        let tx = tx.clone();
        move |m: TFMessage| {
            for t in &m.transforms {
                if frame_name(&t.header.frame_id) == frame_name(&map)
                    && frame_name(&t.child_frame_id) == frame_name(&odom)
                {
                    let tr = &t.transform;
                    let q = quaternion(&tr.rotation);
                    let odom_in_map = Pose::from_ros(tr.translation.x, tr.translation.y, q);
                    send(&tx, Update::MapToOdom(odom_in_map));
                }
            }
        }
    })?;
    let static_qos = QoSProfile::topics_default()
        .reliable()
        .transient_local()
        .keep_last(100);
    let _tf_static =
        node.create_subscription(topic(Topic::TfStatic).as_str().qos(static_qos), {
            let shared = Arc::clone(shared);
            move |m: TFMessage| {
                let mut statics = shared
                    .statics
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner);
                for t in &m.transforms {
                    let tr = &t.transform;
                    let at = [tr.translation.x, tr.translation.y, tr.translation.z];
                    let transform = Transform3::from_ros(at, quaternion(&tr.rotation));
                    statics.insert(&t.header.frame_id, &t.child_frame_id, transform);
                }
            }
        })?;

    // The panel: each sensor from its own topic.
    let _imu = node.create_subscription(topic(Topic::Imu).as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: Imu| {
            let rate = m.angular_velocity.z.to_degrees();
            send(
                &tx,
                heard(Source::Imu, format!("yaw rate {rate:+.1} deg/s")),
            )
        }
    })?;
    let _gps = node.create_subscription(topic(Topic::Gps).as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: NavSatFix| send(&tx, heard(Source::Gps, gps_detail(&m)))
    })?;
    let _wheels = node.create_subscription(topic(Topic::Wheels).as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: WheelSpeeds| send(&tx, heard(Source::Wheels, wheels_detail(&m)))
    })?;
    let _wheel_odom =
        node.create_subscription(topic(Topic::WheelOdom).as_str().sensor_data_qos(), {
            let tx = tx.clone();
            move |m: Odometry| {
                let v = m.twist.twist.linear.x;
                send(&tx, heard(Source::WheelOdom, format!("{v:+.2} m/s")))
            }
        })?;
    let _vehicle =
        node.create_subscription(topic(Topic::VehicleStatus).as_str().sensor_data_qos(), {
            let tx = tx.clone();
            move |m: VehicleStatusMsg| send(&tx, Update::Vehicle(vehicle(&m)))
        })?;
    let _track =
        node.create_subscription(topic(Topic::TrackStatus).as_str().sensor_data_qos(), {
            let tx = tx.clone();
            move |m: DiagnosticStatus| {
                let values = m
                    .values
                    .iter()
                    .map(|kv| (kv.key.as_str(), kv.value.as_str()));
                send(
                    &tx,
                    Update::Track(TrackStatus::from_values(&m.message, values)),
                )
            }
        })?;
    let latched = QoSProfile::topics_default()
        .reliable()
        .transient_local()
        .keep_last(1);
    let _source = node.create_subscription(topic(Topic::Source).as_str().qos(latched), {
        let tx = tx.clone();
        move |m: DataSource| send(&tx, Update::Origin(announcement(&m)))
    })?;

    let track_parameters =
        node.create_client::<SetParameters>(config.track_parameters_service.as_str())?;

    if let Some(r) = ready.take() {
        r.send(Ok((
            config,
            Arc::clone(executor.commands()),
            track_parameters,
        )))
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

/// `tracker := name`, as `ros2 param set /track tracker <name>` sends it.
fn tracker_request(name: &str) -> SetParameters_Request {
    SetParameters_Request {
        parameters: vec![Parameter {
            name: "tracker".into(),
            value: ParameterValue {
                type_: ParameterType::PARAMETER_STRING,
                string_value: name.into(),
                ..Default::default()
            },
        }],
    }
}

/// The node's answer: its one result, or why there was none.
fn swap_reply(asked: &'static str, response: &SetParameters_Response) -> Update {
    let outcome = match response.results.first() {
        Some(r) if r.successful => Ok(()),
        Some(r) => Err(r.reason.clone()),
        None => Err("no answer from the track node".into()),
    };
    Update::SwapReply(Reply { asked, outcome })
}

/// `/localization/ndt/status` in plain words.
fn ndt_status(m: &DiagnosticStatus) -> NdtStatus {
    let values = m
        .values
        .iter()
        .map(|kv| (kv.key.as_str(), kv.value.as_str()));
    NdtStatus::from_message(m.level, &m.message, values)
}

fn heard(source: Source, detail: String) -> Update {
    Update::Heard { source, detail }
}

fn stamp(t: &Time) -> Stamp {
    Stamp::new(t.sec, t.nanosec)
}

fn quaternion(q: &QuaternionMsg) -> Quaternion {
    Quaternion {
        x: q.x,
        y: q.y,
        z: q.z,
        w: q.w,
    }
}

fn pose(p: &PoseMsg) -> Pose {
    Pose::from_ros(p.position.x, p.position.y, quaternion(&p.orientation))
}

fn odometry(who: Localizer, m: &Odometry) -> Update {
    Update::Pose {
        who,
        pose: pose(&m.pose.pose),
        frame: m.header.frame_id.clone(),
        speed: Some(m.twist.twist.linear.x),
    }
}

/// The fix's kind (sensor_msgs/NavSatStatus) and its horizontal spread.
fn gps_detail(m: &NavSatFix) -> String {
    let kind = match m.status.status {
        s if s < 0 => "no fix",
        0 => "fix",
        1 => "SBAS fix",
        _ => "RTK / GBAS fix",
    };
    let spread = m.position_covariance[0].max(m.position_covariance[4]);
    if spread.is_finite() && spread > 0.0 {
        format!("{kind}, {:.1} m", spread.sqrt())
    } else {
        kind.to_owned()
    }
}

/// The measured wheels' mean speed.
fn wheels_detail(m: &WheelSpeeds) -> String {
    let measured: Vec<f32> = [m.front_left, m.front_right, m.rear_left, m.rear_right]
        .into_iter()
        .filter(|v| v.is_finite())
        .collect();
    if measured.is_empty() {
        return "none measured".to_owned();
    }
    let mean = measured.iter().sum::<f32>() / measured.len() as f32;
    format!("{mean:+.2} m/s ({} wheels)", measured.len())
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

/// The node's parameters: each topic and frame's name, and the numbers. Read once.
struct Params {
    _names: Vec<ReadOnlyParameter<Arc<str>>>,
    _numbers: Vec<ReadOnlyParameter<f64>>,
    _max_cubes: ReadOnlyParameter<i64>,
}

impl Params {
    /// Declares them, with `config` (config/debug_hud.yaml) as the defaults, and returns
    /// the settings as ROS has them (a --params-file or -p wins).
    fn declare(node: &Node, mut config: DebugConfig) -> Result<(Self, DebugConfig)> {
        let mut names = Vec::new();
        for t in Topic::ALL {
            let p = node
                .declare_parameter::<Arc<str>>(t.parameter())
                .default(config.topic(t).into())
                .read_only()?;
            config.set_topic(t, p.get().to_string());
            names.push(p);
        }
        for f in Frame::ALL {
            let p = node
                .declare_parameter::<Arc<str>>(f.parameter())
                .default(config.frame(f).into())
                .read_only()?;
            config.set_frame(f, p.get().to_string());
            names.push(p);
        }
        let service = node
            .declare_parameter::<Arc<str>>("track_parameters_service")
            .default(config.track_parameters_service.as_str().into())
            .read_only()?;
        config.track_parameters_service = service.get().to_string();
        names.push(service);
        let map = node
            .declare_parameter::<Arc<str>>("ndt_map")
            .default(config.ndt_map.as_str().into())
            .read_only()?;
        config.ndt_map = map.get().to_string();
        names.push(map);
        let mut numbers = Vec::new();
        for n in Number::ALL {
            let p = node
                .declare_parameter(n.parameter())
                .default(config.number(n))
                .read_only()?;
            config.set_number(n, p.get());
            numbers.push(p);
        }
        let max_cubes = node
            .declare_parameter("max_cubes")
            .default(i64::from(config.max_cubes))
            .read_only()?;
        config.max_cubes =
            u32::try_from(max_cubes.get()).context("max_cubes must be a positive whole number")?;
        config.validate()?;
        let params = Self {
            _names: names,
            _numbers: numbers,
            _max_cubes: max_cubes,
        };
        Ok((params, config))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_gazebo_like_cloud_becomes_cubes() {
        let mut m = PointCloud2::default();
        m.header.frame_id = "lidar_link".into();
        m.header.stamp.sec = 1;
        for (i, name) in ["x", "y", "z"].iter().enumerate() {
            m.fields.push(ros_env::sensor_msgs::msg::PointField {
                name: (*name).into(),
                offset: 4 * i as u32,
                datatype: 7,
                count: 1,
            });
        }
        m.width = 2;
        m.height = 1;
        m.point_step = 12;
        m.row_step = 24;
        m.data = [5.0_f32, 0.0, -1.0, f32::INFINITY, 0.0, 0.0]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let mut statics = StaticFrames::default();
        statics.insert(
            "base_link",
            "lidar_link",
            Transform3 {
                translation: [0.0, 0.0, 1.8],
                ..Transform3::IDENTITY
            },
        );
        let mut poses = PoseBuffer::new(Duration::from_secs(1));
        poses.push(Stamp::new(0, 900_000_000), Pose::new(0.0, 0.0, 0.0));
        poses.push(Stamp::new(1, 100_000_000), Pose::new(0.0, 0.0, 0.0));
        let settings = DebugConfig::default().cube_settings();
        let scan = place(&m, "base_link", &statics, &poses, &settings).expect("placed");
        assert_eq!((scan.cubes.len(), scan.skipped), (1, 1));
    }

    #[test]
    fn a_swap_asks_for_one_string_and_reads_the_answer() {
        let r = tracker_request("lqr");
        assert_eq!(r.parameters[0].name, "tracker");
        assert_eq!(r.parameters[0].value.string_value, "lqr");
        assert_eq!(r.parameters[0].value.type_, ParameterType::PARAMETER_STRING);
        let mut response = SetParameters_Response::default();
        let Update::SwapReply(none) = swap_reply("lqr", &response) else {
            panic!("a reply")
        };
        assert!(none.outcome.is_err());
        response
            .results
            .push(ros_env::rcl_interfaces::msg::SetParametersResult {
                successful: false,
                reason: "live_swap is off".into(),
            });
        let Update::SwapReply(no) = swap_reply("lqr", &response) else {
            panic!("a reply")
        };
        assert_eq!(no.outcome, Err("live_swap is off".into()));
    }

    #[test]
    fn a_fix_says_its_kind() {
        let mut m = NavSatFix::default();
        m.status.status = -1;
        assert_eq!(gps_detail(&m), "no fix");
        m.status.status = 0;
        m.position_covariance[0] = 4.0;
        assert_eq!(gps_detail(&m), "fix, 2.0 m");
    }

    #[test]
    fn unmeasured_wheels_are_left_out() {
        let m = WheelSpeeds {
            front_left: 1.0,
            front_right: 2.0,
            rear_left: f32::NAN,
            rear_right: f32::NAN,
            ..Default::default()
        };
        assert_eq!(wheels_detail(&m), "+1.50 m/s (2 wheels)");
    }

    /// The node itself, with ROS: a cloud becomes cubes placed by the pose at its stamp
    /// (from /tf_static and the local EKF), map -> odom arrives, and a tracker swap gets
    /// the service's answer. Needs ROS sourced; this computer only, on ROS_DOMAIN_ID (94
    /// unless set), with topics of its own.
    #[test]
    fn a_cloud_becomes_cubes_and_a_swap_gets_its_answer() -> Result<()> {
        use crate::source::DataSource;
        use ros_env::geometry_msgs::msg::TransformStamped;
        use ros_env::rcl_interfaces::msg::SetParametersResult;
        use ros_env::sensor_msgs::msg::PointField;

        if std::env::var_os("ROS_DOMAIN_ID").is_none() {
            std::env::set_var("ROS_DOMAIN_ID", "94");
        }
        std::env::set_var("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST");
        let prefix = format!("/debug_hud_test_{}", std::process::id());
        let mut config = DebugConfig::default();
        for t in Topic::ALL {
            let name = format!("{prefix}{}", config.topic(t));
            config.set_topic(t, name);
        }
        config.track_parameters_service = format!("{prefix}/track/set_parameters");
        let (mut hud, config) = RosSource::start(vec!["debug_hud".into()], config)?;

        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let helper = executor.create_node("debug_hud_test_helper")?;
        let latched = QoSProfile::topics_default()
            .reliable()
            .transient_local()
            .keep_last(10);
        let statics: Publisher<TFMessage> =
            helper.create_publisher(config.topic(Topic::TfStatic).qos(latched))?;
        let tf: Publisher<TFMessage> = helper.create_publisher(config.topic(Topic::Tf))?;
        let odom: Publisher<Odometry> = helper.create_publisher(config.topic(Topic::LocalOdom))?;
        let lidar: Publisher<PointCloud2> = helper.create_publisher(config.topic(Topic::Lidar))?;
        let _service = helper.create_service::<SetParameters, _>(
            config.track_parameters_service.as_str(),
            |request: SetParameters_Request| SetParameters_Response {
                results: vec![SetParametersResult {
                    successful: false,
                    reason: format!(
                        "live_swap is off ({})",
                        request.parameters[0].value.string_value
                    ),
                }],
            },
        )?;
        let spin = |executor: &mut Executor, ms| {
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(ms)));
        };
        let transform = |parent: &str, child: &str, x: f64, z: f64| {
            let mut t = TransformStamped::default();
            t.header.frame_id = parent.into();
            t.child_frame_id = child.into();
            t.transform.translation.x = x;
            t.transform.translation.z = z;
            t.transform.rotation.w = 1.0;
            t
        };
        spin(&mut executor, 1500);
        statics.publish(TFMessage {
            transforms: vec![transform("base_link", "lidar_link", 0.0, 2.0)],
        })?;
        // The car drives +x at 1 m/s from x = 0, stamps 10.0 s to 10.5 s; the cloud is
        // stamped 10.25 s, when the car was at x = 0.25.
        let pose_at = |t: f64| {
            let mut m = Odometry::default();
            m.header.frame_id = "odom".into();
            m.header.stamp.sec = t as i32;
            m.header.stamp.nanosec = ((t.fract()) * 1e9).round() as u32;
            m.pose.pose.position.x = t - 10.0;
            m.pose.pose.orientation.w = 1.0;
            m
        };
        for i in 0..=10 {
            odom.publish(pose_at(10.0 + 0.05 * f64::from(i)))?;
            spin(&mut executor, 20);
        }
        tf.publish(TFMessage {
            transforms: vec![transform("map", "odom", 100.0, 0.0)],
        })?;
        let mut cloud = PointCloud2::default();
        cloud.header.frame_id = "lidar_link".into();
        cloud.header.stamp.sec = 10;
        cloud.header.stamp.nanosec = 250_000_000;
        cloud.fields = ["x", "y", "z"]
            .iter()
            .zip(0..)
            .map(|(&name, i)| PointField {
                name: name.into(),
                offset: 4 * i,
                datatype: 7,
                count: 1,
            })
            .collect();
        (cloud.width, cloud.height, cloud.point_step, cloud.row_step) = (1, 1, 12, 12);
        cloud.data = [5.0_f32, 0.0, -1.0]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let mut scan = None;
        let deadline = Instant::now() + Duration::from_secs(5);
        while scan.is_none() && Instant::now() < deadline {
            lidar.publish(&cloud)?;
            spin(&mut executor, 200);
            scan = hud.take_scan();
        }
        let scan = scan.context("no scan from the cloud")?;
        // 5 m ahead of the lidar, which is over base_link at x = 0.25: x = 5.25, z = 1.
        let [x, _, z] = scan.cubes[0].centre;
        assert!(
            (x - 5.3).abs() < 0.11 && (z - 1.1).abs() < 0.11,
            "{:?}",
            scan.cubes
        );
        assert!((scan.car.position.x - 0.25).abs() < 1e-6);
        let news = hud.drain(Instant::now());
        assert!(
            news.iter()
                .any(|u| matches!(u, Update::MapToOdom(p) if (p.position.x - 100.0).abs() < 1e-9)),
            "no map -> odom"
        );

        // Keys 1-4: the service's own answer comes back.
        let mut reply = None;
        let deadline = Instant::now() + Duration::from_secs(5);
        while reply.is_none() && Instant::now() < deadline {
            if hud.swap_tracker(Tracker::Lqr).is_ok() {
                for _ in 0..10 {
                    spin(&mut executor, 100);
                    reply = hud.drain(Instant::now()).into_iter().find_map(|u| match u {
                        Update::SwapReply(r) => Some(r),
                        _ => None,
                    });
                    if reply.is_some() {
                        break;
                    }
                }
            } else {
                spin(&mut executor, 200);
            }
        }
        let reply = reply.context("no answer to the swap")?;
        assert_eq!(reply.asked, "lqr");
        assert_eq!(reply.outcome, Err("live_swap is off (lqr)".into()));
        Ok(())
    }

    /// Against a running simulator (not run by default): the node with the built-in
    /// topics, for `secs` seconds (DEBUG_HUD_LIVE_SECS, 20), printing what the panel and
    /// the cubes would show, then asking the track node for `lqr` and printing its answer.
    /// `pixi run debugdemo` without a screen: start the sim headless on a domain, then
    /// `cargo test --release -p debug-app -- --ignored --nocapture live_sim` on it.
    #[test]
    #[ignore]
    fn live_sim() -> Result<()> {
        use crate::source::DataSource;
        use debug_core::state::DebugState;

        let secs: u64 = std::env::var("DEBUG_HUD_LIVE_SECS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(20);
        let (mut hud, config) = RosSource::start(vec!["debug_hud".into()], DebugConfig::default())?;
        let mut state = DebugState::new(&config);
        let end = Instant::now() + Duration::from_secs(secs);
        let mut scans = 0;
        while Instant::now() < end {
            let now = Instant::now();
            for u in hud.drain(now) {
                state.apply(u, now);
            }
            if let Some(scan) = hud.take_scan() {
                scans += 1;
                state.set_scan(scan, now);
            }
            thread::sleep(Duration::from_millis(30));
        }
        let now = Instant::now();
        println!("scans placed: {scans}, rate {:?} Hz", state.scan_rate());
        if let Some(scan) = &state.scan {
            let mut bands = std::collections::BTreeMap::new();
            for c in &scan.cubes {
                *bands.entry(format!("{:?}", c.band)).or_insert(0) += 1;
            }
            println!(
                "last scan: {} cubes from {} points ({} skipped, {} cut), car at {:?}; bands {bands:?}",
                scan.cubes.len(), scan.points, scan.skipped, scan.cut, scan.car
            );
        }
        println!(
            "scan problem: {:?}; frame problem: {:?}",
            state.scan_problem, state.frame_problem
        );
        for source in Source::PANEL {
            println!(
                "{:12} {:6} {}",
                source.label(),
                state.health(source, now).word(),
                state.detail(source).unwrap_or("")
            );
        }
        for who in Localizer::ALL {
            println!(
                "{:12} in odom {:?}, {:?} m from EKF local",
                who.label(),
                state.pose_in_odom(who),
                state.offset_from_local(who)
            );
        }
        println!(
            "map -> odom {:?}; plan points {}",
            state.map_to_odom.odom_in_map,
            state.plan_in_odom().len()
        );
        println!(
            "kept cubes {} ({} cut); NDT status {:?}; AMCL particles in odom {}",
            state.kept.len(),
            state.kept.cut(),
            state.ndt,
            state.particles_in_odom().count()
        );
        let map = debug_core::ndt::MapFile::read(&config.ndt_map, config.ndt_map_cell);
        println!("NDT map: {map}");
        println!(
            "banner {:?}; tracker {:?}",
            state.origins.banner(),
            state.track
        );
        println!(
            "swap allowed: {:?}",
            debug_core::swap::allowed(&state.origins)
        );
        if hud.swap_tracker(Tracker::Lqr).is_ok() {
            let end = Instant::now() + Duration::from_secs(5);
            while Instant::now() < end {
                if let Some(Update::SwapReply(r)) = hud
                    .drain(Instant::now())
                    .into_iter()
                    .find(|u| matches!(u, Update::SwapReply(_)))
                {
                    println!("swap: {}", r.line());
                    break;
                }
                thread::sleep(Duration::from_millis(50));
            }
        } else {
            println!("swap: no track node");
        }
        Ok(())
    }
}
