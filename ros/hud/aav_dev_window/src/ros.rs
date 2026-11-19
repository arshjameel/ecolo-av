//! The ROS side of the developer window: the node `dev_window`, on its own thread, turning
//! each message into an [`Update`]. Never reads the lidar's point cloud: the car makes the
//! cubes.
//!
//! Subscribes: the cubes, the road, both odometries, NDT's pose and status, the scene's picture,
//! `/rosout`, `/plan`,
//! `/map`, the costmap, `/system/source`, `/vehicle/status`, the output gate, the
//! watchdog, and the squiggly line's four, all base_link: `/ego/trace` (Path),
//! `/ego/plan_ahead` (Path), `/ego/stop_point` (PoseStamped), `/ego/ahead`
//! (DiagnosticStatus); the road ahead, map (`aav_interfaces/RoadAhead`); the things
//! near the car, odom (`aav_interfaces/Things`, named by the camera); the street route,
//! map (`/drivers/nav/route/path`, Path, latched) (`config/dev_window.yaml`).
//! Publishes, only with `--developer`: `/goal_pose`, `/safety/estop` (only ever true).
//! Calls, only with `--developer` and every sensor SIM: `/navigate_to_pose/_action/cancel_goal`
//! (action_msgs/CancelGoal, every goal: CANCEL THE GOAL) and `/track/set_parameters`
//! (rcl_interfaces/SetParameters, `tracker`: SWAP, TODO 114). Serves nothing.

use std::{
    sync::atomic::{AtomicU64, Ordering},
    sync::{
        mpsc::{self, Receiver, SyncSender},
        Arc, Mutex, PoisonError,
    },
    thread,
    time::{Duration, Instant},
};

use debug_core::{
    cloud::{CloudView, Field},
    cubes::{CubeScan, Metres},
    frames::{Stamp, StaticFrames},
    ndt::NdtStatus,
    poses::PoseBuffer,
    scan::ScanError,
    voxels::{self, VoxelSettings},
};
use hud_core::{
    ahead::{StopMark, Summary},
    estop::{EstopButton, PERIOD},
    heartbeat::{Frames, Heartbeat},
    origin::{Announcement, Entry, Origin},
    picture::Picture,
    things::Thing,
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Faults, Mode, VehicleStatus},
};
use map_core::{
    geom::{Point, Pose, Quaternion},
    grid::{Grid, GridInfo},
};
use rclrs::*;
use ros_env::aav_interfaces::msg::{
    DataSource, RoadAhead as RoadAheadMsg, Things, VehicleStatus as VehicleStatusMsg,
};
use ros_env::action_msgs::srv::{CancelGoal, CancelGoal_Request, CancelGoal_Response};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::DiagnosticStatus;
use ros_env::geometry_msgs::msg::{Pose as PoseMsg, PoseStamped, PoseWithCovarianceStamped};
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry, Path};
use ros_env::rcl_interfaces::msg::{Log, Parameter, ParameterType, ParameterValue};
use ros_env::rcl_interfaces::srv::{SetParameters, SetParameters_Request, SetParameters_Response};
use ros_env::sensor_msgs::msg::{Image, PointCloud2};
use ros_env::std_msgs::msg::{Bool, String as StringMsg, UInt64};
use serde::Deserialize;

use crate::feed::{Feed, Level, RoadAhead, Update};

const NODE_NAME: &str = "dev_window";
/// The frame the cubes are made in.
const BASE: &str = "base_link";
/// How many updates may wait for the window. More are dropped: the window is stuck.
const QUEUE: usize = 512;
/// The most cubes kept from one cloud: the nearest stay.
const MAX_CUBES: usize = 100_000;
/// One cube's edge, metres: `aav_voxels`' size.
const CUBE_SIZE: f64 = 0.25;
/// The word ROS uses to keep discovery on this computer.
const LOCALHOST: &str = "LOCALHOST";

/// Every topic's name (`config/dev_window.yaml`).
#[derive(Debug, Deserialize)]
struct Topics {
    cubes: String,
    road: String,
    local_odom: String,
    global_odom: String,
    picture: String,
    ndt_pose: String,
    ndt_status: String,
    log: String,
    plan: String,
    map: String,
    costmap: String,
    source: String,
    status: String,
    gate: String,
    watchdog: String,
    driving: String,
    goal: String,
    estop: String,
    cancel_goal: String,
    track_parameters: String,
    screen_alive: String,
    trace: String,
    plan_ahead: String,
    stop_point: String,
    ahead: String,
    road_ahead: String,
    things: String,
    route: String,
}

/// A Path's points, as they are (base_link for the squiggly line's two).
fn path_points(m: &Path) -> Vec<(f64, f64)> {
    m.poses
        .iter()
        .map(|p| (p.pose.position.x, p.pose.position.y))
        .collect()
}

/// The two commands, which exist only in developer mode.
struct Commands {
    goal: Publisher<PoseStamped>,
    estop: Publisher<Bool>,
    cancel: Client<CancelGoal>,
    track: Client<SetParameters>,
    /// Where the services' answers go: the console.
    tx: SyncSender<Update>,
}

/// The node, seen from the window.
pub struct RosFeed {
    updates: Mutex<Receiver<Update>>,
    /// Newest-only: a slow window gets the newest picture, not a backlog.
    picture: Arc<Mutex<Option<Picture>>>,
    commands: Option<Commands>,
    /// How many frames have reached the screen, for the heartbeat.
    drawn: Arc<AtomicU64>,
    estop: Mutex<EstopButton>,
    local: bool,
}

impl RosFeed {
    /// Starts the node. Without `developer`, no publisher exists at all.
    pub fn start(developer: bool) -> Result<Self, String> {
        let local =
            std::env::var("ROS_AUTOMATIC_DISCOVERY_RANGE").is_ok_and(|range| range == LOCALHOST);
        let (ready_tx, ready_rx) = mpsc::channel();
        let (tx, updates) = mpsc::sync_channel(QUEUE);
        let picture = Arc::new(Mutex::new(None));
        let shared = Arc::clone(&picture);
        let drawn = Arc::new(AtomicU64::new(0));
        let frames = Arc::clone(&drawn);
        thread::Builder::new()
            .name("ros".into())
            .spawn(move || {
                if let Err(e) = spin(developer, tx, shared, frames, ready_tx.clone()) {
                    let _ = ready_tx.send(Err(format!("{e:#}")));
                    eprintln!("dev_window: the ROS node stopped: {e:#}");
                }
            })
            .map_err(|e| e.to_string())?;
        let commands = ready_rx
            .recv()
            .map_err(|_| "the ROS thread ended before the node was up".to_owned())?
            .map_err(|e| format!("starting the ROS node (is ROS sourced?): {e}"))?;
        Ok(Self {
            updates: Mutex::new(updates),
            picture,
            commands,
            drawn,
            estop: Mutex::new(EstopButton::default()),
            local,
        })
    }
}

impl Feed for RosFeed {
    fn label(&self) -> &'static str {
        "ROS"
    }

    fn drain(&mut self, _now: Instant) -> Vec<Update> {
        let mut updates: Vec<Update> = self
            .updates
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .try_iter()
            .collect();
        let picture = self
            .picture
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        updates.extend(picture.map(Update::Picture));
        updates
    }

    fn may_command(&self) -> Result<(), String> {
        if self.commands.is_none() {
            return Err("watch only: this run has no --developer".to_owned());
        }
        if !self.local {
            return Err("watch only: ROS isn't kept on this computer".to_owned());
        }
        Ok(())
    }

    fn send_goal(&mut self, goal: Pose) -> Result<(), String> {
        self.may_command()?;
        let commands = self.commands.as_ref().ok_or("no publisher in this run")?;
        let half = goal.yaw / 2.0;
        let mut message = PoseStamped::default();
        message.header.frame_id = "map".into();
        message.pose.position.x = goal.position.x;
        message.pose.position.y = goal.position.y;
        message.pose.orientation.z = half.sin();
        message.pose.orientation.w = half.cos();
        commands.goal.publish(message).map_err(|e| e.to_string())
    }

    fn frame_drawn(&self) {
        self.drawn.fetch_add(1, Ordering::Relaxed);
    }

    fn press_estop(&mut self) -> Result<(), String> {
        self.may_command()?;
        let commands = self.commands.as_ref().ok_or("no publisher in this run")?;
        // The same burst as the HUD's button (hud_core::estop). A true each time, never false.
        let mut button = self.estop.lock().unwrap_or_else(PoisonError::into_inner);
        let start = Instant::now();
        button.press(start);
        let mut step = Duration::ZERO;
        while button.is_sending() {
            if button.poll(start + step).is_some() {
                commands
                    .estop
                    .publish(Bool { data: true })
                    .map_err(|e| e.to_string())?;
            }
            step += PERIOD;
        }
        Ok(())
    }

    fn cancel_goal(&mut self) -> Result<(), String> {
        self.may_command()?;
        let commands = self.commands.as_ref().ok_or("no client in this run")?;
        if !commands
            .cancel
            .service_is_ready()
            .map_err(|e| e.to_string())?
        {
            return Err("no navigator answers (is the stack running?)".to_owned());
        }
        let tx = commands.tx.clone();
        // An empty request (no id, no time) cancels every goal (action_msgs/CancelGoal).
        let promise = commands
            .cancel
            .call_then(
                &CancelGoal_Request::default(),
                move |r: CancelGoal_Response| {
                    send(&tx, cancel_reply(&r));
                },
            )
            .map_err(|e| e.to_string())?;
        // The promise needn't be kept (rclrs): the callback runs when the answer comes.
        drop(promise);
        Ok(())
    }

    fn swap_tracker(&mut self, name: &'static str) -> Result<(), String> {
        self.may_command()?;
        let commands = self.commands.as_ref().ok_or("no client in this run")?;
        if !commands
            .track
            .service_is_ready()
            .map_err(|e| e.to_string())?
        {
            return Err("no track node (start the sim with tracker:=pure_pursuit)".to_owned());
        }
        let tx = commands.tx.clone();
        let promise = commands
            .track
            .call_then(&tracker_request(name), move |r: SetParameters_Response| {
                send(&tx, swap_reply(name, &r));
            })
            .map_err(|e| e.to_string())?;
        drop(promise);
        Ok(())
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

/// The track node's answer, for the console.
fn swap_reply(asked: &str, response: &SetParameters_Response) -> Update {
    match response.results.first() {
        Some(r) if r.successful => {
            Update::Log(Level::Info, format!("swapped: {asked} follows now"))
        }
        Some(r) => Update::Log(Level::Warn, format!("not swapped to {asked}: {}", r.reason)),
        None => Update::Log(Level::Warn, format!("not swapped to {asked}: no answer")),
    }
}

/// The navigator's answer to a cancel, for the console.
fn cancel_reply(response: &CancelGoal_Response) -> Update {
    let n = response.goals_canceling.len();
    match response.return_code {
        CancelGoal_Response::ERROR_NONE => Update::Log(
            Level::Info,
            format!("the goal is cancelled ({n} cancelling)"),
        ),
        CancelGoal_Response::ERROR_UNKNOWN_GOAL_ID => {
            Update::Log(Level::Info, "no goal to cancel".to_owned())
        }
        code => Update::Log(Level::Warn, format!("the cancel was refused (code {code})")),
    }
}

fn send(tx: &SyncSender<Update>, update: Update) {
    // Full or closed: the window is stuck or gone, and newer news will follow.
    let _ = tx.try_send(update);
}

fn stamp(t: &Time) -> Stamp {
    Stamp::new(t.sec, t.nanosec)
}

fn pose(p: &PoseMsg) -> Pose {
    let q = &p.orientation;
    Pose::from_ros(
        p.position.x,
        p.position.y,
        Quaternion {
            x: q.x,
            y: q.y,
            z: q.z,
            w: q.w,
        },
    )
}

fn view(m: &PointCloud2) -> CloudView<'_> {
    CloudView {
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
    }
}

fn grid(m: &OccupancyGrid) -> Result<Grid, String> {
    let info = GridInfo {
        width: m.info.width,
        height: m.info.height,
        resolution: f64::from(m.info.resolution),
        origin: pose(&m.info.origin),
    };
    Grid::new(info, m.header.frame_id.clone(), m.data.clone()).map_err(|e| e.to_string())
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

/// Moves a cube cloud into odom, with the local EKF's pose at its stamp.
fn place(m: &PointCloud2, poses: &PoseBuffer) -> Result<CubeScan, ScanError> {
    let settings = VoxelSettings {
        size: Metres(CUBE_SIZE),
        max_cubes: MAX_CUBES,
    };
    voxels::place(
        &view(m),
        &m.header.frame_id,
        stamp(&m.header.stamp),
        BASE,
        &StaticFrames::default(),
        poses,
        settings,
    )
}

/// The cubes that wait for the pose at their stamp, one newest cloud of each kind.
#[derive(Default)]
struct Waiting {
    cubes: Option<PointCloud2>,
    road: Option<PointCloud2>,
}

/// One cloud into an update, or back to wait for its pose. Other problems reach the console.
fn try_place(
    cloud: PointCloud2,
    poses: &PoseBuffer,
    wrap: fn(CubeScan) -> Update,
    tx: &SyncSender<Update>,
) -> Option<PointCloud2> {
    match place(&cloud, poses) {
        Ok(scan) => {
            send(tx, wrap(scan));
            None
        }
        Err(e) if e.is_too_new() => Some(cloud),
        Err(e) => {
            send(
                tx,
                Update::Log(Level::Warn, format!("cubes not placed: {e}")),
            );
            None
        }
    }
}

type Ready = Result<Option<Commands>, String>;

/// The ROS thread: makes the node, says it's ready, then spins until the process ends.
fn spin(
    developer: bool,
    tx: SyncSender<Update>,
    picture: Arc<Mutex<Option<Picture>>>,
    frames: Arc<AtomicU64>,
    ready: mpsc::Sender<Ready>,
) -> Result<(), RclrsError> {
    let topics: Topics = match serde_yaml_ng::from_str(include_str!("../config/dev_window.yaml")) {
        Ok(topics) => topics,
        Err(e) => {
            let _ = ready.send(Err(format!("config/dev_window.yaml: {e}")));
            return Ok(());
        }
    };
    let context = Context::new(
        std::iter::once(NODE_NAME.to_owned()),
        InitOptions::default(),
    )?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME)?;

    let newest = QoSProfile::topics_default().reliable().keep_last(1);
    let latched = QoSProfile::topics_default()
        .reliable()
        .transient_local()
        .keep_last(1);
    let poses = Arc::new(Mutex::new(PoseBuffer::new(Duration::from_secs(5))));
    let waiting = Arc::new(Mutex::new(Waiting::default()));

    let _local = node.create_subscription(topics.local_odom.as_str().sensor_data_qos(), {
        let (tx, poses, waiting) = (tx.clone(), Arc::clone(&poses), Arc::clone(&waiting));
        move |m: Odometry| {
            let p = pose(&m.pose.pose);
            let mut buffer = poses.lock().unwrap_or_else(PoisonError::into_inner);
            buffer.push(stamp(&m.header.stamp), p);
            send(&tx, Update::CarInOdom(p));
            let mut waiting = waiting.lock().unwrap_or_else(PoisonError::into_inner);
            if let Some(cloud) = waiting.cubes.take() {
                waiting.cubes = try_place(cloud, &buffer, Update::Cubes, &tx);
            }
            if let Some(cloud) = waiting.road.take() {
                waiting.road = try_place(cloud, &buffer, Update::Road, &tx);
            }
        }
    })?;
    let _cubes = node.create_subscription(topics.cubes.as_str().qos(newest), {
        let (tx, poses, waiting) = (tx.clone(), Arc::clone(&poses), Arc::clone(&waiting));
        move |m: PointCloud2| {
            let buffer = poses.lock().unwrap_or_else(PoisonError::into_inner);
            let left = try_place(m, &buffer, Update::Cubes, &tx);
            waiting.lock().unwrap_or_else(PoisonError::into_inner).cubes = left;
        }
    })?;
    let _road = node.create_subscription(topics.road.as_str().qos(newest), {
        let (tx, poses, waiting) = (tx.clone(), Arc::clone(&poses), Arc::clone(&waiting));
        move |m: PointCloud2| {
            let buffer = poses.lock().unwrap_or_else(PoisonError::into_inner);
            let left = try_place(m, &buffer, Update::Road, &tx);
            waiting.lock().unwrap_or_else(PoisonError::into_inner).road = left;
        }
    })?;
    let _global = node.create_subscription(topics.global_odom.as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: Odometry| send(&tx, Update::CarInMap(pose(&m.pose.pose)))
    })?;
    let _picture = node.create_subscription(topics.picture.as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: Box<Image>| match Picture::from_image(
            &m.encoding,
            m.width,
            m.height,
            m.step,
            &m.data,
        ) {
            Ok(p) => *picture.lock().unwrap_or_else(PoisonError::into_inner) = Some(p),
            Err(e) => send(&tx, Update::Log(Level::Warn, format!("the picture: {e}"))),
        }
    })?;
    let _ndt_pose = node.create_subscription(topics.ndt_pose.as_str().reliable(), {
        let tx = tx.clone();
        move |m: PoseWithCovarianceStamped| send(&tx, Update::NdtPose(pose(&m.pose.pose)))
    })?;
    let _ndt_status = node.create_subscription(topics.ndt_status.as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: DiagnosticStatus| {
            let values = m
                .values
                .iter()
                .map(|kv| (kv.key.as_str(), kv.value.as_str()));
            send(
                &tx,
                Update::Ndt(NdtStatus::from_message(m.level, &m.message, values)),
            );
        }
    })?;
    let log_qos = QoSProfile::topics_default().reliable().keep_last(100);
    let _log = node.create_subscription(topics.log.as_str().qos(log_qos), {
        let tx = tx.clone();
        move |m: Log| {
            send(
                &tx,
                Update::Log(Level::from_ros(m.level), format!("[{}] {}", m.name, m.msg)),
            )
        }
    })?;
    let _plan = node.create_subscription(topics.plan.as_str().qos(newest), {
        let tx = tx.clone();
        move |m: Path| {
            let points = m
                .poses
                .iter()
                .map(|p| Point::new(p.pose.position.x, p.pose.position.y))
                .collect();
            send(&tx, Update::Plan(points));
        }
    })?;
    let _road_ahead = node.create_subscription(topics.road_ahead.as_str().qos(newest), {
        let tx = tx.clone();
        move |m: RoadAheadMsg| {
            let points: Vec<_> =
                m.x.iter()
                    .zip(&m.y)
                    .zip(&m.heading)
                    .zip(&m.speed_limit)
                    .map(|(((&x, &y), &heading), &limit)| {
                        (
                            f64::from(x),
                            f64::from(y),
                            f64::from(heading),
                            f64::from(limit),
                        )
                    })
                    .collect();
            let stops: Vec<_> = m
                .stops
                .iter()
                .map(|s| (f64::from(s.along), s.reason.clone()))
                .collect();
            let road =
                RoadAhead::from_parts(&points, f64::from(m.spacing), f64::from(m.end), &stops);
            send(&tx, Update::RoadAhead(road));
        }
    })?;
    let _things = node.create_subscription(topics.things.as_str().qos(newest), {
        let tx = tx.clone();
        move |m: Things| {
            let things = m
                .things
                .iter()
                .map(|t| Thing {
                    id: t.id,
                    name: t.kind.clone(),
                    x: f64::from(t.x),
                    y: f64::from(t.y),
                    vx: f64::from(t.vx),
                    vy: f64::from(t.vy),
                    length: f64::from(t.length),
                    width: f64::from(t.width),
                    height: f64::from(t.height),
                    coasting: t.coasting,
                })
                .collect();
            send(&tx, Update::Things(things));
        }
    })?;
    let _route = node.create_subscription(topics.route.as_str().qos(latched), {
        let tx = tx.clone();
        move |m: Path| {
            let points = m
                .poses
                .iter()
                .map(|p| Point::new(p.pose.position.x, p.pose.position.y))
                .collect();
            send(&tx, Update::Route(points));
        }
    })?;
    let _map = node.create_subscription(topics.map.as_str().qos(latched), {
        let tx = tx.clone();
        move |m: Box<OccupancyGrid>| match grid(&m) {
            Ok(g) => send(&tx, Update::Map(g)),
            Err(e) => send(&tx, Update::Log(Level::Warn, format!("the map: {e}"))),
        }
    })?;
    let _costmap = node.create_subscription(topics.costmap.as_str().qos(latched), {
        let tx = tx.clone();
        move |m: Box<OccupancyGrid>| match grid(&m) {
            Ok(g) => send(&tx, Update::Costmap(g)),
            Err(e) => send(&tx, Update::Log(Level::Warn, format!("the costmap: {e}"))),
        }
    })?;
    let _source = node.create_subscription(topics.source.as_str().qos(latched), {
        let tx = tx.clone();
        move |m: DataSource| send(&tx, Update::Origin(announcement(&m)))
    })?;
    let _status = node.create_subscription(topics.status.as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: VehicleStatusMsg| send(&tx, Update::Vehicle(vehicle(&m)))
    })?;
    let _gate = node.create_subscription(topics.gate.as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: StringMsg| send(&tx, Update::Gate(m.data))
    })?;
    let _watchdog = node.create_subscription(topics.watchdog.as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: StringMsg| send(&tx, Update::Watchdog(m.data))
    })?;
    let _driving = node.create_subscription(topics.driving.as_str().sensor_data_qos(), {
        let tx = tx.clone();
        move |m: DiagnosticStatus| send(&tx, Update::Driving(m.message))
    })?;
    // The squiggly line: all four in base_link, 10 Hz (aav_ego's ego and look_ahead).
    let _trace = node.create_subscription(topics.trace.as_str().qos(newest), {
        let tx = tx.clone();
        move |m: Path| send(&tx, Update::Trace(path_points(&m)))
    })?;
    let _plan_ahead = node.create_subscription(topics.plan_ahead.as_str().qos(newest), {
        let tx = tx.clone();
        move |m: Path| send(&tx, Update::PlanAhead(path_points(&m)))
    })?;
    let _stop_point = node.create_subscription(topics.stop_point.as_str().qos(newest), {
        let tx = tx.clone();
        move |m: PoseStamped| {
            let p = pose(&m.pose);
            send(
                &tx,
                Update::StopPoint(StopMark {
                    x: p.position.x,
                    y: p.position.y,
                    yaw: p.yaw,
                }),
            );
        }
    })?;
    let _ahead = node.create_subscription(topics.ahead.as_str().qos(newest), {
        let tx = tx.clone();
        move |m: DiagnosticStatus| {
            let values = m
                .values
                .iter()
                .map(|kv| (kv.key.as_str(), kv.value.as_str()));
            send(
                &tx,
                Update::Ahead(Summary::from_status(m.level, &m.message, values)),
            );
        }
    })?;

    let commands = if developer {
        let estop_qos = QoSProfile::topics_default().reliable().keep_last(10);
        Some(Commands {
            goal: node.create_publisher(topics.goal.as_str())?,
            estop: node.create_publisher(topics.estop.as_str().qos(estop_qos))?,
            cancel: node.create_client::<CancelGoal>(topics.cancel_goal.as_str())?,
            track: node.create_client::<SetParameters>(topics.track_parameters.as_str())?,
            tx: tx.clone(),
        })
    } else {
        None
    };
    // The window's heartbeat: hud-core's rule, so it stops if the drawing does.
    let _alive = if developer {
        let alive: Publisher<UInt64> = node.create_publisher(topics.screen_alive.as_str())?;
        let mut heartbeat = Heartbeat::default();
        Some(
            node.create_timer_repeating(Duration::from_millis(10), move || {
                let drawn = Frames(frames.load(Ordering::Relaxed));
                if let Some(Frames(count)) = heartbeat.poll(Instant::now(), drawn) {
                    let _ = alive.publish(UInt64 { data: count });
                }
            })?,
        )
    } else {
        None
    };
    if ready.send(Ok(commands)).is_err() {
        return Ok(());
    }
    executor.spin(SpinOptions::default()).first_error()
}
