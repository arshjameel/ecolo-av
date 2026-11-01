//! The ROS side of the map window: the node `map`, spun by rclrs on its own thread.
//!
//! As the HUD's (hud-app's ros.rs): raylib must own the main thread, so the node lives on
//! a thread called "ros", and its callbacks turn each message into a map_core `Update`
//! sent over a channel. It publishes one thing: the place search's goal, on goal_topic,
//! from the window's thread (an rclrs publisher may be used from any thread).
//!
//! QoS:
//! - `/map`: reliable, transient local, depth 1, as map_server sends it (latched), so a
//!   window opened later still gets the map;
//! - the car's position: best effort (SensorDataQoS), which takes reliable publishers too;
//! - the plan and the goal: reliable, as Nav2 and RViz send them; the goal we publish:
//!   reliable, volatile, depth 10, as RViz's "2D Goal Pose" (bt_navigator listens);
//! - `/system/source`: reliable, transient local, depth 1, as the HUD;
//! - the sensor watchdog's verdict: reliable, depth 1, as the HUD;
//! - street_router's street route: reliable, transient local, depth 1 (latched).

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver, SyncSender, TrySendError},
        Arc,
    },
    thread::{self, JoinHandle},
    time::Instant,
};

use anyhow::{anyhow, Context as _, Result};
use hud_core::{
    origin::{Announcement, Entry, Origin},
    units::MetresPerSecond,
};
use map_core::{
    config::{MapConfig, Topic, NODE_NAME},
    geom::{Point, Pose, Quaternion},
    grid::{Grid, GridInfo},
    route::Route,
    state::{Car, Update},
};
use rclrs::*;
use ros_env::aav_interfaces::msg::DataSource;
use ros_env::geometry_msgs::msg::{Pose as PoseMsg, PoseStamped};
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry, Path};
use ros_env::std_msgs::msg::String as StringMsg;

/// How many updates may wait for the screen. More are dropped: the screen is stuck.
const QUEUE: usize = 256;

/// The node, seen from the screen.
pub struct RosSource {
    updates: Receiver<Update>,
    goal: Publisher<PoseStamped>,
    stopped: Arc<AtomicBool>,
    executor: Arc<ExecutorCommands>,
    thread: Option<JoinHandle<()>>,
}

/// What the ROS thread hands back once the node is up.
type Ready = Result<(MapConfig, Arc<ExecutorCommands>, Publisher<PoseStamped>)>;

impl RosSource {
    /// Starts the node on its own thread and waits until it's up. `args` is the whole
    /// command line: ROS reads what follows `--ros-args`. Returns the settings with any
    /// ROS parameters applied.
    pub fn start(args: Vec<String>, config: MapConfig) -> Result<(Self, MapConfig)> {
        let (ready_tx, ready_rx) = mpsc::channel::<Ready>();
        let (update_tx, updates) = mpsc::sync_channel(QUEUE);
        let stopped = Arc::new(AtomicBool::new(false));
        let thread = {
            let stopped = Arc::clone(&stopped);
            thread::Builder::new().name("ros".into()).spawn(move || {
                let mut ready = Some(ready_tx);
                if let Err(e) = spin(args, config, update_tx, &mut ready) {
                    match ready.take() {
                        Some(tx) => {
                            let _ = tx.send(Err(e));
                        }
                        None => eprintln!("map: the ROS node stopped: {e:#}"),
                    }
                }
                stopped.store(true, Ordering::SeqCst);
            })?
        };
        let (config, executor, goal) = ready_rx
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
            goal,
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

    fn send_goal(&mut self, goal: Pose, frame: &str) -> Result<()> {
        self.goal
            .publish(goal_message(goal, frame))
            .context("publishing the goal")
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

/// The ROS thread: makes the node, says it's ready, then spins until halted.
fn spin(
    args: Vec<String>,
    config: MapConfig,
    tx: SyncSender<Update>,
    ready: &mut Option<mpsc::Sender<Ready>>,
) -> Result<()> {
    let context = Context::new(args, InitOptions::default())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME)?;
    // Keep the parameters alive with the node: rclrs undeclares one when it's dropped.
    let (_params, config) = Params::declare(&node, config)?;

    let latched = QoSProfile::topics_default()
        .reliable()
        .transient_local()
        .keep_last(1);
    let _map = node.create_subscription(config.topic(Topic::Map).qos(latched), {
        let tx = tx.clone();
        move |m: Box<OccupancyGrid>| send(&tx, grid(&m))
    })?;
    let _odom = node.create_subscription(config.topic(Topic::Odom).sensor_data_qos(), {
        let tx = tx.clone();
        move |m: Odometry| {
            send(
                &tx,
                Update::Car(Car {
                    pose: pose(&m.pose.pose),
                    speed: MetresPerSecond(m.twist.twist.linear.x as f32),
                    frame: m.header.frame_id,
                }),
            )
        }
    })?;
    let plans = QoSProfile::topics_default().reliable().keep_last(1);
    let _plan = node.create_subscription(config.topic(Topic::Plan).qos(plans), {
        let tx = tx.clone();
        move |m: Path| {
            send(
                &tx,
                Update::Plan {
                    route: route(&m),
                    frame: m.header.frame_id,
                },
            )
        }
    })?;
    let _street = node.create_subscription(config.topic(Topic::StreetRoute).qos(latched), {
        let tx = tx.clone();
        move |m: Path| {
            send(
                &tx,
                Update::StreetRoute {
                    route: route(&m),
                    frame: m.header.frame_id,
                },
            )
        }
    })?;
    let _goal = node.create_subscription(config.topic(Topic::Goal).reliable(), {
        let tx = tx.clone();
        move |m: PoseStamped| {
            send(
                &tx,
                Update::Goal {
                    pose: pose(&m.pose),
                    frame: m.header.frame_id,
                },
            )
        }
    })?;
    let _source = node.create_subscription(config.topic(Topic::Source).qos(latched), {
        let tx = tx.clone();
        move |m: DataSource| send(&tx, Update::Source(announcement(&m)))
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

    let goal_publisher = node.create_publisher::<PoseStamped>(config.topic(Topic::Goal))?;

    if let Some(r) = ready.take() {
        r.send(Ok((
            config,
            Arc::clone(executor.commands()),
            goal_publisher,
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

/// A path's points, start to end.
fn route(m: &Path) -> Route {
    Route::new(m.poses.iter().map(|p| {
        let q = &p.pose.position;
        Point::new(q.x, q.y)
    }))
}

fn pose(p: &PoseMsg) -> Pose {
    let o = &p.orientation;
    Pose::from_ros(
        p.position.x,
        p.position.y,
        Quaternion {
            x: o.x,
            y: o.y,
            z: o.z,
            w: o.w,
        },
    )
}

/// A goal as RViz sends it: the frame, a zero stamp ("the latest", which Nav2 takes),
/// z 0, and the heading as a quaternion about z.
fn goal_message(goal: Pose, frame: &str) -> PoseStamped {
    let mut m = PoseStamped::default();
    m.header.frame_id = frame.to_owned();
    m.pose.position.x = goal.position.x;
    m.pose.position.y = goal.position.y;
    let (s, c) = (goal.yaw / 2.0).sin_cos();
    m.pose.orientation.z = s;
    m.pose.orientation.w = c;
    m
}

fn grid(m: &OccupancyGrid) -> Update {
    let info = GridInfo {
        width: m.info.width,
        height: m.info.height,
        resolution: f64::from(m.info.resolution),
        origin: pose(&m.info.origin),
    };
    Grid::new(info, m.header.frame_id.clone(), m.data.clone())
        .map_or_else(|e| Update::BadMap(e.to_string()), Update::Map)
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

/// The node's parameters: each topic's name, and the limits. Read once, at start-up.
struct Params {
    _topics: Vec<ReadOnlyParameter<Arc<str>>>,
    _files: Vec<ReadOnlyParameter<Arc<str>>>,
    _numbers: Vec<ReadOnlyParameter<f64>>,
}

impl Params {
    /// Declares them, with `config` (config/map.yaml) as the defaults, and returns the
    /// settings as ROS has them (a --params-file or -p wins).
    fn declare(node: &Node, mut config: MapConfig) -> Result<(Self, MapConfig)> {
        let mut topics = Vec::new();
        for t in Topic::ALL {
            let p = node
                .declare_parameter::<Arc<str>>(t.parameter())
                .default(config.topic(t).into())
                .read_only()?;
            config.set_topic(t, p.get().to_string());
            topics.push(p);
        }
        let mut numbers = Vec::new();
        for (name, slot) in [
            ("stale_after", &mut config.stale_after),
            ("plan_stale_after", &mut config.plan_stale_after),
            ("final_approach", &mut config.final_approach),
            ("plan_kept_for", &mut config.plan_kept_for),
            ("arrive_within", &mut config.arrive_within),
        ] {
            let p = node.declare_parameter(name).default(*slot).read_only()?;
            *slot = p.get();
            numbers.push(p);
        }
        let mut files = Vec::new();
        for (name, slot) in [
            ("street_map", &mut config.street_map),
            ("places", &mut config.places),
        ] {
            let p = node
                .declare_parameter::<Arc<str>>(name)
                .default(slot.as_str().into())
                .read_only()?;
            *slot = p.get().to_string();
            files.push(p);
        }
        config.validate()?;
        let params = Self {
            _topics: topics,
            _files: files,
            _numbers: numbers,
        };
        Ok((params, config))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The goal goes out as RViz's would: in the frame, facing the right way.
    #[test]
    fn a_goal_becomes_a_pose_stamped() {
        let m = goal_message(Pose::new(3.0, -4.0, std::f64::consts::FRAC_PI_2), "map");
        assert_eq!(m.header.frame_id, "map");
        assert_eq!((m.pose.position.x, m.pose.position.y), (3.0, -4.0));
        let back = pose(&m.pose);
        assert!((back.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    /// A message's map arrives the right way up: the message's first row is the
    /// grid's lowest (row 0), whatever the picture does with it later.
    #[test]
    fn an_occupancy_grid_message_becomes_a_grid() {
        let mut m = OccupancyGrid::default();
        m.header.frame_id = "map".into();
        m.info.width = 2;
        m.info.height = 1;
        m.info.resolution = 0.5;
        m.info.origin.position.x = -1.0;
        m.info.origin.orientation.w = 1.0;
        m.data = vec![0, 100];
        let Update::Map(g) = grid(&m) else {
            panic!("a valid map")
        };
        assert_eq!(g.info.origin, Pose::new(-1.0, 0.0, 0.0));
        assert_eq!(g.cell(1, 0), Some(map_core::grid::Cell::Occupancy(100)));
        m.data = vec![0];
        assert!(matches!(grid(&m), Update::BadMap(_)));
    }
}
