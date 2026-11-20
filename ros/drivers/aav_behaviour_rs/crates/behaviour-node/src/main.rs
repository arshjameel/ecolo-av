//! behaviour in Rust: why the car should slow, and a speed cap on Nav2 that slows it. A
//! drop-in for aav_behaviour's behaviour_node.py: the same node name, topics, QoS,
//! parameters and answers. The maths is the `behaviour-core` crate, with no ROS.
//!
//! Subscribes  /perception/stop_sign/distance  geometry_msgs/PointStamped: a sign, in its
//!                                  camera's frame at the picture's stamp (kept in odom
//!                                  through /tf at that stamp; no TF then: dropped)
//!             /perception/stop_sign, /safety/stop_hold/lock, /safety/geofence/lock,
//!             /safety/sensor_watchdog/lock, /safety/estop   std_msgs/Bool: the locks
//!             /safety/geofence/state   std_msgs/String: NEAR and why
//!             /safety/geofence/fence   geometry_msgs/PolygonStamped (map, transient local)
//!             /plan                    nav_msgs/Path (map): the goal's and the fence's distance
//!             /drivers/nav/planner/status  diagnostic_msgs/DiagnosticStatus: our planner's
//!                                      how; pass: with lane_map the road ahead follows it
//!             /drivers/point/plan      nav_msgs/Path (map, latched): a drive to a point's line
//!             /drivers/nav/planner/route_stops  geometry_msgs/PoseArray (map): the route's
//!                                      stop lines; each on the plan is a stop (stop_line):
//!                                      stand there 2 s, then go (TODO 220)
//!             /localization/odom/local   nav_msgs/Odometry: the speed, and its age
//!             /localization/odom/global  nav_msgs/Odometry: where the car is on /plan
//!             /sensors/lidar/scan      sensor_msgs/LaserScan (base_link, best effort), with
//!                                      obstacles true: the lane
//!             /perception/voxels/steps  sensor_msgs/PointCloud2 (base_link), with kerbs
//!                                      true: the kerbs; one on the road ahead's line is an
//!                                      obstacle stop, so the car slows for it before the
//!                                      stop zone acts (kerb_ahead, TODO 219)
//!             /world/things            aav_interfaces/Things (odom), with things true: an
//!                                      obstacle stop only for a thing that moves; with
//!                                      moving_hold, a moving thing within moving_radius m
//!                                      or on the road ahead within moving_within s holds
//!                                      the car (WAIT, reason moving)
//!             goal_status_topics       action_msgs/GoalStatusArray: a goal running, or aborted
//!             /tf, /tf_static          the sign's camera in odom
//! Publishes   /drivers/nav/behaviour/state   diagnostic_msgs/DiagnosticStatus, state_rate_hz:
//!                                      state, reason, cap_mps, left_m, warnings
//!             /world/road_ahead        aav_interfaces/RoadAhead (map), road_rate_hz: the
//!                                      plan's next road_length m from the car, its stops,
//!                                      its end, the speed limit by the stopping law; with
//!                                      lane_map, the lane's centre and its edges; while
//!                                      something moving holds the car, a moving stop at
//!                                      0 m (the road ends at the car)
//! Parameters  behaviour_node.py's, the same names and defaults (README). vehicle_yaml '':
//!             aav_description's vehicle.yaml, found on AMENT_PREFIX_PATH.
//!
//! Serves nothing; calls nothing. It only ever asks Nav2 to go slower; it never clears a
//! lock. A clean exit sends nothing (0.0 mid-approach would jump to cruise). Stops cleanly
//! on SIGINT, SIGTERM or SIGHUP.

mod params;

use std::any::Any;
use std::collections::{BTreeSet, HashMap, HashSet};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use behaviour_core::ego_core::car_model::{ActuatorConfig, SpeedLoopProfile};
use behaviour_core::ego_core::fmt_g;
use behaviour_core::ego_core::frames::Transform;
use behaviour_core::ego_core::stop_law::{StopCurve, INPUT_DELAY};
use behaviour_core::fence::Polygon;
use behaviour_core::hypot;
use behaviour_core::lanes::{self, LaneMap, SMOOTH};
use behaviour_core::machine::{
    diagnostic, Behaviour, Decision, Lock, Settings, State, Stop, StopReason, World,
};
use behaviour_core::paths::{
    active_plan, along_until, kerb_ahead, lane_ahead, moving_ahead, moving_near, path_left, Car,
    Crossing, MovingFor, Nearby, Point, Seen, Whose, MOVING, REACH,
};
use behaviour_core::py_min;
use behaviour_core::road::{build, capped, BuildOptions, Strip};
use behaviour_core::stop_lines::{StopLineSettings, StopLines};
use behaviour_core::tf::Buffer;
use rclrs::*;
use ros_env::aav_interfaces::msg::{RoadAhead, StopPoint, Things};
use ros_env::action_msgs::msg::{GoalStatus, GoalStatusArray};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{PointStamped, PolygonStamped, PoseArray};
use ros_env::nav_msgs::msg::{Odometry, Path};
use ros_env::sensor_msgs::msg::{LaserScan, PointCloud2, PointField};
use ros_env::std_msgs::msg::{Bool, Header, String as Text};
use ros_env::tf2_msgs::msg::TFMessage;

use crate::params::Declarer;

/// s a sign point counts for.
const SIGN_FRESH: f64 = 5.0;
/// A goal running: accepted, executing or cancelling.
const RUNNING: [i8; 3] = [
    GoalStatus::STATUS_ACCEPTED,
    GoalStatus::STATUS_EXECUTING,
    GoalStatus::STATUS_CANCELING,
];

/// The parameters the ticks read.
struct Params {
    base_speed: f64,
    stop_distance: f64,
    front: f64,
    half_width: f64,
    stop_gap: f64,
    near_depth: f64,
    plan_timeout: f64,
    things: bool,
    things_timeout: f64,
    /// With things: something moving near holds the car.
    moving_hold: bool,
    nearby: Nearby,
    road_length: f64,
    road_spacing: f64,
    /// s: a kerb older than this is none (TODO 219).
    kerbs_timeout: f64,
}

struct BehaviourNode {
    p: Params,
    curve: StopCurve,
    lanes: Option<LaneMap>,
    machine: Behaviour,
    tf: Buffer,
    locks: HashMap<Lock, bool>,
    near: bool,
    fence: Option<Polygon>,
    plan: Vec<Point>,
    /// s.
    plan_time: f64,
    passing: bool,
    point_plan: Vec<Point>,
    /// The route's stop lines (TODO 220), and the `left`s of those not yet stopped at.
    stop_lines: StopLines,
    line_lefts: Vec<f64>,
    speed: Option<f64>,
    odom_time: f64,
    pose: Option<Point>,
    /// rad, map: the global pose's heading.
    pose_yaw: f64,
    /// The last road ahead's line, (x, y) in map.
    road_line: Vec<Point>,
    /// m to the nearest moving thing that holds the car (infinity: none).
    near_moving: f64,
    /// The things that have moved long enough to count.
    moving_for: MovingFor,
    /// The nearest scan point in the lane, x in base_link (m).
    ahead: f64,
    /// The nearest kerb on the road ahead, m along it, and when (TODO 219).
    kerb: f64,
    kerb_time: f64,
    /// (x, y, yaw) in odom, from the local odometry.
    odom_pose: Option<Car>,
    moving: f64,
    things_time: f64,
    /// (x, y) in odom, and the camera's frame.
    sign: Option<(f64, f64, String)>,
    sign_time: f64,
    goal_active: bool,
    aborted: bool,
    seen_aborted: HashSet<[u8; 16]>,
    running: HashMap<String, bool>,
    decision: Option<Decision>,
    said: HashSet<String>,
    pub_state: Publisher<DiagnosticStatus>,
    pub_road: Publisher<RoadAhead>,
    clock: Clock,
    logger: Logger,
}

/// aav_description's vehicle.yaml, found on AMENT_PREFIX_PATH (as ament_index does).
fn installed_vehicle_yaml() -> Option<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .ok()?
        .split(':')
        .map(|prefix| PathBuf::from(prefix).join("share/aav_description/config/vehicle.yaml"))
        .find(|p| p.is_file())
}

fn stamp(ns: i64) -> Time {
    Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

fn ns(t: &Time) -> i64 {
    i64::from(t.sec) * 1_000_000_000 + i64::from(t.nanosec)
}

fn path_points(msg: &Path) -> Vec<Point> {
    msg.poses
        .iter()
        .map(|s| (s.pose.position.x, s.pose.position.y))
        .collect()
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("behaviour")?;
    let _running = start(&node)?;
    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly (sending nothing).
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut d = Declarer::new(node);
    // plan_timeout 25 s: within 8 m of the goal the to-pose tree keeps its path for up to
    // 20 s without replanning, so /plan goes quiet there.
    let base_speed = d.number("base_speed", 1.0)?;
    let floor = d.number("floor", 0.3)?;
    let accel = d.number("accel", 0.5)?;
    let fall = d.number("fall", 2.0)?;
    let ask_after = d.number("ask_after", 30.0)?;
    let release_hold = d.number("release_hold", 1.0)?;
    let max_deceleration = d.number("max_deceleration", 2.0)?;
    let max_jerk = d.number("max_jerk", 2.0)?;
    let vehicle_yaml = d.text("vehicle_yaml", "")?;
    let speed_loop = d.text("speed_loop", "default")?;
    let stop_distance = d.number("stop_distance_m", 4.7)?;
    let front = d.number("front", 2.0)?;
    let half_width = d.number("half_width", 0.6)?;
    let stop_gap = d.number("stop_gap", 1.0)?;
    let near_depth = d.number("near_depth", 2.5)?;
    let plan_timeout = d.number("plan_timeout", 25.0)?;
    let rate_hz = d.number("rate_hz", 20.0)?;
    let state_rate_hz = d.number("state_rate_hz", 5.0)?;
    let obstacles = d.flag("obstacles", true)?;
    let road_rate_hz = d.number("road_rate_hz", 10.0)?;
    let road_length = d.number("road_length", 40.0)?;
    let road_spacing = d.number("road_spacing", 0.5)?;
    // A lanes.yaml: the road ahead's line on the lane's centre. Empty: the plan's.
    let lane_map = d.text("lane_map", "")?;
    // /world/things: wait only for what moves, leave a parked thing to the planner.
    let things = d.flag("things", false)?;
    let things_timeout = d.number("things_timeout", 1.0)?;
    // /perception/voxels/steps (kerbs:=true): a kerb on the road ahead slows the car as an
    // obstacle does (TODO 219).
    let kerbs = d.flag("kerbs", false)?;
    let kerbs_timeout = d.number("kerbs_timeout", 1.0)?;
    // With things: something moving near holds the car (Ben, 16 Nov: "if we see a car
    // people or stuff moving around us we should likely just stop"). m, s, m/s.
    let moving_hold = d.flag("moving_hold", true)?;
    let nearby = Nearby {
        radius: d.number("moving_radius", 10.0)?,
        within: d.number("moving_within", 3.0)?,
        moving: d.number("moving_speed", MOVING)?,
        ..Nearby::default()
    };
    let moving_settle = d.number("moving_settle", 1.5)?;
    let p = Params {
        base_speed,
        stop_distance,
        front,
        half_width,
        stop_gap,
        near_depth,
        plan_timeout,
        things,
        things_timeout,
        moving_hold,
        nearby,
        road_length,
        road_spacing,
        kerbs_timeout,
    };
    let goal_topics = d.names("goal_status_topics", &["/navigate_to_pose/_action/status"])?;
    for (name, hz) in [
        ("rate_hz", rate_hz),
        ("state_rate_hz", state_rate_hz),
        ("road_rate_hz", road_rate_hz),
    ] {
        if hz.is_nan() || hz <= 0.0 {
            bail!("{name}:={hz}: must be above 0");
        }
    }

    let path = if vehicle_yaml.is_empty() {
        installed_vehicle_yaml()
            .context("vehicle_yaml is empty and aav_description's vehicle.yaml isn't installed")?
    } else {
        PathBuf::from(&vehicle_yaml)
    };
    let car = ActuatorConfig::from_vehicle_yaml(&path)
        .with_context(|| format!("vehicle_yaml {}", path.display()))?;
    let profile: SpeedLoopProfile = speed_loop
        .parse()
        .map_err(|e| anyhow::anyhow!("speed_loop:={speed_loop}: {e}"))?;
    let curve = StopCurve::from_car_model(
        &car,
        max_deceleration,
        max_jerk,
        INPUT_DELAY,
        Some(&profile.config()),
    )?;
    let logger = node.logger().clone();
    let lanes = if lane_map.is_empty() {
        None
    } else {
        let map = lanes::load(&lane_map).with_context(|| format!("lane_map {lane_map}"))?;
        log_info!(
            &logger,
            "lane map {lane_map}: {} roads; the road ahead follows the lane centre",
            map.roads().len()
        );
        Some(map)
    };
    let machine = Behaviour::new(
        curve,
        Settings {
            base: base_speed,
            floor,
            accel,
            fall,
            ask_after,
            release_hold,
            ..Settings::default()
        },
    )?;
    log_info!(
        &logger,
        "{}; cap {}-{} m/s on the road ahead",
        curve.describe(),
        fmt_g(floor),
        fmt_g(base_speed)
    );

    let state = BehaviourNode {
        p,
        curve,
        lanes,
        machine,
        tf: Buffer::new(),
        locks: HashMap::new(),
        near: false,
        fence: None,
        plan: Vec::new(),
        plan_time: f64::NEG_INFINITY,
        passing: false,
        point_plan: Vec::new(),
        stop_lines: StopLines::new(StopLineSettings::default()),
        line_lefts: Vec::new(),
        speed: None,
        odom_time: f64::NEG_INFINITY,
        pose: None,
        pose_yaw: 0.0,
        road_line: Vec::new(),
        near_moving: f64::INFINITY,
        moving_for: MovingFor::new(moving_settle, nearby.moving),
        ahead: f64::INFINITY,
        kerb: f64::INFINITY,
        kerb_time: f64::NEG_INFINITY,
        odom_pose: None,
        moving: f64::INFINITY,
        things_time: f64::NEG_INFINITY,
        sign: None,
        sign_time: f64::NEG_INFINITY,
        goal_active: false,
        aborted: false,
        seen_aborted: HashSet::new(),
        running: HashMap::new(),
        decision: None,
        said: HashSet::new(),
        pub_state: node.create_publisher("/drivers/nav/behaviour/state")?,
        pub_road: node.create_publisher("/world/road_ahead")?,
        clock: node.get_clock(),
        logger,
    };
    let worker = node.create_worker(state);
    let mut keep: Vec<Box<dyn Any>> = Vec::new();

    for lock in Lock::ALL {
        keep.push(Box::new(worker.create_subscription(
            lock.topic().keep_last(10),
            move |s: &mut BehaviourNode, m: Bool| {
                s.locks.insert(lock, m.data);
            },
        )?));
    }
    keep.push(Box::new(worker.create_subscription(
        "/safety/geofence/state".keep_last(10),
        |s: &mut BehaviourNode, m: Text| s.near = m.data.starts_with("NEAR"),
    )?));
    keep.push(Box::new(
        worker.create_subscription(
            "/safety/geofence/fence"
                .keep_last(1)
                .reliable()
                .transient_local(),
            |s: &mut BehaviourNode, m: PolygonStamped| s.on_fence(&m),
        )?,
    ));
    keep.push(Box::new(worker.create_subscription(
        "/plan".keep_last(10),
        |s: &mut BehaviourNode, m: Path| {
            s.plan = path_points(&m);
            s.plan_time = s.now();
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/drivers/nav/planner/status".keep_last(10),
        |s: &mut BehaviourNode, m: DiagnosticStatus| {
            if let Some(how) = m.values.iter().find(|v| v.key == "how") {
                s.passing = how.value == "pass";
            }
        },
    )?));
    keep.push(Box::new(
        worker.create_subscription(
            "/drivers/point/plan"
                .keep_last(1)
                .reliable()
                .transient_local(),
            |s: &mut BehaviourNode, m: Path| s.point_plan = path_points(&m),
        )?,
    ));
    keep.push(Box::new(worker.create_subscription(
        "/drivers/nav/planner/route_stops".keep_last(1),
        |s: &mut BehaviourNode, m: PoseArray| {
            s.stop_lines.set_lines(
                m.poses
                    .iter()
                    .map(|p| (p.position.x, p.position.y))
                    .collect(),
            );
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/localization/odom/local".keep_last(10),
        |s: &mut BehaviourNode, m: Odometry| s.on_local(&m),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/localization/odom/global".keep_last(10),
        |s: &mut BehaviourNode, m: Odometry| {
            s.pose = Some((m.pose.pose.position.x, m.pose.pose.position.y));
            let q = &m.pose.pose.orientation;
            s.pose_yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        },
    )?));
    if things {
        keep.push(Box::new(worker.create_subscription(
            "/world/things".keep_last(10),
            |s: &mut BehaviourNode, m: Things| s.on_things(&m),
        )?));
    }
    if kerbs {
        keep.push(Box::new(worker.create_subscription(
            "/perception/voxels/steps".sensor_data_qos(),
            |s: &mut BehaviourNode, m: PointCloud2| s.on_steps(&m),
        )?));
    }
    if obstacles {
        keep.push(Box::new(worker.create_subscription(
            "/sensors/lidar/scan".sensor_data_qos(),
            |s: &mut BehaviourNode, m: LaserScan| {
                let ranges: Vec<f64> = m.ranges.iter().map(|&r| f64::from(r)).collect();
                s.ahead = lane_ahead(
                    &ranges,
                    f64::from(m.angle_min),
                    f64::from(m.angle_increment),
                    s.p.front,
                    s.p.half_width,
                    REACH,
                );
            },
        )?));
    }
    keep.push(Box::new(worker.create_subscription(
        "/perception/stop_sign/distance".keep_last(10),
        |s: &mut BehaviourNode, m: PointStamped| s.on_sign(&m),
    )?));
    for topic in goal_topics {
        let key = topic.clone();
        keep.push(Box::new(worker.create_subscription(
            topic.as_str().keep_last(10),
            move |s: &mut BehaviourNode, m: GoalStatusArray| s.on_status(&m, &key),
        )?));
    }
    // As tf2's listener: /tf keep last 100; /tf_static latched.
    keep.push(Box::new(worker.create_subscription(
        "/tf".keep_last(100).reliable(),
        |s: &mut BehaviourNode, m: TFMessage| s.on_tf(&m, false),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/tf_static".keep_last(100).reliable().transient_local(),
        |s: &mut BehaviourNode, m: TFMessage| s.on_tf(&m, true),
    )?));

    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate_hz).node_time(),
        |s: &mut BehaviourNode| s.tick(),
    )?));
    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / state_rate_hz).node_time(),
        |s: &mut BehaviourNode| s.publish_state(),
    )?));
    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / road_rate_hz).node_time(),
        |s: &mut BehaviourNode| s.publish_road(),
    )?));
    keep.push(Box::new(worker));
    keep.extend(d.finish());
    Ok(keep)
}

impl BehaviourNode {
    /// s, this node's clock (the simulator's with use_sim_time).
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn header(&self, frame: &str) -> Header {
        Header {
            stamp: stamp(self.clock.now().nsec),
            frame_id: frame.to_string(),
        }
    }

    // --- inputs ------------------------------------------------------------------------

    fn on_tf(&mut self, msg: &TFMessage, fixed: bool) {
        for t in &msg.transforms {
            let r = &t.transform;
            self.tf.set(
                &t.header.frame_id,
                &t.child_frame_id,
                ns(&t.header.stamp),
                Transform {
                    translation: [r.translation.x, r.translation.y, r.translation.z],
                    rotation: [r.rotation.x, r.rotation.y, r.rotation.z, r.rotation.w],
                },
                fixed,
            );
        }
    }

    fn on_fence(&mut self, msg: &PolygonStamped) {
        let corners: Vec<Point> = msg
            .polygon
            .points
            .iter()
            .map(|p| (f64::from(p.x), f64::from(p.y)))
            .collect();
        self.fence = if corners.len() >= 3 {
            match Polygon::new(corners) {
                Ok(fence) => Some(fence),
                Err(e) => {
                    log_error!(&self.logger, "the fence is ignored: {e}");
                    None
                }
            }
        } else {
            None
        };
    }

    fn on_local(&mut self, msg: &Odometry) {
        self.speed = Some(msg.twist.twist.linear.x);
        self.odom_time = self.now();
        let (q, at) = (&msg.pose.pose.orientation, &msg.pose.pose.position);
        let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        self.odom_pose = Some((at.x, at.y, yaw));
    }

    fn on_things(&mut self, msg: &Things) {
        let Some(car) = self.odom_pose else {
            return;
        };
        let seen: Vec<Seen> = msg
            .things
            .iter()
            .map(|t| Seen {
                x: f64::from(t.x),
                y: f64::from(t.y),
                vx: f64::from(t.vx),
                vy: f64::from(t.vy),
                length: f64::from(t.length),
                width: f64::from(t.width),
            })
            .collect();
        // The car's speed: a person walking in counts if they are in the lane by the time
        // the car would reach them (TODO 194).
        let speed = self.speed.map_or(0.0, |v| v.max(0.0));
        // Only what has moved long enough (MovingFor): the parked box flickered into an
        // obstacle stop as the car passed it (17 Nov).
        let now = self.now();
        let tracked: Vec<(u32, bool, Seen)> = msg
            .things
            .iter()
            .zip(&seen)
            .map(|(t, s)| (t.id, t.coasting, *s))
            .collect();
        let sure = self.moving_for.update(now, &tracked);
        self.moving = moving_ahead(
            &sure,
            car,
            self.p.front,
            self.p.half_width,
            REACH,
            MOVING,
            speed,
            Some(Crossing::default()),
        );
        self.near_moving = if self.p.moving_hold {
            moving_near(
                &sure,
                car,
                self.p.front,
                &self.road_here(),
                self.p.half_width,
                self.p.nearby,
            )
        } else {
            f64::INFINITY
        };
        self.things_time = self.now();
    }

    /// The last road ahead's line in base_link (by the global pose), or none.
    fn road_here(&self) -> Vec<Point> {
        let Some((gx, gy)) = self.pose else {
            return Vec::new();
        };
        let (c, s) = (self.pose_yaw.cos(), self.pose_yaw.sin());
        self.road_line
            .iter()
            .map(|&(x, y)| (c * (x - gx) + s * (y - gy), -s * (x - gx) + c * (y - gy)))
            .collect()
    }

    /// Whether a moving thing near holds the car (fresh things only).
    fn moving_holds(&self, now: f64) -> bool {
        self.p.things
            && now - self.things_time <= self.p.things_timeout
            && self.near_moving.is_finite()
    }

    /// x (base_link) of what the car slows for: a moving thing, or the scan's; and a kerb
    /// on the road ahead (TODO 219), whichever is nearer: a kerb counts with the things
    /// too (it never moves, and the stop zone stops for it).
    fn obstacle_ahead(&self, now: f64) -> f64 {
        let ahead = if self.p.things && now - self.things_time <= self.p.things_timeout {
            self.moving
        } else {
            self.ahead
        };
        if now - self.kerb_time <= self.p.kerbs_timeout {
            py_min(ahead, self.kerb)
        } else {
            ahead
        }
    }

    /// The kerbs (base_link): the nearest on the road ahead, m along it.
    fn on_steps(&mut self, msg: &PointCloud2) {
        let steps = cloud_xy(msg);
        self.kerb = kerb_ahead(
            &steps,
            &self.road_here(),
            self.p.front,
            self.p.half_width,
            REACH,
        );
        self.kerb_time = self.now();
    }

    /// Keep the sign in odom, at the picture's stamp: a GPS jump moves map, and mustn't
    /// move the sign.
    fn on_sign(&mut self, msg: &PointStamped) {
        let frame = &msg.header.frame_id;
        // A stamp of 0: the newest (tf2's Time()).
        let at = Some(ns(&msg.header.stamp)).filter(|&t| t != 0);
        match self.tf.lookup("odom", frame, at) {
            Ok(t) => {
                let p = t.then(&Transform {
                    translation: [msg.point.x, msg.point.y, msg.point.z],
                    rotation: Transform::IDENTITY.rotation,
                });
                self.sign = Some((p.translation[0], p.translation[1], frame.clone()));
                self.sign_time = self.now();
            }
            Err(e) => {
                // No TF at the picture's stamp: drop the picture, and say why.
                log_warn!(
                    self.logger.throttle(Duration::from_secs(5)),
                    "stop sign dropped: {e}"
                );
            }
        }
    }

    fn on_status(&mut self, msg: &GoalStatusArray, topic: &str) {
        let running = msg.status_list.iter().any(|s| RUNNING.contains(&s.status));
        self.running.insert(topic.to_string(), running);
        self.goal_active = self.running.values().any(|&r| r);
        for s in &msg.status_list {
            if s.status == GoalStatus::STATUS_ABORTED
                && self.seen_aborted.insert(s.goal_info.goal_id.uuid)
            {
                self.aborted = true;
            }
        }
    }

    // --- each tick ---------------------------------------------------------------------

    /// m from the sign's camera now to the sign, less the stop distance.
    fn sign_left(&self) -> Option<f64> {
        let (sx, sy, frame) = self.sign.as_ref()?;
        let t = self.tf.lookup("odom", frame, None).ok()?;
        let range = hypot(sx - t.translation[0], sy - t.translation[1]);
        Some(range - self.p.stop_distance)
    }

    /// The plan the car drives, and whose: the point driver's line, or /plan.
    fn driving(&self, now: f64) -> (&[Point], Whose) {
        let fresh = !self.plan.is_empty() && now - self.plan_time < self.p.plan_timeout;
        active_plan(&self.point_plan, &self.plan, fresh)
    }

    fn stops(&self, now: f64) -> Vec<Stop> {
        let mut found = Vec::new();
        if let Some(left) = self.sign_left() {
            if now - self.sign_time < SIGN_FRESH {
                found.push(Stop::new(StopReason::Sign, left));
            }
        }
        let ahead = self.obstacle_ahead(now);
        if ahead.is_finite() {
            found.push(Stop::new(
                StopReason::Obstacle,
                ahead - self.p.front - self.p.stop_gap,
            ));
        }
        found.extend(
            self.line_lefts
                .iter()
                .map(|&left| Stop::new(StopReason::StopLine, left)),
        );
        let (plan, whose) = self.driving(now);
        if let (false, Some((gx, gy))) = (plan.is_empty(), self.pose) {
            if self.goal_active || whose == Whose::Point {
                found.push(Stop::new(StopReason::Goal, path_left(plan, gx, gy).0));
            }
            if let Some(fence) = &self.fence {
                let depth = self.p.near_depth;
                let along = along_until(plan, gx, gy, |x, y| fence.depth(x, y) < depth);
                if along.is_finite() {
                    found.push(Stop::new(StopReason::Fence, along));
                }
            }
        }
        found
    }

    fn tick(&mut self) {
        let now = self.now();
        let plan = self.driving(now).0.to_vec();
        self.line_lefts = self
            .stop_lines
            .update(now, &plan, self.pose, self.speed, self.p.front);
        let world = World {
            now,
            speed: self.speed,
            odom_age: now - self.odom_time,
            locks: self
                .locks
                .iter()
                .filter(|(_, &on)| on)
                .map(|(&l, _)| l)
                .collect::<BTreeSet<_>>(),
            near: self.near,
            stops: self.stops(now),
            sign_age: now - self.sign_time,
            // A drive to a point is a goal too: its line is there while it runs.
            goal_active: self.goal_active || self.point_plan.len() >= 2,
            goal_aborted: self.aborted,
            moving: self.moving_holds(now),
        };
        self.aborted = false;
        let d = self.machine.step(&world);
        let changed = self
            .decision
            .as_ref()
            .is_none_or(|old| (old.state, &old.reason) != (d.state, &d.reason));
        if changed {
            let why = if d.reason.is_empty() {
                String::new()
            } else {
                format!(": {}", d.reason)
            };
            log_info!(
                &self.logger,
                "{}{why} (cap {:.2} m/s, left {:.1} m)",
                d.state,
                d.cap,
                d.left
            );
        }
        for warning in &d.warnings {
            if !self.said.contains(warning) {
                log_warn!(&self.logger, "{warning}");
            }
        }
        self.said = d.warnings.iter().cloned().collect();
        self.decision = Some(d);
    }

    /// The road ahead: the plan (or its lane's centre) from the car, cut at the end.
    fn publish_road(&mut self) {
        let now = self.now();
        let (plan, _) = self.driving(now);
        let mut plan = plan.to_vec();
        if let (Some(lanes), false, Some(pose)) = (&self.lanes, plan.is_empty(), self.pose) {
            match lanes::lane_line(
                lanes,
                &plan,
                pose,
                self.p.road_spacing,
                self.p.road_length,
                SMOOTH,
                self.passing,
            ) {
                Ok(line) => plan = line,
                Err(e) => log_error!(&self.logger, "the lane line: {e}"),
            }
        }
        let (state, reason) = self.decision.as_ref().map_or_else(
            || (String::new(), String::new()),
            |d| (d.state.to_string(), d.reason.to_string()),
        );
        let options = BuildOptions {
            state: &state,
            reason: &reason,
            spacing: self.p.road_spacing,
            length: self.p.road_length,
        };
        let mut stops = self.stops(now);
        if self.machine.holding() {
            // Held for something moving: the road ends at the car, so ego holds it at 0.
            stops.push(Stop::new(StopReason::Moving, 0.0));
        }
        let strip = match build(
            &plan,
            self.pose,
            &stops,
            &self.curve,
            self.p.base_speed,
            &options,
        ) {
            Ok(strip) => strip,
            Err(e) => {
                log_error!(&self.logger, "the road ahead: {e}");
                return;
            }
        };
        let strip = match &self.lanes {
            Some(lanes) => lanes::with_edges(strip, lanes, self.passing),
            None => strip,
        };
        // The machine's cap, into the road ahead: ego caps by it (TODO 233).
        let strip = capped(strip, self.decision.as_ref().map_or(0.0, |d| d.cap));
        self.road_line = strip
            .x
            .iter()
            .copied()
            .zip(strip.y.iter().copied())
            .collect();
        let msg = road_msg(&strip, self.header("map"));
        if let Err(e) = self.pub_road.publish(msg) {
            log_error!(&self.logger, "road_ahead: {e}");
        }
    }

    fn publish_state(&mut self) {
        let Some(d) = &self.decision else {
            return;
        };
        let level = if !d.warnings.is_empty() || d.state == State::Ask {
            DiagnosticStatus::WARN
        } else {
            DiagnosticStatus::OK
        };
        let why = if d.reason.is_empty() {
            String::new()
        } else {
            format!(": {}", d.reason)
        };
        let msg = DiagnosticStatus {
            level,
            name: "behaviour".into(),
            message: format!("{}{why}", d.state),
            values: diagnostic(d)
                .into_iter()
                .map(|(key, value)| KeyValue {
                    key: key.into(),
                    value,
                })
                .collect(),
            ..Default::default()
        };
        if let Err(e) = self.pub_state.publish(msg) {
            log_error!(&self.logger, "behaviour/state: {e}");
        }
    }
}

/// A cloud's (x, y) points (float32 fields x and y); none without them.
fn cloud_xy(msg: &PointCloud2) -> Vec<Point> {
    let offset = |name: &str| {
        msg.fields
            .iter()
            .find(|f| f.name == name && f.datatype == PointField::FLOAT32)
            .map(|f| f.offset as usize)
    };
    let (Some(ox), Some(oy)) = (offset("x"), offset("y")) else {
        return Vec::new();
    };
    let step = msg.point_step as usize;
    if step == 0 {
        return Vec::new();
    }
    let read = |at: usize| {
        msg.data.get(at..at + 4).map(|b| {
            let bytes = [b[0], b[1], b[2], b[3]];
            f64::from(if msg.is_bigendian {
                f32::from_be_bytes(bytes)
            } else {
                f32::from_le_bytes(bytes)
            })
        })
    };
    (0..(msg.width as usize) * (msg.height as usize))
        .map_while(|i| Some((read(i * step + ox)?, read(i * step + oy)?)))
        .collect()
}

/// The message for a strip (road_ahead_ros.py's `to_msg`).
fn road_msg(strip: &Strip, header: Header) -> RoadAhead {
    let narrow = |v: &[f64]| v.iter().map(|&x| x as f32).collect::<Vec<f32>>();
    RoadAhead {
        header,
        spacing: strip.spacing as f32,
        x: narrow(&strip.x),
        y: narrow(&strip.y),
        heading: narrow(&strip.heading),
        speed_limit: narrow(&strip.speed_limit),
        left_hard: narrow(&strip.left_hard),
        right_hard: narrow(&strip.right_hard),
        left_soft: narrow(&strip.left_soft),
        right_soft: narrow(&strip.right_soft),
        stops: strip
            .stops
            .iter()
            .map(|s| StopPoint {
                along: s.along as f32,
                reason: s.reason.to_string(),
                kind: s.kind.code(),
            })
            .collect(),
        end: strip.end as f32,
        state: strip.state.clone(),
        reason: strip.reason.clone(),
    }
}
