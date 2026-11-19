//! navigator: our own navigator (Nav2's bt_navigator and waypoint_follower, in Rust).
//!
//! Actions     Nav2's own names, so it is a drop-in for bt_navigator and waypoint_follower:
//!             no goal sender needs to know which navigator runs.
//!             /navigate_to_pose           nav2_msgs/action/NavigateToPose: to one map pose
//!             /navigate_through_poses
//!                                         nav2_msgs/action/NavigateThroughPoses: one path
//!                                         through each pose
//!             /follow_waypoints
//!                                         nav2_msgs/action/FollowWaypoints: map poses,
//!                                         passed without stopping (waypoints: pass_through)
//!             /follow_gps_waypoints
//!                                         nav2_msgs/action/FollowGPSWaypoints: lat/lon,
//!                                         put on the map by navsat_transform's datum
//!                                         (navsat.yaml), then as follow_waypoints
//! Subscribes  /goal_pose                  geometry_msgs/PoseStamped (map): a to-pose goal
//!                                         from RViz, the map window, the dev window
//!             /tf                         tf2_msgs/TFMessage: map -> odom -> base_link;
//!                                         older than TRANSFORM_TIMEOUT: no car pose (no
//!                                         waypoint passed, no plan kept, no progress)
//! Publishes   /drivers/nav/navigator/status
//!                                         diagnostic_msgs/DiagnosticStatus, 2 Hz and at
//!                                         each goal's end: the goal, the step, recoveries
//! Calls       the planner's ComputePathToPose and ComputePathThroughPoses
//!             (planner_action, planner_through_action), its IsPathValid once a second
//!             on the final approach (is_path_valid_service: planner_server's, or ours,
//!             aav_planner, where none runs), the follower's FollowPath
//!             (follow_action), the costmaps' ClearEntireCostmap (clear_local,
//!             clear_global: comma-separated lists), and with backup_action set,
//!             behavior_server's BackUp; without it (no behaviour server: all ours), the
//!             BackUp recovery is the follower's FollowPath on a straight line behind the
//!             car (the follower must reverse: mppi_lag with reversing:=true).
//!             Every call has a time limit (settings: plan_timeout, follow_timeout: the
//!             follower's silence, recovery_timeout): a server that dies mid-goal fails
//!             the goal; it never hangs it.
//! Parameters  settings (JSON: aav_description's vehicle.navigator_params), the action and
//!             service names above, navsat_file, map_frame, odom_frame, base_frame.
//!
//! navigation.launch.py navigator:=own starts it, in place of bt_navigator and
//! waypoint_follower. The maths is the `navigator` crate; this file only moves messages.

use std::any::Any;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{anyhow, bail, Context as _, Result};
use futures::channel::oneshot;
use futures::stream;
use map_core::earth::{Datum, LatLon, BUILT_IN_NAVSAT_YAML};
use navigator::call::{follow_call, Called};
use navigator::path::straight_back;
use navigator::{
    Abort, BackUp as BackUpSettings, Command, FollowError, Mission, Navigator, Outcome, PlanError,
    Pose, Settings, Token, WaypointMode,
};
use rclrs::*;
use ros_env::builtin_interfaces::msg::{Duration as DurationMsg, Time};
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geographic_msgs::msg::GeoPose;
use ros_env::geometry_msgs::msg::{Point, Pose as PoseMsg, PoseStamped, Quaternion};
use ros_env::nav2_msgs::action::{
    BackUp, BackUp_Goal, ComputePathThroughPoses, ComputePathThroughPoses_Goal, ComputePathToPose,
    ComputePathToPose_Goal, FollowGPSWaypoints, FollowGPSWaypoints_Feedback,
    FollowGPSWaypoints_Result, FollowPath, FollowPath_Goal, FollowWaypoints,
    FollowWaypoints_Feedback, FollowWaypoints_Result, NavigateThroughPoses,
    NavigateThroughPoses_Feedback, NavigateThroughPoses_Result, NavigateToPose,
    NavigateToPose_Feedback, NavigateToPose_Result,
};
use ros_env::nav2_msgs::msg::MissedWaypoint;
use ros_env::nav2_msgs::srv::{
    ClearEntireCostmap, ClearEntireCostmap_Response, IsPathValid, IsPathValid_Request,
    IsPathValid_Response,
};
use ros_env::nav_msgs::msg::Path;
use ros_env::std_msgs::msg::Header;
use ros_env::tf2_msgs::msg::TFMessage;
use serde_json::Value;

/// m between the poses of the BackUp recovery's straight line (as the planners' paths).
const BACK_UP_SPACING: f64 = 0.05;

/// s: a /tf transform older than this is no car pose (robot_localization sends at 30 Hz).
const TRANSFORM_TIMEOUT: f64 = 1.0;

/// Nav2's action names: ours serves them in bt_navigator's and waypoint_follower's place.
const PREFIX: &str = "/";
const STATUS_TOPIC: &str = "/drivers/nav/navigator/status";
const GOAL_TOPIC: &str = "/goal_pose";
/// s between ticks, and between feedback and status messages.
const TICK: f64 = 0.1;
const REPORT_EVERY: u32 = 5;
/// Ticks between IsPathValid calls on the final approach: once a second, as the trees.
const CHECK_EVERY: u32 = 10;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("navigator")?;

    let mut handles: Vec<Box<dyn Any>> = Vec::new();
    let mut text_param = |name: &str, default: &str| -> Result<String> {
        let p = node
            .declare_parameter::<Arc<str>>(name)
            .default(default.into())
            .read_only()
            .with_context(|| format!("parameter {name} (text)"))?;
        let value = p.get().to_string();
        handles.push(Box::new(p));
        Ok(value)
    };
    let settings_json = text_param("settings", "{}")?;
    let names = Names {
        plan: text_param("planner_action", "/compute_path_to_pose")?,
        plan_through: text_param("planner_through_action", "/compute_path_through_poses")?,
        follow: text_param("follow_action", "/follow_path")?,
        clear_local: list(&text_param(
            "clear_local",
            "/local_costmap/clear_entirely_local_costmap",
        )?),
        clear_global: list(&text_param(
            "clear_global",
            "/global_costmap/clear_entirely_global_costmap",
        )?),
        backup: text_param("backup_action", "")?,
        is_path_valid: text_param("is_path_valid_service", "/is_path_valid")?,
    };
    let navsat_file = text_param("navsat_file", "")?;
    let frames = Frames {
        map: text_param("map_frame", "map")?,
        odom: text_param("odom_frame", "odom")?,
        base: text_param("base_frame", "base_link")?,
    };
    let spec: Value = serde_json::from_str(&settings_json).context("settings: JSON")?;
    let settings = settings_from(&spec)?;
    let navsat = if navsat_file.is_empty() {
        BUILT_IN_NAVSAT_YAML.to_string()
    } else {
        std::fs::read_to_string(&navsat_file).with_context(|| format!("reading {navsat_file}"))?
    };
    let datum = Datum::from_navsat_yaml(&navsat).map_err(|e| anyhow!("the datum: {e}"))?;
    log_info!(
        node.logger(),
        "navigator (ours): plans {} / {}, follows {}, waypoints {:?}, back up {}; datum {:.6}, {:.6}",
        names.plan,
        names.plan_through,
        names.follow,
        settings.waypoints,
        if settings.backup.is_some() { "on" } else { "off" },
        datum.origin.lat,
        datum.origin.lon
    );

    let clients = Clients {
        plan: node.create_action_client::<ComputePathToPose>(names.plan.as_str())?,
        plan_through: node
            .create_action_client::<ComputePathThroughPoses>(names.plan_through.as_str())?,
        follow: node.create_action_client::<FollowPath>(names.follow.as_str())?,
        clear_local: names
            .clear_local
            .iter()
            .map(|n| node.create_client::<ClearEntireCostmap>(n.as_str()))
            .collect::<Result<_, _>>()?,
        clear_global: names
            .clear_global
            .iter()
            .map(|n| node.create_client::<ClearEntireCostmap>(n.as_str()))
            .collect::<Result<_, _>>()?,
        backup: if names.backup.is_empty() {
            None
        } else {
            Some(node.create_action_client::<BackUp>(names.backup.as_str())?)
        },
        is_path_valid: node.create_client::<IsPathValid>(names.is_path_valid.as_str())?,
    };

    let core = Core {
        nav: Navigator::new(settings),
        frames,
        datum,
        map_odom: None,
        odom_base: None,
        goals: HashMap::new(),
        next_id: 0,
        clients,
        commands: Arc::clone(node.commands()),
        me: None,
        status: node.create_publisher(STATUS_TOPIC)?,
        clock: node.get_clock(),
        logger: node.logger().clone(),
        ticks: 0,
        checking: false,
        step: "idle",
        last_end: String::new(),
        done: (0, 0),
    };
    let worker = node.create_worker(core);
    let me = worker.clone();
    drop(worker.run(move |c: &mut Core| c.me = Some(me)));
    let _tf = worker.create_subscription("/tf", |c: &mut Core, msg: TFMessage| c.on_tf(&msg))?;
    let _goal_pose = worker.create_subscription(GOAL_TOPIC, |c: &mut Core, msg: PoseStamped| {
        c.on_goal_pose(&msg)
    })?;
    let _tick = worker
        .create_timer_repeating(Duration::from_secs_f64(TICK).node_time(), |c: &mut Core| {
            c.tick()
        })?;

    let w = worker.clone();
    let _to_pose = node.create_action_server::<NavigateToPose, _>(
        format!("{PREFIX}navigate_to_pose").as_str(),
        move |requested: RequestedGoal<NavigateToPose>| {
            let worker = w.clone();
            async move {
                let goal = requested.accept().execute();
                let asked = goal.goal().clone();
                let feedback = Feedback::ToPose(goal.feedback_publisher());
                let (done, finished) = oneshot::channel();
                let id = worker
                    .run(move |c: &mut Core| {
                        c.start_goal(Ask::ToPose(asked.pose.clone()), done, feedback)
                    })
                    .await
                    .unwrap_or(0);
                let result = |code: u16, msg: &str| NavigateToPose_Result {
                    error_code: code,
                    error_msg: msg.into(),
                };
                match goal.until_cancel_requested(finished).await {
                    Ok(Ok(Outcome::Succeeded { .. })) => goal.succeeded_with(result(0, "")),
                    Ok(Ok(Outcome::Aborted { reason, .. })) => {
                        goal.aborted_with(result(reason.code(), &reason.to_string()))
                    }
                    Ok(_) => goal.aborted_with(result(0, "stopped")),
                    Err(_) => {
                        let _ = worker.run(move |c: &mut Core| c.cancel_goal(id)).await;
                        goal.begin_cancelling()
                            .cancelled_with(result(0, "cancelled"))
                    }
                }
            }
        },
    )?;

    let w = worker.clone();
    let _through = node.create_action_server::<NavigateThroughPoses, _>(
        format!("{PREFIX}navigate_through_poses").as_str(),
        move |requested: RequestedGoal<NavigateThroughPoses>| {
            let worker = w.clone();
            async move {
                let goal = requested.accept().execute();
                let asked = goal.goal().clone();
                let feedback = Feedback::Through(goal.feedback_publisher());
                let (done, finished) = oneshot::channel();
                let id = worker
                    .run(move |c: &mut Core| {
                        c.start_goal(Ask::Through(asked.poses.clone()), done, feedback)
                    })
                    .await
                    .unwrap_or(0);
                let result = |code: u16, msg: &str| NavigateThroughPoses_Result {
                    error_code: code,
                    error_msg: msg.into(),
                };
                match goal.until_cancel_requested(finished).await {
                    Ok(Ok(Outcome::Succeeded { .. })) => goal.succeeded_with(result(0, "")),
                    Ok(Ok(Outcome::Aborted { reason, .. })) => {
                        goal.aborted_with(result(reason.code(), &reason.to_string()))
                    }
                    Ok(_) => goal.aborted_with(result(0, "stopped")),
                    Err(_) => {
                        let _ = worker.run(move |c: &mut Core| c.cancel_goal(id)).await;
                        goal.begin_cancelling()
                            .cancelled_with(result(0, "cancelled"))
                    }
                }
            }
        },
    )?;

    let w = worker.clone();
    let _waypoints = node.create_action_server::<FollowWaypoints, _>(
        format!("{PREFIX}follow_waypoints").as_str(),
        move |requested: RequestedGoal<FollowWaypoints>| {
            let worker = w.clone();
            async move {
                let goal = requested.accept().execute();
                let asked = goal.goal().clone();
                let given = asked.poses.clone();
                let feedback = Feedback::Waypoints(goal.feedback_publisher());
                let (done, finished) = oneshot::channel();
                let id = worker
                    .run(move |c: &mut Core| {
                        c.start_goal(Ask::Waypoints(asked.poses.clone()), done, feedback)
                    })
                    .await
                    .unwrap_or(0);
                let result = |code: u16, msg: &str, missed: &[usize]| FollowWaypoints_Result {
                    missed_waypoints: missed_list(missed, &given),
                    error_code: code,
                    error_msg: msg.into(),
                };
                match goal.until_cancel_requested(finished).await {
                    Ok(Ok(Outcome::Succeeded { missed })) => {
                        goal.succeeded_with(result(0, "", &missed))
                    }
                    Ok(Ok(Outcome::Aborted { reason, missed })) => {
                        goal.aborted_with(result(reason.code(), &reason.to_string(), &missed))
                    }
                    Ok(_) => goal.aborted_with(result(0, "stopped", &[])),
                    Err(_) => {
                        let _ = worker.run(move |c: &mut Core| c.cancel_goal(id)).await;
                        goal.begin_cancelling()
                            .cancelled_with(result(0, "cancelled", &[]))
                    }
                }
            }
        },
    )?;

    let w = worker.clone();
    let _gps = node.create_action_server::<FollowGPSWaypoints, _>(
        format!("{PREFIX}follow_gps_waypoints").as_str(),
        move |requested: RequestedGoal<FollowGPSWaypoints>| {
            let worker = w.clone();
            async move {
                let goal = requested.accept().execute();
                let asked = goal.goal().clone();
                let feedback = Feedback::Gps(goal.feedback_publisher());
                let (done, finished) = oneshot::channel();
                let gps = asked.gps_poses.clone();
                let started = worker
                    .run(move |c: &mut Core| {
                        let poses = c.gps_to_map(&gps);
                        let given = c.stamped(&poses);
                        (c.start_goal(Ask::Map(poses), done, feedback), given)
                    })
                    .await;
                let (id, given) = started.unwrap_or_default();
                let result = |code: u16, msg: &str, missed: &[usize]| FollowGPSWaypoints_Result {
                    missed_waypoints: missed_list(missed, &given),
                    error_code: code as i16,
                    error_msg: msg.into(),
                };
                match goal.until_cancel_requested(finished).await {
                    Ok(Ok(Outcome::Succeeded { missed })) => {
                        goal.succeeded_with(result(0, "", &missed))
                    }
                    Ok(Ok(Outcome::Aborted { reason, missed })) => {
                        goal.aborted_with(result(reason.code(), &reason.to_string(), &missed))
                    }
                    Ok(_) => goal.aborted_with(result(0, "stopped", &[])),
                    Err(_) => {
                        let _ = worker.run(move |c: &mut Core| c.cancel_goal(id)).await;
                        goal.begin_cancelling()
                            .cancelled_with(result(0, "cancelled", &[]))
                    }
                }
            }
        },
    )?;

    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    drop(handles);
    Ok(())
}

/// A comma-separated list of names; empty items dropped.
fn list(text: &str) -> Vec<String> {
    text.split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(String::from)
        .collect()
}

/// The navigator's settings from vehicle.navigator_params' JSON (missing: the defaults).
fn settings_from(spec: &Value) -> Result<Settings> {
    let d = Settings::default();
    let number = |key: &str, default: f64| -> Result<f64> {
        match &spec[key] {
            Value::Null => Ok(default),
            v => v
                .as_f64()
                .ok_or_else(|| anyhow!("{key}: a number is needed")),
        }
    };
    let backup = match &spec["backup"] {
        Value::Null => None,
        b => Some(BackUpSettings {
            distance: b["distance"]
                .as_f64()
                .ok_or_else(|| anyhow!("backup.distance"))?,
            speed: b["speed"].as_f64().ok_or_else(|| anyhow!("backup.speed"))?,
        }),
    };
    let waypoints = match spec["waypoints"].as_str() {
        None | Some("pass_through") => WaypointMode::PassThrough,
        Some("stop_at_each") => WaypointMode::StopAtEach,
        Some(other) => bail!("waypoints: {other}: pass_through or stop_at_each"),
    };
    Ok(Settings {
        replan_period: number("replan_period", d.replan_period)?,
        through_replan_period: number("through_replan_period", d.through_replan_period)?,
        final_approach: number("final_approach", d.final_approach)?,
        path_end_tolerance: number("path_end_tolerance", d.path_end_tolerance)?,
        path_max_age: number("path_max_age", d.path_max_age)?,
        keep_path_age: number("keep_path_age", d.keep_path_age)?,
        retries: number("retries", f64::from(d.retries))? as u32,
        wait: number("wait", d.wait)?,
        backup,
        pass_radius: number("pass_radius", d.pass_radius)?,
        on_path: number("on_path", d.on_path)?,
        waypoints,
        stop_on_failure: spec["stop_on_failure"]
            .as_bool()
            .unwrap_or(d.stop_on_failure),
        replan_failures: number("replan_failures", f64::from(d.replan_failures))? as u32,
        plan_timeout: number("plan_timeout", d.plan_timeout)?,
        follow_timeout: number("follow_timeout", d.follow_timeout)?,
        recovery_timeout: number("recovery_timeout", d.recovery_timeout)?,
    })
}

fn missed_list(missed: &[usize], given: &[PoseStamped]) -> Vec<MissedWaypoint> {
    missed
        .iter()
        .map(|&i| MissedWaypoint {
            index: i as u32,
            goal: given.get(i).cloned().unwrap_or_default(),
            error_code: 0,
        })
        .collect()
}

struct Names {
    plan: String,
    plan_through: String,
    follow: String,
    clear_local: Vec<String>,
    clear_global: Vec<String>,
    backup: String,
    is_path_valid: String,
}

struct Frames {
    map: String,
    odom: String,
    base: String,
}

struct Clients {
    plan: ActionClient<ComputePathToPose>,
    plan_through: ActionClient<ComputePathThroughPoses>,
    follow: ActionClient<FollowPath>,
    clear_local: Vec<Client<ClearEntireCostmap>>,
    clear_global: Vec<Client<ClearEntireCostmap>>,
    backup: Option<ActionClient<BackUp>>,
    is_path_valid: Client<IsPathValid>,
}

/// Where each goal's feedback goes.
enum Feedback {
    ToPose(FeedbackPublisher<NavigateToPose>),
    Through(FeedbackPublisher<NavigateThroughPoses>),
    Waypoints(FeedbackPublisher<FollowWaypoints>),
    Gps(FeedbackPublisher<FollowGPSWaypoints>),
    /// /goal_pose: nobody to tell.
    None,
}

/// A goal as it arrived.
enum Ask {
    ToPose(PoseStamped),
    Through(Vec<PoseStamped>),
    Waypoints(Vec<PoseStamped>),
    /// Already map poses (GPS waypoints, put on the map).
    Map(Vec<Pose>),
}

struct Waiting {
    done: Option<oneshot::Sender<Outcome>>,
    feedback: Feedback,
}

/// The worker's payload.
struct Core {
    nav: Navigator,
    frames: Frames,
    datum: Datum,
    /// map -> odom and odom -> base_link, each with when it arrived (s).
    map_odom: Option<(Pose, f64)>,
    odom_base: Option<(Pose, f64)>,
    goals: HashMap<u64, Waiting>,
    next_id: u64,
    clients: Clients,
    commands: Arc<ExecutorCommands>,
    /// This worker, to send answers back into.
    me: Option<Worker<Core>>,
    status: Publisher<DiagnosticStatus>,
    clock: Clock,
    logger: Logger,
    ticks: u32,
    /// An IsPathValid call is out: no second one until it answers.
    checking: bool,
    step: &'static str,
    last_end: String,
    /// Goals succeeded, and not.
    done: (u64, u64),
}

fn yaw_of(q: &Quaternion) -> f64 {
    (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
}

fn pose_of(p: &PoseMsg) -> Pose {
    Pose::new(p.position.x, p.position.y, yaw_of(&p.orientation))
}

fn msg_of(p: &Pose) -> PoseMsg {
    PoseMsg {
        position: Point {
            x: p.x,
            y: p.y,
            z: 0.0,
        },
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: (p.yaw / 2.0).sin(),
            w: (p.yaw / 2.0).cos(),
        },
    }
}

fn seconds(d: f64) -> DurationMsg {
    let d = if d.is_finite() { d.max(0.0) } else { 0.0 };
    DurationMsg {
        sec: d.floor() as i32,
        nanosec: ((d - d.floor()) * 1e9) as u32,
    }
}

impl Core {
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn stamp(&self) -> Time {
        self.clock.now().to_ros_msg().unwrap_or_default()
    }

    /// The car in map, from /tf; `None` while either transform is missing or stale.
    fn car(&self) -> Option<Pose> {
        let (a, a_at) = self.map_odom.as_ref()?;
        let (b, b_at) = self.odom_base.as_ref()?;
        let fresh = self.now() - a_at.min(*b_at) <= TRANSFORM_TIMEOUT;
        fresh.then(|| a.compose(b))
    }

    fn on_tf(&mut self, msg: &TFMessage) {
        for t in &msg.transforms {
            let parent = t.header.frame_id.trim_start_matches('/');
            let child = t.child_frame_id.trim_start_matches('/');
            let r = &t.transform;
            let pose = Pose::new(r.translation.x, r.translation.y, yaw_of(&r.rotation));
            if parent == self.frames.map && child == self.frames.odom {
                self.map_odom = Some((pose, self.now()));
            } else if parent == self.frames.odom && child == self.frames.base {
                self.odom_base = Some((pose, self.now()));
            }
        }
    }

    /// A goal pose on the map: as it is in map (or with no frame); through map -> odom
    /// from odom; through the car's pose from base_link (Nav2 turns goals from any frame
    /// it has a transform for). `None` for another frame, or when /tf is stale. The latest
    /// transforms, not those at the goal's stamp.
    fn in_map(&self, p: &PoseStamped) -> Option<Pose> {
        let f = p.header.frame_id.trim_start_matches('/');
        let pose = pose_of(&p.pose);
        if f.is_empty() || f == self.frames.map {
            Some(pose)
        } else if f == self.frames.odom {
            let (map_odom, at) = self.map_odom.as_ref()?;
            (self.now() - at <= TRANSFORM_TIMEOUT).then(|| map_odom.compose(&pose))
        } else if f == self.frames.base {
            self.car().map(|car| car.compose(&pose))
        } else {
            None
        }
    }

    fn stamped(&self, poses: &[Pose]) -> Vec<PoseStamped> {
        let header = Header {
            stamp: self.stamp(),
            frame_id: self.frames.map.clone(),
        };
        poses
            .iter()
            .map(|p| PoseStamped {
                header: header.clone(),
                pose: msg_of(p),
            })
            .collect()
    }

    /// Lat/lon to map poses, as navsat_transform's datum puts them. The heading as given
    /// (map y is true north at the datum).
    fn gps_to_map(&self, gps: &[GeoPose]) -> Vec<Pose> {
        gps.iter()
            .map(|g| {
                let p = self
                    .datum
                    .to_map(LatLon::new(g.position.latitude, g.position.longitude));
                let q = &g.orientation;
                let q = Quaternion {
                    x: q.x,
                    y: q.y,
                    z: q.z,
                    w: q.w,
                };
                Pose::new(p.x, p.y, yaw_of(&q))
            })
            .collect()
    }

    fn on_goal_pose(&mut self, msg: &PoseStamped) {
        let _ = self.start_goal(Ask::ToPose(msg.clone()), None, Feedback::None);
    }

    /// A new goal: its number (0 if refused at once).
    fn start_goal(
        &mut self,
        ask: Ask,
        done: impl Into<Option<oneshot::Sender<Outcome>>>,
        feedback: Feedback,
    ) -> u64 {
        self.next_id += 1;
        let id = self.next_id;
        let all = |ps: &[PoseStamped], me: &Self| -> Option<Vec<Pose>> {
            ps.iter().map(|p| me.in_map(p)).collect()
        };
        let mission = match &ask {
            Ask::ToPose(p) => self.in_map(p).map(Mission::ToPose),
            Ask::Through(ps) => all(ps, self).map(Mission::ThroughPoses),
            Ask::Waypoints(ps) => all(ps, self).map(Mission::Waypoints),
            Ask::Map(ps) => Some(Mission::Waypoints(ps.clone())),
        };
        self.goals.insert(
            id,
            Waiting {
                done: done.into(),
                feedback,
            },
        );
        let Some(mission) = mission else {
            self.finish(
                id,
                Outcome::Aborted {
                    reason: Abort::InvalidGoal(format!(
                        "a pose is not in {}, {} or {} (or /tf is over {TRANSFORM_TIMEOUT} s old)",
                        self.frames.map, self.frames.odom, self.frames.base
                    )),
                    missed: Vec::new(),
                },
            );
            return id;
        };
        log_info!(&self.logger, "goal {id}: {mission:?}");
        let now = self.now();
        let out = self.nav.start(id, mission, now);
        self.carry_out(out);
        let car = self.car();
        let out = self.nav.tick(now, car.as_ref());
        self.carry_out(out);
        id
    }

    fn cancel_goal(&mut self, id: u64) {
        let out = self.nav.cancel(id);
        self.carry_out(out);
    }

    fn tick(&mut self) {
        let now = self.now();
        let car = self.car();
        let out = self.nav.tick(now, car.as_ref());
        self.carry_out(out);
        self.ticks += 1;
        if self.ticks.is_multiple_of(CHECK_EVERY) {
            self.check_path(now, car.as_ref());
        }
        if self.ticks.is_multiple_of(REPORT_EVERY) {
            self.report(now, car.as_ref());
        }
    }

    /// Feedback for the goal running, and the status line.
    fn report(&mut self, now: f64, car: Option<&Pose>) {
        let progress = self.nav.progress(car);
        if let Some(p) = progress {
            let current_pose = PoseStamped {
                header: Header {
                    stamp: self.stamp(),
                    frame_id: self.frames.map.clone(),
                },
                pose: car.map(msg_of).unwrap_or_default(),
            };
            if let Some(w) = self.goals.get(&p.id) {
                let left = p.distance_remaining as f32;
                let taken = seconds(now - p.started);
                let recoveries = p.recoveries.min(i16::MAX as u32) as i16;
                match &w.feedback {
                    Feedback::ToPose(f) => {
                        let _ = f.publish(NavigateToPose_Feedback {
                            current_pose,
                            navigation_time: taken,
                            estimated_time_remaining: seconds(0.0),
                            number_of_recoveries: recoveries,
                            distance_remaining: left,
                        });
                    }
                    Feedback::Through(f) => {
                        let _ = f.publish(NavigateThroughPoses_Feedback {
                            current_pose,
                            navigation_time: taken,
                            estimated_time_remaining: seconds(0.0),
                            number_of_recoveries: recoveries,
                            distance_remaining: left,
                            number_of_poses_remaining: p.poses_remaining.min(i16::MAX as usize)
                                as i16,
                        });
                    }
                    Feedback::Waypoints(f) => {
                        let _ = f.publish(FollowWaypoints_Feedback {
                            current_waypoint: p.waypoint as u32,
                        });
                    }
                    Feedback::Gps(f) => {
                        let _ = f.publish(FollowGPSWaypoints_Feedback {
                            current_waypoint: p.waypoint as u32,
                        });
                    }
                    Feedback::None => {}
                }
            }
        }
        self.publish_status(progress);
    }

    fn publish_status(&mut self, progress: Option<navigator::Progress>) {
        let kv = |k: &str, v: String| KeyValue {
            key: k.into(),
            value: v,
        };
        let mut values = vec![
            kv("step", self.step.into()),
            kv("succeeded", self.done.0.to_string()),
            kv("failed", self.done.1.to_string()),
            kv("last_end", self.last_end.clone()),
        ];
        if let Some(p) = progress {
            values.extend([
                kv("goal", p.id.to_string()),
                kv(
                    "distance_remaining_m",
                    format!("{:.1}", p.distance_remaining),
                ),
                kv("recoveries", p.recoveries.to_string()),
                kv("waypoint", p.waypoint.to_string()),
                kv("poses_remaining", p.poses_remaining.to_string()),
            ]);
        } else {
            self.step = "idle";
        }
        let level = if self.step.starts_with("recover") {
            DiagnosticStatus::WARN
        } else {
            DiagnosticStatus::OK
        };
        let _ = self.status.publish(&DiagnosticStatus {
            level,
            name: "navigator".into(),
            message: self.step.into(),
            hardware_id: String::new(),
            values,
        });
    }

    fn finish(&mut self, id: u64, outcome: Outcome) {
        let text = match &outcome {
            Outcome::Succeeded { missed } if missed.is_empty() => "succeeded".to_string(),
            Outcome::Succeeded { missed } => format!("succeeded, missed {missed:?}"),
            Outcome::Aborted { reason, .. } => format!("aborted: {reason}"),
            Outcome::Cancelled => "cancelled".to_string(),
        };
        if matches!(outcome, Outcome::Succeeded { .. }) {
            self.done.0 += 1;
            log_info!(&self.logger, "goal {id}: {text}");
        } else {
            self.done.1 += 1;
            log_warn!(&self.logger, "goal {id}: {text}");
        }
        self.last_end = format!("goal {id}: {text}");
        if let Some(mut w) = self.goals.remove(&id) {
            if let Some(done) = w.done.take() {
                let _ = done.send(outcome);
            }
        }
        self.publish_status(None);
    }

    /// Carry out the navigator's commands; each answer comes back through the worker.
    fn carry_out(&mut self, commands: Vec<Command>) {
        for command in commands {
            match command {
                Command::Plan { token, goals } => {
                    self.step = "planning";
                    self.plan(token, goals);
                }
                Command::Follow { token, path } => {
                    self.step = "following";
                    self.follow(token, path);
                }
                Command::StopFollowing => {
                    let client = self.clients.follow.clone();
                    drop(self.commands.run(async move {
                        let mut answer = client.cancel_all_goals();
                        let _ = (&mut *answer).await;
                    }));
                }
                Command::Clear {
                    token,
                    local,
                    global,
                } => {
                    self.step = "recovering: clearing costmaps";
                    self.clear(token, local, global);
                }
                Command::BackUp {
                    token,
                    distance,
                    speed,
                } => {
                    self.step = "recovering: backing up";
                    self.back_up(token, distance, speed);
                }
                Command::Finished { id, outcome } => self.finish(id, outcome),
            }
        }
    }

    /// Run `answer` (an async step) and hand its result to `then` on the worker.
    fn spawn<T, F, A>(&self, work: F, then: A)
    where
        T: Send + 'static,
        F: std::future::Future<Output = T> + Send + 'static,
        A: FnOnce(&mut Core, T, f64) + Send + 'static,
    {
        let Some(me) = self.me.clone() else {
            return;
        };
        drop(self.commands.run(async move {
            let out = work.await;
            drop(me.run(move |c: &mut Core| {
                let now = c.now();
                then(c, out, now);
            }));
        }));
    }

    fn plan(&mut self, token: Token, goals: Vec<Pose>) {
        let stamped = self.stamped(&goals);
        let then = move |c: &mut Core, answer: Result<Vec<Pose>, (PlanError, String)>, now: f64| {
            if let Err((error, why)) = &answer {
                log_warn!(
                    c.logger.throttle(Duration::from_secs(2)),
                    "no path ({error:?}): {why}"
                );
            }
            let out = c.nav.planned(token, answer, now);
            c.carry_out(out);
        };
        if stamped.len() == 1 {
            let client = self.clients.plan.clone();
            let goal = ComputePathToPose_Goal {
                goal: stamped[0].clone(),
                start: PoseStamped::default(),
                planner_id: String::new(),
                use_start: false,
            };
            self.spawn(
                async move {
                    if !client.server_is_available().unwrap_or(false) {
                        return Err((
                            PlanError::Unknown,
                            "no planner serves ComputePathToPose".into(),
                        ));
                    }
                    let Some(g) = client.request_goal(goal).await else {
                        return Err((PlanError::Unknown, "the planner refused the goal".into()));
                    };
                    let (status, result) = g.result.await;
                    if status == GoalStatusCode::Succeeded {
                        Ok(result.path.poses.iter().map(|p| pose_of(&p.pose)).collect())
                    } else {
                        Err((PlanError::from_code(result.error_code), result.error_msg))
                    }
                },
                then,
            );
        } else {
            let client = self.clients.plan_through.clone();
            let goal = ComputePathThroughPoses_Goal {
                goals: stamped,
                start: PoseStamped::default(),
                planner_id: String::new(),
                use_start: false,
            };
            self.spawn(
                async move {
                    if !client.server_is_available().unwrap_or(false) {
                        return Err((
                            PlanError::Unknown,
                            "no planner serves ComputePathThroughPoses".into(),
                        ));
                    }
                    let Some(g) = client.request_goal(goal).await else {
                        return Err((PlanError::Unknown, "the planner refused the goal".into()));
                    };
                    let (status, result) = g.result.await;
                    if status == GoalStatusCode::Succeeded {
                        Ok(result.path.poses.iter().map(|p| pose_of(&p.pose)).collect())
                    } else {
                        Err((PlanError::from_code(result.error_code), result.error_msg))
                    }
                },
                then,
            );
        }
    }

    fn follow(&mut self, token: Token, path: Vec<Pose>) {
        let client = self.clients.follow.clone();
        let goal = FollowPath_Goal {
            path: Path {
                header: Header {
                    stamp: self.stamp(),
                    frame_id: self.frames.map.clone(),
                },
                poses: self.stamped(&path),
            },
            controller_id: String::new(),
            goal_checker_id: String::new(),
            progress_checker_id: String::new(),
        };
        let me = self.me.clone();
        let logger = self.logger.clone();
        self.spawn(
            async move {
                if !client.server_is_available().unwrap_or(false) {
                    return Err((FollowError::Unknown, "no follower serves FollowPath".into()));
                }
                // The goal's feedback counts from the moment it is asked for, not only once
                // the follower's answer has come (TODO 240: call.rs).
                let requested = client.request_goal(goal);
                let id = *requested.goal_id();
                let feedback = stream::unfold(client.receive_feedback(id), |mut f| async move {
                    f.recv().await.map(|_| ((), f))
                });
                let answer = async move { requested.await.map(|g| g.result) };
                let by_id = || client.receive_result(id);
                let heard = || {
                    if let Some(me) = &me {
                        drop(me.run(|c: &mut Core| {
                            let now = c.now();
                            c.nav.follower_heard(now);
                        }));
                    }
                };
                match follow_call(answer, feedback, by_id, heard).await {
                    Called::Refused => {
                        Err((FollowError::Unknown, "the follower refused the path".into()))
                    }
                    Called::Done {
                        result: (status, result),
                        answered,
                    } => {
                        if !answered {
                            log_warn!(
                                &logger,
                                "the follower's answer to a path never came; its feedback \
                                 showed it was driving it (TODO 240)"
                            );
                        }
                        match status {
                            GoalStatusCode::Succeeded => Ok(()),
                            _ => Err((
                                FollowError::from_code(result.error_code),
                                format!("{status:?}"),
                            )),
                        }
                    }
                }
            },
            move |c: &mut Core, answer: Result<(), (FollowError, String)>, now| {
                let out = c.nav.followed(token, answer, now);
                c.carry_out(out);
            },
        );
    }

    /// BackUp with no behaviour server: the follower backs along a straight line behind
    /// the car (`straight_back`), checking the costmap as it goes. No car pose: it fails,
    /// and the next recovery is another kind.
    fn back_up_following(&mut self, token: Token, distance: f64) {
        let path = self
            .car()
            .map(|car| straight_back(&car, distance, BACK_UP_SPACING));
        let client = self.clients.follow.clone();
        let goal = path.map(|poses| FollowPath_Goal {
            path: Path {
                header: Header {
                    stamp: self.stamp(),
                    frame_id: self.frames.map.clone(),
                },
                poses: self.stamped(&poses),
            },
            controller_id: String::new(),
            goal_checker_id: String::new(),
            progress_checker_id: String::new(),
        });
        self.spawn(
            async move {
                let Some(goal) = goal else {
                    return false;
                };
                if !client.server_is_available().unwrap_or(false) {
                    return false;
                }
                let Some(g) = client.request_goal(goal).await else {
                    return false;
                };
                g.result.await.0 == GoalStatusCode::Succeeded
            },
            move |c: &mut Core, ok: bool, now| {
                let out = c.nav.recovered(token, ok, now);
                c.carry_out(out);
            },
        );
    }

    /// While the path is kept, not replanned (the final approach, or a to-pose path up to
    /// `keep_path_age`), ask the planner's IsPathValid about it, as the trees do. No answer, or no server: nothing changes
    /// (the follower's own costmap checks and collision_monitor still stop the car).
    fn check_path(&mut self, now: f64, car: Option<&Pose>) {
        if self.checking {
            return;
        }
        let Some((made, poses)) = self.nav.path_to_check(now, car) else {
            return;
        };
        let request = IsPathValid_Request {
            path: Path {
                header: Header {
                    stamp: self.stamp(),
                    frame_id: self.frames.map.clone(),
                },
                poses: self.stamped(poses),
            },
        };
        let client = self.clients.is_path_valid.clone();
        self.checking = true;
        self.spawn(
            async move {
                if !client.service_is_ready().unwrap_or(false) {
                    return None;
                }
                let promise: Promise<IsPathValid_Response> = client.call(&request).ok()?;
                promise.await.ok().map(|r| r.is_valid)
            },
            move |c: &mut Core, valid: Option<bool>, _now| {
                c.checking = false;
                if let Some(valid) = valid {
                    if !valid {
                        log_warn!(
                            &c.logger,
                            "IsPathValid: the kept path is blocked: planning again"
                        );
                    }
                    c.nav.path_checked(made, valid);
                }
            },
        );
    }

    fn clear(&mut self, token: Token, local: bool, global: bool) {
        let mut clients: Vec<Client<ClearEntireCostmap>> = Vec::new();
        if local {
            clients.extend(self.clients.clear_local.iter().cloned());
        }
        if global {
            clients.extend(self.clients.clear_global.iter().cloned());
        }
        self.spawn(
            async move {
                let mut ok = true;
                for client in clients {
                    if !client.service_is_ready().unwrap_or(false) {
                        continue; // that costmap isn't running in this launch
                    }
                    match client.call(&Default::default()) {
                        Ok(p) => {
                            let p: Promise<ClearEntireCostmap_Response> = p;
                            ok &= p.await.is_ok();
                        }
                        Err(_) => ok = false,
                    }
                }
                ok
            },
            move |c: &mut Core, ok: bool, now| {
                let out = c.nav.recovered(token, ok, now);
                c.carry_out(out);
            },
        );
    }

    fn back_up(&mut self, token: Token, distance: f64, speed: f64) {
        if self.clients.backup.is_none() {
            return self.back_up_following(token, distance);
        }
        let client = self.clients.backup.clone();
        let goal = BackUp_Goal {
            target: Point {
                x: -distance.abs(),
                y: 0.0,
                z: 0.0,
            },
            speed: speed.abs() as f32,
            time_allowance: seconds(10.0 + distance.abs() / speed.abs().max(0.05)),
        };
        self.spawn(
            async move {
                let Some(client) = client else {
                    return false;
                };
                if !client.server_is_available().unwrap_or(false) {
                    return false;
                }
                let Some(g) = client.request_goal(goal).await else {
                    return false;
                };
                g.result.await.0 == GoalStatusCode::Succeeded
            },
            move |c: &mut Core, ok: bool, now| {
                let out = c.nav.recovered(token, ok, now);
                c.carry_out(out);
            },
        );
    }
}
