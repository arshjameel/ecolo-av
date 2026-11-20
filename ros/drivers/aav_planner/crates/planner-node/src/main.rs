//! planner: our own path planner (Nav2's State Lattice, or its Hybrid-A*, in Rust), in
//! Nav2's planner_server's place (the only planner since 17 Nov).
//!
//! Action      drivers/nav/planner/compute_path_to_pose
//!                                         nav2_msgs/action/ComputePathToPose: one path from
//!                                         the car (or the goal's start, use_start) to the
//!                                         goal, in map. navigation.launch.py planner:=own
//!                                         points the navigator's compute_path_to_pose here
//!             drivers/nav/planner/compute_path_through_poses
//!                                         nav2_msgs/action/ComputePathThroughPoses: one path
//!                                         through each goal in turn (a plan a leg, joined, as
//!                                         Nav2's planner_server did); our navigator's
//!                                         (aav_navigator) is pointed here
//! Subscribes  costmap_topic               nav2_msgs/Costmap: the global costmap the plan is
//!                                         made on: Nav2's /global_costmap/costmap_raw
//!                                         (latched, with costmap_updates_topic's
//!                                         nav2_msgs/CostmapUpdate patches), or ours,
//!                                         /drivers/nav/costmap/global_raw (costmap:=own).
//!                                         Older than costmap_timeout: goals fail, TIMEOUT
//!             /tf                         tf2_msgs/TFMessage: map -> odom -> base_link, the
//!                                         car's pose in map (when use_start is false).
//!                                         Either older than TRANSFORM_TIMEOUT: goals
//!                                         fail, TF_ERROR (the EKFs send at 30 Hz)
//! Publishes   /plan                       nav_msgs/Path: each path found
//!             drivers/nav/planner/status  diagnostic_msgs/DiagnosticStatus, one a plan: the
//!                                         times, iterations, length, cusps, error; with
//!                                         a lane map, "how": lane, pass (and pass_m:
//!                                         how far left round a parked obstacle), or
//!                                         lattice and why;
//!                                         "cut": the goal was off the costmap
//!             drivers/nav/planner/route   nav_msgs/Path: the route level's whole way to the
//!                                         goal with each plan (the lane line, or the
//!                                         start and the goals off it), for a costmap
//!                                         made of chunks (costmap_chunks:=, TODO 175)
//! Serves      is_path_valid_service (empty: none; navigation.launch.py gives
//!             /is_path_valid)
//!                                         nav2_msgs/srv/IsPathValid, as planner_server's:
//!                                         from the pose nearest the car on, a pose on a
//!                                         lethal or inscribed cell is invalid. No costmap,
//!                                         a stale one, or a path not in map: invalid
//!                                         (the trees then plan again)
//! Calls       nothing.
//! Parameters  settings (JSON: aav_description's vehicle.planner_params, from the tuned
//!             nav2.yaml's Lattice block, or with "search": "hybrid" (planner_search:=hybrid)
//!             its GridBased Hybrid-A* block; vehicle.yaml's outline and the global
//!             costmap's inflation), costmap_topic, costmap_updates_topic (empty: none),
//!             costmap_latched, map_frame, odom_frame, base_frame, is_path_valid_service.
//!             With "lanes" in settings (navigation.launch.py lane_route:=, path:=lane), the
//!             lane-centre planner (lattice_planner::lane): the right lane's centre along
//!             the lane map, the lattice joining the car and the goal to it; off the map,
//!             the lattice alone. "lanes"."pass": false (lane_pass:=false): a blocked
//!             lane line is the lattice's, not passed. "lanes"."corners": "clothoid"
//!             (lane_corners:=clothoid): the lane line's corners as clothoid, arc,
//!             clothoid (TODO 211, a challenger); else circle arcs.
//!
//! A goal off the costmap (a global costmap made of the chunks round the car): the path
//! goes as far as the way there stays on it (lattice_planner::steps), and the navigator's
//! next replan goes on from there.
//!
//! The maths is the `lattice-planner` crate; this file only moves messages. Each goal is
//! planned on its own thread, on the newest costmap.

use std::any::Any;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{anyhow, bail, Context as _, Result};
use costmap::{Arrival, Grid};
use futures::channel::oneshot;
use lanelet_map::{LaneletMap, Origin};
use lattice_planner::jerk_pass::JerkSettings;
use lattice_planner::lane::{roads_from_json, How, LaneRouter, LaneSettings, CLOTHOID_TRANSITION};
use lattice_planner::steps;
use lattice_planner::{cusps, length, planner_from_json, Plan, PlanError, Pose};
use rclrs::*;
use ros_env::builtin_interfaces::msg::{Duration as DurationMsg, Time};
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{Point, Pose as PoseMsg, PoseArray, PoseStamped, Quaternion};
use ros_env::nav2_msgs::action::{
    ComputePathThroughPoses, ComputePathThroughPoses_Result as ThroughAnswer, ComputePathToPose,
    ComputePathToPose_Result as Answer,
};
use ros_env::nav2_msgs::msg::{Costmap as CostmapMsg, CostmapUpdate};
use ros_env::nav2_msgs::srv::{IsPathValid, IsPathValid_Request, IsPathValid_Response};
use ros_env::nav_msgs::msg::Path;
use ros_env::std_msgs::msg::Header;
use ros_env::tf2_msgs::msg::TFMessage;
use serde_json::Value;

/// s: a /tf transform older than this is no pose (robot_localization sends at 30 Hz).
const TRANSFORM_TIMEOUT: f64 = 1.0;

const ACTION: &str = "/drivers/nav/planner/compute_path_to_pose";
const THROUGH_ACTION: &str = "/drivers/nav/planner/compute_path_through_poses";
const PLAN_TOPIC: &str = "/plan";
const STATUS_TOPIC: &str = "/drivers/nav/planner/status";
const ROUTE_TOPIC: &str = "/drivers/nav/planner/route";
const STOPS_TOPIC: &str = "/drivers/nav/planner/route_stops";

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("planner")?;

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
    let settings_json = text_param("settings", "")?;
    let costmap_topic = text_param("costmap_topic", "/global_costmap/costmap_raw")?;
    let updates_topic = text_param("costmap_updates_topic", "")?;
    let latched = text_param("costmap_latched", "true")? == "true";
    let valid_service = text_param("is_path_valid_service", "")?;
    let frames = Frames {
        map: text_param("map_frame", "map")?,
        odom: text_param("odom_frame", "odom")?,
        base: text_param("base_frame", "base_link")?,
    };
    if settings_json.trim().is_empty() {
        bail!("the settings parameter is empty: navigation.launch.py planner:=own sets it");
    }
    let spec: Value = serde_json::from_str(&settings_json).context("settings: JSON")?;
    let planner = Arc::new(planner_from_json(&spec)?);
    // s: a costmap older than this is not planned on (planner_server's
    // costmap_update_timeout, plus the global costmap's publish period).
    let costmap_timeout = match &spec["costmap_timeout"] {
        Value::Null => 2.0,
        v => v
            .as_f64()
            .ok_or_else(|| anyhow!("costmap_timeout: a number is needed"))?,
    };
    let planner_through = Arc::clone(&planner);
    // path:=lane: the lane-centre planner, the lattice joining its ends.
    let lanes = match &spec["lanes"] {
        Value::Null => None,
        block => {
            // lane_pass:=false: a blocked lane line is the lattice's, as before 16 Nov.
            let settings = LaneSettings {
                pass: if block["pass"].as_bool().unwrap_or(true) {
                    LaneSettings::default().pass
                } else {
                    None
                },
                // lane_pass_shape:=jerk: the pass a jerk-limited QP at the car's top speed
                // (TODO 214, a challenger); else the half cosine.
                pass_jerk: (block["pass_shape"].as_str() == Some("jerk")).then(|| {
                    (
                        JerkSettings::default(),
                        block["pass_speed"].as_f64().unwrap_or(1.0),
                    )
                }),
                // lane_corners:=clothoid: clothoid, arc, clothoid (TODO 211, a
                // challenger); else circle arcs.
                transition: if block["corners"].as_str() == Some("clothoid") {
                    CLOTHOID_TRANSITION
                } else {
                    0.0
                },
                ..LaneSettings::default()
            };
            // lane_graph:=lanelet2: {"lanelet2": "<map.osm>"}; else lanes.yaml's roads.
            Some(Arc::new(match block["lanelet2"].as_str() {
                Some(osm) => {
                    let text =
                        std::fs::read_to_string(osm).with_context(|| format!("lanes: {osm}"))?;
                    let map = LaneletMap::from_osm(&text, Origin::default())
                        .map_err(|e| anyhow!("lanes: {osm}: {e}"))?;
                    LaneRouter::from_lanelets(map, settings)
                }
                None => LaneRouter::new(
                    roads_from_json(block).map_err(|e| anyhow!("lanes: {e}"))?,
                    settings,
                ),
            }))
        }
    };
    let lanes_through = lanes.clone();
    log_info!(
        node.logger(),
        "{} (ours){}; costmap {costmap_topic}",
        planner.describe(),
        lanes.as_ref().map_or(String::new(), |l| format!(
            "; along the lane map's lanes ({}), the lattice off it",
            l.describe()
        ))
    );

    let core = Core {
        frames,
        grid: None,
        costmap_heard: Arrival::new(),
        costmap_timeout,
        map_odom: None,
        odom_base: None,
        plan_pub: node.create_publisher(PLAN_TOPIC)?,
        route_pub: node.create_publisher(ROUTE_TOPIC)?,
        stops_pub: node.create_publisher(STOPS_TOPIC)?,
        status: node.create_publisher(STATUS_TOPIC)?,
        clock: node.get_clock(),
        logger: node.logger().clone(),
        plans: 0,
        failures: 0,
    };
    let worker = node.create_worker(core);
    let full = if latched {
        costmap_topic
            .as_str()
            .keep_last(1)
            .reliable()
            .transient_local()
    } else {
        costmap_topic.as_str().keep_last(1).reliable()
    };
    let mut keep: Vec<Box<dyn Any>> = vec![
        Box::new(
            worker.create_subscription(full, |s: &mut Core, msg: CostmapMsg| s.on_costmap(&msg))?,
        ),
        Box::new(worker.create_subscription("/tf", |s: &mut Core, msg: TFMessage| s.on_tf(&msg))?),
    ];
    if !valid_service.is_empty() {
        keep.push(Box::new(worker.create_service::<IsPathValid, _>(
            valid_service.as_str(),
            |s: &mut Core, req: IsPathValid_Request| s.is_path_valid(&req.path),
        )?));
    }
    if !updates_topic.is_empty() {
        keep.push(Box::new(
            worker.create_subscription(
                updates_topic
                    .as_str()
                    .keep_last(10)
                    .reliable()
                    .transient_local(),
                |s: &mut Core, msg: CostmapUpdate| s.on_update(&msg),
            )?,
        ));
    }

    let goals = worker.clone();
    let _server = node.create_action_server::<ComputePathToPose, _>(
        ACTION,
        move |requested: RequestedGoal<ComputePathToPose>| {
            let worker = goals.clone();
            let planner = Arc::clone(&planner);
            let lanes = lanes.clone();
            async move {
                let goal = requested.accept().execute();
                let asked = goal.goal().clone();
                // The newest costmap and the car's pose, from the worker.
                let inputs = worker
                    .run(move |s: &mut Core| {
                        s.inputs(
                            std::slice::from_ref(&asked.goal),
                            asked.use_start,
                            &asked.start,
                        )
                    })
                    .await
                    .unwrap_or_else(|_| Err((Answer::UNKNOWN, "the node is stopping".into())));
                let (grid, start, goal_poses, stamp) = match inputs {
                    Ok(i) => i,
                    Err((code, why)) => {
                        let _ = worker.run(move |s: &mut Core| s.failed(code, &why)).await;
                        return goal.aborted_with(answer_error(code, "see the status"));
                    }
                };
                let stop = Arc::new(AtomicBool::new(false));
                let stop_thread = Arc::clone(&stop);
                let (done, finished) = oneshot::channel();
                std::thread::spawn(move || {
                    let stop = || stop_thread.load(Ordering::Relaxed);
                    let goal = goal_poses[0];
                    let r = match &lanes {
                        Some(l) => l
                            .plan(&planner, &grid, &start, &goal, &stop)
                            .map(|p| (p.plan, Some(p.how), p.cut, (p.route, p.stops))),
                        None => steps::plan_toward(&planner, &grid, &start, &goal, &stop)
                            .map(|(p, cut)| (p, None, cut, (vec![start, goal], Vec::new()))),
                    };
                    let _ = done.send(r);
                });
                match goal.until_cancel_requested(finished).await {
                    Ok(Ok(Ok((plan, how, cut, route)))) => {
                        let reply = worker
                            .run(move |s: &mut Core| {
                                s.succeeded(&plan, how, cut, &route.0, &route.1, stamp)
                            })
                            .await;
                        match reply {
                            Ok(a) => goal.succeeded_with(a),
                            Err(_) => goal.aborted_with(answer_error(Answer::UNKNOWN, "stopping")),
                        }
                    }
                    Ok(Ok(Err(e))) => {
                        let code = error_code(e);
                        let text = e.to_string();
                        let _ = worker.run(move |s: &mut Core| s.failed(code, &text)).await;
                        goal.aborted_with(answer_error(code, &e.to_string()))
                    }
                    Ok(Err(_)) => goal.aborted_with(answer_error(Answer::UNKNOWN, "lost")),
                    Err(_) => {
                        stop.store(true, Ordering::Relaxed);
                        goal.begin_cancelling()
                            .cancelled_with(answer_error(Answer::NONE, "cancelled"))
                    }
                }
            }
        },
    )?;

    let goals = worker.clone();
    let _through = node.create_action_server::<ComputePathThroughPoses, _>(
        THROUGH_ACTION,
        move |requested: RequestedGoal<ComputePathThroughPoses>| {
            let worker = goals.clone();
            let planner = Arc::clone(&planner_through);
            let lanes = lanes_through.clone();
            async move {
                let goal = requested.accept().execute();
                let asked = goal.goal().clone();
                if asked.goals.is_empty() {
                    return goal.aborted_with(through_error(
                        ThroughAnswer::NO_VIAPOINTS_GIVEN,
                        "no goals given",
                    ));
                }
                let inputs = worker
                    .run(move |s: &mut Core| s.inputs(&asked.goals, asked.use_start, &asked.start))
                    .await
                    .unwrap_or_else(|_| Err((Answer::UNKNOWN, "the node is stopping".into())));
                let (grid, start, goal_poses, stamp) = match inputs {
                    Ok(i) => i,
                    Err((code, why)) => {
                        let _ = worker.run(move |s: &mut Core| s.failed(code, &why)).await;
                        return goal.aborted_with(through_error(code + 100, "see the status"));
                    }
                };
                let stop = Arc::new(AtomicBool::new(false));
                let stop_thread = Arc::clone(&stop);
                let (done, finished) = oneshot::channel();
                std::thread::spawn(move || {
                    let stop = || stop_thread.load(Ordering::Relaxed);
                    let r = match &lanes {
                        Some(l) => l
                            .plan_through(&planner, &grid, &start, &goal_poses, &stop)
                            .map(|p| (p.plan, Some(p.how), p.cut, (p.route, p.stops))),
                        None => planner
                            .plan_through(&grid, &start, &goal_poses, &stop)
                            .map(|p| {
                                let route = std::iter::once(start).chain(goal_poses).collect();
                                (p, None, false, (route, Vec::new()))
                            }),
                    };
                    let _ = done.send(r);
                });
                match goal.until_cancel_requested(finished).await {
                    Ok(Ok(Ok((plan, how, cut, route)))) => {
                        let reply = worker
                            .run(move |s: &mut Core| {
                                s.succeeded(&plan, how, cut, &route.0, &route.1, stamp)
                            })
                            .await;
                        match reply {
                            Ok(a) => goal.succeeded_with(ThroughAnswer {
                                path: a.path,
                                planning_time: a.planning_time,
                                error_code: ThroughAnswer::NONE,
                                error_msg: String::new(),
                            }),
                            Err(_) => {
                                goal.aborted_with(through_error(ThroughAnswer::UNKNOWN, "stopping"))
                            }
                        }
                    }
                    Ok(Ok(Err((leg, e)))) => {
                        // Nav2's through-poses codes are its to-pose ones plus 100.
                        let code = error_code(e);
                        let text = format!("leg {}: {e}", leg + 1);
                        let reply = text.clone();
                        let _ = worker.run(move |s: &mut Core| s.failed(code, &text)).await;
                        let through = if code == Answer::NONE {
                            code
                        } else {
                            code + 100
                        };
                        goal.aborted_with(through_error(through, &reply))
                    }
                    Ok(Err(_)) => goal.aborted_with(through_error(ThroughAnswer::UNKNOWN, "lost")),
                    Err(_) => {
                        stop.store(true, Ordering::Relaxed);
                        goal.begin_cancelling()
                            .cancelled_with(through_error(ThroughAnswer::NONE, "cancelled"))
                    }
                }
            }
        },
    )?;

    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    drop((keep, handles));
    Ok(())
}

fn error_code(e: PlanError) -> u16 {
    match e {
        PlanError::StartOutsideMap => Answer::START_OUTSIDE_MAP,
        PlanError::GoalOutsideMap => Answer::GOAL_OUTSIDE_MAP,
        PlanError::StartOccupied => Answer::START_OCCUPIED,
        PlanError::GoalOccupied => Answer::GOAL_OCCUPIED,
        PlanError::Timeout => Answer::TIMEOUT,
        PlanError::NoValidPath => Answer::NO_VALID_PATH,
        PlanError::Cancelled => Answer::NONE,
        PlanError::WrongResolution => Answer::UNKNOWN,
    }
}

fn through_error(code: u16, why: &str) -> ThroughAnswer {
    ThroughAnswer {
        error_code: code,
        error_msg: why.into(),
        ..Default::default()
    }
}

fn answer_error(code: u16, why: &str) -> Answer {
    Answer {
        error_code: code,
        error_msg: why.into(),
        ..Default::default()
    }
}

struct Frames {
    map: String,
    odom: String,
    base: String,
}

/// The worker's payload.
struct Core {
    frames: Frames,
    /// The newest costmap, shared with the planning threads.
    grid: Option<Arc<Grid>>,
    /// When the costmap (or a patch to it) last arrived.
    costmap_heard: Arrival,
    /// s: older than this, a goal fails with TIMEOUT (Nav2's PlannerTimedOut).
    costmap_timeout: f64,
    /// map -> odom and odom -> base_link, each with when it arrived (s).
    map_odom: Option<(Pose, f64)>,
    odom_base: Option<(Pose, f64)>,
    plan_pub: Publisher<Path>,
    route_pub: Publisher<Path>,
    stops_pub: Publisher<PoseArray>,
    status: Publisher<DiagnosticStatus>,
    clock: Clock,
    logger: Logger,
    plans: u64,
    failures: u64,
}

/// Poses as a Path's, each with `header`.
fn stamped(header: &Header, poses: &[Pose]) -> Vec<PoseStamped> {
    poses
        .iter()
        .map(|p| PoseStamped {
            header: header.clone(),
            pose: PoseMsg {
                position: ros_env::geometry_msgs::msg::Point {
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
            },
        })
        .collect()
}

fn yaw_of(q: &Quaternion) -> f64 {
    (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
}

fn pose_of(p: &PoseMsg) -> Pose {
    Pose::new(p.position.x, p.position.y, yaw_of(&p.orientation))
}

/// b placed in a's frame: a ∘ b.
fn compose(a: &Pose, b: &Pose) -> Pose {
    let (s, c) = a.yaw.sin_cos();
    Pose::new(
        a.x + c * b.x - s * b.y,
        a.y + s * b.x + c * b.y,
        lattice_planner::curves::wrap(a.yaw + b.yaw),
    )
}

type Inputs = (Arc<Grid>, Pose, Vec<Pose>, Time);

impl Core {
    fn on_costmap(&mut self, msg: &CostmapMsg) {
        let m = &msg.metadata;
        if msg.header.frame_id.trim_start_matches('/') != self.frames.map {
            log_warn!(
                self.logger.throttle(Duration::from_secs(5)),
                "the costmap is in {}, not {}: ignored",
                msg.header.frame_id,
                self.frames.map
            );
            return;
        }
        let (w, h) = (m.size_x as usize, m.size_y as usize);
        if msg.data.len() != w * h {
            return;
        }
        let mut grid = Grid::new(
            (m.origin.position.x, m.origin.position.y),
            f64::from(m.resolution),
            w,
            h,
            0,
        );
        grid.data.clone_from(&msg.data);
        self.grid = Some(Arc::new(grid));
        self.costmap_heard.mark(self.now());
    }

    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    /// Nav2's partial update: a rectangle of new costs.
    fn on_update(&mut self, msg: &CostmapUpdate) {
        let Some(grid) = self.grid.as_mut() else {
            return;
        };
        let (x0, y0) = (msg.x as usize, msg.y as usize);
        let (w, h) = (msg.size_x as usize, msg.size_y as usize);
        if x0 + w > grid.width || y0 + h > grid.height || msg.data.len() != w * h {
            return;
        }
        // Copy on write: a planning thread may still hold the old one.
        let grid = Arc::make_mut(grid);
        for row in 0..h {
            let at = (y0 + row) * grid.width + x0;
            grid.data[at..at + w].copy_from_slice(&msg.data[row * w..(row + 1) * w]);
        }
        self.costmap_heard.mark(self.now());
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

    /// Nav2's IsPathValid: invalid from the pose nearest the car on, where a pose sits on
    /// a lethal or inscribed cell (`Grid::blocked_along`).
    fn is_path_valid(&self, path: &Path) -> IsPathValid_Response {
        let invalid = |indices: Vec<i32>| IsPathValid_Response {
            is_valid: false,
            invalid_pose_indices: indices,
        };
        let frame = path.header.frame_id.trim_start_matches('/');
        let now = self.now();
        let grid = match &self.grid {
            Some(g) if self.costmap_heard.fresh(now, self.costmap_timeout) => g,
            _ => return invalid(Vec::new()),
        };
        if !(frame.is_empty() || frame == self.frames.map) {
            return invalid(Vec::new());
        }
        let points: Vec<(f64, f64)> = path
            .poses
            .iter()
            .map(|p| (p.pose.position.x, p.pose.position.y))
            .collect();
        let car = match (&self.map_odom, &self.odom_base) {
            (Some((a, _)), Some((b, _))) => Some(compose(a, b)),
            _ => None,
        };
        let from = car.map_or(0, |c| {
            points
                .iter()
                .enumerate()
                .min_by(|(_, p), (_, q)| {
                    let d = |(x, y): &(f64, f64)| (x - c.x).hypot(y - c.y);
                    d(p).total_cmp(&d(q))
                })
                .map_or(0, |(i, _)| i)
        });
        let blocked = grid.blocked_along(&points, from);
        if blocked.is_empty() {
            IsPathValid_Response {
                is_valid: true,
                invalid_pose_indices: Vec::new(),
            }
        } else {
            invalid(
                blocked
                    .into_iter()
                    .map(|i| i32::try_from(i).unwrap_or(i32::MAX))
                    .collect(),
            )
        }
    }

    /// What a goal needs: the costmap, the start, the goals, and the time now.
    fn inputs(
        &self,
        goals: &[PoseStamped],
        use_start: bool,
        start: &PoseStamped,
    ) -> Result<Inputs, (u16, String)> {
        let grid = self
            .grid
            .clone()
            .ok_or((Answer::UNKNOWN, "no costmap yet".to_string()))?;
        // Nav2's planner waits costmap_update_timeout for a current costmap, then fails
        // with TIMEOUT (207; 307 through poses). Ours doesn't plan on a stale one.
        let now = self.now();
        if !self.costmap_heard.fresh(now, self.costmap_timeout) {
            let age = self.costmap_heard.age(now).unwrap_or(f64::INFINITY);
            return Err((
                Answer::TIMEOUT,
                format!(
                    "the costmap is {age:.1} s old (limit {:.1} s)",
                    self.costmap_timeout
                ),
            ));
        }
        let frame_ok = |f: &str| {
            let f = f.trim_start_matches('/');
            f.is_empty() || f == self.frames.map
        };
        if let Some(wrong) = goals.iter().find(|g| !frame_ok(&g.header.frame_id)) {
            return Err((
                Answer::TF_ERROR,
                format!(
                    "a goal is in {}, not {}",
                    wrong.header.frame_id, self.frames.map
                ),
            ));
        }
        let start = if use_start {
            if !frame_ok(&start.header.frame_id) {
                return Err((Answer::TF_ERROR, "the start is not in the map frame".into()));
            }
            pose_of(&start.pose)
        } else {
            match (&self.map_odom, &self.odom_base) {
                (Some((a, a_at)), Some((b, b_at))) => {
                    let age = now - a_at.min(*b_at);
                    if age > TRANSFORM_TIMEOUT {
                        return Err((
                            Answer::TF_ERROR,
                            format!(
                                "map -> odom -> base_link is {age:.1} s old \
                                 (limit {TRANSFORM_TIMEOUT:.1} s)"
                            ),
                        ));
                    }
                    compose(a, b)
                }
                _ => {
                    return Err((
                        Answer::TF_ERROR,
                        "no map -> odom -> base_link on /tf yet".into(),
                    ))
                }
            }
        };
        let stamp = self.clock.now().to_ros_msg().unwrap_or_default();
        let goals = goals.iter().map(|g| pose_of(&g.pose)).collect();
        Ok((grid, start, goals, stamp))
    }

    /// `how`: with a lane map, the lane or the lattice and why (the status's "how").
    /// `cut`: the goal was off the costmap; `route`: the whole way, on its own topic.
    fn succeeded(
        &mut self,
        plan: &Plan,
        how: Option<How>,
        cut: bool,
        route: &[Pose],
        stops: &[(f64, f64)],
        stamp: Time,
    ) -> Answer {
        self.plans += 1;
        let header = Header {
            stamp,
            frame_id: self.frames.map.clone(),
        };
        let route = Path {
            header: header.clone(),
            poses: stamped(&header, route),
        };
        if let Err(e) = self.route_pub.publish(&route) {
            log_error!(&self.logger, "{ROUTE_TOPIC}: {e}");
        }
        if how.is_some() {
            let stops = PoseArray {
                header: header.clone(),
                poses: stops
                    .iter()
                    .map(|&(x, y)| PoseMsg {
                        position: Point { x, y, z: 0.0 },
                        ..PoseMsg::default()
                    })
                    .collect(),
            };
            if let Err(e) = self.stops_pub.publish(&stops) {
                log_error!(&self.logger, "{STOPS_TOPIC}: {e}");
            }
        }
        let path = Path {
            header: header.clone(),
            poses: stamped(&header, &plan.poses),
        };
        if let Err(e) = self.plan_pub.publish(&path) {
            log_error!(&self.logger, "/plan: {e}");
        }
        let kv = |k: &str, v: String| KeyValue {
            key: k.into(),
            value: v,
        };
        let mut values = vec![
            kv("search_ms", format!("{:.2}", plan.search_time * 1e3)),
            kv("total_ms", format!("{:.2}", plan.total_time * 1e3)),
            kv("iterations", plan.iterations.to_string()),
            kv("length_m", format!("{:.2}", length(&plan.poses))),
            kv("cusps", cusps(&plan.poses).to_string()),
            kv("poses", plan.poses.len().to_string()),
            kv("plans", self.plans.to_string()),
            kv("failures", self.failures.to_string()),
            kv("cut", cut.to_string()),
        ];
        if how.is_some() {
            values.push(kv("stops", stops.len().to_string()));
        }
        match how {
            Some(How::Lane(m)) => {
                values.push(kv("how", "lane".into()));
                values.push(kv("lane_m", format!("{m:.1}")));
            }
            Some(How::Pass(m, d)) => {
                values.push(kv("how", "pass".into()));
                values.push(kv("lane_m", format!("{m:.1}")));
                values.push(kv("pass_m", format!("{d:.2}")));
            }
            Some(How::Lattice(why)) => values.push(kv("how", format!("lattice: {}", why.word()))),
            None => {}
        }
        let status = DiagnosticStatus {
            level: DiagnosticStatus::OK,
            name: "planner".into(),
            message: "path".into(),
            hardware_id: String::new(),
            values,
        };
        let _ = self.status.publish(&status);
        let secs = plan.total_time;
        Answer {
            path,
            planning_time: DurationMsg {
                sec: secs.floor() as i32,
                nanosec: ((secs - secs.floor()) * 1e9) as u32,
            },
            error_code: Answer::NONE,
            error_msg: String::new(),
        }
    }

    fn failed(&mut self, code: u16, why: &str) {
        self.failures += 1;
        log_warn!(
            self.logger.throttle(Duration::from_secs(2)),
            "no path ({code}): {why}"
        );
        let status = DiagnosticStatus {
            level: DiagnosticStatus::ERROR,
            name: "planner".into(),
            message: why.into(),
            hardware_id: String::new(),
            values: vec![
                KeyValue {
                    key: "error_code".into(),
                    value: code.to_string(),
                },
                KeyValue {
                    key: "failures".into(),
                    value: self.failures.to_string(),
                },
            ],
        };
        let _ = self.status.publish(&status);
    }
}
