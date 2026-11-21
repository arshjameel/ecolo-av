//! mppi_lag: MPPI that knows the actuators' lag, in Nav2's controller_server's FollowPath place.
//!
//! Action      follow_path                 nav2_msgs/action/FollowPath: the navigator's, a new
//!                                         goal each time the planner replans (the newest
//!                                         replaces the last without a stop)
//! Subscribes  localization/odom/local     nav_msgs/Odometry (odom_topic): the pose in odom
//!                                         and the speed
//!             /tf                         tf2_msgs/TFMessage: map -> odom. Held, and moved
//!                                         toward the newest at most map_correction_speed m/s
//!                                         and map_correction_turn_rate rad/s (TODO 186:
//!                                         follow in odom, correct slowly); each new path
//!                                         comes into odom by the held one, and the running
//!                                         path moves with it. map_correction false (the
//!                                         launch's map_correction:=newest) or rates 0: the
//!                                         newest, once a path
//!             vehicle/steering, vehicle/throttle, vehicle/brake
//!                                         aav_interfaces/ActuatorState: the measured levels,
//!                                         while fresh (measured_timeout); else the model's own
//!             local_costmap/costmap_raw   nav2_msgs/Costmap (costmap_topic): for CostCritic
//!                                         and the path's clear points (use_costmap); older
//!                                         than costmap_timeout: zero, and the path fails
//!             /world/road_ahead            aav_interfaces/RoadAhead (road_ahead_topic; empty: off), map:
//!                                         the lane's line and edges, moved into odom by the
//!                                         held map -> odom, for LaneCritic while fresh
//!                                         (road_ahead_timeout); NaN edges or stale: no lane.
//!                                         State WAIT, reason moving (the behaviour holds
//!                                         the car for something moving near): the
//!                                         progress check waits
//! Publishes   cmd_vel_nav                 geometry_msgs/TwistStamped, at rate_hz while a path
//!                                         runs: (v, v tan(steer) / wheelbase), to Nav2's
//!                                         velocity smoother, collision_monitor, then
//!                                         /drivers/nav/cmd_vel
//!             drivers/nav/track/status    diagnostic_msgs/DiagnosticStatus, every tick: the
//!                                         same keys as aav_track's, plus the model and MPPI's
//!             drivers/nav/local_plan      nav_msgs/Path, odom, every tick: the poses MPPI's
//!                                         chosen sequence leads to (local_plan_topic), the
//!                                         screens' faint line (aav_ego look_ahead)
//!             drivers/nav/lag_aware       std_msgs/Bool, reliable, transient local, depth 1:
//!                                         true when a path starts with model lag (ego's lead
//!                                         must stay off while this drives), false when it
//!                                         ends and at start and exit (always false with model
//!                                         ideal). Named for the twist_mux slot it drives
//!                                         through: nav
//! Serves      nothing else; calls nothing.
//! Parameters  read once at start (config/mppi_lag.yaml, and navigation.launch.py's from
//!             nav2.yaml, vehicle.yaml and ego.yaml): see the README. speed_loop: the
//!             spine's speed loop profile the car runs (default, smooth, firm: the lag
//!             model's PI; lqi: the shared car model), as sim.launch.py speed_loop:=
//!             (TODO 163); an unknown name is refused.
//!
//! navigation.launch.py tracker:=mppi_lag (the default) starts it. With reversing:=true it reverses as Nav2's MPPI does (vx_min
//! below 0, enforce_path_inversion: it stops at each cusp, then follows the next stretch). Everything after cmd_vel_nav is unchanged. The maths is the
//! `mppi-lag` crate; this file only moves messages.

use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use futures::channel::oneshot;
use mppi_lag::car_model::SpeedLoopProfile;
use mppi_lag::map_correction::{into_map, into_odom, Correction};
use mppi_lag::{
    critics::{inflation_cost, Critic},
    lag::Actuators,
    Arrival, Costmap, Edges, Ego, Follower, FollowerSettings, GlobalPath, IdealModel, Inversion,
    LagModel, Lane, LanePoint, MapCorrection, Observation, Outcome, PathAngleMode, Pose, Report,
    Settings, SharedModel, SlewLimits, SpeedLoop, Tick,
};
use rclrs::*;
use ros_env::aav_interfaces::msg::{ActuatorState, RoadAhead};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{
    Point, Pose as PoseMsg, PoseStamped, Quaternion, Twist, TwistStamped, Vector3,
};
use ros_env::nav2_msgs::action::{FollowPath, FollowPath_Feedback, FollowPath_Result};
use ros_env::nav2_msgs::msg::Costmap as CostmapMsg;
use ros_env::nav_msgs::msg::{Odometry, Path};
use ros_env::std_msgs::msg::{Bool, Header};
use ros_env::tf2_msgs::msg::TFMessage;

mod trace;

/// The name on the status topic, as aav_track's trackers have theirs.
const TRACKER: &str = "mppi_lag";

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("mppi_lag")?;
    if let Some(to) = trace::start() {
        log_info!(node.logger(), "AAV_TRACE: the tick's stages timed, to {to}");
    }

    let (config, _params) = Config::declare(&node)?;
    let follower = config.follower()?;
    let lag_aware = follower.lag_aware();

    // Latched: true while this drives with the lag model, so ego's lead doesn't cancel the
    // lag twice; false otherwise (behavior_server's moves share the nav slot).
    let lag_pub = node.create_publisher::<Bool>(
        config
            .lag_aware_topic
            .as_str()
            .keep_last(1)
            .reliable()
            .transient_local(),
    )?;
    lag_pub.publish(Bool { data: false })?;
    let lag_pub_at_exit = lag_pub.clone();

    let core = Core {
        lag_pub,
        lag_on: false,
        cmd: node.create_publisher(config.cmd_topic.as_str())?,
        status: node.create_publisher(config.status_topic.as_str())?,
        local_plan: node.create_publisher(config.local_plan_topic.as_str())?,
        clock: node.get_clock(),
        logger: node.logger().clone(),
        follower,
        odom: None,
        steering: None,
        throttle: None,
        brake: None,
        map_to_odom: None,
        correction: MapCorrection::new(config.map_correction),
        corrected: None,
        costmap: None,
        costmap_heard: Arrival::new(),
        road_ahead: None,
        lane_known: false,
        moving_hold: None,
        job: None,
        next_id: 0,
        last_tick: None,
        missed: 0,
        loop_max: 0.0,
        config: config.clone(),
    };
    log_info!(
        node.logger(),
        "MPPI with the {} model (speed loop {}) follows Nav2's paths at {} Hz, up to {} m/s; \
         {} samples x {} steps; lag_aware {lag_aware}",
        config.model,
        config.speed_loop.name(),
        config.rate_hz,
        config.mppi.vx_max,
        config.mppi.batch_size,
        config.mppi.time_steps
    );

    let worker = node.create_worker(core);
    let _odom = worker
        .create_subscription(config.odom_topic.as_str(), |s: &mut Core, msg: Odometry| {
            s.on_odom(&msg)
        })?;
    let _tf = worker.create_subscription("/tf", |s: &mut Core, msg: TFMessage| s.on_tf(&msg))?;
    let _steering = worker
        .create_subscription("vehicle/steering", |s: &mut Core, msg: ActuatorState| {
            s.steering = s.measured(&msg)
        })?;
    let _throttle = worker
        .create_subscription("vehicle/throttle", |s: &mut Core, msg: ActuatorState| {
            s.throttle = s.measured(&msg)
        })?;
    let _brake = worker
        .create_subscription("vehicle/brake", |s: &mut Core, msg: ActuatorState| {
            s.brake = s.measured(&msg)
        })?;
    let _costmap = if config.use_costmap {
        Some(worker.create_subscription(
            config.costmap_topic.as_str(),
            |s: &mut Core, msg: CostmapMsg| s.on_costmap(&msg),
        )?)
    } else {
        None
    };
    // The road ahead's lane, for LaneCritic (navigation.launch.py names the topic only with
    // behaviour:=true and a lane map).
    let _road_ahead = if config.road_ahead_topic.is_empty() {
        None
    } else {
        Some(worker.create_subscription(
            config.road_ahead_topic.as_str(),
            |s: &mut Core, msg: RoadAhead| s.on_road_ahead(&msg),
        )?)
    };
    let period = Duration::from_secs_f64(1.0 / config.rate_hz);
    let _tick = worker.create_timer_repeating(period.node_time(), |s: &mut Core| s.tick())?;

    let goals = worker.clone();
    let _server = node.create_action_server::<FollowPath, _>(
        "follow_path",
        move |requested: RequestedGoal<FollowPath>| {
            let worker = goals.clone();
            async move {
                let goal = requested.accept().execute();
                let path = goal.goal().path.clone();
                let feedback = goal.feedback_publisher();
                let (done, finished) = oneshot::channel();
                let id = worker
                    .run(move |s: &mut Core| s.start(&path, done, feedback))
                    .await
                    .unwrap_or(u64::MAX);
                let result = |code: u16| FollowPath_Result {
                    error_code: code,
                    ..Default::default()
                };
                match goal.until_cancel_requested(finished).await {
                    Ok(Ok(Finish::Succeeded)) => {
                        goal.succeeded_with(result(FollowPath_Result::NONE))
                    }
                    Ok(Ok(Finish::Aborted(code))) => goal.aborted_with(result(code)),
                    // Replaced by a newer path, or the node is going away.
                    Ok(Err(_)) => goal.aborted_with(result(FollowPath_Result::NONE)),
                    Err(_) => {
                        let _ = worker.run(move |s: &mut Core| s.cancel(id)).await;
                        goal.begin_cancelling()
                            .cancelled_with(result(FollowPath_Result::NONE))
                    }
                }
            }
        },
    )?;

    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    // Going away: no lag-aware driver any more.
    lag_pub_at_exit.publish(Bool { data: false })?;
    Ok(())
}

/// How a path ended, sent from the tick to the action's task.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Finish {
    Succeeded,
    Aborted(u16),
}

/// The car model the follower plans with (`model` parameter).
enum AnyFollower {
    /// The lag model: lag-aware.
    Lag(Box<Follower<LagModel>>),
    /// The lag-free model, as Nav2's MPPI: for comparison runs.
    Ideal(Box<Follower<IdealModel>>),
    /// The shared car model (lag-aware): for a speed loop the lag model can't run (the LQI).
    Shared(Box<Follower<SharedModel>>),
}

macro_rules! each {
    ($self:expr, $f:ident => $body:expr) => {
        match $self {
            AnyFollower::Lag($f) => $body,
            AnyFollower::Ideal($f) => $body,
            AnyFollower::Shared($f) => $body,
        }
    };
}

impl AnyFollower {
    fn lag_aware(&self) -> bool {
        each!(self, f => f.lag_aware())
    }
    fn follow(&mut self, path: GlobalPath, replacing: bool) {
        each!(self, f => f.follow(path, replacing))
    }
    fn stop(&mut self) {
        each!(self, f => f.stop())
    }
    fn observe(&mut self, seen: &Observation) {
        each!(self, f => f.observe(seen))
    }
    fn tick(&mut self, now: f64, costmap: Option<&Costmap>) -> Option<Tick> {
        each!(self, f => f.tick(now, costmap))
    }
    fn set_lane(&mut self, lane: Option<Lane>) {
        each!(self, f => f.set_lane(lane))
    }
    fn held(&mut self) {
        each!(self, f => f.held())
    }
}

/// One FollowPath goal.
struct Job {
    id: u64,
    done: oneshot::Sender<Finish>,
    feedback: FeedbackPublisher<FollowPath>,
    started: f64,
    /// The path came in map: it moves as the held map -> odom moves.
    in_map: bool,
}

/// The worker's payload: the follower, what it has heard, and what it publishes with.
struct Core {
    config: Config,
    /// `/drivers/nav/lag_aware`, and what it last said.
    lag_pub: Publisher<Bool>,
    lag_on: bool,
    follower: AnyFollower,
    cmd: Publisher<TwistStamped>,
    status: Publisher<DiagnosticStatus>,
    /// MPPI's chosen future, in odom: the screens' faint line.
    local_plan: Publisher<Path>,
    clock: Clock,
    logger: Logger,
    /// (pose, speed, when) from odometry.
    odom: Option<(Pose, f64, f64)>,
    /// (level, when) of each measured actuator.
    steering: Option<(f64, f64)>,
    throttle: Option<(f64, f64)>,
    brake: Option<(f64, f64)>,
    /// map -> odom: odom's pose in map, and when.
    map_to_odom: Option<(Pose, f64)>,
    /// The held map -> odom (TODO 186), slewed toward `map_to_odom`.
    correction: MapCorrection,
    /// When the held map -> odom last moved on (this node's clock).
    corrected: Option<f64>,
    costmap: Option<Costmap>,
    /// When the costmap last arrived (this node's clock).
    costmap_heard: Arrival,
    /// The road ahead's lane in its own frame (map), its frame, and when it arrived.
    road_ahead: Option<(Lane, String, f64)>,
    /// The last tick had a fresh lane (the status's `lane`).
    lane_known: bool,
    /// When the road ahead last said the behaviour holds the car for something moving
    /// (state WAIT, reason moving): the progress check waits meanwhile.
    moving_hold: Option<f64>,
    job: Option<Job>,
    next_id: u64,
    last_tick: Option<f64>,
    missed: u64,
    loop_max: f64,
}

fn yaw_of(q: &Quaternion) -> f64 {
    (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
}

fn seconds(t: &Time) -> f64 {
    f64::from(t.sec) + f64::from(t.nanosec) * 1e-9
}

impl Core {
    /// Say on `lag_aware` whether a lag-aware path is running (only when it changes).
    fn set_lag_aware(&mut self, running: bool) {
        let on = running && self.follower.lag_aware();
        if on != self.lag_on {
            self.lag_on = on;
            if let Err(e) = self.lag_pub.publish(Bool { data: on }) {
                log_error!(&self.logger, "lag_aware: {e}");
            }
        }
    }

    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn stamp(&self) -> Time {
        self.clock.now().to_ros_msg().unwrap_or_default()
    }

    fn on_odom(&mut self, msg: &Odometry) {
        let p = &msg.pose.pose;
        let pose = Pose::new(p.position.x, p.position.y, yaw_of(&p.orientation));
        let speed = msg.twist.twist.linear.x;
        if [pose.x, pose.y, pose.yaw, speed]
            .iter()
            .all(|v| v.is_finite())
        {
            self.odom = Some((pose, speed, self.now()));
        }
    }

    fn on_tf(&mut self, msg: &TFMessage) {
        let odom = self.config.odom_frame.as_str();
        for t in &msg.transforms {
            if t.header.frame_id.trim_start_matches('/') == self.config.map_frame
                && t.child_frame_id.trim_start_matches('/') == odom
            {
                let r = &t.transform;
                let pose = Pose::new(r.translation.x, r.translation.y, yaw_of(&r.rotation));
                self.map_to_odom = Some((pose, seconds(&t.header.stamp)));
            }
        }
    }

    fn measured(&self, msg: &ActuatorState) -> Option<(f64, f64)> {
        let level = f64::from(msg.measured);
        level.is_finite().then(|| (level, self.now()))
    }

    fn on_costmap(&mut self, msg: &CostmapMsg) {
        let m = &msg.metadata;
        if msg.header.frame_id.trim_start_matches('/') != self.config.odom_frame {
            log_warn!(
                self.logger.throttle(Duration::from_secs(5)),
                "the costmap is in {}, not {}: ignored",
                msg.header.frame_id,
                self.config.odom_frame
            );
            return;
        }
        self.costmap = Costmap::new(
            (m.origin.position.x, m.origin.position.y),
            f64::from(m.resolution),
            m.size_x as usize,
            m.size_y as usize,
            msg.data.clone(),
        );
        if self.costmap.is_some() {
            self.costmap_heard.mark(self.now());
        }
    }

    /// The road ahead: keep its lane (line and edges) as it came; `None` without edges.
    fn on_road_ahead(&mut self, msg: &RoadAhead) {
        let edge = |list: &[f32], i: usize| list.get(i).map_or(f64::NAN, |&v| f64::from(v));
        let points = msg
            .x
            .iter()
            .zip(&msg.y)
            .enumerate()
            .map(|(i, (&x, &y))| LanePoint {
                x: f64::from(x),
                y: f64::from(y),
                edges: Edges::from_numbers(
                    edge(&msg.left_hard, i),
                    edge(&msg.right_hard, i),
                    edge(&msg.left_soft, i),
                    edge(&msg.right_soft, i),
                ),
            });
        let frame = msg.header.frame_id.trim_start_matches('/').to_string();
        self.road_ahead = Lane::new(points).map(|lane| (lane, frame, self.now()));
        if msg.state == "WAIT" && msg.reason == "moving" {
            self.moving_hold = Some(self.now());
        }
    }

    /// The road ahead's lane in odom, if it is fresh and map -> odom is known.
    fn lane_in_odom(&self, now: f64) -> Option<Lane> {
        let (lane, frame, heard) = self.road_ahead.as_ref()?;
        if now - heard > self.config.road_ahead_timeout {
            return None;
        }
        if frame.is_empty() || *frame == self.config.odom_frame {
            return Some(lane.clone());
        }
        if *frame != self.config.map_frame {
            log_warn!(
                self.logger.throttle(Duration::from_secs(5)),
                "the road ahead is in {frame}, not {} or {}: ignored",
                self.config.map_frame,
                self.config.odom_frame
            );
            return None;
        }
        let (newest, stamp) = self.map_to_odom?;
        if now - stamp > self.config.transform_tolerance + 1.0 {
            return None;
        }
        // The held one while a map path runs, so the lane and the path move together.
        let odom_in_map = match (&self.job, self.correction.held()) {
            (Some(job), Some(held)) if job.in_map && self.config.map_correction.on() => held,
            _ => newest,
        };
        Some(lane.moved(|x, y| {
            let p = into_odom(&Pose::new(x, y, 0.0), &odom_in_map);
            (p.x, p.y)
        }))
    }

    /// A new FollowPath goal: its path into odom, then follow it. Returns the job's id.
    fn start(
        &mut self,
        msg: &Path,
        done: oneshot::Sender<Finish>,
        feedback: FeedbackPublisher<FollowPath>,
    ) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        let (path, in_map) = match self.path_in_odom(msg) {
            Ok(found) => found,
            Err((code, why)) => {
                log_warn!(&self.logger, "FollowPath refused: {why}");
                let _ = done.send(Finish::Aborted(code));
                return id;
            }
        };
        let replacing = self.job.is_some();
        if let Some(old) = self.job.take() {
            // The planner replanned: the newest path carries on, with no stop between.
            drop(old.done);
        } else {
            self.missed = 0;
            self.loop_max = 0.0;
            self.last_tick = None;
        }
        self.corrected = Some(self.now());
        self.follower.follow(path, replacing);
        self.set_lag_aware(true);
        self.job = Some(Job {
            id,
            done,
            feedback,
            started: self.now(),
            in_map,
        });
        id
    }

    /// The goal's path in odom, and whether it came in map. A map path comes by the held map
    /// -> odom while one runs (none: the newest, which is then held).
    fn path_in_odom(&mut self, msg: &Path) -> Result<(GlobalPath, bool), (u16, String)> {
        let mut poses: Vec<Pose> = msg
            .poses
            .iter()
            .map(|s| {
                let p = &s.pose;
                Pose::new(p.position.x, p.position.y, yaw_of(&p.orientation))
            })
            .collect();
        let frame = msg.header.frame_id.trim_start_matches('/');
        let in_map = !frame.is_empty() && frame != self.config.odom_frame;
        if in_map {
            if frame != self.config.map_frame {
                return Err((
                    FollowPath_Result::TF_ERROR,
                    format!(
                        "a path in {frame}: only {} or {}",
                        self.config.map_frame, self.config.odom_frame
                    ),
                ));
            }
            let Some((odom_in_map, stamp)) = self.map_to_odom else {
                return Err((FollowPath_Result::TF_ERROR, "no map -> odom yet".into()));
            };
            let age = self.now() - stamp;
            if age > self.config.transform_tolerance + 1.0 {
                return Err((
                    FollowPath_Result::TF_ERROR,
                    format!("map -> odom is {age:.1} s old"),
                ));
            }
            let driving = self.job.as_ref().is_some_and(|j| j.in_map);
            let frame = self.correction.for_new_path(odom_in_map, driving);
            poses = poses.iter().map(|p| into_odom(p, &frame)).collect();
        }
        GlobalPath::new(poses).map(|p| (p, in_map)).ok_or((
            FollowPath_Result::INVALID_PATH,
            "the path is empty, or not all numbers".into(),
        ))
    }

    fn cancel(&mut self, id: u64) {
        if self.job.as_ref().is_some_and(|j| j.id == id) {
            self.job = None;
            self.follower.stop();
            self.publish(0.0, 0.0);
            self.set_lag_aware(false);
        }
    }

    fn finish(&mut self, finish: Finish, why: &str) {
        self.publish(0.0, 0.0);
        self.follower.stop();
        self.set_lag_aware(false);
        if let Some(job) = self.job.take() {
            if finish != Finish::Succeeded {
                log_warn!(&self.logger, "FollowPath failed: {why}");
            }
            let _ = job.done.send(finish);
        }
    }

    fn fresh(&self, reading: Option<(f64, f64)>, now: f64) -> Option<f64> {
        reading
            .filter(|(_, when)| now - when <= self.config.measured_timeout)
            .map(|(level, _)| level)
    }

    fn tick(&mut self) {
        let started = Instant::now();
        let now = self.now();
        let period = 1.0 / self.config.rate_hz;
        if let Some((pose, speed, when)) = self.odom {
            if now - when <= self.config.odom_timeout {
                let seen = Observation {
                    pose,
                    speed,
                    steering: self.fresh(self.steering, now),
                    throttle: self.fresh(self.throttle, now),
                    brake: self.fresh(self.brake, now),
                };
                self.follower.observe(&seen);
            }
        }
        let Some(job) = self.job.as_ref() else {
            // Keeps the estimate's actuators moving on with the car held.
            self.follower.tick(now, None);
            return;
        };
        if let Some(last) = self.last_tick {
            let late = ((now - last) / period).round() as i64 - 1;
            self.missed += late.max(0) as u64;
        }
        self.last_tick = Some(now);
        let fresh_odom = self
            .odom
            .is_some_and(|(_, _, when)| now - when <= self.config.odom_timeout);
        if !fresh_odom {
            if self.odom.is_none() && now - job.started <= self.config.odom_timeout {
                return; // a goal can arrive before the first odometry
            }
            let why = format!("no odometry on {}", self.config.odom_topic);
            return self.finish(Finish::Aborted(FollowPath_Result::UNKNOWN), &why);
        }
        // Nav2's controller: no current costmap within costmap_update_timeout, a zero
        // command and CONTROLLER_TIMED_OUT. A path gets the limit for its first one.
        if self.config.use_costmap {
            let checked = self
                .costmap_heard
                .check(now, job.started, self.config.costmap_timeout);
            if let Err(stale) = checked {
                let why = format!("the costmap on {}: {stale}", self.config.costmap_topic);
                return self.finish(
                    Finish::Aborted(FollowPath_Result::CONTROLLER_TIMED_OUT),
                    &why,
                );
            }
        }
        self.correct(now);
        let costmap = if self.config.use_costmap {
            self.costmap.as_ref()
        } else {
            None
        };
        let lane = self.lane_in_odom(now);
        self.lane_known = lane.is_some();
        self.follower.set_lane(lane);
        if self
            .moving_hold
            .is_some_and(|heard| now - heard <= self.config.road_ahead_timeout)
        {
            self.follower.held();
        }
        match self.follower.tick(now, costmap) {
            None => {}
            Some(Tick::Done(outcome)) => {
                let (finish, why) = match outcome {
                    Outcome::Succeeded => (Finish::Succeeded, "at the goal"),
                    Outcome::Stuck => (
                        Finish::Aborted(FollowPath_Result::FAILED_TO_MAKE_PROGRESS),
                        "no progress in time",
                    ),
                    Outcome::NoValidControl => (
                        Finish::Aborted(FollowPath_Result::NO_VALID_CONTROL),
                        "every sampled future collides",
                    ),
                    Outcome::PathOutsideCostmap => (
                        Finish::Aborted(FollowPath_Result::INVALID_PATH),
                        "no part of the path ahead is inside the costmap",
                    ),
                    Outcome::NoEstimate => (
                        Finish::Aborted(FollowPath_Result::UNKNOWN),
                        "no odometry yet",
                    ),
                };
                self.finish(finish, why);
            }
            Some(Tick::Drive(report)) => {
                let c = report.command;
                self.publish(c.speed, c.yaw_rate(self.config.wheelbase));
                if let Some(job) = self.job.as_ref() {
                    let _ = job.feedback.publish(FollowPath_Feedback {
                        distance_to_goal: report.foot.left as f32,
                        speed: report.speed as f32,
                    });
                }
                let loop_ms = started.elapsed().as_secs_f64() * 1000.0;
                self.loop_max = self.loop_max.max(loop_ms);
                self.report(loop_ms, &report);
                self.publish_local_plan(&report.plan.poses);
            }
        }
    }

    /// Move the held map -> odom toward the newest (fresh only), and the running map path
    /// with it (TODO 186). A snap (a relocalisation) is logged.
    fn correct(&mut self, now: f64) {
        let in_map = self.job.as_ref().is_some_and(|j| j.in_map);
        let since = self.corrected.replace(now);
        let (Some((newest, stamp)), Some((car, _, _))) = (self.map_to_odom, self.odom) else {
            return;
        };
        if !in_map || now - stamp > self.config.transform_tolerance + 1.0 {
            return;
        }
        let dt = since.map_or(1.0 / self.config.rate_hz, |t| now - t);
        let Some((correction, shift)) = self.correction.update(newest, &car, dt) else {
            return;
        };
        if let Correction::Snapped { distance, angle } = correction {
            log_warn!(
                &self.logger,
                "map -> odom moved {distance:.2} m and {:.1} degrees at the car: a \
                 relocalisation, taken at once",
                angle.to_degrees()
            );
        }
        each!(&mut self.follower, f => f.move_path(|p| shift.apply(p)));
    }

    /// MPPI's chosen future, in odom, stamped now.
    fn publish_local_plan(&self, poses: &[Pose]) {
        let header = Header {
            stamp: self.stamp(),
            frame_id: self.config.odom_frame.clone(),
        };
        let poses = poses
            .iter()
            .map(|p| PoseStamped {
                header: header.clone(),
                pose: PoseMsg {
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
                },
            })
            .collect();
        let _ = self.local_plan.publish(Path { header, poses });
    }

    fn publish(&self, v: f64, w: f64) {
        let msg = TwistStamped {
            header: Header {
                stamp: self.stamp(),
                frame_id: "base_link".into(),
            },
            twist: Twist {
                linear: Vector3 {
                    x: v,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: w,
                },
            },
        };
        if let Err(e) = self.cmd.publish(msg) {
            log_error!(&self.logger, "cmd_vel_nav: {e}");
        }
    }

    /// m: how far the held map -> odom puts the car from where the newest puts it (0 with
    /// the correction off, or before both are known).
    fn map_gap(&self) -> f64 {
        let (Some(held), Some((newest, _)), Some((car, _, _))) =
            (self.correction.held(), self.map_to_odom, self.odom)
        else {
            return 0.0;
        };
        into_map(&car, &held).distance(&into_map(&car, &newest))
    }

    fn report(&self, loop_ms: f64, r: &Report) {
        let period_ms = 1000.0 / self.config.rate_hz;
        let values = [
            ("tracker", TRACKER.to_string()),
            ("model", self.config.model.clone()),
            ("speed_loop", self.config.speed_loop.name().to_string()),
            ("loop_ms", format!("{loop_ms:.3}")),
            ("loop_ms_max", format!("{:.3}", self.loop_max)),
            ("missed_ticks", self.missed.to_string()),
            ("cross_track_m", format!("{:.3}", r.foot.cross_track)),
            ("heading_error_rad", format!("{:.4}", r.foot.heading_error)),
            ("steer_rad", format!("{:.4}", r.command.steer)),
            ("speed_mps", format!("{:.3}", r.command.speed)),
            ("left_m", format!("{:.2}", r.foot.left)),
            ("est_steering_rad", format!("{:.4}", r.steering)),
            ("best_cost", format!("{:.4}", r.plan.best_cost)),
            ("collided_share", format!("{:.3}", r.plan.collided)),
            ("lane", self.lane_known.to_string()),
            ("map_gap_m", format!("{:.3}", self.map_gap())),
        ];
        let msg = DiagnosticStatus {
            level: if loop_ms > period_ms {
                DiagnosticStatus::WARN
            } else {
                DiagnosticStatus::OK
            },
            name: "track".into(),
            message: TRACKER.into(),
            hardware_id: "aav_mppi_lag".into(),
            values: values
                .into_iter()
                .map(|(key, value)| KeyValue {
                    key: key.into(),
                    value,
                })
                .collect(),
        };
        let _ = self.status.publish(msg);
    }
}

/// The `speed_loop` parameter's profile: spine-core's names (default, smooth, lqi, firm).
fn speed_loop_profile(name: &str) -> Result<SpeedLoopProfile> {
    name.parse()
        .map_err(|e| anyhow::anyhow!("speed_loop:={name}: {e}"))
}

/// Everything read from the parameters, once.
#[derive(Debug, Clone)]
struct Config {
    model: String,
    /// The spine's speed loop profile the car runs (TODO 163).
    speed_loop: SpeedLoopProfile,
    rate_hz: f64,
    odom_topic: String,
    odom_frame: String,
    map_frame: String,
    odom_timeout: f64,
    transform_tolerance: f64,
    measured_timeout: f64,
    cmd_topic: String,
    status_topic: String,
    local_plan_topic: String,
    lag_aware_topic: String,
    costmap_topic: String,
    use_costmap: bool,
    /// The road ahead (aav_interfaces/RoadAhead), for LaneCritic; empty: not read.
    road_ahead_topic: String,
    /// s: a road ahead older than this is dropped (it comes at 10 Hz).
    road_ahead_timeout: f64,
    /// s: a costmap older than this stops the car and fails the path (Nav2's
    /// costmap_update_timeout, plus the local costmap's publish period).
    costmap_timeout: f64,
    wheelbase: f64,
    max_steering_angle: f64,
    actuators: Actuators,
    ego: Ego,
    mppi: Settings,
    follower: FollowerSettings,
    critics: Vec<Critic>,
    footprint: Vec<(f64, f64)>,
    /// How fast the held map -> odom follows the newest (TODO 186).
    map_correction: SlewLimits,
}

/// Declares the parameters; hold the handles as long as the node (rclrs undeclares a
/// parameter when its handle drops).
struct Declared {
    _handles: Vec<Box<dyn std::any::Any>>,
}

struct Declarer<'a> {
    node: &'a Node,
    handles: Vec<Box<dyn std::any::Any>>,
}

impl Declarer<'_> {
    fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        let p = self
            .node
            .declare_parameter(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name} (a float)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        if !value.is_finite() {
            bail!("{name}:={value}: not a number");
        }
        Ok(value)
    }

    fn integer(&mut self, name: &str, default: i64) -> Result<i64> {
        let p = self
            .node
            .declare_parameter(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name} (an integer)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        Ok(value)
    }

    fn count(&mut self, name: &str, default: usize) -> Result<usize> {
        let value = self.integer(name, default as i64)?;
        usize::try_from(value).with_context(|| format!("{name}:={value}: must be 0 or more"))
    }

    fn flag(&mut self, name: &str, default: bool) -> Result<bool> {
        let p = self
            .node
            .declare_parameter(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name} (true or false)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        Ok(value)
    }

    fn text(&mut self, name: &str, default: &str) -> Result<String> {
        let p = self
            .node
            .declare_parameter::<Arc<str>>(name)
            .default(default.into())
            .read_only()
            .with_context(|| format!("parameter {name} (text)"))?;
        let value = p.get().to_string();
        self.handles.push(Box::new(p));
        Ok(value)
    }

    fn numbers(&mut self, name: &str, default: &[f64]) -> Result<Vec<f64>> {
        let p = self
            .node
            .declare_parameter::<Arc<[f64]>>(name)
            .default(default.into())
            .read_only()
            .with_context(|| format!("parameter {name} (a list of floats)"))?;
        let value = p.get().to_vec();
        self.handles.push(Box::new(p));
        Ok(value)
    }

    fn names(&mut self, name: &str, default: &[&str]) -> Result<Vec<String>> {
        let default: Arc<[Arc<str>]> = default.iter().map(|s| Arc::from(*s)).collect();
        let p = self
            .node
            .declare_parameter::<Arc<[Arc<str>]>>(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name} (a list of names)"))?;
        let value = p.get().iter().map(|s| s.to_string()).collect();
        self.handles.push(Box::new(p));
        Ok(value)
    }

    /// A critic's own parameters, Nav2's names, under `<Critic>.`.
    fn critic(&mut self, critic: Critic) -> Result<Critic> {
        let n = critic.name();
        let weight = |d: &mut Self, w: f64| d.number(&format!("{n}.cost_weight"), w);
        let power = |d: &mut Self, p: u32| -> Result<u32> {
            let v = d.integer(&format!("{n}.cost_power"), i64::from(p))?;
            u32::try_from(v.max(1)).context("cost_power")
        };
        let threshold = |d: &mut Self, t: f64| d.number(&format!("{n}.threshold_to_consider"), t);
        Ok(match critic {
            Critic::Constraint(mut c) => {
                c.weight = weight(self, c.weight)?;
                c.power = power(self, c.power)?;
                Critic::Constraint(c)
            }
            Critic::Cost(mut c) => {
                c.weight = weight(self, c.weight)?;
                c.power = power(self, c.power)?;
                c.critical_cost = self.number(&format!("{n}.critical_cost"), c.critical_cost)?;
                c.collision_cost = self.number(&format!("{n}.collision_cost"), c.collision_cost)?;
                c.near_goal_distance =
                    self.number(&format!("{n}.near_goal_distance"), c.near_goal_distance)?;
                c.trajectory_point_step = self.count(
                    &format!("{n}.trajectory_point_step"),
                    c.trajectory_point_step,
                )?;
                c.consider_footprint =
                    self.flag(&format!("{n}.consider_footprint"), c.consider_footprint)?;
                Critic::Cost(c)
            }
            Critic::Goal(mut c) => {
                c.weight = weight(self, c.weight)?;
                c.power = power(self, c.power)?;
                c.threshold = threshold(self, c.threshold)?;
                Critic::Goal(c)
            }
            Critic::GoalAngle(mut c) => {
                c.weight = weight(self, c.weight)?;
                c.power = power(self, c.power)?;
                c.threshold = threshold(self, c.threshold)?;
                Critic::GoalAngle(c)
            }
            Critic::PathAlign(mut c) => {
                c.weight = weight(self, c.weight)?;
                c.power = power(self, c.power)?;
                c.threshold = threshold(self, c.threshold)?;
                c.offset_from_furthest =
                    self.count(&format!("{n}.offset_from_furthest"), c.offset_from_furthest)?;
                c.trajectory_point_step = self.count(
                    &format!("{n}.trajectory_point_step"),
                    c.trajectory_point_step,
                )?;
                c.max_path_occupancy_ratio = self.number(
                    &format!("{n}.max_path_occupancy_ratio"),
                    c.max_path_occupancy_ratio,
                )?;
                c.use_path_orientations = self.flag(
                    &format!("{n}.use_path_orientations"),
                    c.use_path_orientations,
                )?;
                Critic::PathAlign(c)
            }
            Critic::PathFollow(mut c) => {
                c.weight = weight(self, c.weight)?;
                c.power = power(self, c.power)?;
                c.threshold = threshold(self, c.threshold)?;
                c.offset_from_furthest =
                    self.count(&format!("{n}.offset_from_furthest"), c.offset_from_furthest)?;
                Critic::PathFollow(c)
            }
            Critic::PathAngle(mut c) => {
                c.weight = weight(self, c.weight)?;
                c.power = power(self, c.power)?;
                c.threshold = threshold(self, c.threshold)?;
                c.offset_from_furthest =
                    self.count(&format!("{n}.offset_from_furthest"), c.offset_from_furthest)?;
                c.max_angle_to_furthest = self.number(
                    &format!("{n}.max_angle_to_furthest"),
                    c.max_angle_to_furthest,
                )?;
                let mode = self.integer(&format!("{n}.mode"), 0)?;
                c.mode = PathAngleMode::from_number(mode).with_context(|| {
                    format!(
                        "{n}.mode:={mode}: 0 (forward), 1 (either way) or 2 (the way the \
                         path faces)"
                    )
                })?;
                Critic::PathAngle(c)
            }
            Critic::PreferForward(mut c) => {
                c.weight = weight(self, c.weight)?;
                c.power = power(self, c.power)?;
                c.threshold = threshold(self, c.threshold)?;
                Critic::PreferForward(c)
            }
            Critic::Lane(mut c) => {
                c.soft_weight = self.number(&format!("{n}.soft_weight"), c.soft_weight)?;
                c.hard_weight = self.number(&format!("{n}.hard_weight"), c.hard_weight)?;
                c.power = power(self, c.power)?;
                c.trajectory_point_step = self.count(
                    &format!("{n}.trajectory_point_step"),
                    c.trajectory_point_step,
                )?;
                Critic::Lane(c)
            }
        })
    }
}

impl Config {
    fn declare(node: &Node) -> Result<(Self, Declared)> {
        let mut d = Declarer {
            node,
            handles: Vec::new(),
        };
        let a = Actuators::default();
        let actuators = Actuators {
            max_accel: d.number("actuators.max_accel", a.max_accel)?,
            throttle_deadband: d.number("actuators.throttle_deadband", a.throttle_deadband)?,
            throttle_lag: d.number("actuators.throttle_lag", a.throttle_lag)?,
            max_brake: d.number("actuators.max_brake", a.max_brake)?,
            brake_lag: d.number("actuators.brake_lag", a.brake_lag)?,
            drag: d.number("actuators.drag", a.drag)?,
            steer_rate: d.number("actuators.steer_rate", a.steer_rate)?,
            steer_lag: d.number("actuators.steer_lag", a.steer_lag)?,
            steer_offset: d.number("actuators.steer_offset", a.steer_offset)?,
            steer_deadband: d.number("actuators.steer_deadband", a.steer_deadband)?,
            throttle_dead_time: d.number("actuators.throttle_dead_time", 0.0)?,
            brake_dead_time: d.number("actuators.brake_dead_time", 0.0)?,
            steer_dead_time: d.number("actuators.steer_dead_time", 0.0)?,
        };
        let wheelbase = d.number("wheelbase", 1.561)?;
        let max_steering_angle = d.number("max_steering_angle", 0.5)?;
        let e = Ego::default();
        let ego = Ego {
            max_acceleration: d.number("ego.max_acceleration", e.max_acceleration)?,
            max_deceleration: d.number("ego.max_deceleration", e.max_deceleration)?,
            max_steering_rate: d.number("ego.max_steering_rate", e.max_steering_rate)?,
            min_speed_for_steering: d
                .number("ego.min_speed_for_steering", e.min_speed_for_steering)?,
            max_steering_angle,
        };
        let rate_hz = d.number("rate_hz", 20.0)?;
        let s = Settings::default();
        let mppi = Settings {
            time_steps: d.count("time_steps", s.time_steps)?,
            model_dt: d.number("model_dt", s.model_dt)?,
            batch_size: d.count("batch_size", s.batch_size)?,
            iteration_count: d.count("iteration_count", s.iteration_count)?,
            temperature: d.number("temperature", s.temperature)?,
            gamma: d.number("gamma", s.gamma)?,
            // Below 0 (the default): the steering's control cost is gamma's, as Nav2's.
            steer_gamma: Some(d.number("steer_gamma", -1.0)?).filter(|g| *g >= 0.0),
            steer_gamma_speed: d.number("steer_gamma_speed", s.steer_gamma_speed)?,
            speed_std: d.number("vx_std", s.speed_std)?,
            steer_std: d.number("steer_std", s.steer_std)?,
            steer_std_speed: d.number("steer_std_speed", s.steer_std_speed)?,
            steer_std_min: d.number("steer_std_min", s.steer_std_min)?,
            noise_correlation: d.number("noise_correlation", s.noise_correlation)?,
            // TODO 212: smooth samples the steering's change (the launch's
            // mppi_lag_sampling:=smooth); correlated, the champion, by default.
            sampling: d
                .text("sampling", "correlated")?
                .parse()
                .map_err(|e| anyhow::anyhow!("{e}"))?,
            steer_rate_std: d.number("steer_rate_std", s.steer_rate_std)?,
            vx_max: d.number("vx_max", s.vx_max)?,
            vx_min: d.number("vx_min", s.vx_min)?,
            max_steer: max_steering_angle,
            retry_attempt_limit: d.count("retry_attempt_limit", s.retry_attempt_limit)?,
            threads: d.count("threads", s.threads)?,
            seed: d.count("seed", s.seed as usize)? as u64,
        };
        if rate_hz <= 0.0 || (mppi.model_dt - 1.0 / rate_hz).abs() > 1e-6 {
            bail!(
                "model_dt ({}) must equal 1 / rate_hz ({rate_hz} Hz), as Nav2's MPPI asks",
                mppi.model_dt
            );
        }
        if mppi.batch_size == 0 || mppi.time_steps < 2 {
            bail!("batch_size must be over 0 and time_steps over 1");
        }
        let f = FollowerSettings::default();
        let follower = FollowerSettings {
            max_search: d.number("max_robot_pose_search_dist", f.max_search)?,
            prune_distance: d.number("prune_distance", f.prune_distance)?,
            xy_goal_tolerance: d.number("xy_goal_tolerance", f.xy_goal_tolerance)?,
            yaw_goal_tolerance: d.number("yaw_goal_tolerance", f.yaw_goal_tolerance)?,
            progress_radius: d.number("progress_radius", f.progress_radius)?,
            progress_angle: d.number("progress_angle", f.progress_angle)?,
            progress_time: d.number("progress_time", f.progress_time)?,
            tracking_unknown: d.flag("track_unknown_space", f.tracking_unknown)?,
            inversion: None,
            cusp_reach: d.number("cusp_reach", f.cusp_reach)?,
            cusp_wait: d.number("cusp_wait", f.cusp_wait)?,
        };
        // Nav2's enforce_path_inversion: stop at each cusp, then follow the next stretch.
        let inversion = Inversion::default();
        let tolerance = Inversion {
            xy: d.number("inversion_xy_tolerance", inversion.xy)?,
            yaw: d.number("inversion_yaw_tolerance", inversion.yaw)?,
        };
        let follower = FollowerSettings {
            inversion: d
                .flag("enforce_path_inversion", false)?
                .then_some(tolerance),
            ..follower
        };

        // The car's outline, padded (nav2.yaml's local costmap footprint + padding).
        let flat = d.numbers("footprint", &[1.8, 0.5, 1.8, -0.5, -0.2, -0.5, -0.2, 0.5])?;
        if flat.len() < 6 || flat.len() % 2 != 0 {
            bail!(
                "footprint: x, y pairs, at least three corners (got {} numbers)",
                flat.len()
            );
        }
        let padding = d.number("footprint_padding", 0.05)?;
        let footprint: Vec<(f64, f64)> = flat
            .chunks(2)
            .map(|xy| {
                let pad = |v: f64| v + padding * v.signum();
                (pad(xy[0]), pad(xy[1]))
            })
            .collect();
        // CostCritic checks the whole outline over the inflation's cost at the outline's
        // furthest corner (Nav2's circumscribed cost).
        let circumscribed = footprint
            .iter()
            .map(|(x, y)| x.hypot(*y))
            .fold(0.0, f64::max);
        let inscribed = footprint
            .iter()
            .zip(footprint.iter().cycle().skip(1))
            .map(|(a, b)| {
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                (a.0 * b.1 - a.1 * b.0).abs() / dx.hypot(dy).max(1e-9)
            })
            .fold(f64::INFINITY, f64::min);
        let inflation_radius = d.number("inflation_radius", 2.0)?;
        let scaling = d.number("cost_scaling_factor", 3.0)?;
        let possible_collision_cost = if inflation_radius < circumscribed {
            0.0
        } else {
            inflation_cost(circumscribed, inscribed, scaling)
        };

        let mut critics = Vec::new();
        for name in d.names("critics", &mppi_lag::critics::NAMES)? {
            let Some(critic) = Critic::by_name(&name) else {
                bail!(
                    "critics: {name} isn't one of ours ({}, {})",
                    mppi_lag::critics::NAMES.join(", "),
                    mppi_lag::critics::OWN.join(", ")
                );
            };
            let critic = match d.critic(critic)? {
                Critic::Cost(mut c) => {
                    c.possible_collision_cost = possible_collision_cost;
                    c.tracking_unknown = follower.tracking_unknown;
                    Critic::Cost(c)
                }
                other => other,
            };
            critics.push(critic);
        }

        // `map_correction` (the launch file's switch, never in the yaml: a node's own file
        // wins over the launch's values) false: the old way, whatever the rates say.
        let m = SlewLimits::default();
        let slew = d.flag("map_correction", false)?;
        let speed = d.number("map_correction_speed", m.speed)?;
        let map_correction = SlewLimits {
            speed: if slew { speed } else { 0.0 },
            turn_rate: d.number("map_correction_turn_rate", m.turn_rate)?,
            snap_distance: d.number("map_correction_snap_distance", m.snap_distance)?,
            snap_angle: d.number("map_correction_snap_angle", m.snap_angle)?,
        };
        let config = Self {
            model: d.text("model", "lag")?,
            speed_loop: speed_loop_profile(
                &d.text("speed_loop", SpeedLoopProfile::Default.name())?,
            )?,
            rate_hz,
            odom_topic: d.text("odom_topic", "/localization/odom/local")?,
            odom_frame: d.text("odom_frame", "odom")?,
            map_frame: d.text("map_frame", "map")?,
            odom_timeout: d.number("odom_timeout", 1.0)?,
            transform_tolerance: d.number("transform_tolerance", 0.5)?,
            measured_timeout: d.number("measured_timeout", 0.5)?,
            cmd_topic: d.text("cmd_topic", "cmd_vel_nav")?,
            status_topic: d.text("status_topic", "/drivers/nav/track/status")?,
            local_plan_topic: d.text("local_plan_topic", "/drivers/nav/local_plan")?,
            lag_aware_topic: d.text("lag_aware_topic", "/drivers/nav/lag_aware")?,
            costmap_topic: d.text("costmap_topic", "/local_costmap/costmap_raw")?,
            use_costmap: d.flag("use_costmap", true)?,
            road_ahead_topic: d.text("road_ahead_topic", "")?,
            road_ahead_timeout: d.number("road_ahead_timeout", 0.5)?,
            costmap_timeout: d.number("costmap_timeout", 0.8)?,
            wheelbase,
            max_steering_angle,
            actuators,
            ego,
            mppi,
            follower,
            critics,
            footprint,
            map_correction,
        };
        Ok((
            config,
            Declared {
                _handles: d.handles,
            },
        ))
    }

    /// The follower, with the model the `model` parameter names.
    fn follower(&self) -> Result<AnyFollower> {
        let follower = |model| {
            Follower::new(
                model,
                self.ego,
                self.mppi.clone(),
                self.critics.clone(),
                self.follower,
                self.footprint.clone(),
            )
        };
        let lag = LagModel {
            actuators: self.actuators,
            wheelbase: self.wheelbase,
            max_steering_angle: self.max_steering_angle,
            ..LagModel::default()
        };
        Ok(match self.model.as_str() {
            "lag" => match SpeedLoop::from_profile(self.speed_loop) {
                Ok(speed_loop) => {
                    AnyFollower::Lag(Box::new(follower(LagModel { speed_loop, ..lag })))
                }
                // The LQI: the shared model runs the spine's own loop, any law.
                Err(_) => AnyFollower::Shared(Box::new(Follower::new(
                    SharedModel::like(&lag, self.speed_loop),
                    self.ego,
                    self.mppi.clone(),
                    self.critics.clone(),
                    self.follower,
                    self.footprint.clone(),
                ))),
            },
            "ideal" => AnyFollower::Ideal(Box::new(Follower::new(
                IdealModel {
                    wheelbase: self.wheelbase,
                    max_steering_angle: self.max_steering_angle,
                    // Ego's ramps are in front of it; the car itself answers at once.
                    accel: 100.0,
                    decel: 100.0,
                },
                self.ego,
                self.mppi.clone(),
                self.critics.clone(),
                self.follower,
                self.footprint.clone(),
            ))),
            other => bail!("model:={other}: lag (the actuators' lag) or ideal (none, as Nav2's)"),
        })
    }
}
