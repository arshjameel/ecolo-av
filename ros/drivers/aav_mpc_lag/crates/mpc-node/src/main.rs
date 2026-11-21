//! mpc_lag_rs: the lag-aware MPC in Rust, in controller_server's FollowPath place. A
//! challenger beside aav_track's Python mpc_lag (TODO 132, 199): the same maths, the same
//! numbers, the same checks.
//!
//! Action      follow_path                 nav2_msgs/action/FollowPath: the navigator's, a new
//!                                         goal each time the planner replans (the newest
//!                                         replaces the last without a stop)
//! Subscribes  localization/odom/local     nav_msgs/Odometry (odom_topic): the pose in odom
//!                                         and the speed
//!             /tf                         tf2_msgs/TFMessage: map -> odom. Held, and moved
//!                                         toward the newest at most map_correction_speed m/s
//!                                         and map_correction_turn_rate rad/s (TODO 186:
//!                                         follow in odom, correct slowly); each new map path
//!                                         comes into odom by the held one, and the running
//!                                         path moves with it. map_correction false (the
//!                                         launch's map_correction:=newest) or rates 0: the
//!                                         newest, once a path
//! Publishes   cmd_vel_nav                 geometry_msgs/TwistStamped, at rate_hz while a path
//!                                         runs: (its set speed, v tan(steer) / wheelbase), to
//!                                         Nav2's velocity smoother, collision_monitor, then
//!                                         /drivers/nav/cmd_vel
//!             drivers/nav/track/status    diagnostic_msgs/DiagnosticStatus, every tick: the
//!                                         same keys as aav_track's mpc_lag
//!             drivers/nav/lag_aware       std_msgs/Bool, reliable, transient local, depth 1:
//!                                         true while a path runs (it models the lags itself,
//!                                         so ego's lead stays off), false when it ends, at
//!                                         start and at exit
//! Serves      nothing else; calls nothing.
//! Parameters  read once at start (config/mpc_lag_rs.yaml, and navigation.launch.py's
//!             vehicle.mpc_lag_rs_params: nav2.yaml, vehicle.yaml, ego.yaml): see the README.
//!
//! Reversing  (TODO 213) a path is cut at its cusps (mpc-lag's `gears`): a backwards
//!             stretch is given to the MPC mirrored (the car turned round, its speed and
//!             steering negated) and driven backwards at max_reverse_speed at most (0: a
//!             path that reverses is refused, INVALID_PATH). The car is held at 0 within
//!             0.3 m of a cusp; the next stretch is taken once it stands there (under
//!             0.05 m/s), or after standing 2 s within 1.5 m. The goal is checked on the
//!             last stretch only; the progress check starts again at each stretch.
//!
//! navigation.launch.py tracker:=mpc_lag_rs starts it in mppi_lag's place. Everything
//! after cmd_vel_nav is unchanged. The maths is the `mpc-lag` crate (`Geared`); the map
//! correction and Nav2's goal and progress checks are `mppi-lag`'s (aav_mppi_lag); this
//! file only moves messages.

use std::{
    f64::consts::PI,
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use car_model::ActuatorConfig;
use futures::channel::oneshot;
use mpc_lag::{
    wrap, Command as MpcCommand, CuspRules, Direction, Geared, Limits, MpcLag, Observed, Settings,
};
use mppi_lag::follower::{GoalCheck, ProgressCheck};
use mppi_lag::map_correction::{into_map, into_odom, Correction};
use mppi_lag::{MapCorrection, Pose, SlewLimits};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{Quaternion, Twist, TwistStamped, Vector3};
use ros_env::nav2_msgs::action::{FollowPath, FollowPath_Feedback, FollowPath_Result};
use ros_env::nav_msgs::msg::{Odometry, Path as PathMsg};
use ros_env::std_msgs::msg::{Bool, Header};
use ros_env::tf2_msgs::msg::TFMessage;
use spine_core::speed_loop::SpeedLoopProfile;

/// The name on the status topic and in the launch's `tracker:=`.
const TRACKER: &str = "mpc_lag_rs";

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(TRACKER)?;

    let (config, _params) = Config::declare(&node)?;
    let mpc = MpcLag::new(config.limits, config.car, config.settings)
        .map_err(|e| anyhow::anyhow!("mpc_lag: {e}"))?;
    let mpc = Geared::new(mpc, config.max_reverse_speed, CuspRules::default());

    // Latched: true while a path runs, so ego's lead doesn't cancel the lags twice.
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

    log_info!(
        node.logger(),
        "the lag-aware MPC (Rust) follows Nav2's paths at {} Hz, up to {} m/s; {} ticks \
         ahead; dead time {:?} ticks; done within {} m and {} rad; {}",
        config.rate_hz,
        config.settings.cruise,
        config.settings.horizon,
        mpc.mpc().dead_ticks(),
        config.xy_goal_tolerance,
        config.yaw_goal_tolerance,
        if config.max_reverse_speed > 0.0 {
            format!("backwards up to {} m/s", config.max_reverse_speed)
        } else {
            "forward only".to_string()
        }
    );
    let core = Core {
        lag_pub,
        lag_on: false,
        cmd: node.create_publisher(config.cmd_topic.as_str())?,
        status: node.create_publisher(config.status_topic.as_str())?,
        clock: node.get_clock(),
        logger: node.logger().clone(),
        goal: GoalCheck::new(config.xy_goal_tolerance, config.yaw_goal_tolerance),
        progress: ProgressCheck::new(
            config.progress_radius,
            config.progress_angle,
            config.progress_time,
        ),
        mpc,
        steer: 0.0,
        odom: None,
        map_to_odom: None,
        correction: MapCorrection::new(config.map_correction),
        corrected: None,
        job: None,
        next_id: 0,
        last_tick: None,
        missed: 0,
        loop_max: 0.0,
        config: config.clone(),
    };

    let worker = node.create_worker(core);
    let _odom = worker
        .create_subscription(config.odom_topic.as_str(), |s: &mut Core, msg: Odometry| {
            s.on_odom(&msg)
        })?;
    let _tf = worker.create_subscription("/tf", |s: &mut Core, msg: TFMessage| s.on_tf(&msg))?;
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

/// One FollowPath goal.
struct Job {
    id: u64,
    done: oneshot::Sender<Finish>,
    feedback: FeedbackPublisher<FollowPath>,
    started: f64,
    /// The path came in map: it moves as the held map -> odom moves.
    in_map: bool,
}

/// The worker's payload: the tracker, what it has heard, and what it publishes with.
struct Core {
    config: Config,
    /// `/drivers/nav/lag_aware`, and what it last said.
    lag_pub: Publisher<Bool>,
    lag_on: bool,
    cmd: Publisher<TwistStamped>,
    status: Publisher<DiagnosticStatus>,
    clock: Clock,
    logger: Logger,
    goal: GoalCheck,
    progress: ProgressCheck,
    /// The MPC, forward and back.
    mpc: Geared,
    /// rad: the last steering sent.
    steer: f64,
    /// (pose, speed, when) from odometry.
    odom: Option<(Pose, f64, f64)>,
    /// map -> odom: odom's pose in map, and when.
    map_to_odom: Option<(Pose, f64)>,
    /// The held map -> odom (TODO 186), slewed toward `map_to_odom`.
    correction: MapCorrection,
    /// When the held map -> odom last moved on (this node's clock).
    corrected: Option<f64>,
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
    /// Say on `lag_aware` whether a path is running (only when it changes).
    fn set_lag_aware(&mut self, on: bool) {
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

    /// A new FollowPath goal: its path into odom, then follow it. Returns the job's id.
    fn start(
        &mut self,
        msg: &PathMsg,
        done: oneshot::Sender<Finish>,
        feedback: FeedbackPublisher<FollowPath>,
    ) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        let in_map = match self.path_in_odom(msg).and_then(|(poses, in_map)| {
            self.mpc
                .follow(&poses)
                .map(|()| in_map)
                .map_err(|e| (FollowPath_Result::INVALID_PATH, e.to_string()))
        }) {
            Ok(in_map) => in_map,
            Err((code, why)) => {
                log_warn!(&self.logger, "FollowPath refused: {why}");
                let _ = done.send(Finish::Aborted(code));
                return id;
            }
        };
        if let Some(old) = self.job.take() {
            // The planner replanned: the newest path carries on, with no stop between.
            drop(old.done);
        } else {
            self.goal.reset();
            self.progress.reset();
            self.missed = 0;
            self.loop_max = 0.0;
            self.last_tick = None;
            // The car was held since the last path: start the estimate from it as measured.
            self.mpc.forget();
        }
        self.corrected = Some(self.now());
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

    /// The goal's poses in odom, and whether they came in map. A map path comes by the held
    /// map -> odom while one runs (none: the newest, which is then held).
    fn path_in_odom(&mut self, msg: &PathMsg) -> Result<(Vec<mpc_lag::Pose>, bool), (u16, String)> {
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
        Ok((poses.iter().map(|p| (p.x, p.y, p.yaw)).collect(), in_map))
    }

    fn cancel(&mut self, id: u64) {
        if self.job.as_ref().is_some_and(|j| j.id == id) {
            self.job = None;
            self.publish(0.0, 0.0);
            self.set_lag_aware(false);
        }
    }

    fn finish(&mut self, finish: Finish, why: &str) {
        self.publish(0.0, 0.0);
        self.set_lag_aware(false);
        if let Some(job) = self.job.take() {
            if finish != Finish::Succeeded {
                log_warn!(&self.logger, "FollowPath failed: {why}");
            }
            let _ = job.done.send(finish);
        }
    }

    fn tick(&mut self) {
        let started = Instant::now();
        let now = self.now();
        let period = 1.0 / self.config.rate_hz;
        let Some(job) = self.job.as_ref() else {
            return;
        };
        if let Some(last) = self.last_tick {
            let late = ((now - last) / period).round() as i64 - 1;
            self.missed += late.max(0) as u64;
        }
        self.last_tick = Some(now);
        let odom = self
            .odom
            .filter(|(_, _, when)| now - when <= self.config.odom_timeout);
        let Some((pose, speed, _)) = odom else {
            if self.odom.is_none() && now - job.started <= self.config.odom_timeout {
                return; // a goal can arrive before the first odometry
            }
            let why = format!("no odometry on {}", self.config.odom_topic);
            return self.finish(Finish::Aborted(FollowPath_Result::UNKNOWN), &why);
        };
        self.correct(now);
        if self.mpc.advance(pose.x, pose.y, speed, now) {
            // At a cusp: the next stretch, the other way, from a stop.
            self.progress.reset();
            log_info!(
                &self.logger,
                "at a cusp: now {}",
                match self.mpc.direction() {
                    Direction::Forward => "forward",
                    Direction::Backward => "backwards",
                }
            );
        }
        let Some((gx, gy, gyaw)) = self.mpc.goal() else {
            return;
        };
        // Only on the last stretch: a three-point turn may pass near its own goal.
        if self.mpc.on_last() && self.goal.done(&pose, &Pose::new(gx, gy, gyaw)) {
            return self.finish(Finish::Succeeded, "at the goal");
        }
        if self.progress.stuck(&pose, now) {
            let why = format!(
                "moved under {} m in {} s",
                self.config.progress_radius, self.config.progress_time
            );
            return self.finish(
                Finish::Aborted(FollowPath_Result::FAILED_TO_MAKE_PROGRESS),
                &why,
            );
        }
        let seen = Observed {
            x: pose.x,
            y: pose.y,
            yaw: pose.yaw,
            v: speed,
            steer: self.steer,
        };
        let cmd = match self.mpc.step(&seen, period) {
            Ok(cmd) => cmd,
            Err(e) => {
                let why = format!("the plan: {e}");
                return self.finish(Finish::Aborted(FollowPath_Result::UNKNOWN), &why);
            }
        };
        self.steer = cmd.steer;
        self.publish(
            cmd.speed,
            cmd.speed * cmd.steer.tan() / self.config.limits.wheelbase,
        );
        let left = self.mpc.left(pose.x, pose.y);
        if let Some(job) = self.job.as_ref() {
            let _ = job.feedback.publish(FollowPath_Feedback {
                distance_to_goal: left.max(0.0) as f32,
                speed: speed as f32,
            });
        }
        let loop_ms = started.elapsed().as_secs_f64() * 1000.0;
        self.loop_max = self.loop_max.max(loop_ms);
        self.report(loop_ms, &pose, left, &cmd);
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
        self.mpc.move_path(|x, y, yaw| {
            let p = shift.apply(&Pose::new(x, y, yaw));
            (p.x, p.y, p.yaw)
        });
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

    fn report(&self, loop_ms: f64, pose: &Pose, left: f64, cmd: &MpcCommand) {
        let period_ms = 1000.0 / self.config.rate_hz;
        let r = &cmd.report;
        // The foot is on the stretch as the MPC sees it: mirrored backwards.
        let backward = self.mpc.direction() == Direction::Backward;
        let yaw = if backward { pose.yaw + PI } else { pose.yaw };
        let (cross, heading) = r
            .foot
            .map_or((0.0, 0.0), |f| (f.cross, wrap(yaw - f.heading)));
        let values = [
            ("tracker", TRACKER.to_string()),
            ("loop_ms", format!("{loop_ms:.3}")),
            ("loop_ms_max", format!("{:.3}", self.loop_max)),
            ("missed_ticks", self.missed.to_string()),
            ("cross_track_m", format!("{cross:.3}")),
            ("heading_error_rad", format!("{heading:.4}")),
            ("steer_rad", format!("{:.4}", cmd.steer)),
            ("speed_mps", format!("{:.3}", cmd.speed)),
            ("left_m", format!("{left:.2}")),
            ("solve_ms", format!("{:.4}", r.solve_ms)),
            ("iterations", r.iterations.to_string()),
            ("solved", u8::from(r.solved).to_string()),
            ("wheels_est_rad", format!("{:.4}", r.wheels_est)),
            ("speed_limit_mps", format!("{:.4}", r.speed_limit)),
            ("feed_forward_rad", format!("{:.4}", r.feed_forward)),
            ("map_gap_m", format!("{:.3}", self.map_gap())),
            ("backwards", u8::from(backward).to_string()),
        ];
        let msg = DiagnosticStatus {
            level: if loop_ms > period_ms {
                DiagnosticStatus::WARN
            } else {
                DiagnosticStatus::OK
            },
            name: "track".into(),
            message: TRACKER.into(),
            hardware_id: "aav_mpc_lag".into(),
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

/// Everything read from the parameters, once.
#[derive(Debug, Clone)]
struct Config {
    rate_hz: f64,
    odom_topic: String,
    odom_frame: String,
    map_frame: String,
    odom_timeout: f64,
    transform_tolerance: f64,
    cmd_topic: String,
    status_topic: String,
    lag_aware_topic: String,
    xy_goal_tolerance: f64,
    yaw_goal_tolerance: f64,
    progress_radius: f64,
    progress_angle: f64,
    progress_time: f64,
    limits: Limits,
    /// m/s backwards at most; 0: forward only (vehicle.yaml's, with reversing:=true).
    max_reverse_speed: f64,
    car: ActuatorConfig,
    settings: Settings,
    /// How fast the held map -> odom follows the newest (TODO 186).
    map_correction: SlewLimits,
}

/// The parameters' handles: rclrs undeclares a parameter when its handle drops.
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
        if !value.is_finite() && !(name == "progress_angle" && value == f64::INFINITY) {
            bail!("{name}:={value}: not a number");
        }
        Ok(value)
    }

    fn count(&mut self, name: &str, default: usize) -> Result<usize> {
        let p = self
            .node
            .declare_parameter(name)
            .default(default as i64)
            .read_only()
            .with_context(|| format!("parameter {name} (an integer)"))?;
        let value: i64 = p.get();
        self.handles.push(Box::new(p));
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
}

impl Config {
    fn declare(node: &Node) -> Result<(Self, Declared)> {
        let mut d = Declarer {
            node,
            handles: Vec::new(),
        };
        // vehicle.yaml's actuators (vehicle.mpc_lag_rs_params; dead times 0 while unset).
        let a = ActuatorConfig::default();
        let car = ActuatorConfig {
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
            ..a
        };
        let limits = Limits {
            wheelbase: d.number("wheelbase", 1.561)?,
            max_steer: d.number("max_steering_angle", 0.5)?,
            max_steer_rate: d.number("max_steering_rate", 0.6)?,
        };
        let rate_hz = d.number("rate_hz", 20.0)?;
        if rate_hz <= 0.0 {
            bail!("rate_hz:={rate_hz}: must be above 0");
        }
        // The plan's own gains: config/mpc_lag_rs.yaml, the same as aav_track's track.yaml
        // mpc_lag: block (aav_navigation test_launch_params.py checks).
        let s = Settings::default();
        let q = d.numbers("mpc_lag.q", &s.q)?;
        let q: [f64; 3] = q.as_slice().try_into().with_context(|| {
            format!("mpc_lag.q: three weights (e_y, e_theta, wheels), not {q:?}")
        })?;
        let loop_name = d.text("mpc_lag.speed_loop", s.speed_loop.name())?;
        let speed_loop: SpeedLoopProfile = loop_name
            .parse()
            .map_err(|e| anyhow::anyhow!("mpc_lag.speed_loop:={loop_name}: {e}"))?;
        let settings = Settings {
            cruise: d.number("cruise_speed", s.cruise)?,
            accel: d.number("acceleration", s.accel)?,
            decel: d.number("deceleration", s.decel)?,
            max_brake: d.number("max_brake", s.max_brake)?,
            lateral_accel: d.number("lateral_accel", s.lateral_accel)?,
            period: 1.0 / rate_hz,
            horizon: d.count("mpc_lag.horizon", s.horizon)?,
            q,
            tube: d.number("mpc_lag.tube", s.tube)?,
            r: d.number("mpc_lag.r", s.r)?,
            r_rate: d.number("mpc_lag.r_rate", s.r_rate)?,
            q_speed: d.number("mpc_lag.q_speed", s.q_speed)?,
            r_speed: d.number("mpc_lag.r_speed", s.r_speed)?,
            w_slack: d.number("mpc_lag.w_slack", s.w_slack)?,
            terminal: d.number("mpc_lag.terminal", s.terminal)?,
            latency: d.number("mpc_lag.latency", s.latency)?,
            speed_loop,
            substeps: d.count("mpc_lag.substeps", s.substeps)?,
            max_iter: d.count("mpc_lag.max_iter", s.max_iter)?,
            rho: d.number("mpc_lag.rho", s.rho)?,
            sqp: d.count("mpc_lag.sqp", s.sqp)?,
        };
        if !d.flag("stamped", true)? {
            bail!("stamped:=false: this node sends TwistStamped only (nav2.yaml's enable_stamped_cmd_vel)");
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
            rate_hz,
            odom_topic: d.text("odom_topic", "/localization/odom/local")?,
            odom_frame: d.text("odom_frame", "odom")?,
            map_frame: d.text("map_frame", "map")?,
            odom_timeout: d.number("odom_timeout", 1.0)?,
            transform_tolerance: d.number("transform_tolerance", 0.5)?,
            cmd_topic: d.text("cmd_topic", "cmd_vel_nav")?,
            status_topic: d.text("status_topic", "/drivers/nav/track/status")?,
            lag_aware_topic: d.text("lag_aware_topic", "/drivers/nav/lag_aware")?,
            xy_goal_tolerance: d.number("xy_goal_tolerance", 1.0)?,
            yaw_goal_tolerance: d.number("yaw_goal_tolerance", 0.75)?,
            progress_radius: d.number("progress_radius", 0.5)?,
            progress_angle: d.number("progress_angle", 0.5)?,
            progress_time: d.number("progress_time", 10.0)?,
            limits,
            max_reverse_speed: d.number("max_reverse_speed", 0.0)?.max(0.0),
            car,
            settings,
            map_correction,
        };
        Ok((
            config,
            Declared {
                _handles: d.handles,
            },
        ))
    }
}
