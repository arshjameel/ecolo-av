//! point_driver, in Rust: drive to one point B (lat/lon or map x/y) and stop there
//! smoothly. The same node as aav_drive_to_point's point_driver.py, topic for topic.
//!
//! Action      drive_to_point             aav_interfaces/action/DriveToPoint
//! Subscribes  localization/odom/global   nav_msgs/Odometry: where the car is on the map, its speed
//!             sensors/gps/fix            sensor_msgs/NavSatFix (best effort): fresh? RTK?
//!             safety/estop               std_msgs/Bool: true stops the drive
//!             safety/sensor_watchdog/lock  std_msgs/Bool: true, or silent for
//!                                        watchdog_timeout, stops the drive
//!             safety/geofence/fence      geometry_msgs/PolygonStamped, transient local: a B
//!                                        outside it is refused before the car moves
//!             system/source              aav_interfaces/DataSource, latched: REAL needs a
//!                                        person to confirm (the goal's `confirmed`)
//!             control/cmd_vel            geometry_msgs/TwistStamped: twist_mux's choice, to
//!                                        stop when someone else takes over
//! Publishes   drivers/point/cmd_vel      geometry_msgs/TwistStamped, at rate_hz while a goal
//!                                        runs (twist_mux priority 25)
//!             drivers/point/plan         nav_msgs/Path (map), latched: the line A -> B while a
//!                                        drive runs, empty when it ends
//! Calls       /fromLL                    robot_localization/srv/FromLL: lat/lon to the map
//!             /ego/get_parameters        rcl_interfaces/srv/GetParameters: ego's caps
//! Parameters  config/drive_to_point.yaml's (rate_hz, cruise_speed, ... stop_law), and
//!             max_brake, wheelbase, max_steering_angle from vehicle.yaml (the launch file)
//!
//! The maths and rules are the point-core crate (no ROS). One goal at a time; it stops at
//! once on a cancel, the e-stop, the watchdog's lock or silence, stale inputs, someone else
//! driving, a car that won't move, or leaving the drive's corridor.

use std::collections::{HashMap, VecDeque};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{anyhow, bail, Context as _, Result};
use behaviour_core::fence::Polygon;
use futures::channel::oneshot;
use futures::future::{self, Either};
use point_core::ego_core::car_model::ActuatorConfig;
use point_core::fmt_g;
use point_core::line::{min_radius, Point, Pose};
use point_core::rules::{self, Limits, Now, Refusal};
use point_core::run::{self, Caps, Outcome, PlanError, Run, Settings, Status};
use rclrs::*;
use ros_env::aav_interfaces::action::{DriveToPoint, DriveToPoint_Feedback, DriveToPoint_Result};
use ros_env::aav_interfaces::msg::DataSource;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::geographic_msgs::msg::GeoPoint;
use ros_env::geometry_msgs::msg::{PolygonStamped, PoseStamped, TwistStamped};
use ros_env::nav_msgs::msg::{Odometry, Path};
use ros_env::rcl_interfaces::srv::{GetParameters, GetParameters_Request, GetParameters_Response};
use ros_env::robot_localization::srv::{FromLL, FromLL_Request, FromLL_Response};
use ros_env::sensor_msgs::msg::NavSatFix;
use ros_env::std_msgs::msg::{Bool, Header};

/// Ego's caps, as its parameter names.
const EGO_PARAMS: [&str; 4] = [
    "max_speed",
    "max_acceleration",
    "max_deceleration",
    "max_jerk",
];
/// s of twist_mux passing someone else's commands: stop.
const TAKEOVER_TIME: f64 = 0.3;
/// Our last commands kept, to tell ours from someone else's.
const OURS: usize = 50;
/// sensor_msgs/NavSatStatus.STATUS_FIX.
const STATUS_FIX: i8 = 0;

/// How it slows into B.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StopLaw {
    /// approach.py's sqrt(2 a (d - lead v)), the default.
    Approach,
    /// The stopping law (stop_law.py), a trial.
    Hybrid,
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("point_driver")?;
    let (_keep, cmd) = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    let spun = executor.spin(SpinOptions::default()).first_error();
    // A last stop; twist_mux's timeout does the rest.
    let _ = cmd.publish(twist(&node.get_clock(), 0.0, 0.0));
    spun?;
    Ok(())
}

/// The node's parameters, read once.
struct Params {
    rate_hz: f64,
    settings: Settings,
    limits: Limits,
    radius: f64,
    law: StopLaw,
}

fn read_params(node: &Node, keep: &mut Handles) -> Result<Params> {
    let mut number = |name: &str, default: f64| -> Result<f64> {
        let p = node
            .declare_parameter::<ParameterValue>(name)
            .default(ParameterValue::Double(default))
            .read_only()
            .with_context(|| format!("parameter {name} (a number)"))?;
        let value = match p.get() {
            ParameterValue::Double(v) => v,
            ParameterValue::Integer(v) => v as f64,
            other => bail!("{name}:={other:?}: not a number"),
        };
        keep.push(Box::new(p));
        Ok(value)
    };
    let d = Settings::default();
    let settings = Settings {
        cruise_speed: number("cruise_speed", d.cruise_speed)?,
        acceleration: number("acceleration", d.acceleration)?,
        max_brake: number("max_brake", d.max_brake)?,
        brake_fraction: number("brake_fraction", d.brake_fraction)?,
        deceleration: number("deceleration", d.deceleration)?,
        brake_lead: number("brake_lead", d.brake_lead)?,
        stop_band: number("stop_band", d.stop_band)?,
        creep: number("creep", d.creep)?,
        lookahead_min: number("lookahead_min", d.lookahead_min)?,
        lookahead_time: number("lookahead_time", d.lookahead_time)?,
        lateral_accel: number("lateral_accel", d.lateral_accel)?,
        still_speed: number("still_speed", d.still_speed)?,
        stuck_time: number("stuck_time", d.stuck_time)?,
        still_time: number("still_time", d.still_time)?,
        max_off_line: number("max_off_line", d.max_off_line)?,
        max_overrun: number("max_overrun", d.max_overrun)?,
    };
    let rate_hz = number("rate_hz", 50.0)?;
    if rate_hz.is_nan() || rate_hz <= 0.0 {
        bail!("rate_hz:={rate_hz}: a rate must be above 0 Hz");
    }
    let wheelbase = number("wheelbase", 1.57)?;
    let max_steering = number("max_steering_angle", 0.5)?;
    let l = Limits::default();
    let mut limits = Limits {
        max_distance: number("max_distance", l.max_distance)?,
        pose_timeout: number("pose_timeout", l.pose_timeout)?,
        gps_timeout: number("gps_timeout", l.gps_timeout)?,
        watchdog_timeout: number("watchdog_timeout", l.watchdog_timeout)?,
        ..l
    };
    let gps = node
        .declare_parameter::<bool>("require_gps")
        .default(true)
        .read_only()?;
    limits.require_gps = gps.get();
    keep.push(Box::new(gps));
    let law = node
        .declare_parameter::<Arc<str>>("stop_law")
        .default("approach".into())
        .read_only()?;
    let law_name = law.get().to_string();
    keep.push(Box::new(law));
    let law = match law_name.as_str() {
        "approach" => StopLaw::Approach,
        "hybrid" => StopLaw::Hybrid,
        other => bail!("stop_law:={other}: pick one of approach, hybrid"),
    };
    let radius = min_radius(wheelbase, max_steering).map_err(|e| anyhow!("{e}"))?;
    Ok(Params {
        rate_hz,
        settings,
        limits,
        radius,
        law,
    })
}

/// aav_description's vehicle.yaml, where colcon installed it.
fn installed_vehicle_yaml() -> Option<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .ok()?
        .split(':')
        .map(|prefix| PathBuf::from(prefix).join("share/aav_description/config/vehicle.yaml"))
        .find(|p| p.is_file())
}

fn yaw_of(q: &ros_env::geometry_msgs::msg::Quaternion) -> f64 {
    (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
}

fn stamp(clock: &Clock) -> Time {
    clock.now().to_ros_msg().unwrap_or_default()
}

fn twist(clock: &Clock, v: f64, curvature: f64) -> TwistStamped {
    let mut msg = TwistStamped {
        header: Header {
            stamp: stamp(clock),
            frame_id: "base_link".into(),
        },
        ..TwistStamped::default()
    };
    msg.twist.linear.x = v;
    msg.twist.angular.z = v * curvature;
    msg
}

/// The drive running now.
struct Active {
    run: Run,
    cancel_asked: bool,
    done: Option<oneshot::Sender<()>>,
    feedback: FeedbackPublisher<DriveToPoint>,
}

/// What the node knows, and the drive running.
struct Core {
    p: Params,
    car: Option<ActuatorConfig>,
    busy: bool,
    pose: Option<(Pose, f64)>,
    pose_time: Option<f64>,
    fix_time: Option<f64>,
    fix_status: Option<i8>,
    estop: bool,
    locked: bool,
    lock_time: Option<f64>,
    sources: HashMap<String, Vec<u8>>,
    ours: VecDeque<(f64, f64)>,
    other_since: Option<f64>,
    fence: Option<Polygon>,
    active: Option<Active>,
    cmd: Publisher<TwistStamped>,
    plan: Publisher<Path>,
    clock: Clock,
    logger: Logger,
}

impl Core {
    fn seconds(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_pose(&mut self, msg: &Odometry) {
        let q = &msg.pose.pose;
        let v = msg.twist.twist.linear.x;
        if !(q.position.x.is_finite() && q.position.y.is_finite() && v.is_finite()) {
            return; // a NaN is no position: it goes stale, and the drive stops
        }
        self.pose = Some((
            Pose {
                x: q.position.x,
                y: q.position.y,
                yaw: yaw_of(&q.orientation),
            },
            v,
        ));
        self.pose_time = Some(self.seconds());
    }

    fn on_fix(&mut self, msg: &NavSatFix) {
        if msg.status.status >= STATUS_FIX && msg.latitude.is_finite() && msg.longitude.is_finite()
        {
            self.fix_time = Some(self.seconds());
            self.fix_status = Some(msg.status.status);
        }
    }

    fn on_lock(&mut self, msg: &Bool) {
        self.locked = msg.data;
        self.lock_time = Some(self.seconds());
    }

    fn on_fence(&mut self, msg: &PolygonStamped) {
        let corners: Vec<(f64, f64)> = msg
            .polygon
            .points
            .iter()
            .map(|p| (f64::from(p.x), f64::from(p.y)))
            .collect();
        match Polygon::new(corners) {
            Ok(fence) => self.fence = Some(fence),
            Err(e) => log_warn!(&self.logger, "the geofence sent no usable fence: {e}"),
        }
    }

    fn on_source(&mut self, msg: &DataSource) {
        self.sources.insert(
            msg.announcer.clone(),
            msg.sensors.iter().map(|s| s.source).collect(),
        );
    }

    fn on_mux(&mut self, msg: &TwistStamped) {
        let theirs = (msg.twist.linear.x, msg.twist.angular.z);
        if !self.busy
            || self.ours.is_empty()
            || !rules::taken_over(self.ours.iter().copied(), theirs)
        {
            self.other_since = None;
        } else if self.other_since.is_none() {
            self.other_since = Some(self.seconds());
        }
    }

    fn now(&self) -> Now {
        let t = self.seconds();
        let age = |at: Option<f64>| at.map(|s| t - s);
        Now {
            pose_age: age(self.pose_time),
            fix_age: age(self.fix_time),
            estop: self.estop,
            locked: self.locked,
            kind: rules::source_kind(self.sources.values().flatten().copied()),
            taken_over: self.other_since.is_some_and(|s| t - s > TAKEOVER_TIME),
            lock_age: age(self.lock_time),
        }
    }

    fn publish(&mut self, v: f64, curvature: f64) {
        let msg = twist(&self.clock, v, curvature);
        if self.ours.len() == OURS {
            self.ours.pop_front();
        }
        self.ours
            .push_back((msg.twist.linear.x, msg.twist.angular.z));
        if let Err(e) = self.cmd.publish(msg) {
            log_warn!(&self.logger, "drivers/point/cmd_vel: {e}");
        }
    }

    /// Send the drive's line A -> B (map), or an empty path: no drive.
    fn publish_line(&self, points: &[Point]) {
        let header = Header {
            stamp: stamp(&self.clock),
            frame_id: "map".into(),
        };
        let mut msg = Path {
            header: header.clone(),
            ..Path::default()
        };
        for &(x, y) in points {
            let mut pose = PoseStamped {
                header: header.clone(),
                ..PoseStamped::default()
            };
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.orientation.w = 1.0;
            msg.poses.push(pose);
        }
        if let Err(e) = self.plan.publish(msg) {
            log_warn!(&self.logger, "drivers/point/plan: {e}");
        }
    }

    /// The timer: one tick of the drive running, if any.
    fn tick(&mut self) {
        let now = self.now();
        let t = self.seconds();
        let Some((pose, measured)) = self.pose else {
            return;
        };
        let Some(active) = self.active.as_mut() else {
            return;
        };
        let step = active
            .run
            .tick(t, &now, &pose, measured, active.cancel_asked);
        if let Some(why) = step.stopped {
            log_warn!(&self.logger, "drive to point stopping: {why}");
        }
        if let Some(c) = step.feedback {
            let _ = active.feedback.publish(DriveToPoint_Feedback {
                phase: c.phase.code(),
                distance_left: c.left as f32,
                cross_track: c.cross as f32,
                commanded_speed: c.speed as f32,
                measured_speed: measured as f32,
            });
        }
        if step.finished {
            if let Some(done) = active.done.take() {
                let _ = done.send(());
            }
        }
        if let Some((v, k)) = step.send {
            self.publish(v, k);
        }
    }

    /// Start a drive: Ok(the done signal), or why not.
    fn begin(
        &mut self,
        target: Point,
        caps: &Caps,
        goal: &ros_env::aav_interfaces::action::DriveToPoint_Goal,
        feedback: FeedbackPublisher<DriveToPoint>,
    ) -> Result<oneshot::Receiver<()>, String> {
        let s = self.p.settings;
        let approach = run::plan(
            f64::from(goal.cruise_speed),
            f64::from(goal.acceleration),
            f64::from(goal.deceleration),
            caps,
            &s,
            self.car.as_ref(),
        )
        .map_err(|e| e.to_string())?;
        let Some((start, _)) = self.pose else {
            return Err(Refusal::Stop(rules::Stop::NoPosition).to_string());
        };
        let why = rules::refuse_target(&start, target, &self.p.limits, self.p.radius)
            .or_else(|| {
                rules::refuse_outside(self.fence.as_ref().map(|f| f.depth(target.0, target.1)))
            })
            .or_else(|| rules::refuse_start(&self.now(), &self.p.limits, goal.confirmed));
        if let Some(why) = why {
            return Err(why.to_string());
        }
        let run = Run::new(
            target,
            approach,
            s,
            self.p.limits,
            self.p.radius,
            &start,
            self.seconds(),
        )
        .map_err(|e| PlanError::Line(e).to_string())?;
        log_info!(
            &self.logger,
            "drive to ({:.2}, {:.2}): {:.1} m; cruise {} m/s, up {} m/s^2, down {} m/s^2 \
             (max_brake {} x {})",
            target.0,
            target.1,
            run.distance(),
            fmt_g(approach.cruise),
            fmt_g(approach.accel),
            fmt_g(approach.decel),
            fmt_g(s.max_brake),
            fmt_g(s.brake_fraction)
        );
        self.ours.clear();
        self.other_since = None;
        self.publish_line(&[(start.x, start.y), target]);
        let (done, finished) = oneshot::channel();
        self.active = Some(Active {
            run,
            cancel_asked: false,
            done: Some(done),
            feedback,
        });
        Ok(finished)
    }

    /// End the drive: a last zero, no line, and how it went.
    fn end(&mut self) -> Option<Outcome> {
        let active = self.active.take()?;
        self.publish(0.0, 0.0);
        self.publish_line(&[]);
        let (pose, _) = self.pose?;
        Some(active.run.outcome(
            &pose,
            self.fix_status,
            self.p.limits.require_gps,
            self.seconds(),
        ))
    }
}

/// A future that is ready after `d` of wall-clock time.
fn after(d: Duration) -> oneshot::Receiver<()> {
    let (tx, rx) = oneshot::channel();
    std::thread::spawn(move || {
        std::thread::sleep(d);
        let _ = tx.send(());
    });
    rx
}

/// `promise`'s answer, or None after `seconds` of wall-clock time.
async fn within<T>(promise: Promise<T>, seconds: f64) -> Option<T> {
    match future::select(promise, after(Duration::from_secs_f64(seconds))).await {
        Either::Left((Ok(answer), _)) => Some(answer),
        _ => None,
    }
}

/// The services a goal calls.
#[derive(Clone)]
struct Calls {
    from_ll: Client<FromLL>,
    ego: Client<GetParameters>,
}

impl Calls {
    /// B in the map frame, or why not.
    async fn target_of(
        &self,
        goal: &ros_env::aav_interfaces::action::DriveToPoint_Goal,
    ) -> Result<Point, String> {
        if !goal.use_lat_lon {
            return Ok((goal.x, goal.y));
        }
        let (lat, lon) = (goal.latitude, goal.longitude);
        if !(lat.is_finite() && lon.is_finite() && lat.abs() <= 90.0 && lon.abs() <= 180.0) {
            return Err("latitude and longitude must be on the globe".into());
        }
        if !self.from_ll.service_is_ready().unwrap_or(false) {
            return Err("no /fromLL (navsat_transform): lat/lon needs the GPS running".into());
        }
        let request = FromLL_Request {
            ll_point: GeoPoint {
                latitude: lat,
                longitude: lon,
                altitude: 0.0,
            },
        };
        let promise: Promise<FromLL_Response> = self
            .from_ll
            .call(&request)
            .map_err(|e| format!("/fromLL: {e}"))?;
        match within(promise, 3.0).await {
            Some(res) => Ok((res.map_point.x, res.map_point.y)),
            None => Err("/fromLL did not answer in 3 s".into()),
        }
    }

    /// Ego's caps (none if it doesn't answer).
    async fn ego_caps(&self) -> Caps {
        if !self.ego.service_is_ready().unwrap_or(false) {
            return Caps::default();
        }
        let request = GetParameters_Request {
            names: EGO_PARAMS.iter().map(|n| n.to_string()).collect(),
        };
        let Ok(promise) = self.ego.call(&request) else {
            return Caps::default();
        };
        let promise: Promise<GetParameters_Response> = promise;
        let Some(res) = within(promise, 2.0).await else {
            return Caps::default();
        };
        // rcl_interfaces/ParameterType: 2 integer, 3 double; anything else isn't a number.
        let value = |i: usize| {
            res.values.get(i).and_then(|v| match v.type_ {
                2 => Some(v.integer_value as f64),
                3 => Some(v.double_value),
                _ => None,
            })
        };
        Caps {
            max_speed: value(0),
            max_acceleration: value(1),
            max_deceleration: value(2),
            max_jerk: value(3),
        }
    }
}

fn result(target: Point, message: &str) -> DriveToPoint_Result {
    DriveToPoint_Result {
        success: false,
        message: message.into(),
        target_x: target.0,
        target_y: target.1,
        ..DriveToPoint_Result::default()
    }
}

/// One goal, from its acceptance to its end.
async fn drive(
    worker: Worker<Core>,
    calls: Calls,
    goal: ExecutingGoal<DriveToPoint>,
) -> TerminatedGoal {
    let asked: ros_env::aav_interfaces::action::DriveToPoint_Goal = (**goal.goal()).clone();
    let none = (f64::NAN, f64::NAN);
    let refuse = |goal: ExecutingGoal<DriveToPoint>, worker: &Worker<Core>, why: String, at| {
        let w = worker.clone();
        let text = why.clone();
        drop(w.run(move |c: &mut Core| {
            log_warn!(&c.logger, "drive to point refused: {text}");
        }));
        goal.aborted_with(result(at, &why))
    };
    let start = worker
        .run({
            let confirmed = asked.confirmed;
            move |c: &mut Core| rules::refuse_start(&c.now(), &c.p.limits, confirmed)
        })
        .await
        .ok()
        .flatten();
    if let Some(why) = start {
        return refuse(goal, &worker, why.to_string(), none);
    }
    let target = match calls.target_of(&asked).await {
        Ok(t) => t,
        Err(why) => return refuse(goal, &worker, why, none),
    };
    let caps = calls.ego_caps().await;
    let feedback = goal.feedback_publisher();
    let started = worker
        .run({
            let asked = asked.clone();
            move |c: &mut Core| c.begin(target, &caps, &asked, feedback)
        })
        .await
        .unwrap_or_else(|e| Err(format!("error: {e:?}")));
    let finished = match started {
        Ok(f) => f,
        Err(why) => return refuse(goal, &worker, why, target),
    };
    let (goal, cancelling) = match goal.until_cancel_requested(finished).await {
        Ok(_) => (Some(goal), None),
        Err(finished) => {
            let _ = worker
                .run(|c: &mut Core| {
                    if let Some(a) = c.active.as_mut() {
                        a.cancel_asked = true;
                    }
                })
                .await;
            let cancelling = goal.begin_cancelling();
            let _ = finished.await;
            (None, Some(cancelling))
        }
    };
    let end = worker
        .run(|c: &mut Core| {
            let end = c.end();
            if let Some(e) = &end {
                log_info!(&c.logger, "drive to point: {}", e.message);
            }
            end
        })
        .await
        .ok()
        .flatten();
    let Some(end) = end else {
        let r = result(target, "error: no pose at the end");
        return match (goal, cancelling) {
            (Some(g), _) => g.aborted_with(r),
            (None, Some(c)) => c.aborted_with(r),
            (None, None) => unreachable!("a goal is either running or cancelling"),
        };
    };
    let r = DriveToPoint_Result {
        success: end.success,
        message: end.message.clone(),
        target_x: target.0,
        target_y: target.1,
        distance_left: end.left as f32,
        tolerance: end.tolerance as f32,
        distance_travelled: end.travelled as f32,
        duration: end.duration as f32,
    };
    match (goal, cancelling, end.status) {
        (Some(g), _, Status::Succeeded) => g.succeeded_with(r),
        (Some(g), _, _) => g.aborted_with(r),
        (None, Some(c), Status::Cancelled) => c.cancelled_with(r),
        (None, Some(c), Status::Succeeded) => c.succeeded_with(r),
        (None, Some(c), Status::Aborted) => c.aborted_with(r),
        (None, None, _) => unreachable!("a goal is either running or cancelling"),
    }
}

/// What the node keeps alive: its parameters, subscriptions, timer, server and worker.
type Handles = Vec<Box<dyn std::any::Any>>;

fn start(node: &Node) -> Result<(Handles, Publisher<TwistStamped>)> {
    let mut keep: Handles = Vec::new();
    let p = read_params(node, &mut keep)?;
    // hybrid: the stopping law, from the shared car model's brake (vehicle.yaml).
    let car = match p.law {
        StopLaw::Approach => None,
        StopLaw::Hybrid => {
            let path = installed_vehicle_yaml()
                .context("stop_law hybrid needs aav_description's vehicle.yaml, not installed")?;
            Some(
                ActuatorConfig::from_vehicle_yaml(&path)
                    .with_context(|| format!("vehicle.yaml {}", path.display()))?,
            )
        }
    };
    let rate = p.rate_hz;
    let latched = |topic: &'static str| topic.keep_last(1).reliable().transient_local();
    let cmd: Publisher<TwistStamped> = node.create_publisher("drivers/point/cmd_vel")?;
    let core = Core {
        p,
        car,
        busy: false,
        pose: None,
        pose_time: None,
        fix_time: None,
        fix_status: None,
        estop: false,
        locked: false,
        lock_time: None,
        sources: HashMap::new(),
        ours: VecDeque::with_capacity(OURS),
        other_since: None,
        fence: None,
        active: None,
        cmd: cmd.clone(),
        plan: node.create_publisher(latched("drivers/point/plan"))?,
        clock: node.get_clock(),
        logger: node.logger().clone(),
    };
    core.publish_line(&[]);
    let worker = node.create_worker(core);
    keep.push(Box::new(worker.create_subscription(
        "localization/odom/global",
        |c: &mut Core, m: Odometry| c.on_pose(&m),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "sensors/gps/fix".keep_last(5).best_effort(),
        |c: &mut Core, m: NavSatFix| c.on_fix(&m),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "safety/estop",
        |c: &mut Core, m: Bool| c.estop = m.data,
    )?));
    keep.push(Box::new(worker.create_subscription(
        "safety/sensor_watchdog/lock",
        |c: &mut Core, m: Bool| c.on_lock(&m),
    )?));
    keep.push(Box::new(worker.create_subscription(
        latched("system/source"),
        |c: &mut Core, m: DataSource| c.on_source(&m),
    )?));
    keep.push(Box::new(worker.create_subscription(
        latched("safety/geofence/fence"),
        |c: &mut Core, m: PolygonStamped| c.on_fence(&m),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "control/cmd_vel",
        |c: &mut Core, m: TwistStamped| c.on_mux(&m),
    )?));
    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        |c: &mut Core| c.tick(),
    )?));
    let calls = Calls {
        from_ll: node.create_client::<FromLL>("/fromLL")?,
        ego: node.create_client::<GetParameters>("/ego/get_parameters")?,
    };
    let w = worker.clone();
    keep.push(Box::new(node.create_action_server::<DriveToPoint, _>(
        "drive_to_point",
        move |requested: RequestedGoal<DriveToPoint>| {
            let worker = w.clone();
            let calls = calls.clone();
            async move {
                // One goal at a time: busy at once, so a second goal, before this one
                // runs, is refused.
                let free = worker
                    .run(|c: &mut Core| {
                        if c.busy {
                            log_warn!(
                                &c.logger,
                                "rejected a goal: another drive to a point is running"
                            );
                            false
                        } else {
                            c.busy = true;
                            true
                        }
                    })
                    .await
                    .unwrap_or(false);
                if !free {
                    return requested.reject();
                }
                let goal = requested.accept().execute();
                let ended = drive(worker.clone(), calls, goal).await;
                let _ = worker.run(|c: &mut Core| c.busy = false).await;
                ended
            }
        },
    )?));
    keep.push(Box::new(worker));
    Ok((keep, cmd))
}
