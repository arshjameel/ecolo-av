//! safety_filter: the safety filter as a ROS 2 node (TODO 104). It sits in front of
//! twist_mux for the automatic drivers only (K1), so a person on the joystick or RC never
//! goes through it. **In shadow by default**: it reads each driver's command and publishes
//! what it would send on its own topics, so it drives nothing until a launch file wires it
//! in. The maths is the `safety-filter` crate, with no ROS.
//!
//! ```text
//! shadow (default):  /drivers/<d>/cmd_vel ──► safety_filter ──► /safety/filter/shadow/<d>/cmd_vel
//! live (K1):         /drivers/<d>/cmd_vel_raw ──► safety_filter ──► /drivers/<d>/cmd_vel ──► twist_mux
//! ```
//!
//! Subscribes  in_topic for each d in drivers   geometry_msgs/TwistStamped: the driver's
//!                                         command ({d} in in_topic is the driver's name)
//!             /world/road_ahead           aav_interfaces/RoadAhead (map): the line, its
//!                                         edges (soft), the end
//!             /localization/odom/global   nav_msgs/Odometry (map): the pose
//!             /vehicle/odom               nav_msgs/Odometry: the measured speed
//! Publishes   out_topic for each d        geometry_msgs/TwistStamped: the filtered command,
//!                                         the driver's header kept
//!             /safety/filter/status       diagnostic_msgs/DiagnosticStatus, one per
//!                                         command: hardware_id the driver; level OK
//!                                         (passed), WARN (trimmed); message the rules
//!                                         that acted; values every number of the report
//!                                         (v_in, w_in, v_out, w_out, stop_m, h_speed, v_allow,
//!                                         h_left, h_right, w_low, w_high, V, V_dot,
//!                                         slack, speed_now, offset, twist, room, dt):
//!                                         record it and a drive can be scored from the bag
//! Parameters  drivers (["nav", "point", "ai"]), in_topic ("/drivers/{d}/cmd_vel"),
//!             out_topic ("/safety/filter/shadow/{d}/cmd_vel"); road_timeout,
//!             pose_timeout, speed_timeout (0.5 s); the filter's settings (README).

use std::any::Any;
use std::sync::Arc;

use anyhow::{bail, Context as _, Result};
use ego_core::stop_law::StopCurve;
use rclrs::*;
use ros_env::aav_interfaces::msg::RoadAhead;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{Twist, TwistStamped, Vector3};
use ros_env::nav_msgs::msg::Odometry;
use safety_filter::{Command, Inputs, Lane, Report, SafetyFilter, Settings};

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("safety_filter")?;
    let mut d = Declarer::new(&node);
    let drivers = d.names("drivers", &["nav", "point", "ai"])?;
    let in_topic = d.text("in_topic", "/drivers/{d}/cmd_vel")?;
    let out_topic = d.text("out_topic", "/safety/filter/shadow/{d}/cmd_vel")?;
    if in_topic == out_topic {
        bail!("in_topic and out_topic are the same ({in_topic}): the filter would hear itself");
    }
    let timeouts = Timeouts {
        road: d.number("road_timeout", 0.5)?,
        pose: d.number("pose_timeout", 0.5)?,
        speed: d.number("speed_timeout", 0.5)?,
    };
    let curve = StopCurve::new(
        d.number("brake", 1.0)?,
        d.number("jerk", 2.0)?,
        d.number("delay", 0.35)?,
    )
    .context("the stopping law (brake, jerk, delay)")?;
    let settings = Settings {
        curve,
        margin: d.number("margin", 0.5)?,
        max_offset: d.number("max_offset", 3.0)?,
        alpha_speed: d.number("alpha_speed", 1.0)?,
        look_ahead: d.number("look_ahead", 1.5)?,
        alpha_lane: d.number("alpha_lane", 1.0)?,
        half_width: d.number("half_width", 0.5)?,
        lane_margin: d.number("lane_margin", 0.2)?,
        clf_rate: d.number("clf_rate", 0.5)?,
        clf_weight: d.number("clf_weight", 1.0)?,
        wheelbase: d.number("wheelbase", 1.561)?,
        max_steer: d.number("max_steer", 0.5)?,
        max_speed: d.number("max_speed", 1.0)?,
        max_reverse: d.number("max_reverse", 0.5)?,
    }
    .check()
    .context("the filter's settings")?;
    log_info!(
        node.logger(),
        "safety filter for [{}]: {in_topic} -> {out_topic}; {}; margin {} m, look-ahead {} m, \
         Lyapunov weight {}",
        drivers.join(", "),
        settings.curve.describe(),
        settings.margin,
        settings.look_ahead,
        settings.clf_weight
    );

    let core = Core {
        clock: node.get_clock(),
        timeouts,
        filters: drivers
            .iter()
            .map(|_| (SafetyFilter::new(settings.clone()), None))
            .collect(),
        outs: drivers
            .iter()
            .map(|name| node.create_publisher(topic_for(&out_topic, name).as_str()))
            .collect::<Result<Vec<_>, _>>()?,
        status: node.create_publisher("/safety/filter/status")?,
        names: drivers.clone(),
        road: None,
        pose: None,
        speed: None,
    };
    let worker = node.create_worker(core);
    let mut keep: Vec<Box<dyn Any>> = Vec::new();
    for (i, name) in drivers.iter().enumerate() {
        keep.push(Box::new(worker.create_subscription(
            topic_for(&in_topic, name).as_str(),
            move |s: &mut Core, msg: TwistStamped| s.on_cmd(i, &msg),
        )?));
    }
    keep.push(Box::new(worker.create_subscription(
        "/world/road_ahead",
        |s: &mut Core, msg: RoadAhead| {
            let now = s.now();
            s.road = Some((lane_of(&msg), now));
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/localization/odom/global",
        |s: &mut Core, msg: Odometry| {
            let p = &msg.pose.pose;
            let q = &p.orientation;
            let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            let now = s.now();
            s.pose = Some(((p.position.x, p.position.y, yaw), now));
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/vehicle/odom",
        |s: &mut Core, msg: Odometry| {
            let v = msg.twist.twist.linear.x;
            if v.is_finite() {
                let now = s.now();
                s.speed = Some((v, now));
            }
        },
    )?));
    keep.push(Box::new(worker));
    keep.extend(d.finish());

    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    drop(keep);
    Ok(())
}

/// A topic pattern with the driver's name in place of `{d}`.
fn topic_for(pattern: &str, driver: &str) -> String {
    pattern.replace("{d}", driver)
}

/// The road ahead a RoadAhead message carries: the line and its soft edges.
fn lane_of(msg: &RoadAhead) -> Lane {
    let wide = |v: &[f32]| v.iter().map(|&x| f64::from(x)).collect::<Vec<f64>>();
    Lane {
        spacing: f64::from(msg.spacing),
        x: wide(&msg.x),
        y: wide(&msg.y),
        heading: wide(&msg.heading),
        left: wide(&msg.left_soft),
        right: wide(&msg.right_soft),
        end: f64::from(msg.end),
    }
}

/// How old each input may be (s).
struct Timeouts {
    road: f64,
    pose: f64,
    speed: f64,
}

/// The worker's payload.
struct Core {
    clock: Clock,
    timeouts: Timeouts,
    /// Each driver's filter and when its last command came (s).
    filters: Vec<(SafetyFilter, Option<f64>)>,
    outs: Vec<Publisher<TwistStamped>>,
    status: Publisher<DiagnosticStatus>,
    names: Vec<String>,
    /// The newest road ahead and when it came (s).
    road: Option<(Lane, f64)>,
    /// The newest pose (x, y, yaw) and when it came (s).
    pose: Option<((f64, f64, f64), f64)>,
    /// The newest measured speed and when it came (s).
    speed: Option<(f64, f64)>,
}

impl Core {
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_cmd(&mut self, i: usize, msg: &TwistStamped) {
        let now = self.now();
        let fresh = |at: f64, limit: f64| now - at <= limit;
        let road = self
            .road
            .as_ref()
            .filter(|(_, at)| fresh(*at, self.timeouts.road))
            .map(|(lane, _)| lane);
        let pose = self
            .pose
            .filter(|(_, at)| fresh(*at, self.timeouts.pose))
            .map(|(p, _)| p);
        let frame = match (road, pose) {
            (Some(lane), Some((x, y, yaw))) => lane.frame(x, y, yaw),
            _ => None,
        };
        let speed = self
            .speed
            .filter(|(_, at)| fresh(*at, self.timeouts.speed))
            .map(|(v, _)| v);
        let Some((filter, last)) = self.filters.get_mut(i) else {
            return;
        };
        let dt = last.map_or(0.0, |t| now - t);
        *last = Some(now);
        let cmd = Command {
            v: msg.twist.linear.x,
            w: msg.twist.angular.z,
        };
        let report = filter.step(cmd, &Inputs { frame, speed, dt });
        let out = report.output;
        if let Some(publisher) = self.outs.get(i) {
            let _ = publisher.publish(TwistStamped {
                header: msg.header.clone(),
                twist: Twist {
                    linear: Vector3 {
                        x: out.v,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: out.w,
                    },
                },
            });
        }
        let name = self.names.get(i).cloned().unwrap_or_default();
        let _ = self.status.publish(status(&name, &report, frame, dt));
    }
}

/// The report as a DiagnosticStatus: every number, for the bag.
fn status(
    driver: &str,
    r: &Report,
    frame: Option<safety_filter::Frame>,
    dt: f64,
) -> DiagnosticStatus {
    let rules: Vec<String> = r.rules.iter().map(ToString::to_string).collect();
    let f = frame.unwrap_or(safety_filter::Frame {
        along: f64::NAN,
        offset: f64::NAN,
        twist: f64::NAN,
        curvature: f64::NAN,
        room: f64::NAN,
        left: f64::NAN,
        right: f64::NAN,
    });
    let values = [
        ("v_in", r.input.v),
        ("w_in", r.input.w),
        ("v_out", r.output.v),
        ("w_out", r.output.w),
        ("speed_now", r.speed_now),
        ("stop_m", r.stop_distance),
        ("h_speed", r.h_speed),
        ("v_allow", r.v_allow),
        ("h_left", r.h_left),
        ("h_right", r.h_right),
        ("w_low", r.w_low),
        ("w_high", r.w_high),
        ("V", r.lyapunov),
        ("V_dot", r.lyapunov_rate),
        ("slack", r.slack),
        ("offset", f.offset),
        ("twist", f.twist),
        ("curvature", f.curvature),
        ("room", f.room),
        ("dt", dt),
    ]
    .into_iter()
    .map(|(key, value)| KeyValue {
        key: key.into(),
        value: format!("{value:.4}"),
    })
    .chain(std::iter::once(KeyValue {
        key: "rules".into(),
        value: rules.join(" "),
    }))
    .collect();
    DiagnosticStatus {
        level: if r.trimmed() { 1 } else { 0 },
        name: "safety_filter".into(),
        message: if rules.is_empty() {
            "passed".into()
        } else {
            rules.join(" ")
        },
        hardware_id: driver.into(),
        values,
    }
}

/// Declares parameters and keeps their handles (rclrs undeclares a parameter when its
/// handle drops), as ego-node's `Declarer`.
struct Declarer<'a> {
    node: &'a Node,
    handles: Vec<Box<dyn Any>>,
}

impl<'a> Declarer<'a> {
    fn new(node: &'a Node) -> Self {
        Declarer {
            node,
            handles: Vec::new(),
        }
    }

    fn finish(self) -> Vec<Box<dyn Any>> {
        self.handles
    }

    /// A read-only number (a float, or a whole number).
    fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(ParameterValue::Double(default))
            .read_only()
            .with_context(|| format!("parameter {name} (a number)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        match value {
            ParameterValue::Double(v) if v.is_finite() => Ok(v),
            ParameterValue::Integer(v) => Ok(v as f64),
            other => bail!("{name}:={other:?}: not a number"),
        }
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

    fn names(&mut self, name: &str, default: &[&str]) -> Result<Vec<String>> {
        let default: Arc<[Arc<str>]> = default.iter().map(|s| Arc::from(*s)).collect();
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(ParameterValue::StringArray(default))
            .read_only()
            .with_context(|| format!("parameter {name} (a list of names)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        match value {
            ParameterValue::StringArray(names) => Ok(names.iter().map(|s| s.to_string()).collect()),
            other => bail!("{name}:={other:?}: not a list of names"),
        }
    }
}
