//! control_interface: lane keeping on twist_mux's AI slot, in Rust. A drop-in for
//! aav_lane_geometry's Python node (the 2025-26 team's): the same name, topics, QoS,
//! parameters, states and commands. `steering` adds the challengers.
//!
//! Subscribes  /perception/lane/geometry  aav_lane_geometry/LaneGeometry (reliable, depth 10)
//! Publishes   /drivers/ai/cmd_vel        geometry_msgs/TwistStamped, base_link: twist_mux's
//!                                        AI slot (priority 10), so every other driver wins
//!             /drivers/ai/lane_status    std_msgs/String: the state and numbers, as text
//!             /drivers/ai/lane_state     std_msgs/String: the state, once per lane message
//! Parameters  the Python's, with its defaults: base_speed_mps, max_angular_z,
//!             min_speed_mps, min_curvature_m, lane_change_rate_threshold,
//!             lane_change_offset_threshold, lane_change_speed, lane_change_timeout_s,
//!             stop_wait_s, decel_rate, accel_rate, pid_kp, pid_ki, pid_kd,
//!             invert_offset_sign.
//!             Ours: steering (pid, the default; pure_pursuit; stanley), lag_s, and
//!             pure pursuit's lookahead_m, lookahead_gain_s, lookahead_min_m,
//!             lookahead_max_m; Stanley's stanley_gain, stanley_soft_speed, wheelbase,
//!             max_steering_angle (README.md).
//!
//! No services, no calls. A 0.1 s timer on the node's clock is the watchdog. As the
//! Python, the PID and the states time themselves on the wall clock; the watchdog on the
//! node's (the simulator's under use_sim_time). The logic is the lane-follow-core crate.

use std::any::Any;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{bail, Context as _, Result};
use lane_follow_core::{
    Follower, Geometry, Law, LawName, Line, Pid, PidGains, PursuitSettings, Reply, Settings,
    StanleySettings,
};
use rclrs::*;
use ros_env::aav_lane_geometry::msg::LaneGeometry;
use ros_env::geometry_msgs::msg::TwistStamped;
use ros_env::std_msgs::msg::{Header, String as Text};

/// twist_mux's timeout for the AI slot (lane_speed.py's AI_SLOT_TIMEOUT_S).
const AI_SLOT_TIMEOUT_S: f64 = 0.5;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("control_interface")?;
    let _keep = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The node's parameters, declared read-only; their handles kept alive.
struct Params<'a> {
    node: &'a Node,
    keep: Vec<Box<dyn Any>>,
}

impl Params<'_> {
    fn value(&mut self, name: &str, default: ParameterValue) -> Result<ParameterValue> {
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name}"))?;
        let value = p.get();
        self.keep.push(Box::new(p));
        Ok(value)
    }

    /// A number, float or whole.
    fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        match self.value(name, ParameterValue::Double(default))? {
            ParameterValue::Double(v) => Ok(v),
            ParameterValue::Integer(v) => Ok(v as f64),
            other => bail!("{name}:={other:?}: not a number"),
        }
    }

    fn flag(&mut self, name: &str, default: bool) -> Result<bool> {
        match self.value(name, ParameterValue::Bool(default))? {
            ParameterValue::Bool(v) => Ok(v),
            other => bail!("{name}:={other:?}: not true or false"),
        }
    }

    fn text(&mut self, name: &str, default: &str) -> Result<String> {
        match self.value(name, ParameterValue::String(default.into()))? {
            ParameterValue::String(v) => Ok(v.to_string()),
            other => bail!("{name}:={other:?}: not a word"),
        }
    }
}

/// s since 1970 on the wall clock: Python's time.time().
fn wall() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

fn geometry(m: &LaneGeometry) -> Geometry {
    let line = |a: f32, b: f32, c: f32| Line {
        a: f64::from(a),
        b: f64::from(b),
        c: f64::from(c),
    };
    Geometry {
        offset: f64::from(m.vehicle_offset_m),
        mean_radius: f64::from(m.mean_curvature_m),
        lane_width: f64::from(m.lane_width_m),
        left_detected: m.left_lane_detected,
        right_detected: m.right_lane_detected,
        both_detected: m.both_lanes_detected,
        left: line(m.left_poly_a, m.left_poly_b, m.left_poly_c),
        right: line(m.right_poly_a, m.right_poly_b, m.right_poly_c),
    }
}

struct Driver {
    follower: Follower,
    clock: Clock,
    logger: Logger,
    cmd: Publisher<TwistStamped>,
    status: Publisher<Text>,
    state: Publisher<Text>,
}

impl Driver {
    /// The node's clock, s.
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_geometry(&mut self, msg: LaneGeometry) {
        let reply = self
            .follower
            .on_geometry(&geometry(&msg), wall(), self.now());
        self.publish(reply);
    }

    fn tick(&mut self) {
        let reply = self.follower.tick(wall(), self.now());
        self.publish(reply);
    }

    /// Publish and log what the follower decided, in the order it did.
    fn publish(&self, reply: Reply) {
        if let Some(quiet) = reply.quiet_for {
            log_warn!(
                self.logger.throttle(Duration::from_secs(2)),
                "No geometry message for {quiet:.1}s -> slowing down"
            );
        }
        for (old, new) in &reply.transitions {
            log_info!(&self.logger, "State: {old} -> {new}");
        }
        for c in &reply.commands {
            let mut out = TwistStamped {
                header: Header {
                    stamp: self.clock.now().to_ros_msg().unwrap_or_default(),
                    frame_id: "base_link".into(),
                },
                ..TwistStamped::default()
            };
            out.twist.linear.x = c.linear_x;
            out.twist.angular.z = c.angular_z;
            if let Err(e) = self.cmd.publish(out) {
                log_warn!(self.logger.throttle(Duration::from_secs(2)), "cmd_vel: {e}");
            }
        }
        if let Some(line) = reply.status {
            if let Err(e) = self.status.publish(Text { data: line }) {
                log_warn!(self.logger.throttle(Duration::from_secs(2)), "status: {e}");
            }
        }
        if let Some(state) = reply.state {
            let data = state.as_str().to_string();
            if let Err(e) = self.state.publish(Text { data }) {
                log_warn!(self.logger.throttle(Duration::from_secs(2)), "state: {e}");
            }
        }
    }
}

fn lock(d: &Mutex<Driver>) -> MutexGuard<'_, Driver> {
    d.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// The steering law the parameters ask for.
fn law(p: &mut Params<'_>, start: f64) -> Result<Law> {
    let gains = PidGains {
        kp: p.number("pid_kp", PidGains::default().kp)?,
        ki: p.number("pid_ki", PidGains::default().ki)?,
        kd: p.number("pid_kd", PidGains::default().kd)?,
    };
    let invert_offset_sign = p.flag("invert_offset_sign", true)?;
    let name: LawName = p.text("steering", LawName::Pid.as_str())?.parse()?;
    let lag = p.number("lag_s", PursuitSettings::default().lag)?;
    let d = PursuitSettings::default();
    let pursuit = PursuitSettings {
        lookahead: p.number("lookahead_m", d.lookahead)?,
        lookahead_gain: p.number("lookahead_gain_s", d.lookahead_gain)?,
        lookahead_min: p.number("lookahead_min_m", d.lookahead_min)?,
        lookahead_max: p.number("lookahead_max_m", d.lookahead_max)?,
        lag,
    };
    if !(pursuit.lookahead_min > 0.0 && pursuit.lookahead_min <= pursuit.lookahead_max) {
        bail!(
            "lookahead_min_m:={} and lookahead_max_m:={}: need 0 < min <= max",
            pursuit.lookahead_min,
            pursuit.lookahead_max
        );
    }
    let s = StanleySettings::default();
    let stanley = StanleySettings {
        gain: p.number("stanley_gain", s.gain)?,
        soft_speed: p.number("stanley_soft_speed", s.soft_speed)?,
        wheelbase: p.number("wheelbase", s.wheelbase)?,
        max_steer: p.number("max_steering_angle", s.max_steer)?,
        lag,
    };
    if !(stanley.wheelbase > 0.0 && stanley.soft_speed > 0.0) {
        bail!("wheelbase and stanley_soft_speed must be above 0");
    }
    if lag.is_nan() || lag < 0.0 {
        bail!("lag_s:={lag}: must be 0 or more");
    }
    Ok(match name {
        LawName::Pid => Law::Pid {
            pid: Pid::new(gains, start),
            invert_offset_sign,
        },
        LawName::PurePursuit => Law::PurePursuit(pursuit),
        LawName::Stanley => Law::Stanley(stanley),
    })
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut p = Params {
        node,
        keep: Vec::new(),
    };
    let d = Settings::default();
    let settings = Settings {
        base_speed: p.number("base_speed_mps", d.base_speed)?,
        max_angular_z: p.number("max_angular_z", d.max_angular_z)?,
        min_speed: p.number("min_speed_mps", d.min_speed)?,
        min_curvature: p.number("min_curvature_m", d.min_curvature)?,
        lane_change_rate_threshold: p
            .number("lane_change_rate_threshold", d.lane_change_rate_threshold)?,
        lane_change_offset_threshold: p.number(
            "lane_change_offset_threshold",
            d.lane_change_offset_threshold,
        )?,
        lane_change_speed: p.number("lane_change_speed", d.lane_change_speed)?,
        lane_change_timeout: p.number("lane_change_timeout_s", d.lane_change_timeout)?,
        stop_wait: p.number("stop_wait_s", d.stop_wait)?,
        decel_rate: p.number("decel_rate", d.decel_rate)?,
        accel_rate: p.number("accel_rate", d.accel_rate)?,
    };
    let clock = node.get_clock();
    let started = wall();
    let law = law(&mut p, started)?;
    let invert = matches!(
        law,
        Law::Pid {
            invert_offset_sign: true,
            ..
        }
    );
    let logger = node.logger().clone();
    log_info!(
        &logger,
        "Control interface ready. Subscribing to /perception/lane/geometry and publishing \
         /drivers/ai/cmd_vel"
    );
    match law.name() {
        LawName::Pid => log_info!(&logger, "invert_offset_sign={}", py_bool(invert)),
        other => log_info!(
            &logger,
            "steering={other} (a challenger to the 2025-26 PID)"
        ),
    }
    if settings.stop_wait >= AI_SLOT_TIMEOUT_S - 0.1 {
        log_warn!(
            &logger,
            "stop_wait_s={} is not below twist_mux's {AI_SLOT_TIMEOUT_S} s for the AI slot: \
             when lane messages stop, ego stops the car first, not this node's slow-down",
            settings.stop_wait
        );
    }
    let now = clock.now().nsec as f64 * 1e-9;
    let mut keep = p.keep;
    let driver = Arc::new(Mutex::new(Driver {
        follower: Follower::new(settings, law, started, now),
        clock,
        logger,
        cmd: node.create_publisher("/drivers/ai/cmd_vel".keep_last(10))?,
        status: node.create_publisher("/drivers/ai/lane_status".keep_last(10))?,
        state: node.create_publisher("/drivers/ai/lane_state".keep_last(10))?,
    }));
    let d = Arc::clone(&driver);
    keep.push(Box::new(node.create_subscription(
        "/perception/lane/geometry".keep_last(10),
        move |msg: LaneGeometry| lock(&d).on_geometry(msg),
    )?));
    let d = Arc::clone(&driver);
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_millis(100).node_time(),
        move || lock(&d).tick(),
    )?));
    keep.push(Box::new(driver));
    Ok(keep)
}

/// True or False, as the Python's log line writes a bool.
fn py_bool(v: bool) -> &'static str {
    if v {
        "True"
    } else {
        "False"
    }
}
