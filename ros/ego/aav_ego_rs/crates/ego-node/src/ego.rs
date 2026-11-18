//! The node `ego`: messages in and out of ego-core (ego_node.py's twin).

use std::any::Any;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use ego_core::ackermann::{EgoModel, Limits, STEERING_RATE_RANGE};
use ego_core::car_model::{ActuatorConfig, SpeedLoopProfile};
use ego_core::dynamics::{describe, Body, Described, LogLine, PAYLOAD_RANGE};
use ego_core::lag_lead::{
    lead_wanted, CommandKey, LagAwareWatch, LagLead, LeadLimits, LeadMode, LeadTimes,
};
use ego_core::pedal::Pedal;
use ego_core::road_ahead::{allowed, cap, StopOnLine, Strip};
use ego_core::stop_ahead::{stop_point, MAX_TIME};
use ego_core::stop_law::{StopCurve, INPUT_DELAY};
use ego_core::trace::{trace, Twin};
use ego_core::{fmt_g, round_half_even};
use rclrs::*;
use ros_env::aav_interfaces::msg::RoadAhead;
use ros_env::ackermann_msgs::msg::AckermannDriveStamped;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::geometry_msgs::msg::{PoseStamped, Twist, TwistStamped};
use ros_env::nav_msgs::msg::{Odometry, Path};
use ros_env::std_msgs::msg::Bool;

use crate::params::{as_number, Declarer};

/// The program slots of twist_mux.yaml that might lead for the lag themselves.
const LAG_AWARE_DRIVERS: [&str; 4] = ["scripted", "point", "nav", "ai"];
/// The people's slots that drive with the pedal.
const PEDAL_DRIVERS: [&str; 3] = ["joy", "rc", "keyboard"];
/// ns: a vehicle/odom speed older than this is not a wheel speed for the pedal's shift.
const WHEEL_FRESH: i64 = 500_000_000;
/// The stop point's step (s), as stop_ahead.py's default.
const STOP_DT: f64 = 0.02;

/// What keeps the node running: its worker, subscriptions, timers and parameters.
pub struct Running {
    _keep: Vec<Box<dyn Any>>,
}

/// ns.
fn ns(t: &Time) -> i64 {
    i64::from(t.sec) * 1_000_000_000 + i64::from(t.nanosec)
}

/// A builtin_interfaces/Time from ns.
pub fn stamp(ns: i64) -> Time {
    Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

/// aav_description's vehicle.yaml, found on AMENT_PREFIX_PATH (as ament_index does).
fn installed_vehicle_yaml() -> Option<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .ok()?
        .split(':')
        .map(|prefix| PathBuf::from(prefix).join("share/aav_description/config/vehicle.yaml"))
        .find(|p| p.is_file())
}

fn quaternion_z_w(yaw: f64) -> (f64, f64) {
    ((yaw / 2.0).sin(), (yaw / 2.0).cos())
}

/// The road ahead's limit on the automatic drivers (road_limit.py).
struct RoadLimit {
    drivers: Vec<String>,
    pose_timeout: f64,
    curve: StopCurve,
    strip: Option<Strip>,
    pose: Option<(f64, f64)>,
    pose_time: Option<i64>,
    trimming: bool,
}

/// The live parameters, read each tick.
struct Live {
    steering_rate: MandatoryParameter<ParameterValue>,
    payload: MandatoryParameter<ParameterValue>,
    lag_lead: MandatoryParameter<Arc<str>>,
}

struct Ego {
    model: EgoModel,
    /// ns.
    timeout: i64,
    frame_id: String,
    sim: bool,
    cmd: Option<(f64, f64)>,
    cmd_key: Option<CommandKey>,
    /// ns, when the command arrived.
    cmd_time: i64,
    last_tick: Option<i64>,
    mode: LeadMode,
    lead: LagLead,
    lead_on: bool,
    watch: LagAwareWatch,
    pedal: Pedal,
    pedal_watch: LagAwareWatch,
    estop: bool,
    /// The last vehicle/odom speed, and when it came (ns).
    wheel: Option<(f64, i64)>,
    twin: Twin,
    trace_horizon: f64,
    trace_dt: f64,
    trace_every: usize,
    road: Option<RoadLimit>,
    live: Live,
    pub_cmd: Publisher<AckermannDriveStamped>,
    pub_trace: Publisher<Path>,
    pub_stop: Publisher<PoseStamped>,
    clock: Clock,
    logger: Logger,
}

pub fn start(node: &Node) -> Result<Running> {
    let mut d = Declarer::new(node);
    let def = Limits::default();
    let live_rate = d.live_number(
        "max_steering_rate",
        def.max_steering_rate,
        STEERING_RATE_RANGE,
    )?;
    let limits = Limits {
        wheelbase: d.number("wheelbase", def.wheelbase)?,
        max_steering_angle: d.number("max_steering_angle", def.max_steering_angle)?,
        max_speed: d.number("max_speed", def.max_speed)?,
        max_reverse_speed: d.number("max_reverse_speed", def.max_reverse_speed)?,
        max_acceleration: d.number("max_acceleration", def.max_acceleration)?,
        max_deceleration: d.number("max_deceleration", def.max_deceleration)?,
        max_steering_rate: as_number(&live_rate.get()).unwrap_or(def.max_steering_rate),
        min_speed_for_steering: d.number("min_speed_for_steering", def.min_speed_for_steering)?,
        max_jerk: d.number("max_jerk", def.max_jerk)?,
    };
    let b = Body::default();
    let live_payload = d.live_number("payload", b.payload, PAYLOAD_RANGE)?;
    let body = Body {
        mass: d.number("mass", b.mass)?,
        payload: as_number(&live_payload.get()).unwrap_or(b.payload),
        track: d.number("track", b.track)?,
        cg_height: d.number("cg_height", b.cg_height)?,
        tyre_grip: d.number("tyre_grip", b.tyre_grip)?,
        full_throttle_accel: d.number("full_throttle_accel", b.full_throttle_accel)?,
        full_brake_decel: d.number("full_brake_decel", b.full_brake_decel)?,
        max_lateral_accel: d.number("max_lateral_accel", b.max_lateral_accel)?,
    };
    let timeout = d.number("cmd_timeout", 0.3)?;
    let rate = d.number("rate_hz", 50.0)?;
    if rate <= 0.0 {
        bail!("rate_hz:={rate}: must be above 0");
    }
    let frame_id = d.text("frame_id", "base_link")?;
    let use_stamped = d.flag("use_stamped", true)?;

    // The lead and the trace.
    let live_mode = d.live_text("lag_lead", "auto", |v| {
        v.parse::<LeadMode>().map(|_| ()).map_err(|e| e.to_string())
    })?;
    let mode: LeadMode = live_mode.get().parse()?;
    let lead_limits = LeadLimits {
        filter_time: d.number("lag_lead_filter", 0.1)?,
        max_speed_lead: d.number("lag_lead_max_speed", 0.3)?,
        max_steer_lead: d.number("lag_lead_max_steer", 0.1)?,
    };
    let drivers = d.names("lag_aware_drivers", &LAG_AWARE_DRIVERS)?;
    let trace_horizon = d.number("trace_horizon", 2.0)?;
    let trace_rate = d.number("trace_rate_hz", 10.0)?;
    let profile_name = d.text("trace_speed_loop", "default")?;
    let profile: SpeedLoopProfile = profile_name
        .parse()
        .map_err(|e| anyhow::anyhow!("trace_speed_loop:={profile_name}: {e}"))?;
    let vehicle_yaml = d.text("vehicle_yaml", "")?;
    let path = if vehicle_yaml.is_empty() {
        installed_vehicle_yaml()
            .context("vehicle_yaml is empty and aav_description's vehicle.yaml isn't installed")?
    } else {
        PathBuf::from(vehicle_yaml)
    };
    let cfg = ActuatorConfig::from_vehicle_yaml(&path)
        .with_context(|| format!("vehicle_yaml {}", path.display()))?;
    let times = LeadTimes::from_config(&cfg);
    let watch = LagAwareWatch::new(&drivers);
    let loop_config = profile.config();
    let sim = node
        .use_undeclared_parameters()
        .get::<bool>("use_sim_time")
        .unwrap_or(false);
    let logger = node.logger().clone();
    log_info!(
        &logger,
        "lag_lead {mode} (on {}): leads {:.2} s speeding up, {:.2} s slowing, {:.2} s \
         steering, all guesses until tests 3.10-4.5; lag-aware drivers watched: {}",
        if sim { "a simulator" } else { "the wall clock" },
        times.speed_up,
        times.slow_down,
        times.steer,
        if drivers.is_empty() {
            "none".to_string()
        } else {
            drivers.join(", ")
        }
    );

    // The road ahead.
    let road_on = d.flag("road_ahead", false)?;
    let road_drivers = d.names("road_ahead_drivers", &["nav", "point"])?;
    let road_pose_timeout = d.number("road_pose_timeout", 1.0)?;
    let road = if road_on {
        let unwatched: Vec<&String> = road_drivers.iter().filter(|r| !watch.watches(r)).collect();
        if !unwatched.is_empty() {
            bail!(
                "road_ahead_drivers {unwatched:?} must also be in lag_aware_drivers: ego only \
                 knows whose command it is for those"
            );
        }
        let jerk = if limits.max_jerk > 0.0 {
            limits.max_jerk
        } else {
            1e6
        };
        let curve = StopCurve::from_car_model(
            &cfg,
            limits.max_deceleration,
            jerk,
            INPUT_DELAY,
            Some(&loop_config),
        )?;
        log_info!(
            &logger,
            "the road ahead limits {}: {}",
            road_drivers.join(", "),
            curve.describe()
        );
        Some(RoadLimit {
            drivers: road_drivers,
            pose_timeout: road_pose_timeout,
            curve,
            strip: None,
            pose: None,
            pose_time: None,
            trimming: false,
        })
    } else {
        None
    };

    // The pedal.
    let pedal_drivers = d.names("pedal_drivers", &PEDAL_DRIVERS)?;
    let p = Pedal::default();
    let pedal = Pedal {
        let_off: d.number("pedal_let_off", p.let_off)?,
        full: d.number("pedal_full", p.full)?,
        shift_hold: d.number("pedal_shift_hold", p.shift_hold)?,
        stop_speed: d.number("pedal_stop_speed", p.stop_speed)?,
    };
    if !pedal_drivers.is_empty() {
        log_info!(
            &logger,
            "the pedal for {}: let off slows at {} m/s^2, full back brakes at {} m/s^2 (a \
             full stick: {} m/s); the other way after {} s at a standstill (and the wheels \
             under {} m/s, when measured); off during an e-stop",
            pedal_drivers.join(", "),
            fmt_g(pedal.let_off),
            fmt_g(limits.max_deceleration),
            fmt_g(pedal.full),
            fmt_g(pedal.shift_hold),
            fmt_g(pedal.stop_speed)
        );
    }

    let trace_dt = 1.0 / rate;
    let trace_every = if trace_rate > 0.0 {
        round_half_even(rate / trace_rate).max(1.0) as usize
    } else {
        1
    };
    let ego = Ego {
        model: EgoModel::new(limits, Some(body)),
        timeout: (timeout * 1e9) as i64,
        frame_id,
        sim,
        cmd: None,
        cmd_key: None,
        cmd_time: 0,
        last_tick: None,
        mode,
        lead: LagLead::new(times, lead_limits),
        lead_on: false,
        watch,
        pedal,
        pedal_watch: LagAwareWatch::new(&pedal_drivers),
        estop: false,
        wheel: None,
        twin: Twin::new(cfg, loop_config),
        trace_horizon,
        trace_dt,
        trace_every,
        road,
        live: Live {
            steering_rate: live_rate,
            payload: live_payload,
            lag_lead: live_mode,
        },
        pub_cmd: node.create_publisher("ego/actuator_cmd")?,
        pub_trace: node.create_publisher("ego/trace")?,
        pub_stop: node.create_publisher("ego/stop_point")?,
        clock: node.get_clock(),
        logger: logger.clone(),
    };
    log_info!(
        &logger,
        "wheelbase {} m, max steering {} rad, max speed {} m/s (Rust)",
        limits.wheelbase,
        limits.max_steering_angle,
        limits.max_speed
    );
    ego.log_body();

    let mut keep: Vec<Box<dyn Any>> = Vec::new();
    let worker = node.create_worker(ego);
    if use_stamped {
        keep.push(Box::new(worker.create_subscription(
            "control/cmd_vel",
            |s: &mut Ego, msg: TwistStamped| s.on_stamped(&msg),
        )?));
    } else {
        keep.push(Box::new(worker.create_subscription(
            "control/cmd_vel",
            |s: &mut Ego, msg: Twist| s.on_twist(&msg),
        )?));
    }
    keep.push(Box::new(worker.create_subscription(
        "vehicle/odom",
        |s: &mut Ego, msg: Odometry| {
            let speed = msg.twist.twist.linear.x;
            if speed.is_finite() {
                s.twin.measured(speed);
                s.wheel = Some((speed, s.now()));
            }
        },
    )?));
    for driver in &drivers {
        let name = driver.clone();
        keep.push(Box::new(
            worker.create_subscription(
                format!("drivers/{driver}/lag_aware")
                    .as_str()
                    .keep_last(1)
                    .reliable()
                    .transient_local(),
                move |s: &mut Ego, msg: Bool| s.watch.set_aware(&name, msg.data),
            )?,
        ));
    }
    // One subscription per driver's cmd_vel, for both watches.
    let mut cmd_drivers: Vec<&String> = drivers.iter().collect();
    for p in &pedal_drivers {
        if !cmd_drivers.contains(&p) {
            cmd_drivers.push(p);
        }
    }
    for driver in cmd_drivers {
        let name = driver.clone();
        keep.push(Box::new(worker.create_subscription(
            format!("drivers/{driver}/cmd_vel").as_str(),
            move |s: &mut Ego, msg: TwistStamped| {
                let key = key_of(&msg);
                s.watch.sent(&name, key);
                s.pedal_watch.sent(&name, key);
            },
        )?));
    }
    // Reliable, as the backends' subscriptions: the HUD sends true five times.
    keep.push(Box::new(worker.create_subscription(
        "safety/estop",
        |s: &mut Ego, msg: Bool| s.estop = msg.data,
    )?));
    if road_on {
        keep.push(Box::new(worker.create_subscription(
            "world/road_ahead",
            |s: &mut Ego, msg: RoadAhead| {
                if let Some(r) = s.road.as_mut() {
                    r.strip = Some(strip_of(&msg));
                }
            },
        )?));
        keep.push(Box::new(worker.create_subscription(
            "localization/odom/global",
            |s: &mut Ego, msg: Odometry| {
                let now = s.now();
                if let Some(r) = s.road.as_mut() {
                    r.pose = Some((msg.pose.pose.position.x, msg.pose.pose.position.y));
                    r.pose_time = Some(now);
                }
            },
        )?));
    }
    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        |s: &mut Ego| s.tick(),
    )?));
    if trace_rate > 0.0 {
        keep.push(Box::new(worker.create_timer_repeating(
            Duration::from_secs_f64(1.0 / trace_rate).node_time(),
            |s: &mut Ego| s.publish_trace(),
        )?));
    }
    keep.push(Box::new(worker));
    keep.extend(d.finish());
    Ok(Running { _keep: keep })
}

fn key_of(msg: &TwistStamped) -> CommandKey {
    CommandKey {
        sec: msg.header.stamp.sec,
        nanosec: msg.header.stamp.nanosec,
        vx: msg.twist.linear.x,
        wz: msg.twist.angular.z,
    }
}

/// The road ahead a RoadAhead message carries (road_ahead_ros.from_msg), what ego reads.
pub fn strip_of(msg: &RoadAhead) -> Strip {
    let wide = |v: &[f32]| v.iter().map(|&x| f64::from(x)).collect::<Vec<f64>>();
    Strip {
        spacing: f64::from(msg.spacing),
        x: wide(&msg.x),
        y: wide(&msg.y),
        speed_limit: wide(&msg.speed_limit),
        stops: msg
            .stops
            .iter()
            .map(|s| StopOnLine {
                along: f64::from(s.along),
                reason: s.reason.clone(),
            })
            .collect(),
        end: f64::from(msg.end),
    }
}

impl Ego {
    fn now(&self) -> i64 {
        self.clock.now().nsec
    }

    fn log_body(&self) {
        let lim = &self.model.lim;
        let Some(body) = &self.model.body else {
            return;
        };
        let described = Described {
            wheelbase: lim.wheelbase,
            max_steering_angle: lim.max_steering_angle,
            max_speed: lim.max_speed,
            max_acceleration: lim.max_acceleration,
            max_deceleration: lim.max_deceleration,
            max_jerk: lim.max_jerk,
        };
        for line in describe(body, &described) {
            match line {
                LogLine::Info(s) => log_info!(&self.logger, "{s}"),
                LogLine::Warning(s) => log_warn!(&self.logger, "{s}"),
            }
        }
    }

    /// The live parameters: take a change made while running (checked when set).
    fn apply_live(&mut self) {
        if let Some(rate) = as_number(&self.live.steering_rate.get()) {
            if rate != self.model.lim.max_steering_rate {
                self.model.lim.max_steering_rate = rate;
                log_warn!(&self.logger, "max_steering_rate is now {rate} rad/s");
            }
        }
        if let Some(payload) = as_number(&self.live.payload.get()) {
            let changed = self
                .model
                .body
                .as_mut()
                .filter(|b| b.payload != payload)
                .map(|b| b.payload = payload)
                .is_some();
            if changed {
                self.log_body();
            }
        }
        if let Ok(mode) = self.live.lag_lead.get().parse::<LeadMode>() {
            if mode != self.mode {
                self.mode = mode;
                log_warn!(&self.logger, "lag_lead is now {mode}");
            }
        }
    }

    fn on_stamped(&mut self, msg: &TwistStamped) {
        let now = self.now();
        self.cmd_key = Some(key_of(msg));
        let t = ns(&msg.header.stamp);
        if t != 0 && (now - t).abs() as f64 * 1e-9 > 0.5 {
            log_warn!(
                self.logger.throttle(Duration::from_secs(10)),
                "cmd_vel stamp is more than 0.5 s from this clock: check chrony on every \
                 machine (or this is a replayed bag)"
            );
        }
        self.store(&msg.twist, now);
    }

    fn on_twist(&mut self, msg: &Twist) {
        // No stamp: never matched to a driver, so never lag-aware.
        self.cmd_key = None;
        let now = self.now();
        self.store(msg, now);
    }

    fn store(&mut self, twist: &Twist, arrived: i64) {
        self.cmd = Some((twist.linear.x, twist.angular.z));
        self.cmd_time = arrived;
    }

    /// The command ego obeys now, or `None`: silence means stop.
    fn fresh_cmd(&self, now: i64) -> Option<(f64, f64)> {
        self.cmd.filter(|_| now - self.cmd_time <= self.timeout)
    }

    /// The pedal if a pedal driver's fresh command drives and no e-stop.
    fn pedal_now(&self, cmd: Option<(f64, f64)>) -> Option<Pedal> {
        if cmd.is_none() || self.estop || self.pedal_watch.whose(self.cmd_key.as_ref()).is_none() {
            return None;
        }
        Some(self.pedal)
    }

    /// cmd, its forward speed trimmed to the road ahead if an automatic driver's.
    fn road_limit(&mut self, cmd: Option<(f64, f64)>, now: i64) -> Option<(f64, f64)> {
        let cmd = cmd?;
        let whose = self.watch.whose(self.cmd_key.as_ref()).map(str::to_string);
        let max_speed = self.model.lim.max_speed;
        let Some(road) = self.road.as_mut() else {
            return Some(cmd);
        };
        if !whose.is_some_and(|w| road.drivers.contains(&w)) {
            return Some(cmd);
        }
        let age = road
            .pose_time
            .map_or(f64::INFINITY, |t| (now - t) as f64 * 1e-9);
        let limit = allowed(
            road.strip.as_ref(),
            road.pose,
            age,
            road.pose_timeout,
            &road.curve,
            max_speed,
        );
        let out = cap(cmd, limit);
        let trimming = out != cmd;
        if trimming != road.trimming {
            if trimming {
                log_info!(
                    &self.logger,
                    "the road ahead now trims the automatic driver to {limit:.2} m/s"
                );
            } else {
                log_info!(
                    &self.logger,
                    "the road ahead no longer trims the automatic driver"
                );
            }
        }
        road.trimming = trimming;
        Some(out)
    }

    fn tick(&mut self) {
        self.apply_live();
        let now = self.now();
        let dt = self
            .last_tick
            .map_or(0.0, |last| (now - last) as f64 * 1e-9);
        self.last_tick = Some(now);
        let fresh = self.fresh_cmd(now);
        let cmd = self.road_limit(fresh, now);
        let aware = cmd
            .and_then(|_| self.watch.driving(self.cmd_key.as_ref()))
            .map(str::to_string);
        let lead_on = cmd.is_some() && lead_wanted(self.mode, self.sim, aware.is_some());
        if lead_on != self.lead_on && cmd.is_some() {
            log_info!(
                &self.logger,
                "the lag lead is now {}{}",
                if lead_on { "on" } else { "off" },
                aware.map_or(String::new(), |a| format!(" ({a} is lag-aware)"))
            );
        }
        self.lead_on = lead_on;
        if !lead_on {
            self.lead.reset();
        }
        let step = dt.min(0.1);
        self.model.pedal = self.pedal_now(cmd);
        self.model.wheel_speed = self
            .wheel
            .filter(|&(_, at)| now - at <= WHEEL_FRESH)
            .map(|(speed, _)| speed);
        let out = self
            .model
            .step(cmd, step, lead_on.then_some(&mut self.lead));
        // The trace's twin: each measured speed (vehicle/odom), then what ego sent.
        if let Err(e) = self.twin.sent(out.speed, out.steering, step) {
            log_warn!(
                self.logger.throttle(Duration::from_secs(5)),
                "the twin: {e}"
            );
        }

        let mut msg = AckermannDriveStamped::default();
        msg.header.stamp = stamp(now);
        msg.header.frame_id = self.frame_id.clone();
        msg.drive.speed = out.speed as f32;
        msg.drive.steering_angle = out.steering as f32;
        msg.drive.acceleration = out.accel as f32;
        msg.drive.steering_angle_velocity = self.model.lim.max_steering_rate as f32;
        if let Err(e) = self.pub_cmd.publish(msg) {
            log_error!(&self.logger, "ego/actuator_cmd: {e}");
        }
    }

    fn publish_trace(&mut self) {
        let now = self.now();
        let poses = match trace(
            &self.twin,
            &self.model,
            self.fresh_cmd(now),
            self.lead_on.then_some(&self.lead),
            self.trace_horizon,
            self.trace_dt,
            self.trace_every,
        ) {
            Ok(poses) => poses,
            Err(e) => {
                log_warn!(
                    self.logger.throttle(Duration::from_secs(5)),
                    "the trace: {e}"
                );
                return;
            }
        };
        let mut out = Path::default();
        out.header.stamp = stamp(now);
        out.header.frame_id = self.frame_id.clone();
        out.poses = poses
            .iter()
            .map(|p| {
                let mut ps = PoseStamped::default();
                ps.header.frame_id = self.frame_id.clone();
                // rclpy's Duration(seconds=t): whole nanoseconds, cut toward 0.
                ps.header.stamp = stamp(now + (p.t * 1e9) as i64);
                ps.pose.position.x = p.x;
                ps.pose.position.y = p.y;
                (ps.pose.orientation.z, ps.pose.orientation.w) = quaternion_z_w(p.yaw);
                ps
            })
            .collect();
        if let Err(e) = self.pub_trace.publish(out) {
            log_error!(&self.logger, "ego/trace: {e}");
        }
        match stop_point(&self.twin.cfg, &self.twin.state, STOP_DT, MAX_TIME) {
            Ok(stop) => {
                let mut ps = PoseStamped::default();
                ps.header.frame_id = self.frame_id.clone();
                ps.header.stamp = stamp(now);
                ps.pose.position.x = stop.x;
                ps.pose.position.y = stop.y;
                (ps.pose.orientation.z, ps.pose.orientation.w) = quaternion_z_w(stop.yaw);
                if let Err(e) = self.pub_stop.publish(ps) {
                    log_error!(&self.logger, "ego/stop_point: {e}");
                }
            }
            Err(e) => log_warn!(
                self.logger.throttle(Duration::from_secs(5)),
                "the stop point: {e}"
            ),
        }
    }
}
