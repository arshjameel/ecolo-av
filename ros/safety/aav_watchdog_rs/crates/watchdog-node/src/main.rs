//! sensor_watchdog in Rust: stops the automatic drivers when a sensor they need dies, lies
//! or freezes. A drop-in for aav_safety's sensor_watchdog.py: the same node name, topics,
//! QoS, parameters and answers. The logic is the `watchdog-core` crate, with no ROS.
//! **A safety lock**: the default since Ben, 16 Nov; `watchdog_impl:=python` falls back.
//!
//! Subscribes  each input's topic, as config/sensor_watchdog.yaml says (the launch file
//!             picks which: inputs), by its kind: odom nav_msgs/Odometry, imu
//!             sensor_msgs/Imu, gps sensor_msgs/NavSatFix, image sensor_msgs/Image
//!             (reliable, newest only), cloud sensor_msgs/PointCloud2 (best effort),
//!             heartbeat std_msgs/UInt64, detections vision_msgs/Detection2DArray (only
//!             its own frame_id's count), actuator aav_interfaces/ActuatorState, speed
//!             ackermann_msgs/AckermannDriveStamped (against /vehicle/odom's speed); node:
//!             our own nodes, looked for in the ROS graph each tick; status
//!             diagnostic_msgs/DiagnosticStatus: our nodes in their own words (FAULT)
//!             /vehicle/odom (the wheel speed) and /sensors/imu/data (the odom input's
//!             cross check), where no input watches them
//! Publishes   safety/sensor_watchdog/lock   std_msgs/Bool, at rate_hz: true while an input
//!                                           marked stop isn't OK (twist_mux's sensors lock)
//!             safety/sensor_watchdog/state  std_msgs/String, at rate_hz: OK, or STOP/WARN
//!                                           and what is wrong
//!             diagnostics                   diagnostic_msgs/DiagnosticArray, once a second
//! Serves      safety/sensor_watchdog/clear  std_srvs/Trigger: forget the latched faults
//!
//! Parameters  inputs, rate_hz (10), and each input's, as the Python (README).
//! Stops cleanly on SIGINT, SIGTERM or SIGHUP.

mod params;

use std::any::Any;
use std::collections::HashSet;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use rclrs::*;
use ros_env::aav_interfaces::msg::ActuatorState;
use ros_env::ackermann_msgs::msg::AckermannDriveStamped;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::{Image, Imu, NavSatFix, NavSatStatus, PointCloud2};
use ros_env::std_msgs::msg::{Bool, Header, String as Text, UInt64};
use ros_env::std_srvs::srv::{Trigger, Trigger_Request, Trigger_Response};
use ros_env::vision_msgs::msg::Detection2DArray;
use watchdog_core::feed::{Feed, Said, IMU_TOPIC, VEHICLE_ODOM};
use watchdog_core::watch::{Kind, Rule, Severity, Watchdog};
use watchdog_core::wheels::CrossCheck;
use watchdog_core::{fmt_g, picture_digest};

use crate::params::Declarer;

struct Dog {
    feed: Feed,
    node: Node,
    pub_lock: Publisher<Bool>,
    pub_state: Publisher<Text>,
    pub_diag: Publisher<DiagnosticArray>,
    ticks: u64,
    diag_every: u64,
    started: Instant,
    clock: Clock,
    logger: Logger,
}

fn stamp(ns: i64) -> Time {
    Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("sensor_watchdog")?;
    let _running = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// Each input's rule from its parameters (`<name>.<key>`), with the Python's defaults.
fn read_rule(d: &mut Declarer<'_>, name: &str) -> Result<Rule> {
    let key = |k: &str| format!("{name}.{k}");
    let cross = if d.flag(&key("cross_check"), false)? {
        let c = CrossCheck::default();
        Some(CrossCheck {
            window: d.number(&key("cross_window"), c.window)?,
            speed_tolerance: d.number(&key("cross_speed_tolerance"), c.speed_tolerance)?,
            yaw_tolerance: d.number(&key("cross_yaw_tolerance"), c.yaw_tolerance)?,
            persist: d.number(&key("cross_persist"), c.persist)?,
            gps_window: d.number(&key("cross_gps_window"), c.gps_window)?,
            gps_tolerance: d.number(&key("cross_gps_tolerance"), c.gps_tolerance)?,
            ..c
        })
    } else {
        None
    };
    let kind_text = d.text(&key("kind"), "odom")?;
    let kind: Kind = kind_text.parse().map_err(|k| {
        anyhow::anyhow!(
            "{name}: kind {k:?} is not one of {:?}",
            Kind::ALL.map(Kind::as_str)
        )
    })?;
    let rule = Rule {
        name: name.to_string(),
        topic: d.text(&key("topic"), "")?,
        kind,
        max_age: d.number(&key("max_age"), 1.0)?,
        stop: d.flag(&key("stop"), true)?,
        frozen_after: d.number(&key("frozen_after"), 0.0)?,
        max_jump_speed: d.number(&key("max_jump_speed"), 0.0)?,
        jump_margin: d.number(&key("jump_margin"), 5.0)?,
        cross_check: cross,
        frame_id: d.text(&key("frame_id"), "")?,
        latch: d.flag(&key("latch"), false)?,
        latch_grace: d.number(&key("latch_grace"), 5.0)?,
        follow_tolerance: d.number(&key("follow_tolerance"), 0.0)?,
        follow_persist: d.number(&key("follow_persist"), 1.0)?,
        over_margin: d.number(&key("over_margin"), 0.0)?,
        over_warn: d.number(&key("over_warn"), 1.0)?,
        over_persist: d.number(&key("over_persist"), 2.0)?,
        over_slowing: d.number(&key("over_slowing"), 0.3)?,
        status_name: d.text(&key("status_name"), "")?,
        min_level: {
            let level = d.number(&key("min_level"), 2.0)?;
            Severity::from_min_level(level as i64)
                .filter(|_| level.fract() == 0.0)
                .ok_or_else(|| {
                    anyhow::anyhow!("{name}: min_level is 1 (WARN), 2 (ERROR) or 3 (STALE)")
                })?
        },
        quiet_ok: d.flag(&key("quiet_ok"), false)?,
    };
    Ok(rule.checked()?)
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut d = Declarer::new(node);
    let names = d.names("inputs", &["odom"])?;
    let rate = d.number("rate_hz", 10.0)?;
    if rate.is_nan() || rate <= 0.0 {
        anyhow::bail!("rate_hz:={rate}: must be above 0");
    }
    let rules = names
        .iter()
        .map(|n| read_rule(&mut d, n))
        .collect::<Result<Vec<_>>>()?;
    let logger = node.logger().clone();
    let watching: Vec<String> = rules
        .iter()
        .map(|r| {
            format!(
                "{} ({}, {} s, {})",
                r.name,
                r.topic,
                fmt_g(r.max_age),
                if r.stop { "stop" } else { "warn" }
            )
        })
        .collect();
    // Five ticks' time without one, on a steady clock: the watchdog itself was stopped,
    // so nothing latches for SETTLE s after (a STALE input still stops). 3 s was too
    // short on a busy computer: DDS took 7 s to bring the HUD's beats back after a 10 s
    // stop, and the HUD latched LOST for a stop that was the watchdog's own (17 Nov).
    const SETTLE: f64 = 10.0;
    let dog = Watchdog::new(rules.clone(), (5.0 / rate).max(0.5), SETTLE);
    let cross_checked = dog.cross_checked();
    let state = Dog {
        feed: Feed::new(dog),
        node: node.clone(),
        pub_lock: node.create_publisher("safety/sensor_watchdog/lock")?,
        pub_state: node.create_publisher("safety/sensor_watchdog/state")?,
        pub_diag: node.create_publisher("diagnostics")?,
        ticks: 0,
        diag_every: (rate.round_ties_even() as u64).max(1),
        started: Instant::now(),
        clock: node.get_clock(),
        logger: logger.clone(),
    };
    let worker = node.create_worker(state);
    let mut keep: Vec<Box<dyn Any>> = Vec::new();
    for rule in &rules {
        let name = rule.name.clone();
        let topic = rule.topic.as_str();
        match rule.kind {
            // No topic: each tick looks for it in the ROS graph.
            Kind::Node => {}
            Kind::Odom => keep.push(Box::new(worker.create_subscription(
                topic.keep_last(10),
                move |s: &mut Dog, m: Odometry| {
                    let (t, p) = (&m.twist.twist, &m.pose.pose.position);
                    let now = s.now();
                    s.feed
                        .odom(now, &name, [t.linear.x, t.linear.y, t.angular.z, p.x, p.y]);
                },
            )?)),
            Kind::Imu => keep.push(Box::new(worker.create_subscription(
                topic.keep_last(10),
                move |s: &mut Dog, m: Imu| {
                    let (w, a) = (&m.angular_velocity, &m.linear_acceleration);
                    let now = s.now();
                    s.feed.imu(now, &name, [w.x, w.y, w.z, a.x, a.y, a.z]);
                },
            )?)),
            Kind::Gps => keep.push(Box::new(worker.create_subscription(
                topic.keep_last(10),
                move |s: &mut Dog, m: NavSatFix| {
                    let at =
                        f64::from(m.header.stamp.sec) + f64::from(m.header.stamp.nanosec) * 1e-9;
                    let has_fix = m.status.status >= NavSatStatus::STATUS_FIX;
                    let now = s.now();
                    s.feed
                        .gps(now, &name, at, (m.latitude, m.longitude), has_fix);
                },
            )?)),
            Kind::Image => keep.push(Box::new(worker.create_subscription(
                topic.keep_last(1).reliable(),
                move |s: &mut Dog, m: Image| {
                    // A frozen camera repeats its whole picture: a sample of it is enough.
                    let now = s.now();
                    s.feed.image(now, &name, picture_digest(&m.data));
                },
            )?)),
            Kind::Cloud => keep.push(Box::new(worker.create_subscription(
                topic.sensor_data_qos(),
                move |s: &mut Dog, m: PointCloud2| {
                    // A frozen lidar repeats its whole cloud: a sample, as for a picture.
                    let now = s.now();
                    s.feed.cloud(now, &name, picture_digest(&m.data));
                },
            )?)),
            Kind::Heartbeat => keep.push(Box::new(worker.create_subscription(
                topic.keep_last(10),
                move |s: &mut Dog, m: UInt64| {
                    let now = s.now();
                    s.feed.heartbeat(now, &name, m.data);
                },
            )?)),
            Kind::Detections => keep.push(Box::new(worker.create_subscription(
                topic.keep_last(10),
                move |s: &mut Dog, m: Detection2DArray| {
                    // Every number in the boxes, for the NaN check.
                    let mut values = Vec::new();
                    for det in &m.detections {
                        let b = &det.bbox;
                        values.extend([
                            b.center.position.x,
                            b.center.position.y,
                            b.size_x,
                            b.size_y,
                        ]);
                        values.extend(det.results.iter().map(|r| r.hypothesis.score));
                    }
                    let now = s.now();
                    s.feed.detections(now, &name, &m.header.frame_id, values);
                },
            )?)),
            Kind::Actuator => keep.push(Box::new(worker.create_subscription(
                topic.keep_last(10),
                move |s: &mut Dog, m: ActuatorState| {
                    let now = s.now();
                    s.feed
                        .actuator(now, &name, f64::from(m.command), f64::from(m.measured));
                },
            )?)),
            Kind::Speed => keep.push(Box::new(worker.create_subscription(
                topic.keep_last(10),
                move |s: &mut Dog, m: AckermannDriveStamped| {
                    let now = s.now();
                    s.feed.speed(now, &name, f64::from(m.drive.speed));
                },
            )?)),
            Kind::Status => keep.push(Box::new(worker.create_subscription(
                topic.keep_last(10),
                move |s: &mut Dog, m: DiagnosticStatus| {
                    // One of our nodes in its own words (FAULT, TODO 171).
                    let now = s.now();
                    s.feed.status(now, &name, &m.name, m.level, &m.message);
                },
            )?)),
        }
    }
    if !rules.iter().any(|r| r.topic == VEHICLE_ODOM) {
        keep.push(Box::new(worker.create_subscription(
            VEHICLE_ODOM.keep_last(10),
            |s: &mut Dog, m: Odometry| {
                let now = s.now();
                s.feed.wheels(now, m.twist.twist.linear.x);
            },
        )?));
    }
    if cross_checked && !rules.iter().any(|r| r.topic == IMU_TOPIC) {
        keep.push(Box::new(worker.create_subscription(
            IMU_TOPIC.keep_last(10),
            |s: &mut Dog, m: Imu| {
                let now = s.now();
                s.feed
                    .imu_only(now, m.linear_acceleration.x, m.angular_velocity.z);
            },
        )?));
    }
    keep.push(Box::new(worker.create_service::<Trigger, _>(
        "safety/sensor_watchdog/clear",
        |s: &mut Dog, _: Trigger_Request| {
            s.feed.clear();
            let message = "latched GPS jumps, wheel disagreements, stuck steering, overspeed \
                           and lost inputs (the HUD) forgotten";
            log_warn!(&s.logger, "cleared by a person: {message}");
            Trigger_Response {
                success: true,
                message: message.into(),
            }
        },
    )?));
    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        |s: &mut Dog| s.tick(),
    )?));
    keep.push(Box::new(worker));
    keep.extend(d.finish());
    log_info!(&logger, "watching {}", watching.join(", "));
    Ok(keep)
}

impl Dog {
    /// s, this node's clock (the simulator's with use_sim_time).
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    /// Our nodes in the ROS graph, by full name.
    fn running(&self) -> HashSet<String> {
        if self.feed.node_inputs().is_empty() {
            return HashSet::new();
        }
        match self.node.get_node_names() {
            Ok(names) => names
                .into_iter()
                .map(|n| format!("{}/{}", n.namespace.trim_end_matches('/'), n.name))
                .collect(),
            Err(e) => {
                log_warn!(
                    self.logger.throttle(Duration::from_secs(10)),
                    "the ROS graph: {e}"
                );
                HashSet::new()
            }
        }
    }

    fn tick(&mut self) {
        let now = self.now();
        let running = self.running();
        let steady = self.started.elapsed().as_secs_f64();
        let tick = self.feed.tick(now, steady, &running);
        for said in &tick.said {
            match said {
                Said::Back(x) => log_info!(&self.logger, "{x}"),
                Said::Stops(x) => log_error!(&self.logger, "{x}"),
                Said::Warns(x) => log_warn!(&self.logger, "{x}"),
            }
        }
        if let Err(e) = self.pub_lock.publish(Bool { data: tick.locked }) {
            log_error!(&self.logger, "lock: {e}");
        }
        if let Err(e) = self.pub_state.publish(Text { data: tick.summary }) {
            log_error!(&self.logger, "state: {e}");
        }
        self.ticks += 1;
        if self.ticks.is_multiple_of(self.diag_every) {
            self.publish_diagnostics(&tick.states);
        }
    }

    fn publish_diagnostics(&mut self, states: &[watchdog_core::watch::Verdict]) {
        let header = Header {
            stamp: stamp(self.clock.now().nsec),
            frame_id: String::new(),
        };
        let now = self.now();
        let status = self
            .feed
            .diagnostics(now, states)
            .into_iter()
            .map(|s| DiagnosticStatus {
                level: s.level.byte(),
                name: s.name,
                message: s.message,
                hardware_id: s.hardware_id,
                values: s
                    .values
                    .into_iter()
                    .map(|(key, value)| KeyValue { key, value })
                    .collect(),
            })
            .collect();
        if let Err(e) = self.pub_diag.publish(DiagnosticArray { header, status }) {
            log_error!(&self.logger, "diagnostics: {e}");
        }
    }
}
