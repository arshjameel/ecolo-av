//! localization_health in Rust: warns when localization's inputs or outputs stop being
//! believable. A drop-in for aav_localization_health's localization_health.py: the same
//! node name, topics, QoS, parameters and answers. The checks are the `health-core`
//! crate, with no ROS. Warn only: it never stops the car or touches the EKFs.
//!
//! Subscribes  sensors/imu/data           sensor_msgs/Imu: the gyro's bias parked, gravity
//!             vehicle/odom               nav_msgs/Odometry: the wheels' speed
//!             vehicle/wheels             aav_interfaces/WheelSpeeds: the front wheels
//!             sensors/gps/fix            sensor_msgs/NavSatFix: fix, covariance
//!             localization/odom/local    nav_msgs/Odometry: ekf_local's output
//!             localization/odom/global   nav_msgs/Odometry: ekf_global's output
//!             /tf, /tf_static            tf2_msgs/TFMessage: base_link -> imu_link,
//!                                        gps_link; map -> base_link (as tf2's listener)
//! Publishes   localization/health        std_msgs/String, at rate_hz: OK, or WARN and
//!                                        every check that isn't OK
//!             diagnostics                diagnostic_msgs/DiagnosticArray, at rate_hz: one
//!                                        status per check, "localization_health: <check>"
//!
//! Parameters  rate_hz (1.0), and health-core's `Settings` by the same names
//!             (aav_localization_health's config/localization_health.yaml). The wheelbase
//!             and front track come from aav_description's vehicle.yaml.
//! Each change of state is logged once. Stops cleanly on SIGINT, SIGTERM or SIGHUP.

use std::any::Any;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use health_core::format_g;
use health_core::health::{summary, Check, Ekf, Health, Level, Settings, Status};
use health_core::tree::Tree;
use rclrs::*;
use ros_env::aav_interfaces::msg::WheelSpeeds;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::{Imu, NavSatFix};
use ros_env::std_msgs::msg::{Header, String as Text};
use ros_env::tf2_msgs::msg::TFMessage;
use serde::Deserialize;

/// The static frames base_link must reach.
const STATIC: [&str; 2] = ["imu_link", "gps_link"];

struct HealthNode {
    health: Health,
    tree: Tree,
    levels: HashMap<Check, Level>,
    pub_state: Publisher<Text>,
    pub_diag: Publisher<DiagnosticArray>,
    clock: Clock,
    logger: Logger,
}

#[derive(Deserialize)]
struct VehicleFile {
    vehicle: Vehicle,
}

#[derive(Deserialize)]
struct Vehicle {
    wheelbase: f64,
    track: f64,
}

/// aav_description's vehicle.yaml, found on AMENT_PREFIX_PATH (as ament_index does).
fn installed_vehicle_yaml() -> Option<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .ok()?
        .split(':')
        .map(|prefix| PathBuf::from(prefix).join("share/aav_description/config/vehicle.yaml"))
        .find(|p| p.is_file())
}

/// A read-only number parameter, as rclpy's: a float or a whole number.
fn number(node: &Node, keep: &mut Vec<Box<dyn Any>>, name: &str, default: f64) -> Result<f64> {
    let p = node
        .declare_parameter::<ParameterValue>(name)
        .default(ParameterValue::Double(default))
        .read_only()
        .with_context(|| format!("parameter {name} (a number)"))?;
    let value = p.get();
    keep.push(Box::new(p));
    match value {
        ParameterValue::Double(v) if v.is_finite() => Ok(v),
        ParameterValue::Integer(v) => Ok(v as f64),
        other => bail!("{name}:={other:?}: not a number"),
    }
}

fn seconds(t: &Time) -> f64 {
    f64::from(t.sec) + f64::from(t.nanosec) * 1e-9
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
    let node = executor.create_node("localization_health")?;
    let _running = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut keep: Vec<Box<dyn Any>> = Vec::new();
    let mut fields = Vec::new();
    for (name, default) in Settings::default().fields() {
        fields.push((name, number(node, &mut keep, name, default)?));
    }
    let settings = Settings::from_fields(&fields).context("the settings")?;
    let rate = number(node, &mut keep, "rate_hz", 1.0)?;
    if rate <= 0.0 {
        bail!("rate_hz:={rate}: must be more than 0");
    }
    let path =
        installed_vehicle_yaml().context("aav_description's vehicle.yaml isn't installed")?;
    let text = std::fs::read_to_string(&path).with_context(|| path.display().to_string())?;
    let car: VehicleFile =
        serde_yaml_ng::from_str(&text).with_context(|| path.display().to_string())?;
    let state = HealthNode {
        health: Health::new(car.vehicle.wheelbase, car.vehicle.track, settings),
        tree: Tree::new(),
        levels: HashMap::new(),
        pub_state: node.create_publisher("localization/health")?,
        pub_diag: node.create_publisher("diagnostics")?,
        clock: node.get_clock(),
        logger: node.logger().clone(),
    };
    let worker = node.create_worker(state);
    keep.push(Box::new(worker.create_subscription(
        "sensors/imu/data".keep_last(50),
        |s: &mut HealthNode, m: Imu| {
            let now = s.now();
            let a = &m.linear_acceleration;
            s.health.imu(
                now,
                seconds(&m.header.stamp),
                &m.header.frame_id,
                m.angular_velocity.z,
                [a.x, a.y, a.z],
            );
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "vehicle/odom".keep_last(50),
        |s: &mut HealthNode, m: Odometry| {
            let now = s.now();
            s.health.odom(
                now,
                seconds(&m.header.stamp),
                &m.header.frame_id,
                &m.child_frame_id,
                m.twist.twist.linear.x,
            );
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "vehicle/wheels".keep_last(50),
        |s: &mut HealthNode, m: WheelSpeeds| {
            let now = s.now();
            s.health.wheels(
                now,
                seconds(&m.header.stamp),
                &m.header.frame_id,
                f64::from(m.front_left),
                f64::from(m.front_right),
            );
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "sensors/gps/fix".keep_last(10),
        |s: &mut HealthNode, m: NavSatFix| {
            let now = s.now();
            s.health.gps_fix(
                now,
                seconds(&m.header.stamp),
                &m.header.frame_id,
                m.status.status,
                m.position_covariance_type,
                &m.position_covariance,
            );
        },
    )?));
    for (topic, which) in [
        ("localization/odom/local", Ekf::Local),
        ("localization/odom/global", Ekf::Global),
    ] {
        keep.push(Box::new(worker.create_subscription(
            topic.keep_last(10),
            move |s: &mut HealthNode, m: Odometry| {
                let now = s.now();
                let (p, t) = (&m.pose.pose.position, &m.twist.twist);
                s.health.ekf(
                    which,
                    now,
                    seconds(&m.header.stamp),
                    &m.header.frame_id,
                    &m.child_frame_id,
                    (p.x, p.y),
                    t.linear.x,
                    t.angular.z,
                );
            },
        )?));
    }
    // As tf2's listener: /tf keep last 100; /tf_static latched.
    keep.push(Box::new(worker.create_subscription(
        "/tf".keep_last(100).reliable(),
        |s: &mut HealthNode, m: TFMessage| s.on_tf(&m, false),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/tf_static".keep_last(100).reliable().transient_local(),
        |s: &mut HealthNode, m: TFMessage| s.on_tf(&m, true),
    )?));
    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        |s: &mut HealthNode| s.tick(),
    )?));
    keep.push(Box::new(worker));
    Ok(keep)
}

impl HealthNode {
    /// s, this node's clock (the simulator's with use_sim_time).
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_tf(&mut self, msg: &TFMessage, fixed: bool) {
        for t in &msg.transforms {
            self.tree.set(
                &t.header.frame_id,
                &t.child_frame_id,
                seconds(&t.header.stamp),
                fixed,
            );
        }
    }

    fn look_at_tf(&mut self) {
        let missing = STATIC
            .iter()
            .filter(|f| !self.tree.connected("base_link", f))
            .map(|f| (*f).to_string())
            .collect();
        let now = self.now();
        let age = self
            .tree
            .newest_common("map", "base_link")
            .map(|at| now - at);
        self.health.tf(missing, age);
    }

    fn tick(&mut self) {
        self.look_at_tf();
        let ns = self.clock.now().nsec;
        let statuses = self.health.report(self.now());
        if let Err(e) = self.pub_state.publish(Text {
            data: summary(&statuses),
        }) {
            log_error!(&self.logger, "localization/health: {e}");
        }
        for s in &statuses {
            let before = self.levels.insert(s.check, s.level).unwrap_or(Level::Ok);
            if before != s.level {
                let line = format!("{}: {}, {}", s.check.as_str(), s.level, s.message);
                if s.level == Level::Ok {
                    log_info!(&self.logger, "{line}");
                } else {
                    log_warn!(&self.logger, "{line}");
                }
            }
        }
        let array = DiagnosticArray {
            header: Header {
                stamp: stamp(ns),
                frame_id: String::new(),
            },
            status: statuses.iter().map(diagnostic).collect(),
        };
        if let Err(e) = self.pub_diag.publish(array) {
            log_error!(&self.logger, "diagnostics: {e}");
        }
    }
}

/// One check as a /diagnostics status (DiagnosticStatus: OK 0, WARN 1, STALE 3).
fn diagnostic(s: &Status) -> DiagnosticStatus {
    DiagnosticStatus {
        level: match s.level {
            Level::Ok => DiagnosticStatus::OK,
            Level::Warn => DiagnosticStatus::WARN,
            Level::Stale => DiagnosticStatus::STALE,
        },
        name: format!("localization_health: {}", s.check.as_str()),
        message: s.message.clone(),
        hardware_id: "localization".into(),
        values: s
            .values
            .iter()
            .map(|(key, v)| KeyValue {
                key: key.clone(),
                value: format_g(*v, 4),
            })
            .collect(),
    }
}
