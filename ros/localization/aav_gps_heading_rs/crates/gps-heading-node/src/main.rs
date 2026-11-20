//! gps_heading, in Rust: the car's heading from the GPS receiver's velocity, for the
//! global EKF. The same node as aav_gps_heading's gps_heading.py, topic for topic.
//!
//! Subscribes  sensors/gps/fix_velocity   geometry_msgs/TwistWithCovarianceStamped: the
//!                                        antenna's velocity, ENU, m/s (the receiver's
//!                                        Doppler)
//!             vehicle/odom               nav_msgs/Odometry: the wheels' signed speed and
//!                                        the turn rate
//! Publishes   localization/gps/heading   sensor_msgs/Imu, in base_link: only an
//!                                        orientation (the heading) and its variance; no
//!                                        turn rate or acceleration (covariance -1, REP-145)
//! Parameters  min_speed (0.3 m/s), max_speed_gap (0.3 m/s), floor_deg (2.0): the
//!             gps-heading-core crate's CourseSettings. vehicle_yaml ('': aav_description's
//!             vehicle.yaml, on AMENT_PREFIX_PATH): where the antenna sits (gps_link).
//!
//! Nothing is published while the car is slower than min_speed.

use std::any::Any;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use gps_heading_core::{degrees, heading, quaternion_z, radians, CourseSettings};
use rclrs::*;
use ros_env::geometry_msgs::msg::{Quaternion, TwistWithCovarianceStamped};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::Imu;
use ros_env::std_msgs::msg::Header;

/// rad^2: roll and pitch, which this says nothing about (the EKF is 2D).
const LOOSE: f64 = 1.0;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("gps_heading")?;
    let _keep = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

struct Heading {
    settings: CourseSettings,
    antenna: (f64, f64),
    wheels: Option<(f64, f64)>,
    out: Publisher<Imu>,
    logger: Logger,
}

impl Heading {
    fn on_velocity(&self, msg: TwistWithCovarianceStamped) {
        let Some((speed, turn)) = self.wheels else {
            return;
        };
        let v = &msg.twist.twist.linear;
        let cov = &msg.twist.covariance;
        let Some((yaw, variance)) = heading(
            v.x,
            v.y,
            cov[0],
            cov[7],
            speed,
            turn,
            self.antenna,
            &self.settings,
        ) else {
            return;
        };
        let (x, y, z, w) = quaternion_z(yaw);
        let mut imu = Imu {
            header: Header {
                stamp: msg.header.stamp,
                frame_id: "base_link".to_string(),
            },
            orientation: Quaternion { x, y, z, w },
            ..Imu::default()
        };
        imu.orientation_covariance[0] = LOOSE;
        imu.orientation_covariance[4] = LOOSE;
        imu.orientation_covariance[8] = variance;
        imu.angular_velocity_covariance[0] = -1.0;
        imu.linear_acceleration_covariance[0] = -1.0;
        if let Err(e) = self.out.publish(imu) {
            log_warn!(
                self.logger.throttle(Duration::from_secs(2)),
                "localization/gps/heading: {e}"
            );
        }
    }
}

fn lock(h: &Mutex<Heading>) -> MutexGuard<'_, Heading> {
    h.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// aav_description's vehicle.yaml, found on AMENT_PREFIX_PATH (as ament_index does).
fn installed_vehicle_yaml() -> Option<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .ok()?
        .split(':')
        .map(|prefix| PathBuf::from(prefix).join("share/aav_description/config/vehicle.yaml"))
        .find(|p| p.is_file())
}

/// The antenna's (x, y) from base_link: vehicle.yaml's sensors.gps_link.xyz.
fn antenna(path: &str) -> Result<(f64, f64)> {
    let path = if path.is_empty() {
        installed_vehicle_yaml()
            .context("vehicle_yaml is empty and aav_description's vehicle.yaml isn't installed")?
    } else {
        PathBuf::from(path)
    };
    let text = std::fs::read_to_string(&path).with_context(|| path.display().to_string())?;
    let yaml: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&text).with_context(|| path.display().to_string())?;
    let xyz = &yaml["sensors"]["gps_link"]["xyz"];
    let at = |i: usize| {
        xyz[i]
            .as_f64()
            .with_context(|| format!("{}: sensors.gps_link.xyz[{i}]", path.display()))
    };
    Ok((at(0)?, at(1)?))
}

fn number(node: &Node, keep: &mut Vec<Box<dyn Any>>, name: &str, default: f64) -> Result<f64> {
    let p = node
        .declare_parameter::<ParameterValue>(name)
        .default(ParameterValue::Double(default))
        .read_only()
        .with_context(|| format!("parameter {name}"))?;
    let value = match p.get() {
        ParameterValue::Double(v) => v,
        ParameterValue::Integer(v) => v as f64,
        other => bail!("{name}:={other:?}: not a number"),
    };
    keep.push(Box::new(p));
    Ok(value)
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut keep: Vec<Box<dyn Any>> = Vec::new();
    let d = CourseSettings::default();
    let settings = CourseSettings {
        min_speed: number(node, &mut keep, "min_speed", d.min_speed)?,
        max_speed_gap: number(node, &mut keep, "max_speed_gap", d.max_speed_gap)?,
        // As the Python: the default is degrees(radians(2.0)), read back as radians.
        floor: radians(number(node, &mut keep, "floor_deg", degrees(d.floor))?),
    };
    let p = node
        .declare_parameter::<ParameterValue>("vehicle_yaml")
        .default(ParameterValue::String("".into()))
        .read_only()?;
    let path = match p.get() {
        ParameterValue::String(s) => s.to_string(),
        other => bail!("vehicle_yaml:={other:?}: not a path"),
    };
    keep.push(Box::new(p));
    let antenna = antenna(&path)?;
    log_info!(
        node.logger(),
        "gps_heading: a heading above {} m/s, the antenna at ({}, {}) from base_link",
        settings.min_speed,
        antenna.0,
        antenna.1
    );
    let me = Arc::new(Mutex::new(Heading {
        settings,
        antenna,
        wheels: None,
        out: node.create_publisher("localization/gps/heading")?,
        logger: node.logger().clone(),
    }));
    let m = Arc::clone(&me);
    keep.push(Box::new(node.create_subscription(
        "vehicle/odom",
        move |msg: Odometry| {
            lock(&m).wheels = Some((msg.twist.twist.linear.x, msg.twist.twist.angular.z));
        },
    )?));
    let m = Arc::clone(&me);
    keep.push(Box::new(node.create_subscription(
        "sensors/gps/fix_velocity",
        move |msg: TwistWithCovarianceStamped| lock(&m).on_velocity(msg),
    )?));
    keep.push(Box::new(me));
    Ok(keep)
}
