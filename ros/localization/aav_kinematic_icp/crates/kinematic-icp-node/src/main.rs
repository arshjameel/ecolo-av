//! kinematic_icp: lidar odometry that knows it is on a car (TODO 216), in Rust. A drop-in
//! for aav_kiss_icp's node: the same topics, frame and keys, so ekf_local's
//! `kiss_ekf:=true` file reads it unchanged. A challenger (`odom_lidar:=kinematic_icp`).
//! Never TF.
//!
//! Subscribes  /sensors/lidar/points          sensor_msgs/PointCloud2, reliable, keep last
//!                                            1: a scan that comes while one is matched
//!                                            replaces the one waiting
//!             /vehicle/odom                  nav_msgs/Odometry (the wheels; `wheel_topic`):
//!                                            each step's prior, its twist.linear.x and
//!                                            twist.angular.z while `wheel_timeout` fresh;
//!                                            none: the last step held (as KISS-ICP)
//! Publishes   /localization/kiss_icp/odom    nav_msgs/Odometry: frame kiss_odom (where
//!                                            base_link was at the first scan), child
//!                                            base_link; pose; twist in base_link (vx and
//!                                            vyaw; vy always 0: a car); the cloud's stamp.
//!                                            The twist's covariance from twist_linear_sd,
//!                                            twist_angular_sd; the pose's a fixed 1
//!             /localization/kiss_icp/status  diagnostic_msgs/DiagnosticStatus after each
//!                                            scan: OK (TRACKING) or ERROR (a cloud
//!                                            without x, y, z); match_ms, points, matched,
//!                                            threshold, wheels, clouds, clouds_dropped
//! Parameters  lidar_xyz, lidar_rpy (the mount, vehicle.yaml by the launch file),
//!             body_min, body_max (the car's outline in base_link, x y), min_height,
//!             min_range, max_range, voxel_size, max_points_per_voxel, lidar_period,
//!             twist_linear_sd (m/s), twist_angular_sd (rad/s), wheel_topic,
//!             wheel_timeout (s), wheel_rho_sd (m a step), wheel_theta_sd (rad a step).
//!             config/kinematic_icp.yaml.
//!
//! No serving, no calls. The maths is the `kinematic-icp` crate, with no ROS.

use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use kinematic_icp::{Odometry as Lidar, Point, Settings, Step};
use rclrs::*;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::Quaternion;
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::PointCloud2;
use ros_env::std_msgs::msg::Header;
use voxels::{
    cloud::{self, CloudView, Field},
    Mount,
};

const FRAME: &str = "kiss_odom";
/// The pose's variance: it drifts; ekf_local reads the twist.
const POSE_VARIANCE: f64 = 1.0;
const WARN_EVERY: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("kinematic_icp")?;
    let p = Params::declare(&node)?;
    let settings = p.settings();
    let mount = Mount::new(
        three("lidar_xyz", p.lidar_xyz.get())?,
        three("lidar_rpy", p.lidar_rpy.get())?,
    );
    let body = (
        two("body_min", p.body_min.get())?,
        two("body_max", p.body_max.get())?,
    );
    let wheel_topic = p.wheel_topic.get().to_string();
    log_info!(
        node.logger(),
        "kinematic_icp (Rust): {:.1}-{:.0} m, above {:.2} m, {:.2} m voxels; the wheels' \
         prior from {wheel_topic}",
        settings.min_range,
        settings.max_range,
        settings.min_height,
        settings.voxel_size
    );
    let state = State {
        lidar: Lidar::new(settings),
        mount,
        body,
        period: p.lidar_period.get(),
        twist_variance: (
            p.twist_linear_sd.get().powi(2),
            p.twist_angular_sd.get().powi(2),
        ),
        wheel_timeout: p.wheel_timeout.get(),
        wheels: None,
        last_stamp: None,
        clouds: 0,
        dropped: 0,
        logger: node.logger().clone(),
        odom: node.create_publisher("/localization/kiss_icp/odom".keep_last(10))?,
        status: node.create_publisher("/localization/kiss_icp/status".keep_last(10))?,
    };
    let worker = node.create_worker(state);
    let _cloud = worker.create_subscription(
        "/sensors/lidar/points".keep_last(1).reliable(),
        |s: &mut State, msg: PointCloud2| s.on_cloud(&msg),
    )?;
    let _wheels = worker.create_subscription(
        wheel_topic.as_str().keep_last(10),
        |s: &mut State, msg: Odometry| {
            let t = &msg.twist.twist;
            s.wheels = Some((seconds(&msg.header), t.linear.x, t.angular.z));
        },
    )?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

struct State {
    lidar: Lidar,
    mount: Mount,
    /// The car's outline in base_link: (x, y) from and to.
    body: ([f64; 2], [f64; 2]),
    period: f64,
    twist_variance: (f64, f64),
    wheel_timeout: f64,
    /// The wheels' newest: (stamp s, speed m/s, yaw rate rad/s).
    wheels: Option<(f64, f64, f64)>,
    last_stamp: Option<f64>,
    clouds: u64,
    dropped: u64,
    logger: Logger,
    odom: Publisher<Odometry>,
    status: Publisher<DiagnosticStatus>,
}

impl State {
    fn on_cloud(&mut self, msg: &PointCloud2) {
        let start = Instant::now();
        let stamp = seconds(&msg.header);
        if let Some(last) = self.last_stamp {
            let gap = ((stamp - last) / self.period).round() - 1.0;
            if gap >= 1.0 {
                self.dropped += gap as u64;
            }
        }
        self.last_stamp = Some(stamp);
        self.clouds += 1;
        let points = match read(msg) {
            Ok(points) => points,
            Err(e) => {
                self.publish_status(DiagnosticStatus::ERROR, &format!("ERROR: {e}"), &[]);
                return;
            }
        };
        let ([x0, y0], [x1, y1]) = self.body;
        let base: Vec<Point> = points
            .iter()
            .map(|p| self.mount.to_base(p))
            .filter(|p| !((x0..=x1).contains(&p.x) && (y0..=y1).contains(&p.y)))
            .map(|p| [p.x, p.y, p.z])
            .collect();
        let wheels = self
            .wheels
            .filter(|(t, _, _)| (stamp - t).abs() <= self.wheel_timeout)
            .map(|(_, v, w)| (v, w));
        let step = self.lidar.step(&base, stamp, wheels);
        if let Err(e) = self.odom.publish(self.odometry(&msg.header, &step)) {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "/localization/kiss_icp/odom: {e}"
            );
        }
        let ms = start.elapsed().as_secs_f64() * 1e3;
        let values = [
            ("match_ms", format!("{ms:.2}")),
            ("points", base.len().to_string()),
            ("matched", step.matched.to_string()),
            ("threshold", format!("{:.3}", step.threshold)),
            ("wheels", wheels.is_some().to_string()),
            ("clouds", self.clouds.to_string()),
            ("clouds_dropped", self.dropped.to_string()),
        ];
        self.publish_status(DiagnosticStatus::OK, "TRACKING", &values);
    }

    fn odometry(&self, header: &Header, step: &Step) -> Odometry {
        let mut msg = Odometry {
            header: Header {
                stamp: header.stamp.clone(),
                frame_id: FRAME.into(),
            },
            child_frame_id: "base_link".into(),
            ..Odometry::default()
        };
        let pose = &mut msg.pose.pose;
        pose.position.x = step.pose.x;
        pose.position.y = step.pose.y;
        let (s, c) = (step.pose.yaw / 2.0).sin_cos();
        pose.orientation = Quaternion {
            x: 0.0,
            y: 0.0,
            z: s,
            w: c,
        };
        for i in 0..6 {
            msg.pose.covariance[i * 7] = POSE_VARIANCE;
        }
        msg.twist.twist.linear.x = step.speed;
        msg.twist.twist.angular.z = step.yaw_rate;
        let (lin, ang) = self.twist_variance;
        for i in 0..3 {
            msg.twist.covariance[i * 7] = lin;
            msg.twist.covariance[(i + 3) * 7] = ang;
        }
        msg
    }

    fn publish_status(&self, level: u8, message: &str, values: &[(&str, String)]) {
        let status = DiagnosticStatus {
            level,
            name: "kinematic_icp".into(),
            message: message.into(),
            hardware_id: String::new(),
            values: values
                .iter()
                .map(|(k, v)| KeyValue {
                    key: (*k).into(),
                    value: v.clone(),
                })
                .collect(),
        };
        if let Err(e) = self.status.publish(status) {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "/localization/kiss_icp/status: {e}"
            );
        }
    }
}

fn seconds(header: &Header) -> f64 {
    f64::from(header.stamp.sec) + f64::from(header.stamp.nanosec) * 1e-9
}

/// The cloud's x, y, z, in the lidar's frame.
fn read(msg: &PointCloud2) -> Result<Vec<voxels::Point>, cloud::CloudError> {
    let view = CloudView {
        fields: msg
            .fields
            .iter()
            .map(|f| Field {
                name: &f.name,
                offset: f.offset,
                datatype: f.datatype,
                count: f.count,
            })
            .collect(),
        width: msg.width,
        height: msg.height,
        point_step: msg.point_step,
        row_step: msg.row_step,
        is_bigendian: msg.is_bigendian,
        data: &msg.data,
    };
    cloud::read_xyz(&view)
}

/// The node's parameters (kept alive with the node).
struct Params {
    lidar_xyz: MandatoryParameter<Arc<[f64]>>,
    lidar_rpy: MandatoryParameter<Arc<[f64]>>,
    body_min: MandatoryParameter<Arc<[f64]>>,
    body_max: MandatoryParameter<Arc<[f64]>>,
    min_height: ReadOnlyParameter<f64>,
    min_range: ReadOnlyParameter<f64>,
    max_range: ReadOnlyParameter<f64>,
    voxel_size: ReadOnlyParameter<f64>,
    max_points_per_voxel: ReadOnlyParameter<i64>,
    lidar_period: ReadOnlyParameter<f64>,
    twist_linear_sd: ReadOnlyParameter<f64>,
    twist_angular_sd: ReadOnlyParameter<f64>,
    wheel_topic: ReadOnlyParameter<Arc<str>>,
    wheel_timeout: ReadOnlyParameter<f64>,
    wheel_rho_sd: ReadOnlyParameter<f64>,
    wheel_theta_sd: ReadOnlyParameter<f64>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        let d = Settings::default();
        let number = |name: &str, default: f64| {
            node.declare_parameter(name)
                .default(default)
                .read_only()
                .with_context(|| format!("parameter {name}"))
        };
        let list = |name: &str| {
            node.declare_parameter::<Arc<[f64]>>(name)
                .mandatory()
                .with_context(|| format!("parameter {name}: the launch file sets it"))
        };
        Ok(Self {
            lidar_xyz: list("lidar_xyz")?,
            lidar_rpy: list("lidar_rpy")?,
            body_min: list("body_min")?,
            body_max: list("body_max")?,
            min_height: number("min_height", d.min_height)?,
            min_range: number("min_range", d.min_range)?,
            max_range: number("max_range", d.max_range)?,
            voxel_size: number("voxel_size", d.voxel_size)?,
            max_points_per_voxel: node
                .declare_parameter("max_points_per_voxel")
                .default(i64::try_from(d.max_points_per_voxel).unwrap_or(20))
                .read_only()
                .context("parameter max_points_per_voxel")?,
            lidar_period: number("lidar_period", 0.1)?,
            twist_linear_sd: number("twist_linear_sd", 0.15)?,
            twist_angular_sd: number("twist_angular_sd", 0.01)?,
            wheel_topic: node
                .declare_parameter("wheel_topic")
                .default(Arc::from("/vehicle/odom"))
                .read_only()
                .context("parameter wheel_topic")?,
            wheel_timeout: number("wheel_timeout", 0.5)?,
            wheel_rho_sd: number("wheel_rho_sd", d.wheel_rho_sd)?,
            wheel_theta_sd: number("wheel_theta_sd", d.wheel_theta_sd)?,
        })
    }

    fn settings(&self) -> Settings {
        Settings {
            min_height: self.min_height.get(),
            min_range: self.min_range.get(),
            max_range: self.max_range.get(),
            voxel_size: self.voxel_size.get(),
            max_points_per_voxel: usize::try_from(self.max_points_per_voxel.get()).unwrap_or(20),
            wheel_rho_sd: self.wheel_rho_sd.get(),
            wheel_theta_sd: self.wheel_theta_sd.get(),
            ..Settings::default()
        }
    }
}

fn three(name: &str, value: Arc<[f64]>) -> Result<[f64; 3]> {
    match *value {
        [a, b, c] if [a, b, c].iter().all(|v| v.is_finite()) => Ok([a, b, c]),
        _ => bail!("{name}:={value:?}: needs 3 numbers"),
    }
}

fn two(name: &str, value: Arc<[f64]>) -> Result<[f64; 2]> {
    match *value {
        [a, b] if a.is_finite() && b.is_finite() => Ok([a, b]),
        _ => bail!("{name}:={value:?}: needs 2 numbers"),
    }
}
