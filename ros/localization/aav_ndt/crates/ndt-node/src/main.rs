//! ndt: where the car is on a saved lidar map, in Rust. A trial: nothing drives by it.
//!
//! Subscribes  sensors/lidar/points            sensor_msgs/PointCloud2, reliable, keep
//!                                             last 1: a cloud that comes in while one is
//!                                             being matched replaces the one waiting
//!             localization/odom/local          nav_msgs/Odometry: the local EKF, for the
//!                                             guess (a 1 s buffer, interpolated at each
//!                                             cloud's stamp)
//!             localization/odom/global         nav_msgs/Odometry: ekf_global, where it
//!                                             starts again after LOST (and the first pose,
//!                                             with start_from_global)
//!             localization/ndt/initial_pose    geometry_msgs/PoseWithCovarianceStamped:
//!                                             start again from here (map frame)
//! Publishes   localization/ndt/pose            geometry_msgs/PoseWithCovarianceStamped:
//!                                             base_link in the map, stamped with the
//!                                             cloud's stamp; good fits only
//!             localization/ndt/status          diagnostic_msgs/DiagnosticStatus, after
//!                                             every cloud: TRACKING, WAITING or LOST,
//!                                             stamp (the cloud's, s), score_per_point,
//!                                             overlap, iterations, match_ms (wall clock),
//!                                             points, poor_in_row, restarts,
//!                                             clouds_dropped, x, y, yaw, and cov_xx,
//!                                             cov_xy, cov_xyaw, cov_yy, cov_yyaw,
//!                                             cov_yawyaw (-H⁻¹, before k and the floors)
//! Parameters  map (a PCD file), cubes (m, biggest first), lidar_xyz, lidar_rpy (vehicle.
//!             yaml's lidar_link), body_min, body_max (the car and its sensors in base_link,
//!             vehicle.self_box: 15 cm wider than them),
//!             ground (above or lowest), min_height, ground_cell, max_range, thin (m),
//!             min_score_per_point, max_correction_m, max_correction_deg, lost_after, max_iterations, step_m, step_deg,
//!             outlier_ratio, covariance_scale, variance_floor_xy (m²),
//!             variance_floor_yaw (rad²), start_x, start_y, start_yaw, start_from_global,
//!             cloud_period (s); config/ndt.yaml and launch/ndt.launch.py
//!
//! Never TF. The maths is `ndt-core`, with no ROS; this file only turns messages into
//! calls. The design: docs/designs/2026-10-29_ndt.md.

use std::{
    fs::File,
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use ndt_core::{
    cloud::{read_xyz, BodyBox, CloudView, Field},
    localizer::{raw_covariance, Outcome, Poor, Skip},
    pcd, Ground, Localizer, LocalizerSettings, MapSettings, MatchSettings, Matcher, Mount, Pose2,
    Report, ScanFilter, Tracking,
};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{Pose, PoseWithCovarianceStamped, Quaternion};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::PointCloud2;
use ros_env::std_msgs::msg::Header;

/// How often a repeated warning may be logged.
const WARN_EVERY: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("ndt")?;
    let params = Params::declare(&node)?;

    let map_path = params.map.get();
    let started = Instant::now();
    let map = pcd::read(File::open(&*map_path).with_context(|| format!("map:={map_path}"))?)
        .with_context(|| format!("map:={map_path}"))?;
    let cubes = params.cubes.get();
    let settings = params.match_settings();
    let matcher = Matcher::new(&map, &cubes, MapSettings::with_cube, settings)
        .with_context(|| format!("the cubes of {map_path}"))?;
    log_info!(
        node.logger(),
        "{map_path}: {} points; cubes (m, curves) {:?}; ready in {:.1} s",
        map.len(),
        matcher.levels().collect::<Vec<_>>(),
        started.elapsed().as_secs_f64()
    );
    let mut localizer = Localizer::new(matcher, params.localizer_settings()?);
    if !params.start_from_global.get() {
        let start = Pose2::new(
            params.start_x.get(),
            params.start_y.get(),
            params.start_yaw.get(),
        );
        localizer.set_start(start, None);
        log_info!(node.logger(), "starting from {start:?} (map frame)");
    }

    let state = State {
        localizer,
        period: seconds("cloud_period", params.cloud_period.get())?,
        last_cloud: None,
        dropped: 0,
        logger: node.logger().clone(),
        pose: node.create_publisher("localization/ndt/pose")?,
        status: node.create_publisher("localization/ndt/status")?,
    };
    let worker = node.create_worker(state);
    let _cloud = worker.create_subscription(
        "sensors/lidar/points".keep_last(1).reliable(),
        |s: &mut State, cloud: PointCloud2| s.on_cloud(&cloud),
    )?;
    let _local = worker.create_subscription(
        "localization/odom/local",
        |s: &mut State, odom: Odometry| {
            s.localizer
                .odom(duration(&odom.header.stamp), pose2(&odom.pose.pose));
        },
    )?;
    let _global = worker.create_subscription(
        "localization/odom/global",
        |s: &mut State, odom: Odometry| {
            s.localizer
                .global(duration(&odom.header.stamp), pose2(&odom.pose.pose));
        },
    )?;
    let _initial = worker.create_subscription(
        "localization/ndt/initial_pose",
        |s: &mut State, msg: PoseWithCovarianceStamped| {
            let start = pose2(&msg.pose.pose);
            log_info!(&s.logger, "starting again from {start:?}");
            s.localizer
                .set_start(start, Some(duration(&msg.header.stamp)));
        },
    )?;

    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The worker's payload.
struct State {
    localizer: Localizer,
    /// The lidar's period: a longer gap between clouds means some were dropped.
    period: Duration,
    last_cloud: Option<Duration>,
    dropped: u64,
    logger: Logger,
    pose: Publisher<PoseWithCovarianceStamped>,
    status: Publisher<DiagnosticStatus>,
}

impl State {
    fn on_cloud(&mut self, cloud: &PointCloud2) {
        let stamp = duration(&cloud.header.stamp);
        // Clouds skipped since the last: the ones replaced in the queue of 1 while we
        // matched (or lost on the way).
        if let Some(last) = self.last_cloud.filter(|last| stamp > *last) {
            let gap = (stamp - last).as_secs_f64() / self.period.as_secs_f64();
            self.dropped += (gap.round() as u64).saturating_sub(1);
        }
        self.last_cloud = Some(stamp);
        let view = CloudView {
            fields: cloud
                .fields
                .iter()
                .map(|f| Field {
                    name: &f.name,
                    offset: f.offset,
                    datatype: f.datatype,
                    count: f.count,
                })
                .collect(),
            width: cloud.width,
            height: cloud.height,
            point_step: cloud.point_step,
            row_step: cloud.row_step,
            is_bigendian: cloud.is_bigendian,
            data: &cloud.data,
        };
        let points = match read_xyz(&view) {
            Ok(points) => points,
            Err(e) => {
                log_warn!(
                    self.logger.throttle(WARN_EVERY),
                    "/sensors/lidar/points: {e}"
                );
                return;
            }
        };
        let started = Instant::now();
        let report = self.localizer.cloud(stamp, points);
        let match_ms = started.elapsed().as_secs_f64() * 1e3;
        if let Outcome::Pose(fit) = &report.outcome {
            let msg = PoseWithCovarianceStamped {
                header: Header {
                    stamp: cloud.header.stamp.clone(),
                    frame_id: "map".into(),
                },
                pose: ros_env::geometry_msgs::msg::PoseWithCovariance {
                    pose: pose_msg(&fit.pose),
                    covariance: self.localizer.covariance(fit),
                },
            };
            if let Err(e) = self.pose.publish(msg) {
                log_warn!(
                    self.logger.throttle(WARN_EVERY),
                    "/localization/ndt/pose: {e}"
                );
            }
        }
        if let Err(e) = self
            .status
            .publish(status(&report, stamp, match_ms, self.dropped))
        {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "/localization/ndt/status: {e}"
            );
        }
    }
}

/// The status after one cloud.
fn status(report: &Report, stamp: Duration, match_ms: f64, dropped: u64) -> DiagnosticStatus {
    let (level, why, fit) = match &report.outcome {
        Outcome::Pose(fit) => (DiagnosticStatus::OK, String::new(), Some(fit)),
        Outcome::Poor { fit, why } => (
            DiagnosticStatus::WARN,
            match why {
                Poor::LowScore => ": a poor fit (score per point too low)".into(),
                Poor::NoConvergence => ": a poor fit (out of iterations)".into(),
                Poor::Jumped => ": a poor fit (it jumped from the guess)".into(),
            },
            Some(fit),
        ),
        Outcome::Skipped(Skip::NoStart) => {
            (DiagnosticStatus::WARN, ": no start pose yet".into(), None)
        }
        Outcome::Skipped(Skip::Failed(e)) => (DiagnosticStatus::WARN, format!(": {e}"), None),
    };
    let level = if report.tracking == Tracking::Lost {
        DiagnosticStatus::ERROR
    } else {
        level
    };
    let mut values = vec![
        ("stamp", format!("{:.3}", stamp.as_secs_f64())),
        ("points", report.points.to_string()),
        ("poor_in_row", report.poor_in_row.to_string()),
        ("restarts", report.restarts.to_string()),
        ("clouds_dropped", dropped.to_string()),
        ("match_ms", format!("{match_ms:.2}")),
    ];
    if let Some(fit) = fit {
        values.extend([
            ("score_per_point", format!("{:.3}", fit.score_per_point)),
            ("overlap", format!("{:.3}", fit.overlap)),
            ("iterations", fit.iterations.to_string()),
            ("x", format!("{:.3}", fit.pose.x)),
            ("y", format!("{:.3}", fit.pose.y)),
            ("yaw", format!("{:.4}", fit.pose.yaw)),
        ]);
        // -H⁻¹ before k and the floors: x, y, yaw's (co)variances, to fit k (NEES).
        if let Some(c) = raw_covariance(&fit.hessian) {
            let at = |r: usize, k: usize| format!("{:.3e}", c[(r, k)]);
            values.extend([
                ("cov_xx", at(0, 0)),
                ("cov_xy", at(0, 1)),
                ("cov_xyaw", at(0, 2)),
                ("cov_yy", at(1, 1)),
                ("cov_yyaw", at(1, 2)),
                ("cov_yawyaw", at(2, 2)),
            ]);
        }
    }
    DiagnosticStatus {
        level,
        name: "ndt".into(),
        message: format!("{}{why}", report.tracking.as_str()),
        hardware_id: String::new(),
        values: values
            .into_iter()
            .map(|(key, value)| KeyValue {
                key: key.into(),
                value,
            })
            .collect(),
    }
}

/// A message's stamp, as time since the clock's zero (Gazebo's, in sim).
fn duration(stamp: &Time) -> Duration {
    Duration::new(u64::try_from(stamp.sec).unwrap_or(0), stamp.nanosec)
}

/// x, y and yaw of a pose (height, roll and pitch dropped).
fn pose2(pose: &Pose) -> Pose2 {
    let q = &pose.orientation;
    let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    Pose2::new(pose.position.x, pose.position.y, yaw)
}

fn pose_msg(p: &Pose2) -> Pose {
    let mut pose = Pose::default();
    pose.position.x = p.x;
    pose.position.y = p.y;
    let (s, c) = (p.yaw / 2.0).sin_cos();
    pose.orientation = Quaternion {
        x: 0.0,
        y: 0.0,
        z: s,
        w: c,
    };
    pose
}

fn seconds(name: &str, value: f64) -> Result<Duration> {
    if value.is_nan() || value <= 0.0 {
        bail!("{name}:={value}: must be above 0 s");
    }
    Duration::try_from_secs_f64(value).with_context(|| format!("{name}:={value} (seconds)"))
}

/// The node's parameters: config/ndt.yaml, and the car's measurements from vehicle.yaml
/// (the launch file passes them).
///
/// Keep this alive as long as the node: rclrs undeclares a parameter when its handle is
/// dropped. Read-only, because they are read once.
struct Params {
    map: ReadOnlyParameter<Arc<str>>,
    cubes: ReadOnlyParameter<Arc<[f64]>>,
    lidar_xyz: ReadOnlyParameter<Arc<[f64]>>,
    lidar_rpy: ReadOnlyParameter<Arc<[f64]>>,
    body_min: ReadOnlyParameter<Arc<[f64]>>,
    body_max: ReadOnlyParameter<Arc<[f64]>>,
    ground: ReadOnlyParameter<Arc<str>>,
    min_height: ReadOnlyParameter<f64>,
    ground_cell: ReadOnlyParameter<f64>,
    max_range: ReadOnlyParameter<f64>,
    thin: ReadOnlyParameter<f64>,
    min_score_per_point: ReadOnlyParameter<f64>,
    lost_after: ReadOnlyParameter<i64>,
    max_iterations: ReadOnlyParameter<i64>,
    step_m: ReadOnlyParameter<f64>,
    step_deg: ReadOnlyParameter<f64>,
    outlier_ratio: ReadOnlyParameter<f64>,
    max_correction_m: ReadOnlyParameter<f64>,
    max_correction_deg: ReadOnlyParameter<f64>,
    covariance_scale: ReadOnlyParameter<f64>,
    variance_floor_xy: ReadOnlyParameter<f64>,
    variance_floor_yaw: ReadOnlyParameter<f64>,
    start_x: ReadOnlyParameter<f64>,
    start_y: ReadOnlyParameter<f64>,
    start_yaw: ReadOnlyParameter<f64>,
    start_from_global: ReadOnlyParameter<bool>,
    cloud_period: ReadOnlyParameter<f64>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        let number = |name: &str, default: f64| -> Result<ReadOnlyParameter<f64>> {
            Ok(node.declare_parameter(name).default(default).read_only()?)
        };
        let count = |name: &str, default: i64| -> Result<ReadOnlyParameter<i64>> {
            Ok(node.declare_parameter(name).default(default).read_only()?)
        };
        let list = |name: &str, default: &[f64]| -> Result<ReadOnlyParameter<Arc<[f64]>>> {
            Ok(node
                .declare_parameter(name)
                .default(Arc::from(default))
                .read_only()?)
        };
        let filter = ScanFilter::default();
        let local = LocalizerSettings::default();
        let search = MatchSettings::default();
        Ok(Self {
            map: node
                .declare_parameter("map")
                .default("maps/ndt/test_lot.pcd".into())
                .read_only()?,
            cubes: list("cubes", &[4.0, 2.0, 1.0])?,
            lidar_xyz: list("lidar_xyz", &[0.9, 0.0, 1.8])?,
            lidar_rpy: list("lidar_rpy", &[0.0, 0.0, 0.0])?,
            body_min: list("body_min", &[-0.39, -0.65])?,
            body_max: list("body_max", &[1.99, 0.65])?,
            ground: node
                .declare_parameter("ground")
                .default("above".into())
                .read_only()?,
            min_height: number("min_height", 0.3)?,
            ground_cell: number("ground_cell", 1.0)?,
            max_range: number("max_range", filter.max_range)?,
            thin: number("thin", filter.thin)?,
            min_score_per_point: number("min_score_per_point", local.min_score_per_point)?,
            lost_after: count("lost_after", local.lost_after as i64)?,
            max_iterations: count("max_iterations", search.max_iterations as i64)?,
            step_m: number("step_m", search.step_m)?,
            step_deg: number("step_deg", search.step_rad.to_degrees())?,
            outlier_ratio: number("outlier_ratio", search.outlier_ratio)?,
            max_correction_m: number("max_correction_m", local.max_correction.0)?,
            max_correction_deg: number("max_correction_deg", local.max_correction.1.to_degrees())?,
            covariance_scale: number("covariance_scale", local.covariance_scale)?,
            variance_floor_xy: number("variance_floor_xy", local.variance_floor[0])?,
            variance_floor_yaw: number("variance_floor_yaw", local.variance_floor[1])?,
            start_x: number("start_x", 0.0)?,
            start_y: number("start_y", 0.0)?,
            start_yaw: number("start_yaw", 0.0)?,
            start_from_global: node
                .declare_parameter("start_from_global")
                .default(false)
                .read_only()?,
            cloud_period: number("cloud_period", 0.1)?,
        })
    }

    fn match_settings(&self) -> MatchSettings {
        MatchSettings {
            max_iterations: usize::try_from(self.max_iterations.get())
                .unwrap_or(1)
                .max(1),
            step_m: self.step_m.get(),
            step_rad: self.step_deg.get().to_radians(),
            outlier_ratio: self.outlier_ratio.get(),
            ..MatchSettings::default()
        }
    }

    fn localizer_settings(&self) -> Result<LocalizerSettings> {
        let three = |name: &str, v: Arc<[f64]>| -> Result<[f64; 3]> {
            <[f64; 3]>::try_from(&*v).map_err(|_| anyhow::anyhow!("{name}: give 3 numbers"))
        };
        let two = |name: &str, v: Arc<[f64]>| -> Result<[f64; 2]> {
            <[f64; 2]>::try_from(&*v).map_err(|_| anyhow::anyhow!("{name}: give 2 numbers"))
        };
        let ground = match &*self.ground.get() {
            "above" => Ground::Above(self.min_height.get()),
            "lowest" => Ground::AboveLowest {
                cell: self.ground_cell.get(),
                height: self.min_height.get(),
            },
            other => bail!("ground:={other}: use above (flat, Gazebo) or lowest (the car)"),
        };
        let lost_after = usize::try_from(self.lost_after.get())
            .ok()
            .filter(|n| *n > 0)
            .context("lost_after: a count above 0")?;
        Ok(LocalizerSettings {
            filter: ScanFilter {
                ground,
                body: Some(BodyBox {
                    min: two("body_min", self.body_min.get())?,
                    max: two("body_max", self.body_max.get())?,
                }),
                max_range: self.max_range.get(),
                thin: self.thin.get(),
            },
            mount: Mount::from_xyz_rpy(
                three("lidar_xyz", self.lidar_xyz.get())?,
                three("lidar_rpy", self.lidar_rpy.get())?,
            ),
            min_score_per_point: self.min_score_per_point.get(),
            lost_after,
            start_from_global: self.start_from_global.get(),
            max_correction: (
                self.max_correction_m.get(),
                self.max_correction_deg.get().to_radians(),
            ),
            covariance_scale: self.covariance_scale.get(),
            variance_floor: [self.variance_floor_xy.get(), self.variance_floor_yaw.get()],
            ..LocalizerSettings::default()
        })
    }
}
