//! pose_check: NDT leads ekf_global's pose, the GPS checks it (TODO 210, a challenger:
//! `pose_source:=ndt`).
//!
//! Subscribes  localization/ndt/pose           geometry_msgs/PoseWithCovarianceStamped:
//!                                            NDT's good fits
//!             localization/ndt/status         diagnostic_msgs/DiagnosticStatus: TRACKING,
//!                                            WAITING or LOST, and score_per_point
//!             localization/gps/odom           nav_msgs/Odometry: each GPS fix on the map
//!                                            (navsat_transform)
//!             sensors/gps/fix                 sensor_msgs/NavSatFix: the receiver's own
//!                                            say (a fix or not, NaN)
//!             localization/amcl/pose          geometry_msgs/PoseWithCovarianceStamped:
//!                                            AMCL's, where it runs (a seed for NDT)
//!             localization/odom/global        nav_msgs/Odometry: the fused heading (a GPS
//!                                            seed's yaw)
//! Publishes   localization/pose_check/ndt_pose   PoseWithCovarianceStamped: NDT's fits,
//!                                            passed on while NDT is trusted (ekf_global's
//!                                            pose0)
//!             localization/pose_check/gps_odom   Odometry: the GPS fixes, passed on only
//!                                            while NDT isn't trusted and the GPS is
//!                                            (ekf_global's odom1)
//!             localization/pose_check/fix     Odometry: every absolute pose passed on, of
//!                                            either: the geofence's "a fix arrived"
//!             localization/pose_check/status  DiagnosticStatus, at rate_hz: name
//!                                            pose_check, OK (NDT, the GPS agrees), WARN
//!                                            (NDT with the GPS in doubt, or the GPS
//!                                            leading), ERROR (HOLD: neither trusted; the
//!                                            sensor watchdog's pose_check input stops the
//!                                            car); values source, ndt, gps, gap_m, nees,
//!                                            score, handovers, seeds
//!             localization/ndt/initial_pose   PoseWithCovarianceStamped: where a lost NDT
//!                                            starts again (AMCL, else the GPS)
//! Parameters  config/pose_check.yaml (pose-check-core's Settings), rate_hz
//!
//! Every hand-over and every seed is logged. The logic is pose-check-core, with no ROS.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use anyhow::{Context as _, Result};
use pose_check_core::{Checker, Cov2, NdtState, Pose2, Settings, Tick};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{
    Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion,
};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::NavSatFix;
use ros_env::std_msgs::msg::Header;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("pose_check")?;
    let _keep = start(&node)?;
    let commands = Arc::clone(executor.commands());
    let stopping = Arc::new(AtomicBool::new(false));
    let stop = Arc::clone(&stopping);
    ctrlc::set_handler(move || {
        stop.store(true, Ordering::Release);
        commands.halt_spinning();
    })?;
    // rcl_wait now and then answers RCL_RET_TIMEOUT with no timeout asked for (seen once
    // in 24 campus tours, 19 Nov: the node exited, and the watchdog held the car on its
    // silence). A safety input must not die of that: spin again.
    loop {
        let errors = executor.spin(SpinOptions::default());
        if stopping.load(Ordering::Acquire) || !context.ok() {
            return Ok(());
        }
        let timeouts = errors.iter().all(|e| {
            matches!(
                e,
                RclrsError::RclError {
                    code: RclReturnCode::Timeout,
                    ..
                }
            )
        });
        match errors.into_iter().next() {
            Some(e) if !timeouts => return Err(e.into()),
            Some(_) => log_warn!(node.logger(), "the executor timed out: spinning again"),
            None => return Ok(()),
        }
    }
}

struct State {
    checker: Checker,
    clock: Clock,
    logger: Logger,
    ndt_out: Publisher<PoseWithCovarianceStamped>,
    gps_out: Publisher<Odometry>,
    fix_out: Publisher<Odometry>,
    status: Publisher<DiagnosticStatus>,
    seed: Publisher<PoseWithCovarianceStamped>,
}

impl State {
    /// s on the node's clock (the simulator's with use_sim_time).
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_ndt(&mut self, msg: PoseWithCovarianceStamped) {
        let c = &msg.pose.covariance;
        let pass = self.checker.ndt_pose(
            seconds(&msg.header.stamp),
            pose2(&msg.pose.pose),
            Cov2::new(c[0], c[1], c[7]),
        );
        if pass {
            self.publish_fix(&msg.header, &msg.pose);
            self.sent(self.ndt_out.publish(msg), "ndt_pose");
        }
    }

    fn on_ndt_status(&mut self, msg: DiagnosticStatus) {
        let Some(state) = NdtState::from_message(&msg.message) else {
            return;
        };
        let score = msg
            .values
            .iter()
            .find(|kv| kv.key == "score_per_point")
            .and_then(|kv| kv.value.parse().ok());
        let now = self.now();
        self.checker.ndt_status(now, state, score);
    }

    fn on_gps(&mut self, msg: Odometry) {
        let c = &msg.pose.covariance;
        let p = &msg.pose.pose.position;
        let pass = self.checker.gps(
            seconds(&msg.header.stamp),
            p.x,
            p.y,
            Cov2::new(c[0], c[1], c[7]),
        );
        if pass {
            self.publish_fix(&msg.header, &msg.pose);
            self.sent(self.gps_out.publish(msg), "gps_odom");
        }
    }

    fn on_fix(&mut self, msg: NavSatFix) {
        let finite = msg.latitude.is_finite() && msg.longitude.is_finite();
        // sensor_msgs/NavSatStatus: -1 no fix; 0 a fix; 1, 2 augmented.
        self.checker.gps_fix(finite && msg.status.status >= 0);
    }

    fn on_amcl(&mut self, msg: PoseWithCovarianceStamped) {
        let c = &msg.pose.covariance;
        self.checker.amcl(
            seconds(&msg.header.stamp),
            pose2(&msg.pose.pose),
            Cov2::new(c[0], c[1], c[7]),
            c[35],
        );
    }

    fn publish_fix(&self, header: &Header, pose: &PoseWithCovariance) {
        let msg = Odometry {
            header: header.clone(),
            child_frame_id: "base_link".into(),
            pose: pose.clone(),
            ..Odometry::default()
        };
        self.sent(self.fix_out.publish(msg), "fix");
    }

    /// Logs a failed publish, at most every 2 s.
    fn sent(&self, result: Result<(), RclrsError>, what: &str) {
        if let Err(e) = result {
            log_warn!(self.logger.throttle(Duration::from_secs(2)), "{what}: {e}");
        }
    }

    fn tick(&mut self) {
        let now = self.now();
        if now <= 0.0 {
            return; // the simulator's clock hasn't started
        }
        let Tick {
            report,
            handover,
            seed,
        } = self.checker.tick(now);
        if let Some(h) = handover {
            log_warn!(&self.logger, "hand-over {} -> {}: {}", h.from, h.to, h.why);
        }
        if let Some(seed) = seed {
            log_warn!(
                &self.logger,
                "NDT seeded from {} at ({:.2}, {:.2}, {:.1} deg)",
                seed.from,
                seed.pose.x,
                seed.pose.y,
                seed.pose.yaw.to_degrees()
            );
            let mut msg = PoseWithCovarianceStamped::default();
            msg.header.stamp = time(now);
            msg.header.frame_id = "map".into();
            msg.pose.pose = pose_msg(&seed.pose);
            msg.pose.covariance[0] = seed.var_xy;
            msg.pose.covariance[7] = seed.var_xy;
            msg.pose.covariance[35] = seed.var_yaw;
            self.sent(self.seed.publish(msg), "initial_pose");
        }
        let number = |v: Option<f64>, digits: usize| {
            v.map_or_else(|| "-".to_string(), |v| format!("{v:.digits$}"))
        };
        let values = [
            ("source", report.source.to_string()),
            ("ndt", report.ndt.map_or("ok".into(), |d| d.to_string())),
            ("gps", report.gps.map_or("ok".into(), |d| d.to_string())),
            ("gap_m", number(report.gap, 2)),
            ("nees", number(report.nees, 1)),
            ("score", number(report.score, 2)),
            ("handovers", report.handovers.to_string()),
            ("seeds", report.seeds.to_string()),
        ];
        let msg = DiagnosticStatus {
            level: report.level.byte(),
            name: "pose_check".into(),
            message: report.message,
            hardware_id: String::new(),
            values: values
                .into_iter()
                .map(|(key, value)| KeyValue {
                    key: key.into(),
                    value,
                })
                .collect(),
        };
        self.sent(self.status.publish(msg), "status");
    }
}

fn lock(s: &Mutex<State>) -> MutexGuard<'_, State> {
    s.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// The parameters, read once: pose-check-core's Settings.
fn settings(node: &Node) -> Result<(Settings, f64)> {
    let d = Settings::default();
    let number = |name: &str, default: f64| -> Result<f64> {
        let p = node.declare_parameter(name).default(default).read_only()?;
        Ok(p.get())
    };
    let pairs = node
        .declare_parameter("min_pairs")
        .default(d.min_pairs as i64)
        .read_only()?
        .get();
    let s = Settings {
        ndt_max_age: number("ndt_max_age", d.ndt_max_age)?,
        trust_score: number("trust_score", d.trust_score)?,
        gps_max_age: number("gps_max_age", d.gps_max_age)?,
        gps_max_std: number("gps_max_std", d.gps_max_std)?,
        max_jump_speed: number("max_jump_speed", d.max_jump_speed)?,
        jump_margin: number("jump_margin", d.jump_margin)?,
        window: number("window", d.window)?,
        min_pairs: usize::try_from(pairs).context("min_pairs: a count")?,
        pair_max_dt: number("pair_max_dt", d.pair_max_dt)?,
        pair_var: number("pair_var", d.pair_var)?,
        disagree_nees: number("disagree_nees", d.disagree_nees)?,
        disagree_m: number("disagree_m", d.disagree_m)?,
        agree_m: number("agree_m", d.agree_m)?,
        amcl_max_age: number("amcl_max_age", d.amcl_max_age)?,
        amcl_max_std: number("amcl_max_std", d.amcl_max_std)?,
        amcl_max_yaw_std: number("amcl_max_yaw_std", d.amcl_max_yaw_std)?,
        reseed_every: number("reseed_every", d.reseed_every)?,
    };
    Ok((s, number("rate_hz", 10.0)?))
}

fn start(node: &Node) -> Result<Vec<Box<dyn std::any::Any>>> {
    let (settings, hz) = settings(node)?;
    if !(hz.is_finite() && hz > 0.0) {
        anyhow::bail!("rate_hz:={hz}: a rate must be above 0 Hz");
    }
    let checker = Checker::new(settings).context("config/pose_check.yaml")?;
    let logger = node.logger().clone();
    log_info!(
        &logger,
        "NDT leads, the GPS checks: trust_score {}, disagree {} m and d'S^-1d {}",
        settings.trust_score,
        settings.disagree_m,
        settings.disagree_nees
    );
    let state = Arc::new(Mutex::new(State {
        checker,
        clock: node.get_clock(),
        logger,
        ndt_out: node.create_publisher("localization/pose_check/ndt_pose")?,
        gps_out: node.create_publisher("localization/pose_check/gps_odom")?,
        fix_out: node.create_publisher("localization/pose_check/fix")?,
        status: node.create_publisher("localization/pose_check/status")?,
        seed: node.create_publisher("localization/ndt/initial_pose")?,
    }));
    let mut keep: Vec<Box<dyn std::any::Any>> = Vec::new();
    let s = Arc::clone(&state);
    keep.push(Box::new(node.create_subscription(
        "localization/ndt/pose",
        move |m: PoseWithCovarianceStamped| lock(&s).on_ndt(m),
    )?));
    let s = Arc::clone(&state);
    keep.push(Box::new(node.create_subscription(
        "localization/ndt/status",
        move |m: DiagnosticStatus| lock(&s).on_ndt_status(m),
    )?));
    let s = Arc::clone(&state);
    keep.push(Box::new(node.create_subscription(
        "localization/gps/odom",
        move |m: Odometry| lock(&s).on_gps(m),
    )?));
    let s = Arc::clone(&state);
    keep.push(Box::new(
        node.create_subscription("sensors/gps/fix", move |m: NavSatFix| lock(&s).on_fix(m))?,
    ));
    let s = Arc::clone(&state);
    keep.push(Box::new(node.create_subscription(
        "localization/amcl/pose",
        move |m: PoseWithCovarianceStamped| lock(&s).on_amcl(m),
    )?));
    let s = Arc::clone(&state);
    keep.push(Box::new(node.create_subscription(
        "localization/odom/global",
        move |m: Odometry| lock(&s).checker.heading(pose2(&m.pose.pose).yaw),
    )?));
    let s = Arc::clone(&state);
    // Wall time for the timer: it ticks before the simulator's clock starts too.
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_secs_f64(1.0 / hz),
        move || lock(&s).tick(),
    )?));
    keep.push(Box::new(state));
    Ok(keep)
}

/// A stamp in seconds.
fn seconds(stamp: &Time) -> f64 {
    f64::from(stamp.sec) + f64::from(stamp.nanosec) * 1e-9
}

fn time(s: f64) -> Time {
    let sec = s.floor();
    Time {
        sec: sec as i32,
        nanosec: ((s - sec) * 1e9) as u32,
    }
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
