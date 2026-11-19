//! amcl: AMCL in Rust, in stock nav2_amcl's place (`amcl_impl:=rust`). Its topics,
//! parameters and frames are the stock node's, so nothing downstream notices the swap.
//! Never TF.
//!
//! Subscribes  scan_topic (/sensors/lidar/scan)  sensor_msgs/LaserScan, sensor-data QoS,
//!                                               in base_frame_id (our scan is flattened
//!                                               into base_link; another frame is refused)
//!             map_topic (map)                   nav_msgs/OccupancyGrid, latched: the map
//!                                               (first_map_only: the first one only)
//!             initialpose                       geometry_msgs/PoseWithCovarianceStamped:
//!                                               start again here (global_frame_id); the
//!                                               launch file remaps it to
//!                                               /localization/amcl/initial_pose
//!             odom_topic (/localization/odom/local)
//!                                               nav_msgs/Odometry: odom -> base_link. The
//!                                               stock node reads that from TF; ekf_local
//!                                               publishes both, the same numbers
//! Publishes   amcl_pose                         geometry_msgs/PoseWithCovarianceStamped,
//!                                               latched (keep last 1): base_link in the
//!                                               map, the scan's stamp, after each resample
//!                                               (remapped to /localization/amcl/pose)
//!             particle_cloud                    nav2_msgs/ParticleCloud, sensor-data QoS,
//!                                               after each update (remapped to
//!                                               /localization/amcl/particles)
//! Parameters  nav2_amcl's that apply (config/amcl.yaml in aav_navigation): alpha1-4,
//!             min_particles, max_particles, pf_err, pf_z, recovery_alpha_slow,
//!             recovery_alpha_fast, update_min_d, update_min_a, resample_interval,
//!             z_hit, z_rand, sigma_hit, laser_likelihood_max_dist, max_beams,
//!             laser_min_range, laser_max_range, transform_tolerance (here: how much
//!             older than a scan the odometry may be), set_initial_pose,
//!             initial_pose.{x,y,z,yaw}, global_frame_id, odom_frame_id, base_frame_id,
//!             scan_topic, map_topic, first_map_only; read and checked only:
//!             robot_model_type (differential), laser_model_type (likelihood_field),
//!             tf_broadcast (true is refused); read and unused: alpha5, save_pose_rate.
//!             Ours: odom_topic, seed (0: from the clock).
//!
//! Not a lifecycle node: it starts working when the map comes. The maths is `amcl-core`,
//! with no ROS; this file only turns messages into calls.

use std::{
    sync::Arc,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use amcl_core::{
    Amcl, AmclSettings, DifferentialMotion, DistanceField, Estimate, FilterSettings, FreeSpace,
    LikelihoodField, Metres, OccupancyMap, Pose2, Radians, Scan, Step,
};
use anyhow::{bail, Context as _, Result};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::geometry_msgs::msg::{
    Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion,
};
use ros_env::nav2_msgs::msg::{Particle as ParticleMsg, ParticleCloud};
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry};
use ros_env::sensor_msgs::msg::LaserScan;
use ros_env::std_msgs::msg::Header;

/// How often a repeated warning may be logged.
const WARN_EVERY: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("amcl")?;
    let params = Params::declare(&node)?;
    params.check()?;
    let settings = params.settings()?;
    let start = params.set_initial_pose.get().then(|| {
        let pose = Pose2::new(
            params.initial_x.get(),
            params.initial_y.get(),
            params.initial_yaw.get(),
        );
        // nav2's spread for a start from the parameters: 0.5 m, 15°.
        let yaw = (std::f64::consts::PI / 12.0).powi(2);
        (
            None,
            pose,
            [[0.25, 0.0, 0.0], [0.0, 0.25, 0.0], [0.0, 0.0, yaw]],
        )
    });
    let odom_topic = params.odom_topic.get();
    let scan_topic = params.scan_topic.get();
    let map_topic = params.map_topic.get();
    log_info!(
        node.logger(),
        "{} to {} particles, {} beams, sigma_hit {} m; scan {scan_topic}, map {map_topic}, \
         odometry {odom_topic}; never TF",
        settings.filter.min_particles,
        settings.filter.max_particles,
        params.max_beams.get(),
        params.sigma_hit.get()
    );

    let state = State {
        amcl: None,
        settings,
        model: ModelSettings {
            z_hit: params.z_hit.get(),
            z_rand: params.z_rand.get(),
            sigma_hit: params.sigma_hit.get(),
            max_dist: params.laser_likelihood_max_dist.get(),
            max_beams: usize::try_from(params.max_beams.get()).unwrap_or(0),
        },
        ranges: (params.laser_min_range.get(), params.laser_max_range.get()),
        start,
        first_map_only: params.first_map_only.get(),
        global_frame: params.global_frame_id.get(),
        odom_frame: params.odom_frame_id.get(),
        base_frame: params.base_frame_id.get(),
        odom_topic: odom_topic.clone(),
        logger: node.logger().clone(),
        pose: node.create_publisher("amcl_pose".keep_last(1).reliable().transient_local())?,
        particles: node.create_publisher("particle_cloud".sensor_data_qos())?,
    };
    let worker = node.create_worker(state);
    let _scan = worker.create_subscription(
        scan_topic.as_ref().sensor_data_qos(),
        |s: &mut State, msg: LaserScan| s.on_scan(&msg),
    )?;
    let _map = worker.create_subscription(
        map_topic.as_ref().keep_last(1).reliable().transient_local(),
        |s: &mut State, msg: OccupancyGrid| s.on_map(&msg),
    )?;
    let _initial = worker.create_subscription(
        "initialpose",
        |s: &mut State, msg: PoseWithCovarianceStamped| s.on_initial_pose(&msg),
    )?;
    let _odom = worker
        .create_subscription(odom_topic.as_ref(), |s: &mut State, msg: Odometry| {
            s.on_odom(&msg)
        })?;

    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The likelihood field's settings, kept to build it again for a new map.
struct ModelSettings {
    z_hit: f64,
    z_rand: f64,
    sigma_hit: f64,
    max_dist: f64,
    max_beams: usize,
}

/// A start pose: its stamp (None: now), the pose, x, y and yaw's covariance.
type Start = (Option<Duration>, Pose2, [[f64; 3]; 3]);

/// The worker's payload.
struct State {
    /// None until the first map.
    amcl: Option<Amcl<LikelihoodField>>,
    settings: AmclSettings,
    model: ModelSettings,
    /// laser_min_range, laser_max_range (m; 0 or below: the scan's own).
    ranges: (f64, f64),
    /// A start pose that came before the map.
    start: Option<Start>,
    first_map_only: bool,
    global_frame: Arc<str>,
    odom_frame: Arc<str>,
    base_frame: Arc<str>,
    odom_topic: Arc<str>,
    logger: Logger,
    pose: Publisher<PoseWithCovarianceStamped>,
    particles: Publisher<ParticleCloud>,
}

impl State {
    fn on_map(&mut self, msg: &OccupancyGrid) {
        if self.first_map_only && self.amcl.is_some() {
            return;
        }
        let info = &msg.info;
        let origin = pose2(&info.origin);
        let map = match OccupancyMap::from_occupancy(
            info.width as usize,
            info.height as usize,
            Metres(f64::from(info.resolution)),
            origin,
            &msg.data,
        ) {
            Ok(map) => map,
            Err(e) => {
                log_warn!(&self.logger, "the map: {e}");
                return;
            }
        };
        let m = &self.model;
        let field = DistanceField::new(&map, Metres(m.max_dist));
        let model = LikelihoodField::new(field, m.z_hit, m.z_rand, m.sigma_hit, m.max_beams);
        let free = FreeSpace::new(&map);
        log_info!(
            &self.logger,
            "the map: {} x {} cells of {} m, {} free",
            info.width,
            info.height,
            info.resolution,
            free.len()
        );
        match self.amcl.as_mut() {
            Some(amcl) => amcl.set_map(model, free),
            None => match Amcl::new(self.settings, model, free) {
                Ok(amcl) => self.amcl = Some(amcl),
                Err(e) => {
                    log_error!(&self.logger, "{e}");
                    return;
                }
            },
        }
        match self.start.take() {
            Some(start) => self.start_at(start),
            None => log_info!(
                &self.logger,
                "waiting for a start pose on initialpose (set_initial_pose is false)"
            ),
        }
    }

    fn on_initial_pose(&mut self, msg: &PoseWithCovarianceStamped) {
        let frame = msg.header.frame_id.trim_start_matches('/');
        if frame != &*self.global_frame {
            log_warn!(
                &self.logger,
                "ignoring a start pose in {frame}: it must be in {}",
                self.global_frame
            );
            return;
        }
        let c = &msg.pose.covariance;
        let covariance = [[c[0], c[1], 0.0], [c[6], c[7], 0.0], [0.0, 0.0, c[35]]];
        let start = (
            Some(duration(&msg.header.stamp)),
            pose2(&msg.pose.pose),
            covariance,
        );
        if self.amcl.is_some() {
            self.start_at(start);
        } else {
            self.start = Some(start);
        }
    }

    fn start_at(&mut self, (stamp, pose, covariance): Start) {
        if let Some(amcl) = self.amcl.as_mut() {
            log_info!(
                &self.logger,
                "starting at x {:.3}, y {:.3}, yaw {:.3}",
                pose.x,
                pose.y,
                pose.yaw
            );
            amcl.set_pose(stamp, pose, &covariance);
        }
    }

    fn on_odom(&mut self, msg: &Odometry) {
        let (frame, child) = (
            msg.header.frame_id.trim_start_matches('/'),
            msg.child_frame_id.trim_start_matches('/'),
        );
        if frame != &*self.odom_frame || child != &*self.base_frame {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "{}: {frame} -> {child}, not {} -> {}: ignored",
                self.odom_topic,
                self.odom_frame,
                self.base_frame
            );
            return;
        }
        if let Some(amcl) = self.amcl.as_mut() {
            amcl.odom(duration(&msg.header.stamp), pose2(&msg.pose.pose));
        }
    }

    fn on_scan(&mut self, msg: &LaserScan) {
        let Some(amcl) = self.amcl.as_mut() else {
            log_warn!(self.logger.throttle(WARN_EVERY), "waiting for the map");
            return;
        };
        let frame = msg.header.frame_id.trim_start_matches('/');
        if frame != &*self.base_frame {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "a scan in {frame}: this node reads scans in {} only",
                self.base_frame
            );
            return;
        }
        if msg.range_max <= 0.0 {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "a scan with range_max {}: ignored",
                msg.range_max
            );
            return;
        }
        let scan = Scan::from_ranges(
            &msg.ranges,
            f64::from(msg.angle_min),
            f64::from(msg.angle_increment),
            (f64::from(msg.range_min), f64::from(msg.range_max)),
            self.ranges,
            Pose2::default(),
        );
        let stamp = duration(&msg.header.stamp);
        match amcl.scan(stamp, &scan) {
            Step::NoStart => log_warn!(
                self.logger.throttle(WARN_EVERY),
                "no start pose yet: send one on initialpose"
            ),
            Step::NoOdometry => log_warn!(
                self.logger.throttle(WARN_EVERY),
                "no odometry at the scan's stamp on {}",
                self.odom_topic
            ),
            Step::Still => {}
            Step::Updated { pose, .. } => {
                let header = Header {
                    stamp: msg.header.stamp.clone(),
                    frame_id: self.global_frame.to_string(),
                };
                let cloud = ParticleCloud {
                    header: header.clone(),
                    particles: amcl
                        .particles()
                        .iter()
                        .map(|p| ParticleMsg {
                            pose: pose_msg(&p.pose),
                            weight: p.weight,
                        })
                        .collect(),
                };
                if let Err(e) = self.particles.publish(cloud) {
                    log_warn!(self.logger.throttle(WARN_EVERY), "particle_cloud: {e}");
                }
                if let Some(estimate) = pose {
                    self.publish_pose(header, &estimate);
                }
            }
        }
    }

    fn publish_pose(&self, header: Header, e: &Estimate) {
        let covariance = e.covariance_6x6();
        if covariance
            .iter()
            .chain([&e.pose.x, &e.pose.y])
            .any(|v| v.is_nan())
        {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "the pose or its covariance is NaN: not published"
            );
            return;
        }
        let msg = PoseWithCovarianceStamped {
            header,
            pose: PoseWithCovariance {
                pose: pose_msg(&e.pose),
                covariance,
            },
        };
        if let Err(e) = self.pose.publish(msg) {
            log_warn!(self.logger.throttle(WARN_EVERY), "amcl_pose: {e}");
        }
    }
}

/// A message's stamp, as time since the clock's zero (the simulator's, in sim).
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

/// The node's parameters: nav2_amcl's names and defaults, and two of ours.
///
/// Keep this alive as long as the node: rclrs undeclares a parameter when its handle is
/// dropped. Read-only, because they are read once.
struct Params {
    alpha: [ReadOnlyParameter<f64>; 4],
    _alpha5: ReadOnlyParameter<f64>,
    _save_pose_rate: ReadOnlyParameter<f64>,
    min_particles: ReadOnlyParameter<i64>,
    max_particles: ReadOnlyParameter<i64>,
    pf_err: ReadOnlyParameter<f64>,
    pf_z: ReadOnlyParameter<f64>,
    recovery_alpha_slow: ReadOnlyParameter<f64>,
    recovery_alpha_fast: ReadOnlyParameter<f64>,
    update_min_d: ReadOnlyParameter<f64>,
    update_min_a: ReadOnlyParameter<f64>,
    resample_interval: ReadOnlyParameter<i64>,
    z_hit: ReadOnlyParameter<f64>,
    z_rand: ReadOnlyParameter<f64>,
    sigma_hit: ReadOnlyParameter<f64>,
    laser_likelihood_max_dist: ReadOnlyParameter<f64>,
    max_beams: ReadOnlyParameter<i64>,
    laser_min_range: ReadOnlyParameter<f64>,
    laser_max_range: ReadOnlyParameter<f64>,
    transform_tolerance: ReadOnlyParameter<f64>,
    set_initial_pose: ReadOnlyParameter<bool>,
    initial_x: ReadOnlyParameter<f64>,
    initial_y: ReadOnlyParameter<f64>,
    _initial_z: ReadOnlyParameter<f64>,
    initial_yaw: ReadOnlyParameter<f64>,
    global_frame_id: ReadOnlyParameter<Arc<str>>,
    odom_frame_id: ReadOnlyParameter<Arc<str>>,
    base_frame_id: ReadOnlyParameter<Arc<str>>,
    scan_topic: ReadOnlyParameter<Arc<str>>,
    map_topic: ReadOnlyParameter<Arc<str>>,
    first_map_only: ReadOnlyParameter<bool>,
    robot_model_type: ReadOnlyParameter<Arc<str>>,
    laser_model_type: ReadOnlyParameter<Arc<str>>,
    tf_broadcast: ReadOnlyParameter<bool>,
    odom_topic: ReadOnlyParameter<Arc<str>>,
    seed: ReadOnlyParameter<i64>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        let number = |name: &str, default: f64| -> Result<ReadOnlyParameter<f64>> {
            Ok(node.declare_parameter(name).default(default).read_only()?)
        };
        let count = |name: &str, default: i64| -> Result<ReadOnlyParameter<i64>> {
            Ok(node.declare_parameter(name).default(default).read_only()?)
        };
        let flag = |name: &str, default: bool| -> Result<ReadOnlyParameter<bool>> {
            Ok(node.declare_parameter(name).default(default).read_only()?)
        };
        let text = |name: &str, default: &str| -> Result<ReadOnlyParameter<Arc<str>>> {
            Ok(node
                .declare_parameter(name)
                .default(Arc::from(default))
                .read_only()?)
        };
        Ok(Self {
            alpha: [
                number("alpha1", 0.2)?,
                number("alpha2", 0.2)?,
                number("alpha3", 0.2)?,
                number("alpha4", 0.2)?,
            ],
            _alpha5: number("alpha5", 0.2)?,
            _save_pose_rate: number("save_pose_rate", 0.5)?,
            min_particles: count("min_particles", 500)?,
            max_particles: count("max_particles", 2000)?,
            pf_err: number("pf_err", 0.05)?,
            pf_z: number("pf_z", 0.99)?,
            recovery_alpha_slow: number("recovery_alpha_slow", 0.0)?,
            recovery_alpha_fast: number("recovery_alpha_fast", 0.0)?,
            update_min_d: number("update_min_d", 0.25)?,
            update_min_a: number("update_min_a", 0.2)?,
            resample_interval: count("resample_interval", 1)?,
            z_hit: number("z_hit", 0.5)?,
            z_rand: number("z_rand", 0.5)?,
            sigma_hit: number("sigma_hit", 0.2)?,
            laser_likelihood_max_dist: number("laser_likelihood_max_dist", 2.0)?,
            max_beams: count("max_beams", 60)?,
            laser_min_range: number("laser_min_range", -1.0)?,
            laser_max_range: number("laser_max_range", 100.0)?,
            transform_tolerance: number("transform_tolerance", 1.0)?,
            set_initial_pose: flag("set_initial_pose", false)?,
            initial_x: number("initial_pose.x", 0.0)?,
            initial_y: number("initial_pose.y", 0.0)?,
            _initial_z: number("initial_pose.z", 0.0)?,
            initial_yaw: number("initial_pose.yaw", 0.0)?,
            global_frame_id: text("global_frame_id", "map")?,
            odom_frame_id: text("odom_frame_id", "odom")?,
            base_frame_id: text("base_frame_id", "base_footprint")?,
            scan_topic: text("scan_topic", "scan")?,
            map_topic: text("map_topic", "map")?,
            first_map_only: flag("first_map_only", false)?,
            robot_model_type: text("robot_model_type", "nav2_amcl::DifferentialMotionModel")?,
            laser_model_type: text("laser_model_type", "likelihood_field")?,
            tf_broadcast: flag("tf_broadcast", true)?,
            odom_topic: text("odom_topic", "/localization/odom/local")?,
            seed: count("seed", 0)?,
        })
    }

    /// Refuses what this node doesn't do: TF, another motion or laser model.
    fn check(&self) -> Result<()> {
        if self.tf_broadcast.get() {
            bail!(
                "tf_broadcast:=true: this node never publishes TF (ekf_global keeps \
                 map -> odom); set it false"
            );
        }
        let robot = self.robot_model_type.get();
        if !matches!(
            &*robot,
            "nav2_amcl::DifferentialMotionModel" | "differential"
        ) {
            bail!("robot_model_type:={robot}: only the differential model (a car can't slide)");
        }
        let laser = self.laser_model_type.get();
        if &*laser != "likelihood_field" {
            bail!("laser_model_type:={laser}: only likelihood_field");
        }
        if self.max_beams.get() < 2 {
            bail!("max_beams:={}: at least 2", self.max_beams.get());
        }
        Ok(())
    }

    fn settings(&self) -> Result<AmclSettings> {
        let size = |name: &str, v: i64| -> Result<usize> {
            usize::try_from(v).with_context(|| format!("{name}:={v}: not below 0"))
        };
        let seed = match self.seed.get() {
            0 => SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(1, |t| t.as_nanos() as u64),
            s => s as u64,
        };
        let tolerance = self.transform_tolerance.get();
        if !(tolerance >= 0.0 && tolerance.is_finite()) {
            bail!("transform_tolerance:={tolerance}: seconds, 0 or above");
        }
        let [a1, a2, a3, a4] = &self.alpha;
        let settings = AmclSettings {
            filter: FilterSettings {
                min_particles: size("min_particles", self.min_particles.get())?,
                max_particles: size("max_particles", self.max_particles.get())?,
                pf_err: self.pf_err.get(),
                pf_z: self.pf_z.get(),
                alpha_slow: self.recovery_alpha_slow.get(),
                alpha_fast: self.recovery_alpha_fast.get(),
            },
            motion: DifferentialMotion {
                alpha1: a1.get(),
                alpha2: a2.get(),
                alpha3: a3.get(),
                alpha4: a4.get(),
            },
            update_min_d: Metres(self.update_min_d.get()),
            update_min_a: Radians(self.update_min_a.get()),
            resample_interval: size("resample_interval", self.resample_interval.get())?,
            odom_tolerance: Duration::from_secs_f64(tolerance),
            seed,
        };
        settings.filter.check()?;
        if settings.resample_interval == 0 {
            bail!("resample_interval:=0: at least 1");
        }
        Ok(settings)
    }
}
