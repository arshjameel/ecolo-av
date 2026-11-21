//! voxels: the roof lidar's cloud as coarse cubes around base_link, in Rust. A drop-in
//! for aav_voxels' Python node: the same name, topics, QoS, frames, fields and status.
//!
//! Subscribes  /sensors/lidar/points       sensor_msgs/PointCloud2 (the lidar's driver; the
//!                                         simulator's), reliable, keep last 1: a cloud that
//!                                         comes while one is cut replaces the one waiting
//!             /localization/odom/local    nav_msgs/Odometry, only with steps:=true: where
//!                                         base_link is in odom, to keep the kerbs
//! Publishes   /perception/voxels/cubes    sensor_msgs/PointCloud2, base_link, x y z
//!                                         float32, reliable, depth 1: for every cube (size,
//!                                         0.25 m) holding a point between min_height and
//!                                         max_height above the fitted road, its point
//!                                         nearest base_link; the cloud's stamp
//!             /perception/voxels/ground   sensor_msgs/PointCloud2, base_link, the same QoS:
//!                                         the centre of every flat square where the lidar
//!                                         saw the road (free space: the costmap clears with it)
//!             /perception/voxels/steps    sensor_msgs/PointCloud2, base_link, the same QoS,
//!                                         only with steps:=true: the high side of every
//!                                         sudden step in the ground (a kerb), 0.1 m squares
//!                                         (voxels::steps, TODO 119), not the foot of
//!                                         anything taller than a kerb (voxels::kerbs_only,
//!                                         TODO 189), and those of the last
//!                                         step_keep s, kept in odom (voxels::StepMemory);
//!                                         those under every beam (nearer the lidar than
//!                                         its height / tan 15 degrees) while they stay
//!                                         there, however long (TODO 219)
//!             /perception/voxels/status   diagnostic_msgs/DiagnosticStatus, every cloud: ms,
//!                                         cubes, ground_cubes, points, fitted, roll_deg,
//!                                         pitch_deg, clouds_dropped (and steps, with
//!                                         steps:=true)
//! Parameters  size (m), min_height, max_height (m above the road), range_max (m from
//!             base_link): nav2.yaml's voxels: block, as the Python reads it. lidar_xyz (m),
//!             lidar_rpy (rad): vehicle.yaml's lidar_link; body ([x from, x to, half the
//!             width], m): aav_lidar scan.body_box. navigation.launch.py passes all three
//!             (voxels_impl:=rust); they have no defaults, so a missing one stops the node.
//!             steps (bool, false), step_min (m, 0.10), step_radius (m, 0.3), step_range
//!             (m, 12), step_keep (s, 10): the kerbs' step finder and how long it keeps
//!             each kerb (navigation.launch.py kerbs:=true). ground (plane, the default;
//!             patchwork: the ground a plane a patch, ring by ring, so a lower kerb is a
//!             cube: voxels::patchwork, TODO 215, a challenger; navigation.launch.py
//!             ground:=patchwork), patch_min_height (m, 0.15: its floor over a patch's own
//!             plane).
//!
//! No serving, no calls, never TF. The maths is the `voxels` crate, with no ROS.

use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use rclrs::*;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::{PointCloud2, PointField};
use ros_env::std_msgs::msg::Header;
use voxels::{
    cloud::{self, CloudView, Field},
    cubes, kerbs_only,
    patchwork::{self, Ground, PatchSettings},
    steps, usable, Blind, Body, CubeShape, Cubes, Mount, Point, Pose2, StepMemory, StepShape,
    LOWEST_BEAM,
};

/// The lidar's period (10 Hz): to count the clouds that came while one was cut.
const PERIOD: f64 = 0.1;
/// How often a repeated warning may be logged.
const WARN_EVERY: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("voxels")?;
    let params = Params::declare(&node)?;
    let (shape, mount, lidar_xyz) = params.read()?;
    let step_shape = params.read_steps()?;
    let step_keep = params.step_keep.get();
    let ground_finder = params.read_ground()?;
    log_info!(
        node.logger(),
        "voxels (Rust): {:.2} m cubes from {:.2} to {:.2} m above the fitted road, out to \
         {:.0} m; the lidar at {lidar_xyz:?}",
        shape.size,
        shape.min_height,
        shape.max_height,
        shape.range_max
    );
    if let Some(s) = &step_shape {
        log_info!(
            node.logger(),
            "voxels (Rust): kerbs on: steps over {:.2} m within {:.2} m, out to {:.0} m, kept \
             {step_keep:.0} s, on /perception/voxels/steps",
            s.min_step,
            s.radius,
            s.range_max
        );
    }

    // Reliable, depth 1: best-effort readers (the costmap's, the stop zone's) still
    // connect, and a Mac's UDP doesn't lose whole clouds.
    let state = State {
        shape,
        patches: ground_finder,
        mount,
        last_stamp: None,
        dropped: 0,
        logger: node.logger().clone(),
        cubes: node.create_publisher("/perception/voxels/cubes".keep_last(1).reliable())?,
        ground: node.create_publisher("/perception/voxels/ground".keep_last(1).reliable())?,
        status: node.create_publisher("/perception/voxels/status".keep_last(10).reliable())?,
        kerbs: match step_shape {
            Some(shape) => Some(Kerbs {
                // A kerb under every beam stays while it is there (TODO 219).
                memory: StepMemory::new(step_keep, shape.range_max, shape.cell)
                    .with_blind(Blind::under(lidar_xyz, LOWEST_BEAM)),
                shape,
                pose: None,
                publisher: node
                    .create_publisher("/perception/voxels/steps".keep_last(1).reliable())?,
            }),
            None => None,
        },
    };
    let kerbs = state.kerbs.is_some();
    let worker = node.create_worker(state);
    let _cloud = worker.create_subscription(
        "/sensors/lidar/points".keep_last(1).reliable(),
        |s: &mut State, msg: PointCloud2| s.on_cloud(&msg),
    )?;
    // Where the car is in odom, to keep the kerbs where they are as it drives.
    let _odom = if kerbs {
        Some(worker.create_subscription(
            "/localization/odom/local".keep_last(1),
            |s: &mut State, msg: Odometry| {
                if let Some(k) = s.kerbs.as_mut() {
                    k.pose = Some(pose_of(&msg));
                }
            },
        )?)
    } else {
        None
    };

    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The worker's payload.
struct State {
    shape: CubeShape,
    /// Patchwork's settings with ground:=patchwork (TODO 215); None: one plane.
    patches: Option<PatchSettings>,
    mount: Mount,
    /// The last cloud's stamp (s), to count the ones dropped since.
    last_stamp: Option<f64>,
    dropped: u64,
    logger: Logger,
    cubes: Publisher<PointCloud2>,
    ground: Publisher<PointCloud2>,
    status: Publisher<DiagnosticStatus>,
    /// The kerbs' step finder, its memory and publisher, with steps:=true.
    kerbs: Option<Kerbs>,
}

/// The kerbs: found in each cloud, kept in odom while the car drives up to them.
struct Kerbs {
    shape: StepShape,
    memory: StepMemory,
    /// The newest odometry: base_link in odom. None until it comes: then no memory.
    pose: Option<Pose2>,
    publisher: Publisher<PointCloud2>,
}

/// base_link in odom, flat, from an odometry message.
fn pose_of(msg: &Odometry) -> Pose2 {
    let p = &msg.pose.pose;
    let q = &p.orientation;
    let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    Pose2 {
        x: p.position.x,
        y: p.position.y,
        yaw,
    }
}

impl State {
    fn on_cloud(&mut self, msg: &PointCloud2) {
        let start = Instant::now();
        let stamp = f64::from(msg.header.stamp.sec) + f64::from(msg.header.stamp.nanosec) * 1e-9;
        if let Some(last) = self.last_stamp {
            // Keep-last-1: a gap of n lidar periods is n - 1 clouds dropped (as ndt counts).
            // Python's round(): halves to even.
            let gap = ((stamp - last) / PERIOD).round_ties_even();
            self.dropped += (gap.max(0.0) as u64).saturating_sub(1);
        }
        self.last_stamp = Some(stamp);
        let points = match read(msg) {
            Ok(points) => points,
            Err(e) => {
                log_warn!(
                    self.logger.throttle(WARN_EVERY),
                    "/sensors/lidar/points: {e}"
                );
                return;
            }
        };
        let base: Vec<Point> = points.iter().map(|p| self.mount.to_base(p)).collect();
        let found = match &self.patches {
            Some(patches) => patchwork::cubes(&base, &self.shape, patches),
            None => cubes(&base, &self.shape),
        };
        let header = Header {
            stamp: msg.header.stamp.clone(),
            frame_id: "base_link".into(),
        };
        for (publisher, points, topic) in [
            (&self.cubes, &found.obstacles, "/perception/voxels/cubes"),
            (&self.ground, &found.ground, "/perception/voxels/ground"),
        ] {
            if let Err(e) = publisher.publish(xyz32(&header, points)) {
                log_warn!(self.logger.throttle(WARN_EVERY), "{topic}: {e}");
            }
        }
        let mut kerb_points = None;
        if let Some(k) = self.kerbs.as_mut() {
            // The kerbs only: the foot of a person, a post or a wall is the cubes', and it
            // may walk away, so it must not be kept under every beam (TODO 189).
            let good = usable(&base, &self.shape);
            let found_steps = kerbs_only(
                steps(&good, &k.shape),
                &good,
                &found.road,
                &k.shape,
                self.shape.max_height,
            );
            // Kept in odom, the ones still in reach sent back; no odometry yet: these only.
            let held = match k.pose {
                Some(pose) => k.memory.update(stamp, pose, &found_steps),
                None => found_steps,
            };
            kerb_points = Some(held.len());
            if let Err(e) = k.publisher.publish(xyz32(&header, &held)) {
                log_warn!(
                    self.logger.throttle(WARN_EVERY),
                    "/perception/voxels/steps: {e}"
                );
            }
        }
        let ms = start.elapsed().as_secs_f64() * 1e3;
        if let Err(e) = self
            .status
            .publish(status(&found, ms, self.dropped, kerb_points))
        {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "/perception/voxels/status: {e}"
            );
        }
    }
}

/// The cloud's x, y, z, in the lidar's frame.
fn read(msg: &PointCloud2) -> Result<Vec<Point>, cloud::CloudError> {
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

/// The points as an x y z float32 cloud, one row: as sensor_msgs_py's create_cloud_xyz32.
fn xyz32(header: &Header, points: &[Point]) -> PointCloud2 {
    let width = u32::try_from(points.len()).unwrap_or(u32::MAX);
    PointCloud2 {
        header: header.clone(),
        height: 1,
        width,
        fields: cloud::XYZ32_FIELDS
            .iter()
            .map(|f| PointField {
                name: f.name.to_string(),
                offset: f.offset,
                datatype: f.datatype,
                count: f.count,
            })
            .collect(),
        is_bigendian: false,
        point_step: cloud::XYZ32_STEP,
        row_step: cloud::XYZ32_STEP.saturating_mul(width),
        data: cloud::write_xyz32(points),
        is_dense: false,
    }
}

/// The status after one cloud: the Python's keys and formats; `steps` too, with steps:=true.
fn status(found: &Cubes, ms: f64, dropped: u64, steps: Option<usize>) -> DiagnosticStatus {
    let road = &found.road;
    let (level, message) = if road.is_fitted() {
        (DiagnosticStatus::OK, "road fitted")
    } else {
        (DiagnosticStatus::WARN, "road taken as flat (no good fit)")
    };
    let values = [
        ("ms", format!("{ms:.2}")),
        ("cubes", found.obstacles.len().to_string()),
        ("ground_cubes", found.ground.len().to_string()),
        ("points", found.points.to_string()),
        ("fitted", road.is_fitted().to_string()),
        ("roll_deg", format!("{:.2}", road.roll().to_degrees())),
        ("pitch_deg", format!("{:.2}", road.pitch().to_degrees())),
        ("clouds_dropped", dropped.to_string()),
    ]
    .into_iter()
    .chain(steps.map(|n| ("steps", n.to_string())));
    DiagnosticStatus {
        level,
        name: "voxels".into(),
        message: message.into(),
        hardware_id: String::new(),
        values: values
            .map(|(key, value)| KeyValue {
                key: key.into(),
                value,
            })
            .collect(),
    }
}

/// The node's parameters.
///
/// Keep this alive as long as the node: rclrs undeclares a parameter when its handle is
/// dropped. Read-only, because they are read once.
struct Params {
    size: ReadOnlyParameter<f64>,
    min_height: ReadOnlyParameter<f64>,
    max_height: ReadOnlyParameter<f64>,
    range_max: ReadOnlyParameter<f64>,
    lidar_xyz: MandatoryParameter<Arc<[f64]>>,
    lidar_rpy: MandatoryParameter<Arc<[f64]>>,
    body: MandatoryParameter<Arc<[f64]>>,
    steps: ReadOnlyParameter<bool>,
    step_min: ReadOnlyParameter<f64>,
    step_radius: ReadOnlyParameter<f64>,
    step_range: ReadOnlyParameter<f64>,
    step_keep: ReadOnlyParameter<f64>,
    ground: ReadOnlyParameter<Arc<str>>,
    patch_min_height: ReadOnlyParameter<f64>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        let d = CubeShape::default();
        let s = StepShape::default();
        let number = |name: &str, default: f64| {
            node.declare_parameter(name)
                .default(default)
                .read_only()
                .with_context(|| format!("parameter {name} (m)"))
        };
        let list = |name: &str| {
            node.declare_parameter::<Arc<[f64]>>(name)
                .mandatory()
                .with_context(|| {
                    format!("parameter {name}: a list of 3 numbers (navigation.launch.py sets it)")
                })
        };
        Ok(Self {
            size: number("size", d.size)?,
            min_height: number("min_height", d.min_height)?,
            max_height: number("max_height", d.max_height)?,
            range_max: number("range_max", d.range_max)?,
            lidar_xyz: list("lidar_xyz")?,
            lidar_rpy: list("lidar_rpy")?,
            body: list("body")?,
            steps: node
                .declare_parameter("steps")
                .default(false)
                .read_only()
                .context("parameter steps (true: find kerbs)")?,
            step_min: number("step_min", s.min_step)?,
            step_radius: number("step_radius", s.radius)?,
            step_range: number("step_range", s.range_max)?,
            step_keep: number("step_keep", 10.0)?,
            ground: node
                .declare_parameter("ground")
                .default(Arc::from("plane"))
                .read_only()
                .context("parameter ground (plane or patchwork)")?,
            patch_min_height: number("patch_min_height", PatchSettings::default().min_height)?,
        })
    }

    /// The shape, the lidar's mount, and where the lidar is (for the log).
    fn read(&self) -> Result<(CubeShape, Mount, [f64; 3])> {
        let [back, front, half_width] = three("body", self.body.get())?;
        let shape = CubeShape {
            size: self.size.get(),
            min_height: self.min_height.get(),
            max_height: self.max_height.get(),
            range_max: self.range_max.get(),
            body: Body {
                back,
                front,
                half_width,
            },
            ..CubeShape::default()
        }
        .checked()?;
        let xyz = three("lidar_xyz", self.lidar_xyz.get())?;
        let rpy = three("lidar_rpy", self.lidar_rpy.get())?;
        Ok((shape, Mount::new(xyz, rpy), xyz))
    }

    /// Patchwork's settings with ground:=patchwork (TODO 215); None: one plane.
    fn read_ground(&self) -> Result<Option<PatchSettings>> {
        let ground: Ground = self
            .ground
            .get()
            .parse()
            .map_err(|e: String| anyhow::anyhow!(e))?;
        Ok(match ground {
            Ground::Plane => None,
            Ground::Patchwork => Some(PatchSettings {
                min_height: self.patch_min_height.get(),
                ..PatchSettings::default()
            }),
        })
    }

    /// The step finder's shape, with steps:=true.
    fn read_steps(&self) -> Result<Option<StepShape>> {
        if !self.steps.get() {
            return Ok(None);
        }
        Ok(Some(
            StepShape {
                min_step: self.step_min.get(),
                radius: self.step_radius.get(),
                range_max: self.step_range.get(),
                ..StepShape::default()
            }
            .checked()?,
        ))
    }
}

fn three(name: &str, value: Arc<[f64]>) -> Result<[f64; 3]> {
    match *value {
        [a, b, c] if [a, b, c].iter().all(|v| v.is_finite()) => Ok([a, b, c]),
        _ => bail!("{name}:={value:?}: needs 3 numbers"),
    }
}
