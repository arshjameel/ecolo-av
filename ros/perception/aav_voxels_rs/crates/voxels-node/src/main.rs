//! voxels: the roof lidar's cloud as coarse cubes around base_link, in Rust. A drop-in
//! for aav_voxels' Python node: the same name, topics, QoS, frames, fields and status.
//!
//! Subscribes  /sensors/lidar/points       sensor_msgs/PointCloud2 (the lidar's driver; the
//!                                         simulator's), reliable, keep last 1: a cloud that
//!                                         comes while one is cut replaces the one waiting
//! Publishes   /perception/voxels/cubes    sensor_msgs/PointCloud2, base_link, x y z
//!                                         float32, reliable, depth 1: for every cube (size,
//!                                         0.25 m) holding a point between min_height and
//!                                         max_height above the fitted road, its point
//!                                         nearest base_link; the cloud's stamp
//!             /perception/voxels/ground   sensor_msgs/PointCloud2, base_link, the same QoS:
//!                                         the centre of every flat square where the lidar
//!                                         saw the road (free space: the costmap clears with it)
//!             /perception/voxels/status   diagnostic_msgs/DiagnosticStatus, every cloud: ms,
//!                                         cubes, ground_cubes, points, fitted, roll_deg,
//!                                         pitch_deg, clouds_dropped
//! Parameters  size (m), min_height, max_height (m above the road), range_max (m from
//!             base_link): nav2.yaml's voxels: block, as the Python reads it. lidar_xyz (m),
//!             lidar_rpy (rad): vehicle.yaml's lidar_link; body ([x from, x to, half the
//!             width], m): aav_lidar scan.body_box. navigation.launch.py passes all three
//!             (voxels_impl:=rust); they have no defaults, so a missing one stops the node.
//!
//! No serving, no calls, never TF. The maths is the `voxels` crate, with no ROS.

use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use rclrs::*;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::sensor_msgs::msg::{PointCloud2, PointField};
use ros_env::std_msgs::msg::Header;
use voxels::{
    cloud::{self, CloudView, Field},
    cubes, Body, CubeShape, Cubes, Mount, Point,
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
    log_info!(
        node.logger(),
        "voxels (Rust): {:.2} m cubes from {:.2} to {:.2} m above the fitted road, out to \
         {:.0} m; the lidar at {lidar_xyz:?}",
        shape.size,
        shape.min_height,
        shape.max_height,
        shape.range_max
    );

    // Reliable, depth 1: best-effort readers (the costmap's, collision_monitor's) still
    // connect, and a Mac's UDP doesn't lose whole clouds.
    let state = State {
        shape,
        mount,
        last_stamp: None,
        dropped: 0,
        logger: node.logger().clone(),
        cubes: node.create_publisher("/perception/voxels/cubes".keep_last(1).reliable())?,
        ground: node.create_publisher("/perception/voxels/ground".keep_last(1).reliable())?,
        status: node.create_publisher("/perception/voxels/status".keep_last(10).reliable())?,
    };
    let worker = node.create_worker(state);
    let _cloud = worker.create_subscription(
        "/sensors/lidar/points".keep_last(1).reliable(),
        |s: &mut State, msg: PointCloud2| s.on_cloud(&msg),
    )?;

    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The worker's payload.
struct State {
    shape: CubeShape,
    mount: Mount,
    /// The last cloud's stamp (s), to count the ones dropped since.
    last_stamp: Option<f64>,
    dropped: u64,
    logger: Logger,
    cubes: Publisher<PointCloud2>,
    ground: Publisher<PointCloud2>,
    status: Publisher<DiagnosticStatus>,
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
        let found = cubes(&base, &self.shape);
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
        let ms = start.elapsed().as_secs_f64() * 1e3;
        if let Err(e) = self.status.publish(status(&found, ms, self.dropped)) {
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

/// The status after one cloud: the Python's keys and formats.
fn status(found: &Cubes, ms: f64, dropped: u64) -> DiagnosticStatus {
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
    ];
    DiagnosticStatus {
        level,
        name: "voxels".into(),
        message: message.into(),
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
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        let d = CubeShape::default();
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
}

fn three(name: &str, value: Arc<[f64]>) -> Result<[f64; 3]> {
    match *value {
        [a, b, c] if [a, b, c].iter().all(|v| v.is_finite()) => Ok([a, b, c]),
        _ => bail!("{name}:={value:?}: needs 3 numbers"),
    }
}
