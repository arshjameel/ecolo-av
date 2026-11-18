//! lidar_scan: the roof lidar's cloud as a 2D scan around base_link, in Rust. A drop-in
//! for aav_lidar's Python node: the same name, topics, QoS, frame and ranges.
//!
//! Subscribes  /sensors/lidar/points  sensor_msgs/PointCloud2 (the lidar's driver; the
//!                                    simulator's), reliable, keep last 1: the newest only
//! Publishes   /sensors/lidar/scan    sensor_msgs/LaserScan, best effort (sensor data), in
//!                                    base_link, the cloud's stamp: for each direction, the
//!                                    nearest point between min_height and max_height above
//!                                    the road, inf where none
//! Parameters  min_height, max_height (m above the road), angle_min, angle_max,
//!             angle_increment (rad), range_min, range_max (m from base_link), scan_time
//!             (s): nav2.yaml's lidar_scan: block, as the Python reads it. lidar_xyz (m),
//!             lidar_rpy (rad): vehicle.yaml's lidar_link; body ([x from, x to, half the
//!             width], m): the car's own outline (lidar_scan::body_box).
//!             navigation.launch.py passes all three (lidar_impl:=rust); they have no
//!             defaults, so a missing one stops the node.
//!
//! No serving, no calls, never TF. The maths is the `lidar-scan` crate, with no ROS.

use std::{sync::Arc, time::Duration};

use anyhow::{bail, Context as _, Result};
use lidar_scan::{
    cloud::{self, CloudView, Field},
    flatten, Body, Mount, Point, ScanShape,
};
use rclrs::*;
use ros_env::sensor_msgs::msg::{LaserScan, PointCloud2};
use ros_env::std_msgs::msg::Header;

/// How often a repeated warning may be logged.
const WARN_EVERY: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("lidar_scan")?;
    let params = Params::declare(&node)?;
    let (shape, mount, lidar_xyz) = params.read()?;
    let scan_time = params.scan_time.get();
    log_info!(
        node.logger(),
        "lidar_scan (Rust): {} directions, obstacles from {:.2} to {:.2} m above the road, \
         the lidar at {lidar_xyz:?} in base_link",
        shape.count(),
        shape.min_height,
        shape.max_height
    );

    let state = State {
        shape,
        mount,
        scan_time,
        logger: node.logger().clone(),
        scan: node.create_publisher("/sensors/lidar/scan".sensor_data_qos())?,
    };
    let worker = node.create_worker(state);
    // Reliable: best effort loses whole clouds where they travel by UDP (docs/mac.md).
    // Depth 1: a late cloud is dropped for the newest.
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
    shape: ScanShape,
    mount: Mount,
    scan_time: f64,
    logger: Logger,
    scan: Publisher<LaserScan>,
}

impl State {
    fn on_cloud(&mut self, msg: &PointCloud2) {
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
        let s = &self.shape;
        // LaserScan's numbers are float32, as the Python's message makes them.
        let scan = LaserScan {
            header: Header {
                stamp: msg.header.stamp.clone(),
                frame_id: "base_link".into(),
            },
            angle_min: s.angle_min as f32,
            angle_max: (s.angle_min + (s.count() as f64 - 1.0) * s.angle_increment) as f32,
            angle_increment: s.angle_increment as f32,
            time_increment: 0.0,
            scan_time: self.scan_time as f32,
            range_min: s.range_min as f32,
            range_max: s.range_max as f32,
            ranges: flatten(&base, s).into_iter().map(|r| r as f32).collect(),
            intensities: Vec::new(),
        };
        if let Err(e) = self.scan.publish(scan) {
            log_warn!(self.logger.throttle(WARN_EVERY), "/sensors/lidar/scan: {e}");
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

/// The node's parameters.
///
/// Keep this alive as long as the node: rclrs undeclares a parameter when its handle is
/// dropped. Read-only, because they are read once.
struct Params {
    min_height: ReadOnlyParameter<f64>,
    max_height: ReadOnlyParameter<f64>,
    angle_min: ReadOnlyParameter<f64>,
    angle_max: ReadOnlyParameter<f64>,
    angle_increment: ReadOnlyParameter<f64>,
    range_min: ReadOnlyParameter<f64>,
    range_max: ReadOnlyParameter<f64>,
    scan_time: ReadOnlyParameter<f64>,
    lidar_xyz: MandatoryParameter<Arc<[f64]>>,
    lidar_rpy: MandatoryParameter<Arc<[f64]>>,
    body: MandatoryParameter<Arc<[f64]>>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        let d = ScanShape::default();
        let number = |name: &str, default: f64| {
            node.declare_parameter(name)
                .default(default)
                .read_only()
                .with_context(|| format!("parameter {name}"))
        };
        let list = |name: &str| {
            node.declare_parameter::<Arc<[f64]>>(name)
                .mandatory()
                .with_context(|| {
                    format!("parameter {name}: a list of 3 numbers (navigation.launch.py sets it)")
                })
        };
        Ok(Self {
            min_height: number("min_height", d.min_height)?,
            max_height: number("max_height", d.max_height)?,
            angle_min: number("angle_min", d.angle_min)?,
            angle_max: number("angle_max", d.angle_max)?,
            angle_increment: number("angle_increment", d.angle_increment)?,
            range_min: number("range_min", d.range_min)?,
            range_max: number("range_max", d.range_max)?,
            scan_time: number("scan_time", 0.1)?,
            lidar_xyz: list("lidar_xyz")?,
            lidar_rpy: list("lidar_rpy")?,
            body: list("body")?,
        })
    }

    /// The shape, the lidar's mount, and where the lidar is (for the log).
    fn read(&self) -> Result<(ScanShape, Mount, [f64; 3])> {
        let [back, front, half_width] = three("body", self.body.get())?;
        let shape = ScanShape {
            min_height: self.min_height.get(),
            max_height: self.max_height.get(),
            angle_min: self.angle_min.get(),
            angle_max: self.angle_max.get(),
            angle_increment: self.angle_increment.get(),
            range_min: self.range_min.get(),
            range_max: self.range_max.get(),
            body: Body {
                back,
                front,
                half_width,
            },
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
