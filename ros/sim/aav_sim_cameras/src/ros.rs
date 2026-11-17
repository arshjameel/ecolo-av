//! The node `sim_cameras`: the pictures out on Gazebo's camera topics, the car's pose in.
//!
//! Subscribes: `/vehicle/sim/truth` (nav_msgs/Odometry, map frame: where the simulated car
//! is; only with `--pose truth`); `/vehicle/sim/entities` (visualization_msgs/MarkerArray,
//! latched: the models the simulator placed while running, each drawn from its SDF file).
//! Publishes, for each camera in `config/cameras.yaml`: `<topic>/image_raw`
//! (sensor_msgs/Image, rgb8, the camera's link as frame_id) and, except the chase camera,
//! `<topic>/camera_info` (sensor_msgs/CameraInfo). Reliable, keep last 10: as
//! ros_gz_bridge publishes Gazebo's. With `--pose truth`, `/vehicle/sim/cameras/free_until`
//! (rosgraph_msgs/Clock): after each pose, the sim time of the next picture due, for the
//! simulator's lockstep (`camera_lockstep:=true`: its clock waits there for the pictures).
//! Serves and calls nothing.

use std::{sync::mpsc, thread, time::Duration};

use camera_core::{CameraSpec, Frame, Pose, Rig, Rotation, Vec3};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::nav_msgs::msg::Odometry;
use ros_env::rosgraph_msgs::msg::Clock;
use ros_env::sensor_msgs::msg::{CameraInfo, Image};
use ros_env::std_msgs::msg::Header;
use ros_env::visualization_msgs::msg::{Marker, MarkerArray};

const NODE_NAME: &str = "sim_cameras";
/// Where the simulated car is: the simulator's truth.
pub const TRUTH: &str = "/vehicle/sim/truth";
/// The cameras' word to the simulator's lockstep: nothing due before this sim time.
pub const FREE_UNTIL: &str = "/vehicle/sim/cameras/free_until";

struct Outputs {
    image: Publisher<Image>,
    info: Option<Publisher<CameraInfo>>,
}

/// The node's publishers, and the car's poses as they arrive.
pub struct RosOut {
    outputs: Vec<Outputs>,
    infos: Vec<CameraInfo>,
    frame_ids: Vec<String>,
    free_until: Option<Publisher<Clock>>,
}

/// A pose of the car at a sim time.
pub type Stamped = (Duration, Pose);

/// Where the placed models are: the simulator's list.
pub const ENTITIES: &str = "/vehicle/sim/entities";

/// A model the simulator placed: its name, its SDF file, its pose in the map.
pub type Placed = (String, std::path::PathBuf, Pose);

/// What the node hands the drawing loop: the car's poses, and the placed models' lists.
pub type Inputs = (mpsc::Receiver<Stamped>, mpsc::Receiver<Vec<Placed>>);

impl RosOut {
    /// Starts the node on its own thread. With `truth`, the car's poses arrive on the
    /// returned receiver, newest last.
    pub fn start(rig: &Rig, truth: bool) -> Result<(Self, Inputs), String> {
        let (tx, rx) = mpsc::sync_channel::<Stamped>(256);
        let (placed_tx, placed_rx) = mpsc::channel::<Vec<Placed>>();
        let (ready_tx, ready_rx) = mpsc::channel();
        let specs = rig.cameras.clone();
        thread::Builder::new()
            .name("ros".into())
            .spawn(move || {
                let run = || -> Result<(), RclrsError> {
                    let context = Context::new(
                        std::iter::once(NODE_NAME.to_owned()),
                        InitOptions::default(),
                    )?;
                    let mut executor = context.create_basic_executor();
                    let node = executor.create_node(NODE_NAME)?;
                    let outputs = specs
                        .iter()
                        .map(|c| {
                            Ok(Outputs {
                                image: node.create_publisher(c.image_topic().as_str())?,
                                info: if c.camera_info {
                                    Some(node.create_publisher(c.info_topic().as_str())?)
                                } else {
                                    None
                                },
                            })
                        })
                        .collect::<Result<Vec<_>, RclrsError>>()?;
                    let _truth = if truth {
                        let tx = tx.clone();
                        Some(
                            node.create_subscription(TRUTH.reliable(), move |m: Odometry| {
                                let _ = tx.try_send((stamp_of(&m.header.stamp), pose_of(&m)));
                            })?,
                        )
                    } else {
                        None
                    };
                    let _entities = node.create_subscription(
                        ENTITIES.reliable().transient_local(),
                        move |m: MarkerArray| {
                            let _ = placed_tx.send(placed_of(&m));
                        },
                    )?;
                    let free_until = if truth {
                        Some(node.create_publisher::<Clock>(FREE_UNTIL)?)
                    } else {
                        None
                    };
                    let _ = ready_tx.send(Ok((outputs, free_until)));
                    executor.spin(SpinOptions::default()).first_error()
                };
                if let Err(e) = run() {
                    let _ = ready_tx.send(Err(format!("{e:#}")));
                    eprintln!("sim_cameras: the ROS node stopped: {e:#}");
                }
            })
            .map_err(|e| e.to_string())?;
        let (outputs, free_until) = ready_rx
            .recv()
            .map_err(|_| "the ROS thread ended before the node was up".to_owned())?
            .map_err(|e| format!("starting the ROS node (is ROS sourced?): {e}"))?;
        let infos = rig.cameras.iter().map(camera_info).collect();
        let frame_ids = rig.cameras.iter().map(|c| c.frame_id.clone()).collect();
        Ok((
            Self {
                outputs,
                infos,
                frame_ids,
                free_until,
            },
            (rx, placed_rx),
        ))
    }

    /// Tells the simulator's lockstep that nothing is due before `next` (sim time).
    pub fn free_until(&self, next: Duration) -> Result<(), String> {
        match &self.free_until {
            Some(p) => p
                .publish(Clock {
                    clock: time_of(next),
                })
                .map_err(|e| e.to_string()),
            None => Ok(()),
        }
    }

    /// Publishes one picture and its camera_info, with the same stamp.
    pub fn publish(&self, frame: Frame) -> Result<(), String> {
        let i = frame.camera;
        let header = Header {
            stamp: time_of(frame.stamp),
            frame_id: self.frame_ids[i].clone(),
        };
        let out = &self.outputs[i];
        if let Some(info) = &out.info {
            let mut msg = self.infos[i].clone();
            msg.header = header.clone();
            info.publish(msg).map_err(|e| e.to_string())?;
        }
        let step = frame.step();
        let image = Image {
            header,
            height: frame.height,
            width: frame.width,
            encoding: Frame::ENCODING.to_owned(),
            is_bigendian: 0,
            step,
            data: frame.rgb,
        };
        out.image.publish(image).map_err(|e| e.to_string())
    }
}

/// camera_info as Gazebo's bridge sends it (no header yet).
fn camera_info(spec: &CameraSpec) -> CameraInfo {
    let lens = &spec.lens;
    CameraInfo {
        height: lens.height(),
        width: lens.width(),
        distortion_model: lens.distortion_model().to_owned(),
        d: lens.d().to_vec(),
        k: lens.k(),
        r: lens.r(),
        p: lens.p(),
        ..Default::default()
    }
}

fn stamp_of(t: &Time) -> Duration {
    Duration::new(u64::try_from(t.sec).unwrap_or(0), t.nanosec)
}

fn time_of(d: Duration) -> Time {
    Time {
        sec: i32::try_from(d.as_secs()).unwrap_or(i32::MAX),
        nanosec: d.subsec_nanos(),
    }
}

fn pose_of(m: &Odometry) -> Pose {
    let p = &m.pose.pose;
    let q = &p.orientation;
    Pose::new(
        Vec3::new(p.position.x, p.position.y, p.position.z),
        Rotation::from_quaternion(q.x, q.y, q.z, q.w),
    )
}

/// The simulator's list of placed models: its MESH_RESOURCE markers (a DELETEALL first).
fn placed_of(m: &MarkerArray) -> Vec<Placed> {
    m.markers
        .iter()
        .filter(|k| k.action == Marker::ADD && k.type_ == Marker::MESH_RESOURCE)
        .map(|k| {
            let p = &k.pose;
            let q = &p.orientation;
            let file = k
                .mesh_resource
                .strip_prefix("file://")
                .unwrap_or(&k.mesh_resource);
            (
                k.ns.clone(),
                std::path::PathBuf::from(file),
                Pose::new(
                    Vec3::new(p.position.x, p.position.y, p.position.z),
                    Rotation::from_quaternion(q.x, q.y, q.z, q.w),
                ),
            )
        })
        .collect()
}
