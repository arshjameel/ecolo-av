//! stop_sign_watcher, in Rust: stop the car at stop signs, through a lock on twist_mux. The
//! same node as aav_detector's stop_sign_watcher.py, topic for topic.
//!
//! Subscribes  /perception/detections  vision_msgs/Detection2DArray (from the detector)
//!             camera_info_topics      sensor_msgs/CameraInfo, one topic per camera (default
//!                                     /sensors/camera/<front_left, front_right, rear>/
//!                                     camera_info): its lens, kept by the frame it names
//!             /world/things           aav_interfaces/Things, odom (lidar_distance only)
//!             /tf, /tf_static         odom <-> each forward camera (lidar_distance only)
//! Publishes   /perception/stop_sign   std_msgs/Bool, 10 Hz and with each detection: true
//!                                     while the car must stay stopped
//!             /perception/stop_sign/distance  geometry_msgs/PointStamped: where the
//!                                     tallest stop sign a forward camera sees is, in its
//!                                     frame at the picture's stamp. Only while CLEAR. With
//!                                     lidar_distance, a thing named stop sign within
//!                                     lidar_gate_m of that guess gives the point instead
//! Parameters  stop_distance_m (4.7), focal_px (343), sign_height_m (0.75), min_score
//!             (0.5), hold_s (4), ignore_s (10), forward_words ([front, left, right]),
//!             backward_words ([rear]), camera_info_topics, lidar_distance (false),
//!             lidar_gate_m (3.0)
//!
//! twist_mux's stop_sign lock (priority 30) holds navigation, the point driver and the AI
//! slot; never a person or a test. It never expires on silence. It only ever stops the
//! car. The rules are the stop-sign-core crate (no ROS).

use std::any::Any;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use behaviour_core::tf::Buffer;
use ego_core::frames::Transform;
use rclrs::*;
use ros_env::aav_interfaces::msg::Things;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::geometry_msgs::msg::{Point, PointStamped};
use ros_env::sensor_msgs::msg::CameraInfo;
use ros_env::std_msgs::msg::Bool;
use ros_env::tf2_msgs::msg::TFMessage;
use ros_env::vision_msgs::msg::Detection2DArray;
use stop_sign_core::{
    biggest_sign, facing_forward, is_optical, lidar_sign, sign_point, Sighting, State, StopRule,
    StopSignWatcher,
};

const CAMERAS: [&str; 3] = ["front_left", "front_right", "rear"];

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("stop_sign_watcher")?;
    let _keep = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The node's parameters, declared read-only; their handles kept alive.
struct Params<'a> {
    node: &'a Node,
    keep: Vec<Box<dyn Any>>,
}

impl Params<'_> {
    fn value(&mut self, name: &str, default: ParameterValue) -> Result<ParameterValue> {
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name}"))?;
        let value = p.get();
        self.keep.push(Box::new(p));
        Ok(value)
    }

    fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        match self.value(name, ParameterValue::Double(default))? {
            ParameterValue::Double(v) => Ok(v),
            ParameterValue::Integer(v) => Ok(v as f64),
            other => bail!("{name}:={other:?}: not a number"),
        }
    }

    fn flag(&mut self, name: &str, default: bool) -> Result<bool> {
        match self.value(name, ParameterValue::Bool(default))? {
            ParameterValue::Bool(v) => Ok(v),
            other => bail!("{name}:={other:?}: not true or false"),
        }
    }

    fn words(&mut self, name: &str, default: &[&str]) -> Result<Vec<String>> {
        let list: Arc<[Arc<str>]> = default.iter().map(|s| Arc::from(*s)).collect();
        match self.value(name, ParameterValue::StringArray(list))? {
            ParameterValue::StringArray(v) => Ok(v.iter().map(|s| s.to_string()).collect()),
            ParameterValue::ByteArray(b) if b.is_empty() => Ok(Vec::new()),
            other => bail!("{name}:={other:?}: not a list of words"),
        }
    }
}

fn ns(t: &Time) -> i64 {
    i64::from(t.sec) * 1_000_000_000 + i64::from(t.nanosec)
}

/// A point through a transform: rotated, then moved.
fn apply(t: &Transform, p: [f64; 3]) -> [f64; 3] {
    let r = t.rotate(p);
    [
        r[0] + t.translation[0],
        r[1] + t.translation[1],
        r[2] + t.translation[2],
    ]
}

struct Watcher {
    watcher: StopSignWatcher,
    /// frame_id -> fy (px), from each camera's CameraInfo.
    focal: HashMap<String, f64>,
    /// frame_id -> (fx, cx) (px), for the sign's bearing.
    lens: HashMap<String, (f64, f64)>,
    forward: Vec<String>,
    backward: Vec<String>,
    use_lidar: bool,
    gate_m: f64,
    /// (frame, [(kind, x, y)]) from /world/things.
    things: (String, Vec<(String, f64, f64)>),
    tf: Buffer,
    state: State,
    clock: Clock,
    logger: Logger,
    lock: Publisher<Bool>,
    point: Publisher<PointStamped>,
}

impl Watcher {
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_camera_info(&mut self, msg: &CameraInfo) {
        let frame = msg.header.frame_id.clone();
        let fy = msg.k[4];
        if msg.k[0] > 0.0 {
            self.lens.insert(frame.clone(), (msg.k[0], msg.k[2]));
        }
        if fy > 0.0 && self.focal.get(&frame) != Some(&fy) {
            log_info!(
                &self.logger,
                "{frame}: focal length {fy:.0} px: stops for a stop sign {:.0} px tall",
                self.watcher.rule.min_height_px(Some(fy))
            );
            self.focal.insert(frame, fy);
        }
    }

    fn on_boxes(&mut self, msg: &Detection2DArray) {
        let boxes: Vec<Sighting<'_>> = msg
            .detections
            .iter()
            .filter_map(|d| {
                d.results.first().map(|r| Sighting {
                    label: &r.hypothesis.class_id,
                    score: r.hypothesis.score,
                    height: d.bbox.size_y,
                    centre: d.bbox.center.position.x,
                })
            })
            .collect();
        let frame = &msg.header.frame_id;
        let focal = self.focal.get(frame).copied();
        if focal.is_none() && !boxes.is_empty() {
            log_warn!(
                self.logger.throttle(Duration::from_secs(30)),
                "no CameraInfo for camera '{frame}': its stop signs are judged with focal_px \
                 {}. Calibrate it, or set focal_px",
                self.watcher.rule.focal_px
            );
        }
        if self.watcher.state() == State::Clear
            && facing_forward(frame, &self.forward, &self.backward)
        {
            self.publish_point(msg, &boxes, focal);
        }
        let now = self.now();
        let stop = self.watcher.update(&boxes, now, focal);
        self.publish(stop);
    }

    fn publish_point(&self, msg: &Detection2DArray, boxes: &[Sighting<'_>], focal: Option<f64>) {
        let Some((centre, height)) = biggest_sign(boxes, self.watcher.rule.min_score) else {
            return;
        };
        let frame = &msg.header.frame_id;
        let rule = &self.watcher.rule;
        let mut p = sign_point(
            centre,
            height,
            focal.filter(|f| *f != 0.0).unwrap_or(rule.focal_px),
            self.lens.get(frame).copied(),
            is_optical(frame),
            rule.sign_height_m,
        );
        if self.use_lidar {
            if let Some(found) = self.lidar_point(frame, ns(&msg.header.stamp), p) {
                p = found;
            }
        }
        let out = PointStamped {
            header: msg.header.clone(),
            point: Point {
                x: p[0],
                y: p[1],
                z: p[2],
            },
        };
        if let Err(e) = self.point.publish(out) {
            log_warn!(
                self.logger.throttle(Duration::from_secs(5)),
                "stop_sign/distance: {e}"
            );
        }
    }

    /// The named sign's lidar place in the camera's frame, or `None`: tf2 at the picture's
    /// stamp, into the things' frame and back.
    fn lidar_point(&self, camera: &str, stamp: i64, p: [f64; 3]) -> Option<[f64; 3]> {
        let (frame, things) = &self.things;
        if things.is_empty() {
            return None;
        }
        let t = self.tf.lookup(frame, camera, Some(stamp)).ok()?;
        let guess = apply(&t, p);
        let (x, y) = lidar_sign((guess[0], guess[1]), things, self.gate_m)?;
        Some(apply(&t.inverse(), [x, y, guess[2]]))
    }

    fn on_tf(&mut self, msg: &TFMessage, fixed: bool) {
        for t in &msg.transforms {
            let r = &t.transform;
            self.tf.set(
                &t.header.frame_id,
                &t.child_frame_id,
                ns(&t.header.stamp),
                Transform {
                    translation: [r.translation.x, r.translation.y, r.translation.z],
                    rotation: [r.rotation.x, r.rotation.y, r.rotation.z, r.rotation.w],
                },
                fixed,
            );
        }
    }

    fn tick(&mut self) {
        let now = self.now();
        let stop = self.watcher.tick(now);
        self.publish(stop);
    }

    fn publish(&mut self, stop: bool) {
        if self.watcher.state() != self.state {
            self.state = self.watcher.state();
            log_info!(&self.logger, "stop sign: {}", self.state.word());
        }
        if let Err(e) = self.lock.publish(Bool { data: stop }) {
            log_warn!(
                self.logger.throttle(Duration::from_secs(5)),
                "stop_sign: {e}"
            );
        }
    }
}

fn lock(w: &Mutex<Watcher>) -> MutexGuard<'_, Watcher> {
    w.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut p = Params {
        node,
        keep: Vec::new(),
    };
    let d = StopRule::default();
    let rule = StopRule {
        stop_distance_m: p.number("stop_distance_m", d.stop_distance_m)?,
        focal_px: p.number("focal_px", d.focal_px)?,
        sign_height_m: p.number("sign_height_m", d.sign_height_m)?,
        min_score: p.number("min_score", d.min_score)?,
        hold_s: p.number("hold_s", d.hold_s)?,
        ignore_s: p.number("ignore_s", d.ignore_s)?,
    };
    let forward = p.words("forward_words", &["front", "left", "right"])?;
    let backward = p.words("backward_words", &["rear"])?;
    let default_topics: Vec<String> = CAMERAS
        .iter()
        .map(|c| format!("/sensors/camera/{c}/camera_info"))
        .collect();
    let topics = p.words(
        "camera_info_topics",
        &default_topics
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>(),
    )?;
    let use_lidar = p.flag("lidar_distance", false)?;
    let gate_m = p.number("lidar_gate_m", 3.0)?;
    let mut keep = p.keep;
    let logger = node.logger().clone();
    log_info!(
        &logger,
        "stops {} m before a {} m stop sign: {:.0} px tall with focal_px {}, the lens of a \
         camera without CameraInfo",
        rule.stop_distance_m,
        rule.sign_height_m,
        rule.min_height_px(None),
        rule.focal_px
    );
    let watcher = Arc::new(Mutex::new(Watcher {
        watcher: StopSignWatcher::new(rule),
        focal: HashMap::new(),
        lens: HashMap::new(),
        forward,
        backward,
        use_lidar,
        gate_m,
        things: ("odom".to_string(), Vec::new()),
        tf: Buffer::new(),
        state: State::Clear,
        clock: node.get_clock(),
        logger,
        lock: node.create_publisher("/perception/stop_sign")?,
        point: node.create_publisher("/perception/stop_sign/distance")?,
    }));
    for topic in &topics {
        let w = Arc::clone(&watcher);
        keep.push(Box::new(
            node.create_subscription(topic.as_str(), move |msg: CameraInfo| {
                lock(&w).on_camera_info(&msg)
            })?,
        ));
    }
    let w = Arc::clone(&watcher);
    keep.push(Box::new(node.create_subscription(
        "/perception/detections",
        move |msg: Detection2DArray| lock(&w).on_boxes(&msg),
    )?));
    if use_lidar {
        let w = Arc::clone(&watcher);
        keep.push(Box::new(node.create_subscription(
            "/world/things",
            move |msg: Things| {
                let frame = msg.header.frame_id.clone();
                let things = msg
                    .things
                    .iter()
                    .map(|t| (t.kind.clone(), f64::from(t.x), f64::from(t.y)))
                    .collect();
                lock(&w).things = (frame, things);
            },
        )?));
        // As tf2's listener: /tf keep last 100; /tf_static latched.
        let w = Arc::clone(&watcher);
        keep.push(Box::new(node.create_subscription(
            "/tf".keep_last(100).reliable(),
            move |msg: TFMessage| lock(&w).on_tf(&msg, false),
        )?));
        let w = Arc::clone(&watcher);
        keep.push(Box::new(node.create_subscription(
            "/tf_static".keep_last(100).reliable().transient_local(),
            move |msg: TFMessage| lock(&w).on_tf(&msg, true),
        )?));
    }
    let w = Arc::clone(&watcher);
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_millis(100).node_time(),
        move || lock(&w).tick(),
    )?));
    keep.push(Box::new(watcher));
    Ok(keep)
}
