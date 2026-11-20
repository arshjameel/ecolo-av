//! things: the lidar's cubes as things with an id, a speed and a memory, in Rust. A
//! drop-in for aav_things' Python node: the same name, topics, QoS, frames, parameters
//! and answers.
//!
//! Subscribes  /perception/voxels/cubes    sensor_msgs/PointCloud2 (aav_voxels), base_link:
//!                                         every 0.25 m cube over the road; reliable, the
//!                                         newest only
//!             /perception/radar/clusters  sensor_msgs/PointCloud2 (aav_radar_clusters), the
//!                                         radar's frame: x y velocity cluster; each
//!                                         cluster's mean radial speed, the car's own taken
//!                                         off, gives the nearest thing its speed along the
//!                                         ray (radar:=true only, best effort)
//!             /localization/odom/local    nav_msgs/Odometry: the car's speed (for the radar)
//!             /perception/detections      vision_msgs/Detection2DArray (names true only): a
//!                                         thing whose box overlaps one takes its label
//!             camera_info_topics          sensor_msgs/CameraInfo (names true only): each lens
//!             /map                        nav_msgs/OccupancyGrid, transient local (background
//!                                         true, no background_chunks): cubes on its walls
//!                                         (grown background_slack m) are dropped before
//!                                         grouping; with background_below, only lower ones.
//!                                         With background_chunks (a folder of the NDT map's
//!                                         chunks) the 3 x 3 chunks round the car instead
//!             /tf, /tf_static             odom <- base_link at each cloud's stamp; odom <-
//!                                         the radar; map <- base_link (background); base_link
//!                                         <- each camera (names)
//! Publishes   /world/things               aav_interfaces/Things, odom, at `rate` (10 Hz):
//!                                         every confirmed thing, moved on its speed to the
//!                                         stamp
//!             /world/things/markers       visualization_msgs/MarkerArray, odom: a box in its
//!                                         kind's colour, a speed arrow, and its words
//! Parameters  aav_things' (config/things.yaml): size, gap, min_cubes, max_size; the
//!             tracker's (gate ... small_slack; per_kind, car_along, car_across,
//!             car_heading_speed, person_accel, refind_gap, sightings: TODO 217); radar, rate, background, background_slack,
//!             background_chunks, background_min_z, background_below; names, name_min_iou,
//!             name_hold, name_min_side, name_ceiling, name_min_score, camera_info_topics.
//!
//! A cloud newer than /tf waits for it (things_core::tf_wait), never blocking. No serving,
//! no calls. The maths is the `things-core` crate, with no ROS.

mod params;

use std::{any::Any, path::PathBuf, sync::Arc, time::Duration};

use anyhow::{Context as _, Result};
use behaviour_core::tf::Buffer;
use ego_core::frames::Transform;
use params::Declarer;
use rclrs::*;
use ros_env::aav_interfaces::msg::{Thing, Things};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::geometry_msgs::msg::Point as PointMsg;
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry};
use ros_env::sensor_msgs::msg::{CameraInfo, PointCloud2};
use ros_env::std_msgs::msg::{ColorRGBA, Header};
use ros_env::tf2_msgs::msg::TFMessage;
use ros_env::vision_msgs::msg::{Detection2DArray, ObjectHypothesisWithPose};
use ros_env::visualization_msgs::msg::{Marker, MarkerArray};
use things_core::{
    background::{new_only, Background, ChunkBackground},
    cloud::{read_columns, CloudView, Field},
    group, looks,
    naming::{rotation, Axes, Camera, CameraMount, Lens, Namer, Seen},
    radar::{clusters, RadarPoint},
    tf_wait::{Look, Step, Waiting},
    to_frame, yaw_of, GroupShape, Pose2, Track, Tracker, TrackerShape,
};

const ODOM: &str = "odom";
const BASE: &str = "base_link";

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("things")?;
    let _keep = build(&node)?;
    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The node's settings, from its parameters.
struct Settings {
    groups: GroupShape,
    tracker: TrackerShape,
    radar: bool,
    rate: f64,
    background: bool,
    slack: f64,
    chunks: String,
    min_z: f64,
    below: Option<f64>,
    names: bool,
    namer: Namer,
    min_score: f64,
    camera_topics: Vec<String>,
}

fn settings(d: &mut Declarer<'_>) -> Result<Settings> {
    let g = GroupShape::default();
    let t = TrackerShape::default();
    let groups = GroupShape {
        size: d.number("size", g.size)?,
        gap: d.number("gap", g.gap)?,
        min_cubes: d.count("min_cubes", g.min_cubes as u32)? as usize,
        max_size: d.number("max_size", g.max_size)?,
    };
    let tracker = TrackerShape {
        gate: d.number("gate", t.gate)?,
        position_noise: d.number("position_noise", t.position_noise)?,
        accel_noise: d.number("accel_noise", t.accel_noise)?,
        radar_noise: d.number("radar_noise", t.radar_noise)?,
        radar_gate: d.number("radar_gate", t.radar_gate)?,
        confirm: d.count("confirm", t.confirm)?,
        keep: d.number("keep", t.keep)?,
        keep_blind: d.number("keep_blind", t.keep_blind)?,
        blind_range: d.number("blind_range", t.blind_range)?,
        keep_tentative: d.number("keep_tentative", t.keep_tentative)?,
        start_speed_sigma: d.number("start_speed_sigma", t.start_speed_sigma)?,
        still_speed: d.number("still_speed", t.still_speed)?,
        gate_chi2: d.number("gate_chi2", t.gate_chi2)?,
        shape_ratio: d.number("shape_ratio", t.shape_ratio)?,
        shape_slack: d.number("shape_slack", t.shape_slack)?,
        small_size: d.number("small_size", t.small_size)?,
        small_speed: d.number("small_speed", t.small_speed)?,
        small_slack: d.number("small_slack", t.small_slack)?,
        // TODO 217: a motion model per kind (the launch's things_tracker:=per_kind).
        per_kind: d.flag("per_kind", t.per_kind)?,
        car_along: d.number("car_along", t.car_along)?,
        car_across: d.number("car_across", t.car_across)?,
        car_heading_speed: d.number("car_heading_speed", t.car_heading_speed)?,
        person_accel: d.number("person_accel", t.person_accel)?,
        refind_gap: d.number("refind_gap", t.refind_gap)?,
        sightings: d.count("sightings", t.sightings as u32)? as usize,
    };
    let radar = d.flag("radar", false)?;
    let rate = d.number("rate", 10.0)?;
    anyhow::ensure!(rate > 0.0, "rate:={rate}: needs to be above 0 (Hz)");
    let background = d.flag("background", false)?;
    let slack = d.number("background_slack", 0.25)?;
    let chunks = d.text("background_chunks", "")?;
    let min_z = d.number("background_min_z", 0.3)?;
    // m: only cubes lower than this drop (0: any height).
    let below = d.number("background_below", 0.0)?;
    let names = d.flag("names", false)?;
    let mut namer = Namer::default();
    namer.min_iou = d.number("name_min_iou", namer.min_iou)?;
    namer.hold = d.number("name_hold", namer.hold)?;
    namer.min_side = d.number("name_min_side", namer.min_side)?;
    // The voxels' max_height: a thing reaching it has an unseen top.
    namer.ceiling = d.number("name_ceiling", namer.ceiling)?;
    let min_score = d.number("name_min_score", 0.5)?;
    let camera_topics = if names {
        let default: Vec<String> = ["front_left", "front_right", "rear"]
            .iter()
            .map(|c| format!("/sensors/camera/{c}/camera_info"))
            .collect();
        let refs: Vec<&str> = default.iter().map(String::as_str).collect();
        d.names("camera_info_topics", &refs)?
    } else {
        Vec::new()
    };
    Ok(Settings {
        groups,
        tracker,
        radar,
        rate,
        background,
        slack,
        chunks,
        min_z,
        below: (below > 0.0).then_some(below),
        names,
        namer,
        min_score,
        camera_topics,
    })
}

/// Everything the node keeps alive: parameters, subscriptions, the timer.
fn build(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut d = Declarer::new(node);
    let s = settings(&mut d)?;
    let logger = node.logger().clone();
    let chunks = if s.background && !s.chunks.is_empty() {
        Some(
            ChunkBackground::open(&PathBuf::from(&s.chunks), s.slack, s.min_z)
                .with_context(|| format!("background_chunks:={}", s.chunks))?,
        )
    } else {
        None
    };
    let background_says = match (s.background, &chunks) {
        (false, _) => "off".to_string(),
        (true, Some(_)) => format!("on: the NDT chunks in {}", s.chunks),
        (true, None) => "on: /map".to_string(),
    };
    log_info!(
        &logger,
        "things (Rust): cubes within {:.2} m make a thing; kept {:.0} s unseen ({:.0} s within \
         {:.1} m); radar {}; the saved map as background {background_says}",
        s.groups.gap,
        s.tracker.keep,
        s.tracker.keep_blind,
        s.tracker.blind_range,
        if s.radar { "on" } else { "off" }
    );
    let use_map = s.background && chunks.is_none();
    let state = ThingsNode {
        groups: s.groups,
        tracker: Tracker::new(s.tracker),
        tf: Buffer::new(),
        car: [0.0, 0.0],
        car_velocity: [0.0, 0.0],
        cubes: Waiting::default(),
        radar: Waiting::default(),
        behind: 0.0,
        background: None,
        chunks,
        below: s.below,
        slack: s.slack,
        use_background: s.background,
        dropped_background: 0,
        names: s.names,
        namer: s.namer,
        min_score: s.min_score,
        lenses: Vec::new(),
        things: node.create_publisher("/world/things".keep_last(10))?,
        markers: node.create_publisher("/world/things/markers".keep_last(10))?,
        clock: node.get_clock(),
        logger,
    };
    let worker = node.create_worker(state);
    let mut keep: Vec<Box<dyn Any>> = Vec::new();
    keep.push(Box::new(worker.create_subscription(
        "/perception/voxels/cubes".keep_last(1).reliable(),
        |n: &mut ThingsNode, msg: PointCloud2| {
            n.cubes.add(seconds(&msg.header.stamp), msg);
            n.drain();
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/localization/odom/local".keep_last(10),
        |n: &mut ThingsNode, msg: Odometry| n.on_odom(&msg),
    )?));
    if s.radar {
        keep.push(Box::new(worker.create_subscription(
            "/perception/radar/clusters".sensor_data_qos(),
            |n: &mut ThingsNode, msg: PointCloud2| {
                if msg.width * msg.height > 0 {
                    n.radar.add(seconds(&msg.header.stamp), msg);
                    n.drain();
                }
            },
        )?));
    }
    if s.names {
        for topic in &s.camera_topics {
            keep.push(Box::new(worker.create_subscription(
                topic.as_str().keep_last(10),
                |n: &mut ThingsNode, msg: CameraInfo| n.on_camera_info(&msg),
            )?));
        }
        keep.push(Box::new(worker.create_subscription(
            "/perception/detections".keep_last(10),
            |n: &mut ThingsNode, msg: Detection2DArray| n.on_detections(&msg),
        )?));
    }
    if use_map {
        keep.push(Box::new(worker.create_subscription(
            "/map".keep_last(1).reliable().transient_local(),
            |n: &mut ThingsNode, msg: OccupancyGrid| n.on_map(&msg),
        )?));
    }
    // As tf2's listener: /tf keep last 100; /tf_static latched.
    keep.push(Box::new(worker.create_subscription(
        "/tf".keep_last(100).reliable(),
        |n: &mut ThingsNode, msg: TFMessage| n.on_tf(&msg, false),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/tf_static".keep_last(100).reliable().transient_local(),
        |n: &mut ThingsNode, msg: TFMessage| n.on_tf(&msg, true),
    )?));
    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / s.rate).node_time(),
        |n: &mut ThingsNode| n.publish(),
    )?));
    keep.push(Box::new(worker));
    keep.extend(d.finish());
    Ok(keep)
}

/// The worker's payload.
struct ThingsNode {
    groups: GroupShape,
    tracker: Tracker,
    tf: Buffer,
    /// base_link in odom at the last cloud (m).
    car: [f64; 2],
    /// m/s in odom.
    car_velocity: [f64; 2],
    /// Each stream's clouds waiting for their transform, oldest first.
    cubes: Waiting<PointCloud2>,
    radar: Waiting<PointCloud2>,
    /// s: the last waiting cloud's lead over the newest transform.
    behind: f64,
    background: Option<Background>,
    chunks: Option<ChunkBackground>,
    below: Option<f64>,
    slack: f64,
    use_background: bool,
    dropped_background: u64,
    names: bool,
    namer: Namer,
    min_score: f64,
    /// Each camera's lens by its frame.
    lenses: Vec<(String, Lens)>,
    things: Publisher<Things>,
    markers: Publisher<MarkerArray>,
    clock: Clock,
    logger: Logger,
}

/// Which stream a waiting cloud is in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stream {
    Cubes,
    Radar,
}

impl Stream {
    fn name(self) -> &'static str {
        match self {
            Self::Cubes => "cubes",
            Self::Radar => "radar",
        }
    }
}

fn ns(t: &Time) -> i64 {
    i64::from(t.sec) * 1_000_000_000 + i64::from(t.nanosec)
}

fn seconds(t: &Time) -> f64 {
    f64::from(t.sec) + f64::from(t.nanosec) * 1e-9
}

/// A time in s as a stamp, as the Python's `stamp_of`.
fn stamp_of(t: f64) -> Time {
    let sec = t.floor();
    Time {
        sec: sec as i32,
        nanosec: ((t - sec) * 1e9) as u32,
    }
}

fn pose_of(t: &Transform) -> Pose2 {
    let [x, y, z, w] = t.rotation;
    Pose2 {
        x: t.translation[0],
        y: t.translation[1],
        yaw: yaw_of(x, y, z, w),
    }
}

impl ThingsNode {
    /// s, this node's clock (the simulator's with use_sim_time).
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
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

    fn on_odom(&mut self, msg: &Odometry) {
        // The local EKF's twist is in base_link: turn it into odom.
        let q = &msg.pose.pose.orientation;
        let yaw = yaw_of(q.x, q.y, q.z, q.w);
        let v = &msg.twist.twist.linear;
        self.car_velocity = [
            v.x * yaw.cos() - v.y * yaw.sin(),
            v.x * yaw.sin() + v.y * yaw.cos(),
        ];
    }

    fn on_map(&mut self, msg: &OccupancyGrid) {
        let info = &msg.info;
        let (w, h) = (info.width as usize, info.height as usize);
        self.background = Background::from_grid(
            &msg.data,
            w,
            h,
            f64::from(info.resolution),
            [info.origin.position.x, info.origin.position.y],
            self.slack,
        );
        match self.background {
            Some(_) => log_info!(
                &self.logger,
                "background: the saved map, {w} x {h} cells at {:.2} m; walls grown {:.2} m",
                info.resolution,
                self.slack
            ),
            None => log_warn!(
                &self.logger,
                "background: /map has {} cells, not {w} x {h}: ignored",
                msg.data.len()
            ),
        }
    }

    fn on_camera_info(&mut self, msg: &CameraInfo) {
        if let Some(lens) = Lens::from_k(&msg.k, msg.width, msg.height) {
            let frame = msg.header.frame_id.clone();
            match self.lenses.iter_mut().find(|(f, _)| *f == frame) {
                Some((_, slot)) => *slot = lens,
                None => self.lenses.push((frame, lens)),
            }
        }
    }

    /// Keep each camera's newest boxes, with its lens and its mount on the car.
    fn on_detections(&mut self, msg: &Detection2DArray) {
        let frame = &msg.header.frame_id;
        let Some(lens) = self
            .lenses
            .iter()
            .find(|(f, _)| f == frame)
            .map(|(_, l)| *l)
        else {
            return;
        };
        let Ok(t) = self.tf.lookup(BASE, frame, None) else {
            return;
        };
        let [x, y, z, w] = t.rotation;
        let mount = CameraMount {
            position: t.translation,
            rotation: rotation(x, y, z, w),
            axes: Axes::of_frame(frame),
        };
        let seen = msg
            .detections
            .iter()
            .filter_map(|d| {
                // The highest score; the first of equals, as Python's max().
                let mut best: Option<&ObjectHypothesisWithPose> = None;
                for r in &d.results {
                    if best.is_none_or(|b| r.hypothesis.score > b.hypothesis.score) {
                        best = Some(r);
                    }
                }
                let best = best?;
                if best.hypothesis.score < self.min_score {
                    return None;
                }
                let (cx, cy) = (d.bbox.center.position.x, d.bbox.center.position.y);
                let (hw, hh) = (d.bbox.size_x / 2.0, d.bbox.size_y / 2.0);
                Some(Seen {
                    label: best.hypothesis.class_id.clone(),
                    score: best.hypothesis.score,
                    area: [cx - hw, cy - hh, cx + hw, cy + hh],
                })
            })
            .collect();
        let stamp = self.now();
        self.namer.set_camera(
            frame,
            Camera {
                lens,
                mount,
                seen,
                stamp,
            },
        );
    }

    /// What TF says about a cloud's stamp (tf_wait's `look`).
    fn look(tf: &Buffer, frame: &str, stamp_ns: i64, behind: &mut f64) -> Look {
        if tf.lookup(ODOM, frame, Some(stamp_ns)).is_ok() {
            return Look {
                found: true,
                newest: None,
            };
        }
        match tf.newest(ODOM, frame) {
            Ok(newest) => {
                let newest = newest.map(|n| n as f64 * 1e-9);
                if let Some(n) = newest {
                    *behind = stamp_ns as f64 * 1e-9 - n;
                }
                Look {
                    found: false,
                    newest,
                }
            }
            Err(_) => Look {
                found: false,
                newest: None,
            },
        }
    }

    /// Use every waiting cloud whose transform has come, oldest first; say the drops.
    fn drain(&mut self) {
        let now = self.now();
        for stream in [Stream::Cubes, Stream::Radar] {
            let (tf, behind) = (&self.tf, &mut self.behind);
            let queue = match stream {
                Stream::Cubes => &mut self.cubes,
                Stream::Radar => &mut self.radar,
            };
            let ready = queue.take(
                |_, msg| Self::look(tf, &msg.header.frame_id, ns(&msg.header.stamp), behind),
                now,
            );
            let dropped = std::mem::take(&mut queue.dropped);
            for (msg, step) in ready {
                let Some(pose) = self.pose(&msg, step) else {
                    continue;
                };
                match stream {
                    Stream::Cubes => self.use_cubes(&msg, pose),
                    Stream::Radar => self.use_clusters(&msg, pose),
                }
            }
            if dropped > 0 {
                log_warn!(
                    self.logger.throttle(Duration::from_secs(5)),
                    "{}: {dropped} cloud(s) dropped, no {ODOM} transform at their stamp (the \
                     newest was {:.2} s behind)",
                    stream.name(),
                    self.behind
                );
            }
        }
    }

    /// The cloud's frame in odom at its stamp, or the newest ([`Step::Newest`]).
    fn pose(&self, msg: &PointCloud2, step: Step) -> Option<Pose2> {
        let at = (step != Step::Newest).then(|| ns(&msg.header.stamp));
        match self.tf.lookup(ODOM, &msg.header.frame_id, at) {
            Ok(t) => Some(pose_of(&t)),
            Err(e) => {
                log_warn!(
                    self.logger.throttle(Duration::from_secs(5)),
                    "no {ODOM} <- {}: {e}",
                    msg.header.frame_id
                );
                None
            }
        }
    }

    /// The cubes not on the saved map; all of them without a map or a map pose.
    fn drop_background(&mut self, points: Vec<[f64; 3]>, frame: &str) -> Vec<[f64; 3]> {
        if !self.use_background
            || (self.background.is_none() && self.chunks.is_none())
            || points.is_empty()
        {
            return points;
        }
        let Ok(t) = self.tf.lookup("map", frame, None) else {
            return points;
        };
        let pose = pose_of(&t);
        if let Some(chunks) = self.chunks.as_mut() {
            let loads = chunks.loads;
            match chunks.update(pose.x, pose.y) {
                Ok(bg) => self.background = Some(bg.clone()),
                Err(e) => {
                    log_warn!(
                        self.logger.throttle(Duration::from_secs(5)),
                        "background: {e}"
                    );
                    return points;
                }
            }
            if chunks.loads != loads {
                if let Some(here) = chunks.here() {
                    log_info!(
                        &self.logger,
                        "background: the NDT chunks round ({}, {})",
                        here.i,
                        here.j
                    );
                }
            }
        }
        let Some(bg) = &self.background else {
            return points;
        };
        let kept = new_only(&points, pose, bg, self.below);
        self.dropped_background += (points.len() - kept.len()) as u64;
        log_info!(
            self.logger.throttle(Duration::from_secs(10)),
            "background: {} cubes on the saved map so far, not things",
            self.dropped_background
        );
        kept
    }

    fn use_cubes(&mut self, msg: &PointCloud2, pose: Pose2) {
        let rows = match read_columns(&view(msg), &["x", "y", "z"]) {
            Ok(rows) => rows,
            Err(e) => {
                log_warn!(
                    self.logger.throttle(Duration::from_secs(5)),
                    "/perception/voxels/cubes: {e}"
                );
                return;
            }
        };
        let points: Vec<[f64; 3]> = rows.iter().map(|r| [r[0], r[1], r[2]]).collect();
        let points = self.drop_background(points, &msg.header.frame_id);
        let found = to_frame(&group(&points, &self.groups), pose);
        self.car = [pose.x, pose.y];
        self.tracker
            .step(seconds(&msg.header.stamp), &found, self.car);
        if self.names {
            self.name_things();
        }
    }

    /// Each thing's name from the cameras (the Python's Tracker.name hook).
    fn name_things(&mut self) {
        let Ok(t) = self.tf.lookup(ODOM, BASE, None) else {
            return;
        };
        let car = pose_of(&t);
        let (c, s) = (car.yaw.cos(), car.yaw.sin());
        let now = self.now();
        let namer = &mut self.namer;
        self.tracker.rename(|track: &Track| {
            let (dx, dy) = (track.x[0] - car.x, track.x[1] - car.y);
            namer.name(
                track.id,
                c * dx + s * dy,
                -s * dx + c * dy,
                track.length,
                track.width,
                track.height,
                now,
            )
        });
    }

    fn use_clusters(&mut self, msg: &PointCloud2, pose: Pose2) {
        let rows = match read_columns(&view(msg), &["x", "y", "velocity", "cluster"]) {
            Ok(rows) => rows,
            Err(e) => {
                log_warn!(
                    self.logger.throttle(Duration::from_secs(5)),
                    "/perception/radar/clusters: {e}"
                );
                return;
            }
        };
        let points: Vec<RadarPoint> = rows
            .iter()
            .map(|r| RadarPoint {
                x: r[0],
                y: r[1],
                velocity: r[2],
                cluster: r[3],
            })
            .collect();
        let found = clusters(&points, pose, self.car_velocity);
        self.tracker.radar(seconds(&msg.header.stamp), &found);
    }

    fn publish(&mut self) {
        self.drain();
        let now = self.clock.now();
        let t = now.nsec as f64 * 1e-9;
        if self.tracker.time.is_none() {
            return;
        }
        self.tracker.expire(t, self.car);
        let shape = self.tracker.shape;
        let header = Header {
            stamp: Time {
                sec: now.nsec.div_euclid(1_000_000_000) as i32,
                nanosec: now.nsec.rem_euclid(1_000_000_000) as u32,
            },
            frame_id: ODOM.into(),
        };
        let out = Things {
            header,
            things: self
                .tracker
                .at(t)
                .iter()
                .map(|track| Thing {
                    id: track.id,
                    kind: track.kind.clone(),
                    x: track.x[0] as f32,
                    y: track.x[1] as f32,
                    vx: track.x[2] as f32,
                    vy: track.x[3] as f32,
                    length: track.length as f32,
                    width: track.width as f32,
                    height: track.height as f32,
                    position_sigma: track.position_sigma() as f32,
                    confidence: track.confidence(t, &shape) as f32,
                    last_seen: stamp_of(track.last_seen),
                    seen: track.seen,
                    radar: track.radar,
                    coasting: track.coasting,
                })
                .collect(),
        };
        let array = markers(&out);
        if let Err(e) = self.things.publish(out) {
            log_warn!(
                self.logger.throttle(Duration::from_secs(5)),
                "/world/things: {e}"
            );
        }
        if let Err(e) = self.markers.publish(array) {
            log_warn!(
                self.logger.throttle(Duration::from_secs(5)),
                "/world/things/markers: {e}"
            );
        }
    }
}

/// A cloud's layout and bytes, for things_core::cloud.
fn view(msg: &PointCloud2) -> CloudView<'_> {
    CloudView {
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
    }
}

fn colour(rgb: [f64; 3], a: f32) -> ColorRGBA {
    ColorRGBA {
        r: rgb[0] as f32,
        g: rgb[1] as f32,
        b: rgb[2] as f32,
        a,
    }
}

/// A box, a speed arrow and the words for each thing; DELETEALL first (the Python's
/// `markers`).
fn markers(things: &Things) -> MarkerArray {
    let header = &things.header;
    let mut out = vec![Marker {
        header: header.clone(),
        action: Marker::DELETEALL,
        ..Marker::default()
    }];
    for th in &things.things {
        let id = (th.id * 3) as i32;
        let (x, y, h) = (f64::from(th.x), f64::from(th.y), f64::from(th.height));
        let mut cube = Marker {
            header: header.clone(),
            ns: "things".into(),
            id,
            type_: Marker::CUBE,
            color: colour(looks::colour(&th.kind, th.coasting), 0.5),
            ..Marker::default()
        };
        cube.pose.position.x = x;
        cube.pose.position.y = y;
        cube.pose.position.z = h / 2.0;
        cube.pose.orientation.w = 1.0;
        cube.scale.x = f64::from(th.length).max(0.1);
        cube.scale.y = f64::from(th.width).max(0.1);
        cube.scale.z = h.max(0.1);
        let mut arrow = Marker {
            header: header.clone(),
            ns: "things".into(),
            id: id + 1,
            type_: Marker::ARROW,
            points: vec![
                PointMsg { x, y, z: h },
                PointMsg {
                    x: f64::from(th.x + th.vx),
                    y: f64::from(th.y + th.vy),
                    z: h,
                },
            ],
            color: colour([0.2, 0.6, 1.0], 1.0),
            ..Marker::default()
        };
        arrow.scale.x = 0.08;
        arrow.scale.y = 0.2;
        arrow.scale.z = 0.2;
        let mut text = Marker {
            header: header.clone(),
            ns: "things".into(),
            id: id + 2,
            type_: Marker::TEXT_VIEW_FACING,
            color: colour([1.0, 1.0, 1.0], 1.0),
            text: looks::caption(
                th.id,
                &th.kind,
                f64::from(th.vx),
                f64::from(th.vy),
                th.radar,
            ),
            ..Marker::default()
        };
        text.pose.position.x = x;
        text.pose.position.y = y;
        text.pose.position.z = h + 0.5;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.4;
        out.extend([cube, arrow, text]);
    }
    MarkerArray { markers: out }
}
