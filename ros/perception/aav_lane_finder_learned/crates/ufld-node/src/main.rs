//! lane_finder, learned (TODO 262b): the lane lines in the front camera's pictures, found by
//! our UFLD-v2-style model through ONNX Runtime, as /perception/lane/geometry. A challenger
//! to the classic finder (`lane_finder_impl:=learned`): the same name, topics, QoS and
//! message, the lines fitted the same way.
//!
//! Subscribes  /sensors/camera/front_left/image_raw    sensor_msgs/Image (reliable, the newest
//!                                                     only): rgb8, bgr8, rgba8, bgra8 or mono8
//!             /sensors/camera/front_left/camera_info  sensor_msgs/CameraInfo: the lens
//! Publishes   /perception/lane/geometry               aav_lane_geometry/LaneGeometry, one per
//!                                                     picture, in base_link; each line's
//!                                                     paint (white or yellow) from the
//!                                                     model's colour head
//! Parameters  aav_lane_finder's (config/lane_finder.yaml): camera_xyz, camera_rpy, focal_px,
//!             use_camera_info, the Settings it shares (min_confidence, fit_scale, min_span,
//!             lane_width, min_lane_width, max_lane_width, lookahead, wheelbase); the others
//!             (the bird's-eye grid, the paint, the windows, debug_image) are read and unused.
//!             Its own: model (an .onnx with its .json beside; empty = models/lane-ufld.onnx),
//!             device (auto, cpu, coreml, cuda, tensorrt), exist_threshold (0.5).
//!
//! No serving, no calls. The maths is the `ufld-core` crate, with no ROS.

mod params;

use std::{
    any::Any,
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use lane_core::{Distortion, Encoding, Lane, Lens, Mount, Picture, Settings};
use params::Declarer;
use rclrs::*;
use ros_env::aav_lane_geometry::msg::LaneGeometry;
use ros_env::sensor_msgs::msg::{CameraInfo, Image};
use ros_env::std_msgs::msg::Header;
use ufld_core::{decode, input, paints, Learned, Model, Provider};

const EVERY: Duration = Duration::from_secs(10);
const WARN_EVERY: Duration = Duration::from_secs(30);
const ERROR_EVERY: Duration = Duration::from_secs(5);
const DEFAULT_MODEL: &str = "models/lane-ufld.onnx";

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("lane_finder")?;
    let _keep = build(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The model file: an absolute path, or one from the repository's root ($PIXI_PROJECT_ROOT,
/// else the working folder), as the detector finds its model.
fn model_path(model: &str) -> Result<PathBuf> {
    let model = match model.trim() {
        "" => DEFAULT_MODEL,
        m => m,
    };
    let path = match model.strip_prefix("~/") {
        Some(rest) => PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(rest),
        None => PathBuf::from(model),
    };
    let root = std::env::var_os("PIXI_PROJECT_ROOT")
        .map(PathBuf::from)
        .map_or_else(std::env::current_dir, Ok)?;
    let path = root.join(path);
    if !path.exists() {
        bail!(
            "{} is missing: pixi run -e ml python3 tools/lane_data.py, then tools/lane_train.py \
             (aav_lane_finder_learned's README)",
            path.display()
        );
    }
    Ok(path)
}

fn settings(d: &mut Declarer<'_>) -> Result<Settings> {
    let s = Settings::default();
    // The classic finder's own settings: declared so its YAML loads, not used here.
    for (name, default) in [
        ("paint_contrast", s.paint_contrast),
        ("paint_width", s.paint_width),
        ("window_margin", s.window_margin),
        ("x_min", 4.0),
        ("x_max", 16.0),
        ("y_max", 4.0),
        ("resolution", 0.05),
    ] {
        d.number(name, default)?;
    }
    d.count("windows", s.windows)?;
    d.count("min_pixels", s.min_pixels)?;
    d.flag("debug_image", false)?;
    Ok(Settings {
        min_span: d.number("min_span", s.min_span)?,
        fit_scale: d.number("fit_scale", s.fit_scale)?,
        min_confidence: d.number("min_confidence", s.min_confidence)?,
        lane_width: d.number("lane_width", s.lane_width)?,
        min_lane_width: d.number("min_lane_width", s.min_lane_width)?,
        max_lane_width: d.number("max_lane_width", s.max_lane_width)?,
        lookahead: d.number("lookahead", s.lookahead)?,
        wheelbase: d.number("wheelbase", s.wheelbase)?,
        ..s
    })
}

fn build(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut d = Declarer::new(node);
    let mount = Mount {
        xyz: d.three("camera_xyz", [1.80, 0.29, 1.20])?,
        rpy: d.three("camera_rpy", [0.0, 0.0, 0.0])?,
    };
    let focal = d.number("focal_px", 343.0)?;
    let use_info = d.flag("use_camera_info", true)?;
    let settings = settings(&mut d)?;
    let threshold = d.number("exist_threshold", 0.5)?;
    let path = model_path(&d.text("model", "")?)?;
    let provider = Provider::from_device(&d.text("device", "auto")?)?;
    let logger = node.logger().clone();
    let t0 = Instant::now();
    let model = Model::open(&path, provider)
        .with_context(|| format!("{}: the ONNX model", path.display()))?;
    log_info!(
        &logger,
        "lane_finder (learned, Rust): {} on {provider}{}, loaded in {:.1} s; camera at {:?} m, \
         turned {:?} rad; lines fitted {:?} m ahead",
        path.display(),
        model.coverage().map_or_else(String::new, |c| format!(
            " ({} of {} nodes, {} pieces)",
            c.taken, c.nodes, c.partitions
        )),
        t0.elapsed().as_secs_f64(),
        mount.xyz,
        mount.rpy,
        model.meta().fit_x
    );
    let state = FinderNode {
        learned: Learned::new(model.meta().clone(), settings),
        model,
        mount,
        focal,
        use_info,
        threshold,
        info: None,
        seen: None,
        frames: 0,
        busy: Duration::ZERO,
        since: Instant::now(),
        geometry: node.create_publisher("/perception/lane/geometry".keep_last(10))?,
        logger,
    };
    let worker = node.create_worker(state);
    let mut keep: Vec<Box<dyn Any>> = vec![
        Box::new(
            worker.create_subscription(
                "/sensors/camera/front_left/camera_info"
                    .keep_last(1)
                    .reliable(),
                |n: &mut FinderNode, msg: CameraInfo| n.info = Some(msg),
            )?,
        ),
        Box::new(
            worker.create_subscription(
                "/sensors/camera/front_left/image_raw"
                    .keep_last(1)
                    .reliable(),
                |n: &mut FinderNode, msg: Image| n.on_image(msg),
            )?,
        ),
        Box::new(worker),
    ];
    keep.extend(d.finish());
    Ok(keep)
}

struct FinderNode {
    model: Model,
    learned: Learned,
    mount: Mount,
    focal: f64,
    use_info: bool,
    threshold: f64,
    info: Option<CameraInfo>,
    seen: Option<(bool, bool)>,
    frames: u32,
    busy: Duration,
    since: Instant,
    geometry: Publisher<LaneGeometry>,
    logger: Logger,
}

impl FinderNode {
    /// The lens: camera_info's, if it describes pictures of this size (the classic node's).
    fn lens_for(&self, width: usize, height: usize) -> Lens {
        let pinhole = Lens::pinhole(width, height, self.focal);
        let Some(info) = self.info.as_ref().filter(|i| self.use_info && i.k[0] > 0.0) else {
            return pinhole;
        };
        if (info.width as usize, info.height as usize) != (width, height) {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "camera_info is for {} x {} pictures, these are {width} x {height}: using \
                 focal_px",
                info.width,
                info.height
            );
            return pinhole;
        }
        let distortion = Distortion::from_camera_info(&info.distortion_model, &info.d)
            .unwrap_or_else(|e| {
                log_warn!(
                    self.logger.throttle(WARN_EVERY),
                    "ignoring the distortion: {e}"
                );
                None
            });
        Lens {
            width,
            height,
            fx: info.k[0],
            fy: info.k[4],
            cx: info.k[2],
            cy: info.k[5],
            distortion,
        }
    }

    fn find(&mut self, msg: &Image) -> anyhow::Result<Lane> {
        let (w, h) = (msg.width as usize, msg.height as usize);
        let picture = Picture::new(
            &msg.data,
            w,
            h,
            msg.step as usize,
            Encoding::from_ros(&msg.encoding)?,
        )?;
        let x = input(&picture, self.model.meta())?;
        let answers = self.model.run(&x)?;
        let lines = decode(
            &answers.loc,
            &answers.exist,
            self.model.meta(),
            self.threshold,
        )?;
        let paints = match &answers.colour {
            Some(c) => paints(c, self.model.meta())?,
            None => Vec::new(),
        };
        let lens = self.lens_for(w, h);
        Ok(self.learned.lane(&lines, &paints, &lens, &self.mount)?)
    }

    fn on_image(&mut self, msg: Image) {
        let t0 = Instant::now();
        let lane = match self.find(&msg) {
            Ok(lane) => lane,
            Err(e) => {
                log_error!(
                    self.logger.throttle(ERROR_EVERY),
                    "lane finding failed: {e}"
                );
                return;
            }
        };
        if let Err(e) = self.geometry.publish(message(&lane, &msg.header)) {
            log_warn!(
                self.logger.throttle(ERROR_EVERY),
                "/perception/lane/geometry: {e}"
            );
        }
        self.report(&lane, t0.elapsed());
    }

    fn report(&mut self, lane: &Lane, busy: Duration) {
        let seen = (lane.left.is_some(), lane.right.is_some());
        if self.seen != Some(seen) {
            let names = match seen {
                (true, true) => "left and right",
                (true, false) => "left",
                (false, true) => "right",
                (false, false) => "none",
            };
            log_info!(&self.logger, "lines seen: {names}");
            self.seen = Some(seen);
        }
        self.frames += 1;
        self.busy += busy;
        if self.since.elapsed() >= EVERY {
            log_info!(
                &self.logger,
                "{} pictures in 10 s, {:.1} ms each",
                self.frames,
                1000.0 * self.busy.as_secs_f64() / f64::from(self.frames)
            );
            self.frames = 0;
            self.busy = Duration::ZERO;
            self.since = Instant::now();
        }
    }
}

/// The lane as LaneGeometry, stamped with the picture's time, in base_link (the classic
/// node's `message`).
fn message(lane: &Lane, picture: &Header) -> LaneGeometry {
    let mut msg = LaneGeometry {
        header: Header {
            stamp: picture.stamp.clone(),
            frame_id: "base_link".into(),
        },
        left_curvature_m: lane.left_radius as f32,
        right_curvature_m: lane.right_radius as f32,
        mean_curvature_m: lane.mean_radius as f32,
        curve_direction: lane.curve_direction as f32,
        vehicle_offset_m: lane.offset as f32,
        lane_width_m: lane.width as f32,
        left_confidence: lane.left_confidence as f32,
        right_confidence: lane.right_confidence as f32,
        left_lane_detected: lane.left.is_some(),
        right_lane_detected: lane.right.is_some(),
        both_lanes_detected: lane.left.is_some() && lane.right.is_some(),
        steering_hint: lane.steering_hint as f32,
        left_paint: lane.left_paint.code(),
        right_paint: lane.right_paint.code(),
        ..LaneGeometry::default()
    };
    if let Some(l) = &lane.left {
        (msg.left_poly_a, msg.left_poly_b, msg.left_poly_c) = (l.a as f32, l.b as f32, l.c as f32);
    }
    if let Some(r) = &lane.right {
        (msg.right_poly_a, msg.right_poly_b, msg.right_poly_c) =
            (r.a as f32, r.b as f32, r.c as f32);
    }
    msg
}
