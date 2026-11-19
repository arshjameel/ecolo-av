//! lane_finder: the lane lines in the front camera's pictures, as /perception/lane/geometry,
//! in Rust. A drop-in for aav_lane_finder's Python node: the same name, topics, QoS,
//! parameters and answers.
//!
//! Subscribes  /sensors/camera/front_left/image_raw    sensor_msgs/Image (reliable, the newest
//!                                                     only): rgb8, bgr8, rgba8, bgra8 or mono8
//!             /sensors/camera/front_left/camera_info  sensor_msgs/CameraInfo: the lens's focal
//!                                                     length, centre and distortion
//! Publishes   /perception/lane/geometry               aav_lane_geometry/LaneGeometry, one per
//!                                                     picture, in base_link, each line's
//!                                                     paint (white or yellow) too: the
//!                                                     costmap's camera lanes and lane_pose
//!                                                     read it
//!             /perception/lane/image                  sensor_msgs/Image, bgr8: the bird's-eye
//!                                                     view with the paint, the windows and the
//!                                                     lines found (only with debug_image)
//! Parameters  aav_lane_finder's (config/lane_finder.yaml): camera_xyz, camera_rpy, focal_px,
//!             use_camera_info, x_min, x_max, y_max, resolution, every field of Settings
//!             (paint_contrast ... wheelbase, yellow_ratio), debug_image.
//!
//! No serving, no calls. The camera's place on the car comes from vehicle.yaml, through
//! the launch file; until camera_info arrives, the node uses focal_px and the middle of
//! the picture. The maths is the `lane-core` crate, with no ROS.

mod params;

use std::{
    any::Any,
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::Result;
use lane_core::{
    draw, BirdsEye, Distortion, Encoding, Grid, Lane, LaneFinder, Lens, Mount, Picture, Settings,
};
use params::Declarer;
use rclrs::*;
use ros_env::aav_lane_geometry::msg::LaneGeometry;
use ros_env::sensor_msgs::msg::{CameraInfo, Image};
use ros_env::std_msgs::msg::Header;

const EVERY: Duration = Duration::from_secs(10);
const WARN_EVERY: Duration = Duration::from_secs(30);
const ERROR_EVERY: Duration = Duration::from_secs(5);

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("lane_finder")?;
    let _keep = build(&node)?;
    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The node's settings, from its parameters (the Python's names and defaults).
struct Config {
    mount: Mount,
    focal: f64,
    use_info: bool,
    grid: Grid,
    settings: Settings,
    debug: bool,
}

fn config(d: &mut Declarer<'_>) -> Result<Config> {
    let mount = Mount {
        xyz: d.three("camera_xyz", [1.80, 0.29, 1.20])?,
        rpy: d.three("camera_rpy", [0.0, 0.0, 0.0])?,
    };
    let focal = d.number("focal_px", 343.0)?;
    let use_info = d.flag("use_camera_info", true)?;
    let g = Grid::default();
    let grid = Grid {
        x_min: d.number("x_min", g.x_min)?,
        x_max: d.number("x_max", g.x_max)?,
        y_max: d.number("y_max", g.y_max)?,
        resolution: d.number("resolution", g.resolution)?,
    };
    anyhow::ensure!(
        grid.resolution > 0.0 && grid.rows() > 0 && grid.cols() > 0,
        "x_min, x_max, y_max, resolution: an empty view"
    );
    let s = Settings::default();
    let settings = Settings {
        paint_contrast: d.number("paint_contrast", s.paint_contrast)?,
        paint_width: d.number("paint_width", s.paint_width)?,
        windows: d.count("windows", s.windows)?,
        window_margin: d.number("window_margin", s.window_margin)?,
        min_pixels: d.count("min_pixels", s.min_pixels)?,
        min_span: d.number("min_span", s.min_span)?,
        fit_scale: d.number("fit_scale", s.fit_scale)?,
        min_confidence: d.number("min_confidence", s.min_confidence)?,
        lane_width: d.number("lane_width", s.lane_width)?,
        min_lane_width: d.number("min_lane_width", s.min_lane_width)?,
        max_lane_width: d.number("max_lane_width", s.max_lane_width)?,
        lookahead: d.number("lookahead", s.lookahead)?,
        wheelbase: d.number("wheelbase", s.wheelbase)?,
        yellow_ratio: d.number("yellow_ratio", s.yellow_ratio)?,
    };
    anyhow::ensure!(settings.windows > 0, "windows:=0: needs at least one");
    let debug = d.flag("debug_image", false)?;
    Ok(Config {
        mount,
        focal,
        use_info,
        grid,
        settings,
        debug,
    })
}

/// Everything the node keeps alive: parameters, subscriptions, the worker.
fn build(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut d = Declarer::new(node);
    let c = config(&mut d)?;
    let logger = node.logger().clone();
    log_info!(
        &logger,
        "lane_finder (Rust): camera at {:?} m, turned {:?} rad; looking {}-{} m ahead; the \
         offset {} m ahead",
        c.mount.xyz,
        c.mount.rpy,
        c.grid.x_min,
        c.grid.x_max,
        c.settings.lookahead
    );
    // Only the newest picture, and reliable: a camera picture is too big for one UDP
    // packet, and best effort loses the whole picture when one piece is lost.
    let state = FinderNode {
        mount: c.mount,
        focal: c.focal,
        use_info: c.use_info,
        grid: c.grid,
        settings: c.settings,
        info: None,
        finder: None,
        seen: None,
        frames: 0,
        busy: Duration::ZERO,
        since: Instant::now(),
        geometry: node.create_publisher("/perception/lane/geometry".keep_last(10))?,
        debug: if c.debug {
            Some(node.create_publisher("/perception/lane/image".keep_last(2))?)
        } else {
            None
        },
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

/// The worker's payload.
struct FinderNode {
    mount: Mount,
    focal: f64,
    use_info: bool,
    grid: Grid,
    settings: Settings,
    info: Option<CameraInfo>,
    finder: Option<LaneFinder>,
    /// Which lines the last picture showed (left, right).
    seen: Option<(bool, bool)>,
    frames: u32,
    busy: Duration,
    since: Instant,
    geometry: Publisher<LaneGeometry>,
    debug: Option<Publisher<Image>>,
    logger: Logger,
}

impl FinderNode {
    /// The lens: camera_info's, if it describes pictures of this size.
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
        let distortion = match Distortion::from_camera_info(&info.distortion_model, &info.d) {
            Ok(d) => d,
            Err(e) => {
                log_warn!(
                    self.logger.throttle(WARN_EVERY),
                    "ignoring the distortion: {e}"
                );
                None
            }
        };
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

    /// The finder for this lens: a new look-up table when the lens changes, keeping the
    /// lane's width it has seen.
    fn finder_for(&mut self, width: usize, height: usize) -> &mut LaneFinder {
        let lens = self.lens_for(width, height);
        match self.finder.take() {
            Some(same) if *same.view().lens() == lens => self.finder.insert(same),
            old => {
                let width_seen = old.map_or(self.settings.lane_width, |f| f.width);
                let view = BirdsEye::new(lens, self.mount, self.grid);
                let mut finder = LaneFinder::new(view, self.settings);
                finder.width = width_seen;
                let near = finder.view().nearest_seen().map_or_else(
                    || "nowhere in the view".to_string(),
                    |x| format!("{x:.1} m ahead"),
                );
                log_info!(
                    &self.logger,
                    "lens: {} x {}, focal {:.1} px; the road is seen from {near}",
                    lens.width,
                    lens.height,
                    lens.fx
                );
                self.finder.insert(finder)
            }
        }
    }

    fn on_image(&mut self, msg: Image) {
        let t0 = Instant::now();
        let (w, h) = (msg.width as usize, msg.height as usize);
        let lane = Encoding::from_ros(&msg.encoding)
            .and_then(|e| Picture::new(&msg.data, w, h, msg.step as usize, e))
            .and_then(|picture| self.finder_for(w, h).find(&picture));
        let lane = match lane {
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
        if let (Some(publisher), Some(view)) =
            (&self.debug, self.finder.as_ref().and_then(LaneFinder::last))
        {
            let picture = draw(view, &lane, &self.grid, 2);
            let out = Image {
                header: msg.header.clone(),
                height: picture.height as u32,
                width: picture.width as u32,
                encoding: "bgr8".into(),
                is_bigendian: 0,
                step: 3 * picture.width as u32,
                data: picture.bgr,
            };
            if let Err(e) = publisher.publish(out) {
                log_warn!(
                    self.logger.throttle(ERROR_EVERY),
                    "/perception/lane/image: {e}"
                );
            }
        }
        self.report(&lane, t0.elapsed());
    }

    fn report(&mut self, lane: &Lane, busy: Duration) {
        let seen = (lane.left.is_some(), lane.right.is_some());
        if self.seen != Some(seen) {
            let names: Vec<&str> = [("left", seen.0), ("right", seen.1)]
                .iter()
                .filter(|(_, s)| *s)
                .map(|(n, _)| *n)
                .collect();
            let names = if names.is_empty() {
                "none".to_string()
            } else {
                names.join(" and ")
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

/// The lane as LaneGeometry, stamped with the picture's time, in base_link.
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
