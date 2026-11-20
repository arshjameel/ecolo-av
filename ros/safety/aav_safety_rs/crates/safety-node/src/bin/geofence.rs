//! geofence, in Rust: stops every program before the car's stopping distance reaches the
//! fence's edge. The same node as aav_safety's geofence_node.py, topic for topic.
//!
//! Subscribes  /localization/odom/global  nav_msgs/Odometry: the global EKF, map frame
//!                                        (position, heading, forward speed, covariance)
//!             /localization/gps/odom     nav_msgs/Odometry: each GPS fix into the EKF;
//!                                        only its arrival counts (geofence.launch.py's
//!                                        fix_topic remaps it)
//! Publishes   /safety/geofence/lock      std_msgs/Bool, at rate_hz: twist_mux's geofence
//!                                        lock (priority 51). True while STARTING, NEAR,
//!                                        OUTSIDE, JUMP, LOST or COASTING
//!             /safety/geofence/state     std_msgs/String, at rate_hz: `STATE: why`
//!             /safety/geofence/fence     geometry_msgs/PolygonStamped, map frame,
//!                                        transient local, depth 1: the fence
//! Serves      /safety/geofence/clear     std_srvs/Trigger: a person drove the car back in
//!                                        after OUTSIDE, or understood a JUMP
//!
//! Parameters  geofence_node.py's, the same names: set by geofence.launch.py (the fence:
//!             site or centre_x, centre_y, radius; datum, stands, gps, coasting, and the
//!             stopping numbers, with no defaults) and config/geofence.yaml (margin,
//!             hysteresis, max_std, max_age, gps_max_age, settle_time, jump_margin,
//!             rate_hz, sites.<name>.datum and .corners). One more: vehicle_yaml ('':
//!             aav_description's vehicle.yaml, found on AMENT_PREFIX_PATH), for the car's
//!             corners. The rules: safety-core's geofence module.
//!
//! Times are on the steady clock (arrival), so a jump in sim time can't age a pose.

use std::any::Any;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Context as _, Result};
use rclrs::*;
use ros_env::geometry_msgs::msg::{Point32, Polygon as Outline, PolygonStamped};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::std_msgs::msg::{Bool, Header, String as Text};
use ros_env::std_srvs::srv::{Trigger, Trigger_Request, Trigger_Response};
use safety_core::geofence::{
    footprint_points, site_fence, Circle, Fence, Geofence, Mode, Motion, Pose, Settings, State,
};
use safety_core::pyfmt::{f1, fixed, repr};
use safety_node::{lock, run, Params};

/// s between the stands mode's reminders.
const LOUD: f64 = 5.0;

struct Node_ {
    fence: Geofence,
    start: Instant,
    was: Option<State>,
    reminded: f64,
    logger: Logger,
    lock: Publisher<Bool>,
    state: Publisher<Text>,
}

impl Node_ {
    fn now(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }

    fn on_pose(&mut self, msg: &Odometry) {
        let q = &msg.pose.pose.orientation;
        let cov = &msg.pose.covariance;
        let spread = safety_core::pyfmt::max(cov[0], cov[7]);
        let std = if spread >= 0.0 {
            spread.sqrt()
        } else {
            f64::NAN
        };
        let now = self.now();
        self.fence.update(
            now,
            Pose {
                x: msg.pose.pose.position.x,
                y: msg.pose.pose.position.y,
                yaw: (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
                speed: msg.twist.twist.linear.x,
                std,
            },
        );
    }

    fn tick(&mut self) {
        let now = self.now();
        let v = self.fence.check(now);
        let throttle = Duration::from_secs(2);
        if let Err(e) = self.lock.publish(Bool { data: v.locked }) {
            log_warn!(self.logger.throttle(throttle), "lock: {e}");
        }
        let line = format!("{}: {}", v.state, v.why);
        if let Err(e) = self.state.publish(Text { data: line }) {
            log_warn!(self.logger.throttle(throttle), "state: {e}");
        }
        if self.was != Some(v.state) && v.state == State::Ok {
            log_info!(&self.logger, "geofence {}: {}", v.state, v.why);
            self.was = Some(v.state);
        } else if self.was != Some(v.state) {
            let held = if v.locked {
                ": every program is held; people may drive"
            } else {
                ""
            };
            log_error!(&self.logger, "geofence {}: {}{held}", v.state, v.why);
            self.was = Some(v.state);
        } else if v.state == State::Stands && now - self.reminded > LOUD {
            log_error!(&self.logger, "geofence {}: {}", v.state, v.why);
            self.reminded = now;
        }
    }

    fn clear(&mut self) -> Trigger_Response {
        let now = self.now();
        let (success, message) = self.fence.clear(now);
        if success {
            log_warn!(&self.logger, "clear: {message}");
        } else {
            log_error!(&self.logger, "clear refused: {message}");
        }
        Trigger_Response { success, message }
    }
}

/// aav_description's vehicle.yaml, found on AMENT_PREFIX_PATH (as ament_index does).
fn installed_vehicle_yaml() -> Option<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .ok()?
        .split(':')
        .map(|prefix| PathBuf::from(prefix).join("share/aav_description/config/vehicle.yaml"))
        .find(|p| p.is_file())
}

/// The car's corners in base_link, from vehicle.yaml's body (vehicle.footprint_points).
fn footprint(path: &str) -> Result<Vec<(f64, f64)>> {
    let path = if path.is_empty() {
        installed_vehicle_yaml()
            .context("vehicle_yaml is empty and aav_description's vehicle.yaml isn't installed")?
    } else {
        PathBuf::from(path)
    };
    let text = std::fs::read_to_string(&path).with_context(|| path.display().to_string())?;
    let yaml: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&text).with_context(|| path.display().to_string())?;
    let body = &yaml["vehicle"]["body"];
    let number = |key: &str| {
        body[key]
            .as_f64()
            .with_context(|| format!("{}: vehicle.body.{key} is not a number", path.display()))
    };
    Ok(footprint_points(
        number("length")?,
        number("width")?,
        number("rear_overhang")?,
    ))
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut p = Params::new(node);
    let d = Settings::default();
    let settings = Settings {
        margin: p.number("margin", d.margin)?,
        hysteresis: p.number("hysteresis", d.hysteresis)?,
        max_std: p.number("max_std", d.max_std)?,
        max_age: p.number("max_age", d.max_age)?,
        gps_max_age: p.number("gps_max_age", d.gps_max_age)?,
        settle_time: p.number("settle_time", d.settle_time)?,
        jump_margin: p.number("jump_margin", d.jump_margin)?,
    };
    // No defaults for the stopping numbers: the launch file works them out from the
    // configs, and a node started without them must not guess (NaN is refused).
    let motion = Motion {
        reaction_time: p.number("reaction_time", f64::NAN)?,
        braking: p.number("braking", f64::NAN)?,
        max_accel: p.number("max_accel", f64::NAN)?,
        max_speed: p.number("max_speed", f64::NAN)?,
        max_reverse_speed: p.number("max_reverse_speed", f64::NAN)?,
        max_curvature: p.number("max_curvature", f64::NAN)?,
    };
    let site = p.text("site", "")?;
    let rate = p.number("rate_hz", 10.0)?;
    let stands = p.flag("stands", false)?;
    let (fence, place) = if site.is_empty() {
        let cx = p.number("centre_x", 0.0)?;
        let cy = p.number("centre_y", 0.0)?;
        let radius = p.number("radius", 30.0)?;
        (
            Fence::Circle(Circle::new(cx, cy, radius)?),
            format!(
                "a {} m circle round the start spot ({}, {})",
                fixed(radius, 0),
                f1(cx),
                f1(cy)
            ),
        )
    } else {
        let datum = p.numbers("datum", &[0.0])?;
        let corners = p.numbers(&format!("sites.{site}.corners"), &[0.0])?;
        let site_datum = p.numbers(&format!("sites.{site}.datum"), &[0.0])?;
        if corners == [0.0] {
            bail!("site {site}: no sites.{site}.corners in config/geofence.yaml");
        }
        let fence = site_fence(&site, &corners, &site_datum, &datum, 1e-7)?;
        let shown: Vec<String> = site_datum.iter().map(|v| repr(*v)).collect();
        (
            Fence::Polygon(fence),
            format!("site {site} (datum [{}])", shown.join(", ")),
        )
    };
    let vehicle_yaml = p.text("vehicle_yaml", "")?;
    let mode = Mode {
        gps: p.flag("gps", true)?,
        coasting: p.flag("coasting", false)?,
        stands,
    };
    if rate.is_nan() || rate <= 0.0 {
        bail!("rate_hz:={rate}: a rate must be above 0 Hz");
    }
    let geofence = Geofence::new(footprint(&vehicle_yaml)?, fence, motion, settings, mode)?;
    let (forward, _) = geofence.stopping(motion.max_speed);
    let logger = node.logger().clone();
    log_info!(
        &logger,
        "the fence: {place}; stopping {} m from {} m/s (reaction {} s, braking {} m/s^2) plus \
         a {} m margin and the pose spread",
        f1(forward),
        f1(motion.max_speed),
        fixed(motion.reaction_time, 2),
        fixed(motion.braking, 2),
        f1(settings.margin)
    );
    if stands {
        log_error!(
            &logger,
            "STANDS MODE: no geofence. The wheels must be off the ground"
        );
    }

    let mut keep = p.into_keep();
    let outline: Vec<Point32> = geofence
        .fence()
        .outline()
        .iter()
        .map(|&(x, y)| Point32 {
            x: x as f32,
            y: y as f32,
            z: 0.0,
        })
        .collect();
    let drawn: Publisher<PolygonStamped> =
        node.create_publisher("/safety/geofence/fence".keep_last(1).transient_local())?;
    drawn.publish(PolygonStamped {
        header: Header {
            stamp: node.get_clock().now().to_ros_msg().unwrap_or_default(),
            frame_id: "map".to_string(),
        },
        polygon: Outline { points: outline },
    })?;
    keep.push(Box::new(drawn));

    let me = Arc::new(Mutex::new(Node_ {
        fence: geofence,
        start: Instant::now(),
        was: None,
        reminded: 0.0,
        logger,
        lock: node.create_publisher("/safety/geofence/lock")?,
        state: node.create_publisher("/safety/geofence/state")?,
    }));
    let m = Arc::clone(&me);
    keep.push(Box::new(node.create_subscription(
        "/localization/odom/global",
        move |msg: Odometry| lock(&m).on_pose(&msg),
    )?));
    let m = Arc::clone(&me);
    keep.push(Box::new(node.create_subscription(
        "/localization/gps/odom",
        move |_: Odometry| {
            let mut n = lock(&m);
            let now = n.now();
            n.fence.gps_arrived(now);
        },
    )?));
    let m = Arc::clone(&me);
    keep.push(Box::new(node.create_service::<Trigger, _>(
        "/safety/geofence/clear",
        move |_: Trigger_Request| lock(&m).clear(),
    )?));
    let m = Arc::clone(&me);
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        move || lock(&m).tick(),
    )?));
    keep.push(Box::new(me));
    Ok(keep)
}

fn main() -> Result<()> {
    run("geofence", start)
}
