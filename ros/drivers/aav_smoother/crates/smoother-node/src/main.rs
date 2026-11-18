//! car_smoother: our velocity smoother for a car, in Nav2's velocity_smoother's place.
//!
//! Subscribes  cmd_vel              geometry_msgs/TwistStamped: the follower's command
//!                                  (navigation.launch.py remaps it to cmd_vel_nav)
//!             odom_topic           nav_msgs/Odometry, only with feedback CLOSED_LOOP: the
//!                                  measured speed (the latest, not Nav2's average;
//!                                  older than velocity_timeout: not used, the last
//!                                  speed sent instead)
//! Publishes   cmd_vel_smoothed     geometry_msgs/TwistStamped, every 1/smoothing_frequency s
//!                                  while there is a command (remapped to /drivers/nav/cmd_vel
//!                                  or /control/cmd_vel_smoothed): the speed within the
//!                                  limits, the turn rate from where the wheels can be
//! Serves      nothing; calls nothing.
//! Parameters  Nav2's own, read once at start: smoothing_frequency, feedback, max_velocity,
//!             min_velocity, max_accel, max_decel, deadband_velocity, velocity_timeout,
//!             odom_topic, enable_stamped_cmd_vel (true only),
//!             stamp_smoothed_velocity_with_smoothing_time; and the car's: vehicle_yaml
//!             (the car model's numbers; '' = aav_description's), steering_rate (rad/s; 0 =
//!             vehicle.yaml's steer_rate), min_speed_for_steering (ego's).
//!
//! Not a lifecycle node: it runs from the start. The maths is the `car-smoother` crate;
//! this file only moves messages.

use std::{path::PathBuf, sync::Arc, time::Duration};

use anyhow::{bail, Context as _, Result};
use car_model::ActuatorConfig;
use car_smoother::{Car, Feedback, Nav2Limits, Reading, Settings, Smoother};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::geometry_msgs::msg::{Twist, TwistStamped, Vector3};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::std_msgs::msg::Header;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("velocity_smoother")?;

    let mut d = Declarer {
        node: &node,
        handles: Vec::new(),
    };
    if !d.flag("enable_stamped_cmd_vel", true)? {
        bail!("enable_stamped_cmd_vel:=false: car_smoother speaks TwistStamped only");
    }
    let frequency = d.number("smoothing_frequency", 20.0)?;
    let feedback: Feedback = d.text("feedback", "OPEN_LOOP")?.parse()?;
    let limits = Nav2Limits {
        max_velocity: d.numbers("max_velocity", &[0.5, 0.0, 2.5])?,
        min_velocity: d.numbers("min_velocity", &[-0.5, 0.0, -2.5])?,
        max_accel: d.numbers("max_accel", &[2.5, 0.0, 3.2])?,
        max_decel: d.numbers("max_decel", &[-2.5, 0.0, -3.2])?,
        deadband_velocity: d.numbers("deadband_velocity", &[0.0, 0.0, 0.0])?,
    };
    let timeout = d.number("velocity_timeout", 1.0)?;
    let odom_topic = d.text("odom_topic", "odom")?;
    // Taken so Nav2's file loads; Nav2's other names (odom_duration, scale_velocities,
    // use_realtime_priority) are ignored.
    let stamp_with_time = d.flag("stamp_smoothed_velocity_with_smoothing_time", false)?;
    let settings = Settings::from_nav2(frequency, feedback, &limits, timeout)?;

    let yaml = d.text("vehicle_yaml", "")?;
    let yaml = if yaml.is_empty() {
        default_vehicle_yaml()?
    } else {
        PathBuf::from(yaml)
    };
    let model = ActuatorConfig::from_vehicle_yaml(&yaml)
        .with_context(|| format!("reading {}", yaml.display()))?;
    let rate = d.number("steering_rate", 0.0)?;
    let car = Car {
        wheelbase: model.wheelbase,
        max_steering: model.max_steering,
        steering_rate: if rate > 0.0 { rate } else { model.steer_rate },
        min_speed_for_steering: d.number("min_speed_for_steering", 0.05)?,
        max_accel: model.max_accel,
        max_brake: model.max_brake,
    };
    let smoother = Smoother::new(settings, car)?;
    log_info!(
        node.logger(),
        "car_smoother at {frequency} Hz, {:?}: {}..{} m/s, +{} / -{} m/s², turns under {} \
         rad/s; the wheels at most {} rad/s to {} rad (L {} m, {})",
        settings.feedback,
        settings.min_speed,
        settings.max_speed,
        settings.accel.min(car.max_accel),
        settings.decel.min(car.max_brake),
        settings.max_turn_rate,
        car.steering_rate,
        car.max_steering,
        car.wheelbase,
        yaml.display()
    );

    let core = Core {
        smoother,
        publisher: node.create_publisher("cmd_vel_smoothed")?,
        clock: node.get_clock(),
        logger: node.logger().clone(),
        header: Header::default(),
        received: 0.0,
        measured: None,
        stamp_with_time,
    };
    let worker = node.create_worker(core);
    let _cmd = worker.create_subscription("cmd_vel", |s: &mut Core, msg: TwistStamped| {
        s.on_command(msg)
    })?;
    let _odom = if feedback == Feedback::ClosedLoop {
        Some(
            worker.create_subscription(odom_topic.as_str(), |s: &mut Core, msg: Odometry| {
                s.measured = Some(Reading {
                    speed: msg.twist.twist.linear.x,
                    at: s.now(),
                })
            })?,
        )
    } else {
        None
    };
    let period = Duration::from_secs_f64(1.0 / frequency);
    let _tick = worker.create_timer_repeating(period.node_time(), |s: &mut Core| s.tick())?;

    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// aav_description's vehicle.yaml, through AMENT_PREFIX_PATH.
fn default_vehicle_yaml() -> Result<PathBuf> {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    std::env::split_paths(&prefixes)
        .map(|p| p.join("share/aav_description/config/vehicle.yaml"))
        .find(|p| p.is_file())
        .context("no aav_description/config/vehicle.yaml on AMENT_PREFIX_PATH: set vehicle_yaml")
}

struct Core {
    smoother: Smoother,
    publisher: Publisher<TwistStamped>,
    clock: Clock,
    logger: Logger,
    /// The last command's header, for the smoothed command's.
    header: Header,
    /// s: when the last command was stamped (or arrived, if unstamped).
    received: f64,
    /// The latest odometry speed, with when it arrived (closed loop).
    measured: Option<Reading>,
    stamp_with_time: bool,
}

impl Core {
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_command(&mut self, msg: TwistStamped) {
        let stamp = &msg.header.stamp;
        let at = if stamp.sec == 0 && stamp.nanosec == 0 {
            self.now()
        } else {
            seconds(stamp)
        };
        match self
            .smoother
            .command(msg.twist.linear.x, msg.twist.angular.z, at)
        {
            Ok(()) => {
                self.header = msg.header;
                if self.header.stamp.sec == 0 && self.header.stamp.nanosec == 0 {
                    self.header.stamp = time_of(at);
                }
                self.received = at;
            }
            Err(e) => log_error!(&self.logger, "{e}"),
        }
    }

    fn tick(&mut self) {
        let now = self.now();
        let timeout = self.smoother.settings().timeout;
        let measured = self.measured.and_then(|r| r.fresh(now, timeout));
        let Some(out) = self.smoother.tick(now, measured) else {
            return;
        };
        let mut header = self.header.clone();
        if now - self.received > timeout {
            // Stopping for a stale command: stamped now, as Nav2's zero command is.
            header.stamp = time_of(now);
        } else if self.stamp_with_time {
            header.stamp = time_of(now);
        }
        let msg = TwistStamped {
            header,
            twist: Twist {
                linear: Vector3 {
                    x: out.v,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: out.w,
                },
            },
        };
        if let Err(e) = self.publisher.publish(msg) {
            log_error!(&self.logger, "publishing cmd_vel_smoothed: {e}");
        }
    }
}

fn seconds(t: &Time) -> f64 {
    t.sec as f64 + t.nanosec as f64 * 1e-9
}

fn time_of(s: f64) -> Time {
    let sec = s.floor();
    Time {
        sec: sec as i32,
        nanosec: ((s - sec) * 1e9).round().min(999_999_999.0) as u32,
    }
}

struct Declarer<'a> {
    node: &'a Node,
    handles: Vec<Box<dyn std::any::Any>>,
}

impl Declarer<'_> {
    fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        let p = self
            .node
            .declare_parameter(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name} (a float)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        if !value.is_finite() {
            bail!("{name}:={value}: not a number");
        }
        Ok(value)
    }

    fn flag(&mut self, name: &str, default: bool) -> Result<bool> {
        let p = self
            .node
            .declare_parameter(name)
            .default(default)
            .read_only()
            .with_context(|| format!("parameter {name} (true or false)"))?;
        let value = p.get();
        self.handles.push(Box::new(p));
        Ok(value)
    }

    fn text(&mut self, name: &str, default: &str) -> Result<String> {
        let p = self
            .node
            .declare_parameter::<Arc<str>>(name)
            .default(default.into())
            .read_only()
            .with_context(|| format!("parameter {name} (text)"))?;
        let value = p.get().to_string();
        self.handles.push(Box::new(p));
        Ok(value)
    }

    fn numbers(&mut self, name: &str, default: &[f64]) -> Result<Vec<f64>> {
        let p = self
            .node
            .declare_parameter::<Arc<[f64]>>(name)
            .default(default.into())
            .read_only()
            .with_context(|| format!("parameter {name} (a list of floats)"))?;
        let value = p.get().to_vec();
        self.handles.push(Box::new(p));
        Ok(value)
    }
}
