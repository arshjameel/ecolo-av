//! rc_driver, in Rust: the RC transmitter's sticks as a driver, for the RC via ROS mode
//! (TODO 48). The same node as aav_rc's rc_driver.py, topic for topic.
//!
//! Subscribes  vehicle/rc           aav_interfaces/RcSticks: the mode switch and the sticks, as
//!                                  the ESP32 reads them (legacy_uart_bridge, from its uplink)
//! Publishes   drivers/rc/cmd_vel   geometry_msgs/TwistStamped, rate_hz (50), only while the mode
//!                                  switch is on RC via ROS (twist_mux priority 95): the throttle
//!                                  stick x max_speed (reverse: max_reverse_speed), the steering
//!                                  stick x max_steering_angle as a turn rate (the bicycle model).
//!                                  One zero as the switch leaves RC via ROS; zeros while
//!                                  vehicle/rc is quiet on RC via ROS (a stop)
//! Parameters  max_speed, max_reverse_speed (m/s), max_steering_angle (rad), wheelbase (m),
//!             stale (s), rate_hz, frame_id
//!
//! The rules are the rc-core crate (no ROS). In RC direct the ESP32 drives from the sticks
//! itself, and this publishes nothing.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use rc_core::{Limits, RcDrive, Sticks, Switch};
use rclrs::*;
use ros_env::aav_interfaces::msg::RcSticks;
use ros_env::geometry_msgs::msg::TwistStamped;
use ros_env::std_msgs::msg::Header;

/// How often the "quiet" warning may be logged.
const WARN_EVERY: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("rc_driver")?;
    let _keep = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// A read-only number (float or whole), and its handle to keep.
fn number(node: &Node, name: &str, default: f64) -> Result<(f64, Box<dyn std::any::Any>)> {
    let p = node
        .declare_parameter::<ParameterValue>(name)
        .default(ParameterValue::Double(default))
        .read_only()
        .with_context(|| format!("parameter {name} (a number)"))?;
    let value = match p.get() {
        ParameterValue::Double(v) => v,
        ParameterValue::Integer(v) => v as f64,
        other => bail!("{name}:={other:?}: not a number"),
    };
    Ok((value, Box::new(p)))
}

struct Driver {
    drive: RcDrive,
    switch: Option<Switch>,
    frame_id: String,
    clock: Clock,
    logger: Logger,
    publisher: Publisher<TwistStamped>,
}

impl Driver {
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_rc(&mut self, msg: &RcSticks) {
        let switch = Switch::from_byte(msg.mode_switch);
        let now = self.now();
        self.drive.heard(
            Sticks {
                switch,
                throttle: f64::from(msg.throttle),
                steering: f64::from(msg.steering),
                brake: msg.brake,
            },
            now,
        );
        if self.switch != Some(switch) {
            log_info!(&self.logger, "mode switch: {}", switch.words());
            self.switch = Some(switch);
        }
    }

    fn tick(&mut self) {
        let was_driving = self.drive.driving();
        let now = self.now();
        let Some(cmd) = self.drive.command(now) else {
            return;
        };
        if was_driving
            && self
                .drive
                .heard_at()
                .is_some_and(|at| now - at > self.drive.limits.stale)
        {
            log_warn!(
                self.logger.throttle(WARN_EVERY),
                "vehicle/rc quiet on RC via ROS: holding a stop"
            );
        }
        let mut msg = TwistStamped {
            header: Header {
                stamp: self.clock.now().to_ros_msg().unwrap_or_default(),
                frame_id: self.frame_id.clone(),
            },
            ..TwistStamped::default()
        };
        msg.twist.linear.x = cmd.v;
        msg.twist.angular.z = cmd.w;
        if let Err(e) = self.publisher.publish(msg) {
            log_warn!(self.logger.throttle(WARN_EVERY), "drivers/rc/cmd_vel: {e}");
        }
    }
}

fn lock(d: &Mutex<Driver>) -> MutexGuard<'_, Driver> {
    d.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

fn start(node: &Node) -> Result<Vec<Box<dyn std::any::Any>>> {
    let mut keep: Vec<Box<dyn std::any::Any>> = Vec::new();
    let mut get = |name: &str, default: f64| -> Result<f64> {
        let (value, handle) = number(node, name, default)?;
        keep.push(handle);
        Ok(value)
    };
    let d = Limits::default();
    let limits = Limits {
        max_speed: get("max_speed", d.max_speed)?,
        max_reverse_speed: get("max_reverse_speed", d.max_reverse_speed)?,
        max_steering_angle: get("max_steering_angle", d.max_steering_angle)?,
        wheelbase: get("wheelbase", d.wheelbase)?,
        stale: get("stale", d.stale)?,
    };
    let rate = get("rate_hz", 50.0)?;
    if rate.is_nan() || rate <= 0.0 {
        bail!("rate_hz:={rate}: a rate must be above 0 Hz");
    }
    let frame_id = node
        .declare_parameter::<Arc<str>>("frame_id")
        .default("base_link".into())
        .read_only()?;
    let driver = Arc::new(Mutex::new(Driver {
        drive: RcDrive::new(limits),
        switch: None,
        frame_id: frame_id.get().to_string(),
        clock: node.get_clock(),
        logger: node.logger().clone(),
        publisher: node.create_publisher("drivers/rc/cmd_vel")?,
    }));
    keep.push(Box::new(frame_id));
    let d = Arc::clone(&driver);
    keep.push(Box::new(
        node.create_subscription("vehicle/rc", move |msg: RcSticks| lock(&d).on_rc(&msg))?,
    ));
    let d = Arc::clone(&driver);
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        move || lock(&d).tick(),
    )?));
    keep.push(Box::new(driver));
    Ok(keep)
}
