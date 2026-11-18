//! joy_alive, in Rust: pass the joystick on to teleop_twist_joy only while it is really
//! there and moving. The same node as aav_safety's joy_alive_node.py, topic for topic.
//!
//! Runs in the namespace /drivers/joy (aav_teleop teleop.launch.py), between joy_node and
//! teleop_twist_joy_node.
//!
//! Subscribes  joy           sensor_msgs/Joy (/drivers/joy/joy): joy_node, 20 Hz while the
//!                           receiver is in, the last state again and again
//! Publishes   joy_checked   sensor_msgs/Joy (/drivers/joy/joy_checked): each message as it
//!                           came, or, while stopped, a released copy (every axis and button
//!                           0), so teleop_twist_joy sends one zero and then nothing; once
//!                           more the moment silence trips it
//!             alive         std_msgs/String (/drivers/joy/alive), at rate_hz: one line for
//!                           people, OK, WAITING, SILENT or FROZEN, a dash, and why
//! Parameters  enable_button (0), motion_axes ([1]), motion_threshold (0.05),
//!             silent_after (s, 0.5), frozen_after (s, 2.0; 0 off), rate_hz (10):
//!             aav_teleop config/joy_alive.yaml. The rules are the joy-alive-core crate.
//!
//! Times come from the steady clock: joy_node stamps each repeat afresh, so stamps prove
//! nothing.

use std::any::Any;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use anyhow::{bail, Context as _, Result};
use joy_alive_core::{JoyAlive, Settings, OK};
use rclrs::*;
use ros_env::sensor_msgs::msg::Joy;
use ros_env::std_msgs::msg::{Header, String as Text};

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("joy_alive")?;
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

    /// A number, float or whole.
    fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        match self.value(name, ParameterValue::Double(default))? {
            ParameterValue::Double(v) => Ok(v),
            ParameterValue::Integer(v) => Ok(v as f64),
            other => bail!("{name}:={other:?}: not a number"),
        }
    }

    /// A button or axis number (0 or more).
    fn index(&mut self, name: &str, default: i64) -> Result<usize> {
        match self.value(name, ParameterValue::Integer(default))? {
            ParameterValue::Integer(v) => usize::try_from(v)
                .map_err(|_| anyhow::anyhow!("{name} must be a button number (>= 0)")),
            other => bail!("{name}:={other:?}: not a whole number"),
        }
    }

    /// A list of axis numbers (each 0 or more).
    fn indices(&mut self, name: &str, default: &[i64]) -> Result<Vec<usize>> {
        match self.value(name, ParameterValue::IntegerArray(default.into()))? {
            ParameterValue::IntegerArray(v) => v
                .iter()
                .map(|&i| {
                    usize::try_from(i)
                        .map_err(|_| anyhow::anyhow!("{name}: {i} is not an axis number"))
                })
                .collect(),
            ParameterValue::ByteArray(b) if b.is_empty() => Ok(Vec::new()),
            other => bail!("{name}:={other:?}: not a list of axis numbers"),
        }
    }
}

struct Alive {
    check: JoyAlive,
    start: Instant,
    was: String,
    clock: Clock,
    logger: Logger,
    out: Publisher<Joy>,
    state: Publisher<Text>,
}

impl Alive {
    /// s on the steady clock.
    fn now(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }

    fn released(&self) -> Joy {
        let (axes, buttons) = self.check.sizes();
        Joy {
            header: Header {
                stamp: self.clock.now().to_ros_msg().unwrap_or_default(),
                frame_id: "joy".to_string(),
            },
            axes: vec![0.0; axes],
            buttons: vec![0; buttons],
        }
    }

    fn on_joy(&mut self, msg: Joy) {
        let now = self.now();
        let ok = self.check.on_joy(now, &msg.axes, &msg.buttons);
        let out = if ok { msg } else { self.released() };
        if let Err(e) = self.out.publish(out) {
            log_warn!(
                self.logger.throttle(Duration::from_secs(2)),
                "joy_checked: {e}"
            );
        }
        self.report(false);
    }

    fn tick(&mut self) {
        let now = self.now();
        if self.check.tick(now) && self.check.sizes().1 > 0 {
            if let Err(e) = self.out.publish(self.released()) {
                log_warn!(
                    self.logger.throttle(Duration::from_secs(2)),
                    "joy_checked: {e}"
                );
            }
        }
        self.report(true);
    }

    fn report(&mut self, publish: bool) {
        let line = self.check.state(self.now());
        let name = line.split(' ').next().unwrap_or_default().to_string();
        if name != self.was {
            if name == OK {
                log_info!(&self.logger, "the joystick: {line}");
            } else {
                log_error!(&self.logger, "the joystick: {line}");
            }
            self.was = name;
        }
        if publish {
            if let Err(e) = self.state.publish(Text { data: line }) {
                log_warn!(self.logger.throttle(Duration::from_secs(2)), "alive: {e}");
            }
        }
    }
}

fn lock(a: &Mutex<Alive>) -> MutexGuard<'_, Alive> {
    a.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut p = Params {
        node,
        keep: Vec::new(),
    };
    let d = Settings::default();
    let settings = Settings {
        enable_button: p.index("enable_button", 0)?,
        motion_axes: p.indices("motion_axes", &[1])?,
        motion_threshold: p.number("motion_threshold", d.motion_threshold)?,
        silent_after: p.number("silent_after", d.silent_after)?,
        frozen_after: p.number("frozen_after", d.frozen_after)?,
    };
    let rate = p.number("rate_hz", 10.0)?;
    if rate.is_nan() || rate <= 0.0 {
        bail!("rate_hz:={rate}: a rate must be above 0 Hz");
    }
    let mut keep = p.keep;
    let alive = Arc::new(Mutex::new(Alive {
        check: JoyAlive::new(settings)?,
        start: Instant::now(),
        was: String::new(),
        clock: node.get_clock(),
        logger: node.logger().clone(),
        out: node.create_publisher("joy_checked")?,
        state: node.create_publisher("alive")?,
    }));
    let a = Arc::clone(&alive);
    keep.push(Box::new(
        node.create_subscription("joy", move |msg: Joy| lock(&a).on_joy(msg))?,
    ));
    let a = Arc::clone(&alive);
    // On the node's clock, as the Python's timer; the check itself reads the steady clock.
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        move || lock(&a).tick(),
    )?));
    keep.push(Box::new(alive));
    Ok(keep)
}
