//! command_mux_state: says which driver twist_mux passes on, and why the others don't.
//!
//! twist_mux never says whom it picked. This node reads the same parameter files
//! (aav_command_mux `mux.launch.py` hands it the same list, in the same order), hears the
//! same inputs and locks, and applies twist_mux's rule (the mux-state-core crate).
//!
//! Subscribes  every input in the files    geometry_msgs/TwistStamped (Twist with
//!                                         use_stamped false): /drivers/<driver>/cmd_vel
//!             every lock in the files     std_msgs/Bool: /safety/estop, the watchdog's...
//! Publishes   /control/command_mux/state  diagnostic_msgs/DiagnosticStatus, at rate_hz
//!                                         and at once when the driver, the locks or the
//!                                         held inputs change: name command_mux, level
//!                                         OK or WARN (nobody drives while a lock holds
//!                                         someone), message one line ("keyboard (90)"),
//!                                         values driver, priority, topic, age_s, locks,
//!                                         lock_priority, held (mux-state-core report.rs)
//! Parameters  config_files (string list: twist_mux's parameter files, later ones win),
//!             rate_hz (10), state_topic (/control/command_mux/state)
//!
//! Times are on the node's clock, as twist_mux's (the simulator's with use_sim_time). The
//! QoS is twist_mux's: reliable, volatile, depth 10.

use std::any::Any;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use mux_state_core::{Mux, MuxConfig};
use rclrs::*;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{Twist, TwistStamped};
use ros_env::std_msgs::msg::Bool;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("command_mux_state")?;
    let _keep = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

struct State {
    mux: Mux,
    clock: Clock,
    last_key: String,
    out: Publisher<DiagnosticStatus>,
    logger: Logger,
}

impl State {
    /// s on the node's clock.
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_input(&mut self, i: usize) {
        let now = self.now();
        self.mux.input(i, now);
        self.report(false);
    }

    fn on_lock(&mut self, i: usize, value: bool) {
        let now = self.now();
        self.mux.lock(i, value, now);
        self.report(false);
    }

    /// Sends the state: always on the timer, otherwise only when it changed.
    fn report(&mut self, always: bool) {
        let choice = self.mux.choice(self.now());
        let key = choice.key();
        let changed = key != self.last_key;
        if changed {
            let locks = choice
                .values()
                .into_iter()
                .find(|(k, _)| *k == "locks")
                .map(|(_, v)| v)
                .unwrap_or_default();
            log_info!(
                &self.logger,
                "driving: {}; locks: {locks}",
                choice.summary()
            );
            self.last_key = key;
        }
        if !(always || changed) {
            return;
        }
        let msg = DiagnosticStatus {
            level: choice.level().byte(),
            name: "command_mux".into(),
            message: choice.summary(),
            hardware_id: String::new(),
            values: choice
                .values()
                .into_iter()
                .map(|(key, value)| KeyValue {
                    key: key.into(),
                    value,
                })
                .collect(),
        };
        if let Err(e) = self.out.publish(msg) {
            log_warn!(self.logger.throttle(Duration::from_secs(2)), "state: {e}");
        }
    }
}

fn lock(s: &Mutex<State>) -> MutexGuard<'_, State> {
    s.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut keep: Vec<Box<dyn Any>> = Vec::new();
    let files = node
        .declare_parameter::<ParameterValue>("config_files")
        .default(ParameterValue::StringArray(Arc::from(Vec::new())))
        .read_only()
        .context("parameter config_files (a list of files)")?;
    let paths: Vec<String> = match files.get() {
        ParameterValue::StringArray(v) => v.iter().map(|s| s.to_string()).collect(),
        other => bail!("config_files:={other:?}: not a list of files"),
    };
    keep.push(Box::new(files));
    if paths.is_empty() {
        bail!("config_files is empty: give twist_mux's parameter files, in its order");
    }
    let texts = paths
        .iter()
        .map(|p| std::fs::read_to_string(p).with_context(|| format!("reading {p}")))
        .collect::<Result<Vec<_>>>()?;
    let refs: Vec<&str> = texts.iter().map(String::as_str).collect();
    let config = MuxConfig::from_yaml(&refs).with_context(|| format!("in {paths:?}"))?;

    let rate = node
        .declare_parameter("rate_hz")
        .default(10.0)
        .read_only()?;
    let hz: f64 = rate.get();
    keep.push(Box::new(rate));
    if !(hz.is_finite() && hz > 0.0) {
        bail!("rate_hz:={hz}: a rate must be above 0 Hz");
    }
    let topic = node
        .declare_parameter::<Arc<str>>("state_topic")
        .default("/control/command_mux/state".into())
        .read_only()?;
    let out = node.create_publisher::<DiagnosticStatus>(&*topic.get())?;
    keep.push(Box::new(topic));

    let logger = node.logger().clone();
    log_info!(
        &logger,
        "watching {} inputs and {} locks, as twist_mux: {}",
        config.inputs.len(),
        config.locks.len(),
        paths.join(", ")
    );
    let stamped = config.use_stamped;
    let inputs: Vec<String> = config.inputs.iter().map(|s| s.topic.clone()).collect();
    let locks: Vec<String> = config.locks.iter().map(|s| s.topic.clone()).collect();
    let state = Arc::new(Mutex::new(State {
        mux: Mux::new(config),
        clock: node.get_clock(),
        last_key: String::new(),
        out,
        logger,
    }));
    for (i, t) in inputs.iter().enumerate() {
        let s = Arc::clone(&state);
        if stamped {
            keep.push(Box::new(
                node.create_subscription(t.as_str(), move |_: TwistStamped| lock(&s).on_input(i))?,
            ));
        } else {
            keep.push(Box::new(
                node.create_subscription(t.as_str(), move |_: Twist| lock(&s).on_input(i))?,
            ));
        }
    }
    for (i, t) in locks.iter().enumerate() {
        let s = Arc::clone(&state);
        keep.push(Box::new(
            node.create_subscription(t.as_str(), move |m: Bool| lock(&s).on_lock(i, m.data))?,
        ));
    }
    let s = Arc::clone(&state);
    // Wall time for the timer: it must tick even before the simulator's clock starts.
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_secs_f64(1.0 / hz),
        move || lock(&s).report(true),
    )?));
    keep.push(Box::new(state));
    Ok(keep)
}
