//! goal_leash, in Rust: cancel a laptop driver's goals when the laptop goes quiet
//! (TODO 116). The same node as aav_safety's goal_leash_node.py, topic for topic.
//!
//! Runs on the car, next to the mux. The language driver runs on a laptop and sends goals
//! to drivers on the car; a ROS action carries on when its client vanishes. While it
//! runs, it beats; when the beats stop for `timeout` s, this cancels every goal on
//! `actions` once, so the car slows to a stop the driver's own way (not an e-stop). A
//! goodbye beat (0) disarms it. Nothing happens until a first beat.
//!
//! Subscribes  heartbeat_topic   std_msgs/UInt64 (/drivers/language/heartbeat), reliable,
//!                               depth 10: 1, 2, 3... at 5 Hz; 0 = goodbye
//! Calls       <action>/_action/cancel_goal   action_msgs/srv/CancelGoal, for each of
//!                               `actions`: an empty goal id and a zero stamp, ROS's
//!                               "cancel every goal"
//! Parameters  heartbeat_topic, actions ([/scripted_drive, /follow_gps_waypoints]),
//!             timeout (s, 1.0), rate_hz (10). The rule: safety-core's leash module.
//!
//! Times come from the steady clock: a laptop's stamps and this machine's clock may differ.

use std::any::Any;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Result};
use rclrs::*;
use ros_env::action_msgs::srv::{CancelGoal, CancelGoal_Request, CancelGoal_Response};
use ros_env::std_msgs::msg::UInt64;
use safety_core::leash::Leash;
use safety_core::pyfmt::g;
use safety_node::{lock, run, Params};

struct GoalLeash {
    leash: Leash,
    start: Instant,
    actions: Vec<String>,
    cancels: Vec<(String, Client<CancelGoal>)>,
    logger: Logger,
}

impl GoalLeash {
    fn now(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }

    fn on_beat(&mut self, msg: UInt64) {
        let was = self.leash.armed();
        let now = self.now();
        self.leash.beat(msg.data, now);
        if self.leash.armed() && !was {
            log_info!(
                &self.logger,
                "a laptop driver is beating: its goals are on the leash"
            );
        } else if was && !self.leash.armed() {
            log_info!(&self.logger, "the laptop driver said goodbye");
        }
    }

    fn tick(&mut self) {
        let now = self.now();
        if !self.leash.check(now) {
            return;
        }
        log_error!(
            &self.logger,
            "the laptop driver went quiet for {} s: cancelling the goals on {}; the car slows \
             to a stop",
            g(self.leash.timeout()),
            self.actions.join(", ")
        );
        for (name, client) in &self.cancels {
            if !client.service_is_ready().unwrap_or(false) {
                log_warn!(&self.logger, "{name}: no action server to cancel");
                continue;
            }
            let sent: Result<Promise<CancelGoal_Response>, _> =
                client.call(&CancelGoal_Request::default());
            if let Err(e) = sent {
                log_error!(&self.logger, "{name}: the cancel was not sent: {e}");
            }
        }
    }
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut p = Params::new(node);
    let topic = p.text("heartbeat_topic", "/drivers/language/heartbeat")?;
    let actions = p.texts("actions", &["/scripted_drive", "/follow_gps_waypoints"])?;
    let timeout = p.number("timeout", 1.0)?;
    let rate = p.number("rate_hz", 10.0)?;
    if rate.is_nan() || rate <= 0.0 {
        bail!("rate_hz:={rate}: a rate must be above 0 Hz");
    }
    let mut keep = p.into_keep();
    let cancels = actions
        .iter()
        .map(|a| {
            let client =
                node.create_client::<CancelGoal>(format!("{a}/_action/cancel_goal").as_str())?;
            Ok((a.clone(), client))
        })
        .collect::<Result<Vec<_>>>()?;
    let me = Arc::new(Mutex::new(GoalLeash {
        leash: Leash::new(timeout),
        start: Instant::now(),
        actions: actions.clone(),
        cancels,
        logger: node.logger().clone(),
    }));
    let m = Arc::clone(&me);
    keep.push(Box::new(node.create_subscription(
        topic.as_str().keep_last(10).reliable(),
        move |msg: UInt64| lock(&m).on_beat(msg),
    )?));
    let m = Arc::clone(&me);
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        move || lock(&m).tick(),
    )?));
    log_info!(
        node.logger(),
        "watching {topic}: {} s quiet cancels the goals on {}",
        g(timeout),
        actions.join(", ")
    );
    keep.push(Box::new(me));
    Ok(keep)
}

fn main() -> Result<()> {
    run("goal_leash", start)
}
