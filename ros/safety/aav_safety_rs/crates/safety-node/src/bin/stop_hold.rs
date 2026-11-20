//! stop_hold, in Rust: once the stop zone stops the car, hold the stop until the way stays
//! clear. The same node as aav_safety's stop_hold.py, topic for topic.
//!
//! Subscribes  /safety/stop_zone/state  nav2_msgs/CollisionMonitorState: the stop zone
//!                                      (aav_stop_zone), each time its action changes
//! Publishes   /safety/stop_hold/lock   std_msgs/Bool, at rate_hz and on each state: true
//!                                      from a STOP until the zone has said something else
//!                                      for clear_time s without a break. twist_mux's
//!                                      stop_zone lock (priority 21)
//! Parameters  clear_time (s, 1.0), rate_hz (10): aav_safety config/stop_hold.yaml. The
//!             rule is safety-core's hold module.
//!
//! Times come from the steady clock: a hold must not end early if sim time jumps.

use std::any::Any;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Result};
use rclrs::*;
use ros_env::nav2_msgs::msg::CollisionMonitorState;
use ros_env::std_msgs::msg::Bool;
use safety_core::hold::StopHold;
use safety_node::{lock, run, Params};

struct Node_ {
    hold: StopHold,
    start: Instant,
    was: bool,
    logger: Logger,
    out: Publisher<Bool>,
}

impl Node_ {
    fn now(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }

    fn tick(&mut self) {
        let now = self.now();
        let locked = self.hold.locked(now);
        if let Err(e) = self.out.publish(Bool { data: locked }) {
            log_warn!(
                self.logger.throttle(Duration::from_secs(2)),
                "/safety/stop_hold/lock: {e}"
            );
        }
        if locked != self.was {
            if locked {
                log_info!(&self.logger, "holding Nav2 after a stop");
            } else {
                log_info!(
                    &self.logger,
                    "the stop zone stayed clear for {} s: Nav2 may go",
                    safety_core::pyfmt::repr(self.hold.clear_time())
                );
            }
            self.was = locked;
        }
    }
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut p = Params::new(node);
    let clear_time = p.number("clear_time", 1.0)?;
    let rate = p.number("rate_hz", 10.0)?;
    if rate.is_nan() || rate <= 0.0 {
        bail!("rate_hz:={rate}: a rate must be above 0 Hz");
    }
    let mut keep = p.into_keep();
    let me = Arc::new(Mutex::new(Node_ {
        hold: StopHold::new(clear_time)?,
        start: Instant::now(),
        was: false,
        logger: node.logger().clone(),
        out: node.create_publisher("/safety/stop_hold/lock")?,
    }));
    let m = Arc::clone(&me);
    keep.push(Box::new(node.create_subscription(
        "/safety/stop_zone/state",
        move |msg: CollisionMonitorState| {
            let mut n = lock(&m);
            let now = n.now();
            n.hold
                .update(now, msg.action_type == CollisionMonitorState::STOP);
            n.tick();
        },
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
    run("stop_hold", start)
}
