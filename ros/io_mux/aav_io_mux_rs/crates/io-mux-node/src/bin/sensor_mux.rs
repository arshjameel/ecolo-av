//! sensor_mux in Rust: checks that every sensor comes from the one source this run asked
//! for. A drop-in for aav_io_mux's sensor_mux.py: the same node name, topics, QoS,
//! parameters and answers. The logic is `io-mux-core`, with no ROS.
//! **A safety lock**: the default since Ben, 17 Nov; `io_mux_impl:=python` falls back.
//!
//! Subscribes  system/source            aav_interfaces/DataSource (reliable, transient
//!                                      local, depth 50): who says what is REAL, SIM,
//!                                      BENCH or REPLAY
//! Publishes   system/sensor_mux/state  aav_interfaces/SourceCheck, every `period`: the
//!                                      verdict, read by the output mux
//!
//! Parameters  source (real; sim, bench or replay), period (0.5 s), startup (2.0 s:
//!             silent while discovery settles), hold (5.0 s: how long a graph doubt
//!             lasts before it counts), grace (30.0 s from start: a missing announcement
//!             counts only after it)
//!
//! It moves no data: each tick it reads the ROS graph (the publishers of each topic in
//! `provenance::SENSORS`, and of /system/source) and judges it against the
//! announcements (`muxes::SensorMux`). Its clock is a steady one, not simulated time.
//! A graph that can't be read skips the tick: no verdict, so the output mux waits.
//! Stops cleanly on SIGINT, SIGTERM or SIGHUP.

use std::any::Any;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Result;
use io_mux_core::muxes::{Level, SensorMux};
use io_mux_core::provenance::SENSORS;
use io_mux_node::{
    announcers_alive, entries_of, publishers_of, steady_now, Declarer, SOURCE_DEPTH,
};
use rclrs::*;
use ros_env::aav_interfaces::msg::{DataSource, SensorSource, SourceCheck};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::std_msgs::msg::Header;

struct State {
    mux: SensorMux,
    node: Node,
    publisher: Publisher<SourceCheck>,
    logger: Logger,
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("sensor_mux")?;
    let _running = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut d = Declarer::new(node);
    let source = d.text("source", "real")?;
    let period = d.number("period", 0.5)?;
    if period <= 0.0 {
        anyhow::bail!("period:={period}: must be above 0");
    }
    let startup = d.number("startup", 2.0)?;
    let grace = d.number("grace", 30.0)?;
    let hold = d.number("hold", 5.0)?;
    let mux = SensorMux::new(&source, steady_now(), startup, hold, grace)?;
    let logger = node.logger().clone();
    log_info!(
        &logger,
        "source:={source}: every sensor must be {}",
        mux.expected()
    );
    let worker = node.create_worker(State {
        mux,
        node: node.clone(),
        publisher: node.create_publisher("system/sensor_mux/state".keep_last(10))?,
        logger,
    });
    let mut keep: Vec<Box<dyn Any>> = vec![
        Box::new(
            worker.create_subscription(
                "system/source"
                    .keep_last(SOURCE_DEPTH)
                    .reliable()
                    .transient_local(),
                |s: &mut State, msg: DataSource| {
                    s.mux.hear(&msg.announcer, entries_of(&msg), steady_now());
                },
            )?,
        ),
        // A steady clock: the check runs on simulated time too, before /clock starts.
        Box::new(worker.create_timer_repeating(
            Duration::from_secs_f64(period).steady_time(),
            |s: &mut State| s.tick(),
        )?),
    ];
    keep.push(Box::new(worker));
    keep.extend(d.finish());
    Ok(keep)
}

impl State {
    fn tick(&mut self) {
        let now = steady_now();
        if now < self.mux.ready_at() {
            return;
        }
        let graph = || -> Result<_> {
            let alive = announcers_alive(&self.node)?;
            let publishers = SENSORS
                .iter()
                .map(|(_, topic)| Ok((topic.to_string(), publishers_of(&self.node, topic)?)))
                .collect::<Result<HashMap<_, _>>>()?;
            Ok((alive, publishers))
        };
        let (alive, publishers) = match graph() {
            Ok(g) => g,
            Err(e) => {
                log_error!(&self.logger, "{e:#}: no verdict this tick");
                return;
            }
        };
        let Some((check, log)) = self.mux.tick(now, &alive, &publishers) else {
            return;
        };
        let ns = self.node.get_clock().now().nsec;
        let msg = SourceCheck {
            header: Header {
                stamp: Time {
                    sec: ns.div_euclid(1_000_000_000) as i32,
                    nanosec: ns.rem_euclid(1_000_000_000) as u32,
                },
                frame_id: String::new(),
            },
            source: check.source.byte(),
            ok: check.ok,
            problems: check.problems,
            doubts: check.doubts,
            sensors: check
                .sensors
                .into_iter()
                .map(|s| SensorSource {
                    name: s.name,
                    topic: s.topic,
                    source: s.kind.byte(),
                })
                .collect(),
        };
        if let Err(e) = self.publisher.publish(msg) {
            log_error!(&self.logger, "system/sensor_mux/state: {e}");
        }
        if let Some(log) = log {
            match log.level {
                Level::Error => log_error!(&self.logger, "{}", log.text),
                Level::Warning => log_warn!(&self.logger, "{}", log.text),
                Level::Info => log_info!(&self.logger, "{}", log.text),
            }
        }
    }
}
