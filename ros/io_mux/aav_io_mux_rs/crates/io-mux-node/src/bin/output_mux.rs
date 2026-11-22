//! output_mux in Rust: passes ego's commands to one target, only while every source is
//! that target's. A drop-in for aav_io_mux's output_mux.py: the same node name, topics,
//! QoS, parameters, service and answers. The logic is `io-mux-core`, with no ROS.
//! **A safety lock** (it makes the heartbeat the ESP32 brakes without): the default since
//! Ben, 17 Nov; the Python one was removed on 22 Nov.
//!
//! Subscribes  ego/actuator_cmd          ackermann_msgs/AckermannDriveStamped (ego, 50 Hz)
//!             system/sensor_mux/state   aav_interfaces/SourceCheck (the sensor mux, 2 Hz)
//!             system/source             aav_interfaces/DataSource (reliable, transient
//!                                       local, depth 50)
//!             hud/<screen>/heartbeat    std_msgs/UInt64: only for each of `screens`
//! Publishes   control/<target>/actuator_cmd  ackermann_msgs/AckermannDriveStamped: ego's
//!                                       command, unchanged, while the gate is open
//!             control/<target>/heartbeat  aav_interfaces/Heartbeat: one for each command
//!                                       passed on, counter +1 (a byte), so it moves only
//!                                       while OPEN. The Motor Pi relays it to the ESP32
//!             safety/output_mux/state   std_msgs/String, 2 Hz and on each change: OPEN,
//!                                       WAIT or STOP, the target, and why
//! Serves      safety/output_mux/clear   std_srvs/Trigger: a person reopens a latched stop
//!
//! Parameters  target (car; sim, turtle, carla or bench), settle (2.0 s), stale_after
//!             (3.0 s), screens (comma-separated names, empty: none; refused for car)
//!
//! The gate (`provenance::Gate`) opens once the sensor mux says ok for the target's
//! source, nothing on /system/source says otherwise, and both have held for `settle` s.
//! Evidence latches a stop until a person clears it; doubt only holds it shut. Its clock
//! is a steady one, not simulated time. A ROS graph that can't be read skips that
//! re-judging (the report then ages, and the gate waits). Stops cleanly on SIGINT,
//! SIGTERM or SIGHUP.

use std::any::Any;
use std::collections::HashSet;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Result;
use io_mux_core::muxes::{screen_names, Level, OutputMux, Published};
use io_mux_core::provenance::Kind;
use io_mux_node::{announcers_alive, entries_of, steady_now, Declarer, SOURCE_DEPTH};
use rclrs::*;
use ros_env::aav_interfaces::msg::{DataSource, Heartbeat, SourceCheck};
use ros_env::ackermann_msgs::msg::AckermannDriveStamped;
use ros_env::std_msgs::msg::{String as Text, UInt64};
use ros_env::std_srvs::srv::{Trigger, Trigger_Request, Trigger_Response};

struct State {
    mux: OutputMux,
    node: Node,
    out: Publisher<AckermannDriveStamped>,
    heartbeat: Publisher<Heartbeat>,
    state: Publisher<Text>,
    logger: Logger,
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("output_mux")?;
    let _running = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn start(node: &Node) -> Result<Vec<Box<dyn Any>>> {
    let mut d = Declarer::new(node);
    let target = d.text("target", "car")?;
    let settle = d.number("settle", 2.0)?;
    let stale_after = d.number("stale_after", 3.0)?;
    let screens = screen_names(&d.text("screens", "")?);
    let mux = OutputMux::new(&target, settle, stale_after, screens)?;
    let gate = mux.gate();
    let (topic, beat_topic, expected) = (
        gate.target.command_topic(),
        gate.target.heartbeat_topic(),
        gate.expected,
    );
    let screens = gate.screens.clone();
    let logger = node.logger().clone();
    let worker = node.create_worker(State {
        mux,
        node: node.clone(),
        out: node.create_publisher(topic.as_str().keep_last(10))?,
        heartbeat: node.create_publisher(beat_topic.as_str().keep_last(10))?,
        state: node.create_publisher("safety/output_mux/state".keep_last(10))?,
        logger: logger.clone(),
    });
    let mut keep: Vec<Box<dyn Any>> = vec![
        Box::new(worker.create_subscription(
            "ego/actuator_cmd".keep_last(10),
            |s: &mut State, msg: AckermannDriveStamped| s.on_cmd(msg),
        )?),
        Box::new(worker.create_subscription(
            "system/sensor_mux/state".keep_last(10),
            |s: &mut State, msg: SourceCheck| s.on_check(msg),
        )?),
        Box::new(
            worker.create_subscription(
                "system/source"
                    .keep_last(SOURCE_DEPTH)
                    .reliable()
                    .transient_local(),
                |s: &mut State, msg: DataSource| s.on_source(&msg),
            )?,
        ),
        Box::new(worker.create_service::<Trigger, _>(
            "safety/output_mux/clear",
            |s: &mut State, _: Trigger_Request| s.on_clear(),
        )?),
        // Little work beside the commands, so a command never waits long behind it: the
        // graph (who still announces) twice a second, and the state with it.
        Box::new(worker.create_timer_repeating(
            Duration::from_secs_f64(0.5).steady_time(),
            |s: &mut State| s.on_timer(),
        )?),
    ];
    for screen in screens {
        let topic = format!("hud/{screen}/heartbeat");
        keep.push(Box::new(worker.create_subscription(
            topic.as_str().keep_last(10),
            move |s: &mut State, _: UInt64| s.mux.beat(&screen, steady_now()),
        )?));
    }
    keep.push(Box::new(worker));
    keep.extend(d.finish());
    log_info!(
        &logger,
        "target:={target}: ego/actuator_cmd -> {topic}, while every source is {expected}"
    );
    Ok(keep)
}

impl State {
    fn alive(&self) -> Option<HashSet<String>> {
        match announcers_alive(&self.node) {
            Ok(alive) => Some(alive),
            Err(e) => {
                log_error!(&self.logger, "{e:#}: not judged this time");
                None
            }
        }
    }

    fn on_cmd(&mut self, msg: AckermannDriveStamped) {
        let Some(counter) = self.mux.command(steady_now()) else {
            return;
        };
        let header = msg.header.clone();
        if let Err(e) = self.out.publish(msg) {
            log_error!(&self.logger, "the command: {e}");
        }
        if let Err(e) = self.heartbeat.publish(Heartbeat { header, counter }) {
            log_error!(&self.logger, "the heartbeat: {e}");
        }
    }

    fn on_check(&mut self, msg: SourceCheck) {
        let Some(alive) = self.alive() else { return };
        let said = self.mux.check(
            steady_now(),
            &alive,
            Kind::from_byte(msg.source),
            msg.ok,
            msg.problems,
            msg.doubts,
        );
        self.say(said);
    }

    fn on_source(&mut self, msg: &DataSource) {
        let Some(alive) = self.alive() else { return };
        let said = self
            .mux
            .source(steady_now(), &alive, &msg.announcer, entries_of(msg));
        self.say(said);
    }

    fn on_timer(&mut self) {
        let Some(alive) = self.alive() else { return };
        let said = self.mux.timer(steady_now(), &alive);
        self.say(Some(said));
    }

    fn on_clear(&mut self) -> Trigger_Response {
        // Without the graph, every announcer would look gone: refuse instead.
        let Some(alive) = self.alive() else {
            return Trigger_Response {
                success: false,
                message: "the ROS graph can't be read now: nothing cleared, try again".into(),
            };
        };
        let (cleared, said) = self.mux.clear(steady_now(), &alive);
        let response = match cleared {
            Ok(message) => {
                log_warn!(&self.logger, "clear: {message}");
                Trigger_Response {
                    success: true,
                    message,
                }
            }
            Err(message) => {
                log_error!(&self.logger, "clear: {message}");
                Trigger_Response {
                    success: false,
                    message,
                }
            }
        };
        self.say(said);
        response
    }

    /// Publish the state line, and log a change.
    fn say(&mut self, said: Option<Published>) {
        let Some(Published { line, log }) = said else {
            return;
        };
        if let Some(log) = log {
            match log.level {
                Level::Error => log_error!(&self.logger, "{}", log.text),
                Level::Warning => log_warn!(&self.logger, "{}", log.text),
                Level::Info => log_info!(&self.logger, "{}", log.text),
            }
        }
        if let Err(e) = self.state.publish(Text { data: line }) {
            log_error!(&self.logger, "safety/output_mux/state: {e}");
        }
    }
}
