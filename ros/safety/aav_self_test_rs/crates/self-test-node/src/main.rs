//! boot_self_test: is everything that keeps the car safe here and answering? READY or NOT
//! READY, and why (TODO 255). spine_bridge refuses /spine/arm until it says READY
//! (`require_ready`). It only reads: it moves nothing, and a person can always e-stop.
//!
//! Subscribes  safety/sensor_watchdog/state  std_msgs/String: every sensor at its rate
//!             safety/geofence/state, safety/output_mux/state  std_msgs/String: alive
//!             vehicle/status    aav_interfaces/VehicleStatus (actuators: status)
//!             control/car/link_frame  aav_interfaces/LinkFrame (actuators: link)
//!             sensors/imu/data, sensors/gps/fix, vehicle/odom  (each in `clocks`): stamps
//! Reads       the ROS graph: a publisher on safety/stop_zone/state (locks: stop_zone)
//! Publishes   safety/self_test/status  diagnostic_msgs/DiagnosticStatus, `rate_hz` (2):
//!             name boot_self_test; level OK (READY) or ERROR (NOT READY); message the
//!             line; one value per check: OK, or why not
//! Parameters  actuators (status, link or none), locks (watchdog, geofence, output_gate,
//!             stop_zone), clocks (imu, gps, odom), settle (2 s), line_timeout (1 s),
//!             status_timeout (0.5 s), max_clock_offset (0.5 s), rate_hz (2)
//!
//! The logic is self-test-core, with no ROS.

use std::sync::Arc;
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use rclrs::*;
use ros_env::aav_interfaces::msg::{LinkFrame, VehicleStatus};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::{Imu, NavSatFix};
use ros_env::std_msgs::msg::String as Text;
use self_test_core::{ClockSource, Config, Lock, SelfTest};

const STOP_ZONE_STATE: &str = "/safety/stop_zone/state";

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("boot_self_test")?;
    let _keep = build(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

struct Test {
    node: Node,
    clock: Clock,
    logger: Logger,
    test: SelfTest,
    status: Publisher<DiagnosticStatus>,
    last_line: String,
}

impl Test {
    /// s, this node's clock: the same epoch as the stamps it compares.
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn stamp(&mut self, source: ClockSource, stamp: &Time) {
        let now = self.now();
        let at = f64::from(stamp.sec) + f64::from(stamp.nanosec) * 1e-9;
        self.test.stamp(source, at, now);
    }

    fn tick(&mut self) {
        if self.test.config().locks.contains(&Lock::StopZone) {
            match self.node.count_publishers(STOP_ZONE_STATE) {
                Ok(n) => self.test.stop_zone_publishers(n),
                Err(e) => log_warn!(
                    self.logger.throttle(Duration::from_secs(10)),
                    "the ROS graph: {e}"
                ),
            }
        }
        let now = self.now();
        let verdict = self.test.verdict(now);
        let line = verdict.line();
        if line != self.last_line {
            if verdict.ready {
                log_info!(&self.logger, "{line}");
            } else {
                log_warn!(&self.logger, "{line}");
            }
            self.last_line = line.clone();
        }
        let msg = DiagnosticStatus {
            level: if verdict.ready {
                DiagnosticStatus::OK
            } else {
                DiagnosticStatus::ERROR
            },
            name: "boot_self_test".into(),
            message: line,
            hardware_id: String::new(),
            values: verdict
                .outcomes
                .into_iter()
                .map(|o| KeyValue {
                    key: o.name,
                    value: o.why.unwrap_or_else(|| "OK".into()),
                })
                .collect(),
        };
        if let Err(e) = self.status.publish(msg) {
            log_error!(&self.logger, "status: {e}");
        }
    }
}

/// A read-only parameter, kept alive by the caller.
struct Params<'a> {
    node: &'a Node,
    handles: Vec<Box<dyn std::any::Any>>,
}

impl Params<'_> {
    fn number(&mut self, name: &str, default: f64) -> Result<f64> {
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(ParameterValue::Double(default))
            .read_only()
            .with_context(|| format!("parameter {name} (a number)"))?;
        let value = match p.get() {
            ParameterValue::Double(v) => v,
            ParameterValue::Integer(v) => v as f64,
            other => bail!("{name}:={other:?}: not a number"),
        };
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

    fn names(&mut self, name: &str, default: &[&str]) -> Result<Vec<String>> {
        let default: Arc<[Arc<str>]> = default.iter().map(|s| Arc::from(*s)).collect();
        let p = self
            .node
            .declare_parameter::<ParameterValue>(name)
            .default(ParameterValue::StringArray(default))
            .read_only()
            .with_context(|| format!("parameter {name} (a list of names)"))?;
        let value = match p.get() {
            ParameterValue::StringArray(names) => names.iter().map(|s| s.to_string()).collect(),
            // An empty list in YAML ([]) has no type: take it as no names.
            ParameterValue::ByteArray(b) if b.is_empty() => Vec::new(),
            ParameterValue::IntegerArray(b) if b.is_empty() => Vec::new(),
            ParameterValue::DoubleArray(b) if b.is_empty() => Vec::new(),
            ParameterValue::BoolArray(b) if b.is_empty() => Vec::new(),
            other => bail!("{name}:={other:?}: not a list of names"),
        };
        self.handles.push(Box::new(p));
        Ok(value)
    }
}

fn build(node: &Node) -> Result<Vec<Box<dyn std::any::Any>>> {
    let mut p = Params {
        node,
        handles: Vec::new(),
    };
    let defaults = Config::default();
    let cfg = Config {
        actuators: p.text("actuators", "status")?.parse()?,
        locks: p
            .names("locks", &["watchdog", "geofence", "output_gate"])?
            .iter()
            .map(|n| n.parse())
            .collect::<Result<_, _>>()?,
        clocks: p
            .names("clocks", &[])?
            .iter()
            .map(|n| n.parse())
            .collect::<Result<_, _>>()?,
        settle: p.number("settle", defaults.settle)?,
        line_timeout: p.number("line_timeout", defaults.line_timeout)?,
        status_timeout: p.number("status_timeout", defaults.status_timeout)?,
        max_clock_offset: p.number("max_clock_offset", defaults.max_clock_offset)?,
    };
    let rate = p.number("rate_hz", 2.0)?;
    if rate.is_nan() || rate <= 0.0 {
        bail!("rate_hz:={rate}: a rate must be above 0 Hz");
    }
    log_info!(
        node.logger(),
        "checking: sensors (the watchdog), actuators {:?}, locks [{}], clocks [{}]",
        cfg.actuators,
        cfg.locks
            .iter()
            .map(|l| l.name())
            .collect::<Vec<_>>()
            .join(", "),
        cfg.clocks
            .iter()
            .map(|c| c.name())
            .collect::<Vec<_>>()
            .join(", ")
    );
    let clocks = cfg.clocks.clone();
    let test = Test {
        node: node.clone(),
        clock: node.get_clock(),
        logger: node.logger().clone(),
        test: SelfTest::new(cfg),
        status: node.create_publisher("safety/self_test/status")?,
        last_line: String::new(),
    };
    let worker = node.create_worker(test);
    let mut keep: Vec<Box<dyn std::any::Any>> = Vec::new();
    keep.push(Box::new(worker.create_subscription(
        "safety/sensor_watchdog/state".keep_last(10),
        |s: &mut Test, m: Text| {
            let now = s.now();
            s.test.watchdog_line(&m.data, now);
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "safety/geofence/state".keep_last(10),
        |s: &mut Test, _: Text| {
            let now = s.now();
            s.test.lock_heard(Lock::Geofence, now);
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "safety/output_mux/state".keep_last(10),
        |s: &mut Test, _: Text| {
            let now = s.now();
            s.test.lock_heard(Lock::OutputGate, now);
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "vehicle/status".keep_last(10),
        |s: &mut Test, m: VehicleStatus| {
            let now = s.now();
            s.test.vehicle_status(m.faults, now);
        },
    )?));
    keep.push(Box::new(worker.create_subscription(
        "control/car/link_frame".keep_last(10),
        |s: &mut Test, _: LinkFrame| {
            let now = s.now();
            s.test.link_frame(now);
        },
    )?));
    for source in clocks {
        match source {
            ClockSource::Imu => keep.push(Box::new(worker.create_subscription(
                "sensors/imu/data".keep_last(10).best_effort(),
                |s: &mut Test, m: Imu| s.stamp(ClockSource::Imu, &m.header.stamp),
            )?)),
            ClockSource::Gps => keep.push(Box::new(worker.create_subscription(
                "sensors/gps/fix".keep_last(10).best_effort(),
                |s: &mut Test, m: NavSatFix| s.stamp(ClockSource::Gps, &m.header.stamp),
            )?)),
            ClockSource::Odom => keep.push(Box::new(worker.create_subscription(
                "vehicle/odom".keep_last(10).best_effort(),
                |s: &mut Test, m: Odometry| s.stamp(ClockSource::Odom, &m.header.stamp),
            )?)),
        }
    }
    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        |s: &mut Test| s.tick(),
    )?));
    keep.push(Box::new(worker));
    keep.extend(p.handles);
    Ok(keep)
}
