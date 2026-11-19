//! legacy_uart_bridge, in Rust: drive the ESP32 from ego's commands, and relay the Orin's
//! heartbeat. The same node as aav_legacy's legacy_uart_bridge.py, topic for topic.
//!
//! Subscribes  control/car/actuator_cmd  ackermann_msgs/AckermannDriveStamped: ego's command,
//!                               from the output mux (target car). Never /ego/actuator_cmd
//!             control/car/heartbeat     aav_interfaces/Heartbeat: the output mux's counter
//!             safety/estop      std_msgs/Bool (true: full brake now, until false)
//!             drivers/buttons/steering_calibration  aav_interfaces/SteeringCalibration
//!                               (uplink only): the panel's limp or jog, passed on only as
//!                               the relay's rules allow
//!             vehicle/odom      nav_msgs/Odometry, only with speed_loop.mode pi
//! Publishes   system/source     aav_interfaces/DataSource, latched: the actuators REAL (or SIM)
//!             safety/legacy_uart_bridge/lock  std_msgs/Bool, latched and again at 5 Hz:
//!                               true with frame_version 1 (no firmware timeout)
//!             control/car/link_frame  aav_interfaces/LinkFrame at link_frame_rate_hz:
//!                               what really went out, decoded from the bytes written
//!             and, only with uplink true:
//!             vehicle/status    aav_interfaces/VehicleStatus, 10 Hz
//!             vehicle/steering, vehicle/brake, vehicle/throttle  aav_interfaces/ActuatorState
//!             vehicle/rc        aav_interfaces/RcSticks (rc_driver reads it)
//!             vehicle/steering_actuator/raw, vehicle/brake_actuator/raw  aav_interfaces/ActuatorRaw
//!             vehicle/steering_actuator/calibration  std_msgs/String: on each change, 2 Hz
//!             vehicle/esp32/uplink_health  aav_interfaces/UplinkHealth, 1 Hz
//! Writes      frames at rate_hz on the serial port: frame_version 1 (13 bytes), 2 (17) or
//!             3 (20), packed by the firmware's own legacy-link
//! Reads       uplink frames on the same port, with uplink true
//! Parameters  every one of the Python's (aav_legacy's config/legacy.yaml); adaptive_brake
//!             (false; true needs speed_loop.mode pi), adaptive_brake.* (the car model) and
//!             brake.* (the brake's limits), from vehicle.yaml through legacy.launch.py
//!
//! port is a device (/dev/ttyAMA0) or socket://host:port (Ravensim's sim_esp32). If it won't
//! open, the bridge says so and tries again every second; no frames go out until it opens.
//! On exit it writes five stop frames, to leave the car braked. The logic is the
//! legacy-bridge crate, with no ROS. Where it differs from the Python, on purpose: a
//! calibration request with an unknown mode is logged and dropped (the Python's callback
//! raised, which stopped the node).

mod params;
mod port;

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use anyhow::{anyhow, bail, Result};
use legacy_bridge::link::frame_fields;
use legacy_bridge::uplink::{self, UplinkReader};
use legacy_bridge::{
    programs_locked, AdaptiveBrake, BrakeLimits, CalMode, CalWhere, CarModel, FrameVersion,
    LinkReport, Mapping, Relay, RelaySettings, SpeedConfig, SpeedControl, Tuning,
};
use legacy_link::{CalState, Uplink};
use rclrs::*;
use ros_env::aav_interfaces::msg::{
    ActuatorRaw, ActuatorState, DataSource, Heartbeat, LinkFrame, RcSticks, SensorSource,
    SteeringCalibration, UplinkHealth, VehicleStatus,
};
use ros_env::ackermann_msgs::msg::AckermannDriveStamped;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::nav_msgs::msg::Odometry;
use ros_env::std_msgs::msg::{Bool, Header, String as StringMsg};

use params::Declarer;
use port::Port;

/// How often a repeated error may be logged.
const ERROR_EVERY: Duration = Duration::from_secs(2);
/// s between tries to open the port.
const REOPEN_AFTER: f64 = 1.0;
/// s between /vehicle/status messages.
const STATUS_EVERY: f64 = 0.1;
/// s between repeats of the calibration line while the panel asks.
const CAL_LINE_EVERY: f64 = 0.5;
/// Stop frames written on the way out.
const LEAVING_FRAMES: usize = 5;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("legacy_uart_bridge")?;
    let (bridge, keep) = start(&node)?;
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    lock(&bridge).leave_braked();
    drop(keep);
    Ok(())
}

/// The things that must live as long as the node.
type Keep = Vec<Box<dyn std::any::Any>>;

/// Declares the parameters, opens the port, and starts every subscription and timer.
fn start(node: &Node) -> Result<(Arc<Mutex<Bridge>>, Keep)> {
    let mut d = Declarer::new(node);
    let port = d.text("port", "/dev/ttyAMA0")?;
    let baud = u32::try_from(d.integer("baud", 115_200)?)
        .map_err(|_| anyhow!("baud: not a serial speed"))?;
    let source = match d.text("source", "real")?.as_str() {
        "real" => SensorSource::REAL,
        "sim" => SensorSource::SIM,
        other => bail!("source '{other}': use real, or sim (Ravensim's stand-in)"),
    };
    let rate = d.number("rate_hz", 50.0)?;
    let cmd_timeout = d.number("cmd_timeout", 0.3)?;
    let version = FrameVersion::try_from(d.integer("frame_version", 2)?)?;
    let uplink_on = d.flag("uplink", false)?;
    let allow = d.flag("allow_programs_without_timeout", false)?;
    let uplink_timeout = d.number("uplink_timeout", 0.2)?;
    let calibration: CalWhere = d.text("steering_calibration", "none")?.parse()?;
    let defaults = Mapping::default();
    let mapping = Mapping {
        max_steering_angle: d.number("max_steering_angle", defaults.max_steering_angle)?,
        steering_sign: d.number("steering_sign", defaults.steering_sign)?,
        full_throttle_speed: d.number("full_throttle_speed", defaults.full_throttle_speed)?,
        stop_speed: d.number("stop_speed", defaults.stop_speed)?,
        hold_brake: d.number("hold_brake", defaults.hold_brake)?,
        stop_brake: d.number("stop_brake", defaults.stop_brake)?,
    };
    let pi = match d.text("speed_loop.mode", "open")?.as_str() {
        "open" => false,
        "pi" => true,
        other => bail!("speed_loop.mode '{other}': use open (the 2025-26 mapping) or pi"),
    };
    let odom_timeout = d.number("speed_loop.odom_timeout", 0.3)?;
    let speed_loop = if pi {
        let mut cfg = SpeedConfig::default();
        for (name, default) in SpeedConfig::fields() {
            cfg.set(name, d.number(&format!("speed_loop.{name}"), default)?)?;
        }
        Some(SpeedControl::new(cfg))
    } else {
        None
    };
    // The adaptive brake (TODO 258): only with the pi loop, which asks for a brake; the
    // car model and the brake's limits come from vehicle.yaml (legacy.launch.py).
    let adaptive = if d.flag("adaptive_brake", false)? {
        if !pi {
            bail!(
                "adaptive_brake needs speed_loop.mode pi: it sits after the speed loop, and \
                 open sends no brake to adapt"
            );
        }
        let car_default = CarModel::default();
        let car = CarModel {
            max_brake: d.number("adaptive_brake.max_brake", car_default.max_brake)?,
            brake_lag: d.number("adaptive_brake.brake_lag", car_default.brake_lag)?,
            brake_dead_time: d.number(
                "adaptive_brake.brake_dead_time",
                car_default.brake_dead_time,
            )?,
            drag: d.number("adaptive_brake.drag", car_default.drag)?,
            throttle_dead_time: d.number(
                "adaptive_brake.throttle_dead_time",
                car_default.throttle_dead_time,
            )?,
            throttle_lag: d.number("adaptive_brake.throttle_lag", car_default.throttle_lag)?,
        };
        let limits = BrakeLimits::new(
            d.number("brake.released", 400.0)?,
            d.number("brake.touch", 800.0)?,
            d.number("brake.full", 3500.0)?,
            d.number("brake.safe_max", 3200.0)?,
            d.number("brake.max_hold_time", 5.0)?,
        )?;
        Some(AdaptiveBrake::new(car, limits, Tuning::default()))
    } else {
        None
    };
    let link_rate = d.number("link_frame_rate_hz", 10.0)?;
    if rate.is_nan() || rate <= 0.0 {
        bail!("rate_hz:={rate}: a rate must be above 0 Hz");
    }

    let logger = node.logger().clone();
    if calibration != CalWhere::Nowhere {
        log_warn!(
            &logger,
            "steering_calibration {calibration:?}: the hardware panel may make the steering \
             actuator limp or jog it. Only with the hardware lead, the car on stands or the \
             ESP32 on the bench"
        );
    }
    if let Some(l) = &speed_loop {
        log_info!(
            &logger,
            "speed_loop.mode pi: closed on /vehicle/odom, {:?}",
            l.cfg
        );
    }
    let settings = RelaySettings {
        mapping,
        cmd_timeout,
        version,
        odom_timeout,
        calibration,
        uplink_timeout,
    };
    let latched = |topic: &'static str| topic.keep_last(1).reliable().transient_local();
    let uplink_pubs = if uplink_on {
        Some(UplinkPubs {
            status: node.create_publisher("vehicle/status")?,
            steering: node.create_publisher("vehicle/steering")?,
            throttle: node.create_publisher("vehicle/throttle")?,
            brake: node.create_publisher("vehicle/brake")?,
            health: node.create_publisher("vehicle/esp32/uplink_health")?,
            rc: node.create_publisher("vehicle/rc")?,
            steering_raw: node.create_publisher("vehicle/steering_actuator/raw")?,
            brake_raw: node.create_publisher("vehicle/brake_actuator/raw")?,
            calibration: node.create_publisher("vehicle/steering_actuator/calibration")?,
        })
    } else {
        None
    };
    let mut bridge = Bridge {
        relay: match adaptive {
            Some(brake) => {
                log_info!(
                    &logger,
                    "adaptive_brake: on, after the pi loop; never above brake.safe_max"
                );
                Relay::new(settings, speed_loop).with_adaptive_brake(brake)
            }
            None => Relay::new(settings, speed_loop),
        },
        link: LinkReport::new(link_rate)?,
        reader: UplinkReader::new(uplink_timeout),
        port: None,
        port_name: port,
        baud,
        last_open: 0.0,
        clock: node.get_clock(),
        logger: logger.clone(),
        link_pub: node.create_publisher("control/car/link_frame")?,
        uplink: uplink_pubs,
        last_status: None,
        uplink_was_fresh: false,
        cal_line: None,
        cal_said: None,
    };
    bridge.open_port();
    let bridge = Arc::new(Mutex::new(bridge));
    let mut keep: Keep = Vec::new();

    let b = Arc::clone(&bridge);
    keep.push(Box::new(node.create_subscription(
        "control/car/actuator_cmd",
        move |msg: AckermannDriveStamped| {
            let mut b = lock(&b);
            let now = b.now();
            b.relay.command(
                f64::from(msg.drive.speed),
                f64::from(msg.drive.steering_angle),
                now,
            );
        },
    )?));
    let b = Arc::clone(&bridge);
    keep.push(Box::new(node.create_subscription(
        "control/car/heartbeat",
        move |msg: Heartbeat| lock(&b).relay.heartbeat(msg.counter),
    )?));
    let b = Arc::clone(&bridge);
    keep.push(Box::new(node.create_subscription(
        "safety/estop",
        move |msg: Bool| {
            let mut b = lock(&b);
            if b.relay.set_estop(msg.data) {
                if msg.data {
                    log_warn!(&b.logger, "e-stop ON: braking");
                } else {
                    log_warn!(&b.logger, "e-stop released");
                }
            }
        },
    )?));
    if pi {
        let b = Arc::clone(&bridge);
        keep.push(Box::new(node.create_subscription(
            "vehicle/odom",
            move |msg: Odometry| {
                let mut b = lock(&b);
                let now = b.now();
                b.relay.wheel_speed(msg.twist.twist.linear.x, now);
            },
        )?));
    }
    let b = Arc::clone(&bridge);
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        move || lock(&b).tick(),
    )?));
    if uplink_on {
        let b = Arc::clone(&bridge);
        keep.push(Box::new(node.create_timer_repeating(
            Duration::from_secs(1).node_time(),
            move || lock(&b).publish_health(),
        )?));
        let b = Arc::clone(&bridge);
        keep.push(Box::new(node.create_subscription(
            "drivers/buttons/steering_calibration",
            move |msg: SteeringCalibration| {
                let mut b = lock(&b);
                let now = b.now();
                match CalMode::try_from(msg.mode) {
                    Ok(mode) => b.relay.calibrate(mode, i64::from(msg.target), now),
                    Err(e) => log_error!(b.logger.throttle(ERROR_EVERY), "{e}"),
                }
            },
        )?));
        let b = Arc::clone(&bridge);
        keep.push(Box::new(node.create_timer_repeating(
            Duration::from_millis(10).node_time(),
            move || lock(&b).read_uplink(),
        )?));
    }

    // Say once, latched, that this drives the real car (/system/source).
    let source_pub = node.create_publisher::<DataSource>(latched("system/source"))?;
    let mut sensors = vec![SensorSource {
        name: "actuators".into(),
        topic: String::new(),
        source,
    }];
    if uplink_on {
        sensors.extend(
            [
                ("vehicle", "/vehicle/status"),
                ("steering", "/vehicle/steering"),
                ("throttle", "/vehicle/throttle"),
                ("brake", "/vehicle/brake"),
                ("rc", "/vehicle/rc"),
            ]
            .iter()
            .map(|(name, topic)| SensorSource {
                name: (*name).into(),
                topic: (*topic).into(),
                source,
            }),
        );
    }
    source_pub.publish(DataSource {
        header: Header {
            stamp: lock(&bridge).stamp(),
            frame_id: String::new(),
        },
        announcer: node.name(),
        sensors,
    })?;
    keep.push(Box::new(source_pub));

    // The no-timeout lock: latched for late joiners that ask for it, and sent again at
    // 5 Hz, as twist_mux subscribes volatile and would miss a message sent before it
    // started (on the car it runs on the Orin, this on the Motor Pi).
    let locked = programs_locked(version, allow);
    let lock_pub = node.create_publisher::<Bool>(latched("safety/legacy_uart_bridge/lock"))?;
    lock_pub.publish(Bool { data: locked })?;
    let again = lock_pub.clone();
    keep.push(Box::new(node.create_timer_repeating(
        Duration::from_millis(200).node_time(),
        move || {
            let _ = again.publish(Bool { data: locked });
        },
    )?));
    keep.push(Box::new(lock_pub));
    if version == FrameVersion::V1 {
        log_warn!(
            &logger,
            "frame_version 1: the C++ firmware has no command timeout and no heartbeat; keep \
             the e-stop in reach"
        );
        if locked {
            log_warn!(
                &logger,
                "no-timeout lock ON (/safety/legacy_uart_bridge/lock): twist_mux holds every \
                 program (scripted, replay, point, navigation, AI); people and the actuator \
                 tests still drive"
            );
        } else {
            log_error!(
                &logger,
                "allow_programs_without_timeout: programs may drive through firmware with NO \
                 command timeout. Supervised stand only: wheels off the ground, a person at \
                 the e-stop"
            );
        }
    } else {
        log_info!(
            &logger,
            "frame_version {version}: relaying the counter of the Orin from \
             control/car/heartbeat; the ESP32 brakes 100 ms after it stops changing"
        );
    }
    keep.extend(d.finish());
    Ok((bridge, keep))
}

/// The bridge, even if a callback panicked while holding it: the car must still be braked.
fn lock(b: &Mutex<Bridge>) -> MutexGuard<'_, Bridge> {
    b.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// What only the uplink publishes.
struct UplinkPubs {
    status: Publisher<VehicleStatus>,
    steering: Publisher<ActuatorState>,
    throttle: Publisher<ActuatorState>,
    brake: Publisher<ActuatorState>,
    health: Publisher<UplinkHealth>,
    rc: Publisher<RcSticks>,
    steering_raw: Publisher<ActuatorRaw>,
    brake_raw: Publisher<ActuatorRaw>,
    calibration: Publisher<StringMsg>,
}

/// The node's state: the relay, the port, the uplink reader and the publishers.
struct Bridge {
    relay: Relay,
    link: LinkReport,
    reader: UplinkReader,
    port: Option<Port>,
    port_name: String,
    baud: u32,
    last_open: f64,
    clock: Clock,
    logger: Logger,
    link_pub: Publisher<LinkFrame>,
    uplink: Option<UplinkPubs>,
    last_status: Option<f64>,
    uplink_was_fresh: bool,
    cal_line: Option<String>,
    cal_said: Option<f64>,
}

impl Bridge {
    /// s, this node's clock (the simulator's with use_sim_time).
    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn stamp(&self) -> Time {
        self.clock.now().to_ros_msg().unwrap_or_default()
    }

    fn header(&self) -> Header {
        Header {
            stamp: self.stamp(),
            frame_id: String::new(),
        }
    }

    /// Opens the port; on failure, says so and tries again later.
    fn open_port(&mut self) {
        self.last_open = self.now();
        match Port::open(&self.port_name, self.baud) {
            Ok(p) => {
                self.port = Some(p);
                log_info!(&self.logger, "serial port {} open", self.port_name);
            }
            Err(e) => {
                self.port = None;
                log_error!(
                    self.logger.throttle(Duration::from_secs(10)),
                    "serial port {}: {e}; trying again every second",
                    self.port_name
                );
            }
        }
    }

    /// One tick: this tick's frame, written; a sample of it reported when due.
    fn tick(&mut self) {
        let now = self.now();
        let Some(port) = self.port.as_mut() else {
            if now - self.last_open >= REOPEN_AFTER {
                self.open_port();
            }
            return;
        };
        let frame = self.relay.frame(now);
        match port.write(&frame) {
            Err(e) => {
                self.link.failed();
                log_error!(
                    self.logger.throttle(ERROR_EVERY),
                    "serial write failed: {e}; reopening"
                );
                self.port = None;
            }
            Ok(sent) if !self.link.wrote(&frame, sent) => log_error!(
                self.logger.throttle(ERROR_EVERY),
                "serial write took {sent} of {} bytes",
                frame.len()
            ),
            Ok(_) => {
                if self.link.due(now) {
                    self.publish_link_frame(&frame);
                }
            }
        }
    }

    /// Reports a frame just written. A report must never stop the frames: log, carry on.
    fn publish_link_frame(&self, frame: &[u8]) {
        let sent = frame_fields(frame)
            .map_err(anyhow::Error::from)
            .and_then(|f| {
                self.link_pub
                    .publish(LinkFrame {
                        header: self.header(),
                        version: f.version,
                        counter: f.counter,
                        armed: f.armed,
                        estop: f.estop,
                        steering: f.steering,
                        speed: f.speed,
                        accel: f.accel,
                        cal_mode: f.cal_mode,
                        jog_target: f.jog_target,
                        check: f.check,
                        written: self.link.written(),
                        write_errors: self.link.write_errors(),
                    })
                    .map_err(anyhow::Error::from)
            });
        if let Err(e) = sent {
            log_error!(
                self.logger.throttle(ERROR_EVERY),
                "link frame not reported: {e}"
            );
        }
    }

    /// The uplink reader's counts (1 Hz).
    fn publish_health(&self) {
        let Some(pubs) = &self.uplink else { return };
        let h = self.reader.health(self.now());
        let _ = pubs.health.publish(UplinkHealth {
            header: self.header(),
            fresh: h.fresh,
            frames: h.frames,
            lost: h.lost,
            bad: h.bad,
        });
    }

    /// Reads the ESP32's uplink frames (100 Hz), and publishes each one.
    fn read_uplink(&mut self) {
        let now = self.now();
        let Some(port) = self.port.as_mut() else {
            return;
        };
        let data = match port.read_waiting() {
            Ok(data) => data,
            Err(e) => {
                log_error!(self.logger.throttle(ERROR_EVERY), "serial read failed: {e}");
                return;
            }
        };
        let stamp = self.stamp();
        for u in self.reader.feed(&data, now) {
            self.relay.uplink_seen(now);
            self.publish_uplink(&u, &stamp, now);
        }
        self.say_calibration(now);
        let fresh = self.reader.fresh(now);
        if fresh != self.uplink_was_fresh {
            if fresh {
                log_info!(&self.logger, "uplink: frames from the ESP32");
            } else {
                log_warn!(
                    &self.logger,
                    "uplink: no frame from the ESP32 for {:.1} s (TX1 wired? firmware built \
                     with uplink?)",
                    self.reader.timeout
                );
            }
            self.uplink_was_fresh = fresh;
        }
    }

    /// The calibration's line, on each change and at 2 Hz while asked.
    fn say_calibration(&mut self, now: f64) {
        let Some(pubs) = &self.uplink else { return };
        let wanted = self.relay.cal_wanted(now);
        let line =
            uplink::calibration_line(wanted, self.relay.cal_refusal(now), self.reader.latest());
        let due = wanted
            && self
                .cal_said
                .is_none_or(|said| now - said >= CAL_LINE_EVERY);
        let changed = self.cal_line.as_deref() != Some(line.as_str());
        if changed || due {
            if changed {
                log_info!(&self.logger, "steering calibration: {line}");
            }
            let _ = pubs.calibration.publish(StringMsg { data: line.clone() });
            self.cal_line = Some(line);
            self.cal_said = Some(now);
        }
    }

    fn publish_uplink(&mut self, u: &Uplink, stamp: &Time, now: f64) {
        let command = self.relay.fresh_command(now).map(|c| c.steering);
        let due = self
            .last_status
            .is_none_or(|last| now - last >= STATUS_EVERY);
        if due {
            self.last_status = Some(now);
        }
        let Some(pubs) = &self.uplink else { return };
        let header = || Header {
            stamp: stamp.clone(),
            frame_id: String::new(),
        };
        let rc = uplink::rc(u);
        let _ = pubs.rc.publish(RcSticks {
            header: header(),
            mode_switch: rc.mode_switch,
            throttle: rc.throttle,
            steering: rc.steering,
            brake: rc.brake,
        });
        for (publisher, (command, measured)) in [
            (&pubs.steering, uplink::steering_state(u, command)),
            (&pubs.throttle, uplink::throttle_state(u)),
            (&pubs.brake, uplink::brake_state(u)),
        ] {
            let _ = publisher.publish(ActuatorState {
                header: header(),
                command,
                measured,
            });
        }
        for (publisher, raw) in [
            (
                &pubs.steering_raw,
                uplink::raw(u.counts.steering, u.calibration),
            ),
            (&pubs.brake_raw, uplink::raw(u.counts.brake, CalState::Off)),
        ] {
            let _ = publisher.publish(ActuatorRaw {
                header: header(),
                valid: raw.valid,
                position: raw.position,
                calibration: raw.calibration,
            });
        }
        if due {
            let s = uplink::status(u);
            let _ = pubs.status.publish(VehicleStatus {
                header: Header {
                    stamp: stamp.clone(),
                    frame_id: "base_link".into(),
                },
                mode: s.mode,
                armed: s.armed,
                estop_active: s.estop_active,
                speed: s.speed,
                steering_angle: s.steering_angle,
                throttle: s.throttle,
                brake: s.brake,
                faults: s.faults,
            });
        }
    }

    /// Leaves the car braked: five stop frames, then the port closed.
    fn leave_braked(&mut self) {
        let Some(port) = self.port.as_mut() else {
            return;
        };
        for _ in 0..LEAVING_FRAMES {
            let frame = self.relay.stop_frame();
            match port.write(&frame) {
                Ok(sent) => {
                    self.link.wrote(&frame, sent);
                }
                Err(_) => {
                    self.link.failed();
                    break;
                }
            }
        }
        self.port = None;
    }
}
