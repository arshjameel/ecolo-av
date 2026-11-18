//! stop_zone: the stop zone as a ROS 2 node, **the car's stop zone** since 17 Nov (TODO
//! 182): it sits between the smoother and twist_mux and sends the command on, or zero when
//! points stand in the box ahead (behind, reversing) or a sensor goes silent. It ran in
//! shadow beside Nav2's collision_monitor first and stopped every time it did, no later
//! (docs/safety-case.md). The decision is the `stop-zone` crate.
//!
//! Subscribes  cmd_in (/control/cmd_vel_smoothed)
//!                                         geometry_msgs/TwistStamped: the command to check
//!             each source's topic         sensor_msgs/LaserScan or PointCloud2, best
//!                                         effort, as nav2.yaml's collision_monitor block
//!                                         names them (the radar, the lidar scan, the
//!                                         cubes, the kerbs' steps, the ghost...)
//! Publishes   cmd_out (/drivers/nav/cmd_vel; shadow: /safety/stop_zone/shadow/cmd_vel)
//!                                         geometry_msgs/TwistStamped: the command, or zero
//!                                         when it stops (none stop_pub_timeout s after a
//!                                         stop, as Nav2's)
//!             state (/safety/stop_zone/state; shadow: /safety/stop_zone/shadow/state)
//!                                         nav2_msgs/CollisionMonitorState, on each change
//!                                         (as collision_monitor's): STOP with the box's
//!                                         polygon name, or "invalid source" for a silent
//!                                         sensor; DO_NOTHING when clear. stop_hold reads it
//!             status (/safety/stop_zone/status; shadow: /safety/stop_zone/shadow/status)
//!                                         diagnostic_msgs/DiagnosticStatus, each command:
//!                                         OK or WARN and why; the box; the stop
//!             box (/safety/stop_zone/box; shadow: /safety/stop_zone/shadow/box)
//!                                         geometry_msgs/PolygonStamped, base_link, each
//!                                         command: the box in use, for rviz and Foxglove
//!                                         (collision_monitor's polygon_pub_topic)
//! Parameters  settings (JSON: aav_description's vehicle.stop_zone_params: the boxes, the
//!             polygon's name, min_points, the timeouts, each source's topic, kind,
//!             heights, range and mount); cmd_in, cmd_out, state, status, box. The
//!             defaults are the shadow's topics: a stop zone started by hand drives nothing.
//!
//! navigation.launch.py starts it wherever a sensor can feed it (stop_zone:=).

use std::sync::Arc;

use anyhow::{Context as _, Result};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{Point32, Polygon as Outline, PolygonStamped, Twist};
use ros_env::geometry_msgs::msg::{TwistStamped, Vector3};
use ros_env::nav2_msgs::msg::CollisionMonitorState;
use ros_env::sensor_msgs::msg::{LaserScan, PointCloud2};
use ros_env::std_msgs::msg::Header;
use serde::Deserialize;
use stop_zone::{
    Command, Mount, Point, Polygon, Reading, Settings, Source, SourceKind, SpeedBand, StopZone, Why,
};

/// The JSON settings, as vehicle.stop_zone_params writes them.
#[derive(Deserialize)]
struct Json {
    /// collision_monitor's polygon name ("StopZone"): what the state says when it stops.
    #[serde(default = "stop_zone_name")]
    polygon: String,
    min_points: usize,
    source_timeout: f64,
    stop_pub_timeout: f64,
    bands: Vec<JsonBand>,
    sources: Vec<JsonSource>,
}

#[derive(Deserialize)]
struct JsonBand {
    name: String,
    points: Vec<[f64; 2]>,
    speed: [f64; 2],
    turn: [f64; 2],
}

#[derive(Deserialize)]
struct JsonSource {
    name: String,
    topic: String,
    /// "scan" or "pointcloud" (nav2.yaml's type).
    kind: String,
    #[serde(default = "low")]
    min_height: f64,
    #[serde(default = "high")]
    max_height: f64,
    #[serde(default)]
    min_range: f64,
    /// [x, y, z, yaw] in base_link: where the source's frame sits.
    #[serde(default)]
    mount: [f64; 4],
}

fn stop_zone_name() -> String {
    "StopZone".into()
}

fn low() -> f64 {
    0.05 // Nav2's defaults
}

fn high() -> f64 {
    0.5
}

impl Json {
    fn zone(&self) -> Result<StopZone> {
        let bands = self
            .bands
            .iter()
            .map(|b| SpeedBand {
                name: b.name.clone(),
                polygon: Polygon(b.points.iter().map(|[x, y]| Point::new(*x, *y)).collect()),
                speed: b.speed[0]..=b.speed[1],
                turn: b.turn[0]..=b.turn[1],
            })
            .collect();
        let sources = self
            .sources
            .iter()
            .map(|s| {
                let kind = match s.kind.as_str() {
                    "scan" => Ok(SourceKind::Scan),
                    "pointcloud" => Ok(SourceKind::Cloud {
                        min_height: s.min_height,
                        max_height: s.max_height,
                    }),
                    other => Err(anyhow::anyhow!("source {}: type {other}?", s.name)),
                }?;
                let [x, y, z, yaw] = s.mount;
                Ok(Source {
                    name: s.name.clone(),
                    kind,
                    mount: Mount { x, y, z, yaw },
                    min_range: s.min_range,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        let settings = Settings {
            bands,
            min_points: self.min_points,
            source_timeout: self.source_timeout,
            stop_pub_timeout: self.stop_pub_timeout,
        };
        Ok(StopZone::new(settings, sources)?)
    }
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("stop_zone")?;
    let text = |name: &str, default: &str| -> Result<String> {
        let p = node
            .declare_parameter::<Arc<str>>(name)
            .default(default.into())
            .read_only()?;
        Ok(p.get().to_string())
    };
    let json = text("settings", "")?;
    let cmd_in = text("cmd_in", "/control/cmd_vel_smoothed")?;
    let cmd_out = text("cmd_out", "/safety/stop_zone/shadow/cmd_vel")?;
    let state_topic = text("state", "/safety/stop_zone/shadow/state")?;
    let status_topic = text("status", "/safety/stop_zone/shadow/status")?;
    let box_topic = text("box", "/safety/stop_zone/shadow/box")?;
    let spec: Json = serde_json::from_str(&json).context("parameter settings: JSON")?;
    let zone = spec.zone().context("the stop zone's settings")?;
    log_info!(
        node.logger(),
        "stop zone: {} boxes, {} points stop, sources [{}]; {cmd_in} -> {cmd_out}",
        spec.bands.len(),
        spec.min_points,
        spec.sources
            .iter()
            .map(|s| format!("{} {}", s.name, s.topic))
            .collect::<Vec<_>>()
            .join(", ")
    );

    let core = Core {
        readings: vec![None; zone.sources().len()],
        zone,
        clock: node.get_clock(),
        out: node.create_publisher(cmd_out.as_str())?,
        state: node.create_publisher(state_topic.as_str())?,
        status: node.create_publisher(status_topic.as_str())?,
        drawn: node.create_publisher(box_topic.as_str())?,
        boxes: spec
            .bands
            .iter()
            .map(|b| (b.name.clone(), outline(&b.points)))
            .collect(),
        logger: node.logger().clone(),
        polygon: spec.polygon.clone(),
        last_stop: None,
        last_state: None,
    };
    let worker = node.create_worker(core);
    let mut keep: Vec<Box<dyn std::any::Any>> = Vec::new();
    for (i, source) in spec.sources.iter().enumerate() {
        let topic = source.topic.as_str().best_effort();
        if source.kind == "scan" {
            keep.push(Box::new(worker.create_subscription(
                topic,
                move |s: &mut Core, msg: LaserScan| s.readings[i] = Some(scan_reading(&msg)),
            )?));
        } else {
            keep.push(Box::new(worker.create_subscription(
                topic,
                move |s: &mut Core, msg: PointCloud2| s.readings[i] = Some(cloud_reading(&msg)),
            )?));
        }
    }
    let _cmd = worker.create_subscription(cmd_in.as_str(), |s: &mut Core, msg: TwistStamped| {
        s.on_cmd(&msg)
    })?;

    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The worker's payload.
struct Core {
    zone: StopZone,
    readings: Vec<Option<Reading>>,
    clock: Clock,
    out: Publisher<TwistStamped>,
    state: Publisher<CollisionMonitorState>,
    status: Publisher<DiagnosticStatus>,
    /// The box in use, drawn.
    drawn: Publisher<PolygonStamped>,
    /// Each band's box, by its name, ready to send.
    boxes: Vec<(String, Outline)>,
    logger: Logger,
    /// collision_monitor's polygon name for the box.
    polygon: String,
    /// The last stop's reason, to log only changes.
    last_stop: Option<String>,
    /// The last state sent, to send only changes (as collision_monitor).
    last_state: Option<(u8, String)>,
}

/// A box's corners as a ROS polygon.
fn outline(points: &[[f64; 2]]) -> Outline {
    Outline {
        points: points
            .iter()
            .map(|&[x, y]| Point32 {
                x: x as f32,
                y: y as f32,
                z: 0.0,
            })
            .collect(),
    }
}

/// The state collision_monitor would send for this stop: (action, polygon name).
fn monitor_state(stop: Option<&Why>, polygon: &str) -> (u8, String) {
    match stop {
        None => (CollisionMonitorState::DO_NOTHING, String::new()),
        Some(why) => (CollisionMonitorState::STOP, why.polygon_name(polygon)),
    }
}

impl Core {
    fn on_cmd(&mut self, msg: &TwistStamped) {
        let now = self.clock.now().nsec as f64 * 1e-9;
        let cmd = Command {
            x: msg.twist.linear.x,
            turn: msg.twist.angular.z,
        };
        let readings: Vec<Option<&Reading>> = self.readings.iter().map(Option::as_ref).collect();
        let decision = self.zone.step(now, cmd, &readings);
        if let Some(out) = decision.out {
            let twist = Twist {
                linear: Vector3 {
                    x: out.x,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: out.turn,
                },
            };
            let _ = self.out.publish(TwistStamped {
                header: msg.header.clone(),
                twist,
            });
        }
        let why = decision.stop.as_ref().map(ToString::to_string);
        if why != self.last_stop {
            match &why {
                Some(why) => log_warn!(&self.logger, "stop: {why}"),
                None => log_info!(&self.logger, "clear"),
            }
            self.last_stop = why.clone();
        }
        let state = monitor_state(decision.stop.as_ref(), &self.polygon);
        if self.last_state.as_ref() != Some(&state) {
            let _ = self.state.publish(CollisionMonitorState {
                action_type: state.0,
                polygon_name: state.1.clone(),
            });
            self.last_state = Some(state);
        }
        let band = decision.band.clone().unwrap_or_default();
        if let Some((_, polygon)) = self.boxes.iter().find(|(name, _)| *name == band) {
            let _ = self.drawn.publish(PolygonStamped {
                header: Header {
                    stamp: msg.header.stamp.clone(),
                    frame_id: "base_link".into(),
                },
                polygon: polygon.clone(),
            });
        }
        let _ = self.status.publish(DiagnosticStatus {
            level: if why.is_some() { 1 } else { 0 },
            name: "stop_zone".into(),
            message: why.clone().unwrap_or_else(|| "clear".into()),
            hardware_id: String::new(),
            values: [
                ("stop", why.is_some().to_string()),
                ("band", band),
                ("sent", decision.out.is_some().to_string()),
            ]
            .into_iter()
            .map(|(key, value)| KeyValue {
                key: key.into(),
                value,
            })
            .collect(),
        });
    }
}

fn seconds(t: &Time) -> f64 {
    f64::from(t.sec) + f64::from(t.nanosec) * 1e-9
}

/// A scan's rays as points on its plane (its own frame); inf, NaN and out-of-range rays
/// left out.
fn scan_reading(msg: &LaserScan) -> Reading {
    let (min, max) = (f64::from(msg.range_min), f64::from(msg.range_max));
    let points = msg
        .ranges
        .iter()
        .enumerate()
        .filter_map(|(k, &r)| {
            let r = f64::from(r);
            (r.is_finite() && r >= min && r <= max).then(|| {
                let a = f64::from(msg.angle_min) + k as f64 * f64::from(msg.angle_increment);
                [r * a.cos(), r * a.sin(), 0.0]
            })
        })
        .collect();
    Reading {
        stamp: seconds(&msg.header.stamp),
        points,
    }
}

/// A cloud's float32 x, y, z (its own frame); a cloud without them reads as empty.
fn cloud_reading(msg: &PointCloud2) -> Reading {
    let field = |name: &str| {
        msg.fields
            .iter()
            .find(|f| f.name == name && f.datatype == 7) // FLOAT32
            .map(|f| f.offset as usize)
    };
    let step = msg.point_step as usize;
    let points = match (field("x"), field("y"), field("z")) {
        (Some(x), Some(y), Some(z)) if step > 0 => {
            let read = |p: &[u8], at: usize| -> Option<f64> {
                let b: [u8; 4] = p.get(at..at + 4)?.try_into().ok()?;
                Some(f64::from(if msg.is_bigendian {
                    f32::from_be_bytes(b)
                } else {
                    f32::from_le_bytes(b)
                }))
            };
            msg.data
                .chunks_exact(step)
                .filter_map(|p| Some([read(p, x)?, read(p, y)?, read(p, z)?]))
                .collect()
        }
        _ => Vec::new(),
    };
    Reading {
        stamp: seconds(&msg.header.stamp),
        points,
    }
}
