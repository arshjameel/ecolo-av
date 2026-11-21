//! costmap: our own local and global costmaps (Nav2's layers ported; the only ones since 17 Nov).
//!
//! Subscribes  /map                        nav_msgs/OccupancyGrid, transient local: the
//!                                         static layer (not when every costmap is made
//!                                         of chunks)
//!             localization/odom/local     nav_msgs/Odometry (odom_topic): the car in odom,
//!                                         kept 2 s to place each scan at its own stamp.
//!                                         Older than transform_tolerance (or map -> odom,
//!                                         for the global one): no update, no grid
//!             /tf                         tf2_msgs/TFMessage: map -> odom
//!             /tf_static                  tf2_msgs/TFMessage: each sensor's mount
//!             /plan                       nav_msgs/Path (route_topic), only with a lanes
//!                                         block: the newest route, so the lanes layer
//!                                         knows which way each road is driven
//!             /perception/lane/geometry   aav_lane_geometry/LaneGeometry (the lanes
//!                                         block's camera.topic), only with a lanes
//!                                         block whose source is camera or both: the
//!                                         camera's lines, drawn as lane costs where the
//!                                         lane map has no road (TODO 264)
//!             drivers/nav/planner/route   nav_msgs/Path (ahead_topic), only with a
//!                                         chunks block: the route level's whole way to
//!                                         the goal; its chunks ahead are held too
//!             each source's topic         sensor_msgs/LaserScan or PointCloud2, best
//!                                         effort, as nav2.yaml's layers name them (the
//!                                         lidar scan; the radar, the cubes, the cameras'
//!                                         points where they run)
//! Publishes   drivers/nav/costmap/local   nav_msgs/OccupancyGrid, odom, at the local
//!                                         block's publish_frequency (Nav2's 0-100 table)
//!             drivers/nav/costmap/local_raw  nav2_msgs/Costmap: the same, Nav2's raw
//!                                         costs (0-255), for aav_mppi_lag
//!                                         (costmap_topic)
//!             drivers/nav/costmap/global, .../global_raw  the same for the global one,
//!                                         map frame, the map's size
//!             drivers/nav/costmap/status  diagnostic_msgs/DiagnosticStatus, at each
//!                                         publish: the update times (ms), the lethal
//!                                         cells, what it is waiting for, the routes
//!                                         that redrew the lanes
//! Serves      drivers/nav/costmap/clear_entirely_local, .../clear_entirely_global
//!                                         nav2_msgs/srv/ClearEntireCostmap: forget every
//!                                         mark, as Nav2's clear_entirely_* do. With
//!                                         costmap:=own and tracker:=mppi_lag the launch
//!                                         file points bt_navigator's local clear here
//! Calls       nothing.
//! Parameters  local, global: each a JSON block (aav_description's
//!             vehicle.costmap_params, from nav2.yaml); empty: that costmap doesn't run.
//!             A global block with "lanes" (navigation.launch.py lane_costs:=) adds the
//!             lanes layer: the roads as a soft cost (costmap::lanes, TODO 148). Its
//!             "source" (lane_source:=): "map" (the default: its "roads"), "camera"
//!             (the lane finder's lines, costmap::camera_lanes, TODO 264; "camera":
//!             {topic, min_confidence, max_age, behind, ahead, edge_inset}) or "both"
//!             (the map wins where it has a road).
//!             A global block with "chunks" ({dir, ahead, margin}; navigation.launch.py
//!             costmap_chunks:=) builds it from the map's 64 m chunks round the car and
//!             along the route ahead, not from /map (costmap::chunked, TODO 175).
//!             A local block with "chunks" (navigation.launch.py local_chunks:=true): its
//!             static layer from the 3 x 3 chunks round the car, not /map. Every costmap
//!             made of chunks: no /map subscription at all.
//!             odom_topic, map_topic, base_frame, and the topic names above.
//!
//! navigation.launch.py costmap:=own starts it. Nav2's own costmaps keep running. The
//! maths is the `costmap` crate; this file only moves messages.
//!
//! Threads, as Nav2's (each Costmap2DROS has its own map update thread): one rclrs worker
//! takes every message and, on each costmap's timer, gathers its inputs and hands them to
//! that costmap's own thread, which updates and publishes it. So the global costmap's
//! slow work (a new map on the campus: half a second) never holds up the local one. A
//! thread still busy when its next inputs come skips to the newest (`skipped` on the
//! status). A clear is handed over the same way and done before the next update.

use std::{
    any::Any,
    collections::{HashMap, VecDeque},
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc,
    },
    thread::JoinHandle,
    time::{Duration, Instant},
};

use anyhow::{anyhow, bail, Context as _, Result};
use costmap::{
    cost::{occupancy, LETHAL},
    footprint, Arrival, CameraLanes, CameraSettings, Changed, ChunkSource, Costmap, Lanes,
    Observation, ObstacleLayer, Paint, Pose2, PoseHistory, Ramp, Road, Seen, SeenLine, Settings,
    Source, StaticMap, Transform3,
};
use rclrs::*;
use ros_env::aav_lane_geometry::msg::LaneGeometry;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{Point, Pose as PoseMsg, Quaternion};
use ros_env::nav2_msgs::msg::{Costmap as CostmapMsg, CostmapMetaData};
use ros_env::nav2_msgs::srv::{
    ClearEntireCostmap, ClearEntireCostmap_Request, ClearEntireCostmap_Response,
};
use ros_env::nav_msgs::msg::{MapMetaData, OccupancyGrid, Odometry, Path};
use ros_env::sensor_msgs::msg::{LaserScan, PointCloud2};
use ros_env::std_msgs::msg::Header;
use ros_env::tf2_msgs::msg::TFMessage;
use serde_json::Value;

/// How long the odometry is kept, s: a scan is placed at its own stamp.
const ODOM_KEPT: f64 = 2.0;
/// Messages kept per sensor topic: enough for the radar's 0.2 s persistence.
const KEPT_PER_TOPIC: usize = 8;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("costmap")?;

    let mut handles: Vec<Box<dyn Any>> = Vec::new();
    let mut text_param = |name: &str, default: &str| -> Result<String> {
        let p = node
            .declare_parameter::<Arc<str>>(name)
            .default(default.into())
            .read_only()
            .with_context(|| format!("parameter {name} (text)"))?;
        let value = p.get().to_string();
        handles.push(Box::new(p));
        Ok(value)
    };
    let local_json = text_param("local", "")?;
    let global_json = text_param("global", "")?;
    let odom_topic = text_param("odom_topic", "/localization/odom/local")?;
    let map_topic = text_param("map_topic", "/map")?;
    let base_frame = text_param("base_frame", "base_link")?;
    let local_topic = text_param("local_topic", "/drivers/nav/costmap/local")?;
    let global_topic = text_param("global_topic", "/drivers/nav/costmap/global")?;
    let status_topic = text_param("status_topic", "/drivers/nav/costmap/status")?;
    let clear_prefix = text_param("clear_prefix", "/drivers/nav/costmap/clear_entirely_")?;
    let route_topic = text_param("route_topic", "/plan")?;
    let ahead_topic = text_param("ahead_topic", "/drivers/nav/planner/route")?;

    let status: Publisher<DiagnosticStatus> = node.create_publisher(status_topic.as_str())?;
    let mut maps = Vec::new();
    let mut threads = Vec::new();
    for (name, json, topic) in [
        ("local", &local_json, &local_topic),
        ("global", &global_json, &global_topic),
    ] {
        if json.trim().is_empty() {
            continue;
        }
        let block: Value =
            serde_json::from_str(json).with_context(|| format!("parameter {name}: JSON"))?;
        let spec = Spec::build(name, &block).with_context(|| format!("the {name} costmap"))?;
        log_info!(node.logger(), "{}", spec.describe());
        let (jobs, inbox) = mpsc::channel();
        maps.push(Handle {
            name,
            in_map: spec.frame == "map",
            lanes: spec.costmap.lanes().is_some(),
            camera: spec.camera.clone(),
            camera_shown: false,
            chunks: spec.costmap.chunked(),
            inputs: spec.inputs.clone(),
            update_hz: spec.update_hz,
            transform_tolerance: spec.transform_tolerance,
            jobs,
        });
        let one = One::new(
            spec,
            node.create_publisher(topic.as_str())?,
            node.create_publisher(format!("{topic}_raw").as_str())?,
            Arc::clone(&status),
            node.get_clock(),
            node.logger().clone(),
        );
        threads.push(
            std::thread::Builder::new()
                .name(format!("costmap-{name}"))
                .spawn(move || one.run(&inbox))
                .with_context(|| format!("starting the {name} costmap's thread"))?,
        );
    }
    if maps.is_empty() {
        bail!("neither local nor global is set: nothing to do");
    }

    // Every topic any source reads, once, and its kind.
    let mut topics: Vec<(String, Kind)> = Vec::new();
    for input in maps.iter().flat_map(|m| m.inputs.iter().flatten()) {
        if !topics.iter().any(|(name, _)| *name == input.topic) {
            topics.push((input.topic.clone(), input.kind));
        }
    }
    let rates: Vec<f64> = maps.iter().map(|m| m.update_hz).collect();
    let lanes = maps.iter().any(|m| m.lanes);
    let camera_topic = maps
        .iter()
        .find_map(|m| m.camera.as_ref().map(|c| c.topic.clone()));
    let chunks = maps.iter().any(|m| m.chunks);
    // Every costmap made of chunks: /map isn't needed (on the campus, 41 million cells).
    let wants_map = maps.iter().any(|m| !m.chunks);
    let names: Vec<&'static str> = maps.iter().map(|m| m.name).collect();

    let core = Core {
        maps,
        halt: Arc::clone(executor.commands()),
        clock: node.get_clock(),
        logger: node.logger().clone(),
        base_frame,
        raw: HashMap::new(),
        odom: PoseHistory::new(ODOM_KEPT),
        odom_heard: Arrival::new(),
        map_odom_heard: Arrival::new(),
        map_from_odom: None,
        mounts: HashMap::new(),
        camera_heard: Arrival::new(),
    };
    let worker = node.create_worker(core);
    // Held until exit: rclrs drops a subscription or timer with its handle.
    let mut keep: Vec<Box<dyn Any>> = vec![
        Box::new(
            worker.create_subscription(odom_topic.as_str(), |s: &mut Core, msg: Odometry| {
                s.on_odom(&msg)
            })?,
        ),
        Box::new(worker.create_subscription("/tf", |s: &mut Core, msg: TFMessage| s.on_tf(&msg))?),
        Box::new(worker.create_subscription(
            "/tf_static".keep_last(100).reliable().transient_local(),
            |s: &mut Core, msg: TFMessage| s.on_tf_static(&msg),
        )?),
    ];
    if wants_map {
        keep.push(Box::new(worker.create_subscription(
            map_topic.as_str().keep_last(1).reliable().transient_local(),
            |s: &mut Core, msg: OccupancyGrid| s.on_map(msg),
        )?));
    }
    if lanes {
        keep.push(Box::new(worker.create_subscription(
            route_topic.as_str(),
            |s: &mut Core, msg: Path| s.on_route(&msg),
        )?));
    }
    if let Some(topic) = camera_topic {
        keep.push(Box::new(worker.create_subscription(
            topic.as_str(),
            |s: &mut Core, msg: LaneGeometry| s.on_lanes(&msg),
        )?));
    }
    if chunks {
        keep.push(Box::new(worker.create_subscription(
            ahead_topic.as_str(),
            |s: &mut Core, msg: Path| s.on_ahead(&msg),
        )?));
    }
    for (topic, kind) in topics {
        let name = topic.clone();
        match kind {
            Kind::Scan => keep.push(Box::new(worker.create_subscription(
                topic.as_str().best_effort(),
                move |s: &mut Core, msg: LaserScan| s.on_scan(&name, &msg),
            )?)),
            Kind::Cloud => keep.push(Box::new(worker.create_subscription(
                topic.as_str().best_effort(),
                move |s: &mut Core, msg: PointCloud2| s.on_cloud(&name, &msg),
            )?)),
        }
    }
    for (i, (hz, name)) in rates.into_iter().zip(names).enumerate() {
        let period = Duration::from_secs_f64(1.0 / hz.max(0.1));
        keep.push(Box::new(worker.create_timer_repeating(
            period.node_time(),
            move |s: &mut Core| s.tick(i),
        )?));
        // Nav2's ClearEntireCostmap, for the behaviour trees' recovery (the launch file
        // points bt_navigator's local clear here when mppi_lag reads ours).
        let service = format!("{clear_prefix}{name}");
        keep.push(Box::new(worker.create_service::<ClearEntireCostmap, _>(
            service.as_str(),
            move |s: &mut Core, _: ClearEntireCostmap_Request| {
                s.clear(i);
                ClearEntireCostmap_Response::default()
            },
        )?));
    }

    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    drop((keep, handles));
    // A costmap's thread only ends on a panic while the node runs (its inbox stays open).
    if let Some(t) = threads.iter().find(|t: &&JoinHandle<()>| t.is_finished()) {
        bail!(
            "{} stopped",
            t.thread().name().unwrap_or("a costmap's thread")
        );
    }
    Ok(())
}

/// A source's message type.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Kind {
    Scan,
    Cloud,
}

/// Where a source's messages come from.
#[derive(Debug, Clone)]
struct Input {
    topic: String,
    kind: Kind,
    /// s: messages older than the newest by more than this are left out (0: newest only).
    persistence: f64,
    /// A scan's inf rays count (its source's `inf_is_valid`).
    inf_is_valid: bool,
    /// rad: a cloud's horizontal view (ours, `view_angle`; 0: none). Where none of its
    /// points lies, nothing is there: those rays clear (obstacle::view_points).
    view_angle: f64,
    /// m: those rays' length, past the marking range so they never mark.
    view_range: f64,
}

/// One message from a sensor, in its own frame.
struct Raw {
    stamp: f64,
    frame: String,
    points: Vec<[f64; 3]>,
    /// A scan's inf rays, as points just short of its range; none for a cloud.
    inf_points: Vec<[f64; 3]>,
}

/// One costmap, as its block in the parameters describes it.
struct Spec {
    name: &'static str,
    frame: String,
    costmap: Costmap,
    /// inputs[layer][source].
    inputs: Vec<Vec<Input>>,
    update_hz: f64,
    publish_hz: f64,
    /// s: odometry (and map -> odom, for a map-frame costmap) older than this stops the
    /// updates; a scan further than this from any odometry is left out (Nav2's
    /// `transform_tolerance`, 0.3 s by default).
    transform_tolerance: f64,
    /// The camera's lanes (a lanes block with source camera or both).
    camera: Option<CameraInput>,
}

/// Where the camera's lines come from, and which to trust.
#[derive(Debug, Clone)]
struct CameraInput {
    topic: String,
    /// A line below this confidence (0-1) isn't drawn.
    min_confidence: f64,
    /// s: lines older than this (this node's clock) are forgotten.
    max_age: f64,
}

fn num(v: &Value, key: &str) -> Result<f64> {
    v[key]
        .as_f64()
        .ok_or_else(|| anyhow!("{key}: a number is needed"))
}

fn flag(v: &Value, key: &str) -> Result<bool> {
    v[key]
        .as_bool()
        .ok_or_else(|| anyhow!("{key}: true or false is needed"))
}

fn text(v: &Value, key: &str) -> Result<String> {
    v[key]
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| anyhow!("{key}: text is needed"))
}

/// The lanes block: {"roads": [{name, points: [[x, y]...], width, lanes, oneway,
/// edge_inset}], "ramp": {line, wrong_lane, kerb, off_road, off_road_band}} (the ramp and
/// each of its keys optional: costmap::Ramp's defaults).
fn lanes_from(v: &Value) -> Result<Lanes> {
    Ok(Lanes::new(roads_from(v)?, ramp_from(v)?)?)
}

/// The block's roads.
fn roads_from(v: &Value) -> Result<Vec<Road>> {
    let roads = v["roads"]
        .as_array()
        .ok_or_else(|| anyhow!("roads: a list is needed"))?
        .iter()
        .map(|r| {
            let points = r["points"]
                .as_array()
                .and_then(|ps| {
                    ps.iter()
                        .map(|p| Some((p[0].as_f64()?, p[1].as_f64()?)))
                        .collect()
                })
                .ok_or_else(|| anyhow!("points: a list of [x, y] is needed"))?;
            Ok(Road {
                name: r["name"].as_str().unwrap_or_default().to_owned(),
                points,
                width: num(r, "width")?,
                lanes: r["lanes"].as_u64().unwrap_or(0) as u32,
                oneway: r["oneway"].as_bool().unwrap_or(false),
                edge_inset: r["edge_inset"]
                    .as_f64()
                    .unwrap_or(costmap::lanes::EDGE_INSET),
            })
        })
        .collect::<Result<Vec<_>>>()?;
    Ok(roads)
}

/// The block's ramp (each key optional: costmap::Ramp's defaults).
fn ramp_from(v: &Value) -> Result<Ramp> {
    let d = Ramp::default();
    let r = &v["ramp"];
    let cost = |key: &str, default: u8| -> Result<u8> {
        match r[key].as_u64() {
            None => Ok(default),
            Some(c) => u8::try_from(c).map_err(|_| anyhow!("ramp {key}: 0 to 252")),
        }
    };
    let ramp = Ramp {
        line: cost("line", d.line)?,
        wrong_lane: cost("wrong_lane", d.wrong_lane)?,
        kerb: cost("kerb", d.kerb)?,
        off_road: cost("off_road", d.off_road)?,
        off_road_band: r["off_road_band"].as_f64().unwrap_or(d.off_road_band),
    };
    Ok(ramp)
}

/// Which lanes a lanes block draws: (the map's roads, the camera's lines).
fn lane_sources(v: &Value) -> Result<(bool, bool)> {
    match v["source"].as_str().unwrap_or("map") {
        "map" => Ok((true, false)),
        "camera" => Ok((false, true)),
        "both" => Ok((true, true)),
        other => bail!("source {other:?}: map, camera or both"),
    }
}

/// The block's "camera": the camera's lanes and where their lines come from (each key
/// optional).
fn camera_from(v: &Value) -> Result<(CameraLanes, CameraInput)> {
    let c = &v["camera"];
    let d = CameraSettings::default();
    let settings = CameraSettings {
        behind: c["behind"].as_f64().unwrap_or(d.behind),
        ahead: c["ahead"].as_f64().unwrap_or(d.ahead),
        edge_inset: c["edge_inset"].as_f64().unwrap_or(d.edge_inset),
        min_width: c["min_width"].as_f64().unwrap_or(d.min_width),
        max_width: c["max_width"].as_f64().unwrap_or(d.max_width),
    };
    let input = CameraInput {
        topic: c["topic"]
            .as_str()
            .unwrap_or("/perception/lane/geometry")
            .to_owned(),
        min_confidence: c["min_confidence"].as_f64().unwrap_or(0.3),
        max_age: c["max_age"].as_f64().unwrap_or(0.5),
    };
    Ok((CameraLanes::new(ramp_from(v)?, settings)?, input))
}

impl Spec {
    /// From vehicle.costmap_params' block.
    fn build(name: &'static str, spec: &Value) -> Result<Self> {
        let outline: Vec<(f64, f64)> = spec["footprint"]
            .as_array()
            .and_then(|corners| {
                corners
                    .iter()
                    .map(|p| Some((p[0].as_f64()?, p[1].as_f64()?)))
                    .collect()
            })
            .ok_or_else(|| anyhow!("footprint: a list of [x, y] is needed"))?;
        let inflation = match &spec["inflation"] {
            Value::Null => None,
            i => Some((num(i, "radius")?, num(i, "scaling")?)),
        };
        let settings = Settings {
            rolling: flag(spec, "rolling")?,
            width: num(spec, "width")?,
            height: num(spec, "height")?,
            resolution: num(spec, "resolution")?,
            track_unknown: flag(spec, "track_unknown")?,
            footprint: footprint::padded(&outline, num(spec, "footprint_padding")?),
            inflation,
            static_layer: flag(spec, "static_layer")?,
        };
        let probe = Costmap::new(settings.clone(), Vec::new());
        let mut layers = Vec::new();
        let mut inputs = Vec::new();
        for l in spec["layers"].as_array().into_iter().flatten() {
            let mut sources = Vec::new();
            let mut ins = Vec::new();
            for s in l["sources"].as_array().into_iter().flatten() {
                let kind = match text(s, "data_type")?.as_str() {
                    "LaserScan" => Kind::Scan,
                    "PointCloud2" => Kind::Cloud,
                    other => bail!("data_type {other}: LaserScan or PointCloud2 only"),
                };
                ins.push(Input {
                    topic: text(s, "topic")?,
                    kind,
                    persistence: num(s, "observation_persistence")?,
                    inf_is_valid: flag(s, "inf_is_valid")?,
                    view_angle: num(s, "view_angle")?,
                    view_range: num(s, "obstacle_max_range")?.max(num(s, "raytrace_max_range")?)
                        + 0.1,
                });
                sources.push(Source {
                    name: text(s, "name")?,
                    marking: flag(s, "marking")?,
                    clearing: flag(s, "clearing")?,
                    min_obstacle_height: num(s, "min_obstacle_height")?,
                    max_obstacle_height: num(s, "max_obstacle_height")?,
                    obstacle_min_range: num(s, "obstacle_min_range")?,
                    obstacle_max_range: num(s, "obstacle_max_range")?,
                    raytrace_min_range: num(s, "raytrace_min_range")?,
                    raytrace_max_range: num(s, "raytrace_max_range")?,
                    inf_is_valid: flag(s, "inf_is_valid")?,
                });
            }
            let mut layer = ObstacleLayer::new(
                &text(l, "name")?,
                sources,
                probe.grid(),
                settings.track_unknown,
            );
            layer.min_obstacle_height = num(l, "min_obstacle_height")?;
            layer.max_obstacle_height = num(l, "max_obstacle_height")?;
            layer.footprint_clearing = flag(l, "footprint_clearing")?;
            layers.push(layer);
            inputs.push(ins);
        }
        let mut costmap = Costmap::new(settings, layers);
        let mut camera = None;
        match &spec["lanes"] {
            Value::Null => {}
            l => {
                let (map, seen) = lane_sources(l).context("lanes")?;
                if map {
                    costmap.set_lanes(lanes_from(l).context("lanes")?);
                }
                if seen {
                    let (lanes, input) = camera_from(l).context("lanes camera")?;
                    costmap.set_camera_lanes(lanes);
                    // A rolling costmap draws no lanes: it needs no lines.
                    camera = costmap.camera_lanes().map(|_| input);
                }
            }
        }
        match &spec["chunks"] {
            Value::Null => {}
            c => {
                let dir = text(c, "dir")?;
                let source = ChunkSource::open(
                    std::path::Path::new(&dir),
                    num(c, "ahead")?,
                    num(c, "margin")? as i32,
                )
                .with_context(|| format!("chunks: {dir}"))?;
                costmap
                    .set_chunks(source)
                    .with_context(|| format!("chunks: {dir}"))?;
            }
        }
        Ok(Spec {
            name,
            frame: text(spec, "global_frame")?,
            costmap,
            inputs,
            update_hz: num(spec, "update_frequency")?,
            publish_hz: num(spec, "publish_frequency")?,
            transform_tolerance: spec["transform_tolerance"].as_f64().unwrap_or(0.3),
            camera,
        })
    }

    fn describe(&self) -> String {
        let s = self.costmap.settings();
        let size = if s.rolling && self.costmap.chunked() {
            format!(
                "a {} m x {} m window of {} m cells, its map from the 64 m chunks round the car",
                s.width, s.height, s.resolution
            )
        } else if s.rolling {
            format!(
                "a {} m x {} m window of {} m cells",
                s.width, s.height, s.resolution
            )
        } else if self.costmap.chunked() {
            "the map's 64 m chunks round the car and along the route ahead".to_owned()
        } else {
            "the map's size and cells".to_owned()
        };
        let layers: Vec<&str> = self
            .costmap
            .layers()
            .iter()
            .map(|l| l.name.as_str())
            .collect();
        let lanes = self.costmap.lanes().map_or_else(
            || "off".to_owned(),
            |l| format!("{} roads, {:?}", l.roads().len(), l.ramp()),
        );
        let lanes = match &self.camera {
            Some(c) => format!(
                "{lanes}; from the camera ({}, confidence {} and up, {} s at most)",
                c.topic, c.min_confidence, c.max_age
            ),
            None => lanes,
        };
        format!(
            "{} costmap in {}: {size}; static map {}; obstacle layers [{}]; inflation {:?}; \
             lanes {lanes}; update {} Hz, publish {} Hz",
            self.name,
            self.frame,
            s.static_layer,
            layers.join(", "),
            s.inflation,
            self.update_hz,
            self.publish_hz
        )
    }
}

/// What the worker hands a costmap's thread.
enum Job {
    /// A new map: each thread reads it into its own costs.
    Map(Arc<MapData>),
    /// A recovery asked: forget every mark.
    Clear,
    /// A new route (map frame), for the lanes layer.
    Route(Arc<Vec<(f64, f64)>>),
    /// The route level's whole way (map frame), for the chunks ahead.
    Ahead(Arc<Vec<(f64, f64)>>),
    /// The camera's lines and base_link in map at their stamp; None: forget them.
    Lanes(Option<(Seen, Pose2)>),
    /// The timer: update now with these inputs, or say why it can't.
    Tick(Tick),
}

enum Tick {
    Update(Inputs),
    Stalled(String),
}

/// One update's inputs, gathered on the worker.
struct Inputs {
    /// The car in the costmap's frame.
    robot: Pose2,
    /// The costmap's frame in map, if known.
    map_from_frame: Option<Pose2>,
    /// observations[layer][source]: each a list.
    observations: Vec<Vec<Vec<Observation>>>,
    /// The inputs that were missing (empty: none).
    waiting: String,
}

/// /map, as it came.
struct MapData {
    origin: (f64, f64),
    resolution: f64,
    width: usize,
    height: usize,
    values: Vec<i8>,
}

/// One costmap as the worker sees it: what to gather for it, and its thread's inbox.
struct Handle {
    name: &'static str,
    /// Its frame is map (else odom).
    in_map: bool,
    /// It has a lanes layer: it takes the routes.
    lanes: bool,
    /// It draws the camera's lanes: it takes the lines.
    camera: Option<CameraInput>,
    /// The camera's lines are drawn (not yet forgotten).
    camera_shown: bool,
    /// It is made of chunks: it takes the route ahead, not /map.
    chunks: bool,
    inputs: Vec<Vec<Input>>,
    update_hz: f64,
    transform_tolerance: f64,
    jobs: Sender<Job>,
}

/// One costmap on its own thread: its layers, its messages, how its updates went.
struct One {
    spec: Spec,
    last_publish: Option<Instant>,
    update_ms: f64,
    worst_ms: f64,
    /// ms: the last publish (the messages made and sent).
    publish_ms: f64,
    updates: u64,
    /// Inputs left out because the thread was still busy with older ones.
    skipped: u64,
    /// How often a recovery cleared it.
    clears: u64,
    /// The routes that changed the lanes' costs.
    lane_routes: u64,
    /// Made of chunks: how many times the window moved, and the last move (chunks read,
    /// dropped and redone; ms reading and stitching: the next update's redo is in
    /// update_ms).
    chunk_moves: u64,
    chunk_last: String,
    waiting: String,
    /// The grid changed since the messages were last made.
    dirty: bool,
    /// The grid's lethal cells, counted when the messages were made.
    lethal: usize,
    raw_msg: CostmapMsg,
    grid_msg: OccupancyGrid,
    grid_pub: Publisher<OccupancyGrid>,
    raw_pub: Publisher<CostmapMsg>,
    status: Publisher<DiagnosticStatus>,
    clock: Clock,
    logger: Logger,
}

impl One {
    fn new(
        spec: Spec,
        grid_pub: Publisher<OccupancyGrid>,
        raw_pub: Publisher<CostmapMsg>,
        status: Publisher<DiagnosticStatus>,
        clock: Clock,
        logger: Logger,
    ) -> Self {
        let header = Header {
            stamp: Time::default(),
            frame_id: spec.frame.clone(),
        };
        Self {
            last_publish: None,
            update_ms: 0.0,
            worst_ms: 0.0,
            publish_ms: 0.0,
            updates: 0,
            skipped: 0,
            clears: 0,
            lane_routes: 0,
            chunk_moves: 0,
            chunk_last: String::new(),
            waiting: String::new(),
            dirty: true,
            lethal: 0,
            raw_msg: CostmapMsg {
                header: header.clone(),
                ..CostmapMsg::default()
            },
            grid_msg: OccupancyGrid {
                header,
                ..OccupancyGrid::default()
            },
            spec,
            grid_pub,
            raw_pub,
            status,
            clock,
            logger,
        }
    }

    /// Take jobs until the node ends. When it falls behind, the maps and clears are all
    /// done, in order, and only the newest tick.
    fn run(mut self, inbox: &Receiver<Job>) {
        while let Ok(first) = inbox.recv() {
            let mut tick = None;
            for job in std::iter::once(first).chain(inbox.try_iter()) {
                match job {
                    Job::Map(map) => self.set_map(&map),
                    Job::Clear => self.clear(),
                    Job::Route(route) => {
                        if self.spec.costmap.set_route(&route) {
                            self.lane_routes += 1;
                        }
                    }
                    Job::Ahead(route) => self.spec.costmap.set_ahead(&route),
                    Job::Lanes(Some((seen, car))) => {
                        self.spec.costmap.see_lanes(Some(&seen), &car);
                    }
                    Job::Lanes(None) => {
                        self.spec.costmap.see_lanes(None, &Pose2::IDENTITY);
                    }
                    Job::Tick(t) => {
                        if let Some(Tick::Update(_)) = tick.replace(t) {
                            self.skipped += 1;
                        }
                    }
                }
            }
            match tick {
                Some(Tick::Update(inputs)) => self.update(inputs),
                Some(Tick::Stalled(why)) => self.stalled(why),
                None => {}
            }
        }
    }

    fn stamp(&self) -> Time {
        self.clock.now().to_ros_msg().unwrap_or_default()
    }

    fn set_map(&mut self, m: &MapData) {
        let track = self.spec.costmap.settings().track_unknown;
        match StaticMap::from_occupancy(m.origin, m.resolution, m.width, m.height, &m.values, track)
        {
            Some(map) => self.spec.costmap.set_map(map),
            None => log_warn!(&self.logger, "the map's size and data disagree: ignored"),
        }
    }

    /// A recovery asked: forget the marks (the next update recomputes every cell).
    fn clear(&mut self) {
        self.spec.costmap.clear();
        self.clears += 1;
        log_info!(&self.logger, "{} costmap cleared (asked)", self.spec.name);
    }

    fn due(&self) -> bool {
        self.last_publish.is_none_or(|t| {
            t.elapsed().as_secs_f64() >= 1.0 / self.spec.publish_hz.max(0.01) - 1e-3
        })
    }

    /// One update, and its publish when due.
    fn update(&mut self, inputs: Inputs) {
        // The car in the map frame: the local costmap's robot is in odom.
        let car = if self.spec.costmap.settings().rolling {
            inputs
                .map_from_frame
                .as_ref()
                .map(|t| t.apply((inputs.robot.x, inputs.robot.y)))
        } else {
            Some((inputs.robot.x, inputs.robot.y))
        };
        if let Some(car) = car.filter(|_| self.spec.costmap.chunked()) {
            let started = Instant::now();
            match self.spec.costmap.follow_chunks(car) {
                Ok(None) => {}
                Ok(Some(moved)) => {
                    let w = moved.window;
                    self.chunk_moves += 1;
                    self.chunk_last = format!(
                        "{}x{} chunks from ({}, {}); read {}, dropped {}, redo {}; {:.1} ms",
                        w.across(),
                        w.up(),
                        w.i0,
                        w.j0,
                        moved.changes.loaded.len(),
                        moved.changes.dropped.len(),
                        moved.redo,
                        started.elapsed().as_secs_f64() * 1e3
                    );
                    log_info!(
                        &self.logger,
                        "{} costmap: {}",
                        self.spec.name,
                        self.chunk_last
                    );
                }
                Err(e) => return self.stalled(format!("the map's chunks: {e}")),
            }
        }
        if !self.spec.costmap.ready() {
            return self.stalled("the map".to_owned());
        }
        let started = Instant::now();
        let observations: Vec<Vec<Vec<&Observation>>> = inputs
            .observations
            .iter()
            .map(|l| l.iter().map(|s| s.iter().collect()).collect())
            .collect();
        let changed =
            self.spec
                .costmap
                .update(&inputs.robot, inputs.map_from_frame.as_ref(), &observations);
        self.dirty |= changed != Changed::Nothing;
        let ms = started.elapsed().as_secs_f64() * 1e3;
        self.update_ms = ms;
        self.worst_ms = self.worst_ms.max(ms);
        self.updates += 1;
        self.waiting = inputs.waiting;
        if self.due() {
            self.last_publish = Some(Instant::now());
            if let Err(e) = self.publish() {
                log_warn!(
                    self.logger.throttle(Duration::from_secs(5)),
                    "publishing the {} costmap: {e}",
                    self.spec.name
                );
            }
        }
    }

    /// It can't update: say why on its status, at its publish rate; no grid.
    fn stalled(&mut self, why: String) {
        self.waiting = why;
        if self.due() {
            self.last_publish = Some(Instant::now());
            if let Err(e) = self.publish_status() {
                log_warn!(
                    self.logger.throttle(Duration::from_secs(5)),
                    "publishing the {} costmap's status: {e}",
                    self.spec.name
                );
            }
        }
    }

    /// The two messages' cells, sizes and corner, from the grid: only after it changed
    /// (on the campus, 41 million cells a message).
    fn refresh(&mut self) {
        let g = self.spec.costmap.grid();
        let origin = PoseMsg {
            position: Point {
                x: g.origin_x,
                y: g.origin_y,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        };
        self.raw_msg.metadata = CostmapMetaData {
            layer: "master".to_owned(),
            resolution: g.resolution as f32,
            size_x: g.width as u32,
            size_y: g.height as u32,
            origin: origin.clone(),
            ..CostmapMetaData::default()
        };
        self.raw_msg.data.clone_from(&g.data);
        self.grid_msg.info = MapMetaData {
            resolution: g.resolution as f32,
            width: g.width as u32,
            height: g.height as u32,
            origin,
            ..MapMetaData::default()
        };
        self.grid_msg.data.clear();
        self.grid_msg
            .data
            .extend(g.data.iter().map(|&c| occupancy(c)));
        self.lethal = g.data.iter().filter(|&&c| c == LETHAL).count();
        self.dirty = false;
    }

    /// Both grids, stamped now, every publish period: the readers check their age (the
    /// planner, mppi_lag), so an unchanged grid is sent again, its cells not made again.
    fn publish(&mut self) -> Result<()> {
        let started = Instant::now();
        if self.dirty {
            self.refresh();
        }
        let stamp = self.stamp();
        self.raw_msg.header.stamp = stamp.clone();
        self.raw_msg.metadata.map_load_time = stamp.clone();
        self.raw_msg.metadata.update_time = stamp.clone();
        self.grid_msg.header.stamp = stamp.clone();
        self.grid_msg.info.map_load_time = stamp;
        self.raw_pub.publish(&self.raw_msg)?;
        self.grid_pub.publish(&self.grid_msg)?;
        self.publish_ms = started.elapsed().as_secs_f64() * 1e3;
        self.publish_status()
    }

    /// Its status line: its update times, its lethal cells, what it waits for.
    fn publish_status(&self) -> Result<()> {
        let g = self.spec.costmap.grid();
        let name = self.spec.name;
        let values = vec![
            kv("costmap", name),
            kv("update_ms", &format!("{:.2}", self.update_ms)),
            kv("worst_ms", &format!("{:.2}", self.worst_ms)),
            kv("publish_ms", &format!("{:.2}", self.publish_ms)),
            kv("updates", &self.updates.to_string()),
            kv("skipped", &self.skipped.to_string()),
            kv("clears", &self.clears.to_string()),
            kv("lane_routes", &self.lane_routes.to_string()),
            kv("chunk_moves", &self.chunk_moves.to_string()),
            kv("chunks", &self.chunk_last),
            kv("lethal_cells", &self.lethal.to_string()),
            kv("cells", &format!("{}x{}", g.width, g.height)),
            kv("waiting_for", &self.waiting),
        ];
        self.status.publish(DiagnosticStatus {
            level: if self.waiting.is_empty() { 0 } else { 1 },
            name: format!("costmap {name}"),
            message: if self.waiting.is_empty() {
                format!("{name} costmap: {:.2} ms an update", self.update_ms)
            } else {
                format!("{name} costmap: waiting for {}", self.waiting)
            },
            hardware_id: String::new(),
            values,
        })?;
        Ok(())
    }
}

struct Core {
    maps: Vec<Handle>,
    /// Stops the node if a costmap's thread has gone (a panic), so the launch restarts it.
    halt: Arc<ExecutorCommands>,
    clock: Clock,
    logger: Logger,
    base_frame: String,
    /// Each sensor topic's newest messages, newest last.
    raw: HashMap<String, VecDeque<Raw>>,
    /// base_link in odom, by stamp.
    odom: PoseHistory,
    /// When the odometry last arrived (this node's clock).
    odom_heard: Arrival,
    /// odom's pose in map (the map -> odom transform).
    map_from_odom: Option<Pose2>,
    map_odom_heard: Arrival,
    /// Each fixed frame's parent and its mount in it (/tf_static).
    mounts: HashMap<String, (String, Transform3)>,
    /// When the camera's lines last came (this node's clock).
    camera_heard: Arrival,
}

/// The lines a LaneGeometry trusts: detected, and at least `min_confidence` sure.
fn seen_of(msg: &LaneGeometry, min_confidence: f64) -> Seen {
    let line = |detected: bool, confidence: f32, a: f32, b: f32, c: f32, paint: u8| {
        (detected && f64::from(confidence) >= min_confidence).then(|| SeenLine {
            a: f64::from(a),
            b: f64::from(b),
            c: f64::from(c),
            paint: Paint::from_code(paint),
        })
    };
    Seen {
        left: line(
            msg.left_lane_detected,
            msg.left_confidence,
            msg.left_poly_a,
            msg.left_poly_b,
            msg.left_poly_c,
            msg.left_paint,
        ),
        right: line(
            msg.right_lane_detected,
            msg.right_confidence,
            msg.right_poly_a,
            msg.right_poly_b,
            msg.right_poly_c,
            msg.right_paint,
        ),
        width: f64::from(msg.lane_width_m),
    }
}

fn seconds(t: &Time) -> f64 {
    f64::from(t.sec) + f64::from(t.nanosec) * 1e-9
}

fn yaw_of(q: &Quaternion) -> f64 {
    (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
}

fn bare(frame: &str) -> &str {
    frame.trim_start_matches('/')
}

impl Core {
    /// Hand costmap i's thread a job; if the thread has gone, stop the node.
    fn send(&self, i: usize, job: Job) {
        let one = &self.maps[i];
        if one.jobs.send(job).is_err() {
            log_error!(
                &self.logger,
                "the {} costmap's thread has stopped: the node stops",
                one.name
            );
            self.halt.halt_spinning();
        }
    }

    /// A recovery asked: costmap i forgets its marks, before its next update.
    fn clear(&mut self, i: usize) {
        self.send(i, Job::Clear);
    }

    /// The map, to every costmap's thread: each reads it there, not on this worker.
    fn on_map(&mut self, msg: OccupancyGrid) {
        let i = &msg.info;
        let (origin, resolution) = (
            (i.origin.position.x, i.origin.position.y),
            f64::from(i.resolution),
        );
        let (width, height) = (i.width as usize, i.height as usize);
        let map = Arc::new(MapData {
            origin,
            resolution,
            width,
            height,
            values: msg.data,
        });
        for k in 0..self.maps.len() {
            self.send(k, Job::Map(Arc::clone(&map)));
        }
    }

    /// A new route: to each costmap with lanes. Only a route in map is drawn.
    fn on_route(&mut self, msg: &Path) {
        if bare(&msg.header.frame_id) != "map" {
            log_warn!(
                self.logger.throttle(Duration::from_secs(10)),
                "a route in {:?}, not map: the lanes ignore it",
                msg.header.frame_id
            );
            return;
        }
        let route: Arc<Vec<(f64, f64)>> = Arc::new(
            msg.poses
                .iter()
                .map(|p| (p.pose.position.x, p.pose.position.y))
                .filter(|p| p.0.is_finite() && p.1.is_finite())
                .collect(),
        );
        for k in 0..self.maps.len() {
            if self.maps[k].lanes {
                self.send(k, Job::Route(Arc::clone(&route)));
            }
        }
    }

    /// The camera's lines: to each costmap that draws them, placed at the picture's stamp
    /// (map -> odom, then the odometry then). Lines it can't place are left out; old
    /// ones are forgotten in `tick`.
    fn on_lanes(&mut self, msg: &LaneGeometry) {
        let now = self.now();
        let t = seconds(&msg.header.stamp);
        let frame = bare(&msg.header.frame_id);
        if !frame.is_empty() && frame != self.base_frame {
            log_warn!(
                self.logger.throttle(Duration::from_secs(10)),
                "lane lines in {frame:?}, not {}: ignored",
                self.base_frame
            );
            return;
        }
        for k in 0..self.maps.len() {
            let Some(input) = self.maps[k].camera.clone() else {
                continue;
            };
            let tolerance = self.maps[k].transform_tolerance;
            let car = self
                .odom
                .at(t, tolerance)
                .and_then(|p| self.map_from_odom.map(|m| m.then(&p)));
            let seen = seen_of(msg, input.min_confidence);
            let Some(car) = car.filter(|_| seen.left.is_some() || seen.right.is_some()) else {
                continue;
            };
            self.camera_heard.mark(now);
            self.maps[k].camera_shown = true;
            self.send(k, Job::Lanes(Some((seen, car))));
        }
    }

    /// The route level's whole way: to each costmap made of chunks.
    fn on_ahead(&mut self, msg: &Path) {
        if bare(&msg.header.frame_id) != "map" {
            return;
        }
        let route: Arc<Vec<(f64, f64)>> = Arc::new(
            msg.poses
                .iter()
                .map(|p| (p.pose.position.x, p.pose.position.y))
                .filter(|p| p.0.is_finite() && p.1.is_finite())
                .collect(),
        );
        for k in 0..self.maps.len() {
            if self.maps[k].chunks {
                self.send(k, Job::Ahead(Arc::clone(&route)));
            }
        }
    }

    fn on_odom(&mut self, msg: &Odometry) {
        let p = &msg.pose.pose;
        let pose = Pose2::new(p.position.x, p.position.y, yaw_of(&p.orientation));
        let t = seconds(&msg.header.stamp);
        if !(pose.x.is_finite() && pose.y.is_finite() && pose.yaw.is_finite()) {
            return;
        }
        self.odom.push(t, pose);
        self.odom_heard.mark(self.now());
    }

    fn now(&self) -> f64 {
        self.clock.now().nsec as f64 * 1e-9
    }

    fn on_tf(&mut self, msg: &TFMessage) {
        for t in &msg.transforms {
            if bare(&t.header.frame_id) == "map" && bare(&t.child_frame_id) == "odom" {
                let r = &t.transform;
                self.map_from_odom = Some(Pose2::new(
                    r.translation.x,
                    r.translation.y,
                    yaw_of(&r.rotation),
                ));
                self.map_odom_heard.mark(self.now());
            }
        }
    }

    fn on_tf_static(&mut self, msg: &TFMessage) {
        for t in &msg.transforms {
            let r = &t.transform;
            self.mounts.insert(
                bare(&t.child_frame_id).to_owned(),
                (
                    bare(&t.header.frame_id).to_owned(),
                    Transform3 {
                        translation: [r.translation.x, r.translation.y, r.translation.z],
                        rotation: [r.rotation.x, r.rotation.y, r.rotation.z, r.rotation.w],
                    },
                ),
            );
        }
    }

    /// A frame's mount in base_link, through /tf_static's chain.
    fn mount(&self, frame: &str) -> Option<Transform3> {
        let mut acc = Transform3::IDENTITY;
        let mut f = bare(frame);
        for _ in 0..16 {
            if f == self.base_frame {
                return Some(acc);
            }
            let (parent, t) = self.mounts.get(f)?;
            acc = t.then(&acc);
            f = parent;
        }
        None
    }

    fn keep_raw(&mut self, topic: &str, raw: Raw) {
        let list = self.raw.entry(topic.to_owned()).or_default();
        list.push_back(raw);
        while list.len() > KEPT_PER_TOPIC {
            list.pop_front();
        }
    }

    fn on_scan(&mut self, topic: &str, msg: &LaserScan) {
        let points = |ranges: &[f32], inf_is_valid| {
            costmap::obstacle::scan_points(
                f64::from(msg.angle_min),
                f64::from(msg.angle_increment),
                f64::from(msg.range_min),
                f64::from(msg.range_max),
                ranges,
                inf_is_valid,
            )
        };
        // The inf rays apart: only a source with inf_is_valid takes them.
        let infs: Vec<f32> = msg
            .ranges
            .iter()
            .map(|&r| if r == f32::INFINITY { r } else { f32::NAN })
            .collect();
        let raw = Raw {
            stamp: seconds(&msg.header.stamp),
            frame: msg.header.frame_id.clone(),
            points: points(&msg.ranges, false),
            inf_points: points(&infs, true),
        };
        self.keep_raw(topic, raw);
    }

    fn on_cloud(&mut self, topic: &str, msg: &PointCloud2) {
        let field = |name: &str| {
            msg.fields
                .iter()
                .find(|f| f.name == name && f.datatype == 7) // FLOAT32
                .map(|f| f.offset as usize)
        };
        let (Some(x), Some(y), Some(z)) = (field("x"), field("y"), field("z")) else {
            log_warn!(
                self.logger.throttle(Duration::from_secs(10)),
                "{topic}: no float32 x, y, z: ignored"
            );
            return;
        };
        let step = msg.point_step as usize;
        let read = |p: &[u8], at: usize| -> Option<f64> {
            let b: [u8; 4] = p.get(at..at + 4)?.try_into().ok()?;
            let v = if msg.is_bigendian {
                f32::from_be_bytes(b)
            } else {
                f32::from_le_bytes(b)
            };
            Some(f64::from(v))
        };
        let points = if step == 0 {
            Vec::new()
        } else {
            msg.data
                .chunks_exact(step)
                .filter_map(|p| Some([read(p, x)?, read(p, y)?, read(p, z)?]))
                .filter(|p| p.iter().all(|v| v.is_finite()))
                .collect()
        };
        let raw = Raw {
            stamp: seconds(&msg.header.stamp),
            frame: msg.header.frame_id.clone(),
            points,
            inf_points: Vec::new(),
        };
        self.keep_raw(topic, raw);
    }

    /// Costmap i's timer: its inputs gathered here, its update and publish on its thread.
    fn tick(&mut self, i: usize) {
        let in_map = self.maps[i].in_map;
        let now = self.now();
        // The camera's lines too old (or none trusted since): forget them.
        if let Some(input) = &self.maps[i].camera {
            if self.maps[i].camera_shown && !self.camera_heard.fresh(now, input.max_age) {
                self.maps[i].camera_shown = false;
                self.send(i, Job::Lanes(None));
            }
        }
        let tolerance = self.maps[i].transform_tolerance;
        // Stale odometry (or map -> odom): no scan placed, nothing published, so a reader
        // checking the costmap's age stops too. The status says why.
        let Some((_, odom_pose)) = self.odom.latest() else {
            return self.stalled(i, "odometry".to_owned());
        };
        if !self.odom_heard.fresh(now, tolerance) {
            let age = self.odom_heard.age(now).unwrap_or(f64::INFINITY);
            return self.stalled(i, format!("odometry ({age:.2} s old)"));
        }
        let robot = if in_map {
            match self.map_from_odom {
                Some(_) if !self.map_odom_heard.fresh(now, tolerance) => {
                    let age = self.map_odom_heard.age(now).unwrap_or(f64::INFINITY);
                    return self.stalled(i, format!("map -> odom ({age:.2} s old)"));
                }
                Some(m) => m.then(&odom_pose),
                None => return self.stalled(i, "map -> odom".to_owned()),
            }
        } else {
            odom_pose
        };
        // Each source's observations, in this costmap's frame.
        let mut missing = Vec::new();
        let observations = self.maps[i]
            .inputs
            .iter()
            .map(|layer| {
                layer
                    .iter()
                    .map(|input| self.observations(input, in_map, tolerance, &mut missing))
                    .collect()
            })
            .collect();
        let map_from_frame = if in_map {
            Some(Pose2::IDENTITY)
        } else {
            self.map_from_odom
        };
        let inputs = Inputs {
            robot,
            map_from_frame,
            observations,
            waiting: missing.join(", "),
        };
        self.send(i, Job::Tick(Tick::Update(inputs)));
    }

    /// Costmap i can't update: its thread says why on its status; no grid.
    fn stalled(&mut self, i: usize, why: String) {
        self.send(i, Job::Tick(Tick::Stalled(why)));
    }

    /// A source's messages within its persistence, in this costmap's frame.
    fn observations(
        &self,
        input: &Input,
        in_map: bool,
        tolerance: f64,
        missing: &mut Vec<String>,
    ) -> Vec<Observation> {
        let Some(list) = self.raw.get(&input.topic).filter(|l| !l.is_empty()) else {
            missing.push(input.topic.clone());
            return Vec::new();
        };
        let newest = list.back().map_or(0.0, |r| r.stamp);
        let wanted: Vec<&Raw> = if input.persistence > 0.0 {
            list.iter()
                .filter(|r| newest - r.stamp <= input.persistence)
                .collect()
        } else {
            list.back().into_iter().collect()
        };
        let mut out = Vec::new();
        for raw in wanted {
            let Some(mount) = self.mount(&raw.frame) else {
                missing.push(format!("{} -> {}", self.base_frame, raw.frame));
                continue;
            };
            // tf2's rule: no odometry within the tolerance of the scan's stamp, no place.
            let Some(odom_pose) = self.odom.at(raw.stamp, tolerance) else {
                missing.push(format!("odometry at {}'s stamp", input.topic));
                continue;
            };
            let base = match (in_map, self.map_from_odom) {
                (false, _) => odom_pose,
                (true, Some(m)) => m.then(&odom_pose),
                (true, None) => continue,
            };
            let whole = Transform3::from_pose2(&base).then(&mount);
            let infs = if input.inf_is_valid {
                raw.inf_points.as_slice()
            } else {
                &[]
            };
            let view = if input.kind == Kind::Cloud {
                costmap::obstacle::view_points(&raw.points, input.view_angle, input.view_range)
            } else {
                Vec::new()
            };
            let points = raw
                .points
                .iter()
                .chain(infs)
                .chain(&view)
                .map(|&p| whole.apply(p))
                .collect();
            out.push(Observation {
                origin: whole.translation,
                points,
            });
        }
        out
    }
}

fn kv(key: &str, value: &str) -> KeyValue {
    KeyValue {
        key: key.to_owned(),
        value: value.to_owned(),
    }
}
