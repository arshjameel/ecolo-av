//! map_server: the map, as one latched OccupancyGrid, in Nav2's map_server's place (Nav2
//! deleted, 17 Nov, TODO 184). It reads a map server's `.yaml` and its `.pgm` as Nav2's
//! does in trinary mode (`map-chunks`' `read_map`: dark is taken, light free, between
//! unknown) and sends the whole map once, transient local, so a reader that starts later
//! still gets it.
//!
//! Subscribes  nothing.
//! Publishes   topic_name (/map)            nav_msgs/OccupancyGrid, reliable, transient
//!                                         local, keep last 1: the map, in frame_id
//! Serves      nothing.
//! Calls       nothing.
//! Parameters  yaml_filename (the map's .yaml: required), topic_name (/map),
//!             frame_id (map), as Nav2's map_server names them.
//!
//! navigation.launch.py starts it as map_server; with amcl_map:= a second one,
//! amcl_map_server, sends AMCL's own map.

use std::path::Path;
use std::sync::Arc;

use anyhow::{bail, Context as _, Result};
use map_chunks::files::read_map;
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::geometry_msgs::msg::{Point, Pose, Quaternion};
use ros_env::nav_msgs::msg::{MapMetaData, OccupancyGrid};
use ros_env::std_msgs::msg::Header;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("map_server")?;
    let text = |name: &str, default: &str| -> Result<String> {
        let p = node
            .declare_parameter::<Arc<str>>(name)
            .default(default.into())
            .read_only()
            .with_context(|| format!("parameter {name}"))?;
        Ok(p.get().to_string())
    };
    let yaml = text("yaml_filename", "")?;
    let topic = text("topic_name", "/map")?;
    let frame = text("frame_id", "map")?;
    if yaml.is_empty() {
        bail!("map_server needs yaml_filename (a map server's .yaml)");
    }
    let map = read_map(Path::new(&yaml)).with_context(|| format!("the map {yaml}"))?;
    let [x, y, yaw] = map.yaml.origin;
    let (sin, cos) = (yaw / 2.0).sin_cos();
    let width = u32::try_from(map.width).context("the map is too wide")?;
    let height = u32::try_from(map.height).context("the map is too tall")?;
    let now = node.get_clock().now();
    let stamp = Time {
        sec: i32::try_from(now.nsec.div_euclid(1_000_000_000)).unwrap_or(0),
        nanosec: u32::try_from(now.nsec.rem_euclid(1_000_000_000)).unwrap_or(0),
    };
    let grid = OccupancyGrid {
        header: Header {
            stamp: stamp.clone(),
            frame_id: frame.clone(),
        },
        info: MapMetaData {
            map_load_time: stamp,
            resolution: map.yaml.resolution as f32,
            width,
            height,
            origin: Pose {
                position: Point { x, y, z: 0.0 },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: sin,
                    w: cos,
                },
            },
        },
        data: map.data,
    };
    let publisher: Publisher<OccupancyGrid> =
        node.create_publisher(topic.as_str().keep_last(1).reliable().transient_local())?;
    publisher.publish(grid)?;
    log_info!(
        node.logger(),
        "{yaml}: {width} x {height} cells of {} m on {topic}, in {frame}",
        map.yaml.resolution
    );

    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
