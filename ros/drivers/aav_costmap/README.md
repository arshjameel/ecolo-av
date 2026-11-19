# aav_costmap

Our own costmaps, in Rust: Nav2's `costmap_2d` layers ported (`costmap:=own`).
The simulator's default since 14 Nov (`stack:=ours`); the only costmaps, sim and car, since Nav2 went on 17 Nov.
The page: [docs/own-costmap.md](../../../docs/own-costmap.md).

```
/map, the scan (and radar, cubes, cameras where they run), odometry, /tf_static
  -> costmap -> /drivers/nav/costmap/local (+ _raw)   odom, 20 m window, 5 Hz
             -> /drivers/nav/costmap/global (+ _raw)  map, the map's size, 1 Hz
  local_raw -> mppi_lag (tracker:=mppi_lag)
lane_costs:= a lanes.yaml, and /plan -> the lanes layer on the global one (a challenger)
lane_source:=camera|both, and /perception/lane/geometry -> the same costs from the camera's lines
costmap_chunks:= a chunks folder, and /drivers/nav/planner/route -> the global one holds
  only the 64 m chunks round the car and along the route ahead, not /map (TODO 175)
local_chunks:=true -> the local one stamps its window from the 3 x 3 chunks round the car too
```

## Overview

| Part | What it is |
|---|---|
| `crates/costmap` | the maths, no ROS: grid, static map, obstacle layers, inflation, the layered costmap (a fixed one in 64 m tiles, each on its own core: rayon); `lanes.rs`, the lanes layer (ours, TODO 148); `camera_lanes.rs`, the same costs from the camera's lines (TODO 264); `chunked.rs`, the global costmap from map chunks (ours, TODO 175) |
| `crates/costmap-node` | the ROS node, binary `costmap` (rclrs): messages in and out only |

```sh
pixi run sim costmap:=own tracker:=mppi_lag
pixi run sim-check --costmap own --tracker mppi_lag --lidar-obstacle --plan-around
```

## Interface

**`costmap`** (`crates/costmap-node/src/main.rs`), started by `aav_navigation`'s
`navigation.launch.py` (the only costmaps since Nav2's went, 17 Nov)

| Item | What |
|---|---|
| Subscribes | `/map` (transient local; not when every costmap is made of chunks); `/localization/odom/local`; `/tf` (map -> odom); `/tf_static` (sensor mounts); each layer's sources from nav2.yaml (`/sensors/lidar/scan`; the radar, cubes, camera points and ghost where they run), best effort; `/plan` (`route_topic`) only with a lanes block; `/perception/lane/geometry` (the block's `camera.topic`) only with a lanes block whose `source` is `camera` or `both`; `/drivers/nav/planner/route` (`ahead_topic`) only with a chunks block |
| Publishes | `/drivers/nav/costmap/local`, `/global` (OccupancyGrid); `/drivers/nav/costmap/local_raw`, `/global_raw` (nav2_msgs/Costmap); `/drivers/nav/costmap/status` (DiagnosticStatus) |
| Serves | `/drivers/nav/costmap/clear_entirely_local`, `_global` (nav2_msgs/srv/ClearEntireCostmap): forget every mark. The navigator calls them in its recoveries |
| Parameters | `local`, `global`: one JSON block each, made by `aav_description`'s `vehicle.costmap_params` from the tuned nav2.yaml; the global one may hold `lanes` (`lane_map.costmap_block`, from `navigation.launch.py lane_costs:=<lanes.yaml>`) and `chunks` (`vehicle.costmap_chunks_block`: `dir`, `ahead` 100 m, `margin` 1 chunk; from `costmap_chunks:=<folder>`); the local one may hold `chunks` too (`ahead` 0, `margin` 0: the 3 x 3 round the car, its static layer stamped from them; `local_chunks:=true`); `odom_topic`, `map_topic`, `base_frame`, `route_topic`, the topic names |

**`map_server`** (`crates/costmap-node/src/map_server.rs`), started by `navigation.launch.py`
in Nav2's `map_server`'s place (17 Nov, TODO 184; and as `amcl_map_server` for AMCL's map)

| Item | What |
|---|---|
| Publishes | `topic_name` (`/map`): the map YAML's grid (`map_chunks::files::read_map`, Nav2's trinary rule), transient local (latched), once |
| Parameters | `yaml_filename`, `topic_name`, `frame_id` (`map`) |

### The lanes layer (`lane_costs`, a challenger, TODO 148)

Each road of the lane map as a soft cost over the inflation, global costmap only:

| Where (the way the route drives) | Cost (0-252) |
|---|---|
| the right lane's centre | 0, rising straight to... |
| its own lines | 80 (`line`) |
| over the centre line (two-way) | 150 (`wrong_lane`) |
| the kerb | 200 (`kerb`), rising from the edge line |
| 1 m past the kerb | 200 (`off_road`), then nothing |

The block's `source` (`lane_source:=`): `map` (the default, its `roads`), `camera` (the
lane finder's lines, `camera_lanes.rs`: the same ramp; the left line is the centre line
unless the right one is yellow) or `both` (the map wins where it has a road).

Which lane is right comes from `/plan`: each road segment the newest route runs along.
Until a route says, a two-way road's two lane centres are both 0. Never as high as
inscribed: the planner may cross a line when it pays. The obstacles and unknown cells
are never lowered.

## Invariants

- The same input as Nav2's was: the launch file builds them from the tuned nav2.yaml.
- Nav2's cost codes and its 0-100 table; Nav2's message types on `*_raw`.
- The lanes layer only raises free and inflated cells, and never to inscribed.
- No ROS in `crates/costmap`; no maths in the node.
- Each costmap updates and publishes on its own thread, so the local one never waits
  for the global one. The global one is worked out whole only when its map comes or it is
  cleared; after that only the area the sensors touched (Nav2's bounds).
- `cargo fmt` and `cargo clippy` clean.

## Side Effects

- Publishes the topics above and serves the two clear services. Nothing else.

## Verification

```sh
cargo test -p costmap                                  # the layers and hand-made scenes
cargo run --release -p costmap --example timing        # how long an update takes (local; global at the campus's size)
cargo run --release -p costmap --example chunks -- ../../../maps/campus/chunks ../../../maps/campus/campus.yaml
                                                       # the chunks timed on 1 core and all; checked against the whole map
cargo bench -p costmap                                  # the same cases, measured by criterion
pixi run build && colcon test --packages-select aav_costmap
```
