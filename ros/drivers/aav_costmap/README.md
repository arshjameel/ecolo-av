# aav_costmap

Our own costmaps, in Rust: Nav2's `costmap_2d` layers ported, run **beside** Nav2's
costmaps (`costmap:=own`). The simulator's default since 14 Nov (`stack:=ours`); Nav2's stays on the car.
The page: [docs/own-costmap.md](../../../docs/own-costmap.md).

```
/map, the scan (and radar, cubes, cameras where they run), odometry, /tf_static
  -> costmap -> /drivers/nav/costmap/local (+ _raw)   odom, 20 m window, 5 Hz
             -> /drivers/nav/costmap/global (+ _raw)  map, the map's size, 1 Hz
  local_raw -> mppi_lag (tracker:=mppi_lag), instead of Nav2's /local_costmap/costmap_raw
```

## Overview

| Part | What it is |
|---|---|
| `crates/costmap` | the maths, no ROS: grid, static map, obstacle layers, inflation, the layered costmap |
| `crates/costmap-node` | the ROS node, binary `costmap` (rclrs): messages in and out only |

```sh
pixi run sim costmap:=own tracker:=mppi_lag
pixi run sim-check --costmap own --tracker mppi_lag --lidar-obstacle --plan-around
```

## Interface

**`costmap`** (`crates/costmap-node/src/main.rs`), started by `aav_navigation`'s
`navigation.launch.py costmap:=own`

| Item | What |
|---|---|
| Subscribes | `/map` (transient local); `/localization/odom/local`; `/tf` (map -> odom); `/tf_static` (sensor mounts); each layer's sources from nav2.yaml (`/sensors/lidar/scan`; the radar, cubes, camera points and ghost where they run), best effort |
| Publishes | `/drivers/nav/costmap/local`, `/global` (OccupancyGrid); `/drivers/nav/costmap/local_raw`, `/global_raw` (nav2_msgs/Costmap); `/drivers/nav/costmap/status` (DiagnosticStatus) |
| Serves | `/drivers/nav/costmap/clear_entirely_local`, `_global` (nav2_msgs/srv/ClearEntireCostmap): forget every mark. With `tracker:=mppi_lag`, `bt_navigator`'s local clear is remapped here |
| Parameters | `local`, `global`: one JSON block each, made by `aav_description`'s `vehicle.costmap_params` from the tuned nav2.yaml; `odom_topic`, `map_topic`, `base_frame`, the topic names |

## Invariants

- The same input as Nav2's: the launch file builds both from the same tuned nav2.yaml.
- Nav2's cost codes and its 0-100 table; Nav2's message types on `*_raw`.
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
pixi run build && colcon test --packages-select aav_costmap
```
