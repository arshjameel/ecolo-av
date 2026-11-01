# aav_ndt

## Overview

NDT in Rust (rclrs), a **trial**: where the car is on a saved lidar map, in x, y and yaw.
Ben's idea (28 Oct), after Autoware's `ndt_scan_matcher`; our own code. The design:
`docs/designs/2026-10-29_ndt.md`. Never TF, and nothing drives by it until it wins the
trials; `controller-compare --ndt` scores it against Gazebo's truth.

```
/sensors/lidar/points ─┐                       ┌─▶ /localization/ndt/pose
/localization/odom/local ─▶ ndt (ndt-core) ────┤
/localization/odom/global ┘   maps/ndt/*.pcd   └─▶ /localization/ndt/status
```

## Interface

### Crates

| Crate | Holds |
|---|---|
| [`ndt-core`](./crates/ndt-core/README.md) | the maths, no ROS: cubes, score, Newton, filters, the guess, LOST |
| `crates/ndt-node` | the ROS node, binary `ndt`: messages in, calls to `ndt-core`, messages out |

### Node `ndt`

| Direction | Topic | Type |
|---|---|---|
| sub | `/sensors/lidar/points` (reliable, keep last 1) | sensor_msgs/PointCloud2 |
| sub | `/localization/odom/local` (the guess) | nav_msgs/Odometry |
| sub | `/localization/odom/global` (the restart after LOST) | nav_msgs/Odometry |
| sub | `/localization/ndt/initial_pose` | geometry_msgs/PoseWithCovarianceStamped |
| pub | `/localization/ndt/pose` (good fits only, the cloud's stamp, map frame) | geometry_msgs/PoseWithCovarianceStamped |
| pub | `/localization/ndt/status` (every cloud) | diagnostic_msgs/DiagnosticStatus |

Status: level OK (a good fit), WARN (poor, or no start), ERROR (LOST); message
`TRACKING`, `WAITING` or `LOST`; values `score_per_point`, `overlap`, `iterations`,
`match_ms`, `points`, `poor_in_row`, `restarts`, `clouds_dropped`, `x`, `y`, `yaw`.

Parameters: `config/ndt.yaml` (cubes, filters, the fit's floor, the step cap, the
covariance), plus from the launch file the lidar's mount and the body (vehicle.yaml),
`map`, and the start (`start_x`, `start_y`, `start_yaw`, `start_from_global`).

### Launch

`ndt.launch.py [ndt_map:=maps/ndt/test_lot.pcd] [ndt_x:=] [ndt_y:=] [ndt_yaw:=]
[ndt_from_global:=false] [use_sim_time:=false]`. In Gazebo: `pixi run sim
world:=test_lot ndt:=true`, after `pixi run ndt-map drive` made the map.

## Invariants

- Never publishes TF; nothing reads its pose to drive (a trial, docs/trials.md's rule).
- Gazebo's truth is never read by the node: only `tools/ndt_map.py` uses it, offline.
- A pose is published only for a good fit; LOST after 5 poor fits in a row, then it
  starts again from `ekf_global`, counted in `restarts`.
- Rust API Guidelines; `cargo fmt` and `cargo clippy` clean; no ROS in `ndt-core`.

## Side Effects

- Reads the map file at start-up (refused by the launch file if it is missing).
- The build runs `cargo build --release --locked` into colcon's build folder.

## Verification

```sh
cd ros/localization/aav_ndt && cargo test --locked -p ndt-core
pixi run build && colcon test --packages-select aav_ndt && colcon test-result --verbose
```

`tools/test.sh` runs the first. `colcon test` adds the launch file's test and the node's
own on a made-up map (`test/test_ndt_node.py`: a cloud in, a pose within 5 cm out).
