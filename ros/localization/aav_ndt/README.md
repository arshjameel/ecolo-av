# aav_ndt

## Overview

NDT in Rust (rclrs), a **trial**: where the car is on a saved lidar map, in x, y and yaw.
Ben's idea (28 Oct), after Autoware's `ndt_scan_matcher`; our own code. The design:
`docs/designs/2026-10-29_ndt.md`. Never TF. Off by default, nothing reads its pose;
with `ndt_ekf:=true` (a trial) `ekf_global` takes it beside the GPS; with
`pose_source:=ndt` (a challenger, TODO 210) it **leads** `ekf_global` and the GPS only
checks it, through [aav_pose_check](../aav_pose_check/README.md), which also seeds a lost
NDT (`/localization/ndt/initial_pose`) from AMCL or the GPS.
`measure controllers --ndt` and `--ndt-ekf` score it against the simulator's truth.

```
/sensors/lidar/points ─┐                       ┌─▶ /localization/ndt/pose
/localization/odom/local ─▶ ndt (ndt-core) ────┤
/localization/odom/global ┘   maps/ndt/*.pcd   └─▶ /localization/ndt/status
```

**Chunks** (TODO 175, step 4): `ndt-core` `chunks.rs` cuts a map into
one `.pcd` per 64 m chunk and builds only the 3 x 3 round the car (`ChunkedMap`), cube for
cube the same as the whole map. `cargo run --release --bin ndt-chunks -- map.pcd out/ -900
-900 [x y]` writes them and compares: on the campus, 1.2 % of the cubes, 0.01 s to build.

| In the sim (15 Nov, campus tour) | Whole map | Chunks |
|---|---|---|
| Off the truth, mean, per leg (m) | 0.051 / 0.014 / 0.011 | 0.058 / 0.014 / 0.011 |
| Ready at start-up | 2.5 s | 0.02 s |
| A new 3 x 3 as the car drives | - | read 0-3 ms, built 8-22 ms |

The chunks follow where the next match starts (`Localizer::guess`), so a restart from
`ekf_global` after LOST finds its map. `sim.launch.py` uses the campus's chunks by default
once `pixi run campus` made them; `ndt_chunks:=off` for the whole map.

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
`TRACKING`, `WAITING` or `LOST`; values `stamp` (the cloud's), `score_per_point`,
`overlap`, `iterations`, `match_ms`, `points`, `poor_in_row`, `restarts`,
`clouds_dropped`, `x`, `y`, `yaw`, and `cov_xx` ... `cov_yawyaw` (-H⁻¹ before k and
the floors: `measure controllers --ndt` fits k from them).

A fit is poor (not published) when its score per point is under
`min_score_per_point`, it ran out of iterations, or, while tracking, it moved more
than `max_correction_m` / `max_correction_deg` from the guess (a jump).

Parameters: `config/ndt.yaml` (cubes, filters, the fit's floor, the step cap, the
covariance), plus from the launch file the lidar's mount and the car's own box
(`body_min`, `body_max`: `vehicle.self_box`, the body and its sensors, 15 cm wider),
`map`, and the start (`start_x`, `start_y`, `start_yaw`, `start_from_global`).

### Launch

`ndt.launch.py [ndt_map:=maps/ndt/test_lot.pcd] [ndt_x:=] [ndt_y:=] [ndt_yaw:=]
[ndt_from_global:=false] [use_sim_time:=false]`. In Ravensim: `pixi run sim
world:=test_lot ndt:=true`, after `pixi run ndt-map drive` made the map.

## Invariants

- Never publishes TF. Only `ekf_global` may read its pose, and only with `ndt_ekf:=true`
  (a trial, docs/trials.md), or through `pose_check` with `pose_source:=ndt` (TODO 210).
- The simulator's truth is never read by the node: only `tools/ndt_map.py` uses it, offline.
- A pose is published only for a good fit (score, converged, no jump); LOST after 5
  poor fits in a row, then it starts again from `ekf_global`, counted in `restarts`.
- Rust API Guidelines; `cargo fmt` and `cargo clippy` clean; no ROS in `ndt-core`.

## Side Effects

- Reads the map file at start-up (refused by the launch file if it is missing).
- The build runs `cargo build --release --locked` into colcon's build folder.

## Verification

```sh
cd ros/localization/aav_ndt && cargo test --locked -p ndt-core
cargo bench -p ndt-core                                 # a match and building the cubes, on the tests' yard (criterion)
pixi run build && colcon test --packages-select aav_ndt && colcon test-result --verbose
```

`tools/test.sh` runs the first. `colcon test` adds the launch file's test and the node's
own on a made-up map (`test/test_ndt_node.py`: a cloud in, a pose within 5 cm out).
