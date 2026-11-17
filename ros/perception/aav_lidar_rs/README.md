# aav_lidar_rs

## Overview

`lidar_scan` in Rust (rclrs): a **drop-in** for `aav_lidar`'s Python node. The same node
name, topics, QoS, frame and ranges. Ben, 17 Nov: "lets port the 4 expensive things to
rust ... so we get our performance under control" (TODO 185).

```
/sensors/lidar/points ──► lidar_scan (Rust) ──► /sensors/lidar/scan
   (0.9 MB cloud)                                  (720 ranges round base_link)
```

Pick it with `lidar_impl:=rust` (`navigation.launch.py`, `sim.launch.py`,
`car.launch.py`), beside `lidar:=true`.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/lidar-scan` | `ScanShape`, `body_box`, `flatten`; `ground` (the body-roll sums) | no: `cargo test` on any laptop |
| `crates/lidar-scan-node` | the node, binary `lidar_scan` | yes (rclrs 0.8, `ros-env` 0.3) |

It reuses `aav_voxels_rs`'s `voxels` crate: `Point`, `Body`, the lidar's `Mount` and the
`PointCloud2` reader.

### Node `lidar_scan`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/lidar/points` | `sensor_msgs/PointCloud2`, reliable, depth 1 |
| out | `/sensors/lidar/scan` | `sensor_msgs/LaserScan`, best effort, `base_link`, the cloud's stamp |

| Parameter | Default | Means |
|---|---|---|
| `min_height`, `max_height` | 0.3 m, 2.0 m | points between these heights count |
| `angle_min`, `angle_max`, `angle_increment` | -π, π, 0.5° | 720 directions |
| `range_min`, `range_max` | 0.2 m, 30 m | from `base_link` |
| `scan_time` | 0.1 s | |
| `lidar_xyz`, `lidar_rpy` | none: must be given | `vehicle.yaml`'s `lidar_link` |
| `body` | none: must be given | the car's own outline: [x from, x to, half width] |

The Python reads `vehicle.yaml` itself; the launch file passes the Rust node the same
numbers (`voxels_car`, as for `aav_voxels_rs`).

## Invariants

- The same ranges as `scan.py`: `tests/vectors.rs` reads `vectors.json`, which
  `gen_vectors.py` writes from the Python. Change one, change both, write the vectors again.
- The same node name and topics as the Python: only one of the two runs (`lidar_impl:=`).
- `lidar-scan` has no ROS; `cargo fmt` and `cargo clippy -D warnings` clean.

## Verification

```sh
cd ros/perception/aav_lidar_rs && pixi run cargo test -p lidar-scan
pixi run build && colcon test --packages-select aav_lidar_rs
```
