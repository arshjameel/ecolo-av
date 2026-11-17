# aav_things_rs

## Overview

`things` in Rust (rclrs): a **drop-in** for `aav_things`' Python node. The same node name,
topics, QoS, parameters and answers. Ben, 17 Nov: "lets port the 4 expensive things to
rust ... so we get our performance under control" (TODO 185).

```
/perception/voxels/cubes ─► the saved map's walls dropped ─► groups ─► to odom ─┐
/perception/radar/clusters ─► a speed along each ray ───────────────────────────┤
/perception/detections + camera_info ─► names ──────────────────────────────────┤
                                                                                 ▼
                                  tracker (a Kalman filter per thing) ─► /world/things
                                                                     └─► /world/things/markers
```

Pick it with `things_impl:=rust` (`navigation.launch.py`, `sim.launch.py`,
`car.launch.py`), beside `things:=true`.

What it does, the gates and the memory: `aav_things`' README.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/things-core` | grouping, the tracker, the radar's clusters, the background, the TF wait, the names, the looks | no: `cargo test` on any laptop |
| `crates/things-node` | the node, binary `things` | yes (rclrs 0.8, `ros-env` 0.3) |

Reused, not copied: `ego-core`'s Python `hypot`, `map-chunks`' grid, `ndt-core`'s PCD
reader, `behaviour-core`'s TF buffer, `voxels`' point type.

### `things-core`

| Module | Holds | From |
|---|---|---|
| `groups` | `group`, `to_frame`, `Group`, `GroupShape` | `groups.py` |
| `tracker` | `Tracker` (`step`, `radar`, `expire`, `at`, `rename`), `Track`, `TrackerShape`, `predict` | `tracker.py` |
| `radar` | `clusters`: a cluster cloud's rows as speeds along rays, in odom | `things_node.py` `use_clusters` |
| `background` | `Background`, `new_only`, `ChunkBackground` | `background.py` |
| `tf_wait` | `Waiting`, `Step` (an enum), `decide` | `tf_wait.py` |
| `naming` | `Namer`, `Lens`, `CameraMount`, `Axes` (an enum), `thing_box`, `iou` | `naming.py`, `aav_detector`'s `ground.py` |
| `looks` | `colour`, `caption` | `looks.py` |
| `cloud` | named columns of a `PointCloud2`, any number type | new |

### Node `things`

The same as `aav_things`' (its README): every topic, QoS and parameter, with the same
names and defaults. It reads `aav_things`' `config/things.yaml`.

## Invariants

- The same answers as the Python modules: `tests/vectors.rs` reads `vectors.json`, which
  `gen_vectors.py` writes from them. Change one, change both, write the vectors again.
- The same node name and topics as the Python: only one of the two runs (`things_impl:=`).
- Only the tracker uses the background. The stop zone and the costmaps see every point.
- `things-core` has no ROS; `cargo fmt` and `cargo clippy -D warnings` clean.

## Verification

```sh
cd ros/perception/aav_things_rs && pixi run cargo test -p things-core
pixi run python ros/perception/aav_things_rs/gen_vectors.py     # after changing the Python
pixi run build && colcon test --packages-select aav_things_rs
```
