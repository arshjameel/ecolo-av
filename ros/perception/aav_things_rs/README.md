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

The default since 17 Nov with `things:=true`, in the simulator and on the car;
`things_impl:=python` (`navigation.launch.py`, `sim.launch.py`, `car.launch.py`) runs the
Python one instead.

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

## How it matches the Python

`gen_vectors.py` plays the node's whole pipeline in Python (the map's walls, groups, odom,
the radar, the tracker, the names) and writes every thing; `tests/vectors.rs` plays the same.

| Vectors | Clouds | Things listed | Largest difference |
|---|---|---|---|
| made up (a walker, a car, the radar, a row of posts) and recorded in Ravensim (a lidar-obstacle run) | 298 | 2,237 | 4.6e-15 m (allowed 1e-9) |

The same ids, kinds, positions, speeds, sizes and confidences. Names and the saved map's
background have their own vectors.

## Measured

Ravensim, `things:=true`, `/clock` at 100 Hz, standing; 60 s of `ps` after 75 s
(`/tmp/p1/sample_cpu.py`), Python then Rust back to back.

| World | Load at start | Python | Rust |
|---|---|---|---|
| road | 19.4 / 10.9 | 7.6 % of a core, 78 MB | **1.5 %**, 31 MB |
| campus (the 6400 x 6400 `/map` as background) | 19.2 / 13.7 | 7.4 %, 353 MB | **1.5 %**, 230 MB |

### In the simulator

Python and Rust back to back, the same load (`ROS_DOMAIN_ID` 180-194):

| Check | Load at start | Python | Rust |
|---|---|---|---|
| `sim-check --movers-only` | 19.7 / 19.8 | 2 of 3 (the person along the roadside: goal not reached) | **3 of 3** |
| `sim-check --lidar-obstacle` | 7.7 / 26.7 | passes, 0.18 m | passes, 0.19 m |
| the movers in that first pair | 7.7 / 26.7 | 2 of 3 | 1 of 3; both logs: odometry stale, goals aborted (the load: so run again above) |
| lot + University Drive, 2 runs each, `own_calm_chunks`, reversing | 18.2 / 17.0 | 10 of 10 legs, 1 false hold | 10 of 10 legs (one run again\*), **0 false holds** |

\* One University Drive run never moved: the output mux latched STOP at start-up
(`sim_gps`, "which nobody announces on /system/source"). The same race is in both
languages' logs; not the things.

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
