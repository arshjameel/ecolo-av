# aav_things_rs

## Overview

`things` in Rust (rclrs): a **drop-in** for `aav_things`' Python node. The same node name,
topics, QoS, parameters and answers. Ben, 17 Nov: "lets port the 4 expensive things to
rust ... so we get our performance under control" (TODO 185).

**The Python node was removed on 20 Nov** (Ben, 16 Nov: "lets remove python were possible"; tag `save/python-things`). `vectors.json` is now a **golden fixture**: the Python wrote it, and `gen_vectors.py` went with it, so never regenerate it; change it only with a reason.

```
/perception/voxels/cubes ─► the saved map's walls dropped ─► groups ─► to odom ─┐
/perception/radar/clusters ─► a speed along each ray ───────────────────────────┤
/perception/detections + camera_info ─► names ──────────────────────────────────┤
                                                                                 ▼
                                  tracker (a Kalman filter per thing) ─► /world/things
                                                                     └─► /world/things/markers
```

The default since 17 Nov with `things:=true`, in the simulator and on the car.

What it does, the gates and the memory: below.

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
| `tracker` | `Tracker` (`step`, `radar`, `expire`, `at`, `rename`), `Track`, `TrackerShape`, `predict`; with `per_kind` (TODO 217, a challenger, `things_tracker:=per_kind`): `accel_covariance` (a vehicle's noise long along its heading and narrow across, a person's round), `predict_with`, `fitted_speed` (a thing found again re-fits its speed from its last sightings); the shared vectors have a per-kind run | `tracker.py` |
| `radar` | `clusters`: a cluster cloud's rows as speeds along rays, in odom | `things_node.py` `use_clusters` |
| `background` | `Background`, `new_only`, `ChunkBackground` | `background.py` |
| `tf_wait` | `Waiting`, `Step` (an enum), `decide` | `tf_wait.py` |
| `naming` | `Namer`, `Lens`, `CameraMount`, `Axes` (an enum), `thing_box`, `iou` | `naming.py`, `aav_detector`'s `ground.py` |
| `looks` | `colour`, `caption` | `looks.py` |
| `cloud` | named columns of a `PointCloud2`, any number type | new |

### Node `things`

The same as the Python's was: every topic, QoS and parameter, with the same names and
defaults. It reads its own `config/things.yaml` (moved here from the Python, 20 Nov).

## How it matches the Python

`gen_vectors.py` played the node's whole pipeline in Python (until 20 Nov) (the map's walls, groups, odom,
the radar, the tracker, the names) and wrote every thing; `tests/vectors.rs` plays the same.

| Vectors | Clouds | Things listed | Largest difference |
|---|---|---|---|
| made up (a walker, a car, the radar, a row of posts; once with a motion model per kind, TODO 217) and recorded in Ravensim (a lidar-obstacle run) | 378 | 3,166 | 4.6e-15 m (allowed 1e-9) |

The same ids, kinds, positions, speeds, sizes and confidences. Names and the saved map's
background have their own vectors.

## Measured

Ravensim, `things:=true`, `/clock` at 100 Hz, standing; 60 s of `ps` after 75 s
(`/tmp/p1/sample_cpu.py`), Python then Rust back to back.

| World | Load at start | Python | Rust |
|---|---|---|---|
| road | 19.4 / 10.9 | 7.6 % of a core, 78 MB | **1.5 %**, 31 MB |
| campus (the 6400 x 6400 `/map` as background) | 19.4 / 19.2 | 7.4 %, 353 MB | **1.2 %**, 188 MB |

### On many cores (TODO 195)

`cargo run --release -p things-core --example timing -- maps/campus/chunks/ndt maps/campus/campus.yaml`
(Ben's MacBook, M4 Max, idle, 20 Nov):

| What | One core | rayon | Kept? |
|---|---|---|---|
| the campus `/map` (6400 x 6400) as the background, walls grown | 112 ms | **7.4 ms** | yes |
| a chunk crossing: the NDT map's 3 x 3 chunks read | 2.5 ms (worst 11.9) | **1.0 ms** (worst 3.0) | yes |
| one cloud, 700 cubes: background, groups, tracker (idle: 0.15 ms) | 0.16 ms\* | 0.20 ms\* (grouping's neighbours shared out) | no |
| one cloud, 15 cubes | 0.006 ms\* | 0.054 ms\* | no |

\* Measured while a sim league ran on the same Mac, one after the other. A cloud is too small
to share out: rayon's start-up costs more than the work.

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

## What it does (from the Python package's README, 20 Nov)

The first form of "the things" ([the whole car](../../../docs/designs/2026-11-14_the-whole-car.md),
"Seeing: things, not just points"; TODO 156). Before it, nothing near the car had an id,
a speed or a memory. `things` turns the lidar's cubes into one list:

```
/perception/voxels/cubes ─► group (Euclidean, 0.6 m) ─► one Kalman filter each ─► /world/things
/perception/radar/clusters ─► speed along the ray ───────────┘    (odom, 10 Hz)  ─► .../markers
```

| Step | How |
|---|---|
| Group | cubes closer than `gap` (0.6 m, flat) are one thing; fewer than 2 cubes is noise; longer than 8 m is a wall (the map's job) |
| Track | one constant-velocity Kalman filter per thing (x, y, vx, vy in odom); greedy nearest association inside a 2 m gate. AB3DMOT (Weng 2020): the simple tracker is a strong baseline. Since 17 Nov (TODO 222) a pair must also pass three checks: inside the thing's own uncertainty (Mahalanobis distance² under 9.21), a like footprint (not 2.5 times larger or smaller and 0.6 m apart), and a small thing (0.8 m or less: a post, a person) no faster than 3.5 m/s since it was last seen; a parked thing unseen keeps a small speed doubt. Before, a row of thin posts hopped post to post at up to 13 m/s. `per_kind` (TODO 217, a challenger, `things_tracker:=per_kind`): a motion model per kind (a vehicle's acceleration noise a bicycle's, a person's round) and a speed re-fitted from the last sightings when a thing is found again; the Rust twin gives the same answers |
| Radar | where `radar:=true`, each cluster's mean radial speed, the car's own speed taken off, updates the nearest thing's speed along the ray |
| Memory | a thing seen 3 times stays listed 3 s after it leaves view, moving on its speed (slower than 0.5 m/s: parked). Within 4 m of the car, 30 s: the roof lidar can't see a low box there, so not seeing it says nothing |
| Background | `things_background:=true` (off by default): cubes on the saved map are dropped before grouping, so a wall seen again is not a new thing (`background.py`, TODO 175). Two saved maps, below. Safety never relies on it: the stop zone and the costmaps read the lidar's own topics, never `/world/things` |
| Names | `names: true` (off by default, `things_names:=true`): each thing's box is drawn into every camera's picture; the detector's box overlapping it most (IoU ≥ 0.3, score ≥ 0.5) gives its `kind` (person, car, stop sign), held 3 s unseen (`naming.py`, TODO 192). A thing reaching the cubes' 2 m ceiling (`name_ceiling`) has an unseen top, such as a stop sign's pole under its plate: it is matched on its bearing instead (`name_min_side`, 0.5). Passed in the sim: `pixi run -e ml sim-check --thing-names`. The lidar says where, the camera says what. On the car it needs the cameras' mounts calibrated (Q22) |

#### The background: which saved map

| `things_background_chunks:=` | The background | Taken |
|---|---|---|
| empty | `/map`, the occupancy grid (`Background.from_grid`) | every wall cell, grown 0.25 m. On the campus that is all that isn't road or lot: a person on a path or the grass is dropped too |
| `maps/campus/chunks/ndt` | the NDT map's 3 x 3 chunks round the car (`ChunkBackground`), read again in each new chunk | only where the saved 3D map has points over `background_min_z` (0.3 m): buildings and poles. 3 % of the cells round the campus start |

The scores (16 Nov, our simulator): TODO 175 in `TODO.md`.

#### Stock or ours?

| Looked at | Verdict |
|---|---|
| Autoware `multi_object_tracker` (Apache-2.0) | not on RoboStack Jazzy; pulls in Autoware's message and utility tree |
| Autoware `euclidean_cluster_object_detector` (on RoboStack, Apache-2.0) | the clustering only, in Autoware's messages, on the raw cloud; our cubes are already the cloud cut to the road |
| `laser_segmentation` (on RoboStack) | segments a 2D scan only, no tracking |
| `vision_msgs/Detection3DArray` for the output | no speed field; so `aav_interfaces/Things` |

So ours: about 300 lines of numpy, with tests.

### Interface

#### Node `things`

| In / out | Topic | Type |
|---|---|---|
| in | `/perception/voxels/cubes` | `PointCloud2`, `base_link` (aav_voxels) |
| in | `/perception/radar/clusters` | `PointCloud2`, the radar's frame (with `radar:=true`) |
| in | `/localization/odom/local` | `Odometry`: the car's speed, for the radar |
| in | TF | `odom ← base_link` at each cloud's stamp; `odom ← radar_link`. A cloud newer than /tf waits for it (below) |
| out | `/world/things` | `aav_interfaces/Things`, odom, 10 Hz |
| out | `/world/things/markers` | `MarkerArray`, odom: a box coloured by kind, a speed arrow, the name and id ("person 3 1.2 m/s") |

#### Parameters (`config/things.yaml`)

| Parameter | Default | Means |
|---|---|---|
| `gap` | 0.6 m | cubes this close make one thing |
| `min_cubes`, `max_size` | 2, 8 m | noise below, walls above |
| `gate` | 2 m | farther from a thing is another thing |
| `confirm` | 3 | clouds seen before it is listed |
| `keep`, `keep_blind`, `blind_range` | 3 s, 30 s, 4 m | the memory |
| `still_speed` | 0.5 m/s | slower, an unseen thing is taken as parked |
| `gate_chi2` | 9.21 | a group further than this (Mahalanobis distance²) from a thing's own uncertainty is another thing |
| `shape_ratio`, `shape_slack` | 2.5, 0.6 m | a footprint that many times larger or smaller, and that much, is another thing |
| `small_size`, `small_speed`, `small_slack` | 0.8 m, 3.5 m/s, 0.3 m | a thing no wider never moves faster, give or take the slack |
| `radar` | false | the launch file sets it to `radar:=` |
| `background` | false | the launch file sets it to `things_background:=` (never in `things.yaml`: a node's own block there outranks the launch's) |
| `background_chunks` | empty | the launch's `things_background_chunks:=`: a folder of NDT chunks, `chunks.yaml` beside it |
| `background_slack`, `background_min_z` | 0.25 m, 0.3 m | walls grown for pose error; NDT points lower are the ground |

#### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `groups.py` | yes | `GroupShape`, `Group`, `group`, `to_frame` |
| `tracker.py` | yes | `TrackerShape`, `Track`, `Tracker`, `predict`, `update`, `RadarCluster`, `ground_radial_speed` |
| `looks.py` | yes | `colour`, `caption`: a thing's colour by kind and its words, for the markers (the Rust screens use the same colours: `hud_core::things`) |
| `background.py` | yes | `Background` (`from_grid`, `from_points`, `on_map`), `new_only`, `ChunkBackground`, `read_pcd`, `read_grid` |
| `tf_wait.py` | yes | `Waiting`, `decide`, `Step`, `WaitShape`: clouds waiting for their transform (tf2's MessageFilter, in short) |
| `things_node.py` | no | the node |

## Invariants

- The same answers as the Python modules: `tests/vectors.rs` reads `vectors.json`, a golden
  fixture since the Python went (20 Nov): never regenerate it.
- Only the tracker uses the background. The stop zone and the costmaps see every point.
- `things-core` has no ROS; `cargo fmt` and `cargo clippy -D warnings` clean.

## Verification

```sh
cd ros/perception/aav_things_rs && pixi run cargo test -p things-core
pixi run build && colcon test --packages-select aav_things_rs
```
