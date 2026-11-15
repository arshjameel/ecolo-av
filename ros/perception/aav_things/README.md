# aav_things

## Overview

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
| Track | one constant-velocity Kalman filter per thing (x, y, vx, vy in odom); greedy nearest association inside a 2 m gate. AB3DMOT (Weng 2020): the simple tracker is a strong baseline |
| Radar | where `radar:=true`, each cluster's mean radial speed, the car's own speed taken off, updates the nearest thing's speed along the ray |
| Memory | a thing seen 3 times stays listed 3 s after it leaves view, moving on its speed (slower than 0.5 m/s: parked). Within 4 m of the car, 30 s: the roof lidar can't see a low box there, so not seeing it says nothing |
| Background | `background: true` (off by default): cubes on the saved map's walls (`/map`, grown `background_slack`, 0.25 m) are dropped before grouping, so a wall seen again is not a new thing (`background.py`, TODO 175). Safety never relies on it |
| Names | `names: true` (off by default, `things_names:=true`): each thing's box is drawn into every camera's picture; the detector's box overlapping it most (IoU ≥ 0.3, score ≥ 0.5) gives its `kind` (person, car, stop sign), held 3 s unseen (`naming.py`, TODO 192). The lidar says where, the camera says what. On the car it needs the cameras' mounts calibrated (Q22) |

### Stock or ours?

| Looked at | Verdict |
|---|---|
| Autoware `multi_object_tracker` (Apache-2.0) | not on RoboStack Jazzy; pulls in Autoware's message and utility tree |
| Autoware `euclidean_cluster_object_detector` (on RoboStack, Apache-2.0) | the clustering only, in Autoware's messages, on the raw cloud; our cubes are already the cloud cut to the road |
| `laser_segmentation` (on RoboStack) | segments a 2D scan only, no tracking |
| `vision_msgs/Detection3DArray` for the output | no speed field; so `aav_interfaces/Things` |

So ours: about 300 lines of numpy, with tests.

## Interface

### Node `things`

| In / out | Topic | Type |
|---|---|---|
| in | `/perception/voxels/cubes` | `PointCloud2`, `base_link` (aav_voxels) |
| in | `/perception/radar/clusters` | `PointCloud2`, the radar's frame (with `radar:=true`) |
| in | `/localization/odom/local` | `Odometry`: the car's speed, for the radar |
| in | TF | `odom ← base_link` at each cloud's stamp; `odom ← radar_link`. A cloud newer than /tf waits for it (below) |
| out | `/world/things` | `aav_interfaces/Things`, odom, 10 Hz |
| out | `/world/things/markers` | `MarkerArray`, odom: box, speed arrow, id |

### Parameters (`config/things.yaml`)

| Parameter | Default | Means |
|---|---|---|
| `gap` | 0.6 m | cubes this close make one thing |
| `min_cubes`, `max_size` | 2, 8 m | noise below, walls above |
| `gate` | 2 m | farther from a thing is another thing |
| `confirm` | 3 | clouds seen before it is listed |
| `keep`, `keep_blind`, `blind_range` | 3 s, 30 s, 4 m | the memory |
| `still_speed` | 0.5 m/s | slower, an unseen thing is taken as parked |
| `radar` | false | the launch file sets it to `radar:=` |

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `groups.py` | yes | `GroupShape`, `Group`, `group`, `to_frame` |
| `tracker.py` | yes | `TrackerShape`, `Track`, `Tracker`, `predict`, `update`, `RadarCluster`, `ground_radial_speed` |
| `tf_wait.py` | yes | `Waiting`, `decide`, `Step`, `WaitShape`: clouds waiting for their transform (tf2's MessageFilter, in short) |
| `things_node.py` | no | the node |

## Invariants

- `groups.py`, `tracker.py` and `tf_wait.py` have no ROS imports, and their own tests.
- A callback never blocks on /tf. A cloud newer than the newest transform:

  | Its lead | Then |
  |---|---|
  | up to 0.1 s | the newest transform stands in |
  | more | it waits (at most 30 clouds), oldest first |
  | still waiting after 1.5 s | dropped; the drops are said once every 5 s |
- Output in `odom` (REP-105); speeds over the ground, not relative to the car.
- A thing is listed only once seen 3 times: a one-cloud flash never shows.
- `kind` is `unknown` until camera-to-lidar calibration exists. Never guessed.
- Off by default (`things:=false`). Nothing reads `/world/things` yet.

## Side Effects

- Publishes the two topics above. With `things:=true`, `navigation.launch.py` also starts
  `voxels` (as `screen_cubes:=true` does): Nav2 stays on the flat scan unless `voxels:=true`.

## Verification

```sh
pixi run python -m pytest ros/perception/aav_things/test
pixi run sim world:=test_lot obstacle:=true things:=true   # then: ros2 topic echo /world/things
```
