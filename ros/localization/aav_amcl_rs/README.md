# aav_amcl_rs

## Overview

AMCL in Rust (rclrs), in stock `nav2_amcl`'s place: **`amcl_impl:=rust`**, the default
(TODO 231). A **trial**, scored only, as stock's was: never TF, nothing drives by it.
`measure controllers --amcl` scores it against the simulator's truth. Stock was
removed on 20 Nov (TODO 232, tag `save/nav2-amcl`); its scores stay below.

```
/sensors/lidar/scan ──────┐                         ┌─▶ /localization/amcl/pose
/map (or amcl_map:=) ─────┼─▶ amcl (amcl-core) ─────┤
/localization/odom/local ─┤                         └─▶ /localization/amcl/particles
/localization/amcl/initial_pose ┘
```

How it works, per scan:

| Step | What happens |
|---|---|
| 1 | The odometry at the scan's stamp; nothing more until the car drove `update_min_d` or turned `update_min_a` |
| 2 | Each particle moves as the odometry did, plus noise (alpha1-4; no sideways slip) |
| 3 | Each is weighed: how near the scan's beam ends land to the map's walls (the likelihood field) |
| 4 | Every `resample_interval` updates: a new set, low-variance, as many as KLD says (500 to 2000) |
| 5 | The pose: the heaviest cluster's mean, the whole set's covariance |

The scan model is a trait (`ScanModel`), so a learned one can stand in later.

## Interface

### Crates

| Crate | Holds |
|---|---|
| `crates/amcl-core` | the maths, no ROS: map and distance field, motion model, scan model, filter, odometry buffer |
| `crates/amcl-node` | the ROS node, binary `amcl`: messages in, calls to `amcl-core`, messages out |

### Node `amcl`

The stock node's topics, parameters and frames: `navigation.launch.py` gives it the same
`config/amcl.yaml` and remappings.

| Direction | Topic | Type |
|---|---|---|
| sub | `scan_topic` (`/sensors/lidar/scan`), in `base_link` | sensor_msgs/LaserScan |
| sub | `map_topic` (`/map`, or `/localization/amcl/map`), latched | nav_msgs/OccupancyGrid |
| sub | `initialpose` → `/localization/amcl/initial_pose` | geometry_msgs/PoseWithCovarianceStamped |
| sub | `odom_topic` (`/localization/odom/local`) | nav_msgs/Odometry |
| pub | `amcl_pose` → `/localization/amcl/pose`, latched | geometry_msgs/PoseWithCovarianceStamped |
| pub | `particle_cloud` → `/localization/amcl/particles` | nav2_msgs/ParticleCloud |

Where it differs from stock:

| | Stock `nav2_amcl` | Ours |
|---|---|---|
| Odometry | TF `odom → base_link` | the same transform from `ekf_local`'s odometry (rclrs has no TF library) |
| Lifecycle | a lifecycle node, with a manager | a plain node: works once the map comes |
| TF | `tf_broadcast` | never; `tf_broadcast: true` is refused |
| Models | beam, likelihood field; differential, omni | likelihood field and differential only (others refused) |
| Resampling | each pick drawn alone | low-variance, its picks shuffled so KLD can stop anywhere |
| Recovery | averages divided by the particle count; both zeroed after injecting | the weighted mean likelihood; only the fast average starts again |
| Seed | the clock | `seed` (0: the clock), so a test repeats |

Recovery is off in `amcl.yaml` (`recovery_alpha_*` are 0, as stock's default).

## Invariants

- Never publishes TF. Its pose is only scored.
- Rust API Guidelines; `cargo fmt` and `cargo clippy` clean; no ROS in `amcl-core`.

## Side Effects

- None beyond its topics. The build runs `cargo build --release --locked` into colcon's
  build folder.

## Verification

```sh
cd ros/localization/aav_amcl_rs && cargo test --locked -p amcl-core
pixi run build && colcon test --packages-select aav_amcl_rs && colcon test-result --verbose
```

`tools/test.sh` runs the first: a near guess found, a kidnap recovered, KLD's bounds,
the motion model's spread. `colcon test` adds the node's own test
(`test/test_amcl_node.py`: stock's `amcl.yaml`, a made-up map, a pose within 0.15 m).
