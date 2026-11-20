# aav_kinematic_icp

## Overview

**Lidar odometry that knows it is on a car** (TODO 216, a challenger to
[aav_kiss_icp](../aav_kiss_icp/README.md)). Kinematic-ICP's idea (T. Guadagnino et al.,
PRBonn, 2025; MIT), rewritten in Rust, no code taken:

| | KISS-ICP (the champion) | `kinematic_icp` |
|---|---|---|
| A step | free in 6 directions | **one arc**: `rho` m along, `theta` rad of turn; the car can't slide sideways |
| The wheels | not used | their step is the **prior**, weighed against the points |
| A long corridor, an open car park | slides along it | the wheels carry the step |
| Sideways speed out | whatever the match says | always 0 |

```
/sensors/lidar/points ─► kinematic_icp ─► /localization/kiss_icp/odom   (the same topics:
/vehicle/odom (wheels) ─┘              └► /localization/kiss_icp/status   a drop-in)
```

```sh
pixi run sim world:=road pose:=kinematic_ekf          # ekf_local takes its twist
pixi run sim world:=road kiss_icp:=true odom_lidar:=kinematic_icp   # scored only
```

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/kinematic-icp` | `Motion` (the arc), `Pose2`, `VoxelMap`, `register` (Gauss-Newton on rho and theta, Geman-McClure weights, the wheels' prior), `Odometry` (scan after scan; KISS-ICP's adaptive threshold) | no |
| `crates/kinematic-icp-node` | the node, binary `kinematic_icp` | yes (rclrs 0.8) |

### Node `kinematic_icp`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/lidar/points` | `PointCloud2`; reliable, keep last 1 |
| in | `/vehicle/odom` (`wheel_topic`) | `Odometry`: the wheels' speed and turn rate, the prior (never `ekf_local`'s output: no loop) |
| out | `/localization/kiss_icp/odom` | `Odometry`: frame `kiss_odom`, child `base_link`; twist vx, vyaw (vy 0) |
| out | `/localization/kiss_icp/status` | `DiagnosticStatus`: `match_ms`, `points`, `matched`, `threshold`, `wheels`, `clouds`, `clouds_dropped` |

| Parameter | Default | Means |
|---|---|---|
| `min_height` | 0.3 m | the ground cut out, as KISS-ICP's |
| `min_range`, `max_range` | 1, 50 m | flat from `base_link`; the map forgets beyond `max_range` |
| `voxel_size`, `max_points_per_voxel` | 0.5 m, 20 | the map |
| `wheel_rho_sd`, `wheel_theta_sd` | 0.05 m, 0.02 rad a step | how much the wheels are trusted |
| `wheel_timeout` | 0.5 s | older wheels: the last step held |
| `twist_linear_sd`, `twist_angular_sd` | 0.15 m/s, 0.01 rad/s | the twist's covariance, as KISS-ICP's |
| `lidar_xyz`, `lidar_rpy`, `body_min`, `body_max` | `vehicle.yaml` | set by the launch file |

## Invariants

- The same topics, frame and twist as `aav_kiss_icp`: `ekf_local`'s `kiss_ekf:=true` file
  reads either. Only one runs (`odom_lidar:=`). Never TF.
- No ROS in `kinematic-icp`; `cargo fmt`, `cargo clippy` clean; `Cargo.lock` committed.

## Side Effects

- Publishes the two topics above. Nothing else.

## Verification

```sh
cd ros/localization/aav_kinematic_icp && cargo test --locked -p kinematic-icp   # tools/test.sh runs it
pixi run build && colcon test --packages-select aav_kinematic_icp
```

| Test (synthetic scans of a walled yard with posts, and a corridor) | Shows |
|---|---|
| a 40-step arc at 2 m/s, honest wheels | within 5 cm and 0.3° of the truth; twist 2 m/s, 0.15 rad/s |
| wheels reading 20 % fast | the points correct them: under a tenth of the wheels' error |
| no wheels | the last step held (as KISS-ICP), within 10 cm |
| a corridor (nothing along it) | the wheels carry the step; no sideways drift |
| the threshold | shrinks once answers agree with their priors |

Not yet run in Ravensim or on the car: scored in stage 2.
