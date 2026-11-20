# ros/localization

## Overview

**Where am I?** Two stock robot_localization filters, as REP-105 asks:

```
map ──ekf_global (GPS, GPS heading)──▶ odom ──ekf_local (wheels, IMU)──▶ base_link
```

## Subdirectories & Boundaries

| Package | What it does | Main output |
|---|---|---|
| [aav_localization](./aav_localization/README.md) | config and launch for `ekf_local`, `ekf_global` and `navsat_transform` | TF `map→odom→base_link`, `/localization/odom/*` |
| [aav_gps_heading_rs](./aav_gps_heading_rs/README.md) | a heading from the GPS's Doppler velocity, while the car moves; Rust (the Python removed 20 Nov) | `/localization/gps/heading` |
| [aav_ndt](./aav_ndt/README.md) | a trial: NDT on a saved lidar map, in Rust; never TF; into `ekf_global` only with `ndt_ekf:=true` | `/localization/ndt/pose` |
| [aav_pose_check](./aav_pose_check/README.md) | a challenger (TODO 210, `pose_source:=ndt`): NDT leads `ekf_global`'s pose, the GPS only checks it; says which is wrong when they disagree; seeds a lost NDT from AMCL or the GPS; HOLD (an ERROR the watchdog stops on) when neither is trusted | `/localization/pose_check/status` |
| [aav_amcl_rs](./aav_amcl_rs/README.md) | a trial: AMCL in Rust, stock `nav2_amcl`'s place (`amcl_impl:=rust`, the default; `stock` the fall-back); never TF, scored only | `/localization/amcl/pose` |
| [aav_kiss_icp](./aav_kiss_icp/README.md) | a trial: KISS-ICP lidar odometry (how far the car moved, no map); never TF; its twist into `ekf_local` only with `kiss_ekf:=true` | `/localization/kiss_icp/odom` |
| [aav_kinematic_icp](./aav_kinematic_icp/README.md) | a challenger (TODO 216): lidar odometry that knows the car can't slide sideways, the wheels its prior, in Rust; KISS-ICP's drop-in (`odom_lidar:=kinematic_icp`) | `/localization/kiss_icp/odom` |
| [aav_lane_pose](./aav_lane_pose/README.md) | a trial: the lane finder's lines against the lane map, the car's place across its lane; into `ekf_global` only with `lane_pose:=true` | `/localization/lane/pose` |
| [aav_localization_health_rs](./aav_localization_health_rs/README.md) | warn-only checks: gyro bias, IMU mounting, GPS quality, stamps, frames, TF, front wheels, runaway EKFs; Rust (the Python removed 20 Nov) | `/localization/health`, `/diagnostics` |

## Invariants

- REP-105 frames; map x east, y north, origin at the datum.
- No magnetometer: the start heading is `start_yaw`, then GPS heading once moving.
- No GPS heading below 0.3 m/s, or when the GPS and wheel speeds disagree.

## Side Effects

- Publishes TF and odometry; serves the EKF `set_pose` and navsat `fromLL`/`toLL` services.

## Verification

- Run: `pixi run test`; `aav_localization` has no tests of its own (see its README);
  `aav_localization_health_rs`'s checks are tested without ROS.
