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
| [aav_gps_heading](./aav_gps_heading/README.md) | a heading from the GPS's Doppler velocity, while the car moves | `/localization/gps/heading` |
| [aav_ndt](./aav_ndt/README.md) | a trial: NDT on a saved lidar map, in Rust; never TF | `/localization/ndt/pose` |

## Invariants

- REP-105 frames; map x east, y north, origin at the datum.
- No magnetometer: the start heading is `start_yaw`, then GPS heading once moving.
- No GPS heading below 0.3 m/s, or when the GPS and wheel speeds disagree.

## Side Effects

- Publishes TF and odometry; serves the EKF `set_pose` and navsat `fromLL`/`toLL` services.

## Verification

- Run: `pixi run test`; `aav_localization` has no tests of its own (see its README).
