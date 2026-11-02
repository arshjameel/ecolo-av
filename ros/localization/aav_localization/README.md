# aav_localization

## Overview

Where the car is. Stock `robot_localization` (3.8.3, Jazzy), configured here. No code of
our own: a launch file and its config files.

| Node | Gives | Used by |
|---|---|---|
| `ekf_local` | TF `odom → base_link`: smooth, never jumps, slowly drifts | Nav2's controller and local costmap |
| `ekf_global` | TF `map → odom`: GPS-anchored, can jump | Nav2's planner, global costmap, goals |
| `navsat_transform` | each GPS fix as a map-frame position | `ekf_global` |
| `gps_heading` ([aav_gps_heading](../aav_gps_heading/README.md)) | the heading from the GPS velocity | `ekf_global` |

The two EKFs never read each other's topic. They meet only in TF.

```
/vehicle/odom, /sensors/imu/data ──> ekf_local ──> /localization/odom/local, TF odom → base_link
  + /localization/gps/odom, /localization/gps/heading ──> ekf_global ──> /localization/odom/global, TF map → odom
/sensors/gps/fix, /localization/odom/global ──> navsat_transform ──> /localization/gps/odom, /localization/gps/fix
```

## Interface

### Launch `launch/localization.launch.py`

| Argument | Default | Means |
|---|---|---|
| `gps` | true | false (bench, indoors): only `ekf_local`, and a static identity `map → odom` |
| `gps_heading` | true | false leaves out `gps_heading` |
| `kiss_ekf` | false | a trial: `ekf_local` also takes KISS-ICP's twist (`/localization/kiss_icp/odom`: vx, vyaw; never its pose), with `config/ekf_local_kiss.yaml` on top. `sim.launch.py kiss_ekf:=true` starts KISS-ICP too |
| `ndt_ekf` | false | a trial: `ekf_global` also takes NDT's pose (`/localization/ndt/pose`: x, y, yaw), with `config/ekf_global_ndt.yaml` on top. `sim.launch.py ndt_ekf:=true` starts NDT too |
| `start_x`, `start_y`, `start_yaw` | 0, 0, 0 | where the car starts: m east and north of the datum, rad counter-clockwise from east |
| `use_sim_time` | false | |

### Topics

| Node | In | Out |
|---|---|---|
| ekf_local | `/vehicle/odom` (vx, vy, vyaw), `/sensors/imu/data` (yaw rate); with `kiss_ekf:=true`, `/localization/kiss_icp/odom` (vx, vyaw) | `/localization/odom/local`, TF |
| ekf_global | the same, plus `/localization/gps/odom` (x, y), `/localization/gps/heading` (yaw); with `ndt_ekf:=true`, `/localization/ndt/pose` (x, y, yaw) | `/localization/odom/global`, TF |
| navsat_transform | `/sensors/gps/fix`, `/sensors/imu/data`, `/localization/odom/global`, TF `base_link → gps_link` | `/localization/gps/odom`, `/localization/gps/fix` |

### Services

| Service | Server |
|---|---|
| `/ekf_local/set_pose`, `/ekf_global/set_pose` (also `enable`, `reset`, `toggle` under each) | the EKFs (renamed so they don't collide) |
| `/fromLL`, `/toLL`, `/datum` | navsat_transform |

### Config

| File | Key settings |
|---|---|
| `config/ekf_local.yaml` | 30 Hz, `two_d_mode`, `world_frame: odom` |
| `config/ekf_global.yaml` | 30 Hz, `world_frame: map`; GPS fixes refused past 10 sigma (`odom1_pose_rejection_threshold`); yaw process noise 0.01; x, y start 100 m unsure, so the first fix decides them |
| `config/ekf_local_kiss.yaml` | only with `kiss_ekf:=true`: `odom1` = KISS-ICP's twist, vx and vyaw; refused past 5 sigma; `smooth_lagged_data` with 1 s of history (its twists carry the cloud's stamp) |
| `config/ekf_global_ndt.yaml` | only with `ndt_ekf:=true`: `pose0` = NDT's pose, x, y, yaw; refused past 5 sigma; `smooth_lagged_data` with 1 s of history (NDT's poses carry the cloud's stamp, so they come late) |
| `config/navsat.yaml` | `wait_for_datum: true`; `datum` [45.3860, -75.6960, 0] is a placeholder near Carleton (TODO: the test site) |

## Invariants

- REP-105: `map → odom → base_link`. `map` is x east, y north, origin = the datum.
- Keep the datum's third value, `magnetic_declination_radians` and `yaw_offset` at 0:
  they would rotate the map frame.
- No magnetometer: the start heading comes from `start_yaw`; then the gyro carries it,
  and GPS corrects it while moving.
- A GPS fix more than 10 sigma off (a 20 m jump) is refused, except the first; the sensor
  watchdog holds Nav2 for a jump meanwhile.
- Antenna offset comes from TF (`vehicle.yaml`), so its place there matters.
- Parameter names checked against robot_localization 3.8.3's source.

## Side Effects

- Publishes TF `odom → base_link` and `map → odom` (or a static `map → odom` without GPS).
- Publishes the topics and serves the services above.

## Verification

This package has no tests of its own (config only); `aav_bringup`'s
`test_sim_launch.py` checks `ndt_ekf` and `ekf_global_ndt.yaml`, `kiss_ekf` and
`ekf_local_kiss.yaml`.

```sh
pixi run build && colcon test --packages-select aav_localization && colcon test-result --verbose
pixi run sim-check        # Gazebo: compares /localization/odom/global with the truth
pixi run heading-check    # the global EKF's heading, forward and reversing
```

`pixi run test` runs everything.
