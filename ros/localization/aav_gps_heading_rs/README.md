# aav_gps_heading_rs

## Overview

`gps_heading` in Rust (rclrs): **the default since 19 Nov** (Ben, 16 Nov: "lets remove
python were possible"), in place of `aav_gps_heading`'s Python node. The same node name,
topics, parameters and answers. The Python was removed on 20 Nov (tag
`save/python-gps-heading`).

```
/sensors/gps/fix_velocity ─┐
/vehicle/odom ─────────────┴─► gps_heading ─► /localization/gps/heading ─► ekf_global
```

The car's heading from the receiver's Doppler velocity, while the car moves: the
maths and why are in `aav_gps_heading`'s README.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/gps-heading-core` | `heading`, `CourseSettings`, `antenna_direction`, `wrap`, `quaternion_z`, `radians`, `degrees`, Python's `hypot` | no: `cargo test` on any laptop |
| `crates/gps-heading-node` | the process, binary `gps_heading` | yes (rclrs 0.8, `ros-env` 0.3) |

### Node `gps_heading`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/gps/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped`: ENU m/s (ublox_gps; the simulator's sim_gps) |
| in | `/vehicle/odom` | `nav_msgs/Odometry`: the wheels' signed speed and turn rate |
| out | `/localization/gps/heading` | `sensor_msgs/Imu`, `base_link`: orientation (yaw from east, counter-clockwise) and its variance only |

The node uses relative names; in the root namespace they are the absolute ones above.

| Parameter | Default | Means |
|---|---|---|
| `min_speed` | 0.3 m/s | slower than this (GPS or wheels): no heading |
| `max_speed_gap` | 0.3 m/s | the GPS's and the wheels' speeds must agree to this |
| `floor_deg` | 2.0° | added to every heading's standard deviation |

The antenna's place comes from `vehicle.yaml`'s `gps_link` (through `aav_description`).

One more parameter: `vehicle_yaml` (`''`: `aav_description`'s, on `AMENT_PREFIX_PATH`), for the
antenna's place (`sensors.gps_link.xyz`).

### Which one runs

`localization.launch.py gps_heading_impl:=rust` (the one choice, a hook for a challenger) (`vehicle.GPS_HEADING_IMPLS`,
default `rust`), with `gps:=true` and `gps_heading:=true`.

## Invariants

- The same answers as `course.py`, **to the bit**: `tests/vectors.rs` reads
  `vectors.json`, a **golden fixture** since 20 Nov: `course.py` wrote it (4,000 cases forward and
  reversing, turning, slow, disagreeing, NaN and infinite; `wrap` and the quaternion on 508
  angles). The Python and `gen_vectors.py` went on 20 Nov
  (`save/python-gps-heading`): never regenerate it, edit it only with a reason.
- To match to the bit: CPython's `hypot`, Python's `%` and `radians`, and `sin` and `cos`
  kept apart (an optimised build may join them into `sincos`, which differs in the last
  bit on macOS).
- `gps-heading-core` has no ROS; no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## Where it differs from the Python

| Python | Rust |
|---|---|
| `min_speed:=0` and a GPS speed of exactly 0: `ZeroDivisionError`, the node dies | no heading, as for any other unusable velocity |

## Side Effects

- Publishes `/localization/gps/heading`; reads `vehicle.yaml` once at start. Stops
  cleanly on SIGINT, SIGTERM or SIGHUP.

## Verification

```sh
cd ros/localization/aav_gps_heading_rs && cargo test --locked -p gps-heading-core  # tools/test.sh
pixi run build && colcon test --packages-select aav_gps_heading_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `gps-heading-core` unit tests | forward, reversing, slow, disagreeing, `wrap` |
| `tests/vectors.rs` | `course.py`'s answers, to the bit |
| `test/test_gps_heading_rs_node.py` | the process: driving north, a heading near π/2 with REP-145's −1s; slow, nothing |
