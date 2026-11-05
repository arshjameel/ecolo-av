# aav_gps_heading

## Overview

The car's heading from the GPS receiver's own velocity (course over ground), for the
global EKF. TODO 13; the background is in `docs/learn-localization.md`.

A receiver measures its velocity from the satellites' Doppler shift, good to about
0.05 m/s, even while its position wanders by a metre. On a car that doesn't slide
sideways, that velocity points where the car points, or the opposite way when reversing.

```
heading = the velocity's direction - the antenna's own direction of travel on the car
```

The car has no working magnetometer, so without this the gyro alone carries the heading.
[aav_localization](../aav_localization/README.md) starts the node and fuses its output.

## Interface

### Node `gps_heading`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/gps/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped`: ENU m/s (ublox_gps; Gazebo's gazebo_gps) |
| in | `/vehicle/odom` | `nav_msgs/Odometry`: the wheels' signed speed and turn rate |
| out | `/localization/gps/heading` | `sensor_msgs/Imu`, `base_link`: orientation (yaw from east, counter-clockwise) and its variance only |

The node uses relative names; in the root namespace they are the absolute ones above.

| Parameter | Default | Means |
|---|---|---|
| `min_speed` | 0.3 m/s | slower than this (GPS or wheels): no heading |
| `max_speed_gap` | 0.3 m/s | the GPS's and the wheels' speeds must agree to this |
| `floor_deg` | 2.0° | added to every heading's standard deviation |

The antenna's place comes from `vehicle.yaml`'s `gps_link` (through `aav_description`).

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `course.py` | yes | `CourseSettings`, `heading()` (yaw and variance, or `None`), `antenna_direction()`, `wrap()`, `quaternion_z()` |
| `gps_heading.py` | no | the node |

## Invariants

- `course.py` has no ROS imports, and its own tests.
- Nothing is published below `min_speed`, or when the speeds disagree, or on NaN. A
  parked car keeps the heading the gyro carries.
- Reversing turns the heading round (the antenna travels at π).
- In a turn, the antenna's swing is taken out: β = atan2(w·x_a, v − w·y_a).
- The Imu message has no turn rate or acceleration: their covariance is −1 (REP-145).
  Roll and pitch variance is 1 rad² (the EKF is 2D).
- Radians, m/s; yaw counter-clockwise from east, as the map frame.
- No output until the first `/vehicle/odom` arrives.

## Side Effects

- Publishes `/localization/gps/heading`. Reads `vehicle.yaml` once at start.

## Verification

```sh
cd ros/localization/aav_gps_heading && python3 -m pytest -q   # the maths, no ROS
pixi run build && colcon test --packages-select aav_gps_heading && colcon test-result --verbose
```

`tools/test.sh` runs the first; `pixi run test` runs everything. Against Gazebo's
truth, forward and reversing: `pixi run tools/ros.sh --local python3 tools/heading_check.py` (about 4 min a run).
