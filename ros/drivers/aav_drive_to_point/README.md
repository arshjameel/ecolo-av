# aav_drive_to_point

Drive to one point B and stop there smoothly (`docs/a-to-b.md`). `point_driver` is an
action server (`/drive_to_point`); `drive_to_point` is its command-line client
(`pixi run drive-to-point`). B is a GPS point (lat/lon) or a map point (x, y). B is where
`base_link`, the ground under the rear axle, stops. Forward only. Called `aav_goto`
(node `goto_driver`, action `/goto`, slot `goto`) until 26 Oct 2026.

```
pixi run drive-to-point -> /drive_to_point goal -> point_driver
  -> /drivers/point/cmd_vel -> twist_mux (25) -> ego
```

## Overview

It speeds up gently, holds a cruise speed, then slows down by the distance left, so it
stops at B. It steers by pure pursuit: first it turns towards B, then it follows the
straight line to it. Priority 25 puts it below every person, test and script, above
Nav2. It drives on GPS, so the stop-sign and sensor locks (30) hold it.

```sh
pixi run drive-to-point --sim --ahead 20                # Gazebo, 20 m ahead of the car
pixi run drive-to-point --sim --x 30 --y 5              # a map point (x east, y north)
pixi run drive-to-point --lat 45.38630 --lon -75.69580  # the real car: asks you to type "drive"
```

## Interface

**`point_driver`**: `ros2 launch aav_drive_to_point drive_to_point.launch.py [use_sim_time:=false] [watchdog:=true]`

| Item | What |
|---|---|
| Action server | `/drive_to_point` (`aav_interfaces/action/DriveToPoint`): lat/lon or x/y, `cruise_speed`, `acceleration`, `deceleration` (0 = default), `confirmed` |
| Subscribes | `/localization/odom/global` (`nav_msgs/Odometry`), `/sensors/gps/fix` (`sensor_msgs/NavSatFix`, best effort), `/safety/estop` (`std_msgs/Bool`), `/safety/sensor_watchdog/lock` (`std_msgs/Bool`), `/safety/geofence/fence` (`geometry_msgs/PolygonStamped`, latched: refuse a B outside it), `/system/source` (`aav_interfaces/DataSource`, latched), `/control/cmd_vel` (`geometry_msgs/TwistStamped`) |
| Publishes | `/drivers/point/cmd_vel` (`geometry_msgs/TwistStamped`) at `rate_hz` while a goal runs |
| Calls | `/fromLL` (`robot_localization/srv/FromLL`), for a lat/lon goal; `/ego/get_parameters`, for ego's caps |

Feedback: phase (turn, accelerate, hold, decelerate, stopping), distance left, cross
track (+ left), commanded and measured speed. Result: success, B in the map frame,
distance left, tolerance, distance, duration.

Parameters (`config/drive_to_point.yaml`; `max_brake`, `wheelbase`, `max_steering_angle` from
`vehicle.yaml`):

| Name | Default | Meaning |
|---|---|---|
| `cruise_speed` | 1.0 m/s | when the goal gives 0; ego's `max_speed` caps it (1.0 m/s if ego doesn't answer) |
| `acceleration` | 0.5 m/s² | speeding up; ego's `max_acceleration` caps it |
| `brake_fraction`, `deceleration` | 0.5, 0 | slow down at `max_brake x brake_fraction`, unless `deceleration` is set; ego's `max_deceleration` caps it |
| `brake_lead` | 0.8 s | start slowing this much sooner, times the speed |
| `stop_band`, `creep` | 0.05 m, 0.1 m/s | zero inside the band; the least it asks for short of B |
| `lookahead_min`, `lookahead_time` | 2.0 m, 1.0 s | pure pursuit's carrot |
| `lateral_accel` | 1.5 m/s² | slow down in tight turns |
| `max_distance` | 200 m | refuse a B further away |
| `pose_timeout`, `gps_timeout` | 0.5 s, 2.0 s | stop when position or fix is older |
| `require_gps` | true | false only where no GPS runs |
| `watchdog_timeout` | 1.0 s | stop when the watchdog's lock is silent; `watchdog:=false` sets 0 |
| `stuck_time`, `still_speed`, `still_time` | 5.0 s, 0.05 m/s, 1.0 s | stuck; standing still; at B before it reports |
| `rate_hz` | 50 Hz | command rate |

**`drive_to_point`** (`point_cli.py`; `tools/drive_to_point.sh`): `--lat/--lon`, `--x/--y`, or `--ahead [--left]`;
`--speed`; `--sim` (this computer only, Gazebo's clock). Reads `/system/source` and
`/localization/odom/global`; calls `/drive_to_point`. Ctrl-C cancels the goal.

Modules (all but the two nodes have no ROS):

| Module | Holds |
|---|---|
| `approach.py` | `Approach.step`: the speed, `v = sqrt(2 a (d - lead v))`; `planned_deceleration`, `stopping_distance` |
| `line.py` | `Pose`, `Line`, `pursuit_curvature`, `reachable`, `min_radius`, `turn_speed_limit` |
| `drive.py` | `Drive`, `Steering`, `Command`: speed and steering together, one tick |
| `rules.py` | `check_numbers`, `refuse_start`, `refuse_target`, `refuse_outside` (the geofence), `stop_reason`, `not_moving`, `tolerance`, `confirmed` |
| `point_driver.py`, `point_cli.py` | the action server and its client |

## Invariants

- `approach.py`, `line.py`, `drive.py`, `rules.py` have no ROS imports.
- One goal at a time.
- Refuses B over `max_distance` away, under 0.5 m away (it is there already), or inside
  the car's tightest turn (it would circle B for ever), or outside the geofence's fence. Refuses REPLAY. On REAL (or nobody announcing) it needs `confirmed`; the client asks the
  person to type "drive". `/system/source` kinds come from `aav_actuator_tests` `safety.py`,
  so both agree on what REAL means.
- Stops at once (5 zero commands; ego's own deceleration) on: cancel, e-stop, the
  watchdog's lock or its silence, a stale position or fix, someone else driving
  (twist_mux passing another command for 0.3 s), or the car not moving though asked to.
- Also stops when the drive runs past its limits:

  | Limit | Over |
  |---|---|
  | measured speed | `cruise × 1.2 + 0.3` m/s |
  | distance driven | `2·d + 2πR + 10` m (d = distance to B, R = tightest turn) |
  | time | `2 × ideal time(d + πR) + 30` s |
- Slowing follows the distance left, not the time, so a late brake corrects itself.
- At B: within 1 m with an RTK fix, 2 m without.
- Map frame: x east, y north; metres, radians, seconds; + left.

## Side Effects

- Publishes `/drivers/point/cmd_vel` while a goal runs; action feedback and result.
- Calls `/fromLL` and ego's parameter service. No files, no hardware.

## Verification

```sh
pixi run build && colcon test --packages-select aav_drive_to_point && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_drive_to_point && python3 -m pytest -q` (`tools/test.sh`
runs it). `pixi run test` runs everything. In Gazebo: `pixi run sim-check --drive-to-point`.

| Test | Covers |
|---|---|
| `test_approach.py` | speed up, hold, stop at B |
| `test_line.py` | pure pursuit, reachable |
| `test_drive.py` | whole drives on a fake car, with brake lag |
| `test_rules.py` | refusals and stop reasons |
