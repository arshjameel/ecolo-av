# aav_scripted

The scripted driver (phase 1a): an action server that drives one speed profile. It
speeds up, holds for a time or a distance, then slows down to a stop. It publishes
`/drivers/scripted/cmd_vel`, twist_mux priority 50, so a person on the joystick,
the RC or the keyboard always outranks it.

```
/scripted_drive goal -> scripted_driver -> /drivers/scripted/cmd_vel -> twist_mux -> ego
```

## Overview

`aav_language`'s `drive()` and `turn()` tools send their moves here, a whole distance
with its ramps. People can send a goal by hand:

```sh
ros2 launch aav_scripted scripted.launch.py
ros2 action send_goal /scripted_drive aav_interfaces/action/ScriptedDrive \
  "{target_speed: 1.0, acceleration: 0.5, deceleration: 1.0, hold_time: 5.0}" --feedback
```

## Interface

| Item | What |
|---|---|
| Node `scripted_driver` | `ros2 launch aav_scripted scripted.launch.py [use_sim_time:=false]` |
| Action server | `/scripted_drive` (`aav_interfaces/action/ScriptedDrive`) |
| Subscribes | `/localization/odom/local` (`nav_msgs/Odometry`): speed and distance travelled; `/safety/estop` (`std_msgs/Bool`); `/control/cmd_vel` (`geometry_msgs/TwistStamped`, twist_mux's choice: is someone else driving?) |
| Publishes | `/drivers/scripted/cmd_vel` (`geometry_msgs/TwistStamped`), frame `base_link`, while a goal runs |

The goal:

| Field | Unit | Meaning |
|---|---|---|
| `target_speed` | m/s | negative drives in reverse |
| `acceleration`, `deceleration` | m/s² | both > 0 |
| `hold_time` | s | time at speed, used when `hold_distance` is 0 |
| `hold_distance` | m | distance at speed (needs fresh odometry) |
| `curvature` | 1/m | + turns left; the turn rate is `v x curvature` |

Feedback (10 Hz): phase (accelerate, hold, decelerate, stopped), commanded and measured
speed, distance travelled. Result: success, message, distance, duration. A goal that
ends early is aborted, its message saying why (`stopped: e-stop`).

Parameters (`config/scripted.yaml`):

| Name | Default | Meaning |
|---|---|---|
| `max_speed` | 2.0 m/s | goals asking for more are rejected (ego caps lower anyway) |
| `rate_hz` | 50 Hz | command rate |
| `odom_timeout` | 0.5 s | `hold_distance` goals need odometry newer than this |

Modules:

| Module | Holds |
|---|---|
| `speed_profile.py` | `SpeedProfile(target, accel, decel, hold_time, hold_distance)`: `step(dt, distance)` -> speed, `cancel()` (slow down), `stop()` (zero at once); phases `ACCELERATE`, `HOLD`, `DECELERATE`, `STOPPED`. `goal_problem(...)`: why a goal can't run; `stop_reason(...)`: why a running goal must end. No ROS |
| `scripted_driver.py` | the ROS node `ScriptedDriver` |

## Invariants

- `speed_profile.py` has no ROS imports.
- One goal at a time: the goal is claimed when it is accepted, so a second can't slip in
  before the first starts.
- A goal is rejected if another runs, if the e-stop is on, if a number isn't finite, if
  `|target_speed|` is above `max_speed`, if a ramp is not positive, if `hold_time` or
  `hold_distance` is negative, or if `hold_distance` is set without fresh odometry. A bad
  goal can never wedge the server.
- Cancelling slows down at `deceleration` and stops; it never cuts to zero mid-speed.
- Ends at once (zero, then the 5 zeros below) on an e-stop, or when someone else drives
  (twist_mux passing another command for 0.3 s, as `aav_drive_to_point`). A `hold_distance` goal
  whose odometry stops arriving slows down and ends. twist_mux's e-stop lock holds it
  anyway; this makes the goal end instead of waiting.
- Each goal ends with 5 zero commands, so twist_mux sees the stop.
- Each step uses the time that really passed on the node's clock (the simulator's in a
  simulator), capped at 0.5 s, not a fixed `1 / rate_hz`. A fixed step stretched the
  profile on a busy computer (bench, 10 Oct: 22 m instead of 6.5 m).
- Runs on a single-threaded executor: a `MultiThreadedExecutor` spun a whole core.
- Metres, seconds, radians; positive curvature = left.

## Side Effects

- Publishes `/drivers/scripted/cmd_vel` while a goal runs; action feedback and result.
- Logs rejected goals.
- No files, no hardware.

## Verification

```sh
pixi run build && colcon test --packages-select aav_scripted && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_scripted && python3 -m pytest -q`
(`tools/test.sh` runs it). `pixi run test` runs everything.
`test/test_speed_profile.py` covers the profile's phases.
