# aav_track

Trial path followers, beside Nav2's own (`docs/trials.md`). The `track` node stands in
for controller_server's FollowPath: same planner, same goals, same safety chain after it.
Only one follower runs: chosen at launch with `tracker:=`.

```
bt_navigator --FollowPath ("follow_path")--> track --cmd_vel_nav--> velocity_smoother
   -> collision_monitor -> /drivers/nav/cmd_vel (twist_mux 20) -> ego
controller_server keeps running; its own action is renamed follow_path_unused
```

## Overview

| `tracker:=` | Steers by | Module |
|---|---|---|
| `none` (default) | controller_server: MPPI or RPP (`controller:=`); this node doesn't run | - |
| `pure_pursuit` | an arc to a carrot on the path: the trial's control | `trackers/pure_pursuit.py` |
| `stanley` | heading error + atan(k e / v), at the front axle | `trackers/stanley.py` |
| `lqr` | -K x about the path's bend, K by speed and bend (Riccati), + a 2 s preview of the bends ahead | `trackers/lqr.py` |
| `mpc` | a 1.5 s plan of steering, within full lock and the steering rate (scipy) | `trackers/mpc.py` |
| `mpc_full` | a 9 m plan of speed and steering together, within the steering, throttle and brake (its own speed) | `trackers/mpc_full.py` |

Every tracker drives with **one speed policy** (`speed.py`) and Nav2's **own checks**
(`checks.py`), so a trial compares steering only. The exception is `mpc_full`: it plans
its own speed (`Command.speed`), within the policy's limits.

```sh
pixi run sim world:=test_lot reversing:=false tracker:=lqr
pixi run controller-compare --trackers pure_pursuit lqr --tours lot --legs 1
```

## Interface

**`track`** (`track_node.py`), started by `aav_navigation`'s `navigation.launch.py tracker:=<name>`

| Item | What |
|---|---|
| Action server | `/follow_path` (`nav2_msgs/action/FollowPath`): the newest path replaces the last without a stop |
| Subscribes | `/localization/odom/local` (`nav_msgs/Odometry`); `/tf` (map -> odom, once per path) |
| Publishes | `/cmd_vel_nav` (`geometry_msgs/TwistStamped`, as nav2.yaml's `enable_stamped_cmd_vel`), `/drivers/nav/track/status` (`diagnostic_msgs/DiagnosticStatus`: loop time, missed ticks, cross track, steering) |

Results: succeeded; aborted with Nav2's codes `FAILED_TO_MAKE_PROGRESS`, `TF_ERROR`,
`INVALID_PATH`, `UNKNOWN` (no odometry); canceled.

Parameters. From `navigation.launch.py` (`vehicle.track_params`: nav2.yaml, vehicle.yaml,
ego.yaml; nothing copied):

| Name | From |
|---|---|
| `rate_hz`, `odom_topic`, `stamped` | controller_server's `controller_frequency`, `odom_topic`, `enable_stamped_cmd_vel` |
| `cruise_speed` | RPP's `desired_linear_vel`, or `max_speed:=` (sim_speed) |
| `acceleration` | the velocity smoother's `max_accel` |
| `max_brake` | vehicle.yaml `actuators.max_brake` (half is planned: `approach.planned_deceleration`) |
| `lookahead_min`, `lookahead_max`, `lookahead_time`, `transform_tolerance` | RPP's |
| `xy_goal_tolerance`, `yaw_goal_tolerance` | `general_goal_checker` |
| `progress_radius`, `progress_angle`, `progress_time` | `progress_checker` |
| `wheelbase`, `max_steering_angle` | vehicle.yaml |
| `max_steering_rate` | the lower of vehicle.yaml `steer_rate` and ego.yaml `max_steering_rate` |

From `config/track.yaml`: `lateral_accel` (1.0 m/s², slow for bends), `odom_timeout`, and
each tracker's gains (`stanley.*`, `lqr.*`, `mpc.*`, `mpc_full.*`; every tracker's are declared).
`mpc_full` also takes `cruise_speed`, `acceleration`, `max_brake`, `lateral_accel` and
vehicle.yaml's `max_accel` (the throttle).

**Live swap (sim only).** `live_swap` (default false; `sim.launch.py` sets it through
`navigation.launch.py live_swap:=true`, the car never does): `ros2 param set /track
tracker lqr` builds the new tracker between ticks, gives it the path and its place on it
(`hint`), starts lqr's and mpc's steering estimate at the last command (not the wheels'
measured angle: the node doesn't read it), resets the loop statistics and logs the swap.
Without `live_swap`, or for an unknown name, the change is refused with its reason. The
debug HUD's panel shows the tracker from the status topic.

Modules (all but `track_node.py` have no ROS):

| Module | Holds |
|---|---|
| `path.py` | `Path` (arc length, `project` -> `Foot`, `point_at`, `curvature_at`), `moved` (map into odom), `wrap` |
| `checks.py` | `GoalCheck`, `ProgressCheck`: Nav2's SimpleGoalChecker and PoseProgressChecker |
| `speed.py` | `SpeedPolicy`: cruise, `line.turn_speed_limit` for a bend ahead, `approach.Approach` to stop |
| `trackers/__init__.py` | `State`, `Limits`, `Command`, `Tracker`, `SteerLag`, `make(name, limits, model)` |
| `trackers/*.py` | the five trackers |
| `qp.py` | a small QP solver (OSQP's ADMM, numpy) for `mpc_full` |

Built from: `aav_drive_to_point`'s `line.py` (`pursuit_curvature`, `turn_speed_limit`)
and `approach.py`; `aav_ego`'s `vehicle_model.py` (`VehicleModel.lateral`, for LQR and
MPC); `aav_description`'s `vehicle.yaml`.

## Invariants

- Forward only: `navigation.launch.py` refuses a tracker with `reversing:=true`.
- Never both followers: with a tracker, controller_server's action is `follow_path_unused`.
- Every steering command within full lock and the steering rate; at v = 0, w = 0 (no
  steering at a standstill: ego holds the wheels).
- Nothing in the safety path changes: the smoother, collision_monitor, twist_mux's slot,
  the locks and e-stops all sit after `cmd_vel_nav`. The trackers don't read the costmap.
- REP-103/105: odom frame, metres, radians; + left.

## Side Effects

- Publishes `/cmd_vel_nav` and its status only while a path runs. No files, no hardware.

## Verification

```sh
pixi run build && colcon test --packages-select aav_track && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_track && python3 -m pytest -q` (`tools/test.sh`
runs it). In Gazebo: `pixi run controller-compare --trackers ...`.

| Test | Covers |
|---|---|
| `test_path.py` | the foot, cross track, bends, past the ends, into odom |
| `test_checks.py` | the goal and progress checks |
| `test_speed.py` | cruise, slow before a bend, stop at the end |
| `test_trackers.py` | each tracker on a pretend car (`sim_car.py`: ego's model, Gazebo's actuators): straight, a curve, a 1 m offset, at 1 and 3 m/s; the forward U-turn in a 14 m lane, at the planner's radius and at full lock; limits; LQR's schedule and preview; mpc_full's speed (slows to swing, stops at the end, throttle and brake); MPC's step times |
| `test_qp.py` | the QP solver against hand answers and scipy's SLSQP |
| `test_track_node.py` | the node with ROS, on a free domain: a path to success, a newer path replacing the last without a stop, an empty path refused, a live swap refused without `live_swap` and carried out mid-path with it (skips without the built workspace) |
