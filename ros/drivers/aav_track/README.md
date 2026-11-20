# aav_track

Trial path followers, beside `mppi_lag` (`docs/trials.md`). The `track` node serves
FollowPath in `mppi_lag`'s place: same planner, same goals, same safety chain after it.
Only one follower runs: chosen at launch with `tracker:=`.

```
navigator --FollowPath--> track --cmd_vel_nav--> velocity_smoother
   -> stop zone -> /drivers/nav/cmd_vel (twist_mux 20) -> ego
```

## Overview

| `tracker:=` | Steers by | Module |
|---|---|---|
| `mppi_lag` (default) | `aav_mppi_lag` (Rust); this node doesn't run | - |
| `pure_pursuit` | an arc to a carrot on the path: the trial's control | `trackers/pure_pursuit.py` |
| `stanley` | heading error + atan(k e / v), at the front axle | `trackers/stanley.py` |
| `lqr` | -K x about the path's bend, K by speed and bend (Riccati), + a 2 s preview of the bends ahead | `trackers/lqr.py` |
| `mpc` | a 1.5 s plan of steering, within full lock and the steering rate (scipy) | `trackers/mpc.py` |
| `mpc_full` | a 9 m plan of speed and steering together, within the steering, throttle and brake (its own speed) | `trackers/mpc_full.py` |
| `mpc_lag` | a 1.5 s plan of steering and set speed through the car's lags: the spine's speed loop, throttle, brake, the steering motor and linkage, dead times (its own speed; **lag-aware**) | `trackers/mpc_lag.py` |

Every tracker drives with **one speed policy** (`speed.py`) and Nav2's **own checks**
(`checks.py`), so a trial compares steering only. The exceptions are `mpc_full` and
`mpc_lag`: they plan their own speed (`Command.speed`), within the policy's limits.

```sh
pixi run sim world:=test_lot tracker:=lqr          # reverses too (TODO 174)
pixi run measure controllers --trackers pure_pursuit lqr --tours lot --legs 1
```

## Interface

**`track`** (`track_node.py`), started by `aav_navigation`'s `navigation.launch.py tracker:=<name>`

| Item | What |
|---|---|
| Action server | `/follow_path` (`nav2_msgs/action/FollowPath`): the newest path replaces the last without a stop |
| Subscribes | `/localization/odom/local` (`nav_msgs/Odometry`); `/tf` (map -> odom: held, moved toward the newest at most 0.1 m/s and 1°/s at the car, `slow_frame.py`, TODO 186; a map path stays in map and the car's odometry is brought there by the held one); `/world/road_ahead` (`aav_interfaces/RoadAhead`, `road_ahead_topic`: only with `behaviour:=true` and a lane map; each new path kept inside the lane's soft edges while it is 0.5 s fresh) |
| Publishes | `/cmd_vel_nav` (`geometry_msgs/TwistStamped`, as nav2.yaml's `enable_stamped_cmd_vel`), `/drivers/nav/track/status` (`diagnostic_msgs/DiagnosticStatus`: loop time, missed ticks, cross track, steering), `/drivers/nav/lag_aware` (`std_msgs/Bool`, latched: true while the tracker models the lags itself, `mpc_lag`, so ego's lead stays off; false otherwise and as the node stops) |

Results: succeeded; aborted with Nav2's codes `FAILED_TO_MAKE_PROGRESS`, `TF_ERROR`,
`INVALID_PATH`, `UNKNOWN` (no odometry); canceled.

Parameters. From `navigation.launch.py` (`vehicle.track_params`: nav2.yaml, vehicle.yaml,
ego.yaml; nothing copied):

| Name | From |
|---|---|
| `road_ahead_topic`, `lane_margin` | the launch's: `/world/road_ahead` with a lane map, else empty (off); 0.1 m kept inside each soft edge |
| `rate_hz`, `odom_topic`, `stamped` | `nav2.yaml`'s `controller_server` block: `controller_frequency`, `odom_topic`, `enable_stamped_cmd_vel` |
| `cruise_speed` | RPP's `desired_linear_vel`, or `max_speed:=` (sim_speed) |
| `acceleration`, `deceleration` | the velocity smoother's `max_accel`, `max_decel` |
| `max_brake` | vehicle.yaml `actuators.max_brake` (half is planned: `approach.planned_deceleration`) |
| `lookahead_min`, `lookahead_max`, `lookahead_time`, `transform_tolerance` | RPP's |
| `xy_goal_tolerance`, `yaw_goal_tolerance` | `general_goal_checker` |
| `progress_radius`, `progress_angle`, `progress_time` | `progress_checker` |
| `wheelbase`, `max_steering_angle` | vehicle.yaml |
| `max_steering_rate` | the lower of vehicle.yaml `steer_rate` and ego.yaml `max_steering_rate` |
| `max_reverse_speed` | vehicle.yaml's with `reversing:=true`, else 0 (TODO 174) |
| `mpc_lag.speed_loop` | the spine's speed loop profile `mpc_lag` predicts: the sim's `speed_loop:=` (`default`, `smooth`, `firm`; `lqi` refused at launch with `tracker:=mpc_lag`); never in `track.yaml` (TODO 163) |

From `config/track.yaml`: `lateral_accel` (1.0 m/s², slow for bends), `odom_timeout`, the
map correction (TODO 186: `map_correction_speed` 0.1 m/s, 0 the old way;
`map_correction_turn_rate` 1°/s; `map_correction_snap_distance` 2 m and
`map_correction_snap_angle` 10°, taken at once past them; the launch's bool
`map_correction`, false with `map_correction:=newest`: the old way), and
each tracker's gains (`stanley.*`, `lqr.*`, `mpc.*`, `mpc_full.*`, `mpc_lag.*`; every
tracker's are declared). `mpc_full` also takes `cruise_speed`, `acceleration`, `max_brake`,
`lateral_accel` and vehicle.yaml's `max_accel` (the throttle). `mpc_lag` takes those,
`deceleration`, and vehicle.yaml's whole `actuators:` section (`car_model.ActuatorConfig`).

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
| `gears.py` | `stretches`: the path cut at its cusps; `Gears`: which stretch, the next taken at the cusp (within 0.3 m and nearly still, or standing 2 s within 1.5 m); `mirror_state`: a backwards stretch as the tracker sees it (TODO 174) |
| `lane.py` | `keep_in_lane`: each path point moved sideways inside the road ahead's soft edges (TODO 169) |
| `path.py` | `Path` (arc length, `project` -> `Foot`, `point_at`, `curvature_at`), `moved` (map into odom), `wrap` |
| `slow_frame.py` | `SlowFrame`: the held map -> odom, slewed toward the newest within `SlewLimits` (the same rule as `aav_mppi_lag`'s `map_correction.rs`) |
| `checks.py` | `GoalCheck`, `ProgressCheck`: Nav2's SimpleGoalChecker and PoseProgressChecker |
| `speed.py` | `SpeedPolicy`: cruise, `line.turn_speed_limit` for a bend ahead, `approach.Approach` on the stopping law (`aav_description` `stop_law.py`, since 7 Nov) to stop |
| `trackers/__init__.py` | `State`, `Limits`, `Command`, `Tracker`, `SteerLag`, `make(name, limits, model)` |
| `trackers/*.py` | the six trackers |
| `qp.py` | a small QP solver (OSQP's ADMM, numpy) for `mpc_full` and `mpc_lag` |
| `lag_model.py` | the shared car model's maths (`aav_description` `car_model.py`) again, on arrays and in the path's frame, for `mpc_lag`'s plan: `step()`; its numbers and speed loops are `car_model`'s (`CarParams` is `car_model.ActuatorConfig`). `mpc_lag`'s estimate runs `car_model.step` itself |

Built from: `aav_drive_to_point`'s `line.py` (`pursuit_curvature`, `turn_speed_limit`)
and `approach.py`; `aav_ego`'s `vehicle_model.py` (`VehicleModel.lateral`, for LQR and
MPC); `aav_description`'s `vehicle.yaml`.

## Invariants

- Reverses where the plan says, with `reversing:=true` (TODO 174): the path cut at its
  cusps (`gears.py`), a backwards stretch mirrored for the tracker (the car turned round,
  speed and steering negated), at `max_reverse_speed` at most. Without it
  (`max_reverse_speed` 0) a path that reverses is refused (`INVALID_PATH`).
- Never both followers: with a tracker, `mppi_lag` doesn't start.
- Every steering command within full lock and the steering rate; at v = 0, w = 0 (no
  steering at a standstill: ego holds the wheels).
- Nothing in the safety path changes: the smoother, the stop zone, twist_mux's slot,
  the locks and e-stops all sit after `cmd_vel_nav`. The trackers don't read the costmap.
- REP-103/105: odom frame, metres, radians; + left.

## Side Effects

- Publishes `/cmd_vel_nav` and its status only while a path runs. No files, no hardware.

## Verification

```sh
pixi run build && colcon test --packages-select aav_track && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_track && python3 -m pytest -q` (`tools/test.sh`
runs it). In Ravensim: `pixi run measure controllers --trackers ...`.

| Test | Covers |
|---|---|
| `test_lane.py` | inside left alone, over either edge moved in, unknown edges and far points kept, a lane narrower than the car, a line heading north |
| `test_path.py` | the foot, cross track, bends, past the ends, into odom |
| `test_slow_frame.py` | the slew at its speed and turn rate, about the car; a snap; noise held to the rate; off is the old way; the car in map gives the same foot as the path in odom |
| `test_checks.py` | the goal and progress checks |
| `test_gears.py` | the cuts at cusps, the mirror, the cusp's rules; every tracker backing up a straight line, and through a cusp and backwards round a bend (`sim_car.follow_geared`, TODO 174) |
| `test_speed.py` | cruise, slow before a bend, stop at the end |
| `test_trackers.py` | each tracker on a pretend car (`sim_car.py`: ego's model, the simulator's actuators): straight, a curve, a 1 m offset, at 1 and 3 m/s; the forward U-turn in a 14 m lane, at the planner's radius and at full lock; limits; LQR's schedule and preview; mpc_full's speed (slows to swing, stops at the end, throttle and brake); MPC's step times |
| `test_qp.py` | the QP solver against hand answers and scipy's SLSQP |
| `test_lag_model.py` | the model's spec (lags, rate, deadband, brake, bicycle); the same states as `car_model.step`, step for step (both speed loops, every branch) |
| `test_trackers.py` (mpc_lag) | no speed overshoot; the 2 m/s U-turn with vehicle.yaml's lags and with them doubled (`follow(lag_scale=2)`); the stop; the dead-time buffer; step time |
| `test_track_node.py` | the node with ROS, on a free domain: a path to success, a newer path replacing the last without a stop, a map path by the held map -> odom (a 0.3 m jump taken in at 0.1 m/s; TODO 186), an empty path refused, a live swap refused without `live_swap` and carried out mid-path with it (skips without the built workspace) |
