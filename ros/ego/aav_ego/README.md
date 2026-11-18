# aav_ego

Ego is the virtual car between the command mux and the backends. It turns the one
velocity command, `/control/cmd_vel` (v, w), into one Ackermann command,
`/ego/actuator_cmd` (speed, steering angle), within the car's limits.

```
twist_mux -> /control/cmd_vel -> ego -> /ego/actuator_cmd (50 Hz, always)
          -> output mux (aav_io_mux) -> one backend: spine, legacy serial, Ravensim, turtlesim, CARLA
```

## Overview

Every backend gets the same command through the output mux, so the drivers never know
which car they drive. Ego holds the limits every driver gets, whatever it asks for: top
speed, reverse speed, acceleration, jerk, steering angle and rate, and a body model
(mass, load, tyres) that slows the car for turns.

## Interface

| Item | What |
|---|---|
| Node `ego` | `ros2 launch aav_ego ego.launch.py [use_sim_time:=false] [max_speed:=0.3] [sim_max_speed:=]` |
| Subscribes | `/control/cmd_vel` (`geometry_msgs/TwistStamped`; plain `Twist` when the parameter `use_stamped` is false, in `ego.yaml`: not a launch argument) |
| | `/vehicle/odom` (`nav_msgs/Odometry`): the measured speed, for the trace's twin |
| | `/drivers/<d>/lag_aware` (`std_msgs/Bool`, latched) and `/drivers/<d>/cmd_vel`, for each `d` in `lag_aware_drivers`: is a lag-aware driver driving? |
| | `/drivers/<d>/cmd_vel` for each `d` in `pedal_drivers`, and `/safety/estop` (`std_msgs/Bool`): the pedal |
| | `/world/road_ahead` (`aav_interfaces/RoadAhead`) and `/localization/odom/global`, only with `road_ahead` true: an automatic driver's forward speed (`road_ahead_drivers`) never above what the road ahead allows where the car is; no road, or a stale pose, allows 0. People are never trimmed |
| Publishes | `/ego/actuator_cmd` (`ackermann_msgs/AckermannDriveStamped`) at `rate_hz`: speed (m/s), steering_angle (rad, + left), acceleration (m/s²: the limit used; while speeding up with `max_jerk` > 0, the eased acceleration now), steering_angle_velocity (`max_steering_rate`) |
| | `/ego/trace` (`nav_msgs/Path`, frame `base_link`) at `trace_rate_hz`: where the car goes over `trace_horizon` s |
| | `/ego/stop_point` (`geometry_msgs/PoseStamped`, frame `base_link`, with each trace): where a full brake now stops the car |
| Node `look_ahead` | started by `ego.launch.py` (`look_ahead:=true`, the default): the squiggly line's numbers for the screens. Subscribes `/ego/trace`, `/ego/stop_point`, `/drivers/nav/local_plan`, `/plan`, `/local_costmap/costmap`, `/world/road_ahead`, TF; publishes `/ego/plan_ahead` (Path, base_link) and `/ego/ahead` (DiagnosticStatus: gap_m, plan, stop_m, goal_m, obstacle_m, room_m, front_m, rear_m, half_width_m, road_mps, road_m, road_why). Parameters in its docstring (`look_ahead_node.py`) |

Launch arguments:

| Name | Meaning |
|---|---|
| `max_speed` | m/s, a lower forward cap than `ego.yaml`'s (a first drive on stands). Must be above 0 and at most `ego.yaml`'s |
| `sim_max_speed` | m/s, a higher cap, up to 5.0, only with `use_sim_time:=true` |
| `look_ahead` | `true`: also start `look_ahead` (the screens' squiggly line); `false`: ego alone |
| `road_ahead` | `false`; `true`: hold navigation to `/world/road_ahead` (`sim.launch.py` sets it with `behaviour:=true`, its writer) |
| `speed_loop` | `default`: the spine's speed loop profile the car runs (`default`, `smooth`, `lqi`, `firm`), set as `trace_speed_loop` over `ego.yaml`'s, so the trace's twin assumes it; `sim.launch.py` and `bench.launch.py` pass theirs (TODO 163). An unknown name is refused |
| `ego_impl` | `rust`: [`aav_ego_rs`](../aav_ego_rs/README.md), ego and look_ahead in one Rust process with the same names, topics and answers (the default since Ben, 16 Nov; `sim.launch.py` and `car.launch.py` pass it on); `python`: this package's two nodes |

Parameters. From `config/ego.yaml`:

| Name | Default | Meaning |
|---|---|---|
| `max_speed` | 1.0 m/s | forward cap |
| `max_acceleration` | 1.0 m/s² | speeding up |
| `max_deceleration` | 2.0 m/s² | slowing down, also when commands go stale |
| `max_jerk` | 2.0 m/s³ | speeding up only; 0 = no limit |
| `max_steering_rate` | 0.6 rad/s | live: may change while running (0.05 to 5.0) |
| `min_speed_for_steering` | 0.05 m/s | below this the wheels hold |
| `payload` | 0 kg | live: people and cargo aboard (0 to 300) |
| `max_lateral_accel` | 2.0 m/s² | sideways comfort limit in a turn |
| `cmd_timeout` | 0.3 s | since the last command **arrived**, then stop |
| `rate_hz`, `frame_id`, `use_stamped` | 50, `base_link`, true | |
| `lag_lead` | `auto` | live. The lag lead: `auto` in a simulator only (off on the car, SAFE); `on` the car too, once tests 3.10-4.5 measured the lags; `off` never. Never while a lag-aware driver drives |
| `lag_lead_filter`, `lag_lead_max_speed`, `lag_lead_max_steer` | 0.1 s, 0.3 m/s, 0.1 rad | the request's filter; the most the lead adds or takes |
| `lag_aware_drivers` | scripted, point, nav, ai | whose `/drivers/<d>/lag_aware` ego watches |
| `pedal_drivers`, `pedal_let_off`, `pedal_full` | joy, rc, keyboard; 1.0 m/s²; 1.0 m/s | the pedal (`pedal.py`): while a person drives, the stick is an accelerator and a brake. Let off slows (let go: `pedal_let_off`), pulled back brakes by how far (all the way: `max_deceleration`), then reverses at a standstill; `pedal_full` is what a full stick asks. Off during an e-stop (`/safety/estop`), and for a stale command or a turn's cap. `[]`: off |
| `road_ahead`, `road_ahead_drivers`, `road_pose_timeout` | false, nav and point, 1.0 s | the road ahead's limit (`road_limit.py`): on or off, whose commands (each also in `lag_aware_drivers`), how old the pose may be |
| `trace_horizon`, `trace_rate_hz`, `trace_speed_loop` | 2.0 s, 10 Hz, `default` | the trace; 0 Hz: none |
| `vehicle_yaml` | `''` | the car model's numbers (`''`: `aav_description`'s) |

From `ros/bringup/aav_description/config/vehicle.yaml` (the launch file passes them):
`wheelbase`, `max_steering_angle`, `max_reverse_speed`, and the body (`mass`, `track`,
`cg_height`, `tyre_grip`, `full_throttle_accel`, `full_brake_decel`).

Modules:

| Module | Holds |
|---|---|
| `ackermann.py` | `Limits`, `EgoModel.step(cmd, dt)` -> (speed, steering, accel); `steering_angle`, `turn_rate`, `slew_speed`, `ease_speed`, `curvature`; `LIVE_LIMITS`, `live_change()` |
| `dynamics.py` | `Body`; `speed_up_limit`, `braking_limit`, `lateral_limit`, `curve_speed`, `stopping_distance`, `energy`, `describe()` (start-up log lines) |
| `vehicle_model.py` | the car as a linear model (transfer functions, state space, margins), for tuning and the report: `VehicleModel`, `from_vehicle_yaml()`, `summary()`. Never drives the car. Used by `tools/vehicle_model.py` (`pixi run python3 tools/vehicle_model.py`, `docs/vehicle-model.md`) |
| `pedal.py` | `Pedal`, `slow_rate(pedal, current, target, max_deceleration)`: how hard a person's let-off or pull-back slows |
| `lag_lead.py` | `LagLead` (the lead), `LeadTimes`, `LeadLimits`, `lead_wanted(mode, sim, aware)`, `LagAwareWatch` (whose command is it?), `command_key` |
| `road_limit.py` | the road ahead's limit: `allowed(strip, pose, age, timeout, curve, base)`, `cap(cmd, limit)` (keeps the curvature) |
| `trace_ahead.py` | `Twin` (ego's copy of the car), `trace(...)` -> poses in `base_link`, `ego_commands` |
| `stop_ahead.py` | `stop_point(cfg, state)` -> `StopPoint` (where a full brake now stops, base_link; the car model), `s_of_v` (the design's S(v)) |
| `look_ahead.py` | the squiggly line's maths: `crop_ahead`, `gap`, `grade`, `to_goal`, `Grid`, `first_hit`, `extend`, `describe` |
| `look_ahead_node.py` | the ROS node `look_ahead` |
| `ego_node.py` | the ROS node `Ego` |

## Invariants

- `ackermann.py`, `dynamics.py`, `vehicle_model.py`, `lag_lead.py`, `trace_ahead.py`,
  `stop_ahead.py`, `road_limit.py` and `look_ahead.py` have no ROS imports. The car model is `aav_description`'s `car_model.py` (`docs/car-model.md`).
- The lag lead runs before every limit: `max_speed`, the steering limit and rate, the slews,
  the jerk and the body still apply after it. It never softens a slow-down, never changes
  direction, never touches a stop, and never runs while a lag-aware driver drives.
- Bicycle model about the rear axle (`base_link`): `steering = atan(L w / v)`, clamped
  to `max_steering_angle`. Below `min_speed_for_steering` the wheels hold: a car can't
  turn on the spot.
- A capped speed keeps the requested curvature `w / v`.
- No fresh command (older than `cmd_timeout` since it arrived) means stop: speed slews
  to 0 at `max_deceleration`, steering holds. Header stamps only log a warning when
  more than 0.5 s off (check chrony).
- A change of direction passes through a stop.
- The pedal grades only a person's own slowing: a stop, an e-stop and a turn's cap always brake at `max_deceleration`.
- Jerk limits only speeding up. Braking is never softened, and never weaker for a heavy
  car; a turn slows the car rather than steering less.
- Turn speed stays under `max_lateral_accel` and under half of what slides or tips the car.
- Publishes every tick, at 50 Hz, even with no command.
- Only `max_steering_rate`, `payload` and `lag_lead` change while running; others are
  refused with "restart ego". `max_speed:=` only lowers the cap; `sim_max_speed:=` only in a simulator.
- Ackermann appears only from here on (`/ego/actuator_cmd`); drivers send `TwistStamped`.

## Side Effects

- Publishes `/ego/actuator_cmd`, `/ego/trace` and `/ego/stop_point` (`look_ahead`: `/ego/plan_ahead`, `/ego/ahead`); logs the limits, what the body allows
  and the lag lead's numbers at start-up, and each time the lead turns on or off.
- No files, no hardware.

## Verification

```sh
pixi run build && colcon test --packages-select aav_ego && colcon test-result --verbose
```

Quick, without ROS: `cd ros/ego/aav_ego && python3 -m pytest -q` (`tools/test.sh` runs
it; `test_ego_launch.py` skips without ROS). `pixi run test` runs everything.

| Test | Covers |
|---|---|
| `test_ackermann.py` | the conversion, slew, jerk, live changes |
| `test_dynamics.py` | the body's limits |
| `test_vehicle_model.py` | the linear model (needs numpy, scipy) |
| `test_ego_launch.py` | the `max_speed` and `sim_max_speed` caps; `look_ahead` on by default, and off |
| `test_pedal.py` | the pedal: let off more slows more, back harder brakes harder, full back stops then reverses, let go rolls to a stop, a stale command still brakes fully |
| `test_lag_lead.py` | the lead: when it runs, its limits, ego's limits after it, it helps on the car model |
| `test_road_limit.py` | the road ahead's limit: no road or a stale pose allow 0; it falls to 0 at the end; the curvature kept; reverse untouched |
| `test_trace_ahead.py` | the trace and the twin |
| `test_stop_ahead.py` | the stop point: parked, S(v), dead time, turning, backwards, no brake |
| `test_look_ahead.py` | the plan cut ahead, the gap and its grade, the goal, the costmap's first obstacle |
| `test_ego_node.py` | the node in this process: `/ego/trace`; the lead off for a lag-aware driver (needs ROS) |
