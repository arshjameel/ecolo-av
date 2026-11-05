# aav_ego

Ego is the virtual car between the command mux and the backends. It turns the one
velocity command, `/control/cmd_vel` (v, w), into one Ackermann command,
`/ego/actuator_cmd` (speed, steering angle), within the car's limits.

```
twist_mux -> /control/cmd_vel -> ego -> /ego/actuator_cmd (50 Hz, always)
          -> output mux (aav_io_mux) -> one backend: spine, legacy serial, Gazebo, turtlesim, CARLA
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
| Publishes | `/ego/actuator_cmd` (`ackermann_msgs/AckermannDriveStamped`) at `rate_hz`: speed (m/s), steering_angle (rad, + left), acceleration (m/s²: the limit used; while speeding up with `max_jerk` > 0, the eased acceleration now), steering_angle_velocity (`max_steering_rate`) |

Launch arguments:

| Name | Meaning |
|---|---|
| `max_speed` | m/s, a lower forward cap than `ego.yaml`'s (a first drive on stands). Must be above 0 and at most `ego.yaml`'s |
| `sim_max_speed` | m/s, a higher cap, up to 5.0, only with `use_sim_time:=true` |

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

From `ros/bringup/aav_description/config/vehicle.yaml` (the launch file passes them):
`wheelbase`, `max_steering_angle`, `max_reverse_speed`, and the body (`mass`, `track`,
`cg_height`, `tyre_grip`, `full_throttle_accel`, `full_brake_decel`).

Modules:

| Module | Holds |
|---|---|
| `ackermann.py` | `Limits`, `EgoModel.step(cmd, dt)` -> (speed, steering, accel); `steering_angle`, `turn_rate`, `slew_speed`, `ease_speed`, `curvature`; `LIVE_LIMITS`, `live_change()` |
| `dynamics.py` | `Body`; `speed_up_limit`, `braking_limit`, `lateral_limit`, `curve_speed`, `stopping_distance`, `energy`, `describe()` (start-up log lines) |
| `vehicle_model.py` | the car as a linear model (transfer functions, state space, margins), for tuning and the report: `VehicleModel`, `from_vehicle_yaml()`, `summary()`. Never drives the car. Used by `tools/vehicle_model.py` (`pixi run python3 tools/vehicle_model.py`, `docs/vehicle-model.md`) |
| `ego_node.py` | the ROS node `Ego` |

## Invariants

- `ackermann.py`, `dynamics.py` and `vehicle_model.py` have no ROS imports.
- Bicycle model about the rear axle (`base_link`): `steering = atan(L w / v)`, clamped
  to `max_steering_angle`. Below `min_speed_for_steering` the wheels hold: a car can't
  turn on the spot.
- A capped speed keeps the requested curvature `w / v`.
- No fresh command (older than `cmd_timeout` since it arrived) means stop: speed slews
  to 0 at `max_deceleration`, steering holds. Header stamps only log a warning when
  more than 0.5 s off (check chrony).
- A change of direction passes through a stop.
- Jerk limits only speeding up. Braking is never softened, and never weaker for a heavy
  car; a turn slows the car rather than steering less.
- Turn speed stays under `max_lateral_accel` and under half of what slides or tips the car.
- Publishes every tick, at 50 Hz, even with no command.
- Only `max_steering_rate` and `payload` change while running; others are refused with
  "restart ego". `max_speed:=` only lowers the cap; `sim_max_speed:=` only in a simulator.
- Ackermann appears only from here on (`/ego/actuator_cmd`); drivers send `TwistStamped`.

## Side Effects

- Publishes `/ego/actuator_cmd`; logs the limits and what the body allows at start-up.
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
| `test_ego_launch.py` | the `max_speed` and `sim_max_speed` caps |
