# aav_smoother

## Overview

Our own velocity smoother, for a car, in Rust: a trial beside Nav2's `velocity_smoother`
(`smoother:=own`; the default stays Nav2's). The page: [docs/own-smoother.md](../../../docs/own-smoother.md).

```
follower -> cmd_vel_nav -> car_smoother -> /drivers/nav/cmd_vel (or -> collision_monitor)
```

Nav2's smoother limits `v` and `w` each on its own, like a robot that turns on the spot.
Ours keeps Nav2's speed limits, timeout, deadband and feedback, and steers like the car:

| | Nav2's | Ours |
|---|---|---|
| Turn rate | `max_velocity[2]`, changed at `max_accel[2]` | `w = v tan(steer) / L`: the wheels move at most the steering rate, never past full lock |
| Speed and turn together | scaled together (`scale_velocities`) | the bend `w / v` kept; it changes only as fast as the wheels turn |
| A tight turn | `w` cut | the car slowed (under `max_velocity[2]`) |
| Speed limits | nav2.yaml's | nav2.yaml's, and never above the car's `max_accel`, `max_brake` |

## Interface

| Item | What |
|---|---|
| Node | `car_smoother`, run as `velocity_smoother` (Nav2's name: it reads the same block of the tuned `nav2.yaml`). Not a lifecycle node |
| Subscribes | `cmd_vel` (TwistStamped; remapped to `cmd_vel_nav`); `odom_topic` (Odometry) only with `feedback: CLOSED_LOOP` |
| Publishes | `cmd_vel_smoothed` (TwistStamped; remapped to `/drivers/nav/cmd_vel` or `/control/cmd_vel_smoothed`) |
| Nav2's parameters | `smoothing_frequency`, `feedback`, `max_velocity`, `min_velocity`, `max_accel`, `max_decel`, `deadband_velocity`, `velocity_timeout`, `odom_topic`, `enable_stamped_cmd_vel` (true only), `stamp_smoothed_velocity_with_smoothing_time` |
| The car's | `vehicle_yaml` (read through the car model, `car-model`), `steering_rate` (rad/s: the tighter of `actuators.steer_rate` and ego's `max_steering_rate`), `min_speed_for_steering` (ego's) — `vehicle.smoother_params()` |

| Crate | What |
|---|---|
| `crates/car-smoother` | the maths, no ROS: `Smoother`, `Settings` (`from_nav2`), `Car`, `slew` |
| `crates/smoother-node` | the rclrs node, binary `car_smoother` |

## Invariants

- Same topics, parameters and message types as Nav2's; nothing downstream can tell.
- `|w| <= |v| tan(max_steering) / L`, always; the wheels move at most `steering_rate`.
- A command older than `velocity_timeout`: slow to a stop, then publish nothing (Nav2's).
- Closed loop: odometry older than `velocity_timeout` is not used; the last speed sent stands in (Nav2's keeps the last reading for ever).
- Below `min_speed_for_steering` the wheels hold, as ego holds them.
- Never on the car: `car.launch.py` always passes `smoother:=nav2`.

## Side Effects

Publishes the smoothed command; reads `vehicle.yaml`.

## Verification

```sh
cd ros/drivers/aav_smoother && cargo test -p car-smoother     # also in tools/test.sh
pixi run measure controllers --stacks own --smoothers nav2 own --tours lot
```
