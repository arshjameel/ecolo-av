# ros/ego

## Overview

**ego, the virtual car.** It turns the chosen `(v, ω)` into an Ackermann command the
car can actually follow, within its speed, acceleration, jerk, steering and body limits.

```
/control/cmd_vel (TwistStamped) ──▶ ego ──▶ /ego/actuator_cmd (AckermannDriveStamped, 50 Hz)
                                       ├──▶ /ego/trace (Path, base_link, 10 Hz): where it goes
                                       └──▶ /ego/stop_point (base_link): where a full brake stops it
look_ahead (beside ego) ──▶ /ego/plan_ahead, /ego/ahead: the screens' squiggly line
```

ego holds the car model (`docs/car-model.md`): it can lead for the actuators' lag
(`lag_lead`, off on the car until measured) and shows where the car is going.

## Subdirectories & Boundaries

| Package | What it is |
|---|---|
| [aav_ego](./aav_ego/README.md) | the `ego` node, its bicycle-model maths (`ackermann.py`, `dynamics.py`) and a linear vehicle model for tuning |
| [aav_ego_rs](./aav_ego_rs/README.md) | a challenger: `ego` and `look_ahead` in Rust, one process, the same topics and answers (`ego_impl:=rust`) |

## Invariants

- Ackermann starts here: everything upstream is `TwistStamped`.
- A command older than 0.3 s means stop at full deceleration. Braking is never softened.
- Speed, acceleration, jerk, steering-rate, lateral and timeout limits come from
  `aav_ego/config/ego.yaml`. `vehicle.yaml` gives the wheelbase, steering angle, reverse
  speed and body. `max_speed:=` can only lower the cap.

## Side Effects

- Publishes `/ego/actuator_cmd` all the time, `/ego/trace` and `/ego/stop_point` at 10 Hz;
  `look_ahead` publishes `/ego/plan_ahead` and `/ego/ahead`.

## Verification

- Run: `pixi run test` (or `colcon test --packages-select aav_ego`).
