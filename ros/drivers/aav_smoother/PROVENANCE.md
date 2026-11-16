# Provenance

The `car-smoother` crate keeps what Nav2's `velocity_smoother` does for the speed, and
replaces its turn-rate limits with the car's steering. Rewritten in Rust's own style
(enums, `Result`, no ROS), not transliterated.

## Where it came from

| | |
|---|---|
| Repo | https://github.com/ros-planning/navigation2 |
| Package | `nav2_velocity_smoother` |
| Commit | the Jazzy commit in `vendor/ros2-src.repos` (as `aav_planner`'s and `aav_navigator`'s) |
| Licence | Apache-2.0 (Samsung Research America, Steve Macenski and the Nav2 authors) |
| Read 8 Nov 2026 | `src/velocity_smoother.cpp`: `on_configure` (parameters and their checks), `inputCommandStampedCallback`, `findEtaConstraint`, `applyConstraints`, `smootherTimer` |

## What was taken

| Ours | Nav2's |
|---|---|
| the parameter names, defaults and sign checks (`Settings::from_nav2`) | `on_configure` |
| refuse a NaN or infinite command | `validateTwist` |
| a command's time: its stamp, or now if unstamped | `inputCommandStampedCallback` |
| past `velocity_timeout`: aim for zero; once at zero, publish nothing | `smootherTimer` |
| open loop from the last command sent, closed loop from odometry | `smootherTimer` (ours takes the latest odometry, not `OdomSmoother`'s average) |
| speed limit, then accel when getting faster the same way, else decel (`slew`) | `applyConstraints` |
| the deadband applied after the last command is kept | `smootherTimer` |
| the output stamped with the command's stamp (or the smoothing time) | `stamp_smoothed_velocity_with_smoothing_time` |

## What is ours

- The turn rate from a steering angle that moves at most `steering_rate` and stops at full
  lock: `w = v tan(steer) / L` (the bicycle model, as ego's `ackermann.py`).
- The bend `w / v` kept while the speed ramps, instead of Nav2's `eta` scaling.
- `max_velocity[2]` slows the car in a tight turn instead of cutting `w`.
- The speed's limits held inside the car model's `max_accel` and `max_brake`.
- Not a lifecycle node; no dynamic parameters.

## Changes since

- 15 Nov 2026 (TODO 176): the Rust API Guidelines pass: `#[must_use]` on pure functions, doc comments, `Debug`, and the lints that keep them; no change in behaviour.
