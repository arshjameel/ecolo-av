# ros/sim

## Overview

**Stand-ins for the car.** Each backend takes ego's commands from
`/control/<target>/actuator_cmd` and answers on the same `/vehicle/*` and `/sensors/*`
topics as the real car, so everything upstream runs unchanged.

## Subdirectories & Boundaries

| Package | Target | What it is |
|---|---|---|
| [aav_turtlesim](./aav_turtlesim/README.md) | turtle | turtlesim driven like a car: the quickest whole-loop test |
| [aav_sim_models](./aav_sim_models/README.md) | gazebo | Gazebo Harmonic: worlds, models, an actuator model and one node per sensor, with faults to set on purpose |
| [aav_simulator](./aav_simulator/README.md) | gazebo (`simulator:=own`, the default) | our own simulator in Rust, in `gz sim`'s place: Gazebo's worlds, the car, the lidar, radar, IMU, GPS and clock on the bridges' topics; the cameras through `aav_sim_cameras` |
| [aav_spine_sim](./aav_spine_sim/README.md) | bench | the real `spine-core` logic (Rust) on the CAN frames, driving a car model or Gazebo's car |
| [aav_bench](./aav_bench/README.md) | bench | pretend sensors that match the real drivers' topics, types, frames and QoS |
| [aav_carla](./aav_carla/README.md) | carla | a CARLA car behind the real car's topics (never yet run against a real server) |
| [aav_sim_cameras](./aav_sim_cameras/README.md) | sim | our own simulator's cameras: Bevy draws each car camera offscreen, on Gazebo's camera topics. Its own Cargo workspace, not a colcon package |

## Invariants

- Exactly one backend takes ego's commands at a time.
- A stale command means speed 0: after 0.2 s in turtlesim, Gazebo and CARLA, and after
  0.1 s on the bench (spine-core's own watchdog).
- Every stand-in announces SIM or BENCH on `/system/source`, never REAL.
- Stand-ins run through `tools/ros.sh --local`, so they never reach a real car.
- Logic modules have no ROS imports.
- Limits come from `vehicle.yaml`, except the bench spine's: `spine_sim` runs spine-core's
  built-in limits (`SpineConfig::default()`: 3.0 m/s, 1.5 m/s², 0.1 s watchdog). It takes
  only the car's size and steering limit from `vehicle.yaml`, for the model car.

## Side Effects

- Starts simulators (gz sim, turtlesim) and publishes `/clock` where sim time is used.
- `aav_carla` talks TCP to the CARLA server; `mock_lidar` sends UDP to localhost.

## Verification

- Run: `pixi run test`, then `pixi run sim-check` (with no GPU, as on oirp-spss: `tools/software_gl.sh pixi run sim-check`).
