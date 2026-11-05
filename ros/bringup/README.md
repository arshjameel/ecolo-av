# ros/bringup

## Overview

How the whole stack starts, and what the car is. Every profile launches the same graph;
only the backend and the data source change.

## Subdirectories & Boundaries

| Package | What it is |
|---|---|
| [aav_bringup](./aav_bringup/README.md) | the launch profiles (below), `stack.launch.py target:=`, Foxglove, per-computer parts for the car |
| [aav_description](./aav_description/README.md) | **`config/vehicle.yaml`, the only place car measurements live**; the URDF; `vehicle.py`, which works out Nav2, ego and spine parameters from it |
| [aav_run_info](./aav_run_info/README.md) | `run_info`: which algorithms the run uses (planner, path follower, obstacles, localizers), on `/system/algorithms`, for the cockpit |

| Profile | Command | Backend | Source |
|---|---|---|---|
| turtle | `pixi run turtle` | turtlesim | SIM |
| sim | `pixi run sim` | Gazebo | SIM |
| bench | `pixi run bench` | spine_sim + mocks | BENCH |
| car | `pixi run car <machine>` | legacy UART bridge or spine_bridge | REAL |
| carla | `pixi run carla-car` | CARLA | SIM |
| replay | `ros2 launch aav_bringup replay.launch.py` + `ros2 bag play` | none (drives nothing) | REPLAY |

## Invariants

- Exactly one backend runs; `source:=` must match the target.
- Stand-in profiles run through `tools/ros.sh --local`; `pixi run car` does not.
- Never copy a car measurement: load it from `vehicle.yaml`.
- REP-103 and REP-105; `base_link` on the ground under the rear axle.

## Side Effects

- Starts every node of a profile. `car` opens real devices (serial, CAN, USB, UDP).
- foxglove_bridge on port 8765 (localhost for stand-ins; watch-only on the car).

## Verification

- Run: `pixi run test` (the launch tests need it, not just `tools/test.sh`), then
  `pixi run check <turtle|sim|car|replay>` against a running profile.
