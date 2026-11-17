# ros

## Overview

The ROS 2 Jazzy packages, one folder per area of the picture (Arsh, 25 Oct). colcon
finds every package under here. How they connect: `docs/how-it-fits.md`.

```
sources ─▶ localization, perception ─▶ drivers ─▶ command_mux ─▶ ego ─▶ io_mux ─▶ backend
 (sensors)                                         (twist_mux)            (one)    can_bridge = the car
                                                                                    sim = a stand-in
                   safety: locks on command_mux · hud: screens · bringup: launch + vehicle.yaml
```

## Subdirectories & Boundaries

| Area | What it does |
|---|---|
| [sources](./sources/README.md) | sensor drivers: IMU, GPS, radar, lidar, cameras, battery |
| [localization](./localization/README.md) | two robot_localization EKFs: `map → odom → base_link`; GPS heading |
| [perception](./perception/README.md) | objects, stop signs, lanes, a lidar scan, radar clusters |
| [drivers](./drivers/README.md) | everything that proposes a command: joystick, RC, tests, scripted, drive to point, Nav2, language; and street routes as GPS waypoints (`aav_routes`) |
| [command_mux](./command_mux/README.md) | twist_mux: one command for ego, plus the locks |
| [ego](./ego/README.md) | the virtual car: `TwistStamped` in, Ackermann out, within the car's limits |
| [io_mux](./io_mux/README.md) | exactly one backend, and the real car only on real data |
| [can_bridge](./can_bridge/README.md) | the real car's backend: CAN to the spine, or UART to the 2025-26 ESP32 |
| [sim](./sim/README.md) | stand-in backends: turtlesim, Ravensim, bench mode, CARLA; and `spine_sim` (`aav_spine_sim`), the spine's real logic for bench mode and `sim spine:=true` |
| [safety](./safety/README.md) | the sensor watchdog and ghost obstacles |
| [hud](./hud/README.md) | the in-car screens: the HUD and the map window (Rust, raylib) |
| [bringup](./bringup/README.md) | the launch profiles, and `vehicle.yaml`: the only place car measurements live |
| [ecolo_msgs](./ecolo_msgs/README.md) | the team's own message and action types |

## Global Invariants

- Topic names start with their area: `/<area>/<device or node>/<thing>`.
  `docs/interfaces.yaml` is the contract; change it first.
- Drivers publish `TwistStamped` on `/drivers/<driver>/cmd_vel`. Ackermann appears only
  after ego (`/ego/actuator_cmd`). Exactly one backend runs at a time.
- REP-103 and REP-105: metres, radians, seconds; x forward, y left; positive steering = left.
- A node's maths lives in a module with no ROS imports, with its own tests.
- Each node's module docstring lists what it subscribes, publishes, serves and calls.
- A person on the joystick always outranks the computer. Nothing in software clears the e-stop on its own.

## Global Side Effects

- On the car: the sensors' serial, USB and UDP ports, CAN on `can0`, and the actuators.
- In stand-ins: simulator processes, `/clock`, localhost-only DDS (`tools/ros.sh --local`).

## Verification

- Run: `pixi run test` (builds, then `colcon test` and the style check), and
  `pixi run check <turtle|sim|car|replay>` against a running profile.
