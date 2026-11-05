# aav_bringup

## Overview

Starts the whole stack. One launch file per **profile**: who plays the car (turtlesim,
Gazebo, the bench's stand-ins, CARLA or the real car). Above the backend, every profile
runs the car's own nodes and configs. This package holds only launch files, machine
files, foxglove_bridge's settings and RViz layouts: no nodes of its own.

## Interface

### Profiles

| Profile | Launch file | Command | Who plays the car | Source | Clock |
|---|---|---|---|---|---|
| turtle | `turtle.launch.py` | `pixi run turtle` | turtlesim (`aav_turtlesim`) | SIM | wall |
| sim | `sim.launch.py` | `pixi run sim world:=road` | Gazebo (`aav_gazebo`); with `spine:=true`, `spine_bridge` + `spine_sim` in the loop | SIM | simulated |
| bench | `bench.launch.py` | `pixi run bench` | `spine_sim` and mock sensors (`aav_bench`, `aav_spine_sim`) | BENCH | wall |
| car | `car.launch.py machine:=<m>` | `pixi run car orin` | the real car: `legacy_uart_bridge` (`actuators:=uart`, today) or `spine_bridge` (`actuators:=can`) | REAL | wall |
| carla | `carla.launch.py host:=<ip>` | `pixi run carla-car host:=...` | CARLA (`aav_carla`'s `carla_adapter`) | SIM | CARLA's |
| replay | `replay.launch.py` | `ros2 launch aav_bringup replay.launch.py` + `ros2 bag play` | nobody: it drives nothing | REPLAY | the bag's |

Two helpers:

| File | Does |
|---|---|
| `stack.launch.py target:=turtle\|gazebo\|bench\|car\|carla [source:=]` | one launch for every target; refuses a `source:=` that isn't the target's own |
| `foxglove.launch.py` | foxglove_bridge for Foxglove or Lichtblick: `foxglove_address`, `foxglove_port` (8765), `foxglove_control`, `foxglove_topics` (`wifi` or `all`), `foxglove_preview` |

Each launch file's docstring lists all its arguments. The common ones:

| Argument | Profiles | Does |
|---|---|---|
| `nav`, `teleop`, `rviz`, `foxglove` | sim, bench, carla | Nav2, the joystick, RViz, foxglove_bridge |
| `perception`, `lane`, `camera_obstacles` | sim, bench | the object detector and the stop-sign watcher; lane keeping; boxes into Nav2's costmap |
| `hud` | sim, bench | the HUD window (`aav_hud`, raylib); needs a screen |
| `bridge:=rust\|python` | sim, bench, car | which `spine_bridge` (`aav_spine.bridges`) |
| `reversing` | car, sim, bench, carla | may Nav2 back up (at 0.5 m/s at most). `true` everywhere; on the car only where the lidar watches behind. Until the wheels tell direction (H5), the sign is the last command's |
| `max_speed`; `sim_speed` | car; sim, carla | the top speed, m/s |
| `start_y` (m), `start_yaw` (rad); the car also `start_x` | car, sim, bench | where the car starts in the map frame |
| `arm` | sim (`spine:=true`), bench | `arm_when_open` arms the spine after 12 s |

### Machine files (`config/machines/`)

`car.launch.py` reads `<machine>.yaml`: which **parts** run on that computer, and the
detector's and camera streams' settings. Arguments win over the file (`gps:=false`,
`lidar:=false`).

| Machine | Runs |
|---|---|
| `orin` | description, mux, ego, scripted, drive_to_point, sensors, lidar, camera_stream, stop_signs, localization, navigation, spine, foxglove |
| `motor_pi` | `legacy_uart` only: the Pi wired to the ESP32's UART |
| `motor_pi_alone` | the first moves: mux, ego, stop_signs, teleop and `legacy_uart`, all on the Motor Pi, so the heartbeat is made there. Never with `car orin` too (docs/car-day.md, A9b) |
| `laptop` | teleop and a detector |
| `all` | everything on one computer |

The parts are listed in `PARTS` in `car.launch.py` and its docstring.

### Other files

| File | Holds |
|---|---|
| `config/foxglove_bridge.yaml` | the WiFi topic list: no raw cameras, no full lidar (docs/bandwidth.md) |
| `rviz/drive.rviz`, `road.rviz`, `bench.rviz` | RViz layouts |

## Invariants

- **Exactly one backend** runs at a time. `stack.launch.py` checks `source:=` against
  the target (`aav_io_mux.provenance.choose`); the sensor and output muxes check again
  while it runs.
- The real car runs only on REAL data; a simulator never drives on real sensors.
- Stand-ins (`turtle`, `sim`, `bench`, `carla-car`) run through `tools/ros.sh --local`,
  so they never reach a real car on the network. `pixi run car` does not.
- Car measurements come from `aav_description`'s `vehicle.load()`. Never copy a number
  into a launch file.
- Included launch files share their arguments, so pass every one. That is why
  foxglove's arguments start with `foxglove_` and the HUD's with `hud_`.
- On the car, foxglove_bridge only watches (`foxglove_control:=false`): anyone on the
  WiFi reaches it.
- `legacy_encoder` and `spine` never run together: two `/vehicle/odom` publishers stop
  the output mux.
- A part that can't run here (no PyTorch, no model, no usb_cam) is skipped with a warning.
- Only stand-ins arm the spine themselves, and only once the output mux says OPEN. On
  the car, a person arms.
- Every machine uses the same `ROS_DOMAIN_ID` (docs/platforms.md).

## Side Effects

- Spawns every node of the chosen profile, as child processes.
- `car`: opens the real devices through their drivers (serial, CAN, USB, UDP).
- foxglove_bridge listens on port 8765: `127.0.0.1` for stand-ins, `0.0.0.0` on the car.
- `replay`: publishes `/system/source` (REPLAY), latched, as the node `replay_source`.

## Verification

```sh
pixi run build && colcon test --packages-select aav_bringup && colcon test-result --verbose
```

The tests (`test/`) load each launch file and check what it starts. `pixi run test`
runs every package's tests. Against a running profile: `pixi run check <profile>`,
`pixi run bench-check`, `pixi run sim-check`.
