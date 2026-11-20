# aav_sim_models

Ravensim's Python half: the car behind the same topics as the real one. It holds the
worlds and their models, `ravensim.launch.py`, `sim_adapter` (the car itself) and one
node per sensor, as on the car. Ravensim's Rust half is `aav_simulator` (the world, the
rays, the clock) and `aav_sim_cameras` (the cameras).

Until 16 Nov this package was `aav_gazebo`, and its nodes were `gazebo_*`: Gazebo was
removed then ([Gazebo, as it was](../../../docs/history/gazebo-as-it-was.md)).

`aav_bringup`'s `sim.launch.py` starts it, with everything else as on the car and
`use_sim_time:=true`. `pixi run sim` runs that.

## Overview

```
/control/sim/actuator_cmd ─> sim_adapter ─> /vehicle/sim/cmd_vel ─> simulator (Rust)
                               (the actuator model)
simulator: /vehicle/sim/imu, navsat, radar ─> sim_imu, sim_gps, sim_radar ─> /sensors/...
simulator: /vehicle/sim/odom, /joint_states ─> sim_adapter ─> /vehicle/odom, wheels, status...
```

| World (`worlds/`) | Gives |
|---|---|
| `empty.sdf` | flat ground; renders nothing, runs headless anywhere |
| `test_lot.sdf` | the walls, building and parked cars of `aav_navigation`'s example lot map; renders the sensors too |
| `road.sdf` | a road with a stop sign; renders the cameras, the radar stand-in and the roof lidar. Its lane map: `road.lanes.yaml` (`sim.launch.py lane_map:=world`) |
| `campus` | Carleton, from OpenStreetMap; made by `pixi run campus` into `maps/campus/`, not kept in git (`docs/campus.md`) |

A world renders sensors only if it carries the marker `<plugin filename="ravensim"
name="ravensim::sensors"/>` (`worlds.has_sensors`; Gazebo's old Sensors system counts too).
`test_lot.sdf` and `road.sdf` both do. In such a world the lidar and the radar run by
default (`lidar:=true`, `radar:=true`); set either to false to leave it out.
So does a chase camera behind and above the car (`chase_camera:=true`), for Lichtblick's
cockpit layout: `/vehicle/sim/chase_camera/compressed`.
The cameras need a GPU, or `tools/software_gl.sh` (then the clock waits for each picture:
camera lockstep, `docs/own-simulator.md`). Every world is named
`aav`, and its origin must sit on the datum in `navsat.yaml`.

With `spine:=true` the spine is in the loop: `spine_bridge` takes ego's commands, and
`spine_sim` (`aav_spine_sim`) runs the spine's logic. Its outputs come back here, to the
actuator model.

With `esp32:=true` today's car is in the loop instead: the car's own `legacy_uart_bridge`
takes ego's commands and writes its 13-byte frames over local TCP to `sim_esp32`, which
runs the 2025-26 C++ firmware's logic (`esp32_firmware.py`: the throttle ramp, the brake,
no command timeout). `esp32_loop:=pi|open` is the bridge's `speed_loop.mode`.

## Interface

### Launch: `launch/ravensim.launch.py`

Arguments: `world` (empty, test_lot, road, or a path), `obstacle`, `start_x`, `start_y` (m),
`start_yaw` (rad), `lidar`, `cameras`, `camera_lockstep` (auto, true, false; auto = on
under `tools/software_gl.sh`), `graphics` (auto, simple, pretty; auto = pretty on a GPU), `chase_camera`, `radar`, `actuator_model`, `speed_loop`,
`payload` (kg aboard), `adaptive_brake`, `lag_scale`, `faults`, `spine`, `arm`, `bridge`
(rust or python), `esp32`, `esp32_loop` (pi or open), `noise_seed`, `wheel_noise`,
`wheel_slip`, `clock_hz`.

### Nodes

| Node | Subscribes | Publishes |
|---|---|---|
| `sim_adapter` | `/control/sim/actuator_cmd` (AckermannDriveStamped; not with spine), `/vehicle/sim/spine_outputs` (SpineOutputs; only with spine), `/vehicle/sim/esp32_outputs` (SpineOutputs; only with esp32), `/vehicle/sim/odom`, `/joint_states` | `/vehicle/sim/cmd_vel` (Twist), `/vehicle/odom`, `/vehicle/status`, `/vehicle/steering`, `/vehicle/throttle`, `/vehicle/brake`, `/vehicle/wheels` (with spine: `/vehicle/sim/wheels`, `/vehicle/sim/steering`), `/system/source` (latched, all SIM) |
| `sim_imu` | `/vehicle/sim/imu`, `/vehicle/sim/odom` | `/sensors/imu/data_raw` (Imu, `imu_link`, no orientation) |
| `sim_gps` | `/vehicle/sim/navsat`, `/vehicle/sim/truth` | `/sensors/gps/fix` (NavSatFix, `gps_link`), `/sensors/gps/fix_velocity` (TwistWithCovarianceStamped) |
| `sim_radar` | `/vehicle/sim/radar/points` (best effort), `/vehicle/sim/odom` | `/sensors/radar/points` (best effort, `radar_link`, the TI driver's fields); only with `radar:=true` (the default) in a world that renders sensors |
| `sim_esp32` | TCP 127.0.0.1:47000 + the ROS domain: `legacy_uart_bridge`'s frames; only with `esp32:=true` | `/vehicle/sim/esp32_outputs` (SpineOutputs, 50 Hz) |
| `sim_camera_relay` | `/vehicle/sim/camera/front_left/image_raw` | `/sensors/camera/front_left/image_raw`; only with `faults:=true` in a world that renders sensors |
| `sim_lidar_relay` | `/vehicle/sim/lidar/points` (the simulator's lidar, remapped there) | `/sensors/lidar/points`, as it came; only with `faults:=true` and the lidar (TODO 246) |
| `truth_localizer` | `/vehicle/sim/truth`, TF `odom → base_link`; only with `sim.launch.py truth_pose:=true` (a test: the wiggle, TODO 149) | `/localization/odom/global` (the truth), TF `map → odom` so that `map → base_link` is the truth; `ekf_global` doesn't run |
| `simulator` (`aav_simulator`, Rust) | `/vehicle/sim/cmd_vel`, `/vehicle/sim/cameras/free_until` | `/clock`, `/joint_states`, `/vehicle/sim/truth`, `odom`, `imu`, `navsat`, `radar/points`, `/sensors/lidar/points` |
| `sim_cameras` (`aav_sim_cameras`, Rust, a process) | `/vehicle/sim/truth` | the cameras, `/vehicle/sim/cameras/free_until` |

### Parameters

| Node | Parameters |
|---|---|
| `sim_adapter` | `actuator_model` (true), `speed_loop` (default, smooth, lqi or firm), `payload` (0 kg aboard, live), `adaptive_brake` (false, live; not with spine or esp32: `docs/stopping.md`), `spine` (false), `esp32` (false), `rate_hz` (50), `cmd_timeout` (0.2 s), `wheelbase`, `wheel_radius` (m, from `vehicle.yaml` via the launch file), `rendered`, `radar`, `lidar`, `speed_variance`, `fault.odom`; the actuators' `fault.steer` (ok, stuck), `fault.throttle` (ok, stuck), `fault.brake` (ok, weak, dead) |
| `sim_imu` | `gyro_stddev` (0.02 rad/s), `accel_stddev` (0.2 m/s²), `imu_x` (m), `gyro_bias` (rad/s), `fault.imu`, `accel_from` (simulator: Ravensim's own reading, the default since 18 Nov; odometry: the way before, `imu.py`), `seed` (3) |
| `sim_gps` | `gps_stddev` (0.5 m), `gps_speed_stddev` (0.05 m/s), `antenna_x`, `antenna_y` (m), `every` (1), `fault.gps`, `seed` (5: the velocity noise) |
| `sim_radar` | `radar_range` (8.4 m), `fault.radar` |
| `sim_camera_relay` | `fault.camera` |
| `sim_lidar_relay` | `fault.lidar` (ok, stop, freeze) |

`fault.<sensor>` is one of ok, stop, freeze, nan, jump and drift (GPS only: 5 m north
over a minute, then held; TODO 210). Set it live:
`pixi run sim-fault gps stop`.

The actuators' faults (TODO 159, the actuator model only) sit between the speed loop and
the car: a stuck steering actuator or throttle holds what it was asked for when it failed;
a weak brake gives 30 % of what is asked, a dead one none. `/vehicle/steering`, `throttle`
and `brake` still carry what was asked as `command`, and what the car did as `measured`.
`pixi run sim-fault brake weak`; `pixi run sim-check --actuator-faults` measures each.

### Modules

| Module | Holds | ROS? |
|---|---|---|
| `actuator_model.py` | re-exports `aav_description.car_model` (moved there 7 Nov: one car model for ego and the sims): `SpeedLoop` (spine-core's gains), `Actuators`, `Ideal`, `ActuatorConfig`, `speed_loop_profile` | no |
| `esp32_firmware.py` | `Esp32Firmware`: the 2025-26 C++ ESP32's frame handling, throttle ramp and brake | no |
| `faults.py` | the sensors' fault modes, `Fault`; the actuators', `ActuatorFaults` | no |
| `imu.py` | the car's own acceleration from the odometry (`accel_from odometry`: Gazebo's IMU left it out; Ravensim's reads it since 18 Nov) | no |
| `gps.py` | the receiver's velocity from the truth | no |
| `radar.py` | the simulator's radar cloud as the TI radar's; with its `speed` field, a moving thing's own Doppler speed (TODO 194) | no |
| `wheels.py` | each wheel's speed from joint states | no |
| `sources.py` | what `/system/source` announces | no |
| `worlds.py` | which world renders which sensors (`has_sensors`); each world's lane map (`lane_map_file`); the simulator's name (`check_simulator`; the old name `own` went on 20 Nov) and `camera_lockstep` | no |
| `pose_frames.py` | `map_to_odom`: the `map → odom` that puts the car on the truth (`truth_localizer`) | no |
| `relay.py` | what the sensor nodes share: fault parameter, clean start and stop | yes |
| `sim_*.py` | the seven nodes (`sim_adapter`, `sim_imu`, `sim_gps`, `sim_radar`, `sim_camera_relay`, `sim_lidar_relay`, `sim_esp32`) | yes |

Models (`models/`): `obstacle` (the bench's 1 m box), `post` (person-sized, for the
lidar's obstacle check in `tools/sim_check.py`), `kerb` (0.3 m high, `sim-check --kerb`), `stop_sign`.

## Invariants

- Logic modules have no ROS imports and have their own tests.
- Exactly one backend takes ego's commands: `sim_adapter`, or `spine_bridge` with
  `spine:=true`, or `legacy_uart_bridge` with `esp32:=true`.
- No command for `cmd_timeout` (0.2 s): target speed 0. This copies the spine's
  watchdog, though the spine's own is shorter (0.1 s). With the spine, `spine_sim`
  quiet for `cmd_timeout`: full brake.
- The actuator model, not the simulator, sets the limits. Its numbers are `vehicle.yaml`'s
  `actuators:` section.
- Each sensor node is its own process, so a slow one can't make another late.
- The simulator's truth (`/vehicle/sim/truth`) is for checks and the GPS's velocity only;
  it never reaches localization, except in one test that asks for it by name
  (`truth_pose:=true`: `truth_localizer` in `ekf_global`'s place).
- Units: m, m/s, rad, s. Positive steering turns left. Throttle and brake are 0 to 1.

## Side Effects

- Starts `simulator` (with `obstacle:=true`, the obstacle in the car's path) and, in a world
  that renders them, the `sim_cameras` process (no window).
- Publishes `/clock` and the topics above.

## Verification

```sh
pixi run build && colcon test --packages-select aav_sim_models && colcon test-result --verbose
```

`pixi run test` runs everything. The whole loop: `pixi run sim-check` (on a machine
with no GPU: `tools/software_gl.sh pixi run sim-check`).
