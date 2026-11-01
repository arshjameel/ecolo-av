# aav_gazebo

Gazebo Harmonic as the car, behind the same topics as the real one. It holds the
worlds and their models, the `ros_gz_bridge` configs, `gazebo_adapter` (the car
itself) and one node per sensor, as on the car.

`aav_bringup`'s `sim.launch.py` starts it, with everything else as on the car and
`use_sim_time:=true`. `pixi run sim` runs that.

## Overview

```
/control/gazebo/actuator_cmd ─> gazebo_adapter ─> /gazebo/cmd_vel ─bridge─> Ackermann system
                               (the actuator model)
Gazebo IMU, NavSat, radar ─bridge─> gazebo_imu, gazebo_gps, gazebo_radar ─> /sensors/...
Gazebo odometry, joints ─bridge─> gazebo_adapter ─> /vehicle/odom, wheels, status...
```

| World (`worlds/`) | Gives |
|---|---|
| `empty.sdf` | flat ground; renders nothing, runs headless anywhere |
| `test_lot.sdf` | the walls, building and parked cars of `aav_navigation`'s example lot map; renders the sensors too |
| `road.sdf` | a road with a stop sign; renders the cameras, the radar stand-in and the roof lidar |

A world renders sensors only if it loads Gazebo's Sensors system (`worlds.has_sensors`).
`test_lot.sdf` and `road.sdf` both do. In such a world the lidar and the radar run by
default (`lidar:=true`, `radar:=true`); set either to false to leave it out.
That needs a GPU, ogre2 on a Mac, or `tools/software_gl.sh`. Every world is named
`aav`, and its origin must sit on the datum in `navsat.yaml`.

With `spine:=true` the spine is in the loop: `spine_bridge` takes ego's commands, and
`spine_sim` (`aav_spine_sim`) runs the spine's logic. Its outputs come back here, to the
actuator model.

With `esp32:=true` today's car is in the loop instead: the car's own `legacy_uart_bridge`
takes ego's commands and writes its 13-byte frames over local TCP to `gazebo_esp32`, which
runs the 2025-26 C++ firmware's logic (`esp32_firmware.py`: the throttle ramp, the brake,
no command timeout). `esp32_loop:=pi|open` is the bridge's `speed_loop.mode`.

## Interface

### Launch: `launch/gazebo.launch.py`

Arguments: `world` (empty, test_lot, road), `gui`, `follow`, `obstacle`, `start_y` (m),
`start_yaw` (rad), `lidar`, `radar`, `actuator_model`, `speed_loop`, `faults`, `spine`,
`arm`, `bridge` (rust or python), `esp32`, `esp32_loop` (pi or open).

### Nodes

| Node | Subscribes | Publishes |
|---|---|---|
| `gazebo_adapter` | `/control/gazebo/actuator_cmd` (AckermannDriveStamped; not with spine), `/vehicle/gazebo/spine_outputs` (SpineOutputs; only with spine), `/vehicle/gazebo/esp32_outputs` (SpineOutputs; only with esp32), `/gazebo/odom`, `/joint_states` | `/gazebo/cmd_vel` (Twist), `/vehicle/odom`, `/vehicle/status`, `/vehicle/steering`, `/vehicle/throttle`, `/vehicle/brake`, `/vehicle/wheels` (with spine: `/vehicle/gazebo/wheels`, `/vehicle/gazebo/steering`), `/system/source` (latched, all SIM) |
| `gazebo_imu` | `/gazebo/imu`, `/gazebo/odom` | `/sensors/imu/data_raw` (Imu, `imu_link`, no orientation) |
| `gazebo_gps` | `/gazebo/navsat`, `/vehicle/gazebo/truth` | `/sensors/gps/fix` (NavSatFix, `gps_link`), `/sensors/gps/fix_velocity` (TwistWithCovarianceStamped) |
| `gazebo_radar` | `/gazebo/radar/points` (best effort), `/gazebo/odom` | `/sensors/radar/points` (best effort, `radar_link`, the TI driver's fields); only with `radar:=true` (the default) in a world that renders sensors |
| `gazebo_esp32` | TCP 127.0.0.1:47000 + the ROS domain: `legacy_uart_bridge`'s frames; only with `esp32:=true` | `/vehicle/gazebo/esp32_outputs` (SpineOutputs, 50 Hz) |
| `gazebo_camera` | `/gazebo/camera/front_left/image_raw` | `/sensors/camera/front_left/image_raw`; only with `faults:=true` in a world that renders sensors |
| `gz_bridge`, `gz_road_bridge`, `gz_lidar_bridge` | (stock `ros_gz_bridge`, `config/bridge*.yaml`) | `/clock`, `/joint_states`, `/vehicle/gazebo/truth`, the cameras, `/sensors/lidar/points` |

### Parameters

| Node | Parameters |
|---|---|
| `gazebo_adapter` | `actuator_model` (true), `speed_loop` (default, smooth or lqi), `spine` (false), `esp32` (false), `rate_hz` (50), `cmd_timeout` (0.2 s), `wheelbase`, `wheel_radius` (m, from `vehicle.yaml` via the launch file), `rendered`, `radar`, `lidar`, `speed_variance`, `fault.odom` |
| `gazebo_imu` | `gyro_stddev` (0.02 rad/s), `accel_stddev` (0.2 m/s²), `imu_x` (m), `gyro_bias` (rad/s), `fault.imu` |
| `gazebo_gps` | `gps_stddev` (0.5 m), `gps_speed_stddev` (0.05 m/s), `antenna_x`, `antenna_y` (m), `every` (1), `fault.gps` |
| `gazebo_radar` | `radar_range` (8.4 m), `fault.radar` |
| `gazebo_camera` | `fault.camera` |

`fault.<sensor>` is one of ok, stop, freeze, nan, jump (GPS only). Set it live:
`pixi run sim-fault gps stop`.

### Modules

| Module | Holds | ROS? |
|---|---|---|
| `actuator_model.py` | `SpeedLoop` (spine-core's gains), `Actuators`, `Ideal`, `ActuatorConfig`, `speed_loop_profile` | no |
| `esp32_firmware.py` | `Esp32Firmware`: the 2025-26 C++ ESP32's frame handling, throttle ramp and brake | no |
| `faults.py` | the fault modes, `Fault` | no |
| `imu.py` | the car's own acceleration, which Gazebo's IMU misses | no |
| `gps.py` | the receiver's velocity from the truth | no |
| `radar.py` | Gazebo's lidar cloud as the TI radar's | no |
| `wheels.py` | each wheel's speed from joint states | no |
| `sources.py` | what `/system/source` announces | no |
| `worlds.py` | which world renders which sensors (`has_sensors`) | no |
| `relay.py` | what the sensor nodes share: fault parameter, clean start and stop | yes |
| `gazebo_*.py` | the six nodes | yes |

Models (`models/`): `obstacle` (the bench's 1 m box), `post` (person-sized, for the
lidar's obstacle check in `tools/sim_check.py`), `stop_sign`.

## Invariants

- Logic modules have no ROS imports and have their own tests.
- Exactly one backend takes ego's commands: `gazebo_adapter`, or `spine_bridge` with
  `spine:=true`, or `legacy_uart_bridge` with `esp32:=true`.
- No command for `cmd_timeout` (0.2 s): target speed 0. This copies the spine's
  watchdog, though the spine's own is shorter (0.1 s). With the spine, `spine_sim`
  quiet for `cmd_timeout`: full brake.
- The actuator model, not Gazebo, sets the limits. Its numbers are `vehicle.yaml`'s
  `actuators:` section.
- Each sensor node is its own process, so a slow one can't make another late.
- Gazebo's truth (`/vehicle/gazebo/truth`) is for checks and the GPS's velocity only;
  it never reaches localization.
- Units: m, m/s, rad, s. Positive steering turns left. Throttle and brake are 0 to 1.

## Side Effects

- Starts `gz sim` (and on a Mac a second `gz sim -g` window), spawns the car and, with
  `obstacle:=true`, a box.
- Writes a temporary `bridge_road_*.yaml`, deleted at exit.
- Publishes `/clock` and the topics above.
- On oirp-spss, Gazebo needs `GZ_IP=127.0.0.1` and its own `GZ_PARTITION`.

## Verification

```sh
pixi run build && colcon test --packages-select aav_gazebo && colcon test-result --verbose
```

`pixi run test` runs everything. The whole loop: `pixi run sim-check` (on a machine
with no GPU: `tools/software_gl.sh pixi run sim-check`).
