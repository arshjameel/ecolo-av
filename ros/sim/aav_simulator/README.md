# aav_simulator

## Overview

Our own simulator, in Rust: a drop-in for `gz sim` and its `ros_gz_bridge`s
(`sim.launch.py simulator:=own`, `docs/own-simulator.md`). It reads Gazebo's own world files,
moves the car, casts the lidar's and radar's rays, and publishes on the topics the bridges
use. `aav_gazebo`'s nodes (`gazebo_adapter`, `gazebo_imu`, `gazebo_gps`, `gazebo_radar`) and
the whole stack above them run unchanged. The cameras are their own program,
`aav_sim_cameras`, drawing the same world file; the launch starts both. **The default
since 8 Nov**; `simulator:=gazebo` starts Gazebo instead.

```text
/control/sim/actuator_cmd ─> gazebo_adapter ─> /gazebo/cmd_vel ─> simulator (the car)
                             (actuator model)                       │
  /clock, /gazebo/odom, imu, navsat, radar/points, /joint_states, ◄─┘
  /vehicle/sim/truth, /sensors/lidar/points ─> the same nodes as with Gazebo
```

## Interface

### Crates

| Crate | Holds | ROS? |
|---|---|---|
| `crates/sim-core` | the worlds (SDF, `model://`, OBJ), the car (`CarModel`), the ground under each wheel (`ground.rs`: slopes, kerbs), the lidars, the IMU and GPS, the loop, the camera plug-in point (`SensorPlugin`) | no, no Bevy |
| `crates/sim-core`, feature `shared` | `SharedCar`: the shared `car-model` crate (`aav_description`) behind `CarModel`, the same states as `ActuatorCar` to 1e-9 (`shared_car.rs`). The node doesn't use it yet: it runs `TwistFollower` behind `gazebo_adapter`'s model, as with Gazebo | no |
| `crates/sim-node` | the node, binary `simulator` (rclrs) | yes |

### Node `simulator`

| Direction | Topic | Type, rate |
|---|---|---|
| sub | `/gazebo/cmd_vel` | Twist: speed and turn rate, kept until the next |
| pub | `/clock` | 1 kHz, as Gazebo |
| pub | `/gazebo/odom` | Odometry, 50 Hz, `odom` |
| pub | `/vehicle/sim/truth` | Odometry, 50 Hz, `map`: height and tilt on the ground too |
| pub | `/gazebo/imu` | Imu, 50 Hz, `imu_link`, gravity +9.8 |
| pub | `/gazebo/navsat` | NavSatFix, 5 Hz, `gps_link` |
| pub | `/joint_states` | JointState, 100 Hz: four wheels, two steering joints |
| pub | `/sensors/lidar/points` | PointCloud2, 10 Hz, `lidar_link`, 1800 x 16, reliable (`lidar`) |
| pub | `/gazebo/radar/points` | PointCloud2, 10 Hz, `radar_link`, 61 x 3, best effort (`radar`) |
| pub | `/vehicle/sim/entities` | MarkerArray, latched: every placed model (its SDF file, its pose), for the cameras |
| service | `/vehicle/sim/spawn_entity`, `/vehicle/sim/delete_entity` | `simulation_interfaces`: place or remove a model |
| service | `/vehicle/sim/set_entity_state` | `simulation_interfaces/SetEntityState`: put the car (`aav`) at a pose, standing still (Gazebo's `set_pose`) |

Parameters: `world`, `models`, `start_x`, `start_y`, `start_yaw`, the car's size and sensor
mounts (`aav_gazebo`'s `gazebo.launch.py` passes `vehicle.yaml`'s), `lidar`, `radar`,
`obstacle`, `body_length` (0: drives through things), `ground_step` (0.1 m: the highest step
a wheel rolls up; 0: flat ground), `seed`, `step` (0.001 s), `clock_hz`
(1000), `real_time_factor` (1.0; 0 = as fast as it can), `threads`. The node's docstring has
them all.

## Invariants

- Same names, types, frames, rates and QoS as Gazebo's bridges: `pixi run drop-in` checks.
- The same seed gives the same sensor streams, on any number of `threads` (the lidar forks one noise stream per ring).
- `sim-core` has no ROS and no Bevy, and its own tests.
- Rust API Guidelines; `cargo fmt` and `cargo clippy` clean.

## Side Effects

- Publishes `/clock`: run it only through `tools/ros.sh --local`, never beside a car.
- The build runs `cargo build --release --locked` into colcon's build folder.

## Verification

```sh
cd ros/sim/aav_simulator && cargo test --locked -p sim-core --features shared
pixi run sim-check --simulator own
```

`tools/test.sh` runs the first; `pixi run test` runs it through colcon too.
