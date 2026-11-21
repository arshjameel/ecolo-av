# aav_simulator

## Overview

**Ravensim**, our own simulator, in Rust (`sim.launch.py simulator:=ravensim`,
`docs/own-simulator.md`). It took the place of `gz sim` and its `ros_gz_bridge`s (Gazebo was
removed 16 Nov). It reads SDF world files, moves the car, casts the lidar's and radar's rays,
and publishes on `/vehicle/sim/*`. `aav_sim_models`'s nodes (`sim_adapter`, `sim_imu`, `sim_gps`, `sim_radar`) and
the whole stack above them run unchanged. The cameras are their own program,
`aav_sim_cameras`, drawing the same world file; the launch starts both. **The default
since 8 Nov, the only simulator since 16 Nov** (its old name, `simulator:=own`, went on 20 Nov).

```text
/control/sim/actuator_cmd ─> sim_adapter ─> /vehicle/sim/cmd_vel ─> simulator (the car)
                             (actuator model)                       │
  /clock, /vehicle/sim/odom, imu, navsat, radar/points, /joint_states, ◄─┘
  /vehicle/sim/truth, /sensors/lidar/points ─> the same nodes as on the car
```

## Interface

### Crates

| Crate | Holds | ROS? |
|---|---|---|
| `crates/sim-core` | the worlds (SDF, `model://`, OBJ), the car (`CarModel`), the ground under each wheel (`ground.rs`: slopes, kerbs), the lidars, the IMU and GPS, the loop, the camera plug-in point (`SensorPlugin`), camera lockstep (`lockstep.rs`: `CameraGate`) | no, no Bevy |
| `crates/sim-core`, feature `shared` | `SharedCar`: the shared `car-model` crate (`aav_description`) behind `CarModel`, the same states as `ActuatorCar` to 1e-9 (`shared_car.rs`). The node doesn't use it yet: it runs `TwistFollower` behind `sim_adapter`'s model | no |
| `crates/sim-core`, `node.rs` | what the node decides, without ROS (S5, 18 Nov): parameter checks, which service requests it accepts and why not (`Refusal`), stamps, its pace against the wall (`Pace`). Tested on any laptop | no |
| `crates/sim-node` | the node, binary `simulator` (rclrs): ROS's messages to and from `sim-core`. Not tested by itself: that needs a ROS harness (start the binary, call its services, read its topics), like `aav_navigation`'s `test_respawn.py` | yes |

### Node `simulator`

| Direction | Topic | Type, rate |
|---|---|---|
| sub | `/vehicle/sim/cmd_vel` | Twist: speed and turn rate, kept until the next |
| sub | `/vehicle/sim/cameras/free_until` | Clock: the cameras' next picture's sim time; with `camera_lockstep`, the clock holds there until a later one comes |
| pub | `/clock` | 1 kHz |
| pub | `/vehicle/sim/odom` | Odometry, 50 Hz, `odom` |
| pub | `/vehicle/sim/truth` | Odometry, 50 Hz, `map`: height and tilt on the ground too |
| pub | `/vehicle/sim/imu` | Imu, 50 Hz, `imu_link`, gravity +9.8. Since 18 Nov (S3) it reads as a real IMU: turn rates on all three axes, from how the car turned since the last reading (slopes too); the mean acceleration since then (the car model catches each command in 1-2 ms at up to 10 m/s², so an instant's would read 10 or 0); the turn's pull at the sensor's place; gravity as the tilted car feels it. The car's IMU sends 200 Hz; this one stays at Gazebo's 50 |
| pub | `/vehicle/sim/navsat` | NavSatFix, 5 Hz, `gps_link` |
| pub | `/joint_states` | JointState, 100 Hz: four wheels, two steering joints |
| pub | `/sensors/lidar/points` | PointCloud2, 10 Hz, `lidar_link`, 1800 x 16, reliable (`lidar`) |
| pub | `/vehicle/sim/radar/points` | PointCloud2, 10 Hz, `radar_link`, 61 x 3, best effort (`radar`). Both clouds carry one field more than Gazebo's did: `speed`, how fast what each ray hit moves along it (a person walking; 0 for the still world), so `sim_radar` gives a mover its own Doppler speed (TODO 194) |
| pub | `/vehicle/sim/entities` | MarkerArray, latched: every placed model (its SDF file, its pose), for the cameras |
| service | `/vehicle/sim/spawn_entity`, `/vehicle/sim/delete_entity` | `simulation_interfaces`: place or remove a model |
| service | `/vehicle/sim/set_entity_state` | `simulation_interfaces/SetEntityState`: put the car (`aav`) at a pose, standing still; or a placed model at a pose, then moving at the twist (map frame: x, y, turn), as a person walking (TODO 170; the cameras draw it where it was put) |

Parameters: `world`, `models`, `start_x`, `start_y`, `start_yaw`, the car's size and sensor
mounts (`aav_sim_models`'s `ravensim.launch.py` passes `vehicle.yaml`'s), `lidar`, `radar`,
`obstacle`, `body_length` (0: drives through things), `ground_step` (0.1 m: the highest step
a wheel rolls up; 0: flat ground), `seed`, `wheel_noise` (m/s on each wheel reading) and `wheel_slip` (a fraction: the wheels read that much too far), both 0, off, by default (the odometry then integrates the readings and drifts; `ravensim.launch.py` `wheel_noise:=`, `wheel_slip:=`, `noise_seed:=`), `step` (0.001 s), `clock_hz`
(1000), `real_time_factor` (1.0; 0 = as fast as it can), `threads`, `camera_lockstep` (false;
`ravensim.launch.py camera_lockstep:=auto` turns it on under `tools/software_gl.sh`),
`camera_timeout` (10 s: run free if the cameras say nothing that long). The node's docstring has
them all.

## Invariants

- Same names, types, frames, rates and QoS as Gazebo's bridges had: `pixi run drop-in` checks.
- The same seed gives the same sensor streams, on any number of `threads` (the lidar forks one noise stream per ring).
- `sim-core` has no ROS and no Bevy, and its own tests.
- Rust API Guidelines; `cargo fmt` and `cargo clippy` clean.

## Side Effects

- Publishes `/clock`: run it only through `tools/ros.sh --local`, never beside a car.
- The build runs `cargo build --release --locked` into colcon's build folder.

## Verification

```sh
cd ros/sim/aav_simulator && cargo test --locked -p sim-core --features shared
pixi run sim-check
```

`tools/test.sh` runs the first; `pixi run test` runs it through colcon too.
