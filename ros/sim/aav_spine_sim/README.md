# aav_spine_sim

`spine_sim`: the spine, on a laptop. The firmware's real logic (`spine-core`, Rust)
runs on the CAN frames `spine_bridge` sends, and drives a car instead of motors. It
answers with the frames the real spine sends, so nothing above it can tell.

It stands in for the spine board and the actuators until they exist.

## Overview

A Rust package. CMake only runs `cargo` and installs the binary, as in
`aav_imu_yahboom`, so colcon and launch files treat it like any other node.

| Crate | What | ROS? |
|---|---|---|
| `crates/bench-car` | the spine's logic (`spine-core`) and the wheel nodes' logic (`wheel-core`), unchanged, driving a simple car model | no |
| `crates/spine-sim` | the ROS node around it (rclrs 0.8, `ros-env` for messages) | yes |

The `car` parameter picks which car the spine drives:

| `car` | Used by | The car |
|---|---|---|
| `model` | bench mode (`pixi run bench`) | `bench-car`'s `Plant`, on this computer's clock |
| `gazebo` | `sim.launch.py spine:=true` | Gazebo's car, through `gazebo_adapter`'s actuator model, on Gazebo's clock |

```
spine_bridge ─/can/tx─> spine_sim (spine-core, 100 Hz) ─/can/rx─> spine_bridge
                           │ car:=model:  Plant ─> /vehicle/bench/truth ─> aav_bench
                           │ car:=gazebo: /vehicle/gazebo/spine_outputs ─> gazebo_adapter
```

## Interface

### Node: `spine_sim`

| Direction | Topic | Type | When |
|---|---|---|---|
| Subscribes | `/can/tx` | `can_msgs/Frame` | always: what `spine_bridge` puts on the bus |
| Subscribes | `/vehicle/bench/hw_estop` | `std_msgs/Bool` | always: true = hardware e-stop pressed |
| Subscribes | `/vehicle/bench/driver_faults` | `std_msgs/UInt16` | always: pretend a driver failed (fault bits) |
| Subscribes | `/vehicle/gazebo/wheels` | `aav_interfaces/WheelSpeeds` | `car:=gazebo` |
| Subscribes | `/vehicle/gazebo/steering` | `aav_interfaces/ActuatorState` | `car:=gazebo` |
| Publishes | `/can/rx` | `can_msgs/Frame` | always: VEHICLE_FEEDBACK 50 Hz, VEHICLE_STATUS 10 Hz, WHEEL_SPEED_L and _R 100 Hz |
| Publishes | `/vehicle/bench/truth` | `nav_msgs/Odometry` (`map` to `base_link`, 50 Hz) | `car:=model` |
| Publishes | `/vehicle/gazebo/spine_outputs` | `aav_interfaces/SpineOutputs` (100 Hz) | `car:=gazebo` |

Parameters: `car` (model), `wheelbase`, `track`, `wheel_radius` (m), `max_steering`
(rad), `start_x`, `start_y` (m), `start_yaw` (rad), `speed_loop` (default, smooth or lqi).

### `bench-car` (library)

| Item | Does |
|---|---|
| `Bench::new(spine, plant_cfg, start, wheels)` | the spine plus the model car |
| `Bench::with_car(spine, car, wheels)` | the spine plus any `Car` |
| `Bench::receive(id, data)` | one frame from the bus |
| `Bench::tick(dt_ms)` | runs `Spine::step`, moves the car, returns the frames to send |
| `Bench::set_hw_estop`, `set_driver_faults` | the bench's switches |
| `Bench::outputs()` | what the spine wrote to the actuators on the last tick (`car:=gazebo` publishes it) |
| `Bench::pose()` | where the model car is: the ground truth (model car only) |
| `Bench::car()`, `car_mut()` | the car the spine drives; `car_mut` gives it new measurements |
| `Frame` | one CAN frame: `(id, [u8; 8])` |
| `SpeedLoopProfile`, `SpineConfig` | re-exported from `spine-core`, so a bench picks its own settings |
| `Car` (trait) | what the spine needs of a car: speed, steering, wheel roll |
| `Plant`, `PlantConfig`, `Pose` | the model car: steering motor, throttle, brake, encoder, pose |
| `MeasuredCar` | a car moved elsewhere (Gazebo): `measure_speed`, `measure_steering`, `measure_front_wheels` |
| `BENCH_WHEELS`, `FEEDBACK_PERIOD_MS`, `STATUS_PERIOD_MS`, `WHEEL_PERIOD_MS` | the wheel nodes' setup and frame periods |

## Invariants

- `bench-car` has no ROS: it is plain Rust, tested with `cargo test -p bench-car`.
- `spine-core` and `wheel-core` are used unchanged, from `firmware/spine/crates/`: the
  bench runs what the boards will run.
- CAN ids and layouts are `firmware/can/aav.dbc`'s: 0x010 ESTOP_CMD and 0x100
  DRIVE_CMD in; 0x180 VEHICLE_FEEDBACK, 0x181 VEHICLE_STATUS, 0x200 and 0x201 wheel
  speeds out. Only standard 11-bit data frames are read.
- The spine ticks every 10 ms, as on the board. On Gazebo's clock, more than 10 ticks
  behind counts as a stall or a reset, not catch-up.
- Wheel frames carry a magnitude only (`direction_known = 0`), like the real sensors.
- `PlantConfig::default` numbers are guesses until the car is measured (Q1-Q5).
  `wheelbase` and `track` come from `vehicle.yaml` via the launch files.
- The spine runs on `SpineConfig::default()`, with only the speed loop's profile
  changed. So its limits are the firmware's own, not `vehicle.yaml`'s: a 100 ms command
  watchdog, 3.0 m/s, 1.5 m/s² up and 2.5 m/s² down.

## Side Effects

- Publishes the topics above. A thread runs the spine at 100 Hz; the main thread spins.
- No hardware, no files.

## Verification

```sh
cd ros/sim/aav_spine_sim && cargo test -p bench-car
pixi run build && colcon test --packages-select aav_spine_sim && colcon test-result --verbose
```

`colcon test` runs the same `cargo test -p bench-car` (CMake test `bench_car`).
`tools/test.sh` does not. `pixi run test` runs everything. The whole loop:
`pixi run bench-check`.
