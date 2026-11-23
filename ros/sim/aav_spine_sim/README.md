# aav_spine_sim

`spine_sim`: the spine, on a laptop. The firmware's real logic (`spine-core`, Rust)
runs on the CAN frames `spine_bridge` sends, and drives a car instead of motors. It
answers with the frames the real spine sends, so nothing above it can tell.

It stands in for the spine board and the actuators until they exist.

With `rc:=true` it has an RC receiver too, as the ESP32 does, and `rc_pad` lets a PS4
gamepad play the FS-i6X transmitter, so the firmware's RC rules can be driven on a laptop
(23 Nov): `pixi run rc-pad` (below).

## Overview

A Rust package. CMake only runs `cargo` and installs the binary, as in
`aav_imu_yahboom`, so colcon and launch files treat it like any other node.

| Crate | What | ROS? |
|---|---|---|
| `crates/bench-car` | the spine's logic (`spine-core`) and the wheel nodes' logic (`wheel-core`), unchanged, driving a simple car model | no |
| `crates/spine-sim` | the ROS nodes around it (rclrs 0.8, `ros-env` for messages): `spine_sim`, and `rc_pad` | yes |

The `car` parameter picks which car the spine drives:

| `car` | Used by | The car |
|---|---|---|
| `model` | bench mode (`pixi run bench`) | `bench-car`'s `Plant`, on this computer's clock |
| `ravensim` | `sim.launch.py spine:=true` | Ravensim's car, through `sim_adapter`'s actuator model, on Ravensim's clock |

```
spine_bridge ─/can/tx─> spine_sim (spine-core, 100 Hz) ─/can/rx─> spine_bridge
                           │ car:=model:  Plant ─> /vehicle/bench/truth ─> aav_bench
                           │ car:=ravensim: /vehicle/sim/spine_outputs ─> sim_adapter
```

## Interface

### Node: `spine_sim`

| Direction | Topic | Type | When |
|---|---|---|---|
| Subscribes | `/can/tx` | `can_msgs/Frame` | always: what `spine_bridge` puts on the bus |
| Subscribes | `/vehicle/bench/hw_estop` | `std_msgs/Bool` | always: true = hardware e-stop pressed |
| Subscribes | `/vehicle/bench/driver_faults` | `std_msgs/UInt16` | always: pretend a driver failed (fault bits) |
| Subscribes | `/vehicle/bench/rc_channels` | `std_msgs/UInt16MultiArray` | `rc:=true`: the RC receiver's 14 iBUS channels (µs), from `rc_pad` |
| Subscribes | `/vehicle/sim/wheels` | `aav_interfaces/WheelSpeeds` | `car:=ravensim` |
| Subscribes | `/vehicle/sim/steering` | `aav_interfaces/ActuatorState` | `car:=ravensim` |
| Publishes | `/can/rx` | `can_msgs/Frame` | always: VEHICLE_FEEDBACK 50 Hz, VEHICLE_STATUS 10 Hz, WHEEL_SPEED_L and _R 100 Hz |
| Publishes | `/vehicle/bench/truth` | `nav_msgs/Odometry` (`map` to `base_link`, 50 Hz) | `car:=model` |
| Publishes | `/vehicle/sim/spine_outputs` | `aav_interfaces/SpineOutputs` (100 Hz) | `car:=ravensim` |

Parameters: `car` (model), `wheelbase`, `track`, `wheel_radius` (m), `max_steering`
(rad), `start_x`, `start_y` (m), `start_yaw` (rad), `speed_loop` (default, smooth, lqi or firm),
`rc` (false: no receiver, as before; true: a receiver fitted and `rc_fitted`, as on the
ESP32, so the RC is lost until its channels come, and a silent 100 ms is a lost RC).

### Node: `rc_pad`

A PS4 gamepad as the FS-i6X and its receiver: the sticks and buttons as the channels the
car's receiver sends, 50 times a second (`bench-car`'s `Pad`).

| Direction | Topic | Type | |
|---|---|---|---|
| Subscribes | `/drivers/rc_pad/joy` | `sensor_msgs/Joy` | joy's `game_controller_node` (named `rc_pad_joy`) |
| Publishes | `/vehicle/bench/rc_channels` | `std_msgs/UInt16MultiArray` | the 14 channels, µs; nothing once the gamepad has been quiet 0.5 s, so `spine_sim` sees a silent receiver |

Parameter: `mode_start` (high: manual, RC direct; or low: normal).

| The gamepad (a DualShock 4, measured 23 Nov) | The FS-i6X | Channel (µs) |
|---|---|---|
| left stick up and down (`axes[1]`) | the left stick: the pedal | CH3: 1500 − 500 × the stick; down at most 1985 (+97 %) |
| right stick up and down (`axes[3]`) | the right stick: stop (down), give (up) | CH2: the same |
| right stick left and right (`axes[2]`) | the right stick: steering | CH4: 1500 − 500 × the stick |
| R1 (`buttons[10]`), each press | the mode switch | CH6: 1100 (low, normal) or 1900 (high, manual) |
| L1 (`buttons[9]`), held | the radio lost | CH6: 1000, the failsafe's value |

### Launch: `launch/rc_pad.launch.py` (`pixi run rc-pad`)

The RC's rules on a laptop, with nothing else running: the gamepad, `rc_pad`,
`spine_sim car:=model rc:=true`, and rviz showing the model car's path from above
(`config/rc_pad.rviz`). Arguments: `mode_start` (high), `rviz` (true). With no computer,
give in normal mode leaves the car standing still; manual drives it from the sticks.
In Ravensim: `pixi run sim spine:=true rc_pad:=true [rc_pad_mode:=low]`.

### `bench-car` (library)

| Item | Does |
|---|---|
| `Bench::new(spine, plant_cfg, start, wheels)` | the spine plus the model car |
| `Bench::with_car(spine, car, wheels)` | the spine plus any `Car` |
| `Bench::receive(id, data)` | one frame from the bus |
| `Bench::tick(dt_ms)` | runs `Spine::step`, moves the car, returns the frames to send |
| `Bench::set_hw_estop`, `set_driver_faults` | the bench's switches |
| `Bench::fit_rc(map)`, `rc_channels(&channels)` | an RC receiver read with `map` (`RcMap::TODAY`), and its channels as they arrive |
| `rc::Receiver` | the board's receiver: channels in as the bytes of an iBUS frame, through the ibus crate's parser and its 100 ms silence timeout (`RC_TIMEOUT_MS`), out as `RcMap::rc_input` decodes them |
| `rc::Pad`, `rc::PadLayout` (`PS4`) | a gamepad's axes and buttons as the FS-i6X's channels |
| `Bench::outputs()` | what the spine wrote to the actuators on the last tick (`car:=ravensim` publishes it) |
| `Bench::pose()` | where the model car is: the ground truth (model car only) |
| `Bench::car()`, `car_mut()` | the car the spine drives; `car_mut` gives it new measurements |
| `Frame` | one CAN frame: `(id, [u8; 8])` |
| `SpeedLoopProfile`, `SpineConfig` | re-exported from `spine-core`, so a bench picks its own settings |
| `Car` (trait) | what the spine needs of a car: speed, steering, wheel roll |
| `Plant`, `PlantConfig`, `Pose` | the model car: steering motor, throttle, brake, encoder, pose |
| `MeasuredCar` | a car moved elsewhere (Ravensim): `measure_speed`, `measure_steering`, `measure_front_wheels` |
| `BENCH_WHEELS`, `FEEDBACK_PERIOD_MS`, `STATUS_PERIOD_MS`, `WHEEL_PERIOD_MS` | the wheel nodes' setup and frame periods |

## Invariants

- `bench-car` has no ROS: it is plain Rust, tested with `cargo test -p bench-car`.
- `spine-core`, `wheel-core` and `ibus` are used unchanged, from `firmware/spine/crates/`:
  the bench runs what the boards will run. With `rc:=true` the RC goes through the same
  iBUS parser, timeout and channel map as the ESP32's.
- Without `rc:=true` the spine has no receiver (`rc: None`, `rc_fitted` false), as before.
- CAN ids and layouts are `firmware/can/aav.dbc`'s: 0x010 ESTOP_CMD and 0x100
  DRIVE_CMD in; 0x180 VEHICLE_FEEDBACK, 0x181 VEHICLE_STATUS, 0x200 and 0x201 wheel
  speeds out. Only standard 11-bit data frames are read.
- The spine ticks every 10 ms, as on the board. On Ravensim's clock, more than 10 ticks
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
