# aav_spine

## Overview

The ROS side of the CAN spine, in Python. `spine_bridge` turns ego's commands and the
e-stop into CAN frames, and the spine's and wheel nodes' frames into `/vehicle/*`.
Its Rust twin, `aav_spine_bridge`, is the same node and the default; this one stays
until the Rust one has run on the car (`bridge:=python` picks it). This package also
owns the shared launch file and config both twins use.

## Interface

### Executables

| Executable | Does |
|---|---|
| `spine_bridge` | the bridge (topics below) |
| `arm_when_open` | calls `/spine/arm` each `retry` s (1.0) until armed or `give_up` s (120.0); exits 0 or 1. Stand-ins only |

### `spine_bridge` topics (role `spine`)

| Direction | Topic | Type |
|---|---|---|
| sub | `/control/<target>/actuator_cmd` (`car`, `bench` or `gazebo`, by `source`) | ackermann_msgs/AckermannDriveStamped |
| sub | `/safety/estop` | std_msgs/Bool |
| sub | `/safety/output_mux/state` | std_msgs/String |
| sub | `/can/rx` | can_msgs/Frame |
| pub | `/can/tx` | can_msgs/Frame |
| pub | `/vehicle/odom` (twist only, `base_link`) | nav_msgs/Odometry |
| pub | `/vehicle/wheels` | aav_interfaces/WheelSpeeds |
| pub | `/vehicle/status` | aav_interfaces/VehicleStatus |
| pub | `/vehicle/joint_states` | sensor_msgs/JointState |
| pub | `/vehicle/steering`, `/vehicle/throttle`, `/vehicle/brake` | aav_interfaces/ActuatorState |
| pub | `/system/source` (latched) | aav_interfaces/DataSource |
| service | `/spine/arm` | std_srvs/SetBool |

Role `sensors` (today's wiring) reads only the wheel frames: it publishes
`/vehicle/odom`, `/vehicle/wheels` and `/system/source`, and has no `/can/tx`, no
`/spine/arm` and no e-stop.

### Parameters

| Name | Default | Unit | Set in |
|---|---|---|---|
| `cmd_timeout` | 0.1 | s | `config/spine.yaml` |
| `wheel_timeout` | 0.1 | s | the node's default only |
| `gate_timeout` | 1.5 | s | `config/spine.yaml` |
| `drive_rate_hz`, `estop_rate_hz` | 50.0, 10.0 | Hz | `config/spine.yaml` |
| `speed_variance`, `yaw_rate_variance` | 0.01, 0.02 | (m/s)², (rad/s)² | `config/spine.yaml` |
| `wheelbase`, `track` | from `vehicle.yaml` | m | the launch file |
| `source` | `real` (`bench`, `sim`) | | the node's default; bench and sim launches set it |
| `role` | `spine` (`sensors`) | | the node's default, or `spine.launch.py`'s `role` |
| `steering_joints` | front left and right steering joints | | `config/spine.yaml` |

### Launch

`spine.launch.py [interface:=can0] [bridge:=rust|python] [role:=spine|sensors] [use_sim_time:=false]`:
ros2_socketcan's receiver (to `/can/rx`), its sender (from `/can/tx`, role `spine`
only), and `spine_bridge`.

### Modules

| Module | ROS? | Holds |
|---|---|---|
| `spine_bridge.py` | yes | the node `SpineBridge` |
| `arm_when_open.py` | yes | the arming helper |
| `can_codec.py` | no | `pack`, `unpack`, `FRAMES`, the CAN ids, `crc8_sae_j1850`, `counter_step`; VEHICLE_STATUS's modes `MODE_MANUAL` 0, `MODE_RC` 1, `MODE_COMPUTER` 2, `MODE_TEST` 3 |
| `wheel_speed.py` | no | `FrontWheels`, `Reading`, `drive_sign`, `rear_axle_speed` |
| `output_gate.py` | no | `OutputGate`, `try_again`, `GATE_SHUT` |
| `bridges.py` | no | `package()`, `role_for()`, `check_role()`: which bridge, which role |

## Invariants

- CAN ids and layouts come from `firmware/can/aav.dbc`. After a change, update
  `can_codec.py` and `protocol.rs`, run `firmware/can/gen_test_vectors.py`, and run
  both test suites.
- Every frame ends in a CRC-8/SAE-J1850 over the id and data, with a 4-bit counter.
- Reads only `/control/<target>/actuator_cmd`, never `/ego/actuator_cmd`.
- `DRIVE_CMD` goes out at 50 Hz only while ego's command is younger than `cmd_timeout`.
  When it stops, the spine's 100 ms watchdog brakes: at most 0.2 s in all.
- `/spine/arm` is refused during an e-stop, and unless the output mux said OPEN for
  this bridge's target within `gate_timeout`. Disarming always works.
- Arming sends a few `armed=0` frames before `armed=1`: the spine arms on the edge.
- Gate times use a steady clock, never the node's (simulated time may pause).
- `/vehicle/odom`'s speed: the front wheels' mean while both are fresh, else the spine's.
- `arm_when_open` never retries after an e-stop refusal, and never arms twice.
- The logic modules have no ROS imports.

## Side Effects

- Writes CAN frames on `/can/tx`; ros2_socketcan sends them on `can0`.
- Bring the bus up first: `sudo ip link set can0 up type can bitrate 500000`
  (Linux only).

## Verification

```sh
pixi run build && colcon test --packages-select aav_spine && colcon test-result --verbose
```

`pixi run test` runs every package's tests. Against a running graph with the spine in
it: `pixi run check spine`.
