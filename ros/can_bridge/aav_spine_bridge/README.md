# aav_spine_bridge

## Overview

`spine_bridge` in Rust (rclrs): the same node as `aav_spine`'s Python one, topic for
topic, and the default (`bridge:=rust`). It packs frames with the firmware's own code
(`spine-core`), so the Orin and the spine share one CAN implementation. A Cargo
workspace that colcon builds through `CMakeLists.txt`.

## Interface

### Crates

| Crate | Holds |
|---|---|
| [`spine-bridge`](./crates/spine-bridge/README.md) | the logic, no ROS: commands to frames, frames to motion |
| `crates/bridge-node` | the ROS node, binary `spine_bridge`: messages in, calls to `spine-bridge`, messages out; and binary `arm_when_open` |

### Node `spine_bridge`

Topics, service and parameters are `aav_spine`'s, word for word: see
[aav_spine](../aav_spine/README.md). In short:

| Direction | Topic |
|---|---|
| sub | `/control/<target>/actuator_cmd`, `/safety/estop`, `/safety/output_mux/state`, `/safety/self_test/status` (with `require_ready`), `/can/rx` |
| pub | `/can/tx`, `/vehicle/odom`, `/vehicle/wheels`, `/vehicle/status`, `/vehicle/joint_states`, `/vehicle/steering`, `/vehicle/throttle`, `/vehicle/brake`, `/system/source` |
| service | `/spine/arm` (std_srvs/SetBool) |

Parameters: `aav_spine`'s `config/spine.yaml`, plus `wheelbase` and `track` from
`vehicle.yaml`, `source` and `role`. Start it with `aav_spine`'s `spine.launch.py`.

### `arm_when_open`

`aav_spine`'s `arm_when_open.py` in Rust (18 Nov), the default in bench mode and Ravensim's `spine:=true` (`arm_when_open_impl:=python` falls back): it asks `/spine/arm` each `retry` s while the gate alone refuses, for `give_up` s; exits 0 once armed, 1 otherwise. The rule is `spine-bridge`'s `try_again`, tested with the Python's cases.

Where it differs from the Python, on purpose:

| What | Rust |
|---|---|
| Extended-id frames | ignored (another device's, never ours) |
| Parameters | read-only, read once |

## Invariants

- The same behaviour as `aav_spine`'s `spine_bridge`: the same topics, timeouts, gate,
  ready gate (the boot self-test, `require_ready`) and arming edge. A change to one goes into both.
- CAN frames are `firmware/can/aav.dbc`, packed by `spine-core`'s `protocol`.
- Role `sensors` never publishes on `/can/tx`.
- rclrs has no signal handling yet: `ctrlc` catches SIGINT, SIGTERM and SIGHUP.
- Rust API Guidelines; `cargo fmt` and `cargo clippy` clean.

## Side Effects

- Publishes CAN frames on `/can/tx` (ros2_socketcan sends them on the bus).
- The build runs `cargo build --release --locked` into colcon's build folder.

## Verification

```sh
cd ros/can_bridge/aav_spine_bridge && cargo test --locked -p spine-bridge
```

`tools/test.sh` runs that. `colcon test --packages-select aav_spine_bridge` runs the
same through CMake, and `pixi run test` runs everything. Building needs Rust 1.85 or
newer: `pixi run build`.
