# aav_legacy_rs

## Overview

`legacy_uart_bridge` in Rust (rclrs): the **default** since 17 Nov, in place of
`aav_legacy`'s Python one. Ben, 17 Nov: "Rust tomorrow ... I don't think we want to do
this again later". The same node name, topics, QoS, parameters and frames.
`legacy_impl:=python` falls back until a car day proves it.

```
/control/car/actuator_cmd ─┐
/control/car/heartbeat ────┼─> legacy_uart_bridge ──frames (v1, v2, v3)──> ESP32 (UART)
/safety/estop ─────────────┘          │  ▲
                                      │  └── uplink frames (uplink: true)
            /vehicle/status, /vehicle/rc, /vehicle/steering, ... <─┘
            /safety/legacy_uart_bridge/lock, /control/car/link_frame, /system/source
```

**One source of truth for the bytes**: every frame is packed and decoded by the
firmware's own crate, `firmware/spine/crates/legacy-link`, the code the Rust ESP32 runs.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/legacy-bridge` | the logic: the mapping, the relay (counter, stale stop, e-stop, the steering calibration's rules, the lock), the `pi` speed loop and after it the adaptive brake (`firmware/spine/crates/adaptive-brake`, TODO 258; why here: `aav_legacy`'s README), the uplink reader, the link report | no: `cargo test` on any laptop |
| `crates/legacy-node` | the node, binary `legacy_uart_bridge`; the serial port (`serialport`, no libudev) or `socket://host:port` (Ravensim's `sim_esp32`) | yes (rclrs 0.8, `ros-env` 0.3) |

colcon builds it through `CMakeLists.txt` (`cargo build --release --locked`), installed as
`lib/aav_legacy_rs/legacy_uart_bridge`. `aav_legacy`'s `legacy.launch.py` starts it with
`config/legacy.yaml`; `ravensim.launch.py esp32:=true` too.

### Node `legacy_uart_bridge`

The same as the Python's (`aav_legacy`'s README): every topic, QoS, parameter and default.

### `legacy-bridge`

| Module | Holds | From |
|---|---|---|
| `mapping` | `Mapping`, `to_legacy`, `from_pedals`, `stop_command`, `clamp1` | `legacy_protocol.py` |
| `relay` | `Relay` (each tick's frame), `FrameVersion`, `CalWhere`, `CalMode`, `CalRefusal`, `programs_locked` | `uart_relay.py` |
| `speed` | `SpeedControl`, `SpeedConfig`, `Pedals` | `speed_control.py` |
| `uplink` | `UplinkParser`, `UplinkReader`, `status`, `rc`, `raw`, `calibration_line`, `Health` | `legacy_protocol.py`, `uplink_reader.py` |
| `link` | `LinkReport`, `frame_fields` | `link_report.py` |

## Invariants

- **The same answers as the Python, exactly**: `tests/vectors.rs` reads `vectors.json`,
  which `gen_vectors.py` writes by running the Python modules: every frame byte for byte
  (versions 1, 2, 3; open and pi; the calibration on stands), every pedal, every uplink
  field, every word of the calibration line. Change one, change both, then
  `pixi run python ros/can_bridge/aav_legacy_rs/gen_vectors.py`.
- The bridge only relays the Orin's counter, never makes one up.
- A stale command or an e-stop sends brake frames; on exit, five stop frames.
- Same node name and topics: nothing downstream can tell which runs. Only one runs.
- `legacy-bridge` has no ROS. Rust API Guidelines: enums, `Result`, no `unwrap` in
  library code; `cargo fmt` and `cargo clippy -D warnings` clean. `Cargo.lock` committed.

## Side Effects

- Writes and reads the serial port; drives the real car's throttle, brake and steering.

## Verification

```sh
cd ros/can_bridge/aav_legacy_rs && cargo test -p legacy-bridge   # tools/test.sh runs it
```

The process itself: `aav_legacy`'s chain tests run each test with both bridges
(`test_heartbeat_chain.py` with the Rust muxes too, `test_uplink_chain.py` with the Rust
`rc_driver`, `test_no_timeout_lock.py`, `test_steering_calibration_chain.py`).
