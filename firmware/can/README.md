# firmware/can: the CAN contract

## Overview

`aav.dbc` is the contract between ROS (the Orin) and the boards on the CAN bus: every
frame, its ID, its sender, and its signals. Today it is version 0.2.0, the KEFC plan
(`docs/can-plan.md`).

With today's wiring the Orin only listens (the wheel frames); the ESP32 is driven over the
Motor-Pi's UART. With the CAN spine, the Orin sends `ESTOP_CMD` and `DRIVE_CMD` too.

| File | What |
|---|---|
| `aav.dbc` | the frame map, in the standard DBC format (opens in SavvyCAN, cantools, Kvaser...) |
| `gen_test_vectors.py` | makes the shared test vectors from the Python codec |
| `test_vectors.json` | generated: 12 frames, their values and their bytes. Do not edit |

## Interface

Classic CAN at 500 kbit/s, 11-bit IDs, little-endian signals.

| ID | Name | Sender | Rate | Signals |
|---|---|---|---|---|
| 0x010 (16) | `ESTOP_CMD` | ORIN | on change and 10 Hz | `estop_request` (1 = stop now) |
| 0x100 (256) | `DRIVE_CMD` | ORIN | 50 Hz | `target_speed` (m/s, 0.001), `steering_angle` (rad, 0.0001, + left), `accel_limit` (m/s², 0 = default), `steer_rate_limit` (rad/s, 0 = default), `armed` |
| 0x180 (384) | `VEHICLE_FEEDBACK` | SPINE | 50 Hz | `measured_speed` (m/s), `measured_steering_angle` (rad, + left), `wheel_ticks` (wraps at 65536) |
| 0x181 (385) | `VEHICLE_STATUS` | SPINE | 10 Hz | `mode` (0 MANUAL, 1 RC, 2 COMPUTER, 3 TEST), `armed`, `estop_active`, `throttle` (%), `brake` (%), `faults` (bit field) |
| 0x200 (512) | `WHEEL_SPEED_L` | WHEEL_L | 100 Hz | `wheel_speed` (m/s), `pulse_count` (wraps at 65536), `direction_known` |
| 0x201 (513) | `WHEEL_SPEED_R` | WHEEL_R | 100 Hz | the same, for the right front wheel |

Every frame also has a 4-bit `counter` (rolling, per ID) and a last-byte `checksum`.

| Node | Is |
|---|---|
| ORIN | the autonomy computer: `spine_bridge` through `ros2_socketcan` |
| SPINE | the control board (the ESP32 MicroMod); not on the bus with today's wiring |
| WHEEL_L, WHEEL_R | the front wheel sensors: the hardware lead's Arduino (Arduino C++); one board may send both |

The `faults` bits match `aav_interfaces/VehicleStatus` `FAULT_*`: 1 cmd timeout, 2 bad
checksum, 4 bad counter, 8 steering driver, 16 brake driver, 32 throttle DAC, 64 wheel
encoder, 128 CAN bus, 256 RC signal lost, 512 overspeed.

### Who reads it

| Reader | Where |
|---|---|
| The Python codec | `ros/can_bridge/aav_spine/aav_spine/can_codec.py` |
| The Rust firmware | `firmware/spine/crates/spine-core/src/protocol.rs` |
| The Rust bridge | `ros/can_bridge/aav_spine_bridge/crates/spine-bridge` |
| The wheel nodes' logic | `firmware/spine/crates/wheel-core` |
| The bus load | `tools/can_load.py` (`pixi run python3 tools/can_load.py`) |

## Invariants

- **The checksum:** CRC-8/SAE-J1850 (poly 0x1D, init 0xFF, not reflected, xorout 0xFF)
  over the ID's low byte, the ID's high byte, then the other data bytes.
- `DRIVE_CMD`: with no valid frame for 100 ms, the spine cuts the throttle, brakes and
  holds the steering.
- `ESTOP_CMD`: the spine latches a stop until it gets 0 and is re-armed. The hardware
  e-stop and the RC transmitter always win over `armed`.
- Units are SI and REP-103: metres, radians, seconds; positive steering turns left.
- `test_vectors.json` and `spine-core/tests/vectors.rs` are generated: never edit them.
- **The change procedure** (root `CLAUDE.md`). After any change to `aav.dbc`:
  1. update `can_codec.py` and `protocol.rs`;
  2. run `python3 firmware/can/gen_test_vectors.py`;
  3. run the test suites, Python and Rust (below).

## Side Effects

- `gen_test_vectors.py` writes `firmware/can/test_vectors.json` and
  `firmware/spine/crates/spine-core/tests/vectors.rs`, then runs `rustfmt` on the second
  if it is installed.
- `aav.dbc` and `test_vectors.json` are data: no side effects.

## Verification

```sh
python3 firmware/can/gen_test_vectors.py                     # after a change
cd ros/can_bridge/aav_spine && python3 -m pytest -q          # the codec, the DBC, the vectors
cd firmware/spine && cargo test -p spine-core                # the firmware, the vectors
cd ros/can_bridge/aav_spine_bridge && cargo test -p spine-bridge   # the Rust bridge
```

`tools/test.sh` runs all three. The Python test also checks that every frame and signal in
`aav.dbc` matches `can_codec.py`.
