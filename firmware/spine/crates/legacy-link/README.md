# legacy-link

## Overview

The 2025-26 UART link, from the Motor-Pi to the ESP32 control board, in plain `no_std`
Rust. It lets the Rust ESP32 program be a drop-in for the C++.

```text
 byte  0..3      4..7     8..11    12
      steering  speed    accel    sum(bytes 0..11) & 0xFF      115200 baud, 8N1, no header
```

## Interface

### Version 1: the 2025-26 frame

| Item | Does | The C++ (`ros2_handler.cpp`) |
|---|---|---|
| `Frame::encode`, `Frame::decode` | 13 bytes ↔ three `f32` | `memcpy`, the sum |
| `Parser::new`, `Parser::push` | a parser; one byte in; a frame, a frame turned down, or nothing | the byte loop: a bad sum drops one byte |
| `Frame::check` | each value within -1..1 | `fabsf(...) <= 1.0f` |
| `Frame::targets` | the steering and brake Jrk targets, and the DAC (`Cut`, `Idle`, `Drive`) | the rest of `handleROS2Messages()`, with its `f32` maths |
| `Targets`, `Throttle` | the C++'s three outputs; the DAC as `Cut`, `Idle` or `Drive { code, reverse }` | |
| `STEER_CENTRE`, `BRAKE_RELEASED`, `BRAKE_FULL`, `DAC_CAP` | 1950, 400, 3500, 700 | `JRK_CENTER`, `JRK_MIN`, `JRK_MAX`, `DAC_MAX_CAP` |
| `FrameError` | why a frame was turned down: `Checksum`, `OutOfRange`, `Sync`, `Version`, `Flags`, `Field` | |
| `BAUD`, `FRAME_LEN` | 115200, 13 | |

### Version 2: the Orin's heartbeat (TODO 50)

```text
 byte  0     1   2..5      6..9   10..13  14       15     16
       0xA5  2   steering  speed  accel   counter  flags  CRC-8/SAE-J1850 of bytes 1..15
```

| Item | Does |
|---|---|
| `FrameV2::encode`, `FrameV2::decode` | 17 bytes ↔ the values, the counter, `Flags { armed, estop }`; a bad sync, CRC, version or flag is an error |
| `ParserV2::new`, `ParserV2::push` | a parser; finds frames by the sync byte; a bad frame: start again at the next `0xA5` in it |
| `Heartbeat::new`, `Heartbeat::fresh` | the rule: true only when the counter changed (the first frame after boot only sets it) |
| `Heartbeat::last` | the last counter seen, if any |
| `Flags::to_byte`, `Flags::from_byte` | the flags ↔ byte 15; an unknown bit is `FrameError::Flags` |
| `crc8` | CRC-8/SAE-J1850, the same as `aav.dbc`'s |
| `SYNC`, `VERSION`, `FRAME_V2_LEN` | `0xA5`, 2, 17 |

Why and how: [the heartbeat](../../../../docs/rust-firmware.md#the-heartbeat-version-2-of-the-serial-frame-24-oct).

### Version 3: the steering actuator's calibration (1 Nov)

Version 2, plus a request from a person on the hardware panel: the steering actuator
**limp** (its motor off, the wheels turned by hand) or a **jog** target in counts.

```text
 byte  0     1   2..13           14       15     16        17..18       19
       0xA5  3   the three f32   counter  flags  cal mode  jog target   CRC-8 of bytes 1..18
```

| Byte 16 | Mode | Bytes 17..18 |
|---|---|---|
| 0 | off: drive as the frame says | 0 |
| 1 | limp | 0 |
| 2 | jog | the target, 0 to 4095, little-endian |

Why a new version, not flags in version 2: the target needs two more bytes, and a
firmware that knows only version 2 turns a version 3 frame down whole (its watchdog then
brakes) rather than misread it. `ParserV3` reads version 2 frames too, so the Orin's
`frame_version: 2` keeps working.

| Item | Does |
|---|---|
| `FrameV3::encode`, `FrameV3::decode` | 20 bytes ↔ the version 2 fields and a `CalRequest` (`Off`, `Limp`, `Jog(counts)`); an unknown mode, a target with limp, or one past 4095 is `Field` |
| `ParserV3::new`, `ParserV3::push` | versions 2 and 3, told apart by byte 1; any other version is `Version` at once |
| `SteerCal` | the rules (below), one tick at a time: `take` a fresh frame's `Ask`, `step` with the tick's `Situation`, get a `SteeringDrive` (`Spine`, `MotorOff`, `Target(counts)`) |
| `CalState`, `Refusal` | what it does, for the uplink: `Off`, `Limp`, `Jog`, `Hold`, `Return`, `Refused(why)` |
| `CAL_RATE`, `CAL_TIMEOUT_MS`, `COUNTS_MAX` | 300 counts a second, 100 ms, 4095 |

The rules, each with its test in `src/steer_cal.rs`:

| Rule | Why |
|---|---|
| only while the frame asks for no speed and a brake, and the spine's throttle is 0 and its brake on | the car stands still, braked |
| never with an e-stop, nor while the RC (direct) or a test drives | they have the steering |
| never without a position (the Jrk's feedback mode must be analog) | limp and jog need it |
| any frame asking for a mode has its throttle held at 0, refused or not; the board's DAC stays at 0 while it runs | belt and braces |
| a jog target is clamped to the software limits (400 to 3500 today) and moves at 300 counts a second | the steering actuator's own (Jrk) limits are the last line, not the first |
| leaving holds where the wheels are (target = position), then moves back at 300 counts a second | no snap |
| 100 ms with no fresh frame asking for it: leave, and hold there until fresh frames come | the Orin's heartbeat is gone |

### The uplink (TODO 50, TODO 48)

The other way: ESP32 to Motor-Pi, 50 times a second. Version 3 (what this firmware
sends since 1 Nov): 40 bytes, the same sync byte and CRC-8 (over bytes 1..38).
Version 2 was the first 34 bytes and its CRC: 35 bytes. `Uplink::decode` reads both.

```text
 byte  0     1  2     3    4     5     6       7      8..9    10..33
       0xA5  3  kind  seq  echo  mode  switch  flags  faults  six f32 values
 byte  34..35           36..37        38           39
       steering counts  brake counts  cal state    CRC-8
```

| Item | Does |
|---|---|
| `Uplink::encode`, `Uplink::decode` | 40 bytes (decode: 35 too) ↔ the status: sequence, echo of the Orin's counter, mode, switch, armed, e-stop, faults, the Jrks' positions, what drives the throttle and brake, the sticks; in version 3 also `Counts` and the `CalState` |
| `Counts` | the steering and brake Jrks' positions, raw: their scaled feedback, 0 to 4095, in target units; `None` without analog feedback. Flag bits 6 and 7 say they are known |
| `DriveMode` | `Stopped`, `RcDirect`, `Computer`: the same numbers as `VehicleStatus.mode` |
| `ModeSwitch` | `NoSignal`, `Computer`, `RcViaRos`, `RcDirect` |
| `Sticks` | the RC throttle and steering (-1..1, positive forward and left), and the gear switch's brake |
| `UPLINK_LEN`, `UPLINK_V2_LEN`, `KIND_STATUS` | 40, 35, 1 |

Steering is in radians, positive left: converted with the calibration that the counts
are there to measure. A position the ESP32 can't measure is NaN on
the wire, with its flag bit clear (`None` here).

`spine-board`'s `LegacyLink` turns the targets into a `spine_core::DirectCmd`.

## Invariants

- `no_std`, no dependencies, no allocator, no hardware.
- Version 1 is byte-for-byte the C++'s: the same frame, the same resync (a bad sum
  drops one byte), the same `f32` maths for the targets. The three witnesses below
  hold it there.
- A value outside -1..1, or NaN, turns the frame down (`OutOfRange`).
- Versions 2 and 3 and the uplink: a bad sync, CRC, version, kind, flag or field is an
  error, never a guess. A wrong uplink kind comes back as `FrameError::Version`: there
  is no separate kind error.
- `Heartbeat::fresh` is true only when the counter changed. The first frame after
  boot only sets it.
- The ROS side (`legacy_protocol.py`) and this crate must agree: regenerate the
  vectors after changing either.

## Side Effects

None. The board reads and writes the UART (115200 baud, 8N1); this crate only
turns bytes into values and back. `gen_vectors.py` writes `vectors.json` and
`tests/vectors.rs` when you run it.

## Verification

```sh
cd firmware/spine
cargo test -p legacy-link
```

The other two witnesses are in the next section; `tools/test.sh` runs all three.

## Test vectors: three witnesses

`gen_vectors.py` makes the frames with `legacy_protocol.py` (the ROS side), and writes
`vectors.json` and `tests/vectors.rs`.

| Witness | Test |
|---|---|
| this crate | `cargo test -p legacy-link` |
| `legacy_protocol.py` | `ros/can_bridge/aav_legacy/test/test_legacy_protocol.py` |
| the C++ itself, compiled on the laptop with the stand-ins in `cpp_check/` | `tools/tests/test_legacy_cpp.py` |

After changing `legacy_protocol.py` or this crate:

```sh
python3 firmware/spine/crates/legacy-link/gen_vectors.py
tools/test.sh
```
