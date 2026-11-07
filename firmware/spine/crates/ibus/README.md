# ibus

## Overview

The FlySky iBUS protocol, from the car's RC receiver (an FS-iA6B). On today's car it
arrives on GPIO 32 at 115200 baud. Plain `no_std` Rust with no hardware access: the
board reads the UART and feeds the bytes in.

## The frame

A 32-byte frame comes about every 7 ms.

| Bytes | Hold |
|---|---|
| 0, 1 | `0x20 0x40`: the length (32), then the command (servo data) |
| 2 to 29 | 14 channels, each a little-endian `u16`: 1000 to 2000 µs |
| 30, 31 | the checksum, little-endian: 0xFFFF minus the sum of bytes 0 to 29 |

## Interface

| Item | Does |
|---|---|
| `Parser::push(byte)` | finds frames one byte at a time; after noise it resyncs, even inside a bad frame |
| `Receiver::new(timeout_ms)` | a parser, plus the latest frame and when it came |
| `Receiver::push(byte, now_ms)` | feeds one byte; true when it completed a frame |
| `Receiver::channels(now_ms)` | the latest channels, or `None` if silent longer than the timeout |
| `Receiver::frames()` | good frames so far (wraps) |
| `RcMap::TODAY` | today's channels and switch points (below); also `RcMap::default()` |
| `RcMap::THREE_POSITION` | the same, with the mode switch's middle as RC via ROS (TODO 48, proposed) |
| `RcMap`'s public fields | build your own map: channels `mode_switch`, `throttle`, `steering`, `gear`; usable range `valid_above`, `valid_below`; mode switch `direct_above`, `via_ros_above` (`Option`); gear switch `forward_below`, `reverse_above`; sticks `centre`, `half_travel`, `throttle_deadband` |
| `RcMap::switch(value)` | the `spine_core::RcSwitch` for a mode-switch value: computer, via ROS or direct |
| `RcMap::rc_input(&channels)` | the `spine_core::RcInput`: mode switch, throttle and steering (each -1 to 1), and whether the gear switch is on brake |
| `decode`, `encode`, `checksum` | one frame, without a parser |
| `Channels::get(index)` | one channel; index 0 is the transmitter's CH1 |
| `BAUD`, `FRAME_LEN`, `CHANNELS`, `HEADER` | 115200, 32, 14, `0x20 0x40` |

It depends only on `spine-core`, for `RcInput` and `RcSwitch`.

## Today's channels

The 2025-26 firmware counted channels from 0, as its iBUS library does. So its
"channel 5" (and `docs/firmware.md`'s) is CH6 on the transmitter.

| Index | Transmitter | Use | Switch points |
|---|---|---|---|
| 5 | CH6 | override switch | RC drives above 1500 |
| 2 | CH3 | throttle stick | counts only below 1475; 1000 is full |
| 3 | CH4 | steering stick | 1000 is +1 (left?), 2000 is -1 |
| 4 | CH5 | gear switch | forward below 1200, reverse above 1800, brake between |

Every channel must read between 100 and 2200, or it is out of range.

TODO(Q7), to check on the transmitter:

- which switches are CH5 and CH6. At start-up the C++ prints "SwC: TOP=Brake,
  MID=Fwd, LOW=Rev", but its code puts the brake in the middle. The code is kept;
- whether CH3 is reversed. The throttle counts only below the stick's middle;
- what the receiver sends when the radio link drops: its failsafe values, set on the
  transmitter. The timeout here only catches a receiver that goes silent;
- with Q2: whether +1 really steers left.

## Invariants

- `no_std`, no hardware access, no allocation: bytes in, channels out.
- Only a frame with the right header and checksum counts. A bad frame never
  refreshes the receiver.
- A receiver silent for longer than `timeout_ms` reads `None`. The clock may wrap.
- A channel is usable only strictly between 100 and 2200.
- Mode switch out of range: `rc_input` is `None` (no usable RC).
- A stick or the gear switch out of range: zero throttle, straight wheels.
- Throttle and steering stay between -1 and 1. The crate never sends radians or
  Jrk counts: that is `spine-board`'s job.

## Side Effects

None. The board reads the UART; this crate only parses what it is given.

## Verification

```sh
cd firmware/spine
cargo test -p ibus
```

The tests cover noise, frames cut off mid-way, bytes that look like a header inside
the data, 100,000 random bytes, the timeout, and every switch point above.

## Ported from

The 2025-26 firmware in `firmware/legacy/aav_umod_esp32/src/` (from AAV_cortex,
commit 90de619e52). It read iBUS with bmellink's IBusBM Arduino library, version 1.1.4.

| Here | C++ file | C++ function or name |
|---|---|---|
| `decode`, `checksum`, `Parser` | the IBusBM library | `IBusBM::loop()`, `readChannel()` |
| the channel indexes | `modules/hardware_configs.h` | `RC_EN_CH`, `THR_RC_CH`, `STR_RC_CH`, `DRIVE_MODE_CH` |
| `BAUD` | `modules/hardware_configs.h` | `IBUS_BAUD` |
| override above 1500 | `main.cpp` | `loop()`: `rc_en_raw > 1500 && rc_en_raw < 2200` |
| usable between 100 and 2200 | `modules/rc_handler/rc_handler.cpp` | `handleRCMode()` |
| gear switch points | `modules/rc_handler/rc_handler.cpp` | `handleRCMode()`: `drive_mode` |
| throttle: below centre, 5 % deadband | `modules/rc_handler/rc_handler.cpp` | `handleRCMode()`: `map(...)` and `throttle < -0.05f` |
| steering: 1000 is +1 | `modules/rc_handler/rc_handler.cpp` | `handleRCMode()`: `current_steering` |

Changed on purpose:

| What | C++ | Here |
|---|---|---|
| Finding frames | waits for a 3 ms gap, timed when `loop()` runs rather than when bytes arrive | the header and checksum; resyncs inside a bad frame |
| Receiver goes silent | keeps the last values, and the last throttle, forever | `None` after the timeout |
| Steering past 2000 | `map()` went negative; cast to `uint16_t` and clamped, it was full lock the other way | stops at -1 |
| Steering scale | 1550 Jrk counts per unit on one side, 1950 on the other | -1 to 1 both ways; `spine-board` turns radians into Jrk counts |
| Override on, a stick out of range | `jrkSafeStop()`: full brake, wheels straight, throttle off | RC takes over and asks for zero speed and straight wheels |
| Override switch out of range | read as off, so the computer drives | `None`: no usable RC; `spine-core` stops if RC was driving |
| Gear switch on brake | brake full on (Jrk target 3500) | `RcInput::brake`; `spine-core`'s `RC_GEAR_BRAKE` decides: the full brake (today), or zero speed, so the speed loop brakes |
| What the throttle means | a share of the DAC, open loop | a share of the top speed, for `spine-core`'s speed loop |
