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
| `RcMap`'s public fields | build your own map: channels `mode_switch`, `throttle`, `steering`, `stop_stick`; usable range `valid_above`, `valid_below`; the failsafe `lost_below`; mode switch `direct_above`, `via_ros_above` (`Option`); sticks `centre`, `half_travel`; the left stick's middle `brake_band`, `throttle_reversed`; a stick's end `full` (0.88); the steering centred to give, `centred` (0.15) |
| `RcMap::steer_right_gain` | right turns (the stick below centre) times this, then clamped to -1: 1.0 in `TODAY`; `legacy_link::STEER_RIGHT_PER_LEFT` gives the C++'s (the ESP32's `cpp-rc-steering`, 7 Nov) |
| `RcMap::switch(value)` | the `spine_core::RcSwitch` for a mode-switch value: normal (`Computer`), via ROS or manual (`Direct`) |
| `RcMap::rc_input(&channels)` | the `spine_core::RcInput`: mode switch, throttle and steering (each -1 to 1), whether the left stick is in its middle (`middle`) or past 88 % (`full`), and the right stick (`right`: `Centre`, `Stop`, `Give`); `None` for a mode switch out of range or on the failsafe |
| `decode`, `encode`, `checksum` | one frame, without a parser |
| `Channels::get(index)` | one channel; index 0 is the transmitter's CH1 |
| `BAUD`, `FRAME_LEN`, `CHANNELS`, `HEADER` | 115200, 32, 14, `0x20 0x40` |

It depends only on `spine-core`, for `RcInput` and `RcSwitch`.

## Today's channels

The 2025-26 firmware counted channels from 0, as its iBUS library does. So its
"channel 5" (and `docs/firmware.md`'s) is CH6 on the transmitter.

| Index | Transmitter | Use | Switch points |
|---|---|---|---|
| 5 | CH6 | mode switch | normal; manual (RC direct) above 1500 (`THREE_POSITION`: RC via ROS 1301..1700, manual above); **under 1050: the failsafe (1000), RC lost**; the switch's low sends 1100 (End points CH6, low side 80%) |
| 2 | CH3 | the left stick, up and down | the middle ±20 % (1400..1600; ±30 % until 26 Nov): throttle off; below it forward, above it reverse; past 88 % (under 1060, over 1940) its end |
| 3 | CH4 | the right stick, sideways: steering | 1000 is +1 (left?), 2000 is -1; centred within ±15 % (1425..1575) |
| 1 | CH2 | the right stick, up and down (springs back) | over 1940 (down past 88 %): the stop; under 1060 (up past 88 %) with the steering centred: give the car to the computer |
| 4 | CH5 | (SwC) not read since 21 Nov: its wiring is loose | |

Every channel must read between 100 and 2200, or it is out of range.

Outside the middle the throttle is rescaled: 0 at the middle's edge, 1 at the stick's end,
for the uplink; the spine reads only its sign, as the RC's speeds are fixed (Arsh and Ben,
21 Nov: the FS-i6X's left stick doesn't spring back, so where it rests is the driver's
choice; [the design](../../../../docs/designs/2026-11-22_rc-right-stick.md)). The FS-i6X's
sticks reach -100 % up and +97 % down, so 88 % leaves room.

The transmitter's failsafe must set CH6 to 1000 and CH2 to its stop (the right stick
held down), and the switch's low must send 1100:
[Car day, the RC transmitter](../../../../docs/car-day.md#the-rc-transmitter).

TODO(Q7), to check on the transmitter:

- which switch is CH6;
- whether CH3 is reversed: the left stick pushed away from you should read near 1000.
  If it reads near 2000, set `throttle_reversed`;
- that the failsafe really sends CH6 at 1000 when the transmitter is switched off, after End points moved the switch's low to 1100. The
  timeout here only catches a receiver that goes silent;
- with Q2: whether +1 really steers left.

## Invariants

- `no_std`, no hardware access, no allocation: bytes in, channels out.
- Only a frame with the right header and checksum counts. A bad frame never
  refreshes the receiver.
- A receiver silent for longer than `timeout_ms` reads `None`. The clock may wrap.
- A channel is usable only strictly between 100 and 2200.
- Mode switch out of range, or under `lost_below` (the failsafe): `rc_input` is `None`
  (no usable RC).
- A stick out of range (CH2, CH3 or CH4): the right stick's stop, straight wheels. The
  gear switch is never read.
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
| throttle: below centre is forward | `modules/rc_handler/rc_handler.cpp` | `handleRCMode()`: `map(...)` and `throttle < -0.05f` |
| steering: 1000 is +1 | `modules/rc_handler/rc_handler.cpp` | `handleRCMode()`: `current_steering` |

Changed on purpose:

| What | C++ | Here |
|---|---|---|
| Finding frames | waits for a 3 ms gap, timed when `loop()` runs rather than when bytes arrive | the header and checksum; resyncs inside a bad frame |
| Receiver goes silent | keeps the last values, and the last throttle, forever | `None` after the timeout |
| Steering past 2000 | `map()` went negative; cast to `uint16_t` and clamped, it was full lock the other way | stops at -1 |
| Steering scale | 1550 Jrk counts per unit on one side, 1950 on the other | -1 to 1 both ways; `spine-board` turns radians into Jrk counts |
| Override on, a stick out of range | `jrkSafeStop()`: full brake, wheels straight, throttle off | the right stick's stop (a full brake) and straight wheels |
| Override switch out of range | read as off, so the computer drives | `None`: no usable RC; `spine-core` stops if RC was driving |
| Forward, reverse and brake | the gear switch (CH5), the throttle on half the stick | the left stick's whole travel, throttle off in its middle; the right stick's up and down (CH2) for the stop; CH5 not read (21 Nov) |
| The radio link drops | keeps the last values | the failsafe's CH6 under 1050 is no usable RC |
| What the throttle means | a share of the DAC, open loop | a share of the top speed, for `spine-core`'s speed loop |
