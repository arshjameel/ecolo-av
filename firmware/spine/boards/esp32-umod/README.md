# spine-esp32-umod

## Overview

The spine on today's control board: the SparkFun ESP32 MicroMod (the original ESP32,
Xtensa) on the AAV carrier, beside the Motor-Pi. The Rust replacement for
`firmware/legacy/aav_umod_esp32`. **Never flashed yet.** The C++ stays until this is proven
on the car.

It only moves pins: the deciding is in `spine-core`, the devices in `spine-board`.

## Interface

### What it does

| Part | How |
|---|---|
| The brain | `spine-core`, every 10 ms |
| The devices | `spine-board`: two Jrk G2s, the DAC, the PCA9555, on I2C 21/22 at 100 kHz |
| Commands | the Motor-Pi's frames on GPIO16 (default: version 2, 17 bytes, with the Orin's heartbeat; 13-byte version 1 with `link-v1`), or CAN on TWAI (`can`) |
| RC override | iBUS on GPIO32 |
| Speed | none on this board: open loop |
| Watchdog | TIMG1, 100 ms |
| Log | the USB cable: a `STATUS` line every 200 ms, an event line at each change, an `FSM` line with the five state regions (`spine_core::fsm`) and a `LINK` line with the frame counts, with each `STATUS` |

The pin map is `src/pins.rs` (and `docs/rust-firmware.md`).

| GPIO | Use |
|---|---|
| 16 | UART1 RX, 115200 8N1: the Motor-Pi's frames |
| 17 | UART1 TX: the uplink, only with `uplink` (the wire is unconfirmed, TODO 50) |
| 32 | UART2 RX: the RC receiver's iBUS |
| 21, 22 | I2C SDA, SCL: Jrk steering 0x05, Jrk brake 0x04, DAC 0x0C, PCA9555 0x20 |
| 2, 18, 19, 14 | the status LED; the carrier's red, green, blue LEDs |
| 25, 26 | TWAI TX, RX with `can`: **a guess**, see `src/pins.rs` |
| 27 | the e-stop's spare contact with `estop-input`: **a guess** (Q6) |

The direction output is pin P00 of the PCA9555, not an ESP32 pin.

### Features

| Feature | Use |
|---|---|
| `legacy-uart` (default) | today: a drop-in for the C++, with `aav_legacy`'s `legacy_uart_bridge` |
| `link-v1` | with `legacy-uart`: the 2025-26 13-byte frame, no heartbeat (`frame_version: 1`) |
| `uplink` | with `legacy-uart`: a status frame back to the Motor-Pi every 20 ms (version 3 since 1 Nov: with both Jrks' positions in counts and the steering calibration's state), and the three-position RC mode switch. Off until the hardware lead confirms the wire |
| `can` | the KEFC plan: `firmware/can/aav.dbc`. Its pins are unknown: see `src/pins.rs` |
| `estop-input` | the e-stop's spare contact on GPIO27, once wired (Q6) |
| `qemu` | runs in Espressif's QEMU, never on the car: stand-in I2C devices, a busy-wait tick |

| File | What |
|---|---|
| `src/main.rs` | the settings, the board glue, the loop, the logs |
| `src/pins.rs` | the pin map, the one list |
| `src/can.rs` | TWAI (CAN) at 500 kbit/s, 11-bit IDs, bus-off recovery every 1 s |
| `rust-toolchain.toml` | the `esp` toolchain |

## Invariants

- **The red button always stops the car**, whatever this program does (Ben). It stays
  wired as it is; the program only reads a spare contact. No output is in its path.
- **Safe after any reset:** `SpineBoard::init` puts the throttle at 0 and the brake on;
  `spine-core` boots latched and drives only after a stop, then a drive command.
- **Exactly one link:** `legacy-uart` or `can`. Wrong mixes are `compile_error!`s
  (`link-v1` or `uplink` without `legacy-uart`, `qemu` with `can`).
- **The heartbeat (TODO 50):** with version 2, a frame whose counter hasn't changed isn't
  a command. 100 ms after the counter stops, the spine brakes, cuts the throttle and
  latches, however many frames still arrive.
- The tick is 10 ms and the watchdog 100 ms: `spine-core` expects them.
- The CAN IDs are `firmware/can/aav.dbc`'s, through `spine_core::protocol`.
- Pins live here and in `docs/rust-firmware.md`: change both together. ROS never sees one.
- Leave `estop-input` off until it is wired: a floating input would trip at random.

## Side Effects

- Drives the steering and brake Jrk G2s, the throttle DAC and the direction pin: it moves
  the car. Bench only, wheels off the ground (`docs/esp32-bench.md`).
- Reads UART1 (Motor-Pi) and UART2 (iBUS); with `uplink`, writes UART1 TX.
- With `can`: sends and receives on the CAN bus through TWAI.
- Writes logs to UART0 (the USB cable); drives the four LEDs.
- The hardware watchdog resets the chip if a tick hangs.

## Verification

```sh
pixi run esp tools        # once: Espressif's Rust, espup, espflash (needs rustup)
pixi run esp build        # or: --no-default-features --features can
pixi run esp qemu-check   # in QEMU: it brakes within 0.2 s when the heartbeat stops
pixi run esp flash        # bench only, wheels off the ground: docs/esp32-bench.md
```

It is a workspace of its own: it builds only for `xtensa-esp32-none-elf`, with the `esp`
toolchain (`rust-toolchain.toml`). `tools/test.sh` runs `tools/esp32.sh check` and the
QEMU check, where that toolchain is installed. The check runs `cargo fmt`, one build, and
clippy on six feature mixes:

| Checked | Never checked |
|---|---|
| the default; `can,estop-input`; `qemu`; `link-v1`; `uplink`; `qemu,uplink` | `link-v1` with `uplink`; `can` without `estop-input`; `estop-input` on the UART link |

The logic is tested on a laptop: `cd firmware/spine && cargo test`.

## Settings

In `src/main.rs`:

| Constant | Set to | Why |
|---|---|---|
| `TICK`, `WATCHDOG_MS` | 10 ms, 100 ms | as the Pico 2 program; `spine-core` expects them |
| `I2C_KHZ` | 100 | as the C++ |
| `FULL_THROTTLE_SPEED` | 3.0 m/s | the open loop's scale; `legacy.yaml`'s guess (Q4) |
| `IDLE_BRAKE` | 1.0 | the C++ kept the brake on while nobody drives; Ben decides |

What differs from the C++, and what Ben must decide: `docs/rust-firmware.md`.
