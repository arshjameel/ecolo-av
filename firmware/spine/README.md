# firmware/spine: the firmware, in Rust

## Overview

Everything the spine board runs, the car's two other Pico 2s (the wheel encoder and the
status display, the sentinel), and the CAN wheel nodes' logic. Every crate is `no_std` and
is tested on a laptop, without a board. The board programs are in `boards/`:
`esp32-umod`, today's ESP32 control board (the Rust replacement for
`firmware/legacy/aav_umod_esp32`), and three Pico 2 programs.

```text
  logic (no hardware)        drivers (embedded-hal 1.0)        glue            board programs
  spine-core, aav-inputs,    jrk-g2, ad5311, pca9555,     -->  spine-board -->  boards/esp32-umod
  ibus, legacy-link,         mcp2515, hd44780                                    boards/pico2
  wheel-encoder, sentinel,                                                       boards/wheel-encoder-pico2
  wheel-core                                                                     boards/sentinel-pico2
```

## Interface

| Crate | What it is | Tests |
|---|---|---|
| [`spine-core`](./crates/spine-core/README.md) | the spine's logic: modes, arming, the 100 ms watchdog, the speed loop (closed, or open with no wheel speed), the encoder, the CAN frames, open-loop commands, the state machine and the hardware test mode (`fsm`), and the three choices still open (below) | 120 |
| [`wheel-core`](./crates/wheel-core/README.md) | the front wheel nodes' logic: Hall sensor pulses to speed, and the WHEEL_SPEED frame | 9 |
| [`legacy-link`](./crates/legacy-link/README.md) | the 2025-26 UART link: the Motor-Pi's 13-byte frames, and the targets the C++ set for them | 72 |
| [`aav-inputs`](./crates/aav-inputs/README.md) | clean inputs: debounced switches, an e-stop that trips at once, counting with a lockout | 9 |
| [`jrk-g2`](./crates/jrk-g2/README.md) | the Pololu Jrk G2 motor controllers (steering, brake), over I2C | 15 |
| [`ad5311`](./crates/ad5311/README.md) | the throttle DAC, over I2C | 10 |
| [`pca9555`](./crates/pca9555/README.md) | the I/O expander (the direction output), over I2C | 6 |
| [`mcp2515`](./crates/mcp2515/README.md) | the CAN controller a Pico 2 needs, over SPI: bit timing from the crystal, filters, send, receive, error state | 34 |
| [`ibus`](./crates/ibus/README.md) | the FlySky RC receiver's iBUS frames | 25 |
| [`spine-board`](./crates/spine-board/README.md) | the glue: spine-core's outputs to the devices, the devices to its inputs, the `Board` trait a board program implements, and the `CommandLink` trait: CAN or the UART | 79 |
| [`wheel-encoder`](./crates/wheel-encoder/README.md) (15 Oct) | two spoke sensors, counted the 2025-26 way (the spine and the wheel encoder share it), and the wheel encoder's UDP packet | 12 |
| [`sentinel`](./crates/sentinel/README.md) (15 Oct) | the status display's logic: its I2C packet, its 13 LCD pages, the knob, the beeps, the matrix | 40 |
| [`hd44780`](./crates/hd44780/README.md) (15 Oct) | the sentinel's 16x2 character LCD, on six GPIO pins | 5 |

Each crate's own README has its API, and where its behaviour came from in the 2025-26 C++.

`wheel-core` has no board yet: since 23 Oct the hardware lead's Arduino sends the
WHEEL_SPEED frames, in Arduino C++.

| Board program | Its README |
|---|---|
| `esp32-umod` | [boards/esp32-umod](./boards/esp32-umod/README.md) |
| `pico2` | [boards/pico2](./boards/pico2/README.md) |
| `wheel-encoder-pico2` | [boards/wheel-encoder-pico2](./boards/wheel-encoder-pico2/README.md) |
| `sentinel-pico2` | [boards/sentinel-pico2](./boards/sentinel-pico2/README.md) |

## Three choices still open

Each is one constant in `spine-core` (`src/spine.rs`), set to what `docs/firmware.md`
proposes. Change one only with the team; the board program and the bench both follow.

| Constant | Set to | The 2025-26 firmware |
|---|---|---|
| `ESTOP_STEERING` | `Hold`: on any stop the spine makes, the wheels stay where they are | centred them (`Centre`) |
| `IDLE_BRAKE` | `0.0`: with nobody driving, after boot too, the brake lets go once the e-stop input clears | kept it full on (`1.0`) |
| `RC_GEAR_BRAKE` | `SpeedZero`: the gear switch's brake position asks for zero speed | full brake at once (`FullBrake`) |

## Invariants

- Every crate in this workspace is `no_std`, and builds for the Pico 2's chip
  (`thumbv8m.main-none-eabihf`). `pixi run test` checks it when that Rust target is
  installed; without it, the check is skipped and says nothing.
- Logic crates touch no hardware; drivers talk only through `embedded-hal` 1.0 or
  `embedded-io`. A board crate is the only place that knows its chip.
- `spine-core` decides; `spine-board` and the boards only carry its decisions out.
- Any e-stop, driver fault, lost RC or 100 ms without a command stops the car. The
  computer must disarm, then arm, to drive again.
- Pins, I2C addresses and calibration live here, in the firmware, never in ROS.
- The CAN frames are `firmware/can/aav.dbc`'s. `spine-core`'s `protocol` and
  `can_codec.py` share generated test vectors; so do `legacy-link` and
  `legacy_protocol.py`, and `wheel-encoder` and `aav_legacy`.
- A board crate is not a workspace member: each under `boards/` is its own workspace.

## Side Effects

The crates have none of their own: they parse, decide and encode. The board programs
drive the car: the I2C devices (steering and brake Jrks, throttle DAC, expander), CAN,
the Motor-Pi's UART, the RC receiver, UDP (the wheel encoder) and the sentinel's
display. Flashing a board changes what the car runs.

## Verification

```bash
pixi run test                          # all of it, with the rest of the repo
cd firmware/spine && cargo test        # just these crates
cargo test -p <crate>                  # one crate, from firmware/spine
cargo build --target thumbv8m.main-none-eabihf -p spine-board   # for the Pico 2's chip
```

`pixi run test` also builds every crate for the Pico 2's chip, so a crate that pulls in the
standard library fails. It builds the three board programs too, and checks their UF2 files.
It does both only when the `thumbv8m.main-none-eabihf` target is installed
(`rustup target add thumbv8m.main-none-eabihf`); without it, they are skipped silently.

## Run it without a board

Bench mode runs `spine-core` on a laptop, on the CAN frames from `spine_bridge`, driving a
model car: `pixi run bench` (`docs/bench.md`).

## The board program: `boards/esp32-umod`

The spine on today's ESP32 control board (the SparkFun ESP32 MicroMod, Xtensa), with
Embassy: the Rust replacement for the C++ that drives the car today. The same pins and
devices; commands from the Motor-Pi's UART (default) or CAN
(`--no-default-features --features can`: the two links can't both be on).
[Its own README](./boards/esp32-umod/README.md) has the details. It needs Espressif's Rust, which Pixi doesn't have:

| Command | Does |
|---|---|
| `pixi run esp-tools` | installs it for your user, once (needs rustup) |
| `pixi run esp-build` | builds the program |
| `pixi run esp-flash` | flashes it on the USB cable: bench only (`docs/esp32-bench.md`) |

Untested on the board: nothing has been flashed yet. The C++ stays until it is proven.

## The Pico 2 board programs: `boards/`

Each is a workspace of its own: it builds only for the RP2350 (a Pico 2), with Embassy.
The pin maps, the wiring rules, and what is untested: `docs/rust-firmware.md`.
**None has been flashed yet.**

| Board | Does | Build a UF2 | Flash through a probe |
|---|---|---|---|
| `boards/pico2` | the spine, on the bench: `spine-board`, and CAN through an MCP2515, every 10 ms. The car's chip is still Q10-Q13 | `pixi run spine-uf2` | `pixi run spine-flash` |
| `boards/wheel-encoder-pico2` | the wheel encoder: spokes to the Motor Pi over UDP, through a W5500. Replaces the 2025-26 C++ one | `pixi run wheel-uf2` | `pixi run wheel-flash` |
| `boards/sentinel-pico2` | the status display: an I2C device at 0x42; an LCD, a matrix, LEDs, a buzzer. Replaces the 2025-26 C++ one | `pixi run sentinel-uf2` | `pixi run sentinel-flash` |

The UF2 file lands in the board's folder: hold BOOTSEL, plug the Pico 2 in, drag it on.
Flashing through a probe needs `pixi run spine-tools` once.
