# sentinel-pico2

## Overview

The status display's board program, for a Raspberry Pi Pico 2 (RP2350). It is the Rust
version of the 2025-26 `AAV_sentinel` Pico 2: an I2C device at 0x42 that shows the car's
telemetry on a 16x2 LCD, an 8x8 LED matrix and a 10-LED bar, with a knob, a button and a
buzzer.

**Builds; never flashed.** The pin map, how to build and flash, and what is still
untested are in `docs/rust-firmware.md`.

## Interface

| File | What |
|---|---|
| `src/main.rs` | the settings, the pin map in code, the loop, the I2C device, the knob sampler, the buzzer |
| `memory.x` | the RP2350's memory, from Embassy's RP2350 examples |
| `build.rs`, `.cargo/config.toml` | the linker scripts, the target, the probe-rs runner |
| `PROVENANCE.md` | the 2025-26 code it replaces, and what changed |

The logic (the packet, the pages, the beeps, the knob) is in the crate `sentinel`, and
the LCD's driver in `hd44780`.

### What comes in

| Input | What |
|---|---|
| I2C0, device 0x42 | a 46-byte telemetry packet: packed, little-endian, starts `0xAA 0x01`, ends in an XOR checksum (`sentinel::telemetry`). Reads get zeros |
| The knob (KY-040) | turns the page; its switch pins it |
| The button | goes home |

Who sends the packets isn't in any repo we have: by its "Waiting Pi..." screen, a
Raspberry Pi (`PROVENANCE.md`).

### Pins

| GPIO | Use |
|---|---|
| 0, 1 | I2C0 SDA, SCL (the device), internal pull-ups |
| 2, 3, 4 | the knob: CLK, DT, switch |
| 5, 6 | the button, the buzzer |
| 7, 8, 9-12 | the LCD: RS, E, D4-D7 (4-bit) |
| 13, 14, 15 | the MAX7219 matrix on SPI1 at 1 MHz: CS, CLK, DIN |
| 28, 27, 26, 22, 21, 20, 19, 18, 17, 16 | the LED bar, LED 0 to 9 |
| 25 | the power LED: on while the program runs |

### Timings

| Thing | Value |
|---|---|
| The loop | 5 ms |
| The knob and button | read every 1 ms, in a higher-priority executor |
| Watchdog | 500 ms |

## Invariants

- The I2C address (0x42) and the packet layout stay the 2025-26 ones: the sender is
  unchanged.
- All the deciding is in `sentinel`; this program only moves pins.
- A beep never holds anything else up: the buzzer has its own task.
- The pin map is also a table in `docs/rust-firmware.md`: change both together.
- Inherited: record every change in `PROVENANCE.md`.

## Side Effects

- Answers on the I2C bus as a device at 0x42.
- Drives the LCD, the matrix, the LED bar, the buzzer and the power LED.
- Logs through defmt over a debug probe (nothing without one).
- The hardware watchdog resets the chip on a hang.

## Verification

```sh
pixi run pico sentinel build     # build it, and make sentinel-pico2.uf2 here, to drag on
pixi run pico sentinel flash   # or flash it through a debug probe (pixi run pico spine tools first)
```

`tools/test.sh` builds it (`cargo build --release --locked` here) and checks its UF2 file.
The logic is tested on a laptop: `cd firmware/spine && cargo test -p sentinel -p hd44780`.
