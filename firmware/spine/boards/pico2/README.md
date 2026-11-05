# spine-pico2

## Overview

The spine's board program, for a Raspberry Pi Pico 2 (RP2350) on the bench. It runs
`spine-board` and `spine-core` on the chip, with Embassy, and talks plain CAN to
`spine_bridge` through an MCP2515. The car's own chip is still open (Q10-Q13).
**Never flashed yet.**

The pin map, how to build and flash, and what is still untested are in
`docs/rust-firmware.md`.

## Interface

| File | What |
|---|---|
| `src/main.rs` | the settings, the pin map in code, the loop, the wheel sampler, the watchdog |
| `src/can.rs` | CAN through the MCP2515: send, receive, and keeping the chip up |
| `memory.x` | the RP2350's memory, from Embassy's RP2350 examples |
| `build.rs`, `.cargo/config.toml` | the linker scripts, the target, the probe-rs runner |

### What it does

| Part | How |
|---|---|
| The loop | `SpineBoard::tick` every 10 ms |
| CAN | MCP2515 on SPI0 at 4 MHz, 500 kbit/s, polled each tick. Its filters keep only `ESTOP_CMD` (0x010) and `DRIVE_CMD` (0x100) |
| The devices | the two Jrk G2s, the DAC and the expander on one I2C0 bus at 100 kHz |
| The RC receiver | iBUS on UART1, receive only, through an interrupt-filled buffer |
| The wheel | two spoke sensors, read every 1 ms by a higher-priority task (`SpokeSensors::TODAY`) |
| The e-stop | a spare contact of the button, read only |
| Watchdog | fed after every tick: a hang resets the chip within 100 ms |
| The LED | blinks slowly while all runs, fast while CAN isn't up |

### Pins

| GPIO | Use |
|---|---|
| 16, 17, 18, 19 | SPI0 to the MCP2515: SO (MISO), CS, SCK, SI (MOSI) |
| 4, 5 | I2C0 SDA, SCL |
| 9 | UART1 RX: iBUS |
| 10, 11 | the wheel's spoke sensors A, B (pull-ups) |
| 15 | the e-stop's spare contact (pull-up): high is open (TODO Q6) |
| 25 | the LED |

### Settings

| Constant | Set to | Why |
|---|---|---|
| `CAN_CRYSTAL` | 8 MHz | read it off the MCP2515 module's can (8 or 16) |
| `CAN_MODE` | `Normal` | `Loopback` checks the SPI wiring with no bus |
| `CAN_CHECK_MS`, `CAN_RETRY_MS` | 100, 1000 | the chip's health check; a retry while it is down |
| `ESTOP_OPEN` | `High` | the contact closes to ground while the button is out |

## Invariants

- **The red button always stops the car** (Ben). It stays wired as it is; the program
  only reads a spare contact and tells ROS. No output is in its path.
- **Safe after any reset:** `SpineBoard::init` puts the throttle at 0; `spine-core`
  boots latched, and drives only once ROS disarms, then arms.
- **The brake after a reset:** `init` puts it on, but this board uses
  `SpineConfig::default()`, whose `IDLE_BRAKE` is `0.0` (the setting `docs/firmware.md`
  proposes). So the brake lets go on the first tick. `esp32-umod` sets `1.0` instead.
- The tick is 10 ms and the watchdog 100 ms: `spine-core` expects them.
- The CAN IDs and bitrate are `firmware/can/aav.dbc`'s (`spine_core::protocol`).
- The spine works without CAN: the RC still drives. A chip that doesn't answer is tried
  again every second; one that reset itself is set up again.
- The pin map is also a table in `docs/rust-firmware.md`: change both together.

## Side Effects

- Drives the steering and brake Jrk G2s, the throttle DAC and the direction output.
- Sends and receives CAN frames through the MCP2515.
- Logs through defmt over a debug probe (nothing without one).
- The hardware watchdog resets the chip on a hang.

## Verification

It is a workspace of its own, not a member of `firmware/spine`'s: it builds only for the
RP2350 (`thumbv8m.main-none-eabihf`), with Embassy's HAL for that chip.

```sh
pixi run pico spine build         # build it, and make spine-pico2.uf2 here, to drag on
pixi run pico spine flash       # or flash it through a debug probe (pixi run pico spine tools first)
cd firmware/spine/boards/pico2 && cargo build --release   # pixi run test does this too
```

`tools/test.sh` builds it and checks its UF2 file. The logic is tested on a laptop:
`cd firmware/spine && cargo test -p spine-board -p spine-core -p mcp2515`.
