# wheel-encoder-pico2

## Overview

The wheel encoder's board program, for a Raspberry Pi Pico 2 (RP2350) with a WIZnet
W5500 Ethernet chip. It is the Rust version of the 2025-26 `Wheel_Encoders` Pico 2: it
counts the wheel's spokes, and sends the count to the Motor Pi over UDP every 100 ms,
byte for byte as before. `aav_legacy`'s `encoder_udp` reads it unchanged.

**Builds; never flashed.** The pin map, how to build and flash, and what is still
untested are in `docs/rust-firmware.md`.

## Interface

| File | What |
|---|---|
| `src/main.rs` | the settings (MAC, addresses, ports), the pin map in code, the sampler, the sender, the watchdog |
| `memory.x` | the RP2350's memory, from Embassy's RP2350 examples |
| `build.rs`, `.cargo/config.toml` | the linker scripts, the target, the probe-rs runner |
| `PROVENANCE.md` | the 2025-26 code it replaces, and what changed |

The logic (the spoke counting, the packet) is in the crate `wheel-encoder`.

### The packet

| Field | Value |
|---|---|
| Body | 4 bytes, a big-endian `int32`: spokes since the last packet, positive forward |
| Rate | every 100 ms |
| To | 192.168.1.193 (the Motor Pi), UDP port 10111 |
| From | UDP port 11101 |
| Address | DHCP; MAC 54:65:6d:70:65:6c ("Tempel") |

Examples both sides check: `firmware/udp/wheel_encoder_vectors.txt`.

### Pins

| GPIO | Use |
|---|---|
| 2, 10 | spoke sensors A, B (no pull: the IR modules drive them) |
| 16, 17, 18, 19 | SPI0 to the W5500 at 50 MHz: MISO, CS, SCK, MOSI |
| 20, 21 | W5500 reset, interrupt |
| 25 | the LED: slow blink with an address, fast while waiting for one |

## Invariants

- The packet, the ports, the address and the MAC stay the 2025-26 ones, so
  `encoder_udp` needs no change.
- The counter only runs on; the sender remembers what it sent. No spoke is lost between
  reading and clearing.
- 8 spokes a turn; sensors read every 1 ms, debounced 10 ms (`aav_inputs::Lockout`).
- It waits for DHCP, and keeps trying.
- The pin map is also a table in `docs/rust-firmware.md`: change both together.
- Inherited: record every change in `PROVENANCE.md`.

## Side Effects

- Sends a UDP packet to 192.168.1.193:10111 every 100 ms; asks for a DHCP lease.
- Drives the LED.
- Logs through defmt over a debug probe (nothing without one).
- A 1 s hardware watchdog, fed after every packet, resets the chip on a hang.

## Verification

```sh
pixi run pico wheel build     # build it, and make wheel-encoder-pico2.uf2 here, to drag on
pixi run pico wheel flash   # or flash it through a debug probe (pixi run pico spine tools first)
```

`tools/test.sh` builds it (`cargo build --release --locked` here) and checks its UF2 file.
The logic is tested on a laptop: `cd firmware/spine && cargo test -p wheel-encoder`.
