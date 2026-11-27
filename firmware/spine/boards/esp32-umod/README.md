# spine-esp32-umod

## Overview

The spine on today's control board: the SparkFun ESP32 MicroMod (the original ESP32,
Xtensa) on the AAV carrier, beside the Motor-Pi. The Rust replacement for
`firmware/legacy/aav_umod_esp32`. **First bench boot 2026-11-25:** flashed to a bare ESP32
MicroMod (USB only, no devices on the I2C bus) and booted into the safe latched state —
throttle 0, brake full on, disarmed, `DEVICES 0/4`. **Not yet flashed on the car, and not
yet proven:** the C++ stays until it drives the real Jrks, DAC and direction pin on the
bench (`docs/esp32-bench.md`).

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
| Log | the USB cable: a `STATUS` line every 200 ms, an event line at each change, an `FSM` line with the five state regions (`spine_core::fsm`) and a `LINK` line with the frame counts, with each `STATUS`; once a second a `TICK` line (the worst gap between ticks and the overruns, the timing test's baseline) and a `HEALTH` line (below); at boot and every 10 s a `CONFIG` and a `DEVICES` line (below). The C++'s own lines too: its boot lines (pins, devices, `Ready`), its `STATUS:… \| STR:OK BRK:OK` every 200 ms, a `FAULT` line when the faults change, a `JRK … ERR` line when a Jrk's errors change (`docs/esp32-cpp-parity.md`). With `json-log`: the JSON status lines instead (`firmware/status`), after the boot's text lines |
| First look | after `init`, before the first tick: each of the four I2C devices probed (no data), and each Jrk's feedback mode read (a RAM setting; never its errors, which a read clears). Read only, about 1 ms, the car already safe |

The lines `pixi run esp first-check` reads (`docs/esp32-bench.md`, 2a):

| Line | Says |
|---|---|
| `CONFIG t=… build=… features=… link=uart frame=2 tick_ms=10 watchdog_ms=100 cmd_timeout_ms=100 idle_brake=1.00 full_throttle_speed=3.00 i2c_khz=100 steering_jrk=0x05 … steer_centre=1950 … brake_full=3500 … last_reset=ChipPowerOn steer_counts_per_rad=3100.0 … max_speed=1.00 … throttle_cap_code=550 … rc_lost_ms=100 … uart_baud=115200 frame_hz=50 uplink_ms=20 …` | the build and every setting it runs with (below, "Settings"); about 1 KB, so it goes out 96 bytes a tick and never holds the tick |
| `DEVICES 4/4 answered t=… steering=0x05,answered,analog brake=0x04,answered,analog dac=0x0c,answered expander=0x20,answered` | which devices answered at boot; a Jrk's feedback mode, or `unknown` |
| `HEALTH t=… link=ok age_ms=12 counter_moving=yes frames=… bad=… skipped=… repeats=… devices=4/4 faults=0x0000 mode=Manual armed=false` | the Pi link's verdict (`ok`, `no_frames`, `stale`, `counter_frozen`, `bad_frames`; `can` on CAN), judged on the board by the 100 ms rule |

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
| `legacy-uart` (default) | the frames of versions 2, 3 and 4 (with the wheel speed), today: a drop-in for the C++, with `aav_legacy`'s `legacy_uart_bridge` |
| `link-v1` | with `legacy-uart` and no `uplink` (`--no-default-features --features legacy-uart,link-v1`): the 2025-26 13-byte frame, no heartbeat (`frame_version: 1`) |
| `uplink` | with `legacy-uart`: a status frame back to the Motor-Pi every 20 ms (version 3 since 1 Nov: with both Jrks' positions in counts and the steering calibration's state), and the three-position RC mode switch. On by default (T3.6): the wire is assumed; if it is missing, only the status goes quiet. A two-position RC switch works the same |
| `can` | the KEFC plan: `firmware/can/aav.dbc`. Its pins are unknown: see `src/pins.rs` |
| `estop-input` | the e-stop's spare contact on GPIO27, once wired (Q6) |
| `qemu` | runs in Espressif's QEMU, never on the car: stand-in I2C devices, a busy-wait tick |
| `json-log` | the JSON status lines (`firmware/status`: `hb`, `status` 5 a second, `event`, `health` once a second, `config` and `devices` every 10 s) on the USB cable in place of the text lines, 96 bytes a tick so the print never waits |
| `wifi-status` | **off by default; read only.** Joins a WiFi and serves the status page, `/status.json`, `/health`, `/config`, `/scan`, a WebSocket of the status lines (10 a second) and one of the text log (`/log`). Below: "WiFi status". Not with `qemu` |
| `wifi-update` | **off by default, never in the car-day build.** `wifi-status`, plus a firmware update over WiFi on port 8080, only while parked and with the password. Below: "Update over WiFi" |
| `cpp-stop-centre` | a stop the spine makes centres the wheels, as the C++'s `jrkSafeStop()` (default: held) |
| `cpp-rc-steering` | RC right turns 1950 counts from centre, left 1550, as the C++ (default: 1550 both) |
| `cpp-throttle-ramp` | the C++'s DAC ramp: 6 codes up, 12 down a frame (default: 600 up a second, down at once). UART only |
| `cpp-parity` | the three above: the C++'s behaviour, to compare on the bench (`docs/esp32-cpp-parity.md`) |

| File | What |
|---|---|
| `src/main.rs` | the settings, the board glue, the loop, the logs |
| `src/pins.rs` | the pin map, the one list |
| `src/can.rs` | TWAI (CAN) at 500 kbit/s, 11-bit IDs, bus-off recovery every 1 s |
| `src/report.rs` | the status lines from what the tick already has (`json-log`, `wifi-status`) |
| `src/wifi.rs` | the WiFi station, its web server and WebSockets (`wifi-status`) |
| `src/update.rs` | the update over WiFi (`wifi-update`); its checks are the `app-update` crate |
| `partitions-ota.csv` | the flash layout with two OTA slots, for `wifi-update` |
| `page.html` | the status page `wifi-status` serves: graphs of steering, brake and throttle |
| `src/wifi_secrets.rs.example` | copy it to `src/wifi_secrets.rs` (git ignores it) for `wifi-status`; `UPDATE_PASSWORD` for `wifi-update` |
| `build.rs` | the build's commit and features for the `hb` line; the WiFi credentials' copy |
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
- **Reporting never steers.** `json-log` and `wifi-status` only read what the tick already
  has; they read no device for it (reading a Jrk's errors clears them) and change no input.
- **`wifi-status` is read only:** no path takes a command, a setting or a firmware update.
  It is off by default, and the car-day build (`tools/car_day_build.sh`) leaves it off.
- **`wifi-update` writes only while parked:** nobody driving, not armed, throttle 0, brake
  full on, checked at every 4 KB; only with the password; only a valid image of this
  program; the boot switches only after a full write and read-back. Never in the car-day build.
- **The default build moves the car as before:** the C++'s behaviours that change how it
  moves are features (`cpp-*`); the default only gains log lines.
- **The tick never waits on the WiFi**: lines are handed over without waiting, and a slow
  reader loses lines (a gap in `seq`), never time. The timing test below is the proof.
- **The WiFi's password is never in git, a log or a line.** It lives only in
  `src/wifi_secrets.rs` (git ignores it) and in the build's `target/` copy. A `wifi-status`
  ELF holds it: never publish one. The inherited repos' old passwords are public: never
  reuse one (Q34).

## Side Effects

- Drives the steering and brake Jrk G2s, the throttle DAC and the direction pin: it moves
  the car. Bench only, wheels off the ground (`docs/esp32-bench.md`).
- Reads UART1 (Motor-Pi) and UART2 (iBUS); with `uplink`, writes UART1 TX.
- With `can`: sends and receives on the CAN bus through TWAI.
- Writes logs to UART0 (the USB cable); drives the four LEDs.
- The hardware watchdog resets the chip if a tick hangs.
- With `wifi-update`: writes the other OTA slot of the flash, then restarts the chip.
- With `wifi-status`: the radio, as a WiFi station; a web server on port 80; with `/scan`,
  zero-length I2C writes to 0x01 to 0x77, 4 a tick, only while stopped and not armed.

## Verification

```sh
pixi run esp tools        # once: Espressif's Rust, espup, espflash (needs rustup)
pixi run esp build        # or: --no-default-features --features can
pixi run esp qemu-check   # in QEMU: it brakes within 0.2 s when the heartbeat stops
pixi run esp first-check --qemu   # the first check on the QEMU build, booted idle
pixi run esp flash        # bench only, wheels off the ground: docs/esp32-bench.md
pixi run esp first-check  # right after the flash: SAFE TO GO ON or STOP (only reads)
```

`tools/esp32_first_check.py`'s tests (`tools/tests/test_esp32_first_check.py`) read made-up
logs, both forms, and two recorded in QEMU on 7 Nov (text and `json-log`).

It is a workspace of its own: it builds only for `xtensa-esp32-none-elf`, with the `esp`
toolchain (`rust-toolchain.toml`). `tools/test.sh` runs `tools/esp32.sh check` and the
QEMU check, where that toolchain is installed. The check runs `cargo fmt`, one build, and
clippy on thirteen feature mixes, and links the biggest (`json-log,wifi-update`):

| Checked | Never checked |
|---|---|
| the default (with `uplink`); `can,estop-input`; `legacy-uart` alone; `legacy-uart,qemu`; `legacy-uart,link-v1`; `qemu,uplink`; `json-log`; `json-log,wifi-status` and `wifi-status` (with the example's made-up credentials); `json-log,wifi-update` (the same; its update password is always refused); `cpp-parity`; `can,estop-input,cpp-stop-centre,cpp-rc-steering` | `link-v1` with `uplink`; `can` without `estop-input`; `estop-input` on the UART link; `json-log` in QEMU (tried by hand, 7 Nov); the `cpp-*` features in QEMU; `wifi-update` without `json-log` |

The logic is tested on a laptop: `cd firmware/spine && cargo test`.

## Settings

In `src/main.rs`:

| Constant | Set to | Why |
|---|---|---|
| `TICK`, `WATCHDOG_MS` | 10 ms, 100 ms | as the Pico 2 program; `spine-core` expects them |
| `I2C_KHZ` | 100 | as the C++ |
| `FULL_THROTTLE_SPEED` | 3.0 m/s | the open loop's scale; `legacy.yaml`'s guess (Q4) |
| `IDLE_BRAKE` | 1.0 | the C++ kept the brake on while nobody drives; Ben decides |

What differs from the C++, and what Ben must decide: `docs/rust-firmware.md`; every C++
behaviour, and where the Rust has it: `docs/esp32-cpp-parity.md`.

### What the CONFIG line says, and where each value comes from

`board_config` in `src/main.rs` fills it from the constants the code runs with, never a
copy. Today's default build:

| Area | Keys (text form) | Today | From |
|---|---|---|---|
| Steering | `steer_min`, `steer_centre`, `steer_max` | 400, 1950, 3500 counts | `Calibration::TODAY.steering` |
| | `steer_counts_per_rad`, `steer_max_rad`, `steer_rate_rad_s` | 3100, 0.5 rad, 2.2 rad/s | the calibration; `SpineConfig` (`MAX_STEERING_ANGLE`); `STEER_RATE` here: the steering motor's own speed, filmed 27 Nov (spine-core's default is 0.6) |
| | `steer_cal_rate`, `steer_cal_timeout_ms`, `steer_on_stop` | 300 counts/s, 100 ms, `hold` | `legacy_link::CAL_RATE`, `CAL_TIMEOUT_MS`; `ESTOP_STEERING` |
| Speed | `max_speed`, `max_reverse_speed` | 1.0, 0.5 m/s | `MAX_SPEED`, `MAX_REVERSE_SPEED` |
| | `throttle_cap_code`, `reverse_cap_code` | 550, 554 of 1023 | `SpineConfig::open_loop_throttle_cap`, the code the spine caps at: the open loop from `THROTTLE_START_CODE` (475) and `THROTTLE_START_REVERSE_CODE` (525), where the wheels start (Arsh, 27 Nov) |
| | `throttle_full_code`, `frame_dac_cap`, `dac_rise_per_s` | 700, 700, 600 codes/s | the calibration (`full_code`, the DAC's own limit); `legacy_link::DAC_CAP` |
| | `full_throttle_speed`, `max_accel`, `max_decel`, `overspeed_margin` | 3.0 m/s, 1.5, 2.5 m/s², 0.25 m/s | `FULL_THROTTLE_SPEED`; `SpineConfig` |
| Brake | `brake_released`, `brake_full` | 400, 3500 counts | the calibration |
| | `idle_brake`, `brake_stop`, `brake_overspeed` | 1.0, 1.0, 0.5 | `IDLE_BRAKE` here; `SpineConfig` |
| | `rc_band`, `rc_brake_ms`, `rc_lost_below`, `rc_fitted` | 0.20, 1000 ms, 1050 µs, `true` | `ibus::RcMap` (`brake_band`, `lost_below`); `RC_BRAKE_MS`; `SpineConfig::rc_fitted` here (21 Nov) |
| Timeouts | `cmd_timeout_ms`, `watchdog_ms` | 100, 100 ms | `SpineConfig`; `WATCHDOG_MS` |
| | `rc_lost_ms`, `speed_lost_ms`, `overspeed_ms`, `estop_release_ms`, `bad_frames_ms` | 100, 300, 2000, 50, 3000 ms | the calibration; `SpineConfig`; `OVERSPEED_WINDOW_MS`; `BAD_WINDOW_MS` |
| Rates | `uart_baud`, `frame`, `frame_hz`, `uplink_ms` | 115200, version 2, 50 Hz, 20 ms | `legacy_link::BAUD`, `FRAME_HZ`; `UPLINK_PERIOD_MS` |
| | `ibus_baud`, `i2c_khz`, `can_kbit` | 115200, 100 kHz, `none` | `ibus::BAUD`; `I2C_KHZ`; `can::KBIT` |
| | `tick_ms`, `jrk_write_ms`, `dac_write_ms`, `status_ms` | 10, 10, 10, 200 ms | `TICK`: every device gets its target every tick |

An unknown value is `none` in the text form and `null` in JSON (`i2c_khz` in QEMU,
`uart_baud` on CAN).

## WiFi status (`wifi-status`)

Read only, off by default, **never flashed yet**. The plan's step 8
(`docs/designs/2026-11-07_esp32-debug-link_brief.md`): only after the timing test below.

| Path, port 80 | Gives |
|---|---|
| `/`, `/status` | `page.html`: a big banner, "Pi link: OK" (green), "NO FRAMES", "COUNTER FROZEN: is the mux running?", "BAD FRAMES"; the devices at boot; steering, brake and throttle graphs (asked, position, low to high), pause, a window, a line filter |
| `/status.json` | the latest `status` line (`{}` before the first) |
| `/health` | the latest `health` line: the link's verdict, the age of its last frame, the counter moving, frames, bad, skipped, the devices, the faults, the uptime, the last reset, the build |
| `/config` | the latest `config` line: the build, its features, the numbers it runs with |
| `/scan` | starts an I2C scan if none is going; `{"scan":"running","found":[4,5],"names":[…]}`, and the C++'s `Found: 0x05 <- JRK steer` lines in the log when done. It waits while armed or driving |
| `/ws` | a WebSocket: every line, as text; two clients at most; a slow one loses lines |
| `/log` | a WebSocket of the text log, as the C++'s on port 81 |

### Set it up

1. Ask the hardware lead for the bench WiFi's name and password. Never use the ones in the
   inherited repos (Q34).
2. `cp src/wifi_secrets.rs.example src/wifi_secrets.rs`, and put them there. Git ignores
   that file. Without it the build stops with a `compile_error!` that says this.
3. `pixi run esp build --features wifi-status` (add `json-log` to see the lines on the cable
   too). Never share that ELF: it holds the password.
4. Try the page first without a board: `python3 tools/esp32_status_sim.py`, then open
   <http://127.0.0.1:8765/>. On the board: `http://<its address>/` (its log says it).

### Sizes (7 Nov, built, not flashed)

| Build | App image | RAM, static |
|---|---|---|
| default | 157 KB | data 8.7 KB + bss 10.4 KB; the rest of DRAM is the stack |
| `wifi-status` | 793 KB of the 4 MB partition | data 17.3 KB + bss 101 KB; the radio's 72 KB heap in the bootloader's RAM (`dram2`) |
| `json-log,wifi-update` | 874 KB of a 1.9 MB OTA slot | data 17.9 KB + bss 118 KB; the same heap; 56 KB left for the stack |

The radio's heap sits in `dram2`, the RAM the bootloader used: beside the statics it
pushed them past the end of DRAM, and the link failed.

The crates: `esp-radio` 1.0.0-beta.1 (WiFi), `esp-alloc` 0.11 (its heap), `embassy-net`
0.9.1, `picoserve` 0.20.1 (HTTP and WebSocket). They resolve with esp-hal 1.2.2 and
esp-rtos 0.4.0 as they are; `build-std` builds `alloc` too.

## Update over WiFi (`wifi-update`)

Off by default, **never flashed yet**, never in the car-day build. The 2025-26 C++'s
`/update`, with the checks it didn't have (`docs/esp32-cpp-parity.md`).

| Step | How |
|---|---|
| 1 | put a password of 12 characters or more in `src/wifi_secrets.rs`'s `UPDATE_PASSWORD`; the example's is always refused |
| 2 | once, over USB: `espflash flash --partition-table partitions-ota.csv` with the `wifi-update` build. The default table has no OTA slots |
| 3 | `espflash save-image --chip esp32 <the ELF> app.bin` |
| 4 | the car parked, then `curl --data-binary @app.bin -H "X-Update-Password: …" http://<board>:8080/update` |

| Reply | Means |
|---|---|
| 200 | written, read back, the slot's SHA-256 good; it restarts in 1 s, once parked |
| 400, 413 | not an ESP32 app image of this program; too big for the slot |
| 403 | the password is wrong (wait 10 s), or none is set up |
| 409 | the car isn't parked, an update is running, or no OTA slots |
| 500 | a flash write or the read-back failed: the boot slot is unchanged |

The password crosses the WiFi in plain HTTP: the bench network only. Each 4 KB erase holds
the tick about 45 ms, while parked; a slow
flash chip's (up to 400 ms) trips the watchdog, and the chip restarts into the old program.

## Bench test: the tick's timing with the radio

The radio shares the chip with the tick. These steps measure whether it costs the tick
time. Wheels off the ground, the hardware lead there (`docs/esp32-bench.md`). Read
`worst_gap_us` and `overruns` (the `TICK` text line, or `tick` in a `status` line); write
each row down.

| # | Build and state | Watch for | Pass |
|---|---|---|---|
| 1 | default build, no radio, 10 min | the baseline | write the numbers down |
| 2 | `json-log`, 10 min | the cable log's own cost | `worst_gap_us` within 2 ms of step 1 |
| 3 | `json-log,wifi-status`, joined, no client, 10 min | the radio idle | no watchdog reset; `overruns` 0 |
| 4 | the same, the page open on a laptop, 30 min | a client streaming | `overruns` 0; `seq` gaps only if the WiFi is weak |
| 5 | the same, two clients, plus `/scan` every 10 s (disarmed) | the most load | `worst_gap_us` under 30 ms (the proposed pass number: the review, B7) |
| 6 | the router switched off for 1 min, then on, with the page open | the WiFi going away and back | no reset, no `overruns`; it joins again by itself, the page reconnects |
| 7 | step 4, armed, with the Pi's frames | the scan waits | `/scan` says "paused"; no I2C fault |

If any step fails, `wifi-status` stays off: the cable (`json-log`) carries the lines.
