# firmware/status: the ESP32's status lines

## Overview

The contract for what the ESP32 reports as JSON, one object per line. The firmware writes it; the
web page, `hw_panel` and `tools/link_check.py` read it. Design:
`docs/designs/2026-11-07_esp32-debug-link_brief.md`. `golden.jsonl` holds one of each line, and every
reader and writer tests against it. New lines go at its end: readers' tests find them by
place.

## Interface

| `t` | When | Holds |
|---|---|---|
| `hb` | Once a second | Alive: uptime, boot ID, build, features, last reset, mode |
| `status` | 5 to 20 times a second | The Pi's last frame, what went to each device, low and high since the last line, positions, link counts, RC, state, tick timing |
| `event` | At each change | What changed, from and to |
| `health` | Once a second | Is it working: the Pi link's verdict, the age of its last frame, the counter moving, the link's counts, the devices at boot (answered, expected, missing), faults, mode, last reset, build, boot ID. `/health` serves the latest |
| `config` | 250 ms after boot, then every 10 s | The build, features, version, link and frame version, tick, watchdog and command timeout (ms), `idle_brake`, `full_throttle_speed`, the I2C speed (`null`: stand-ins in QEMU), the four addresses, the steering's min, centre and max, the brake's released and full, the full-throttle DAC code, last reset; then every other setting that decides how the car moves and talks (7 Nov, the table below). `/config` serves the latest |
| `devices` | 350 ms after boot, then every 10 s | Which I2C devices answered at boot (`answered`, `expected`), and each one's `name`, `address`, `answered` and `feedback` (a Jrk's feedback mode: `none`, `analog`, `frequency`, `other`, `unknown`; `null` for the others) |

| Field in `status` | Means |
|---|---|
| `rx` | The last good frame: `v`, `counter`, `check` (its last byte), `steer`, `speed`, `accel` (-1..1), `age_ms`. `null` before the first |
| `out` | What the firmware asked of each device: `steer_counts`, `brake_counts`, `dac`, `reverse` |
| `low`, `high` | The lowest and highest of `steer_counts`, `brake_counts` and `dac` since the last line, so a short spike still shows |
| `pos` | Each Jrk's position in counts, `null` when not known. Throttle has no sensor, so it has none |
| `link` | `frames`, `bad`, `skipped`, `repeats` since boot, and `verdict`: the board's own judgement of the Pi link (below) |
| `rc` | `switch`, and the raw channels |
| `state` | `mode`, `armed`, `estop`, `faults` |
| `tick` | `worst_gap_us`: the longest gap between ticks since boot. `overruns`: ticks later than 10 ms |

The `config` line's settings (7 Nov). Each comes from the constant the firmware runs with;
`null` where the build has no such thing (CAN has no UART, say). New keys were added at the
end of `steering` and `brake`, and as three groups after `last_reset`: no key moved.

| Where | Keys |
|---|---|
| `steering` | `min`, `centre`, `max` (counts); `counts_per_rad`; `max_angle_rad`; `rate_rad_s` (the spine's slew); `cal_rate` (counts/s) and `cal_timeout_ms` (the steering calibration, version 3 frames); `on_stop` (`hold` or `centre`) |
| `brake` | `released`, `full` (counts); `stop`, `overspeed` (0..1); `rc_band` (the RC left stick's brake band, 0..1 of its travel each way); `rc_brake_ms` (the RC's brake before forward and reverse change, and normal mode's window); `rc_lost_below` (µs: the mode switch under it is the transmitter's failsafe); `rc_fitted` (`true`: the RC must stay in normal mode too) |
| `speed` | `max_forward`, `max_reverse` (m/s); `cap_code`, `reverse_cap_code` (the DAC code an open-loop command can reach: the limit over `full_throttle_speed`); `frame_dac_cap` (the UART frame's scale, the C++'s `DAC_MAX_CAP`); `rise_per_s` (DAC codes/s); `max_accel`, `max_decel` (m/s²); `overspeed_margin` (m/s) |
| `timeouts` | `rc_lost_ms`, `speed_lost_ms`, `overspeed_ms`, `estop_release_ms`, `bad_frames_ms` |
| `rates` | `uart_baud`, `frame_hz` (what the Motor-Pi sends), `uplink_ms`, `can_kbit`, `ibus_baud`, `jrk_write_ms`, `dac_write_ms` (each device gets its target every tick), `status_ms` |

The text `CONFIG` line on the cable has the same settings, flat, after `last_reset`
(`steer_counts_per_rad=… max_speed=… uart_baud=…`); an unknown value there is `none`, never
`0`.

| Value | Can be |
|---|---|
| `state.mode` | `stopped`, `rc`, `computer`, `test` |
| `rc.switch` | `computer`, `via_ros`, `direct`, `no_signal` |
| `rx.v` | 1, 2 or 3. In version 1, `counter` is `null`: that frame has none |
| `rx.steer`, `speed`, `accel` | as the frame said; `null` if not a finite number (JSON has no NaN) |
| `rc.ch` | up to 14 channels; `[]` while the receiver is silent |
| `link.verdict`, `health.link` | `ok` (a new counter within 100 ms), `no_frames` (none since boot), `stale` (frames stopped), `counter_frozen` (frames come, the Orin's counter doesn't: its mux isn't running), `bad_frames` (bad frames or stray bytes in the last 3 s); `null` on CAN, which the board doesn't judge |

## Invariants

- Every line has `t`, `schema`, `seq` and `up_ms`. A gap in `seq` means lost lines: draw a gap.
- A reader ignores fields it doesn't know. A reader that doesn't know the `schema` number says so,
  and treats the values as unknown.
- Change the schema number when a field changes meaning or goes away. Change this file and
  `golden.jsonl` first.
- No password, key or WiFi name ever goes in a line: anyone on the WiFi can read them.
- The firmware builds lines from values the tick already has. It never reads a device for them:
  reading a Jrk's errors clears them.
- A slow reader loses lines. It never slows the tick.

## Side Effects

None: a contract and test data.

## Verification

Each reader's and writer's own tests read `golden.jsonl`. `tools/test.sh` runs them.

| Who | Side | Test |
|---|---|---|
| `firmware/spine/crates/status-lines` | writes (the ESP32) | `cargo test -p status-lines`: each line, byte for byte |
| `tools/esp32_status_sim.py` | writes (a pretend board, for the page and readers) | `tools/tests/test_esp32_status_sim.py`: the golden lines' fields, `/health`, `/config` |
| `ros/hud/aav_hud/crates/panel-core` | reads (`hw_panel --esp32`) | `cargo test -p panel-core`: each golden line; `health`, `config`, `devices` read as kinds it doesn't know yet |
| `tools/esp32_first_check.py` | reads (the cable, after a flash) | `tools/tests/test_esp32_first_check.py`: the golden lines as a log |
| `firmware/spine/boards/esp32-umod/page.html` | reads (the WebSocket) | none yet: tried by hand against the sim |
