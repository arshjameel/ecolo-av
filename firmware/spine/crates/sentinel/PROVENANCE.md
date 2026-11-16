# Provenance

The status display's logic, from the 2025-26 team's "sentinel" Pico 2, rewritten in Rust
(not transliterated) on 15 Oct 2026. The board program that uses it, and the full list of
what changed: `boards/sentinel-pico2/PROVENANCE.md`.

| | |
|---|---|
| Repo | github.com/AAV-2025-2026/AAV_cortex (public) |
| Branch | `6-pico2-aav-firmware` |
| Commit | `90de619e52` |
| Folder | `AAV_sentinel/AAV_sentinal_pico` |

| Here | Was |
|---|---|
| `telemetry` | `aav_telemetry.hpp`: the packed struct, its enums, `aav_telem_checksum()`; `process_telemetry()`'s checks |
| `pages` | `render_page()` and its helpers |
| `controls` | `pot_irq()` (the knob's table and the buttons' 200 ms lockout) |
| `alerts` | the `beep_*()` patterns; `handle_comms_watchdog()`; the alerts in `process_telemetry()` |
| `matrix` | the `pat_*` pictures; `matrix_tick()` |
| `Sentinel` | `handle_input()` (pages, pin, home, the 10 s timeout) |
| the `hd44780` crate | `lcd_display.cpp` (the LCD's 4-bit start-up and commands, from the datasheet) |

## Changes since

| When | What |
|---|---|
| 15 Nov (TODO 176) | the Rust API Guidelines pass: `#[must_use]` on pure functions, doc comments, `Debug` derives. No change in behaviour |
