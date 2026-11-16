# Provenance

The rules here come from the 2025-26 team's wheel encoder, rewritten in Rust (not
transliterated) on 15 Oct 2026. The board program that uses it, and the full list of
what changed: `boards/wheel-encoder-pico2/PROVENANCE.md`.

| | |
|---|---|
| Repo | github.com/AAV-2025-2026/Wheel_Encoders (public) |
| Branch | `main` |
| Commit | `f824e9d805` |
| From | `wiznet.ino`: `sensor_callback()` (the counting), `loop()` and `pack_be_int32()` (the packet) |

| Here | Was |
|---|---|
| `SpokeSensors` | `sensor_callback()`: A's rising edge counts; backwards if B rose in the last 20 ms and is still high; 10 ms debounce. Moved here from `spine-board` (11 Oct) on 15 Oct, so both boards share it |
| `SpokeDelta::to_bytes` | `pack_be_int32()`: a big-endian int32 |
| `Reporter` | `loop()`: read the counter, then set it to 0, every 100 ms. Now the counter runs on, and the reporter subtracts |

## Changes since

| When | What |
|---|---|
| 15 Nov (TODO 176) | the Rust API Guidelines pass: `#[must_use]` on pure functions, doc comments, `Debug` derives. No change in behaviour |
