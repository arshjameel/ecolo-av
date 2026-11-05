# hud-core

## Overview

The in-car HUD's logic, with no ROS and no raylib, so it is tested on any laptop. The
`hud` program (`hud-app`) feeds it messages as `Update`s and draws the `HudState` it
keeps. `map-core` shares its health, origin, watchdog and formatting code.

## Interface

| Module | Holds |
|---|---|
| `config` | `HudConfig` from `config/hud.yaml` (`BUILT_IN_YAML`, built in); `Topic`, `NODE_NAME` = `hud` |
| `units` | speed in m/s and km/h, angles |
| `vehicle` | `/vehicle/status`: mode (`Mode`: MANUAL 0, RC 1, COMPUTER 2, TEST 3; any other number shows as `MODE n?`), gear letter, fault names |
| `gps` | a fix: what kind, how accurate (`Fix`, `RawFix`) |
| `health` | `LastSeen`, `Health`: OK / STALE / NEVER per topic |
| `origin` | `/system/source`: `Origin` (real, sim, bench, replay), `Origins`, the banner |
| `picture` | a camera picture as RGBA pixels |
| `estop` | `EstopButton`: `press`, `poll`; `REPEATS` = 5, `PERIOD` = 100 ms |
| `heartbeat` | `Heartbeat::poll(now, Frames)`: a beat every 100 ms while frames rise |
| `watchdog` | the sensor watchdog's verdict, as a banner |
| `gate` | `verdict(...)`: GO / WAIT / STOP and the main reason (e-stop, output mux, watchdog); `Gate::parse`, `Sensors::parse` |
| `state` | `HudState::new`, `apply(Update, now)`, `estop_pressed`, `take_new_picture` |
| `format` | numbers as short text for the screen |
| `screen` | which monitor a window opens on (`--monitor HDMI-1`) |
| `theme` | every colour of the four screens (HUD, map, debug screen, hardware panel): `Theme`, `ThemeName` (`initial_d`, the default; `mocha`, `latte`, `calm`), `Style` (glass or arcade; `corner`), `Rgba` with `contrast` (WCAG); Catppuccin's palettes (MIT) |
| `robot` | the little robot, the screens' easter egg (a tribute to WALL-E, our own drawing): `top_view` (the map's car from above, 2.5D, `TILT`), `Odometer` (turns the treads), `peek` (two eyes over a bottom edge), `side_view` (driving across); `EasterEgg::update(now, Watch)` → `Show`: when it peeks, drives, hides. Its colours: `Theme::robot()` |

## Invariants

- No ROS and no raylib: only `serde`, `serde_yaml_ng` and `thiserror`.
- The e-stop sends only `true`, five times, 100 ms apart. Nothing here makes a `false`.
- A heartbeat only while the drawn-frame count rises: a frozen screen stops it.
- Topic names come from `config/hud.yaml`; a unit test checks it parses.
- Every theme passes the contrast test: 4.5 : 1 for words, 3 : 1 for shapes (the e-stop).
  `initial_d` also passes every screen's lines and marks, and has no Catppuccin colour
  but the localizers'.
- SI units inside (m/s, rad, s); km/h only in text for the screen.
- Times use `Instant` (a steady clock).
- The little robot never shows while the car moves, an e-stop is on or a warning
  shows, and hides on any input or change (`EasterEgg`; its tests).

## Side Effects

None: pure logic. `config/hud.yaml` is compiled in with `include_str!`.

## Verification

```sh
cd ros/hud/aav_hud && cargo test --locked -p hud-core
```

`tools/test.sh` runs it (with `map-core`).
