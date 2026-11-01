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

## Invariants

- No ROS and no raylib: only `serde`, `serde_yaml_ng` and `thiserror`.
- The e-stop sends only `true`, five times, 100 ms apart. Nothing here makes a `false`.
- A heartbeat only while the drawn-frame count rises: a frozen screen stops it.
- Topic names come from `config/hud.yaml`; a unit test checks it parses.
- SI units inside (m/s, rad, s); km/h only in text for the screen.
- Times use `Instant` (a steady clock).

## Side Effects

None: pure logic. `config/hud.yaml` is compiled in with `include_str!`.

## Verification

```sh
cd ros/hud/aav_hud && cargo test --locked -p hud-core
```

`tools/test.sh` runs it (with `map-core`).
