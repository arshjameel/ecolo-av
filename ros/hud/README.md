# ros/hud

## Overview

The screens in the car: one package, `aav_hud`, written in Rust with raylib (Ben, 15 Oct,
Q29). The 2025-26 Flutter app and its Flask back end were archived on 25 Oct
(`archive/2026-27-flutter-hud/`).

## Subdirectories & Boundaries

| Package | What it is |
|---|---|
| [aav_hud](./aav_hud/README.md) | four raylib screens: `hud` (status, e-stop, GO/STOP), `map` (Nav2 map, place search, `/goal_pose`), `debug_hud` (the seeing screen: the lidar's cubes, NDT's map and match, the localizers; key L every sensor; listens only) and `hw_panel` (every sensor's number, simple buttons through twist_mux's slot 45). Logic crates: [hud-core](./aav_hud/crates/hud-core/README.md), [map-core](./aav_hud/crates/map-core/README.md), [debug-core](./aav_hud/crates/debug-core/README.md), [panel-core](./aav_hud/crates/panel-core/README.md) |

## Invariants

- A HUD only **sets** the e-stop (`/safety/estop` true, 5 × 0.1 s), never clears it.
- A goal goes to Nav2 only after a person confirms it.
- `hw_panel`'s buttons drive only while a person holds GO, only through twist_mux, and
  only SIM or BENCH until TODO 49b.
- Topic names live in config files (`aav_hud/config/`), not in code.

## Side Effects

- Opens windows (needs a screen).
- Publishes the e-stop, `/hud/heartbeat`, goals and the buttons' command.

## Verification

- Run: `pixi run test` (also runs `cargo test -p hud-core -p map-core -p debug-core -p panel-core`).
