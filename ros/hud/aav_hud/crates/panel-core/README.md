# panel-core

## Overview

The hardware panel's logic, with no ROS and no raylib, so it is tested on any laptop.
The `hw_panel` program (`panel-app`) feeds it every sensor's topic, `/system/source`,
the e-stop and twist_mux's output, and the person's presses; it says what to draw and
what to send. Design: `docs/designs/2026-10-29_two-screens.md`.

```
presses (Speed +/-, arrows, CENTRE, STOP), GO held ──► buttons ──► a command, or nothing
/system/source, --stands, vehicle.yaml, the locks ──► gate (on: cap / off: why)
/safety/estop, /control/cmd_vel (a takeover), a source change ──► zero, GO pressed again
```

## Interface

| Module | Holds |
|---|---|
| `config` | `PanelConfig` from `config/hw_panel.yaml` (built in); `Topic`, `Number`, `NODE_NAME` = `hw_panel` |
| `vehicle` | `Vehicle` from vehicle.yaml (built in): `wheelbase`, `max_steering_angle`, `speed_wheels` (`SpeedWheels`) |
| `rules` | `aav_actuator_tests/safety.py`, ported with its test vectors: `Kind`, `source_kind`, `preflight_refusal` (`Refusal`), `unseen` (`Sample`, `Unseen`), `taken_over` |
| `gate` | `gate(&Situation, Caps)` -> `Gate::On { cap, watch_unseen }` or `Gate::Off(Off)`: SIM and BENCH on (1.0 m/s); REAL on stands only past the preflight's refusal (0.3 m/s); REAL on the ground, unannounced, REPLAY off |
| `buttons` | `Buttons`: `press(Press)`, `set_go`, `observe_mux`, `zero(Zeroed)`, `command()` -> `Option<Command>`; `Command::from_steering` (ω = v tan δ / L) |
| `state` | `PanelState`, `Update`, `Row`: each sensor's number, OK / STALE / NEVER, rates for the big topics, `kind()`, `estop_on()`, `geofence_holds()`; `wheels_note` ("front, not driven") |

It builds on [`hud-core`](../hud-core/README.md) (`health`, `origin`, `vehicle`, `gps`,
`state::Battery`); the app uses hud-core's `estop` and `heartbeat` too.

## Invariants

- No ROS and no raylib.
- The car moves only while GO is held; letting go sends 0.5 s of zeros, then nothing.
- Speed is never below 0 (no reverse) and never above the gate's cap.
- An e-stop, a takeover (twist_mux passing someone else's command for 0.3 s), a source
  change, the gate going off, or (on stands) 4 s of an unseen speed zero everything;
  GO must be let go and pressed again, and Speed + pressed again.
- Every rule errs towards "this is the real car": nothing announced is REAL, and so is a
  source number this crate doesn't know (where `safety.py` would say SIM).
- `rules`' tests are `test_safety.py`'s vectors; change both together.

## Side Effects

- None.

## Verification

```sh
cd ros/hud/aav_hud && cargo test --locked -p panel-core
```

`tools/test.sh` runs it (with `hud-core`, `map-core` and `debug-core`).
