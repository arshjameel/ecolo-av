# ros/safety

## Overview

Software safety layers on top of the red button. They can only **stop** the computer's
drivers; they never move the car, and they never hold back a person on a live joystick
(the joystick-alive check stops only a silent or frozen one).

## Subdirectories & Boundaries

| Package | What it does | Main output |
|---|---|---|
| [aav_safety](./aav_safety/README.md) | the sensor watchdog: stale, NaN, frozen, jumped or disagreeing inputs, stuck steering or overspeed, lock twist_mux; the stop hold: keeps collision_monitor's stops from flickering; the geofence: every program stops before the fence's edge; the joystick-alive check: a silent or frozen joystick stops; the goal leash: a laptop driver that goes quiet loses its goals | `/safety/sensor_watchdog/lock`, `/safety/stop_hold/lock`, `/safety/geofence/lock` |
| [aav_ghost](./aav_ghost/README.md) | ghost obstacles: a pretend person only Nav2 sees, to test stopping | `/safety/ghost/scan` |
| [aav_stop_zone](./aav_stop_zone/README.md) | the stop zone in Rust, in shadow beside Nav2's collision_monitor (`stop_zone_shadow:=true`, TODO 182) | `/safety/stop_zone/shadow/cmd_vel`, `.../state` |

## Invariants

- The watchdog lock (priority 30) holds Nav2, the point driver and the AI slot only, never people or tests.
- Fail-safe: 1 s without a watchdog lock message counts as locked.
- The stop hold's lock (priority 21) holds Nav2 and the AI slot only; its silence locks nothing.
- JUMP, DISAGREE, STUCK, OVERSPEED and LOST latch until a person clears them (`/safety/sensor_watchdog/clear`).
- The geofence's lock (priority 51) holds every program, never people or the actuator
  tests; 1 s of silence locks too. OUTSIDE and JUMP latch until a person clears them.
- A ghost only ever adds an obstacle.
- The joystick-alive check only releases the joystick (a silent or frozen pad), latched until the deadman is released.

## Side Effects

- Publishes locks, state, `/diagnostics` and the ghost scan; logs state changes.

## Verification

- Run: `pixi run test` (or `colcon test --packages-select aav_safety aav_ghost`).
