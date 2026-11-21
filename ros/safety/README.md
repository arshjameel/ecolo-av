# ros/safety

## Overview

Software safety layers on top of the red button. They can only **stop** the computer's
drivers; they never move the car, and they never hold back a person on a live joystick
(the joystick-alive check stops only a silent or frozen one).

## Subdirectories & Boundaries

| Package | What it does | Main output |
|---|---|---|
| [aav_safety](./aav_safety/README.md) | the sensor watchdog: stale, NaN, frozen, jumped or disagreeing inputs, stuck steering or overspeed, lock twist_mux; the stop hold: keeps the stop zone's stops from flickering; the geofence: every program stops before the fence's edge; the joystick-alive check: a silent or frozen joystick stops; the goal leash: a laptop driver that goes quiet loses its goals | `/safety/sensor_watchdog/lock`, `/safety/stop_hold/lock`, `/safety/geofence/lock` |
| [aav_joy_alive_rs](./aav_joy_alive_rs/README.md) | the joystick-alive check in Rust, the default (Ben, 17 Nov): the Python's answers, event by event; `joy_alive_impl:=python` for the Python | `/drivers/joy/joy_checked`, `/drivers/joy/alive` |
| [aav_watchdog_rs](./aav_watchdog_rs/README.md) | the sensor watchdog in Rust, the default (Ben, 16 Nov): the Python's answers, tick by tick; `watchdog_impl:=python` for the Python | the same as the Python's |
| [aav_ghost](./aav_ghost/README.md) | ghost obstacles: a pretend person only navigation sees, to test stopping | `/safety/ghost/scan` |
| [aav_stop_zone](./aav_stop_zone/README.md) | the stop zone in Rust, the car's since 17 Nov: zero the command for points in the box ahead (behind, reversing) or a silent sensor; Nav2's collision_monitor ported, swapped after its shadow run (TODO 182) | `/drivers/nav/cmd_vel`, `/safety/stop_zone/state` |
| [aav_safety_filter_rs](./aav_safety_filter_rs/README.md) | the safety filter for the automatic drivers (TODO 104): trims a command by the least that keeps the car able to stop and inside its lane (CBFs), with a pull to the lane's line (a Lyapunov term); in front of twist_mux (K1), in shadow by default | `/safety/filter/status`, `/safety/filter/shadow/<d>/cmd_vel` |

## Invariants

- The watchdog lock (priority 30) holds navigation, the point driver and the AI slot only, never people or tests.
- Fail-safe: 1 s without a watchdog lock message counts as locked.
- The safety filter only trims an automatic driver's command (never faster, never reversed); it is not a lock, and a person never goes through it.
- The stop hold's lock (priority 21) holds navigation and the AI slot only; its silence locks nothing.
- JUMP, DISAGREE, STUCK, OVERSPEED and LOST latch until a person clears them (`/safety/sensor_watchdog/clear`).
- The geofence's lock (priority 51) holds every program, never people or the actuator
  tests; 1 s of silence locks too. OUTSIDE and JUMP latch until a person clears them.
- A ghost only ever adds an obstacle.
- The joystick-alive check only releases the joystick (a silent or frozen pad), latched until the deadman is released.

## Side Effects

- Publishes locks, state, `/diagnostics` and the ghost scan; logs state changes.

## Verification

- Run: `pixi run test` (or `colcon test --packages-select aav_safety aav_ghost`).
