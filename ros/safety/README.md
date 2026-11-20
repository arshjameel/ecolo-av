# ros/safety

## Overview

Software safety layers on top of the red button. They can only **stop** the computer's
drivers; they never move the car, and they never hold back a person on a live joystick
(the joystick-alive check stops only a silent or frozen one).

## Subdirectories & Boundaries

| Package | What it does | Main output |
|---|---|---|
| [aav_safety](./aav_safety/README.md) | the safety nodes' maths (Python, no ROS), configs and launch files; the nodes run in Rust (below). The sensor watchdog: stale, NaN, frozen, jumped or disagreeing inputs, stuck steering or overspeed, lock twist_mux; the stop hold: keeps the stop zone's stops from flickering; the geofence: every program stops before the fence's edge; the joystick-alive check: a silent or frozen joystick stops; the goal leash: a laptop driver that goes quiet loses its goals | `/safety/sensor_watchdog/lock`, `/safety/stop_hold/lock`, `/safety/geofence/lock` |
| [aav_joy_alive_rs](./aav_joy_alive_rs/README.md) | the joystick-alive check in Rust, the only one (Ben, 17 Nov; the Python node removed 20 Nov): the Python's answers, event by event | `/drivers/joy/joy_checked`, `/drivers/joy/alive` |
| [aav_watchdog_rs](./aav_watchdog_rs/README.md) | the sensor watchdog in Rust, the only one (Ben, 16 Nov; the Python node removed 20 Nov): the Python's answers, tick by tick | the same as the Python's |
| [aav_safety_rs](./aav_safety_rs/README.md) | the geofence, the stop hold and the goal leash in Rust, the only ones (19 Nov; the Python nodes removed 20 Nov): the Python's answers, to the bit | the same as the Python's |
| [aav_self_test_rs](./aav_self_test_rs/README.md) | the boot self-test (TODO 255): every sensor (through the watchdog), every actuator, every lock alive, the clocks; `spine_bridge` refuses to arm until READY. It only reads | `/safety/self_test/status` |
| [aav_ghost](./aav_ghost/README.md) | ghost obstacles: a pretend person only navigation sees, to test stopping | `/safety/ghost/scan` |
| [aav_stop_zone](./aav_stop_zone/README.md) | the stop zone in Rust, the car's since 17 Nov: zero the command for points in the box ahead (behind, reversing) or a silent sensor; Nav2's collision_monitor ported, swapped after its shadow run (TODO 182) | `/drivers/nav/cmd_vel`, `/safety/stop_zone/state` |

## Invariants

- The watchdog lock (priority 30) holds navigation, the point driver and the AI slot only, never people or tests.
- Fail-safe: 1 s without a watchdog lock message counts as locked.
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
