# ros/command_mux

## Overview

Picks **one** command for ego from all the drivers: the stock twist_mux, set up for the car.

```
/drivers/*/cmd_vel ──▶ twist_mux (+ locks) ──▶ /control/cmd_vel ──▶ ego
```

## Subdirectories & Boundaries

| Package | What it is |
|---|---|
| [aav_command_mux](./aav_command_mux/README.md) | twist_mux config: nine inputs (joy 100 … ai 10), locks `estop` (255), `no_timeout` (60), `geofence` (51), `stop_sign` and `sensors` (30), `stop_zone` (21) |

## Invariants

- The highest fresh input wins (0.5 s timeout).
- Locks never hold a person; only the e-stop holds everyone.
- The no-timeout lock (60) holds every program while the car's firmware has no command
  timeout (`frame_version: 1`); people and the actuator tests still drive.
- The sensors and geofence locks lock after 1 s of silence (fail-safe).
- The geofence lock (51) holds every program; people and the actuator tests still drive.

## Side Effects

- Publishes `/control/cmd_vel` and lock diagnostics.

## Verification

- Run: `pixi run test` (`tools/tests/test_topic_areas.py` and `aav_bringup`'s launch tests check the config).
