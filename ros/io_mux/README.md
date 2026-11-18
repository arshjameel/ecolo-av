# ros/io_mux

## Overview

Makes sure **exactly one backend** plays the car, and the real car moves only on real data.

```
/ego/actuator_cmd ──▶ output_mux ──▶ /control/<target>/actuator_cmd + heartbeat
                         ▲
        sensor_mux ──────┘  (is every sensor from the source this run asked for?)
```

## Subdirectories & Boundaries

| Package | What it is |
|---|---|
| [aav_io_mux](./aav_io_mux/README.md) | `sensor_mux` (checks each sensor's source) and `output_mux` (one target: car, sim, bench, turtle or carla), in Python: the fall-back (`io_mux_impl:=python`) |
| [aav_io_mux_rs](./aav_io_mux_rs/README.md) | the same two nodes in Rust, the default (Ben, 17 Nov): the same names, topics and answers |

## Invariants

- An unknown source is never REAL. A silent sensor mux closes the gate.
- Evidence of a wrong source latches STOP until a person clears it.
- The heartbeat moves only while the gate is open.

## Side Effects

- Publishes the gate state and the heartbeat; reads the ROS graph each tick.

## Verification

- Run: `pixi run test` (or `colcon test --packages-select aav_io_mux aav_io_mux_rs`).
