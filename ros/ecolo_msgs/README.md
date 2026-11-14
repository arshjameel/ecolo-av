# ros/ecolo_msgs

## Overview

The team's own ROS message and action types. Everything else uses stock types where it can.

## Subdirectories & Boundaries

| Package | What it holds |
|---|---|
| [aav_interfaces](./aav_interfaces/README.md) 15 messages (`VehicleStatus`, `ActuatorState`, `WheelSpeeds`, `SpineOutputs`, `Heartbeat`, `RcSticks`, `ActuatorRaw`, `SteeringCalibration`, `LinkFrame`, `UplinkHealth`, `SensorSource`, `DataSource`, `SourceCheck`, `RoadAhead`, `StopPoint`) and 2 actions (`ScriptedDrive`, `DriveToPoint`) |

## Invariants

- `firmware/can/aav.dbc` is the wire-format truth: `VehicleStatus` fault bits match CAN 0x181.
- SI units; positive steering = left; NaN = not known.
- A new type goes in `CMakeLists.txt` and `docs/interfaces.yaml`.

## Side Effects

- None at run time; the build generates the Python and C++ bindings.

## Verification

- Run: `pixi run build` (the types must generate).
