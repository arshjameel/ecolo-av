# aav_interfaces

## Overview

The team's own ROS message and action types. No code: the build generates the bindings.
The Python nodes use them, and so do the Rust ones (`aav_spine_bridge`). Where a stock type fits, we use it instead (`TwistStamped`,
`Odometry`, `NavSatFix`...). `LaneGeometry` lives in
[aav_lane_geometry](../../perception/aav_lane_geometry/README.md), not here.

## Interface

### Messages (`msg/`)

Every message starts with `std_msgs/Header header`, except `SensorSource` (an entry inside
`DataSource` and `SourceCheck`, which carry the header).

| Message | Topic, publisher | Fields |
|---|---|---|
| `VehicleStatus` | `/vehicle/status`, every backend (spine_bridge: 10 Hz, CAN 0x181 with 0x180) | `mode` (`MODE_MANUAL` 0, `MODE_RC` 1, `MODE_COMPUTER` 2, `MODE_TEST` 3), `armed`, `estop_active`, `speed` (m/s, − in reverse), `steering_angle` (rad, + left), `throttle`, `brake` (0..1), `faults` (uint16 bits, below) |
| `ActuatorState` | `/vehicle/steering`, `/vehicle/throttle`, `/vehicle/brake`, the backend | `command`, `measured`; NaN = not known |
| `WheelSpeeds` | `/vehicle/wheels`, the backend | `front_left`, `front_right`, `rear_left`, `rear_right` (m/s, NaN = not measured), `direction_measured` |
| `SpineOutputs` | `/vehicle/gazebo/spine_outputs`, spine_sim, 100 Hz (simulation only) | `throttle`, `brake` (0..1), `reverse`, `steering` (rad, + left) |
| `Heartbeat` | `/control/<target>/heartbeat`, the output mux | `counter` (uint8, +1 per command passed on, wraps 255 → 0) |
| `RcSticks` | `/vehicle/rc`, legacy_uart_bridge, 50 Hz | `mode_switch` (`SWITCH_NONE` 0, `SWITCH_COMPUTER` 1, `SWITCH_RC_VIA_ROS` 2, `SWITCH_RC_DIRECT` 3), `throttle`, `steering` (−1..1, + forward, + left), `brake` |
| `ActuatorRaw` | `/vehicle/steering_actuator/raw`, `/vehicle/brake_actuator/raw`, legacy_uart_bridge, 50 Hz | `valid`, `position` (counts, 0..4095: the Jrk's scaled feedback, not converted), `calibration` (`CALIBRATION_*`: the firmware's steering calibration state) |
| `SteeringCalibration` | `/drivers/buttons/steering_calibration`, hw_panel, 10 Hz while held | `mode` (`MODE_OFF` 0, `MODE_LIMP` 1, `MODE_JOG` 2), `target` (counts, for jog) |
| `SensorSource` | (inside the two below) | `name`, `topic`, `source` (`REAL` 0, `SIM` 1, `BENCH` 2, `REPLAY` 3) |
| `DataSource` | `/system/source`, latched, each data node | `announcer`, `sensors` (`SensorSource[]`) |
| `SourceCheck` | `/system/sensor_mux/state`, the sensor mux, 2 Hz | `source`, `ok`, `problems[]`, `doubts[]`, `sensors` (`SensorSource[]`) |

`VehicleStatus` fault bits, the same as `faults` in CAN 0x181:

| Bit | Name | Bit | Name |
|---|---|---|---|
| 1 | `FAULT_CMD_TIMEOUT` (no DRIVE_CMD for 100 ms) | 32 | `FAULT_THROTTLE_DAC` |
| 2 | `FAULT_BAD_CHECKSUM` | 64 | `FAULT_WHEEL_ENCODER` |
| 4 | `FAULT_BAD_COUNTER` | 128 | `FAULT_CAN_BUS` |
| 8 | `FAULT_STEERING_DRIVER` | 256 | `FAULT_RC_SIGNAL_LOST` |
| 16 | `FAULT_BRAKE_DRIVER` | 512 | `FAULT_OVERSPEED` |

### Actions (`action/`)

| Action | Server | Goal | Result | Feedback |
|---|---|---|---|---|
| `ScriptedDrive` | aav_scripted `scripted_driver` | `target_speed` (m/s, − reverse), `acceleration`, `deceleration` (m/s²), `hold_time` (s), `hold_distance` (m, 0 = use time), `curvature` (1/m, + left) | `success`, `message`, `distance_travelled` (m), `duration` (s) | `phase` (`ACCELERATE`, `HOLD`, `DECELERATE`, `STOPPED`), `commanded_speed`, `measured_speed`, `distance_travelled` |
| `DriveToPoint` | aav_drive_to_point `point_driver` | `use_lat_lon`, `latitude`, `longitude` (deg), `x`, `y` (m, map), `cruise_speed` (m/s), `acceleration`, `deceleration` (m/s², 0 = default), `confirmed` | `success`, `message`, `target_x`, `target_y`, `distance_left`, `tolerance` (m), `distance_travelled` (m), `duration` (s) | `phase` (`TURN`, `ACCELERATE`, `HOLD`, `DECELERATE`, `STOPPING`), `distance_left`, `cross_track` (m, + left), `commanded_speed`, `measured_speed` |

## Invariants

- `firmware/can/aav.dbc` is the source of truth for the wire format. `VehicleStatus`'s
  fault bits must match its `faults` signal.
- SI units and REP-103: m, rad, s; + steering and + curvature turn left.
- NaN means "not known" in `ActuatorState` and `WheelSpeeds`.
- `Heartbeat`'s counter only rises while the output mux's gate is open. The ESP32 brakes
  when it hasn't changed for 100 ms.
- `DriveToPoint` with `/system/source` REAL needs `confirmed` (a person said yes).
- A new type goes in `CMakeLists.txt`'s `rosidl_generate_interfaces` list, and in
  `docs/interfaces.yaml` where a topic uses it.

## Side Effects

None at run time. The build generates the bindings.

## Verification

This package has no tests of its own; the packages that use the types test them.

```sh
pixi run build && colcon test --packages-select aav_interfaces && colcon test-result --verbose
ros2 interface show aav_interfaces/msg/VehicleStatus      # after the build
```

`pixi run test` runs everything.
