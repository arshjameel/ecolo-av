# aav_rc

The RC transmitter as a ROS driver (TODO 48). A person with the RC transmitter drives
through twist_mux, ego and the output mux, so ROS's safety checks still apply.
`rc_driver` turns `/vehicle/rc` into `/drivers/rc/cmd_vel`, but only while the
transmitter's mode switch is on **RC via ROS**.

```
ESP32 reads the receiver -> legacy_uart_bridge -> /vehicle/rc -> rc_driver
  -> /drivers/rc/cmd_vel -> twist_mux (priority 95) -> ego -> output mux -> the car
```

## Overview

The mode switch (CH6; the three positions are a proposal, `docs/decisions.md`):

| Switch | Who drives | rc_driver sends |
|---|---|---|
| computer | navigation and the other programs | nothing |
| RC via ROS | a person, with the sticks, through ego | a command every tick, zeros too |
| RC direct | a person, straight from the ESP32 (fallback) | nothing |
| no signal | nobody: the ESP32 brakes | nothing |

## Interface

| Item | What |
|---|---|
| Node `rc_driver` | `ros2 launch aav_rc rc.launch.py [max_speed:=1.0] [use_sim_time:=false] [rc_impl:=rust]`: the Rust twin (`aav_rc_rs`) by default since 17 Nov; `rc_impl:=python` runs this package's |
| Subscribes | `/vehicle/rc` (`aav_interfaces/RcSticks`): mode switch, throttle and steering sticks (-1 to 1), gear on brake |
| Publishes | `/drivers/rc/cmd_vel` (`geometry_msgs/TwistStamped`), frame `base_link` |

Parameters:

| Name | Default | Meaning |
|---|---|---|
| `max_speed` | 1.0 m/s | speed at full stick forward |
| `max_reverse_speed` | 0.5 m/s | speed at full stick back (vehicle.yaml) |
| `max_steering_angle` | 0.5 rad | steering at full stick (vehicle.yaml) |
| `wheelbase` | 1.57 m | for the bicycle model (vehicle.yaml) |
| `stale` | 0.2 s | `/vehicle/rc` older than this is stale |
| `rate_hz` | 50 Hz | publish rate |
| `frame_id` | `base_link` | header frame |

The launch file reads the reverse speed, steering limit and wheelbase from
`ros/bringup/aav_description/config/vehicle.yaml`. `aav_legacy`'s `legacy.launch.py`
starts this node beside `legacy_uart_bridge`.

Modules:

| Module | Holds |
|---|---|
| `rc_drive.py` | the rules, no ROS: `Limits`, `Sticks`, `twist(sticks, limits)` -> (v, w), `RcDrive.heard()` and `RcDrive.command()` |
| `rc_driver.py` | the ROS node `RcDriver` |

## Invariants

- `rc_drive.py` has no ROS imports.
- Sends only while the switch is on RC via ROS. Leaving it: one zero, then nothing
  (as teleop_twist_joy does on its deadman).
- `/vehicle/rc` quiet for more than `stale` while on RC via ROS: zeros (a stop) until it
  comes back.
- The left stick in its brake band: zero speed (a stop through ego, not the ESP32's full brake). NaN sticks count as 0; sticks clamp to -1..1.
- Steering becomes a turn rate by the bicycle model, `w = v tan(angle) / wheelbase`.
  So the car can't steer standing still (ego holds the wheels below 0.05 m/s).
- Positive steering = left; metres, radians, seconds.
- RC direct never comes here: the ESP32 drives from the sticks itself.
- Channels and calibration stay in the firmware (`ibus` `RcMap`); ROS never sees them.

## Side Effects

- Publishes `/drivers/rc/cmd_vel` at 50 Hz while driving.
- Logs each mode-switch change, and a warning while holding a stop.
- No files, no hardware.

## Verification

```sh
pixi run build && colcon test --packages-select aav_rc && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_rc && python3 -m pytest -q` (`tools/test.sh` runs
it). `pixi run test` runs everything. `test/test_rc_drive.py` covers the switch, the stop
on silence and the bicycle model.
