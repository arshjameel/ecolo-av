# aav_command_mux

The command mux: every `/drivers/<driver>/cmd_vel` in, one `/control/cmd_vel` out, to
ego. It is the stock `twist_mux`, set up by our config and launch files: no code of our
own.

```
/drivers/joy, rc, keyboard, tests, scripted, buttons, replay, point, nav, ai /cmd_vel
        -> twist_mux (highest priority that is fresh, unless a lock holds it)
        -> /control/cmd_vel -> ego
```

## Overview

Holding the joystick deadman, stick centred, outranks every program: it stops the car.
Letting go hands control back to the next input after 0.5 s. To stop for good, press
the e-stop button, or publish `/safety/estop` true.

## Interface

| Item | What |
|---|---|
| Launch | `ros2 launch aav_command_mux mux.launch.py geofence_lock:=true\|false [use_sim_time:=false] [sensor_lock:=true]` |
| Node | `twist_mux` (package twist_mux), output `cmd_vel_out` remapped to `/control/cmd_vel` |
| Publishes | `/control/cmd_vel` (`geometry_msgs/TwistStamped`) |

Inputs (`config/twist_mux.yaml`), all `TwistStamped`, timeout 0.5 s:

| Name | Topic | Priority | Who |
|---|---|---|---|
| joy | `/drivers/joy/cmd_vel` | 100 | safety driver on the joystick (`aav_teleop`) |
| rc | `/drivers/rc/cmd_vel` | 95 (proposed) | RC transmitter on RC via ROS (`aav_rc`) |
| keyboard | `/drivers/keyboard/cmd_vel` | 90 | a laptop (`pixi run keyboard`, Foxglove) |
| tests | `/drivers/tests/cmd_vel` | 60 | `aav_actuator_tests` |
| scripted | `/drivers/scripted/cmd_vel` | 50 | `aav_scripted` |
| buttons | `/drivers/buttons/cmd_vel` | 45 | the hardware panel's buttons (`aav_hud` `hw_panel`), only while GO is held |
| replay | `/drivers/replay/cmd_vel` | 40 | `tools/replay_commands.sh` |
| point | `/drivers/point/cmd_vel` | 25 | `aav_drive_to_point` |
| nav | `/drivers/nav/cmd_vel` | 20 | Nav2 (`aav_navigation`) |
| ai | `/drivers/ai/cmd_vel` | 10 | a learned driver; today the lane keeping |

Locks (`std_msgs/Bool`; a lock holds every input below its priority):

| Name | Topic | Priority | Timeout | Notes |
|---|---|---|---|---|
| estop | `/safety/estop` | 255 | 0 (never on silence) | the backends also brake on it directly |
| geofence | `/safety/geofence/lock` | 51 | 1.0 s | `aav_safety`'s `geofence`: holds every program (the buttons too) near the fence's edge or with no trusted pose; silence locks too |
| no_timeout | `/safety/legacy_uart_bridge/lock` | 60 | 0 | `aav_legacy`: true with `frame_version: 1` (firmware with no command timeout); holds every program (the buttons too), not people or the actuator tests |
| stop_sign | `/perception/stop_sign` | 30 | 0 | `aav_detector`; holds point, Nav2 and AI |
| sensors | `/safety/sensor_watchdog/lock` | 30 | 1.0 s | `aav_safety`; silence locks too |
| stop_zone | `/safety/stop_hold/lock` | 21 | 0 | `aav_safety`'s `stop_hold`: holds collision_monitor's stops until the zone stays clear 1 s; holds Nav2 and AI only |

Launch arguments:

| Name | Default | Meaning |
|---|---|---|
| `geofence_lock` | **required** | `true`: every program needs the geofence's heartbeat. `false` (no geofence: the turtle, CARLA): loads `config/no_geofence_lock.yaml` after the others, geofence timeout 0 |
| `sensor_lock` | `true` | `true`: Nav2, the point driver and AI need the watchdog's heartbeat. `false` (no watchdog, the turtle): loads `config/no_sensor_lock.yaml`, sensors timeout 0 |
| `use_sim_time` | `false` | the simulator's clock |

## Invariants

- One output: ego obeys only `/control/cmd_vel`. Drivers never talk to the wheels.
- Highest fresh priority wins; freshness is arrival time, header stamps are ignored.
- No fresh input: nothing is published, and ego stops after its own timeout.
- Locks never hold a person (100, 95, 90); estop holds everyone.
- `no_timeout` (60) holds every program (scripted 50, buttons 45, replay 40, point, Nav2, AI) while
  the car's firmware has no command timeout. The actuator tests (60) still pass: a person
  runs them. Timeout 0: without the legacy bridge it never locks.
- `stop_sign` and `sensors` (30) never hold a test (60, 50, 40) or the buttons (45).
- `stop_zone` (21) holds only Nav2 (20) and AI (10): collision_monitor guards only Nav2.
  Timeout 0: profiles without collision_monitor never send it.
- `sensors` is fail-safe: the watchdog sends false 10 times a second, so 1 s of silence
  locks Nav2, the point driver and AI.
- `geofence` (51) holds every program (the buttons, 45, too), never a person or the actuator tests (60), and
  is fail-safe like `sensors`. `geofence_lock` has no default, so every profile says it.
- `sensor_lock` and `geofence_lock` must be `true` or `false`; anything else raises.
- The override is a second file, not a dict: a dict in the launch file silently loses
  to `twist_mux.yaml` (seen 15 Oct; `docs/conventions.md`).
- Every input topic must be a `/drivers/` topic in `docs/interfaces.yaml`.

## Side Effects

- Publishes `/control/cmd_vel`, and `/diagnostics` (each lock, once a second).
- No files, no hardware.

## Verification

No tests of its own. Others check its files:

| Test | Checks |
|---|---|
| `tools/tests/test_topic_areas.py` | twist_mux reads only `/drivers/` topics from the contract |
| `ros/bringup/aav_bringup/test/test_sim_launch.py` | the files `sensor_lock` loads, in order |

```sh
python3 -m pytest -q tools/tests/test_topic_areas.py
pixi run build && colcon test --packages-select aav_bringup && colcon test-result --verbose
```

`pixi run test` runs everything.
