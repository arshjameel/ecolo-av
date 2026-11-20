# aav_safety

## Overview

**Since 20 Nov this package holds the maths, the configs and the launch files; the nodes
run in Rust** ([`aav_watchdog_rs`](../aav_watchdog_rs/README.md),
[`aav_joy_alive_rs`](../aav_joy_alive_rs/README.md),
[`aav_safety_rs`](../aav_safety_rs/README.md)). The Python nodes were removed (tag
`save/python-safety-nodes`); the modules stay, as the vectors' source and for the tools
that import them (`bench_check.py`, `aav_language`, `aav_drive_to_point`).

The sensor watchdog. When a sensor navigation, the point driver or the AI slot needs dies, lies or freezes,
it stops them, and says why loudly: in the log, on `/diagnostics` and on the HUD. People and
test scripts can still drive.

And the stop hold (`stop_hold`). The stop zone (`aav_stop_zone`) decides afresh on every
command, so an obstacle at its stop zone's edge flickers it between stop and go, and the
car creeps on. The hold keeps a stop until the monitor has said "go" for `clear_time` s
without a break.

```
monitor:  STOP  go  STOP  go  go  go  go ...  (1 s of go)
lock:    true  true true true true true ...   false
```

And the joystick-alive check (`joy_alive`, TODO 74a): joy_node repeats the joystick's
last state while its receiver is in, so a pad turned off could keep driving. It sits
between joy_node and teleop_twist_joy (aav_teleop starts it) and stops the joystick when
it goes silent (0.5 s) or freezes while driving (2 s), until the deadman is released.

And the geofence (`geofence`): an area no program may leave, without navigation. It holds
every program (scripted, replay, point, navigation, AI) before the car's stopping distance
reaches the fence's edge, and while the global pose can't be trusted. People still drive
(`docs/geofence.md`).

The watchdog works through a twist_mux lock (priority 30, in `aav_command_mux`). Each input's checks:

| State | Means |
|---|---|
| `NEVER` | nothing yet |
| `STALE` | nothing good for `max_age` s. A GPS fix with no fix (or NaN) isn't good: after `max_age` it reads STALE, "no fix ..." |
| `NAN` | NaN or inf in the newest message |
| `FROZEN` | the same values for `frozen_after` s (a camera or the lidar only while the car moves) |
| `JUMP` | two GPS fixes too far apart for the car. Latched |
| `DISAGREE` | the wheels' speed or turn against the IMU and the GPS (`wheel_check.py`). Latched |
| `STUCK` | an actuator (`/vehicle/steering`) more than `follow_tolerance` off its command for `follow_persist` s (TODO 207). Latched; NaN on a side is no check, not a fault |
| `OVER` | the car more than `over_margin` over ego's speed for `over_warn` s while not slowing (TODO 207). A warning only |
| `OVERSPEED` | the same for `over_persist` s: a stuck throttle. Latched. A stale wheel speed or command is no check |
| `LOST` | a `latch` input (the HUD) dropped out after its start-up. Latched |
| `FAULT` | a `status` input: one of our nodes says `min_level` or worse in its own status (the costmap waiting for odometry, the planner finding no path). Not latched |

A person clears latched faults: `pixi run clear watchdog`. More in
`docs/simulation.md` ("Sensor failures on purpose").

## Interface

### Node `sensor_watchdog`

| In / out | Topic | Type |
|---|---|---|
| in | each input's `topic` (below) | by `kind` |
| in | `/vehicle/odom` (always; also the speed input's measured speed), `/sensors/imu/data` (for the cross check) | |
| out | `/safety/sensor_watchdog/lock`, at `rate_hz` | `std_msgs/Bool`: true while a `stop` input isn't OK; a heartbeat |
| out | `/safety/sensor_watchdog/state`, at `rate_hz` | `std_msgs/String`: `OK`, or `STOP`/`WARN` and why |
| out | `/diagnostics`, 1 Hz | `diagnostic_msgs/DiagnosticArray`: one status per input |
| serves | `/safety/sensor_watchdog/clear` | `std_srvs/Trigger`: forget latched faults |

### Node `stop_hold`

| In / out | Topic | Type |
|---|---|---|
| in | `/safety/stop_zone/state` | `nav2_msgs/CollisionMonitorState`: on each change (`aav_stop_zone`) |
| out | `/safety/stop_hold/lock`, at `rate_hz` | `std_msgs/Bool`: twist_mux's `stop_zone` lock (priority 21) |

Parameters (`config/stop_hold.yaml`): `clear_time` (s, 1.0), `rate_hz` (10).
`aav_navigation`'s `navigation.launch.py` starts it wherever it starts the stop zone.

### Node `joy_alive`

In the namespace `/drivers/joy` (aav_teleop `teleop.launch.py`).

| In / out | Topic | Type |
|---|---|---|
| in | `/drivers/joy/joy` | `sensor_msgs/Joy`: joy_node, 20 Hz while the receiver is in |
| out | `/drivers/joy/joy_checked` | `sensor_msgs/Joy`: each message, or a released copy (all 0) while stopped |
| out | `/drivers/joy/alive`, at `rate_hz` | `std_msgs/String`: `STATE - why` |

| State | Joystick | Lets go |
|---|---|---|
| `OK` | passes | |
| `WAITING` (start-up) | released | the deadman seen released once |
| `SILENT` (nothing for `silent_after`, 0.5 s) | released | the deadman seen released |
| `FROZEN` (deadman held, a `motion_axes` axis over `motion_threshold`, all values bit-identical for `frozen_after`, 2 s; 0 off) | released | the deadman seen released |

Parameters: aav_teleop `config/joy_alive.yaml`: `enable_button` 0, `motion_axes` [1],
`motion_threshold` 0.05, `silent_after` 0.5, `frozen_after` 2.0, `rate_hz` 10.

The node is [`aav_joy_alive_rs`](../aav_joy_alive_rs/README.md) (the Python one went on
20 Nov). `test_joy_alive_vectors.py` checks `joy_alive.py` still gives the answers the
Rust is checked against: after a change here, run `aav_joy_alive_rs/gen_vectors.py` (and
port it).

### Node `goal_leash` (TODO 116)

The language driver (`aav_language`) runs on a laptop and sends goals to drivers on the
car; a ROS action carries on when its client vanishes. So the laptop beats, and the leash
cancels its goals if the beats stop:

| When | The leash |
|---|---|
| no beat yet | nothing (no laptop driver) |
| beats on `/drivers/language/heartbeat` (`std_msgs/UInt64`, 1, 2, 3...) | armed |
| quiet over `timeout` (1 s) | cancels every goal on `actions` (`/scripted_drive`, `/follow_gps_waypoints`) once: the car slows to a stop the driver's way, no e-stop |
| beat 0 (goodbye: closed cleanly, its goals cancelled) | disarmed, nothing cancelled |

Started next to the mux on the car, in the sim and on the bench. Calls each action's
`_action/cancel_goal` with an empty goal id and a zero stamp (ROS's "every goal").

### Node `geofence`

| In / out | Topic | Type |
|---|---|---|
| in | `/localization/odom/global` | `nav_msgs/Odometry`: pose, heading, speed, covariance |
| in | `/localization/gps/odom` (`fix_topic`) | `nav_msgs/Odometry`: only its arrival (silent 2 s: LOST). With NDT leading (`pose_source:=ndt`), `/localization/pose_check/fix`: NDT's pose or the GPS's, whichever `ekf_global` takes |
| out | `/safety/geofence/lock`, at `rate_hz` | `std_msgs/Bool`: twist_mux's `geofence` lock (priority 51, fail-safe 1 s) |
| out | `/safety/geofence/state`, at `rate_hz` | `std_msgs/String`: `STATE: why` |
| out | `/safety/geofence/fence` | `geometry_msgs/PolygonStamped`, map frame, transient local |
| serves | `/safety/geofence/clear` | `std_srvs/Trigger`: a person clears OUTSIDE or JUMP (`pixi run clear geofence`) |

| State | Lock | Lets go |
|---|---|---|
| `OK` | open | |
| `STARTING` (under `settle_time` of good poses) | on | by itself |
| `NEAR` (a stopping path within `margin` + σ of the edge) | on | `hysteresis` further in |
| `LOST` (no pose `max_age`, σ over `max_std`, NaN, no GPS `gps_max_age`) | on | by itself |
| `OUTSIDE`, `JUMP` | on | latched: a person clears it |
| `COASTING` (the legacy speed loop's stop not measured, in either mode) | on | `vehicle.yaml` `open_loop_decel` or `pi_loop_decel`, for the running mode |
| `STANDS` (`stands:=true`) | open, logged every 5 s | |

`launch/geofence.launch.py backend:=spine|uart|sim|sim_esp32 [radius:=30]
[centre_x:=0] [centre_y:=0] [site:=] [max_speed:=] [gps:=true] [stands:=false]
[fix_topic:=/localization/gps/odom]`: it works
out the stopping numbers (`motion_from`) from `ego.yaml`, `vehicle.yaml` and the backend's
config, and passes them. `config/geofence.yaml`: `margin` 1.0, `hysteresis` 0.5, `max_std`
3.0, `max_age` 1.0, `gps_max_age` 2.0, `settle_time` 3.0, `jump_margin` 2.0, `pose_lag`
0.1, `rate_hz` 10, and `sites.<name>.datum`, `.corners`. car.launch.py starts it wherever
it starts the mux; the bench always; Ravensim with `geofence:=true` (the default).

### Inputs (`config/sensor_watchdog.yaml`)

| Name | Topic | Kind | max_age | Stops? |
|---|---|---|---|---|
| `odom` | `/vehicle/odom` | odom, with cross check | 0.5 s | yes |
| `imu` | `/sensors/imu/data` | imu | 0.5 s | yes |
| `gps` | `/sensors/gps/fix` | gps (jump: 15 m/s + 5 m) | 2.0 s | yes |
| `gps_check` | `/sensors/gps/fix` | gps, as `gps` | 2.0 s | no: with NDT leading (`pose_source:=ndt`, TODO 210) it stands in for `gps`, as the GPS no longer moves the pose |
| `pose_check` | `/localization/pose_check/status` | status, FAULT at ERROR (HOLD: neither NDT nor the GPS trusted) | 1.0 s | yes: with `pose_source:=ndt` only |
| `local`, `global` | `/localization/odom/local`, `/global` | odom | 0.5 s | yes |
| `camera` | `/sensors/camera/front_left/image_raw` | image | 1.0 s | yes |
| `front_left_stream`, `front_right_stream` | `/perception/detections` (by `frame_id`) | detections | 1.0 s | yes |
| `hud` | `/hud/heartbeat` | heartbeat, latched | 1.0 s | yes |
| `steering` | `/vehicle/steering` | actuator (STUCK: 0.15 rad for 1 s) | 1.0 s | yes; the sim only so far (not with `esp32:=true`), the car's uplink has no measured angle |
| `speed` | `/ego/actuator_cmd`, against `/vehicle/odom`'s speed | speed (0.2 m/s over while not slowing: OVER after 1 s, OVERSPEED after 2 s) | 1.0 s | yes; the sim only so far |
| `lidar` | `/sensors/lidar/points` | cloud (FROZEN: the same cloud 1 s while moving) | 1.0 s | yes, as the camera (TODO 246). Watched only where a lidar runs |
| `radar` | `/sensors/radar/points` | cloud | 1.0 s | no (the stop zone stops navigation itself) |
| `node_costmap`, `node_planner`, `node_navigator`, `node_mppi_lag`, `node_smoother`, `node_behaviour`, `node_voxels`, `node_things`, `node_ego`, `node_look_ahead` | the node's full name (`/planner`...), looked for in the ROS graph | node | 3.0 s | no: warn only (Ben, 14 Nov). The sim watches the ones it starts |
| `status_costmap_local`, `status_costmap_global` | `/drivers/nav/costmap/status` (by `status_name`) | status, FAULT at WARN | 3.0 s, 5.0 s | no: warn only (Ben, 18 Nov, TODO 171) |
| `status_planner` | `/drivers/nav/planner/status` | status, FAULT at ERROR, `quiet_ok` | 3.0 s | no |
| `status_navigator` | `/drivers/nav/navigator/status` | status, FAULT at ERROR | 2.0 s | no |
| `status_mppi_lag` | `/drivers/nav/track/status` (`track`) | status, FAULT at WARN (a late tick), `quiet_ok` | 1.0 s | no |

**Why the lidar stops the car (TODO 246).** The stop zone and the costmap see through
the roof lidar. The stop zone already stops navigation when a source goes silent, but
not the point driver or the AI slot, and it never says why on the HUD. So the lidar
stops like the camera: 1.0 s (10 clouds at 10 Hz, as the camera's 10 pictures), `stop:
true`. Each launch adds it only where a lidar runs (`lidar:=false`, the bench without
one, turtle: not watched), as the camera.

**Why the status inputs only warn.** The car already stops when a navigation node it
needs dies or hangs: each respawns after 1 s, `mppi_lag` stops on a costmap older than
`costmap_timeout`, and twist_mux holds a driver silent for its timeout. A lock here would
also hold the point driver and the AI slot for a navigation fault. So they say it on the
HUD, the dev window, `/diagnostics` and the log. **The switch:** `stop: true` on an
input makes it hold Nav2 and the AI slot while it isn't OK. The smoother has no status
topic: `node_smoother` watches it in the graph only.

Per input parameters: `topic`, `kind`, `max_age` (s), `stop`, `frozen_after` (s),
`max_jump_speed` (m/s), `jump_margin` (m), `frame_id`, `latch`, `latch_grace` (s);
for odom, `cross_check`, `cross_window` (s), `cross_speed_tolerance` (m/s),
`cross_yaw_tolerance` (rad/s), `cross_persist` (s), `cross_gps_window` (s),
`cross_gps_tolerance` (m/s); for actuator, `follow_tolerance` and `follow_persist` (s); for speed, `over_margin`
(m/s), `over_warn` (s), `over_persist` (s), `over_slowing` (m/s²); for status, `status_name`
(only its messages count; empty: all), `min_level` (1 WARN, 2 ERROR, 3 STALE: FAULT),
`quiet_ok` (silence is OK: a node that speaks only while it works). Global: `inputs`, `rate_hz` (10).

### Launch and modules

`launch/watchdog.launch.py inputs:=odom,imu,gps,local,global [use_sim_time:=false]
[watchdog_impl:=rust]`: each profile passes the inputs it has (sim, bench, car).
`watchdog_impl:=rust`, the default since Ben, 16 Nov ("watchdog in rust sure"), runs
[`aav_watchdog_rs`](../aav_watchdog_rs/README.md): the same node in Rust, with this one's
answers tick by tick; the only one since the Python node went (20 Nov).

Since 19 Nov `geofence`, `stop_hold` and `goal_leash` run in Rust by default too:
[`aav_safety_rs`](../aav_safety_rs/README.md); their Python nodes went on 20 Nov. `test_safety_rs_vectors.py` checks `geofence.py`,
`hold.py` and `leash.py` still give the answers the Rust is checked against: after a change
to one, run `aav_safety_rs/gen_vectors.py` (and port it).

| Module | No ROS? | Holds |
|---|---|---|
| `watchdog.py` | yes | `Rule`, `Watch`, `Watchdog`; the states above; `KINDS` |
| `wheel_check.py` | yes | `CrossCheck` (settings), `WheelCheck`: wheels vs IMU vs GPS |
| `hold.py` | yes | `StopHold`: locked from a stop until `clear_time` s of no stop |
| `geofence.py` | yes | `Circle`, `Polygon`, `site_fence`; `Motion`, `reaction_time`, `braking`, `backend_stale`, `measured_stop`, `coasts`, `motion_from`, `stopping_distance`, `along_arc`; `Settings`, `Pose`, `Geofence` (the states) |
| `joy_alive.py` | yes | `JoyAlive`: the joystick's `WAITING`, `SILENT`, `FROZEN` and their latch |
| `leash.py` | yes | `Leash`: armed by beats, trips once after `timeout` s quiet, disarmed by `GOODBYE` |

## Invariants

- `watchdog.py`, `wheel_check.py`, `hold.py`, `geofence.py`, `joy_alive.py` and `leash.py` have no ROS imports, and their own tests.
- `goal_leash` only ever cancels goals: it never drives, and never sets the e-stop.
- `joy_alive` only ever releases the joystick; it never makes a command of its own. Its
  latch lets go only on a released deadman, so a pad coming back with the trigger held stays stopped.
- The geofence's lock (51) holds every program, never a person or the actuator tests.
  Fail-safe: 1 s of silence holds them too. Its fence is fixed (the start spot or a
  site), so a restarted node finds an outside car outside again.
- The geofence copies no car number: its stopping numbers come from the configs.
- Only people call `/safety/geofence/clear`; it refuses while the car is outside.
- The stop hold's lock holds navigation (20) and the AI slot (10) only. Timeout 0: its silence
  locks nothing, and a `stop_hold` that dies while true keeps the stop.
- The stop hold times on the steady clock, so a jump in sim time can't end it early.
- OVER and OVERSPEED count fresh numbers only: a wheel speed older than the `speed`
  input's `max_age`, or a command after a gap longer than it, is no check. Quiet wheels
  are the `odom` input's own fault (STALE), so they never latch OVERSPEED (16 Nov).
- Speeds compare as sizes, so reversing counts too. A car that slows at 0.3 m/s² or
  more is never over: a healthy stop lags its command (the car model, `test_watchdog.py`).
- The lock holds the point driver (25), navigation (20) and the AI slot (10), never a person or a test.
- Fail-safe: twist_mux treats 1 s without a lock message as locked, so a dead or hung
  watchdog stops them too.
- Ages are about 5 messages at the sensor's rate, never under ego's 0.5 s command timeout.
- A cloud's FROZEN check (the lidar's) compares the CRC-32 of every 997th byte, as a
  picture's, and only while the wheels say over 0.2 m/s: the bench's pretend lidar has no
  noise, so a still car there sees the same cloud.
- `inputs` is set by the launch file, never in the YAML, or the file would win.
- After a stall of the watchdog itself (seen on a steady clock), it latches nothing for 3 s.
- One HUD dropout within `latch_grace` (5 s) of its first beat is forgiven.
- Each change of state is logged once.

## Side Effects

- Publishes the lock, the state and `/diagnostics`; serves `clear`.
- Logs an error when an input stops the car, a warning when it only warns.

## Verification

```sh
cd ros/safety/aav_safety && python3 -m pytest -q        # the logic, no ROS
pixi run build && colcon test --packages-select aav_safety && colcon test-result --verbose
```

`tools/test.sh` runs the first; `pixi run test` runs everything. In Ravensim:
`pixi run sim-check --faults`, or break a sensor by hand with `pixi run sim-fault gps stop`.
