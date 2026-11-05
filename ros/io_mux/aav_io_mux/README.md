# aav_io_mux

The sensor mux and the output mux: where the data comes from, and where the commands
go. Together they make sure a simulator's commands, or a run on simulated data, can
never drive the real car (`docs/sources-and-targets.md`, `docs/plan.md` S-4).

```
/system/source (who says REAL, SIM, BENCH, REPLAY) + the ROS graph
      -> sensor_mux -> /system/sensor_mux/state -> output_mux (the gate)
ego -> /ego/actuator_cmd -> output_mux -> /control/<target>/actuator_cmd -> one backend
                                       -> /control/<target>/heartbeat
```

## Overview

Each profile's launch file (turtle, sim, bench, car, carla) includes `io_mux.launch.py`
with its own target. The sensor mux moves no data: it checks the graph. So the heavy
topics (pictures, point clouds) cost nothing extra.

| Target | Its source | Command topic |
|---|---|---|
| `car` | REAL | `/control/car/actuator_cmd` |
| `gazebo`, `turtle`, `carla` | SIM | `/control/<target>/actuator_cmd` |
| `bench` | BENCH | `/control/bench/actuator_cmd` |

## Interface

Launch: `ros2 launch aav_io_mux io_mux.launch.py target:=car|gazebo|bench|turtle|carla [source:=real|sim|bench|replay]`.
An empty `target` starts no output mux (a replay drives nothing). An empty `source`
takes the target's. A source the target can't take is refused before anything starts.

**`sensor_mux`**

| Item | What |
|---|---|
| Subscribes | `/system/source` (`aav_interfaces/DataSource`; reliable, transient local, depth 50) |
| Publishes | `/system/sensor_mux/state` (`aav_interfaces/SourceCheck`), every `period` |
| Parameters | `source` (`real`), `period` (0.5 s), `startup` (2.0 s silent while discovery settles), `hold` (5.0 s before a graph doubt counts) |

It watches `provenance.SENSORS`: IMU, GPS, three cameras, radar, lidar, `/vehicle/odom`,
`/vehicle/wheels`, `/vehicle/status`, `/vehicle/battery`.

**`output_mux`**

| Item | What |
|---|---|
| Subscribes | `/ego/actuator_cmd` (`ackermann_msgs/AckermannDriveStamped`), `/system/sensor_mux/state`, `/system/source` |
| Publishes | `/control/<target>/actuator_cmd` (ego's command, unchanged, while OPEN); `/control/<target>/heartbeat` (`aav_interfaces/Heartbeat`, counter +1 per command passed on, a byte); `/safety/output_mux/state` (`std_msgs/String`, 2 Hz and on change: OPEN, WAIT or STOP, the target, and why) |
| Serves | `/safety/output_mux/clear` (`std_srvs/Trigger`): a person reopens a latched STOP (`pixi run clear gate`) |
| Parameters | `target` (`car`), `settle` (2.0 s), `stale_after` (3.0 s) |

Modules:

| Module | Holds |
|---|---|
| `provenance.py` | the rules, no ROS: `SOURCES`, `TARGETS`, `SENSORS`, `command_topic`, `choose`, `Announcers`, `foreign`, `check_sensors`, `Hold`, `Report`, `Gate` |
| `heartbeat.py` | no ROS: `heartbeat_topic(target)`, `Counter.beat()` (1 ... 255, 0) |
| `sensor_mux.py`, `output_mux.py` | the two ROS nodes |
| `domains.py` | `free_domain()`: a free ROS domain (150-230) for a test or check; used by the guard test, `sim_check.py`, `bench_check.py` |

## Invariants

- `provenance.py` and `heartbeat.py` have no ROS imports.
- Every rule errs towards "this is the real car, and something is wrong": an unknown
  source number is never REAL; a silent sensor mux closes the gate.
- Each sensor topic has at most one publisher, from the source asked for. An
  unannounced publisher counts as REAL (the car's stock drivers can't announce).
- An announcement counts only while its announcer still publishes `/system/source`.
- **Evidence latches STOP**: an announcement of another source, or the sensor mux's
  problems. Only `/safety/output_mux/clear` reopens it, and only once the evidence is gone.
- **Doubt only holds WAIT**: the sensor mux not heard yet or quiet for `stale_after`,
  its doubts, an announcer not heard yet. It reopens after `settle` s of all clear.
- Closed, nothing goes out; each backend's own timeout brakes (spine_bridge 0.1 s then
  the spine's 100 ms; legacy_uart_bridge 0.3 s; Gazebo 0.2 s).
- Each backend reads only its own target's topic, so a simulator's ego can't reach the car.
- The heartbeat moves only while OPEN. The ESP32 brakes, latched, when it stops for 100 ms.
- Both nodes time on a steady clock, not simulated time.

## Side Effects

- Publishes the topics above; logs each state change (STOP as an error).
- Reads the ROS graph (`get_publishers_info_by_topic`) every tick.
- `domains.py` starts probe nodes on other ROS domains and sets
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in its process.

## Verification

```sh
pixi run build && colcon test --packages-select aav_io_mux && colcon test-result --verbose
```

Quick, without ROS: `cd ros/io_mux/aav_io_mux && python3 -m pytest -q` (`tools/test.sh`
runs it). `pixi run test` runs everything.

| Test | Covers |
|---|---|
| `test_provenance.py` | the rules: choose, announcers, check, hold, gate |
| `test_heartbeat.py` | the counter wraps in a byte |
| `test_real_backend_guard.py` | real nodes on a private domain: a SIM announcement stops DRIVE_CMD frames to the real spine_bridge (needs ROS) |
