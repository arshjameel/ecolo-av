# aav_watchdog_rs

## Overview

The sensor watchdog in Rust (rclrs): the **default**, in place of `aav_safety`'s Python
`sensor_watchdog`. The same node name, topics, QoS, parameters and answers. Ben, 14 Nov:
"convert anything that uses a lot of resources to rust" (TODO 185).

**A safety lock.** Ben decided (16 Nov: "watchdog in rust sure"): this one is the default
on the car and in the simulator. The Python was removed on 20 Nov (tag `save/python-safety-nodes`). until a car day
proves it.

```
each input's topic (odom, IMU, GPS, cameras, lidar, detections, actuators, nodes...)
        │
        ▼
 sensor_watchdog ──► /safety/sensor_watchdog/lock  (10 Hz): true = twist_mux holds nav, AI
                 ──► /safety/sensor_watchdog/state (10 Hz): OK, or STOP/WARN and what
                 ──► /diagnostics (1 Hz)
                 ◄── /safety/sensor_watchdog/clear (Trigger): a person forgets the latches
```

Pick it with `watchdog_impl:=rust` (`watchdog.launch.py`, `sim.launch.py`). What it
checks, its states and which latch: `aav_safety`'s README.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/watchdog-core` | the logic: every check, the latches, the wheels' cross check, the tick, `/diagnostics`' words | no: `cargo test` on any laptop |
| `crates/watchdog-node` | the node, binary `sensor_watchdog` | yes (rclrs 0.8, `ros-env` 0.3) |

colcon builds it through `CMakeLists.txt` (`cargo build --release --locked`), installed as
`lib/aav_watchdog_rs/sensor_watchdog`. It reuses `aav_ego_rs`'s `ego-core` (Python's
`hypot` and `%g`). The launch file gives it `aav_safety`'s `config/sensor_watchdog.yaml`.

### Node `sensor_watchdog`

The same as the Python's (`aav_safety`'s README): every topic, QoS, parameter and default,
and the clear service. Nothing added.

### `watchdog-core`

| Module | Holds | From |
|---|---|---|
| `watch` | `Watchdog`, `Watch`, `Rule` (`checked()` → `RuleError`), `Kind`, `State` (enums), `Verdict` | `watchdog.py` |
| `wheels` | `WheelCheck`, `CrossCheck`: the wheels against the IMU and the GPS | `wheel_check.py` |
| `feed` | `Feed`: the node without ROS: each message in, `tick()` → states, lock, state line, `Said`; `diagnostics()` | `sensor_watchdog.py` |
| `lib` | `py_sum`, `py_max`, `py_min`, `crc32`, `picture_digest` | Python's own |

## Invariants

- **The same answers as the Python, exactly**: `tests/vectors.rs` reads `vectors.json`,
  which `gen_vectors.py` writes by running the Python node's logic. Every tick: each input's
  state and words, whether it stops, the lock, the state line, what is logged, and
  `/diagnostics`. No tolerance: text compared as text. Change one, change both.
- The same latches: JUMP, DISAGREE, STUCK, OVERSPEED and LOST hold until the clear service.
- Same node name and topics: nothing downstream can tell which runs. Only one runs.
- Fail-safe as the Python: it only ever locks; twist_mux counts 1 s of silence as locked.
- `watchdog-core` has no ROS. Rust API Guidelines: enums (`Kind`, `State`, `Level`, `Said`,
  `Digest`), `Result` (`RuleError`, `CrossCheckError`), no `unwrap` in library code;
  `cargo fmt` and `cargo clippy -D warnings` clean. `Cargo.lock` committed.

## How it matches the Python

`gen_vectors.py` plays 4 runs of 400 ticks (10 Hz) through the Python's logic, with faults
on purpose: frozen wheels while speeding up, NaN, silences, a lost GPS fix and a jump, a
frozen camera, a silent and a frozen lidar (TODO 246), the HUD dropping out, stuck steering, overspeed, a node gone, a stalled
node, a costmap that waits then hangs, a planner that finds no path (status inputs, TODO 171; one run with `stop: true`), and a clear.

| Compared | Ticks | Different |
|---|---|---|
| every input's state, words and stop; the lock; the state line; what is logged | 1,600 | 0 |
| `/diagnostics`: level, message, hardware id, every value | once a second | 0 |
| states reached | all 12 (FAULT since 18 Nov) | |
| a picture's digest against `zlib.crc32` | random pictures | 0 |

## Measured

Our simulator's road world, headless (`pixi run sim-check`, through `tools/ros.sh
--local`), Ben's MacBook (M4 Max), 16 Nov. CPU: the node's CPU seconds a second over about
29 s of the run, sampled with `ps` every 2 s.

| `/clock` 100 Hz | Python `sensor_watchdog` | Rust `sensor_watchdog` |
|---|---|---|
| share of a core | **9.9 %** | **1.4 %** |

| sim-check, `watchdog_impl:=` | Python | Rust |
|---|---|---|
| the road world (start-up, contract, sensors, lidar, scripted drive, EKF, wheels vs IMU and GPS) | pass | pass |
| `--actuator-faults`: brake weak or dead, throttle stuck, steering stuck | pass: OVERSPEED 4.6 s after the stuck throttle, steering STUCK 4.1 s after | pass: 4.7 s and 4.1 s |
| `--faults`: gps stop, watchdog hang, hud stop, imu stop | pass | pass |
| `--faults`: odom stop and the ten after it, before 16 Nov's fix | **fail** | **fail, the same way** |
| `--faults` since the fix (14 cases) | 14, then 13 | 8 (a busy computer), 12, 10 |

Before the fix, when `/vehicle/odom` stopped, the `speed` input kept the wheels' last
speed (0.5 m/s), so with ego asking 0 it latched OVERSPEED, and every fault after it found
the lock already on. Fixed in both, the same way (TODO 207): a stale wheel speed, or a
command after a gap, is no check; `gen_vectors.py` now plays a 3 s odom silence with ego
asking 0. Since then OVERSPEED never latched there. The cases lost since are "Nav2 would
not drive before the fault": the global EKF jumps 9 m when the wheels come back and the
path follower loses its path (a navigation flake; the watchdog says OK by then).

## Side Effects

- Publishes the lock, the state and `/diagnostics`; serves the clear service.
- Stops cleanly on SIGINT, SIGTERM or SIGHUP (`ctrlc`).

## Verification

```sh
cd ros/safety/aav_watchdog_rs && cargo test --locked -p watchdog-core      # no ROS; tools/test.sh
pixi run python ros/safety/aav_watchdog_rs/gen_vectors.py                 # after changing the Python
pixi run build && colcon test --packages-select aav_watchdog_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `watchdog-core` unit tests | `test_watchdog.py`'s and `test_wheel_check.py`'s cases, ported |
| `tests/vectors.rs` | the Python's answers, tick by tick (above) |
| `test/test_watchdog_rs_node.py` | the process: a GPS input steady (OK), silent (STALE, locked), back (OK), a 111 m jump (JUMP, locked, still locked when steady), the clear service (OK), a clean Ctrl-C |

In the simulator: `pixi run sim watchdog_impl:=rust`. Not yet run on the car.
