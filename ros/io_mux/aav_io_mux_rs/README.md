# aav_io_mux_rs

## Overview

The sensor mux and the output mux in Rust (rclrs): the **default**, in place of
`aav_io_mux`'s Python nodes. The same node names, topics, QoS, parameters, service and
answers.

**A safety lock.** The output mux makes the heartbeat the ESP32 brakes without. Ben
decided (17 Nov: "Rust tomorrow ... I don't think we want to do this again later"): Rust
on the car (the Motor Pi alone too) and in the simulator. The only muxes since 22 Nov:
the Python nodes were removed (tag `save/python-fallbacks-1`); `aav_io_mux` keeps the
launch file and the rules in Python.

```
/system/source + the ROS graph ──► sensor_mux ──► /system/sensor_mux/state ──┐
                                                                             ▼
/ego/actuator_cmd ─────────────────────────────────────────────────► output_mux
         ──► /control/<target>/actuator_cmd + /control/<target>/heartbeat (only while OPEN)
         ──► /safety/output_mux/state (OPEN, WAIT or STOP)
         ◄── /safety/output_mux/clear (Trigger): a person reopens a latched STOP
```

What the muxes check, and why: `aav_io_mux`'s README and `docs/sources-and-targets.md`.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/io-mux-core` | the logic: `provenance` (the rules), `muxes` (each node's work), `Counter` (the heartbeat) | no: `cargo test` on any laptop |
| `crates/io-mux-node` | the two nodes, binaries `sensor_mux` and `output_mux` | yes (rclrs 0.8, `ros-env` 0.3) |

colcon builds it through `CMakeLists.txt` (`cargo build --release --locked`), installed as
`lib/aav_io_mux_rs/sensor_mux` and `lib/aav_io_mux_rs/output_mux`. It reuses `aav_ego_rs`'s
`ego-core` (Python's `%g`).

### Picking it

`io_mux.launch.py io_mux_impl:=rust` (`vehicle.DEFAULT_IO_MUX_IMPL`; the only value, a hook).
`car.launch.py` and `sim.launch.py` declare it and pass it on; the other profiles (bench,
turtle, carla, replay) take it from the command line the same way.

### Nodes

As `aav_io_mux`'s README lists them (the Python's, as they were): every topic, QoS, parameter and
default, and the clear service. Nothing added. The parameters are read only.

### `io-mux-core`

| Module | Holds | From |
|---|---|---|
| `provenance` | `Kind` (REAL, SIM, BENCH, REPLAY, `Unknown(n)`), `Target`, `ChoiceError`, `SENSORS`, `Announcers`, `foreign`, `check_sensors`, `Hold`, `Report`, `Gate` (`clear` → `Result`) | `provenance.py` |
| `muxes` | `SensorMux`, `OutputMux`: each message and tick in, what to publish and log out; `screen_names`; `Level`, `Log`, `Published` | `muxes.py` |
| `lib` | `Counter`: 1, 2 ... 255, 0 | `heartbeat.py` |

## Invariants

- **The same answers as the Python, exactly.** `aav_io_mux`'s `muxes.py` is the nodes'
  work without ROS (the Python nodes called it too, until 22 Nov). `gen_vectors.py` runs it and writes
  `vectors.json`; `tests/vectors.rs` replays every step in Rust, and `aav_io_mux`'s
  `test_vectors.py` checks the Python still writes the same file. No tolerance: text as
  text. Change one, change both.
- Fail-safe as the Python, and where the Python would crash: a ROS graph that can't be
  read skips that judgement (the sensor mux publishes nothing, so the output mux waits);
  a clear is refused.
- Both time on a steady clock, never simulated time.
- `io-mux-core` has no ROS. Rust API Guidelines: enums (`Kind`, `Target`, `Level`),
  `Result` (`ChoiceError`, a clear's answer), no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean (the node crate too). `Cargo.lock` committed.

## How it matches the Python

`gen_vectors.py` plays hand-written stories and 70 random runs through the Python:

| Story | Covers |
|---|---|
| the car on its stock drivers | unannounced publishers are REAL; quiet while discovery settles |
| a simulator on the car | NOT REAL at once; STOP latched; commands and heartbeat held; clear refused while it lives, still latched once it goes, then cleared and settled |
| the simulator's busy start-up | sensors 10 s before their announcer: WAIT through the 30 s grace, never a latch |
| never announced | after the grace and the hold, a problem (NOT SIM) |
| two publishers, an announcer not heard yet, an unknown number | doubts, pending doubts, UNKNOWN(n) never REAL |
| a quiet sensor mux, one checking the wrong source, "says no", unsure | WAIT without latching; the latches with their reasons |
| screens | waiting, beats, a quiet screen; refused for the car |
| every limit exactly | `settle`, `stale_after`, `hold`, `grace`, the announcers' 10 s, the screens' 1 s, at the boundary |
| the heartbeat | +1 per command passed on, wrapping 255 → 0 |
| each target and the refusals | car, sim, turtle, carla, bench; `target:=gazebo`, `source:=gazebo`, empty |

| Compared | Steps | Different |
|---|---|---|
| each SourceCheck (source, ok, problems, doubts, sensors) and log line | 1,847 | 0 |
| each state line and log line, command and heartbeat counter, clear's answer | 4,704 | 0 |

Mutation check (17 Nov): flipping any limit's `<` or `>=`, dropping the sort, the
"says no", or the announcers' forgetting, each fails `tests/vectors.rs`.

## Measured

Our simulator's road world, headless (`pixi run sim-check`, through `tools/ros.sh
--local`), Ben's MacBook, 17 Nov. CPU: each node's CPU seconds a second over about 25 s
of the run, from 15 s after it started, sampled with `ps`.

| | Python | Rust |
|---|---|---|
| `sensor_mux` | 17.2 % of a core | 4.1 % |
| `output_mux` | 21.7 % | 4.7 % |
| sim-check's road world (start-up, contract, sensors, lidar, scripted drive, EKF, wheels) | pass | pass (twice) |

Most of what is left is the ROS graph read each tick (12 topics, twice a second).

## Side Effects

- Publishes the topics above; reads the ROS graph every tick; serves the clear service.
- Stops cleanly on SIGINT, SIGTERM or SIGHUP (`ctrlc`).

## Verification

```sh
cd ros/io_mux/aav_io_mux_rs && cargo test --locked -p io-mux-core     # no ROS; tools/test.sh
pixi run python ros/io_mux/aav_io_mux_rs/gen_vectors.py              # after changing the Python
pixi run build && colcon test --packages-select aav_io_mux_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `tests/vectors.rs` | the Python's answers, step by step (above) |
| `lib.rs`'s unit test | the counter wraps in a byte |
| `test/test_io_mux_rs_nodes.py` | the two processes, target car: OPEN, commands and a +1 heartbeat; a SIM announcement: STOP, nothing goes on, clear refused; it goes: still STOP, cleared, OPEN; a clean Ctrl-C |
| `aav_io_mux`'s `test_real_backend_guard.py`, `aav_legacy`'s `test_heartbeat_chain.py` | these muxes with the real backends: `spine_bridge` and `legacy_uart_bridge`, each in Rust and in Python |

In the simulator: `pixi run sim` (the default). Not yet run on the car.
