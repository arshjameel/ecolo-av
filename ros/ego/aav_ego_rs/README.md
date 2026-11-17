# aav_ego_rs

## Overview

ego and look_ahead in Rust (rclrs), **one process holding both nodes**: the default, in
place of `aav_ego`'s two Python nodes. The same node names, topics, QoS, parameters and
answers. Ben, 14 Nov: "convert anything that uses a lot of resources to rust" (TODO 185).
The simulator's default since 16 Nov (below), and the car's too (Ben, 16 Nov);
`ego_impl:=python` falls back until a car day proves this one.

```
/control/cmd_vel ──► ego ──► /ego/actuator_cmd (50 Hz)          one process: `ego`
                      ├────► /ego/trace, /ego/stop_point (10 Hz)
                      ▼
                 look_ahead ──► /ego/plan_ahead, /ego/ahead (10 Hz)
```

Pick it with `ego_impl:=rust` (`ego.launch.py`, `sim.launch.py`). Not in `stacks.yaml`:
the league's levels choose algorithms; this is the same algorithm in another language,
like `voxels_impl:=`.

How fast: see "Measured" below.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/ego-core` | the maths: ego's step, the body, the pedal, the lead, the stopping law, the road ahead, the trace, the stop point, the squiggly line, a small TF tree | no: `cargo test` on any laptop |
| `crates/ego-node` | the process, binary `ego`: the nodes `ego` and `look_ahead` | yes (rclrs 0.8, `ros-env` 0.3) |

colcon builds it through `CMakeLists.txt` (`cargo build --release --locked`), installed as
`lib/aav_ego_rs/ego`. It uses the shared crates `car-model` (the twin, the trace, the stop
point) and `spine-core` (the speed loop): reused, not copied.

### Node `ego`

The same as `aav_ego`'s (its README): `/control/cmd_vel` in (TwistStamped, or Twist with
`use_stamped` false); `/vehicle/odom`; `/drivers/<d>/lag_aware` (latched) and
`/drivers/<d>/cmd_vel`; `/safety/estop`; with `road_ahead` true, `/world/road_ahead` and
`/localization/odom/global`. Out: `/ego/actuator_cmd`, `/ego/trace`, `/ego/stop_point`.
Every parameter of `ego.yaml` and `vehicle.py`'s `ego_params`, the same names and
defaults. Live: `max_steering_rate`, `payload`, `lag_lead` (checked when set); the rest
read only.

| Parameter | Rust only |
|---|---|
| `vehicle_yaml` | `''`: `aav_description`'s `vehicle.yaml`, found on `AMENT_PREFIX_PATH` (`ego.launch.py` passes the path) |

### Node `look_ahead`

The same topics as the Python's: `/ego/trace`, `/ego/stop_point`, `/drivers/nav/local_plan`,
`/plan`, `/local_costmap/costmap`, `/world/road_ahead`, `/tf`, `/tf_static` in;
`/ego/plan_ahead` and `/ego/ahead` (the same keys) out. The same parameters, and:

| Parameter | Means |
|---|---|
| `body` | `[front, rear, half width]`, m from `base_link`: `vehicle.yaml`'s outline (the Python reads it itself). No default: `ego.launch.py` passes it |

`--ego-only` (before `--ros-args`) starts ego alone: `ego.launch.py look_ahead:=false`.

### Its parameters

The two nodes share one process, so `ego.launch.py` writes one parameter file, by node
name (`/ego:`, `/look_ahead:`): both have a `rate_hz`, and a plain dict would reach both.

### `ego-core`

| Module | Holds | From |
|---|---|---|
| `ackermann` | `Limits`, `EgoModel::step` → `Output`, `steering_angle`, `slew_speed`, `ease_speed`, `curvature` | `ackermann.py` |
| `dynamics` | `Body` (limits, curve speed, stopping distance), `describe` → `LogLine`s | `dynamics.py` |
| `pedal` | `Pedal::slow_rate` | `pedal.py` |
| `lag_lead` | `LeadMode` (auto, on, off), `LagLead`, `LeadTimes`, `LeadLimits`, `lead_wanted`, `LagAwareWatch`, `CommandKey` | `lag_lead.py` |
| `stop_law` | `StopCurve` (`from_car_model`, `from_numbers`, `cap`, `distance`), `CurveError` | `aav_description/stop_law.py` |
| `road_ahead` | `Strip`, `along_of`, `room_at`, `limit_at`; `allowed`, `cap` | `road_ahead.py` (readers), `road_limit.py` |
| `trace` | `Twin`, `ego_commands`, `trace` → `Pose`s | `trace_ahead.py` |
| `stop_ahead` | `stop_point` → `StopPoint` | `stop_ahead.py` |
| `look_ahead` | `crop_ahead`, `cut`, `gap`, `grade` → `Level`, `to_goal`, `Grid`, `extend`, `first_hit`, `describe`, `road`, `Frame2D` | `look_ahead.py`, `look_ahead_node.py` |
| `frames` | `Tree`, `Transform`: the newest of each TF link, looked up through the common root | new |
| `hypot` | Python's own `math.hypot` (CPython's `vector_norm`) | new |

## Invariants

- The same answers as the Python modules: `tests/vectors.rs` reads `vectors.json`, which
  `gen_vectors.py` writes from them. Change one, change both, and write the vectors again.
- The same node names and topics as the Python, so nothing downstream can tell which
  runs. Only one of the two runs at a time (`ego_impl:=`). The watchdog's `node_ego` and
  `node_look_ahead` find both.
- The safety rules are ego's own, unchanged: a stale command stops at full deceleration;
  braking is never softened; a change of direction passes through a stop; the lead never
  touches a stop.
- `ego-core` has no ROS. Rust API Guidelines: enums (`LeadMode`, `Level`, `LogLine`),
  `Result` (`CurveError`, `ModelError`), no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## How it matches the Python

`gen_vectors.py` runs the Python modules (no ROS) on random and hand-picked cases.

| Part | Numbers | Largest difference |
|---|---|---|
| ego's step: speed, steering, accel (7 runs: plain, body and jerk, loaded, two pedals, the lead, unfiltered) | 6,900 | 0 |
| the pedal's slowing | 200 | 0 |
| the body's limits, and its log lines (as text) | 64 | 0 |
| the stopping law (3 cars × 4 speed loops × 3 limit sets) | 474 | 0 |
| the road ahead: along, room, limits, allowed, the cap, look_ahead's `road` | 940 | 0 |
| the stop point (3 cars, pedal states with dead times) | 245 | 0 |
| ego's commands over a trace | 5,400 | 0 |
| the twin after 60 ticks | 189 | 1.6e-8 (allowed 1e-3) |
| the trace's poses | 3,402 | 3.7e-8 (allowed 1e-3) |
| look_ahead: length, project, crop, cut, gap, grade, to_goal, nearest | 498 | 0 |
| look_ahead: extend, the first obstacle, the grid's cells | 345 | 0 |
| look_ahead's message (as text) | 135 lines | the same |

Why the twin and trace differ: the Rust runs spine-core's own speed loop, in `f32` (the
spine's chip); the Python's `SpeedLoop` is its `f64` port. Everything else is the same
bit for bit, with two details:

| Detail | Why |
|---|---|
| `hypot` is Python's own algorithm | libm's `hypot` may differ in the last bit; a segment exactly 0.1 m long then needs 2 bumper checks instead of 1, and the obstacle distance moved by 5 cm |
| the test reads JSON with `float_roundtrip` | serde_json's default number parser may be 1 bit off |

## Measured

Our simulator's road world, headless (`pixi run sim-check`, through `tools/ros.sh
--local`), Ben's MacBook (M4 Max), 15 Nov. CPU: each process's CPU seconds a second over
the same 35-40 s of the run, sampled with `ps`.

| `/clock` | Python `ego` | Python `look_ahead` | Python, both | Rust, both (one process) |
|---|---|---|---|---|
| 100 Hz (`--with clock_hz:=100`) | 10.5 % | 6.5 % | **17 %** | **2.4 %** |
| 1000 Hz (the simulator's default) | 34 % | 24 % | **58 %** | **4.0 %** |

| The drive (sim-check's road world) | Python | Rust |
|---|---|---|
| every check (start-up, the graph against the contract, sensors, the scripted drive, the EKF) | pass | pass |
| scripted drive | 5.83 m (plan 6.5), +0.00 m to the side | 5.83 m, +0.00 m |
| `--behaviour-only` (`road_ahead` on: an obstacle, then a Nav2 goal) | pass: stopped 0.93 m short, goal 1.40 m off | pass, twice: 0.97 and 0.90 m short, goal 0.14 and 0.27 m off |
| ego trims Nav2 to the road ahead; the lead turns off for the lag-aware `nav` | logged | logged, the same lines |

Nav2's runs differ a little from run to run, whichever ego drives.

Most of the Python's cost at 1000 Hz is `/clock` itself: rclpy wakes for each message.

### The simulator's default

Since 16 Nov this is **the simulator's default** (`sim.launch.py`,
`vehicle.SIM_DEFAULT_EGO_IMPL`), and the car's too (`vehicle.DEFAULT_EGO_IMPL`,
`ego.launch.py`, `car.launch.py`; Ben, 16 Nov). Both gates were no worse with Rust (ego, behaviour and voxels all Rust, against all Python):

| Gate | Python | Rust |
|---|---|---|
| sim-check, the road world | pass | pass (scripted drive 5.83 m, the same) |
| lot, 8 legs (two runs of 2) | 8/8; sway 0.019, 0.022 | 8/8; sway 0.022, 0.021 |
| lot, left-right changes per 100 m | 4.9, 6.3 | 6.8, 4.5 |
| lot, off the path worst (m) | 0.48, 0.64 | 0.48, 0.50 |
| university_box, 2 legs | 2/2; sway 0.069, 0.070 | 2/2; sway 0.057, 0.066 |
| university_box, off the path worst (m) | 0.47, 0.38 | 0.35, 0.57 |

`pixi run measure controllers --stacks own_calm_chunks --tours lot university_box --runs 2
--reversing --launch behaviour:=true lane_route:=true` and the three `_impl:=`s. The
differences are within one run to the next.

## Side Effects

- Publishes the five topics above. Stops cleanly on SIGINT, SIGTERM or SIGHUP (`ctrlc`).
- `ego.launch.py ego_impl:=rust` writes its parameter file in the system's temp folder.
- colcon builds into its own `target/` folder in the build directory.

## Verification

```sh
cd ros/ego/aav_ego_rs && cargo test --locked -p ego-core            # no ROS; tools/test.sh
python3 ros/ego/aav_ego_rs/gen_vectors.py                           # after changing the Python
pixi run build && colcon test --packages-select aav_ego_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `ego-core` unit tests | each module's own cases |
| `tests/vectors.rs` | the Python's answers (above); `-- --nocapture` prints each largest difference |
| `test/test_ego_rs_node.py` | the process: a command drives, the bicycle's steering, the trace and stop point, look_ahead's keys, a stop when the command goes silent, a clean Ctrl-C; no `body` stops it; `--ego-only` |
| `aav_ego`'s `test_ego_launch.py` | `ego_impl:=rust`: one process, both nodes' parameter files |

In the simulator: `pixi run sim world:=road ego_impl:=rust`. Not yet run on the car.
