# aav_behaviour_rs

## Overview

The behaviour planner in Rust (rclrs): a **challenger** beside `aav_behaviour`'s Python
node. The same node name, topics, QoS, parameters and answers. Ben, 14 Nov: "convert
anything that uses a lot of resources to rust" (TODO 185). The Python stays the default
until this one has won its runs.

```
locks, odometry, /plan, the scan or the things, a stop sign, the fence
        │
        ▼
   behaviour ──► /drivers/nav/speed_limit (20 Hz): Nav2's cap, 0.0 = no limit
             ──► /drivers/nav/behaviour/state (5 Hz): state, reason, cap, left
             ──► /world/road_ahead (10 Hz): the road ahead, its stops, its end
```

Pick it with `behaviour_impl:=rust` (`navigation.launch.py`, `sim.launch.py`), with
`behaviour:=true` (or `stack:=ours`). Not in `stacks.yaml`: the league's levels choose
algorithms; this is the same algorithm in another language, like `ego_impl:=`.

What it does, the states and the cap: `aav_behaviour`'s README and `docs/stopping.md`.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/behaviour-core` | the maths: the states, the stops along the plan and in the lane, the road ahead's writer, the lane map, the fence's depth, a TF buffer with history | no: `cargo test` on any laptop |
| `crates/behaviour-node` | the node, binary `behaviour` | yes (rclrs 0.8, `ros-env` 0.3) |

colcon builds it through `CMakeLists.txt` (`cargo build --release --locked`), installed as
`lib/aav_behaviour_rs/behaviour`. It uses `aav_ego_rs`'s `ego-core` (the stopping law,
Python's `hypot`, the TF transform) and through it `car-model`: reused, not copied.

### Node `behaviour`

The same as `aav_behaviour`'s (its README): every topic, QoS and parameter, with the same
names and defaults. One addition:

| Parameter | Rust only |
|---|---|
| `vehicle_yaml` | `''`: `aav_description`'s `vehicle.yaml`, found on `AMENT_PREFIX_PATH` (the Python finds it through `ament_index`) |

### `behaviour-core`

| Module | Holds | From |
|---|---|---|
| `machine` | `Behaviour::step(&World) -> Decision`; `State`, `StopReason`, `Lock`, `Reason` (enums), `Settings`, `Stop`, `nav2_cap`, `diagnostic` | `behaviour.py`, `stop_law.nav2_cap` |
| `paths` | `active_plan` → `Whose`, `path_left`, `along_until`, `lane_ahead`, `moving_ahead`, `Seen`, `Crossing` | `behaviour.py` |
| `road` | `resample`, `build` → `Strip`, `StopOnLine`, `StopKind` | `road_ahead.py` (the writer) |
| `lanes` | `LaneMap::lane_at`, `lane_offset`, `load`, `from_yaml`, `lane_line`, `with_edges` | `lane_map.py` |
| `fence` | `Polygon::depth` | `aav_safety/geofence.py` |
| `tf` | `Buffer`: each link's last 10 s, looked up at a stamp (interpolated, as tf2) or at the newest shared time | new |
| `py_sum`, `py_max`, `py_min` | Python's compensated `sum()`, and its `max`/`min` (the first wins a tie or a NaN) | new |

## Invariants

- The same answers as the Python modules: `tests/vectors.rs` reads `vectors.json`, which
  `gen_vectors.py` writes from them. Change one, change both, and write the vectors again.
- The same node name and topics as the Python, so nothing downstream can tell which runs.
  Only one of the two runs at a time (`behaviour_impl:=`).
- The rules are the Python's, unchanged: it only ever slows Nav2; a cap is 0.0 or in
  [floor, base]; it never clears a lock; a clean exit sends nothing.
- `behaviour-core` has no ROS. Rust API Guidelines: enums (`State`, `StopReason`, `Lock`,
  `Reason`, `Whose`, `StopKind`), `Result` (`SettingsError`, `RoadError`, `LanesError`,
  `FenceError`, `LookupError`), no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## How it matches the Python

`gen_vectors.py` runs the Python modules (no ROS) on random and hand-picked cases.

| Part | Numbers | Largest difference |
|---|---|---|
| the machine: 8 runs of 600 ticks (4 settings × 2 stopping laws), every state, reason, warning and state-topic text | 9,600 + the text | 0, the text the same |
| `nav2_cap` | 400 | 0 |
| `path_left`, `along_until` (the fence's depth), the fence's depth | 750 | 0 |
| `lane_ahead` (float32 scans), `moving_ahead` | 360 | 0 |
| the road ahead, the lane line and its edges, `lane_at` (a map of 5 roads: a junction, two-way, one-way, a bend) | 21,761 | 0 |

The same bit for bit: Python's own `hypot` (ego-core's) and its compensated `sum()`.

Not in the vectors, the node's own part: a sign dropped when TF has nothing at its
picture's stamp (`PROVENANCE.md`, "Where it differs").

## Measured

See TODO 185 and `docs/features.md`'s row: CPU in our simulator's road world, Python
against Rust, and the sim checks run with it.

## Side Effects

- Publishes the three topics above. Stops cleanly on SIGINT, SIGTERM or SIGHUP (`ctrlc`).
- colcon builds into its own `target/` folder in the build directory.

## Verification

```sh
cd ros/drivers/aav_behaviour_rs && cargo test --locked -p behaviour-core     # no ROS; tools/test.sh
pixi run python ros/drivers/aav_behaviour_rs/gen_vectors.py                  # after changing the Python
pixi run build && colcon test --packages-select aav_behaviour_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `behaviour-core` unit tests | `test_behaviour.py`'s cases, ported; each module's own |
| `tests/vectors.rs` | the Python's answers (above); `-- --nocapture` prints each largest difference |
| `test/test_behaviour_rs_node.py` | the process: a goal along a plan (DRIVE, the road ahead on the road world's lane, its kerb), a thing ahead (APPROACH, the cap, the road's end at its stop), the stop zone's lock (WAIT at the floor), a clean Ctrl-C |

In the simulator: `pixi run sim world:=road behaviour:=true behaviour_impl:=rust`. Not yet
run on the car.
