# aav_behaviour_rs

## Overview

The behaviour planner in Rust (rclrs): the **default**, in place of `aav_behaviour`'s
Python node. The same node name, topics, QoS, parameters and answers. Ben, 14 Nov:
"convert anything that uses a lot of resources to rust" (TODO 185). The simulator's
default since 16 Nov (below), and the car's too (Ben, 16 Nov).

**The Python node was removed on 20 Nov** (Ben, 16 Nov: "lets remove python were possible"; tag `save/python-behaviour`). `vectors.json` is now a **golden fixture**: the Python wrote it, and `gen_vectors.py` went with it, so never regenerate it; change it only with a reason.

```
locks, odometry, /plan, the scan or the things, a stop sign, the fence
        │
        ▼
   behaviour ──► /drivers/nav/behaviour/state (5 Hz): state, reason, cap, left
             ──► /world/road_ahead (10 Hz): the road ahead, its stops, its end, its
                 limits under the states' cap (TODO 233)
```

Pick it with `behaviour_impl:=rust` (`navigation.launch.py`, `sim.launch.py`), with
`behaviour:=true` (or `stack:=ours`). Not in `stacks.yaml`: the league's levels choose
algorithms; this is the same algorithm in another language, like `ego_impl:=`.

What it does, the states and the cap: below, and `docs/stopping.md`.

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
| `paths` | `active_plan` → `Whose`, `path_left`, `along_until`, `lane_ahead`, `kerb_ahead`, `moving_ahead`, `Seen`, `Crossing`; the hold for anything moving: `MovingFor`, `moving_near`, `Nearby` | `behaviour.py` |
| `road` | `resample`, `build` → `Strip`, `StopOnLine`, `StopKind`, `capped` (the states' cap in the limits) | `road_ahead.py` (the writer) |
| `stop_lines` | `StopLines`: the route's stop lines as stops, served once stood at 2 s (TODO 220) | `behaviour.py` |
| `lanes` | `LaneMap::lane_at`, `lane_offset`, `load`, `from_yaml`, `lane_line`, `with_edges` | `lane_map.py` |
| `fence` | `Polygon::depth` | `aav_safety/geofence.py` |
| `tf` | `Buffer`: each link's last 10 s, looked up at a stamp (interpolated, as tf2) or at the newest shared time | new |
| `py_sum`, `py_max`, `py_min` | Python's compensated `sum()`, and its `max`/`min` (the first wins a tie or a NaN) | new |

## What it does (from the Python package's README, 20 Nov)

| Stop for | `left` (m) | Final stop |
|---|---|---|
| a stop sign | camera range to the sign − the watcher's 4.7 m | the `stop_sign` lock |
| an obstacle in the lane | lidar scan: nearest point ahead − bumper − stop zone | the stop zone, then the `stop_zone` lock |
| a kerb on the road ahead (`kerbs`, TODO 219) | the steps: along the road ahead to the kerb − bumper − stop zone (an `obstacle` stop) | the stop zone's `steps`, then the `stop_zone` lock |
| the fence | along `/plan` to where depth < `near_depth` | the geofence lock |
| the goal | along `/plan` to its end | the navigator's goal check |

The cap is `v_cap(left)` (`aav_description` `stop_law.py`), the lowest that binds;
0.0 (no limit) or between the floor (0.3 m/s) and the top speed, never in between.

```sh
pixi run sim world:=road perception:=true behaviour:=true
ros2 topic echo /drivers/nav/behaviour/state
```

### Interface

**`behaviour`** (`behaviour_node.py`), started by `aav_navigation`'s
`navigation.launch.py behaviour:=true` (`sim.launch.py behaviour:=true` passes it on).
The states' cap goes into the road ahead's limits (no limit above it; TODO 233): ego holds
every follower to them. Until 17 Nov it went to `/drivers/nav/speed_limit`, which only
the follower read.

| In / out | Topic | Type |
|---|---|---|
| in | `/perception/stop_sign/distance` | `geometry_msgs/PointStamped` (camera frame, kept in `odom` by `/tf`) |
| in | `/perception/stop_sign`, `/safety/stop_hold/lock`, `/safety/geofence/lock`, `/safety/sensor_watchdog/lock`, `/safety/estop` | `std_msgs/Bool`: the locks |
| in | `/safety/geofence/state`, `/safety/geofence/fence` | `std_msgs/String` (NEAR), `geometry_msgs/PolygonStamped` |
| in | `/plan`, `/localization/odom/local`, `/localization/odom/global` | `nav_msgs/Path`, `nav_msgs/Odometry` |
| in | `/drivers/nav/planner/route_stops` | `geometry_msgs/PoseArray` (map): the route's stop lines (Lanelet2's); each on the plan is a stop (`stop_line`) in the road ahead: stand there 2 s, then go, as at a seen stop sign (`StopLines`, TODO 220) |
| in | `/drivers/point/plan` | `nav_msgs/Path` (latched): a drive to a point's line; while it runs it is the plan (`active_plan`) |
| in | `/sensors/lidar/scan` | `sensor_msgs/LaserScan` (not with `voxels:=true`) |
| in | `/perception/voxels/steps` | `sensor_msgs/PointCloud2` (base_link), with `kerbs`: the kerbs (TODO 219) |
| in | `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` |
| in | `/drivers/nav/planner/status` | `diagnostic_msgs/DiagnosticStatus`: our planner's `how`; while it says `pass`, with `lane_map` the road ahead follows the pass and its left soft edge opens to the kerb there (Ben, 16 Nov) |
| out | `/drivers/nav/behaviour/state` | `diagnostic_msgs/DiagnosticStatus`, 5 Hz: state, reason, cap, left, warnings |
| in | `/world/things` | `aav_interfaces/Things` (odom), with `things`: what moves ahead |
| out | `/world/road_ahead` | `aav_interfaces/RoadAhead` (map), 10 Hz: `/plan`'s next 40 m from the car, its stops, its end, the speed limit by the stopping law ([the whole car](../../../docs/designs/2026-11-14_the-whole-car.md#the-road-ahead)). With `lane_map`: the line is the car's lane's centre along the route, with its edges |

Parameters (from `vehicle.behaviour_params`: nav2.yaml, vehicle.yaml, ego.yaml):

| Name | Default | Means |
|---|---|---|
| `base_speed` | MPPI's `vx_max` (or `sim_speed`) | the top cap |
| `floor`, `accel`, `fall` | 0.3 m/s, 0.5 m/s², 2.0 m/s² | lowest cap; rise in GO; fastest fall |
| `ask_after` | 30 s | a non-sign lock this long: ASK |
| `release_hold` | 1 s | an obstacle must stay gone this long before APPROACH lets go of it (TODO 189: no flapping at the lane's edge); slowing is never held |
| `max_deceleration`, `max_jerk`, `speed_loop` | ego's; the launch's `speed_loop:=` (`default`) | the stopping law's numbers; the speed loop the car runs (TODO 163) |
| `front`, `half_width`, `stop_gap` | the body and the stop zone | the lane ahead |
| `stop_distance_m`, `near_depth`, `plan_timeout` | 4.7 m, 2.5 m, 25 s | the to-pose tree keeps a path up to 20 s near the goal |
| `road_rate_hz`, `road_length`, `road_spacing` | 10 Hz, 40 m, 0.5 m | the road ahead: how often, how far, how fine |
| `lane_map` | empty | a `lanes.yaml` (`aav_description` `lane_map.py`): the road ahead's line on the right lane's centre, with its hard and soft edges. Empty: the line is `/plan`'s, edges NaN. `sim.launch.py lane_map:=world` gives the world's own |
| `things`, `things_timeout` | false, 1.0 s | true (`navigation.launch.py things:=true`): read `/world/things` (`aav_things_rs`); an obstacle stop only for a thing that moves (0.3 m/s or more), in the lane or walking into it before the car gets there (`Crossing`, TODO 194); a parked thing is the planner's to go round (TODO 162). The stop zone's lock still stops the car for anything. A list older than `things_timeout`: the scan's rule, where the scan runs |
| `kerbs`, `kerbs_timeout` | false, 1.0 s | true (`navigation.launch.py kerbs:=true`, with the lidar): read `/perception/voxels/steps`; a kerb within `half_width` of the road ahead's line (straight ahead without one) is an obstacle stop, measured along the line (`kerb_ahead`, TODO 219), with the things too. Steps older than `kerbs_timeout`: none |
| `moving_hold`, `moving_radius`, `moving_within`, `moving_speed`, `moving_settle` | true, 10 m, 3 s, 0.3 m/s, 1.5 s | with `things`: anything moving (`moving_speed` or more, for `moving_settle` s: `MovingFor`) within `moving_radius` of the car, or on the road ahead within `moving_within` s at its speed, holds the car: WAIT, reason `moving`, and the road ahead ends at the car (a `moving` stop at 0 m). It goes `release_hold` s after the last one has gone or stood still (Ben, 16 Nov; `docs/stopping.md`) |

| Module | No ROS? | Holds |
|---|---|---|
| `behaviour.py` | yes | `Behaviour.step(World) -> Decision`, `Settings`, `Stop`; `active_plan`, `path_left`, `along_until`, `lane_ahead`, `kerb_ahead`, `moving_ahead`, `MovingFor`, `moving_near`, `Nearby`, `diagnostic` |
| `behaviour_node.py` | no | the node; the road ahead's maths is `aav_description`'s `road_ahead.py`, the lane's is `lane_map.py` |

## Invariants

- The same answers as the Python modules: `tests/vectors.rs` reads `vectors.json`, a golden
  fixture since the Python went (20 Nov): never regenerate it.
- The same node name and topics as the Python, so nothing downstream can tell which runs.
  Only one of the two runs at a time (`behaviour_impl:=`).
- The rules are the Python's, unchanged: it only ever slows navigation; a cap is 0.0 or in
  [floor, base]; it never clears a lock; a clean exit sends nothing.
- `behaviour-core` has no ROS. Rust API Guidelines: enums (`State`, `StopReason`, `Lock`,
  `Reason`, `Whose`, `StopKind`), `Result` (`SettingsError`, `RoadError`, `LanesError`,
  `FenceError`, `LookupError`), no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## How it matches the Python

`gen_vectors.py` runs the Python modules (no ROS) on random and hand-picked cases.

| Part | Numbers | Largest difference |
|---|---|---|
| the machine: 8 runs of 600 ticks (4 settings × 2 stopping laws), every state, reason, warning and state-topic text, the hold for something moving (`holding`) | 9,600 + the text | 0, the text the same |
| `nav2_cap` | 400 | 0 |
| `path_left`, `along_until` (the fence's depth), the fence's depth | 750 | 0 |
| `lane_ahead` (float32 scans), `moving_ahead` | 360 | 0 |
| `kerb_ahead` (the kerbs along the road ahead, TODO 219) | 200 | 0 |
| `moving_near` (300 cases), `MovingFor` (40 runs of 60 ticks) | 300 + 5,274 | 0 |
| the road ahead (its stops, the `moving` hold too, the cap), the lane line and its edges, `lane_at` (a map of 5 roads: a junction, two-way, one-way, a bend) | 21,774 | 0 |
| `StopLines` (40 runs of 150 ticks: standing, creeping, passing) | 5,453 | 0 |

The same bit for bit: Python's own `hypot` (ego-core's) and its compensated `sum()`.

Not in the vectors, the node's own part: a sign dropped when TF has nothing at its
picture's stamp (`PROVENANCE.md`, "Where it differs").

## Measured

Our simulator's road world, headless (`pixi run sim-check --with clock_hz:=100`, through
`tools/ros.sh --local`; `stack:=ours`, so the behaviour planner runs), Ben's MacBook (M4
Max), 16 Nov. CPU: the node's CPU seconds a second over the same 28 s of the run, sampled
with `ps` every 2 s.

| `/clock` 100 Hz | Python `behaviour` | Rust `behaviour` |
|---|---|---|
| share of a core | **9.0 %** | **1.3 %** |

| sim-check, `behaviour_impl:=` | Python | Rust |
|---|---|---|
| the road world (start-up, the graph against the contract, sensors, lidar, the scripted drive, the EKF, wheels) | pass | pass |
| scripted drive | 5.83 m (plan 6.5), +0.00 m to the side | the same |
| `--behaviour-only`: an obstacle (the stop zone), then a goal | pass: 0.93 m short, states DRIVE > APPROACH(obstacle) > WAIT(obstacle) > GO(obstacle) > DRIVE > APPROACH(goal), lowest cap 0.3; goal 0.92 m off | pass: the same distance and states; goal 0.90 m off |
| `--behaviour-things-only`: a parked box with the things | fail once (the load was 9.4: the box flickered into APPROACH(obstacle)) | pass: no wait for the parked box, 0.46 m clear |

The Python's one fail was on a busy computer (load 9 on 16 cores); it isn't the port's.

### The simulator's default

Since 16 Nov this is **the simulator's default** (`sim.launch.py`,
`vehicle.SIM_DEFAULT_BEHAVIOUR_IMPL`), and the car's too (`vehicle.DEFAULT_BEHAVIOUR_IMPL`,
`navigation.launch.py`, `car.launch.py`; Ben, 16 Nov). Both gates were no worse with Rust (ego, behaviour and voxels all Rust, against all Python):

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

- Publishes the three topics above. Stops cleanly on SIGINT, SIGTERM or SIGHUP (`ctrlc`).
- colcon builds into its own `target/` folder in the build directory.

## Verification

```sh
cd ros/drivers/aav_behaviour_rs && cargo test --locked -p behaviour-core     # no ROS; tools/test.sh
pixi run build && colcon test --packages-select aav_behaviour_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `behaviour-core` unit tests | `test_behaviour.py`'s cases, ported; each module's own |
| `tests/vectors.rs` | the Python's answers (above); `-- --nocapture` prints each largest difference |
| `test/test_behaviour_rs_node.py` | the process: a goal along a plan (DRIVE, the road ahead on the road world's lane, its kerb), a thing ahead (APPROACH, the cap, the road's end at its stop), the stop zone's lock (WAIT at the floor), a clean Ctrl-C |

In the simulator: `pixi run sim world:=road behaviour:=true behaviour_impl:=rust`. Not yet
run on the car.
