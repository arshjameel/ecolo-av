# aav_description

## Overview

The car's body: its measurements, its URDF and its frames. `config/vehicle.yaml` is
**the only place car measurements live**. Every other package reads them from here,
through the Python helper `aav_description.vehicle`.

## Interface

### Launch files

| File | Starts |
|---|---|
| `description.launch.py [use_sim_time:=] [joint_state_publisher:=true]` | `robot_state_publisher` from the URDF; `joint_state_publisher`, which merges `/vehicle/joint_states` |
| `view.launch.py` (`pixi run view`) | the model in RViz, with joint sliders |

### Topics

| Topic | Type | Who |
|---|---|---|
| `/robot_description` | std_msgs/String | `robot_state_publisher` publishes |
| `/tf_static`, `/tf` | tf2_msgs/TFMessage | `robot_state_publisher` publishes |
| `/vehicle/joint_states` | sensor_msgs/JointState | `joint_state_publisher` subscribes (from `spine_bridge`) |
| `/joint_states` | sensor_msgs/JointState | `joint_state_publisher` publishes (off in Ravensim, which publishes its own) |

### `config/vehicle.yaml`

| Section | Holds |
|---|---|
| `vehicle` | wheelbase, track, wheel size (m), `max_steering_angle` (rad), `max_reverse_speed` (m/s), body outline, mass (kg), centre of mass, tyre grip, encoder counts, `speed_wheels` (which wheels `/vehicle/odom` measures: front today, TODO 49b) |
| `actuators` | the actuator model for Ravensim's `sim_adapter`: accel, brake, lags, steering rate. Also read by ego (`max_accel`, `max_brake`, via `ego_params`), `aav_drive_to_point`'s launch (`max_brake`) and `carla_adapter` (`max_accel`). **Every value is a guess** |
| `brake` | the brake actuator's travel and limits, like the steering's ends: `released`, `touch`, `full`, `safe_max` (targets) and `max_hold_time` (s). **Placeholders** until car day A7 rows 6-8; `released` and `full` match the firmware's `calibration.rs` (`test_vehicle.py` checks). Read by the adaptive brake |
| `sensors` | each sensor's mount (`xyz` m, `rpy` rad) from `base_link`: IMU, GPS, radar, three cameras, lidar |

Many values are still `TODO measure` (docs/car_measurements.md).

### `aav_description.vehicle` (no ROS)

| Function | Returns |
|---|---|
| `config_path()` | where the installed `vehicle.yaml` is (ament_index) |
| `load(path=None)` | the YAML file as a dict |
| `min_turning_radius(v)` | `base_link`'s radius at full lock (m) |
| `wheel_angles(v, centre)`, `centre_angle(v, inner, outer)` | Ackermann: each front wheel from the bicycle angle, and back |
| `footprint(v)`, `stop_zone_points(v)`, `rear_stop_zone_points(v)` | the costmaps' footprint and the stop zone's boxes |
| `self_box(cfg, margin=SELF_MARGIN)` | the car and every sensor on it from above, 15 cm wider: lidar points inside are the car itself (NDT's map and scan, the debug HUD's cubes) |
| `footprint_points(v)`, `polygon(points)` | the body outline as corners; corners as Nav2's polygon string |
| `blind_behind(reversing, **sensors)`, `reverse_limit(v, reversing)` | may navigation back up, and how fast |
| `nav2_rewrites(v, reversing)`, `nav2_tuned(...)` | navigation's settings (`nav2.yaml`, in Nav2's layout) from the car |
| `plan_turning_radius(v)`, `s_bend_length(radius, offset)`, `final_approach(v)` | the planner's radius; the room ahead an S-bend needs; how near the goal the navigator keeps its path (TODO 17) |
| `ego_params(cfg)`, `spine_params(v)` | ego's and `spine_bridge`'s parameters |
| `check_planner(name, real, trial)`, `lattice_problems(meta, v, resolution)` | a planner's name, checked (only `CAR_PLANNERS` with `real`; with `trial`, car.launch.py's `diffusion_trial:=true`, `TRIAL_CAR_PLANNERS` only); what's wrong with a lattice primitives file for this car and map |
| `diffusion_trial_speed(max_speed)` | the diffusion car trial's top speed: empty gives `DIFFUSION_TRIAL_SPEED` (0.5 m/s); above it is refused |
| `lattice_file_name(headings, moves)`, `check_lattice_moves(moves)`, `smooth_moves(moves)` | the primitives file for 16 headings (the default) or 32 (`LATTICE_HEADINGS`, `lattice_headings:=`), and for Nav2's moves or the smooth ones (`LATTICE_MOVES`, `lattice_moves:=`; 16 headings only) |
| `costmap_chunks_block(folder, ahead, margin)` | our global costmap's `chunks` block (`costmap_chunks:=`, TODO 175): the folder (checked for `chunks.yaml`), `CHUNKS_AHEAD` (100 m of route) and `CHUNKS_MARGIN` (1 chunk more round them) |
| `check_plan_smoothing(s)`, `with_plan_smoothing(cfg, s)`, `check_steer_gamma(g)` | `plan_smoothing:=` (`PLAN_SMOOTHINGS`: `nav2`, or `strong`'s smoother weights into GridBased's `smoother`); `mppi_lag_steer_gamma:=` as a number (TODO 168); `check_mppi_batch(b)`: `mppi_lag_batch:=` as a whole number (TODO 152) |
| `check_planner_search(search, planner)`, `planner_params(tuned, v, costmap, search, nav2)` | our planner's search (`PLANNER_SEARCHES`, `planner_search:=`: `lattice` or `hybrid`, with `planner:=own` only); its settings: the tuned Lattice block, or with `hybrid` nav2.yaml's own Hybrid-A* block, the planning radius, the motion model and the global costmap's cells |
| `check_tracker(name, reversing)`, `track_params(nav2, cfg, ego, max_speed, reversing=)` | a trial tracker's name, checked (`FORWARD_ONLY_TRACKERS`, refused with `reversing`: empty since 20 Nov, when `mpc_lag_rs` learnt to reverse, TODO 213); `max_reverse_speed` with `reversing` (TODO 174); `aav_track`'s parameters from nav2.yaml, vehicle.yaml and ego.yaml |
| `mpc_lag_rs_params(nav2, cfg, ego, max_speed, reversing=)` | `aav_mpc_lag`'s (`tracker:=mpc_lag_rs`): `track_params` without the lookahead (`max_reverse_speed` with `reversing`, TODO 213), and vehicle.yaml's actuators as `actuators.<key>` (`MPPI_LAG_ACTUATORS`) |
| `mppi_lag_params(nav2, cfg, ego, max_speed, reversing, lane)` | `aav_mppi_lag`'s: nav2.yaml's MPPI block and checks, the costmap's outline, vehicle.yaml's actuators (`MPPI_LAG_ACTUATORS`), ego's ramps; with `reversing`, set up to reverse as `nav2_tuned` sets Nav2's MPPI; with `lane` (a lane map), `/world/road_ahead` (`ROAD_AHEAD_TOPIC`) and our `LaneCritic` (`LANE_CRITIC`) |
| `check_speed_loop(name, tracker)`, `DEFAULT_SPEED_LOOP` | `speed_loop:=`, checked against `car_model.SPEED_LOOP_PROFILES` (spine-core's names); `PI_ONLY_TRACKERS` (`mpc_lag`, `mpc_lag_rs`) refuse `lqi`. `track_params`, `mpc_lag_rs_params`, `mppi_lag_params` and `behaviour_params` take `speed_loop=` (default `default`), so every model of the car assumes the loop the sim runs (TODO 163) |
| `check_speed_loop_model(name, car_loop, tracker)` | `speed_loop_model:=` (sim, bench): the loop the models assume when it is not the car's; empty gives `car_loop` (TODO 163) |
| `obstacle_heights(v)`, `centre_of_mass(...)` | the lidar's height band; centre of mass from the scales |

Constants: `SELF_MARGIN`, `SENSOR_HALF_SIZE` (m), `STOP_ZONE_AHEAD`, `STOP_ZONE_BEHIND`, `STOP_ZONE_MARGIN` (m),
`SEES_BEHIND`, `PLAN_TURN_MARGIN`, `GROUND_CLEARANCE`, `OVERHEAD_MARGIN` (m), and
`VOXELS_IMPLS` (`python`: `aav_voxels`, `rust`: `aav_voxels_rs`) with `DEFAULT_VOXELS_IMPL = 'python'` and `check_voxels_impl(name)` (`voxels_impl:=`); the same for `EGO_IMPLS`, `BEHAVIOUR_IMPLS`, `WATCHDOG_IMPLS` and `LOCALIZATION_HEALTH_IMPLS` (`ego_impl:=`, `behaviour_impl:=`, `watchdog_impl:=`, `localization_health_impl:=`; their `DEFAULT_*_IMPL` is `rust`, on the car and in the simulator, since Ben, 16 Nov; `SIM_DEFAULT_EGO_IMPL`, `SIM_DEFAULT_BEHAVIOUR_IMPL`, `SIM_DEFAULT_LOCALIZATION_HEALTH_IMPL` equal them); `LIDAR_IMPLS` and `THINGS_IMPLS` (`lidar_impl:=`, `things_impl:=`; `rust`: `aav_lidar_rs`, `aav_things_rs`, the default on the car and in the simulator since 17 Nov); `IO_MUX_IMPLS` (`io_mux_impl:=`, both muxes together; `rust`: `aav_io_mux_rs`, the default on the car and in the simulator since Ben, 17 Nov) with `check_io_mux_impl(name)`; `CAMERA_STREAM_IMPLS` (`camera_stream_impl:=`; `rust`: `aav_camera_stream_rs`, the default since 18 Nov) with `check_camera_stream_impl(name)` and `camera_stream_runs(impl, detector, decoder)` (the Python one with a detector or `decoder:=opencv`);
`CONTROLLERS = ('rpp', 'mppi')` with `DEFAULT_CONTROLLER = 'mppi'` (the car's, and the fallback of
`sim.launch.py`'s `controller`); `TRACKERS = ('none', 'pure_pursuit', 'stanley', 'lqr',
'mpc', 'mpc_full', 'mpc_lag', 'mppi_lag', 'mpc_lag_rs')` with `DEFAULT_TRACKER = 'none'` (`tracker:=`, `docs/trials.md`), kept apart from
`CONTROLLERS`; `PLANNERS = ('hybrid', 'lattice', 'smac2d', 'navfn', 'theta_star', 'diffusion', 'own')` with
`DEFAULT_PLANNER = 'lattice'` and `FORWARD_ONLY_PLANNER = 'hybrid'` (`default_planner(reversing)` picks), `CAR_PLANNERS = ('hybrid', 'lattice')` (the ones that keep to
the turning radius), `TRIAL_CAR_PLANNERS = ('diffusion',)` with `DIFFUSION_TRIAL_SPEED = 0.5`
(m/s: the car trial's cap, behind `diffusion_trial:=true`) and `PLANNER_BLOCKS` (each
one's block in `nav2.yaml`).

### `aav_description.car_model` (no ROS): one car model

How the car answers a command, for ego (its lead and its trace), Ravensim's `sim_adapter` and the
drivers' tests (moved here from `aav_sim_models`'s `actuator_model.py`, 7 Nov; that module
still re-exports every name). Its Rust twin is [`car-model/`](./car-model/README.md);
`car-model/vectors.json` checks both. The page: `docs/vehicle-model.md`, "One car model".

| Item | What |
|---|---|
| `ActuatorConfig` (`from_vehicle(cfg)`, `from_vehicle_yaml(path)`) | the `actuators:` numbers, the dead times (0 until measured), `max_steering`, `wheelbase`, `mass`, `payload` (kg aboard, 0; `share` = mass / (mass + payload) scales the throttle's push and the brake) |
| `Command(steer, throttle, brake, reverse, speed=None)` | one step's command: pedals, or a set speed for a speed loop |
| `CarState` | `x, y, yaw, v, steer, throttle_level, brake_level`, the steering motor's insides, the delay lines |
| `step(cfg, state, command, dt, loop=None)` | the next `CarState`: dead times, lags, speed, steering, the bicycle |
| `predict(cfg, state, commands, dt, horizon, loop=None)` | the trace: a state every `dt` for `horizon` s (the loop copied) |
| `Actuators`, `SpeedLoop`, `speed_loop_profile`, `Ideal`, `LQI_MODEL` | the simulated stateful car, the spine's speed loop (ported), the old instant car |

### `aav_description.stop_law` (no ROS): how the car stops

One stopping law for every planned stop (`docs/stopping.md`), from this car model's numbers.

| Item | What |
|---|---|
| `StopCurve(a, j, rho)` | `distance(v)` = S(v), `cap(d)` = the speed that stops in d m, `demand(v)`, `h(room, v, margin)`, `tau` |
| `from_car_model(cfg, ego_max_deceleration, max_jerk, input_delay, loop)`, `from_numbers(max_brake, ...)` | the curve for this car: a from the brake and ego, ρ from the brake's lag and dead time and the speed loop's lag (`loop_lag`) |
| `nav2_cap(left, curve, base, floor)` | a speed limit for navigation: 0.0 (none) or between the floor (0.3 m/s) and `base` |
| `stop_point(cfg, speed, ramp, loop)` | where the car model stops when its set speed falls at `ramp`: (metres, seconds) |
| `stop_following(cfg, speed, commands, loop)` | the same for recorded set speeds `(t, v)` (`sim-check --loaded-stop`) |

### `aav_description.road_ahead` (no ROS): the road ahead

The strip every level shares (`/world/road_ahead`; [the whole car](../../../docs/designs/2026-11-14_the-whole-car.md#the-road-ahead)): here beside the stopping law, so its writer and every reader use one copy.

| Item | What |
|---|---|
| `build(plan, car, stops, curve, base)` | the writer's (`aav_behaviour_rs`): the plan's next 40 m from the car, a point every 0.5 m; its stops; its end (the nearest stop); the speed limit by the stopping law, 0 at the end |
| `limit_at(strip, x, y, curve, base)`, `room_at`, `along_of` | the readers' (ego): the speed allowed here, the metres left to the end; no strip allows 0 |
| `capped(strip, cap)` | the writer's: no limit above the behaviour planner's cap (TODO 233); 0.0 is no cap |
| `Strip`, `StopOnLine`, `resample`; `WAIT`, `FINAL` | the strip; a stop on it; the plan cut into points |
| `road_ahead_ros.py`: `to_msg`, `from_msg` | `Strip` and `aav_interfaces/RoadAhead` (this one imports ROS messages) |

### `aav_description.lane_map` (no ROS): the lane map

Which lane the car drives in, so the road ahead's line is the lane's centre (W3, W5).

```text
   ══════════════ left kerb ══════════════   ┐ left_hard
   ── ── ── ──    centre line   ── ── ──     ┤ left_soft
   ● ─────── the car's lane centre ──────►
   ────────────── edge line ──────────────   ┤ right_soft
   ══════════════ right kerb ═════════════   ┘ right_hard
```

| Item | What |
|---|---|
| `load(path)`, `from_dict(data)` | a `lanes.yaml`: each road's centre line, `width` (kerb to kerb), `lanes` (all, both ways; default width / 3.5), `oneway`, `edge_inset` (0.5 m) |
| `LaneMap.lane_at(x, y, heading)` | the car's lane there: the rightmost for the way it drives, its centre and edges; None off every road |
| `LaneMap.roads_at(x, y)` | every road whose tarmac holds the point, either way: two or more is a junction (the lane pose trial's gate) |
| `lane_line(lanes, route, car)` | the route ahead moved onto its lane's centre; off every road it stays; eased over 3 m where roads change |
| `LANE_GRAPHS`, `DEFAULT_LANE_GRAPH`, `check_lane_graph(name)` | the route level's map for `lane_graph:=`: `lanes_yaml` (the default) or `lanelet2` (TODO 175) |
| `lanelet2_beside(lanes_yaml)`, `planner_block(lanes_yaml, graph)` | the Lanelet2 map in the same folder (`<folder>.osm`, else its only `.osm`); our planner's `lanes` block: the roads, or `{"lanelet2": path}` (the roads, with a warning, if there is no map) |
| `with_edges(strip, lanes)` | a `Strip` with its four edges from the map (NaN off every road) |

| Lane map | Where | Covers |
|---|---|---|
| the road world | `aav_sim_models` `worlds/road.lanes.yaml` (in git) | one lane, 3.5 m between the lines, either way |
| the campus | `maps/campus/lanes.yaml` (made by `pixi run campus`, not in git) | 129 painted roads; a right lane each way on two-way roads |

Lanelet2 was tried first (14 Nov): its Python modules don't import on a Mac, so this format
for now (`docs/decisions.md`).

### `aav_description.stacks` (no ROS): the league

`config/stacks.yaml`: each level's champion and challengers, and the stacks `best`, `car`,
`stock`, `ours` (`docs/league.md`). `sim.launch.py stack:=` reads it; `DEFAULT = 'ours'` is
the simulator's stack since 14 Nov.

| Item | What |
|---|---|
| `load(path=None)`, `check(league)` | the file, checked: every champion and every stack's pick is a level's option |
| `choices(league, stack)`, `switches(league, stack)` | each level's option; the launch switches they set |
| `resolve(league, stack, given, fallback)` | the switches to use: each one given by hand, else the stack's, else its fallback. A switch not given is `CHOSEN` (empty): `sim.launch.py` declares them so |
| `describe(league, stack, levels)` | one line for the log (and the levels picked by hand) |
| `LEVEL_SWITCHES`, `picked(league, stack, levels)` | `pose`, `path`, `steer`, `lanes`: a level's own switch (`path:=hybrid_rs`), over the stack's pick; `resolve` and `switches` take the same `levels` |

### `aav_description.adaptive_brake` (no ROS): a heavier car

After the speed loop, before the brake: more brake when the car slows less than the empty
car model says (`docs/stopping.md`, "A heavier car").

| Item | What |
|---|---|
| `BrakeLimits` (`from_vehicle(cfg)`) | `vehicle.yaml`'s `brake:`; `fraction(target)`, `touch_fraction`, `safe_max_fraction` |
| `AdaptiveBrakeConfig` | how fast it learns, the most it multiplies (3), the rise rate, what counts as high force |
| `AdaptiveBrake(car, limits, tuning)` | `update(throttle, brake, speed, dt)` → `BrakeOut` (brake, gain, effectiveness, capped, spent); `reset()` |

### URDF (`urdf/`)

`aav.urdf.xacro` builds `base_link`, `chassis`, four wheels with two steering joints,
and one link per sensor in `vehicle.yaml`. Gazebo's `aav.gazebo.xacro` and
`aav.sensors.gazebo.xacro` were removed with Gazebo (16 Nov); its `sim`, `lidar` and
`chase_camera` arguments on 20 Nov (TODO 241).

### Rust crates beside the package (no ROS, their own workspaces)

| Crate | What |
|---|---|
| [`car-model/`](./car-model/README.md) | the car model's Rust twin |
| [`map-chunks/`](./map-chunks/README.md) | the map in fixed 64 m chunks |
| [`lanelet-map/`](./lanelet-map/README.md) | Lanelet2 maps in Rust |
| [`map-edit/`](./map-edit/README.md) | the map editor's edits, saves and layers (TODO 251) |
| [`cloud-view/`](./cloud-view/README.md) | a `PointCloud2`'s bytes read by field name: one reader for every Rust crate (TODO 176) |

## Invariants

- One file holds every measurement. Never copy a number into another package.
- REP-103 and REP-105: metres, radians; x forward, y left, z up; positive steering = left.
- `base_link` is on the ground under the centre of the rear axle. Measure from there.
- `max_steering_angle` is the virtual centre wheel (bicycle model), as ego and the planner use.
- `firmware/spine`'s `spine-core` keeps copies of the steering limit and reverse speed.
  `test_vehicle.py` checks they match: change both together.
- `vehicle.py` has no ROS imports, except `config_path()` (ament_index); `car_model.py` has none.
- `car_model.py` and `car-model/` (Rust) give the same states: after changing either, run
  `python3 car-model/gen_vectors.py`, then both test suites.

## Side Effects

- `vehicle.py` reads `config/vehicle.yaml` from disk.
- The launch files start `robot_state_publisher`, `joint_state_publisher` and RViz.

## Verification

```sh
pixi run build && colcon test --packages-select aav_description && colcon test-result --verbose
```

`pixi run test` runs every package's tests. To see a change: `pixi run view`.
