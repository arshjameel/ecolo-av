# aav_navigation

Our navigation, set up for the car, as one more driver: goals in, `/drivers/nav/cmd_vel`
out to twist_mux (priority 20). Every node is ours, each a port of a Nav2 server (Nav2
itself went on 17 Nov, TODO 184: [Nav2, as it was](../../../docs/history/nav2-as-it-was.md)).
This package is the launch file and the settings.

```
goal (RViz, HUD, street_router, language_drive) -> navigator -> planner + mppi_lag
  -> velocity_smoother (ours) -> [stop zone] -> /drivers/nav/cmd_vel -> twist_mux -> ego
```

## Overview

It starts our nodes: the map server, the costmaps, the planner, the navigator, a path
follower, the smoother, and the stop zone where a sensor feeds it. AMCL (ours,
`aav_amcl_rs`; stock `nav2_amcl` removed 20 Nov, TODO 232) only as a trial, `amcl:=true`:
robot_localization does localization. The car's footprint, turning radius and reverse
speed come from
`ros/bringup/aav_description/config/vehicle.yaml`; `aav_description`'s
`vehicle.nav2_tuned()` and `vehicle.rewrite()` write them into a copy of `config/nav2.yaml`
(the blocks our ports read, in Nav2's layout).

```sh
ros2 launch aav_navigation navigation.launch.py
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 20.0, y: 5.0}}}}"
```

## Interface

Launch: `ros2 launch aav_navigation navigation.launch.py [args]`

| Argument | Default | Meaning |
|---|---|---|
| `map` | `maps/empty.yaml` | the map YAML (`maps/README.md`) |
| `radar` | false | the local costmap's obstacle layer reads `/sensors/radar/points` |
| `lidar` | false | `lidar_scan` (aav_lidar) flattens `/sensors/lidar/points` to `/sensors/lidar/scan`, for the costmap and the stop zone |
| `voxels` | false | with `lidar`: a trial, `aav_voxels`' `voxels` cuts the cloud into 0.25 m cubes; the costmap's lidar_layer and the stop zone read those instead of the scan, and `lidar_scan` runs only for `amcl` ([design](../../../docs/designs/2026-10-30_voxels.md)) |
| `voxels_impl` | `python` | with `voxels` or `screen_cubes`: `python` (`aav_voxels`) or `rust` (`aav_voxels_rs`: the same node, topics and cubes; the launch file passes it `vehicle.yaml`'s lidar mount and the car's outline) |
| `lidar_impl` | `rust` | with `lidar`: `rust` (`aav_lidar_rs`, the default since 17 Nov: the same scan; the launch file passes it `vehicle.yaml`'s lidar mount and the car's outline) or `python` (`aav_lidar`) |
| `things_impl` | `rust` | with `things`: `rust` (`aav_things_rs`; the Python removed 20 Nov) |
| `kerbs` | false | with `lidar` (ignored without): the kerbs (TODO 119, [kerbs](../../../docs/kerbs.md)). The Rust `voxels` node (`aav_voxels_rs`, whatever `voxels_impl` says) runs with `steps:=true`, and the local costmap's `kerb_layer` marks `/perception/voxels/steps`: marks only, kept until the window rolls past; the stop zone's `steps` source stops for them |
| `stop_zone` | empty | the stop zone (`aav_stop_zone`); empty: on wherever radar, lidar, ghost or camera guard runs |
| `camera_obstacles` | false | the costmap's camera layer reads `/perception/camera_obstacles` (off until Q22) |
| `camera_guard` | false | with `camera_obstacles`: the stop zone stops for them too |
| `ghost` | false | a pretend person (`aav_ghost`, `/safety/ghost/scan`) |
| `reversing` | false | true: Reeds-Shepp paths, reverse to `-max_reverse_speed`, the navigator backs up 1 m |
| `max_speed` | empty (1.0 m/s) | the follower's top speed (`sim.launch.py sim_speed:=`) |
| `controller` | `mppi` | the only one: nav2.yaml's MPPI block, which `mppi_lag` reads |
| `planner` | `own` | `own`: our planner (`aav_planner`: Nav2's State Lattice in Rust, `config/lattice_ackermann.json`; refused if it no longer fits `vehicle.yaml` or the map's cells); `diffusion`: the model draws, the guard checks, ours behind it (a trial; the car only with `diffusion_trial`). `docs/own-planner.md` |
| `lattice_headings` | 16 | 16 headings (`config/lattice_ackermann.json`) or 32, a trial (`config/lattice_ackermann_32.json`) |
| `lattice_moves` | `nav2` | `smooth` (1 m/s) or `smooth_fast` (2.5 m/s), a trial: moves whose curvature changes no faster than the steering turns (`config/lattice_ackermann_smooth*.json`), and the lattice's smoother off |
| `planner_search` | `lattice` | `hybrid`, a challenger: our planner runs Nav2's Hybrid-A* ported instead of the State Lattice, on `nav2.yaml`'s `GridBased` block (Reeds-Shepp with `reversing:=true`, else Dubins; `docs/own-planner.md`) |
| `plan_around` | true | false: the planner sees only the map, so the car stops (stop tests) |
| `tracker` | `mppi_lag` | the path follower: `mppi_lag` (`aav_mppi_lag`: Nav2's MPPI with the lag, the champion); a trial (`aav_track`): `pure_pursuit`, `stanley`, `lqr`, `mpc`, `mpc_full`, `mpc_lag`; or `mpc_lag_rs` (`aav_mpc_lag`). It serves `follow_path`. `aav_track`'s reverse since 19 Nov (TODO 174); `mpc_lag_rs` since 20 Nov (TODO 213; `docs/trials.md`) |
| `navigator`, `costmap`, `smoother` | `own` | ours, the only option each since Nav2 went (`docs/own-navigator.md`, `docs/own-costmap.md`, `docs/own-smoother.md`) |
| `replan_hz` | 1 | how often the navigator replans while it drives; 0: plan once |
| `plan_smoothing` | `nav2` | the planner's smoother: `nav2`, Nav2's weights; `strong` (TODO 168): w_data 0.02 in the GridBased block's `smoother`, so a road at an angle is no staircase. our planner reads it (`vehicle.PLAN_SMOOTHINGS`). The sim's `ours` stack picks `strong` |
| `mppi_lag_steer_gamma` | empty | with `tracker:=mppi_lag`: its steering's control cost weight (`steer_gamma`); empty: nav2.yaml's gamma. 0.05 (TODO 168): less sampling jitter; it eases to gamma below 1 m/s. The sim's `ours` stack picks 0.05 |
| `mppi_lag_batch` | empty | with `tracker:=mppi_lag`: its samples a tick (`batch_size`); empty: nav2.yaml's 2000. 1000 (TODO 152): half the work a tick, a challenger (`steer:=mppi_lag_1k`) |
| `map_correction` | `newest` | with a `tracker`: `slow` (TODO 186; the sim's `ours` stack picks it), the follower's held map -> odom moves toward the newest at most 0.1 m/s and 1°/s at the car, so the GPS's jitter never jumps the path under the car; `newest`: each new path by the newest map -> odom, as until 15 Nov. The follower gets it as its bool parameter `map_correction`, never in its own yaml (the file would win) |
| `speed_loop` | `default` | the spine's speed loop profile the car runs (`default`, `smooth`, `lqi`, `firm`): the follower's model (`mppi_lag`, `mpc_lag`, `mpc_lag_rs`; the two MPCs refuse `lqi`) and the behaviour planner's stopping law assume it. `sim.launch.py` and `bench.launch.py` pass their own (TODO 163); the car's spine runs `default` |
| `lane_costs` | empty | a `lanes.yaml`, drawn into our global costmap as soft costs, lowest at the right lane's centre (which lane by `/plan`), so our planner keeps right and crosses a line only when it pays. A challenger (TODO 148, `docs/own-costmap.md`). `sim.launch.py lane_costs:=true` passes the world's lane map |
| `costmap_chunks` | empty | a folder of the map's 64 m chunks (`chunks.yaml`, `fine/`; `pixi run campus` writes the campus's). Our global costmap then holds only the chunks round the car and along the route ahead (`/drivers/nav/planner/route`), each read and inflated on its own core, not `/map`; our planner plans a goal off it in steps (TODO 175, `docs/own-costmap.md`). `sim.launch.py costmap_chunks:=auto` passes the campus's |
| `lane_route` | empty | a `lanes.yaml`; our planner then plans the right lane's centre along it (routed by its roads), the lattice joining the car and the goal to it, and the lattice alone off the map. A challenger (TODO 148, the league's `path:=lane`; `docs/own-planner.md`). `sim.launch.py lane_route:=true` passes the world's lane map |
| `lane_pass` | true | with `lane_route`: something parked on the lane line is passed by moving only its stretch left, the least that clears it, then back (Ben, 16 Nov; `docs/own-planner.md`); false: the whole path is the lattice's, as before |
| `lane_corners` | arc | with `lane_route`: the lane line's corners as circle arcs; `clothoid`: clothoid, arc, clothoid, the curvature never jumping (TODO 211, a challenger: a tie on University Drive, 17 Nov) |
| `lane_pass_shape` | cosine | with `lane_route`: a parked obstacle passed by a half-cosine shift; `jerk`: a QP along the lane holding the lateral jerk at `max_speed` (TODO 214, a challenger, `path:=lane_jerk`) |
| `lane_graph` | `lanes_yaml` | with `lane_route`: `lanelet2` routes on the Lanelet2 map beside the `lanes.yaml` (`maps/campus/campus.osm`, `lane_map.planner_block`): its lanes, neighbours and stop lines (`/drivers/nav/planner/route_stops`). A challenger (TODO 175, step 6) |
| `local_chunks` | false | with `costmap_chunks`: true, the local costmap's static layer from the 3 x 3 chunks round the car too, so no costmap of ours reads `/map` (TODO 175) |
| `amcl` | false | true (needs `lidar:=true`): AMCL, scored only, on `/localization/amcl/pose` (`config/amcl.yaml`: `tf_broadcast: false`) |
| `amcl_impl` | `rust` | with `amcl`: `rust` (`aav_amcl_rs`, the default since 18 Nov, TODO 231: a plain node); stock `nav2_amcl` removed 20 Nov (TODO 232). The same `config/amcl.yaml`, topics and frames |
| `amcl_x`, `amcl_y`, `amcl_yaw` | 0 | AMCL's start pose, map frame (`sim.launch.py` gives the sim's start) |
| `amcl_map` | empty | empty: AMCL reads `/map`; else its own map YAML, on `/localization/amcl/map` (`sim.launch.py`: `maps/example_lot_amcl.yaml`, [why](./maps/README.md#why-amcl-has-a-map-of-its-own)) |
| `use_sim_time`, `autostart` | false, true | |

Nodes started (all ours):

| Node | Does |
|---|---|
| `map_server` (`aav_costmap`) | publishes `/map`, latched |
| `costmap` (`aav_costmap`) | the local and global costmaps, `/drivers/nav/costmap/*` |
| `planner` (`aav_planner`) | `/drivers/nav/planner/compute_path_to_pose` and `_through_poses`, `/plan`, `/is_path_valid` |
| `navigator` (`aav_navigator`) | `/navigate_to_pose`, `/navigate_through_poses`, `/follow_waypoints`, `/follow_gps_waypoints`: plan, follow, replan, recover |
| `mppi_lag` (`aav_mppi_lag`), or `track` (`aav_track`), or `mpc_lag_rs` (`aav_mpc_lag`) | `follow_path` -> `cmd_vel_nav` |
| `velocity_smoother` (`aav_smoother`'s `car_smoother`) | `cmd_vel_nav` -> `/drivers/nav/cmd_vel`, or -> `/control/cmd_vel_smoothed` |
| `stop_zone` (`aav_stop_zone`) | `/control/cmd_vel_smoothed` -> `/drivers/nav/cmd_vel`; `/safety/stop_zone/state`, `status`, `box` |
| `stop_hold` (`aav_safety_rs`) | with the stop zone: holds its stops 1 s (`/safety/stop_hold/lock`) |
| `behaviour` (`aav_behaviour_rs`) | with `behaviour:=true`: the road ahead, the speed cap |
| `diffusion_planner` | with `planner:=diffusion` |
| `lidar_scan` (`aav_lidar_rs`, or `aav_lidar` with `lidar_impl:=python`), `ghost_obstacle` | with `lidar:=true`, `ghost:=true` |
| `things` (`aav_things_rs`) | with `lidar:=true things:=true` |
| `voxels` (`aav_voxels`, or `aav_voxels_rs` with `voxels_impl:=rust`) | with `voxels:=true` or `screen_cubes:=true`; `aav_voxels_rs` with `kerbs:=true` |
| `amcl` (`aav_amcl_rs`), `amcl_map_server` | with `amcl:=true`: our AMCL; with `amcl_map:=`, our map server for its map |

Inputs: `/localization/odom/local`, `/tf`, `/goal_pose`, and the obstacle topics above.
All commands are `geometry_msgs/TwistStamped`.

Files:

| Folder | Holds |
|---|---|
| `config/nav2.yaml` | the blocks our ports read, in Nav2's layout (the costmaps, MPPI, the lattice, the smoother, the stop zone) |
| `config/lattice_ackermann.json` | the State Lattice planner's moves for our car: made by Nav2's generator, `pixi run python3 tools/lattice_primitives.py` (`tools/lattice_primitives.py`); never edit by hand. `lattice_ackermann_32.json`: the same with 32 headings, a trial (`--headings 32`, `lattice_headings:=32`) |
| `config/lattice_ackermann_smooth.json`, `_smooth_fast.json` | the smooth moves, a trial (`lattice_moves:=`): made by `pixi run python3 tools/lattice_smooth.py` (ours: Nav2's generator draws no ramps); never edit by hand |
| `config/amcl.yaml` | AMCL, the trial (`amcl:=true`), ours and stock alike: `tf_broadcast: false`, the lidar scan, base_link |
| `maps/` | `empty` and `example_lot` maps and their specs ([maps/README.md](./maps/README.md)) |

## Invariants

- Forward only unless `reversing:=true`: nothing senses behind the car. Ravensim and the
  bench turn it on, and the car where the lidar watches behind.
- No spin: a car can't turn on the spot.
- Our nodes aren't lifecycle nodes, so each respawns 1 s after it dies (`test_respawn.py`),
  the stop zone and the map server too. While one is down the car stops: the costmap's
  and odometry's age checks, twist_mux's timeout.
- The navigator backs up only with `reversing:=true`, inside `max_reverse_speed`.
- The stop zone stops the car for 3 or more points in its box, and when a source goes
  silent. It guards only navigation. Sources: radar, lidar (or the cubes), the kerbs'
  steps, the ghost, and the camera obstacles (`camera_guard`). The box follows the way
  the car goes:

  | Going | Stop zone |
  |---|---|
  | forward (or standing) | front bumper to 1 m ahead |
  | reversing | rear bumper to 1 m behind |
- The ghost only adds obstacles; it never clears or hides a real one.
- Navigation is a driver like any other: below every person, test, script and the point driver in
  twist_mux; the stop-sign and sensor locks (30) hold it.
- Map frame x east, y north; origin = the navsat datum.

## Side Effects

- Publishes `/drivers/nav/cmd_vel`, `/map`, `/plan` and our costmaps.
- Writes a tuned copy of `nav2.yaml` to a temporary file (`nav2_*.yaml`), removed when
  the launch ends.
- Calls `/fromLL` for GPS waypoints.

## Verification

```sh
pixi run build && colcon test --packages-select aav_navigation && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_navigation && python3 -m pytest -q`
(`tools/test.sh` runs it). `pixi run test` runs everything. In the simulator:
`pixi run sim-check`; to compare the followers: `pixi run measure controllers`.
