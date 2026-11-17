# aav_navigation

Nav2, set up for the car, as one more driver: goals in, `/drivers/nav/cmd_vel` out to
twist_mux (priority 20). Only stock Nav2 servers, set up by our config, launch file and
behaviour trees: no node of our own.

```
goal (RViz, HUD, street_router, language_drive) -> bt_navigator -> planner + controller
  -> velocity_smoother -> [collision_monitor] -> /drivers/nav/cmd_vel -> twist_mux -> ego
```

## Overview

It starts only the servers a car needs (not nav2_bringup's full list: no docking, no
route server, and AMCL only as a trial, `amcl:=true`, since robot_localization does
localization). The car's footprint,
turning radius and reverse speed come from
`ros/bringup/aav_description/config/vehicle.yaml`; `aav_description`'s
`vehicle.nav2_tuned()` and `nav2_rewrites()` write them into a copy of `config/nav2.yaml`.

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
| `lidar` | false | `lidar_scan` (aav_lidar) flattens `/sensors/lidar/points` to `/sensors/lidar/scan`, for the costmap and collision_monitor |
| `voxels` | false | with `lidar`: a trial, `aav_voxels`' `voxels` cuts the cloud into 0.25 m cubes; the costmap's lidar_layer and collision_monitor read those instead of the scan, and `lidar_scan` runs only for `amcl` ([design](../../../docs/designs/2026-10-30_voxels.md)) |
| `voxels_impl` | `python` | with `voxels` or `screen_cubes`: `python` (`aav_voxels`) or `rust` (`aav_voxels_rs`: the same node, topics and cubes; the launch file passes it `vehicle.yaml`'s lidar mount and the car's outline) |
| `kerbs` | false | with `lidar` (ignored without): the kerbs (TODO 119, [kerbs](../../../docs/kerbs.md)). The Rust `voxels` node (`aav_voxels_rs`, whatever `voxels_impl` says) runs with `steps:=true`, and the local costmap's `kerb_layer` (Nav2's and ours) marks `/perception/voxels/steps`: marks only, kept until the window rolls past. Not the stop zone's |
| `collision_monitor` | empty | empty: on wherever radar, lidar, ghost or camera guard runs |
| `camera_obstacles` | false | the costmap's camera layer reads `/perception/camera_obstacles` (off until Q22) |
| `camera_guard` | false | with `camera_obstacles`: collision_monitor stops for them too |
| `ghost` | false | a pretend person (`aav_ghost`, `/safety/ghost/scan`) |
| `reversing` | false | true: Reeds-Shepp paths, reverse to `-max_reverse_speed`, trees back up 1 m |
| `max_speed` | empty (1.0 m/s) | Nav2's top speed (`sim.launch.py sim_speed:=`) |
| `controller` | `mppi` | or `rpp` (Regulated Pure Pursuit); `docs/path-followers.md` |
| `planner` | `lattice` (`car.launch.py`: `lattice`, or `hybrid` forward only) | the planner behind `GridBased`: `lattice` (SmacPlannerLattice, `config/lattice_ackermann.json`; refused if it no longer fits `vehicle.yaml` or the map's cells) or `hybrid` (SmacPlannerHybrid) keep to the turning radius; `smac2d`, `navfn`, `theta_star` don't (trials; `car.launch.py` refuses them); `own`: our own planner (`aav_planner`, Nav2's lattice in Rust) answers the trees, Nav2's lattice beside it (`docs/own-planner.md`). `docs/controllers.md`, Planners |
| `lattice_headings` | 16 | with `planner:=lattice`: 16 headings (`config/lattice_ackermann.json`) or 32, a trial (`config/lattice_ackermann_32.json`) |
| `lattice_moves` | `nav2` | with `planner:=lattice` or `own`: `smooth` (1 m/s) or `smooth_fast` (2.5 m/s), a trial: moves whose curvature changes no faster than the steering turns (`config/lattice_ackermann_smooth*.json`), and the lattice's smoother off |
| `planner_search` | `lattice` | with `planner:=own`: `hybrid`, a challenger: our planner runs Nav2's Hybrid-A* ported instead of the State Lattice, on `nav2.yaml`'s `GridBased` block (Reeds-Shepp with `reversing:=true`, else Dubins; `docs/own-planner.md`) |
| `plan_around` | true | false: the planner sees only the map, so the car stops (stop tests) |
| `tracker` | `none` | a trial follower (`aav_track`): `pure_pursuit`, `stanley`, `lqr`, `mpc`, `mpc_full`; or `mppi_lag` (`aav_mppi_lag`: Nav2's MPPI with the lag); or `mpc_lag_rs` (`aav_mpc_lag`: `mpc_lag` in Rust, a challenger). It serves `follow_path`; controller_server's becomes `follow_path_unused`. `aav_track`'s are refused with `reversing:=true` (`docs/trials.md`); `mppi_lag` reverses as Nav2's MPPI |
| `navigator` | `nav2` | `own`: our own navigator (`aav_navigator`) in place of `bt_navigator` and `waypoint_follower`, serving their action names (a drop-in: no goal sender knows which runs); with `planner:=own costmap:=own tracker:=mppi_lag` also no `planner_server`, `controller_server` or `behavior_server` (`docs/own-navigator.md`). `car.launch.py` always passes `nav2` |
| `plan_smoothing` | `nav2` | the planner's smoother: `nav2`, Nav2's weights; `strong` (TODO 168): w_data 0.02 in the GridBased block's `smoother`, so a road at an angle is no staircase. Nav2's lattice and ours both read it (`vehicle.PLAN_SMOOTHINGS`). The sim's `ours` stack picks `strong` |
| `mppi_lag_steer_gamma` | empty | with `tracker:=mppi_lag`: its steering's control cost weight (`steer_gamma`); empty: nav2.yaml's gamma. 0.05 (TODO 168): less sampling jitter; it eases to gamma below 1 m/s. The sim's `ours` stack picks 0.05 |
| `map_correction` | `newest` | with a `tracker`: `slow` (TODO 186; the sim's `ours` stack picks it), the follower's held map -> odom moves toward the newest at most 0.1 m/s and 1°/s at the car, so the GPS's jitter never jumps the path under the car; `newest`: each new path by the newest map -> odom, as until 15 Nov. The follower gets it as its bool parameter `map_correction`, never in its own yaml (the file would win) |
| `speed_loop` | `default` | the spine's speed loop profile the car runs (`default`, `smooth`, `lqi`, `firm`): the follower's model (`mppi_lag`, `mpc_lag`, `mpc_lag_rs`; the two MPCs refuse `lqi`) and the behaviour planner's stopping law assume it. `sim.launch.py` and `bench.launch.py` pass their own (TODO 163); the car's spine runs `default` |
| `lane_costs` | empty | with `costmap:=own`: a `lanes.yaml`, drawn into our global costmap as soft costs, lowest at the right lane's centre (which lane by `/plan`), so our planner keeps right and crosses a line only when it pays. A challenger (TODO 148, `docs/own-costmap.md`); refused without `costmap:=own`. `sim.launch.py lane_costs:=true` passes the world's lane map |
| `costmap_chunks` | empty | with `costmap:=own`: a folder of the map's 64 m chunks (`chunks.yaml`, `fine/`; `pixi run campus` writes the campus's). Our global costmap then holds only the chunks round the car and along the route ahead (`/drivers/nav/planner/route`), each read and inflated on its own core, not `/map`; our planner plans a goal off it in steps (TODO 175, `docs/own-costmap.md`). `sim.launch.py costmap_chunks:=auto` passes the campus's |
| `lane_route` | empty | with `planner:=own`: a `lanes.yaml`; our planner then plans the right lane's centre along it (routed by its roads), the lattice joining the car and the goal to it, and the lattice alone off the map. A challenger (TODO 148, the league's `path:=lane`; `docs/own-planner.md`); refused without `planner:=own`. `sim.launch.py lane_route:=true` passes the world's lane map |
| `lane_pass` | true | with `lane_route`: something parked on the lane line is passed by moving only its stretch left, the least that clears it, then back (Ben, 16 Nov; `docs/own-planner.md`); false: the whole path is the lattice's, as before |
| `amcl` | false | true (needs `lidar:=true`): stock AMCL, scored only, on `/localization/amcl/pose` (`config/amcl.yaml`: `tf_broadcast: false`) |
| `amcl_x`, `amcl_y`, `amcl_yaw` | 0 | AMCL's start pose, map frame (`sim.launch.py` gives the sim's start) |
| `amcl_map` | empty | empty: AMCL reads Nav2's `/map`; else its own map YAML, on `/localization/amcl/map` (`sim.launch.py`: `maps/example_lot_amcl.yaml`, [why](./maps/README.md#why-amcl-has-a-map-of-its-own)) |
| `use_sim_time`, `autostart` | false, true | |

Nodes started:

| Node | Does |
|---|---|
| `map_server` | publishes `/map` |
| `planner_server` | SmacPlannerLattice: drivable paths for a car (`/plan`); or `planner:=`'s (forward only: SmacPlannerHybrid), always as `GridBased` |
| `controller_server` | MPPI (Ackermann model) or RPP; `cmd_vel` -> `cmd_vel_nav` |
| `behavior_server` | wait, backup, drive_on_heading (no spin) |
| `bt_navigator` | actions `/navigate_to_pose`, `/navigate_through_poses`; trees in `behavior_trees/` |
| `waypoint_follower` | actions `/follow_waypoints`, `/follow_gps_waypoints` (lat/lon via `/fromLL`) |
| `velocity_smoother` | `cmd_vel_nav` -> `/drivers/nav/cmd_vel`, or -> `/control/cmd_vel_smoothed` |
| `collision_monitor` | `/control/cmd_vel_smoothed` -> `/drivers/nav/cmd_vel`; publishes `/safety/collision_monitor/state`, `stop_zone`, `collision_points_marker` |
| `lidar_scan`, `ghost_obstacle` | with `lidar:=true`, `ghost:=true` |
| `voxels` (`aav_voxels`, or `aav_voxels_rs` with `voxels_impl:=rust`) | with `voxels:=true` or `screen_cubes:=true`; `aav_voxels_rs` with `kerbs:=true` |
| `track` (`aav_track`), or `mppi_lag` (`aav_mppi_lag`) with `tracker:=mppi_lag`, or `mpc_lag_rs` (`aav_mpc_lag`) with `tracker:=mpc_lag_rs` | with `tracker:=` on: `follow_path` -> `cmd_vel_nav`, in controller_server's place |
| `amcl`, `lifecycle_manager_localization` | with `amcl:=true`: AMCL and its own lifecycle manager; with `amcl_map:=`, also `amcl_map_server` |
| `lifecycle_manager_navigation` | brings the servers up |

Inputs: `/localization/odom/local`, `/tf`, `/goal_pose`, and the obstacle topics above.
All commands are `geometry_msgs/TwistStamped`.

Files:

| Folder | Holds |
|---|---|
| `config/nav2.yaml` | every server's parameters |
| `config/lattice_ackermann.json` | the State Lattice planner's moves for our car (`planner:=lattice`): made by Nav2's generator, `pixi run python3 tools/lattice_primitives.py` (`tools/lattice_primitives.py`); never edit by hand. `lattice_ackermann_32.json`: the same with 32 headings, a trial (`--headings 32`, `lattice_headings:=32`) |
| `config/lattice_ackermann_smooth.json`, `_smooth_fast.json` | the smooth moves, a trial (`lattice_moves:=`): made by `pixi run python3 tools/lattice_smooth.py` (ours: Nav2's generator draws no ramps); never edit by hand |
| `config/amcl.yaml` | AMCL, the trial (`amcl:=true`): `tf_broadcast: false`, the lidar scan, base_link |
| `behavior_trees/` | `navigate_to_pose_car.xml`, `navigate_through_poses_car.xml`, and `*_car_reversing.xml` twins with BackUp. The to-pose trees keep their path within 8 m of the goal (`vehicle.final_approach`, TODO 17) |
| `maps/` | `empty` and `example_lot` maps and their specs ([maps/README.md](./maps/README.md)) |

## Invariants

- Forward only unless `reversing:=true`: nothing senses behind the car. Gazebo and the
  bench turn it on, and the car where the lidar watches behind.
- No spin behaviour: a car can't turn on the spot.
- Our own nodes (costmap, planner, navigator, smoother, the followers, behaviour) aren't
  lifecycle nodes, so each respawns 1 s after it dies (`test_respawn.py`). While one is
  down the car stops: the costmap's and odometry's age checks, twist_mux's timeout.
- Each tree and its reversing twin differ only by BackUp, and BackUp stays inside
  `max_reverse_speed`.
- collision_monitor stops the car for 3 or more points in its stop zone, and when a
  source goes silent. It guards only Nav2. Sources: radar, lidar, the ghost, and the
  camera obstacles (`camera_guard`). The zone follows the way the car goes:

  | Going | Stop zone |
  |---|---|
  | forward (or standing) | front bumper to 1 m ahead |
  | reversing | rear bumper to 1 m behind |
- The ghost only adds obstacles; it never clears or hides a real one.
- Nav2 is a driver like any other: below every person, test, script and the point driver in
  twist_mux; the stop-sign and sensor locks (30) hold it.
- Map frame x east, y north; origin = the navsat datum.

## Side Effects

- Publishes `/drivers/nav/cmd_vel` and Nav2's own topics (`/map`, `/plan`, costmaps).
- Writes a tuned copy of `nav2.yaml` to a temporary file (`nav2_*.yaml`), removed when
  the launch ends.
- Calls `/fromLL` for GPS waypoints.

## Verification

```sh
pixi run build && colcon test --packages-select aav_navigation && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_navigation && python3 -m pytest -q`
(`tools/test.sh` runs it). `test/test_behavior_trees.py` checks the twins, no spin, and the final approach.
`pixi run test` runs everything. In Gazebo: `pixi run sim-check`; to compare the
controllers: `pixi run measure controllers`.
