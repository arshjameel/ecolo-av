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
| `collision_monitor` | empty | empty: on wherever radar, lidar, ghost or camera guard runs |
| `camera_obstacles` | false | the costmap's camera layer reads `/perception/camera_obstacles` (off until Q22) |
| `camera_guard` | false | with `camera_obstacles`: collision_monitor stops for them too |
| `ghost` | false | a pretend person (`aav_ghost`, `/safety/ghost/scan`) |
| `reversing` | false | true: Reeds-Shepp paths, reverse to `-max_reverse_speed`, trees back up 1 m |
| `max_speed` | empty (1.0 m/s) | Nav2's top speed (`sim.launch.py sim_speed:=`) |
| `controller` | `mppi` | or `rpp` (Regulated Pure Pursuit); `docs/path-followers.md` |
| `planner` | `hybrid` | the planner behind `GridBased`: `hybrid` (SmacPlannerHybrid) or `lattice` (SmacPlannerLattice, `config/lattice_ackermann.json`; refused if it no longer fits `vehicle.yaml` or the map's cells) keep to the turning radius; `smac2d`, `navfn`, `theta_star` don't (trials; `car.launch.py` refuses them). `docs/controllers.md`, Planners |
| `plan_around` | true | false: the planner sees only the map, so the car stops (stop tests) |
| `tracker` | `none` | a trial follower (`aav_track`): `pure_pursuit`, `stanley`, `lqr`, `mpc`. It serves `follow_path`; controller_server's becomes `follow_path_unused`. Refused with `reversing:=true` (`docs/trials.md`) |
| `amcl` | false | true (needs `lidar:=true`): stock AMCL, scored only, on `/localization/amcl/pose` (`config/amcl.yaml`: `tf_broadcast: false`) |
| `amcl_x`, `amcl_y`, `amcl_yaw` | 0 | AMCL's start pose, map frame (`sim.launch.py` gives the sim's start) |
| `amcl_map` | empty | empty: AMCL reads Nav2's `/map`; else its own map YAML, on `/localization/amcl/map` (`sim.launch.py`: `maps/example_lot_amcl.yaml`, [why](./maps/README.md#why-amcl-has-a-map-of-its-own)) |
| `use_sim_time`, `autostart` | false, true | |

Nodes started:

| Node | Does |
|---|---|
| `map_server` | publishes `/map` |
| `planner_server` | SmacPlannerHybrid: drivable paths for a car (`/plan`); or `planner:=`'s, always as `GridBased` |
| `controller_server` | MPPI (Ackermann model) or RPP; `cmd_vel` -> `cmd_vel_nav` |
| `behavior_server` | wait, backup, drive_on_heading (no spin) |
| `bt_navigator` | actions `/navigate_to_pose`, `/navigate_through_poses`; trees in `behavior_trees/` |
| `waypoint_follower` | actions `/follow_waypoints`, `/follow_gps_waypoints` (lat/lon via `/fromLL`) |
| `velocity_smoother` | `cmd_vel_nav` -> `/drivers/nav/cmd_vel`, or -> `/control/cmd_vel_smoothed` |
| `collision_monitor` | `/control/cmd_vel_smoothed` -> `/drivers/nav/cmd_vel`; publishes `/safety/collision_monitor/state`, `stop_zone`, `collision_points_marker` |
| `lidar_scan`, `ghost_obstacle` | with `lidar:=true`, `ghost:=true` |
| `track` (`aav_track`) | with `tracker:=` on: `follow_path` -> `cmd_vel_nav`, in controller_server's place |
| `amcl`, `lifecycle_manager_localization` | with `amcl:=true`: AMCL and its own lifecycle manager; with `amcl_map:=`, also `amcl_map_server` |
| `lifecycle_manager_navigation` | brings the servers up |

Inputs: `/localization/odom/local`, `/tf`, `/goal_pose`, and the obstacle topics above.
All commands are `geometry_msgs/TwistStamped`.

Files:

| Folder | Holds |
|---|---|
| `config/nav2.yaml` | every server's parameters |
| `config/lattice_ackermann.json` | the State Lattice planner's moves for our car (`planner:=lattice`): made by Nav2's generator, `pixi run lattice-primitives` (`tools/lattice_primitives.py`); never edit by hand |
| `config/amcl.yaml` | AMCL, the trial (`amcl:=true`): `tf_broadcast: false`, the lidar scan, base_link |
| `behavior_trees/` | `navigate_to_pose_car.xml`, `navigate_through_poses_car.xml`, and `*_car_reversing.xml` twins with BackUp. The to-pose trees keep their path within 8 m of the goal (`vehicle.final_approach`, TODO 17) |
| `maps/` | `empty` and `example_lot` maps and their specs ([maps/README.md](./maps/README.md)) |

## Invariants

- Forward only unless `reversing:=true`: nothing senses behind the car. Gazebo and the
  bench turn it on; `car.launch.py` leaves it off.
- No spin behaviour: a car can't turn on the spot.
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
controllers: `pixi run controller-compare`.
