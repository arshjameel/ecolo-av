# ros/drivers

## Overview

**Drivers propose; the mux picks.** Each driver publishes a `TwistStamped` on
`/drivers/<driver>/cmd_vel`. twist_mux (`ros/command_mux`) passes the highest fresh one
to ego. A person always outranks the computer.

## Subdirectories & Boundaries

| Package | Driver | Topic | Priority |
|---|---|---|---|
| [aav_teleop](./aav_teleop/README.md) | joystick (stock joy + teleop_twist_joy), keyboard, e-stop button | `/drivers/joy/cmd_vel`, `/drivers/keyboard/cmd_vel` | 100, 90 |
| [aav_rc](./aav_rc/README.md) | the RC transmitter, through ROS (Python: the fall-back, `rc_impl:=python`) | `/drivers/rc/cmd_vel` | 95 |
| [aav_rc_rs](./aav_rc_rs/README.md) | the same `rc_driver` in Rust, the default since 17 Nov | `/drivers/rc/cmd_vel` | 95 |
| [aav_actuator_tests](./aav_actuator_tests/README.md) | stage-3 and stage-4 actuator tests | `/drivers/tests/cmd_vel` | 60 |
| [aav_scripted](./aav_scripted/README.md) | `/scripted_drive`: one speed profile | `/drivers/scripted/cmd_vel` | 50 |
| [aav_drive_to_point](./aav_drive_to_point/README.md) | `/drive_to_point`: drive to one GPS or map point and stop (Python: the fall-back, `drive_to_point_impl:=python`) | `/drivers/point/cmd_vel` | 25 |
| [aav_drive_to_point_rs](./aav_drive_to_point_rs/README.md) | the same `point_driver` and `drive_to_point` client in Rust, the default since 18 Nov | `/drivers/point/cmd_vel` | 25 |
| [aav_navigation](./aav_navigation/README.md) | navigation for a car: the launch file and settings (`nav2.yaml`, in Nav2's layout) that start our map server, costmaps, planner, navigator, follower, smoother and stop zone; the maps. Nav2 itself went on 17 Nov ([as it was](../../docs/history/nav2-as-it-was.md)) | `/drivers/nav/cmd_vel` | 20 |
| [aav_track](./aav_track/README.md) | trial path followers (pure pursuit, Stanley, LQR, MPC, MPC with speed) in `mppi_lag`'s place: `tracker:=` (`docs/trials.md`) | `cmd_vel_nav`, inside navigation's slot | (20) |
| [aav_mppi_lag](./aav_mppi_lag/README.md) | the path follower: Nav2's MPPI ported to Rust, its model with the actuators' lag: `tracker:=mppi_lag`, the default ([page](../../docs/mppi-lag.md)); says `/drivers/nav/lag_aware` | `cmd_vel_nav`, inside navigation's slot | (20) |
| [aav_mpc_lag](./aav_mpc_lag/README.md) | a trial follower: `aav_track`'s lag-aware MPC (`mpc_lag`) in Rust, a challenger beside the Python one, in `mppi_lag`'s place: `tracker:=mpc_lag_rs`; says `/drivers/nav/lag_aware` | `cmd_vel_nav`, inside navigation's slot | (20) |
| [aav_costmap](./aav_costmap/README.md) | our local and global costmaps in Rust, Nav2's layers ported, and the map server: `costmap:=own`, the only one ([page](../../docs/own-costmap.md)) | `/drivers/nav/costmap/*`, `/map` | (20) |
| [aav_planner](./aav_planner/README.md) | the planner: Nav2's State Lattice ported to Rust, and its Hybrid-A* (`planner_search:=hybrid`), and the lane-centre planner (`path:=lane`); the navigator calls it: `planner:=own` ([page](../../docs/own-planner.md)) | `/plan` | (20) |
| [aav_navigator](./aav_navigator/README.md) | the navigator in Rust (plan, follow, replan, recover; through-poses; waypoints and GPS waypoints by the datum), in Nav2's bt_navigator's and waypoint_follower's old place: `navigator:=own`, the only one ([page](../../docs/own-navigator.md)) | Nav2's action names (`/navigate_to_pose`...) | (20) |
| [aav_smoother](./aav_smoother/README.md) | the velocity smoother in Rust (Nav2's `velocity_smoother` for a car: the steering's rate and full lock): `smoother:=own`, the only one ([page](../../docs/own-smoother.md)) | `/control/cmd_vel_smoothed` or `/drivers/nav/cmd_vel` | (20) |
| [aav_diffusion_planner](./aav_diffusion_planner/README.md) | a trial planner: a model draws the path, a guard checks it, the lattice's path if it fails: `planner:=diffusion` (`docs/trials.md`) | `/plan` | (20) |
| [aav_diffusion_rs](./aav_diffusion_rs/README.md) | the diffusion planner's model worker in Rust, a challenger under the same node: `diffusion_impl:=rust` (ONNX Runtime; Core ML on a Mac, [ONNX](../../docs/onnx.md)) | (its node's) | (20) |
| [aav_behaviour_rs](./aav_behaviour_rs/README.md) | Rust (the Python removed 20 Nov); a trial: the behaviour states and a speed cap from the stopping law, in the road ahead, for a stop sign, an obstacle ahead, the fence and the goal: `behaviour:=true`, on in `stack:=ours` ([page](../../docs/stopping.md)) | `/world/road_ahead`, `/drivers/nav/behaviour/state` | (20) |
| [aav_routes](./aav_routes/README.md) | street routes (Valhalla or OSRM) as GPS waypoints for the navigator; directions on the campus by a place's name (its own road graph) | `/drivers/nav/route/*` | — |
| [aav_language](./aav_language/README.md) | plain-English driving: MCP drive tools and a local-model agent | uses the actions above | — |

The AI slot (priority 10) has no driver since 19 Nov: the lane followers were removed (TODO 264, git tag `save/lane-followers`); the costmap draws the lanes now.
The hardware panel's buttons (`/drivers/buttons/cmd_vel`, priority 45) live in `ros/hud/aav_hud` (`hw_panel`).

## Invariants

- Drivers publish `TwistStamped` only. Ackermann appears only after ego.
- Logic modules have no ROS imports and their own tests.
- On the real car, `aav_drive_to_point` and `aav_actuator_tests` wait for a person to confirm;
  `aav_language`'s `talk` waits for "yes" (its drive server alone doesn't ask). Each caps
  the speed, or leaves it to ego's `max_speed` (`aav_drive_to_point`).
- The programs (`aav_scripted`, `aav_drive_to_point`, `aav_actuator_tests`) stop on the e-stop,
  when a person takes over, and when the inputs they need go stale. twist_mux's e-stop
  lock holds every driver, people too.
- No driver ever clears the e-stop.

## Side Effects

- Publishes the topics above. The actuator tests write runs to `~/aav-tests/`;
  `aav_language` logs tool calls; `aav_routes` calls a local router over HTTP.

## Verification

- Run: `pixi run test` (or `colcon test --packages-select <pkg>` after `pixi run build`).
