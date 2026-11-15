# ros/drivers

## Overview

**Drivers propose; the mux picks.** Each driver publishes a `TwistStamped` on
`/drivers/<driver>/cmd_vel`. twist_mux (`ros/command_mux`) passes the highest fresh one
to ego. A person always outranks the computer.

## Subdirectories & Boundaries

| Package | Driver | Topic | Priority |
|---|---|---|---|
| [aav_teleop](./aav_teleop/README.md) | joystick (stock joy + teleop_twist_joy), keyboard, e-stop button | `/drivers/joy/cmd_vel`, `/drivers/keyboard/cmd_vel` | 100, 90 |
| [aav_rc](./aav_rc/README.md) | the RC transmitter, through ROS | `/drivers/rc/cmd_vel` | 95 |
| [aav_actuator_tests](./aav_actuator_tests/README.md) | stage-3 and stage-4 actuator tests | `/drivers/tests/cmd_vel` | 60 |
| [aav_scripted](./aav_scripted/README.md) | `/scripted_drive`: one speed profile | `/drivers/scripted/cmd_vel` | 50 |
| [aav_drive_to_point](./aav_drive_to_point/README.md) | `/drive_to_point`: drive to one GPS or map point and stop | `/drivers/point/cmd_vel` | 25 |
| [aav_navigation](./aav_navigation/README.md) | Nav2 (Smac Lattice planner, Hybrid-A* forward only; MPPI), behaviour trees, maps | `/drivers/nav/cmd_vel` | 20 |
| [aav_track](./aav_track/README.md) | trial path followers (pure pursuit, Stanley, LQR, MPC, MPC with speed) in controller_server's place: `tracker:=` (`docs/trials.md`) | `cmd_vel_nav`, inside Nav2's slot | (20) |
| [aav_mppi_lag](./aav_mppi_lag/README.md) | a trial follower: Nav2's MPPI in Rust, its model with the actuators' lag, in controller_server's place: `tracker:=mppi_lag` ([page](../../docs/mppi-lag.md)); says `/drivers/nav/lag_aware` | `cmd_vel_nav`, inside Nav2's slot | (20) |
| [aav_costmap](./aav_costmap/README.md) | a trial: our own local and global costmaps in Rust, Nav2's layers ported, beside Nav2's: `costmap:=own`; `mppi_lag` then reads ours ([page](../../docs/own-costmap.md)) | `/drivers/nav/costmap/*`, inside Nav2's slot | (20) |
| [aav_planner](./aav_planner/README.md) | a trial planner: Nav2's State Lattice in Rust, beside Nav2's; the behaviour trees call it: `planner:=own` ([page](../../docs/own-planner.md)) | `/plan`, inside Nav2's slot | (20) |
| [aav_navigator](./aav_navigator/README.md) | a trial: our own navigator in Rust (the trees' plan, follow, replan, recover; through-poses; waypoints and GPS waypoints by the datum) in bt_navigator's and waypoint_follower's place: `navigator:=own`; with `planner:=own costmap:=own tracker:=mppi_lag`, no Nav2 server navigates ([page](../../docs/own-navigator.md)) | Nav2's action names (`/navigate_to_pose`...), inside Nav2's slot | (20) |
| [aav_diffusion_planner](./aav_diffusion_planner/README.md) | a trial planner: a model draws the path, a guard checks it, the lattice's path if it fails: `planner:=diffusion` (`docs/trials.md`) | `/plan`, inside Nav2's slot | (20) |
| [aav_behaviour](./aav_behaviour/README.md) | a trial: the behaviour states and a speed cap on Nav2 from the stopping law, for a stop sign, an obstacle ahead, the fence and the goal: `behaviour:=true` ([page](../../docs/stopping.md)) | `/drivers/nav/speed_limit`, inside Nav2's slot | (20) |
| [aav_routes](./aav_routes/README.md) | street routes (Valhalla or OSRM) as Nav2 GPS waypoints; directions on the campus by a place's name (its own road graph) | `/drivers/nav/route/*` | — |
| [aav_language](./aav_language/README.md) | plain-English driving: MCP drive tools and a local-model agent | uses the actions above | — |

The lane follower on the AI slot (priority 10) lives in `ros/perception/aav_lane_geometry`.
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
