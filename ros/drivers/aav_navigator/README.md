# aav_navigator

Our own navigator, in Rust: what Nav2's `bt_navigator`, our behaviour trees and
`waypoint_follower` do for the car. A trial (`navigator:=own`), **in their place**; Nav2's
stays the default, and on the car. The page: [docs/own-navigator.md](../../../docs/own-navigator.md).

```
goal (RViz, the map window, street_router, language_drive, the tools)
  -> navigator -> ComputePathToPose / ComputePathThroughPoses (our planner, or Nav2's)
               -> FollowPath (mppi_lag, or controller_server)
               -> ClearEntireCostmap (ours, or Nav2's), BackUp (behavior_server, reversing)
```

## Overview

| Part | What it is |
|---|---|
| `crates/navigator` | the maths, no ROS: the goal state machine (plan, follow, replan, the final approach, the recoveries in turn, give up), through-poses, waypoints; `path.rs`: the nearest point, the distance left, a pose passed |
| `crates/navigator-node` | the ROS node, binary `navigator` (rclrs): messages in and out only |

```sh
pixi run sim navigator:=own planner:=own costmap:=own tracker:=mppi_lag reversing:=false
pixi run measure controllers --stacks nav2 own --tours lot obstacle --runs 3
```

## Interface

**`navigator`** (`crates/navigator-node/src/main.rs`), started by `aav_navigation`'s
`navigation.launch.py navigator:=own`

| Item | What |
|---|---|
| Actions | `/drivers/nav/navigator/navigate_to_pose`, `navigate_through_poses`, `follow_waypoints`, `follow_gps_waypoints` (Nav2's types) |
| Subscribes | `/goal_pose` (a to-pose goal); `/tf` (map -> odom -> base_link; older than 1 s: no car pose) |
| Publishes | `/drivers/nav/navigator/status` (DiagnosticStatus, 2 Hz and at each goal's end) |
| Calls | `planner_action`, `planner_through_action` (ComputePathToPose, ComputePathThroughPoses); `follow_action` (FollowPath); `clear_local`, `clear_global` (ClearEntireCostmap, comma-separated lists); `backup_action` (BackUp; empty: none) |
| Parameters | `settings`: one JSON block from `aav_description`'s `vehicle.navigator_params` (the trees' numbers); the names above; `navsat_file` (the datum); the frames |

A goal sender finds the action with `vehicle.navigator_action(navigator, name)`;
`street_router` and `language_drive` take a `navigator` parameter.

## Invariants

- One goal at a time: a newer goal on any action replaces the running one (aborted).
- Nav2's error codes, Nav2's message types: a drop-in for every goal sender.
- It never drives the car itself: the follower does, through the velocity smoother,
  collision_monitor, twist_mux and its locks, which never change.
- Back up only with `reversing:=true`, through behavior_server's BackUp.
- No ROS in `crates/navigator`; no maths in the node. `cargo fmt` and `cargo clippy` clean.

## Side Effects

- Serves the actions, publishes its status. Nothing else.

## Verification

```sh
cargo test -p navigator                      # the goals against a pretend planner and follower
pixi run build && colcon test --packages-select aav_navigator
```
