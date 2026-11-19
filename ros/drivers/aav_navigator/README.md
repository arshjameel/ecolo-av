# aav_navigator

Our own navigator, in Rust: what Nav2's `bt_navigator`, our behaviour trees and
`waypoint_follower` do for the car (`navigator:=own`), **in their place**. The simulator's default since 14 Nov (`stack:=ours`); the only
navigator, sim and car, since Nav2 went on 17 Nov. The page: [docs/own-navigator.md](../../../docs/own-navigator.md).

```
goal (RViz, the map window, street_router, language_drive, the tools)
  -> navigator -> ComputePathToPose / ComputePathThroughPoses (our planner)
               -> FollowPath (mppi_lag, or a trial tracker)
               -> ClearEntireCostmap (ours), BackUp (the follower backing
               along a straight line; reversing)
```

## Overview

| Part | What it is |
|---|---|
| `crates/navigator` | the maths, no ROS: the goal state machine (plan, follow, replan, the final approach, the recoveries in turn, give up), through-poses, waypoints; `path.rs`: the nearest point, the distance left, a pose passed; `call.rs`: one FollowPath call, its feedback counted from the moment it is asked for (TODO 240) |
| `crates/navigator-node` | the ROS node, binary `navigator` (rclrs): messages in and out only |

```sh
pixi run sim
pixi run measure controllers --stacks own own_calm --tours lot obstacle --runs 3
```

## Interface

**`navigator`** (`crates/navigator-node/src/main.rs`), started by `aav_navigation`'s
`navigation.launch.py navigator:=own`

| Item | What |
|---|---|
| Actions | Nav2's own names and types: `/navigate_to_pose`, `/navigate_through_poses`, `/follow_waypoints`, `/follow_gps_waypoints` (Nav2's `bt_navigator` and `waypoint_follower` served them until 17 Nov) |
| Goals | in `map`, `odom` or `base_link` (put on the map by the latest /tf); another frame is refused |
| Subscribes | `/goal_pose` (a to-pose goal); `/tf` (map -> odom -> base_link; older than 1 s: no car pose) |
| Publishes | `/drivers/nav/navigator/status` (DiagnosticStatus, 2 Hz and at each goal's end) |
| Calls | `planner_action`, `planner_through_action` (ComputePathToPose, ComputePathThroughPoses); `follow_action` (FollowPath); `clear_local`, `clear_global` (ClearEntireCostmap, comma-separated lists); `backup_action` (BackUp; empty: the follower's FollowPath on a straight line 1 m behind the car); `is_path_valid_service` (IsPathValid, `/is_path_valid`, once a second while a path is kept: on the final approach, and a to-pose path up to `keep_path_age`, 10 s, as the trees' KeepAValidPath) |
| Parameters | `settings`: one JSON block from `aav_description`'s `vehicle.navigator_params` (the trees' numbers); the names above; `navsat_file` (the datum); the frames |

A goal sender needs no switch: it sends to Nav2's name, and our navigator
answers (`vehicle.navigator_action(name)` gives it).

## Invariants

- One goal at a time: a newer goal on any action replaces the running one (aborted).
- Nav2's error codes, Nav2's message types: a drop-in for every goal sender.
- It never drives the car itself: the follower does, through the velocity smoother,
  collision_monitor, twist_mux and its locks, which never change.
- Back up only with `reversing:=true`: through behavior_server's BackUp, or where none runs
  (all ours) through the follower, which must reverse (`mppi_lag`), on `path::straight_back`.
- No ROS in `crates/navigator`; no maths in the node. `cargo fmt` and `cargo clippy` clean.

## Side Effects

- Serves the actions, publishes its status. Nothing else.

## Verification

```sh
cargo test -p navigator                      # the goals against a pretend planner and follower
pixi run build && colcon test --packages-select aav_navigator
```
