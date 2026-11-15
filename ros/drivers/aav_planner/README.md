# aav_planner

Our own path planner, in Rust: Nav2's Smac **State Lattice** planner ported, run
**beside** Nav2's `planner_server` (`planner:=own`); and Nav2's **Hybrid-A\*** on the same
search, a challenger (`planner_search:=hybrid`, 14 Nov). The simulator's default since 14 Nov (`stack:=ours`); Nav2's stays
on the car. The page: [docs/own-planner.md](../../../docs/own-planner.md).

```
global costmap (Nav2's /global_costmap/costmap_raw, or ours with costmap:=own), /tf
  -> planner -> /plan, /drivers/nav/planner/status
bt_navigator's compute_path_to_pose -> /drivers/nav/planner/compute_path_to_pose (ours)
planner_server keeps the lattice beside it: /compute_path_to_pose, /drivers/nav/planner/nav2_plan
```

## Overview

| Part | What it is |
|---|---|
| `crates/lattice-planner` | the maths, no ROS: the primitives file and Hybrid-A*'s arcs (`Lattice::hybrid`), the A* search (`Expansion`: lattice or Hybrid), the Reeds-Shepp and Dubins curves, the car's outline check, the two estimates (heuristics), the smoother; `scenes.rs`: the hand-made maps and the example lot |
| `crates/planner-node` | the ROS node, binary `planner` (rclrs): messages in and out only, one thread a plan |

```sh
pixi run sim planner:=own
pixi run measure controllers --planners lattice own --reversing --tours lot
pixi run sim planner:=own planner_search:=hybrid                 # Hybrid-A* instead
```

## Interface

**`planner`** (`crates/planner-node/src/main.rs`), started by `aav_navigation`'s
`navigation.launch.py planner:=own`

| Item | What |
|---|---|
| Actions | `/drivers/nav/planner/compute_path_to_pose` (ComputePathToPose) and `/drivers/nav/planner/compute_path_through_poses` (ComputePathThroughPoses: a plan a leg, joined); `bt_navigator`'s two are remapped here, and our navigator calls both |
| Subscribes | the global costmap (`costmap_topic`: Nav2's `/global_costmap/costmap_raw`, latched, with `_raw_updates`; or ours, `/drivers/nav/costmap/global_raw`); `/tf` (map -> odom -> base_link; older than 1 s is no pose: TF_ERROR) |
| Serves | `is_path_valid_service` (nav2_msgs/srv/IsPathValid, as planner_server's; `/is_path_valid` where planner_server doesn't run, else none): from the pose nearest the car on, a pose on a lethal or inscribed cell makes the path invalid |
| Publishes | `/plan` (nav_msgs/Path); `/drivers/nav/planner/status` (DiagnosticStatus, one a plan) |
| Parameters | `settings`: one JSON block from `aav_description`'s `vehicle.planner_params` (nav2.yaml's Lattice block as tuned, or with `"search": "hybrid"` its Hybrid-A* block; the car's outline, the global inflation); `costmap_topic`, `costmap_updates_topic`, `costmap_latched`, the frames |

## Invariants

- The same input as Nav2's lattice: the same primitives file, block and costmap.
  Hybrid-A*: nav2.yaml's own Hybrid-A* block, as Nav2's SmacPlannerHybrid gets it.
- One search for both: they differ only where `search::Expansion` says.
- Nav2's error codes; Nav2's message types.
- No ROS in `crates/lattice-planner`; no maths in the node.
- `cargo fmt` and `cargo clippy` clean.

## Side Effects

- Serves the action, publishes the topics above. Nothing else.

## Verification

```sh
cargo test -p lattice-planner                                # curves, hand-made maps
cargo run --release -p lattice-planner --example scenes      # the maps, timed
cargo run --release -p lattice-planner --example moves       # Nav2's moves against the smooth ones, the lot tour
cargo run --release -p lattice-planner --example hybrid      # Hybrid-A* against the lattice, the lot tour and the maps
cargo run --release -p lattice-planner --example cusps       # the turns in a street by cusp_penalty (TODO 174)
pixi run build && colcon test --packages-select aav_planner
```
