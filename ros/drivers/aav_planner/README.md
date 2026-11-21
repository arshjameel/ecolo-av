# aav_planner

Our own path planner, in Rust: Nav2's Smac **State Lattice** planner ported
(`planner:=own`); and Nav2's **Hybrid-A\*** on the same search, a challenger
(`planner_search:=hybrid`, 14 Nov). The simulator's default since 14 Nov (`stack:=ours`);
the only planner, sim and car, since Nav2 went on 17 Nov. The page: [docs/own-planner.md](../../../docs/own-planner.md).

```
global costmap (ours, /drivers/nav/costmap/global_raw), /tf
  -> planner -> /plan, /drivers/nav/planner/status
the navigator -> /drivers/nav/planner/compute_path_to_pose
```

## Overview

| Part | What it is |
|---|---|
| `crates/lattice-planner` | the maths, no ROS: the primitives file and Hybrid-A*'s arcs (`Lattice::hybrid`), the A* search (`Expansion`: lattice or Hybrid), the Reeds-Shepp and Dubins curves, the car's outline check, the two estimates (heuristics), the smoother; `lane.rs`: the lane-centre planner (`path:=lane`: the right lane's centre by the lane map's road graph, the lattice joining the ends; a loop round the block to a goal close by is the lattice's); with `lane_graph:=lanelet2` it routes on the Lanelet2 map instead (`lanelet-map`: each lane its own centre, neighbours, one way, the stop lines on the way; TODO 175); `pass.rs`: something parked on the lane line is passed by moving only its stretch left, the least that clears it, then back (Ben, 16 Nov; `lane_pass:=false`: the lattice, as before); `jerk_pass.rs`: the same pass as a piecewise-jerk QP along the lane (Apollo's idea; Clarabel), the lateral jerk held at the car's top speed (`lane_pass_shape:=jerk`, TODO 214, a challenger); `steps.rs`: a goal off the costmap planned to where the way there leaves it (TODO 175); `scenes.rs`: the hand-made maps and the example lot; `spec.rs`: the planner from its `settings` JSON (`planner_from_json`, the node's and the diffusion data's); `examples/teach.rs`: our global costmap and planner on a map file, a JSON line a request, the diffusion planner's teachers (`pixi run diffusion data`, TODO 242) |
| `crates/planner-node` | the ROS node, binary `planner` (rclrs): messages in and out only, one thread a plan |

```sh
pixi run sim planner:=own
pixi run measure controllers --planners own own_hybrid --reversing --tours lot
pixi run sim planner:=own planner_search:=hybrid                 # Hybrid-A* instead
```

## Interface

**`planner`** (`crates/planner-node/src/main.rs`), started by `aav_navigation`'s
`navigation.launch.py planner:=own`

| Item | What |
|---|---|
| Actions | `/drivers/nav/planner/compute_path_to_pose` (ComputePathToPose) and `/drivers/nav/planner/compute_path_through_poses` (ComputePathThroughPoses: a plan a leg, joined); our navigator calls both (Nav2's `bt_navigator` did, until 17 Nov) |
| Subscribes | the global costmap (`costmap_topic`: ours, `/drivers/nav/costmap/global_raw`); `/tf` (map -> odom -> base_link; older than 1 s is no pose: TF_ERROR) |
| Serves | `is_path_valid_service` (nav2_msgs/srv/IsPathValid, as Nav2's planner_server's: `/is_path_valid`): from the pose nearest the car on, a pose on a lethal or inscribed cell makes the path invalid |
| Publishes | `/plan` (nav_msgs/Path); `/drivers/nav/planner/route` (nav_msgs/Path: the route level's whole way to the goal, however far: the lane line, or the start and goal; a costmap made of chunks holds the chunks along it); `/drivers/nav/planner/route_stops` (geometry_msgs/PoseArray, with a lane map: the stop lines the route meets; Lanelet2 only); `/drivers/nav/planner/status` (DiagnosticStatus, one a plan; `how`: lane, pass (with `pass_m`) or lattice and why; `cut`: the goal was off the costmap); with a lane map `stops`, how many |
| Parameters | `settings`: one JSON block from `aav_description`'s `vehicle.planner_params` (nav2.yaml's Lattice block as tuned, with its `smoother` weights when `plan_smoothing:=strong`, or with `"search": "hybrid"` its Hybrid-A* block; the car's outline, the global inflation; with `lane_route:=`, `lanes`: the lane map's roads, or `{"lanelet2": "<map.osm>"}` with `lane_graph:=lanelet2`, and the lane-centre planner runs); `costmap_topic`, `costmap_updates_topic`, `costmap_latched`, the frames |

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
cargo bench -p lattice-planner                               # the lot's and campus's legs, lattice and Hybrid-A*, forward and reversing (criterion)
cargo run --release -p lattice-planner --example moves       # Nav2's moves against the smooth ones, the lot tour
cargo run --release -p lattice-planner --example hybrid      # Hybrid-A* against the lattice, the lot tour and the maps
W_DATA=0.02 cargo run --release -p lattice-planner --example staircase   # a road at an angle: the plan's own swing (TODO 168)
cargo run --release -p lattice-planner --example cusps       # the turns in a street by cusp_penalty (TODO 174)
cargo run --release -p lattice-planner --example lane -- <lanes.json>   # path:=lane on the campus's legs (its header: how to make the file)
cargo run --release -p lattice-planner --example steps -- <lanes.json> <chunks>   # the same legs on the chunks round the car, in steps
echo '{"teachers": {}}' | cargo run --release -p lattice-planner --example teach   # the diffusion data's teacher (tools/diffusion_data.py drives it)
pixi run build && colcon test --packages-select aav_planner
```
