# Provenance

The `lattice-planner` crate ports the parts of Nav2's `nav2_smac_planner` that our
nav2.yaml runs as `planner:=lattice` (SmacPlannerLattice) and, since 14 Nov, as
`planner:=hybrid` (SmacPlannerHybrid, Hybrid-A*: Dolgov, Thrun, Montemerlo and Diebel,
2010). It is rewritten in Rust's own style, not transliterated: plain structs and
functions, enums for errors, no plugins, no ROS. The curves come from OMPL, as Nav2
takes them.

Nav2 shares one A* (`AStarAlgorithm<NodeT>`), one `AnalyticExpansion<NodeT>`, one
collision checker and one smoother between its two car planners; only the node type
differs. Ours does the same: one `Search`, and an `Expansion` (lattice or Hybrid) for
the three places they differ.

## Where it came from

| | |
|---|---|
| Repo | https://github.com/ros-planning/navigation2 |
| Package | `nav2_smac_planner` |
| Commit | the Jazzy commit in `vendor/ros2-src.repos` (645abd95, the same as `aav_costmap`'s) |
| Licence | Apache-2.0 (Samsung Research America, Steve Macenski and the Nav2 authors) |
| Read 14 Nov 2026 (Hybrid-A*) | `src/smac_planner_hybrid.cpp` (parameters, `configure`, `createPlan`), `src/node_hybrid.cpp` (`HybridMotionTable::initDubin`, `initReedsShepp`, `getProjections`, `getClosestAngularBin`, `isNodeValid`, `getTraversalCost`, `getHeuristicCost`, `getObstacleHeuristic` with `downsample_obstacle_heuristic`, `getDistanceHeuristic`, `precomputeDistanceHeuristic`, `getNeighbors`, `backtracePath`), `src/a_star.cpp` (the template, shared with the lattice), `src/analytic_expansion.cpp` (shared) |
| Read 8 Nov 2026 | `src/smac_planner_lattice.cpp` (parameters, `createPlan`), `src/a_star.cpp` (`createPath`), `src/node_lattice.cpp` (the motion table, `getNeighbors`, `isNodeValid`, `getTraversalCost`, `getHeuristicCost`, `getDistanceHeuristic`, `precomputeDistanceHeuristic`, `backtracePath`), `src/node_hybrid.cpp` (`resetObstacleHeuristic`, `getObstacleHeuristic`), `src/analytic_expansion.cpp`, `src/collision_checker.cpp`, `src/smoother.cpp`, `include/.../types.hpp` (the smoother's defaults), `include/.../utils.hpp` |
| Curves | OMPL's `DubinsStateSpace` and `ReedsSheppStateSpace` (BSD-3-Clause, Rice University; Mark Moll), the formulas Nav2 calls through OMPL |

## What was taken

| Ours | Nav2's (or OMPL's) |
|---|---|
| `lattice.rs` | `LatticeMotionTable`: the primitives file, `getClosestAngularBin`; `Lattice::hybrid`: `HybridMotionTable::initDubin` and `initReedsShepp` (the arcs: a whole number of heading bins, chord over a cell's diagonal, `allow_primitive_interpolation`), as a lattice of one-pose moves |
| `search.rs`, Hybrid-A* | `Expansion::Hybrid`: `NodeHybrid::isNodeValid` (the move's end only), `getTraversalCost` (the six turn kinds, the first move's `travel_distance_cost`), `getHeuristicCost` from the node's own pose; the reversed moves are the facing heading's moves driven backwards, which are Nav2's `REVERSE`, `REV_LEFT` and `REV_RIGHT` |
| `search.rs` | `AStarAlgorithm::createPath` (the goal, the tolerance and on-approach iterations, the clock and cancel checks), `NodeLattice::getNeighbors` (forward and reversed moves), `isNodeValid` (the sweep along a move), `getTraversalCost` (every penalty), `backtracePath`; `AnalyticExpansion` (when to try, the ancestor refinement, wider radii up to 4x, the score, the max-cost rule) |
| `heuristic.rs` | `getObstacleHeuristic` (the lazy grid search from the goal, 4 cells clear of the edge; Hybrid-A*'s 2 x 2 downsampling, each block its cheapest cell, the answer doubled), `getDistanceHeuristic` and its precomputed table (mirrored in y) |
| `collision.rs` | `GridCollisionChecker::inCollision` (the circumscribed-cost shortcut, 72 turned outlines), `FootprintCollisionChecker::footprintCost` |
| `smoother.rs` | `Smoother`: pieces between cusps, the gradient smoothing with refinement, `updateApproximatePathOrientations`, the Dubins boundary expansions at both ends |
| `curves.rs` | OMPL's Dubins (6 words) and Reeds-Shepp (48 candidate words, Desaulniers and Soumis' corrections), and their interpolation |

## What changed, and why

| Change | Why |
|---|---|
| A node's pose is set only when a move improves its cost | Nav2 sets it whenever a valid move reaches it, so a node's pose and its parent's move can disagree, and the path then jumps by up to a cell |
| A move's validity is checked for each move, not cached on the node it reaches | Nav2 caches the first move's answer, so a node reached first by a blocked move stays blocked for every other move |
| The car's origin is at `origin + x * resolution` in the outline check too | Nav2 adds half a cell there (`mapToWorld`) but not in the path it returns: its outline is checked half a cell off |
| The outline's heading is the nearest of the 72, not the lower one | Nav2 truncates; nearest is never further off |
| The analytic curve keeps its own heading at each pose | Nav2 rounds them to the 16 lattice headings, then the smoother re-points them |
| Two curve samples in the same cell are allowed | Nav2 treats that as a failed curve |
| The distance table's mirrored heading wraps (`(n - h) % n`) | Nav2's index can step one past the end for heading 0 |
| `StartOccupied` only when not one move from the start fits | as Nav2's (`num_iterations == 1`), said plainly |
| No 2D or omni models; no costmap downsampling; no debug markers | our nav2.yaml runs the lattice and Hybrid-A* for a car |
| 14 Nov: Hybrid-A* (`Planner::hybrid`, `planner_search:=hybrid`), on the same search as the lattice, with the lattice's fixes above (a node's pose only when cheaper; each move checked on its own) | TODO 178 (Ben, 14 Nov: "Hybrid-A* in our Rust planner: Now") |
| 14 Nov: Hybrid-A*'s moves are made again when the costmap's cells change size, on the next plan | Nav2 does it in its parameter callback; ours has none |
| 14 Nov: Hybrid-A* has no `use_quadratic_cost_penalty` or `cache_obstacle_heuristic` | nav2.yaml leaves both off |
| 14 Nov: the lattice's distance estimate takes the node's cell, Hybrid-A*'s its pose (`DistanceTable::get` takes both as numbers) | as Nav2's two nodes do |
| 14 Nov: the example lot's map loader moved from `examples/moves.rs` to `scenes::map` (errors, not panics), with the lot tour (`LOT_TOUR`) | the Hybrid-A* tests and `examples/hybrid.rs` plan on it too |
| The node reads one JSON block (`vehicle.planner_params`) | the launch file makes it from the same tuned nav2.yaml Nav2 gets |
| The car's pose comes from `/tf` (map -> odom -> base_link), latest | rclrs has no tf2 yet |
| `Planner::plan_through` (8 Nov): one plan a leg, each from the last goal, joined (the repeated join pose dropped); the failing leg is named | `PlannerServer::computePlanThroughPoses`, which does the same with any planner plugin; for `ComputePathThroughPoses` |
| 14 Nov: a global costmap older than `costmap_timeout` (2 s: planner_server's `costmap_update_timeout` 1 s plus the global costmap's 1 Hz publish period) fails the goal at once with TIMEOUT (207; 307 through poses), `costmap::Arrival` | Nav2's `PlannerTimedOut` after waiting for a current costmap; ours planned on the last one for ever. It fails at once, not after a wait: our navigator's recovery replans |
| 14 Nov: map -> odom or odom -> base_link over 1 s old (`TRANSFORM_TIMEOUT`) fails the goal with TF_ERROR | Nav2 looks the pose up through tf2 with `transform_tolerance`; ours kept the last pose for ever when `/tf` stopped (the gap list) |
