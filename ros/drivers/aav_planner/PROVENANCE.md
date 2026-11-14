# Provenance

The `lattice-planner` crate ports the parts of Nav2's `nav2_smac_planner` that our
nav2.yaml runs as `planner:=lattice` (SmacPlannerLattice). It is rewritten in Rust's own
style, not transliterated: plain structs and functions, enums for errors, no plugins, no
ROS. The curves come from OMPL, as Nav2 takes them.

## Where it came from

| | |
|---|---|
| Repo | https://github.com/ros-planning/navigation2 |
| Package | `nav2_smac_planner` |
| Commit | the Jazzy commit in `vendor/ros2-src.repos` (645abd95, the same as `aav_costmap`'s) |
| Licence | Apache-2.0 (Samsung Research America, Steve Macenski and the Nav2 authors) |
| Read 8 Nov 2026 | `src/smac_planner_lattice.cpp` (parameters, `createPlan`), `src/a_star.cpp` (`createPath`), `src/node_lattice.cpp` (the motion table, `getNeighbors`, `isNodeValid`, `getTraversalCost`, `getHeuristicCost`, `getDistanceHeuristic`, `precomputeDistanceHeuristic`, `backtracePath`), `src/node_hybrid.cpp` (`resetObstacleHeuristic`, `getObstacleHeuristic`), `src/analytic_expansion.cpp`, `src/collision_checker.cpp`, `src/smoother.cpp`, `include/.../types.hpp` (the smoother's defaults), `include/.../utils.hpp` |
| Curves | OMPL's `DubinsStateSpace` and `ReedsSheppStateSpace` (BSD-3-Clause, Rice University; Mark Moll), the formulas Nav2 calls through OMPL |

## What was taken

| Ours | Nav2's (or OMPL's) |
|---|---|
| `lattice.rs` | `LatticeMotionTable`: the primitives file, `getClosestAngularBin` |
| `search.rs` | `AStarAlgorithm::createPath` (the goal, the tolerance and on-approach iterations, the clock and cancel checks), `NodeLattice::getNeighbors` (forward and reversed moves), `isNodeValid` (the sweep along a move), `getTraversalCost` (every penalty), `backtracePath`; `AnalyticExpansion` (when to try, the ancestor refinement, wider radii up to 4x, the score, the max-cost rule) |
| `heuristic.rs` | `getObstacleHeuristic` (the lazy grid search from the goal, 4 cells clear of the edge), `getDistanceHeuristic` and its precomputed table (mirrored in y) |
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
| No 2D, Hybrid-A* or omni models; no costmap downsampling; no debug markers | our nav2.yaml runs the lattice for a car (Hybrid-A* is a later piece) |
| The node reads one JSON block (`vehicle.planner_params`) | the launch file makes it from the same tuned nav2.yaml Nav2 gets |
| The car's pose comes from `/tf` (map -> odom -> base_link), latest | rclrs has no tf2 yet |
| `Planner::plan_through` (8 Nov): one plan a leg, each from the last goal, joined (the repeated join pose dropped); the failing leg is named | `PlannerServer::computePlanThroughPoses`, which does the same with any planner plugin; for `ComputePathThroughPoses` |
| 14 Nov: a global costmap older than `costmap_timeout` (2 s: planner_server's `costmap_update_timeout` 1 s plus the global costmap's 1 Hz publish period) fails the goal at once with TIMEOUT (207; 307 through poses), `costmap::Arrival` | Nav2's `PlannerTimedOut` after waiting for a current costmap; ours planned on the last one for ever. It fails at once, not after a wait: our navigator's recovery replans |
| 14 Nov: map -> odom or odom -> base_link over 1 s old (`TRANSFORM_TIMEOUT`) fails the goal with TF_ERROR | Nav2 looks the pose up through tf2 with `transform_tolerance`; ours kept the last pose for ever when `/tf` stopped (the gap list) |
