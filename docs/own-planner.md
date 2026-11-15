# Our own planner

Ben, 8 Nov: move off Nav2 **one piece at a time**, each beside Nav2 until it wins. The
order: the costmap ([done](own-costmap.md)), then **the path planner** (this page), then
the navigator. Since 14 Nov ours is the simulator's default (`stack:=ours`; Nav2's
lattice with `planner:=lattice` or `stack:=stock`). Nav2 stays on the car.

> **The short version**
> - **Switch:** `planner:=own`. Package `aav_planner` (Rust).
> - **What it is:** Nav2's State Lattice planner (our default) ported: the same
>   primitives file, the same penalties from `nav2.yaml`, the Reeds-Shepp finish, the
>   smoother. It checks the car's outline on the global costmap.
> - **Drop-in:** it answers Nav2's own `ComputePathToPose` action. The behaviour trees
>   call it instead of `planner_server`; nothing else changes.
> - **Test lot, 3 runs each:** **12 of 12 legs**, as Nav2's lattice; the same paths
>   (within a metre), the same planning time (3-5 ms a call).
> - **Campus:** both planners failed the same legs (MPPI gave up, not the planner);
>   ours took about **20 % longer** to plan the long legs (180 ms against 150 ms).
> - **Not yet a winner:** as good on the lot, a little slower on the campus.

## How it fits

```
global costmap ───────────┐   Nav2's /global_costmap/costmap_raw (costmap:=nav2)
                          │   or ours, /drivers/nav/costmap/global_raw (costmap:=own)
/tf (map → odom → car) ───┤
                          ▼
bt_navigator ──ComputePathToPose──▶ planner (ours) ──▶ /plan ──▶ the follower, screens
                                           └──────────▶ /drivers/nav/planner/status
planner_server (Nav2's lattice) keeps running beside it: /compute_path_to_pose
```

| Name | Type | For |
|---|---|---|
| `/drivers/nav/planner/compute_path_to_pose` | action, Nav2's ComputePathToPose | bt_navigator (remapped here) |
| `/plan` | nav_msgs/Path | the follower's path, the screens |
| `/drivers/nav/planner/status` | DiagnosticStatus | each plan's times, length, cusps, or why none |
| `/drivers/nav/planner/nav2_plan` | nav_msgs/Path | Nav2's lattice's own `/plan`, moved aside |

**How the swap works.** `navigation.launch.py` remaps two names in `bt_navigator`:
`compute_path_to_pose` and `compute_path_through_poses` → ours. The costmap step did the
same with the clear service. Our navigator ([page](own-navigator.md)) calls both too.

## What it does

```
start ─▶ A* over the moves ─▶ now and then: one curve straight to the goal? ─▶ smooth ─▶ path
          (primitives file)      (Reeds-Shepp; Dubins when forward only)
```

| Step | What it does | Nav2's |
|---|---|---|
| moves | 6 car-shaped moves from each of 16 headings (our file, `tools/lattice_primitives.py`); backwards too with `reversing:=true` | `LatticeMotionTable` |
| does it fit? | the car's padded outline at each pose along a move; most poses skip it (far from anything) | `GridCollisionChecker` |
| cost of a move | length × (1 + 2 × cost), × 1.05 for a turn, + 0.05 to change turn, × 2 backwards | `getTraversalCost` |
| how far still? | the larger of: round the obstacles (a grid search from the goal), and the car's shortest curve on open ground | the two heuristics |
| the finish | a Reeds-Shepp curve to the goal when one is clear (within 3 m), wider turns if cheaper | `AnalyticExpansion` |
| smoothing | pull each piece straighter, then re-join the true start and goal | `Smoother` |

**Small changes from Nav2** (all in `ros/drivers/aav_planner/PROVENANCE.md`):

- a node's pose changes only when a move makes it cheaper (Nav2 can mix two moves);
- each move is checked for collisions on its own (Nav2 remembers the first move's
  answer for every later one). Safer, but more checks: part of why ours is slower;
- the car's outline is checked where the path puts it (Nav2's is half a cell off).

## Try it

| To | Run |
|---|---|
| See it | `pixi run sim planner:=own`, then send a goal |
| Score it beside Nav2's | `pixi run measure controllers --planners lattice own --reversing --tours lot` |
| On our costmap too | add `costmap:=own` (it then plans on `/drivers/nav/costmap/global_raw`) |
| Test the maths | `cd ros/drivers/aav_planner && cargo test -p lattice-planner` |
| Time the hand-made maps | `cargo run --release -p lattice-planner --example scenes` |
| Try the smooth moves (a trial) | `pixi run sim planner:=own lattice_moves:=smooth` ([why](controllers.md#smooth-moves-a-trial-14-nov)) |
| Compare the moves offline | `cargo run --release -p lattice-planner --example moves` (the lot tour on `example_lot`) |

## Hand-made maps (no ROS)

Our car, our primitives file, `nav2.yaml`'s global costmap (0.25 m cells, 3 m of
inflation). M4 Max, the best of 5.

| Map | Forward only | Reversing |
|---|---|---|
| straight, 25 m | path, 25.0 m, < 0.1 ms | the same |
| U-turn, open ground | path, 20.9 m, 0.3 ms | path, 13.2 m (3 cusps), 0.3 ms |
| U-turn in an 8 m road | **no path** (too narrow forward) | path, 13.3 m (a three-point turn), 1.1 ms |
| parking-lot exit (nose in, wall ahead) | **no path** | path, 19.8 m, backs out first, 0.9 ms |
| narrow gap (2.5 m in a wall) | path, 24.0 m, 0.2 ms | the same |
| no path (1.0 m gap; the car is 1.1 m) | **no path** | **no path** |

The tests check every path: clear of walls, starts at the start, ends at the goal, no
turn under 80 % of the turning radius, no reversing when forward only.

## In our simulator (8 Nov)

`measure controllers --planners lattice own --reversing`, MPPI follows, 3 runs each.
Before each leg, both planners plan **from the same spot on the same map**.

### The test lot (4 legs a tour)

| | Nav2's lattice | Ours |
|---|---|---|
| Legs reached | **12 / 12** | **12 / 12** |
| Tour time, 3 runs | 187, 186, 187 s | 182, 181, 189 s |
| Path length, all legs | 135-138 m | 136-138 m |
| Changes of direction, all legs | 2, 2, 2 | 3, 2, 4 |
| Tightest turn | 3.52-3.54 m | 3.52-3.57 m |
| Turns tighter than the car can | none | none |
| One plan, the whole call (wall) | 4-41 ms | 3-37 ms |

Same spot, same map, leg by leg: the lengths agree within 2 m (once 2.2 m: 44.2 against
42.0 m), and the times within a few ms.

### The campus (3 legs, 2.5 m/s)

| | Nav2's lattice | Ours |
|---|---|---|
| Legs reached | 3 / 9 | 4 / 9 |
| "Out of the lot" (47 m), same spot | 7-12 ms | 2 ms |
| "Along the road" (254 m), same spot | 138-151 ms | **168-195 ms** |
| "Round the bend" (246 m, once), same spot | 780 ms | **914 ms** |

- **Both failed the same way:** at the start of "along the road" MPPI said "Optimizer
  fail to compute path" and Nav2 gave up (Nav2's planner 3 of 3, ours 2 of 3). The
  follower, not the planner: both planned the same 254 m path. Then "round the bend"
  started from wherever the car stopped, often too near the kerb to fit (both planners
  said "start occupied").
- **Ours is about 20 % slower on long legs.** Likely the per-move collision checks
  (above). Both are far inside the 1 s replanning period and the 5 s limit.
- TODO 140's P4-to-Fieldhouse route was not run: these legs already fail before the
  planner matters.

### The verdict

- **As good on the test lot:** the same legs, paths and times as Nav2's lattice.
- **A little slower on the campus's long legs;** nothing there decides between them yet.
- Nav2's lattice stays the default. Next: make ours faster (remember each node's check,
  as Nav2 does, where it is safe), and Hybrid-A* for forward only.

## What's missing

- **Hybrid-A*** (the forward-only default on the car): not ported. `planner:=own` with
  `reversing:=false` plans Dubins moves on the lattice instead.
- **`ComputePathThroughPoses`**: done 8 Nov (`/drivers/nav/planner/compute_path_through_poses`,
  a plan a leg, joined, as Nav2's planner_server does).
- **The navigator step**: done 8 Nov ([our own navigator](own-navigator.md)): it calls
  ours for both, and clears our global costmap.
- **On the car:** untested; the car refuses `planner:=own`.
