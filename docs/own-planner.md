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
> - **Hybrid-A\* too (14 Nov):** `planner_search:=hybrid` runs Nav2's Hybrid-A*
>   ported, on the same search ([below](#hybrid-a-a-second-search-14-nov)). A challenger.

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
| Run Hybrid-A* instead | `pixi run sim planner:=own planner_search:=hybrid` (or `stack:=ours` with the league's `hybrid_rs`) |
| Hybrid-A* against the lattice offline | `cargo run --release -p lattice-planner --example hybrid` |
| ... and in the simulator | `pixi run measure controllers --planners own own_hybrid --reversing --tours lot` |

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

## Hybrid-A*: a second search (14 Nov)

Ben, 14 Nov: "Hybrid-A* in our Rust planner: Now" (TODO 178). Nav2's
SmacPlannerHybrid (Dolgov et al., 2010), ported into the same crate. **A challenger:**
the league's champion stays our lattice (`lattice_rs`); this is `hybrid_rs`.

> - **Switch:** `planner:=own planner_search:=hybrid`. Nothing else changes: the same
>   node, actions, topics and costmap.
> - **Settings:** `nav2.yaml`'s own Hybrid-A* block (`GridBased`), as Nav2's gets it:
>   72 headings, the planning radius (3.57 m), Reeds-Shepp with `reversing:=true`,
>   else Dubins (forward only).

### What is shared, what differs

Nav2 runs both planners on one A*; only the node type differs. Ours too:

```
                  ┌── the lattice: the primitives file's moves, each swept
search.rs (A*) ───┤
                  └── Hybrid-A*: 3 short arcs (6 reversing), checked where they end
  shared: the outline check, both estimates, the curve to the goal, the smoother
```

| Part | The lattice | Hybrid-A* |
|---|---|---|
| moves from a heading | 6 from our file, 16 headings | straight, full left, full right; 72 headings (5°) |
| a move's length | as the file draws it | 0.63 m: just over a cell's diagonal, a 10° turn |
| backwards | the same moves, driven backwards | the same (Nav2's `REVERSE`, `REV_LEFT`, `REV_RIGHT`) |
| does it fit? | along the whole move | at its end only |
| a turn costs | × 1.05, + 0.05 to change | × 1.2 (no change penalty in nav2.yaml) |
| round the obstacles | every cell | 2 × 2 blocks (Nav2's default: a quarter of the work) |

### Offline: the lot tour (no ROS)

`cargo run --release -p lattice-planner --example hybrid`: `example_lot`, the four legs
`measure controllers --tours lot` drives, our car, M4 Max, the best of 3.

**Reversing (the car's setting):**

| Leg | Lattice: m, ms, cusps | Hybrid-A*: m, ms, cusps |
|---|---|---|
| a left turn | 21.9, < 1, 0 | 22.5, < 1, **1** |
| round the building | 40.9, 1, 0 | 41.1, < 1, 0 |
| turn back (35, 42, 0°) to (20, 42, 180°) | 20.8, 2, 1 | 20.1, 8, **3** |
| home | 50.4, 14, 1 | 50.7, 39, **3** |
| **whole tour** | **133.9 m, 17 ms, 2 cusps** | **134.3 m, 48 ms, 7 cusps** |

**Forward only (Dubins):**

| Leg | Lattice: m, ms | Hybrid-A*: m, ms |
|---|---|---|
| a left turn | 21.9, < 1 | 21.9, < 1 |
| round the building | 40.9, 1 | 40.8, < 1 |
| turn back (a loop round) | 39.3, 8 | **36.4**, 5 |
| home | 52.3, 27 | 52.7, 44 |
| **whole tour** | **154.4 m, 36 ms** | **151.9 m, 50 ms** |

- **Tightest turn:** before smoothing, Hybrid-A* keeps to 3.57 m on every leg (its arcs
  are exactly the radius); the lattice to 3.63 m. The shared smoother then tightens
  some corners on both (3.12 m Hybrid, 3.33 m lattice), still wider than full lock
  (2.86 m).
- **Cusps:** Hybrid-A* finishes with a Reeds-Shepp curve from a 5° heading, so near
  the goal it often backs up a little to line up (as Nav2's does). More changes of
  direction for the follower.
- **Time:** 2-3 times the lattice's on long legs (more, shorter moves); all well
  inside the 5 s limit.

### Not done yet

- **The simulator:** not run. Next: `measure controllers --planners own own_hybrid
  --reversing --tours lot`, 3 runs.
- No `use_quadratic_cost_penalty` or `cache_obstacle_heuristic` (nav2.yaml leaves both
  off).

## The staircase and the strong smoother (15 Nov)

TODO 168. The lattice has 16 headings. A road at 8° is planned as steps (0°, 17°,
0°... every ~9 m, 0.2 m high). Nav2's smoother weights (w_data 0.2, w_smooth 0.3) only
take out wiggles shorter than ~2 m, so the steps stay, and the follower drives them.

| Switch | What |
|---|---|
| `plan_smoothing:=nav2` | Nav2's weights (`navigation.launch.py`'s default; `stack:=stock`; the car) |
| `plan_smoothing:=strong` | w_data 0.02 in the GridBased block's `smoother`: Nav2's lattice and ours both read it. The sim's `ours` stack picks it |

```sh
W_DATA=0.02 cargo run --release -p lattice-planner --example staircase   # no ROS
```

The plan's own swing on a 6.5 m road at 2.2 m/s: 0.08-0.16 m with Nav2's weights,
0.02-0.07 m strong. The lot's corners keep to the car (tightest 3.08 m; the car turns at
2.86 m). In the sim it took a share of the campus's sway off; the rest, and the
numbers: [mppi-lag.md, The sway that was left](mppi-lag.md#the-sway-that-was-left).
`tests/scenes.rs` checks the steps go (8°: under half the swing).

## What's missing

- **Hybrid-A*** (the forward-only default on the car): ported 14 Nov
  ([above](#hybrid-a-a-second-search-14-nov)); not yet run in a simulator.
- **`ComputePathThroughPoses`**: done 8 Nov (`/drivers/nav/planner/compute_path_through_poses`,
  a plan a leg, joined, as Nav2's planner_server does).
- **The navigator step**: done 8 Nov ([our own navigator](own-navigator.md)): it calls
  ours for both, and clears our global costmap.
- **On the car:** untested; the car refuses `planner:=own`.
