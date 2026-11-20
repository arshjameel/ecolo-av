# Our own planner

Ben, 8 Nov: move off Nav2 **one piece at a time**, each beside Nav2 until it won. The
order: the costmap ([done](own-costmap.md)), then **the path planner** (this page), then
the navigator. Since 14 Nov ours is the simulator's default (`stack:=ours`).
Nav2 went on 17 Nov ([as it was](history/nav2-as-it-was.md)): ours is the only one, in the sim and on the car. The comparisons with Nav2's below are the record.

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
>   15 Nov: 11 % off without changing a path (`examples/campus.rs` times the legs on
>   the real map, no ROS; the outline's check no longer allocates). Splitting across cores
>   doesn't pay: one step of the search is about 1 µs. 16 Nov: a move that can't improve
>   its node skips the outline check (3-9 % off, the same paths); **re-timed in the sim,
>   still slower** ([below](#re-timed-in-the-sim-16-nov-todo-152)).
> - **Not yet a winner:** as good on the lot, a little slower on the campus.
> - **Hybrid-A\* too (14 Nov):** `planner_search:=hybrid` runs Nav2's Hybrid-A*
>   ported, on the same search ([below](#hybrid-a-a-second-search-14-nov)). A challenger.
> - **The lane-centre planner (16 Nov):** `path:=lane` plans the right lane's centre
>   along the lane map, the lattice only at the ends
>   ([below](#the-lane-centre-planner-pathlane-16-nov)). **`ours`' path since 16 Nov**
>   (3 clean runs each: [the swap](#the-swap-16-nov-3-clean-runs-each)).
> - **Passing something parked (16 Nov):** only the blocked stretch moves left, then
>   back: over the centre line 4.8 s instead of 13, back in 9 m instead of 18
>   ([below](#passing-something-parked-16-nov)). In `ours`.

## How it fits

```
global costmap (ours) ────┐   /drivers/nav/costmap/global_raw
/tf (map → odom → car) ───┤
                          ▼
navigator ──ComputePathToPose──▶ planner (ours) ──▶ /plan ──▶ the follower, screens
                                        └──────────▶ /drivers/nav/planner/status
```

| Name | Type | For |
|---|---|---|
| `/drivers/nav/planner/compute_path_to_pose` | action, Nav2's ComputePathToPose | the navigator |
| `/plan` | nav_msgs/Path | the follower's path, the screens |
| `/drivers/nav/planner/status` | DiagnosticStatus | each plan's times, length, cusps, or why none |

**How the swap worked.** Until 17 Nov `navigation.launch.py` remapped two names in Nav2's
`bt_navigator` (`compute_path_to_pose` and `compute_path_through_poses`) to ours, with
Nav2's `planner_server` running beside it. Now our navigator ([page](own-navigator.md))
calls ours directly, and Nav2's planner is gone ([as it was](history/nav2-as-it-was.md)).

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
| Score it against our Hybrid-A* | `pixi run measure controllers --planners own own_hybrid --reversing --tours lot` |
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
| blocked narrow street (3.5 m, closed 5 m ahead, TODO 174) | **no path** | path, 38.8 m, backs out (1 cusp), 2.2 ms |
| narrow gap (2.5 m in a wall) | path, 24.0 m, 0.2 ms | the same |
| no path (1.0 m gap; the car is 1.1 m) | **no path** | **no path** |

The tests check every path: clear of walls, starts at the start, ends at the goal, no
turn under 80 % of the turning radius, no reversing when forward only.

### A price on changing gear (`cusp_penalty`, 15 Nov, TODO 174)

Nav2 prices a backward move (`reverse_penalty`), never the stop and start between gears.
In a narrow street that gave turns with 5 or 6 cusps, and the car drove them all
(`sim-check --three-point`: 7 changes of direction). Two parts, ours only:

| Part | What |
|---|---|
| `cusp_penalty` (m) | each change between forward and back costs that many metres of straight driving |
| The last curve | with a price set, a Reeds-Shepp curve to the goal that changes gear is refused; the search, which prices each change, finds the end instead |

Offline (`cargo run --release -p lattice-planner --example cusps`), M4 Max:

| Street | `cusp_penalty` 0 (Nav2's way) | 2 m | 4 m | 8 m |
|---|---|---|---|---|
| 8 m road, U-turn | 5 cusps, 13.3 m, 2 ms | 2, 16.1 m, 62 ms | 2, 16.0 m, 100 ms | 2, 15.8 m, 69 ms |
| 6.5 m street (`--three-point`'s) | 6 cusps, 16.5 m, 12 ms | 3, 16.3 m, 55 ms | 3, 16.3 m, 71 ms | 3, 16.3 m, 125 ms |
| parking-lot exit | 1 cusp | 1 | 1 | 1 |

A price alone (the last curve still free) gave 3-5 cusps, no better than chance.

In our simulator (15 Nov, all ours, reversing), 4 m against 0:

| Test | 0 (Nav2's way) | 4 m |
|---|---|---|
| `sim-check --three-point` (6.5 m street) | 9 changes of direction (7 earlier that day) | **4 and 4** (2 runs), 0.65 m from the walls |
| the lot tour (`measure controllers`, 3 and 2 runs) | 12/12, sway 0.031, off the path 0.21 m, worst 1.57 m; turn back 30 s, 2 reversals | 8/8, sway 0.031, 0.21 m, worst 0.98 m; turn back 29-30 s, 2 reversals |
| the campus tour (3 runs and 1) | 9/9, sway 0.231, 0.22 m | 3/3, sway 0.225, 0.22 m |

**4 m is our planner's default since 15 Nov** (`nav2.yaml`'s Lattice block,
`cusp_penalty`; Nav2's planner ignores it). Set 0 for Nav2's way.

### A narrow street, blocked (19 Nov, TODO 174)

Too narrow to turn in, even in three points, and closed ahead: the car must back out.
No special mode: the lattice's backwards moves and the navigator's retry do it.

| Step | Who | Tested by |
|---|---|---|
| the car stands before the block, no progress | the stop zone holds it | |
| clear the local costmap, follow again; then clear both and plan again | the navigator's recoveries | `cargo test -p navigator` (`a_blocked_narrow_street_is_left_backwards_on_the_next_plan`) |
| a path backing out of the street, one cusp | the planner (reversing on) | the scene "blocked narrow street" (`cargo test -p lattice-planner`) |
| drive it, backwards then forwards | `mppi_lag`; `aav_track`'s trackers too since 19 Nov | `aav_track` `test_gears.py` |
| all of it, in our simulator | | `pixi run sim-check --blocked-street-only` (stage 2) |

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
| `plan_smoothing:=nav2` | Nav2's weights (`navigation.launch.py`'s default; the car) |
| `plan_smoothing:=strong` | w_data 0.02 in the GridBased block's `smoother`: Nav2's lattice and ours both read it. The sim's `ours` stack picks it |

```sh
W_DATA=0.02 cargo run --release -p lattice-planner --example staircase   # no ROS
```

The plan's own swing on a 6.5 m road at 2.2 m/s: 0.08-0.16 m with Nav2's weights,
0.02-0.07 m strong. The lot's corners keep to the car (tightest 3.08 m; the car turns at
2.86 m). In the sim it took a share of the campus's sway off; the rest, and the
numbers: [mppi-lag.md, The sway that was left](history/mppi-lag-log.md#the-sway-that-was-left).
`tests/scenes.rs` checks the steps go (8°: under half the swing).

## Re-timed in the sim (16 Nov, TODO 152)

`measure controllers --planners lattice own --reversing --tours campus`, 2 + 2 runs.
Before each leg, both planners plan **from the same spot on the same map**; each one's
own `planning_time` (the search and the smoother). Our simulator, an M4 Max.

| Leg | Ours (ms) | Nav2's lattice (ms) | Ours, slower by |
|---|---|---|---|
| out of the lot (46 m) | 3-7 | 6-9 | none |
| along the road (255 m) | 185, 186, 188, 197 | 122, 145, 146, 171 | **~30 %** (means 189, 146) |
| round the bend (~180 m) | 230, 233, 289, 357 | 227, 235, 242 | ~18 % |

Ours prices each change of gear (`cusp_penalty` 4 m), Nav2's doesn't, so the two searches
differ a little (Nav2's plans had 2-6 cusps).

**The cheap win kept (16 Nov):** a move's cost never falls as the ground's cost rises, so
a move that, on free ground, already costs more than its node's best skips its outline
check. The same nodes and paths. `WHOLE=1 CUSP_PENALTY=4 cargo run --release -p
lattice-planner --example campus` (the whole map, as the sim's), interleaved, best of 5:

| Leg | Before (ms) | After (ms) |
|---|---|---|
| out of the lot, forward | 120-121 | 116 |
| along the road, forward | 256-258 | 249 |
| along the road, reversing | 150-152 | 136-138 |
| round the bend, reversing | 45 | 43 |

**Left:** where the whole map's extra time goes (a search step 0.95 µs there, 0.82 µs on
the tour's corner of it).

### Every core? Measured, and no (19 Nov, TODO 152)

Where a plan's time goes (`sample` on the campus legs): the outline traces are only about
a quarter. The rest is the search itself (the open list, the node table, the obstacle
estimate), one step after another.

| Tried | Along the road, forward (ms) | Kept? |
|---|---|---|
| before (one core) | 198 | |
| each node's moves checked on every core (rayon) | 1356-4066 | **no**: a node has ~20 moves of ~0.1 µs; rayon's hand-off costs more |
| the next 32-512 nodes' moves checked ahead on every core | 2008-3605 | **no**: the next node is almost always a new child, so the work ahead is wasted |
| each node's own outline check kept (`Node::fit`), no list of moves made each step, the node table sized at start | **161** | **yes**: the same paths, the same iterations |

`cargo run --release -p lattice-planner --example campus` (the tour's corner), best of 5,
an M4 Max:

| Leg | Before (ms) | After (ms) |
|---|---|---|
| out of the lot, forward | 121.5 | 102.5 |
| along the road, forward | 197.7 | 161.0 |
| round the bend, forward | 33.7 | 27.8 |
| along the road, reversing | 140.1 | 111.1 |
| round the bend, reversing | 43.9 | 36.0 |

About 18-21 % faster. The outlines turned to each heading were already made once, at
start-up (`Checker::new`). Not yet timed on the Orin.

## The lane-centre planner (`path:=lane`, 16 Nov)

TODO 148 and 169; the Fable review's change 7 ("follow the lane's centre, shift only to
go round": Autoware's idea). A challenger beside the lanes layer (`lane_costs:=true`,
[own-costmap.md](own-costmap.md#the-lanes-layer-15-nov-todo-148-a-challenger)).

```text
 car ─lattice─▶ ═══════ the right lane's centre, routed on the lane map ═══════ ─▶ goal
   (or straight, if the car is on its lane)            (straight, if the goal is on it)
```

| Step | What |
|---|---|
| The map | `lanes.yaml` (the world's own; the campus's from OpenStreetMap), as the lanes layer reads it. Roads join where they share a point |
| The route | Dijkstra over the road segments, never turning back on one; one-way roads only their way |
| The line | each segment moved onto the right lane's centre (`Road::lane`, the same as the lanes layer), corners rounded at the turning radius (3.6 m) |
| The ends | the lattice from the car to the line 8.1 m on, and from 8.1 m before the end to the goal. A car within 0.6 m and 20° of the line goes straight onto it 2 m on; a goal on its end, straight too |
| Something parked on it | only that stretch moves left, the least that clears it, then back ([below](#passing-something-parked-16-nov)) |
| Off the map | the start or goal over 20 m from every road (the lot), a route too short, the line blocked with no room to pass, no join, or a loop round the block (over 2 x the straight distance + 40 m): **the lattice, all of it** |
| Says how | `/drivers/nav/planner/status` `how`: `lane` (and `lane_m`), `pass` (and `pass_m`), or `lattice: off_map`, `short`, `blocked`, `join_failed`, `detour` |

| Try it | Run |
|---|---|
| See it | `pixi run sim world:=campus path:=lane` (or `lane_route:=true`; needs `planner:=own`) |
| The legs, no ROS | `cargo run --release -p lattice-planner --example lane -- <lanes.json>` (its header says how to make the file) |
| Score it | `pixi run measure controllers --stacks own_calm --tours university --runs 2 --reversing --launch behaviour:=true lane_route:=true` |
| Code | `crates/lattice-planner/src/lane.rs` (no ROS; 6 tests, and `tests/lane.rs` with the real lattice: 6) |

**The campus tour can't show it.** Its start (lot P4) and two of its goals are off the lane
map, so every leg falls back to the lattice (`examples/lane.rs`). A new tour,
`university`, has about 290 m of mapped road: out of P4 onto University Drive, down it
135 m, then round the bend south 150 m.

### The scores (16 Nov, our simulator, the University Drive tour)

`own_calm` (ours) with `behaviour:=true`, `--reversing`, 2.5 m/s; kept right by
`tools/lane_score.py`. Results in `/tmp/w148` (not in git).

| Variant | Runs | Legs reached | Sway (m/s²) | Left-right changes /100 m | Off the lane's centre (m) | Over the centre line | Crossings |
|---|---|---|---|---|---|---|---|
| As now | 3 | 7 of 9 | 0.093 | 9.7 | 1.51 | **68 %** | 32 |
| `lane_costs:=true` (the ramp) | 3 | 8 of 9 | 0.087 | 9.1 | 0.59 | 1 % | 2 |
| `path:=lane` | 6 | 11 of 18 | **0.063** | **6.5** | **0.10** | **0 %** | 2 |

| Leg, s each run (x: not reached) | Onto University Drive | Down it | Round the bend |
|---|---|---|---|
| As now | 43, x, 43 | 60, x, 60 | 69, 122, 68 |
| `lane_costs:=true` | 46, x, 41 | 60, 102, 60 | 68, 68, 68 |
| `path:=lane` | x, 43, 43, 43, 44, 45 | x, 60, x, x, 61, 61 | x, 67, x, x, 68, 68 |

**Read with care.**

- **Every leg missed, in every variant, was the computer, not the path.** Another
  session ran a second simulator at the same time (load 17-37 on 16 cores): the local
  costmap went over 0.8 s old and the follower gave up, the sensor watchdog went stale,
  or (2 runs) the global costmap never came, so the planner had nothing to plan on.
  On the quietest runs all three reached every leg (`path:=lane` 6 of 6, the others 3 of 3).
- **The time is the same** when a leg is reached: 43, 60 and 68 s.
- **The first version** joined the car with the lattice at every replan (once a second);
  on the bends it backed up now and then: 10 reversals in 2 runs, the bend leg 86-109 s.
  Now a car on its lane is joined straight: 0 reversals in 6 runs.
- The campus tour (2 runs each, the same load): 2, 1 and 4 of 6 legs; `path:=lane`
  fell back to the lattice there, as expected.

**The verdict then: a challenger, proposed for `ours`.** It kept the lane best and swayed
least, but its reached column was lower on paper (the load), and the league asks for 3
clean runs of each on the same routes.

### The swap (16 Nov, 3 clean runs each)

The same tour and stack, one variant after the other, each batch started with the
machine's load under 8 (no other simulator running). Results in `/tmp/w152` (not in git).

| Variant | Legs reached | Time (s) | Sway (m/s²) | Off the path (m), mean / worst | Left-right changes /100 m | Off the lane's centre (m) | Over the centre line | Crossings |
|---|---|---|---|---|---|---|---|---|
| As now | 9 of 9 | 512 | 0.087 | 0.11 / 0.72 | 9.6 | 1.43 | 61 % | 36 |
| `lane_costs:=true` (the ramp) | 9 of 9 | 517 | 0.090 | 0.13 / 1.06 | 8.7 | 0.59 | 1 % | 3 |
| `path:=lane` | 9 of 9 | 512 | **0.055** | **0.09 / 0.61** | **5.4** | **0.11** | **0 %** | 0 |

Each leg's time, every run: 42-44, 60 and 68-69 s in all three (the ramp's first leg once
51 s). `path:=lane` backed up 0 times.

**No worse on any column: `path:=lane` is `ours`' path since 16 Nov** (`stacks.yaml`;
`decisions.md`). Off the lane map (the lot, a three-point turn in the road world) it is
the lattice, as before. The ramp stays a challenger (`lanes:=ramp`).

### Corners as clothoids (17 Nov, TODO 211): a challenger

The lane line rounds each corner with a circle arc. Its curvature jumps from 0 to 1/radius
where the arc starts, so the steering must jump too. `lane_corners:=clothoid` turns each
corner with a clothoid (Euler spiral), an arc, and a clothoid back (Fraichard and
Scheuer's continuous-curvature turn): the curvature climbs over 4 m, never above the
turning radius's.

```text
   curvature
   1/radius ┤   ┌──────┐            arc:      ┌──────┐
            │  ╱        ╲                     │      │
          0 ┼─╱          ╲─                 ──┘      └──
             clothoid  clothoid               (a jump)
```

University Drive, `path:=lane`, 2 runs each (`pixi run measure controllers --stacks
own_calm_chunks --tours university university_box --runs 2 --reversing --launch
behaviour:=true lane_route:=true lane_corners:=<arc|clothoid>`; lanes by `lane_score.py`):

| Leg | Sway, arc | Sway, clothoid | Left-right changes /100 m | Steering flips /s | Off lane centre (m) |
|---|---|---|---|---|---|
| onto University Drive | 0.089 | 0.101 | 7.3 / 5.4 | 0.19 / 0.15 | 0.13-0.16 / 0.17-0.44 |
| down University Drive | 0.044 | 0.048 | 10.5 / 9.7 | 0.36 / 0.32 | 0.05 / 0.05 |
| round the bend south | 0.047 | 0.042 | 5.3 / 4.6 | 0.19 / 0.22 | 0.10-0.11 / 0.09-0.10 |
| past the parked box | 0.062 | 0.061 | 8.9 / 8.4 | 0.25 / 0.29 | 0.28-0.29 / 0.25-0.31 |
| whole tour | 0.056 | 0.057 | | | |

(Pairs are arc / clothoid.) **A tie**, within one run to the next. One clothoid run
swung 0.44 m off the lane's centre turning onto University Drive (14 % of it over the
centre line); the other did not. So the arcs stay the default and the clothoids wait
beside them. The steering flips that Hybrid-A* halves are not the corners' jumps: next,
the follower (TODO 214, the piecewise-jerk path).

## Passing something parked (16 Nov)

Ben, 16 Nov: "stick to the right side of the lane unless we need to for obstical
avoidance." Autoware's idea again: keep the lane line, move only the blocked stretch.

```text
                 ramp in      hold (blocked + 1 m each side)      ramp out
 lane line ═════════╗ ╭────────────────────────────────────────╮ ╔═════════════ ▶
                    ╚═╯                 ▓ the box                 ╚═╝
           (left by the least shift that fits, plus 0.5 m; never past the left kerb)
```

### Before: the whole path went to the lattice

A lane line over an obstacle gave **the whole path** to the lattice. It swung 2 m left
late (0.2-0.7 m before the box), then **stayed over the centre line** until 18 m past
the box, or near the goal.

### What changed

| Part | What |
|---|---|
| `pass.rs` (no ROS, 7 tests) | each lane-line pose where the car's padded outline doesn't fit: a blocked run, 1 m added each side. The shift: the least left step (every 0.25 m) whose whole ramp-hold-ramp fits, plus 0.5 m when that fits too; at most to the left kerb, less half the car and 0.25 m |
| The ramps | half cosines, curving at most half as sharply as the turning radius; shorter only where the car is already close, never sharper than the radius |
| No room | the other lane taken too, or the goal inside the stretch: the lattice, as before |
| The status | `how: pass`, `pass_m` (the shift) |
| Off switch | `lane_pass:=false` (sim and navigation launch): the old way |
| Sees further | the planner's copy of the lidar layer marks to **18 m** (the follower's: 12 m). At 12 m the box was planned round only 7-11 m ahead at 2.5 m/s; once the car stood 45 s at the stop zone |
| The road ahead | with a lane map, the behaviour planner reads the status: during a pass the line follows it and the left soft edge opens to the kerb, so `LaneCritic` doesn't pull the car back in. Not yet driven with `lane_map:=` |

### The scores (our simulator, 16 Nov)

`measure controllers --stacks own_calm_chunks --tours university_box --runs 3 --reversing
--launch behaviour:=true lane_route:=true`: the car starts in University Drive's right
lane, a 1 m box on the lane's centre 35 m ahead, the goal 90 m on. Scored by
`tools/lane_score.py --box 118.39 -145.69`. Load 6-12. Results in `/tmp/r3` (not in git).

| Variant | Runs | Reached | Over the centre line, most (m) | Over it (s) | Left the lane, m before the box | Back near the lane centre, m after | Closest to the box (m) | Sway (m/s²) | Stops, reversals |
|---|---|---|---|---|---|---|---|---|---|
| Before (the lattice, 12 m) | 3 | 3/3 | 1.20 (1.05-1.29) | 13.2 (8.5-22.5) | 0.4 | 18.5 | 0.84 | 0.093 | 0, 2 |
| The lattice, 18 m | 2 | 2/2 | 1.11 | 9.3 | 8.3 | 13.6 | 0.91 | 0.096 | 0, 0 |
| Pass, 12 m, clearance 0.3 | 3 | 3/3 | 0.74 | 4.0 | 0.7 | 8.5 | 0.46 | 0.063 | 1, 2 (one run stood 45 s) |
| **Pass, 18 m, clearance 0.5 (now)** | 3 | 3/3 | **0.78 (0.52-1.03)** | **4.8** | 3.4 | **9.1** | 0.58 (0.32-0.83) | **0.067** | **0, 0** |

| What it says | So |
|---|---|
| over the line a third as long, back in half the distance | the pass leaves the lane only beside the box |
| closer to the box (0.58 m, least 0.32) | it moves the least that clears; the box's place wanders with the EKF (0.35 m) |
| clearance 0.3 m came within 0.16 m once (seen far off, the box looked narrower) | hence 0.5 m |
| the first try, 12 m: one run stood 45 s at the stop zone, then backed | hence 18 m |

**A person crossing** (`sim-check --moving-only`, the road world): still waited for
(APPROACH, WAIT, GO, 0.61 m from the walker), goal reached. A parked box there
(`--behaviour-things-only`): passed, never waited for, 2.20 m clear. By the sums the
road world's one lane (4.5 m) has no room for a pass (1.45 m to the kerb, 1.5 m needed),
so that was the lattice, as before (the status wasn't recorded).

**Nothing worse without obstacles** (`--tours university --runs 2`, the same stack):

| | Legs | Time (s) | Sway (m/s²) | Off the path, mean / worst (m) | Off the lane's centre (m) | Over the centre line |
|---|---|---|---|---|---|---|
| Before (16 Nov, [chunks](own-costmap.md#the-scores)) | 6 of 6 | 342 | 0.060 | 0.09 / 0.57 | 0.05-0.17 | 0 % |
| Now | 6 of 6 | 341 | 0.056 | 0.09 / 0.58 | 0.06-0.18 | 0 % |

**In `ours` since 16 Nov** (the defaults: `lane_pass:=true`, the 18 m layer).

**Left:** a car coming the other way (the costmap sees only where it is now); a
one-lane road (the lattice still leaves the road); a longer box (a parked car) and two in
a row; `lane_map:=` with `LaneCritic` driven; the car.

## Far goals in steps (16 Nov, TODO 175)

With the global costmap made of chunks ([own-costmap.md](own-costmap.md#from-map-chunks-on-every-core-16-nov-todo-175-195)),
a far goal can be off the costmap. The route level still covers the whole way; the path
goes as far as the costmap does, and the next replan (once a second) goes on.

```text
  ┌──────────── the costmap: the chunks held ─────────────┐
  │ car ════ the lane line (the route level) ═══════ ✕ ◀──┼── 10 m in from the edge
  └───────────────────────────────────────────────────────┘ ═════ ▶ goal
```

| Case | The path |
|---|---|
| goal on the costmap (even near its edge: the test lot's are) | as before |
| goal off it, on the lane map | the lane line, cut where it first comes within 10 m of the edge (`steps::last_inside`) |
| goal off it, off the lane map | the lattice to the farthest free point on the straight line (`steps::toward`) |
| every plan | the whole way on `/drivers/nav/planner/route`: the costmap holds the chunks along it |

The status says `cut: true`. `cargo run --release -p lattice-planner --example steps`
plans the campus and University Drive tours on the chunks, no ROS: every leg's goal was
inside the window (5 x 5 chunks, 320 m), so none was cut.

**A loop round the block** (found with the chunks, 16 Nov): 10 m short of the campus
tour's first goal (off the road, at 45° to it) a replan gave a 430 m route of lanes, and the
car drove it. A lane route longer than 2 x the straight distance + 40 m is now the
lattice's (`Fallback::Detour`, `detour` in the status). It could happen on the whole map
too: there the leg ended first.

## Routing on Lanelet2 (`lane_graph:=lanelet2`, 16 Nov, TODO 175)

The route level of the [three levels](designs/2026-11-15_three-levels.md), step 6. The
lane-centre planner can route on the campus's **Lanelet2** map instead of `lanes.yaml`.
The output is the same (`/plan`, `/drivers/nav/planner/route`): a drop-in.

```text
 lanes.yaml:  road centre lines ──Dijkstra──▶ moved right onto the lane ──▶ the line
 Lanelet2:    lanes (lanelets) ──Dijkstra───────────────────────────────▶ each lane's own centre
              + who follows whom, who sits beside (dashed line), one way, stop lines
```

| Part | What |
|---|---|
| The map | `maps/campus/campus.osm` (`pixi run campus`, `tools/campus_lanelet.py`): the street router's 517 roads, 998 lanes, 1,903 joins, and 98 stop lines from OpenStreetMap's stop signs |
| Where a pose meets it | the lanes within 20 m. On a lane: those running the way it faces. Off every lane: any of the 12 nearest, the way to it counted twice (so no loop round a block to reach the lane facing its way) |
| The route | `lanelet-map` `route_between`: Dijkstra over lanelets, never backwards; a change of lane where the car is, 10 m extra |
| The line | the lanes' centres joined; a lane change eased over 20 m; corners rounded as before (circle arcs; `lane_corners:=clothoid` for clothoids, TODO 211) |
| The stop lines | those on the route, on `/drivers/nav/planner/route_stops` (PoseArray, map) and `stops` in the status. Nothing reads them yet: next, the road ahead |
| Off the map | as before: the lattice, and why |
| Pick it | `lane_graph:=lanelet2` (`path:=lane_lanelet2` in `stacks.yaml`); a world with no `.osm` beside its `lanes.yaml` keeps the roads |

**Against `lanes.yaml`** (`cargo test -p lattice-planner --test lanelet2`, the campus's
6 legs, no ROS):

| Legs | Lanelet2 | lanes.yaml |
|---|---|---|
| on University Drive (2) | 134.4 m, 151.5 m | 134.4 m, 151.7 m; never more than 0.3 m apart |
| out of lot P4 (2) | none: off the map, the lattice | the same |
| the campus's 2 inner legs | 313 m, 245 m: service roads lanes.yaml hasn't got. Both over the detour limit, so the lattice, as with lanes.yaml | none: the lattice |

Lanelet2 has more roads (all 517 the router drives; `lanes.yaml` has the 129 painted
ones), so it finds lane routes where `lanes.yaml` finds none.

**Driving** (16 Nov, our simulator, `measure controllers --stacks own_calm_chunks
own_calm_chunks_ll2 --reversing --runs 2`, `/tmp/maps/b2b`, `b2c`):

| Tour | lanes.yaml (the default) | Lanelet2 |
|---|---|---|
| campus | 6/6 legs, 533-562 s, sway 0.088 | 6/6 legs, **717 s**, sway 0.092 |
| University Drive | 6/6 legs, 343 s, sway 0.056 | 6/6 legs, 389 s, sway 0.062 |

So **Lanelet2 stays a challenger**: every leg reached, but slower. On the campus its
replans take the service roads round blocks (fixed in part: an off-lane start takes any
lane near, a loop limit of 1.5 x + 20 m). Next: stop lines into the road ahead (TODO 220),
clothoid corners for both graphs (TODO 211).

## What's missing

- **Hybrid-A*** (the forward-only default on the car): ported 14 Nov
  ([above](#hybrid-a-a-second-search-14-nov)); not yet run in a simulator.
- **`ComputePathThroughPoses`**: done 8 Nov (`/drivers/nav/planner/compute_path_through_poses`,
  a plan a leg, joined, as Nav2's planner_server does).
- **The navigator step**: done 8 Nov ([our own navigator](own-navigator.md)): it calls
  ours for both, and clears our global costmap.
- **On the car:** untested; the car refuses `planner:=own`.
