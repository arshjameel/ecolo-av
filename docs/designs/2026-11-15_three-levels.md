# Three levels, fixed chunks: the map the car plans on

* **Author:** Ben (the ask), written up 15 Nov 2026
* **Date:** 2026-11-15
* **Status:** Proposed. Nothing here is built yet except what "Today" says. TODO 175.
* **Builds on:** [the whole car](2026-11-14_the-whole-car.md) (the road ahead, the things),
  [one shape](2026-11-13_one-shape.md) (the levels, the league).

Ben, 14 Nov: "I like having hte 3 levels with fixed size map chunks"; "Zoomed out is routes,
middle is how we get from cube to cube and finally close up is obstical avoidance"; "NDT can
give us the stationary things ... then all we need is occupancy graph to track poepole".
Ben, 15 Nov: "the whole 3 levels of nav ... the splitting of the map into quadtrees kind of
thing or octree but fixed size and maybe add voxels (ndt for static stuff and localization)
and then voxels for new occupancies."

## The idea in one line

Cut the world into **fixed 64 m squares (chunks)**. Load only the ones near the car and
along its route. Plan at three zoom levels, each on the chunks it needs.

## Why

| Today | The cost |
|---|---|
| The campus global costmap is one grid: 1600 x 1600 m at 0.25 m = **41 million cells** | memory, and every planner query searches one big map |
| The NDT map is one file: **12.3 million points**, all loaded | start-up time, memory on the Orin |
| Our planner plans the whole way to the goal on that grid | slow on long routes (TODO 152: 180 ms) |
| Everything the lidar sees is "a thing" | a wall seen again looks new; the tracker works harder |

## The picture

```text
 ZOOMED OUT  route planner        the street graph (Lanelet2 / lanes.yaml), A* or Dijkstra
             "which roads"        whole campus, a few hundred nodes, once per goal
                  │ the route: a line of roads
                  ▼
 MIDDLE      path planner         chunks along the route, ~100 m ahead
             "cube to cube"       ┌────┬────┬────┐
                                  │ 1 m│0.25│ 1 m│   near chunks fine, far chunks coarse
                                  ├────┼────┼────┤
                                  │ 1 m│ CAR│0.25│ ──► route
                                  └────┴────┴────┘
                  │ the path (/plan), and the road ahead's line
                  ▼
 CLOSE UP    local planner        the 20 x 20 m costmap at 0.1 m, from the live cubes
             "don't hit it"       MPPI; the stop zone underneath, always
```

The **behaviour planner** sits across all three: it reads the route (stop lines, right of
way), the path (the road ahead), and the close-up (the things).

## Chunks: fixed squares, not a quadtree

| Choice | Why |
|---|---|
| **Fixed 64 m squares**, on a grid from the map's origin; a chunk is `(i, j)` | finding a chunk is one division, no tree to walk or rebalance; each chunk can go to its own thread |
| **Two detail levels**, 1 m and 0.25 m | Ben's "further away less fine": what a quadtree's levels would give, with two fixed sizes |
| **Loaded**: the 3 x 3 chunks round the car, plus those the route crosses in the next 100 m | about 15 chunks of the campus's 625: 2 % |
| **3D inside a chunk**: a hash of 1 m voxels (NDT's cells) | an octree's job, flat: lookups are one hash; only cells with points exist |

A quadtree or an octree fits a map with big empty areas and a few dense ones. A campus is
roads everywhere, so the tree saves little and costs a walk on every lookup. If one level of
detail turns out too coarse, a chunk can split into four 32 m chunks: that is the quadtree
idea, used only where it pays.

## What each chunk holds

| Layer | Cells | From | Used by |
|---|---|---|---|
| **Static map** (NDT) | 1 m voxels: mean and spread of the points | a mapping drive (`ndt-map`), moving things left out | NDT localization; the background for the things |
| **Occupancy** | 1 m far, 0.25 m near | the static map flattened, plus OSM's buildings | the middle path planner |
| **Lanes** | the lanes' lines and stop lines in this chunk | Lanelet2 (or `lanes.yaml`) | the road ahead, behaviour |
| **Live cubes** | 0.25 m, this drive only | the lidar (`voxels`) | the close-up costmap; the things |

## Static and moving: the subtraction

```text
 live cubes ──► is a cube on the static map (± 1 cube for pose error)?
                   yes ─► the wall again: nothing new
                   no  ─► a NEW occupancy ─► grouped ─► a thing ─► tracked (aav_things)
```

**Safety never trusts the subtraction.** The stop zone and the close-up costmap see every
point, static or not. The subtraction only decides what the tracker follows.

## What stays the same (drop-ins)

| Stays | Why |
|---|---|
| `/plan`, the costmaps' topics, `/world/road_ahead`, `/world/things` | every reader keeps working; the league scores old and new the same way |
| The stop zone, the locks | never swapped (CLAUDE.md) |
| Today's whole-map planner and single NDT file | they stay as the champions until the chunked ones win |

New: `/maps/chunks/loaded` (which chunks are in memory, for the dev window and the tests).

## The order

| # | Step | Done when |
|---|---|---|
| 1 | **The chunk store** (Rust crate, no ROS): `(i, j)` from x, y; load and drop by distance; each chunk's layers | laptop tests: the right 15 chunks load along a campus route |
| 2 | **Chunk files**: `chunk-map` cuts a map server's map into one `.pgm` per chunk; then `pixi run campus` and `ndt-map` call it | ✅ occupancy, 15 Nov: the campus in 625 chunks per detail, 1.5 s; read back cell for cell. NDT next |
| 3 | **The middle planner on chunks**: our planner plans ~100 m ahead on the loaded chunks, then again as the car goes | 🔧 15 Nov, first form: `global_window:=192` rolls the global costmap with the car (the 3 x 3 chunks); the street router's 10-30 m legs fit. To score: campus tours, legs and time against the whole map. Later: a far goal planned via the lane route |
| 4 | **NDT on chunks** | 🔧 15 Nov, `ndt-core` `chunks.rs`: the campus NDT map in 699 chunk files; the 3 x 3 round the start holds 10,354 cubes of 890,407 (1.2 %), built in 0.01 s against 0.9 s, cube for cube the same. The node: `ndt_chunks:=` (off by default) loads them as the car drives; builds, not yet run in a simulator |
| 5 | **The subtraction**: the things from new occupancies only | ✅ 16 Nov, the default where `aav_things` runs: cubes under 0.7 m on `/map`'s walls drop. On the campus, driving: 1.7 things listed, not 9.9; every walker found. Below, "Step 5: the scores" |
| 6 | **Lanelet2 in Rust** as the route level's map (TODO 175) | a route by lanes, stop lines in the road ahead |
| 7 | **The dev window** draws the chunks, the lanes, the static map | 🔧 16 Nov: the route, the chunks loaded (`(i, j) fine` or `coarse`), each one's saved map, the lanes and stop lines (key H). Seen off screen only (the screen was locked); live: untested |

Each step is a drop-in beside today's, with `sim-check` and `measure` against the same tours.

### Step 5: the scores (16 Nov)

Our simulator, the campus from the start in lot P4, the car standing, then driving 42 m to
(75, -37). A post the size of a person (0.5 x 1.8 m) crosses at 1 m/s, from 5 m right of the
car, 8, 12 and 16 m ahead; twice more where the drive ends, starting on the grass. Load
average (1 min) at each start: 6 to 14, our own simulator only.

| Background | Runs | Things listed, driving (mean, max) | Things ever, driving | Walker in the lot | Walker from the grass |
|---|---|---|---|---|---|
| none (before) | 3 | 9.9, 33 | 70-76 | found, 100 % | found, 100 % |
| `/map`, every cube | 3 | 0.6, 6 | 3-9 | found, 100 % | **never found** |
| NDT chunks | 2 | 10.7, 33 | 69-76 | found, 100 % | found, 100 % |
| **`/map`, cubes under 0.7 m** (the default) | 2 | **1.7**, 15 | 12-21 | found, 100 % | found, 100 % |

"Found": listed within 0.2 s of the walker being placed (once 1.7 s, with no background);
"100 %": listed in every message while it walked. The walker from the grass ran in the
last 2 runs of each, not the first `/map` and "none" ones. The road world: 1 thing (the stop sign), the same with and
without; walkers found every time. With the new default, `sim-check --moving` (the road
world, a person crossing ahead while the car drives) passed: the behaviour went APPROACH,
WAIT, GO for them, 0.78 m clear, and the car reached its goal.

What it says:

| Finding | So |
|---|---|
| The campus's clutter is its kerbs and raised grass (0.3 m), seen as low cubes off the road | dropping only low cubes on the map's cells removes most of it |
| `/map` marks everything that isn't road as taken | a whole-body subtraction hides a person on the grass from the tracker |
| The campus's NDT map is a guess with walls, trunks and crowns, not kerbs or grass | against it, nothing changes; it would need the kerbs, or a real mapping drive's map |

**Safety never trusts it** (checked in the code, 16 Nov): `aav_things` drops the cubes only
in its own copy, before grouping; nothing republishes them. The stop zone
(`collision_monitor`, `aav_stop_zone`) and the close-up costmap (Nav2's or `aav_costmap`)
read `/sensors/lidar/scan` and `/perception/voxels/cubes` themselves (`nav2.yaml`), never
`/world/things`. Only the behaviour planner, the stop sign watcher and the screens read the
things.

Next: the NDT chunks with the kerbs (`campus_world.py`'s guessed map, or a mapping drive),
then whole-body subtraction against them; a person under 0.7 m on a mapped wall's cell
(a crouching child by a building) is still not a thing.

## Decisions for Ben

| # | Question | Proposed |
|---|---|---|
| C1 | Chunk size | 64 m (a city block; 3 x 3 covers the lidar's 30 m all round) |
| C2 | Detail levels | 1 m far, 0.25 m near (within one chunk of the car) |
| C3 | How far ahead the middle plans | 100 m along the route |
| C4 | Chunk files | one file per chunk and layer, named by `(i, j)` |
| C5 | Where the chunk store lives | a Rust crate shared by the planner, NDT, the things and the dev window |
| C6 | Order | the steps above; Lanelet2 (step 6) may start in parallel |

## Sources

| Source | Kind | Used for |
|---|---|---|
| Autoware's map loader (partial and differential point-cloud map loading) | idea | loading only the map near the car |
| Apollo's HD map tiles | idea | a map cut into fixed tiles |
| OctoMap: A. Hornung et al., Autonomous Robots, 2013 | idea | 3D occupancy; why we use a flat hash instead |
| Quadtrees: R. Finkel, J. Bentley, Acta Informatica, 1974 | idea | the split-where-it-pays fallback |
| NDT (Biber 2003; Magnusson 2009) | idea | the static map's cells |
| Lanelet2 (Poggenhans 2018) | idea | the route level's map |
