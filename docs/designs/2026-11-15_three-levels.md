# Three levels, fixed chunks: the map the car plans on

* **Author:** Ben (the ask), written up 15 Nov 2026
* **Date:** 2026-11-15
* **Status:** Proposed; steps being built (the table below). TODO 175.
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
| 3 | **The middle planner on chunks**: our planner plans ~100 m ahead on the loaded chunks, then again as the car goes | 🔧 first form, `global_window:=192` (the global costmap rolls with the car, the 3 x 3 chunks): **scored 15 Nov, worse, stays a challenger**. Lot 8/8 either way (sway 0.022 against 0.021 m/s², 313 against 322 s); campus **2/6** against 6/6: a far goal's road leaves the window ("out of time or iterations"), or the goal is off it. It needs the next part first: a far goal planned via the lane route, in steps that fit |
| 4 | **NDT on chunks** | ✅ 15 Nov, `ndt-core` `chunks.rs`; the campus NDT map in 699 chunk files. Scored on the campus tour (3 runs against 2 on the whole map): as good (off 0.058 / 0.014 / 0.011 m a leg against 0.051 / 0.014 / 0.011), ready in 0.02 s against 2.5 s, a new 3 x 3 read in 0-3 ms (rayon) and built in 8-22 ms. Fixed on the way: the chunks follow a restart from `ekf_global`. **On by default** with `ndt:=true` on the campus once `pixi run campus` made them (`ndt_chunks:=off`: the whole map) |
| 5 | **The subtraction**: the things from new occupancies only | fewer things on a walled campus road; a person still found |
| 6 | **Lanelet2 in Rust** as the route level's map (TODO 175) | a route by lanes, stop lines in the road ahead |
| 7 | **The dev window** draws the chunks, the lanes, the static map | seen live |

Each step is a drop-in beside today's, with `sim-check` and `measure` against the same tours.

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
