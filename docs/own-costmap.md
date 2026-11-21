# Our own costmap

Ben, 8 Nov: move off Nav2 to **our own versions, one piece at a time**, each beside Nav2
until it won. The order: the costmap, then a path planner in Rust, then the navigator.
This page is the first piece. Since 14 Nov ours is the simulator's default (`stack:=ours`).
Nav2 went on 17 Nov ([as it was](history/nav2-as-it-was.md)): ours is the only one, in the sim and on the car. The comparisons with Nav2's below are the record.

> **The short version**
> - **Switch:** `costmap:=own`. Package `aav_costmap` (Rust).
> - **What it is:** Nav2's costmap layers ported: the map, the sensors' marks with ray
>   clearing, the inflation round the car's outline. A local window and a global map.
> - **Same input as Nav2:** made from the same tuned `nav2.yaml` blocks.
> - **Same output as Nav2:** in the sim, stopped at the post, **every one of 40,000 cells
>   the same** as Nav2's local costmap (5 runs of 5).
> - **Lighter:** our two costmaps 4-5 % of a core; Nav2's local one alone 7-9 %.
> - **Scores:** with ours, MPPI went round the box 3 of 3. The lidar-obstacle leg: 2 of 3
>   at first; **3 of 3 once the recovery cleared ours too** ([scores](#the-scores-8-nov)).
>   Nav2's: 3 of 3 each.
> - **Fast:** 0.4 ms an update without ROS; about 0.5 ms in the sim.
> - **Big maps:** each costmap has its own thread, and the global one is worked out once
>   per map, so the campus never makes the local one late ([threads](#threads-and-bounds-14-nov)).
> - **With `tracker:=mppi_lag`**, MPPI reads ours instead of Nav2's.
> - **From map chunks** (16 Nov, on in `stack:=ours`): the global costmap holds only the
>   64 m chunks round the car and along the route, each inflated on its own core. Every
>   leg as before, a third of the memory ([chunks](#from-map-chunks-on-every-core-16-nov-todo-175-195)).
> - **The local one too** (16 Nov, on in `stack:=ours`): its map from the 3 x 3 chunks round
>   the car, not `/map`. Every leg as before, half the node's memory; nothing of ours reads
>   `/map` on the campus now.

## How it fits

```
/map ─────────────┐
/sensors/lidar/scan (and radar, cubes, cameras where they run)
odometry, /tf ────┤
                  ▼
              costmap ──▶ /drivers/nav/costmap/local   (+ _raw)  odom, 20 m window, 5 Hz
                      ──▶ /drivers/nav/costmap/global  (+ _raw)  map, the map's size, 1 Hz
                      ──▶ /drivers/nav/costmap/status
   local_raw ──▶ mppi_lag (tracker:=mppi_lag)      Nav2's costmaps keep running beside
```

| Topic | Type | For |
|---|---|---|
| `/drivers/nav/costmap/local` | OccupancyGrid (0-100) | screens, RViz |
| `/drivers/nav/costmap/local_raw` | nav2_msgs/Costmap (0-255) | `mppi_lag` |
| `/drivers/nav/costmap/global`, `global_raw` | the same, the whole map | our planner (`planner:=own`) |
| `/drivers/nav/costmap/status` | DiagnosticStatus | update time, what it waits for |
| `/drivers/nav/costmap/clear_entirely_local`, `_global` | service, Nav2's ClearEntireCostmap | the recovery: forget every mark |

**The recovery clears ours too.** When stuck, Nav2's behaviour trees call
`local_costmap/clear_entirely_local_costmap`. With `costmap:=own` and
`tracker:=mppi_lag`, the launch file points that call at our local costmap, the one MPPI
reads. The global clear stays Nav2's under Nav2's navigator; our navigator
(`navigator:=own`) clears ours ([page](own-navigator.md)).

## What it does, layer by layer

| Layer | What it does | Nav2's |
|---|---|---|
| static map | `/map`'s cells: 100 is lethal, unknown kept only in the global one | `StaticLayer` |
| obstacles | each scan point marks a lethal cell (within 12 m, 0.3-2 m high); each ray clears the cells it passes (to 15 m); the car's own cells are cleared | `ObstacleLayer` |
| inflation | cost falls away from each lethal cell: inscribed inside 0.25 m, then 252 × e^(-k(d - 0.25)) out to 2 m (local) or 3 m (global) | `InflationLayer` |

- **The rolling window** moves in whole cells, so a mark stays on the same ground.
- **Scans are placed where the car was** when the scan was taken (odometry kept 2 s).
- **One change from Nav2:** the inflation finds the nearest obstacle exactly (a distance
  transform), not by Nav2's brushfire. The costs are the same.
- **Not ported:** Nav2's 3D voxel layer, range sensors, filters. `nav2.yaml` uses none.
- **One addition (15 Nov): the radar's view clears.** See below.
- **A challenger (15 Nov): the lanes layer** (`lane_costs:=true`), off by default. See below.

## The radar forgets what has gone (15 Nov, TODO 193)

**The problem.** A box in the lane: the radar marks it, the car stops. The box is taken
away. The radar now returns **nothing** there, and in Nav2's rule only a ray to a point
clears. So the box stayed in the radar's layer for ever. MPPI saw a wall that wasn't
there and stood still until its progress check failed (10 s).

**The fix.** A point cloud source can say how wide its view is: `view_angle` (rad), on
the radar's block in `nav2.yaml`, 120° (the TI radar's, and the simulator's stand-in).

| Where the radar sees... | Ours does |
|---|---|
| a point | marks it, clears the ray up to it (as Nav2) |
| nothing, inside its view | clears the ray out to `raytrace_max_range` (8.4 m), as a scan's inf does |
| outside its view (beside, behind) | nothing: old marks stay |

- One ray every 0.5° (`obstacle::view_points`): 7 cm apart at 8.4 m.
- The rays end past `obstacle_max_range`, so they never mark.
- Only the radar's own layer: the lidar's and the cameras' marks are never touched.
- Nav2's costmap had no such setting.
- A real radar's sparse frames: a thing it misses in both frames of 0.2 s is cleared, and
  marked again at the next frame that sees it. the stop zone still reads the radar
  itself, not the costmap.

## The lanes layer (15 Nov, TODO 148, a challenger)

**The idea** (D2 in the [one-shape design](designs/2026-11-13_one-shape.md)): the lane
map's roads drawn into the global costmap as a soft cost, so the planner keeps right.
Ben, 13 Nov: lane lines "it can cross but only if it needs to and to stay on the right side".

```text
 cost
 200 ┤█                                    █   kerb, and 1 m past it
 150 ┤ ████████████████                        the oncoming lane
  80 ┤                 █         █             the lane's own lines
   0 ┤                    ●                    the right lane's centre
     └── left kerb ── centre line ── edge line ── right kerb ──
```

| Part | What |
|---|---|
| Turn it on | `pixi run sim lane_costs:=true` (needs `costmap:=own`; ours has it). Off by default |
| The map | `lane_map:=`'s `lanes.yaml`, else the world's own (the campus's, the road world's). The test lot has none: nothing drawn |
| Which lane is right | the newest `/plan`: each road segment it runs along. Until then a two-way road's two lane centres are both 0 |
| How it adds | **added** to the inflation, up to 252: never a wall. Obstacles and unknown cells stay |
| Where | the global costmap only (the planner's). The local one, and so MPPI, don't see it |
| Code | `crates/costmap/src/lanes.rs` (no ROS, 9 tests); `lane_map.costmap_block`; `navigation.launch.py lane_costs:=` |

**Why added, not the higher of the two.** On the campus the lidar marks both kerbs. The
inflation from them is lowest in the road's **middle**: 59 at the right lane's centre
(2 m from the kerb), about 10 at the centre line. With Nav2's rule (the higher of the
two) the centre line (50) was cheaper than the lane's centre (59): the first runs kept to
the middle. Added, the lane's centre is the cheapest place across the road (a test checks).

### The scores (15 Nov, our simulator, the campus)

`pixi run measure controllers --stacks own --tours campus --runs 2 --launch behaviour:=true`
with nothing more, `lane_costs:=true`, or `lane_map:=world` (the road ahead on the lane's
centre and mppi_lag's LaneCritic: today's nearest to `path:=lane`, which isn't built).
Scored by `tools/lane_score.py` on the one leg every run drove: "along the road".

| Run | Lanes | Reached | Time (s) | Sway (m/s²) | Off the lane's centre (m) | Over the centre line | Crossings |
|---|---|---|---|---|---|---|---|
| A1 | none | **no** (watchdog) | 221 | 0.12 | 0.93 | 0 % | 0 |
| A2 | none | yes | 119 | 0.22 | 1.91 | 75 % | 4 |
| B1 | ramp, the higher of two | yes | 121 | 0.26 | 1.27 | 25 % | 2 |
| B2 | ramp, the higher of two | yes | 127 | 0.23 | 1.37 | 30 % | 2 |
| B3 | ramp, **added** (kept) | yes | 122 | 0.26 | 1.10 | 27 % | 2 |
| B4 | ramp, added | yes | 143 | 0.23 | 0.51 | 2 % | 1 |
| C1 | lane line (`lane_map:=world`) | **no** (follower) | 221 | 0.23 | 1.17 | 30 % | 3 |
| C2 | lane line | **no** (no path) | - | - | - | - | - |

| | None | Ramp, added | Lane line |
|---|---|---|---|
| Reached "along the road" | 1 of 2 | 2 of 2 | 0 of 2 |
| Over the centre line | 75 % | 2-27 % | 30 % |
| Time | 119 s | 122-143 s (B4 took a 48 m longer way) | - |
| Sway | 0.22 | 0.23-0.26 | 0.23 |

**The verdict: a challenger, not in `ours`.** It keeps right more (the time over the
centre line fell from 75 % to 2-27 %), but on 2 runs, 30 s of mapped road each, with no
gain in time or sway. A third ramp run (the added one) is left out: three sims at once
starved it, and the car never moved. The test lot has no lane map: the layer draws
nothing there (1 run, 4 of 4 legs, as without).

**Read with care.**

- Only University Drive is in the lane map on this route: about 30 s of each 120 s leg.
  The rest is roads OpenStreetMap doesn't paint.
- "Over the centre line" counts a left turn across it too.
- The runs are noisy: legs failed for reasons the lanes don't touch (the sensor
  watchdog, "the car doesn't fit at the start", the follower giving up).

**16 Nov: against `path:=lane`, on more road.** A new tour, `university` (about 290 m of
University Drive), 3 runs each: the ramp kept the car 0.59 m off the lane's centre and
over the centre line 1 % of the time (nothing: 1.51 m, 68 %); the lane-centre planner
0.10 m and 0 %, with less sway (0.063 against 0.087 m/s²), the same time a leg. The
table: [own-planner.md](own-planner.md#the-scores-16-nov-our-simulator-the-university-drive-tour).
Later that day, 3 clean runs each: `path:=lane` no worse on any column, and since then
`ours`' path; the ramp stays a challenger (`lanes:=ramp`; TODO 148).

## The lanes from the camera (19 Nov, TODO 264)

Ben, 19 Nov: "We use the perception system to identify the lines and then we bake it into
the cost map. So curb, high cost, yllow line med cost, lright side low cost, car in the
way high cost, person in the way high cost, etc then have the nav system handle it".

Where there is no lane map, the lane finder's two lines draw **the same costs** as the
lane map's roads: one function, `lanes::across`, prices both.

```text
   y (left)
   off_road ┤░░░░░░░░░░░░░░░░░░  assumed: an oncoming lane as wide as ours
       kerb ┤══════════════════
 wrong_lane ┤                     the oncoming lane: medium (150)
       line ┤── ── ── ── ── ──    the left line: the centre line (yellow, or by default)
          0 ┤ ●── the car's lane ──►  x (ahead): 2 m behind to 16 m ahead
       line ┤──────────────────   the right line (white): the edge line
       kerb ┤══════════════════   0.5 m past it: high (200)
   off_road ┤░░░░░░░░░░░░░░░░░░   1 m, then nothing
```

| Part | What |
|---|---|
| Turn it on | `pixi run sim lanes:=camera` (`lane_costs:=true lane_source:=camera`; it starts the lane finder). `lanes:=both`: the lane map wins where it has a road, the camera elsewhere |
| The lines | `/perception/lane/geometry`: each line detected at 0.3 confidence or more, placed on the map by the odometry and map -> odom at the picture's stamp; forgotten after 0.5 s |
| Which side is right | the car's own lane is the one it is in: its left line is the road's centre line. A **yellow right line** says the car is in the oncoming lane: its lane is then right of that line. One white line alone (the edge line): the centre is a lane to its left |
| The costs | `lanes.rs`'s ramp: 0 at the lane's centre, 80 at its lines, 150 over the centre line, 200 at the kerb and 1 m past it. Added to the inflation, at most 252: **never a wall**. Cars and people stay walls (the obstacle layers) |
| Code | `crates/costmap/src/camera_lanes.rs` (no ROS, 8 tests); `layered.rs` (`set_camera_lanes`, `see_lanes`; the map and camera grids merged per cell, the map first); the node's `on_lanes`; `lane_map.lanes_block` |
| Scores | not yet: stage 2 scores it on the same roads as the lane followers ([their numbers](history/lane-followers.md)) |

## Threads and bounds (14 Nov)

**The problem.** On the campus map (6400 × 6400 cells, 41 million) every leg with
`costmap:=own tracker:=mppi_lag` failed: "the costmap on .../local_raw: 0.84 s old
(limit 0.80 s)". Both costmaps ran on one thread, and the global one rebuilt the whole
map each second. While it did, the local one waited.

**The fix, Nav2's way:**

| Nav2 does | Ours now |
|---|---|
| each costmap has its own update thread (`mapUpdateLoop`) | each costmap has its own thread; the ROS worker only gathers the inputs and hands them over |
| each update redoes only the bounds the layers touched (`updateBounds`, `updateCosts`) | the global one is worked out **once per map**; after that only the cells the sensors touched, plus the inflation's reach |
| a late update loop is skipped and logged | a busy thread skips to the newest inputs (`skipped` on the status) |

```
ROS worker: messages, timers ──▶ local thread:  update + publish (5 Hz, 0.4 ms)
                             └─▶ global thread: map once (0.5 s) + publish (1 Hz)
```

**Before and after**, without ROS (`examples/timing.rs`, campus size, this Mac with the
simulator running beside it):

| What | Before | After |
|---|---|---|
| global update, every second | **477-539 ms** | 0.000 ms (nothing changed) |
| global update when the map comes | the same | 477 ms, once, on its own thread |
| global with a lidar layer on it | the whole map again | 0.25 ms (the scan's area) |
| making the two messages' cells | 26 ms each publish | only after a change |
| local waits for the global | yes | **never** |

- **Still sent in full, once a second:** the global grid (41 MB as 0-255, 41 MB as
  0-100). Our planner checks its age (2 s limit), so it must keep coming. It goes out
  on the global thread, so the local one never waits for it. Not timed with ROS yet.
- **The same cells:** a test checks an area update against a whole one, cell for cell.
- **The local one** still rebuilds its whole 200 × 200 window: it moves every update,
  and that takes 0.4 ms.
- **New status values:** `publish_ms` (the last publish) and `skipped`.

## From map chunks, on every core (16 Nov, TODO 175, 195)

Ben, 16 Nov: "why are map chunks and obstical avoidance not using rayon ... we should load
the closest ones or atleas the ones on the path. Lets turn on chunking for sure."

**The idea.** The global costmap holds only the 64 m chunks it needs, not the whole map.
The chunked maps are called **RavenTiles** (Ben, 16 Nov).

```text
 car + the route ahead ─▶ map_chunks::wanted: the 3 x 3 round the car + 100 m of route
                          ─▶ the rectangle round them, one chunk more each side
   ┌────┬────┬────┬────┬────┐
   │kept│kept│kept│new │new │   new chunks: read from fine/i_j.pgm, each on its own core
   ├────┼────┼────┼────┼────┤   kept chunks: their cells kept as they were
   │kept│kept│CAR │redo│new │   redo: a chunk whose neighbour came or went
   └────┴────┴────┴────┴────┘   each chunk inflated on its own core (rayon), then stitched
```

| Part | What |
|---|---|
| `costmap_chunks:=auto` (`sim.launch.py`) | the campus's chunks (`maps/campus/chunks/`, made by `pixi run campus`); the lot and the road world have none, so the whole map |
| `chunked.rs` | the window (`Window`), which chunks to redo, reading a chunk (`ChunkSource`), stitching |
| `layered.rs` | `set_chunks`, `follow_chunks` (each update), `set_ahead` (the route) |
| the route ahead | our planner's `/drivers/nav/planner/route`: the lane line to the goal, however far |
| the same cells | a chunk's costs need only its 8 neighbours (the 3 m inflation is less than a chunk) |

**Every fixed costmap uses the cores now**, chunks or not: the grid is worked out in
64 m tiles, each on its own core, each reading its tile plus the inflation's reach.

### How long it takes (this Mac, M4 Max, 16 cores)

`cargo run --release -p costmap --example chunks` on the campus's chunks; criterion for
the whole map (`cargo bench -p costmap`).

| Step | 1 core | 16 cores (rayon) |
|---|---|---|
| read one chunk (256 x 256 cells) | 0.11 ms | 0.07 ms |
| stitch one chunk | 0.001 ms | 0.001 ms |
| first window, 25 chunks: read and stitch | 1.9 ms | 0.6 ms |
| first window: inflate | 12-13 ms (0.5 ms a chunk) | **1.4 ms** |
| the car one chunk on: read 5, redo 10-15 | 8 ms | **1.3 ms** |
| the whole campus map (41 M cells), worked out | 326 ms | **32 ms** |

**The same cells:** the window against the whole campus map's costmap, away from its
edge: 0 cells differ. Tests: a 320 m map in chunks, cell for cell against the whole
map's costmap, as the car drives (with and without a lidar layer); tiles against one pass.

### The scores

Our simulator, 16 Nov, `measure controllers --stacks own_calm own_calm_chunks --tours lot
campus university --runs 2 --reversing --launch behaviour:=true lane_route:=true` (as
`stack:=ours` drives; results in /tmp/r2, not in git). Load 7-10 at the start, this Mac only.
The lot has no chunks, so both are the whole map there: its rows show the noise.

| Tour | Costmap | Legs | Time (s) | Sway (m/s²) | Off the path (m), mean / worst | Plan (ms, median) | Costmap: CPU, memory | Planner: CPU, memory |
|---|---|---|---|---|---|---|---|---|
| lot | whole map | 8 of 8 | 320 | 0.022 | 0.16 / 0.83 | 2 | 4 %, 37 MB | 4 %, 41 MB |
| lot | (chunks: none here) | 8 of 8 | 331 | 0.019 | 0.14 / 0.47 | 2 | 3 %, 36 MB | 4 %, 48 MB |
| campus | whole map | 6 of 6 | 557 | 0.087 | 0.16 / 1.87 | 28 | 12 %, 474 MB | 10 %, 403 MB |
| campus | **chunks** | 6 of 6 | 574 | 0.090 | 0.16 / 1.78 | **21** | **4 %, 145 MB** | **5 %, 65 MB** |
| University Drive | whole map | 6 of 6 | 344 | 0.065 | 0.10 / 0.99 | 0 (the lane) | 12 %, 470 MB | 10 %, 242 MB |
| University Drive | **chunks** | 6 of 6 | 342 | 0.060 | 0.09 / 0.57 | 0 (the lane) | **4 %, 135 MB** | **5 %, 60 MB** |

| What it says | So |
|---|---|
| every leg reached, either way | the far goals in steps work; no goal was cut (each was inside the 5 x 5 window) |
| the campus 17 s slower | run 1 the same to a second each leg; run 2 slower (41 against 31 s on the first leg): the whole map's run 2 was slower than its run 1 too. Noise |
| a third of the memory and CPU | no 41 M-cell grid, and no 41 MB message once a second |
| the global costmap's moves | 11-12 a tour; each 0.2-14 ms (read, stitch), 5-15 chunks redone |

**On since 16 Nov in `stack:=ours`** (`costmap:=ours_chunks`, `stacks.yaml`). Nav2's
costmap and `costmap:=ours` keep the whole map.

**Found on the way**, both fixed: a goal near the map's edge (the test lot's) was cut
short of; and a 430 m loop of lanes round the block to a goal 10 m off
([own-planner.md](own-planner.md#far-goals-in-steps-16-nov-todo-175)).

### The local costmap from the chunks too (`local_chunks:=true`, 16 Nov)

The local costmap (20 x 20 m, 0.1 m cells, in odom) still took its static layer from
`/map`: the node held the whole 41 M-cell campus map for a 20 m window.

```text
 car in map (map -> odom) ─▶ its chunk ─▶ the 3 x 3 round it (192 m square, 0.6 M cells)
                                           │ read on a chunk change (each on its own core)
                                           ▼
                 the 20 m window, each 0.1 m cell looked up in them (as from /map)
```

| Part | What |
|---|---|
| `layered.rs` | a rolling costmap may now `set_chunks`; `follow_chunks` keeps the 3 x 3 as its static map; the window stamps from it as it did from `/map` |
| the node | the car's place in map from map -> odom; no `/map` subscription when every costmap is made of chunks |
| the same cells | a test drives a window across chunk edges, odom turned and moved off map: cell for cell the window `/map` gives |
| Pick it | `local_chunks:=true` with `costmap_chunks:=` (`costmap:=ours_all_chunks` in `stacks.yaml`) |

**The scores.** Our simulator, 16 Nov, `measure controllers --stacks own_calm_chunks
own_calm_chunks_local --tours campus university_box --runs 2 --reversing --launch
behaviour:=true lane_route:=true` (results in /tmp/maps/b2a, not in git). Load under 8 at
the start, no other simulator running.

| Tour | Local costmap from | Legs | Time (s) | Sway (m/s²) | Off the path (m), mean / worst | Costmap node: CPU, memory |
|---|---|---|---|---|---|---|
| campus | `/map` | 6 of 6 | 562 | 0.088 | 0.20 / 1.90 | 4-5 %, 131-148 MB |
| campus | **the chunks** | 6 of 6 | 533 | 0.089 | 0.17 / 1.67 | 4 %, **66-81 MB** |
| University Drive, a parked box | `/map` | 2 of 2 | 85 | 0.060 | 0.06 / 0.34 | 4 %, 138-140 MB |
| University Drive, a parked box | **the chunks** | 2 of 2 | 87 | 0.065 | 0.07 / 0.34 | 4 %, **75-78 MB** |

| What it says | So |
|---|---|
| every leg, the same times and sway | the same cells, as the test says |
| half the node's memory | the 41 M-cell map (and its message) is gone; the 3 x 3 is 0.6 M cells |
| a chunk change | read in 0.4-1.2 ms, every 10-30 s of driving |

**On since 16 Nov in `stack:=ours`** (`costmap:=ours_all_chunks`). `ours_chunks` (the local
one from `/map`) stays a challenger.

**Who still reads `/map` in `ours` on the campus.** Checked while driving (`ros2 topic info
/map -v`): `map_server` publishes it, **nobody subscribes**. It is still read by:

| Reader | When |
|---|---|
| `aav_things_rs` (the background: walls aren't things) | `things:=true` |
| the screens: RViz, the map window, the dev window, Foxglove's cockpit | when they run |
| AMCL | `amcl:=true`, without its own map |
| `sim_check.py` | its checks |

Next: give those the chunks too, then `map_server` can stop on the campus.

## Try it

| To | Run |
|---|---|
| See it | `pixi run sim costmap:=own tracker:=mppi_lag`, then show `/drivers/nav/costmap/local` |
| Score it | `pixi run sim-check --legs-only --costmap own --tracker mppi_lag --lidar-obstacle --plan-around` |
| The same with Nav2's | the same, `--costmap nav2` |
| Test the maths | `cd ros/drivers/aav_costmap && cargo test -p costmap` |
| Time it | `cargo run --release -p costmap --example timing` |

## The scores (8 Nov)

Our simulator, the road world, `tracker:=mppi_lag`, 3 runs each, on a quiet Mac (M4 Max).

- **Lidar obstacle:** a post 15 m ahead in the lane, radar off, the planner blind to it.
  The car must stop. Then the post goes and it must drive on.
- **Plan around:** a 1 m box in the lane; the planner sees it. The car must go round.

### Lidar obstacle: where it stopped

| Run | Nav2's costmap | Ours |
|---|---|---|
| 1 | stopped 0.71 m short; drove on to the goal | stopped 0.20 m short; drove on to the goal |
| 2 | 0.17 m; to the goal | 0.21 m; **did not reach the goal**: after the post went it drove 6 m on, then "no progress" until Nav2 gave up |
| 3 | 0.26 m; to the goal | **did not stop**: it steered past the post (2.5 m beyond its near side); then to the goal |

- In every run the **stop zone** (collision_monitor) stopped the car first, both costmaps.
- Run 2: when stuck, Nav2's behaviour tree **cleared Nav2's costmap**, not ours, so a
  stale mark in ours could stay. Not proved, but fixed: the recovery now clears ours too.
- Run 3: MPPI is random; with Nav2's costmap it never steered round the post (0 of 3).

**Again, with the clear service** (3 more runs, ours):

| Run | Ours, with the recovery clearing it |
|---|---|
| 4 | stopped 0.64 m short; drove on to the goal (the recovery cleared ours once) |
| 5 | 0.15 m; to the goal (cleared 3 times) |
| 6 | 0.18 m; to the goal (cleared 5 times) |

3 of 3 reached the goal, all stopped by the stop zone first, and the grid was the same as
Nav2's in every cell each time. Three runs can't prove run 2's stall is gone for good.

### Plan around: did it go round?

| Run | Nav2's costmap | Ours |
|---|---|---|
| 1 | round, **2.43 m** clear | round, **2.29 m** clear |
| 2 | 2.35 m | 2.48 m |
| 3 | 2.42 m | 2.11 m |

All six reached the goal 30 m on, never stood still, and the stop zone never fired.

### The same grid?

While the car stood at the post, the probe took Nav2's local costmap and ours at once:

| Run | Cells the same | Lethal cells (Nav2, ours, both) |
|---|---|---|
| 1 | **100.0 %** of 40,000 | 6, 6, 6 |
| 2 | **100.0 %** of 40,000 | 6, 6, 6 |
| 3 | 94.6 % (the car was moving: the two updates came from different scans) | 15, 16, 10 |

### CPU

| Process | Holds | CPU (of one core) |
|---|---|---|
| `controller_server` | Nav2's local costmap (with `mppi_lag` it does nothing else) | 7-9 % |
| `planner_server` | Nav2's global costmap and the planner | 6-8 % |
| `costmap` (ours) | our local **and** global costmaps | **4-5 %** |

Our local update: **0.50-0.55 ms** on average, 0.58 ms at the 95th percentile.

### The verdict

- **The grid is right:** cell for cell the same as Nav2's when both see the same scan.
- **It is lighter:** both of ours together use about half the CPU of Nav2's local one.
- **As good on these legs, once it can be cleared:** 3 of 3 since the clear service, as
  Nav2's. Not yet a winner: six runs a leg is little, it went past the post once (run 3),
  and the radar and cameras are untested. (Nav2 then stayed the default.)

## Without ROS

`cargo test -p costmap`: 52 tests.

| Test | Checks |
|---|---|
| a post ahead | lethal at the post, inscribed within 0.25 m, Nav2's cost 1 m out (26) |
| the post taken away | the next scan's rays clear it |
| clearing | the recovery forgets a mark no ray reaches |
| the window rolls | a mark stays on the same ground as the car drives on |
| the map through map → odom | a wall lands in the right cells |
| the global costmap | the map's own cells, unknown kept, 3 m of gentle inflation (119 at 1 m) |
| the distance transform | exact against brute force |
| an area update | the same cells as a whole one, as the car drives with a lidar layer |
| a map without sensors | worked out once, again only after a clear |
| the lanes layer | lowest at the right lane's centre, the other way swaps the lanes, both low with no route, a route across a road says nothing, an area drawn as the whole, the lane's centre cheapest between two inflated kerbs, a new route's area update the same as a whole one |

## What's missing

- **The planner** reads Nav2's global costmap (`planner_server`). Our planner
  ([page](own-planner.md), `planner:=own`) reads `/drivers/nav/costmap/global_raw` when
  `costmap:=own` is on too.
- **The global costmap's size:** fixed 14 Nov ([threads](#threads-and-bounds-14-nov));
  with chunks (16 Nov) only 5 x 5 or so chunks (1.6 M cells). Sending it in full each
  second is left; partial updates (Nav2's `_updates`) only if that proves slow.
- **Coarse chunks** (1 m far away): the planner's lattice is fixed to 0.25 m cells, so
  every chunk held is the fine one.
- **The local costmap from chunks:** done 16 Nov ([above](#the-local-costmap-from-the-chunks-too-local_chunkstrue-16-nov)); `/map` is now
  read in `ours` only by the things' background and the screens.
- **The global clear:** ours serves it. Our navigator calls it (8 Nov); Nav2's trees
  still clear Nav2's global costmap.
- **Radar, cubes and cameras** (point clouds, placed by `/tf_static`'s mounts): built,
  but untested; the sim's runs had the lidar's scan alone.
- **On the car:** untested; ours is the car's costmap since 17 Nov.
