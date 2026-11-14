# Our own costmap

Ben, 8 Nov: move off Nav2 to **our own versions, one piece at a time**, each beside Nav2
until it wins. The order: the costmap, then a path planner in Rust, then the navigator.
This page is the first piece. Nav2 stays the default, and stays on the car.

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
> - **With `tracker:=mppi_lag`**, MPPI reads ours instead of Nav2's.

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
  and the radar and cameras are untested. Nav2 stays the default.

## Without ROS

`cargo test -p costmap`: 31 tests.

| Test | Checks |
|---|---|
| a post ahead | lethal at the post, inscribed within 0.25 m, Nav2's cost 1 m out (26) |
| the post taken away | the next scan's rays clear it |
| clearing | the recovery forgets a mark no ray reaches |
| the window rolls | a mark stays on the same ground as the car drives on |
| the map through map → odom | a wall lands in the right cells |
| the global costmap | the map's own cells, unknown kept, 3 m of gentle inflation (119 at 1 m) |
| the distance transform | exact against brute force |

## What's missing

- **The planner** reads Nav2's global costmap (`planner_server`). Our planner
  ([page](own-planner.md), `planner:=own`) reads `/drivers/nav/costmap/global_raw` when
  `costmap:=own` is on too.
- **The global costmap's size:** the campus map is large; ours rebuilds it all each second
  (27 ms for 500 m × 500 m without ROS). Fine now; only the changed part if it grows.
- **The global clear:** ours serves it. Our navigator calls it (8 Nov); Nav2's trees
  still clear Nav2's global costmap.
- **Radar, cubes and cameras** (point clouds, placed by `/tf_static`'s mounts): built,
  but untested; the sim's runs had the lidar's scan alone.
- **On the car:** untested. Nav2 stays there.
