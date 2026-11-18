# Our own navigator

Ben, 8 Nov: move off Nav2 **one piece at a time**, each beside Nav2 until it won. The
order: the costmap ([done](own-costmap.md)), the planner ([done](own-planner.md)), then
**the navigator** (this page), the last piece. Since 14 Nov ours is the simulator's
default (`stack:=ours`).
Nav2 went on 17 Nov ([as it was](history/nav2-as-it-was.md)): ours is the only one, in the sim and on the car. The comparisons with Nav2's below are the record.

> **The short version**
> - **Switch:** `navigator:=own`. Package `aav_navigator` (Rust).
> - **What it is:** what Nav2's `bt_navigator`, our behaviour trees for it and
>   `waypoint_follower` do for the car, as one state machine.
> - **Drop-in:** Nav2's four actions, under Nav2's own names (`/navigate_to_pose`...),
>   in `bt_navigator`'s and `waypoint_follower`'s place. No goal sender had to change.
> - **All ours:** `navigator:=own planner:=own costmap:=own tracker:=mppi_lag`. Then no
>   Nav2 server navigates.
> - **Scores:** test lot 7 of 12 legs (Nav2 8); the box 3 of 3 (Nav2 3); campus 4 of 6
>   (Nav2 2); P4 to the Fieldhouse 1 of 2 (Nav2 0). The sim's default since 14 Nov
>   (Ben's choice, `stack:=ours`), not the car's.

## How it fits

```
goal senders ──▶ navigator (ours) ──ComputePathToPose──────────▶ planner (ours or Nav2's)
 RViz, map,          │            ──ComputePathThroughPoses────▶
 street_router,      │            ──FollowPath─────────────────▶ mppi_lag (or a tracker)
 language_drive,     │            ──ClearEntireCostmap─────────▶ costmaps (ours or Nav2's)
 the tools           └──────────── /drivers/nav/navigator/status
                                   mppi_lag ─▶ velocity smoother ─▶ stop zone ─▶ twist_mux
```

The safety locks, the stop zone and twist_mux never change.

| Action (Nav2's name, kept) | Nav2's server it replaced | Used by |
|---|---|---|
| `/navigate_to_pose` | `bt_navigator` | the tools, sim-check, RViz and the screens (`/goal_pose`) |
| `/navigate_through_poses` | `bt_navigator` | anyone with several poses |
| `/follow_waypoints` | `waypoint_follower` | map waypoints |
| `/follow_gps_waypoints` | `waypoint_follower` | street_router, language_drive |

**Who sends to which.** Nobody chooses (since 14 Nov). With `navigator:=own`, Nav2's
two servers don't start, and ours serves their names. So `street_router`,
`language_drive`, `sim-check`, `measure controllers`, `bench-check` and the screens
send to the same place whichever runs. `/goal_pose` reaches ours too.
`vehicle.navigator_action(name)` gives the full name.

## What it does

```
goal ─▶ plan ─▶ follow ─▶ plan again: at most every second, but keep
          │        │        a clear path up to 10 s (near the goal: 20 s, TODO 17)
          ▼        ▼
   clear global  clear local      ◀── a failure a recovery can fix
          └────────┴─▶ in turn: clear both ─▶ wait 5 s ─▶ back up 1 m (reversing only)
                       up to 6 times, then give up with Nav2's error code
```

| Step | Ours | Nav2's trees |
|---|---|---|
| plan | every 1 s (through-poses 3 s); a to-pose path kept up to 10 s while it ends at the goal and IsPathValid says it is clear (15 Nov, TODO 168: planning every second moved the path a few cm each time, and the car swayed twice as much) | the same (the trees' KeepAValidPath, 14 Nov) |
| the final approach | keep the path within 8 m of the goal, under 20 s old | the same |
| a failed replan, while driving a good path | keep driving; give up after 4 in a row | stops the car at once |
| through-poses | drop a pose within 1 m, **or once passed along the path** | within 1 m only |
| waypoints, GPS waypoints | **one through-poses goal**: no stop at each | stop and face each one |
| recoveries | clear, wait, back up, in turn, 6 at most | the same |
| lat/lon to the map | navsat_transform's datum (the same UTM steps) | calls `/fromLL` |
| a new goal | replaces the running one, on any action | refused on the other navigator |
| a goal in another frame (14 Nov) | `odom` and `base_link` put on the map by the latest /tf; any other refused | any frame tf2 knows, at the goal's stamp |
| a server that dies mid-goal (14 Nov) | time limits: no plan in 10 s is a timeout (recovered as one); a follower silent for 4 s ends the goal (107); a clear unanswered in 5 s failed | the lifecycle bond (4 s) takes the stack down |

Each change is in `ros/drivers/aav_navigator/PROVENANCE.md`.

**What all ours left running from Nav2, until 17 Nov:** `map_server` (the map), the
velocity smoother, and collision_monitor. Each is ours now: `aav_costmap`'s map server,
`aav_smoother`, `aav_stop_zone` ([Nav2, as it was](history/nav2-as-it-was.md)).

## Try it

| To | Run |
|---|---|
| See it | `pixi run sim navigator:=own planner:=own costmap:=own tracker:=mppi_lag` (reversing since 14 Nov; `reversing:=false` for forward only) |
| Score all ours against all Nav2 | `pixi run measure controllers --stacks nav2 own --tours lot obstacle --runs 3` |
| The campus route | `pixi run measure controllers --stacks nav2 own --tours fieldhouse --speed 2.5` |
| The graph and a goal | `pixi run sim-check --navigator own --planner own --costmap own --tracker mppi_lag` |
| Test the maths | `cd ros/drivers/aav_navigator && cargo test -p navigator` |

## Scores (8 Nov, our simulator)

All ours (`navigator:=own planner:=own costmap:=own tracker:=mppi_lag`) against all Nav2
(`bt_navigator`, the lattice, Nav2's costmaps, MPPI). Both forward only: on 8 Nov
`mppi_lag` couldn't reverse (it can since 14 Nov: [MPPI that knows the lag](mppi-lag.md#reversing)). `pixi run measure controllers --stacks nav2 own`.

### The test lot, 1 m/s, 3 runs

| Leg | All Nav2 | All ours |
|---|---|---|
| a left turn | 3 / 3 (26-55 s) | 3 / 3 (24 s) |
| round the building | 3 / 3 (46-47 s) | 2 / 3 (44-45 s) |
| turn back (a U-turn in a 14 m lane) | **0 / 3** | **1 / 3** |
| home | 2 / 3 (87-88 s) | 1 / 3 (85 s) |
| **legs reached** | **8 / 12** | **7 / 12** |
| off the path, mean | 0.12 m | 0.17 m |
| sway | 0.038 m/s² | 0.050 m/s² |
| the follower's CPU | 17-18 % | 33-39 % |

- **Forward only, the U-turn is the hard leg for both.** A car that ends it against a
  wall can't start the next leg ("start occupied"): Nav2 lost "home" that way once, ours
  twice (once already on "round the building").

### The box in the lane, 3 runs

| | All Nav2 | All ours |
|---|---|---|
| past the box | 3 / 3, 36-37 s | 3 / 3, 34 s |

### sim-check, all ours

`pixi run sim-check --navigator own --planner own --costmap own --tracker mppi_lag
--obstacle --plan-around`: **all 13 checks pass** (the graph against the contract, the
Nav2 goal past the box after the stop zone, the drive round the box).

### The campus, 2.5 m/s, 2 runs

| Leg | All Nav2 | All ours |
|---|---|---|
| out of the lot (47 m) | **2 / 2** (28-29 s) | **0 / 2**: past the goal, then round a block (251-266 m) |
| along the road (254 m) | 0 / 2: "Start occupied" | 2 / 2 (223-250 s) |
| round the bend (246 m) | 0 / 2: "Start occupied" | 2 / 2 (135-137 s) |
| **P4 to the Fieldhouse** (453 m, 18 GPS waypoints) | **0 / 2**: stuck at the lot exit after 45 m, 16 waypoints missed | **1 / 2** (263 s, 590 m driven); once stuck at the same exit |

- **Out of the lot:** `mppi_lag` misses the goal at 2.5 m/s. It does the same under
  Nav2's navigator (274 m, one run): the follower, not the navigator.
- **Ours gets past "Start occupied"** by clearing the global costmap and planning again.

### Why "along the road" failed (TODO 144's MPPI give-up)

| What | Seen |
|---|---|
| Where | the end of "out of the lot", **inside the lot's exit** (about 73, -38) |
| The static map there | clear: nothing within 2.4 m of the car |
| The global costmap there | **lidar marks** within 1.5 m of the car: only at the two lot exits (59, -44 and 72, -38), in 7 of 6,199 samples |
| Nav2's planner | "Start occupied" |
| Nav2's MPPI (with reversing) | "Optimizer fail": every future starts in collision |

The car stops on its goal in a narrow exit, with the kerbs the lidar marked touching its
padded outline. Every future and every plan starts "in collision", and Nav2 gives up at
once (it doesn't count "start occupied" as fixable). Ours clears the costmap and plans
again. A goal a few metres past the exit would avoid it.

### The verdict

- **8 Nov: not the sim default yet** (it is since 14 Nov, by Ben's choice). On the lot it is as good as Nav2 (7 against 8 of 12,
  inside the noise of the U-turn), on the box the same, on the campus better (4 against 2
  of 6; the route 1 against 0 of 2).
- **But:** `mppi_lag` overshoots the campus's first goal at 2.5 m/s, and uses twice
  Nav2's MPPI's CPU. Fix the follower's goal approach first, then run again.

## What's missing

- **The goal at speed:** `mppi_lag` passes the campus's first goal at 2.5 m/s.
- **Reversing:** since 14 Nov all ours reverses (`mppi_lag`, [page](mppi-lag.md#reversing));
  not yet scored against all Nav2 with `--reversing`.

- **On the car:** untested; `car.launch.py` runs ours (the only navigator since 17 Nov).
- **`IsPathValid`** on the final approach (14 Nov): once a second it asks the planner
  (ours, `aav_planner`); a blocked path is planned
  again. Not yet seen in a sim run.
- **Back up** calls Nav2's `behavior_server` where it runs (reversing only). All ours runs
  none: since 14 Nov the navigator then backs up through the follower, `mppi_lag`, on a
  straight line 1 m behind the car (it checks the costmap as it goes). Not yet seen in a
  sim run.
