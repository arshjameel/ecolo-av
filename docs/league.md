# The league

One file says **what the car is**: each level's champion, and the challengers
waiting to beat it.

```text
  config/stacks.yaml (aav_description)
  ┌────────────┬───────────┬──────────────────────────────┐
  │ Level      │ Champion  │ Challengers                  │
  ├────────────┼───────────┼──────────────────────────────┤
  │ pose       │ ekf       │ ndt_ekf, kiss_ekf, ekf_lane, │
  │            │           │ kinematic_ekf                │
  │ costmap    │ ours      │ ours_chunks, ours_all_chunks │
  │ navigator  │ ours      │                              │
  │ path       │ lattice_rs│ lattice_rs_smooth, hybrid_rs,│
  │            │           │ lattice_rs_strong, lane,     │
  │            │           │ lane_lanelet2, diffusion,    │
  │            │           │ lane_jerk                    │
  │ steer      │ mppi_lag  │ mppi_lag_calm, mpc_lag,      │
  │            │           │ mpc_lag_rs, mppi_lag_smooth  │
  │ smoother   │ ours      │                              │
  │ lanes      │ off       │ ramp (lane costs)            │
  │ behaviour  │ off       │ on (road ahead and things),  │
  │            │           │ on_per_kind (TODO 217)       │
  │ kerbs      │ off       │ on (the step finder),        │
  │            │           │ patchwork (ground by patch)  │
  └────────────┴───────────┴──────────────────────────────┘
          │
          ▼
  pixi run sim                  ours: the simulator's default stack
  pixi run sim stack:=best      every level's champion
```

!!! note "Nav2 went on 17 Nov (TODO 184)"
    Ben, 16 Nov: delete Nav2 "Now". Each level Nav2 held handed over to its port: the
    costmap to `ours`, the navigator to `ours`, the path to `lattice_rs`, the steering to
    `mppi_lag`, the smoother to `ours`, and the stop zone (not a level: a safety lock) to
    `aav_stop_zone` once it had stopped every time Nav2's did, no later. `stack:=stock` and
    the Nav2 options went with it. What Nav2 was and its last scores:
    [Nav2, as it was](history/nav2-as-it-was.md).

## The latest scores

**16 Nov, every level scored** inside `stack:=ours`, then the best combination against it on
all four tours: [the league's scores, 16 Nov](league-results-2026-11-16.md); stage 2: [20 Nov](league-results-2026-11-20.md). No champion
changed; `pose:=ndt_ekf` is the closest challenger (TODO 210). Nav2's last yardstick scores
are there too: the last time it ran.

## The stacks

**Ours is the simulator's default** (Ben, 14 Nov: "Ours default now, then delete").
The champions above are what the car runs.

| `stack:=` | What it runs |
|---|---|
| `ours` (the default) | every level we built: our costmap (`ours_chunks` since 16 Nov: only the map's 64 m chunks round the car and along the route, [why](own-costmap.md#from-map-chunks-on-every-core-16-nov-todo-175-195)), navigator, planner (`lane` since 16 Nov: the right lane's centre on the lane map, `lattice_rs_strong` off it, [why](own-planner.md#the-swap-16-nov-3-clean-runs-each)), `mppi_lag_calm` (`mppi_lag` with `map_correction:=slow` and `mppi_lag_steer_gamma:=0.05`), the road ahead (`behaviour:=true`), and the kerbs (`kerbs:=true` since 16 Nov: the lidar's step finder, [why](kerbs.md#in-the-simulator)). The two calm picks since 15 Nov: they took ours' sway to Nav2's ([why](history/mppi-lag-log.md#the-sway-that-was-left)) |
| `best` | every level's champion |
| `car` | the champions approved for the car (`car.launch.py` runs these today) |
| (empty) | no stack: each switch as you give it, else the champion's |

A switch you give by hand **always wins**:

```sh
pixi run sim world:=campus                       # all ours
pixi run sim world:=campus tracker:=mpc_lag      # all ours, but MPC follows
pixi run sim stack:=best behaviour:=true        # the champions, with the road ahead
```

The launch prints what it picked, such as
`stack:=ours - pose ekf, costmap ours_chunks, navigator ours, path lane, steer mppi_lag_1k, ...`.

!!! note "How a hand-set switch is seen"
    The switches a stack sets are empty by default in `sim.launch.py`, meaning "the
    stack's choice". Any value you give is yours, even one that matches the champion:
    `tracker:=mppi_lag` wins over `stack:=ours`' `mppi_lag_calm`.

## One switch per level (15 Nov)

Fable's clean-up (one shape, "From 36 switches to 8"): pick a level's option by its name in
`stacks.yaml`, not by the switches under it.

| Switch | Options | Sets |
|---|---|---|
| `pose:=` | `ekf`, `ndt_ekf`, `kiss_ekf`, `ekf_lane` (ours since 17 Nov: the lane finder into the EKF), `kinematic_ekf` (TODO 216: Kinematic-ICP's idea in KISS-ICP's place; scored in stage 2) | `ndt_ekf`, `kiss_ekf`, `lane_pose`, `odom_lidar` |
| `path:=` | `lattice_rs`, `lattice_rs_smooth`, `hybrid_rs`, `lattice_rs_strong`, `lane`, `lane_lanelet2`, `diffusion`, `lane_jerk` (TODO 214: a jerk-limited pass; scored in stage 2) | `planner`, `planner_search`, `plan_smoothing`, `lattice_moves`, `lane_route`, `lane_graph`, `lane_pass_shape` |
| `steer:=` | `mppi_lag`, `mppi_lag_calm`, `mppi_lag_1k`, `mpc_lag`, `mpc_lag_rs`, `mppi_lag_smooth` (TODO 212: the steering's change sampled; scored in stage 2) | `controller`, `tracker`, `map_correction`, `mppi_lag_steer_gamma`, `mppi_lag_batch`, `mppi_lag_sampling` |
| `lanes:=` | `off`, `ramp` | `lane_costs` |

```sh
pixi run sim world:=campus path:=hybrid_rs       # all ours, but our Hybrid-A*
pixi run sim world:=campus path:=lane            # all ours, the path on the right lane's centre
pixi run sim stack:=best steer:=mppi_lag_calm    # the champions, but the calm MPPI
```

| Who wins | Over |
|---|---|
| a single switch by hand (`planner_search:=lattice`) | everything |
| a level switch (`path:=hybrid_rs`) | the stack's pick for that level: the level's other switches go back to their fallback (`plan_smoothing` to `nav2`) |
| the stack (`stack:=ours`) | the fallback (the champions') |

`costmap`, `navigator`, `smoother` and `behaviour` are switches already (by `own`, `true`...),
so they stay as they are. The old switches keep working.

## How a challenger wins

| Rule | Why |
|---|---|
| It wins by a margin, on 3 or more runs of the same routes | 7 of 12 against 8 is a tie |
| Scored by one scorecard: `tools/score_drive.py` on a recording | every challenger is judged the same way |
| A person approves each swap | a number alone doesn't change the car |
| The car needs its own win, on the Orin | a sim win isn't a car win |
| Nothing is deleted | the loser stays a challenger (Ben, 5 Nov). Nav2 was the one exception (Ben, 16 Nov) |

To swap a champion: change its `champion:` in `stacks.yaml`, and write the score's date and
recording in its `score:`.

## Where things are

| Item | Where |
|---|---|
| The file | `ros/bringup/aav_description/config/stacks.yaml` |
| The code (no ROS) | `aav_description.stacks`: `load`, `switches`, `resolve`, `describe` |
| Its tests | `ros/bringup/aav_description/test/test_stacks.py`: every switch real, `ours` reverses, a hand-set switch wins; `aav_bringup/test/test_sim_launch.py`: ours by default, `best` is the champions, `stock` refused, a hand-set switch wins |
| The launch | `sim.launch.py stack:=` |
| The design | [one shape](designs/2026-11-13_one-shape.md), "The league"; [the whole car](designs/2026-11-14_the-whole-car.md), step 7 |

Not yet: `stack:=` on `car.launch.py`, and `pixi run measure league` (re-score every
challenger and propose a swap). `pixi run measure controllers --stacks own own_calm` scores
stacks against each other: every run there starts from `stack:=best` (until 17 Nov,
`stack:=stock`) and names the pieces it tests.
