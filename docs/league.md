# The league

One file says **what the car is**: each level's champion, and the challengers
waiting to beat it.

```text
  config/stacks.yaml (aav_description)
  ┌────────────┬───────────┬──────────────────────────────┐
  │ Level      │ Champion  │ Challengers                  │
  ├────────────┼───────────┼──────────────────────────────┤
  │ pose       │ ekf       │ ndt_ekf, kiss_ekf            │
  │ costmap    │ nav2      │ ours                         │
  │ navigator  │ nav2      │ ours                         │
  │ path       │ lattice   │ lattice_rs, lattice_smooth,  │
  │            │           │ lattice_rs_smooth, hybrid,   │
  │            │           │ hybrid_rs, diffusion,        │
  │            │           │ lattice_rs_strong, lane      │
  │ steer      │ mppi      │ mppi_lag, mppi_lag_calm,     │
  │            │           │ mpc_lag, rpp                 │
  │ smoother   │ nav2      │ ours                         │
  │ lanes      │ off       │ ramp (lane costs, ours only) │
  │ behaviour  │ off       │ on (the road ahead)          │
  └────────────┴───────────┴──────────────────────────────┘
          │
          ▼
  pixi run sim                  ours: the simulator's default stack
  pixi run sim stack:=stock     every level Nav2's: the yardstick
```

## The stacks

**Ours is the simulator's default** (Ben, 14 Nov: "Ours default now, then delete").
The champions above stay as they are: they are what the car runs.

| `stack:=` | What it runs |
|---|---|
| `ours` (the default) | every level we built: our costmap, navigator, planner (`lane` since 16 Nov: the right lane's centre on the lane map, `lattice_rs_strong` off it, [why](own-planner.md#the-swap-16-nov-3-clean-runs-each)), `mppi_lag_calm` (`mppi_lag` with `map_correction:=slow` and `mppi_lag_steer_gamma:=0.05`), and the road ahead (`behaviour:=true`). The two calm picks since 15 Nov: they took ours' sway to Nav2's ([why](mppi-lag.md#the-sway-that-was-left)) |
| `stock` | every level stock: Nav2 and robot_localization |
| `best` | every level's champion |
| `car` | the champions approved for the car (`car.launch.py` runs these today) |
| (empty) | no stack: each switch as you give it, else the champion's |

A switch you give by hand **always wins**:

```sh
pixi run sim world:=campus                       # all ours
pixi run sim world:=campus tracker:=mpc_lag      # all ours, but MPC follows
pixi run sim world:=campus costmap:=nav2         # Nav2's costmaps, the rest ours
pixi run sim stack:=stock planner:=own           # Nav2's, but our planner
```

The launch prints what it picked, such as
`stack:=ours - pose ekf, costmap ours, navigator ours, path lane, steer mppi_lag_calm, ...`.

!!! note "How a hand-set switch is seen"
    The switches a stack sets are empty by default in `sim.launch.py`, meaning "the
    stack's choice". Any value you give is yours, even one that matches the champion:
    `costmap:=nav2` wins over `stack:=ours`.

## One switch per level (15 Nov)

Fable's clean-up (one shape, "From 36 switches to 8"): pick a level's option by its name in
`stacks.yaml`, not by the switches under it.

| Switch | Options | Sets |
|---|---|---|
| `pose:=` | `ekf`, `ndt_ekf`, `kiss_ekf` | `ndt_ekf`, `kiss_ekf` |
| `path:=` | `lattice`, `lattice_rs`, `lattice_smooth`, `lattice_rs_smooth`, `hybrid`, `hybrid_rs`, `lattice_rs_strong`, `lane`, `diffusion` | `planner`, `planner_search`, `plan_smoothing`, `lattice_moves`, `lane_route` |
| `steer:=` | `mppi`, `mppi_lag`, `mppi_lag_calm`, `mpc_lag`, `rpp` | `controller`, `tracker`, `map_correction`, `mppi_lag_steer_gamma` |
| `lanes:=` | `off`, `ramp` | `lane_costs` |

```sh
pixi run sim world:=campus path:=hybrid_rs       # all ours, but our Hybrid-A*
pixi run sim world:=campus path:=lane            # all ours, the path on the right lane's centre
pixi run sim stack:=stock steer:=mppi_lag_calm   # Nav2's, but our calm MPPI
```

| Who wins | Over |
|---|---|
| a single switch by hand (`planner_search:=lattice`) | everything |
| a level switch (`path:=hybrid_rs`) | the stack's pick for that level: the level's other switches go back to their fallback (`plan_smoothing` to `nav2`) |
| the stack (`stack:=ours`) | the fallback (Nav2's) |

`costmap`, `navigator`, `smoother` and `behaviour` are switches already (by `own`, `true`...),
so they stay as they are. The old switches keep working.

## How a challenger wins

| Rule | Why |
|---|---|
| It wins by a margin, on 3 or more runs of the same routes | 7 of 12 against 8 is a tie |
| Scored by one scorecard: `tools/score_drive.py` on a recording | every challenger is judged the same way |
| A person approves each swap | a number alone doesn't change the car |
| The car needs its own win, on the Orin | a sim win isn't a car win |
| Nothing is deleted | the loser stays a challenger (Ben, 5 Nov) |

To swap a champion: change its `champion:` in `stacks.yaml`, and write the score's date and
recording in its `score:`.

## Where things are

| Item | Where |
|---|---|
| The file | `ros/bringup/aav_description/config/stacks.yaml` |
| The code (no ROS) | `aav_description.stacks`: `load`, `switches`, `resolve`, `describe` |
| Its tests | `ros/bringup/aav_description/test/test_stacks.py`: every switch real, `ours` reverses, a hand-set switch wins; `aav_bringup/test/test_sim_launch.py`: ours by default, `stock` is Nav2's, a hand-set switch wins |
| The launch | `sim.launch.py stack:=` |
| The design | [one shape](designs/2026-11-13_one-shape.md), "The league"; [the whole car](designs/2026-11-14_the-whole-car.md), step 7 |

Not yet: `stack:=` on `car.launch.py`, and `pixi run measure league` (re-score every
challenger and propose a swap). `pixi run measure controllers --stacks nav2 own` scores
`stock` against `ours` today: every run there starts from `stack:=stock` and names the
pieces it tests, so its scores stay comparable with the runs before 14 Nov.
