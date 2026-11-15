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
  │            │           │ lattice_rs_strong            │
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
| `ours` (the default) | every level we built: our costmap, navigator, planner (`lattice_rs_strong`: `lattice_rs` with `plan_smoothing:=strong`), `mppi_lag_calm` (`mppi_lag` with `map_correction:=slow` and `mppi_lag_steer_gamma:=0.05`), and the road ahead (`behaviour:=true`). The two calm picks since 15 Nov: they took ours' sway to Nav2's ([why](mppi-lag.md#the-sway-that-was-left)) |
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
`stack:=ours - pose ekf, costmap ours, navigator ours, path lattice_rs_strong, steer mppi_lag_calm, ...`.

!!! note "How a hand-set switch is seen"
    The switches a stack sets are empty by default in `sim.launch.py`, meaning "the
    stack's choice". Any value you give is yours, even one that matches the champion:
    `costmap:=nav2` wins over `stack:=ours`.

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
