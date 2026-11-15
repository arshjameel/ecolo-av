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
  │            │           │ diffusion                    │
  │ steer      │ mppi      │ mppi_lag, mpc_lag, rpp       │
  │ smoother   │ nav2      │ ours                         │
  │ behaviour  │ off       │ on (the road ahead)          │
  └────────────┴───────────┴──────────────────────────────┘
          │
          ▼
  pixi run sim stack:=ours      one word sets every level
```

## The stacks

| `stack:=` | What it runs |
|---|---|
| (empty) | every switch as you give it: the same as before |
| `best` | every level's champion |
| `car` | the champions approved for the car (`car.launch.py` runs these today) |
| `stock` | every level stock: Nav2 and robot_localization |
| `ours` | every level we built: our costmap, navigator, planner (`lattice_rs`), `mppi_lag`, and the road ahead (`behaviour:=true`) |

A switch you give by hand **still wins**:

```sh
pixi run sim world:=campus stack:=ours                     # all ours
pixi run sim world:=campus stack:=ours tracker:=mpc_lag    # all ours, but MPC follows
```

The launch prints what it picked, such as
`stack:=ours - pose ekf, costmap ours, navigator ours, path lattice_rs, steer mppi_lag, ...`.

!!! note "A hand-set switch"
    "Given by hand" means "not its default". `stack:=ours planner:=lattice` keeps
    `lattice_rs`, because `lattice` is the default. To keep a default, leave the stack out.

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
| Its tests | `ros/bringup/aav_description/test/test_stacks.py`: every switch real, `ours` reverses, a hand-set switch wins |
| The launch | `sim.launch.py stack:=` |
| The design | [one shape](designs/2026-11-13_one-shape.md), "The league"; [the whole car](designs/2026-11-14_the-whole-car.md), step 7 |

Not yet: `stack:=` on `car.launch.py`, and `pixi run measure league` (re-score every
challenger and propose a swap). `pixi run measure controllers --stacks nav2 own` scores
`stock` against `ours` today.
