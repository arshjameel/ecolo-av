# Barriers in the MPC: keeping the car off things

!!! warning "Outside the capstone's scope (Ben's own research, toward his MASc thesis)"
    Ben's own work, not a team deliverable. Off by default: nothing in the stack turns
    it on, and no safety lock depends on it. Tested on a pretend car and compiled
    against ROS. **Not yet run in Ravensim, and never on the car.**

!!! note "The short version"
    - **Problem:** the path follower follows its path, even into something on it.
    - **Idea** (Ali, Shen, Hashim, 2024): a **control barrier function** in the MPC. The
      room to a keep-out circle may shrink only by a share each tick, so it never
      reaches zero.
    - **Built:** the barrier in `aav_mpc_lag`, a stop rule behind it, and the ghost as
      its first obstacle (`mpc_barriers:=true`).
    - **On a pretend car:** the plain tracker ends 0.5 m inside the object; with
      barriers, none of 36 runs touches it (smallest gap 0.22 m).

To learn the idea from nothing: [the walk-through](barriers-guide.md), then
[the survey](barriers-survey.md).

## Scope: what is the capstone's and what is not

| | The capstone | Ben's MASc research |
|---|---|---|
| The path follower `mpc_lag_rs`, the stop zone, the locks | ✔ | uses them |
| The barrier code here, off by default (`mpc_barriers:=false`) | carried in the stack, not a deliverable, not maintained by the team | ✔ |
| The safety filter (`aav_safety_filter_rs`), in shadow by default | the same | ✔ |
| Showing it is safe: the proofs | no | ✔ kept outside this repository |
| The tests of the method ([the test plan](barriers-test-plan.md)) | no | ✔ |
| The car's own tests ([the testing plan](testing-plan.md)) | ✔ | uses their data |

Every file of it says "outside the capstone's scope" at its top.

## 1. The rule

A **keep-out circle** round the object: its radius, the car's own (1.12 m from the
body's middle to a corner), and a margin (0.2 m).

```text
   H = (the car's middle to the circle's centre)² − radius²      H > 0: outside

   every tick of the plan:    H(next)  ≥  (1 − γ) · H(now)
```

| γ | Behaviour |
|---|---|
| 0.02 | turns away early and wide |
| 0.05 (the default) | later and closer |
| 1 | only "never inside": as late as a plain distance limit |

If H starts at 0 or more and the rule holds every tick, H is never negative (six lines
of induction: the walk-through, step 5).

## 2. The stop rule behind it

The rule above is kept by the **plan**. The car can break it (section 5, finding 1). So
each tick the tracker also asks:

> Holding my steering as it is, would I meet the circle within the distance I need to stop?

If yes, it asks a stop now.

| Part | From |
|---|---|
| The distance to stop, S(v) | the one stopping law (`ego_core::stop_law::StopCurve`): ego's, the behaviour planner's and the safety filter's |
| "Holding my steering" | the bend the car is on now, checked every 5 cm |
| The tracker's own wait | its dead time and two ticks |

## 3. Two forms of one idea

| | The safety filter (`aav_safety_filter_rs`) | The barriers here |
|---|---|---|
| Where | in front of twist_mux, on the computer drivers' commands | inside the path follower |
| Keeps | room to stop before the road's end; inside the lane | off a round object |
| Can it steer round? | no: it slows and trims | yes |
| How | a formula a step, no solver | rows of the MPC's problem |
| State | a node, in shadow | a node switch, off by default |

They share the stopping law. Both are on this branch.

## 4. How to run it

```sh
pixi run sim world:=test_lot tracker:=mpc_lag_rs ghost:=true mpc_barriers:=true
pixi run ghost --local ahead 10      # place the ghost 10 m ahead
```

| Piece | Where |
|---|---|
| H, its slope, the bend held, the room to stop | `ros/drivers/aav_mpc_lag/crates/mpc-lag/src/barrier.rs` |
| The rule in the plan; the stop rule | `tracker.rs` (`set_barriers`, `fence`) |
| Forward stretches only | `geared.rs` (`set_things`) |
| The ghost's marker to circles; the status keys | `crates/mpc-node/src/main.rs` |
| The tests | `tests/barrier.rs`, and `cargo test --release -p mpc-lag --test barrier -- --ignored --nocapture` for the whole table |

The status (`/drivers/nav/track/status`) gains `barrier_m`, `barrier_slack_m2` and
`barrier_stop`.

## 5. What happened on the pretend car

A straight 40 m path, a round object (0.5 m radius) at 15 m. `gap`: metres between the
object and the nearest point of the car's body.

| Object | Cruise | Plain tracker | Barriers, γ 0.05 | With 0.2 s dead time |
|---|---|---|---|---|
| 0.4 m left of the path | 1 m/s | **hit** (−0.50) | stops short, 0.28 | stops short, 0.24 |
| 0.4 m left | 3 m/s | **hit** (−0.50) | goes round, 0.90 | goes round, 0.98 |
| on the path | 1 m/s | **hit** (−0.50) | stops short, 0.34 | stops short, 0.32 |
| on the path | 3 m/s | **hit** (−0.50) | goes round, 0.85 | goes round, 0.72 |
| 1.5 m beside it | 1 m/s | passes at 0.50 | passes at 0.72 | passes at 0.75 |
| 1.5 m beside it | 3 m/s | passes at 0.50 | passes at 1.08 | passes at 1.24 |

γ, on the object 0.4 m left at 3 m/s:

| γ | Turns away at | Widest off the path | Gap |
|---|---|---|---|
| 0.02 | x = 6.1 m | 2.62 m | 1.76 m |
| 0.05 | x = 10.6 m | 1.54 m | 0.90 m |
| 1 | too late to steer: it stops | 0.07 m | 0.27 m |

| # | Finding | Why it matters |
|---|---|---|
| 1 | **The barrier alone crept through.** The first build slowed nicely, then crawled into the object at 0.15 m/s | the spine's speed loop only coasts through a small overspeed; a linear plan can't see its stop. The maths was right; the actuator broke its assumption. Hence the stop rule |
| 2 | **Slow is harder than fast.** At 1 m/s it stops instead of going round | the plan sees 1.5 s = 1.5 m; the keep-out circle is 3.6 m wide |
| 3 | **Safe, and stuck.** A stopped car stays stopped | a barrier only pushes away: choosing a side is the planner's job |
| 4 | **A straight look ahead stops a car that is turning away** | with the real stopping law it stood still mid-swerve; the rule now looks along the bend |
| 5 | **The solver works harder**: a tick 0.3 to 0.5 ms without, 3 to 15 ms with a barrier near; it often ends at its iteration limit | inside the 50 ms tick; a better solver setting is to do |

### The sweep: 420 runs (22 Nov)

Five sweeps, each for four trackers: the cruise speed (0.5 to 4 m/s) on the straight and
on a 25 m bend, where the object stands (1.6 m right to 1.6 m left) at 3 m/s and at
1 m/s, and the car's dead time (0 to 0.4 s). "Distance limit" is the same rule with γ = 1:
a wall at the circle's edge and nothing before it, a step.

| Tracker (105 runs each) | Touched | Stopped short | Reached the end | Least gap |
|---|---|---|---|---|
| Plain | **77** | 0 | 28 | −0.50 m |
| Distance limit only (γ 1) | **1** | 104 | 0 | −0.06 m |
| Barrier, γ 0.05 | 0 | 41 | 64 | 0.21 m |
| Barrier, γ 0.02 | 0 | 39 | 66 | 0.22 m |

| # | What the sweep showed | Where |
|---|---|---|
| 6 | **The barriers never touched**, in 210 runs; the least gap was 0.21 m | every sweep |
| 7 | **The step never gets round, and once it touched**: at 3 m/s with 0.35 s of dead time it met the object (−0.06 m) | dead time |
| 8 | **Going round starts near 3 m/s** head on (γ 0.05: from 3.0; γ 0.02: from 2.75). Below that the car stops, safe and stuck | speed, bend |
| 9 | **A bend changes little**: the same gaps and the same speed where it starts to go round | bend |
| 10 | **Dead time costs the barrier 0.1 to 0.3 m of gap**, never the object | dead time |
| 11 | **At 1 m/s it stops for an object the plain tracker would pass**: one 1.0 to 1.1 m aside, with 0 to 0.1 m to spare | offset, slow |
| 12 | **The solver ran out of turns on 57 to 82 % of the ticks** with a rule on (3 % without) | all: finding 5, measured |

The graphs: `tools/barrier_graphs.py` draws them from the tests' CSVs.

```sh
cd ros/drivers/aav_mpc_lag
AAV_BARRIER_OUT=/tmp/barrier-traces cargo test --release -p mpc-lag --test barrier the_traces -- --ignored
AAV_BARRIER_OUT=/tmp/barrier-traces cargo test --release -p mpc-lag --test barrier the_sweep -- --ignored
python3 ../../../tools/barrier_graphs.py /tmp/barrier-traces /tmp/barrier-graphs.html
```

### In Ravensim, the whole stack (22 Nov)

`sim-check --ghost`: the road world, a ghost 15 m past the bumper, a goal 30 m on, at
navigation's 1 m/s. One run each (the barrier run three times); the stop zone stays on.

| Follower | First to act | Stood still | Gap to the ghost |
|---|---|---|---|
| `mpc_lag_rs`, no barriers | the stop zone, at 0.86 m/s | x = 14.72 m | 0.19 m |
| `mppi_lag` (the default) | the stop zone, at 0.60 m/s | x = 14.29 m | 0.66 m |
| `mpc_lag_rs`, barriers on | **the stop rule, 1.1 m sooner**; the stop zone fired after, at 0.43 m/s | x = 14.16 m | 0.76 m (0.84, 0.88) |

| # | What it showed | |
|---|---|---|
| 13 | **It works in the full stack**: the marker reaches the tracker, the car slows 1.1 m before the stop zone asks, and stops 0.6 m further back than the same tracker without | the last lock (the stop zone) then has half the speed to deal with |
| 14 | **The solver: 16.6 ms a tick near the ghost** (18.2 at worst; 0.7 ms without), a quarter of its ticks unsettled | inside the 50 ms tick, with little room: finding 12 again |
| 15 | **Going round was not tried**: navigation drives 1 m/s, and going round starts near 3 m/s (finding 8) | next: a faster run, or a longer look ahead |

Two things seen on the way, not about the barriers:

- `sim-check --ghost` reports FAIL for every follower: it listens for the local costmap on
  its old name (`/local_costmap/costmap`; ours is `/drivers/nav/costmap/local`).
- One earlier run of `mpc_lag_rs` without barriers, while two other simulators were
  loading the same computer, **drove through the ghost and the stop zone never fired**.
  It did not happen again on a quiet computer. Not explained.

## 6. What it assumes

| Assumption | True on the car? |
|---|---|
| the model matches the car | roughly: the lags are guesses until measured |
| the plan's H is the real H | no: H is made linear each tick about the last plan |
| the car stops as S(v) says | S(v) is ego's law; its numbers are not measured on the car |
| the car holds its steering while it stops | roughly |
| the car knows where it is, and where the object is | as well as localization and the ghost do |
| the object is a circle that stands still | a first shape only |
| the path is straight beside the object | else the path's frame bends the circle |

So: shown on a pretend car, argued for the model. **Not proved for the car.**

## 7. Not done

| | |
|---|---|
| Going round in Ravensim (it needs about 3 m/s), and a league score against `mppi_lag` | next |
| Real obstacles (the costmap, `things`) as circles | the ghost is the only source |
| The stability rule (a Lyapunov rule in the plan) | not started; the safety filter has one for the lane |
| A proof with the dead time | open |
| Moving objects | open |
| Backwards | no barriers on a backwards stretch |
