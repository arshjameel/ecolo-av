# Barriers in the MPC: keeping the car off things

!!! warning "Outside the capstone's scope (Ben's own research, toward his MASc thesis)"
    Ben's own work, not a team deliverable. Off by default: nothing in the stack turns
    it on, and no safety lock depends on it. Tested on a pretend car, and run in
    Ravensim with the whole stack (section 5: [watch it](#watch-it-in-ravensim)).
    **Never on the car.**

!!! note "The short version"
    - **Problem:** the path follower follows its path, even into something on it.
    - **Idea** (Ali, Shen, Hashim, 2024): a **control barrier function** in the MPC. The
      room to a keep-out circle may shrink only by a share each tick, so it never
      reaches zero.
    - **Built:** the barrier in `aav_mpc_lag`, a stop rule behind it, and the ghost as
      its first obstacle (`mpc_barriers:=true`).
    - **On a pretend car:** the plain tracker ends 0.5 m inside the object; with
      barriers, none of 36 runs touches it (smallest gap 0.22 m).
    - **In Ravensim, at 3 m/s:** the plain tracker drives through the ghost, the hard
      distance limit stops close before it, the barrier goes round. See it:
      `pixi run barrier-watch --side-by-side --speed 3`.

The study notes (a walk-through and a survey) are kept outside this repository.

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

`mpc_barrier_gamma:=1` beside it sets γ (empty: 0.05). To watch one drive with
everything set up for you: [Watch it in Ravensim](#watch-it-in-ravensim).

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
| 15 | **Going round was not tried** in these three runs: navigation drives 1 m/s, and going round starts near 3 m/s (finding 8) | tried since: [Watch it in Ravensim](#watch-it-in-ravensim), below |

Two things seen on the way, not about the barriers:

- `sim-check --ghost` reported FAIL for every follower: it listened for the local costmap on
  its old name (`/local_costmap/costmap`; ours is `/drivers/nav/costmap/local`). Fixed
  22 Nov (TODO 268): it passes with `mpc_lag_rs`. With `mppi_lag` it still fails, for
  another reason (the car stands turned aside, so the check reads no gap).
- One earlier run of `mpc_lag_rs` without barriers, while two other simulators were
  loading the same computer, **drove through the ghost and the stop zone never fired**.
  It did not happen again on a quiet computer. Not explained.

### Watch it in Ravensim

One command: the same drive past the same ghost, the follower three ways. It starts
Ravensim's road world with navigation, opens our own navigation view from above (the
mode in its title; RViz only with `--rviz`: [RViz and ours](hud.md#rviz-and-ours)), places the ghost 15 m
past the bumper, sends a goal 40 m straight ahead, and prints one line of result.

```sh
pixi run barrier-watch plain   --speed 3     # the follower (mpc_lag_rs) alone
pixi run barrier-watch step    --speed 3     # a hard distance limit: the rule with γ = 1
pixi run barrier-watch barrier --speed 3     # the barrier, γ = 0.05
pixi run barrier-watch --side-by-side --speed 3    # plain and barrier at once
```

| Option | Does |
|---|---|
| `--speed 3` | m/s the car may drive (1: navigation's own, and the real car's; up to 5) |
| `--offset 0.4` | the ghost's centre, m left of the car's line (0: on it; negative: right) |
| `--side-by-side` | two simulators at once, each on its own ROS domain; `--modes plain,step` picks the two |
| `--gamma 0.02` | with `barrier`: another γ |
| `--headless` | no windows; it stops when the drive ends |
| `--dev-window` | the whole developer window, live, in place of the navigation view |
| `--rviz` | also RViz (`rviz/barrier.rviz`): the stock viewer, the reference only |
| `truth_pose:=true` | (any `sim.launch.py` switch) the simulator's truth as the pose: takes localization's wander out of the picture |
| Ctrl-C | stops everything the command started, by process group |

| In the view | Is |
|---|---|
| the purple dot | the ghost (a person, 0.5 m wide) |
| the purple circle | the barrier's keep-out circle: the body's point may not come inside. Drawn in every mode, so `plain` shows what it drives through |
| the blue line | navigation's plan: straight through the ghost (the planner does not see it) |
| the orange trail | where the car has been |
| the box ahead of the car | the stop zone's box, from the front bumper ahead: white when clear, red with STOP ZONE: STOP while it stops the car |

The view's newest picture is kept beside the CSV (`/tmp/aav-barrier-watch/<mode>-<speed>-<offset>.png`).
The ladder window (`pixi run dry-run`) can start the three modes too, in a box of its own
marked as research.

The window's title says which mode it is. Each run's samples go to
`/tmp/aav-barrier-watch/<mode>-<speed>-<offset>.csv`, the commit and the command first.

!!! note "Every safety lock stays on"
    The stop zone and the ghost's costmap layer are as in every other run: the command
    takes nothing out. It turned out that none of them is in the barrier's way.

**What it showed** (22 Nov, `--headless`, one computer, each run alone). The gap is from
the car's body to the ghost, on the simulator's truth; below zero is inside it. The
first number is the last set of runs (24, commit `c56baee1`); in brackets, every run of
that cell, earlier ones too.

| Speed | The ghost | Plain | Step (γ 1) | Barrier (γ 0.05) |
|---|---|---|---|---|
| 3 m/s | on the line | **drives through**, −0.59 m (5 of 6: −0.59 to −0.71) | stops, 0.64 m (3 of 4 stop, 0.44 to 0.64; 1 stops, backs up and gets round) | **goes round**, 0.75 m (11 of 11: 0.75 to 1.52) |
| 3 m/s | 0.4 m left | **touches**, −0.19 m (2 of 2) | stops, 0.39 m (3 of 3: 0.23 to 0.39) | **goes round**, 0.98 m (3 of 3: 0.71 to 1.59) |
| 3 m/s | 1.0 m left | passes beside, 0.30 m (3 of 3: 0.30 to 0.54) | passes beside, 0.80 m (1 of 2; the other stops, 0.44) | goes round, 1.12 m (2 of 2) |
| 1 m/s | on the line | stops, 0.03 m (2 of 2: 0.03, 0.25) | stops, 0.70 m (2 of 2) | stops, 0.80 m (2 of 2) |
| 1 m/s | 0.4 m left | **touches**, 0.00 m (the other: stops, 0.05) | stops, 0.58 m (2 of 2) | stops, 0.66 m (2 of 2) |
| 1 m/s | 1.0 m left | passes beside, 0.28 m (2 of 2) | **stops**, 0.79 m (2 of 2) | passes beside, 0.61 m (2 of 2) |

| Also | Result |
|---|---|
| Barrier, on the line, 2 m/s | stops, 0.80 m (2 of 2) |
| Barrier, on the line, 2.5 m/s | goes round, 1.11 m (2 of 2) |
| Barrier with γ 0.02, on the line, 3 m/s | goes round wider: 2.68 m off the line, gap 1.71 m (1 run) |
| The ghost 0.4 m right, 3 m/s, barrier | goes round, 1.18 m (1 run) |
| `truth_pose:=true`, 3 m/s, on the line | plain −0.75 m, step stops 0.46 m, barrier goes round 1.49 m: the same picture |
| `--side-by-side`, 3 m/s, twice (once with the windows) | plain −0.69, −0.71 m; barrier 1.50, 1.20 m: the same as alone |

**Who acts first**

| Follower | First to ask a stop | Then |
|---|---|---|
| Plain | the stop zone, when the ghost is 0.5 to 0.9 m from the bumper | at 1 m/s the car stands 0 to 0.25 m before it; at 3 m/s it cannot stop in 1 m and drives through |
| Step | the stop rule, 5.6 to 8.4 m before the ghost at 3 m/s, 1.9 to 2.4 m at 1 m/s; then the stop zone | it creeps on, and the stop zone holds it 0.2 to 0.7 m before the ghost |
| Barrier | the stop rule, at the same distances | at 2.5 m/s and up it slows to 0.3 to 1 m/s, steers round outside the stop zone's box, and speeds up again: the stop zone never fired |

| # | What it showed | |
|---|---|---|
| 16 | **Going round works in Ravensim with every lock on**, from 2.5 m/s | nothing had to be switched off |
| 17 | **It is not one smooth swerve**: the stop rule brakes first (to 0.3 to 1 m/s), then the car steers round and speeds up | 19 to 23 s for the 40 m; plain takes 20 s |
| 18 | **At 2 m/s and below it stops**, as on the pretend car (finding 8) | see "What stops it", below |
| 19 | **The plain tracker at 3 m/s drives through the ghost**: the stop zone asks at 0.5 to 0.7 m, too late for 3 m/s | the stop zone is sized for the car's 1 m/s; `sim_speed:=3` is the simulator's only |
| 20 | **At 1 m/s the stop zone alone leaves 0 to 0.25 m**; the stop rule before it leaves 0.6 to 0.8 m | R3's result again, in more runs |
| 21 | **The step is the worst to watch**: it stops closest (0.23 m once), stops for a ghost 1.0 m aside that the others pass, and once backed up and crept round | what a barrier adds to a plain limit, in one picture |
| 22 | **One plain run passed 0.96 m beside the ghost on the line**: the car was lost, 1.8 m off its line from the start | localization, not the follower. The likely cause, not proved: the follower's log shows its map-to-odom correction jumping by 10° to 25° in 6 of the last 23 runs. `truth_pose:=true` takes it out |
| 23 | **On a busy computer a tick overruns** (25 Nov): the pretend-car run that goes round the object, 53.7 ms mean and 403 ms worst with a Rust build beside it on Ben's MacBook (the 50 ms tick); one GitHub ARM run failed the same test. The test `a_tick_with_a_barrier_fits_well_inside_the_period` (under 25 ms mean) is now run by hand on a quiet machine (`-- --ignored`) | the solver's time is the next thing to fix before the barrier could drive (finding 5, 14) |

**What stops a go-round, and what does not**

| Part | In the way? | How we know |
|---|---|---|
| The tracker's own look ahead | **yes, below 2.5 m/s** | its plan is 1.5 s long: 1.5 m at 1 m/s, and the keep-out circle is 3.1 m across. The stop rule fires 2 m before the ghost, the car creeps straight on, and never turns. The pretend car, with no stop zone at all, stops the same way |
| The stop zone | **it ends the creep** at 1 and 2 m/s: the ghost enters its box just under 1 m from the bumper, and `stop_hold` holds navigation | the runs' samples and logs: the stop rule first, the stop zone about 2 s later. At 2.5 m/s and up the car is already beside the box: it never fires |
| The planner | no | the ghost is not in the planner's costmap (`vehicle.nav2_tuned`): the plan stays straight through it |
| The costmap in the follower | no | `mpc_lag_rs` reads no costmap (the default `mppi_lag` does, and steers by it) |
| The behaviour planner's speed cap | no | it does not read the ghost |
| Navigation's 1 m/s | **yes** | `sim_speed:=` (the command's `--speed`) raises it, in the simulator only |

Not done: whether the barrier alone would creep round at 1 m/s with the ghost out of the
stop zone's sources. That would take a new switch that weakens a lock, and the pretend
car already says no.

!!! warning "How far to trust the numbers"
    One to twelve runs a cell, not the test plan's ten each. The ghost is a person-sized
    cylinder (0.25 m radius; the pretend car's object was 0.5 m). The gap moves by 0.2 to
    0.8 m between runs of the same cell: localization wanders (0.08 m median, more at
    the start). The windows were opened and checked on a picture of RViz's window (one
    run alone, one pair side by side). Ben has not watched it yet.

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
| Ten runs a cell in Ravensim (rungs R3, R4), and a league score against `mppi_lag` | next: `barrier-watch --headless` in a loop |
| Going round below 2.5 m/s (the car drives 1 m/s) | a longer look ahead, or the planner choosing a side (finding 3) |
| A smooth go-round: today the stop rule brakes first (finding 17) | open |
| Real obstacles (the costmap, `things`) as circles | the ghost is the only source |
| The stability rule (a Lyapunov rule in the plan) | not started; the safety filter has one for the lane |
| A proof with the dead time | open |
| Moving objects | open |
| Backwards | no barriers on a backwards stretch |
