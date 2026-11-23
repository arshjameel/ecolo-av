# Proving it: how the tests fit together

**One page for the question "how do we know?"** We have two sets of tests. This page says
what each is for, how one step proves the step before, and how the same test runs on the
car and in the simulator. Written 22 November 2026 by Ben Woodhead (Ben: "Testing. Proofs.
Not just feelings. Systematic").

The detail is in [the testing plan](testing-plan.md), [Every test](tests.md),
[Actuator tests](actuator-tests.md) and [the league](league.md). This page adds nothing to
run: it is the map, and the list of what is missing.

> **The short version**
> 1. **Set 1, the ladder:** does the *car* work? Bench, stands, hallway, lot, route. Each
>    step measures the car and proves the step before.
> 2. **Set 2, the algorithm tests:** which *algorithm* is best? Every one drives the same
>    tests and is scored the same way.
> 3. **Every car test has a twin in the simulator**, with the same command and the same
>    report. Where the two differ, we fix the simulator's numbers.
> 4. **The rules of evidence** (below) apply to both sets.
> 5. Nothing here has run on the car yet. "Built" means it passes in Ravensim.

## Where we are: `pixi run ladder`

```bash
pixi run ladder                      # every rung: the simulator's side and the car's
pixi run ladder next                 # the next open rung and its command (--car: the car's)
pixi run ladder run stands           # run a rung's simulator twin; the result is recorded
pixi run ladder record --from-report ~/aav-tests/<session>    # a car-test session
pixi run ladder record lot pass --car --note "2.5 m/s, dry"   # a rung done by hand
```

| Rule | |
|---|---|
| Progress is one file, `~/aav-tests/ladder.yaml` | a line a result, with its date, commit and who; never rewritten |
| The simulator and the car are kept apart | a pass in Ravensim never marks the car's rung |
| A rung is shut until the rungs under it have passed | a car rung also needs its own simulator twin |
| A pass on a shut rung needs `--override "why"` | and the why is kept |
| It never moves the car | for a car rung it prints the command; a person runs it |

The rungs are `tools/ladder/rungs.yaml`.

### The window: `pixi run ladder-window`

```bash
pixi run ladder-window               # the ladder, in a window (the first time: cargo builds Bevy)
pixi run ladder-window --pretend     # sample data: nothing runs, nothing is recorded
```

![The test ladder's window, pretend data](img/ladder-window.png)

| Part | Shows | Does |
|---|---|---|
| left: the ladder | the top rung at the top; **Simulator** and **The car** side by side; each cell in words: PASS or FAIL with the date, not run, shut | click a rung; hover a shut cell for why |
| right: the rung | what it measures, when it passes, the last result (date, commit, report, override), what shuts it | |
| **Run in the simulator** | on only when the simulator's side is open and has a command; else it says why | `pixi run ladder run <rung>`; one run at a time |
| the console | the run's last lines, "running...", how it ended | **Stop** ends the run and all it started |
| **The car** | its command as text, and "A person starts a car test from the terminal" | **Copy** only |
| **Record by hand** | pass or fail, simulator or car, a note, an override reason | `pixi run ladder record ...`; a refusal is shown in the tool's words |
| top: **The simulated car** | "on the ground" or "ON STANDS", asked every 5 s; off, with the reason, unless a simulator runs on this computer | **Put it on stands** / **Lower it to the ground**: `pixi run jacks up --sim` or `down`; Ravensim's refusal (wheels still spinning) is shown in its words |

| Rule | |
|---|---|
| The window never moves the car | it has no control that starts a car test; the jacks switch is the simulator's only (`--sim`, `--local`, refused when `/system/source` says REAL) |
| The tool is the only writer | the window reads `ladder --json` every 2 s and after each action; every rule stays in `tools/ladder.py` |
| Closing the window stops its run | no simulator is left behind |

The code: `ros/hud/aav_dev_window` (`src/ladder_window/`, and the logic in `crates/ladder-core`).

## The rules of evidence

| Rule | Means |
|---|---|
| Write the claim first | one sentence a run could show to be wrong |
| Fix the pass mark first | the number that decides, before the run, not after |
| Always a baseline | the same run without the new thing |
| Repeat | one run is a story; three or more give a spread (mean ± sd) |
| Keep everything | the commit, the command, the settings, the recording |
| A failure is a result | written down the way a pass is |

## Set 1: the ladder

Each step **uses** the numbers of the step before, and **proves** them by getting the
result the model said it would.

| Step | Where | It measures | It proves the step before by | Hands on |
|---|---|---|---|---|
| 0 | laptops: unit tests, the pretend car | nothing: it checks the code against the maths | — | code that does what we wrote |
| 1 | Ravensim, the whole stack | the scores of a drive | the code works together, not only in parts | a stack fit to put on the car |
| 2 | bench: the real boards, no car | messages, timing, the locks | the stack talks to the real electronics as it did to the simulator's | a car safe to switch on |
| 3 | stands (phase A) | steering: angle, rate, centre; the wheels follow a speed | the bench's commands move the real actuators, the right way | steering numbers → `vehicle.yaml` |
| 4 | hallway (phase B) | speed up, coast, brake: dead time, lag, stopping distance | the model with step 3's numbers predicts the real run | throttle and brake numbers → `vehicle.yaml` |
| 5 | lot | the same at 2 to 3 m/s; circles; a straight line | the hallway's numbers still hold at speed | a model good to 3 m/s |
| 6 | a route, the car driving itself | the same scores as step 1 | the simulator's drive (step 1) predicted the car's | the capstone's result |

**The loop that closes it:** after steps 3 to 5 the new numbers go into `vehicle.yaml`,
the simulator runs steps 0 and 1 again with them, and step 6's scores are compared with
step 1's. The pass mark that exists today: the model's speed within 10 % of the car's, its
stop within 0.2 m ([the model against the car](actuator-tests.md#the-model-against-the-car)).

## The same test, on the car and in the simulator

One command, with `--sim` for the twin. The report has the same form both ways.

| Test | The step input | What we read | Measured by | Tunes | Built |
|---|---|---|---|---|---|
| `speed-up` | speed: 0 to the cap | time to each speed, acceleration | wheels, IMU | `max_accel`, `throttle_lag` | ✅ |
| `brake` | speed: 1, 2, 3 m/s to 0 | stopping distance, deceleration | wheels, IMU | `max_brake`, `brake_lag` | ✅ |
| `coast` | pedals off at 2 m/s | how fast it slows | wheels | `drag` | ✅ |
| `throttle-map` | speed: ¼, ½, ¾, all of the cap | acceleration per throttle | wheels, IMU | `throttle_deadband` | ✅ |
| `steer-sweep`, `steer-rate`, `steer-centre` | steering: lock to lock, small steps | angle, rate, centre | the steering sensor (or by hand) | `steer_rate`, `steer_lag`, `steer_offset` | ✅ |
| `circles` | a fixed steering angle at 1 m/s | the circle's radius | the local EKF, GPS | the real angle per command | ✅ |
| `straight` | steering straight, 10 m | drift to the side | the local EKF, GPS | `steer_offset` | ✅ |
| `car-test --hallway` | all of the above, several passes | each number **with an error bar**; the model over the run | wheels, IMU, NDT | the transfer functions | ✅ |
| Hold a speed | 1 m/s for 30 s, then 2 | how far the speed wanders | wheels, GPS | the speed loop | ⬜ to build |
| A steering step on the ground | 0 to 0.1 rad at 1 and 2 m/s | yaw rate: delay, overshoot, settling | IMU, the local EKF | `steer_lag` on the ground; the model's yaw | ⬜ to build |
| A lane change | 2 m to the side over 15 m, by the follower | sway: metres side to side, how often the steering changes side | the local EKF, GPS | the follower's tuning | ⬜ to build (the scorecard has the sway number) |

**The stands twin is on stands too** (TODO 272, 22 Nov). `car-test --stands --sim` starts
its Ravensim with the car on jacks and lowers it at the end: the rear wheels spin and steer,
the car stays put, and the wheels disagree with the IMU and the GPS, as on the real stands
([how](actuator-tests.md#the-simulated-car-on-stands-pixi-run-jacks)). Before, the simulated
car crawled in arcs and the stands checks ran backwards.

| The first honest run found | Fixed |
|---|---|
| "GPS still" failed on a still car: the receiver's noise passes 0.1 m/s one reading in seven | the median decides, not the largest |
| the live IMU watch stopped a still car one run in three (Ravensim's IMU noise, 0.2 m/s²) | the mark allows for the IMU's own noise at rest, never over 0.25 m/s; a quiet IMU keeps 0.15 |

Both would have shown up first on car day.

GPS and NDT are the **outside witnesses**: they do not pass through the wheels or the IMU,
so they catch a wrong wheel size or a drifting IMU.

## Set 2: the algorithm tests

Every algorithm for a job (a path follower, a planner) is a drop-in for the others, so
every one takes the same tests.

| Test | The question | Scored by | Built |
|---|---|---|---|
| The league's routes | which drives the route best? | `tools/score_drive.py`: time, metres off the plan, sway, holds; 3 or more runs | ✅ |
| A sideways step | started 1 m off the path, how does it come back? Overshoot, settling time | the same harness for every follower | ⬜ exists for `mpc_lag_rs` only |
| A speed step | asked from 1 to 2 m/s, how does it get there? | the same | ⬜ |
| A stop at the goal | how far from the mark, how hard the brake? | the scorecard | ✅ |
| A ghost ahead | does it stop, and how far before? | `sim-check --ghost` | 🟡 runs; it reports FAIL for every follower (it listens for the costmap on an old name) |
| The time of a tick | does it answer inside its period? | the follower's status | ✅ per follower; ⬜ one table |

A challenger replaces the default only by [the league's rules](league.md#how-a-challenger-wins).

## What is missing

| # | Gap | TODO |
|---|---|---|
| 1 | The three tests marked "to build" above: hold a speed, a steering step on the ground, a lane change | 266 |
| 2 | One step-test harness for every follower (sideways step, speed step), with one table | 267 |
| 3 | `sim-check --ghost` listens on the costmap's old name | 268 |
| 4 | A run log: every test run's commit, command, settings and result in one place | 269 |
| 5 | Step 6 against step 1: the car's scores beside the simulator's for the same route | 270, after the car drives |
| 6 | The lot and route rungs have no one-command simulator twin (by hand today) | 273 |
| 7 | One command that opens Ravensim, the hardware panel, the debug screen and the ladder window side by side | 273 |
| 8 | The hardware panel and the debug screen do not show "on stands" (the ladder window and `pixi run jacks status --sim` do) | 273 |
| 9 | On stands the simulator's free-wheel mass (20 kg) and drag are guesses; its odometry reads the rear wheels, today's car the front | 49b, and a stands measurement |
