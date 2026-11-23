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
pixi run ladder run lot --only brake # one of its tests alone; not recorded
pixi run ladder session stands       # the newest simulator session: each step, its numbers
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
| A simulator run uses the simulator that is open | the rungs' commands carry `--attach`: `car-test` listens to `/system/source` on this computer; SIM: it tests that one; nothing heard: it starts its own, with no picture; anything else (REAL...): refused |
| A single test never moves the ladder | `run <rung> --only <test>` records nothing: a rung passes whole |

The rungs are `tools/ladder/rungs.yaml`.

## The dry run: `pixi run dry-run`

**Everything we will do with the real car, done first with the simulated one you can see**
(Ben, 22 Nov: "control the car in ravensim the same way we would control the actual car
... a dry run of everything we will do in real life"; TODO 273).

```bash
pixi run dry-run                     # three screens: Ravensim's, the hardware panel, the tests
pixi run dry-run --nav --debug       # and the navigation view alone, and the debug screen
pixi run dry-run --no-panel          # without the hardware panel
pixi run clear estop --sim           # a person clears the e-stop a test left set
```

Which window is which screen, and why no RViz opens:
[RViz and ours](hud.md#rviz-and-ours) (since 23 Nov the hardware panel opens by default).

| It opens | What it is | For |
|---|---|---|
| the simulator | `sim.launch.py`, no window of its own; its lines go to `/tmp/aav-dry-run/sim-launch.log` | the car |
| the simulator's picture | the developer window, live (`pixi run dev-window --live`) | watching the car while a test runs |
| the ladder window | below | running the tests, reading the results |
| `--panel` | the hardware panel (`pixi run hw-panel`) | every sensor's number; the buttons |
| `--debug` | the debug screen (`pixi run debug-hud`) | the lidar, the localizers |

Ctrl-C, or closing the ladder window, stops every process group it started; it says
"nothing is left running", or names what is. It refuses to start beside another simulator,
or anything else that announces on `/system/source`.

**The world it picks.** The tests' own simulators differ (the stands test takes the road
world with cameras; the hallway and the lot an empty world without). The dry run needs one
world for all of them:

| Switch | Why |
|---|---|
| `world:=road` | the stands test waits for a camera's picture, and its faults break the camera and the lidar: both need something to see; the picture shows a road |
| `nav:=false` | nothing else drives, as in every actuator test's simulator |
| `faults:=true` | the stands test's step 5 breaks sensors on purpose |
| `sim_speed:=3` | ego's cap, for the lot's tests at 2 and 3 m/s |
| `start_x:=-150`, `geofence_radius:=340` | nobody turns the simulated car round: every pass drives on from where the last one stopped. 340 m of flat ground ahead; the fence ends 10 m before the ground does |

Any `sim.launch.py` switch can follow the command (`pixi run dry-run world:=empty`); yours win.

| As on the car | In the dry run |
|---|---|
| the stands test's last step sets the e-stop, and it stays set | the same. The next test is **refused** until a person clears it: `pixi run clear estop --sim` (`--sim`: this computer only) |
| a person puts the car on stands, and takes it off | the runner does: up for phase A, down after; down first for the hallway and the lot. The window's switch does it by hand |
| a person puts the car back on the start line | nobody does: after about 340 m of passes the geofence stops the test. Stop the dry run and start it again |
| the heartbeat fault: a cable pulled | the runner pauses the output mux of **this** simulator (the dry run tells it which process group is its own) |

**Tested, 22 Nov, Ben's MacBook.** See [what can be dry-run](#what-can-be-dry-run-from-the-screen).

### The window: `pixi run ladder-window`

```bash
pixi run ladder-window               # the ladder, in a window (the first time: cargo builds Bevy)
pixi run ladder-window --pretend     # sample data: nothing runs, nothing is recorded
pixi run dry-run                     # the window beside a simulator you can watch (above)
```

![The test ladder's window beside a running Ravensim: one stands test's result and its graph](img/ladder-window.png)

The picture is the window's own screenshot, 22 Nov, beside the dry run's simulator, after
`steer-sweep` was run alone. (Taken before the **Show** switch was added.)

| Part | Shows | Does |
|---|---|---|
| left: the ladder | the top rung at the top; **Simulator** and **The car** side by side; each cell in words: PASS or FAIL with the date, not run, shut | click a rung; hover a shut cell for why |
| middle: the rung | what it measures, when it passes, the last result (date, commit, report, override), what shuts it | |
| **Run in the simulator** | on only when the simulator's side is open and has a command; else it says why. Under it: "Runs in the simulator that is open", or "starts its own, with no picture" | `pixi run ladder run <rung>`; one run at a time |
| **Its tests, one at a time** | the rung's own tests (from `car-test`'s list, not a copy), a **Run** each; simulator only | `pixi run ladder run <rung> --only <test>`; not recorded; the car goes on stands or to the ground by itself |
| right: **The last session** | while a run goes: **RUNNING NOW** and its step. **Show**: the newest run (a single test too), or the session the ladder recorded. Then that session as a table: step, PASS / FAIL / SKIPPED / NOT RUN, what happened, the `vehicle.yaml` lines it produced | click a step |
| the chosen step | one graph, **commanded (dashed) against measured (solid)** over time: speed, or the steering angle for the steering tests; axes labelled; the widest gap; hover reads both lines. Then the test's own numbers (stopping distance, lags, dead times) beside `vehicle.yaml`'s | |
| the session's fits (hallway) | each number as mean ± the passes' spread, n, and **trusted** or **NOT TRUSTED: why** | |
| the console | the run's last lines, "running..." with the step, how it ended | **Stop** ends the run and all it started |
| **The car** | its command as text, and "A person starts a car test from the terminal" | **Copy** only |
| **Record by hand** | pass or fail, simulator or car, a note, an override reason | `pixi run ladder record ...`; a refusal is shown in the tool's words |
| top: **The simulated car** | "on the ground" or "ON STANDS", asked every 5 s; off, with the reason, unless a simulator runs on this computer | **Put it on stands** / **Lower it to the ground**: `pixi run jacks up --sim` or `down`; Ravensim's refusal (wheels still spinning) is shown in its words |

| Rule | |
|---|---|
| The window never moves the car | it has no control that starts a car test; every run it starts is the tool's simulator run (`--sim`); the jacks switch is the simulator's only (`--sim`, `--local`, refused when `/system/source` says REAL) |
| The tool is the only writer, and the only reader of reports | the window reads `ladder --json` every 2 s and `ladder session <rung> --json` when a rung is chosen or a run ends; every rule stays in `tools/ladder.py`, the report's files are read by `tools/ladder_report.py` |
| Colour is never the only signal | every result is a word; the graph's lines differ by their dashes, and the legend says which is which |
| Closing the window stops its run | a simulator it started is not left behind; one it attached to stays, as its owner's |

The code: `ros/hud/aav_dev_window` (`src/ladder_window/`, and the logic in `crates/ladder-core`).

## What can be dry-run from the screen

Ben, 22 Nov: "so I can figure out if we are testing everything". Filled from what was
built and **run on 22 Nov** (Ben's MacBook, `pixi run dry-run --panel --debug`, the tests
started by the ladder tool's own commands, the ones the window's buttons run). "From the
screen" = the ladder window beside the simulator you can see.

**The session cards** ([the testing plan](testing-plan.md#the-sessions-as-cards)):

| Card | From the screen? | What ran, and what is missing |
|---|---|---|
| 0: after the install | 🟡 partly | the **bench** rung runs `bench-check` (its own stand-in, no picture; not run tonight). The boxes on the real boards have no twin |
| 1: the boot self-test | ❌ no | no rung runs `pixi run check` or reads `/safety/self_test/status`. The stands preflight counts every sensor's rate; the e-stop press is stands step 5h |
| 2: stands (phase A) | ✅ yes, 12 of 14 steps | **ran: 12 PASS, 2 SKIPPED**, the heartbeat fault included. Skipped: a Pi's link, a cable (no Pi, no cables). Not there: the steering's ends on the hardware panel (B1-B9), the wheel watcher with a tachometer |
| 3: map the hallway | ❌ no | `record-drive --sim` starts its own simulator from a terminal; no rung, no button |
| 4: the hallway, empty (phase B) | ✅ yes | **ran: 12 of 12 passes, the fitted model matched all 12**. Not as the car: nobody turns the car round (no slope from both ways), no NDT, the "hallway" is open ground |
| 5: the hallway, a person aboard | 🟡 partly, not run | `pixi run dry-run payload:=80` then the hallway rung should do it; nothing records the weight (TODO 257) |
| 6: the lot | 🟡 partly | the rung and each test run from the screen. **ran alone: `circles`, `straight`, `coast` PASS; `speed-up`, `brake`, `throttle-map` stop themselves above 1 m/s** (the runaway rule, TODO 275), so the whole rung fails today. 23 Nov: the cause is the `default` speed loop's known overshoot and the tests' short hold; with `speed_loop:=smooth` the rung passes 6 of 6. Nothing changed: [Ben decides](actuator-tests.md#why-it-fails-23-nov) |
| 7: a campus mapping drive | ❌ no | a person drives it; `record-drive --sim` is a terminal's |
| 8: offline processing | ❌ no | no car and no simulator in it: laptops, hours |
| 9: the car drives itself | 🟡 partly | the **route** rung runs one scored drive round the test lot through the whole stack (**ran: every goal reached, exit 0**), in its own simulator with **no picture**, not on the campus, not on a map of ours |

**Every test** ([Actuator tests](actuator-tests.md#the-tests)):

| Test | From the screen? | Ran 22 Nov, attached to the simulator in view |
|---|---|---|
| `preflight`, the sensors alone | ✅ | PASS, in the stands session |
| `steer-sweep`, `steer-rate`, `steer-centre` | ✅ | PASS, in the stands session; `steer-sweep` alone too, with its graph |
| `wheel-steps` | ✅ | PASS |
| the faults: heartbeat, GPS, IMU, camera, lidar, e-stop | ✅ | PASS. The camera's passes on FROZEN, not on lost (TODO 274) |
| the faults: a Pi's link, a cable | ❌ | no Pi and no cables in Ravensim |
| `hall-coast`, `hall-speed-up`, `hall-brake`, `hall-steer` | ✅ | 3 passes each, all PASS; `hall-brake` alone too |
| `coast`, `circles`, `straight` | ✅ | PASS, each alone |
| `speed-up`, `brake`, `throttle-map` | 🟡 they run, and stop themselves | FAIL: "the speed rose ... while the command fell" at 3, 2 and 1.5 m/s. Up to 1 m/s, the real car's cap, they pass |
| hold a speed, a steering step on the ground, a lane change | ❌ | not built (TODO 266) |

| Not run by a person yet | |
|---|---|
| the window's buttons | every command above was typed as the tool's command (`pixi run ladder run <rung> [--only <test>]`), with the windows open; a click on **Run** starts the same command (the window's tests cover that), but nobody has clicked it beside a simulator |
| the picture while a test runs | the developer window was open; nobody watched it |

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
| 6 | ~~The lot and route rungs have no one-command simulator twin~~ built 22 Nov: `car-test --lot --sim`, `measure controllers --tours lot`. Left: the lot's twin does not judge the 10 % / 0.2 m rule; the route's is scored by the league's table, not by `score_drive.py` on a recording, and runs with no picture | 273 |
| 7 | ~~One command for the screens side by side~~ built 22 Nov: `pixi run dry-run` | 273 |
| 8 | The hardware panel and the debug screen do not show "on stands" (the ladder window and `pixi run jacks status --sim` do) | 273 |
| 10 | The stage-4 tests stop themselves above 1 m/s in Ravensim (the runaway rule): the lot cannot pass at 2 to 3 m/s. The cause is known (23 Nov: the `default` speed loop's overshoot, and a hold shorter than the rule's window); the choice is Ben's | 275 |
| 11 | Nobody turns the simulated car round or puts it back on the start line: a dry run has 340 m of passes, then needs a restart | 273 |
| 9 | On stands the simulator's free-wheel mass (20 kg) and drag are guesses; its odometry reads the rear wheels, today's car the front | 49b, and a stands measurement |
