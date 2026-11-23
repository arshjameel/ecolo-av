# TODO

The current work list, grouped by topic. One line a row; the details live in the linked
pages. Tick items off as they land; each tick = a commit.

- Ben's messages, word for word: `docs/history/messages-*.md`.
- What is finished: [todo-done](docs/history/todo-done.md). The whole list as it was before
  this reorganisation (19 Nov), word for word:
  [there](docs/history/todo-done.md#the-whole-list-as-it-was-on-19-nov-before-the-reorganisation).

Status: ⬜ not started · 🔧 started · 🟡 built, a challenger or still being scored ·
👀 watching · ✅ done (then moved to todo-done).

## What is next

Ben, 19 Nov: "porting all the code and writing all the features left and then running the
tests too. liek th epixi test that is. Then we can start doing like algo testing like
diffused and stuff. start getting an idea of how the platform will work with everything
done". So: **build first** (each batch with `tools/test.sh` and `pixi run test`, no sim
scoring), **then score** the algorithms together. One building helper at a time; tests in
the background; each merged with a real merge (Ben's history stays).

### Stage 1: build everything left

| Batch | What | Rows |
|---|---|---|
| 1 | ✅ ONNX with Core ML (merged 19 Nov) | 243 |
| 2 | ✅ Lanes into the costmap (merged 19 Nov) | 264 |
| 3 | ✅ Before the car (merged 19 Nov) | 261, 258, 255 |
| 4 | ✅ Maps offline (merged 19 Nov) | 259, 260, 250 |
| 5 | ✅ Labels and the twin (merged 19 Nov) | 252, 253 |
| 6 | ✅ Driving (merged 19 Nov) | 174, 152, 219 |
| 7 | ✅ The last Python nodes to Rust (merged 19 Nov) | Rust first |
| 8 | ✅ The challengers built (merged 19 Nov) | 212, 214-217 |
| 9 | 🧪 **Clean-up** (20 Nov): the last of Nav2 (232), Ravensim's old names (241), the leftovers checked (242); the Python twins of gps_heading, localization_health, behaviour, things and aav_safety's five nodes removed (`save/python-*` tags); `docs/codemap.md`. Left: the Python ego, io_mux, lidar, camera_stream, rc, drive_to_point, stop_sign and lane_finder nodes (their launch files, configs or tests still lean on them), and 176 | 232, 241, 242, 176 |
| test | ✅ **One full `pixi run test`** on everything: 2,185 tests, 0 failures (20 Nov); `main` pushed | |

### Stage 2: score the algorithms together

The league (`measure controllers`, `measure onnx`, `tools/lane_score.py`) on the lot, the campus and
the road world, 3 runs each: diffusion, Hybrid-A* and the smooth lattice again (with ours),
the lanes in the costmap (`lanes:=camera`) against the old followers' numbers (`measure controllers` on a lane road, then `tools/lane_score.py` on its results: `measure lane` went with the followers), NDT leading on the campus,
AMCL, the challengers of batch 8. Scores are tracked, not gates; what clearly won't work is
stripped (tagged `save/` first, its scores kept).

✅ Run 20 Nov, overnight: [the scores](docs/league-results-2026-11-20.md). `mppi_lag_1k` is
ours' steering now; diffusion from Rust plans every time (the Python worker on Core ML
barely); `path:=lane` keeps right 100 %. Left: `sim-check --moving` fails (being fixed),
AMCL on the campus (no 2D map), the Orin's times.

### Waits for the car

The car is being upgraded (18 Nov): phases 0, A-D of [the testing plan](docs/testing-plan.md)
(246-250, 173).

## 1. The testing plan

[The testing plan](docs/testing-plan.md) (`docs/html/aav-testing-plan.pdf`, for the team):
the testers' strategy, session by session (rewritten 19 Nov). Ben, 18 Nov: "My prof wanted a
full automated testing system"; 19 Nov: "Add offline processing to the list of stuff we can
and will do".

| # | Task | Status |
|---|---|---|
| 254 | **Phase 0, after the install**: [one page of checks](docs/after-install.md) for each laptop, each car computer, then the car together, before phase A | ✅ written 19 Nov (`aav-after-install.pdf`); ⬜ used on the car |
| 246 | **Phase A, on stands**: `pixi run car-test --stands`: every sensor, steering, throttle on free wheels, faults on purpose (the Orin lost, a sensor stopped, the e-stop) | ✅ built, passes in Ravensim; ⬜ the car |
| 247 | **The debug screen**: who is driving, what each asks, what ego sends, the gate and why (key D on the map) | ✅ built; ⬜ seen on a screen; ⬜ the car |
| 248 | **Phase B, the hallway**: speed-up, coast, brake, steering; IMU blended with the wheels; each run over the model, with error bars | ✅ built; ⬜ the car. Open: 0.05 m/s wheel noise trips the runaway rule (a person decides) |
| 249 | **Phase C, recording**: `pixi run record-drive`, every raw sensor at full rate | ✅ built; ⬜ the car. **87 GB/h**: a big SSD (tell the hardware lead) |
| 250 | **Phase D, maps from drives**: `pixi run map-drive` (KISS-SLAM): NDT's map, a 2D map, 64 m chunks; drives merged | ✅ built, 2.3 cm off the truth in Ravensim; ✅ 3+ drives and the 2D maps merged (`--merge`, `map-area`, 19 Nov); ⬜ the car, loop closures on a real place |
| 251 | **Phase E, the map editor**: [`pixi run map-edit <drive>/map`](docs/map-editor.md): layers (OpenStreetMap, our points, the 2D map, chunks, labels, the path), align, box-delete, label, undo, approve into a new folder | ✅ built 19 Nov; NDT on an edited sim map 3.6-4.9 cm; ⬜ seen on a screen; ⬜ a real drive |
| 252 | **Phase F, labelling**: open-vocabulary models offline, camera boxes onto lidar points, fixed in the editor, our small model retrained after each drive | 🟡 built 19 Nov (batch 5, [Labelling](docs/labelling.md)): `label-drive` (OWLv2, Apache-2.0, on mps) → `labels.yaml` (`source: camera`, `seen_in`) → `map-edit` → `label-train` (a point segmenter, ONNX, Core ML 0.6 ms); Ravensim campus lots P4, P5: labels 73 % right, 50-77 % found. ⬜ a real drive (camera-lidar calibration, TODO 156) |
| 253 | **Phase G, the campus twin** in Ravensim from our maps; passes when a replayed drive scores within NDT's error of the real one | 🟡 built 19 Nov (batch 5): `pixi run twin-world <map>`, `sim world:=<twin>.sdf` starts (lot P5's twin). ⬜ the pass test (stage 2) |
| 255 | **The boot self-test that refuses to arm** (Ben, 19 Nov: "make sure we have control over everything"): at start-up every sensor at rate, every actuator answering (the spine's self-test, 0b), every lock alive, the clocks; READY / NOT READY and why on the HUD; `spine_bridge` refuses `/spine/arm` until READY. Moves nothing | 🟡 built 19 Nov; untested on the car: `aav_self_test_rs` next to the mux, `/safety/self_test/status`; `spine_bridge require_ready` (the car's) refuses to arm until READY. On the HUD (20 Nov): a small green READY, or a red NOT READY banner with why (hud-core `self_test`; unit-tested, untested on a real screen); with `actuators:=uart` nothing arms, so READY is for the person (card 1); 0b still weak |
| 256 | **NDT in the hallway, a third measure of distance**: fixed boxes and cones along the walls, the hallway mapped first (`record-drive` + `map-drive`), NDT's pose recorded by `car-test --hallway` beside the wheels and IMU; trusted across more than along (a plain corridor lets it slide) | ⬜ needs 261 |
| 257 | **The transfer functions with and without a person aboard**: `car-test --hallway --aboard <kg>` notes the load; a fit per load into `vehicle.yaml`; the adaptive brake seeded from both | ⬜ |
| 258 | **The adaptive brake on the car**: today it runs only in the sim's `sim_adapter` (`adaptive_brake:=true`); on the car nothing adapts. Put it on the car's brake path (`spine_bridge` or `spine-core`), never above `safe_max`, the locks never through it | 🟡 built 19 Nov; untested on the car: `legacy_uart_bridge` after its pi loop (`car adaptive_brake:=true`, off by default; needs `speed_loop.mode: pi`), the Rust crate `adaptive-brake` with the Python's vectors; not in `spine-core` yet (`actuators:=can` refuses it). docs/stopping.md, on the car |
| 259 | **Occupancy across passes**: 3+ drives of an area, each ray traced; a cell hit in some passes and free in others (a parked car, a person) removed; never-seen-free kept; then camera labels (252) for cars parked every time. Learn from Removert, ERASOR, DUFOMap (licences to check) | ✅ built 19 Nov (`tools/passes.py`; ideas only: Removert CC BY-NC-SA, ERASOR GPL-3.0, DUFOMap BSD-3); proved on 3 Ravensim drives, a parked box in one removed, the walls kept; ⬜ real drives; ⬜ camera labels (252) |
| 260 | **The offline pipeline as one command**: a campus area from its drives: `map-drive` each, merge, occupancy, chunks, one report | ✅ built 19 Nov: `pixi run map-area <area> <drives...>` into `maps/<area>/`, opens in `map-edit`; ⬜ a campus area |
| 261 | **NDT on the car**: `car.launch.py` has no `ndt:=` / `ndt_map:=` (only the sim launch has); needed for the hallway (256) and `pose_source:=ndt` (210) | 🟡 built 19 Nov; untested on the car: `ndt:=true ndt_map:=` (the Orin's `ndt` part, off by default); `pose_source:=ndt` only with both, wired as the sim |
| 266 | **Three step tests still missing** ([Proving it](docs/proving-it.md)): hold a speed (how far it wanders), a steering step on the ground (yaw rate: delay, overshoot, settling), a lane change (sway); each with `--sim`, the same report both ways | ⬜ |
| 267 | **One step-test harness for every path follower**: a sideways step and a speed step, the same for each `tracker:=`, one table (today only `mpc_lag_rs`'s own crate test has the sideways step) | ⬜ |
| 268 | `sim-check --ghost` listens for the local costmap on its old name (`/local_costmap/costmap`; ours is `/drivers/nav/costmap/local`), so the leg reports FAIL for every follower | ✅ 22 Nov: it reads the same name as the other legs (`LOCAL_COSTMAP`); run: PASS with `--tracker mpc_lag_rs` (0.29 m before it, the costmap 100 of 100). ⬜ with the default `mppi_lag` it still FAILs, twice of two, for another reason: the gap reads `nan` (the car stands turned aside, so the ghost is not straight ahead in its scan) |
| 269 | **A run log**: every test run's commit, command, settings and result in one place, so a number can be traced | ⬜ |
| 270 | **The car's scores beside the simulator's** for the same route: the last step of the ladder against the first | ⬜ after the car drives |
| 271 | **The test ladder with saved progress** (Ben, 22 Nov: "spin up the testing and then start working through the rungs saving our progress as we go"): `pixi run ladder` ([Proving it](docs/proving-it.md)); then a window for it, `pixi run ladder-window` (its own program in `aav_dev_window`, asking the tool; it starts simulator runs only) | ✅ the tool, 22 Nov; ✅ the window, 22 Nov: its tests pass (`ladder-core` 38, the program's 4); opened on a Mac with `--pretend` and on the real tool with `AAV_LADDER_FILE` in `/tmp`, seen by its own screenshot. ⬜ untested: a press of Run, Stop, Record or Copy by a person (the tests press them against a stand-in tool); Linux |
| 272 | **The car on jacks in Ravensim** (Ben, 22 Nov: "put the car on jacks and take it off jacks type thing"): a switch that lifts the simulated car, so the wheels spin and steer and the car stays put, as on stands; `car-test --stands --sim` then tests what the real stands test does (today the simulated car crawls in arcs) | ✅ 22 Nov: `pixi run jacks up --sim`, `down`, `status`, and `sim on_stands:=true` ([how](docs/actuator-tests.md#the-simulated-car-on-stands-pixi-run-jacks)); lowering refused while the wheels spin; `car-test --stands --sim` runs on jacks and lowers at the end: 12 PASS, 2 SKIPPED, Ben's MacBook. Its first runs found two stands checks that a still car's noise failed (the GPS line, the IMU watch): fixed. ⬜ left: the free-wheel mass (20 kg) and drag are guesses; the odometry reads the rear wheels (`speed_wheels:=front` for today's car: TODO 49b); the car is not raised; the camera row 274; the hardware panel and debug screen do not show it; **untested on the car** |
| 273 | **Every rung of the ladder in the simulator, with the screens open** (Ben, 22 Nov: "do all the testing in simulation ... so I can figure out if we are testing everything"): Ravensim, the hardware panel, the debug screen and the ladder window side by side; the lot and route rungs get a one-command simulator twin (today: by hand) | 🟡 22 Nov. **Built and run in Ravensim** (Ben's MacBook): `pixi run dry-run` (Ravensim, its picture, the ladder window; `--panel`, `--debug`; Ctrl-C left nothing running); `car-test --sim --attach` tests the simulator that is open (stands 12 PASS 2 SKIPPED, hallway 12 of 12 passes, both through `ladder run`); the window shows the last session as a table with a graph and numbers, and a Run for each test; `car-test --lot --sim` and `measure controllers --tours lot` are the lot's and the route's twins; `pixi run clear <latch> --sim`; [what can be dry-run](docs/proving-it.md#what-can-be-dry-run-from-the-screen). ⬜ **Not done:** nobody has clicked the window's buttons beside a simulator (the commands were typed), and the final window (with the Show switch) has no screenshot (the screen had locked); the lot's twin fails today (TODO 275) and does not judge the 10 % / 0.2 m rule; the route's twin has no picture and is not scored by `score_drive.py`; nobody turns the simulated car round (340 m, then restart); cards 1, 3, 7 and 8 have no rung; "on stands" on the hardware panel and the debug screen |
| 274 | **Ravensim's cameras have no noise**: on jacks, with the wheels turning, every picture is the same, so the sensor watchdog says `camera FROZEN` (a real camera's noise never repeats). `car-test --stands --sim` step 5d then passes on that word, not on the lost camera. Give `aav_sim_cameras` a little pixel noise, or have 5d ask for `STALE` | ⬜ found 22 Nov |
| 275 | **The stage-4 tests stop themselves above 1 m/s in Ravensim**: `speed-up` (3 m/s), `brake` (2 m/s) and `throttle-map` (1.5 m/s) end on the runaway rule, "the speed rose 2.86 -> 3.04 m/s while the command fell (3.00 -> 0.00): a stuck throttle?". So the lot rung cannot pass at 2 to 3 m/s. The rule is a safety rule: unchanged | ✅ in Ravensim, 23 Nov. **Done 23 Nov (Ben chose: hold the top speed longer):** `speed-up`, `brake` and `throttle-map` now hold their top speed 2.5 s (`procedures.PLATEAU`) before the command falls; the runaway rule and the speed loop are unchanged. `car-test --lot --sim --no-manual` with the `default` loop: 6 of 6 PASS. Each run needs up to 7.5 m more room at 3 m/s. ⬜ the car |
| 276 | **Finish the port from RViz: our own top-down navigation view** (Ben, 23 Nov: "lets finish the port so we don't rely on something that pops up in a weird way like rviz does ... run ravensim on one screen and then a hardware screen on another and a third screen to set the tests") | 🟡 23 Nov. **Built:** the map panel's new layers (trail, local costmap, ghost, keep-out circle, stop zone's box and state, body, scale and grid; `nav-core`, 21 tests); `pixi run nav-view`; RViz opt-in everywhere (`rviz:=true`, `--rviz`); `dry-run`'s three screens; the research entries in the ladder window ([RViz and ours](docs/hud.md#rviz-and-ours)). **Run on Ben's MacBook:** `barrier-watch barrier --speed 3` with our view: its own picture shows the ghost, the circle, the trail and the plan; no `rviz2` process. ⬜ **Untested:** the stop zone's box and the local costmap seen live (only with pretend data: the drive had ended in the picture); `--side-by-side`'s two pictures (the drives ran, no RViz; the screen had locked, so the pictures are black); `pixi run demo` launched with the new nav screen; `dry-run`'s hardware panel and `--nav` (the dry run came up with no RViz and Ctrl-C left nothing running, but the screen had locked, so the raylib panel could not open its window); a press of a research Run; Linux. Left: `pixi run view` and `radar_rviz:=true` still use RViz (asked for by name); delete the `.rviz` files once the view has been used for a while |
