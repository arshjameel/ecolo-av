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

## 2. Maps and position

| # | Task | Status |
|---|---|---|
| 210 | **NDT leads, the GPS checks** (`pose_source:=ndt`): a failing GPS no longer moves or stops the car | 🟡 merged 19 Nov, a challenger: the lot scored (every leg with the GPS stopped, jumped or drifting); ⬜ the campus runs (stopped by the usage limit); the car refuses it until 261 and a real map |
| 231 | **AMCL in Rust** (`aav_amcl_rs`, the default): NDT's recovery | 🟡 ties stock on the lot; left: the campus, more runs |
| 121 | **NDT's score floor on the campus** (1.5 there, 2.3 in the lot): how high does a wrong pose score? | ⬜ |
| 175 | **Three levels, fixed chunks**: routes, paths and moves on 64 m chunks | 🔧 20 Nov, scored on the campus (3 runs each): `global_window:=192` ❌ 5 of 9 legs against 9 of 9. "Along the road" failed every run: the planner searches the whole 192 m window, runs out of time (error 207; 87 % CPU, 600 MB) and the car never moves. Kept off. Left: a route level that hands the planner one chunk's stretch at a time, then score it again |
| 191 | **Stops reset the drift**: wheels that read nothing mean the car stands | 🔧 left: the parked yaw creep, before and after (the car) |
| 94 | **Drive-to-point stops 0.61 m past B** (limit 0.5 m) | 🔧 left: more runs in Ravensim |

## 3. Driving

| # | Task | Status |
|---|---|---|
| 152 | **Our planner faster**: its checks on every core (rayon), then `mppi_lag` with 1000 samples; timed on the Orin | 🔧 built 19 Nov; scoring in stage 2: the planner 18-21 % faster, the same paths (rayon measured 7-20 times slower, dropped: `docs/own-planner.md`); `steer:=mppi_lag_1k`, 0.86 against 1.56 ms a tick; **stage 2: no column worse than `mppi_lag_calm`, half the work: `stack:=ours`' steering since 20 Nov** ([20 Nov](docs/league-results-2026-11-20.md)). Left: the Orin's times, then `stack:=car` |
| 174 | **Reverse everywhere**, through cusps; a narrow street; reversing in `aav_track` | 🔧 built 19 Nov; scoring in stage 2: `aav_track`'s trackers reverse (`gears.py`); a blocked narrow street backs out (planner scene, navigator test, `sim-check --blocked-street`); ✅ `--blocked-street` passes in stage 2 ([20 Nov](docs/league-results-2026-11-20.md)) |
| 189 | **Speed hunting**: one person crossing flipped behaviour 4 times; a 1 s hold built | ✅ 20 Nov: `sim-check --moving` failed twice (the walker's feet were kept as a kerb for ever, and the car stood before it); fixed in the voxels' step memory (`kerbs_only`), then ✅ passes: WAIT(moving) > GO > DRIVE > the goal |
| 163 | **The sim car vs its command**: `speed_loop_model:=` built; the backing legs go 1 m off | 🔧 |
| 169 | **Followers on the road ahead's line** (the lane centre, `/world/road_ahead`) | 🔧 `mppi_lag` done; left: `mpc_lag`, the campus live |
| 219 | **Kerbs at speed**: found 4.7 m out (too late at 3 m/s), 0.10 m never; behaviour slows for them | 🔧 built 19 Nov; scoring in stage 2: both behaviour twins slow for a kerb on the road ahead; a kerb under every beam is kept while the car stands before it. 0.10 m still never seen |
| 206 | **The pedal**: the stick as accelerator, brake and reverse in ego | 🔧 left: a joystick drive in the sim, tune `pedal_let_off`; `ai` as a challenger |
| 171 | **No switch leaves a gap**: respawn, age checks, the nodes' own tests | 🔧 |
| 159 | **The safety case**: one page per hazard | 🔧 left: a throttle stuck wide open; the spine in the loop |
| 207 | **Stuck steering and speed noticed** (the watchdog) | ✅ in the sim; ⬜ the car (needs a measured angle, Q2) |
| 264 | **Lanes are perception, the costmap prices them, navigation drives** (Ben, 19 Nov: "put lanes in the cost map and then behaviour keep to the right side. then the motion planner can just cross the line when its go no other choice", then "drop there code all together ... curb, high cost, yllow line med cost, lright side low cost, car in the way high cost, person in the way high cost"): the lane finder tells yellow from white and the road's edge; the lanes layer (148, `lanes.rs`) is drawn from the camera's lines where there is no lane map: kerb and off-road high, the yellow line and the oncoming lane medium, the right lane's centre low, every lane cost below a wall's; cars and people stay walls (the obstacle layer). Scored on the same roads against the followers' numbers (262: PID 3.9 cm, pure pursuit 2.4 cm on Library Road). Then **the lane-follower drivers are removed** (the 2025-26 PID, its Rust port, pure pursuit, Stanley, the `lane_follower*` switches), tagged `save/` first; the lane finder and `LaneGeometry` stay | ✅ built 19 Nov: both lane finders say each line's paint (`left_paint`, `right_paint`); `aav_costmap` `camera_lanes.rs` draws the lines with the lane map's own costs (`lanes::across`), `lanes:=camera` or `both` (the map wins where it has a road); the car's lane is the one it is in, a yellow right line puts it in the oncoming lane ([docs](docs/own-costmap.md#the-lanes-from-the-camera-19-nov-todo-264)). The followers removed (tag `save/lane-followers`, [scores](docs/history/lane-followers.md)). ⬜ stage 2: score `lanes:=camera` on the same roads |

## 4. Challengers

Each runs beside the champion until it wins on the same routes (Ben, 5 Nov). The ones
scored with Nav2's follower (177, 178) are scored again with ours.

Stage 1, batch 8 (19 Nov): 212, 214, 215, 216 and 217 built and unit-tested, each off by
default behind its league option ([switches](docs/switches.md#stage-1-batch-8-challengers-built-19-nov-scored-in-stage-2));
stage 2 scores them in Ravensim.

| # | Challenger | Status |
|---|---|---|
| 178 | **Hybrid-A\*** in our planner (`planner_search:=hybrid`) | 🟡 stage 2: 15/15 legs; the calmest on the box (sway 0.044 vs 0.218), a tie on the lot; one run changed gear 6 times ([20 Nov](docs/league-results-2026-11-20.md)) |
| 177 | **Steering-rate-aware lattice moves** | 🟡 stage 2: a tie on the lot, worse on the box; one run cut inside the turning radius (0.50 m) ([20 Nov](docs/league-results-2026-11-20.md)) |
| 213 | **`mpc_lag_rs` that reverses and doesn't swing** (Autoware's MPC idea); was 132 | 🟡 **reverses since 20 Nov** (Ben: "fix mpc to have reverse"): cut at cusps, mirrored, stands at each cusp ([README](ros/drivers/aav_mpc_lag/README.md#forward-and-back-todo-213)); lot tour `--reversing`, 1 run: 4/4 legs, 7.8 m backwards, sway 0.054. League 21 Nov (3 runs, [results](docs/league-results-2026-11-20.md#mpc-against-mppi-21-nov-todo-213)): 12/12 lot, 9/9 University, 1.6 ms a tick, sway 2x MPPI's. 🔧 the sway: the tube (Ben 21 Nov), tuned: **0.3 m the default**, University Drive's sway 0.251 to 0.160 (MPPI 0.132), every leg; the lot unchanged. ⬜ `r_rate`, the costmap in its cost |
| 212 | **Smooth MPPI** (sample the steering's change): `steer:=mppi_lag_smooth` (`mppi_lag_sampling:=smooth`) | 🟡 stage 2 against `mppi_lag_calm`: the calmest (sway 0.018 vs 0.025) but further off the path on the box (0.22 vs 0.14 m) ([20 Nov](docs/league-results-2026-11-20.md)) |
| 214 | **Passing with a jerk limit** (Apollo, Autoware): `path:=lane_jerk` (`lane_pass_shape:=jerk`) | 🟡 stage 2 against `path:=lane`: calmer on the box (0.108 vs 0.148), further off the path (0.22 vs 0.11 m); the tours have no pass ([20 Nov](docs/league-results-2026-11-20.md)) |
| 215 | **The ground ring by ring** (Patchwork++), for kerbs: `kerbs:=patchwork` (`ground:=patchwork`) | 🟡 stage 2 against `kerbs:=on`: better on most columns, more left-right changes on the lot (12.5 vs 10.1); real kerbs are its test ([20 Nov](docs/league-results-2026-11-20.md)) |
| 216 | **Kinematic-ICP**: lidar odometry that knows the car can't slide sideways: `pose:=kinematic_ekf` (`odom_lidar:=kinematic_icp`, `aav_kinematic_icp`) | 🟡 stage 2: no gain in the sim, worse off the path on the lot (no wheel slip there); kept for the car ([20 Nov](docs/league-results-2026-11-20.md)) |
| 217 | **A tracker with a motion model per kind** (car, person): `behaviour:=on_per_kind` (`things_tracker:=per_kind`), both twins | 🟡 stage 2 against `behaviour:=on`: better on the lot, worse on the box (sway 0.241 vs 0.204); nobody walks on these tours ([20 Nov](docs/league-results-2026-11-20.md)) |
| 243 | **Our models from Rust** (`ort`): the detector (8 ms) and the diffusion planner's model (`diffusion_impl:=rust`, Core ML takes all 2,178 nodes, 9.7 ms a plan against the CPU's 87) from Rust on a Mac, both checked against PyTorch; `pixi run measure onnx` (answers, coverage, ms, FP16); `onnx-session` shared; TensorRT ready, untested ([ONNX](docs/onnx.md)). Left: the Orin (JetPack 7.2); diffusion's Ravensim scoring deferred (Ben, 19 Nov: algorithm testing after the ports) | 🟡 |
| 262b | **A learned lane finder**: Ultra-Fast-Lane-Detection v2 (MIT), trained by us on Ravensim's pictures (labelled from the lane map) and the car's (252), through ONNX (243), behind `/perception/lane/geometry`. Published weights are out: CULane is non-commercial, BDD100K, TuSimple and CurveLanes unclear ([survey](docs/perception.md#a-learned-lane-finder-the-survey-19-nov)) | 🟡 built 19 Nov (`aav_lane_finder_learned`, `lane_finder_impl:=learned`): 7,472 Ravensim pictures; held out, the car's lines 1-4 cm median at 5-15 m; Core ML 2.4 ms, 1.2 ms FP16 on the Neural Engine; ⬜ driven and scored against the classic (stage 2); ⬜ the car's pictures (252); **merged 19 Nov**: 7,472 Ravensim pictures, ResNet-18 on mps, the car's lines 1.2-3.6 cm median, Core ML 2.4 ms, FP16 1.2 ms on the Neural Engine, `lane_finder_impl:=learned`. Its colour head (95 % right) wired into `left_paint`/`right_paint` (19 Nov, batch 5). **Left:** neighbour lines often wrong (3 m at the 90th %); driving scores (stage 2) |

## 5. Rust

| # | Task | Status |
|---|---|---|
| 176 | **More Rust-like**: every crate against the Rust API Guidelines | 🔧 left: a few spine-core items (a reviewed safety change). Done 20 Nov: **one shared `CloudView`**, the crate `aav_description/cloud-view`: `voxels` (and through it `lidar-scan`, `kinematic-icp`), `ndt-core`, `radar-clustering` (and `debug-core`, the dev window) and `things-core` read clouds through it, four copies now one, every vector the same. Not on it: three nodes that read x, y inline (`costmap-node`, `behaviour-node`, `stop-zone-node`: a safety lock, so a reviewed change) |
| 195 | **rayon**: chunks and costmaps done; the planner measured, no gain (152, 19 Nov); the things (20 Nov) | ✅ done 20 Nov (`things-core` `examples/timing.rs`, Ben's M4 Max, idle): the saved map's background on many cores: the campus `/map` (6400 x 6400) grown in **7.4 ms, not 112 ms** (growing row by row did a third of that on one core, rayon the rest); a chunk crossing's 3 x 3 read in **1.0 ms, not 2.5 ms** (worst 3.0, not 11.9). Answers the same (all vectors, a new cell-by-cell test). A cloud stays on one core: 0.15 ms for 700 cubes, grouping most of it; rayon there measured slower (0.16 to 0.20 ms; 0.005 to 0.054 ms on a small cloud), not kept |
| 265 | **Batch 7: the last on-car Python nodes in Rust** (Ben, 16 Nov: "lets remove python were possible") | 🔧 done (19 Nov), the Rust the default, the Python's answers to the bit: `geofence`, `stop_hold`, `goal_leash` (`aav_safety_rs`), `gps_heading` (`aav_gps_heading_rs`). Left: `lane_pose` (a trial, off by default: needs a TF buffer at the picture's stamp and the lane map's `roads_at` in Rust; first move `aav_behaviour_rs`'s `lanes` and `tf` into a shared crate), `street_router` (on demand, not on the safety path: HTTP to Valhalla or OSRM, the campus graph from OSM XML, 1,100 lines; a poor fit for now), the battery and radar drivers |
| 197 | **Rerun** for viewing a drive: `pixi run -e rerun view-bag` | 🔧 Ben looks, keep or drop |

## 6. Clean-up

| # | Task | Status |
|---|---|---|
| 242 | **Untried after Gazebo**: `stream_check.py`'s moved sign, `sim_frames.py`, `pixi run demo` with Ravensim | 🧪 20 Nov: checked at the import and argument level (Ravensim serves `spawn_entity`, `delete_entity`, `set_entity_state`; every launch switch they pass exists); `stream_check.py`'s words say Ravensim. Not run against a live sim |
| 23 | **The nightly sim job**: drop `continue-on-error` after a few green nights (the 240 fix) | 👀 |
| 114 | **The developer window** on a slower laptop | ⬜ |

## 7. Needs the car or the hardware lead

The steps: [car day](docs/car-day.md), [know the car](docs/know-the-car.md),
[car setup](docs/car-setup.md).

| # | Task |
|---|---|
| 173 | **Ours on the car**: the seven blockers (158), then driven, recorded and scored (157) |
| 158 | **The seven blockers**: the firmware timeout, the brake follows a wish, wheel speed seen, clocks checked... |
| — | **The upgrade** (under way 18 Nov): the Pis to Ubuntu 24.04, the Orin to JetPack 7.2, after a restore is proven |
| 49b, 101 | A speed sensor on the driven wheels; then the brake follows a wish (feed-forward + PI) |
| 46, 49 | The steering and brake position sensors (from the Jrks over I2C?) |
| 22o | Ackermann: both front wheels' real angles (Q2) |
| 14, 60, 61 | Weigh the car; the wheel diameter (roll 10 m); the tracks |
| 67, 79 | The steering actuator's ends and centre; then the brake and throttle |
| 52b, 27 | Refit the car model from the actuator tests; the `smooth` speed loop on the car |
| 104 | The car identifies itself (delays, lags, braking, error bars): phase B |
| 37c, 25, 156 | The cameras on the Orin and the Pi; calibrate them and the lidar (Q22) |
| 218 | Kerbs on the car: record by a real 0.15 m kerb, then `kerbs:=true` |
| 64, 100 | The first lot visit: the datum, the clocks, walk the fence, the site file |
| 120 | The campus map against a real drive (now through 250-253) |
| 68, 74 | The car-ready checklist; the first moves (a throttle cap, the joystick's numbers) |
| 75, 76 | The localization Pi's machine file; Python's speed on the Orin |
| 92, 126 | The people named (e-stop person, safety driver); the hardware debug plan |
| 71 | The hardware panel's actuator pages (wait for the two-way ESP32 link) |
| 0b | The spine's state machine follow-up (Arsh) |
| — | Flash the Rust Picos (wheel, sentinel); a spare Pico 2 and a 3.3 V CAN module |
| — | The Mac as the operator station: measure the car's WiFi |
| — | Tune `ekf_global` for the real GPS receiver (N7, Q18) |
| — | A read-only audit of the car's network; the photo checklist (`docs/car-photos.md`) |
| 65, 125 | The 2024-25 / 2025-26 teams' recordings: find them; rename their topics |

## 8. The team decides

| Question | Today |
|---|---|
| Reversing on long drives: cost more, or stay short? (122) | ours reverses where the plan says |
| The brake after boot, the steering on an e-stop, a brake on the RC gear switch | one constant each in `spine-core` |
| The runaway rule trips on 0.05 m/s wheel noise (248) | unchanged: a safety lock |
| The indoor geofence with `stands:=true` (hallway tests) | the hallway tests use their own distance fence |
| `ROS_DOMAIN_ID` for the car and every laptop | 0-101 safe; never above 232 |
| The spine's arming rules: boot latched, arm on a 0 → 1 edge | Proposed |
| The GPS driver | `ublox_gps`, Tentative |
| N3: `respawn` on device nodes? | none today |
| Rotate the leaked secrets (Q34); answer Q1-Q34 | open |

## 9. Ideas kept, not picked

| Idea | When |
|---|---|
| 108 Shadow mode: the drivers score themselves while a person drives | after real recordings |
| 96 A model builds our own algorithms from our recordings | after the car's data |
| 78 The talking model: explains, never drives | design first |
| Burn (Tracel, MIT/Apache: pure-Rust inference, ONNX import) as a challenger to `ort` (ONNX Runtime), for all-Rust builds; candle and tract noted (Ben, 19 Nov) | once the Orin's TensorRT numbers exist |
| Learned scan models for AMCL (IR-MCL's idea), Transformer MCL, Scan Context | after real maps (231's scan model is swappable) |
| zenoh (`rmw_zenoh`), if Fast DDS gives trouble on the car's WiFi (205) | if needed |
| kiddo nearest neighbours (200), once ICP or grouping comes to Rust | if needed |
| Learned state-space models (Mamba-style) of what the linear model misses | after hours of car logs |
| micro-ROS again | once the hardware is known |
| numpy above 2.3.5 in the `ml` environment | when coremltools passes 9.0 |
| "Two drive paths": the bridge refuses a second mux or heartbeat | later |
| A stop sign seen stays on the map | needs two rules first |
| The little robot's eyes on the HUD | later |

## For the 2027-28 team

Learned models: each needs training data and GPUs we don't have this year (Ben, 28 Oct):
learned BEV (BEVFusion, BEVFormer); one network for perception, prediction and planning
(UniAD); world models (GAIA-1) and vision-language driving (EMMA); predicting others' paths
(MTR, Wayformer); state-space models (Mamba).
