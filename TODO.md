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
| 1 | **ONNX with Core ML**: `measure onnx`, the diffusion planner from Rust, FP16, TensorRT ready (in progress) | 243 |
| 2 | **Lanes into the costmap** (yellow vs white, kerb, right lane), then the lane followers removed | 264 |
| 3 | **Before the car**: NDT in `car.launch.py`, the adaptive brake on the car, the boot self-test that refuses to arm | 261, 258, 255 |
| 4 | **Maps offline**: occupancy across passes, the offline pipeline as one command, 3+ drives and the 2D maps merged | 259, 260, 250 |
| 5 | **Labels and the twin**: the labelling loop (open models offline, licences checked), the campus twin in Ravensim | 252, 253 |
| 6 | **Driving**: reversing in a narrow street and in `aav_track`; the planner's checks on every core and a 1000-sample `mppi_lag` option; behaviour slows for kerbs | 174, 152, 219 |
| 7 | **The last Python nodes to Rust**: the geofence, stop hold and goal leash (safety: shared vectors, both twins), the lane pose, the GPS heading, the street router; then the drivers (battery, radar) where worth it | Rust first |
| 8 | **The challengers built**: smooth MPPI, jerk-limited passing, ground ring by ring, Kinematic-ICP, a tracker per kind | 212, 214-217 |
| 9 | **Clean-up**: the last of Nav2, Ravensim's old names, the leftovers; a more Rust-like pass | 232, 241, 242, 176 |
| test | **One full `pixi run test`** on everything | |

### Stage 2: score the algorithms together

The league (`measure controllers`, `measure lane`, `measure onnx`) on the lot, the campus and
the road world, 3 runs each: diffusion, Hybrid-A* and the smooth lattice again (with ours),
the lanes in the costmap against the old followers' numbers, NDT leading on the campus,
AMCL, the challengers of batch 8. Scores are tracked, not gates; what clearly won't work is
stripped (tagged `save/` first, its scores kept).

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
| 250 | **Phase D, maps from drives**: `pixi run map-drive` (KISS-SLAM): NDT's map, a 2D map, 64 m chunks; drives merged | ✅ built, 2.3 cm off the truth in Ravensim; ⬜ the car, loop closures on a real place, 3+ drives, the 2D maps merged |
| 251 | **Phase E, the map editor**: [`pixi run map-edit <drive>/map`](docs/map-editor.md): layers (OpenStreetMap, our points, the 2D map, chunks, labels, the path), align, box-delete, label, undo, approve into a new folder | ✅ built 19 Nov; NDT on an edited sim map 3.6-4.9 cm; ⬜ seen on a screen; ⬜ a real drive |
| 252 | **Phase F, labelling**: open-vocabulary models offline, camera boxes onto lidar points, fixed in the editor, our small model retrained after each drive | ⬜ |
| 253 | **Phase G, the campus twin** in Ravensim from our maps; passes when a replayed drive scores within NDT's error of the real one | ⬜ |
| 255 | **The boot self-test that refuses to arm** (Ben, 19 Nov: "make sure we have control over everything"): at start-up every sensor at rate, every actuator answering (the spine's self-test, 0b), every lock alive, the clocks; READY / NOT READY and why on the HUD; `spine_bridge` refuses `/spine/arm` until READY. Moves nothing | ⬜ |
| 256 | **NDT in the hallway, a third measure of distance**: fixed boxes and cones along the walls, the hallway mapped first (`record-drive` + `map-drive`), NDT's pose recorded by `car-test --hallway` beside the wheels and IMU; trusted across more than along (a plain corridor lets it slide) | ⬜ needs 261 |
| 257 | **The transfer functions with and without a person aboard**: `car-test --hallway --aboard <kg>` notes the load; a fit per load into `vehicle.yaml`; the adaptive brake seeded from both | ⬜ |
| 258 | **The adaptive brake on the car**: today it runs only in the sim's `sim_adapter` (`adaptive_brake:=true`); on the car nothing adapts. Put it on the car's brake path (`spine_bridge` or `spine-core`), never above `safe_max`, the locks never through it | ⬜ |
| 259 | **Occupancy across passes**: 3+ drives of an area, each ray traced; a cell hit in some passes and free in others (a parked car, a person) removed; never-seen-free kept; then camera labels (252) for cars parked every time. Learn from Removert, ERASOR, DUFOMap (licences to check) | ⬜ after the campus NDT maps |
| 260 | **The offline pipeline as one command**: a campus area from its drives: `map-drive` each, merge, occupancy, chunks, one report | ⬜ after 259 |
| 261 | **NDT on the car**: `car.launch.py` has no `ndt:=` / `ndt_map:=` (only the sim launch has); needed for the hallway (256) and `pose_source:=ndt` (210) | ⬜ |

## 2. Maps and position

| # | Task | Status |
|---|---|---|
| 210 | **NDT leads, the GPS checks** (`pose_source:=ndt`): a failing GPS no longer moves or stops the car | 🟡 merged 19 Nov, a challenger: the lot scored (every leg with the GPS stopped, jumped or drifting); ⬜ the campus runs (stopped by the usage limit); the car refuses it until 261 and a real map |
| 231 | **AMCL in Rust** (`aav_amcl_rs`, the default): NDT's recovery | 🟡 ties stock on the lot; left: the campus, more runs |
| 263 | **Unknown map cells read back as free** (found by the map editor, 19 Nov): grey 205 is p = 0.196, under every map YAML's `free_thresh: 0.25`. `make_map.py` and `map-chunks` now write unknown as 128 (p = 0.498), as `map-drive` and the map editor do; `example_lot_amcl.pgm` remade (244,730 cells inside its obstacles had read as free; `example_lot.pgm` byte for byte the same); a test pins it | ✅ 19 Nov. AMCL on the remade map, the lot, 2 runs: 0.06-0.07 m mean a leg, worst 0.09-0.14 m (18 Nov: 0.05-0.09, worst 0.15-0.20); one run's follower aborted a leg (error 107: the follower, not AMCL) |
| 121 | **NDT's score floor on the campus** (1.5 there, 2.3 in the lot): how high does a wrong pose score? | ⬜ |
| 175 | **Three levels, fixed chunks**: routes, paths and moves on 64 m chunks | 🔧 left: `global_window:=192` scored on the campus; seen live |
| 191 | **Stops reset the drift**: wheels that read nothing mean the car stands | 🔧 left: the parked yaw creep, before and after (the car) |
| 94 | **Drive-to-point stops 0.61 m past B** (limit 0.5 m) | 🔧 left: more runs in Ravensim |

## 3. Driving

| # | Task | Status |
|---|---|---|
| 152 | **Our planner faster**: its checks on every core (rayon), then `mppi_lag` with 1000 samples; timed on the Orin | 🔧 |
| 174 | **Reverse everywhere**, through cusps; a narrow street; reversing in `aav_track` | 🔧 the lot's turn works |
| 189 | **Speed hunting**: one person crossing flipped behaviour 4 times; a 1 s hold built | 🔧 left: `sim-check --moving` again |
| 163 | **The sim car vs its command**: `speed_loop_model:=` built; the backing legs go 1 m off | 🔧 |
| 169 | **Followers on the road ahead's line** (the lane centre, `/world/road_ahead`) | 🔧 `mppi_lag` done; left: `mpc_lag`, the campus live |
| 219 | **Kerbs at speed**: found 4.7 m out (too late at 3 m/s), 0.10 m never; behaviour slows for them | ⬜ |
| 206 | **The pedal**: the stick as accelerator, brake and reverse in ego | 🔧 left: a joystick drive in the sim, tune `pedal_let_off`; `ai` as a challenger |
| 171 | **No switch leaves a gap**: respawn, age checks, the nodes' own tests | 🔧 |
| 159 | **The safety case**: one page per hazard | 🔧 left: a throttle stuck wide open; the spine in the loop |
| 207 | **Stuck steering and speed noticed** (the watchdog) | ✅ in the sim; ⬜ the car (needs a measured angle, Q2) |
| 262 | **The lane follower in Rust, and a better one** (Ben, 19 Nov: "We also need to port the lane following code.or find a better version."): `aav_lane_follower_rs`, the default (`lane_follower_impl:=python` falls back); challengers `lane_follower:=pure_pursuit`, `stanley` on the fitted lane centre, with the car's lag | ✅ ported: the Python's commands on 2,797 shared events; `measure lane`, 3 runs each: the Rust PID scores as the Python one; on Library Road pure pursuit 2.4 cm RMS against the PID's 3.9-4.1, 1 flip a minute against 187; on the straight it settles to 0.4-0.9 cm but comes back from 0.3 m off slower (8.4 cm RMS against 6.7). The PID stays the default. Left: pure pursuit's near look-ahead for a quicker return, then promote it; the car |
| 264 | **Lanes are perception, the costmap prices them, navigation drives** (Ben, 19 Nov: "put lanes in the cost map and then behaviour keep to the right side. then the motion planner can just cross the line when its go no other choice", then "drop there code all together ... curb, high cost, yllow line med cost, lright side low cost, car in the way high cost, person in the way high cost"): the lane finder tells yellow from white and the road's edge; the lanes layer (148, `lanes.rs`) is drawn from the camera's lines where there is no lane map: kerb and off-road high, the yellow line and the oncoming lane medium, the right lane's centre low, every lane cost below a wall's; cars and people stay walls (the obstacle layer). Scored on the same roads against the followers' numbers (262: PID 3.9 cm, pure pursuit 2.4 cm on Library Road). Then **the lane-follower drivers are removed** (the 2025-26 PID, its Rust port, pure pursuit, Stanley, the `lane_follower*` switches), tagged `save/` first; the lane finder and `LaneGeometry` stay | ⬜ after 243 |

## 4. Challengers

Each runs beside the champion until it wins on the same routes (Ben, 5 Nov). The ones
scored with Nav2's follower (177, 178) are scored again with ours.

| # | Challenger | Status |
|---|---|---|
| 178 | **Hybrid-A\*** in our planner (`planner_search:=hybrid`) | 🟡 7/8 legs vs the lattice's 8/8; score again |
| 177 | **Steering-rate-aware lattice moves** | 🟡 score again |
| 213 | **`mpc_lag_rs` that reverses and doesn't swing** (Autoware's MPC idea); was 132 | 🟡 follows closer, sways 2.5x |
| 212 | **Smooth MPPI** (sample the steering's change) | ⬜ |
| 214 | **Passing with a jerk limit** (Apollo, Autoware) | ⬜ |
| 215 | **The ground ring by ring** (Patchwork++), for kerbs | ⬜ |
| 216 | **Kinematic-ICP**: lidar odometry that knows the car can't slide sideways | ⬜ |
| 217 | **A tracker with a motion model per kind** (car, person) | ⬜ |
| 243 | **Our models from Rust** (`ort`): the detector (8 ms) and the diffusion planner's model (`diffusion_impl:=rust`, Core ML takes all 2,178 nodes, 9.7 ms a plan against the CPU's 87) from Rust on a Mac, both checked against PyTorch; `pixi run measure onnx` (answers, coverage, ms, FP16); `onnx-session` shared; TensorRT ready, untested ([ONNX](docs/onnx.md)). Left: the Orin (JetPack 7.2); diffusion's Ravensim scoring deferred (Ben, 19 Nov: algorithm testing after the ports) | 🟡 |
| 262b | **A learned lane finder**: Ultra-Fast-Lane-Detection v2 (MIT), trained by us on Ravensim's pictures (labelled from the lane map) and the car's (252), through ONNX (243), behind `/perception/lane/geometry`. Published weights are out: CULane is non-commercial, BDD100K, TuSimple and CurveLanes unclear ([survey](docs/perception.md#a-learned-lane-finder-the-survey-19-nov)) | ⬜ |

## 5. Rust

| # | Task | Status |
|---|---|---|
| 176 | **More Rust-like**: every crate against the Rust API Guidelines | 🔧 left: a few spine-core items (a reviewed safety change), one shared `CloudView` |
| 195 | **rayon**: chunks and costmaps done; left: the planner (152), the things | 🔧 |
| 197 | **Rerun** for viewing a drive: `pixi run -e rerun view-bag` | 🔧 Ben looks, keep or drop |

## 6. Clean-up

| # | Task | Status |
|---|---|---|
| 232 | **After Nav2**: `nav2_amcl` out of `pixi.toml`; the one-option switches (`costmap:=`, `navigator:=`, `smoother:=`, `controller:=`): keep as hooks or remove; `nav2.yaml`'s name | ⬜ |
| 242 | **Untried after Gazebo**: `stream_check.py`'s moved sign, `sim_frames.py`, `pixi run demo` with Ravensim | ⬜ |
| 241 | **Ravensim's old names out**: `simulator:=own`, `target:=gazebo`, `car:=gazebo`, ignored args | ⬜ |
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
