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

Ben, 18 Nov: "lets do all of them ... we will test in teh car sometime soon". One helper at
a time, each merged into `main` with a real merge (Ben's history stays), the full test
before each push. Scores are tracked, not gates (Ben, 19 Nov: "We can get worse if we
think there is room to improve").

| Order | Next step | Rows |
|---|---|---|
| 1 | Merge NDT leading, the GPS checking | 210 |
| 2 | **The map editor**: our SLAM map over the street map | 251 |
| 3 | The testing page for after everyone installs | 254 |
| 4 | The last of Nav2: `nav2_amcl` out, the one-option switches | 232 |
| 5 | The labelling loop, then the campus twin | 252, 253 |
| 6 | The planner faster; reversing in a narrow street | 152, 174 |
| 7 | Speed hunting; the backing legs' drift | 189, 163 |
| 8 | The challengers, then the Rust crates and a more Rust-like pass | below |
| waits | **The car** (being upgraded, 18 Nov): the stands, the hallway, a mapping drive | 246-250, 173 |

## 1. The testing plan

[The testing plan](docs/testing-plan.md) (`docs/html/aav-testing-plan.pdf`, for the team).
Ben, 18 Nov: "My prof wanted a full automated testing system".

| # | Task | Status |
|---|---|---|
| 254 | **After the install**: one page of checks for each laptop, the Orin and the Pis once they are upgraded, before phase A | ⬜ |
| 246 | **Phase A, on stands**: `pixi run car-test --stands`: every sensor, steering, throttle on free wheels, faults on purpose (the Orin lost, a sensor stopped, the e-stop) | ✅ built, passes in Ravensim; ⬜ the car |
| 247 | **The debug screen**: who is driving, what each asks, what ego sends, the gate and why (key D on the map) | ✅ built; ⬜ seen on a screen; ⬜ the car |
| 248 | **Phase B, the hallway**: speed-up, coast, brake, steering; IMU blended with the wheels; each run over the model, with error bars | ✅ built; ⬜ the car. Open: 0.05 m/s wheel noise trips the runaway rule (a person decides) |
| 249 | **Phase C, recording**: `pixi run record-drive`, every raw sensor at full rate | ✅ built; ⬜ the car. **87 GB/h**: a big SSD (tell the hardware lead) |
| 250 | **Phase D, maps from drives**: `pixi run map-drive` (KISS-SLAM): NDT's map, a 2D map, 64 m chunks; drives merged | ✅ built, 2.3 cm off the truth in Ravensim; ⬜ the car, loop closures on a real place, 3+ drives, the 2D maps merged |
| 251 | **Phase E, the map editor**: layers (OpenStreetMap, our points, the 2D map, chunks, labels), align, box-delete, relabel, approve, save | ⬜ next |
| 252 | **Phase F, labelling**: open-vocabulary models offline, camera boxes onto lidar points, fixed in the editor, our small model retrained after each drive | ⬜ |
| 253 | **Phase G, the campus twin** in Ravensim from our maps; passes when a replayed drive scores within NDT's error of the real one | ⬜ |

## 2. Maps and position

| # | Task | Status |
|---|---|---|
| 210 | **NDT leads, the GPS checks** (`pose_source:=ndt`): a failing GPS no longer moves or stops the car | 🟡 built; the lot scored (every leg with the GPS stopped, jumped or drifting); merging |
| 231 | **AMCL in Rust** (`aav_amcl_rs`, the default): NDT's recovery | 🟡 ties stock on the lot; left: the campus, more runs |
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
| 243 | **Our models from Rust** (`ort`): the detector done on a Mac (8 ms); left: TensorRT on the Orin (JetPack 7.2), the diffusion planner | 🟡 |

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
