# TODO: what is done

The finished rows and the dated logs from `TODO.md`, moved here on 14 Nov (the whole car, step 8: "shrink the docs and TODO"). Each under the heading it had; nothing changed but the place.

## The whole list as it was on 19 Nov, before the reorganisation

Ben, 19 Nov: "lets clean this all up. remove the stale stuff or stuff that doesn't make
ssnce anymore and reorganzine". Every row and status below is word for word as it was;
`TODO.md` now keeps one line per row, grouped by topic.

### What is next

Ben, 18 Nov: "lets do all of them. People are upgrading the car now so lets just keep
going". The software left for 0.4, in this order; one helper at a time, each merged into
`main` with a real merge (no squash, no rebase: Ben's history stays), the full test
before each push. Done since the 15 Nov list: the sway (165, 168), lanes as soft costs
and `path:=lane` (148, swapped in 16 Nov), the loose ends (162, 164), faults on purpose
(159), Ravensim's cameras on a Mac (183).

| Order | Next step | Rows | 18 Nov |
|---|---|---|---|
| 1 | **The last of Nav2**: AMCL ported to Rust (a particle filter on the scan; the trial lost to NDT on 29 Oct, but every algorithm stays), then `nav2_amcl` out of `pixi.toml`; the one-option switches and the pages that say Nav2 runs | 231, 232 | 🟡 AMCL ported (`aav_amcl_rs`, the default; a tie with stock); the docs swept. Left: more runs, the campus, `nav2_amcl` out, the switches |
| 2 | **The sim car runs 14 % over its command** | 163 | 🔧 `speed_loop_model:=` built and scored: `firm` car with `smooth` models stops best, sways a little more; no default changed |
| 3 | **No switch leaves a gap**: reversing everywhere, respawn, age checks | 171 | 🔧 the watchdog reads our nodes, S3-S5 done; left: reversing in `aav_track`, node binary tests, car runs |
| 4 | **Ravensim's cameras on Linux**, and the nightly `sim` job's `obstacle: never stopped` (18 Nov): a flake or real | 240, 23 | 👀 the nightly `--obstacle` failure was the navigator's follow call: fixed; count CI nights |
| 5 | **The edges smoothed**: one start command per job, clear errors, a "start here" page; this file and the docs shrunk | 172, 160 | 🔧 18 Nov: the docs audit, status.md, this file shrunk ([audit](docs/history/audit-2026-11-18-docs.md)) |
| 6 | **Ours scored, 3 runs each**, the lot and the campus | 167 | ✅ 19 Nov: scored (the campus 9/9, sway 0.098; the lot 12/12, sway 0.024). Ben, 19 Nov: no Nav2 yardstick; the scores are tracked, and a change may score worse for a while if it opens room to improve |
| 7 | **Our planner faster** (1000 MPPI samples, then timed on the Orin); **reverse in a narrow street** | 152, 174 | ⬜ |
| 8 | The Rust crates to try (rayon, tracing first); the Rust made more Rust-like | 195-205, 176 | ⬜ |
| waits | The car (hardware lead; being upgraded 18 Nov); one car drive scored | 157, 158, 173 | ⬜ the car being upgraded |

### The testing plan (Ben, 18 Nov)

Ben, 18 Nov: "My prof wanted a full automated testing system"; the map screen as a debug
screen "were we can see the commands being sent and maybe by who". The whole plan:
[The testing plan](docs/testing-plan.md) (`docs/html/aav-testing-plan.pdf`, for the team).

| # | Task | Status |
|---|---|---|
| 246 | **Phase A: `pixi run car-test --stands`**: the preflight, every sensor, the steering tests, throttle on free wheels, then faults on purpose (the Orin's heartbeat stopped, each sensor driver stopped, a Pi's link, the e-stop, a pulled cable when told), one report | ✅ built 19 Nov, run against Ravensim (`--sim --no-manual`); ⬜ the car. Found: the preflight's last-resort e-stop raised instead of pressing (fixed); the watchdog has no lidar input (step 5e fails on the car until it has one). **Fixed 19 Nov:** the watchdog's `lidar` input (STALE 1 s, FROZEN while moving, stops as the camera; both twins, shared vectors), added only where a lidar runs; Ravensim's `fault.lidar` (`sim_lidar_relay`, `faults:=true`); `sim-check --faults` 15 of 15 with `lidar stop`; `car-test --stands --sim --no-manual` 5e PASS (1.12 s) |
| 247 | **The debug screen** on the map window: who is driving (a new topic: the command mux's current driver), what it asks, what ego sends, the output gate and why, what the car did | ✅ built 19 Nov: `command_mux_state` (`aav_mux_state`, Rust: twist_mux's files and rule) on `/control/command_mux/state`; the map window's debug page (key D, `--debug-page`, pretend data in `--demo`); DRIVING in the developer window's strip. In the sim it named keyboard, navigator, takeover and hand-back, agreeing with twist_mux's output (226 of 227; the one was the hand-back's instant). ⬜ the page seen on a screen (none free here); ⬜ the car |
| 248 | **Phase B: the transfer functions in the hallway**: `speed-up`, `coast`, `brake`, steering on the ground; the IMU's acceleration blended with the wheel speed; each run laid over the model's (10 % speed, 0.2 m stop); with error bars (TODO 104) | ✅ built 19 Nov: `pixi run car-test --hallway` (`hallway.py`, `fusion.py`, `errorbars.py`); known numbers found within their error bars in Ravensim (clean, and 0.02 m/s wheel noise) and the car model (0.05 m/s noise, slip, IMU bias); ⬜ the car. Found: wheel noise 0.05 m/s trips `safety.runaway` (unchanged, a person decides); 15 m+ hallway for a trusted drag |
| 249 | **Phase C: the recording kit**: `record.sh` full mode for mapping drives (raw lidar, IMU at full rate, wheels, raw GPS, every camera), clocks checked first, ~40 GB/h | ✅ built 19 Nov: `pixi run record-drive` (`tools/record.sh --mapping`; [Recording a mapping drive](docs/recording.md)): every raw sensor at full rate, the commands, who drives, the watchdog; MCAP + zstd, 2 GB / 5 min files; checks first (disk vs GB/h, rates, stamps, `chronyc` here and over ssh, `/tf_static`'s frames), a summary after with "good for" per job; `drive.yaml`, `vehicle.yaml`, machine files beside it. Estimate **87 GB/h** before zstd (cameras 49, lidar 37), not 40. `--sim` in Ravensim: 102 s, 245 MB, all four jobs good, no gaps; its NDT map built, but blurred from the EKF's poses (median 33 cm off the truth's map with the GPS, 16 cm from wheels + IMU): phase D's SLAM is needed. **Untested:** the car, the clocks across real computers |
| 250 | **Phase D: maps from hand drives**: KISS-SLAM offline (check its licence), NDT map and a 2D map in chunks, several drives merged (keep what is there every time); then NDT as the main position (TODO 210), AMCL as recovery (231) | 🔧 19 Nov: `pixi run map-drive bags/<drive>` ([Maps from drives](docs/maps-from-drives.md)), its own env `slam` (x86_64 Linux, Apple-silicon Macs; no wheel for the Orin). KISS-SLAM (MIT; its wheel holds g2o, BSD, and SuiteSparse's CHOLMOD, LGPL-2.1+: an offline tool, never on the car), MapClosures (MIT), Open3D (MIT, conda-forge), mcap (MIT): the bag read with no ROS. The path flattened to x, y, yaw, fitted to the good GPS fixes (else the EKF's start pose, else local; the YAML says which); NDT's map, a 2D map (0.3-2 m occupied, the rays free, 0.25 m) and their 64 m chunks; `ndt-map build --poses-file`; `record-drive --sim --loop`; `measure controllers --ndt-map`. Ravensim's test lot, two drives (91 m, 208 m with a loop): the SLAM map **2.3 cm** off the truth's (median; 95 % 10-11 cm), the EKF's 27-33 cm (1.1-1.2 m); the 2D map's walls all within a cell of the truth's. NDT on the SLAM map: 3.5-4.7 cm off the truth a leg (worst 11 cm), every cloud a pose, no restarts (on the truth-placed map: 0.3-0.4 cm). `--merge`: two drives, ICP (x, y, yaw), only points both saw (3.1 cm off). **No loop closure found** on the plain lot (MapClosures needs features). ⬜ the car; loop closures on a real place; more than two drives; the 2D maps merged; then TODO 210, 231 |
| 251 | **Phase E: the map editor** (developer window): layers (OpenStreetMap, ours, labels), align, box-delete, relabel, approve, save the chunks | ⬜ |
| 252 | **Phase F: the labelling loop**: open-vocabulary models offline (check licences), camera boxes onto lidar points (156), fixed in the editor, our small model trained and scored after each drive; no non-commercial datasets | ⬜ |
| 253 | **Phase G: the campus twin** in Ravensim from the labelled maps; passes when a replayed drive scores within NDT's error of the real one | ⬜ |

### Ben, 17 Nov: ready to install by Wednesday

| # | Task | Status |
|---|---|---|

### Found 17 Nov (Nav2 and Gazebo deleted)

| # | Task | Status |
|---|---|---|
| 231 | **AMCL, the last Nav2 node**: `amcl:=true` still runs stock `nav2_amcl` and its lifecycle manager (kept in `pixi.toml`). Port it (a particle filter on the scan, Rust), or drop it for NDT (the closer trial) | 🟡 **18 Nov: ported** (`aav_amcl_rs`, Rust; `amcl_impl:=rust` the default, `amcl_impl:=stock` the fall-back; `nav2_amcl` still in `pixi.toml`). The same `amcl.yaml`, topics and frames; never TF; a plain node. Its scan model is a trait (`ScanModel`), so a learned one can be tried later. Ravensim, the lot, 3 runs each: AMCL off the truth per leg, stock 0.103 / 0.051 / 0.067 / 0.075 m, ours 0.089 / 0.053 / 0.060 / 0.076 m (worst 0.20 and 0.15 m): a tie ([trials](docs/history/trials-log.md#18-nov-amcl-in-rust-against-stock)). One earlier run of ours started badly (leg 1 0.27 m mean, 0.94 worst). Left: more runs; the campus; then `nav2_amcl` out of `pixi.toml` (232). Earlier: **18 Nov:** port it (Ben: "lets do all of them"). An AI-based AMCL (Ben heard of one): the nearest found is IR-MCL (PRBonn, 2023): a particle filter whose scan model is a neural occupancy field; 2D lidar, indoors, PyTorch on a GPU, **no licence stated** (so the idea only, credited, never its code). Our port keeps the scan model a swappable part, so a learned one can be tried later |
| 232 | **After Nav2: the leftovers**: `costmap:=`, `navigator:=`, `smoother:=` and `controller:=` now have one option each (keep as hooks for a challenger, or remove); `nav2.yaml` keeps Nav2's layout for our ports (rename its blocks?); about 80 doc pages still name Nav2, mostly as where a port came from or old scores (sweep the ones that say it runs) | ✅ **the docs**, 18 Nov ([audit](docs/history/audit-2026-11-18-docs.md)): every page in `docs/` checked; the ones that said Nav2 or Gazebo runs, the stock AMCL, `stack:=stock` or `/safety/collision_monitor/state` now say ours, and `interfaces.yaml`'s notes too; dated results, ports and credit keep the names. ⬜ the switches, `nav2.yaml`'s name, `nav2_amcl` out of `pixi.toml` (231) |
| 240 | **Ravensim's camera lockstep on Linux**: CI's nightly `sim` job (lavapipe, 4 cores) and oirp-spss. Expect the cameras at 10 Hz in sim time and the real-time factor below 1 (`real-time factor ... camera lockstep held the clock` in the launch log). Proven only on a Mac (17 Nov) | 👀 **18 Nov, CI's nightly `sim`:** the cameras do run in lockstep on Linux (front_right and rear at 10.0 Hz, the left 8.9 Hz) and every check passes but one: `--obstacle` with cameras "never stopped ... goal not reached 12.1 m on" (34 s of CPU), while the same check `--no-cameras` passed (0.39 m short, the stop zone first, the goal reached). Likely the check's waits are in wall time while the sim's clock waits for the pictures, so time runs out before the car reaches the box; the nightly fails every other night (13, 14, 16, 18 Nov). **Later, 18 Nov, from the job's logs:** not the check's timer (its 240 s wasn't reached; the leg lasted 34 s of wall time): the navigator aborted the goal 33 s after it began, "the follower failed (error 107): no word from the follower in 4.0 s" (`follow_timeout`, on the node's clock: sim time). Ravensim's real-time factor was 0.42-0.57, the lockstep holding the clock 126-135 times a 30 s window. So `mppi_lag` (2000 samples x 56 steps, 20 Hz) went 4 s of sim time without feedback on GitHub's 4 cores beside lavapipe. To find: why its feedback stalls (a starved executor, a slow tick, waiting on a costmap), and whether `follow_timeout` should allow for it **Night, 18 Nov, the cause and a fix:** the follower was alive: the car drove at 1.1 m/s right up to the abort (never stopped, so `mppi_lag` was ticking: it sends feedback on every tick that drives), and `mppi_lag` took the navigator's cancel 0.15 s after it. The 4 s of silence began about 10 s of sim time into the drive: the 10 s path renewal (`keep_path_age`), when the follower moves its feedback from the old goal to the new one. The navigator read a goal's feedback only once the follower's answer to it ("accepted") had come, so a late or lost answer made a driving follower look dead. rclrs 0.8 sends the goal request before it registers the answer's slot (`request_goal`), so an answer that comes in between is dropped without a word; not caught on this Mac (0 lost in 24,000 goals, 8 clients, under load), so not proven to be what happened on CI. Not the tick: 4 ms here, never a gap over 0.1 s of sim time in the feedback, with software GL, under 48 and 160 `yes` and on the 4 efficiency cores (real-time factor 0.33, 0.13, 0.1). **Fix** (`aav_navigator` `call.rs`): a goal's feedback counts from the moment it is asked for; feedback before the answer shows the follower took the goal, and its result is then asked for by the goal's id; a goal never answered is logged ("the follower's answer to a path never came"). `follow_timeout` stays 4 s; the stop zone and the watchdog are unchanged. Runs here after the fix: software GL + 48 `yes`, software GL, and plain: all pass (0.73, 0.82, 0.81 m from the box), the log line never. Also: 13, 14 and 16 Nov were Gazebo and Nav2 ("Nav2 goal not reached"), not this: 18 Nov is the only failure of ours so far. Left: count CI nights; if it comes back, the new log line says whether an answer was lost |
| 241 | **Ravensim's old names out after one release**: `simulator:=own`, `stack.launch.py target:=gazebo`, `spine_sim car:=gazebo`, the `gz::sim::systems::Sensors` marker in old campus worlds, `sim.launch.py gui:=` (ignored), the URDF's `sim`, `lidar` and `chase_camera` args (ignored) | ⬜ after 0.4-rc1 |
| 242 | **After Gazebo and Nav2: untried leftovers**: `tools/stream_check.py`'s moved sign (now delete and spawn through Ravensim's services), `tools/sim_frames.py`, `pixi run demo` with Ravensim; **`pixi run diffusion data` is broken** (`tools/diffusion_data.py` starts Nav2's `map_server` and `planner_server`, no longer installed: teach it ours); the screens still say Nav2 (the map window's "Outside Nav2's map" and "Nav2 is driving to the goal", the debug HUD's "Nav2's cubes", the dev window's "waiting for Nav2's map"): rename them with their tests | ✅ **the docs**, 17 Nov (Ben: "lets clean up all the documentation that has nav2 and gazebo"): every page, README and code comment that said how to run things now says Ravensim and our stack; `learn-nav2.md` is now *Learn: navigation*; Nav2 and Gazebo stay only as history, credit, old scores and the names we kept (`nav2_msgs` actions, `nav2.yaml`, `nav2_amcl`); dated pages carry a note with the as-it-was links. ✅ **`diffusion data`**, 17 Nov: our costmap and planner teach it, no ROS (TODO 230). ✅ **the screens**, 17 Nov: the map window says "the navigator" and "the map", the debug HUD "the costmap's cubes", the dev window "waiting for the map", the hardware panel and Foxglove's STOP label no Gazebo or Nav2 (topics and actions kept); their tests too. ⬜ the other leftovers above |

### Ben, 16 Nov

| # | Task | Status |
|---|---|---|
| 243 | **ONNX models from Rust** (Ben, 16 Nov): run our models (the detector, the diffusion planner, later the lane finder) from the Rust nodes with the `ort` crate (ONNX Runtime). Train in PyTorch, export ONNX; on the Mac first through CoreML's execution provider, then TensorRT's on the Orin. Check `ort`'s licence (MIT/Apache) and that its CoreML and TensorRT providers build under pixi; one model end to end, its time a frame against today's Python | 🟡 17 Nov: the detector, end to end ([ONNX from Rust](docs/onnx.md)): `tools/export_model.py --format onnx` (opset 17, 384 px); `detector-core` (Rust, `ort` 2.0.0-rc.15, MIT/Apache) gives the Python's boxes (IoU 0.9999); 8.0 ms a picture on Core ML (the GPU: the Neural Engine takes none) against PyTorch's 14.7; `aav_detector_rs` (`detector_impl:=rust`) passes `sim-check --perception`. The `ml` environment's numpy fixed (Python 3.12 pinned). Left: TensorRT on the Orin (compiles, untested: the doc's car-day steps), a half-precision export for the Neural Engine, the diffusion planner |
| 219 | **Kerbs at speed, and slowing for them** (TODO 119's gaps): a 0.15 m kerb is found only 4.7 m past the bumper (enough at 1 m/s, not at 3 m/s), a 0.10 m one never; the behaviour planner should slow for `/perception/voxels/steps` as for an obstacle, so the stop zone isn't the first to act; the 10 s memory forgets a kerb the car stood before longer | ⬜ |
| 218 | **Kerbs on the car** (TODO 119): record the VLP-16 by a real 0.15 m kerb, run the step finder on the bag (`steps_on_recorded.rs` style), then `kerbs:=true` in `car.launch.py` (not there yet) | ⬜ the car |
| 217 | **The things' tracker, a motion model per kind** (the league's survey: Fast-Poly / Poly-MOT, OC-SORT; MIT): a bicycle model for a car, constant speed for a person; a thing found again re-fits its speed from its last sightings. Ported into `aav_things`' tracker, a challenger to the one constant-velocity filter | ⬜ |
| 216 | **Kinematic-ICP's idea** (PRBonn, MIT): lidar odometry that uses the wheels and knows the car can't slide sideways; ported, the next challenger to `kiss_ekf` (pose level) | ⬜ |
| 215 | **Find the ground ring by ring** (Patchwork++, BSD-2): so a 0.3 m kerb stands out instead of one height for all (TODO 119); ported into the lidar or voxels node | ⬜ |
| 214 | **Passing with a jerk limit** (Apollo's piecewise-jerk path, Autoware's static obstacle avoidance; Apache-2.0): the pass's length from the speed and a lateral-jerk limit, the path a QP along the lane (Clarabel); a challenger to `pass.rs`'s shift | ⬜ |
| 213 | **`mpc_lag_rs` that reverses and doesn't swing** (Autoware's MPC lateral controller, Apache-2.0): a steering-rate cost and reverse gear. Scored 16 Nov ([page](docs/league-results-2026-11-16.md#steer-the-follower)): it follows 2-7 times closer than `mppi_lag_calm` at a fifth of the tick, but sways 2.5 times as much, with NDT too; the launch refuses it with reversing on | ⬜ |
| 212 | **Smooth MPPI in `mppi_lag`** (SMPPI / KMPPI, `pytorch_mppi`, MIT): sample the steering's change, not the steering, so the commands come out smooth without a bigger control cost; scored against `mppi_lag_calm` | ⬜ |
| 210 | **NDT into the EKF as ours' pose** (the league, 16 Nov: [the head-to-head](docs/league-results-2026-11-16.md#final-configuration)): `ndt_ekf` beat `ekf` on sway, off the path and the lane's centre on all four tours, 3 runs each (University Drive: sway 0.046 against 0.064, off the path 0.03 against 0.10 m), but not every column: a stop on the last university leg in 2 of 3 runs, one reverse at the box, more steering flips on the lot. Find those; then a fallback to `ekf` where a world has no NDT map (the launch refuses `ndt_ekf` there), then score again and swap; on the car, a lidar map of the real lot first | ⬜ |
| 209 | **CI speed** (16 Nov: Linux `pixi run test` took 45+ min). Measured by another session: the colcon build grew 1.5 times across untouched packages (2208 to 3426 s: a slower runner or a cold cache), and `aav_mppi_lag`'s colcon test is the slowest (261 to 332 s), its closed-loop runs also in `tools/test.sh` | 🔧 16 Nov: `test.yml`'s linux, arm and mac jobs cache Rust (`actions/cache`): cargo's downloads, `build/*/target` (colcon's release builds) and `tools/test.sh`'s `target` folders, keyed on every `Cargo.lock` and `pixi.lock`; inside `pixi run test` (`AAV_COLCON_TEST_FOLLOWS=1`) `tools/test.sh` leaves mppi_lag's closed-loop runs to colcon test (release), so they run once; alone it runs them too. No test removed. The YAML parses; **untested on CI**. Watch the next runs: the build's time, the cache's size (GitHub keeps 10 GB a repo), and a stale-cache link error (`-l<pkg>__rosidl_typesupport_c`): then delete the `rust-*` caches **18 Nov:** main's linux job after #30 timed out (60 min) in `aav_mppi_lag`'s colcon test: the Rust cache missed, so the build alone took 27 min, and `actions/cache` saves only when the job passes, so the next run would build cold again. Now in linux, arm and mac: the cache is restored, the build is its own step (45 min), the cache is saved right after it whatever happens later, then `pixi run test` gets its full 60 min (jobs 120 min). A failed build is annotated from `build.log`. **Untested on CI** |

### Ben's ideas, 15 Nov (day)

| # | Task | Status |
|---|---|---|
| 206 | **The pedal** (Ben, 15 Nov: "we had to swtich between acceleration, braking and reversing ... make that part of the ego"): a person's stick as accelerator and brake in ego (`pedal.py`): let off slows (let go: 1.0 m/s²), pulled back brakes by how far, then reverses. **Built 15 Nov**, 8 laptop tests; the e-stop, a stale command and a turn still brake fully. Live ego node with a pretend joystick, from 1 m/s: full back 2.0 m/s², 0 in 0.5 s, then reverse to -0.5; a little back 1.2 m/s², then -0.2; let go 1.0 m/s², stays stopped; let go with the e-stop: 2.0 m/s². Left: a joystick drive in the sim (feel), then on the car on stands; tune `pedal_let_off`; try `pedal_drivers` with `ai` as a challenger (Ben: "a similar concept for the ai too?") | 🔧 |

### Rust crates to try (15 Nov)

Ben, 15 Nov: "Would tokio the rust concurrency library help us ... are there other rust
libraries we could consider using", then "lets add all those crates to our todo list".
Each is a trial beside what we have; check its licence and credit it in
`docs/acknowledgements.md` when used. Licences below are to be confirmed.

| # | Crate | Where, and why | Status |
|---|---|---|---|
| 195 | **rayon** (MIT/Apache) | **the three levels' chunks in threads** (Ben, 15 Nov: "for the nav 3 layers processing each block could be done in threads and building the cost map"): each 64 m chunk's costmap, NDT cubes and things built on its own core, joined when ready (TODO 175); then our planner's checks (TODO 152), MPPI's futures (today threads by hand), voxels | 🔧 15 Nov: `map-chunks` `Store::update` loads each new chunk on its own core (rayon), so NDT's 3 x 3 (and later the planner's and the things') build together; the same chunks, map-chunks and ndt-core tests pass. Untimed: no NDT chunk map on this Mac yet (`pixi run campus` makes one). Next: the planner's and costmap's chunks once they load them (175). Timed 15 Nov, this Mac (M4 Max), NDT on the campus in our simulator: each new 3 x 3 (9 chunk files, rayon) read in 0-3 ms, the matcher rebuilt in 8-22 ms; the first 3 x 3 and matcher at start-up 0.02 s, against 2.5 s for the whole map (12.3 M points) 16 Nov (Ben: "why are map chunks and obstical avoidance not using rayon"): our costmap uses every core: a fixed costmap is worked out in 64 m tiles, each on its own core (the whole campus map 326 ms to 32 ms, criterion); our global costmap from chunks reads and inflates each chunk on its own core (25 chunks 14 ms to 2.0 ms; the car one chunk on 8 to 1.3 ms). On in `stack:=ours` (TODO 175). Left: the planner's checks (152), the things |
| 197 | **Rerun** (MIT/Apache) | view a recorded drive (MCAP) frame by frame. Ben, 15 Nov: "maybe if its better then what we already ahve": tried on the same recording beside Foxglove/Lichtblick and the dev window; kept only if it shows a bad drive faster. Not a replay into the nodes (`ros2 bag` stays) | 🔧 15 Nov: `pixi run -e rerun view-bag <drive>.mcap` (Rerun 0.43, its own environment, laptops only); it read our NDT recording (3 topics, 15,228 messages; `rerun mcap convert` in 0.15 s). Left: Ben looks at a drive in it beside Foxglove and the dev window, and says keep or drop |
| 198 | **tokio** (MIT) | ~~device links~~: **not now** (Ben, 15 Nov: "maybe not tokio if its more then just concurrency"). It is a whole async runtime, more than threads; rayon and std's threads cover the nav | ⛔ not planned |
| 200 | **kiddo** (MIT/Apache) | fast nearest neighbours: NDT, grouping the things, ICP | ⏸ 15 Nov, not needed yet: NDT's `neighbours` (ndt-core `map.rs`), the dev window's map match and the debug screen's cubes all look in a hashed grid (the point's cube and the 26 round it), already constant time and what voxel NDT is built on; a kd-tree wouldn't beat it. Grouping the things is Python, and there is no ICP. Take kiddo up when one of those comes to Rust |
| 205 | **zenoh** (`rmw_zenoh`; EPL-2.0/Apache) | ROS 2's newer transport, if Fast DDS keeps giving trouble on the car's WiFi | ⬜ if needed |

### The night of 14-15 November

Ben, 14 Nov night: "chiping througth the list 166-173 and then the 155, 156, 157, etc".

| # | Task | Status |
|---|---|---|
| 177 | **Steering-rate-aware lattice moves** (Ben, 14 Nov: a trial now): moves whose curvature changes no faster than the steering can, from the car model ego uses; beside today's lattice until it wins (no oscillation, 168) | 🟡 built; sim, 14 Nov night (the lot, 2 runs each, Nav2's MPPI following, reversing on): every version 8/8 legs. Our planner with today's moves swayed least (0.019 m/s², 11.7 left-right changes per 100 m), smooth moves 0.021 and 13.2, Nav2's lattice smooth 0.025 and 14.2; Nav2's lattice smooth was closest to the path (0.23 m against 0.26). No win: the champion stays; smooth moves stay a trial. Its gain: no plan tighter than the car's lock (Nav2's lattice planned 2.61 m and 3.19 m with today's moves). The oscillation's cause is elsewhere (168) |
| 178 | **Hybrid-A\* in our Rust planner** (Ben, 14 Nov: "Now"): a port of Nav2's, another `path` option in the league | 🟡 built (planner:=own planner_search:=hybrid). Sim, 15 Nov (the lot, 2 runs, Nav2's MPPI following, reversing on): 7/8 legs against our lattice's 8/8, 585 s against 393, sway 0.015 against 0.018 but 16.1 left-right changes per 100 m against 13.7; one turn back missed. A challenger; the lattice stays the champion |
| 184 | **Nav2 and Gazebo deleted**, once 0.4-rc1 is scored against them (their launch files, configs, trees, Gazebo worlds' plugins, the CI job), keeping their scores in the docs | 🟡 **Nav2 deleted 17 Nov** (Ben, 16 Nov: "Now"): its servers, trees, `stack:=stock`, `costmap:=nav2`, `navigator:=nav2`, `tracker:=none`, Nav2's planners and controllers, `full_costmap`, `stop_zone_shadow` and `sim-check --stop-zone-shadow`; `pixi.toml` keeps only `nav2_msgs`, `nav2_amcl` and its lifecycle manager (231). Its last scores: `docs/league-results-2026-11-16.md`; [Nav2, as it was](docs/history/nav2-as-it-was.md). Left: 230-232 (18 Nov: 230 done; 231 ported, `nav2_amcl` still in `pixi.toml`; 232 open). ✅ **Gazebo deleted 16 Nov** (Ben): `simulator:=gazebo`, `gazebo.launch.py`, the `gazebo_*` nodes (now `sim_*` in `aav_sim_models`, was `aav_gazebo`), the xacro files, the worlds' plugins, `ros_gz` in `pixi.toml`, CI's Gazebo step, `gz_crash.py`, `gz_window.sh`, `gazebo_steering.py`; its last scores in [Gazebo, as it was](docs/history/gazebo-as-it-was.md); `vendor/gazebo/` kept, to read. Old names (`own`, `target:=gazebo`, `car:=gazebo`) work for one release (241) |
| 185 | **Heavy Python to Rust** (Ben, 14 Nov: "convert anything that uses a lot of resources to rust"). Measured, our stack driving the road world: each Python node's own work (with `/clock` at 100 Hz) behaviour 10.7 %, voxels 10.5, ego 9.1, localization_health 8.5, look_ahead 7.7, sensor_watchdog 7.5; our Rust nodes about 2 % each. Order: voxels (the chunked maps' base, 175), ego + look_ahead as one node, behaviour, then the watchdog and health. Perception's models stay Python (the work is on the GPU); the Gazebo nodes go with Gazebo. Also: `/clock` at 1000 Hz cost half the simulator's CPU (all processes 336 % to 163 % at 100 Hz): `clock_hz:=` added; 250 the default once sim-check passes with it | 🔧 voxels: ✅ 15 Nov: the Python's answers on 9 recorded clouds and a whole turn (the plane within 1.2e-15). One whole turn (28,800 points): 4.29 ms Python, **0.53 ms** Rust. Our simulator's road world: the callback 6.8-7.5 ms → 1.4 ms; the node's CPU 25 % → 6 % of a core. ⬜ the car, then make it the default. ego + look_ahead: ✅ 15 Nov, `aav_ego_rs` (`ego_impl:=rust`), one Rust process with both nodes, a challenger: the Python modules' answers to 0 (the trace's f32 speed loop 4e-8); road world, `/clock` 100 Hz: **2.4 %** of a core against 17 % (ego 10.5, look_ahead 6.5), 1000 Hz: 4 % against 58 %; sim-check's scripted drive the same (5.83 m, 0.00 m to the side). ✅ 16 Nov the simulator's default (`SIM_DEFAULT_EGO_IMPL`; sim-check and two scoring pairs on lot and university_box no worse). ⬜ the car, then make it the car's default too. behaviour: ✅ 16 Nov, `aav_behaviour_rs` (`behaviour_impl:=rust`), a challenger: the Python modules' answers bit for bit (4,800 ticks of states, caps and warnings; the road ahead, the lane line and its edges); road world, `/clock` 100 Hz: **1.3 %** of a core against 9.0 %; sim-check's road world, `--behaviour-only` and `--behaviour-things-only` pass with it. ✅ 16 Nov the simulator's default (`SIM_DEFAULT_BEHAVIOUR_IMPL`, the same evidence as ego's). ⬜ the car, then make it the car's default too. watchdog: ✅ 16 Nov, `aav_watchdog_rs` (`watchdog_impl:=rust`), a challenger: the Python node's answers tick by tick (1,600 ticks: states, words, lock, state line, log, /diagnostics; all 11 states); road world, `/clock` 100 Hz: **1.4 %** of a core against 9.9 %; sim-check's road world and `--actuator-faults` pass, `--faults` the same as the Python's. A safety lock: the Python stays the default (a person's call). ⬜ found on the way, in the Python too: after `--faults`' odom stop, the `speed` input keeps the wheels' last speed and latches OVERSPEED, so the ten faults after it fail (sim-check `--faults`, both languages): a stale wheel speed should count as NaN. ✅ 16 Nov fixed in both (TODO 207); the Rust watchdog's own `--faults` run: `--actuator-faults` 6 of 6; `--faults` 8 (a busy computer), 12 and 10 of 14, no OVERSPEED after the odom stop; the failures are the navigation flake in TODO 207 (the Python watchdog had it once in 2 runs), so the Python stays the default. localization_health: ✅ 16 Nov, `aav_localization_health_rs` (`localization_health_impl:=rust`): the Python module's answers exactly (3,600 reports: every check's level, words, numbers and /diagnostics text; 300 TF trees against tf2's own Buffer); road world, `/clock` 100 Hz, parked: **1.5 %** of a core against 9.4 % (sim-check's drive: 1.4 % against 9.9 %); both said `OK`; sim-check's road world 9 of 9 with each. ✅ 16 Nov the simulator's default (`SIM_DEFAULT_LOCALIZATION_HEALTH_IMPL`); warn only, so no safety call. ⬜ the car, then its default too. All five heavy nodes now have a Rust twin. lidar_scan and things (Ben, 17 Nov: "lets port the 4 expensive things to rust"): ✅ 17 Nov, `aav_lidar_rs` (`lidar_impl:=`) and `aav_things_rs` (`things_impl:=`): the Python's answers on shared vectors (6,600 ranges identical; 2,237 things within 5e-15, made up and recorded in Ravensim); Ravensim, `/clock` 100 Hz, Python then Rust back to back: lidar_scan **2.0 %** of a core against 4.8 %, things **1.5 %** against 7.5 % (road and campus); `sim-check --movers` Rust 3/3, Python 2/3; `--lidar-obstacle` both pass; lot + University Drive tours, 2 runs each, false holds Rust 0, Python 1 (one Rust University Drive run never moved: the output mux latched STOP at start-up, "sim_gps, which nobody announces on /system/source", a race seen in both languages' sim logs; 2/2 run again; ⬜ that race, its own item). ✅ the default in the simulator and on the car (`DEFAULT_LIDAR_IMPL`, `DEFAULT_THINGS_IMPL`; car-day.md 4f). ⬜ the car |
| 189 | **Behaviour's speed hunting**: one person crossing flipped it `APPROACH(obstacle) > GO > DRIVE` four times (`sim-check --moving`, 15 Nov); a hold (hysteresis) so a state lasts | 🔧 15 Nov: `release_hold` (1 s): an obstacle must stay gone that long before APPROACH lets go, holding its cap meanwhile; slowing is never held. Tested (a flickering obstacle stays in APPROACH); the sim re-run of `--moving` next |
| 191 | **Stops reset the drift** (the whole-car design, research idea 12): wheels that read nothing mean the car stands, so its speed, sideways speed and turn rate are exactly 0 | 🔧 15 Nov: every backend (our simulator's and Gazebo's adapter, `aav_spine`, the Rust `aav_spine_bridge`, the 2025-26 encoder) sends them nearly certain (variance 1e-6) under 0.01 m/s (`vehicle.standstill_covariance`, `wheels::is_standstill`; tested). Left: measure the parked yaw creep before and after |

### Release candidate 0.4-rc1 (Ben, 14 Nov night)

Ben: "We are still in beta stages for sure but we want to start thinking about refinement like 0.4rc1"; "refining the project towards a release candidate. Something that is fully function and complete and having the edges smoothed out." The candidate's stack is ours (`stack:=ours`, Rust from planner to follower), with Nav2 one switch away. It passes when every row is ✅:

| # | Gate | Status |
|---|---|---|
| 167 | **Ours scores at least as well as stock** on the lot and campus tours, 3 runs each (`measure controllers --stacks`; TODO 164) | 🔧 15 Nov, 3 runs each, reversing on, the corrected campus goals: ours reached every leg (lot 12/12, campus 9/9; Nav2 11/12 and 5/9) and kept closer to the path (mean 0.12 and 0.13 m against Nav2's 0.31 and 0.44; worst 0.8 and 1.1 m against 4.5 and 6.2). But ours sways 3 to 5 times more (lot 0.055 against 0.017 m/s², campus 0.66 against 0.12; 29 to 32 left-right changes per 100 m against 11 to 12): that is 168. Results kept in /tmp/ra/keep/drive-j-167 (the morning's notes). Left: the sway. 15 Nov evening, after 168's fixes, ours again (3 runs each, TODO 164): lot 12/12, campus 9/9; off the path 0.21 and 0.22 m mean (Nav2 0.31, 0.44); sway 0.031 and 0.231 m/s² (Nav2 0.017, 0.12): ours wins on legs and keeping to the path, still loses on sway, by 2x **18 Nov (the docs audit):** can't run as written: the stock stack went with Nav2 on 17 Nov (`stacks.yaml` has `best`, `car`, `ours`), so the 16 Nov league is the last head-to-head ([its scores](docs/league-results-2026-11-16.md)); the gate needs a new yardstick (Ben) **19 Nov, scored against the yardstick** (`measure controllers --stacks own_calm --launch stack:=ours`, 3 runs each, reversing, Ravensim on Ben's Mac, load 0.7-7.2; [results](docs/history/mppi-lag-log.md#19-nov-todays-default-against-the-yardstick)): lot 12/12, sway 0.024 m/s² (Nav2 0.018 ❌, ours 0.021 ❌), 6.6 left-right changes per 100 m (10.3 ✅, 5.7 ❌), 0.15 m off the path (0.25 ✅, 0.15 ✅), 158 s; campus 9/9, sway 0.098 (0.131 ✅, 0.099 ✅), 8.2 changes (11.3 ✅, 8.3 ✅), 0.21 m off (0.50 ✅, 0.18 ❌), 1.94 m/s (1.98 ❌, 2 %). Each ❌ against ours is inside the spread of one run to the next; the new sim IMU (TODO 171) doesn't change the lot (0.024 with the old one), and the campus's 0.03 m is unproven. No clear regression. Left: the lot's sway against Nav2's (as on 15 Nov); Ben to confirm the yardstick |
| 168 | **No oscillation** (Ben: "don't forget reverse and no occelation"): the wiggle under 5 cm on the campus road, both stacks (TODO 165: the GPS noise in the pose, our navigator keeps its path) | ✅ 14 Nov: the global EKF trusts each GPS fix less (x, y process noise 0.05 to 0.005): steering changes side 0.39 to 0.30 a second, worst EKF error 0.53 to 0.32 m, the swing itself unchanged at 0.089 m (RPP, our sim, 2 runs each). Left: each follower's own swing; check the EKF change on the car (its odometry is 17 % off loaded). **15 Nov, the cause found (mppi_lag, all ours):** the steering noise, 0.15 rad at every speed, was sized for 1 m/s; at 2.5 m/s it spread the futures 6 times as wide, MPPI's average rested on a handful of samples and the steering changed side about twice a second. The model wiggles as the sim did once it sees the launch file's 8.4 m of path (the old tests saw 3 m and drove 1.2 m/s). Each suspect alone (pose noise, delay, unknown lag, overshoot) added little; 8000 samples or a warmer softmax halved it. Fix: above 1 m/s the noise shrinks with the speed squared (`steer_std_speed`). Without ROS, the sim's campus path with a sim-like pose: swing 0.18 to 0.11 m, sway 0.59 to 0.26 m/s², 0.76 to 0.37 changes a second; a straight 1.89 to 0 ([cause](docs/history/mppi-lag-log.md#oscillation-the-cause)). Never under 0.024 rad (`steer_std_min`). **And our navigator** planned again every second, each plan a little different: it now keeps a clear to-pose path up to 10 s, as the trees' KeepAValidPath (`keep_path_age`); without ROS the lot's left turn sways 0.075 (the sim's, ours) to 0.038 m/s². The target (drive-j, 15 Nov, before both fixes): Nav2's sway, campus 0.123 against ours 0.662 m/s², the lot 0.017 against 0.055, keeping ours' 100% legs and off-path (0.13 m, 0.12 m). Left: the simulator's campus tour and lot to confirm (Ben); the plan's own jogs (16 lattice headings: 0.13 m at 2.4 m/s, both stacks); the lot at 1 m/s still above Nav2's in the model 15 Nov, the sim confirm (our stack, 3 runs each, reversing, /tmp/ra/drive-k): sway on the lot 0.055 → **0.030** m/s² (Nav2 0.017), on the campus 0.66 → **0.21** (Nav2 0.12): a third of before, not yet Nav2's. Off the path, mean 0.12-0.13 → 0.20-0.21 m (Nav2 0.31-0.44). Legs: lot 12/12; campus 8/9 (run 1's 'out of the lot': the follower aborted, error 103; runs 2 and 3 reached it). Left: the planner's staircase on angled roads (16 headings), and the lot's replanning **15 Nov afternoon, the sway at Nav2's** (all ours, the sim, reversing, 2 runs each, the same build; [the sway that was left](docs/history/mppi-lag-log.md#the-sway-that-was-left)): lot sway 0.032 -> **0.021** m/s² (Nav2 0.018), left-right changes 11.7 -> 5.7 per 100 m (Nav2 10.3), off the path 0.19 -> 0.15 m (Nav2 0.25), 8/8 legs; campus 0.231 -> **0.099** (Nav2 0.131), 11.8 -> 8.3 changes per 100 m (Nav2 11.3), off the path 0.23 -> 0.18 m (Nav2 0.50), 6/6 legs (Nav2 4/6), at 1.98 m/s (Nav2 1.60). Cause, by swapping one stage: the follower (ours with Nav2's MPPI: lot 0.020; Nav2's with mppi_lag: lot 0.029, campus 0.239); mppi_lag drove the lattice's staircase faithfully, and its sampling jittered. Fixes, the sim's `ours` stack now: `plan_smoothing:=strong`, `mppi_lag_steer_gamma:=0.05`, `map_correction:=slow`. Price: the campus's 'along the road' 135-144 s (was 111-116: its three-point start). Untested: the car; Nav2 with the strong smoother. ✅ in the sim **19 Nov, today's default** (3 runs each, TODO 167): lot sway 0.024 m/s², 6.6 changes per 100 m; campus 0.098, 8.2: the 15 Nov fix holds |
| 169 | **The followers steer along the road ahead's line** (the lane centre), not only its speed | 🔧 `mppi_lag` done 14 Nov: with a lane map its `LaneCritic` keeps the outline inside the lane (our simulator, a goal 1.5 m off centre: 0.05-0.13 m over the line, never near the kerb; 0.95 m over and onto the kerb without). 15 Nov: `aav_track`'s trackers too: with a lane map, each new path's points are moved inside the soft edges, less half the car and 0.1 m (`lane.py`, tested; not yet driven in the sim). 15 Nov, the sim drive: `mpc_lag` on the campus with `lane_map:=world` (1 run): out of the lot, then 188 m along the road before its progress check gave up; on University Drive 79 % in the lane, over the centre line 21 % (1 crossing). Who steers along the line now: `mppi_lag` (LaneCritic), `aav_track`'s five (`lane.py`); the planner too with `lane_costs:=true` (TODO 148). Who doesn't, and why: Nav2's MPPI and RPP (stock yardsticks, deleted after 0.4-rc1: no C++ of ours); the point driver (its own straight line A -> B, not a road); the lane follower (the camera's lines, its own job). 16 Nov: the planner too can plan the lane's centre itself (`path:=lane`, TODO 148): 0.10 m off it on University Drive, never over the centre line; `ours`' path since 16 Nov (3 clean runs each, TODO 148). 16 Nov: while our planner passes something parked, the road ahead follows the pass and opens its left soft edge, so `LaneCritic` lets the car out (TODO 208; not yet driven with `lane_map:=`). Left: more than one run of `mpc_lag`, the progress-check abort |
| 171 | **No switch leaves a gap**: reversing everywhere, BackUp, IsPathValid, respawn, age checks, `cameras:=false` in nightly CI | 🔧 14 Nov night, checked item by item (the audit's Status): done but the sim and car runs; tonight goals in odom and base_link, the smoother's stale odometry, repeatable sim noise. Left: reversing in `aav_track` (174), node binary tests, sim-only items; three questions for Ben. 18 Nov (Ben: take the recommendations): **the watchdog reads our nodes' status**, warn only (`kind: status`, FAULT and STALE; costmap, planner, navigator, mppi_lag; `stop: true` is the switch; Python and Rust the same tick by tick); **S3** Ravensim's IMU reads as a real one (three turn rates, mean acceleration; `sim_imu` passes it on); **S4** wheel noise documented with its values and why (`docs/own-simulator.md`); **S5** tests of `scene.rs` (9) and the node's decisions moved to `sim-core` `node.rs` (8). Left: reversing in `aav_track` (174), Hybrid-A* (178, after the release), tests of the node binaries (a ROS harness: start the binary on its own domain, call its services, read its topics, as `test_respawn.py`), car runs |
| 172 | **The edges smoothed**: one start command per job, clear errors, the docs pages shrunk, a "start here" page for the release | 🔧 14 Nov fixed: sim-check builds the cameras first, re-sends a goal Nav2 turns down while starting, `--stack`; measure checks a tour's files first and survives a crash. Seen on the dev window, fixed 14 Nov: the things tracker keeps a cloud until /tf catches up (no more "extrapolation into the future" warnings per cloud; drops said once every 5 s); our navigator serves Nav2's action names, so no goal sender chooses, and the map panel marks a goal sent elsewhere (the plan's end) instead of "no goal" (both untested in the simulator). 15 Nov: the campus widens its fence itself (600 m; `geofence_radius:=` still wins), and `docs/campus.md` no longer turns reversing off; `aav_dev_window` is clippy-clean without `ros` (its pretend data now draws a road ahead too). `demo --debug` runs the default stack (no more `tracker:=pure_pursuit reversing:=false`; give them by hand to swap trackers live). The release's start-here page: `docs/release-0.4.md` (linked first from `docs/index.md`); brought up to date 16 Nov (the lanes, the chunks, the sway now 2x, overspeed). Left: shorter pages **18 Nov ([audit](docs/history/audit-2026-11-18-docs.md)):** dated logs moved to `docs/history/<page>-log.md`, word for word: trials 1470 to 158 lines, MPPI with lag 716 to 461, HUD 1167 to 1077, controllers 757 to 662, car day 1138 to 1097; every page checked for Nav2 or Gazebo said to run, dead links, gone switches and topics |
| 173 | **On the car** (with the hardware lead): the seven blockers (158), then ours driven, recorded and scored | ⬜ |
| 174 | **Reverse everywhere** (Ben: "reverse is important"): every stack reverses where the plan says, through cusps, on the lot and the campus; `reversing:=true` the sim's default | 🔧 14 Nov: `mppi_lag` through cusps fixed (its control cost); ours drives the lot's turn back in 29 s (Nav2 59 s); to add: a three-point turn in a narrow street as its own sim test (the campus's round the bend asked for one at the goal, in a 6.5 m street with walls: ours drove 238 m of 242 and stopped 3.8 m short, every sample hitting a wall). 15 Nov, our simulator, `--three-point`: ✅ turned round in a 6.5 m street, 0.73 m from the walls, 60 s, but 7 changes of direction (a clean one is 2-3: refine). A first run failed from another run on the same ROS domain (an empty GPS-waypoints goal replaced ours). ✅ 15 Nov evening, **the cause: the plan itself**. Nav2's lattice prices a backward move but never the change of gear, and the last Reeds-Shepp curve to the goal changes gear for free: our planner planned 6 cusps in the 6.5 m street (5 in an 8 m road). Ours now prices each change (`cusp_penalty`, 4 m) and refuses a last curve that changes gear: 3 cusps planned (2 in the 8 m road), tested (`a_priced_change_of_gear_turns_in_a_street_with_few_cusps`). `sim-check --three-point`: 9 changes of direction with 0, **4 and 4** with 4 m; the lot tour no worse (8/8, sway 0.031, 2 reversals on turn back, 2 runs), the campus no worse (3/3, 1 run). The default since 15 Nov ([own planner](docs/own-planner.md#a-price-on-changing-gear-cusp_penalty-15-nov-todo-174)). Left: Nav2's own lattice (`stack:=stock`) has no such price (its turn untested here). 16 Nov, **the extra changes**: a run's record (`AAV_THREE_POINT_SAVE`) shows the plan turning round nearer a wall than the stop zone's 1 m, so collision_monitor holds the car 1.3 m short of the cusp; then the navigator plans again from there (20 s on the final approach), with new cusps. Fixed in `mppi_lag`: its progress check never fired on a path `map_correction` moves (a new stretch was told by pose: now by index), and held 3 s within 2 m of a cusp it backs on (both tested). `--three-point`, 2 runs: 4 and 6 changes, 43 and 56 s (before: 5, 64 s, 1 run). Tried and dropped (worse in the sim): the navigator keeping a manoeuvre's path 120 s; the planner keeping the stop zone clear at a cusp (offline 4 cusps; in the sim 2 of 2 failed). Left: plan cusps outside the stop zone's reach without breaking the turn ([Cusps](docs/mppi-lag.md#held-short-of-a-cusp-16-nov-todo-174)) |
| 175 | **Three levels, fixed chunks** (Ben, 14 Nov: "I like having hte 3 levels with fixed size map chunks"; "Zoomed out is routes, middle is how we get from cube to cube and finally close up is obstical avoidance"): the **route planner** (zoomed out: the street graph, A*; Lanelet2 later), the **path planner** (middle: 64 m chunks at 1 m and 0.25 m, ~100 m ahead along the route), the **local planner** (close up: the 0.1 m costmap from the lidar's cubes, obstacle avoidance), the **behaviour planner** across all three. Lanelet2 in Rust (Ben, 14 Nov: "convert lanelet2 to rust"): our own reader of its standard OSM file format and the parts we use (lanelets, stop lines and right of way, the routing graph, A*), no crate exists; it fixes the Mac problem and serves the route and behaviour planners. The dev window (Bevy) learns the new maps: lanes and stop lines, the route, the chunks loaded, the saved voxel map. Our simulator takes Gazebo's sensor numbers (noise, lens, ray pattern), not its rendering (OGRE vs Bevy). The cubes also become the NDT map, in the same chunks, moving things left out. Ben, 14 Nov: "NDT can give us the stationary things ... then all we need is occupancy graph to track poepole": the saved map is the background; live cubes not on it are the things to track (with a cube of slack for pose error); the map also replaces OSM as the middle level's map. Safety never relies on the subtraction: the stop zone and close-up costmap see every point. Order: plan ~100 m ahead; chunk the map; cubes into the close-up costmap; NDT in chunks | 🔧 14 Nov: each costmap on its own thread, only the changed area redone (campus global 500 ms to 0; ours now drives the campus roads). 15 Nov: the design written, `docs/designs/2026-11-15_three-levels.md` (64 m chunks, two detail levels, NDT and live cubes per chunk, the subtraction, 7 steps; C1-C6 for Ben). Step 1 built: `map-chunks` (Rust, `aav_description`): the grid, `wanted` (3 x 3 fine + 100 m of route coarse: 9-15 chunks), the store, cutting a map, 11 tests. Step 2 (occupancy): `chunk-map` cuts a map into one `.pgm` per chunk (the campus: 625 a detail, 1.5 s), read back cell for cell. `pixi run campus` now writes `maps/campus/chunks/` too. Step 5 built (off): `aav_things` `background: true` drops cubes on `/map`'s walls (grown 0.25 m) before grouping; 4 laptop tests; to score in the sim. 16 Nov, scored in our simulator on the campus (the design's "Step 5: the scores"): the switch never reached the node (`things.yaml` outranked the launch: fixed); `/map` whole-body cut the things while driving from 9.9 to 0.6 but never found a person on the grass (`/map` marks all that isn't road); against the NDT chunks (`things_background_chunks:=`, built) nothing changed (the guessed map has no kerbs); dropping only cubes under 0.7 m (`things_background_below:=0.7`) gives 1.7, and every walker found within 0.2 s, 100 % of messages: now the default where `aav_things` runs. Safety checked: the stop zone and the costmaps read the lidar's topics, never the things. Next: kerbs in the NDT chunks, then whole-body against them. Step 4 (core): `ndt-core` `ChunkedMap` and `ndt-chunks`: the campus NDT map in 699 chunks, the 3 x 3 round the car 1.2 % of the cubes, built in 0.01 s (whole: 0.9 s), the same cubes. Step 6 started: `lanelet-map` (Rust) reads Lanelet2 `.osm`, routes by lanes (A*), finds the stop lines on a route; 9 tests. The campus as Lanelet2 (`tools/campus_lanelet.py`, run by `pixi run campus`: `maps/campus/campus.osm`) on the street router's own roads, cut at every junction: 998 lanes, 1,903 joins, dead ends turn round. `lanelet-route` routes in under 1 ms; on 3 trips it agrees with `campus-route` on which are possible, 15-18 % longer (whole first and last lanes). Step 7 started: the dev window draws a lane map (`--lanes`: solid, dashed, stop lines, curbs) and the 3 x 3 chunks round the car (`--chunks`) with the road layer; builds, not yet looked at on a screen. 16 Nov: the chunks loaded (`map_chunks::wanted` on the car and the route: `(i, j) fine` or `coarse`), each one's saved map, and the street route (`/drivers/nav/route/path`), key H; `chunks.rs` (5 tests). The window's own screenshot came out black (the screen was locked); the same `chunks.rs` with the campus files, drawn off screen, looked right (chunks on the lanes, coarse chunks along the route). Left: look at it live. step 3, first form built: `global_window:=192` (the global costmap rolls with the car, the 3 x 3 chunks; to score on the campus); the NDT node: built (`ndt_chunks:=maps/campus/chunks/ndt`, off by default; loads the 3 x 3 round the car and rebuilds its matcher on a new chunk; `pixi run campus` writes the NDT chunks); builds and clippy-clean, not yet run in a simulator. **Scored, 15 Nov** (our simulator, `measure controllers --stacks own_calm --reversing --ndt`, the sim's ours stack; results /tmp/w175, not in git): step 4, NDT on chunks: ✅ as good as the whole map on the campus tour (3 runs against 2: off 0.058 / 0.014 / 0.011 m a leg against 0.051 / 0.014 / 0.011; 98 % of clouds a pose on the long legs), ready in 0.02 s against 2.5 s, each new 3 x 3 read in 0-3 ms and built in 8-22 ms. The first try lost NDT for 28 s: after LOST it restarts from ekf_global, but the chunks stayed round the last fit; fixed (`Localizer::guess`: the chunks follow where the next match starts). **Now on by default** with `ndt:=true` on the campus when `pixi run campus` made the chunks (`ndt_chunks:=off`: the whole map). Step 3, `global_window:=192`: ✗ **worse, stays a challenger**: lot 8/8 either way (sway 0.022 against 0.021 m/s², off the path 0.13 against 0.14 m, 313 against 322 s), campus **2/6** against 6/6: the 120-260 m legs' roads leave the 192 m window (the planner runs out of time) or the goal is off it. Next: plan a far goal via the lane route in steps that fit the window, then score again **16 Nov, step 3 done and on in `stack:=ours`** (`costmap:=ours_chunks`, `costmap_chunks:=auto`): our global costmap holds only the 3 x 3 chunks round the car, those along the route's next 100 m and one ring more (`aav_costmap` `chunked.rs`), each read and inflated on its own core, only the chunks a move changed redone, 0 cells different from the whole map's; our planner publishes the route level's whole way (`/drivers/nav/planner/route`) and plans a goal off the costmap to where the way leaves it, then again (`lattice-planner` `steps.rs`). Scored (lot, campus, University Drive, 2 runs each, as `stack:=ours`): every leg either way (20/20), the same time and sway, costmap 474 to 145 MB and 12 to 4 % CPU, planner 403 to 65 MB; results /tmp/r2, not in git. Fixed on the way: a goal near a map's edge was cut short of; a 430 m lane loop round the block (`Fallback::Detour`). Left: coarse chunks (the lattice is 0.25 m only), the local costmap from chunks (it reads /map), a goal cut at the edge never yet seen in the sim (every tour's goals fit the 5 x 5 window). **16 Nov, the local costmap from the chunks** (`local_chunks:=true`, `costmap:=ours_all_chunks`): campus 6/6 and University box 2/2 either way, the same driving, the costmap node 131-148 to 66-81 MB; **on in `stack:=ours`**; in `ours` nothing reads `/map` on the campus now (still: `aav_things`' background, the screens, AMCL, Nav2's costmaps, sim_check). **Step 6 routed**: the route level on Lanelet2 (`lane_graph:=lanelet2`; `lanelet-map` `route_between`: Dijkstra over lanelets, lane changes, one way; the campus's 98 stop lines on `/drivers/nav/planner/route_stops`): every leg on the campus and University Drive, but slower (campus 717 s against 533-562; University Drive 389 against 343): a challenger. Results /tmp/maps, not in git. Next: TODO 220 |
| 176 | **The Rust made more Rust-like** (Ben: "once we are confident that we are feature complete"): a pass over every crate against the Rust API Guidelines | 🔧 15 Nov, partly done: every firmware crate and the library crates of the Rust nodes (car-model, mppi-lag, lattice-planner, costmap, map-chunks, ndt-core, ego-core, mpc-lag, yahboom-protocol, stop-zone, car-smoother, radar-clustering, spine-bridge, lanelet-map, navigator, voxels) now have `#[must_use]` on pure functions, `Debug` on public types, a doc comment on every public item with `# Errors`, and a lint lock at the crate root (`docs/conventions.md`); so do sim-core, camera-core, bench-car and the HUD's four logic crates (hud-core, map-core, debug-core, panel-core: about 1,300 items, 15 Nov; `tools/test.sh` now tests and lints bench-car too); spine_sim's lock says why it panics. No number codes, sentinels or library unwraps were found outside the wire formats. **Left:** in spine-core, on purpose: `limits::abs` (core's `f32::abs` treats -0.0 and NaN differently) and `Quadrature::update -> i8` (could be a Step enum), for a reviewed safety change; one shared `CloudView` for ndt-core, voxels and radar-clustering, not a pure move yet (each `read_*` returns its own crate's `Point`; radar-clustering reads float32 only, with a `Datatype` enum and `NotFloat32`, which debug-core uses; ndt-core and voxels read float32 or float64 with `NotFloat`, their types the same but `read_xyz` written apart): a design choice first; the ESP32 board's build (no Xtensa Rust here) |

### The whole car: every design merged (14 Nov)

Ben, 13-14 Nov: "a review of everything with the plan of merging everything into the best version possible ... what am I missing". The page: [the whole car](docs/designs/2026-11-14_the-whole-car.md). His words: `docs/history/messages-2026-11-14.md`.

| # | Task | Status |
|---|---|---|
| 154 | **The road ahead** (`/world/road_ahead`): the line to follow (the right lane's centre), hard and soft edges, the speed limit and stop points from `stop_law.py`, and an end the car may not pass; the boss (`aav_behaviour`) is its only writer; Nav2's followers and ours read it; in the sim first | 🔧 14 Nov: the message, the writer, ego holds Nav2 (and so our followers) to it; sim-check passed sign and goal with it on. The point driver (R5) too: its line A -> B makes the strip. The screens show it: the dev window, and the in-car HUD ("ROAD 0.6 m/s, SIGN IN 12 m", through look_ahead). The lane line, 14 Nov: with `lane_map:=world` the line is the right lane's centre and has edges (`lane_map.py`, a `lanes.yaml`: the road world's and the campus's; Lanelet2 doesn't import on a Mac). Seen in the road world: on the centre while /plan swung -0.6..1.9 m. Next: the followers (`mppi_lag`) reading the line and its edges; the campus untested live |
| 156 | **The things** (`/world/things`): camera-to-lidar calibration; the cubes grouped; a small tracker (a stock one first, licence checked); names from the camera's boxes; the radar's speed per cluster; the list remembers what the lidar saw on the way in (TODO 93 and 103's Part B) | 🔧 15 Nov: built as `aav_things` (grouping, a Kalman tracker, the radar's speed, the memory for a low box: 170, 188), names from the camera (192), the behaviour waits for a walker (194, 5 of 5). Left: calibration on the car (Q22) |
| 157 | **Record everything, score something**: `tools/record.sh` gains pictures (compressed), cubes, radar and the locks; one car drive scored; shadow mode's stage 1 (TODO 108) | 🔧 14 Nov: `record.sh` keeps what the stack saw and decided (the front camera as JPEG, boxes, sign, lanes, radar, cubes, locks, the road ahead, behaviour, ego, NDT); 28 s in the sim = 11 MB, 53 topics. `tools/score_drive.py` scores any bag (`scorecard.py`, tested): a 25 m point drive in the sim scored clean but for TODO 163. Next: score a car drive |
| 158 | **The car's seven blockers** in one place (weak links, section 1): the firmware timeout (45), the brake follows a wish, wheel speed seen (60), clocks checked (a `car-day.md` step), the steering's centre (67), a truth for "1 m" | ⬜ with the hardware lead |
| 159 | **A safety case**: one page per hazard: what stops it, the number, who measured it; actuator faults on purpose in the sim | 🔧 14 Nov: [the safety case](docs/safety-case.md), 12 hazards, every number from the sim, bench or QEMU (none from the car yet). ✅ 15 Nov, actuator faults on purpose: `sim_adapter` takes `fault.steer` (stuck), `fault.throttle` (stuck), `fault.brake` (weak, dead), between the speed loop and the car (`faults.ActuatorFaults`, tested); `pixi run sim-fault brake weak`; `sim-check --actuator-faults` measures each, our simulator, 3 runs: the e-stop from 0.9 m/s stops in 1.18-1.20 m healthy, 2.05-2.08 m with a weak brake, 9.7 m with a dead one (still rolling after 30 s), 1.48-1.50 m with the throttle stuck at its cruise level. The steering stuck in a bend (1 run): mppi_lag backed and turned 3 times, 19 m, up to 3.1 m off, stood after 16 s, and **nothing reported it**: the goal kept running. Into [the safety case](docs/safety-case.md) (hazards 3, 7, 8). Left: a steering watchdog (command against measured on `/vehicle/steering`); a throttle stuck wide open; the spine in the loop (`spine:=true`) |
| 161 | **Our stack up to Nav2's and Gazebo's standard** (Ben, 14 Nov: "so we can drop their work and have our own complete stack"): the gap list and its order, [audit](docs/history/audit-2026-11-14-own-stack-gaps.md). First: clippy and fmt checked; a CI job with our simulator; age checks on `/tf`, the costmap and odometry in every Rust node; time limits in the navigator; a watchdog on our nodes | 🔧 14 Nov: clippy and fmt in `tools/test.sh`; costmap age in mppi_lag and our planner; odometry and map -> odom age in our costmap; `/tf` age in our planner and navigator; time limits in the navigator; our nodes respawn; `cameras:=false` and `sim-check --no-cameras` (the CI job itself next, once run here) |
| 163 | **The simulated car runs 14 % over its command** (found by the scorecard, 14 Nov): ego asked at most 1.0 m/s, the sim's truth reached 1.145 m/s (the speed loop's overshoot, `speed_loop` default). The road ahead's limits assume the car goes no faster than told: tune the loop, or give the stopping law the overshoot; check the spine's loop on the car the same way. Scored again with `speed_loop:=smooth`: top speed 1.00 m/s, no overshoot; but over the last 1.5 m before B the car slowed at about 0.3 m/s² while the road ahead's limit fell faster (2.7 s over it, never past the end, stopped 0.22 m from B). Likely both loops brake only weakly past their 0.2 m/s coast band (`brake_gain` 0.5): unproven | 🔧 14 Nov, the car model alone (`car_model.py`, ego ramping 0.5 m/s² up and 1 m/s² down): `default` tops 1.144 m/s for 1.0 (+14 %) and 2.70 for 2.5 (+8 %), 95 % in 4.3 s, stops in 1.27 m; `smooth` 0.985 and 2.44, 5.7 s, 1.17 m; `lqi` 0.998 and 2.48, 5.0 s, 1.21 m. So the overshoot is the PI loop's, not the sim's. Proposed: `smooth` (or `lqi`) as the default in the sim and the spine, after a car test on stands; until then the stopping law should allow for +14 % **15 Nov, the stop proven on the car model** (cruise at 1 m/s, set speed ramped to 0 at 1 m/s²): the car stops past where the set speed reached 0 by 0.77 m (`default`), 0.68 m (`smooth`), 0.72 m (`lqi`): the P-only brake (0.5 per m/s past the 0.2 m/s coast band) is weak, as guessed. Brake gain 2.0 alone: 0.36-0.39 m; with a 0.05 m/s band: 0.19-0.21 m. Built: the profile `firm` (`smooth` + that brake) in spine-core and `car_model.py`, the shared vectors regenerated, `aav_track`'s lag model checked on it; `speed_loop:=firm` in the sim and bench. **16 Nov, scored in our simulator** (`simulator:=own`, `stack:=ours`; 2 runs each): a 25 m drive to a point (`drive_to_point --ahead 25`, recorded, `score_drive.py`): `default` tops 1.14 m/s for 1.00, 6.9-7.1 s over the road ahead's limit, stopped 0.12 and 0.06 m from B; `firm` tops 1.00 m/s, **0.0 s over**, stopped 0.10 and 0.18 m from B. The lot tour (`measure controllers --stacks own_calm --tours lot --runs 2 --reversing`, `--launch speed_loop:=firm`): both 8/8 legs; sway 0.021 against 0.020 m/s²; but `firm` changed left-right 8.0 times per 100 m against 4.1, worst 0.72 m off the path against 0.52, 320 s against 314. **So `firm` stays a challenger; the sim's default stays `default`**: better at the stop, worse on the tour. Likely (unproven): the followers' lag models assume the `default` loop (mppi_lag's `lag.rs`, `aav_track`'s `speed_loop`, ego's `trace_speed_loop`, behaviour's), so `firm` drives a car they mispredict. Next: give them the profile the sim runs, and score again. The car's spine keeps its own default until a test on stands. Results: /tmp/w163 (not kept) **16 Nov later, every model told** (the hypothesis tested): `speed_loop:=` now reaches every model of the car (mppi_lag's lag model, which gained smooth's setpoint weight and clamping; `aav_track`'s and `mpc_lag_rs`'s `mpc_lag.speed_loop`; ego's `trace_speed_loop`, Python and Rust; behaviour's stopping law), from one launch argument, checked by `vehicle.check_speed_loop`. Scored again, same session, own_calm, the lot, reversing: `default` 7.3-8.4 left-right changes per 100 m (5 runs), sway 0.020-0.027 m/s², worst 0.48-1.20 m off; `firm` **18.6, 27.6 and 31.8**, sway 0.039-0.041, worst 0.47-0.75 (3 good runs; 2 others never drove: the planner had no costmap yet, a start-up race under load, not the loop). **So the hypothesis did not hold**: told the truth, `firm` sways about 3 times more, not less. Where (without ROS, `wiggle --features shared`, 1 m/s, sim-like pose): only when MPPI's own model has firm's brake: sway 0.065-0.070 m/s² against 0.021-0.024; a firm car planned with the default or smooth model, or smooth on both, sways as default. Likely (unproven): firm's brake (2.0 per m/s past 0.05 m/s) makes MPPI's sampled speeds brake hard in its rollouts, and that noise reaches the steering. The 25 m stop again (point driver, 2 runs each): `firm` 1.00 m/s top, 0.1 s over the road ahead's limit, stopped 0.08 and 0.12 m from B; `default` 1.14 m/s, 6.7-7.0 s over, 0.06 and 0.16 m. Note: this session's `default` tour (8.0) is twice the first session's (4.1), with another session's sim on the same machine: the first `firm` result (8.0) may have been no worse than `default`. **Not recommended as the sim's default yet.** Next: MPPI with firm's brake (why its rollouts sway), or plan with `smooth` while the car runs `firm` (wiggle: as default); then score again. The spine's default stays until a test on stands **18 Nov, the car on one loop, the models on another** (`speed_loop_model:=`, new: the car keeps `speed_loop:=`, ego's trace, the follower and the behaviour planner take this; empty = the car's, so nothing changed by default; sim and bench). Scored on Ben's Mac (GPU, headless), own_calm, the lot, reversing, 3 runs a batch (load 4-11, other sessions): `default/default` 7.3 and 7.5 left-right changes per 100 m (2 batches), sway 0.021-0.023, worst 0.44-0.53 m off, 155-167 s; `firm/firm` **20.8**, sway 0.034, worst 0.49-0.75, 153-154 s (1 batch); `firm/smooth` 9.6 and 8.6, sway 0.023-0.024, worst 0.46-**1.05** (3 of 6 runs past 0.8 m, on the legs that back up), 159-169 s; every leg reached. The 25 m stop (2 runs each): `default/default` 1.14 m/s, 6.7 s over, 0.13 m from B; `firm/firm` 1.00, 0.2 s, 0.06-0.08 m; `firm/smooth` 1.00, **0.0 s**, 0.07-0.08 m. **So planning with `smooth` takes most of firm's sway away and keeps its stop; but the tour is still a little worse than `default`** (+23 % left-right changes, a worse worst off-path): not proposed as the sim's default yet (docs/league-results-2026-11-16.md). Next: why the backing-up legs go 1 m off (firm's brake at low speed, which smooth's model doesn't see), or MPPI with firm's brake. The spine's default stays until a test on stands |
| 160 | **Shrink the docs and this file**: pages merged or moved to `docs/history/`; TODO's finished sections moved out; a short "what is next" at the top | 🔧 14 Nov: this file's finished rows and dated logs moved to [todo-done](docs/history/todo-done.md) (979 lines to about 560), "What is next" at the top; the pages still to do 15 Nov (day, Ben: "close stuff that doesn't make sense any more"): every old row checked against the code; 652 lines to 225 (109 KB to 38 KB); the old lists moved word for word to todo-done; still open in four short sections (software, the car, the team, ideas), the closed ones with a reason each **18 Nov ([audit](docs/history/audit-2026-11-18-docs.md)):** 29 finished rows and "Closed on 15 Nov" moved word for word to todo-done (285 to 234 lines, 100 to 79 kB); stale rows corrected against the code; status.md brought to 18 Nov |

### One shape, and lanes as soft costs (13 Nov)

Ben, 13 Nov: "How can we make all of this fit together better", and lane lines "it can cross but only if it needs to and to stay on the right side". The page: [one shape](docs/designs/2026-11-13_one-shape.md).

| # | Task | Status |
|---|---|---|
| 148 | **The lane layer**: drawn per route as a ramp (lowest at the right lane's centre) so the planner keeps right; compared with a lane-centre planner (`path:=lane`); then the camera's lines | 🔧 15 Nov: built in our costmap, not Nav2's mask: `aav_costmap` `lanes.rs`, `lane_costs:=true` (league level `lanes`: off, ramp), which lane by /plan, **added** to the inflation (the higher of the two left the car mid-road: the kerbs' inflation is lowest there). Campus, University Drive leg, 2 runs each: over the centre line 2-27 % vs 75 % with none; reached 2/2 vs 1/2; time and sway no better. `lane_map:=world` (the road ahead's lane line + LaneCritic, the nearest to `path:=lane`) 0/2 reached, 30 %. Only ~30 s a leg is on a mapped road. **A challenger, not in `ours`** (`docs/own-costmap.md`). 16 Nov: **`path:=lane` built** (`aav_planner` `lane.rs`: Dijkstra on the lane map's roads, the right lane's centre, the lattice joining the ends, straight when the car is on its lane; off the map the lattice) and a tour on more mapped road (`university`, ~290 m of University Drive). Our simulator, `own_calm` + `behaviour:=true`, reversing: off the lane's centre 1.51 / 0.59 / **0.10 m** (as now / ramp / lane), over the centre line 68 / 1 / **0 %**, sway 0.093 / 0.087 / **0.063**, the same time a leg (43, 60, 68 s); legs reached 7/9, 8/9, 11/18, every miss the load of a second simulator (stale costmaps, the watchdog), 6/6 on the quietest runs (`docs/own-planner.md`). The campus tour can't show it: its start and goals are off the lane map. ✅ 16 Nov, **the swap**: 3 clean runs each on a quiet machine (load under 8), all three 9/9 legs in 512-517 s; `path:=lane` no worse on any column: sway 0.055 against 0.087 / 0.090, off the path 0.09 against 0.11 / 0.13 m, left-right changes 5.4 against 9.6 / 8.7 a 100 m, over the centre line 0 against 61 / 1 %. **`path:=lane` is `ours`' path** (`stacks.yaml`, [the swap](docs/own-planner.md#the-swap-16-nov-3-clean-runs-each)); the ramp stays a challenger. 16 Nov: something parked on the lane is passed, the rest the lane's centre (TODO 208). Left: the camera's lines |
| 150 | **The league** (`stacks.yaml`, `stack:=best/car/stock/ours`, `pixi run measure league`): one champion per level, challengers scored on the same routes and recordings; a swap needs a margin on 3+ runs and a person's yes | 🔧 14 Nov ([page](docs/league.md)): `config/stacks.yaml` (aav_description), `stack:=` on `sim.launch.py` (a switch given by hand still wins), its tests; checked offline. Next: a sim run of each stack, `stack:=` on `car.launch.py`, `pixi run measure league` 15 Nov: one switch per level (one shape's "36 switches to 8"): `pose:=`, `path:=`, `steer:=`, `lanes:=` by the league's names, over the stack's pick; 6 tests; the old switches keep working |
| 152 | **Our planner faster** (cache each square's check per heading, outlines per heading at start-up, several cores: 180 ms against Nav2's 150 ms on long campus plans) and **MPPI with lag** (slow earlier near the goal with `stop_law.py`; fewer futures; timed on the Orin) | 🔧 15 Nov: `examples/campus.rs` times the campus legs on the real map (no ROS). Where the time goes (macOS `sample`): the fit check 26 %, the search loop 25 %, the obstacle heuristic 19 %, the node map 14 %, the open list 15 %: nothing dominates. Done: the outline check no longer allocates (a million traces a long plan): the same paths, along the road 217 → 194 ms, reversing 162 → 145 ms (a busy Mac). Not done, and why: a per-square cache is unsafe (poses aren't on cell centres: rounding would call a pose 0.2 m into a wall free); several cores don't pay (one search step is ~1 µs); pre-sizing the maps: no gain. Left: re-time against Nav2 in the sim. Then MPPI's part tried in our simulator: `mppi_lag` with 1000 samples (nav2.yaml's MPPI `batch_size`, 2000) beside all ours as it is (cusp_penalty 4), `measure controllers --stacks own --reversing`, 2 runs each. The tick halves: lot 1.8 ms against 3.5, campus 2.5-2.9 against 4.2-5.0 (an M4 Max). The ride: lot 8/8, sway 0.029 against 0.031 m/s², off the path 0.18 against 0.21 m, **but left-right changes 16.1 per 100 m against 10.9**; campus 6/6, sway 0.232 against 0.225-0.231, off 0.23 against 0.22 m, worst 1.09 m. Not a clear win: **2000 stays**. Worth it on the Orin if the tick is late there (not yet timed on the Orin). Results /tmp/w162/m152. 16 Nov, **the planner re-timed in our simulator** (`measure controllers --planners lattice own --reversing --tours campus`, 4 runs; before each leg both plan from the same spot on the same map, each planner's own `planning_time`): along the road ours 185-197 ms against Nav2's 122-171 (means 189 and 146: **still ~30 % slower**); round the bend 230-357 against 227-242 (~18 %); out of the lot 3-7 against 6-9. Ours prices each change of gear (`cusp_penalty`), so its search isn't Nav2's step for step. A cheap safe win, kept: a move that can't improve its node even on free ground skips its outline check (the same nodes, the same paths; the whole campus map offline, interleaved: along the road reversing 150 -> 137 ms, forward 257 -> 249). Still not done: the per-square cache, several cores (as before); left, a profile on the whole map (per step 0.95 µs against 0.82 on the tour's corner). ✅ **MPPI slowing earlier near the goal: done another way.** With the road ahead on (`behaviour:=true`, in `ours`), its speed limit is the stopping law's cap to the goal (a FINAL stop) and ego holds `mppi_lag`'s forward speed under it; `mppi_lag` itself doesn't read the law. Results /tmp/w152 |

### Software, still to do (from the older lists)

Tidied 15 Nov (Ben: "close stuff that doesn't make sense any more"). Each row's full
history: [todo-done](docs/history/todo-done.md#the-old-lists-as-they-were-on-15-nov).

| # | Task | Status |
|---|---|---|
| 132 | **The lag-aware MPC** (`tracker:=mpc_lag`): keep off obstacles (costmap cost in the QP); ~~move to Rust before the car~~ in Rust since 15 Nov, a challenger beside the Python (`aav_mpc_lag`, `tracker:=mpc_lag_rs`: 0.75 ms a tick on a Mac against 5.3 ms; the same commands to 1e-3; our simulator's lot tour 4 of 4, as the Python, at a fifth of its tick and CPU). Left: score it over more runs and the campus; the costmap cost, the lane (TODO 169) in the Rust, time it on the Orin | 🟡 built and scored |
| 114 | **The developer window**: the algorithm list with SWAP, cancel a goal, try a slower laptop | 🔧 15 Nov: CANCEL THE GOAL and SWAP built (the follower live, as the debug HUD's keys; the rest chosen at launch, said in the tab), behind the goal's gates; builds with ROS, clippy clean, a test on the list; not yet clicked beside a sim. Left: a slower laptop |
| 120 | **The campus map**: fit a real drive to it, the GPS offset, a person approves map changes | 🟡 |
| 94 | **Drive-to-point stops 0.61 m past B** by the car's own EKF, once a run (limit 0.5 m) | 🔧 15 Nov, our simulator: first **found a worse bug**: `sim-check --drive-to-point` never moved the car ("stood still for 5 s though asked to move"). With the ours stack, `behaviour:=true` told ego to hold the computer drivers to `/world/road_ahead`, but its writer runs inside navigation, and the check runs `nav:=false`: no road ahead, so ego trimmed the point driver to 0. Fixed: ego waits for the road ahead only with `behaviour` and `nav` (`sim.launch.py`; `test_ego_waits_for_the_road_ahead_only_where_its_writer_runs`). Then 9 of 9 drives and the 4 stops passed, one run: every stop 0.05-0.19 m from B by the truth, **at most 0.15 m past B by its EKF** (the limit 0.5). The 0.61 m (Gazebo, 3 Nov, 1 drive in 9) didn't come back in 1 run; the GPS noise in the EKF at the stop is still the likely cause (by the truth it was 0.29-0.44 m). Left: more runs, Gazebo |
| 207 | **Stuck steering is noticed** (found 15 Nov, TODO 159: steering stuck in a bend, the car backed and turned 3 times over 19 m, up to 3.1 m off, and nothing reported it): a watch comparing the steering asked with the steering measured (`/vehicle/steering`), a warning then a stop after a set error for a set time; the same for a stuck throttle (overspeed) | 🔧 15 Nov: the sensor watchdog's `steering` input (kind actuator): more than 0.15 rad off its command for 1 s latches STUCK and holds the programs; `sim-check --actuator-faults` now needs it: noticed 3.6 s after, the car stood 5 s after (1 run; before: 19 m, nothing). 4 tests. **16 Nov, the throttle**: the watchdog's `speed` input (kind speed): the wheels more than 0.2 m/s over ego's speed while not slowing (under 0.3 m/s² over 0.5 s): OVER, a warning, after 1 s; OVERSPEED after 2 s, latched, holds the programs. The numbers from the car model: the `default` loop's overshoot (+0.14 m/s) and every healthy stop never count, in every loop profile (tested). `sim-check --actuator-faults` (2 runs): the throttle stuck while speeding up held 1.37 m/s for 1.00: OVER 3.6 s, OVERSPEED 4.6 s after; the dead brake latches it too; the healthy, weak-brake and stuck-throttle e-stops never do; the 25 m point drives and lot tours (TODO 163) never locked. 8 tests. **16 Nov, fixed**: after `--faults`' odom stop the `speed` input kept the wheels' last 0.5 m/s and latched OVERSPEED, so the ten fault cases after it failed. A wheel speed older than the input's `max_age`, or a command after a longer gap, is now no check (quiet wheels are odometry's own STALE), in the Python and the Rust alike; a test in each; the shared vectors replay it. `sim-check --faults` 14 of 14 fault cases and `--actuator-faults` 6 of 6 (the Python, first run). ⬜ Found then, a flake: 4 more `--faults` runs (Python watchdog 13 of 14; Rust 8 (a busy computer), 12 and 10 of 14) lost cases to "Nav2 would not drive before the fault": after `odom stop` and `odom nan` the global EKF jumps 9 m as the wheels come back, mppi_lag finds "no part of the path ahead inside the costmap" and the navigator aborts goals; OVERSPEED never latched. Make sim-check wait for Nav2 to recover, or the EKF not jump. Left: the car, for both (steering: no measured angle, Q2; speed: watch it on the car once the wheels are trusted) **17 Nov, the flake fixed**: with the wheels quiet both EKFs kept their last speed and acceleration (no speed measurement), `ekf_local` ran 15 m on while the car stood, and `map -> odom` jumped 9-10 m back when the wheels came back. Both EKFs now take the car's command (robot_localization's control term, `vehicle.ekf_control_params`): a command below the speed slows it (at most `max_brake`), a silent one (0.5 s) to 0; never faster (acceleration limit 0: a prediction ahead of the wheels dragged the position on). The EKF files no longer say `use_control: false` (a named entry beat the launch file's). Then a second flake: after `watchdog hang` DDS took 7 s to bring the HUD's beats back and the HUD latched LOST; the watchdog now waits 10 s after its own stall (Python and Rust). `sim-check --faults` **14 of 14 three runs in a row** (17 Nov, the Rust watchdog, load 10-34) |
| 121 | **NDT's score floor on the campus** (1.5 there, 2.3 in the lot): how high does a wrong pose score? | ⬜ |
| 71 | **The hardware panel's pages**: actuators off, measure, direct control, typed-in measurements | 🔧 15 Nov: **MEASURE** built (speed and distance since RESET, a tape run typed in, the wheel diameter's check, know-the-car test 3; SAVE to a file; 9 laptop tests, the demo's screenshot looked at). Left: actuators off and direct control need the two-way ESP32 link (`docs/designs/2026-11-06_bench-link.md`, the Rust firmware), so they wait for it |
| 0b | **The spine's state machine, follow-up** (Arsh's #15): the test mode without a wheel speed; a real self-test | ⬜ Arsh |
| 23 | **sim-check in the nightly CI**: drop `continue-on-error` once it passes a few nights (with 151) | 👀 since 16 Nov the job runs Ravensim only: `tools/software_gl.sh pixi run sim-check --obstacle` (cameras in lockstep) and `--no-cameras --obstacle`; count the nights from its first pass (240) |

### Needs the car or the hardware lead

The steps are written ([car day](docs/car-day.md), [know the car](docs/know-the-car.md)); each
needs the car, a part, or an answer. 157, 158 and 173 are in "What is next".

| # | Task |
|---|---|
| 22o | **Ackermann, the rest**: both front wheels' real angles and the steering curve from `Ackermann.m` (Q2). Moved here 15 Nov: the ideal two-wheel angles are already ours (`aav_description` `vehicle.py` `wheel_angles`, the spine bridge's `wheel_angles`). `Ackermann.m` (the 2025-26 team's linkage: actuator length -> passenger wheel by the law of cosines, the other wheel from the trapezoid's loop equation), run as written, gives **anti-Ackermann** (actuator 510 mm: 10.5° and 11.1°, the outer wheel turning more), so either the car is built so or its signs are mixed; it can't stand in for a measurement. On the car: both wheels' angles at 5 actuator positions (a protractor or the phone level on each wheel), against `wheel_angles` |
| 92 | The people named (e-stop person, safety driver...); the hardware lead's questions: H1 (CORTEX the Motor Pi?), C1 (CAM1's address), `.211`'s job, who owns the router |
| 126 | The hardware debug plan: a USB cable, the current limits, the return wire, where the steering and brake sensors connect |
| 49b | Nothing measures the driven (rear) wheels on stands: a speed sensor there before any closed loop on stands |
| 101 | The brake follows a wish (feed-forward + PI for throttle and brake; TODO 0 joined it): needs 49b, then test 4.3 on the ground |
| 46, 49 | The steering and brake position sensors (probably read from the Jrks over I2C); is I2C good enough on those wires? |
| 14, 60, 61 | Weigh the car (front, back, rear lifted); the wheel diameter by rolling 10 m; the front and rear track |
| 52b, 27 | Refit the car model from the actuator tests; try the `smooth` speed loop on the car |
| 67, 79 | The steering actuator's ends and centre (way A with Pololu's tool, then way B); then the brake and throttle the same way |
| 64, 100 | At the first lot visit: the datum, the clocks, walk the fence, the site file (keep-in, keep-out, speed zones) |
| 104 | The car identifies itself (delays, lags, braking, with error bars); then a safety filter fed by the fence |
| 37c, 25 | The camera stream on the Orin and the Pi; calibrate the cameras (Q22); the rear camera with a real picture |
| 65, 125 | The 2024-25 / 2025-26 teams' recordings: find them; rename their topics to ours |
| 68, 74 | The car-ready checklist; the first moves: a throttle cap, the joystick's button and axis numbers |
| 75, 76 | The localization Pi's machine file; Python's speed on the Orin |
| — | Flash the Rust Picos (wheel, sentinel); something to feed the sentinel, once we know its display is on the car |
| — | Day B: the Pis to Ubuntu 24.04, the Orin to JetPack 7.2, only after a restore is proven |
| — | The Mac as the operator station: measure the car's WiFi first |
| — | Tune `ekf_global` for the real GPS receiver (N7, Q18) |
| — | A read-only audit of the car's network; the photo audit's checklist (`docs/car-photos.md`) |
| — | A spare Pico 2 and a **3.3 V** CAN module, to try the spine on a desk |

### The team decides

| Question | Today |
|---|---|
| 122: should reversing on long drives cost more, or stay short? | Nav2 backed a long way to a goal behind it (5 Nov) |
| The brake after boot, the steering on an e-stop, a brake on the RC gear switch | one constant each in `spine-core` |
| `ROS_DOMAIN_ID` for the car and every laptop | 0-101 safe; never above 232 |
| The spine's arming rules: boot latched, arm on a 0 → 1 edge | Proposed |
| The GPS driver | `ublox_gps`, Tentative |
| N3: `respawn` on device nodes? | none today |
| Rotate the leaked secrets (Q34); answer Q1-Q34 | open |

### Ideas kept, not picked

| Idea | When |
|---|---|
| 108 Shadow mode: the automatic drivers score themselves while a person drives | recording and a score on real bags first |
| 96 A model builds our own algorithms from our recordings | after the car's data |
| 78 The talking model: explains, never drives | design first |
| Learned state-space models (Mamba-style) of what the linear model misses | after hours of car logs |
| micro-ROS again | once the hardware is known (Tentative) |
| numpy above 2.3.5 in the `ml` environment | when coremltools passes 9.0 |
| "Two drive paths": the bridge refuses when a second mux or heartbeat appears | later |
| The little robot's eyes on the HUD | later |
| A stop sign seen stays on the map (the whole car, "cheap things never tried") | needs two rules first (15 Nov): behaviour only slows for a sign, the watcher's lock stops the car once the camera sees it; a remembered sign the camera never confirms needs its own stop-and-go, and a sign taken away needs forgetting. Today a sign is kept 5 s in odom after the last sight |

### For the 2027-28 team

Learned models: worth knowing, but each needs training data and GPUs we don't have this
year (Ben, 28 Oct):

- [ ] **Learned BEV** (BEVFusion, BEVFormer): all cameras and the lidar into one top-down view.
- [ ] **One network for perception, prediction and planning** (UniAD).
- [ ] **World models** (Wayve's GAIA-1) and **vision-language driving** (Waymo's EMMA).
- [ ] **Predicting others' paths** (MTR, Wayformer); at 1 m/s, constant velocity is enough today.
- [ ] **Learned planners** (diffusion planners).
- [ ] **State-space models** (Mamba): see "Ideas to keep".


## Finished rows moved 18 Nov (the docs audit)

Moved from `TODO.md` word for word on 18 Nov (TODO 160), each under the heading it had there. Two rows got a dated note first: 181 (`stack:=stock` went with Nav2) and 232's docs part stays in `TODO.md`.

### Ben, 17 Nov: ready to install by Wednesday

| # | Task | Status |
|---|---|---|
| 245 | **The 0.4-rc1 release check** (Ben, 17 Nov: "we want to have everything solid and ready to install by tomorrow"): the QEMU flake, every sim check in a row, the car kit, CI, a one-page note | ✅ 17 Nov: [0.4-rc1 on the car](docs/release-0.4-rc1.md). Every sim-check mode passes (`--behaviour` after a fix: the check turns the things off); CI green for `aaf3ab80`; the kit builds, 804 crates offline; the QEMU check retries once only when nothing but host-speed timings failed |

### Found 17 Nov (Nav2 and Gazebo deleted)

| # | Task | Status |
|---|---|---|
| 230 | **The diffusion planner's training data without Nav2**: `tools/diffusion_data.py` (`pixi run diffusion data`) runs Nav2's `map_server` and `planner_server` as the teachers; both went (TODO 184). Teach it with our planner (`aav_planner`) and map server instead | ✅ 17 Nov (with TODO 242): `diffusion data` runs on ours, no ROS: `aav_planner`'s `teach` example makes our global costmap from each map and plans with our planner, its settings as navigation.launch.py writes them; the same npz. 4 maps, 22 paths in 8 s; `train.py`'s dataset reads them (1 of 22 refused by the guard) |
| 233 | ✅ **The behaviour planner's speed cap reaches no follower**: `/drivers/nav/speed_limit` was read only by Nav2's controller_server (and `mppi_lag` never read it); ego still caps by `/world/road_ahead`. **17 Nov:** the simpler way, folded into the road ahead: no limit in `/world/road_ahead` is above the cap (`road_ahead.capped`, Python and Rust, the shared vectors exact), so ego holds every follower to it; the dead topic is gone from both nodes, the contract and `record.sh` |
| 234 | **The output mux's "NOT SIM" latch on a busy computer**: `sensor_mux` saw `sim_gps`/`sim_imu` not announced on `/system/source` and `output_mux` latched STOP, so a whole sim-check session never drove (3 of 21 sessions on 17 Nov, load 10-34). Found in the shadow runs; not the stop zone's | ✅ 17 Nov. The cause: discovery on a busy computer showed `sim_gps`, `sim_imu`... up to 10 s before `sim_adapter`, which announces them (its whole process late in the graph; its latched message itself was fine), so the 5 s `hold` turned "nobody announces" into a problem. The fix (`sensor_mux` `grace`, 30 s from start; `provenance.Hold`): a doubt an announcement would settle (a sensor nobody announces, an announcer not heard yet) stays WAIT, closed, until then, then latches as before; another source's announcement and two publishers on one sensor get no grace, so the car's lock is unchanged; 7 unit tests for the orders. Old rate: 3 of 21 sessions on 17 Nov (load 10-34); 7 latched logs among 791 sim logs in /tmp. After: `sim-check` 10 of 10 drove (domains 161-170, load 5-7 at start; 2 sessions waited on the grace, 0.5 s each). Not re-run at load 10-34 |

### Ben, 16 Nov

| # | Task | Status |
|---|---|---|
| 244 | **A prettier Ravensim** (Ben, 16 Nov: "its pretty ugly and flat looking"): lighting and shadows; textures for asphalt, grass and kerbs; sky and haze; OpenStreetMap's building heights on the campus; trees; lane markings painted. **The sensor numbers unchanged**: the lidar, radar and GPS read the same world, and the cameras' scored checks (`sim-check`, the stop-sign and lane tests) pass as before | ✅ 17 Nov: `graphics:=pretty` (the default on a GPU; `simple` in software): grain, sky, haze, warm sun, sharper shadows, the campus's bump maps and stop lines; sim-check, lane and stop-sign checks pass ([Looks](docs/own-simulator.md#looks-graphicspretty-17-nov)). Left: the road world is still a bare road (scenery would change the lidar's world); run `pixi run campus` again for the stop lines |
| 221 | **Stop for anything moving** (Ben, 16 Nov: "if we see a car people or stuff moving around us we should likely just stop"): a thing moving 0.3 m/s or more within 10 m, or onto the road ahead within 3 s, holds the car (WAIT, reason `moving`; the road ahead ends at the car) until 1 s after it has gone or stood still; only a still thing is passed | ✅ 17 Nov: in both behaviour planners, the shared vectors exact (`moving_near`, `MovingFor`, the machine's `holding`); parameters `moving_hold`, `moving_radius`, `moving_within`, `moving_speed`, `moving_settle`. A thing counts once it has moved 1.5 s without growing (the tracker gave the parked box, new tracks, buildings coming into view and the lot's posts a speed: 8 holds passing the box before). `stack:=ours` turns the things on. `mppi_lag`'s progress check waits during the hold. `sim-check --movers --moving --behaviour-things` (new `--movers`): an oncoming car, a person on the roadside, cross traffic and a person crossing held every run (2.0-20 s), the parked box never ([stopping](docs/stopping.md#anything-moving-near-stop-and-wait-17-nov)). Phantom stops, `measure controllers --stacks own_calm_chunks --tours lot university university_box --runs 2 --reversing` with the things: 16/16 legs, 3 short holds in 6 runs (2 in one lot run, 1 on University Drive), from the tracker (TODO 222) |
| 222 | ✅ **The things' tracker swaps between thin posts** (found 17 Nov, TODO 221): in the test lot a row of 0.3 m posts becomes tracks moving 1.5-11 m/s for 1.5 s and more (association hopping post to post), so the hold stops the car briefly about once a lot tour. Fix in `aav_things`: a tighter gate or a nearest-neighbour check for thin things, or TODO 217's motion model **17 Nov:** three gates on each match in `tracker.py`: inside the thing's own uncertainty (Mahalanobis distance² under 9.21), a like footprint, and a small thing (0.8 m or less) no faster than 3.5 m/s since last seen; a parked thing unseen keeps a small speed doubt. A made-up row of posts, 30 passes: 33 false holds before, 0 after; a walker and a car keep their speed. `pixi run measure controllers --stacks own_calm_chunks --tours lot --runs 3 --reversing --launch behaviour:=true lane_route:=true things:=true`: 12/12 legs, **0 holds for something moving** in 3 lot tours (before: 2 in one lot run of 3), no stops, sway 0.019-0.028 |
| 211 | ✅ **The lane's corners as clothoids** (found 16 Nov, the league): on University Drive the Hybrid-A* paths sway half as much as `path:=lane` (steering flips 0.05 against 0.20 a second) but leave the lane; `lane.rs` rounds corners with circle arcs, whose curvature jumps. Continuous-curvature corners (Fraichard and Scheuer), or the piecewise-jerk path (214). **17 Nov:** built as a challenger, `lane_corners:=clothoid` (4 m of clothoid, an arc, 4 m back; under the turning radius's curvature). University and university_box, 2 runs each: a tie (sway 0.057 against 0.056 and 0.061 against 0.062; flips per leg 0.15-0.32 against 0.19-0.36, mixed), one clothoid run 0.44 m off the lane's centre onto University Drive. The arcs stay the default; the steering flips are the follower's, not the corners' (214 next) |
| 208 | **Keep right, leave the lane only to pass** (Ben: "yes we want to know about lanes and yes we want to stick to the right side of the lane unless we need to for obstical avoidance"): on a mapped road the car keeps to the right lane's centre; a parked obstacle is passed into the other lane only as far and as long as needed, then back; a person or moving thing is waited for | ✅ 16 Nov: `path:=lane` passes it (`aav_planner` `pass.rs`): only the blocked stretch moves left, the least that clears it plus 0.5 m, never past the kerb; no room: the lattice, as before. The planner's lidar layer sees 18 m (was 12). University Drive, a 1 m box, 3 runs: over the centre line 4.8 s against 13.2, back 9 m after it against 18.5, no stops; a person crossing still waited for; the no-obstacle tour unchanged. **In `ours`** ([page](docs/own-planner.md#passing-something-parked-16-nov)). Left: a car coming the other way, a one-lane road (the lattice still leaves it), a parked car (longer), `lane_map:=` with `LaneCritic` driven |

### Rust crates to try (15 Nov)

| # | Crate | Where, and why | Status |
|---|---|---|---|
| 196 | **tracing** + **tracy** (MIT/Apache; BSD) | see where each Rust node's time goes, live, against its budget (MPPI 50 ms) | ✅ 15 Nov: MPPI's tick in spans (rollout, critics, weigh, smooth), off by default: `--cmake-args -DMPPI_LAG_FEATURES=trace` (or `tracy`), then `AAV_TRACE=1` (stderr) or `AAV_TRACE=tracy`. Other nodes when one runs late |
| 199 | **clarabel** (Apache-2.0) | a pure-Rust solver for a proper MPC in Rust. 15 Nov: the MPC is in Rust (`aav_mpc_lag`, `tracker:=mpc_lag_rs`, TODO 132), on our own ADMM ported from `qp.py`, not clarabel: the ADMM warm-starts from the last tick and gives the Python's answers; clarabel (interior point) does neither (`aav_mpc_lag` README) | ✅ weighed, not used |
| 201 | **rstar** (MIT/Apache) | finding nearby lanes in the Lanelet2 map, and the map chunks | ✅ 15 Nov: `lanelet-map`'s `nearest` looks in an R-tree of the centre lines' segments, not every lanelet's line; checked against the old scan on a grid of points. The map chunks don't need it: they are a fixed grid, found by key |
| 202 | **mcap** (MIT) | read our recordings in Rust tools: scoring a drive, replay checks (TODO 157) | ✅ 15 Nov: MPPI's `follow_plan` reads a recording (`-- bags/<drive>/<drive>_0.mcap`): each plan on `/plan` (up to 10, spread over the drive) followed again offline, with the same table as the CSV paths. A dev-dependency only. Scoring a whole drive stays Python (`tools/`) |
| 203 | **criterion** (MIT/Apache) | real benchmarks for the planner, MPPI, the costmap, NDT | ✅ 15 Nov: `cargo bench -p` mppi-lag (`tick`), lattice-planner (`plan`: lot and campus legs), costmap (`update`), ndt-core (`align`); dev-dependency only. M4 Max: MPPI 1.2 ms (1000) and 2.0 ms (2000) a tick; the campus's "along the road" 216 ms; a local costmap update 0.42 ms; an NDT match 1.0 ms |
| 204 | **proptest** (MIT/Apache) | random-input tests for the stop zone, the car model, map chunks | ✅ 15 Nov: `tests/props.rs` in three crates. **Stop zone** (7 rules, 2000 cases each): it only passes or stops; 3 points in the box always stop; points outside never do; more points never clear a stop; a silent sensor always stops; NaN points are dropped; the ray test matches a rectangle. **Map chunks** (3): a point is in its chunk; the 3 x 3 is always fine; every route corner ahead is wanted. **Car model** (3): braking only slows and never rolls back; the wheels never pass full lock; a bad dt is an error. No bug found: they guard |

### The night of 14-15 November

| # | Task | Status |
|---|---|---|
| 186 | **Follow in odom, correct slowly** (the whole-car design, "Localization and the wiggle"; 165, 168): the GPS moves map -> odom ~9 cm a fix at 5 Hz, and the followers read each jump as a tracking error **15 Nov: a challenger, not the default:** it won offline only; a new one becomes the default once it wins in the simulator (CLAUDE.md). At first slow looked guilty of TODO 193's stall (2 fails with slow, 1 pass with newest), but newest then failed the same check: the stall isn't slow's. Next: score slow against newest in the league | ✅ **Scored 15 Nov afternoon** (the same runs as newest, all ours, 2 each): campus sway 0.231 -> 0.213 m/s², off the path 0.23 -> 0.13 m; lot sway the same (0.032), off the path 0.19 -> 0.13 m; every leg. It wins: the sim's `ours` stack takes `slow` (with the two sway fixes, TODO 168); the car and `stack:=stock` keep `newest` |
| 181 | **Ours is the default** (Ben, 14 Nov: "remove nav2 and gazeboo"): `stack:=ours` the simulator's default (to use Nav2: `stack:=stock`); the launch tests, sim-check, measure and docs follow | ✅ (18 Nov, below) 🟡 built 14 Nov, branch `ours-default`: `pixi run sim` runs `stack:=ours`; the stack's switches are declared empty ("the stack's choice"), so any switch given by hand wins, `costmap:=nav2` too; `stack:=stock` is Nav2's. The car unchanged. sim-check's default run checks ours; `measure controllers`, `measure timing` and `ndt-map drive` start from `stack:=stock` so their scores stay comparable. Launch tests pass; not yet run in a simulator. ✅ **18 Nov (the docs audit):** done: `stack:=ours` is the simulator's default and has run in every sim-check since; `stack:=stock` went with Nav2 on 17 Nov (184), so the stack's choices are `best`, `car` and `ours` |
| 182 | **The last Nav2 pieces replaced**: the velocity smoother (ours, `aav_smoother`, made champion once it scores), the stop zone (Nav2's collision_monitor: a safety lock, ours in Rust with its own tests first), the map server (becomes the map chunks, 175), the lattice file's generator; `nav2_msgs` types kept or ours | ✅ 17 Nov: **the stop zone swapped.** Ours ran in shadow on every obstacle check (`--obstacle`, `--lidar-obstacle`, `--kerb` both ways, `--moving`, the parked box, `--faults`), 3 rounds: Nav2 stopped 24 times, ours on the same command all 24, never later (the nodes' own logs; sim-check's message timing, at load 10-34, saw ours' messages up to 1.95 s late with the car standing, 8 cm once: delivery, not decision). Now `aav_stop_zone` is the stop zone everywhere (`stop_zone:=`; `/safety/stop_zone/state`, `status`, `box`; stop_hold reads it). The smoother: ours, the champion (Nav2's a tie, 16 Nov). The map server: ours (`aav_costmap` `map_server`). The lattice generator: Nav2's, run from `vendor/`. `nav2_msgs` kept ([Nav2, as it was](docs/history/nav2-as-it-was.md)) |
| 183 | **Gazebo's last job replaced**: Ravensim's cameras on a computer with no GPU (software rendering, e.g. Mesa's lavapipe), so CI and oirp-spss run Ravensim | ✅ on a Mac, 16-17 Nov: the cameras draw on lavapipe (`tools/software_gl.sh`), and the clock waits for each picture (**camera lockstep**, `camera_lockstep:=auto`): `software_gl.sh --force pixi run sim-check --obstacle` passes, the 3 cameras at 10.0 Hz in sim time, the clock held at every picture, never timed out ([cameras](docs/own-simulator-cameras.md#camera-lockstep-16-nov)). CI's `sim` job runs it: **untested on Linux** (240) |

### The whole car: every design merged (14 Nov)

| # | Task | Status |
|---|---|---|
| 155 | **The lane finder as a pose sensor**: its offset in the lane, against the lane map, into the EKF as a sideways correction; the lane driver stays as a challenger | ✅ 17 Nov: **ours' default** (`stacks.yaml` pose `ekf_lane`, on where there is a camera and a lane map). With the EKF steering ours' stack in our simulator: University Drive 9.2-9.9 → 6.9-7.8 cm RMS sideways, its box tour 7.1-8.2 → 1.9-2.1 cm, jitter down a quarter to a half; the campus tour no change (9.1 → 9.4, little paint on its way); sway and off the lane's centre no worse ([trial](docs/trials.md#with-the-ekf-steering-ours-stack-17-nov-now-ours-default)). Left: the car |
| 162 | **The behaviour obstacle leg fails** (`sim-check --behaviour`, 8 Nov and 14 Nov, Gazebo and ours): with `plan_around:=false` the car still steers round the box, so neither APPROACH(obstacle) nor the stop zone fires. Likely the local costmap still sees it (unproven). The design wants a parked box passed and a person waited for: decide what the leg should expect | ✅ 15 Nov: the cause, proven: `plan_around:=false` hides the box only from the planner; the follower's own local costmap (Nav2's MPPI, or mppi_lag) steers round a 1 m box when there is room, and at 1 m/s the stopping law's cap binds only in the last 2 m, so the car was beside the box before APPROACH(obstacle) could fire. A coin toss (2 of 4 on 8 Nov; ours stopped once on 15 Nov). Decided: a parked box to pass is `--behaviour-things`' leg; `--behaviour` now puts a 10 m wall across the road (nothing to go round): APPROACH(obstacle), the stop zone at 0.45 m/s, 0.94 m before it, goal reached (ours, our simulator); stock (Nav2): 0.33 m/s, 1.04 m, goal reached. One run each. A box in a walled lane was tried first: after rocking, the car faced a side wall at an angle and stopped for it ([stopping](docs/stopping.md#the-obstacle-leg-a-wall-across-the-road-15-nov-todo-162)) |
| 164 | **Score "all ours" again** (`navigator:=own planner:=own costmap:=own tracker:=mppi_lag`): until 14 Nov `mppi_lag.yaml` set `costmap_topic`, which beat the launch file's choice, so with `costmap:=own` mppi_lag read Nav2's costmap topic, which nobody published, and drove blind to its costmap. The 8 Nov all-ours scores (lot 7 of 12, campus 4 of 6) were with that. Fixed and tested; sim-check all ours with `--plan-around` now passes (2.21 m clear of the box) | ✅ 15 Nov, re-scored after the sway fixes (TODO 168) and ego's pedal (206): `measure controllers --stacks own --tours lot campus --runs 3 --reversing`, our simulator. **Lot 12/12, campus 9/9** (8 Nov: 7/12, 4/6; drive-k this morning: 12/12, 8/9). Sway lot 0.031 m/s² (drive-k 0.030; Nav2 0.017), campus 0.231 (0.210; Nav2 0.12). Off the path, mean 0.21 and 0.22 m (Nav2 0.31, 0.44); worst 1.57 m (lot, round the building) and 2.74 m (campus, round the bend, run 1 only; runs 2-3 0.76 and 1.05). Left-right changes per 100 m: 11.2 and 12.5. Results: /tmp/w162/m164 (not kept in git) |
| 165 | **The wiggle, what's left** (149's tests): the GPS's jitter through `ekf_global` (0.09 m sideways every 0.2 s in the sim; with the truth as the pose RPP flips half as often): follow in `odom` and apply the map's corrections slowly, or trust each fix less; `mppi_lag`'s flip a second (its sampling); our navigator keeps a valid path as the trees now do (done 15 Nov, TODO 168: `keep_path_age`, 10 s; and mppi_lag's flips, the steering noise by speed) | ✅ 15 Nov afternoon: the GPS jitter (`map_correction:=slow`) and the follower's own jitter (`mppi_lag_steer_gamma:=0.05`) and the plan's staircase (`plan_smoothing:=strong`), all three in the `ours` stack; sway at Nav2's (TODO 168) |

### One shape, and lanes as soft costs (13 Nov)

| # | Task | Status |
|---|---|---|
| 151 | **Placeholder cameras** (`sim_cameras --placeholder`) so GitHub's nightly drive check runs our simulator, not Gazebo; then real pictures on Mesa's software renderer | ✅ not needed: real pictures on lavapipe, in lockstep with the clock (183, 16 Nov); CI's nightly `sim` job runs Ravensim only now, on Linux untested (240) |

### Software, still to do (from the older lists)

| # | Task | Status |
|---|---|---|
| 98 | **The faint car behind every screen**: drawn on all four; the four screenshots are left | ✅ 15 Nov: the four Initial D pictures taken again from `--demo` (`docs/hud.md`, the themes table; the hardware panel's added), the faint car on each |
| 93 | **The cubes let the car into a post once** (1 of 3 runs, 3 Nov): run `sim-check --lidar-obstacle --voxels` again on today's stack | ✅ 15 Nov, our simulator, all ours, 2 runs: **2 of 2 stopped before the post**, 0.20 and 0.19 m short, by the stop zone, the local costmap 100 of 100 there; voxels 6.7 ms a cloud. The 3 Nov miss (Gazebo, Nav2's costmap) didn't come back. Left: Gazebo; more runs before the cubes go near the car |
| 116 | **Does the car stop if the Mac drops off?** The LLM on the Mac sends goals; check, then a heartbeat | ✅ 15 Nov: checked by reading: both goals run on the car (`/scripted_drive`, `/follow_gps_waypoints`) and a ROS action carries on without its client, so yes, the car kept driving. Now the goal leash (`aav_safety` `goal_leash`): the laptop beats at 5 Hz, 1 s quiet cancels its goals (a normal stop), goodbye on a clean close; on the car, in the sim and on the bench; 4 + 2 tests. Not yet tried with a real laptop dropping off |
| 119 | **Foot-tall kerbs**: are they seen? Test a 0.3 m kerb in our simulator | ✅ 16 Nov: **yes, with the step finder** ([kerbs](docs/kerbs.md)). 15 Nov: not seen (the flat scan keeps only what stands 0.3 m over the road; the car drove into the kerb). Now `aav_voxels_rs` finds steps (a point 0.10-0.5 m above another within 0.3 m, flat; `steps.rs`), keeps them 10 s in odom (the roof lidar loses a kerb 3-4 m before the bumper; `memory.rs`), on `/perception/voxels/steps`: the local costmap's `kerb_layer` and the stop zone read it (`kerbs:=true`, on in `stack:=ours`). `sim-check --kerb`: seen 8.4 m past the bumper, the stop zone stopped the car 0.82 m (lidar) and 0.80 m (lidar and radar) before it; no worse on the lot and University Drive (2 runs each). Cast clouds: a 0.15 m kerb from 4.7 m, a 0.10 m one never; no steps on slopes or ramps to 10°, camber, a 7.5 cm bump, grass. The campus map already had every road edge as a lethal kerb (OSM has no kerb tags there). Left: 218, 219 |
| 220 | ✅ **Stop lines into the road ahead** (TODO 175, step 6): the behaviour planner reads `/drivers/nav/planner/route_stops` (Lanelet2's stop lines on the route) and stops at each; then the `/map` readers left (`aav_things`' background, the screens) onto the chunks. **17 Nov:** each stop line on the plan is a stop in the road ahead (`stop_line`, WAIT, the front bumper on the line); standing within 1.5 m of it 2 s serves it, then GO, as at a seen stop sign (`StopLines`, Python and Rust, the shared vectors exact). **The `/map` readers, not small, left:** `aav_things`' background (its NDT-chunk background, `things_background_chunks:=`, is a different source and wants a lot tour against `/map` first); the map window (`aav_hud`, raylib) and the dev window draw `/map` and need a chunk loader each; our costmap reads no `/map` with `costmap_chunks:=auto local_chunks:=true` |

### Closed on 15 Nov

Done another way, fixed already, or no longer making sense. Their words are in the history.

| Rows | Why closed |
|---|---|
| 111, 112, 113 | fixed in the code: the radar's short frame is guarded; `vehicle.yaml` has no duplicate keys; rear-camera signs never trip the lock |
| 124, 149 | done (the wiggle's sources found; what's left is 165) |
| 90, 107, 73 (merge) | one screen per concept, done another way: the Bevy developer window (114) and the hardware panel |
| 115 | our simulator: built, the default since 8 Nov |
| 102 | the behaviour state machine: built (the behaviour planner, 170) |
| 103 | the tracker: built as `aav_things` (156), names from the camera (192) |
| 95 | NDT in the demos: `demo` turns NDT on when its map is there |
| 77, 97 | the 0.2 release notes: 0.3's notes replaced them; 0.4-rc1 gets its own |
| 91 | PR #21: long merged; later work went as #27-#29 |
| 0, 2, 4 | the controllers: 0 joined 101; LQR and `mpc_full` built, challengers in the league (150) |
| 5, 16 | emailing as work lands is a habit, not a task; the short scripted drive not seen since Cyclone |
| Ideas to keep (28-30 Oct) | built: the hardware workbench, debug HUD and two screens (`hw_panel`, the developer window); the preflight (its car run is above); the cubes on the HUD, for Nav2 and as named things (`aav_voxels`, `aav_things`, 192); AMCL (a trial); one model-based controller (LQR, MPC, `mpc_lag`); the four research picks (NDT, MPC, the cubes, the safety case 159) |
| 10-11 Oct lists | done: the C++ in Rust, the raylib HUD, the full sim graph, the lidar on by default, `cargo fmt` and `clippy` in `tools/test.sh`, the small fixes in `docs/work.md`, the car kit (slim environments); `measure controllers` now skips a busy ROS domain |
| 10 Oct, on Ben's Mac | the old `aav/aav` folder is gone (its snapshots, RViz there); `best.pt` replaced by RF-DETR; Ben's questions answered |
| N2, N7 | the HUD sends the e-stop five times, reliably; the wiggle is 124, 149 and 165 |
| "Later" | Rust: done; measuring the car and the secrets: in the lists above |

## The whole car: every design merged (14 Nov)

Ben, 13-14 Nov: "a review of everything with the plan of merging everything into the best version possible ... what am I missing". The page: [the whole car](docs/designs/2026-11-14_the-whole-car.md). His words: `docs/history/messages-2026-11-14.md`.

| # | Task | Status |
|---|---|---|
| 153 | **The whole car** ([design](docs/designs/2026-11-14_the-whole-car.md)): one picture of the world (the grid, the things, the road ahead), one car model, one scorecard; what is missing; the order | ✅ 14 Nov proposed; Ben: build it, cut nothing |

## One shape, and lanes as soft costs (13 Nov)

Ben, 13 Nov: "How can we make all of this fit together better", and lane lines "it can cross but only if it needs to and to stay on the right side". The page: [one shape](docs/designs/2026-11-13_one-shape.md).

| # | Task | Status |
|---|---|---|
| 147 | **One shape** ([design](docs/designs/2026-11-13_one-shape.md)): nine levels, one world model (costmap, a list of things, one pose), one boss with a contract, 36 switches down to 8 | ✅ 13 Nov proposed; Fable reviewed (go, with changes; folded in). ⬜ Ben: D1-D7 |

## A heavier car: the adaptive brake (8 Nov)

Ben, 8 Nov: "if we brake without a person vs with a person ... we are going to need to dynamically realize that we aren't braking as fast as we expect and start applying more force. We don't want to overwhelm the brakes though, so we might want to know what the max values are like the steering system." The page: [a heavier car](docs/stopping.md#4-a-heavier-car-the-adaptive-brake).

| # | Task | Status |
|---|---|---|
| 145 | **The adaptive brake** (`aav_description` `adaptive_brake.py`; `adaptive_brake:=true` in the sim): measured against the empty car model's slowing, more brake for a car that stops too slowly; never above `brake: safe_max`, a rise rate, `max_hold_time` near it; the car model's `payload`; `sim-check --loaded-stop` | ✅ 8 Nov built and tested without ROS: from 2 m/s with 150 kg aboard the car model stops 0.58 m past the empty plan, +0.04 m with the adaptive brake (the page). ✅ 8 Nov `sim-check --loaded-stop-only`, our simulator, 5 of 5: 150 kg without it +0.87 m past the empty model; with it +0.21 m, the next stop +0.05 m; empty +0.04 / +0.05 m either way. ⬜ car day A7 rows 6-8: the brake's real limits into `vehicle.yaml` `brake:`. ⬜ on the car: in `legacy_uart_bridge`'s `pi` loop (Python, the same module), then spine-core in Rust (a twin with shared vectors); the wheel speed's noise first (8 spokes a turn) |

## Off Nav2, one piece at a time (8 Nov)

Ben, 8 Nov: our own versions beside Nav2, each until it wins; Nav2 stays the default and on the car. The order: the costmap, a Rust path planner, the navigator.

| # | Task | Status |
|---|---|---|
| 143 | **Our own costmap** ([page](docs/own-costmap.md), `aav_costmap`, `costmap:=own`): Nav2's layers in Rust (the map, the scan's marks with ray clearing, inflation round the car), local (odom, rolling) and global (the map), on `/drivers/nav/costmap/*`; `mppi_lag` reads ours | ✅ 8 Nov built, 30 tests without ROS; in our simulator 100 % of 40,000 cells the same as Nav2's (stopped at the post, 2 of 2); 4-5 % of a core for both, Nav2's local one 7-9 %. mppi_lag on ours, 3 runs each: plan around 3 of 3 (2.1-2.5 m clear; Nav2's 3 of 3, 2.4 m); lidar obstacle: stopped 0.20, 0.21 m (Nav2's 0.17-0.71), but once steered past the post and once never reached the goal after it went (Nav2's 3 of 3). ✅ 8 Nov: the trees' recovery clears ours too (`clear_entirely_local`, bt_navigator remapped): lidar obstacle again 3 of 3 to the goal (stops 0.64, 0.15, 0.18 m; 100 % of cells the same).; ⬜ radar and camera clouds in the sim; ⬜ the planner on `/drivers/nav/costmap/global_raw` (next piece) |
| 144 | **Our own planner** ([page](docs/own-planner.md), `aav_planner`, `planner:=own`): Nav2's State Lattice in Rust (the primitives file, nav2.yaml's penalties, the Reeds-Shepp finish, the smoother, the car's outline on the global costmap: Nav2's or ours); serves ComputePathToPose, bt_navigator remapped to it, Nav2's lattice beside it | ✅ 8 Nov built; 6 tests without ROS (the curves end on the goal; straight, U-turns, a parking-lot exit, a narrow gap, no path), all under 1.1 ms. Our simulator, 3 runs each with reversing: test lot **12 of 12** as Nav2's lattice (tours 182-189 s against 186-187), the same paths within 2.2 m, 3-37 ms a plan. Campus at 2.5 m/s: 4 of 9 against 3 of 9, both lost to MPPI's "Optimizer fail" at the start of the long leg; ours 168-195 ms there against 138-151 ms. ⬜ faster (cache each node's check where safe); ⬜ Hybrid-A* (forward only); ⬜ ComputePathThroughPoses; ⬜ TODO 140's P4 to Fieldhouse once MPPI gets past that leg; ⬜ the navigator step (next piece) |
| 145 | **Our own navigator** ([page](docs/own-navigator.md), `aav_navigator`, `navigator:=own`): bt_navigator's trees and waypoint_follower in Rust (plan, follow, replan, final approach, recoveries, through-poses, GPS waypoints by the datum), Nav2's actions under `/drivers/nav/navigator/`; goal senders switch by `navigator`; our planner also serves ComputePathThroughPoses; all ours = no Nav2 server navigates | ✅ 8 Nov built; 11 tests without ROS. Our simulator, all ours vs all Nav2, forward only: test lot 7 of 12 legs against 8 (3 runs; the U-turn 1 of 3 against 0); the box 3 of 3 both; sim-check all ours, 13 of 13 pass; campus at 2.5 m/s 4 of 6 against 2 (2 runs); P4 to the Fieldhouse 1 of 2 against 0. Why Nav2 gave up "along the road": lidar marks beside the lot exit where leg 1 ends make the car's own spot occupied ("Start occupied", MPPI "Optimizer fail"); ours clears and replans. ⬜ `mppi_lag` overshoots the campus's first goal at 2.5 m/s (under Nav2's navigator too); ⬜ its CPU (2x Nav2's MPPI); ⬜ not the sim default until those |
| 146 | **Our own velocity smoother** ([page](docs/own-smoother.md), `aav_smoother`, `smoother:=own`): Nav2's velocity_smoother's speed limits, timeout, deadband and feedback, with the turn rate from the steering actuator's rate and full lock (vehicle.yaml through the car model), under Nav2's node name; `measure controllers --smoothers`, and the new row "steering commands ego had to clip" | ✅ 8 Nov built; 16 tests without ROS. Our simulator, all ours, test lot, 3 runs each: 10 of 12 legs both; ego clipped 0-0.1 % against 9-15 %; off the path 0.12 m against 0.15 m (worst 0.65 against 0.92); tour 701 s against 890 s. ⬜ campus at 2.5 m/s; ⬜ `tracker:=mppi_lag` under Nav2's navigator; ⬜ lateral-acceleration cap (ego has it); ⬜ not the default until those |

## 0.3: diffusion on the car, as a trial (7-8 Nov)

Ben, 7 Nov: "We want diffusion." His choice: on the car in 0.3, a trial, low speed, behind the guard and the locks.

| # | Task | Status |
|---|---|---|
| 133 | **Diffusion for 0.3** ([design, As built 0.3](docs/designs/2026-11-02_diffusion-planner.md#as-built-03-7-8-nov), [trials](docs/trials.md#diffusion-7-8-nov-the-u-turn-the-orin-the-car)): why the guard refused (G5 turns, G6 at the U-turn's cusp); each gear smoothed on its own, a wider second try for G5/G6 only, U-turn data, the map encoded once; record and replay plans offline; TensorRT/CUDA/CPU with fallbacks, FP16 export, `pixi run measure diffusion`; the car gate `diffusion_trial:=true`, 0.5 m/s ([car day A12](docs/car-day.md#a12-the-diffusion-planners-trial), test 4.27) | ✅ 8 Nov, branch `worktree-agent-a0cbc7e904c345e4d`: offline 77 → 87 % of plans with a passing sample, G5 2,197 → 850. Simulator, 3 runs each: 77 → **88 %** of plans from the model (84, 89, 92 %; 92 % after the merge), the U-turn 40 → 72 %, 12 of 12 legs, 171-175 s against the lattice's 180-185 s. ⬜ the Orin's timing (TensorRT), the car trial (A12) |

## 0.3: directions on the campus (7 Nov)

Ben, 7 Nov: Tesla-like "higher level navigation for directions"; for 0.3, on the campus. The page: [On the campus](docs/routes.md#on-the-campus).

| # | Task | Status |
|---|---|---|
| 140 | **Directions on the campus**: a place's name (or lat/lon, or a map point) to a route on the campus's own road graph (`campus_graph.py`, from the `.osm` Gazebo's world comes from), cut into Nav2 GPS waypoints; `street_router router:=campus`, `/drivers/nav/route/place`; `pixi run campus-route`; 19 named places in the site file (`campus_site.yaml`, TODO 100's first piece) | ✅ 7 Nov built and tested without ROS; in Gazebo the route goes to Nav2. ❌ 8 Nov Gazebo, P4 to Fieldhouse (453 m): **did not arrive**; 310 m driven, stuck half-turned at a sharp bend (each bend is a stop-and-face goal). ⬜ bend waypoints before the corner, or `navigate_through_poses`; then the run again. ⬜ real drop-off points (`stop:`) measured on the day; a screen to pick a place (the screens' agent); ✅ 8 Nov our navigator (TODO 145) passes the waypoints as one through-poses goal: P4 to the Fieldhouse reached 1 of 2 (263 s; Nav2's waypoint_follower 0 of 2, stuck at the lot exit) |

## 0.3: ego knows the car (7 Nov)

Ben, 7 Nov: "we really want the ego to have a mathematical model for steering, braking and acceleration ... have the algorithms know about lag". The page: [one car model](docs/car-model.md).

| # | Task | Status |
|---|---|---|
| 127 | **One car model**, Python (`aav_description` `car_model.py`) and Rust (`car-model`), shared vectors; dead times; the bicycle; `step`, `predict` | ✅ 7 Nov. Gazebo unchanged |
| 128 | **ego's lag lead** (`lag_lead: auto`, sim only), off under a lag-aware driver (`/drivers/<slot>/lag_aware`) | ✅ 7 Nov, sim. ⬜ `on` for the car after tests 3.10-3.12, 4.1-4.5 |
| 129 | **ego's trace** (`/ego/trace`, 2 s, `base_link`) | ✅ 7 Nov; the HUD and the developer window draw it (136). ⬜ drivers match it to their plan |
| 136 | **The squiggly line** ([page](docs/hud.md#the-squiggly-line-ahead)): ego's trace (bold, green / amber / red by the gap to the plan), the plan (faint), the stop bar from `/ego/stop_point` (the car model, full brake now), the room to the goal or an obstacle; on the HUD (AHEAD) and the developer window (key T), pretend data too | ✅ 7 Nov built. ⬜ stop lines (no topic yet); the line drawn on the camera picture (Q22); the car's real numbers (tests 3.10-3.12, 4.1-4.5) |
| 130 | The lag-aware drivers publish `/drivers/nav/lag_aware`: `aav_track`'s MPC, the Rust MPPI | ✅ 7 Nov: both (rows 131, 132) |

## 0.3: how the car stops, and the behaviour planner (7 Nov)

Ben, 7 Nov: "We need navigation planning and lane following and we need an obstacle avoidance planner to avoid tricky situations ... we can use that to say if we have turned enough or slowing down enough". His picks for 0.3: the behaviour planner and a stop-point check. The page: [how the car stops](docs/stopping.md).

| # | Task | Status |
|---|---|---|
| 137 | **The behaviour planner** (`aav_behaviour`, `behaviour:=true`, a trial): DRIVE, APPROACH, STOPPED, WAIT, GO, ASK; a speed cap on Nav2 (`/drivers/nav/speed_limit`) from **one stopping law** (`aav_description` `stop_law.py`, TODO 110) for a stop sign, an obstacle in the lane, the fence and the goal; `/drivers/nav/behaviour/state` for the screens; the watcher's `/perception/stop_sign/distance` | ✅ 7 Nov built, every transition tested; the sim runs: see the page. ⬜ the `patient` progress checker, ASK's reply, the trackers reading the cap, the car (stage gates) |
| 138 | **The stop-point check** (`pixi run sim-check --stop-point`): drives at 1, 2, 3 m/s, slows at 1 m/s², and scores where the car stopped (truth and localization) against the car model's `stop_point`; `tools/stop_points.py` can score a car bag later, with a tape for truth | ✅ 7 Nov, numbers on the page. ⬜ on the car, after tests 4.1-4.5, with the real numbers |
| 139 | **Wiring**: the shared `car-model` crate behind `sim-core`'s and `mppi-lag`'s `CarModel` traits (feature `shared`) | ✅ 7 Nov, tested against their own copies. ⬜ the nodes switched over, after a quiet-machine sim-check |

## 0.3: Gazebo merged into Rust (7 Nov)

Ben, 7 Nov: "Does the bevy tool still use gazebo in the background. I would love to merge in everything from gazebo so its standalone in rust." His picks: everything in 0.3, all sensors.

| # | Task | Status |
|---|---|---|
| 134 | **Our own simulator's core** ([page](docs/own-simulator.md), `aav_simulator`, `sim simulator:=own`): Gazebo's worlds (test lot, road, campus), the car (a `CarModel` trait, the car-model crate's rules), the roof lidar and the radar stand-in by ray casting, IMU, GPS, wheels, truth, `/clock`, TF through `/joint_states`; the car stops at what it hits; spawn and delete services; `sim-check --simulator own` | ✅ 7 Nov built; on a loaded Mac: drop-in pass (no cameras), a Nav2 goal reached, `sim-check` pass but the box run's last leg (a late TF), `--lidar-obstacle` pass. ⬜ the cameras plug in (another branch); `car-model` behind the trait; a quiet-machine run beside Gazebo's spread; then ours the default |

## Saturday 7 November: release 0.3, our own simulator

Ben, 7 Nov: replace Gazebo with our own Rust simulator, every sensor. The core (world, car, lidar, IMU, GPS, wheels, clock, TF) and the cameras are built side by side.

| # | Task | Status |
|---|---|---|
| 135 | **The simulator's cameras** ([page](docs/own-simulator-cameras.md), `aav_sim_cameras`): Bevy offscreen, Gazebo's four camera topics, `CameraRenderer` for the core. Left: feed it the core's world and truth; read Gazebo's SDF worlds; a no-GPU path for CI; compare with Gazebo's pictures; the detector on its pictures | ✅ built, test world only. ⬜ merge with the core |
| 141 | **The cameras joined to our simulator** ([page](docs/own-simulator-cameras.md)): the core's worlds (test lot, road, campus: boxes, cylinders, planes, OBJ meshes, their pictures, the sky and sun) drawn by `sim_cameras`; `simulator:=own` starts them, so perception, lane and faults run on ours. Also the ground under each wheel (slopes tilt the car, a 0.3 m kerb stops it) | ✅ 8 Nov: `sim-check --simulator own --lane` all pass; drop-in: every camera at 10 Hz. Head to head on a quiet Mac (`sim-check` x 3, `--obstacle`, `--lidar-obstacle`, `--speed-steps`, `--lane-only`): ours passes all, Gazebo the same but one start-up. **The default since 8 Nov:** `--perception`, `--faults`, `--drive-to-point-only` 3 of 3 each on ours; placed models drawn in the pictures. Left: the one stop-sign miss (1 of 8, cause unknown); computers with no GPU use `simulator:=gazebo` |
| 142 | **Lane following for 0.3** ([page](docs/perception.md#in-the-simulators-8-nov)): `measure lane` scores a long drive by the truth (road world, campus roads); the campus's painted roads get edge lines | ✅ 8 Nov on ours: Library Road 171 m at 1.4 m/s, error mean 4 cm, 95 % 13-15 cm, lane lost in 0-1 % of pictures. Gazebo on the campus: lane lost in 68 % (not diagnosed). Left: unpainted service roads; real pictures |

## Saturday 7 November: 0.3, the algorithms know about lag (MPPI)

Ben, 7 Nov: "have the algorithms know about lag". MPPI out of Nav2 into our own Rust driver, its model with the actuators' lag; Nav2 keeps planning.

| # | Task | Status |
|---|---|---|
| 131 | **MPPI that knows the lag** (`aav_mppi_lag`, `tracker:=mppi_lag`, [page](docs/mppi-lag.md)): Nav2's MPPI and its eight critics in Rust, on a lag model behind a `CarModel` trait (the shared car model's rules); `/drivers/nav/lag_aware` true while it drives, for ego's lead | ✅ 7 Nov built, tested without ROS. ⬜ Gazebo against Nav2's MPPI on a quiet computer (7 Nov's runs: the odometry stalled under load and it gave up; the page). ⬜ at merge: the shared `car-model` crate behind `CarModel` (its rules matched already). ⬜ the car's real lags (tests 3.10-3.12, 4.1-4.5), then the same runs again; the Orin's tick time (estimated, not measured) |

## Wednesday 4 November: the weak links, for Thursday's pick

Ben, 4 Nov: "think about the areas of this car and what is the weakest ... ideas on how to improve each section of the driving experience", then "save this full list". The list: [the weak links](docs/designs/2026-11-04_weak-links.md) (every area, the evidence, the ideas, the papers). Words: `docs/history/messages-2026-11-04.md`.

| # | Task | Status |
|---|---|---|
| 99 | **The weak-links list**: every area, what's weakest, ideas tagged eng / res; the three research proposals (A: identification + a CBF safety filter; B: the lidar teaches the camera; C: degeneracy-aware NDT); the site file; the Nav2 verdict; the Drive screen | ✅ 4 Nov: `docs/designs/2026-11-04_weak-links.md`. ⬜ Thursday: Ben picks |
| 105 | **Every research idea judged, and the new ones** (Ben, 4 Nov: "look at all the research ideas we have ... what improvements or edge can we bring"): the 20 ideas we had, each kept, folded or dropped; 12 new ones (shadow mode, the winter lot, the blind-zone memory, the radar's own tracker, the tuner on the harness, the VLM that may only say STOP, the brake-health monitor, speed by trust, shielded learning, pole landmarks, the truth rig, stop-and-stare); three threads (knows itself, the winter lot, learns safely) and a month-by-month plan | ✅ 4 Nov, corrected 5 Nov after the EGM review: a backlog, not a plan; "drop" is now "not pursued; stays available". ⬜ Ben: its proposals |
| 106 | **One great car** (Ben, 4 Nov: "taking the best of everything we know ... Simplify down to the best parts of each algorithm and put them all together into the best of all worlds"): the direction after 0.3; a convergence table, one winner per area, in [one car](docs/designs/2026-11-04_one-car.md) | ✅ 4 Nov written (one car, section 4). **Corrected by Ben, 5 Nov:** one design is for down the road; now we build new algorithms of our own from papers, known algorithms and current implementations. Ben, 5 Nov: keep every algorithm, a new best hybrid per area, **how the car stops first**, stock stays the default until ours wins. ⬜ the stopping design, after the EGM stages |
| 109 | **The EGM review of the 4-5 Nov designs** ([report](docs/designs/2026-11-05_egm-review.md)): none passed; all eight corrected after it, and all but `one-car` are now far too long for "short pages" | ✅ 5 Nov, uncommitted. ⬜ cut them down: the stopping design takes Part 3 of `knows-itself` and the requirements in `behaviour-and-speed`; `one-window` can split off SWAP |
| 110 | **How the car stops** ([design](docs/designs/2026-11-05_how-the-car-stops.md)): Ben's first hybrid, one stopping law from `approach.py`, `dynamics.py`, `speed.py` and `geofence.py` | ✅ 7 Nov `stop_law.py` (row 137), with the speed loop's lag in ρ; the trackers use it, the point driver with `stop_law:=hybrid`. ⬜ the point driver in Gazebo, 10 runs against `approach`; the CBF filter |
| 117 | **One RViz** (Ben, 5 Nov: "two nav screens"): `demo_all.sh` says `rviz:=false` | ✅ edited 5 Nov; not launched yet |
| 118 | **Everything is a drop-in** (Ben, 5 Nov: "just swap out and test"): one conformance check against `docs/interfaces.yaml` for any sensor source or algorithm, and find what still depends on Gazebo by name | ✅ 5 Nov: `pixi run drop-in`, the contract's `drop_in` rules, `/vehicle/sim/` and `/control/sim/`, target `sim`. Against real sensors: not yet |
| 123 | **MPPI held the car near 1 m/s** (found 5 Nov): its 3 m of path was shorter than its futures at any higher speed. The path now grows with `sim_speed`; the campus defaults to 2.5 m/s (mean 2.2) | ✅ Gazebo only |

## Tuesday 3 November: 0.2 polished for the meeting (4 Nov), then the car

Ben, 3 Nov: "Tomorrow is meeting day and we want to have 0.2 fully polished and documented ready to go." The plan: [the brief](docs/designs/2026-11-03_release-0.2-brief.md), "Recommendations for today". Words: `docs/history/messages-2026-11-03.md`.

| # | Task | Status |
|---|---|---|
| 85 | **The 0.2 guide**: every feature, what it does, how to run it, tips, how solid; the state machine; the two EKFs; which parts are Python | ✅ 3 Nov, `guide-0.2`: `docs/release-0.2.md` is now "The 0.2 guide" (same file and PDF): Your first hour, the defaults, every feature as What / How to run / Tips / How solid, the state machine's diagram and its gap, the two EKFs (they meet only in TF), the Python table. ⬜ its defaults table after TODO 88; its cockpit block after TODO 87 |
| 86 | **Everything tested in the simulator** (Ben: "I would like to see diffusion tested in simulation. Acutally everything"): sim-check, bench-check, every switch launched once, diffusion against the lattice 3 runs each | ✅ 3 Nov, `sim-all`: sim-check 9/9, bench-check 11/11, **23 of 23 switches** finished a leg, every sim-check option passes (`--lane` after a fix). Diffusion 12/12 legs, 73 % of plans from the model (2 Nov: 63 %), tours 177, 179 and 270 s against the lattice's 183.4 ± 0.7 ([trials](docs/trials.md)) |
| 87 | **The cockpit as the one driving screen**: the localization views and the running algorithms; screenshots from live Gazebo | ✅ 3 Nov, `cockpit-drive`: an **Algorithms** table from the new `aav_run_info` (`/system/algorithms`, filled from the launch arguments); NDT's and AMCL's poses, `/map`, KISS-ICP's speed; tabs for the localizers, the tracker swap, the switches; screenshot live in the test lot (`docs/img/cockpit-drive.jpg`). ⬜ not showing: the EKFs' arrows and NDT's scan (two layout scripts, 6 alerts to read at the Mac); NDT's map and KISS-ICP's cloud need publishers; frame labels clutter the car; the bench doesn't start `run_info` |
| 88 | **The best defaults**: one table, default where and why; sim winners switched only with their numbers | ✅ 3 Nov: no switch. NDT-EKF 0.018 m against 0.174 m, but needs the lot's map (not in git); the cubes slower and once drove into a post. Tables: `docs/switches.md`, the guide |
| 89 | **The ESP32 in QEMU** on Ben's Mac (`esp tools`, `esp qemu-check`): the heartbeat brake proven before A8 | ✅ 3 Nov: **PASS** at load 17. 6 of 6 cycles (silence, frozen, bad CRC): full brake and DAC 0 in 112-119 ms, latched until a stop; uplink 50 frames a second, 0 lost; RC via ROS, RC direct and a lost RC, 6 of 6. Found and fixed: the checker crashed on a log line QEMU cut short (a busy Mac); such lines are now counted and skipped |

## Monday 2 November: Ben's asks for 0.2

Ben, 2 Nov: "I would like the hud changes in, same with for the map. Something nicer then catpucine. We also want diffused planning, and joystick control ... We want to develop a list of tests." Words: `docs/history/messages-2026-11-02.md`.

| # | Task | Status |
|---|---|---|
| 80 | **The Initial D HUD in** (`hud-initial-d`) | ✅ 2 Nov: merged into `drive-plan` |
| 81 | **Initial D red and black on every screen**: HUD, map window, debug screen, hardware panel (Ben: in place of Catppuccin) | ✅ 2 Nov, `theme-initial-d`: the default on all four; its own map, grid and 3D colours, sharp corners; contrast tests for every mark, the e-stop 5.2 : 1. Screenshots in `docs/hud.md`. Seen with demo data only. ⬜ tests F.7, F.8 on the car's monitors, day and night |
| 82 | **The joystick on the Motor Pi alone** (TODO 74a): the joystick-alive check, so a dead pad stops the car | ✅ 2 Nov, `joy-alive`: `joy_alive` between `joy_node` and `teleop_twist_joy`; SILENT (0.5 s), FROZEN (2 s, deadman held, stick bit-identical), latched until the deadman is released; `/drivers/joy/alive`. bench-check 11 of 11. ⬜ car day: the real pad (A9b 6, 6b, 7; tests 3.J1-3.J3), tune `frozen_after`; (b) the throttle cap, (d) the button and axis numbers |
| 83 | **The diffusion planner in 0.2** (TODO 78; Ben, 2 Nov: "Put it in 0.2"), a trial beside the lattice, off by default, with the guard | ✅ 2 Nov, `diffusion-build`: `planner:=diffusion` (the car refuses it); 45,092 teacher paths, 3.9 M weights, ONNX, 75 ms on the CPU; guard G1-G8. Gazebo lot tour, 2 runs: 8 of 8 legs, 178-182 s (lattice 186-188 s), **63 % of plans from the model** (design: 90 %), the U-turn weakest. 0.3 (TODO 133, 8 Nov): the U-turn and G5/G6 fixed offline, the Orin ready to measure, the car gated. ⬜ the Orin, the car |
| 84 | **The list of tests**: `docs/tests.md` gets the new parts' tests (theme, joystick-alive, diffusion) | ✅ 2 Nov: 1.16-1.17 the screens, 1.18-1.19 diffusion, 3.J1-3.J3 the joystick, F.7-F.8 the monitors |

## Sunday 1 November: the ideas before the second PR

Ben, 1 Nov: "do all the ideas. I like them all but car day. Keep that in a list fo things todo".

| # | Task | Status |
|---|---|---|
| 55 | **The cubes in RViz** (`voxels:=true`) | ✅ 1 Nov, `ideas-1nov`: a "Cubes" display (0.25 m boxes) in `road.rviz` and `drive.rviz` |
| 56 | **The cubes and Betsi's health checks on the debug screen** | ✅ 1 Nov, `hud-cubes-health`: Nav2's cubes as red wireframes (key V), a "Nav2's cubes" section (road fit, count, ms) and "Localization health" (OK, or WARN and each check) on the seeing screen; rows in the sensor panel. Seen in `--demo`; untested live |
| 57 | **Health in `pixi run check`** | ✅ 1 Nov: one line from `/localization/health` at the end; a WARN never fails the check |
| 58 | **KISS-ICP into `ekf_local`** (`kiss_ekf:=true`, a trial) | ✅ 1 Nov, `kiss-ekf`: speed and turn rate only, covariance measured against Gazebo's truth. Not a clear win yet: ekf_local 0.30-1.27 m per 100 m on, 0.44-1.91 off, one run each on a busy Mac. Stays off. Next: 3 runs each on a quiet machine, then ekf_local's process noise; refit on a car bag |
| 59 | **The lattice as the default planner** (54b) | ✅ 1 Nov (Ben): everywhere that may reverse, the car too (TODO 62); Hybrid-A* forward only (the lattice lost the U-turn) |
| 62 | **Reversing and the U-turn on the car** | ✅ 1 Nov (Ben: "reverse and uturn should be in"): `reversing:=true` on the car again where the lidar sees behind; its planner the lattice (Hybrid-A* forward only). Tests: stands 3.2b, ground 4.23-4.26 (tests.md 4f). ⬜ car day: run them. Until H5, the speed's sign is the command's |
| 63 | **The ground plan: maps and scores from the first drive** | ✅ 1 Nov: [tests.md 4e](docs/tests.md#4e-record-the-lot-the-maps-and-the-scores) (record with the lidar, the wheel diameter, NDT's map, score the trials on replay) and the stack for every ground test, pinned by `test_car_launch.py`. `tools/record.sh --lidar`; `ndt-map build --poses` for a real drive. No `best:=true`: the best of the trials *is* the defaults |
| 66 | **The lab's Ubuntu desktop** (stays in the lab: CARLA, the Orin's full image) | ✅ 1 Nov: [car day session 11](docs/car-day.md#session-11-the-labs-ubuntu-desktop); Ben's MacBook is the ground station at the drive. ⬜ fill in: CARLA's version, Ubuntu's, the GPU; 11a the week before |

## Friday 30 October: the lattice, fairly

Ben, 30 Oct: "Reversing is always true i think".

| # | Task | Status |
|---|---|---|
| 54 | **Rerun the planners with reversing on** (hybrid, lattice, lattice32), 3 runs each | ✅ 30 Oct, branch `lattice-rerun`: every leg for all three; the lattice fastest, 184 s a tour against Hybrid's 213 s; 32 headings no better ([trials](docs/trials.md#planners-30-oct-reversing-on-hybrid-lattice-and-lattice32)) |
| 54b | **Ben: make the lattice the default planner?** | ✅ 1 Nov, Ben: yes (TODO 59). Was: Ben decides. Before: the bench, and forward only (the car's `reversing:=false` until H5), where the lattice still loses the U-turn |

## Thursday 29 October: control (Ben, 27 Oct)

Ben: "investigate LQR and maybe collapse all these various controllers into a single
controller ... closed loop". The plan: `docs/designs/2026-10-27_control-plan.md`; the ideas: "Ideas to keep" below.

| # | Task | Needs |
|---|---|---|
| 1 | **A page that explains it all:** PID, LQR, MPC and MPPI, with pictures; which loop runs where (ESP32, Jrk, Orin); what each knows. The next chapter of `docs/learn-control.md` ("Control from zero") | ✅ 29 Oct: [Controllers](docs/controllers.md), after Learn: control. The old RPP-vs-MPPI page is now `docs/path-followers.md` |
| 3 | **LQR for speed and brake (LQI):** one signed acceleration, in `spine-core`; scored against today's PI and P-only brake | ✅ 29 Oct: built (`speed_loop:=lqi`, gains from `pixi run python3 tools/vehicle_model.py --lqi`); the PI stays the default. No overshoot in Gazebo's steps (today's 10-16 %), settles in 4-5 s (11-13 s); at 3 m/s it stops 0.2-0.4 m past B (the PI 0-0.4): [Controllers, 2a](docs/controllers.md#2a-lqi-against-the-pi-results-29-oct). Also the bridge's smooth stop (Autoware's design): the jolt at standstill halves (2b). Left: a ramp feed-forward for the LQI's stop; nothing tried on the car |

Every gain still comes from guessed numbers until the car is measured (`docs/measure-the-car.md`).

## 24 October, night: the car as a linear model

Ben, 24 Oct: "Do we have a mathematical model for the car now for the ego? ... a matrix
control thing ... is that something we should consider? ... Maybe second order system".

| # | Task | Status |
|---|---|---|
| 52 | **The car's linear model**, in SYSC 3600's terms ([The car as a linear model](docs/vehicle-model.md)) | ✅ 24 Oct, `wt/vehicle-model`. `aav_ego` `vehicle_model.py` (no ROS; numpy, scipy.signal): speed K/(τs+1) with the throttle lag and the spine's PI (poles −0.28 ± 0.28j: settles in 13 s); the steering as a second-order system (ωn 12.6 rad/s, ζ 1.03); the bicycle linearised, x = [e_y, e_θ, δ], with linearised pure pursuit (ζ 0.707 at every speed); Bode margins with the delays (lateral, 1 m/s: 52° none, 45° typical 84 ms, 31° worst 259 ms); the discrete loops at 100 Hz and 20 Hz. 27 + 3 tests. `pixi run python3 tools/vehicle_model.py`: the tables, `docs/img/vehicle-model-*.png`. Found: the guesses disagree (drag 0.1 says full throttle holds 15 m/s; the open loop's `FULL_THROTTLE_SPEED` says 3); Gazebo's `steer_p_gain` 10 adds a steering lag after the actuator model: measured in Gazebo (headless, `steer-rate --sim` and `/joint_states`), its wheels follow the model's angle with a 0.110 s first-order lag (0.5 mrad rms) |
| 52c | Make Gazebo's steering match `vehicle.yaml` | ✅ 25 Oct, `wt/goals-steering`. Gazebo's wheels now follow the actuator model's angle with a **0.022 s** lag (was **0.111 s**; 2 runs each, 0.4-0.5 mrad rms left; a third on a busy Mac 0.063 s: the adapter's ticks stalled). Found: raising `steer_p_gain` alone made it worse (0.07-0.16 s): the URDF's steering joints had a 1.0 rad/s speed limit, which Gazebo enforces (the wheels turned at 0.63 rad/s, under the model's 0.8). Fix: the joints' limit 1.0 → 5.0 rad/s, `steer_p_gain` 10 → 100. The lag left is 1/gain + about 12 ms from the adapter's 50 Hz hold. Chose gain 100 over "the adapter's lag less 0.1 s" (11 ms off): `/vehicle/steering` stays the wheels' angle, and a new `steer_lag` needs no change in the xacro. Lateral phase margin in Gazebo 43° at 1 m/s (was 37°; the model 45°). `tools/gazebo_steering.py` measures it (with `actuator-test steer-rate --sim`); `test_vehicle_model_tool.py` keeps the extra lag under a quarter of `steer_lag`. `sim-check`: the scripted drive came up short (2.3-3.8 m of 6.5) with main's settings as with these, on a Mac at load 24 (other simulators): not this change. Next: the same step on the car (T2.7) |
| 52d | **The state-space primer** ([Learn: state space](docs/learn-state-space.md)), for Ben's SYSC 3600 refresher | ✅ 25 Oct, `wt/learn-state-space`. The 12 steps (why, free-body diagrams, Newton, states and A B C D, simulating, transfer functions, poles = eigenvalues, Bode and delay, what it means at 1 vs 3 m/s, shocks and roll, LQR and Kalman, a cheat sheet). `pixi run python3 tools/state_space_demo.py` (`tools/state_space_demo.py`, 16 tests): Euler, RK4 and exact side by side; a quarter-car and a roll model with GUESSED springs. Roll at 3 m/s on full lock: 2°, 46 % of the inside wheels' weight moves out, 0.32 g against a 0.70 g rollover limit: not a control problem; matters for the lidar's tilt. The kinematic bicycle holds to 3 m/s (the tyres' poles 4× faster than the steering's) |
| 52e | **Body roll tilts the lidar** (found by the roll model, [Learn: state space](docs/learn-state-space.md) section 10): at 3 m/s on full lock about 2° of roll, so the ground about 0.7 m off at 20 m, and could show as an obstacle in hard turns. Nothing to do at today's 1 m/s cap. Before raising it: check Nav2's obstacle layer's height filter against a rolled scan (Gazebo has no suspension, so it can't show this: a replayed bag from the car), or tilt-correct the cloud with the IMU's roll | ✅ 25 Oct, `wt/ekf-heading`: **safe up to 4 m/s, no change needed; today's 1 m/s untouched.** The ground filter is `lidar_scan`'s 0.3 m above the road (level body), marked out to 12 m (`lidar_layer` `obstacle_max_range`); collision_monitor's boxes are within 1 m of the bumpers, too close for the road to rise. `aav_lidar` `ground.py` (no ROS, 9 tests): where each VLP-16 beam meets a rolled road. The road shows above 0.3 m within 12 m from **1.46°** of roll. ego's 2 m/s² sideways limit holds the roll to **1.33°** at most (the springs' overshoot included, a sudden full lock from 1-4 m/s through ego's own model): the road at most 0.23 m up, 10 m out. At 5 m/s a sudden full lock overshoots (3.9 m/s², 2.4°): ego brakes 2 m/s² but the wheels turn at 0.6 rad/s. `tools/tests/test_lidar_roll.py` ties vehicle.yaml, ego.yaml, nav2.yaml and the roll model together. Not done: levelling the cloud with the IMU's roll, a trap: madgwick reads 2 m/s² sideways as 11.5° of tilt ([Costmaps](docs/costmaps.md#body-roll-and-the-lidar-todo-52e)). Left: springs and CG height are guesses: softer than ~7400 N/m a corner and the margin goes (measure day: the spring rates and the CG height, M7b) |

## 24 October, evening: the car-day kit

Ben, 24 Oct: "Lets get everything prepared so the only thing left it to get into the car."

| # | Task | Status |
|---|---|---|
| 51 | **The car-day kit**: one runbook, one preflight | ✅ 24 Oct, `wt/car-day`. [docs/car-day.md](docs/car-day.md) (in the docs as "Car day"): sessions 0-10, each with who, how long, what to bring, the commands, what you should see, what to do if not, and 🔒 where the hardware lead must answer first. `pixi run car-day-build` builds the ESP32 firmware (version 2 and `link-v1`) into `car-day/esp32/` (out of git), runs `esp qemu-check`, builds the C++ way back and the docs PDF; `pixi run car-day-check` (ROS-free, 35 tests) says OK / MISSING with the fix, or REMIND. Built on Ben's Mac: both ELFs, QEMU PASS (111-119 ms), the C++, the PDF, and `models/rf-detr-nano.onnx` (108 MB: the Orin then needs only TensorRT's `trtexec`). Found: on JetPack 6, RF-DETR can't run inside the stream node under Pixi (Python 3.12 vs JetPack 6's 3.10 TensorRT): the streams run, the detector waits for the reflash (Q24). Also fixed: every `!!!` box in the HTML docs showed its text as code (four-space bodies with `tab_length` 2). Seen once: `esp qemu-check` in `tools/test.sh` failed at load 18 (QEMU starved: the firmware's clock ran 0.82 s per real second, timeouts of 2.3 s); it passed alone a minute later and in the next full run. A flake under load, not the firmware: QEMU has no `-icount` |
| 53 | **The software on 64-bit ARM Linux, the car's computers, before the car day** (PLAN T9.2) | ✅ 25 Oct, `wt/ci-arm`: two CI jobs on GitHub's free ARM runner (`ubuntu-24.04-arm`, Ubuntu 24.04, 4 cores), on every push to `main` and every pull request. **`arm`** (11-12 min): `pixi run test` passes (1056 tests); the HUD and the map window (`aav_hud`) are built as aarch64 programs, and each draws its demo once on a virtual screen (125 KB and 106 KB pictures; the artifact `arm-screens`). **`car-kit`** (6-7 min): `pixi run car-kit`, then `car_install.sh --all` with no network at all (a network namespace of its own): every package built offline from the kit, the HUD too, in 4.5 min. Fixed on the way, CI only: under `sudo`, the runner's `PSModulePath` came back and colcon chose PowerShell. Still untested: the Orin's and the Pis' own Ubuntu 22.04 and JetPack |

**Ready for the car** (the steps are written; each still needs the car):

|---|---|---|

## 23 October: what we can do without the car

Ben, 23 Oct: "yes lets use cyclone and lets add this to a list of stuff we did and the
justifications for it", and the list below. Words: `docs/history/messages-2026-10-23.md`.
The reasons for each choice: [decisions](docs/decisions.md#decided-on-22-and-23-october).

| # | Task | Status |
|---|---|---|
| 28 | Cyclone DDS the default on every computer | ✅ `pixi.toml`, `tools/ros.sh`, `pixi run doctor`, the docs. To check at the car: it uses one network card |
| 28b | Cyclone's limit of 32 ROS programs per computer (found 23 Oct on Linux: Gazebo's 33rd, the sensor watchdog, failed) | ✅ `tools/cyclonedds.xml` raises it to 120. oirp-spss: 45 test programs, 13 refused before, 0 after; `sim-check --obstacle`, `--spine`, `--plan-around` pass, `--faults` 24 of 25 (`odom freeze`, the known flake: fixed, task 40); colcon's 4 failures there are the HUD it can't build (no libclang) |
| 29 | The GPS antenna over the rear axle in `vehicle.yaml` (x = 0) | ✅ its height still to measure (S1) |
| 30 | The wheel sensors: an Arduino sends CAN frames, the Orin decodes them (PLAN D2) | ✅ decided by the hardware lead (passed on by Ben, 23 Oct); the frames are `aav.dbc` 0x200, 0x201 |
| 31 | Fix the nightly Gazebo test (failed on its first night, 23 Oct) | ✅ merged: software graphics on GitHub, and the scripted drive keeps the simulator's time (task 23). First GitHub run: the night of 23 Oct |
| 32 | The 10-minute timing test (task 11), Fast DDS against Cyclone | ✅ run 23 Oct, 21:00, a quiet Mac. **Cyclone: clean** (worst IMU gap 0.06-0.08 s, nothing over 0.5 s late, 0 stalls, 0 watchdog stops). **Fast DDS:** IMU gaps up to 1 s, camera pictures up to 4.5 s late, 490 moments parked with 3 or more key topics silent at once (the longest 2.4 s), the watchdog stopped Nav2 5 times. The decision holds. Two tool bugs, ✅ fixed 23 Oct: stalls chained overlaps into one (a "236 s stall"): now only topics silent at the same moment count; and the drive, forward only, ended facing a wall after the U-turn and re-sent goals thousands of times: now it reverses, as the car may, and stops after 4 failures in a row. The driving half isn't rerun (Ben agreed: not needed for the decision). Reports: `log/timing-test/` |
| 33 | The HUD heartbeat to the safety watchdog (PLAN T9.8) | ✅ merged: `/hud/heartbeat` at 10 Hz while the screen draws; silent 1 s, Nav2 and the AI stop (not latched); on the car by default (`watch_hud:=false` without a HUD). Questions for the team: latch it? the Flutter app and map window? (the Flutter app: archived 25 Oct) Found: `sim-check --faults` `odom freeze` flaky (fixed: task 40) |
| 34 | The sensor mux and output mux (PLAN T1.5, D3, S-4) | ✅ merged: `aav_io_mux`; backends read only `/control/<target>/actuator_cmd`, filled only while every source is REAL ([sources and targets](docs/sources-and-targets.md)). Questions for the team: "check, not copy" (D3); a bench with a real spine and pretend sensors; the HUD showing the gate; `spine_bridge` refusing to arm while it's shut |
| 35 | The CAN bridge (`aav_spine`) in Rust (PLAN T1.8) | ✅ merged: `aav_spine_bridge`, the default (`bridge:=python` for the old one); reads the output mux's topic, and the guard test runs against both. Untested: the Orin with real SocketCAN |
| 36 | A bandwidth budget: every sensor's rate and size, each link's load, the CAN bus load (PLAN T1.7) | ✅ merged: [bandwidth](docs/bandwidth.md), `pixi run measure bandwidth`, `pixi run python3 tools/can_load.py`. Raw cameras are the only risk; CAN 8.5% |
| 37 | **Compress the cameras** (Ben, 23 Oct: "Lets compress it"; [bandwidth](docs/bandwidth.md)) | ✅ **software done, merged 24 Oct, as 37b**; at the car: car-day session 5. (MediaMTX H.264 on the Pi, decoded on the Orin; [camera pipeline](docs/camera-pipeline.md)); on the car: untested. Before: Today: the camera Pis (Pi 5) run **MediaMTX**, RTSP on 8554: cam1 the Pi camera, cam2 `ffmpeg` from the USB camera (640x480, 30 fps); not ROS (audit 21 Oct). Plan: keep H.264 on the Pis (the Pi 5 has no hardware encoder: x264, low-latency settings), keep MediaMTX as the server; on the Orin, GStreamer with NVIDIA's hardware decoder (`nvv4l2decoder`) turns RTSP into ROS images on the Orin only, so raw pictures never cross the network; the HUD and laptops watch MediaMTX's stream (WebRTC/RTSP) directly. In Gazebo: `image_transport` compressed. Measure latency and size with `pixi run measure bandwidth`. **Ben, 23 Oct, evening:** both front cameras are **ribbon, on one Pi 5** (Ben, 23 Oct: "we have 2 ribbin cameras hanging off one pi"; the Pi 5 has two camera ports; one `camera_ros` per camera; the same clock stamps both, good for the stereo pair; check the audit's CAM2-PI with its USB camera) (CSI: libcamera, `camera_ros`; no MJPEG from the camera, so the Pi 5 encodes in software: measure its CPU); the Orin is **probably an AGX** (hardware encode and decode; to confirm); a **second, lower-end Orin** is coming (PLAN T1.9). **Suggested pipeline (23 Oct, to write up as `docs/camera-pipeline.md`, Proposed):** for the computers, ROS compressed JPEG from `camera_ros` on each Pi (capture time stamped, bag-recordable, the same in Gazebo) → on the Orin, one process per camera: decode (nvJPEG hardware if the CPU is short) → undistort (OpenCV) → YOLO in TensorRT → small detection messages. **Revised again (Ben: ROS cameras, or just stream both to the Orin with MediaMTX?): lean to the stream.** The camera Pi runs Debian 13 with no ROS, and MediaMTX works there today: it owns both cameras (H.264); on the Orin, one node per camera pulls the RTSP stream, decodes it in hardware (GStreamer `nvv4l2decoder`), stamps it, and feeds YOLO in the same process; people watch the same stream in a browser (WebRTC). **Three cameras** (Ben, 23 Oct: "2 cameras on one and one camerea on qnx"): the front pair on one Pi, one camera on the QNX Pi (probably the audit's CAM2-PI, .118, USB 640x480, Debian 13 now: check; likely the rear camera). Ben, later: "There is 2 ribbin cables on the pi that is linux. the qnx doesnt' work": the front pair is ribbon, on the Linux Pi; the QNX Pi and its camera don't work today, so the car has no rear camera for now (reversing already counts only the lidar as seeing behind). Ribbon or USB is not certain (Ben: "it could be on a usb cable. who knows"): at the car, `rpicam-hello --list-cameras` (ribbon) and `v4l2-ctl --list-devices` then `v4l2-ctl -d /dev/videoN --list-formats-ext` (USB: if it offers MJPG or H264, the camera compresses and the Pi does nothing). Measure first: the Pi's CPU for two H.264 streams, the delay end to end, and the time stamp's error. Earlier idea: only one program can open a ribbon camera, so one owner per Pi: `camera_ros`, everything in ROS. People get a small preview topic (lower size and rate, or H.264 through `ffmpeg_image_transport`, which Foxglove/Lichtblick show) instead of MediaMTX; keep MediaMTX only if someone needs a browser view without ROS. GStreamer only where the hardware needs it; Isaac ROS (NITROS, zero copy) as the upgrade. Not OpenCV's DNN module for detection |
| 43 | **The control board's bridge refuses to arm while the safety gate is shut** (Ben, 24 Oct: yes) | ✅ built 24 Oct, `wt/arm-gate`. `/spine/arm` refuses unless `/safety/output_mux/state` said `OPEN` for the bridge's target (car, bench, gazebo) within `gate_timeout` (1.5 s), with the reason; both bridges (`spine-bridge` `gate.rs`, `aav_spine/output_gate.py`). The bench and Gazebo's `arm:=true` now start `arm_when_open`, which asks each second until OPEN. `legacy_uart_bridge` has no arm step. Also fixed: `output_mux`'s clear logged at two severities from one line, which rclpy refuses ([Arming waits for OPEN](docs/sources-and-targets.md#arming-waits-for-open)) |
| 43b | The guard test's SIM case let 13 DRIVE_CMD frames through once on main (load 29): cross-talk or a start-up race? | ✅ 24 Oct, `wt/arm-gate`: **cross-talk**. Our `sim-check --spine` (domain 193) was latched that day by another run's REAL `spine_bridge`; the SIM case passed 16 of 16 at load 24-42 and the whole guard file 44 of 44 at load 10-48, with 24 busy processes; the announcement and the graph agree within 1 ms in one process. Fixed: the guard test, `sim_check.py` and `bench_check.py` take a domain nobody uses (`aav_io_mux/domains.py`), and the test fails naming any other run's nodes. The race's small window (across processes the graph up to 30 ms before the message) closed too: a node on `/system/source` not heard yet holds the gate, and an announcement counts before its node shows in the graph. New tests: Gazebo starting late (frames stop within 1 s), and an announcer seen before its announcement |
| 39 | GitHub's Linux tests failed twice on 23 Oct (`6f801dfc`, `4bed1125`), then passed on the next push with the same code: flaky, **in the build**, not a test (no failing test case; the log's tail only "Starting >>>") | ✅ not seen again: 0 failed pushes to main since `4bed1125` (40 runs to 25 Oct). `ci_annotate.py` now names the failed package and its error lines on the run's page. Next failure: read it there, then fix the cause (a guess: a download during the build, such as crates.io for the Rust packages) |
| 42 | The Mac nightly hung in `pixi run test` on 24 Oct (started 11:50 UTC, cancelled at the job's 90 min limit, nothing to read; 23 Oct's took 12 min). Linux CI and the Gazebo job passed on the same commit (`1ff7f587`); on this Mac all tests finish in 41 s, with local and with subnet discovery | ✅ 25 Oct, `wt/ci-arm`. **Cyclone DDS at a test's end, on the network**: aav_routes' tests all passed, then freeing the last ROS node (`dds_delete`) waited for ever to join Cyclone's multicast reader, which stops only on a packet the computer sends itself; on GitHub's Mac that packet never comes. Found by a probe (native stacks with lldb, Python's with faulthandler, as annotations; commit `82d9eda4`); every GitHub Mac run hung so since Cyclone became the default (23 Oct): the 24 Oct nightly, then 3 of 3 on this branch. pytest's `faulthandler_timeout` never fired: the tests had ended. Fixed: `pixi run test` and `tools/test.sh` keep every test on this computer (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`: unicast on the loopback, no multicast reader), and the street router test takes a domain of its own. That showed cross-talk (two packages' tests side by side on one domain heard each other), so colcon tests one package at a time: 1 min 55 s, not 1 min 41. Then the heartbeat chain test wanted 30 of 50 frames in a second and that slow VM wrote 23: it wants 10 now. The Mac job then passed: 18 min, its tests 15.5 |
| 40 | `sim-check --faults`' `odom freeze` failed most runs since 22 Oct (Mac and oirp-spss) | ✅ 24 Oct, `wt/odom-freeze`. The cause: **MPPI** (Nav2's default since 22 Oct) plans from the wheels' speed, so frozen wheels keep the car only 0.1-0.2 m/s faster, under the GPS part's 0.3 m/s (with `controller:=rpp`: 4 of 4 caught). The watchdog's GPS part now fits 10 s, within 0.15 m/s (normal driving's worst: 0.04-0.08); the check freezes the wheels at 0.2 m/s and counts them wrong by the watchdog's limits. `--faults`: 25 of 25, three runs out of three ([frozen wheels and MPPI](docs/simulation.md#frozen-wheels-and-mppi)). Still to do on the car: the GPS's real noise over 10 s |
| 41 | `aav_camera_stream`'s `test_reader_restarts_an_ended_stream` is flaky (found 24 Oct: 1 in 5 alone on the Mac, and once in `pixi run test`): `os.killpg` in `decoders.py` `stop()` gets `PermissionError`, so the reader starts only twice | ✅ fixed 24 Oct: on macOS `os.killpg` says EPERM, not ESRCH, for a group whose last process has exited but not been reaped; the decoder's stop now treats both as "gone". 10 of 10 passes |
| 37b | **The camera stream, concretely** — ✅ **decided by Ben, 23 Oct**: MediaMTX on the Pi, compressed, to the Orin; OpenCV and detection there ([decisions](docs/decisions.md#decided-on-22-and-23-october)). OpenCV: which version JetPack 7.2 ships, and whether with CUDA and GStreamer, is **not checked** (JetPack 6 shipped 4.x without CUDA). **Ben, 23 Oct: "if htat is opencv 4 then thats what we use"**: JetPack's own OpenCV, whatever version; no own build. OpenCV only does picture work, so CUDA in it matters little. **The detector: RF-DETR Nano** (Ben, 23 Oct: "I thought we were skipping yolo and using the other one"; [newer models](docs/model-comparison.md): the stop sign from 20 m, YOLO11n 10 m; Apache-2.0, not AGPL), in TensorRT, not OpenCV's DNN module. ✅ RF-DETR Nano is the default since 24 Oct (`aav_detector/detectors.py`, YOLO with `detector:=yolo`; Mac: Core ML 10 ms, `sim-check --perception` stopped 4.4 m before the sign). Left: build the TensorRT engine on the Orin (`tools/build_engine.py`) and time it (Ben: "we just need to figure out how to stream and encode it over the network and then what to do with it on the other side") | ✅ **software done, merged 24 Oct; GStreamer on Macs 25 Oct (27 ms)**; at the car: car-day session 5. Was: built 24 Oct, measured on the Mac; untested at the car. ([camera pipeline](docs/camera-pipeline.md)). `aav_camera_stream`: the Pi's `pi/mediamtx.yml` (MediaMTX itself loads it in a test), one `camera_stream` node per camera (decoder as a child program: GStreamer `nvv4l2decoder` on the Orin, ffmpeg or GStreamer on a Mac; OpenCV undistortion; `aav_detector`'s RF-DETR Nano in the same process; detections, a 4 kB preview a second, `/diagnostics`, `/system/source`). `pixi run -e ml measure stream` (Gazebo → OpenH264 → local MediaMTX → node): **26 ms** camera-to-detections with GStreamer, 160 ms with ffmpeg (its decoder holds 2 frames); 2.0 Mbit/s for a noisy picture; stop sign seen from 25 m in Gazebo. To do at the car: install the Pi file, the Pi's CPU for two streams, the delay and stamp error (a stopwatch), `delay` from it; calibrate the ribbon cameras; a `car.launch.py` part for the Orin. The plan as it was: **Pi:** MediaMTX owns each camera: ribbon via its `rpiCamera` source (software H.264 on a Pi 5), USB via `ffmpeg` (pass MJPG/H.264 through if the camera has it); 1280x720, 15 fps, about 2 Mbit/s, a key frame every second, no B-frames, low-latency settings. Config in the repo. **Network:** RTSP, port 8554, wired. **Orin:** one node per camera: GStreamer `rtspsrc ! rtph264depay ! h264parse ! nvv4l2decoder` (NVIDIA hardware; `avdec_h264` or `vtdec` on a Mac), into OpenCV (undistort) and YOLO in TensorRT **in the same process**; publishes detections, plus a small preview now and then. Stamp = arrival minus a measured delay (chrony on the Pi too). NVIDIA's GStreamer plugins are JetPack's own, not Pixi's: the node uses the system's GStreamer on the Orin. **Test without the car:** Gazebo's camera → `ffmpeg` → a local MediaMTX → the node → detections, on a Mac; measure the delay |
| 37b.1 | **RF-DETR Nano is the detector, not YOLO** (Ben, 23 Oct: "I thought we were skipping yolo and using the other one") | ✅ merged 24 Oct. `aav_detector` runs RF-DETR Nano by default (since 20 Oct), YOLO with `detector:=yolo`. New: the models as plain Python objects, no ROS (`aav_detector/detectors.py`: `load_detector()`, `detect(bgr)`), for the camera-stream program; RF-DETR's **Core ML** export (`pixi run -e ml python3 tools/export_model.py --format coreml`: 10 ms a picture on Ben's Mac, 16 on Metal) and its **TensorRT engine** (`python3 tools/build_engine.py` on the Orin: `.pth` → ONNX → FP16 `.engine`, then a test run); `orin.yaml` loads `models/rf-detr-nano.engine`. `sim-check --perception` passed with RF-DETR (first seen at x = 5.9 m, the sign at 25 m; stopped with the bumper 4.4 m before it). **Untested: everything on the Orin** (needs the reflash, Q24): build the engine, time it |
| 38 | **Match the plan to the car's wiring** (Ben, 23 Oct, evening; to confirm with the hardware lead) | ✅ merged 24 Oct; the wiring is **as Ben understands it, the hardware lead confirms** ([today's wiring](docs/can-plan.md#todays-wiring-23-october)). **Actuators** (throttle, steering, brake), maybe the steering-angle and brake-position sensors, on the **ESP32**, through the **Motor Pi** (probably UART); the **CAN bus straight to the Orin**, with the wheel sensors' Arduino and a few others. Built: `spine_bridge role:=sensors` (Rust and Python): no `/can/tx`, no `/spine/arm`, no e-stop path, ignores the spine's frames; `/vehicle/odom` from the two wheels (no yaw rate) and `/vehicle/wheels`; announces only odometry and wheels. `car.launch.py actuators:=uart` (the default): `legacy_uart_bridge` on the Motor Pi drives, the Orin's `spine` part reads CAN for sensors (no CAN sender); `actuators:=can` keeps the CAN spine. `orin.yaml` and `all.yaml` turn the spine part on; the 2025-26 UDP encoder is off (two `/vehicle/odom` would stop the output mux). Tested: the guard test (both languages; no frame on `/can/tx`, the mux opens), launch tests, the Rust node fed wheel frames (1.0 and 1.2 m/s gave odom 1.1). Untested: SocketCAN on the Orin, the Arduino's frames. The Rust ESP32 firmware's 100 ms timeout is on the UART link (Gate 2). Questions for the hardware lead: [audit questions H1-H8](docs/audit-questions.md#8-the-cars-wiring-for-the-hardware-lead-23-oct) |
| 43a | **Foxglove and Lichtblick built in** (Ben, 11 Oct: "at some point"; go-ahead 24 Oct) | ✅ 24 Oct, branch `wt/foxglove` ([Watch from a laptop](docs/foxglove.md)). One bridge launch (`foxglove.launch.py`) for sim, bench, CARLA and the car: stand-ins on 127.0.0.1, the car on its network and watch only; a WiFi topic list without raw cameras, full lidar or costmap layers; the camera preview on the stand-ins too. Three layouts in `tools/foxglove/` (driving, sensors health, map); the sensor watchdog's diagnostics now carry each sensor's rate and gaps. Tested live: road sim and bench, every layout topic served, 1.4 Mbit/s for all three, a goal sent through the bridge drove the car. To do: open the layouts in the app by hand; at the car, `pixi run python3 tools/layout_check.py ws://<Orin>:8765 --data`; offline map tiles for the map panel |
| 44 | **Fix the HUD** (Ben, 24 Oct: "fix the hud?", "go"): (a) the HUD shows the safety gate; (b) a HUD that keeps crashing holds Nav2 until a person clears it (PLAN T9.5, T9.8) | ✅ merged 24 Oct (`wt/hud-gate`). (a) A **GO / STOP panel** under the speed: green GO, amber WAIT, red STOP or AUTO STOP, grey NO GATE, an icon, and the one main reason (E-STOP ON; the output mux's, such as "GPS is SIM"; the watchdog's first input that stops, such as "GPS STALE") ([HUD](docs/hud.md#may-the-car-move-the-go-stop-panel)); logic in hud-core `gate`, tested; **not yet seen on screen** (the Mac's lid was shut: no window). (b) **Latched**: one dropout within 5 s of the HUD's first beat is forgiven, any other is `hud LOST` until `pixi run clear watchdog` ([decisions](docs/decisions.md#decided-on-24-october)); new tasks `clear watchdog`, `clear gate`. `sim-check --faults` 25 of 25 (24 Oct, Mac): `hud stop` stood still 2.4 s after it, "hud LOST", still held 5 s after the beats came back, drove again once cleared; its first run found `watchdog hang` latched the HUD (Gazebo's clock catches up in steps after a stall): fixed, stalls are now seen on a steady clock |

## 24 October, afternoon (Ben's message: `docs/history/messages-2026-10-23.md`)

| # | Task | Status |
|---|---|---|
| 45 | **The ESP32 brakes on a lost heartbeat** (Ben: "if it doesn't get a heartbeat then it applys the breaks and stops the acceleration"). Ben confirms the wiring: Orin → network → Motor Pi → serial → ESP32 → throttle and the steering and brake motor controllers; CAN only the wheel encoders and the few listed; IMU, GPS, lidar on the network | ✅ **software done, merged 24 Oct, proven in QEMU**; at the car: car-day session 6. This is the Rust ESP32 firmware's 100 ms command timeout (`spine-core`, the UART build; `docs/esp32-bench.md`): no fresh command from the Motor Pi within 100 ms → full brake, throttle 0, latched until re-armed. Today's ESP32 C++ firmware has none (audit 16 Oct). **Proven in QEMU, 24 Oct** (`pixi run esp qemu-check`; [the emulator](docs/esp32-bench.md#before-the-board-the-emulator)): the real firmware, fed `aav_legacy`'s own frames, brakes 111-120 ms after the last one (brake Jrk 3500, DAC 0, fault bit 1) and stays braked until a stop frame; the Jrks and DAC are stand-ins there. It also found the firmware never turned the FPU on (fixed). Next: flash it on a spare ESP32 on the bench, the same check with the real chip (Gate 2, 27 Nov), then on the car with the hardware lead. Every link in the chain then fails safe: the Orin's watchdog, the Motor Pi's bridge, the serial cable, the ESP32 |
| 47 | The watchdog checks the camera program's health, not raw pictures (Ben: "that seems like a good idea") | ✅ already done 24 Oct (TODO 37c): the watchdog watches each stream's detections (sent for every picture, ~15 Hz, even with no boxes): quiet 1 s → Nav2 stops. Detections, not `/diagnostics`: those come once a second, averaged over 5 s, so a lost camera would be caught seconds later |
| 48 | **The RC remote and the joystick through ROS, with direct drive kept for testing** (Ben, 24 Oct: "allow for both ... eventually we will move that to ros network") | ✅ **built and merged 24 Oct, Proposed** (off until H9 and H10); at the car: car-day session 6e. Was: Proposed and built 24 Oct (branch `wt/esp32-uplink`; [the uplink](docs/rust-firmware.md#the-uplink-the-esp32s-status-back-to-the-pi-24-oct); a Proposed row in `docs/decisions.md`): with `--features uplink`, CH6 reads three positions: low computer, middle (1301-1700) **RC via ROS**, high **RC direct**. RC via ROS: the ESP32 sends the sticks up and drives only from the Pi's frames; `aav_rc`'s `rc_driver` turns `/vehicle/rc` into `/drivers/rc/cmd_vel` (twist_mux priority 95). Losing the RC in either RC position brakes (fault 256) and latches until a stop. QEMU (fake iBUS on UART2): all six RC steps pass; a ROS test: uplink sticks → `/drivers/rc/cmd_vel`. Left for the car: H10 (is CH6 a three-position switch? the receiver's failsafe), [bench steps U1-U6, R1-R8](docs/esp32-bench.md#the-uplink-and-the-rc-modes-on-the-bench). Earlier: design, then the hardware lead. Today: the RC transmitter → iBUS → the ESP32 drives directly (switch channel 5); the USB joystick → the CORTEX Pi → ROS already (`joy_node`; ours: `aav_teleop` → twist_mux). Plan: a transmitter switch with three positions: **computer** (ROS drives), **direct** (the ESP32 drives from the RC, no computer: for testing, and the fallback), **RC via ROS** (the ESP32 sends the sticks up the serial link; a ROS driver `/drivers/rc/cmd_vel` at people's priority in twist_mux, through ego's limits). Needs an uplink frame ESP32 → Pi (the status the Pi doesn't get today). Direct stays until RC-via-ROS has been proven on stands |
| 50 | **The safety code lives on the ESP32; the serial link triggers it** (Ben, 24 Oct: "thats the place to put the safety code ... some kind of communication system that triggers it") | ✅ **built and merged 24 Oct**; at the car: car-day session 6 (branch `wt/orin-heartbeat`; [the heartbeat](docs/rust-firmware.md#the-heartbeat-version-2-of-the-serial-frame-24-oct)): the output mux's counter on `/control/<target>/heartbeat`, relayed by `legacy_uart_bridge` into a version 2 frame (17 bytes, CRC-8/SAE-J1850), and the Rust ESP32 ignores any frame whose counter didn't change. QEMU: brakes 111-120 ms after the counter freezes (the Pi still sending at 50 Hz) or the CRCs go bad, latched until a stop. The 13-byte frame stays (`link-v1`; the car's `legacy.yaml` says `frame_version: 1` until the Rust is flashed). **The uplink, built 24 Oct** (branch `wt/esp32-uplink`; [the uplink](docs/rust-firmware.md#the-uplink-the-esp32s-status-back-to-the-pi-24-oct)): a 35-byte frame ESP32 → Pi on TX1 (GPIO17), 50 Hz, the same CRC-8: mode, mode switch, fault bits, the echo of the last Orin counter, the Jrks' feedback positions (unknown when a Jrk has no feedback set), the outputs, and the RC sticks; shared Rust/Python vectors. `legacy_uart_bridge` (`uplink: true`) publishes `/vehicle/status`, `/vehicle/steering`, `/vehicle/brake` and `/vehicle/rc`; with `uplink: false` (the default) nothing changes. Behind the firmware feature `uplink` until the hardware lead confirms TX1's wire (H9). QEMU: 50-51 Hz, the echo right in all 6 cycles, 0 frames lost; the check now passes on a loaded computer too. Untested on a real ESP32 and Pi: [bench steps U1-U6, R1-R8](docs/esp32-bench.md#the-uplink-and-the-rc-modes-on-the-bench). Earlier: design agreed in principle (it's the spine design: `docs/rust-firmware.md`). **Ben, 24 Oct: the heartbeat must come from the Orin and travel to the ESP32; the Pi could be off the network and still be sending.** So: the Orin's output mux (the last Orin node, which is shut whenever the safety layer says so) puts a counter in every command; the Pi only relays it, never makes one; the ESP32 brakes if the counter hasn't changed for 100 ms, however many frames the Pi sends. Today's 13-byte frame can't carry it well: a sum checksum, no counter (a stuck Pi resending the same frame would look alive), no arm or e-stop bit, nothing back up. **A version 2 of the serial frame**, as the CAN plan's DRIVE_CMD already has: a counter that must change, armed and e-stop bits, a CRC-8; and an uplink frame ESP32 → Pi (mode, faults, brake and steering positions from the Jrks, RC sticks for TODO 48). The ESP32 enforces its own limits whatever the Pi sends (speed cap, steering rate, braked at boot). Keep the 13-byte frame as a fallback until v2 is proven on the bench |

## Where we stopped (15 Oct, morning)

Everything from the night of 14-15 Oct is on `main` and pushed. GitHub's Linux CI is green
again. The three branches that waited for Ben are merged (Ben, 15 Oct: "These are
important"), on the branch `merge-three-branches`, for him to bring into `main`:

| Merged 15 Oct | From | Still open |
|---|---|---|
| CAN v0.2.0: wheel frames 0x200/0x201, 100 ms timeout; `wheel-core`; wheel speed in `/vehicle/odom`, per wheel on `/vehicle/wheels` | `can-v2-work` (`wt/can-v2` was an older copy) | ~~the wheel node boards (`boards/esp32s3-wheel`)~~ *dropped 23 Oct: the hardware lead's Arduino sends 0x200/0x201* |
| The ESP32 control board's firmware in Rust (today's UART link, CAN optional) | `wt/rust-esp32` | never flashed: `docs/esp32-bench.md` first, with Ben's go-ahead |
| One reverse speed (0.5 m/s, `vehicle.yaml`); Nav2 may reverse with `reversing:=true` (Gazebo, the bench), never on the car by default | `wt/ackermann` (its unfinished part finished) | a goal behind the car, tried in Gazebo. *Since 22 Oct the car may reverse where the lidar sees behind it (task 24)* |

**Found overnight, for Friday's audit:**
- Today's ESP32 firmware has **no command timeout, no e-stop input and no watchdog**, and an
  unprotected web update page.
- SparkFun's ESP32 MicroMod schematic shows its **CAN pads not connected**: check that the
  control board's CAN chip (U7) can reach the ESP32 before ordering the KEFC CAN parts.
- The front cameras are ribbon (CSI); our driver `usb_cam` is USB-only: `camera_ros` instead.
- A lidar is on the roof; we have a simulated one (`lidar:=true`) but no driver. *Since 20 Oct: the stock velodyne driver.*

**Also landed on 15 Oct (after the summary above):**
- the two Pico 2 boards and the radar clustering in Rust; no C++ left in `src/` (now `ros/`);
- Friday's plan, `docs/history/friday-2026-10-16.md`, and every hardware question, `docs/audit-questions.md`;
- the offline install kit for the car: `pixi run car-kit` (1.8 GB, untested on ARM);
- `max_speed:=` caps ego at launch, for the first drive on stands;
- plain-English driving with a local model (`pixi run -e llm talk`, Qwen3-4B), Gazebo only:
  `docs/plain-english.md`;
- better open-source models, including NVIDIA's per-pixel CitySemSegFormer, in
  `docs/perception-and-hud-map.md`.

**Checked by Ben on his Mac, 15 Oct:** the raylib HUD opens (`hud --demo`), and with the turtle it shows speed, throttle and steering live while driving from the keyboard. The car kit is made (1.8 GB, `car-kit/`).

**Next, in order:** Thursday evening, `pixi run car-kit`; Friday's audit and our stack on
the car on stands (`docs/history/friday-2026-10-16.md`); the A-to-B
drive (`docs/a-to-b.md`); the HUD's map panel; Python to Rust, step 1 (spine bridge, ego).

## Where we stopped (11 Oct, 12:30)

Ben, 12:27: "lets try to put everything to a reasonable state and we can audit the
hardware on friday". `main` is pushed and its tests pass. The three pieces in progress are
saved on their branches, pushed to GitHub too. Each branch's unfinished part is in its last
commit, "Work in progress".

| Branch | Done there (committed, checked) | Unfinished (its last commit) |
|---|---|---|
| `wt/sim-full` | the road world in Gazebo: lane lines, a stop sign, an obstacle, the car's camera and radar; the car's whole graph in `pixi run sim`, checked by `sim-check` | ~~the Simulation page, and a picture~~ *Done 15 Oct, with a lidar; merged into `main`* |
| `wt/ackermann` | one reverse speed for the whole car: 0.5 m/s, from `vehicle.yaml` | reversing in Nav2: behaviour trees, settings, a test |
| `wt/rust-ports` | the Rust IMU driver is the default, and the C++ one is gone | ~~the radar clustering as a Rust node (`aav_radar_clusters`)~~ *Done 15 Oct: the C++ is gone too; merged into `main`* |

**Next time, in order:**

1. **Friday 16 Oct: the hardware audit,** read-only first (`docs/work.md`, steps 1-3):
   backups, then the inventory and the measurements.
2. **Drive with W A S D,** the car or the turtle (Ben, 11 Oct). `teleop_twist_keyboard`'s
   keys are fixed (i, j, l and so on), so this is a small keyboard driver of our own, in
   the keyboard's place in the mux.
3. **Finish and merge the three branches:** each one's own checks, then `pixi run test`.
   `wt/sim-full` is merged (15 Oct).
4. **Then** the board drivers in Rust, and the raylib HUD.

## 15 Oct: all the C++ to Rust

Ben, 15 Oct: "We should port all cpp to rust", and remove the old.

- [x] **The wheel encoder and the status display Pico 2s, in Rust and Embassy**
      (`wt/rust-boards`): `boards/wheel-encoder-pico2` (same UDP packet, checked against
      the same examples as `aav_legacy`) and `boards/sentinel-pico2` (same I2C packet),
      with their logic in the crates `wheel-encoder`, `sentinel` and `hd44780`. The C++
      folders are deleted; their source stays on GitHub. *Done 15 Oct: builds, never
      flashed.*
- [x] **The radar clustering in Rust, and the IMU's C++ driver gone** (`wt/rust-ports`):
      `aav_radar_clusters` replaces `aav_object_detection` in the car, bench and Gazebo
      launch files, with the C++ node's answers for 94 clouds kept as tests. *Done 15 Oct:
      no C++ left in `src/` (now `ros/`).*

## 14 Oct: the car in photos, the KEFC CAN plan

Ben sent 32 photos and the KEFC request (CAN bus, wheel speed, steering feedback, new GPS,
a second Orin). Pages: [car photos](docs/car-photos.md), [CAN plan](docs/can-plan.md),
[topic areas](docs/naming.md).

- [x] **The raylib HUD** (Ben: "today maybe"). *Started 14 Oct, on `wt/hud-raylib`. Built: `aav_hud_raylib` (now `aav_hud`), works on a Mac.*
- [x] **Topic names by area** (`docs/naming.md`). *Done 15 Oct: Decided by Ben, renamed
      everywhere (`wt/topic-areas`), checked by `tools/tests/test_topic_areas.py`.*
- [x] **DBC v0.2.0** for the KEFC plan: the ESP32 board as the spine, wheel frames 0x200 and
      0x201, a 100 ms timeout (`docs/can-plan.md`, Proposed) [[needs-you]]. *Merged 15 Oct.*
- [x] **A lidar driver:** a lidar is on the roof; our code has none. Model at the audit.
      *A generic 16-beam lidar runs in Gazebo's road world since 15 Oct
      (`lidar:=true`), on `/sensors/lidar/points`: the real driver should publish the same.
      Nav2 reads it since 15 Oct (`aav_lidar` `lidar_scan`); then `car.launch.py lidar:=true`. Done 20 Oct: the stock velodyne driver for the VLP-16, on in `orin.yaml`; untested with the real lidar.*
- [x] **Gazebo's road world on `main`** (`wt/sim-full`): the camera, radar and a roof
      lidar, with the new topic names. *Done 15 Oct: `sim-check` passes on the Mac.*

## 11 Oct: what we can try at home

Ben, 03:00: "We can only do so much from home so lets focus on stuff that makes sence to
work on." His order, with the work under way:

1. [x] **The full graph, all set up:** every driver, ego, the simulators. In Gazebo: a road
      with lane lines, a stop sign and an obstacle, a camera and a radar on the car, and
      perception, lane keeping, collision_monitor and the HUD in `pixi run sim`. *Done
      15 Oct: merged from `wt/sim-full`, with a lidar too ([Simulation](docs/simulation.md)).*
      - [x] **The full simulation** (Ben, 15 Oct: "We want to fully simulate the car inside
            gazebo with all the sensors and build the full pipeline";
            `docs/full-simulation.md`). *Steps 1-3 built 15 Oct on `wt/full-sim-1`:
            `/system/source` and the HUD's SIMULATED banner, the per-actuator topics, the
            actuator model with guessed numbers. Steps 5 and 6 built 15 Oct: the lidar into
            Nav2, sensor failures on purpose and the sensor watchdog (`aav_safety`). Next:
            step 4, the spine in the loop. Done 15 Oct: `pixi run sim spine:=true`.*
      - [x] **The sensor watchdog's to-dos** (`docs/simulation.md`, "Still to do"): a lock
            that expires if the watchdog dies, frozen wheel odometry, the EKF's own guard
            against a GPS jump, the verdict on the map window, the same faults on the car.
            *Done 15 Oct, but for the car: a fail-safe lock, the wheels cross-checked with
            the IMU and the GPS, ekf_global's GPS gate (holds while the car is stopped),
            the map window's banner. Left: the cross check's limits and the faults on the
            car.*
2. [x] **Our C++ to Rust:** Ben's Rust IMU driver becomes the default and the C++ one goes;
      the radar clustering becomes a Rust ROS node. Python to Rust later (Ben). *Done
      15 Oct (Ben: "We should port all cpp to rust"): `aav_imu_yahboom` and
      `aav_radar_clusters`; no C++ left in `src/` (now `ros/`). Only the ESP32's firmware is C++ (and the hardware lead's Arduino).*
3. [ ] **The Mac on the network, with ML on it:** the laptop's detector (Core ML) on the
      car's compressed pictures. *On in `laptop.yaml` since 11 Oct; to try against the
      bench as the car.*
4. [ ] **Nav2:**
      - [x] the obstacle stop: a box as wide as the lane, and a re-arm after a latch, in
            `bench-check --obstacle`. *Passes on `main`, 11 Oct;*
5. [x] **The board drivers in Rust, ready to try on a board:** the wheel encoder and display
      Picos (Embassy); the spine's Pico 2 program is written. *Done 15 Oct
      (`wt/rust-boards`): all three build; none flashed.*
6. [x] **The HUD in raylib** (Ben, 11 Oct: yes): one Rust program, raylib and rclrs, that is
      also the ROS node. The driving screen first (speed, mode, the e-stop button, the
      camera); the Flutter app stays until it matches. *Started 15 Oct (branch
      `wt/hud-raylib`): the driving screen, the e-stop, GPS and topic health run on a Mac
      (`pixi run hud --demo`, `pixi run hud`). Next: the bench, a Pi, the map page.*
      *25 Oct: it matched, so the Flutter app and its back end were archived
      (`archive/2026-27-flutter-hud/`, Q29); this package took the name `aav_hud`.*

## Overnight (10-10 → 10-11): all the code, then Ackermann, Rust, native style

Ben, 21:45: "Lets try to get a fully functionin gthing going on." The order of work is
Decided ([Decisions](docs/decisions.md)):

1. **The high-level pieces**, by five helpers in parallel, each on its own branch:
   - [x] **The HUD app:** an e-stop button, Cancel, the camera page, the car's status, the
         OpenStreetMap credit. Flutter 3.35.7 in `vendor/` (`pixi run get-flutter`,
         `hud-test`). *Fixed: the app could never connect (Socket.IO tried polling
         first); now WebSocket only. The back end has 98 tests.* *Archived 25 Oct with the
         app (`archive/2026-27-flutter-hud/`): `get-flutter` and `hud-test` are gone.*
   - [x] **The HUD's place database:** `pixi run get places` (51 MB, checksum pinned).
   - [x] **The car's launch:** perception as parts of `car.launch.py` (camera, detector,
         stop_signs, radar_clusters), the GPS driver (`gps_driver:=ublox` by default),
         `gps:=false` fixed. *21 launch tests.*
   - [x] **The lane finder:** `aav_lane_finder`, classic OpenCV, 0.78 ms a picture.
         *`bench-check --lane`: from 0.3 m off the centre line back to within 5 cm in
         about 6 s.*
   - [x] **The spine's board program:** `boards/pico2` (Embassy), the `mcp2515` CAN driver,
         a 100 ms hardware watchdog, the e-stop's spare contact, and `pixi run pico spine build`,
         `pico spine tools`, `pico spine flash`. *Builds; never flashed.*
   - [x] **Obstacles in the costmap:** the radar in Nav2's local costmap, collision_monitor,
         the camera's detections as points on the road, compressed images,
         `bench-check --obstacle`. *Merged 11 Oct.*
   - [x] **Fix the obstacle stop:** after the stop-sign drive, Nav2's recovery cleared its
         costmap and the car drove on past the box; alone, the car stopped but the spine
         latched and it never went on. *Fixed in the check, 11 Oct: the car weaves (N7), so
         the box now spans the lane; a latch from a busy computer is armed again.*
   - [x] **The `lane` part** in `car.launch.py`, off on every machine: it drives whenever
         it sees a lane.
   - [x] **The laptop's detector** (Core ML), once it reads compressed images. *On, 11 Oct.*
   - [x] **`bench-check --perception` again on a quiet Mac:** it failed 5 of 5 on a busy one.
         *Passed on `main`, 11 Oct: the car locked 4 m before the sign and went on.*
2. [ ] **Ackermann done properly:** reversing (Reeds-Shepp in Nav2), both front wheels' real
      angles in the model and RViz, and the steering motor's real curve from the
      `Ackermann.m` linkage (Q2). *Reversing done 22 Oct (task 24); the rest still open.*
3. [x] **All our C++ to Rust:** the IMU (Ben's twin becomes the default, then the C++ goes),
      the radar clustering as a Rust ROS node, the wheel encoder and display Picos (Embassy).
      *The two Picos, the IMU and the radar clustering: done 15 Oct. The Picos build, never
      flashed.*
4. [ ] **Every file in its language's own style:** `cargo fmt --check` and
      `cargo clippy -D warnings` in `pixi run test`.
5. [ ] **The whole car working, end to end.**

### Needs Ben or the team

- [x] Make the repo public? Checked 11 Oct: no secrets in any of its 167 commits. *Waits on Ben's
      yes, and on what to tidy first (the 11 October email). Public since 15 Oct (Ben); now `AAV-2025-2026/ecolo-av`.*
- [x] The HUD on the car's Pi: Flutter has no Linux ARM download; the tile server and the
      OSRM route data. *No longer needed: the Flutter app was archived 25 Oct; the raylib HUD
      builds on ARM (CI's `arm` job), and street routes come from Valhalla.*

## On Ben's Mac (10-10)

Ben's messages, word for word: `docs/history/messages-2026-10-10.md`. "Our version" is
`aav/aav` (`~/Code/aav/aav` on the Mac; retired 25 Oct: now `~/Code/ecolo-av`); this work was done in a clone at `~/aav` while that
folder downloaded, and moves into it as a fast-forward.

- [x] **Build and test on the Mac.** *`pixi run test` green after the `coreutils` fix. The
      turtle, Gazebo headless, a Nav2 goal, the Rust listener, the docs, the doctor and
      recording all run. `record.sh` now goes through `tools/ros.sh` (SIP again).*
- [x] **Email everything we talk about, in detail, with sections** (a standing request).
      *Sent at 12:16, 12:32, 12:39, and with the docs PDF.*
- [x] **Docs for a new team member:** first day, starting points, packages, work,
      decisions, infrastructure, build a driver, the costmap exercise, the older code.
      *The driver page and the costmap exercise were run as written, in a scratch copy.*
- [x] **`.gitignore` in `aav/aav`:** restored at 12:35. *The app's per-message snapshots
      were taking in the Linux `.pixi` files as scp copied them.*
- [x] **A `.gitignore` for every kind of code.** *Applied in 593db7f.*
- [x] **A `vendor/` folder** for downloaded source (Ben, 17:03): `vendor/ros2-src.repos`
      (121 repos) and `vendor/ros2-rust.repos` (9), at exact commits, in git; the 559 MB
      out of it. `pixi run get vendor` downloads; `tools/test.sh` checks `COLCON_IGNORE`.
- [x] **Move today's commits from `~/aav` into `aav/aav`** once the download finishes.
      *Done 16:13: the scp was still copying the server's Linux build output after six hours,
      so it was stopped and rsync brought the rest. The copy matches the server, Ben's
      uncommitted work is intact, and 55 commits fast-forwarded in. The Linux `.pixi`,
      `install` and Rust `target` went to `~/Code/aav/aav-linux-build-output`, not deleted.
      Built (25 packages, Ben's `aav_imu_yahboom` too) and tested green on the Mac.*
- [x] **Gazebo's window on the Mac:** Ben saw the screenshots (16:34).
- [x] **The firmware requirements in `vendor/`, built out of the box** (Ben, 20:54).
      *A `firmware` environment with PlatformIO; its downloads go in `vendor/platformio/`.
      `pixi run -e firmware legacy-esp32` built today's ESP32 firmware from a clean
      checkout on the Mac (48 s). The tools to flash an ARM chip (`probe-rs`, a UF2
      converter) aren't on conda-forge: they come with the first board program.*
      *Done 11 Oct: `tools/elf2uf2.py`, and probe-rs in `vendor/cargo-tools`.*
- [x] **CARLA: 4 or 5, and do we need it?** (20:54) *Parked this year: no Jazzy
      support, no Mac version. Gazebo and bench mode cover the car graph (Q31).*
      *Reopened 15 Oct (Ben: "I think we want carla 5 not 4"): research, `carla_adapter`
      through CARLA's Python client, and Carleton's OpenStreetMap download done
      (docs/carla.md). Left: run it on the NVIDIA computer.*

## The afternoon of 10-10: "merge in all of this", then "stub in the rest"

- [x] **Perception in:** `aav_yolo`, now `aav_detector` (YOLO, detections, the stop-sign watcher and its mux
      lock), `aav_object_detection` (radar clustering), `aav_lane_geometry` (lane keeping
      on `/drivers/ai/cmd_vel`), the camera calibrations in `cameras.launch.py`, the `ml` environment.
- [x] **The graph down to the sensors:** bench mode (`pixi run bench`): spine_sim runs
      spine-core on the CAN frames and drives a model car; pretend IMU, GPS, camera,
      battery and radar. `pixi run bench-check`: 7 of 7. `gps_nmea.launch.py`,
      `perception.launch.py`, udev placeholders.
- [x] **Rust drivers for the spine's devices:** jrk-g2, ad5311, pca9555, ibus, spine-board
      (98 tests); all seven firmware crates build no_std for the Pico 2 in `pixi run test`.
- [x] **The HUD's back end on our topics:** `aav_hud`, 86 tests. *(The Flask one: archived 25 Oct; `aav_hud` is now the raylib HUD.)*
- [x] **Docs:** Bench mode and Perception pages; every page brought up to date.
- [x] **Ben: commit the server work** (`aav_imu_yahboom` and its three edited files). Then
      perception and the bench become parts in `car.launch.py`'s machine files.
      *Done: the server's files are byte for byte in `dfaec15`; perception is parts
      since 11 Oct.*
- [x] **The board program for the spine's chip** (implements `spine-board`'s `Board`): waits
      on the chip (Q10-Q13). *Done 11 Oct for a Pico 2, to try on the bench.*
- [x] **The HUD app's small changes** (docs/hud.md). *Done 11 Oct. Archived 25 Oct.*
- [x] **The first-day steps in `README.md`** (Ben, 17:03).
- [x] **`~/Code/aav/CLAUDE.md`:** every old folder, and what replaced it in `aav` (Ben, 17:03). *That workspace was retired 25 Oct.*
- [x] **An MCP server** (Ben, 17:03): read-only, over stdio. `tools/mcp_server.py`, `.mcp.json`
      and `docs/mcp.md`: 9 tools. Its smoke test reads a live topic.
- [x] **On GitHub** (Ben): `git@github.com:zander1976/aav.git`, private (public since 15 Oct; since 25 Oct `AAV-2025-2026/ecolo-av`). CI: Linux on every push, a
      Mac every night. Commit messages give the intention, with no attribution lines; the
      README's Credits section covers who did what.
- [x] **Ben's Rust IMU driver** (`aav_imu_yahboom`, then `imu_driver:=rust`): reviewed and
      committed; its tests now run in `pixi run test`. *The only IMU driver since 15 Oct.*
- [x] **Cleaned up** (Ben approved): `~/aav`, the Linux build output and the old bundle.

## The first night (10-09 → 10-10)

Ben's messages, word for word: `docs/history/messages-2026-10-09.md`.

- [x] **Build on this machine** with Pixi + RoboStack Jazzy (no admin, no Homebrew).
      `colcon build`, `colcon test`, then run `turtle.launch.py` headless and drive it.
      If something can't build, write down why.
      *Done: all 19 packages build in 11 s; our tests pass; the turtle drives through
      mux → ego; check_graph: 0 failures. The inherited packages' lint failures were
      fixed on 10-10.*
- [x] **Mac tomorrow.** The same `pixi.toml` for macOS (Apple Silicon). Check that every
      package resolves for `osx-arm64`. Write the steps: no Homebrew.
      *Done 10-10 on Ben's Mac (Apple Silicon, macOS 27): build 18 s, `pixi run test` all
      green after one fix (GNU `coreutils`: SIP drops `DYLD_LIBRARY_PATH`). See below.*
- [x] **Port everything to Jazzy as is.** Apply the fixes the audits found (configs,
      inherited code, missing dependencies). Reports: `docs/history/audit-2026-10-10-*.md`.
      - ~~Spine safety, before any firmware is written~~ *Done 10-10: B1 arming always
        sends the 0→1 edge; B2 the spine boots latched; W1 `cmd_timeout` 0.1; W2 driver
        faults stop and latch; W3 `max_decel` 2.5. Each has a test (Rust 47, bridge 4).*
      - ~~Legacy path~~ *Done 10-10: `to_legacy` evens out the steering gain; `encoder_udp`
        goes quiet after 0.3 s without packets; it runs on the Motor Pi, where the
        Pico sends.*
      - ~~pixi.toml deps; inherited packages~~ *Done 10-10: flask, flask-socketio, scipy,
        topic_tools added for all 3 platforms. Radar, battery and IMU code restyled to
        ROS's lint rules and fixed (no `sudo chmod`, udev port names, clean shutdown);
        maincar's NameErrors and start-up fixed; CMake 3.10; `.warning(`. `pixi run test`
        is all green (colcon test: 77, 0 failures).*
      - ~~Live check of sim, car and replay~~ *Done 10-10: every config key is declared
        by its Jazzy node. Fixed: Nav2 on sim time, the sim GPS noise (degrees, not
        metres), a map on the car, steering speed and inflation in the sim, a replay
        profile. Nav2 goals now succeed in Gazebo. Left for the team: see below.*
- [x] **The graph.** Every topic has the right publisher, subscriber, type and QoS.
      Check it live with `tools/check_graph.py` on the turtle run.
      *Done 10-10: turtle, sim and replay have 0 failures. The car, with no hardware,
      fails only on `/sensors/imu/data_raw` and `/sensors/gps/fix`. No QoS mismatches.*
- [x] **Docs as HTML (and PDF)**, short pages, sections, red and white, a picture of how
      it fits together. Markdown stays as the source.
      *Done: `docs/html/index.html` and `pixi run docs` (PDF). Four pictures; pages for
      each point below.*
      - What ROS 2 is, and how we extend it (stock modules vs our nodes)
      - How Python, C++ and Rust fit into Jazzy, and what we give up with Rust
      - How the dual EKF works ("local injects, global reads local + GPS")
      - Run it: turtle, keyboard, Gazebo
      - src vs firmware; one interface, several drivers (the IMU example)
- [x] **Email progress** to cmail, red and white, every few hours and when something lands.
      *Sent at 22:49, 03:50, 04:07 (the morning summary, with the PDF) and 04:17 (the car
      must run Jazzy before a Mac can join it).*
- [x] **Naming and code style** for all code. Inherited code is ours to edit now.
      *Done: `docs/conventions.md`, `pixi run lint` passes. Renaming the inherited
      packages is proposed and waits for the team.*
- [x] **Rust:** pull `rclrs` and the message crates through cargo. Build one example in
      the Pixi environment. Write down what a Rust rewrite would take (ESP32, Pi).
      *Done: `examples/rust-listener` (rclrs 0.8 + ros-env 0.3 from crates.io; the
      message code ships with Jazzy) builds in 18 s and printed ego's commands live.
      `pixi run rust-listener`. The rewrite trade-offs are in docs/languages.md. Removed
      26 Oct: four real rclrs nodes show the same.*
- [x] **Design doc:** fix the ten "last year's", and update the tree, profiles and decisions.
      *Done 10-10 (doc rev 58): all ten now name 2024-25; the tree, the four profiles,
      the mux priorities, the tests, Gazebo before CARLA; Jazzy, one repo and Languages
      marked Decided; new rows: Macs (Decided), bare metal waits (Tentative), spine
      arming (Proposed).*
- [x] **CLAUDE.md:** record tonight's decisions, so no session starts from scratch.
      *Done: docs for a dyslexic reader, Macs first-class, `pixi run test` green, test
      isolation, the pytest skip trap, Rust nodes later, where the TODO and audits live.*

## Decided on the first night

- Teleop (keyboard, joystick, remote) is a **driver**: it feeds the mux, then ego, then
  the turtle or the car.
- **Bare metal: wait.** Only the spine firmware is bare metal.
- **Port to Jazzy as is first**, then consider rewriting in Rust.
- **Simulator:** start with a Gazebo test world.
- **Docs:** short, sectioned, HTML or PDF. Ben is dyslexic, so use tables and pictures
  over walls of text.

## Finished rows moved 15 Nov (day)

From `TODO.md`'s newer sections, as they were. The 15 Nov tidy-up (Ben: "close stuff that doesn't make sense any more").

### The night of 14-15 November

| # | Task | Status |
|---|---|---|
| 179 | **Wheel noise and slip in our simulator** (Ben, 14 Nov): seeded, off by default | ✅ 15 Nov: `wheel_noise:=` (m/s on each reading while the wheels turn) and `wheel_slip:=` (a fraction too far), `noise_seed:=`; the odometry integrates the readings and drifts; standing still it reads nothing; tested in `sim-core` (slip 5 % drifts 3-8 % of the way; the same seed, the same drift). Left: a drive with them on, to see the EKF and NDT hold |
| 180 | **The sensor watchdog warns about our own nodes** (Ben, 14 Nov: warn only, no lock) | ✅ 15 Nov: a `node` kind (the node's full name, looked for in the ROS graph each tick; may not stop the car); `node_*` inputs for costmap, planner, navigator, mppi_lag, smoother, behaviour, voxels, things, ego, look_ahead; the sim watches the ones it starts |
| 187 | **The point driver's corridor fence** (from the old `wip/safety-car-path`, 7 Nov, never finished): a drive to B stops if the car is more than 3 m off the line A -> B, more than 2 m past B, or further from the start than the drive could ever need; independent of Nav2 and the steering (a wrong steering sign, a bad pose). Its code (`line.Fence`) is kept in `archive/wip-safety-car-path`; port it to today's point driver, with tests | ✅ 15 Nov: ported (`line.Fence`, checked each tick until the car stands at B; `max_off_line` 3 m, `max_overrun` 2 m); `test_line.py` tests it. Not yet seen in the sim |
| 188 | **The lidar's memory for a low box** (the whole-car design's cheap thing: `observation_persistence` on the lidar layer): a 0.5 m box the roof lidar loses within 2.5 m of the bumper, the radar off as on the car. `lidar_memory:=` (s; 0, the newest scan only, stays the default until it wins); `sim-check --low-box` runs 0 and 3 s | ✅ 15 Nov, our simulator, radar off: the car goes round the 0.5 m box both ways; 1.67 m clear with no memory, **2.74 m** with `lidar_memory:=3.0` (the box stays marked past the blind zone). Default stays 0.0 until a car day |
| 190 | **The plan's staircase** (the sway's third source, 15 Nov): 16 lattice headings turn a road at an angle into steps; the plan alone swings 0.13 m at 2.4 m/s. Try our stack on the 32-heading file (`measure controllers --stacks own own32`), then the path smoother. **15 Nov 05:45, run** (lot + campus, reversing, 2 runs, 14 legs each): both reach 14/14; 32 headings: sway 0.104 vs 0.110 m/s², wiggle 0.044 vs 0.047 m, off the path 0.140 vs 0.133 m. Within the runs' spread: no win, 16 stays (`lattice_headings:=32` kept as a challenger). The staircase isn't the main sway source | ✅ 15 Nov afternoon: the staircase is there with 32 headings too; the smoother's weights left it (they take out only wiggles under ~2 m). `plan_smoothing:=strong` (w_data 0.02) takes it out: the plan's own swing 0.08-0.16 -> 0.02-0.07 m (`lattice-planner` example `staircase`); in the `ours` stack now (TODO 168) |
| 192 | **Camera and lidar together** (Ben, 15 Nov: "How the camera perception informs the lidar distance stuff"; "object deteciton and display like a stop sign"): project the lidar cubes and things into each camera picture (the extrinsics from vehicle.yaml: exact in the simulator, Q22 on the car), so a thing inside a box takes its name (person, car, stop sign) and the stop sign its lidar distance, not one from its pixel height; the dev window and HUD draw the names. Built and scored in our simulator first. **Step 1 done (15 Nov):** `naming.py` (5 tests) and things `names:=true` / `things_names:=true`; **Step 2 done:** with `things:=true things_names:=true` the stop_sign_watcher takes a named stop sign's distance from the lidar (`lidar_sign`, within 3 m of the pixel-height guess). Neither run in a sim yet. **Step 3 done (15 Nov): the screens draw the names.** The RViz/Foxglove markers say `person 3 1.2 m/s` and colour each box by kind (aav_things `looks.py`: person yellow, vehicle blue, stop sign red, other green, unknown orange); the developer window draws each thing in the 3D scene and on the map with its name (key K); the HUD's AHEAD draws each box with its name and lists named things past its 6 m drawing (STOP SIGN 12 m), placed with `/localization/odom/local` (`hud_core::things`). Tested: unit tests, and the HUD node heard a hand-published Things and odom; no screenshot yet (in that session raylib could not open a window and the dev window's screenshot came out black: look on a screen), and names in a sim still need the ml environment's detector **15 Nov, on a screen** (Ben's Mac, pretend data): the HUD's AHEAD and the dev window draw each thing in its kind's colour with its name; two overlaps fixed (the HUD's far list on a near name; the dev window's names piled on its header). **15 Nov, in the sim with the detector** (`pixi run -e ml sim-check --thing-names`): first run, the sign stayed unknown: the lidar's cubes stop at 2 m, so its thing is the pole, thin and short of the camera's plate box (IoU far under 0.3). Fixed in `naming.py`: a thing reaching the cubes' ceiling is matched on its bearing (overlap left to right ≥ half the narrower box, and some top to bottom); a thing seen whole keeps the strict IoU. Second run: PASS, named stop sign from the start, 0.04 m off the sign, no wrong names. Left: names on the car need the cameras calibrated (Q22) | ✅ |
| 193 | **The car doesn't drive on after an obstacle is taken away** (found 15 Nov, `sim-check --obstacle`, stack:=ours, plan_around:=false): it stops 0.9 m short as it should, but fails 3 runs of 4 (map_correction slow and newest alike). Two ways: (a) all locks clear, behaviour DRIVE with no cap, and the car stands 220 s with nothing logged; (b) the stop zone fires again after the clear, mppi_lag says `FollowPath failed: no progress in time` every 10 s, and the navigator aborts the goal after ~2.5 min. Look at: our navigator's retry after a stop (938d5e5f keeps a clear path 10 s), mppi_lag's progress check while twist_mux's stop_zone lock holds, and when the check takes the box away | ✅ 15 Nov: **the radar's marks never cleared.** The radar marked the box; taken away, the radar returned nothing, and only a ray to a point clears, so the box stayed in our local costmap. MPPI crept at ±0.03 m/s before a wall that wasn't there (half its futures collided) until its progress check failed (10 s); then (b): the navigator's replan in that time (the lattice swings ~1 m to the side) took the car 0.2 m from the stop sign, inside the stop zone's margin, which then held it. Fix: our costmap's radar clears its 120° view where it sees nothing (`view_angle`, `docs/own-costmap.md`). `sim-check --obstacle`: before 2 passes of 4 here, both only after the 10 s stand and a retry; after **8 of 8**, no follower failure. (a), standing with nothing logged, wasn't seen in these 12 runs. Left: the lattice's sideways swings on a replan (TODO 177's), which can still take a blind planner (plan_around:=false) close to the sign; Nav2's costmap (`stack:=stock`) keeps the old rule |
| 194 | **The behaviour misses a walker 1 time in 3** (found 15 Nov, `sim-check --moving-only`, stack:=ours, things:=true): a person crossing 20 m ahead, timed to reach the lane as the car does. 2 of 3 runs the behaviour slows (APPROACH(obstacle)) and passes 0.32-0.33 m ahead of them (the check wants 0.3); 1 of 3 it never slows and only the stop zone stops the car. Look at: how soon `things` confirms the walker (3 clouds), the behaviour's lane test for a thing coming from the side, and its predicted crossing. The check itself was fixed tonight: the walker starts by the car's speed (it used to cross long before the car came), and only a walker ahead of the moving bumper counts (the simulator's walker passes through a standing car) | ✅ 15 Nov: **two causes, recorded** (`/world/things` and the behaviour's state every 0.1 s). (1) The behaviour counted a moving thing only once it was **in** the lane: the walker came in about 1 m from the bumper (`left` -0.7 m), too late to slow; one recorded run touched them (-0.41 m). Now a thing walking towards the lane counts if at its speed it is in the lane before the car gets there, + 2 s, within 6 s (`behaviour.py` `Crossing`). (2) Our simulator's radar stand-in said **everything stands still**, so the tracker held the walker at 0.55 m/s, 20° off (truth 0.8 m/s straight across); lidar alone gets 0.8 in 1 s. Now the simulator's clouds carry what each ray hit moves (`speed`), and `gazebo_radar` adds it to the Doppler speed: 0.8 m/s in about 0.6 s. `sim-check --moving-only`: (1) alone **5 of 5**, 0.44-0.93 m ahead of the moving bumper; (1) + (2) **5 of 5**, 0.52-0.97 m (0.49-0.60 m in all; was 0.38-0.50). Slows from about 1.9 m before the stop. Gazebo's radar still says everything stands still (its walker can't walk anyway) |

### Release candidate 0.4-rc1 (Ben, 14 Nov night)

| # | Task | Status |
|---|---|---|
| 166 | **Every stack runs**: `sim-check` passes on `stack:=stock` and `stack:=ours`, on our simulator and Gazebo | ✅ 14 Nov: `sim-check --obstacle --stack` stock and ours, both simulators, every check passes (ours stops 0.91 m short of the box, stock 0.23-0.34 m) |
| 170 | **The things in the behaviour planner**: wait for what moves, go round what's parked (decides TODO 162). Built (`things:=true`); 14 Nov, our sim: a parked box driven round 2.0 m clear, no APPROACH(obstacle), stop zone quiet (`sim-check --behaviour-things-only`). 15 Nov, our simulator: a person crossing the lane (`sim-check --moving`, our simulator walks a placed model) PASS: slowed, 1.84 m clear, goal reached (but the states flap: TODO 189). Left: Gazebo, the car | ✅ 15 Nov: the behaviour slows for a walker crossing, 5 of 5 in `sim-check --moving-only` (0.5-1.0 m clear), after TODO 194's fixes |

### One shape, and lanes as soft costs (13 Nov)

| # | Task | Status |
|---|---|---|
| 149 | **The wiggle, alone** (TODO 124): truth pose; one plan, no replanning (the single-goal tree replans at 1 Hz); the command before and after the smoother; score the swing in metres. **14 Nov:** the replanning moved RPP and `mppi_lag` and made loops; the GPS's jitter halves RPP's flips; the smoother nothing; MPPI's 6-8 cm is its own. The trees keep a valid path: RPP 0.12-0.13 → 0.09 m, `mppi_lag` 0.14-0.17 → 0.10 m, no loops ([trials](docs/trials.md#the-wiggle-14-nov-todo-124-149)). Left: the slow map correction (165) | ✅ |

## The old lists, as they were on 15 Nov

Everything from 7 Nov back to 10 Oct, word for word, moved here on 15 Nov. What was still open is in `TODO.md`'s "Software, still to do", "Needs the car or the hardware lead", "The team decides" and "Ideas kept, not picked"; what was closed, and why, is in its "Closed on 15 Nov". Headings one level down.

### Saturday 7 November: 0.3, the algorithms know about lag (MPPI)

Ben, 7 Nov: "have the algorithms know about lag". MPPI out of Nav2 into our own Rust driver, its model with the actuators' lag; Nav2 keeps planning.

| # | Task | Status |
|---|---|---|
| 132 | **The lag-aware MPC** (`tracker:=mpc_lag`, [results](docs/trials.md#lag-aware-mpc-7-nov-mpc_lag)): steering and set speed planned through the spine's speed loop, throttle, brake, the steering motor's rate and linkage, dead times and the commands already sent; one QP a tick, on the shared car model (`car_model`: its numbers, speed loops and estimate; `lag_model.step` is its maths on arrays, held to it within 1e-9). Says `/drivers/nav/lag_aware` (latched) so ego's lead stays off. `lag_scale:=2` (Gazebo) for the robustness test | 🟡 built and scored. Pretend car: no speed overshoot; lags x2, the 2 m/s U-turn 0.17 m off (MPC 0.36). Our simulator, lot tour, 3 runs: **6 of 12 legs** (MPPI 8, RPP 9, `mppi_lag` 11): the quickest and closest on the legs it drives, 5 ms a tick, but 0 of 3 on the U-turn by the building (collision_monitor stops it on the planner's path; no aav_track tracker reads the costmap: mpc and pure pursuit fail it too). Gazebo, 1 run: lags as told **4 of 4** (MPPI 2, RPP 2); doubled 2 of 4 (MPPI 3, RPP 3). Left: keep off obstacles (costmap cost in the QP, or slow near them, as RPP); move to Rust before the car (worst tick near 50 ms on the Orin, estimated); model the LQI and the ESP32's open loop |

### Saturday 7 November: the hardware debug plan

Ben, 7 Nov: the Rust replaces the C++, WiFi pages too, with JSON instead of streaming text; graphs of steering, brake and throttle in `hw_panel`; debug one part of the car at a time.

| # | Task | Status |
|---|---|---|
| 126 | **The hardware debug plan** ([brief](docs/designs/2026-11-07_esp32-debug-link_brief.md), "The plan now"; [second review](docs/designs/2026-11-07_esp32-debug-link_review-2.md)). Steps: today's log; USB cable and Pololu's tools; two small fixes and paper sheets; the Pi ready; Rust with no radio; record the joystick path, PASS or FAIL; return wire and graphs in `hw_panel`; the steering page; timing test and WiFi pages. Replaces the 6 Nov [bench link](docs/designs/2026-11-06_bench-link.md) | ⬜ proposed, order chosen by Ben. Nothing built. 🔒 the hardware people: a USB cable, the current limits, the return wire, where the steering and brake sensors connect |

### Wednesday 4 November: the weak links, for Thursday's pick

Ben, 4 Nov: "think about the areas of this car and what is the weakest ... ideas on how to improve each section of the driving experience", then "save this full list". The list: [the weak links](docs/designs/2026-11-04_weak-links.md) (every area, the evidence, the ideas, the papers). Words: `docs/history/messages-2026-11-04.md`.

| # | Task | Status |
|---|---|---|
| 100 | **The clocks, walk the fence, the site file** (keep-in, keep-out, speed zones, named places, missions; stock Nav2 filters) | ⬜ **picked** 4 Nov (Ben: "The clock yes"); the fence and the site file go with the first lot visit |
| 101 | **The brake follows a wish** (control plan step 1: feed-forward + PI for throttle and brake) | ⬜ **picked** 4 Nov. Needs a driven-wheel speed (TODO 49b), then `pi` and test 4.3 **on the ground**: never `pi` on stands without that speed. 5 Nov: the geofence holds programs in every mode until that mode's stop is measured (`pi_loop_decel`) |
| 102 | **A behaviour state machine + a speed profile on the path** ([design](docs/designs/2026-11-05_behaviour-and-speed.md)) (DRIVE → APPROACH → STOPPED → WAIT → GO; PERSON_AHEAD; "stop at X" for the sign, the fence and the goal; jerk-limited speed along the path) | ⬜ **picked** 4 Nov; corrected 5 Nov: a speed cap on Nav2, not one profile for everything; a small node (proposed); no radar targets. The stopping law moves to [how the car stops](docs/designs/2026-11-05_how-the-car-stops.md), which comes first |
| 103 | **The tracker** ([design](docs/designs/2026-11-05_tracks-and-memory.md)): an ID, a speed and a class for each radar cluster and cube group; a person means stop and wait | ⏸ **paused** (Ben, 5 Nov): after the stopping design, and once the radar is on the car. What must be true to resume is in the design; the radar's mode is not a switch |
| 104 | **Research A** ([design](docs/designs/2026-11-05_knows-itself.md)): the car identifies itself (delay, lag, braking, with error bars) and a CBF safety filter in ego, fed by the site file's fence | ⬜ **picked** 4 Nov. Staged: the measured numbers (TODO 52b), the closed loop, the monitor, then the filter beside the locks. Where it goes: proposed before twist_mux, not in ego (K1, Ben's) |
| 107 | **One developer window** ([design](docs/designs/2026-11-05_one-window.md)) (Ben, 5 Nov: "a single screen just for the developers and a list of algo so you can swap"): `demo` opens Gazebo headless and **one** Rust window: 3D, the Nav2 map you click on, NDT and ICP, the algorithm list with SWAP. RViz off unless `--rviz`. The street map and the HUD stay in the car. Not Gazebo's renderer compiled into Rust | ⬜ 0.3; the design corrected after the EGM review. Proposed: raylib, with Gazebo's real picture as a panel; Bevy only on a written test (D1, Ben's). Replaces TODO 73's merge into the cockpit |
| 108 | **Shadow mode** (research thread 3, step 1): while a person drives, Nav2 and the other automatic drivers keep publishing under a SHADOW lock (twist_mux 60), the bag records what each would have done, `shadow_score.py` scores the disagreement | ⬜ **not picked**: proposed, in stages ([design](docs/designs/2026-11-05_shadow-mode.md)): recording and a score on real bags first; a live lock only if that shows a need (S1, Ben's) |
| 111 | **The radar driver can die on a short frame**, in today's default mode too: `unpack` returns `None`, the generator raises, and the publisher reuses the dead generator (`TI.py:117, 125`; `pc_publisher.py:110, 143`). Read, not run | ⬜ |
| 112 | **`vehicle.yaml` has three duplicate keys** under `vehicle:` (`centre_of_mass`, `tyre_grip`, `encoder_counts_per_rev`). The values match today, so harmless until someone edits only one: delete the copies | ⬜ |
| 113 | **The stop-sign watcher lets rear-camera boxes trip the lock** (`stop_sign_watcher.py:34`). Reviewer's word, not checked | ⬜ |
| 114 | **The developer window in Bevy** ([design](docs/designs/2026-11-05_own-simulator.md)) (Ben, 5 Nov): his layout, in `ros/hud/aav_dev_window` | 🔧 live on Gazebo, with goals through its gates (5 Nov). Next: a slower laptop, cancel a goal, the algorithm list, SWAP |
| 115 | **Our own simulator in Rust and Bevy** ([design](docs/designs/2026-11-05_own-simulator.md)) (Ben, 5 Nov: "we want to replace gazebo"): every sensor Gazebo gives, beside Gazebo until it wins on the same routes | ▶ 7 Nov: the core is built (row 134); the cameras on their own branch |
| 116 | **Does the car stop if Ben's MacBook drops off?** The LLM on the Mac sends Nav2 goals; a goal may carry on with the Mac gone. Check, then a heartbeat if so Screen heartbeats now exist for tests (`watch_screens:=`, `docs/simulation.md`), refused on the real car; what the car does about a lost Mac is still to decide | ⬜ before the LLM drives the real car |
| 119 | **The foot-tall kerbs may be invisible to Nav2** (Ben, 5 Nov: the lots have kerbs about a foot tall; `nav2.yaml:366` sets the cubes' floor at 0.3 m, and `docs/learn-lidar.md:141` ignores kerbs). Read, not tested. Check the flat scan's floor, then test a 0.3 m kerb in the simulator. The map may have to hold them | ⬜ before autonomy near a kerb |
| 120 | **The campus world and the one map** (Ben, 5 Nov): OpenStreetMap plus his measurements gives surfaces, kerbs and buildings, anchored by lat/lon to `navsat.yaml`'s datum, each cell saying where it came from. Generates Gazebo's world and NDT's practice map; Bevy draws it with pavement, concrete and grass. Real drives replace it. No RTK (Ben, 5 Nov): the rough map guides the stitching of a real drive, and NDT against the finished map replaces GPS's offset. Tape two or three long distances as the check. A drive only suggests map changes and a person accepts them (Ben, 5 Nov): the window shows them on the map to approve, and that review doubles as the winter record | 🟡 world, one costmap and NDT's guess built and driven in Gazebo (`docs/campus.md`); drive fitting, the GPS offset and approving changes not built; which lot and his measurements: Ben |
| 121 | **NDT's score floor on the campus** (found 5 Nov): 1.5 there, 2.3 in the test lot. Measure how high a wrong pose scores before `ndt_ekf` on the campus | ⬜ |
| 122 | **Reversing on long drives** (found 5 Nov): with `reversing:=true`, Nav2 backed the car a long way, at 0.5 m/s, to a goal behind it. Should reversing cost more, or stay short? | ⬜ decision |
| 124 | **Why the car wiggles** (Ben, 5 Nov; N7): the turn rate flips about once a second on MPPI and RPP. Not the steering lag model (the same without it). NDT in place of GPS's jitter: no clear change. **14 Nov (149):** a new plan every second (fixed: the trees keep a valid path) and the GPS's jitter (165); MPPI's own 6-8 cm swing stays | ✅ |
| 125 | **Read their Humble bags** (after the drive as is, 6 Nov): rename `/velodyne_points`, `/imu/data_raw`, `/aav/gps/fix` to ours, so `ndt-map build` and replay can use them ([tonight](docs/car-evening.md)) | ⬜ after the drive |

### Tuesday 3 November: 0.2 polished for the meeting (4 Nov), then the car

Ben, 3 Nov: "Tomorrow is meeting day and we want to have 0.2 fully polished and documented ready to go." The plan: [the brief](docs/designs/2026-11-03_release-0.2-brief.md), "Recommendations for today". Words: `docs/history/messages-2026-11-03.md`.

| # | Task | Status |
|---|---|---|
| 93 | **The cubes let the car into a post once** (`sim-check --lidar-obstacle --voxels`: 1 of 3 runs, 1.30 m in; 30 Oct it stopped 0.44 m before). Suspect: the lidar's new place (1 Nov, x 0.90 → 0.49 m, z 1.80 → 1.52 m) | ⬜ before `voxels:=true` goes anywhere near the car |
| 94 | **Drive-to-point stops 0.61 m past B** by the car's own EKF, once a run (limit 0.5 m; by Gazebo's truth within 0.46 m). GPS noise or the GPS's new place? | ⬜ |
| 96 | **A model builds our own algorithms** (Ben, 3 Nov: "let a fable running on max build our own algorothms with everythign we have done and the data we collect"). What it needs from us: the recordings (session 10: lidar, cameras, wheels, IMU, GPS, every command), a ground truth (RTK GPS, the tape), and the trials' scoring (`measure controllers`, `measure atob`, the localizer scores), so each algorithm it writes is a trial beside the default, scored on the same routes and the same bags | ⬜ after car day: the data first |
| 95 | **NDT-EKF the default of the test-lot demos** (`demo`, `pick`), which already need its map | ⬜ 0.3 |
| 90 | **One screen per concept** (Ben, 3 Nov: "the gazeboo with nav2 selection with cubes and stuff all as a single screen and then hardware as a seperate screen. Just the stuff we control"): **Drive** (Gazebo or the camera, Nav2 and its choices, the cubes, localization, the algorithms) and **Hardware** (only what we control: the actuators, the sensors, calibration). The debug screen's parts move into Drive; the HUD and the map stay in the car | 0.3: today's step is 87 |
| 97 | **The 0.2 release notes, as an advertisement** (Ben, 3 Nov: "a release note like an advertisement of what is done, what is going to be done. How to use the stuff we have"): `release-notes.md`'s clean look; a short feature list; the names spelled out (NDT, MPPI, ICP...); each screen's picture, its command, what it means and how to use it (the hardware test in front of the car); the cockpit explained; what's left | 🔧 3 Nov |
| 98 | **The faint car behind every screen** (Ben: "there was an outline of the car in the background before. I liked that"): today only the HUD draws it; the map, the debug screen and the hardware panel too, and new screenshots | 🔧 3 Nov, `faint-car`: drawn on all four in Initial D, one painter (`hud_app::paint::faint_car`), the contrast test extended (385 cargo tests). ⬜ the four screenshots: the Mac's screen was locked |
| 91 | **PR #21**: close #20, open #21 from the fork's `main`, after the full tests | ⬜ last |
| 92 | **Ben, tonight**: the people named; the hardware lead's OK for A8 and H1; the restore test's spare SD card | ⬜ Ben |

### Sunday 1 November, night: toward 0.2

Ben, 1 Nov: "Lets maybe aim at this being 0.2 version ... I think we are trying to get to 0.2 so maybe we merge in all the code. Do some testing and then extend the whats new document to include everyhting?"

**0.2 = sim-complete, car-ready. 0.3 = it drove.** After the review ([the road to 0.2](docs/designs/2026-11-01_road-to-0.2.md)), Ben, 1 Nov: split car day; a **small** 0.2; keep every idea for 0.3 and later.

#### Done in 0.2

| # | What |
|---|---|
| 59, 62 | Reversing and the U-turn: the lattice is the default where the car may reverse; Hybrid-A* forward only |
| 63 | The drive-test plan: maps, recording with the lidar, scoring the trials (tests.md 4e, 4f) |
| 67, 70 | The steering calibration below the screen (link v3: limp, jog; raw counts; `SteerCal`), and the hardware panel's page with the lock-to-lock check that stops on resistance. Untested on the car |
| 69 | Fewer pixi commands: 89 → 45, no aliases |
| 72 | The car tests, written: [Know the car](docs/know-the-car.md), one page of order and stop rules |
| 73 | The Lichtblick cockpit, built (screenshots wait for an unlocked Mac) |
| 75 | `tools/car_setup.sh <role>` and [Car setup](docs/car-setup.md) (one computer); STOPs for a pending reboot, a wrong hostname, an upgrade too early. Untested on a car computer |
| | The screens in Catppuccin: HUD, map, debug screen, hardware panel |
| | [Car day](docs/car-day.md) split: **Day A** destroys nothing, **Day B** upgrades; the people, the vault, sign-offs, the brake on stands, the joystick-off test |

#### Left for 0.2 (five rows)

| # | Task | Status |
|---|---|---|
| 68 | **The car-ready checklist**, every box ticked ([car-day.md](docs/car-day.md#ready-for-car-day-the-02-checklist)): `pixi run test`, `car-day-check` 0 missing, `esp qemu-check`, the `car_setup.sh` dry runs, the merges | 🔧 1 Nov: 1513 tests, 0 failures, before the review's fixes |
| 74 | **The first moves' gaps** (the joystick, on the Motor Pi alone, `machine:=motor_pi_alone`; [A9b](docs/car-day.md#a9b-the-joystick-on-stands)). (a) **build a joystick-alive check**: the joystick's messages stop changing, or the pad is gone → stop (`joy_node` repeats the last state while the receiver is in); (b) a **throttle cap** in the firmware for the first moves (the ESP32 can't see speed: today only DAC 700 of 1023), Ben and the hardware lead decide the number; (c) **the Rust firmware only** for the first moves (the C++ has no timeout); (d) the joystick's button and axis numbers. **The Kar-Tech stop is the main stop**, held by the e-stop person | 🔧 hold-to-drive, the stale-command brake, the 100 ms heartbeat brake (Rust) and `max_speed` exist. ⬜ (a) to build; (b), (d) on car day |
| | **Questions to the hardware lead, before the day**: H1 (is CORTEX the Motor Pi?), C1 (CAM1 at .108 or .102?), `.211`'s job, who owns the router (fixed addresses) | ⬜ ([session 3](docs/car-day.md#session-3-questions-for-the-hardware-lead)) |
| | **The people named**: hardware lead, e-stop person, safety driver, laptop person, approver, with phones; and the shared password vault (Q34) | ⬜ ([the people](docs/car-day.md#the-people)) |
| 77 | **The 0.2 release notes**: a "Release 0.2" section in [release-notes.md](docs/release-notes.md) (the stub is there); `whats-new.md` stays as history | ⬜ after 68 |

#### 0.3 and later: every idea kept

Ben, 1 Nov: "I do like the 23 ideas? lets keep them around like the diffusion path finding. I want that in."

| # | Idea | When |
|---|---|---|
| | **Car day itself**: Day A, then the car tests and the map (0.3 = it drove) | after 0.2 |
| | Day B: **the Pis to Ubuntu 24.04** (the Motor Pi after the Rust ESP32; LOCALIZATION after its machine file) | after a restore is proven |
| | Day B: **the Orin's reflash** to JetPack 7.2 (once 7.2 is confirmed for AGX Orin), the lab desktop after it | after the Orin's full image |
| 75 | A machine file for the localization Pi (where are the IMU and GPS?) | before its upgrade |
| 78 | **The diffusion planner**: in 0.2 as a trial (TODO 83); for 0.3, more plans from the model (63 % → 90 %), the U-turn, the Orin (TensorRT) | 0.3: TODO 133 (8 Nov) |
| 78 | **The talking model**: a vision-language model that explains, never drives | design first |
| | **The Initial D HUD**: the arcade gauge in Carleton red and black | ✅ in 0.2, on every screen (TODO 80, 81) |
| | **The little robot** (a tribute to WALL-E): the map's car, eyes peeking on the panel and the debug screen | ✅ 2 Nov; ⬜ the HUD's eyes, after the Initial D HUD |
| 73 | The cockpit's **algorithm switch**; then **merge the cockpit and the debug HUD** into one (the hardware panel stays apart: it drives) | 0.3 |
| 71 | **The hardware panel's pages**: actuators off, measure, direct control, type in measurements, joystick, speed and distance; the car tests from the panel (72); the calibration page's open bits (70: wheel angles typed in; a pixi task for the panel on the car's network) | 0.3 |
| 76 | **Python speed** on the Orin: measure (`timing-test`), the cubes to Rust first if they miss their budget | car day, then 0.3 |
| 79 | **Brake and throttle setting**: measure, then set, like the steering | after the steering |
| | "Two drive paths": the bridge refuses when a second mux or heartbeat appears | 0.3 |
| | Way B on the car: link version 3 with the uplink (H9), the panel's calibration page | after the Rust with the uplink |

### Sunday 1 November: the ideas before the second PR

Ben, 1 Nov: "do all the ideas. I like them all but car day. Keep that in a list fo things todo".

| # | Task | Status |
|---|---|---|
| 60 | **Car day: measure the wheel diameter by rolling** (Betsi's 0.36 m is PROVISIONAL; loaded it read 0.42 m, a 17 % speed error) | ⬜ car day: roll the car 10 m by tape, count the encoder ticks, `vehicle.yaml` `wheel_diameter` |
| 61 | **Car day: the track, front and rear** (0.65 m and 0.85 m, today one average, 0.75 m) | ⬜ car day: check both; decide whether `vehicle.yaml` needs a front and a rear track |
| 64 | **Set the datum to the test lot** (`navsat.yaml`, a placeholder near Carleton) | ⬜ once the lot is chosen: before the first recording |
| 65 | **The 2024-25 / 2025-26 drive recording** (Humble): find it; if it has the lidar, NDT's map of that lot | ⬜ Ben: who has it |
| 67 | **The steering pulls right: find the steering actuator's ends and centre** (calibration.rs `TODO(Q1)`, `TODO(Q2)`) | 🔧 1 Nov, `steer-cal`: below the screen done: serial frame version 3 (limp, jog), the uplink's raw counts, the firmware's rules (`SteerCal`), `/drivers/buttons/steering_calibration`, `/vehicle/steering_actuator/raw`, `steering_limits`, [the procedure and sheet](docs/actuator-tests.md#find-the-steering-actuators-ends-and-centre). ⬜ the panel's page (another helper). ⬜ car day: way A with Pololu's utility (test 3.0); way B once the Rust firmware and the uplink wire are in. Untested on the car |

### Thursday 29 October: control (Ben, 27 Oct)

Ben: "investigate LQR and maybe collapse all these various controllers into a single
controller ... closed loop". The plan: `docs/designs/2026-10-27_control-plan.md`; the ideas: "Ideas to keep" below.

| # | Task | Needs |
|---|---|---|
| 0 | **The plan's step 1: the bridge uses ego's acceleration**, feed-forward + PI for throttle and brake (`docs/designs/2026-10-27_control-plan.md`); scored with `measure atob` | Gazebo, bench |
| 0b | **The spine's state machine (Arsh's #15), follow-up:** the board for today's car (`esp32-umod`) has no wheel speed (`SpeedFeedback::None`), so the test mode's "leave above 0.5 m/s" can't fire there. Refusing throttle tests without a wheel speed: done (`spine.rs` turns them into Hold; found by the two-screens review, 29 Oct). (Test requests have no transport yet, so nothing can trigger it today.) Also: the self-test is one tick with no fault | Arsh |
| 2 | **LQR for steering:** from `vehicle_model.lateral()`, scheduled by speed; follows the planner's path (Smac), in place of pure pursuit; scored with `measure atob` and `measure controllers` | 🔧 29 Oct, branch `trackers-tuning`: rebuilt about the path's bend (gains by speed and bend), the command's rate in the design, a 2 s preview of the bends ahead. Fake car's full-lock U-turn: 0.80 → 0.29 m off. Gazebo: 10 of 12 legs, U-turn 2 of 3 (04:18, was 8 and 1 of 3); on a busy Mac (15:23) 6 of 12, one run starved, one **weaved**. Left: rerun on a quiet Mac with the status topic logged ([trials](docs/trials.md#results-29-october-afternoon-lqr-rebuilt-and-mpc_full)) |
| 4 | **MPC for the whole A-to-B:** speed and steering together, with the brake's and steering's limits; scored against 2 and 3, and MPPI | 🔧 29 Oct, branch `trackers-tuning`: `tracker:=mpc_full`, one QP a tick over the next 9 m (aav_track `qp.py`, OSQP's method in numpy), within full lock, steering rate, throttle, brake, and the policy's limits. Gazebo (busy Mac): 10 of 12 legs (MPC 12, MPPI 10), the same times as MPC, 2.6 ms a tick. At 1 m/s it hardly slows for the U-turn (its model sees no need); at 2 m/s on the fake car it slows to swing the steering. Not scored against 3 (LQI) yet |
| 5 | Unfinished from 26 Oct: the QEMU and NaN fixes (`small-fixes`), the bench fixes (`bench-fixes`). The F6 geofence: ✅ 29 Oct, [The geofence](docs/geofence.md) (branch `geofence`). Left for the team: the test lot's polygon once its datum is surveyed (W3); test 4.3's stop with the legacy speed loop `open` (`vehicle.yaml` `open_loop_decel`), until then programs stay held (COASTING); a `sim-check --geofence` in Gazebo | goal circling ✅ 29 Oct, branch `goal-circling-fix` (TODO 17); the rest open |

Every gain still comes from guessed numbers until the car is measured (`docs/measure-the-car.md`).

### 24 October, night: the car as a linear model

Ben, 24 Oct: "Do we have a mathematical model for the car now for the ego? ... a matrix
control thing ... is that something we should consider? ... Maybe second order system".

| # | Task | Status |
|---|---|---|
| 52b | **Refit the model from the actuator tests at the car** | ⬜ car day: the tests print the numbers; `pixi run python3 tools/vehicle_model.py --set throttle_lag=... --set drag=...`; then the speed loop's gains by pole placement (`place_speed_gains`), not before |

### 24 October, evening: the car-day kit

Ben, 24 Oct: "Lets get everything prepared so the only thing left it to get into the car."

| # | Task | Status |
|---|---|---|

**Ready for the car** (the steps are written; each still needs the car):

| TODO | What | Runbook session |
|---|---|---|
| 14 | weigh and measure the car (Gate 1, 30 Oct) | 2 |
| 28 | Cyclone's one network card: the laptop's and the Orin's file | 0, 4c |
| 37b.1, 37c | RF-DETR's engine (`trtexec` from the ONNX), the Pi file, which camera, CPU, delay, calibration pictures | 4d, 5 |
| 38, 46, 49, 50 | the hardware lead's questions: H1-H8, the Jrk wires, TX1, CORTEX, the camera Pi's address | 3 |
| 43a | Foxglove at the car (their bridge holds 8765: ours on 8766) | 8 |
| 45 | the ESP32: backup, the Rust flash, checks 1-14 and 9a-9c, roll back | 6 |
| 30, 38 | CAN read-only: the Arduino's 0x200, 0x201 | 7 |
| T2.7 | bags for the sim-versus-real report | 10 |

### 23 October: what we can do without the car

Ben, 23 Oct: "yes lets use cyclone and lets add this to a list of stuff we did and the
justifications for it", and the list below. Words: `docs/history/messages-2026-10-23.md`.
The reasons for each choice: [decisions](docs/decisions.md#decided-on-22-and-23-october).

| # | Task | Status |
|---|---|---|
| 37c | The camera stream's follow-ups (merged 24 Oct, `aav_camera_stream`, [camera pipeline](docs/camera-pipeline.md); `pixi run -e ml measure stream`, VideoToolbox by default: 27 ms camera to detections, 15 fps, 0.13 Mbit/s for Gazebo's plain scenery) | 🔧 (1) ✅ 24 Oct, branch `wt/camera-mac`: a Mac decodes with GStreamer by default (`decoder:=auto`: `vtdec`, VideoToolbox, else `avdec_h264`, else ffmpeg with a warning), from a new small `gst` environment (`pixi install -e gst`; no re-solve: `pixi lock` only added it and the `calib` one, every other environment's packages unchanged). stream-check: 4 ms decoded, 27 ms to detections (ffmpeg: 160). (2) ✅ 24 Oct, branch `wt/camera-car`: the sensor watchdog watches each stream through its detections (kind `detections`, by `frame_id`: `front_left_stream`, `front_right_stream`; quiet 1 s or NaN stops Nav2), not `/diagnostics` (1 Hz, its rate over 5 s: slower). `measure stream` checks it: OK for 60 s, STALE 1.1 s after the stream stopped. The lane finder gets `image_raw` from the stream (`publish_image`) when the lane part is on. (3) ✅ 24 Oct, same branch: `car.launch.py`'s `camera_stream` part, on in `orin.yaml` (CAM1-PI `192.168.1.108`, `gst-nvidia`, RF-DETR's engine); it replaces the detector part for streamed cameras, feeds the stop-sign watcher and camera_obstacles, and is skipped (with a warning) until the engine is built. Tried with a stand-in machine file on the Mac (two local streams); untested on the Orin. (4) ✅ 24 Oct, `wt/camera-mac`: the stop-sign watcher stops at a distance (4.7 m, the old 55 px with Gazebo's lens) from each camera's focal length (its `CameraInfo`), so it carries over to the 1280x720 ribbon cameras once calibrated. (5) ✅ 24 Oct, same branch: the calibration is ready for the car: `pixi run calibrate-camera` (the board to print, stock `camera_calibration` on the stream from a `calib` environment, the result filed in `config/calibrations/`; [how](docs/camera-calibration.md)); tried with a made-up board stream, the buttons untested. At the car: install the Pi file, the Pi's CPU for two streams, the real delay, calibrate the cameras (`pixi run calibrate-camera`), check the stop at 4.7 m |

### 24 October, afternoon (Ben's message: `docs/history/messages-2026-10-23.md`)

| # | Task | Status |
|---|---|---|
| 46 | The steering-angle and brake-position sensors are probably on the ESP32 (Ben) | ⬜ hardware lead, question H2 (`docs/audit-questions.md`): which pins, which sensors. **Hint from the old code (24 Oct):** steering and brake each run a Pololu Jrk G2 (I2C 0x05 and 0x04), which closes its own position loop from a feedback input; the 2025-26 ESP32 never reads it back, but the 2024-25 `FinalRev` firmware read the steering position from the Jrk over I2C. So the angle and brake position are likely one I2C read each from the Jrks; then the Rust firmware reads them and sends them up the serial link (Gate 3, closed loop) |
| 49b | **On stands, nothing measures the driven wheels** (found 29 Oct, EGM review of the preflight): the wheel sensors are on the **front** wheels (the Arduino, CAN 0x200/0x201), the motor drives the **rear**. On stands `/vehicle/odom` reads ~0 while the rear wheels spin, so no speed cap or stop that reads the wheels can fire there, and the bridge's new speed loop (`speed_loop.mode pi`) would wind the throttle up. Today's stands tests cap the **command** and the car stays on `mode: open`, so they hold; nothing closed-loop may run on stands until this is answered | ⬜ hardware lead: is there a speed sensor on the rear wheels or the motor (the motor controller's tach output, a Hall sensor)? If not, add one before closed-loop stands tests |
| 49 | **Is I2C good enough for the Jrks?** (Ben, 24 Oct) | ⬜ hardware lead: how long are the wires from the ESP32 to the two Jrks, and do they run beside the motor cables? The load is tiny (50 updates a second, a few bytes each), so speed is fine; the risk is noise on long wires (I2C has no checksum and can lock up). The Rust firmware already drives both Jrks over I2C, flags a Jrk that stops answering (fault bits 8, 16) and brakes. If the wires are long or noisy: the Jrk G2 also takes TTL serial with a CRC (Pololu's protocol), a small change in the firmware |

### Working list, from 22 October (Claude works from this; Ben checks it in the morning)

Everything Ben asked for since 21 Oct, in one place. Progress is updated as each step lands,
and each step is committed. Ben's words: `docs/history/messages-2026-10-22.md`. The
proposal's full list: [docs/plan.md](docs/plan.md).

**The night in one table** (22 Oct, 06:10; all in Gazebo, all pushed):

| | Result |
|---|---|
| MPPI | the default; reaches every goal of the lot tour, U-turn included when it may reverse |
| Obstacles | the planner steers round what the radar and lidar see |
| Heading | the global EKF's heading: worst per leg 7-16° (was up to 76°) |
| ego | a jerk limit; the weight model checked at 3 m/s |
| Gazebo | steering within 3% of the model; one node per sensor; `sim-check` nightly in CI (on trial) |
| ✅ Answered | Cyclone DDS: yes (23 Oct). The GPS antenna: already over the rear axle (23 Oct). Reversing: yes (22 Oct) |

| # | Task | Status | Progress |
|---|---|---|---|
| 1 | MPPI as a choice (`controller:=mppi`), and a script that compares it with Regulated Pure Pursuit | ✅ done | `f22dc4b8`; results in `docs/path-followers.md` |
| 2 | ego knows the car's weight (181 kg) and its body | ✅ done | `7e76850f`; the centre of mass is a guess until the car is weighed (task 14) |
| 3 | The proposal as a to-do list with % done | ✅ done | `docs/plan.md`, `b70300d7` |
| 4 | The AAV colour: the email's blue | ✅ done | `da411333` |
| 5 | Email Ben as work lands | 🔧 ongoing | part 1 ~01:30; part 2 ~02:50 (every goal reached; the DDS finding); part 3 ~03:20 (MPPI default; Cyclone); part 4 ~04:06 (round the box; the U-turn; the early push); part 5 04:38 (three-point turns); part 6 05:14 (jerk limit, steering check, nightly Gazebo; the part 4 time corrected); part 7 06:08 (the EKF heading; the antenna mount; part 6 corrected); part 8 06:41 (good morning: the forward U-turn still a coin toss) |
| 6 | MPPI loops at goals | ✅ fixed (`7697778f`), confirming in 8 | it now turns to the goal's heading from 2.5 m out: left turn 28 s, 21.4 m (was 58 s, 47.5 m with a loop); round the building reached. Confirm in the full comparison (task 8) |
| 7 | The corner problem (docs/plan.md M8): the car stops at the building's corner | ✅ fixed | the planner's path passed 0.86 m from the corner. Now the planner's costmap inflates 3.0 m, gently (1.0), so paths keep to the middle of a lane; and it plans turns 1.25 x wider than full lock (`vehicle.PLAN_TURN_MARGIN`), so the controller has steering to spare. Lot run, 02:00: both kept clear of the building, both reached the first goal in 29 s, each reached 3 of 4 goals (were 1 and 2) |
| 17 | The car still sometimes circles a goal it just missed (a different goal each run) | ✅ fixed 29 Oct, branch `goal-circling-fix` | **Why:** the behaviour tree planned a new path every second, also 1-4 m from the goal. There the car runs a little behind on the last bend, and Smac's only path to the goal's heading at its 3.6 m radius is a 21-25 m loop; and each new path resets controller_server's goal checker. Not the tolerances, nor the EKF. **Fix:** `navigate_to_pose_car*.xml` keep the path within 8 m of the goal (`vehicle.final_approach`: 2 planned radii), while it ends at this goal, `IsPathValid` and under 20 s old; 1 Hz elsewhere. Stock Nav2 nodes. The map window waits 25 s there before NO NEW PLAN. **Runs** (Gazebo lot, first leg, 3 each of RPP, MPPI, pure_pursuit, lqr, `--plans`): before, loop plans in 10 of 12 runs and 3 driven (RPP 59 s, 49.3 m; MPPI 64-65 s, 49.5-50.1 m, for 21); after, no loop plan, all 12 at 0.94-0.98 of the route, 23-30 s. Whole tours after: reversing 8 of 8 goals; forward only RPP 4 of 4, MPPI's U-turn failed as before; the box tour passes ([Path followers](docs/path-followers.md#circling-a-goal-found-and-fixed-29-october-todo-17)). Not done: the through-poses trees (Nav2's waypoints go through to-pose) |
| 8 | Run the full comparison again with 6 and 7; update `docs/path-followers.md` | ✅ done | 6 runs overnight (b-f); the page has the fixes, a table of runs and a picture of the best. Pushed 03:18 (815 tests pass) |
| 9 | Make MPPI the default, if it now does as well as Pure Pursuit (Ben: "I thought we decided to go mppi") | ✅ done | in every lot run MPPI did at least as well; it reached the first goal cleanly in all 4, Pure Pursuit looped there in 2 of 3 (it aims only at a point, so it arrives facing wrong). `vehicle.DEFAULT_CONTROLLER`; `controller:=rpp` for the old one. `sim-check --obstacle` passes with it. Untested on the Orin's CPU |
| 10 | **Split `gazebo_adapter` into one node per sensor**, as on the car, so each runs on its own core (Ben, 22 Oct). Rust for any part still late | ✅ done | `gazebo_imu`, `gazebo_gps`, `gazebo_radar`, `gazebo_camera` (one process each); `gazebo_adapter` keeps the car (actuators, wheels, odometry). The IMU works out the car's acceleration from Gazebo's odometry. Each sensor node takes its own `fault.<sensor>`. `sim-check` passes; `sim-check --faults`: 14 of 16, every fault reaches its node; the 2 fails were a whole-Mac stall (task 11). Not on the car: there each sensor already has its own driver |
| 11 | A long timing test (10 min, parked and driving) to catch the rare late data, before and after 10 | ✅ run 23 Oct (task 32) | a 1 min test was clean: real time 0.99, every topic under 80 ms late. Seen since: the IMU, wheels and GPS silent 0.7-2.1 s together in MPPI's lot run (one adapter, before 10); and after 10, in `--faults`, a stall of everything at once (the global EKF missed its rate, the controller ran at 8 Hz, `map → odom` 1.4 s old). So not only the adapter: find what the whole Mac waits on. 02:13 run: IMU, wheels, both EKFs silent 0.5-0.7 s together, with the Mac 77% idle and nothing else busy: so not a CPU shortage. **Found: it's the messaging (Fast DDS), not macOS.** A bare 50 Hz publisher and subscriber beside a sim run, 25 min: the publisher's timer never slipped more than 0.03 s, a no-ROS loop 0.06 s, but the subscriber waited up to 0.48 s (9 times over 0.1 s). **Cyclone DDS: worst 0.05 s, none over 0.1 s** (25 min beside a sim run), and `AAV_RMW=cyclone pixi run sim-check --obstacle` passes everything (the rear camera at 10 Hz, 7-8 on Fast DDS). Opt-in now: `AAV_RMW=cyclone` (tools/ros.sh). ✅ **Ben, 23 Oct: "yes lets do it"**: Cyclone is the default everywhere (`pixi.toml`); `AAV_RMW=fastdds` goes back | Also seen: Spotlight (`corespotlightd`) up to 75% of a core now and then |
| 18 | Stuck facing a wall: after a loop or a wide turn the collision monitor stops the car facing a wall, and forward only it can't get out | ✅ way out on | not a false stop: at the building the car had turned toward it before passing it, and a box that bends with the steering (`VelocityPolygon`) would have bent further in. So not built. The way out is reversing (Nav2's back-up), off on the car until the team decides. With reversing, both controllers now get round every goal of the lot (run n, task 19). ⚠️ **Team: may the car reverse?** It needs something that sees behind it. **22 Oct, morning: Ben said yes, the car may reverse** (task 24) |
| 20 | Goals: a miss of heading still cost a loop | ✅ done | heading tolerance 0.5 → 0.75 rad (the EKF's heading is up to 25° off; both controllers arrived 35° off by cutting the plan's last bend). Run j, 04:03: the first two goals clean for both (29-32 s, 21-22 m; 53 s, 42 m). The U-turn failed for both: its plan is a light-bulb turn using the whole 14 m lane |
| 19 | MPPI's U-turn: forward in the 14 m lane it went wide or round the building (3 of 4 runs); with reversing, the planner drew a three-point turn but MPPI never reversed: it drove on to the far wall, stopped, and the back-up recovery failed | ✅ done with reversing | three causes, with reversing on: (1) `PreferForwardCritic` kept MPPI from backing up at all (runs h, k: 0 reversals), so it is dropped when reversing: the planner's `reverse_penalty` already decides when to reverse; (2) `PathAngleCritic` mode 2 and inversion tolerances 0.5 m / 0.6 rad, so it follows the path's own direction; (3) the progress checker cancelled the turn 10 times (rocking back and forth moves < 0.5 m): now `PoseProgressChecker`, turning 0.5 rad counts too. Run m, 04:25: turn back reached, 69 s, 2 reversals. **Run n, 04:35, whole tour: all 8 goals, both controllers.** MPPI's turn back: 71 s, 2 reversals, wide; Pure Pursuit's: 74 s, tighter, but 10 reversals and 4 stops. Forward only (run o, 04:46): the U-turn still fails for both, as before the changes: so they did no harm, and only reversing fixes it. Left: MPPI drives on a few metres before it backs up (its own choice: it asks 0.5 m/s forward, easing to 0 over 13 s). Forward only, after the EKF fix: run s (06:16) all 8 goals, the U-turn as a loop; but run t (06:26) MPPI's loop met the north wall (2 of 4), Pure Pursuit looped twice at the first goal. So forward only it's still a coin toss; reversing is the reliable way. Tried 06:35, not kept: MPPI's `az_max` 1.0 → 0.4 (to match ego's steering rate): runs u, v made the U-turn once in two, the same as before. `docs/path-followers.md` |
| 16 | `sim-check`'s scripted drive came up short once (4.43 m, needs 4.5), then 5.99 m twice | 👀 watch | probably the same stalls as 11 |
| 12 | Steer round the box in the lane: sensors in the planner's map (docs/plan.md M5) | ✅ done | the planner's costmap gets the radar and lidar layers (`plan_around:=true`, the default). Both controllers drove round the box 2.6-3.8 m clear. New `sim-check --plan-around` passes (3.9 m clear, no stop); the stop tests run with `plan_around:=false` and still pass (`sim-check --obstacle --lidar-obstacle`, `bench-check --obstacle`) |
| 13 | The global EKF's heading jumps at walking pace (docs/plan.md M9, T6.3) | ✅ 25 Oct, `wt/ekf-heading`: **a heading source, from the GPS receiver's velocity.** `gps_heading` (`aav_gps_heading`, `course.py` with 20 tests): the velocity's direction (Doppler, 0.05 m/s), turned round when reversing, the antenna's swing taken out, only above 0.3 m/s with the GPS's and the wheels' speeds agreeing; into `ekf_global` as imu1, yaw only. Gazebo's `gazebo_gps` now sends `/sensors/gps/fix_velocity` too (the truth + 0.05 m/s). `pixi run tools/ros.sh --local python3 tools/heading_check.py` (3.5 min, 0.5-1 m/s, forward and reversing, empty world), off → on: **5 Hz GPS, 4 runs each**: forward median 2.2-2.8° → 1.6-1.8°, worst 12.7-27.1° → 10.4-13.6°; reversing median 2.3-2.7° → 1.7-2.1°, worst 11.4-18.0° → 8.5-13.1°. **1 Hz (the receiver's factory rate), 3 / 2 runs**: forward median 2.8-5.4° → 2.7-2.9°, reversing 3.1-3.5° → 2.2-3.1°; worst about the same (10-21°). **Gyro biased 0.6°/s, 2 runs each**: median 3.3-4.3° → 1.5-1.9°, worst 12.3-22.3° → 9.4-16.9°. **Started 30° wrong, 2 runs each**: under 5° after 7.8-10.0 s → 5.4-6.1 s (3 s of it parked). Not taken: the magnetometer (raw counts, the motors), lidar matching (not installed; no north), `use_odometry_yaw` / `yaw_offset` / `differential` (no effect with a datum). Left, the hardware lead decides: 5 fixes a second (a receiver setting), RTK (fixes 25× better), two antennas (~0.4°, standing still too). On the car: `gps_driver:=ublox` for the velocity; check with the first bag |  needs a heading source; lower noise helped a little. 22 Oct, 05:30: at 0.5 m/s its heading wanders up to ±25°, forward and back alike (probe: the wheel speed and turn rates are right in reverse, so no sign bug). Over the night's runs, reversing: median 11° off, worst 76°; forward at the same speed: median 4°. **Cause of the reversing part, 05:35:** the antenna is 0.8 m ahead of the rear axle, and `navsat_transform` removes that offset with the EKF's own heading, so a heading error feeds back. Antenna over the rear axle (Gazebo, two runs each): reversing median 4.6-6.6° (was 7.5-12.8°), worst 26-31° (was 39-54°). ✅ **it is over the rear axle already** (Ben, 23 Oct); `vehicle.yaml` now says x = 0. **And in software, 05:55:** the global EKF's yaw process noise 0.06 → 0.01 (`a588a993`): heading median 2.6° off, worst 14° (was 6.2-6.7°, 39-54°); still right within 10 s of a start 29° wrong; better with a biased gyro too. Check with the real IMU's bias. Lot tour, run r (06:04): all 8 goals, worst heading per leg 7-16° (runs n, q: 14-76°) |
| 14 | Weigh the car, front and back, then with the rear lifted (M7, M7b) | ⬜ needs the team | `docs/measure-the-car.md` |
| 24 | **The car may reverse** (Ben, 22 Oct morning: "Yes the car can reverse"), safely | ✅ done | `52d58380`: `reversing:=true` on the car, but only where the lidar sees behind it (`vehicle.blind_behind`; else forward only, with a warning). collision_monitor's stop zone turns round with the command (`velocity_polygon`): reversing, a box 1 m behind the rear bumper. `sim-check --rear-obstacle`: stopped 0.75 m short of a post, then backed on to the goal. `sim-check --obstacle` still passes (0.46 m). Seen: the Gazebo car reversed at 0.57 m/s for ego's 0.5 (its speed loop overshoots) |
| 25 | The rear camera as a sensor behind the car | ✅ wired, ⬜ untested with a real picture | the detector reads more cameras with one model (`extra_images`); camera_obstacles publishes every camera's newest points together (`aav_detector/recent.py`), so the rear camera's don't flicker; collision_monitor stops for them (source `camera`) only with `camera_guard`, which the car sets when the rear camera watches behind: elsewhere (bench, Gazebo) no people detector feeds it, and a silent source would stop Nav2. It counts as seeing behind only when the machine's detector reads `/sensors/camera/rear/image_raw` and keeps people (`car.launch.py watches_behind`; the car's detector keeps stop signs only today). Not tried in Gazebo: it has no person to detect. Needs the cameras calibrated (Q22). Also fixed: `blind_behind` had counted camera_obstacles alone, which reads only the front camera |
| 26 | Check the nightly Gazebo job (task 23) after its first run, 03:00 on 23 Oct | ✅ 25 Oct | nightly `sim`: 21, 22 and 24 Oct passed; 23 Oct failed (no GL: fixed that day); 25 Oct failed on Gazebo's own segmentation fault at start (software rendering on the runner), and every node after it. sim-check now starts once more when Gazebo itself crashed while starting (`tools/gz_crash.py`; a second crash fails). Task 23 keeps watching |
| 27 | The speed loop overshoots its target by about 15% (seen: the Gazebo car reversed at 0.57 m/s for ego's 0.5) | ✅ 25 Oct, branch `wt/speed-loop`: a profile, **not the default** (Ben: "lets do them all"; the firmware keeps today's gains until the stands, TODO 52b) | The cause is the PI's zero (−ki/kp = −0.33, slower than the poles), not windup: clamping alone changes nothing. Fix: a **setpoint weight** (the P term sees b × target), which moves the zero and leaves the poles and margins alone. spine-core `SpeedLoopConfig` gets `setpoint_weight` (1.0) and `anti_windup` (`Bounds`), both today's loop to the bit; `SpeedLoopProfile` names `default` and **`smooth`** (kp 1.0, ki 0.25, b 0.75, integral limit 1.0, clamping). Gazebo's port has the same, and both check `firmware/spine/speed_loop_vectors.txt`. Model (`pixi run python3 tools/vehicle_model.py --speed-loop`, 6 runs per cell, the lag doubled and 110 ms late included): overshoot 17 → 0 % at K, 0 → 0 % at K/5 by drag (today 29 % short of 2 m/s), 17 → 8 % at K/5 by a weak throttle (today 29 % short), 15 → 1 % at 2K; settles (5 %) 13.7 → 5.7 s; phase margin ≥ 42°. Rejected: clamping alone, feed-forward (88 % at 2K), pole placement (22 % at K/5, 31°). Gazebo, `sim-check --speed-steps-only --speed-loop`, 3 runs each: today 15.9 / 15.2 / 10.1 % at −0.5 / 1 / 2 m/s, within 5 % after 11-13 s; `smooth` 0 / 0 / 0 %, 4.2-5.3 s (and the same with the spine in the loop). `sim-check --speed-loop smooth` (road world): all 10 checks pass; `bench-check` (default) and `esp qemu-check` pass. Car day: [where to try it](docs/actuator-tests.md#the-speed-loops-profile-todo-27) (needs the spine's closed loop: today's ESP32 runs open loop) |
| 23 | `sim-check` in the nightly CI (PLAN T2.6) | 👀 watch | 25 Oct: a Gazebo crash at start, now retried once (task 26). Job `sim` in `test.yml`, Linux, `sim-check --obstacle`; first run 03:00 on 23 Oct **failed**: Gazebo crashed, as EGL had no driver (no GPU, no Mesa). Fixed 23 Oct: `tools/software_gl.sh` (Mesa's llvmpipe), and the scripted drive on the simulator's time (a slow simulator cut it short); passes on oirp-spss with 4 cores, Cyclone and Fast DDS. A failure now leaves annotations and the artifact `sim-logs`. On trial (`continue-on-error`): once it passes a few nights, drop `continue-on-error` |
| 22 | Jerk limit in ego (PLAN T5.3) | ✅ done | `max_jerk` 2.0 m/s³ (`ego.yaml`): speeding up, the push builds over 0.5 s and eases off before the set speed (0 to 1 m/s in 1.5 s, was 1 s). Braking is never softened: a stop starts at once. `sim-check --obstacle` passes (stopped 0.38 m before the box) |
| 21 | Gazebo check: a 10° steer gives the predicted radius within 5% (PLAN T2.3) | ✅ done | 22 Oct, 04:55: 9.05-9.14 m for 8.90 m (+2-3%); 20° and full lock also within 3%. `docs/simulation.md` |
| 15 | Gazebo check: a tight turn at `sim_speed:=3` slows to ego's curve speed | ✅ done | asked 3 m/s at full lock: ego allowed 2.40 m/s (the computed limit), the car held 2.37-2.43 m/s with 1.9-2.0 m/s² sideways (limit 2.0); straight, 3.1 m/s. Probe: `/tmp/aav-probe/curve_probe.py` (not in git) |

### 15 Oct: all the C++ to Rust

Ben, 15 Oct: "We should port all cpp to rust", and remove the old.

- [ ] **Flash them** on spare Pico 2s, then on the car: `pixi run pico wheel build`,
      `pico sentinel build` ([Rust on the hardware](docs/rust-firmware.md)) [[needs-you]].
- [ ] **Something to feed the sentinel:** its 2025-26 sender isn't in any repo. A small
      ROS node that writes the packet over I2C (`sentinel::Telemetry::to_packet` shows the
      bytes), once we know the display is on the car (the audit).

### 14 Oct: the car in photos, the KEFC CAN plan

Ben sent 32 photos and the KEFC request (CAN bus, wheel speed, steering feedback, new GPS,
a second Orin). Pages: [car photos](docs/car-photos.md), [CAN plan](docs/can-plan.md),
[topic areas](docs/naming.md).

- [ ] **The simulated lidar on by default:** off for now, as on the Mac it made the full
      graph with YOLO flaky (DDS over loopback UDP). Try Fast DDS's shared memory, or a
      smaller cloud (`docs/simulation.md`).
- [ ] **Friday's audit:** the checklist at the end of `docs/car-photos.md`.

### 11 Oct: what we can try at home

Ben, 03:00: "We can only do so much from home so lets focus on stuff that makes sence to
work on." His order, with the work under way:

1. [x] **The full graph, all set up:** every driver, ego, the simulators. In Gazebo: a road
      with lane lines, a stop sign and an obstacle, a camera and a radar on the car, and
      perception, lane keeping, collision_monitor and the HUD in `pixi run sim`. *Done
      15 Oct: merged from `wt/sim-full`, with a lidar too ([Simulation](docs/simulation.md)).*
      - [ ] **Ackermann done properly**, testable in Gazebo and on the bench: reversing
            (Reeds-Shepp), both front wheels' real angles, the steering curve from
            `Ackermann.m` (Q2). *Paused 11 Oct, on `wt/ackermann`. Reversing done 22 Oct (task 24); both wheels' angles and the steering curve still open.*
2. [x] **Our C++ to Rust:** Ben's Rust IMU driver becomes the default and the C++ one goes;
      the radar clustering becomes a Rust ROS node. Python to Rust later (Ben). *Done
      15 Oct (Ben: "We should port all cpp to rust"): `aav_imu_yahboom` and
      `aav_radar_clusters`; no C++ left in `src/` (now `ros/`). Only the ESP32's firmware is C++ (and the hardware lead's Arduino).*
3. [ ] **The Mac on the network, with ML on it:** the laptop's detector (Core ML) on the
      car's compressed pictures. *On in `laptop.yaml` since 11 Oct; to try against the
      bench as the car.*
4. [ ] **Nav2:**
      - [ ] N7, the weave: the bench's pretend GPS adds fresh noise to every fix, unlike a
            real receiver, whose error drifts slowly. Model it as a slow drift, then tune
            `ekf_global` for the real receiver (Q18). *29 Oct (branch `stop-hold`): the
            drift is modelled (`aav_bench` `models.GpsError`); `bench-check --weave`
            measures the weave. Tuning `ekf_global` is left;*
      - [ ] collision_monitor flickers between stop and go at the zone's edge. *29 Oct
            (branch `stop-hold`): `aav_safety`'s `stop_hold` holds each stop until the
            zone stays clear 1 s (twist_mux `stop_zone`, priority 21); `bench-check
            --zone-edge` tests it.*
5. [x] **The board drivers in Rust, ready to try on a board:** the wheel encoder and display
      Picos (Embassy); the spine's Pico 2 program is written. *Done 15 Oct
      (`wt/rust-boards`): all three build; none flashed.*
6. [x] **The HUD in raylib** (Ben, 11 Oct: yes): one Rust program, raylib and rclrs, that is
      also the ROS node. The driving screen first (speed, mode, the e-stop button, the
      camera); the Flutter app stays until it matches. *Started 15 Oct (branch
      `wt/hud-raylib`): the driving screen, the e-stop, GPS and topic health run on a Mac
      (`pixi run hud --demo`, `pixi run hud`). Next: the bench, a Pi, the map page.*
      *25 Oct: it matched, so the Flutter app and its back end were archived
      (`archive/2026-27-flutter-hud/`, Q29); this package took the name `aav_hud`.*

### Ideas to keep

- [ ] **Your Mac as the operator station** (Ben, 11 Oct): the HUD on the Mac's screen,
      driving the car from the Mac, the Core ML AI driver on the Mac, maybe all of
      perception; the Orin doing the processing when you sit at the car. *Saved, not
      started. What runs where: `docs/history/conversation.md`, 11 October.* It waits on:
      - the car on Jazzy (Q24): a Jazzy Mac can't join the car's Humble;
      - the HUD on a Mac: raylib's runs there as it is; the Flutter app needs CocoaPods (archived 25 Oct);
      - the car's WiFi, measured: how late the pictures arrive, and how often it drops.
- [ ] **A hardware workbench: every sensor and actuator, one at a time** (Ben, 28 Oct,
      word for word in `docs/history/messages-2026-10-28.md`). Just for us. One screen on
      the laptop (the HUD), with the RC or the joystick as well, to isolate the hardware:
      - **each sensor, live:** is it there, how fast, its values (GPS, IMU, lidar,
        cameras, wheel speed, steering pot, battery);
      - **each actuator, by hand:** jog the steering, throttle and brake in small capped
        steps, and see what came back;
      - **on stands, find the limits:** steer slowly until the motor meets resistance:
        the Jrk G2 reports its current and position, so a stall shows as current rising
        while the position stops. Back off a margin: that's the steering limit;
      - **work out the numbers:** acceleration, braking and steering rate, into
        `vehicle.yaml`; then check them by driving in the lot.
      Builds on what we have: `pixi run actuator-test` (its `--stands` tests already measure
      the steering sweep and rate), the spine's hardware test mode (#15, capped at
      `TEST_MAX_THROTTLE`), bench mode and the raylib HUD. New: the live sensor page, the
      jog controls, and the stall search. Check the 2025-26 team's clamps first
      (`JRK_STEER_MIN`/`MAX`, `DAC_MAX_CAP` in `hardware_configs.h`). Steering and braking
      limits on the real car are the hardware lead's to sign off.
- [ ] **Tesla-style cubes on our HUD, then on the camera picture** (Ben, 28 Oct: "I want
      to see the occupancy network", "put it on a hud like we have", "merge the voxels with
      the camera feed to figure out what is where"). No new install: not OctoMap, which
      would bring PCL back (tried 28 Oct, undone). Gazebo first, where every pose is exact:
      1. **cubes from the lidar:** a voxel grid in Rust in `hud-core` (no ROS, own tests):
         each `/sensors/lidar/points` point into its 20 cm cube; cubes not seen for a while fade;
      2. **draw them:** raylib draws 3D cubes; colour by height, like Tesla's view;
      3. **onto the camera:** project each cube into the HUD's camera picture (`picture.rs`),
         from where the camera sits relative to the lidar; colour by distance;
      4. **what is where:** each RF-DETR box gets the distance of the cubes inside it
         ("stop sign, 14 m").
      On the car, 3 and 4 need camera-to-lidar calibration (research pick 4). Nav2's own
      `VoxelLayer` is a separate choice (it changes driving): only if 2D misses something.
- [ ] **Cubes for Nav2** (Ben, 30 Oct: "this is a car not a needle"): built as a trial,
      `voxels:=true` (`aav_voxels`, `docs/designs/2026-10-30_voxels.md`); in Gazebo it stops
      for the post and drives the lot tour like the scan (`docs/trials.md`). Left:
      `--rear-obstacle` and `--plan-around` with `--voxels`; a thin pole in the stop zone
      (`min_points: 3` on cubes); 3 runs of the lot tour; the Orin's ms a cloud; the car,
      leaning in a turn; then Nav2's 3D `VoxelLayer` on the same cubes; then make it the
      default if it wins.
- [ ] **The debug HUD** (Ben, 29 Oct, word for word in `docs/history/messages-2026-10-29.md`):
      the hardware workbench above, as its own HUD screen. It reads each sensor's and
      actuator's own node directly (`/sensors/...`, `/vehicle/...`, the spine's status),
      not what comes out of the muxes, so a fault shows where it starts:
      - **sensors:** shake the car and watch the IMU; turn a wheel and watch the encoder;
        GPS, lidar, cameras, battery, the steering pot;
      - **actuators:** watch the wheels turn, speed up and brake, in the spine's hardware
        test mode (#15): its caps and the e-stop still apply. Never around the spine;
      - **what the car thinks:** the lidar's cubes and the NDT match (where the scan sits
        on the saved map), drawn on the same screen.
- [ ] **measure controllers picks a random ROS domain without checking it's free** (found 29 Oct:
      it picked 191, the same as a Gazebo already running, and the two runs mixed). Until
      fixed, give it one: `AAV_SIM_DOMAIN=213 pixi run measure controllers ...`.
- [ ] **Two screens, not one** (Ben, 29 Oct evening, word for word in `docs/history/messages-2026-10-29.md`):
      1. **The hardware screen**: every sensor live, and **very simple buttons** for the
         actuators: speed +/-, brake +/-, steering left/right, **centre**. Through the
         spine's hardware test mode (#15) and its caps, never around them; the e-stop stays
         the stop. TODO 49b limits the throttle on stands until a rear-wheel speed exists;
      2. **The seeing screen** (today's debug HUD, split): left, big, the 3D cubes; right
         top, a top-down of the lidar map (ICP/KISS-ICP, or the NDT map today) with the
         car on it; right bottom, a top-down of NDT's match (the scan on the map's cells).
      Design: `docs/designs/2026-10-29_two-screens.md` (EGM-reviewed): `hw_panel`, its own
      program, twist_mux slot 45, hold-to-drive; `debug_hud` stays read-only.
      **Steps 1-3 built 29 Oct** (branch `two-screens`): `debug_hud` opens as the seeing
      screen (key L: the old panel); `hw_panel` (`pixi run hw-panel`, `hw-panel --demo`)
      with every sensor and the buttons through `/drivers/buttons/cmd_vel` (slot 45),
      speed on SIM and BENCH only (docs/hud.md, "The hardware panel"). Left: the window
      itself against Gazebo and both screenshots from it (the Mac's screen was locked:
      demo screenshots only); read the legacy bridge's `speed_loop.mode` before stands;
      the bench check (a person's joystick beats the buttons); step 4, option B (brake and
      steering through the spine's test mode, once it has a transport, TODO 0b)
- [ ] **A preflight on stands, before the ground** (Ben, 29 Oct): one scripted run, wheels
      off the ground: speed up, hold, slow down, brake; steer left, right, centre; and
      every sensor checked live (rate, values in range) at the same time. PASS or FAIL per
      line, as `car-day-check` does. Builds on `actuator-test --stands` and bench-check.
      **Built 29 Oct** (branch `preflight`; [the design](docs/designs/2026-10-29_preflight.md),
      revised after its EGM review): `pixi run actuator-test preflight --stands` (`--sim`, `--bench`, `--no-drive`, `--skip`). On the car the drive is
      refused until a rear-wheel speed exists (49b): `--no-drive` checks the sensors only.
      Every actuator test now also stops on 4 s of driving with no measured speed, and on
      a speed rising while the command falls. Left: the car itself (car-day session 9, 3.P)
- [ ] **Cubes into objects, with names** (Ben, 29 Oct: "merge cubes that related like a
      stop sign or a car"): join touching filled cubes into one object (as
      `radar_clusters` does for the radar); then name it: the RF-DETR box it falls in
      (needs the camera-to-lidar calibration, research pick 4), or later a small model
      that reads the cubes' camera crop. Each object: a name, a size, a distance. Shown on
      the debug HUD first.
- [ ] **numpy in the `ml` environment** (Ben, 28 Oct: "do we need to upgrade?"): not now.
      The pin (`<=2.3.5`) keeps `python3 tools/export_model.py --format coreml` working; the everyday environment already
      runs 2.5. Try again when coremltools passes 9.0: newer numpy, run `python3 tools/export_model.py --format coreml`.
- [ ] **Four research picks** (Ben, 28 Oct; details: `docs/designs/2026-10-28_research-picks.md`):
      1. **Autoware**: borrow its designs (PID + feed-forward speed with a smooth stop,
         MPC steering, NDT + EKF localization), not the whole stack. **Our own versions,
         in Rust** (Ben, 28 Oct: "take ideas from that and rebuild them in rust", "lets
         write our own version of it all"): the speed loop with a smooth stop
         (`spine-core`), NDT matching, MPC steering, the occupancy cubes. Cite what we read
         (Autoware is Apache-2.0; the NDT papers) in `PROVENANCE.md` and the report: the
         feed-forward + PI was our plan (27 Oct) before we read Autoware; MPC, NDT and the
         smooth stop came from reading it. Crates, checked 29 Oct: see the research picks;
      2. **lidar localization**: map the lot with KISS-ICP from a VLP-16 bag, then locate
         by NDT or ICP; no RTK on this car, so this is where centimetres come from;
      3. **a safety argument**: Simplex (we have most parts) and RSS (stopping distance
         from measured braking), for the final report;
      4. **seeing from above**: inverse perspective mapping (an AprilTag or a mark at B
         for the last metre) and camera-to-lidar calibration (distances for RF-DETR's boxes).
- [ ] **AMCL: a particle filter against a lidar map** (Ben, 28 Oct; not sure yet if we
      need it): Nav2's stock Adaptive Monte Carlo Localization. Thousands of guesses of
      where the car is; each lidar scan is matched against a map, and unlikely guesses
      die off as the car moves. ~5-10 cm where the lidar sees features (curbs, posts,
      walls), against GPS's 1-3 m: it could matter most for the last metres to B. Needs
      a map first (`slam_toolbox`), and a test site with things to see: a bare lot gives
      it little. Would sit beside the GPS + EKF, not replace them.
- [ ] **One model-based controller instead of many** (Ben, 27 Oct): investigate LQR, and
      collapse today's controllers (pure pursuit, the stopping law in
      `aav_drive_to_point`, the spine's PI and P-only brake) into one controller built
      from `vehicle_model.py`, closed loop, maybe running on the ESP32. Questions:
      - **speed:** throttle and brake as one signed acceleration, LQR with integral action
        (LQI), on the ESP32 where wheel speed and the actuators are;
      - **steering:** a lateral LQR on the path errors (sideways, heading, wheel angle),
        scheduled by speed; it needs the car's pose and path, which live on the Orin;
      - **stop on B:** distance-to-go as a state; the ESP32 could count the last metres on
        wheel odometry, independent of GPS lag (a CAN contract change: D4);
      - **MPC** if the limits matter (brake, steering rate): LQR doesn't know them;
      - scored with `measure atob` against pure pursuit and MPPI; needs the measured car first.
      *Saved, not started (`docs/vehicle-model.md`, "After state space").*
- [ ] **Learned state-space models, Mamba-style** (Ben, 27 Oct): Mamba (and S4 before it)
      runs the same `x' = Ax + Bu` as `vehicle_model.py`, but learns `A`, `B`, `C` and lets
      them depend on the input. Its diagonal `A` is a bank of first-order lags (τ = -1/a),
      like our steering lag. Worth a closer look once hours of car logs exist: a learned
      model of what the linear one misses (drag, slope, the brake's play), checked against
      the fits from the actuator tests. *Saved, not started; needs car data first.*
- [ ] **micro-ROS, again** (Ben, 11 Oct): "lets rethink thinks later after we see the
      hardware". No micro-ROS for now (Tentative).

### Overnight (10-10 → 10-11): all the code, then Ackermann, Rust, native style

Ben, 21:45: "Lets try to get a fully functionin gthing going on." The order of work is
Decided ([Decisions](docs/decisions.md)):

1. **The high-level pieces**, by five helpers in parallel, each on its own branch:
2. [ ] **Ackermann done properly:** reversing (Reeds-Shepp in Nav2), both front wheels' real
      angles in the model and RViz, and the steering motor's real curve from the
      `Ackermann.m` linkage (Q2). *Reversing done 22 Oct (task 24); the rest still open.*
3. [x] **All our C++ to Rust:** the IMU (Ben's twin becomes the default, then the C++ goes),
      the radar clustering as a Rust ROS node, the wheel encoder and display Picos (Embassy).
      *The two Picos, the IMU and the radar clustering: done 15 Oct. The Picos build, never
      flashed.*
4. [ ] **Every file in its language's own style:** `cargo fmt --check` and
      `cargo clippy -D warnings` in `pixi run test`.
5. [ ] **The whole car working, end to end.**

#### Needs Ben or the team

- [ ] A spare Pico 2 and a **3.3 V** MCP2515 CAN module (for example with an SN65HVD230),
      to try the spine on a desk. The common 5 V module (TJA1050) doesn't suit a Pico 2.
- [ ] The e-stop: a photo, and where its wires go (Q6).
- [ ] The brake after boot: `IDLE_BRAKE = 0.0` lets it go; the 2025-26 firmware kept it on.
- [ ] A fresh clone on oirp-spss before anyone works there.

### On Ben's Mac (10-10)

Ben's messages, word for word: `docs/history/messages-2026-10-10.md`. "Our version" is
`aav/aav` (`~/Code/aav/aav` on the Mac; retired 25 Oct: now `~/Code/ecolo-av`); this work was done in a clone at `~/aav` while that
folder downloaded, and moves into it as a fast-forward.

- [ ] **Prune the snapshots from `aav/aav/.git`** (149 MB down to about 1 MB)? *Ben to
      decide: it removes the app's undo points.*
- [ ] **Look at RViz and rqt_graph on the Mac**, and say what they show.
- [ ] **The small fixes** listed at the end of `docs/work.md`.

### The afternoon of 10-10: "merge in all of this", then "stub in the rest"

- [ ] **Team decisions:** the steering on an e-stop; the brake after boot; a brake on the RC
      gear switch (docs/firmware.md). *Since 11 Oct, one constant each in `spine-core`.*
- [ ] **Find `best.pt`**, the 2024-25 stop-sign model. (The 2025-26 lane finder isn't
      needed: we wrote our own, 11 Oct.)
      `best.pt` is probably on the team's Linux machine with the NVIDIA card (Ben, 10 Oct).
      No SSH to it today: later, once all the code is in place (Ben, 21:14).
- [ ] **Tune the speed loop on the car:** on the bench it trails a ramp by about a second.
- [ ] **Slim per-machine Pixi environments** (Orin, Motor Pi, laptop): at Phase 0.
- [ ] **An audit on the car's network** (Ben, 16:42: "at some point"): a read-only audit of
      the hardware. *Ready: the stand-ins stay on their own computer (`tools/ros.sh
      --local`, checked by `tools/test.sh`), so nothing pretend can reach the car.*

### Questions Ben asked

*Answered in the morning email and in the docs: `run.md`, `languages.md`, `mac.md`.*

- Is `aav_turtlesim` different from the regular turtlesim?
- What languages are used now, and should everything be Rust?
- Can it compile on a Mac without Homebrew?
- What does the claude.ai chat (fe09d020…) say? It can't be opened from here, so Ben
  needs to paste the key parts.

### For the team to decide

- **`ROS_DOMAIN_ID`** for the car and every laptop (0–101: safe on Linux; Fast DDS breaks above 232; one that no one nearby uses).
- **Spine arming rules** (design doc, Proposed): boot latched, arm on a 0→1 edge.
- **Move the car to Jazzy** before a laptop can join it (Humble and Jazzy don't mix):
  try Pixi on today's Ubuntu 22.04 first, then reflash the Orin to JetPack 7.2 after a
  backup (`docs/platforms.md`).
- **GPS driver** (B4): `ublox_gps` (in Pixi) is the default since 11 Oct, Tentative.
  `gps_driver:=ublox_dgnss` (apt, the car only) stays an option.
- **N2:** twist_mux hears `/safety/estop` best-effort, and the joystick sends it once per press.
  The backends still brake. Send it repeatedly, or make the mux reliable?
- **N3:** no `respawn` on device nodes, and Nav2 gives up without TF. Add respawn?
- **N7:** Nav2's turn command flips sign about once a second in the sim. Try a longer
  lookahead.

### Later

- Rewrite parts in Rust: now step 3 of the order of work, at the top.
- Measure the car and replace every guessed number in `vehicle.yaml`.
- Rotate the leaked secrets, and back up the Orin and Pis before any reflash.
- Answer Q1–Q34 with the team.

## Moved from TODO.md on 20 Nov (stage 1, batch 9)

Word for word; only the place changed.

### Stage 1: build everything left

| Batch | What | Rows |
|---|---|---|
| 1 | ✅ **ONNX with Core ML**: `measure onnx`, the diffusion planner from Rust, FP16, TensorRT ready (merged 19 Nov) | 243 |
| 2 | ✅ **Lanes into the costmap** (yellow vs white, the lane map's costs, `lanes:=camera`); the lane followers removed (merged 19 Nov) | 264 |
| 3 | ✅ **Before the car** (merged 19 Nov; untested on the car): NDT in `car.launch.py`, the adaptive brake on the car, the boot self-test that refuses to arm | 261, 258, 255 |
| 4 | ✅ **Maps offline** (merged 19 Nov): occupancy across passes, the offline pipeline as one command, 3+ drives and the 2D maps merged | 259, 260, 250 |
| 5 | ✅ **Labels and the twin** (merged 19 Nov): the labelling loop (open models offline, licences checked), the campus twin in Ravensim | 252, 253 |
| 6 | ✅ **Driving** (merged 19 Nov): reversing in a narrow street and in `aav_track`; the planner's checks on every core and a 1000-sample `mppi_lag` option; behaviour slows for kerbs | 174, 152, 219 |
| 7 | ✅ **The last Python nodes to Rust** (merged 19 Nov: geofence, stop hold, goal leash, GPS heading; lane pose and street router left, 265): the geofence, stop hold and goal leash (safety: shared vectors, both twins), the lane pose, the GPS heading, the street router; then the drivers (battery, radar) where worth it | Rust first |
| 8 | ✅ **The challengers built** (merged 19 Nov; scored in stage 2): smooth MPPI, jerk-limited passing, ground ring by ring, Kinematic-ICP, a tracker per kind | 212, 214-217 |

### 2. Maps and position

| # | Task | Status |
|---|---|---|
| 263 | **Unknown map cells read back as free** (found by the map editor, 19 Nov): grey 205 is p = 0.196, under every map YAML's `free_thresh: 0.25`. `make_map.py` and `map-chunks` now write unknown as 128 (p = 0.498), as `map-drive` and the map editor do; `example_lot_amcl.pgm` remade (244,730 cells inside its obstacles had read as free; `example_lot.pgm` byte for byte the same); a test pins it | ✅ 19 Nov. AMCL on the remade map, the lot, 2 runs: 0.06-0.07 m mean a leg, worst 0.09-0.14 m (18 Nov: 0.05-0.09, worst 0.15-0.20); one run's follower aborted a leg (error 107: the follower, not AMCL) |

### 3. Driving

| # | Task | Status |
|---|---|---|
| 262 | **The lane follower in Rust, and a better one** (Ben, 19 Nov: "We also need to port the lane following code.or find a better version."): `aav_lane_follower_rs`, the default (`lane_follower_impl:=python` falls back); challengers `lane_follower:=pure_pursuit`, `stanley` on the fitted lane centre, with the car's lag | ✅ ported: the Python's commands on 2,797 shared events; `measure lane`, 3 runs each: the Rust PID scores as the Python one; on Library Road pure pursuit 2.4 cm RMS against the PID's 3.9-4.1, 1 flip a minute against 187; on the straight it settles to 0.4-0.9 cm but comes back from 0.3 m off slower (8.4 cm RMS against 6.7). The PID stays the default. Left: pure pursuit's near look-ahead for a quicker return, then promote it; the car. **Removed 19 Nov** with the other followers (264): [their scores](docs/history/lane-followers.md) |

### 6. Clean-up

| # | Task | Status |
|---|---|---|
| 232 | **After Nav2**: `nav2_amcl` out of `pixi.toml`; the one-option switches (`costmap:=`, `navigator:=`, `smoother:=`, `controller:=`): keep as hooks or remove; `nav2.yaml`'s name | ✅ 20 Nov: stock `nav2_amcl` and its lifecycle manager gone (`pixi.toml`, `pixi.lock`; tag `save/nav2-amcl`), `amcl_impl:=rust` the one choice; the four switches kept as hooks (the league's `stacks.yaml` names them); `nav2.yaml` keeps its name (Nav2's layout) |
| 241 | **Ravensim's old names out**: `simulator:=own`, `target:=gazebo`, `car:=gazebo`, ignored args | ✅ 20 Nov: all four gone, and the URDF's and `description.launch.py`'s `sim`, `lidar`, `chase_camera`, and `sim.launch.py gui:=` (the tools no longer pass it); tag `save/old-names` |

