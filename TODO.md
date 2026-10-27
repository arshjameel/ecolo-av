# TODO

The current work list, newest first. Tick items off as they land; each tick = a commit.
Ben's messages, word for word: `docs/history/messages-*.md`.

## Thursday 29 October: control (Ben, 27 Oct)

Ben: "investigate LQR and maybe collapse all these various controllers into a single
controller ... closed loop". The ideas: "Ideas to keep" below.

| # | Task | Needs |
|---|---|---|
| 1 | **A page that explains it all:** PID, LQR, MPC and MPPI, with pictures; which loop runs where (ESP32, Jrk, Orin); what each knows. The next chapter of `docs/learn-control.md` ("Control from zero") | nothing |
| 2 | **LQR for steering:** from `vehicle_model.lateral()`, scheduled by speed; follows the planner's path (Smac), in place of pure pursuit; scored with `atob-score` and `controller-compare` | Gazebo |
| 3 | **LQR for speed and brake (LQI):** one signed acceleration, in `spine-core`; scored against today's PI and P-only brake | bench, Gazebo |
| 4 | **MPC for the whole A-to-B:** speed and steering together, with the brake's and steering's limits; scored against 2 and 3, and MPPI | 2, 3 |
| 5 | Unfinished from 26 Oct: goal circling (TODO 17, `wt/goals-steering`), the F6 geofence, the QEMU and NaN fixes (`small-fixes`), the bench fixes (`bench-fixes`) | |

Every gain still comes from guessed numbers until the car is measured (`docs/measure-the-car.md`).

## 24 October, night: the car as a linear model

Ben, 24 Oct: "Do we have a mathematical model for the car now for the ego? ... a matrix
control thing ... is that something we should consider? ... Maybe second order system".

| # | Task | Status |
|---|---|---|
| 52 | **The car's linear model**, in SYSC 3600's terms ([The car as a linear model](docs/vehicle-model.md)) | ✅ 24 Oct, `wt/vehicle-model`. `aav_ego` `vehicle_model.py` (no ROS; numpy, scipy.signal): speed K/(τs+1) with the throttle lag and the spine's PI (poles −0.28 ± 0.28j: settles in 13 s); the steering as a second-order system (ωn 12.6 rad/s, ζ 1.03); the bicycle linearised, x = [e_y, e_θ, δ], with linearised pure pursuit (ζ 0.707 at every speed); Bode margins with the delays (lateral, 1 m/s: 52° none, 45° typical 84 ms, 31° worst 259 ms); the discrete loops at 100 Hz and 20 Hz. 27 + 3 tests. `pixi run vehicle-model`: the tables, `docs/img/vehicle-model-*.png`. Found: the guesses disagree (drag 0.1 says full throttle holds 15 m/s; the open loop's `FULL_THROTTLE_SPEED` says 3); Gazebo's `steer_p_gain` 10 adds a steering lag after the actuator model: measured in Gazebo (headless, `steer-rate --sim` and `/joint_states`), its wheels follow the model's angle with a 0.110 s first-order lag (0.5 mrad rms) |
| 52b | **Refit the model from the actuator tests at the car** | ⬜ car day: the tests print the numbers; `pixi run vehicle-model --set throttle_lag=... --set drag=...`; then the speed loop's gains by pole placement (`place_speed_gains`), not before |
| 52c | Make Gazebo's steering match `vehicle.yaml` | ✅ 25 Oct, `wt/goals-steering`. Gazebo's wheels now follow the actuator model's angle with a **0.022 s** lag (was **0.111 s**; 2 runs each, 0.4-0.5 mrad rms left; a third on a busy Mac 0.063 s: the adapter's ticks stalled). Found: raising `steer_p_gain` alone made it worse (0.07-0.16 s): the URDF's steering joints had a 1.0 rad/s speed limit, which Gazebo enforces (the wheels turned at 0.63 rad/s, under the model's 0.8). Fix: the joints' limit 1.0 → 5.0 rad/s, `steer_p_gain` 10 → 100. The lag left is 1/gain + about 12 ms from the adapter's 50 Hz hold. Chose gain 100 over "the adapter's lag less 0.1 s" (11 ms off): `/vehicle/steering` stays the wheels' angle, and a new `steer_lag` needs no change in the xacro. Lateral phase margin in Gazebo 43° at 1 m/s (was 37°; the model 45°). `tools/gazebo_steering.py` measures it (with `actuator-test steer-rate --sim`); `test_vehicle_model_tool.py` keeps the extra lag under a quarter of `steer_lag`. `sim-check`: the scripted drive came up short (2.3-3.8 m of 6.5) with main's settings as with these, on a Mac at load 24 (other simulators): not this change. Next: the same step on the car (T2.7) |
| 52d | **The state-space primer** ([Learn: state space](docs/learn-state-space.md)), for Ben's SYSC 3600 refresher | ✅ 25 Oct, `wt/learn-state-space`. The 12 steps (why, free-body diagrams, Newton, states and A B C D, simulating, transfer functions, poles = eigenvalues, Bode and delay, what it means at 1 vs 3 m/s, shocks and roll, LQR and Kalman, a cheat sheet). `pixi run state-space-demo` (`tools/state_space_demo.py`, 16 tests): Euler, RK4 and exact side by side; a quarter-car and a roll model with GUESSED springs. Roll at 3 m/s on full lock: 2°, 46 % of the inside wheels' weight moves out, 0.32 g against a 0.70 g rollover limit: not a control problem; matters for the lidar's tilt. The kinematic bicycle holds to 3 m/s (the tyres' poles 4× faster than the steering's) |
| 52e | **Body roll tilts the lidar** (found by the roll model, [Learn: state space](docs/learn-state-space.md) section 10): at 3 m/s on full lock about 2° of roll, so the ground about 0.7 m off at 20 m, and could show as an obstacle in hard turns. Nothing to do at today's 1 m/s cap. Before raising it: check Nav2's obstacle layer's height filter against a rolled scan (Gazebo has no suspension, so it can't show this: a replayed bag from the car), or tilt-correct the cloud with the IMU's roll | ✅ 25 Oct, `wt/ekf-heading`: **safe up to 4 m/s, no change needed; today's 1 m/s untouched.** The ground filter is `lidar_scan`'s 0.3 m above the road (level body), marked out to 12 m (`lidar_layer` `obstacle_max_range`); collision_monitor's boxes are within 1 m of the bumpers, too close for the road to rise. `aav_lidar` `ground.py` (no ROS, 9 tests): where each VLP-16 beam meets a rolled road. The road shows above 0.3 m within 12 m from **1.46°** of roll. ego's 2 m/s² sideways limit holds the roll to **1.33°** at most (the springs' overshoot included, a sudden full lock from 1-4 m/s through ego's own model): the road at most 0.23 m up, 10 m out. At 5 m/s a sudden full lock overshoots (3.9 m/s², 2.4°): ego brakes 2 m/s² but the wheels turn at 0.6 rad/s. `tools/tests/test_lidar_roll.py` ties vehicle.yaml, ego.yaml, nav2.yaml and the roll model together. Not done: levelling the cloud with the IMU's roll, a trap: madgwick reads 2 m/s² sideways as 11.5° of tilt ([Costmaps](docs/costmaps.md#body-roll-and-the-lidar-todo-52e)). Left: springs and CG height are guesses: softer than ~7400 N/m a corner and the margin goes (measure day: the spring rates and the CG height, M7b) |

## 24 October, evening: the car-day kit

Ben, 24 Oct: "Lets get everything prepared so the only thing left it to get into the car."

| # | Task | Status |
|---|---|---|
| 51 | **The car-day kit**: one runbook, one preflight | ✅ 24 Oct, `wt/car-day`. [docs/car-day.md](docs/car-day.md) (in the docs as "Car day"): sessions 0-10, each with who, how long, what to bring, the commands, what you should see, what to do if not, and 🔒 where the hardware lead must answer first. `pixi run car-day-build` builds the ESP32 firmware (version 2 and `link-v1`) into `car-day/esp32/` (out of git), runs `esp-qemu-check`, builds the C++ way back and the docs PDF; `pixi run car-day-check` (ROS-free, 35 tests) says OK / MISSING with the fix, or REMIND. Built on Ben's Mac: both ELFs, QEMU PASS (111-119 ms), the C++, the PDF, and `models/rf-detr-nano.onnx` (108 MB: the Orin then needs only TensorRT's `trtexec`). Found: on JetPack 6, RF-DETR can't run inside the stream node under Pixi (Python 3.12 vs JetPack 6's 3.10 TensorRT): the streams run, the detector waits for the reflash (Q24). Also fixed: every `!!!` box in the HTML docs showed its text as code (four-space bodies with `tab_length` 2). Seen once: `esp-qemu-check` in `tools/test.sh` failed at load 18 (QEMU starved: the firmware's clock ran 0.82 s per real second, timeouts of 2.3 s); it passed alone a minute later and in the next full run. A flake under load, not the firmware: QEMU has no `-icount` |
| 53 | **The software on 64-bit ARM Linux, the car's computers, before the car day** (PLAN T9.2) | ✅ 25 Oct, `wt/ci-arm`: two CI jobs on GitHub's free ARM runner (`ubuntu-24.04-arm`, Ubuntu 24.04, 4 cores), on every push to `main` and every pull request. **`arm`** (11-12 min): `pixi run test` passes (1056 tests); the HUD and the map window (`aav_hud`) are built as aarch64 programs, and each draws its demo once on a virtual screen (125 KB and 106 KB pictures; the artifact `arm-screens`). **`car-kit`** (6-7 min): `pixi run car-kit`, then `car_install.sh --all` with no network at all (a network namespace of its own): every package built offline from the kit, the HUD too, in 4.5 min. Fixed on the way, CI only: under `sudo`, the runner's `PSModulePath` came back and colcon chose PowerShell. Still untested: the Orin's and the Pis' own Ubuntu 22.04 and JetPack |

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
| 36 | A bandwidth budget: every sensor's rate and size, each link's load, the CAN bus load (PLAN T1.7) | ✅ merged: [bandwidth](docs/bandwidth.md), `pixi run bandwidth`, `pixi run can-load`. Raw cameras are the only risk; CAN 8.5% |
| 37 | **Compress the cameras** (Ben, 23 Oct: "Lets compress it"; [bandwidth](docs/bandwidth.md)) | ✅ **software done, merged 24 Oct, as 37b**; at the car: car-day session 5. (MediaMTX H.264 on the Pi, decoded on the Orin; [camera pipeline](docs/camera-pipeline.md)); on the car: untested. Before: Today: the camera Pis (Pi 5) run **MediaMTX**, RTSP on 8554: cam1 the Pi camera, cam2 `ffmpeg` from the USB camera (640x480, 30 fps); not ROS (audit 21 Oct). Plan: keep H.264 on the Pis (the Pi 5 has no hardware encoder: x264, low-latency settings), keep MediaMTX as the server; on the Orin, GStreamer with NVIDIA's hardware decoder (`nvv4l2decoder`) turns RTSP into ROS images on the Orin only, so raw pictures never cross the network; the HUD and laptops watch MediaMTX's stream (WebRTC/RTSP) directly. In Gazebo: `image_transport` compressed. Measure latency and size with `pixi run bandwidth`. **Ben, 23 Oct, evening:** both front cameras are **ribbon, on one Pi 5** (Ben, 23 Oct: "we have 2 ribbin cameras hanging off one pi"; the Pi 5 has two camera ports; one `camera_ros` per camera; the same clock stamps both, good for the stereo pair; check the audit's CAM2-PI with its USB camera) (CSI: libcamera, `camera_ros`; no MJPEG from the camera, so the Pi 5 encodes in software: measure its CPU); the Orin is **probably an AGX** (hardware encode and decode; to confirm); a **second, lower-end Orin** is coming (PLAN T1.9). **Suggested pipeline (23 Oct, to write up as `docs/camera-pipeline.md`, Proposed):** for the computers, ROS compressed JPEG from `camera_ros` on each Pi (capture time stamped, bag-recordable, the same in Gazebo) → on the Orin, one process per camera: decode (nvJPEG hardware if the CPU is short) → undistort (OpenCV) → YOLO in TensorRT → small detection messages. **Revised again (Ben: ROS cameras, or just stream both to the Orin with MediaMTX?): lean to the stream.** The camera Pi runs Debian 13 with no ROS, and MediaMTX works there today: it owns both cameras (H.264); on the Orin, one node per camera pulls the RTSP stream, decodes it in hardware (GStreamer `nvv4l2decoder`), stamps it, and feeds YOLO in the same process; people watch the same stream in a browser (WebRTC). **Three cameras** (Ben, 23 Oct: "2 cameras on one and one camerea on qnx"): the front pair on one Pi, one camera on the QNX Pi (probably the audit's CAM2-PI, .118, USB 640x480, Debian 13 now: check; likely the rear camera). Ben, later: "There is 2 ribbin cables on the pi that is linux. the qnx doesnt' work": the front pair is ribbon, on the Linux Pi; the QNX Pi and its camera don't work today, so the car has no rear camera for now (reversing already counts only the lidar as seeing behind). Ribbon or USB is not certain (Ben: "it could be on a usb cable. who knows"): at the car, `rpicam-hello --list-cameras` (ribbon) and `v4l2-ctl --list-devices` then `v4l2-ctl -d /dev/videoN --list-formats-ext` (USB: if it offers MJPG or H264, the camera compresses and the Pi does nothing). Measure first: the Pi's CPU for two H.264 streams, the delay end to end, and the time stamp's error. Earlier idea: only one program can open a ribbon camera, so one owner per Pi: `camera_ros`, everything in ROS. People get a small preview topic (lower size and rate, or H.264 through `ffmpeg_image_transport`, which Foxglove/Lichtblick show) instead of MediaMTX; keep MediaMTX only if someone needs a browser view without ROS. GStreamer only where the hardware needs it; Isaac ROS (NITROS, zero copy) as the upgrade. Not OpenCV's DNN module for detection |
| 43 | **The control board's bridge refuses to arm while the safety gate is shut** (Ben, 24 Oct: yes) | ✅ built 24 Oct, `wt/arm-gate`. `/spine/arm` refuses unless `/safety/output_mux/state` said `OPEN` for the bridge's target (car, bench, gazebo) within `gate_timeout` (1.5 s), with the reason; both bridges (`spine-bridge` `gate.rs`, `aav_spine/output_gate.py`). The bench and Gazebo's `arm:=true` now start `arm_when_open`, which asks each second until OPEN. `legacy_uart_bridge` has no arm step. Also fixed: `output_mux`'s clear logged at two severities from one line, which rclpy refuses ([Arming waits for OPEN](docs/sources-and-targets.md#arming-waits-for-open)) |
| 43b | The guard test's SIM case let 13 DRIVE_CMD frames through once on main (load 29): cross-talk or a start-up race? | ✅ 24 Oct, `wt/arm-gate`: **cross-talk**. Our `sim-check --spine` (domain 193) was latched that day by another run's REAL `spine_bridge`; the SIM case passed 16 of 16 at load 24-42 and the whole guard file 44 of 44 at load 10-48, with 24 busy processes; the announcement and the graph agree within 1 ms in one process. Fixed: the guard test, `sim_check.py` and `bench_check.py` take a domain nobody uses (`aav_io_mux/domains.py`), and the test fails naming any other run's nodes. The race's small window (across processes the graph up to 30 ms before the message) closed too: a node on `/system/source` not heard yet holds the gate, and an announcement counts before its node shows in the graph. New tests: Gazebo starting late (frames stop within 1 s), and an announcer seen before its announcement |
| 39 | GitHub's Linux tests failed twice on 23 Oct (`6f801dfc`, `4bed1125`), then passed on the next push with the same code: flaky, **in the build**, not a test (no failing test case; the log's tail only "Starting >>>") | ✅ not seen again: 0 failed pushes to main since `4bed1125` (40 runs to 25 Oct). `ci_annotate.py` now names the failed package and its error lines on the run's page. Next failure: read it there, then fix the cause (a guess: a download during the build, such as crates.io for the Rust packages) |
| 42 | The Mac nightly hung in `pixi run test` on 24 Oct (started 11:50 UTC, cancelled at the job's 90 min limit, nothing to read; 23 Oct's took 12 min). Linux CI and the Gazebo job passed on the same commit (`1ff7f587`); on this Mac all tests finish in 41 s, with local and with subnet discovery | ✅ 25 Oct, `wt/ci-arm`. **Cyclone DDS at a test's end, on the network**: aav_routes' tests all passed, then freeing the last ROS node (`dds_delete`) waited for ever to join Cyclone's multicast reader, which stops only on a packet the computer sends itself; on GitHub's Mac that packet never comes. Found by a probe (native stacks with lldb, Python's with faulthandler, as annotations; commit `82d9eda4`); every GitHub Mac run hung so since Cyclone became the default (23 Oct): the 24 Oct nightly, then 3 of 3 on this branch. pytest's `faulthandler_timeout` never fired: the tests had ended. Fixed: `pixi run test` and `tools/test.sh` keep every test on this computer (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`: unicast on the loopback, no multicast reader), and the street router test takes a domain of its own. That showed cross-talk (two packages' tests side by side on one domain heard each other), so colcon tests one package at a time: 1 min 55 s, not 1 min 41. Then the heartbeat chain test wanted 30 of 50 frames in a second and that slow VM wrote 23: it wants 10 now. The Mac job then passed: 18 min, its tests 15.5 |
| 40 | `sim-check --faults`' `odom freeze` failed most runs since 22 Oct (Mac and oirp-spss) | ✅ 24 Oct, `wt/odom-freeze`. The cause: **MPPI** (Nav2's default since 22 Oct) plans from the wheels' speed, so frozen wheels keep the car only 0.1-0.2 m/s faster, under the GPS part's 0.3 m/s (with `controller:=rpp`: 4 of 4 caught). The watchdog's GPS part now fits 10 s, within 0.15 m/s (normal driving's worst: 0.04-0.08); the check freezes the wheels at 0.2 m/s and counts them wrong by the watchdog's limits. `--faults`: 25 of 25, three runs out of three ([frozen wheels and MPPI](docs/simulation.md#frozen-wheels-and-mppi)). Still to do on the car: the GPS's real noise over 10 s |
| 41 | `aav_camera_stream`'s `test_reader_restarts_an_ended_stream` is flaky (found 24 Oct: 1 in 5 alone on the Mac, and once in `pixi run test`): `os.killpg` in `decoders.py` `stop()` gets `PermissionError`, so the reader starts only twice | ✅ fixed 24 Oct: on macOS `os.killpg` says EPERM, not ESRCH, for a group whose last process has exited but not been reaped; the decoder's stop now treats both as "gone". 10 of 10 passes |
| 37b | **The camera stream, concretely** — ✅ **decided by Ben, 23 Oct**: MediaMTX on the Pi, compressed, to the Orin; OpenCV and detection there ([decisions](docs/decisions.md#decided-on-22-and-23-october)). OpenCV: which version JetPack 7.2 ships, and whether with CUDA and GStreamer, is **not checked** (JetPack 6 shipped 4.x without CUDA). **Ben, 23 Oct: "if htat is opencv 4 then thats what we use"**: JetPack's own OpenCV, whatever version; no own build. OpenCV only does picture work, so CUDA in it matters little. **The detector: RF-DETR Nano** (Ben, 23 Oct: "I thought we were skipping yolo and using the other one"; [newer models](docs/model-comparison.md): the stop sign from 20 m, YOLO11n 10 m; Apache-2.0, not AGPL), in TensorRT, not OpenCV's DNN module. ✅ RF-DETR Nano is the default since 24 Oct (`aav_detector/detectors.py`, YOLO with `detector:=yolo`; Mac: Core ML 10 ms, `sim-check --perception` stopped 4.4 m before the sign). Left: build the TensorRT engine on the Orin (`tools/build_engine.py`) and time it (Ben: "we just need to figure out how to stream and encode it over the network and then what to do with it on the other side") | ✅ **software done, merged 24 Oct; GStreamer on Macs 25 Oct (27 ms)**; at the car: car-day session 5. Was: built 24 Oct, measured on the Mac; untested at the car. ([camera pipeline](docs/camera-pipeline.md)). `aav_camera_stream`: the Pi's `pi/mediamtx.yml` (MediaMTX itself loads it in a test), one `camera_stream` node per camera (decoder as a child program: GStreamer `nvv4l2decoder` on the Orin, ffmpeg or GStreamer on a Mac; OpenCV undistortion; `aav_detector`'s RF-DETR Nano in the same process; detections, a 4 kB preview a second, `/diagnostics`, `/system/source`). `pixi run -e ml stream-check` (Gazebo → OpenH264 → local MediaMTX → node): **26 ms** camera-to-detections with GStreamer, 160 ms with ffmpeg (its decoder holds 2 frames); 2.0 Mbit/s for a noisy picture; stop sign seen from 25 m in Gazebo. To do at the car: install the Pi file, the Pi's CPU for two streams, the delay and stamp error (a stopwatch), `delay` from it; calibrate the ribbon cameras; a `car.launch.py` part for the Orin. The plan as it was: **Pi:** MediaMTX owns each camera: ribbon via its `rpiCamera` source (software H.264 on a Pi 5), USB via `ffmpeg` (pass MJPG/H.264 through if the camera has it); 1280x720, 15 fps, about 2 Mbit/s, a key frame every second, no B-frames, low-latency settings. Config in the repo. **Network:** RTSP, port 8554, wired. **Orin:** one node per camera: GStreamer `rtspsrc ! rtph264depay ! h264parse ! nvv4l2decoder` (NVIDIA hardware; `avdec_h264` or `vtdec` on a Mac), into OpenCV (undistort) and YOLO in TensorRT **in the same process**; publishes detections, plus a small preview now and then. Stamp = arrival minus a measured delay (chrony on the Pi too). NVIDIA's GStreamer plugins are JetPack's own, not Pixi's: the node uses the system's GStreamer on the Orin. **Test without the car:** Gazebo's camera → `ffmpeg` → a local MediaMTX → the node → detections, on a Mac; measure the delay |
| 37b.1 | **RF-DETR Nano is the detector, not YOLO** (Ben, 23 Oct: "I thought we were skipping yolo and using the other one") | ✅ merged 24 Oct. `aav_detector` runs RF-DETR Nano by default (since 20 Oct), YOLO with `detector:=yolo`. New: the models as plain Python objects, no ROS (`aav_detector/detectors.py`: `load_detector()`, `detect(bgr)`), for the camera-stream program; RF-DETR's **Core ML** export (`pixi run -e ml export-coreml`: 10 ms a picture on Ben's Mac, 16 on Metal) and its **TensorRT engine** (`python3 tools/build_engine.py` on the Orin: `.pth` → ONNX → FP16 `.engine`, then a test run); `orin.yaml` loads `models/rf-detr-nano.engine`. `sim-check --perception` passed with RF-DETR (first seen at x = 5.9 m, the sign at 25 m; stopped with the bumper 4.4 m before it). **Untested: everything on the Orin** (needs the reflash, Q24): build the engine, time it |
| 37c | The camera stream's follow-ups (merged 24 Oct, `aav_camera_stream`, [camera pipeline](docs/camera-pipeline.md); `pixi run -e ml stream-check`, VideoToolbox by default: 27 ms camera to detections, 15 fps, 0.13 Mbit/s for Gazebo's plain scenery) | 🔧 (1) ✅ 24 Oct, branch `wt/camera-mac`: a Mac decodes with GStreamer by default (`decoder:=auto`: `vtdec`, VideoToolbox, else `avdec_h264`, else ffmpeg with a warning), from a new small `gst` environment (`pixi install -e gst`; no re-solve: `pixi lock` only added it and the `calib` one, every other environment's packages unchanged). stream-check: 4 ms decoded, 27 ms to detections (ffmpeg: 160). (2) ✅ 24 Oct, branch `wt/camera-car`: the sensor watchdog watches each stream through its detections (kind `detections`, by `frame_id`: `front_left_stream`, `front_right_stream`; quiet 1 s or NaN stops Nav2), not `/diagnostics` (1 Hz, its rate over 5 s: slower). `stream-check` checks it: OK for 60 s, STALE 1.1 s after the stream stopped. The lane finder gets `image_raw` from the stream (`publish_image`) when the lane part is on. (3) ✅ 24 Oct, same branch: `car.launch.py`'s `camera_stream` part, on in `orin.yaml` (CAM1-PI `192.168.1.108`, `gst-nvidia`, RF-DETR's engine); it replaces the detector part for streamed cameras, feeds the stop-sign watcher and camera_obstacles, and is skipped (with a warning) until the engine is built. Tried with a stand-in machine file on the Mac (two local streams); untested on the Orin. (4) ✅ 24 Oct, `wt/camera-mac`: the stop-sign watcher stops at a distance (4.7 m, the old 55 px with Gazebo's lens) from each camera's focal length (its `CameraInfo`), so it carries over to the 1280x720 ribbon cameras once calibrated. (5) ✅ 24 Oct, same branch: the calibration is ready for the car: `pixi run calibrate-camera` (the board to print, stock `camera_calibration` on the stream from a `calib` environment, the result filed in `config/calibrations/`; [how](docs/camera-calibration.md)); tried with a made-up board stream, the buttons untested. At the car: install the Pi file, the Pi's CPU for two streams, the real delay, calibrate the cameras (`pixi run calibrate-camera`), check the stop at 4.7 m |
| 38 | **Match the plan to the car's wiring** (Ben, 23 Oct, evening; to confirm with the hardware lead) | ✅ merged 24 Oct; the wiring is **as Ben understands it, the hardware lead confirms** ([today's wiring](docs/can-plan.md#todays-wiring-23-october)). **Actuators** (throttle, steering, brake), maybe the steering-angle and brake-position sensors, on the **ESP32**, through the **Motor Pi** (probably UART); the **CAN bus straight to the Orin**, with the wheel sensors' Arduino and a few others. Built: `spine_bridge role:=sensors` (Rust and Python): no `/can/tx`, no `/spine/arm`, no e-stop path, ignores the spine's frames; `/vehicle/odom` from the two wheels (no yaw rate) and `/vehicle/wheels`; announces only odometry and wheels. `car.launch.py actuators:=uart` (the default): `legacy_uart_bridge` on the Motor Pi drives, the Orin's `spine` part reads CAN for sensors (no CAN sender); `actuators:=can` keeps the CAN spine. `orin.yaml` and `all.yaml` turn the spine part on; the 2025-26 UDP encoder is off (two `/vehicle/odom` would stop the output mux). Tested: the guard test (both languages; no frame on `/can/tx`, the mux opens), launch tests, the Rust node fed wheel frames (1.0 and 1.2 m/s gave odom 1.1). Untested: SocketCAN on the Orin, the Arduino's frames. The Rust ESP32 firmware's 100 ms timeout is on the UART link (Gate 2). Questions for the hardware lead: [audit questions H1-H8](docs/audit-questions.md#8-the-cars-wiring-for-the-hardware-lead-23-oct) |
| 43a | **Foxglove and Lichtblick built in** (Ben, 11 Oct: "at some point"; go-ahead 24 Oct) | ✅ 24 Oct, branch `wt/foxglove` ([Watch from a laptop](docs/foxglove.md)). One bridge launch (`foxglove.launch.py`) for sim, bench, CARLA and the car: stand-ins on 127.0.0.1, the car on its network and watch only; a WiFi topic list without raw cameras, full lidar or costmap layers; the camera preview on the stand-ins too. Three layouts in `tools/foxglove/` (driving, sensors health, map); the sensor watchdog's diagnostics now carry each sensor's rate and gaps. Tested live: road sim and bench, every layout topic served, 1.4 Mbit/s for all three, a goal sent through the bridge drove the car. To do: open the layouts in the app by hand; at the car, `pixi run layout-check ws://<Orin>:8765 --data`; offline map tiles for the map panel |
| 44 | **Fix the HUD** (Ben, 24 Oct: "fix the hud?", "go"): (a) the HUD shows the safety gate; (b) a HUD that keeps crashing holds Nav2 until a person clears it (PLAN T9.5, T9.8) | ✅ merged 24 Oct (`wt/hud-gate`). (a) A **GO / STOP panel** under the speed: green GO, amber WAIT, red STOP or AUTO STOP, grey NO GATE, an icon, and the one main reason (E-STOP ON; the output mux's, such as "GPS is SIM"; the watchdog's first input that stops, such as "GPS STALE") ([HUD](docs/hud.md#may-the-car-move-the-go-stop-panel)); logic in hud-core `gate`, tested; **not yet seen on screen** (the Mac's lid was shut: no window). (b) **Latched**: one dropout within 5 s of the HUD's first beat is forgiven, any other is `hud LOST` until `pixi run watchdog-clear` ([decisions](docs/decisions.md#decided-on-24-october)); new tasks `watchdog-clear`, `gate-clear`. `sim-check --faults` 25 of 25 (24 Oct, Mac): `hud stop` stood still 2.4 s after it, "hud LOST", still held 5 s after the beats came back, drove again once cleared; its first run found `watchdog hang` latched the HUD (Gazebo's clock catches up in steps after a stall): fixed, stalls are now seen on a steady clock |

## 24 October, afternoon (Ben's message: `docs/history/messages-2026-10-23.md`)

| # | Task | Status |
|---|---|---|
| 45 | **The ESP32 brakes on a lost heartbeat** (Ben: "if it doesn't get a heartbeat then it applys the breaks and stops the acceleration"). Ben confirms the wiring: Orin → network → Motor Pi → serial → ESP32 → throttle and the steering and brake motor controllers; CAN only the wheel encoders and the few listed; IMU, GPS, lidar on the network | ✅ **software done, merged 24 Oct, proven in QEMU**; at the car: car-day session 6. This is the Rust ESP32 firmware's 100 ms command timeout (`spine-core`, the UART build; `docs/esp32-bench.md`): no fresh command from the Motor Pi within 100 ms → full brake, throttle 0, latched until re-armed. Today's ESP32 C++ firmware has none (audit 16 Oct). **Proven in QEMU, 24 Oct** (`pixi run esp-qemu-check`; [the emulator](docs/esp32-bench.md#before-the-board-the-emulator)): the real firmware, fed `aav_legacy`'s own frames, brakes 111-120 ms after the last one (brake Jrk 3500, DAC 0, fault bit 1) and stays braked until a stop frame; the Jrks and DAC are stand-ins there. It also found the firmware never turned the FPU on (fixed). Next: flash it on a spare ESP32 on the bench, the same check with the real chip (Gate 2, 27 Nov), then on the car with the hardware lead. Every link in the chain then fails safe: the Orin's watchdog, the Motor Pi's bridge, the serial cable, the ESP32 |
| 46 | The steering-angle and brake-position sensors are probably on the ESP32 (Ben) | ⬜ hardware lead, question H2 (`docs/audit-questions.md`): which pins, which sensors. **Hint from the old code (24 Oct):** steering and brake each run a Pololu Jrk G2 (I2C 0x05 and 0x04), which closes its own position loop from a feedback input; the 2025-26 ESP32 never reads it back, but the 2024-25 `FinalRev` firmware read the steering position from the Jrk over I2C. So the angle and brake position are likely one I2C read each from the Jrks; then the Rust firmware reads them and sends them up the serial link (Gate 3, closed loop) |
| 47 | The watchdog checks the camera program's health, not raw pictures (Ben: "that seems like a good idea") | ✅ already done 24 Oct (TODO 37c): the watchdog watches each stream's detections (sent for every picture, ~15 Hz, even with no boxes): quiet 1 s → Nav2 stops. Detections, not `/diagnostics`: those come once a second, averaged over 5 s, so a lost camera would be caught seconds later |
| 48 | **The RC remote and the joystick through ROS, with direct drive kept for testing** (Ben, 24 Oct: "allow for both ... eventually we will move that to ros network") | ✅ **built and merged 24 Oct, Proposed** (off until H9 and H10); at the car: car-day session 6e. Was: Proposed and built 24 Oct (branch `wt/esp32-uplink`; [the uplink](docs/rust-firmware.md#the-uplink-the-esp32s-status-back-to-the-pi-24-oct); a Proposed row in `docs/decisions.md`): with `--features uplink`, CH6 reads three positions: low computer, middle (1301-1700) **RC via ROS**, high **RC direct**. RC via ROS: the ESP32 sends the sticks up and drives only from the Pi's frames; `aav_rc`'s `rc_driver` turns `/vehicle/rc` into `/drivers/rc/cmd_vel` (twist_mux priority 95). Losing the RC in either RC position brakes (fault 256) and latches until a stop. QEMU (fake iBUS on UART2): all six RC steps pass; a ROS test: uplink sticks → `/drivers/rc/cmd_vel`. Left for the car: H10 (is CH6 a three-position switch? the receiver's failsafe), [bench steps U1-U6, R1-R8](docs/esp32-bench.md#the-uplink-and-the-rc-modes-on-the-bench). Earlier: design, then the hardware lead. Today: the RC transmitter → iBUS → the ESP32 drives directly (switch channel 5); the USB joystick → the CORTEX Pi → ROS already (`joy_node`; ours: `aav_teleop` → twist_mux). Plan: a transmitter switch with three positions: **computer** (ROS drives), **direct** (the ESP32 drives from the RC, no computer: for testing, and the fallback), **RC via ROS** (the ESP32 sends the sticks up the serial link; a ROS driver `/drivers/rc/cmd_vel` at people's priority in twist_mux, through ego's limits). Needs an uplink frame ESP32 → Pi (the status the Pi doesn't get today). Direct stays until RC-via-ROS has been proven on stands |
| 49 | **Is I2C good enough for the Jrks?** (Ben, 24 Oct) | ⬜ hardware lead: how long are the wires from the ESP32 to the two Jrks, and do they run beside the motor cables? The load is tiny (50 updates a second, a few bytes each), so speed is fine; the risk is noise on long wires (I2C has no checksum and can lock up). The Rust firmware already drives both Jrks over I2C, flags a Jrk that stops answering (fault bits 8, 16) and brakes. If the wires are long or noisy: the Jrk G2 also takes TTL serial with a CRC (Pololu's protocol), a small change in the firmware |
| 50 | **The safety code lives on the ESP32; the serial link triggers it** (Ben, 24 Oct: "thats the place to put the safety code ... some kind of communication system that triggers it") | ✅ **built and merged 24 Oct**; at the car: car-day session 6 (branch `wt/orin-heartbeat`; [the heartbeat](docs/rust-firmware.md#the-heartbeat-version-2-of-the-serial-frame-24-oct)): the output mux's counter on `/control/<target>/heartbeat`, relayed by `legacy_uart_bridge` into a version 2 frame (17 bytes, CRC-8/SAE-J1850), and the Rust ESP32 ignores any frame whose counter didn't change. QEMU: brakes 111-120 ms after the counter freezes (the Pi still sending at 50 Hz) or the CRCs go bad, latched until a stop. The 13-byte frame stays (`link-v1`; the car's `legacy.yaml` says `frame_version: 1` until the Rust is flashed). **The uplink, built 24 Oct** (branch `wt/esp32-uplink`; [the uplink](docs/rust-firmware.md#the-uplink-the-esp32s-status-back-to-the-pi-24-oct)): a 35-byte frame ESP32 → Pi on TX1 (GPIO17), 50 Hz, the same CRC-8: mode, mode switch, fault bits, the echo of the last Orin counter, the Jrks' feedback positions (unknown when a Jrk has no feedback set), the outputs, and the RC sticks; shared Rust/Python vectors. `legacy_uart_bridge` (`uplink: true`) publishes `/vehicle/status`, `/vehicle/steering`, `/vehicle/brake` and `/vehicle/rc`; with `uplink: false` (the default) nothing changes. Behind the firmware feature `uplink` until the hardware lead confirms TX1's wire (H9). QEMU: 50-51 Hz, the echo right in all 6 cycles, 0 frames lost; the check now passes on a loaded computer too. Untested on a real ESP32 and Pi: [bench steps U1-U6, R1-R8](docs/esp32-bench.md#the-uplink-and-the-rc-modes-on-the-bench). Earlier: design agreed in principle (it's the spine design: `docs/rust-firmware.md`). **Ben, 24 Oct: the heartbeat must come from the Orin and travel to the ESP32; the Pi could be off the network and still be sending.** So: the Orin's output mux (the last Orin node, which is shut whenever the safety layer says so) puts a counter in every command; the Pi only relays it, never makes one; the ESP32 brakes if the counter hasn't changed for 100 ms, however many frames the Pi sends. Today's 13-byte frame can't carry it well: a sum checksum, no counter (a stuck Pi resending the same frame would look alive), no arm or e-stop bit, nothing back up. **A version 2 of the serial frame**, as the CAN plan's DRIVE_CMD already has: a counter that must change, armed and e-stop bits, a CRC-8; and an uplink frame ESP32 → Pi (mode, faults, brake and steering positions from the Jrks, RC sticks for TODO 48). The ESP32 enforces its own limits whatever the Pi sends (speed cap, steering rate, braked at boot). Keep the 13-byte frame as a fallback until v2 is proven on the bench |

## Working list, from 22 October (Claude works from this; Ben checks it in the morning)

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
| 1 | MPPI as a choice (`controller:=mppi`), and a script that compares it with Regulated Pure Pursuit | ✅ done | `f22dc4b8`; results in `docs/controllers.md` |
| 2 | ego knows the car's weight (181 kg) and its body | ✅ done | `7e76850f`; the centre of mass is a guess until the car is weighed (task 14) |
| 3 | The proposal as a to-do list with % done | ✅ done | `docs/plan.md`, `b70300d7` |
| 4 | The AAV colour: the email's blue | ✅ done | `da411333` |
| 5 | Email Ben as work lands | 🔧 ongoing | part 1 ~01:30; part 2 ~02:50 (every goal reached; the DDS finding); part 3 ~03:20 (MPPI default; Cyclone); part 4 ~04:06 (round the box; the U-turn; the early push); part 5 04:38 (three-point turns); part 6 05:14 (jerk limit, steering check, nightly Gazebo; the part 4 time corrected); part 7 06:08 (the EKF heading; the antenna mount; part 6 corrected); part 8 06:41 (good morning: the forward U-turn still a coin toss) |
| 6 | MPPI loops at goals | ✅ fixed (`7697778f`), confirming in 8 | it now turns to the goal's heading from 2.5 m out: left turn 28 s, 21.4 m (was 58 s, 47.5 m with a loop); round the building reached. Confirm in the full comparison (task 8) |
| 7 | The corner problem (docs/plan.md M8): the car stops at the building's corner | ✅ fixed | the planner's path passed 0.86 m from the corner. Now the planner's costmap inflates 3.0 m, gently (1.0), so paths keep to the middle of a lane; and it plans turns 1.25 x wider than full lock (`vehicle.PLAN_TURN_MARGIN`), so the controller has steering to spare. Lot run, 02:00: both kept clear of the building, both reached the first goal in 29 s, each reached 3 of 4 goals (were 1 and 2) |
| 17 | The car still sometimes circles a goal it just missed (a different goal each run) | 🔧 testing a fix | not the heading: the comparison now records where the global EKF put the car, and at walking pace its heading is good (worst 5°). Its **position** is up to 0.7 m off (Gazebo's GPS noise is 0.5 m), but the goal tolerance was 0.5 m: the car chased GPS noise with a loop. Tolerance now 1.0 m ("GPS without RTK is good to about a metre"); tighten with RTK. Run e, 02:20: **all 8 goals reached**, both controllers. Still extra driving on 2 legs (RPP's left turn 81 m for 21; MPPI's turn back 68 m) |
| 8 | Run the full comparison again with 6 and 7; update `docs/controllers.md` | ✅ done | 6 runs overnight (b-f); the page has the fixes, a table of runs and a picture of the best. Pushed 03:18 (815 tests pass) |
| 9 | Make MPPI the default, if it now does as well as Pure Pursuit (Ben: "I thought we decided to go mppi") | ✅ done | in every lot run MPPI did at least as well; it reached the first goal cleanly in all 4, Pure Pursuit looped there in 2 of 3 (it aims only at a point, so it arrives facing wrong). `vehicle.DEFAULT_CONTROLLER`; `controller:=rpp` for the old one. `sim-check --obstacle` passes with it. Untested on the Orin's CPU |
| 10 | **Split `gazebo_adapter` into one node per sensor**, as on the car, so each runs on its own core (Ben, 22 Oct). Rust for any part still late | ✅ done | `gazebo_imu`, `gazebo_gps`, `gazebo_radar`, `gazebo_camera` (one process each); `gazebo_adapter` keeps the car (actuators, wheels, odometry). The IMU works out the car's acceleration from Gazebo's odometry. Each sensor node takes its own `fault.<sensor>`. `sim-check` passes; `sim-check --faults`: 14 of 16, every fault reaches its node; the 2 fails were a whole-Mac stall (task 11). Not on the car: there each sensor already has its own driver |
| 11 | A long timing test (10 min, parked and driving) to catch the rare late data, before and after 10 | ✅ run 23 Oct (task 32) | a 1 min test was clean: real time 0.99, every topic under 80 ms late. Seen since: the IMU, wheels and GPS silent 0.7-2.1 s together in MPPI's lot run (one adapter, before 10); and after 10, in `--faults`, a stall of everything at once (the global EKF missed its rate, the controller ran at 8 Hz, `map → odom` 1.4 s old). So not only the adapter: find what the whole Mac waits on. 02:13 run: IMU, wheels, both EKFs silent 0.5-0.7 s together, with the Mac 77% idle and nothing else busy: so not a CPU shortage. **Found: it's the messaging (Fast DDS), not macOS.** A bare 50 Hz publisher and subscriber beside a sim run, 25 min: the publisher's timer never slipped more than 0.03 s, a no-ROS loop 0.06 s, but the subscriber waited up to 0.48 s (9 times over 0.1 s). **Cyclone DDS: worst 0.05 s, none over 0.1 s** (25 min beside a sim run), and `AAV_RMW=cyclone pixi run sim-check --obstacle` passes everything (the rear camera at 10 Hz, 7-8 on Fast DDS). Opt-in now: `AAV_RMW=cyclone` (tools/ros.sh). ✅ **Ben, 23 Oct: "yes lets do it"**: Cyclone is the default everywhere (`pixi.toml`); `AAV_RMW=fastdds` goes back | Also seen: Spotlight (`corespotlightd`) up to 75% of a core now and then |
| 18 | Stuck facing a wall: after a loop or a wide turn the collision monitor stops the car facing a wall, and forward only it can't get out | ✅ way out on | not a false stop: at the building the car had turned toward it before passing it, and a box that bends with the steering (`VelocityPolygon`) would have bent further in. So not built. The way out is reversing (Nav2's back-up), off on the car until the team decides. With reversing, both controllers now get round every goal of the lot (run n, task 19). ⚠️ **Team: may the car reverse?** It needs something that sees behind it. **22 Oct, morning: Ben said yes, the car may reverse** (task 24) |
| 20 | Goals: a miss of heading still cost a loop | ✅ done | heading tolerance 0.5 → 0.75 rad (the EKF's heading is up to 25° off; both controllers arrived 35° off by cutting the plan's last bend). Run j, 04:03: the first two goals clean for both (29-32 s, 21-22 m; 53 s, 42 m). The U-turn failed for both: its plan is a light-bulb turn using the whole 14 m lane |
| 19 | MPPI's U-turn: forward in the 14 m lane it went wide or round the building (3 of 4 runs); with reversing, the planner drew a three-point turn but MPPI never reversed: it drove on to the far wall, stopped, and the back-up recovery failed | ✅ done with reversing | three causes, with reversing on: (1) `PreferForwardCritic` kept MPPI from backing up at all (runs h, k: 0 reversals), so it is dropped when reversing: the planner's `reverse_penalty` already decides when to reverse; (2) `PathAngleCritic` mode 2 and inversion tolerances 0.5 m / 0.6 rad, so it follows the path's own direction; (3) the progress checker cancelled the turn 10 times (rocking back and forth moves < 0.5 m): now `PoseProgressChecker`, turning 0.5 rad counts too. Run m, 04:25: turn back reached, 69 s, 2 reversals. **Run n, 04:35, whole tour: all 8 goals, both controllers.** MPPI's turn back: 71 s, 2 reversals, wide; Pure Pursuit's: 74 s, tighter, but 10 reversals and 4 stops. Forward only (run o, 04:46): the U-turn still fails for both, as before the changes: so they did no harm, and only reversing fixes it. Left: MPPI drives on a few metres before it backs up (its own choice: it asks 0.5 m/s forward, easing to 0 over 13 s). Forward only, after the EKF fix: run s (06:16) all 8 goals, the U-turn as a loop; but run t (06:26) MPPI's loop met the north wall (2 of 4), Pure Pursuit looped twice at the first goal. So forward only it's still a coin toss; reversing is the reliable way. Tried 06:35, not kept: MPPI's `az_max` 1.0 → 0.4 (to match ego's steering rate): runs u, v made the U-turn once in two, the same as before. `docs/controllers.md` |
| 16 | `sim-check`'s scripted drive came up short once (4.43 m, needs 4.5), then 5.99 m twice | 👀 watch | probably the same stalls as 11 |
| 12 | Steer round the box in the lane: sensors in the planner's map (docs/plan.md M5) | ✅ done | the planner's costmap gets the radar and lidar layers (`plan_around:=true`, the default). Both controllers drove round the box 2.6-3.8 m clear. New `sim-check --plan-around` passes (3.9 m clear, no stop); the stop tests run with `plan_around:=false` and still pass (`sim-check --obstacle --lidar-obstacle`, `bench-check --obstacle`) |
| 13 | The global EKF's heading jumps at walking pace (docs/plan.md M9, T6.3) | ✅ 25 Oct, `wt/ekf-heading`: **a heading source, from the GPS receiver's velocity.** `gps_heading` (`aav_gps_heading`, `course.py` with 20 tests): the velocity's direction (Doppler, 0.05 m/s), turned round when reversing, the antenna's swing taken out, only above 0.3 m/s with the GPS's and the wheels' speeds agreeing; into `ekf_global` as imu1, yaw only. Gazebo's `gazebo_gps` now sends `/sensors/gps/fix_velocity` too (the truth + 0.05 m/s). `pixi run heading-check` (3.5 min, 0.5-1 m/s, forward and reversing, empty world), off → on: **5 Hz GPS, 4 runs each**: forward median 2.2-2.8° → 1.6-1.8°, worst 12.7-27.1° → 10.4-13.6°; reversing median 2.3-2.7° → 1.7-2.1°, worst 11.4-18.0° → 8.5-13.1°. **1 Hz (the receiver's factory rate), 3 / 2 runs**: forward median 2.8-5.4° → 2.7-2.9°, reversing 3.1-3.5° → 2.2-3.1°; worst about the same (10-21°). **Gyro biased 0.6°/s, 2 runs each**: median 3.3-4.3° → 1.5-1.9°, worst 12.3-22.3° → 9.4-16.9°. **Started 30° wrong, 2 runs each**: under 5° after 7.8-10.0 s → 5.4-6.1 s (3 s of it parked). Not taken: the magnetometer (raw counts, the motors), lidar matching (not installed; no north), `use_odometry_yaw` / `yaw_offset` / `differential` (no effect with a datum). Left, the hardware lead decides: 5 fixes a second (a receiver setting), RTK (fixes 25× better), two antennas (~0.4°, standing still too). On the car: `gps_driver:=ublox` for the velocity; check with the first bag |  needs a heading source; lower noise helped a little. 22 Oct, 05:30: at 0.5 m/s its heading wanders up to ±25°, forward and back alike (probe: the wheel speed and turn rates are right in reverse, so no sign bug). Over the night's runs, reversing: median 11° off, worst 76°; forward at the same speed: median 4°. **Cause of the reversing part, 05:35:** the antenna is 0.8 m ahead of the rear axle, and `navsat_transform` removes that offset with the EKF's own heading, so a heading error feeds back. Antenna over the rear axle (Gazebo, two runs each): reversing median 4.6-6.6° (was 7.5-12.8°), worst 26-31° (was 39-54°). ✅ **it is over the rear axle already** (Ben, 23 Oct); `vehicle.yaml` now says x = 0. **And in software, 05:55:** the global EKF's yaw process noise 0.06 → 0.01 (`a588a993`): heading median 2.6° off, worst 14° (was 6.2-6.7°, 39-54°); still right within 10 s of a start 29° wrong; better with a biased gyro too. Check with the real IMU's bias. Lot tour, run r (06:04): all 8 goals, worst heading per leg 7-16° (runs n, q: 14-76°) |
| 14 | Weigh the car, front and back, then with the rear lifted (M7, M7b) | ⬜ needs the team | `docs/measure-the-car.md` |
| 24 | **The car may reverse** (Ben, 22 Oct morning: "Yes the car can reverse"), safely | ✅ done | `52d58380`: `reversing:=true` on the car, but only where the lidar sees behind it (`vehicle.blind_behind`; else forward only, with a warning). collision_monitor's stop zone turns round with the command (`velocity_polygon`): reversing, a box 1 m behind the rear bumper. `sim-check --rear-obstacle`: stopped 0.75 m short of a post, then backed on to the goal. `sim-check --obstacle` still passes (0.46 m). Seen: the Gazebo car reversed at 0.57 m/s for ego's 0.5 (its speed loop overshoots) |
| 25 | The rear camera as a sensor behind the car | ✅ wired, ⬜ untested with a real picture | the detector reads more cameras with one model (`extra_images`); camera_obstacles publishes every camera's newest points together (`aav_detector/recent.py`), so the rear camera's don't flicker; collision_monitor stops for them (source `camera`) only with `camera_guard`, which the car sets when the rear camera watches behind: elsewhere (bench, Gazebo) no people detector feeds it, and a silent source would stop Nav2. It counts as seeing behind only when the machine's detector reads `/sensors/camera/rear/image_raw` and keeps people (`car.launch.py watches_behind`; the car's detector keeps stop signs only today). Not tried in Gazebo: it has no person to detect. Needs the cameras calibrated (Q22). Also fixed: `blind_behind` had counted camera_obstacles alone, which reads only the front camera |
| 26 | Check the nightly Gazebo job (task 23) after its first run, 03:00 on 23 Oct | ✅ 25 Oct | nightly `sim`: 21, 22 and 24 Oct passed; 23 Oct failed (no GL: fixed that day); 25 Oct failed on Gazebo's own segmentation fault at start (software rendering on the runner), and every node after it. sim-check now starts once more when Gazebo itself crashed while starting (`tools/gz_crash.py`; a second crash fails). Task 23 keeps watching |
| 27 | The speed loop overshoots its target by about 15% (seen: the Gazebo car reversed at 0.57 m/s for ego's 0.5) | ✅ 25 Oct, branch `wt/speed-loop`: a profile, **not the default** (Ben: "lets do them all"; the firmware keeps today's gains until the stands, TODO 52b) | The cause is the PI's zero (−ki/kp = −0.33, slower than the poles), not windup: clamping alone changes nothing. Fix: a **setpoint weight** (the P term sees b × target), which moves the zero and leaves the poles and margins alone. spine-core `SpeedLoopConfig` gets `setpoint_weight` (1.0) and `anti_windup` (`Bounds`), both today's loop to the bit; `SpeedLoopProfile` names `default` and **`smooth`** (kp 1.0, ki 0.25, b 0.75, integral limit 1.0, clamping). Gazebo's port has the same, and both check `firmware/spine/speed_loop_vectors.txt`. Model (`pixi run vehicle-model --speed-loop`, 6 runs per cell, the lag doubled and 110 ms late included): overshoot 17 → 0 % at K, 0 → 0 % at K/5 by drag (today 29 % short of 2 m/s), 17 → 8 % at K/5 by a weak throttle (today 29 % short), 15 → 1 % at 2K; settles (5 %) 13.7 → 5.7 s; phase margin ≥ 42°. Rejected: clamping alone, feed-forward (88 % at 2K), pole placement (22 % at K/5, 31°). Gazebo, `sim-check --speed-steps-only --speed-loop`, 3 runs each: today 15.9 / 15.2 / 10.1 % at −0.5 / 1 / 2 m/s, within 5 % after 11-13 s; `smooth` 0 / 0 / 0 %, 4.2-5.3 s (and the same with the spine in the loop). `sim-check --speed-loop smooth` (road world): all 10 checks pass; `bench-check` (default) and `esp-qemu-check` pass. Car day: [where to try it](docs/actuator-tests.md#the-speed-loops-profile-todo-27) (needs the spine's closed loop: today's ESP32 runs open loop) |
| 23 | `sim-check` in the nightly CI (PLAN T2.6) | 👀 watch | 25 Oct: a Gazebo crash at start, now retried once (task 26). Job `sim` in `test.yml`, Linux, `sim-check --obstacle`; first run 03:00 on 23 Oct **failed**: Gazebo crashed, as EGL had no driver (no GPU, no Mesa). Fixed 23 Oct: `tools/software_gl.sh` (Mesa's llvmpipe), and the scripted drive on the simulator's time (a slow simulator cut it short); passes on oirp-spss with 4 cores, Cyclone and Fast DDS. A failure now leaves annotations and the artifact `sim-logs`. On trial (`continue-on-error`): once it passes a few nights, drop `continue-on-error` |
| 22 | Jerk limit in ego (PLAN T5.3) | ✅ done | `max_jerk` 2.0 m/s³ (`ego.yaml`): speeding up, the push builds over 0.5 s and eases off before the set speed (0 to 1 m/s in 1.5 s, was 1 s). Braking is never softened: a stop starts at once. `sim-check --obstacle` passes (stopped 0.38 m before the box) |
| 21 | Gazebo check: a 10° steer gives the predicted radius within 5% (PLAN T2.3) | ✅ done | 22 Oct, 04:55: 9.05-9.14 m for 8.90 m (+2-3%); 20° and full lock also within 3%. `docs/simulation.md` |
| 15 | Gazebo check: a tight turn at `sim_speed:=3` slows to ego's curve speed | ✅ done | asked 3 m/s at full lock: ego allowed 2.40 m/s (the computed limit), the car held 2.37-2.43 m/s with 1.9-2.0 m/s² sideways (limit 2.0); straight, 3.1 m/s. Probe: `/tmp/aav-probe/curve_probe.py` (not in git) |

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

**Checked by Ben on his Mac, 15 Oct:** the raylib HUD opens (`hud-demo`), and with the turtle it shows speed, throttle and steering live while driving from the keyboard. The car kit is made (1.8 GB, `car-kit/`).

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
- [ ] **Flash them** on spare Pico 2s, then on the car: `pixi run wheel-uf2`,
      `sentinel-uf2` ([Rust on the hardware](docs/rust-firmware.md)) [[needs-you]].
- [ ] **Something to feed the sentinel:** its 2025-26 sender isn't in any repo. A small
      ROS node that writes the packet over I2C (`sentinel::Telemetry::to_packet` shows the
      bytes), once we know the display is on the car (the audit).

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
- [ ] **The simulated lidar on by default:** off for now, as on the Mac it made the full
      graph with YOLO flaky (DDS over loopback UDP). Try Fast DDS's shared memory, or a
      smaller cloud (`docs/simulation.md`).
- [x] **Gazebo's road world on `main`** (`wt/sim-full`): the camera, radar and a roof
      lidar, with the new topic names. *Done 15 Oct: `sim-check` passes on the Mac.*
- [ ] **Friday's audit:** the checklist at the end of `docs/car-photos.md`.

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
      - [x] the obstacle stop: a box as wide as the lane, and a re-arm after a latch, in
            `bench-check --obstacle`. *Passes on `main`, 11 Oct;*
      - [ ] N7, the weave: the bench's pretend GPS adds fresh noise to every fix, unlike a
            real receiver, whose error drifts slowly. Model it as a slow drift, then tune
            `ekf_global` for the real receiver (Q18);
      - [ ] collision_monitor flickers between stop and go at the zone's edge.
5. [x] **The board drivers in Rust, ready to try on a board:** the wheel encoder and display
      Picos (Embassy); the spine's Pico 2 program is written. *Done 15 Oct
      (`wt/rust-boards`): all three build; none flashed.*
6. [x] **The HUD in raylib** (Ben, 11 Oct: yes): one Rust program, raylib and rclrs, that is
      also the ROS node. The driving screen first (speed, mode, the e-stop button, the
      camera); the Flutter app stays until it matches. *Started 15 Oct (branch
      `wt/hud-raylib`): the driving screen, the e-stop, GPS and topic health run on a Mac
      (`pixi run hud-demo`, `pixi run hud`). Next: the bench, a Pi, the map page.*
      *25 Oct: it matched, so the Flutter app and its back end were archived
      (`archive/2026-27-flutter-hud/`, Q29); this package took the name `aav_hud`.*

## Ideas to keep

- [ ] **Your Mac as the operator station** (Ben, 11 Oct): the HUD on the Mac's screen,
      driving the car from the Mac, the Core ML AI driver on the Mac, maybe all of
      perception; the Orin doing the processing when you sit at the car. *Saved, not
      started. What runs where: `docs/history/conversation.md`, 11 October.* It waits on:
      - the car on Jazzy (Q24): a Jazzy Mac can't join the car's Humble;
      - the HUD on a Mac: raylib's runs there as it is; the Flutter app needs CocoaPods (archived 25 Oct);
      - the car's WiFi, measured: how late the pictures arrive, and how often it drops.
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
      - scored with `atob-score` against pure pursuit and MPPI; needs the measured car first.
      *Saved, not started (`docs/vehicle-model.md`, "After state space").*
- [ ] **Learned state-space models, Mamba-style** (Ben, 27 Oct): Mamba (and S4 before it)
      runs the same `x' = Ax + Bu` as `vehicle_model.py`, but learns `A`, `B`, `C` and lets
      them depend on the input. Its diagonal `A` is a bank of first-order lags (τ = -1/a),
      like our steering lag. Worth a closer look once hours of car logs exist: a learned
      model of what the linear one misses (drag, slope, the brake's play), checked against
      the fits from the actuator tests. *Saved, not started; needs car data first.*
- [ ] **micro-ROS, again** (Ben, 11 Oct): "lets rethink thinks later after we see the
      hardware". No micro-ROS for now (Tentative).

## Overnight (10-10 → 10-11): all the code, then Ackermann, Rust, native style

Ben, 21:45: "Lets try to get a fully functionin gthing going on." The order of work is
Decided ([Decisions](docs/decisions.md)):

1. **The high-level pieces**, by five helpers in parallel, each on its own branch:
   - [x] **The HUD app:** an e-stop button, Cancel, the camera page, the car's status, the
         OpenStreetMap credit. Flutter 3.35.7 in `vendor/` (`pixi run get-flutter`,
         `hud-test`). *Fixed: the app could never connect (Socket.IO tried polling
         first); now WebSocket only. The back end has 98 tests.* *Archived 25 Oct with the
         app (`archive/2026-27-flutter-hud/`): `get-flutter` and `hud-test` are gone.*
   - [x] **The HUD's place database:** `pixi run get-places` (51 MB, checksum pinned).
   - [x] **The car's launch:** perception as parts of `car.launch.py` (camera, detector,
         stop_signs, radar_clusters), the GPS driver (`gps_driver:=ublox` by default),
         `gps:=false` fixed. *21 launch tests.*
   - [x] **The lane finder:** `aav_lane_finder`, classic OpenCV, 0.78 ms a picture.
         *`bench-check --lane`: from 0.3 m off the centre line back to within 5 cm in
         about 6 s.*
   - [x] **The spine's board program:** `boards/pico2` (Embassy), the `mcp2515` CAN driver,
         a 100 ms hardware watchdog, the e-stop's spare contact, and `pixi run spine-uf2`,
         `spine-tools`, `spine-flash`. *Builds; never flashed.*
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

- [ ] A spare Pico 2 and a **3.3 V** MCP2515 CAN module (for example with an SN65HVD230),
      to try the spine on a desk. The common 5 V module (TJA1050) doesn't suit a Pico 2.
- [ ] The e-stop: a photo, and where its wires go (Q6).
- [ ] The brake after boot: `IDLE_BRAKE = 0.0` lets it go; the 2025-26 firmware kept it on.
- [ ] A fresh clone on oirp-spss before anyone works there.
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
      out of it. `pixi run vendor` downloads; `tools/test.sh` checks `COLCON_IGNORE`.
- [ ] **Prune the snapshots from `aav/aav/.git`** (149 MB down to about 1 MB)? *Ben to
      decide: it removes the app's undo points.*
- [x] **Move today's commits from `~/aav` into `aav/aav`** once the download finishes.
      *Done 16:13: the scp was still copying the server's Linux build output after six hours,
      so it was stopped and rsync brought the rest. The copy matches the server, Ben's
      uncommitted work is intact, and 55 commits fast-forwarded in. The Linux `.pixi`,
      `install` and Rust `target` went to `~/Code/aav/aav-linux-build-output`, not deleted.
      Built (25 packages, Ben's `aav_imu_yahboom` too) and tested green on the Mac.*
- [x] **Gazebo's window on the Mac:** Ben saw the screenshots (16:34).
- [ ] **Look at RViz and rqt_graph on the Mac**, and say what they show.
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
- [ ] **The small fixes** listed at the end of `docs/work.md`.

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
- [ ] **Team decisions:** the steering on an e-stop; the brake after boot; a brake on the RC
      gear switch (docs/firmware.md). *Since 11 Oct, one constant each in `spine-core`.*
- [ ] **Find `best.pt`**, the 2024-25 stop-sign model. (The 2025-26 lane finder isn't
      needed: we wrote our own, 11 Oct.)
      `best.pt` is probably on the team's Linux machine with the NVIDIA card (Ben, 10 Oct).
      No SSH to it today: later, once all the code is in place (Ben, 21:14).
- [x] **The HUD app's small changes** (docs/hud.md). *Done 11 Oct. Archived 25 Oct.*
- [ ] **Tune the speed loop on the car:** on the bench it trails a ramp by about a second.
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
- [ ] **Slim per-machine Pixi environments** (Orin, Motor Pi, laptop): at Phase 0.
- [ ] **An audit on the car's network** (Ben, 16:42: "at some point"): a read-only audit of
      the hardware. *Ready: the stand-ins stay on their own computer (`tools/ros.sh
      --local`, checked by `tools/test.sh`), so nothing pretend can reach the car.*

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

## Questions Ben asked

*Answered in the morning email and in the docs: `run.md`, `languages.md`, `mac.md`.*

- Is `aav_turtlesim` different from the regular turtlesim?
- What languages are used now, and should everything be Rust?
- Can it compile on a Mac without Homebrew?
- What does the claude.ai chat (fe09d020…) say? It can't be opened from here, so Ben
  needs to paste the key parts.

## For the team to decide

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

## Decided on the first night

- Teleop (keyboard, joystick, remote) is a **driver**: it feeds the mux, then ego, then
  the turtle or the car.
- **Bare metal: wait.** Only the spine firmware is bare metal.
- **Port to Jazzy as is first**, then consider rewriting in Rust.
- **Simulator:** start with a Gazebo test world.
- **Docs:** short, sectioned, HTML or PDF. Ben is dyslexic, so use tables and pictures
  over walls of text.

## Later

- Rewrite parts in Rust: now step 3 of the order of work, at the top.
- Measure the car and replace every guessed number in `vehicle.yaml`.
- Rotate the leaked secrets, and back up the Orin and Pis before any reflash.
- Answer Q1–Q34 with the team.
