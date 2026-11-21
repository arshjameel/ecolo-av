# Status: what works and what doesn't yet

Where things stand on 18 November 2026. Each row carries the date of its last run: a
row dated October hasn't been run again unless it says so. The newest work is in
[Since 16 November](#since-16-november); the older "Next" lists are in
[the status log](history/status-log.md).

| Word | Means |
|---|---|
| "Works" | it ran under ROS 2 Jazzy through Pixi: on oirp-spss (Linux), and since 10 October on Ben's Mac too |
| "On the bench" | [Bench mode](bench.md): the car's own software, with stand-ins for the hardware |
| "In Ravensim" | `pixi run sim`, the default since 8 Nov ([Ravensim](own-simulator.md)) |
| "In Gazebo" | every `sim` run before 8 Nov, and `simulator:=gazebo` until it was removed on 16 Nov ([Gazebo, as it was](history/gazebo-as-it-was.md)) |
| "Untested" | the code and its config exist, but it hasn't run yet |

The work left is in `TODO.md` and [The plan](plan.md).

## On the car

| When | What happened | Record |
|---|---|---|
| 16, 21 Oct | two read-only audits: every computer, port and connection | `docs/history/audit-2026-10-16-car.md`, `audit-2026-10-21-car.md` |
| 6 Nov | "drive as is", nothing of ours flashed or installed: the car didn't move (a fuse and a circuit board, Ben), then moved forward | `docs/history/audit-2026-11-06-car.md` |
| 13 Nov | the car drove "and stopped well" (Ben), with **only the localization update installed**. The full suite isn't on the car yet | `docs/history/messages-2026-11-13.md`: no recording, numbers or audit page in the repo yet |
| 18 Nov | the team upgraded the car (Ben: "People are upgrading the car now"). 0.4-rc1 was ready to install ([on the car](release-0.4-rc1.md)); the ESP32 flash and Motor Pi upgrade rehearsed the night before, the ESP32 in QEMU ([rehearsal](rehearsal-2026-11-18.md)). The Orin moves to JetPack 7.2 or newer: the hardware lead's call | no result of that day in the repo yet: until one is, our stack is [[untested]] on the car |

So every row below is [[untested]] on the car unless it says otherwise. What goes on the
car, in what order: [Rolling out](rollout.md).

## Each box in the picture

| Box | Implemented by | Status |
|---|---|---|
| Wheel speed, the 2025-26 encoder | `aav_legacy` `encoder_udp` (one wheel, into `/vehicle/odom`) | [[untested]] needs the car. Off in `motor_pi.yaml` since 23 Oct: the wheels are on CAN now |
| Wheel speed, the CAN wheel sensors (CAN v0.2.0) | `spine_bridge`: WHEEL_SPEED_L/R (0x200, 0x201) into `/vehicle/wheels` and `/vehicle/odom`'s speed; `wheel-core` (the KEFC nodes' logic) | [[works]] on the bench, against `spine_sim`'s wheel frames (15 Oct) |
| The Orin's CAN bus for sensors only (today's wiring, 23 Oct; the hardware lead confirms) | `spine_bridge role:=sensors`, `car.launch.py actuators:=uart` (the default): no CAN sender, no drive frames; `/vehicle/odom` from the wheels alone | [[works]] in tests (24 Oct, Mac): the Rust node fed wheel frames gave `/vehicle/odom` 1.1 m/s from 1.0 and 1.2, no `/can/tx`; the output mux's guard test. [[untested]] on the car: we haven't seen the Arduino's frames yet ([Today's wiring](can-plan.md#todays-wiring-23-october)) |
| IMU | `aav_imu_yahboom` (Rust) + `imu_filter_madgwick` | [[untested]] needs the device; gives the 2025-26 C++ driver's answers, bit for bit |
| GPS | `ublox_gps` (the stock u-blox driver); `nmea` and `ublox_dgnss` are options | [[untested]] in Pixi on every computer; needs the device. The default is Tentative (Q18) |
| Car model | `aav_description` | [[works]] the wheelbase (1.561 m) and the track are measured (Betsi, 1 Nov); the steering limit, the mass and the actuators are still guesses ([Car numbers](car_measurements.md)) |
| `ekf_local` | robot_localization | [[works]] in the turtle run and on the bench |
| `ekf_global`, `navsat_transform` | robot_localization | [[works]] in Gazebo: parked, it stays within 0.2 m. On the bench: 0.03-0.3 m from the truth |
| Joystick | `aav_teleop` | [[untested]] |
| Keyboard | teleop_twist_keyboard | [[works]] |
| Scripted | `aav_scripted` | [[works]] turtle and the bench |
| Drive to point: GPS A to B (15 Oct) | `aav_drive_to_point`: the `/drive_to_point` action, `pixi run drive-to-point` | [[works]] in Gazebo: `sim-check --drive-to-point` drove to 10, 30 and 60 m targets at any angle, at 1 and 3 m/s, every stop within 0.4 m of B by Gazebo's truth, braking at 1.4-1.6 m/s² for a plan of 1.5 (half `max_brake`); a cancel, the e-stop and a GPS fault each stopped it. [[untested]] on the bench and the car ([A to B](a-to-b.md)) |
| Plain English | `aav_language`: a drive tool server (MCP) and a local model (MLX) | [[works]] in Gazebo and the turtle, 15 Oct: "drive 10 m forward" drove 10.02 m in Gazebo, a 90° turn turned 87.6°. Asks before it moves; refuses the real car without `--car`. [[untested]] on the bench and the car ([Plain English](plain-english.md)) |
| Navigation (Nav2 until 17 Nov; ours since, [Nav2, as it was](history/nav2-as-it-was.md)) | `aav_navigation` | [[works]] in Gazebo: goals of 8 m and 9.5 m reached; on the bench, 40 m past a stop sign. Reversing (`reversing:=true`): on in the simulators and on the bench; on the car by default again since 1 Nov (Ben; off 26 Oct-1 Nov; until the wheels tell direction, H5, the speed's sign is the last command's), where the lidar watches behind it, with a stop zone behind the rear bumper ([[works]] in Gazebo: `sim-check --rear-obstacle` stopped 0.75 m short of a post); its settings are tested; [[works]] in Gazebo, 15 Oct: a goal 5 m behind the car succeeded twice, backing straight up, ego's command never past -0.5 m/s |
| Path followers (22 Oct) | Nav2's MPPI or Regulated Pure Pursuit until 17 Nov (deleted with Nav2: `aav_mppi_lag` since); `pixi run measure controllers` | [[works]] in Gazebo: in the lot both reached all 4 goals in the best run; MPPI reached the first goal cleanly in every run. `sim-check --obstacle` passes with MPPI. Open: stuck facing a wall after a wide turn (forward only); MPPI's forward U-turn ([Path followers](path-followers.md)). [[untested]] on the Orin |
| twist_mux | `aav_command_mux` | [[works]] |
| Sensor mux and output mux (23 Oct) | `aav_io_mux`: `source:=`, `target:=`, `stack.launch.py` | [[works]] in its tests: with Gazebo announcing SIM, the real backend (`spine_bridge`) sent no drive frame. In every profile: the turtle drove through it; `sim-check` and `bench-check` passed with it (23 Oct, on a busy Mac). [[untested]] on the car ([Sources and targets](sources-and-targets.md)) |
| ego | `aav_ego` | [[works]] |
| The car's linear model (24 Oct) | `aav_ego` `vehicle_model.py`, `pixi run python3 tools/vehicle_model.py`: transfer functions, state space, Bode margins with the delays; never drives the car | [[works]] in its 30 tests (textbook cases, the bicycle's Jacobian, discrete vs continuous). Every actuator number a guess until the actuator tests ([The car as a linear model](vehicle-model.md)) |
| ego's body model (22 Oct) | `aav_ego` `dynamics.py`: the mass (181 kg), `payload`, the centre of mass, the tyres | [[works]] in its tests: a load slows speeding up, tight turns slow the car. The numbers are guesses until the car is weighed ([Car numbers](car_measurements.md#weight-and-balance)) |
| turtlesim | `aav_turtlesim` | [[works]] |
| Gazebo | was `aav_gazebo` | **Removed 16 Nov** (Ben): Ravensim plays its part; its nodes and models moved to `aav_sim_models`. It [[works]]ed headless on Linux and a Mac until then; last scores in [Gazebo, as it was](history/gazebo-as-it-was.md) |
| The road world (Gazebo's until 16 Nov, Ravensim's since) | `aav_sim_models` `world:=road` | [[works]] on the Mac, 15 Oct, in Gazebo: the car's camera (10 Hz), radar stand-in (10 Hz) and a roof lidar (10 Hz) render headless; `pixi run sim-check`, `--obstacle`, `--lidar-obstacle`, `--faults`, `--spine` and `--perception` pass. On Linux with no GPU, through `tools/software_gl.sh`: [[works]] on oirp-spss, 23 Oct (`sim-check --obstacle`: the sensors and the stop; the nightly CI job uses it) ([Simulation](simulation.md)) |
| The lidar in navigation, in Gazebo (15 Oct) | `aav_lidar` `lidar_scan` -> the local costmap's `lidar_layer` and the stop zone (Nav2's `collision_monitor` then) | [[works]] with the radar off: it stopped the car 0.16-0.57 m before a person-sized post. **Under load it can be late:** beside other jobs on the Mac it stopped 0.13 m past the post (20 Oct). Watch the Orin's load on the car |
| The lidar in navigation, in bench mode (20 Oct) | `bench lidar:=true`, `mock_lidar` | [[works]] on oirp-spss: `bench-check --lidar` stopped 0.88 m before a box only the lidar saw. On a loaded Mac it failed (the spine latched). **Open:** the contract check missed the bench IMU's (or camera's) publisher in 3 of 3 runs, while the EKF got its data. Not yet explained |
| The lidar in navigation, on the car | the stock velodyne driver, on in `orin.yaml` and `all.yaml` (20 Oct) | [[works]] against a pretend VLP-16 (`tools/fake_vlp16.py`), 10 Hz. The roof lidar can't see a 1 m box inside the stop zone: the radar's job ([Perception](perception.md)). [[untested]] with the real lidar |
| Sensor watchdog (15 Oct) | `aav_safety` `sensor_watchdog`: a twist_mux lock, the HUD's banner | [[works]] in Gazebo: the faults on purpose (`pixi run sim-fault`, `sim-check --faults`) stop navigation and say why, 14 now with the HUD's heartbeat (23 Oct: a frozen or dead HUD stops Nav2 on the car, `watch_hud`; [HUD](hud.md#the-heartbeat)), frozen wheels (cross-checked with the IMU and GPS) and the watchdog's own death (a fail-safe lock) too. Frozen wheels: 25 of 25 since 24 Oct (MPPI had hidden them; the GPS check now fits 10 s: [frozen wheels and MPPI](simulation.md#frozen-wheels-and-mppi)). On the bench, `bench-check` passes with it, and it flagged a 2.5 s stall of the whole Mac there, then let go. [[untested]] on the car ([Simulation](simulation.md#sensor-failures-on-purpose)) |
| Real car | `aav_spine` | [[works]] on the bench, against `spine_sim`: arming, driving, both e-stops |
| The spine in the loop, in Gazebo (15 Oct) | `pixi run sim spine:=true`: `spine_bridge` → CAN topics → `spine_sim` (`car:=ravensim`, was `gazebo`; the spine's logic) → the actuator model | [[works]] on the Mac: `pixi run sim-check --spine` passes (graph, drive, EKF, a Nav2 goal; commands lost: full brake 195-209 ms and latched; both e-stops: brake within 12 ms, latched; disarmed: no motion). [[untested]] on Linux ([Simulation](simulation.md#the-spine-in-the-loop-spinetrue)) |
| Today's car | `aav_legacy` | [[untested]] on the car |
| Spine firmware logic | `firmware/spine/crates/spine-core` | [[works]] 143 tests (counted 13 Nov; 84 on 25 Oct), on the bench, and in Gazebo (`spine:=true`). Since 27 Oct with Arsh's state machine, boot self-test and hardware test mode; since 7 Nov his stop rules (T3.6) |
| Wheel node logic (15 Oct) | `firmware/spine/crates/wheel-core` | [[untested]] 9 tests; no board uses it: the hardware lead's Arduino sends WHEEL_SPEED (23 Oct, [Rust firmware](rust-firmware.md)) |
| Spine device drivers and glue | `jrk-g2`, `ad5311`, `pca9555`, `ibus`, `mcp2515`, `legacy-link`, `spine-board` | [[works]] about 240 tests on 25 Oct (`spine-board` 119 on 13 Nov, was 79), against mocks; `legacy-link` also against the 2025-26 C++ itself |
| ESP32 board program (Rust) | `firmware/spine/boards/esp32-umod` | [[untested]] builds with Espressif's Rust (`pixi run esp tools`), both links; never flashed. Merged 15 Oct; the C++ stays on the car until it is proven. In QEMU (24 Oct): brakes 111-120 ms after the Motor Pi's frames stop, and latches (`pixi run esp qemu-check`); again 3 Nov: 112-119 ms, 6 of 6 cycles. Since 7 Nov: status lines, `esp first-check` and `esp walk` for the first flash (built, [[untested]] on a board). [First flash on the bench](esp32-bench.md) |
| Spine board program | `firmware/spine/boards/pico2` | [[untested]] builds for a Pico 2, and its UF2 file checks out; never flashed. The car's chip is still Q10–Q13 |
| Wheel encoder, in Rust (15 Oct) | `firmware/spine/boards/wheel-encoder-pico2`, crate `wheel-encoder` | [[untested]] builds, its UF2 checks out, 12 tests; never flashed. Same packet as the C++ (shared examples). The car's Pico 2 still runs the C++ |
| Status display, in Rust (15 Oct) | `firmware/spine/boards/sentinel-pico2`, crates `sentinel`, `hd44780` | [[untested]] builds, its UF2 checks out, 45 tests; never flashed. Nothing sends it packets yet |
| Bench mode | `aav_spine_sim`, `aav_bench` | [[works]] `pixi run bench-check`, plus `--perception` and `--lane` |
| Camera | `usb_cam`, the `camera` part of `car.launch.py` | [[untested]] Linux only, off until the camera is fitted; the models are Q22 |
| Front cameras, streamed | the camera Pi's MediaMTX -> `camera_stream_front_left`, `_front_right` (the `camera_stream` part, `orin.yaml`), the detector inside; the sensor watchdog watches their detections | [[works]] on a Mac (`pixi run -e ml measure stream --gst`: 26 ms to detections; the watchdog OK, then STALE when the stream stops). [[untested]] on the Orin and the Pi: skipped there until RF-DETR's TensorRT engine is built ([camera pipeline](camera-pipeline.md)) |
| Stop signs | `aav_detector`: RF-DETR (default since 20 Oct) or YOLO, `detector:=` | [[works]] on the bench with both stock models: RF-DETR's lock came on at 20.7 m, the sign at 25 m (20 Oct). `sim-check --perception` in Gazebo with RF-DETR (24 Oct): first seen at x = 5.9 m, the sign at 25 m; the car stopped with its bumper 4.4 m before it. [[untested]] on the Orin: its TensorRT engine is written, not built |
| Lane keeping | `aav_lane_finder` + `aav_lane_geometry` | [[works]] on the bench: from 0.3 m off the centre line back to within 5 cm in about 6 s |
| Radar clustering | `aav_radar_clusters` (Rust) | [[works]] on the bench and in Gazebo's road world (15 Oct); gives the 2025-26 C++ node's answers, bit for bit |
| Obstacles in navigation | the radar in the local costmap; the stop zone (Nav2's `collision_monitor`, `aav_stop_zone` since 17 Nov); `camera_obstacles` | [[works]] on the bench: the stop zone stops the car 0.3-0.4 m before a box as wide as the lane. Open: the car weaves up to 1.5 m off its line (N7), so it can pass beside a narrow box; the stop zone flickers at its edge. The camera layer is off until the cameras are calibrated (Q22) |
| HUD back end and app (Flask, Flutter) | `archive/2026-27-flutter-hud/` | archived 25 Oct: the raylib HUD replaced it (Q29). Not built, not tested |
| HUD and map window in Rust (raylib) | `aav_hud` | [[works]] on a Mac: `--demo`, and as a node fed by `ros2 topic pub`; its e-stop reaches a subscriber as 5 trues. Its heartbeat (23 Oct): the real window beat at 9.3 Hz, and freezing or closing it made the watchdog say `hud STALE` in 1 s. Since 24 Oct a dropout after its first 5 s is latched (`hud LOST`, `pixi run clear watchdog`), and a GO / STOP panel says whether the car may move (logic tested; not yet seen on screen). [[works]] next to bench mode too. [[untested]] on a Pi and on the car's screen |
| CARLA | `aav_carla` | [[untested]] adapter built and tested without a server; needs the NVIDIA computer ([CARLA](carla.md)) |
| Heading from the GPS velocity (25 Oct) | `aav_gps_heading_rs` `gps_heading` -> `ekf_global` (yaw only), `pixi run tools/ros.sh --local python3 tools/heading_check.py` | [[works]] in Gazebo: at walking pace the heading's median error fell from 2.2-2.8° to 1.6-2.1°, forward and reversing. [[untested]] on the car: needs `gps_driver:=ublox` for the velocity |
| RC via ROS (24 Oct) | `aav_rc` `rc_driver`: `/vehicle/rc` -> `/drivers/rc/cmd_vel` (twist_mux 95); the ESP32's uplink (`--features uplink`) | [[works]] in QEMU (all six RC steps) and in a ROS test. Off until the hardware lead answers H9 and H10. [[untested]] on a real ESP32 and the car |
| The ESP32's uplink (24 Oct) | `legacy_uart_bridge` `uplink: true` -> `/vehicle/status`, `/vehicle/steering`, `/vehicle/brake`, `/vehicle/rc` | [[works]] in QEMU: 50-51 Hz, 0 frames lost. Off by default. [[untested]] on a real ESP32 and Pi ([ESP32 bench](esp32-bench.md)) |
| Street routes (16 Oct) | `aav_routes` (Valhalla, offline) | [[works]] in Gazebo: a street route drove to 0.5 m of its end through Nav2's waypoint follower then ([Street routes](routes.md)). [[untested]] on the car |
| Ghost obstacles (16 Oct) | `aav_ghost`, `pixi run ghost` | [[works]] in Gazebo: `sim-check --ghost` stopped 0.21 m before it ([Ghost obstacles](ghost-obstacles.md)). [[untested]] on the car |
| Actuator tests (15 Oct) | `aav_actuator_tests`, `pixi run actuator-test` | [[works]] in Gazebo. [[untested]] on stands ([Actuator tests](actuator-tests.md)) |
| The speed loop's `smooth` profile (25 Oct) | spine-core `SpeedLoopProfile`, `speed_loop:=smooth` in sim and bench | [[works]] in the model and Gazebo: overshoot 15% -> 0%. Not the default. [[untested]] on stands |
| Watch from a laptop (24 Oct) | `foxglove.launch.py`, three layouts in `tools/foxglove/` | [[works]] live with the road sim and the bench: every layout topic served. The layouts not yet opened in the app by hand. [[untested]] at the car ([Foxglove](foxglove.md)) |
| Car-day kit (24 Oct) | `pixi run car-day-build`, `pixi run car-day-check` | [[works]] on Ben's Mac: both ESP32 builds, QEMU passes, the check's 35 tests ([Car day](car-day.md)) |
| Lidar | a Velodyne VLP-16 on the roof (the 16 Oct audit: 192.168.1.201, sending to the Jetson). The stock `velodyne` driver (`aav_sensors` `lidar:=true`, the `lidar` part in `car.launch.py`) -> `/sensors/lidar/points`; a generic 16-beam one in Gazebo | [[works]] against `tools/fake_vlp16.py`: a post 6 m ahead came out at 5.80-6.00 m, Nav2's scan 6.75 m from `base_link`, 10 Hz (20 Oct). [[untested]] with the real lidar |

## Added since 25 October

One row per package or feature; the numbers and how they were measured are on the linked
page. All of these are trials or sim-only unless the row says "default".

| Box | Implemented by | Status |
|---|---|---|
| **Localization** | | |
| NDT on a saved lidar map (29 Oct) | `aav_ndt` (Rust), `ndt:=true`; into the global EKF with `ndt_ekf:=true` | [[works]] in Gazebo: 1-3 cm off on the lot tour (29 Oct); the EKF 17 → 1.6 cm with it (30 Oct). Off: it needs the lot's map (`pixi run ndt-map drive`, not in git). [[untested]] on a real lot ([Trials](trials.md)) |
| AMCL (29 Oct; in Rust 18 Nov) | ours, `aav_amcl_rs` (stock `nav2_amcl` removed 20 Nov, TODO 232), `amcl:=true` | [[works]] in Gazebo, 29 Oct: 6-11 cm off once tuned; ours in Ravensim, 18 Nov: a tie with stock, 3 runs each. Scored only, never steers |
| KISS-ICP lidar odometry (30 Oct) | `aav_kiss_icp` (stock, MIT), `kiss_icp:=true`, `kiss_ekf:=true` | [[works]] in Gazebo, 30 Oct, one run: 0.04 m drift per 100 m. Into `ekf_local` (1 Nov): no clear win yet. Off |
| Localization health checks (1 Nov, Betsi) | `aav_localization_health_rs` | [[works]] in its tests; `pixi run check` prints its line. It only warns. Thresholds tuned in Gazebo only |
| **Perception** | | |
| The lidar as cubes (30 Oct) | `aav_voxels`, `voxels:=true` | [[works]] in Gazebo, 30 Oct: stopped 0.44 m before the post. **Open:** 3 Nov, 1 of 3 runs drove 1.30 m into it (TODO 93). Off |
| The lane finder as a pose sensor (15 Nov) | `aav_lane_pose`, `lane_pose:=true` | [[works]] in Ravensim: the global EKF's sideways error 9.5 → 2 cm (road world), 9.9 → 5.4 cm (campus). No effect on the wiggle yet (little paint on that tour). Off; not wired on the car ([Trials](trials.md)) |
| Things: the lidar's cubes with an id, a speed, a memory (14 Nov) | `aav_things_rs`, `things:=true` | [[works]] in Ravensim, 15 Nov: the behaviour slows for a walker crossing, 5 of 5 (0.5-1.0 m clear). Names from the camera (`things_names:=true`) and the stop sign's distance from the lidar: built, not yet run with the detector. Off |
| Lane drives, scored (8 Nov; retired 19 Nov with the lane follower, TODO 264) | `pixi run measure lane` (gone) | [[works]] in Ravensim: Library Road, 171 m at 1.4 m/s, mean error 4 cm. In Gazebo on the campus the lane was lost in 68 % of pictures (not diagnosed) ([Perception](history/lane-followers.md#earlier-lane-drives-8-nov-the-2025-26-pid)) |
| **Navigation** | | |
| The State Lattice planner: the default (1 Nov) | Nav2's until 17 Nov (deleted, with `hybrid`, `smac2d`, `navfn`, `theta_star`); ours since, `aav_planner` | [[works]] in Gazebo, 30 Oct, reversing, 3-4 runs: every leg, 184 s a tour (Hybrid-A* 213 s) ([Switches](switches.md)) |
| Trial path followers (29 Oct) | `aav_track`, `tracker:=pure_pursuit`, `stanley`, `lqr`, `mpc`, `mpc_full`, `mpc_lag` | [[works]] in Gazebo, 29 Oct: pure pursuit, Stanley and MPC 12 of 12 legs. `mpc_lag` in Ravensim, 8 Nov: 6 of 12. Forward only; none reads the costmap |
| MPPI that knows the lag (7 Nov) | `aav_mppi_lag` (Rust), `tracker:=mppi_lag` | [[works]] in Ravensim, 8 Nov, 3 runs: 11 of 12 legs (Nav2's MPPI 8). **Open:** overshoots the campus's first goal at 2.5 m/s; twice Nav2's CPU. [[untested]] on the Orin ([page](mppi-lag.md)) |
| Our own costmap (8 Nov) | `aav_costmap` (Rust), `costmap:=own` | [[works]] in Ravensim: 100 % of 40,000 cells as Nav2's; the lidar obstacle 3 of 3 ([page](own-costmap.md)) |
| Our own planner (8 Nov) | `aav_planner` (Rust), `planner:=own` | [[works]] in Ravensim: test lot 12 of 12 legs, as Nav2's lattice. On long campus plans slower: 168-195 ms against 138-151 (TODO 152) ([page](own-planner.md)) |
| Our own navigator (8 Nov) | `aav_navigator` (Rust), `navigator:=own` | [[works]] in Ravensim, forward only: test lot 7 of 12 legs (Nav2 8); campus 4 of 6 (Nav2 2); `sim-check` all ours 13 of 13 ([page](own-navigator.md)) |
| Our own velocity smoother (8 Nov) | `aav_smoother` (Rust), `smoother:=own` | [[works]] in Ravensim, 3 runs: 10 of 12 legs both; ego clipped 0-0.1 % of the steering (Nav2's: 9-15 %) ([page](own-smoother.md)) |
| Diffusion planner (2 Nov; 0.3: 7-8 Nov) | `aav_diffusion_planner`, `planner:=diffusion`; on the car only with `diffusion_trial:=true`, 0.5 m/s | [[works]] in Ravensim, 8 Nov, 3 runs: every leg, 88 % of plans from the model. [[untested]] on the Orin (TensorRT) and the car |
| The behaviour planner (7 Nov; Rust the default 16 Nov) | `aav_behaviour_rs` (the Python removed 20 Nov), `behaviour:=true`, on in `stack:=ours` | [[works]] in its tests (every transition); in Ravensim since (below). Gazebo, 8 Nov: the obstacle stop 2 of 4 (the other 2 went round); the goal reached. **Open:** it never entered APPROACH for the stop sign ([Stopping](stopping.md#2-the-behaviour-planner)) |
| Directions on the campus (7 Nov) | `aav_routes` `router:=campus`, `pixi run campus-route` | [[works]] without ROS. P4 to the Fieldhouse (453 m), 8 Nov: Nav2's waypoint follower 0 of 2, our navigator 1 of 2 ([Routes](routes.md#on-the-campus)) |
| Map chunks, NDT by chunk, Lanelet2 in Rust (15 Nov) | `aav_description` `map-chunks`, `lanelet-map`; `ndt_chunks:=`; `global_window:=` | [[works]] in their tests; the campus cut in 1.5 s; `global_window:=192` passes `sim-check`. Scored 15 Nov: NDT by chunk as good as the whole map on the campus, so the default there with `ndt:=true`; `global_window:=192` reached 2 of 6 campus legs (the long roads leave the window), a challenger. 16 Nov: our global costmap from chunks (`costmap_chunks:=auto`, on in `stack:=ours`): 20 of 20 legs on 3 tours, as the whole map, a third of the memory ([own costmap](own-costmap.md#from-map-chunks-on-every-core-16-nov-todo-175-195)) |
| **Control and safety** | | |
| One car model, ego's lag lead and trace (7 Nov) | `aav_description` `car_model.py` and the Rust `car-model`; `aav_ego` `lag_lead.py`, `/ego/trace`, `/ego/stop_point` | [[works]] in its tests: Python and Rust match the same vectors. The lead is on in simulators only. The lags are guesses until the actuator tests ([Car model](car-model.md)) |
| One stopping law, and the stop-point check (7 Nov) | `aav_description` `stop_law.py`; `sim-check --stop-point` | [[works]] in Gazebo, 8 Nov: stops within 0.25 m of `/ego/stop_point` at 0.5, 0.75 and 1 m/s ([Stopping](stopping.md)) |
| The adaptive brake (8 Nov) | `aav_description` `adaptive_brake.py`, `adaptive_brake:=true` | [[works]] in Ravensim, 5 of 5: with 150 kg aboard, 0.87 m past the empty car's stop without it, 0.21 m with it. Refused with today's ESP32; the brake's limits are placeholders (car day A7) |
| Geofence (29 Oct) | `aav_safety` `geofence`, a twist_mux lock (51) | [[works]] on the bench, 29 Oct: a scripted drive at 1 m/s stopped 1.06 m after the lock closed, inside a 12 m fence. Not yet run in a simulator with a fence you can reach ([Geofence](geofence.md)) |
| Our stop zone in Rust (15 Nov; the stop zone since 17 Nov) | `aav_stop_zone`, `stop_zone:=` | [[works]] in Ravensim: in shadow, 3 rounds of every obstacle check, Nav2 stopped 24 times and ours on the same command each time, never later; then it took the lock ([the swap](safety-case.md#the-stop-zones-swap-17-nov)). [[untested]] on the car |
| Stop hold (29 Oct); the no-timeout lock (26 Oct) | `aav_safety` `stop_hold` (21); `legacy_uart_bridge`'s lock (60) | [[works]] in their tests; `bench-check --zone-edge` |
| The joystick-alive check (2 Nov) | `aav_safety` `joy_alive` | [[works]] on the bench, 2 Nov: `bench-check` 11 of 11. [[untested]] with the real pad |
| The speed loop in the UART bridge (29 Oct) | `legacy_uart_bridge` `speed_loop.mode: pi`; `sim esp32:=true` | [[works]] in Gazebo through the 2025-26 firmware's model. Never `pi` on the car before a driven-wheel speed exists (TODO 101) |
| **Simulation** | | |
| Ravensim: **the default** since 8 Nov, **the only simulator** since 16 Nov | `aav_simulator` (Rust) and its cameras, `aav_sim_cameras` (Bevy; a crate, not a ROS package) | [[works]] on a Mac, 8 Nov: every `sim-check` leg; `--perception`, `--faults` and `--drive-to-point-only` 3 of 3. Without a GPU its cameras draw in software (`tools/software_gl.sh`, 15 Nov): the road world at 27 rounds a second on a Mac; the campus needs a GPU; [[untested]] on Linux. Since 16 Nov the clock waits for each picture in software (camera lockstep), so the cameras keep 10 Hz in sim time ([cameras](own-simulator-cameras.md#camera-lockstep-16-nov)). **Open:** one stop-sign miss in 8 ([page](own-simulator.md)) |
| The campus world (5-8 Nov) | `pixi run campus`, `world:=campus` | [[works]] in Gazebo and ours. Not checked against the real campus ([Campus](campus.md)) |
| Drop-in check (5 Nov) | `pixi run drop-in` | [[works]] in Ravensim, 8 Nov: every stream passes, each camera at 10 Hz. Against real sensors: not yet |
| **Screens and tools** | | |
| Hardware panel, with its steering-calibration and graphs pages (29 Oct - 7 Nov) | `aav_hud` `hw_panel`, `pixi run hw-panel` | [[works]] beside Gazebo and the bench, and with `--demo`. [[untested]] on the car ([HUD](hud.md#the-hardware-panel)) |
| Debug screen (29 Oct) | `aav_hud` `debug_hud`, `pixi run demo --debug` | [[works]] with `--demo`; the cubes and health panels [[untested]] live (1 Nov) |
| Developer window (5 Nov) | `ros/hud/aav_dev_window` (Bevy; a crate, not a ROS package), `pixi run dev-window` | [[works]] live on Gazebo, with goals through its gates (5 Nov). 15 Nov: the things with their names (key K), the lanes and the chunks: built, never looked at on a screen. Left: the algorithm list, SWAP |
| The Drive screen's algorithms table (3 Nov) | `aav_run_info` -> `/system/algorithms`; the Lichtblick cockpit | [[works]] live in Gazebo's test lot, 3 Nov. The bench doesn't start it |
| Switch picker (29 Oct) | `pixi run pick` | [[works]] 3 Nov: 23 of 23 switches each drove a leg in Gazebo |
| Car setup, the rollout, "Know the car" (1-8 Nov) | `tools/car_setup.sh <role>`, [Rolling out](rollout.md), [Know the car](know-the-car.md) | [[untested]] on a car computer |
| Radar and battery drivers (ported 9 Oct) | `aav_radar_ti`, `aav_battery_victron` | [[untested]] need the devices |

## Since 16 November

Ravensim on a Mac unless the row says so; none of it on the car yet.

| Box | Implemented by | Status |
|---|---|---|
| **Our own stack, the only one** (Nav2 deleted 17 Nov, Gazebo 16 Nov) | `stack:=ours`: our costmap, planner, navigator, `mppi_lag`, smoother, stop zone, map server | [[works]] in Ravensim: the league's final, 16 Nov, 30 of 30 legs on three tours ([League scores](league-results-2026-11-16.md)) |
| Rust by default (16-18 Nov) | ego, behaviour, the watchdog, localization_health, the muxes, the Motor Pi's bridge and RC, `lidar_scan`, things, `joy_alive`, the stop-sign watcher, the lane finder, the point driver, `arm_when_open`, `camera_stream`, AMCL; `*_impl:=python` falls back | [[works]] against the Python's answers (shared vectors) and in Ravensim; [[untested]] on the car ([Languages](languages.md)) |
| AMCL in Rust (18 Nov) | `aav_amcl_rs`, `amcl_impl:=rust` (stock removed 20 Nov) | [[works]] the lot, 3 runs each: a tie with stock (5-9 cm a leg). `nav2_amcl` still in `pixi.toml` (TODO 232) |
| The lane's centre as the path (16 Nov) | `path:=lane`, ours' default | [[works]] University Drive, 3 runs: 0 % over the centre line ([Our planner](own-planner.md)) |
| Stop for anything moving (17 Nov) | the behaviour planner, `moving_*` parameters | [[works]] `sim-check --movers --moving`: every mover held the car, the parked box never ([Stopping](stopping.md)) |
| Kerbs (16 Nov) | `kerbs:=true`: the Rust voxels' step finder, the costmap's `kerb_layer` | [[works]] a 0.15 m kerb, at 1 m/s; a 0.10 m one never seen ([Kerbs](kerbs.md)). [[untested]] on a real kerb |
| The pedal waits to change direction (17-18 Nov) | ego's `pedal_shift_hold` | [[works]] in tests and the rehearsal; [[untested]] on the car |
| The speed loop and its model apart (18 Nov) | `speed_loop_model:=` | [[works]]: `firm` car, `smooth` model stops best, sways a little more; not the default (TODO 163) |
| The navigator's follow call (18 Nov) | `aav_navigator` `call.rs` | [[works]] here, with software GL and load; CI's nightly to be counted (TODO 240) |
| The watchdog reads our nodes' status (18 Nov) | `kind: status`, warn only | [[works]] in tests, Python and Rust tick by tick (TODO 171) |
| The detector in Rust, through ONNX (17 Nov) | `aav_detector_rs`, `detector_impl:=rust`, a challenger | [[works]] `sim-check --perception` on Core ML; TensorRT on the Orin [[untested]] ([ONNX](onnx.md)) |
| A prettier Ravensim (17 Nov) | `graphics:=pretty` | [[works]]: the scored checks pass as before |

## Builds and tests

| Check | Result |
|---|---|
| `colcon build`, Jazzy | [[works]] on Linux and the Mac: 37 packages on 25 Oct; 51 on 13 Nov; 72 on 18 Nov (counted by `package.xml`), plus two Bevy crates outside colcon |
| `pixi run test`: build, tests, lint, `colcon test` | [[works]] 25 Oct: 1110 tests, 0 failures, 4 skipped, 37 packages. 3 Nov (0.2): 1575 tests, 0 failures. 15 Nov: 1934 tests, 0 failures, 6 skipped, 54 packages. CI runs it on Linux (x86_64 and 64-bit ARM) for every pull request and every push to `main` |
| Live graph vs `docs/interfaces.yaml`: turtle, sim, road, replay and bench | [[works]] 0 failures each (sim and road again on 15 Oct, after the topic renaming) |
| `pixi run bench-check` (`--perception`, `--lane`, `--obstacle`) | [[works]] on `main`, 11 Oct: the default checks, the stop sign, lane keeping and the obstacle |
| Sim, headless: drive commands (the keyboard's topic) and goals | [[works]] (Ravensim) |
| Replay of a recorded drive | [[works]] the EKFs match the recording within 0.03 m |
| Car, with no hardware attached | comes up; only the missing sensors (IMU, GPS) fail the graph check, as expected. Checked again on 11 Oct, with the perception parts |
| The same `pixi.toml` on macOS | [[works]] on an Apple Silicon Mac (macOS 27), 10 Oct: build 18 s, `pixi run test` all green after one fix (GNU `coreutils`, see [Your Mac](mac.md)) |
| On the Mac: the turtle, Ravensim headless, a goal, the docs, `pixi run doctor`, recording a drive | [[works]] |
| On the Mac: Ravensim's developer window | [[untested]] by a person: `pixi run ravensim` opens it beside the sim (17 Nov) |
| On the Mac: RViz, rqt_graph | [[untested]] they open; nobody has looked at them yet |
| The same `pixi.toml` on 64-bit ARM Linux | [[works]] in CI on Ubuntu 24.04 ARM (25 Oct): `pixi run test`, and the car kit installed with no network. [[untested]] on the car's own computers (Ubuntu 22.04, JetPack 6) |

## Is it all on Jazzy?

Yes. Everything builds against Jazzy, and the turtle, sim and replay profiles run. Every
configuration key was checked against what the running Jazzy nodes accept. The fixes were
small: clean shutdowns, Nav2 on simulated time (Nav2 went on 17 Nov), and the simulated GPS noise. The details
are in `docs/history/audit-2026-10-10-*.md`.

## Next

From the top of `TODO.md` (18 Nov) and [the testing plan](testing-plan.md).

| # | Next | TODO |
|---|---|---|
| 1 | ~~`nav2_amcl` out of `pixi.toml`~~ done 20 Nov (TODO 232); the one-option switches kept as hooks | 231, 232 |
| 2 | The sim car's 14 % overshoot: which speed loop is the default | 163 |
| 3 | Count CI's nightly `sim` job after the navigator's fix; then drop `continue-on-error` | 240, 23 |
| 4 | Ours against stock, 3 runs each, the lot and the campus | 167 |
| 5 | Our planner faster; reverse in a narrow street | 152, 174 |
| 6 | The testing plan's phase A: `pixi run car-test --stands`; the debug screen | 246, 247 |
| waits | The car (the hardware lead; being upgraded 18 Nov): the seven blockers, one drive recorded and scored | 157, 158, 173 |
