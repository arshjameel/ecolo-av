# Status: what works and what doesn't yet

Where things stand on 25 October 2026. "Works" means it ran under ROS 2
Jazzy through Pixi: on oirp-spss (Linux), and since 10 October on Ben's Mac too. "On the
bench" means [Bench mode](bench.md): the car's own software, with stand-ins for the
hardware. "Untested" means the
code and its config exist, but it hasn't run yet. The plan and the work left are in
[Work](work.md).

## Each box in the picture

| Box | Implemented by | Status |
|---|---|---|
| Wheel speed, the 2025-26 encoder | `aav_legacy` `encoder_udp` (one wheel, into `/vehicle/odom`) | [[untested]] needs the car. Off in `motor_pi.yaml` since 23 Oct: the wheels are on CAN now |
| Wheel speed, the CAN wheel sensors (CAN v0.2.0) | `spine_bridge`: WHEEL_SPEED_L/R (0x200, 0x201) into `/vehicle/wheels` and `/vehicle/odom`'s speed; `wheel-core` (the KEFC nodes' logic) | [[works]] on the bench, against `spine_sim`'s wheel frames (15 Oct) |
| The Orin's CAN bus for sensors only (today's wiring, 23 Oct; the hardware lead confirms) | `spine_bridge role:=sensors`, `car.launch.py actuators:=uart` (the default): no CAN sender, no drive frames; `/vehicle/odom` from the wheels alone | [[works]] in tests (24 Oct, Mac): the Rust node fed wheel frames gave `/vehicle/odom` 1.1 m/s from 1.0 and 1.2, no `/can/tx`; the output mux's guard test. [[untested]] on the car: we haven't seen the Arduino's frames yet ([Today's wiring](can-plan.md#todays-wiring-23-october)) |
| IMU | `aav_imu_yahboom` (Rust) + `imu_filter_madgwick` | [[untested]] needs the device; gives the 2025-26 C++ driver's answers, bit for bit |
| GPS | `ublox_gps` (the stock u-blox driver); `nmea` and `ublox_dgnss` are options | [[untested]] in Pixi on every computer; needs the device. The default is Tentative (Q18) |
| Car model | `aav_description` | [[works]] numbers are guesses |
| `ekf_local` | robot_localization | [[works]] in the turtle run and on the bench |
| `ekf_global`, `navsat_transform` | robot_localization | [[works]] in Gazebo: parked, it stays within 0.2 m. On the bench: 0.03-0.3 m from the truth |
| Joystick | `aav_teleop` | [[untested]] |
| Keyboard | teleop_twist_keyboard | [[works]] |
| Scripted | `aav_scripted` | [[works]] turtle and the bench |
| Drive to point: GPS A to B (15 Oct) | `aav_drive_to_point`: the `/drive_to_point` action, `pixi run drive-to-point` | [[works]] in Gazebo: `sim-check --drive-to-point` drove to 10, 30 and 60 m targets at any angle, at 1 and 3 m/s, every stop within 0.4 m of B by Gazebo's truth, braking at 1.4-1.6 m/s² for a plan of 1.5 (half `max_brake`); a cancel, the e-stop and a GPS fault each stopped it. [[untested]] on the bench and the car ([A to B](a-to-b.md)) |
| Plain English | `aav_language`: a drive tool server (MCP) and a local model (MLX) | [[works]] in Gazebo and the turtle, 15 Oct: "drive 10 m forward" drove 10.02 m in Gazebo, a 90° turn turned 87.6°. Asks before it moves; refuses the real car without `--car`. [[untested]] on the bench and the car ([Plain English](plain-english.md)) |
| Nav2 | `aav_navigation` | [[works]] in Gazebo: goals of 8 m and 9.5 m reached; on the bench, 40 m past a stop sign. Reversing (`reversing:=true`): on in Gazebo and on the bench; on the car only with `reversing:=true` (off by default since 26 Oct: the wheels can't tell direction yet, H5), where the lidar watches behind it, with a stop zone behind the rear bumper ([[works]] in Gazebo: `sim-check --rear-obstacle` stopped 0.75 m short of a post); its settings are tested; [[works]] in Gazebo, 15 Oct: a goal 5 m behind the car succeeded twice, backing straight up, ego's command never past -0.5 m/s |
| Path followers (22 Oct) | Nav2's MPPI (default) or Regulated Pure Pursuit, `controller:=rpp`; `pixi run controller-compare` | [[works]] in Gazebo: in the lot both reached all 4 goals in the best run; MPPI reached the first goal cleanly in every run. `sim-check --obstacle` passes with MPPI. Open: stuck facing a wall after a wide turn (forward only); MPPI's forward U-turn ([Path followers](path-followers.md)). [[untested]] on the Orin |
| twist_mux | `aav_command_mux` | [[works]] |
| Sensor mux and output mux (23 Oct) | `aav_io_mux`: `source:=`, `target:=`, `stack.launch.py` | [[works]] in its tests: with Gazebo announcing SIM, the real backend (`spine_bridge`) sent no drive frame. In every profile: the turtle drove through it; `sim-check` and `bench-check` passed with it (23 Oct, on a busy Mac). [[untested]] on the car ([Sources and targets](sources-and-targets.md)) |
| ego | `aav_ego` | [[works]] |
| The car's linear model (24 Oct) | `aav_ego` `vehicle_model.py`, `pixi run vehicle-model`: transfer functions, state space, Bode margins with the delays; never drives the car | [[works]] in its 30 tests (textbook cases, the bicycle's Jacobian, discrete vs continuous). Every actuator number a guess until the actuator tests ([The car as a linear model](vehicle-model.md)) |
| ego's body model (22 Oct) | `aav_ego` `dynamics.py`: the mass (181 kg), `payload`, the centre of mass, the tyres | [[works]] in its tests: a load slows speeding up, tight turns slow the car. The numbers are guesses until the car is weighed ([Car numbers](car_measurements.md#weight-and-balance)) |
| turtlesim | `aav_turtlesim` | [[works]] |
| Gazebo | `aav_gazebo` | [[works]] headless on Linux and a Mac; its window on a Mac, with the car driving (10 Oct) |
| Gazebo's road world | `aav_gazebo` `world:=road` | [[works]] on the Mac, 15 Oct: the car's camera (10 Hz), radar stand-in (10 Hz) and a roof lidar (10 Hz) render headless; `pixi run sim-check`, `--obstacle`, `--lidar-obstacle`, `--faults`, `--spine` and `--perception` pass. On Linux with no GPU, through `tools/software_gl.sh`: [[works]] on oirp-spss, 23 Oct (`sim-check --obstacle`: the sensors and the stop; the nightly CI job uses it) ([Simulation](simulation.md)) |
| The lidar in Nav2, in Gazebo (15 Oct) | `aav_lidar` `lidar_scan` -> the local costmap's `lidar_layer` and `collision_monitor` | [[works]] with the radar off: it stopped the car 0.16-0.57 m before a person-sized post. **Under load it can be late:** beside other jobs on the Mac it stopped 0.13 m past the post (20 Oct). Watch the Orin's load on the car |
| The lidar in Nav2, in bench mode (20 Oct) | `bench lidar:=true`, `mock_lidar` | [[works]] on oirp-spss: `bench-check --lidar` stopped 0.88 m before a box only the lidar saw. On a loaded Mac it failed (the spine latched). **Open:** the contract check missed the bench IMU's (or camera's) publisher in 3 of 3 runs, while the EKF got its data. Not yet explained |
| The lidar in Nav2, on the car | the stock velodyne driver, on in `orin.yaml` and `all.yaml` (20 Oct) | [[works]] against a pretend VLP-16 (`tools/fake_vlp16.py`), 10 Hz. The roof lidar can't see a 1 m box inside the stop zone: the radar's job ([Perception](perception.md)). [[untested]] with the real lidar |
| Sensor watchdog (15 Oct) | `aav_safety` `sensor_watchdog`: a twist_mux lock, the HUD's banner | [[works]] in Gazebo: the faults on purpose (`pixi run sim-fault`, `sim-check --faults`) stop Nav2 and say why, 14 now with the HUD's heartbeat (23 Oct: a frozen or dead HUD stops Nav2 on the car, `watch_hud`; [HUD](hud.md#the-heartbeat)), frozen wheels (cross-checked with the IMU and GPS) and the watchdog's own death (a fail-safe lock) too. Frozen wheels: 25 of 25 since 24 Oct (MPPI had hidden them; the GPS check now fits 10 s: [frozen wheels and MPPI](simulation.md#frozen-wheels-and-mppi)). On the bench, `bench-check` passes with it, and it flagged a 2.5 s stall of the whole Mac there, then let go. [[untested]] on the car ([Simulation](simulation.md#sensor-failures-on-purpose)) |
| Real car | `aav_spine` | [[works]] on the bench, against `spine_sim`: arming, driving, both e-stops |
| The spine in the loop, in Gazebo (15 Oct) | `pixi run sim spine:=true`: `spine_bridge` → CAN topics → `spine_sim` (`car:=gazebo`, the spine's logic) → the actuator model | [[works]] on the Mac: `pixi run sim-check --spine` passes (graph, drive, EKF, a Nav2 goal; commands lost: full brake 195-209 ms and latched; both e-stops: brake within 12 ms, latched; disarmed: no motion). [[untested]] on Linux ([Simulation](simulation.md#the-spine-in-the-loop-spinetrue)) |
| Today's car | `aav_legacy` | [[untested]] on the car |
| Spine firmware logic | `firmware/spine/crates/spine-core` | [[works]] 84 tests, on the bench, and in Gazebo (`spine:=true`) |
| Wheel node logic (15 Oct) | `firmware/spine/crates/wheel-core` | [[untested]] 9 tests; no board uses it: the hardware lead's Arduino sends WHEEL_SPEED (23 Oct, [Rust firmware](rust-firmware.md)) |
| Spine device drivers and glue | `jrk-g2`, `ad5311`, `pca9555`, `ibus`, `mcp2515`, `legacy-link`, `spine-board` | [[works]] about 240 tests (`spine-board` 79), against mocks; `legacy-link` also against the 2025-26 C++ itself |
| ESP32 board program (Rust) | `firmware/spine/boards/esp32-umod` | [[untested]] builds with Espressif's Rust (`pixi run esp-tools`), both links; never flashed. Merged 15 Oct; the C++ stays on the car until it is proven. In QEMU (24 Oct): brakes 111-120 ms after the Motor Pi's frames stop, and latches (`pixi run esp-qemu-check`). [First flash on the bench](esp32-bench.md) |
| Spine board program | `firmware/spine/boards/pico2` | [[untested]] builds for a Pico 2, and its UF2 file checks out; never flashed. The car's chip is still Q10–Q13 |
| Wheel encoder, in Rust (15 Oct) | `firmware/spine/boards/wheel-encoder-pico2`, crate `wheel-encoder` | [[untested]] builds, its UF2 checks out, 12 tests; never flashed. Same packet as the C++ (shared examples). The car's Pico 2 still runs the C++ |
| Status display, in Rust (15 Oct) | `firmware/spine/boards/sentinel-pico2`, crates `sentinel`, `hd44780` | [[untested]] builds, its UF2 checks out, 45 tests; never flashed. Nothing sends it packets yet |
| Bench mode | `aav_spine_sim`, `aav_bench` | [[works]] `pixi run bench-check`, plus `--perception` and `--lane` |
| Camera | `usb_cam`, the `camera` part of `car.launch.py` | [[untested]] Linux only, off until the camera is fitted; the models are Q22 |
| Front cameras, streamed | the camera Pi's MediaMTX -> `camera_stream_front_left`, `_front_right` (the `camera_stream` part, `orin.yaml`), the detector inside; the sensor watchdog watches their detections | [[works]] on a Mac (`pixi run -e ml stream-check --gst`: 26 ms to detections; the watchdog OK, then STALE when the stream stops). [[untested]] on the Orin and the Pi: skipped there until RF-DETR's TensorRT engine is built ([camera pipeline](camera-pipeline.md)) |
| Stop signs | `aav_detector`: RF-DETR (default since 20 Oct) or YOLO, `detector:=` | [[works]] on the bench with both stock models: RF-DETR's lock came on at 20.7 m, the sign at 25 m (20 Oct). `sim-check --perception` in Gazebo with RF-DETR (24 Oct): first seen at x = 5.9 m, the sign at 25 m; the car stopped with its bumper 4.4 m before it. [[untested]] on the Orin: its TensorRT engine is written, not built |
| Lane keeping | `aav_lane_finder` + `aav_lane_geometry` | [[works]] on the bench: from 0.3 m off the centre line back to within 5 cm in about 6 s |
| Radar clustering | `aav_radar_clusters` (Rust) | [[works]] on the bench and in Gazebo's road world (15 Oct); gives the 2025-26 C++ node's answers, bit for bit |
| Obstacles in Nav2 | the radar in the local costmap; `collision_monitor`'s stop zone; `camera_obstacles` | [[works]] on the bench: the stop zone stops the car 0.3-0.4 m before a box as wide as the lane. Open: the car weaves up to 1.5 m off its line (N7), so it can pass beside a narrow box; the stop zone flickers at its edge. The camera layer is off until the cameras are calibrated (Q22) |
| HUD back end and app (Flask, Flutter) | `archive/2026-27-flutter-hud/` | archived 25 Oct: the raylib HUD replaced it (Q29). Not built, not tested |
| HUD and map window in Rust (raylib) | `aav_hud` | [[works]] on a Mac: `--demo`, and as a node fed by `ros2 topic pub`; its e-stop reaches a subscriber as 5 trues. Its heartbeat (23 Oct): the real window beat at 9.3 Hz, and freezing or closing it made the watchdog say `hud STALE` in 1 s. Since 24 Oct a dropout after its first 5 s is latched (`hud LOST`, `pixi run watchdog-clear`), and a GO / STOP panel says whether the car may move (logic tested; not yet seen on screen). [[works]] next to bench mode too. [[untested]] on a Pi and on the car's screen |
| CARLA | `aav_carla` | [[untested]] adapter built and tested without a server; needs the NVIDIA computer ([CARLA](carla.md)) |
| Heading from the GPS velocity (25 Oct) | `aav_gps_heading` `gps_heading` -> `ekf_global` (yaw only), `pixi run heading-check` | [[works]] in Gazebo: at walking pace the heading's median error fell from 2.2-2.8° to 1.6-2.1°, forward and reversing. [[untested]] on the car: needs `gps_driver:=ublox` for the velocity |
| RC via ROS (24 Oct) | `aav_rc` `rc_driver`: `/vehicle/rc` -> `/drivers/rc/cmd_vel` (twist_mux 95); the ESP32's uplink (`--features uplink`) | [[works]] in QEMU (all six RC steps) and in a ROS test. Off until the hardware lead answers H9 and H10. [[untested]] on a real ESP32 and the car |
| The ESP32's uplink (24 Oct) | `legacy_uart_bridge` `uplink: true` -> `/vehicle/status`, `/vehicle/steering`, `/vehicle/brake`, `/vehicle/rc` | [[works]] in QEMU: 50-51 Hz, 0 frames lost. Off by default. [[untested]] on a real ESP32 and Pi ([ESP32 bench](esp32-bench.md)) |
| Street routes (16 Oct) | `aav_routes` (Valhalla, offline) | [[works]] in Gazebo: a street route drove to 0.5 m of its end through Nav2 ([Street routes](routes.md)). [[untested]] on the car |
| Ghost obstacles (16 Oct) | `aav_ghost`, `pixi run ghost` | [[works]] in Gazebo: `sim-check --ghost` stopped 0.21 m before it ([Ghost obstacles](ghost-obstacles.md)). [[untested]] on the car |
| Actuator tests (15 Oct) | `aav_actuator_tests`, `pixi run actuator-test` | [[works]] in Gazebo. [[untested]] on stands ([Actuator tests](actuator-tests.md)) |
| The speed loop's `smooth` profile (25 Oct) | spine-core `SpeedLoopProfile`, `speed_loop:=smooth` in sim and bench | [[works]] in the model and Gazebo: overshoot 15% -> 0%. Not the default. [[untested]] on stands |
| Watch from a laptop (24 Oct) | `foxglove.launch.py`, three layouts in `tools/foxglove/` | [[works]] live with the road sim and the bench: every layout topic served. The layouts not yet opened in the app by hand. [[untested]] at the car ([Foxglove](foxglove.md)) |
| Car-day kit (24 Oct) | `pixi run car-day-build`, `pixi run car-day-check` | [[works]] on Ben's Mac: both ESP32 builds, QEMU passes, the check's 35 tests ([Car day](car-day.md)) |
| Lidar | a Velodyne VLP-16 on the roof (the 16 Oct audit: 192.168.1.201, sending to the Jetson). The stock `velodyne` driver (`aav_sensors` `lidar:=true`, the `lidar` part in `car.launch.py`) -> `/sensors/lidar/points`; a generic 16-beam one in Gazebo | [[works]] against `tools/fake_vlp16.py`: a post 6 m ahead came out at 5.80-6.00 m, Nav2's scan 6.75 m from `base_link`, 10 Hz (20 Oct). [[untested]] with the real lidar |

## Builds and tests

| Check | Result |
|---|---|
| `colcon build`, all 37 packages, Jazzy | [[works]] on Linux and the Mac (25 Oct) |
| `pixi run test`: build, tests, lint, `colcon test` | [[works]] 25 Oct: 1110 tests, 0 failures, 4 skipped, 37 packages (after the Flutter HUD's back end went to the archive). CI runs it on Linux (x86_64 and 64-bit ARM) for every push |
| Live graph vs `docs/interfaces.yaml`: turtle, sim, road, replay and bench | [[works]] 0 failures each (sim and road again on 15 Oct, after the topic renaming) |
| `pixi run bench-check` (`--perception`, `--lane`, `--obstacle`) | [[works]] on `main`, 11 Oct: the default checks, the stop sign, lane keeping and the obstacle |
| Sim, headless: drive commands (the keyboard's topic) and Nav2 goals | [[works]] |
| Replay of a recorded drive | [[works]] the EKFs match the recording within 0.03 m |
| Car, with no hardware attached | comes up; only the missing sensors (IMU, GPS) fail the graph check, as expected. Checked again on 11 Oct, with the perception parts |
| The same `pixi.toml` on macOS | [[works]] on an Apple Silicon Mac (macOS 27), 10 Oct: build 18 s, `pixi run test` all green after one fix (GNU `coreutils`, see [Your Mac](mac.md)) |
| On the Mac: the turtle, Gazebo headless, a Nav2 goal, the docs, `pixi run doctor`, recording a drive | [[works]] |
| On the Mac: Gazebo's window | [[works]] `pixi run sim` opens it, and the camera follows the car |
| On the Mac: RViz, rqt_graph | [[untested]] they open; nobody has looked at them yet |
| The same `pixi.toml` on 64-bit ARM Linux | [[works]] in CI on Ubuntu 24.04 ARM (25 Oct): `pixi run test`, and the car kit installed with no network. [[untested]] on the car's own computers (Ubuntu 22.04, JetPack 6) |

## Is it all on Jazzy?

Yes. Everything builds against Jazzy, and the turtle, sim and replay profiles run. Every
configuration key was checked against what the running Jazzy nodes accept. The fixes were
small: clean shutdowns, Nav2 on simulated time, and the simulated GPS noise. The details
are in `docs/history/audit-2026-10-10-*.md`.

## Next

1. Join the car's network from the Mac (`pixi run doctor`), once the car runs Jazzy.
2. Pick the team's `ROS_DOMAIN_ID` and the GPS driver.
3. Drive the turtle with the joystick.
4. Measure the car, and replace the guesses in `vehicle.yaml`.
5. Work through the car-day sessions in order ([Car day](car-day.md); TODO.md "Ready for the car").
