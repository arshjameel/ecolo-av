# The plan: the proposal as a to-do list

Written 22 October 2026 from the team's proposal, "Ecolo: an open, closed-loop autonomous
vehicle platform" (version 4, the team draft for 23 October). Ben: "Turn that document
into a todo list that we can work through", and "we do want to put mppi in there too".

The task numbers (T1.1 ... T9.8) are the proposal's, so the same numbers work in the issue
tracker. Only the **MPPI track** (M1-M9) is new. The 15 October plan's S and C items are at
the end, mapped to these numbers.

> **The short version**
> - **Software, built and tested in simulation: about 43%** of the committed tasks. **Proven on the car: about 5%** (the audits).
> - **Ahead:** the drivers, ego, the muxes, Gazebo, Nav2, A to B, the HUD. **Behind:** everything that needs the car or new parts: power, calibration, the closed loop.
> - **Next gate, 30 Oct (Know the car):** the car isn't measured yet. That's the most urgent job.
> - **MPPI:** Nav2's default since 22 Oct, tuned for the car in Gazebo, and compared with Regulated Pure Pursuit by `pixi run controller-compare` (`controller:=rpp` for the old one; [controllers](path-followers.md)). It steers round an obstacle the sensors see.
> - **Status:** ✅ done · 🧪 done in simulation only · 🔧 started · ⬜ not started · 🎯 stretch.

## How far along, by sub-project

The percentages are judgement, task by task (below), not measurement. "In software" counts
work that runs in Gazebo or on the bench; "on the car" counts only what ran on the car.

| # | Sub-project | In software | On the car | Weakest point |
|---|---|---:|---:|---|
| 1 | Integration and platform | 55% | 5% | the Orin and Pis not upgraded; the sensor and output muxes not yet run on the car |
| 2 | Simulation and testing | 63% | 0% | sensor noise from guesses; no sim-versus-real report; the nightly Gazebo job on trial |
| 3 | Vehicle hardware and actuation | 33% | 5% | the car isn't measured; the Rust spine never flashed |
| 4 | Power and electrical | 4% | 0% | not started: needs the car and the parts |
| 5 | Ego and vehicle control | 50% | 0% | stop-at-distance lives in the A-to-B driver |
| 6 | Localization and calibration | 18% | 0% | no calibration yet; the heading is much better in Gazebo (22 Oct), not yet checked with the real IMU |
| 7 | Drivers and local planning | 64% | 0% | the real lot's map; forward only, a U-turn in a narrow lane can end facing a wall (reversing fixes it) |
| 8 | Perception | 43% | 0% | no labelled pictures of the lot; no lidar clustering |
| 9 | Heads-up display and tools | 68% | 0% | not tried on the Pi 5 |
| | **All committed tasks** | **about 43%** | **about 5%** | |

## The gates

| Gate | Date | Passes when | Today |
|---|---|---|---|
| 1. Know the car | 30 Oct | audit written up; every dimension measured, with its method | 🔧 audits done 16 and 21 Oct ([car audit](history/audit-2026-10-21-car.md)); **nothing measured yet** ([the sheet](measure-the-car.md)) |
| 2. Safe to move | 27 Nov | the timeout brakes within 100 ms and latches; both e-stops stop the car | 🔧 the Rust spine has the 100 ms timeout (spine-core: 84 tests); in QEMU (24 Oct) it brakes 111-120 ms after the last frame and latches ([emulator](esp32-bench.md#before-the-board-the-emulator)): 100 ms of silence, then the next 10 ms tick. Since 24 Oct the heartbeat comes from the Orin (TODO 50): it brakes 111-120 ms after the Orin's counter stops, though the Motor Pi still sends ([the heartbeat](rust-firmware.md#the-heartbeat-version-2-of-the-serial-frame-24-oct)). Never flashed; today's ESP32 firmware has no timeout |
| 3. Closed loop | 18 Dec | steering angle, wheel speed, brake position read back and tracked | ⬜ the sensors aren't fitted; the CAN messages and the logic exist |
| 4. Drives itself | 26 Feb | GPS A to B, within 1 m, five runs, no takeover | 🧪 in Gazebo: every stop within 0.4 m of B |
| 5. Handover | 2 Apr | scorecard; every layer documented; simulation in 30 minutes | 🔧 docs and onboarding exist; the scorecard doesn't |

## What's done so far

| Area | Done | Where |
|---|---|---|
| One repo, one environment | ROS 2 Jazzy through Pixi on Linux and Macs, no admin rights; CI on Linux (x86_64 and 64-bit ARM) on every push, a Mac nightly | `pixi.toml`, `.github/workflows` |
| The contract | every topic and node in one file, checked against the running graph | `docs/interfaces.yaml`, `pixi run check` |
| Old code | every 2025-26 and 2024-25 repo surveyed; kept, ported or retired, with reasons | `docs/older-code.md`, each `PROVENANCE.md` |
| Rust | the spine, the wheel encoder, the status display, the IMU driver, the radar clustering | `firmware/spine`, `ros/sources`, `ros/perception` |
| Ego and the mux | twist_mux (a person always wins), ego (Ackermann, limits, timeouts), and now the car's weight | `ros/ego`, `ros/command_mux`, `ros/io_mux` |
| Simulation | TurtleSim; Gazebo with the whole car (cameras, radar, lidar, GPS, IMU, wheel speed, actuator model); the bench with the real CAN messages | `pixi run turtle`, `sim`, `bench` |
| Position | two EKFs and navsat_transform: within 0.15 m of Gazebo's truth; replay within 0.03 m | `ros/localization` |
| Drivers | joystick, keyboard, scripted, replay, GPS A to B, Nav2, plain English (local model), street routes | `ros/drivers` |
| Nav2 | Smac Hybrid-A*, **MPPI** (the default) or Regulated Pure Pursuit, collision monitor, ghost obstacles | `ros/drivers/aav_navigation` |
| Safety | a sensor watchdog (14 faults, the HUD's heartbeat among them, stop the car in Gazebo; frozen wheels 25 of 25 since 24 Oct); the collision monitor; the spine's timeout (not flashed) | `ros/safety`, `firmware/spine` |
| Screens | the raylib HUD and the map window, full screen at boot (ready to install) | `ros/hud` |
| Perception | RF-DETR Nano by default, YOLO with `detector:=yolo` (Core ML on a Mac; a TensorRT engine for the Orin); the camera streams (`aav_camera_stream`); radar clustering; a lidar layer | `ros/perception`, [newer models](model-comparison.md) |
| The car | two read-only audits: every computer, port and connection; irreplaceable files copied off | `docs/history/` |
| Docs | 77 pages for a dyslexic reader, as HTML and PDF | `docs/` |

## The to-do list

**Due** is the proposal's date. **%** is how much of the task exists today. **Next** is
the next concrete step.

### 1 · Integration and platform

| # | Task | Due | Status | % | Next |
|---|---|---|---|---:|---|
| T1.1 | Repository, contract and CI | 6 Nov | ✅ | 90 | time a new laptop through [Getting started](getting-started.md) (Arsh, 24 Oct): under 30 min? |
| T1.2 | Bring in the earlier teams' code | 20 Nov | ✅ | 100 | |
| T1.3 | Orin backup and upgrade to JetPack 7.2.1 | 27 Nov | 🔧 | 10 | full disk image, then test the restore (irreplaceable files already copied off) |
| T1.4 | Raspberry Pis to Ubuntu 24.04 | 27 Nov | 🔧 | 10 | one imaging script; the car kit is ready (`pixi run car-kit`) |
| T1.5 | One launch, `target:=` and `source:=`; the sensor mux and output mux | 4 Dec | 🧪 | 80 | built 23 Oct (`aav_io_mux`, `stack.launch.py target:=`; [sources and targets](sources-and-targets.md)): every profile runs both muxes; `sim-check` and `bench-check` pass with them. Next: on the car, `/safety/output_mux/state` says OPEN with only the car's own nodes; the team agrees D3 |
| T1.6 | Contract test | 4 Dec | ✅ | 80 | add topic rates to the nightly check |
| T1.7 | Measure the computers and the network | 18 Dec | 🔧 | 35 | the load report on the Mac (22 Oct, below); the bandwidth budget (23 Oct, [bandwidth](bandwidth.md)): only raw cameras overflow a link, CAN 8.5% busy; `pixi run bandwidth` measured Gazebo (33-35 MB/s). Next: run it on the car, then the Orin's CPU |
| T1.8 | Port to Rust, one module at a time | Jan-Mar | 🔧 | 65 | IMU, radar and the CAN bridge done (`aav_spine_bridge`, the default since 23 Oct; the Python one goes once the Rust has run on the car); next the battery monitor |
| T1.9 | Second Orin (stretch) | Feb-Mar | 🎯 | 0 | KEFC decision, 2 Nov |

### 2 · Simulation and testing

| # | Task | Due | Status | % | Next |
|---|---|---|---|---:|---|
| T2.1 | TurtleSim target | 6 Nov | ✅ | 100 | |
| T2.2 | Ecolo URDF | 20 Nov | 🔧 | 60 | built from `vehicle.yaml`; needs the measurements (T3.2) |
| T2.3 | Gazebo car | 4 Dec | ✅ | 100 | checked 22 Oct: a 10° steer gives 9.05-9.14 m for the predicted 8.90 m (+2-3%); 20° and full lock also within 3% ([simulation](simulation.md#what-gazebo-simulates)) |
| T2.4 | Noise matched to the real sensors | Jan | 🔧 | 20 | needs recordings from the car |
| T2.5 | Replay driver and comparison | 18 Dec | 🔧 | 60 | the replay driver exists; write the drive-comparison script (`tools/drive_metrics.py` is a start) |
| T2.6 | Integration tests in CI, nightly | mid-Jan | 🔧 | 70 | the nightly job `sim` runs `sim-check --obstacle` headless (22 Oct), on trial: passed 21, 22 and 24 Oct; a Gazebo crash at start is now retried once (TODO 26). Once it passes a few nights in a row, let a failure count |
| T2.7 | Sim-versus-real report | Gates 3, 4 | ⬜ | 0 | needs the car |
| T2.8 | Ghost obstacles | 5 Feb | 🧪 | 80 | on stands, then the lot |
| T2.9 | CARLA target (stretch) | Mar | 🎯 | 40 | adapter built; needs the NVIDIA machine |

### 3 · Vehicle hardware and actuation

| # | Task | Due | Status | % | Next |
|---|---|---|---|---:|---|
| T3.1 | Audit | 30 Oct | 🔧 | 70 | a photo of every connector; the power side |
| T3.2 | **Measure the car** (and its weight and balance) | 30 Oct | 🔧 | 10 | **[the sheet](measure-the-car.md)**, M1-M7b: tape, angle finder, four bathroom scales |
| T3.3 | Steering range and centre | 20 Nov | ⬜ | 10 | `pixi run actuator-test` has the sweeps; the car on stands |
| T3.4 | Linkage re-mount (if needed) | 4 Dec | ⬜ | 0 | after T3.3 |
| T3.5 | CAN bus, wheel nodes, brake sensor | 11 Dec | 🔧 | 30 | the message set (`firmware/can/aav.dbc`) and the logic exist; the parts are on the KEFC list |
| T3.6 | Spine firmware in Rust | 27 Nov | 🔧 | 70 | 24 Oct: boots in QEMU and brakes 111-120 ms after the Motor Pi's frames stop (`pixi run esp-qemu-check`). The same day, TODO 50: the version 2 serial frame carries the Orin's heartbeat (counter, flags, CRC-8); QEMU brakes 111-120 ms after the counter freezes or the CRCs go bad. Next: flash it on the bench ([ESP32 bench](esp32-bench.md)), with `frame_version: 2` |
| T3.7 | Bench rig | 20 Nov | 🔧 | 40 | `pixi run bench` covers the software; build the board |
| T3.8 | Hardware test suite | 18 Dec | 🔧 | 60 | the scripts exist (tests.md stages 3-4); run them on stands |
| T3.9 | Brake and throttle loops | 22 Jan | 🔧 | 20 | the PI loop is in `spine-core`; tune it on stands. With today's guessed gains it overshoots about 15% (the Gazebo model, 22 Oct: 0.57 m/s backing up for 0.5). A `smooth` profile (setpoint weight) is ready to try on stands, 0% in the model and Gazebo: TODO 27 |

### 4 · Power and electrical

| # | Task | Due | Status | % | Next |
|---|---|---|---|---:|---|
| T4.1 | Power audit | 13 Nov | 🔧 | 5 | clamp meter, every load |
| T4.2 | Regulated 5 V supplies | 27 Nov | ⬜ | 0 | the ZK-12KX converters (KEFC) |
| T4.3 | 12 V rail at actuator peak | 27 Nov | ⬜ | 0 | |
| T4.4 | Battery monitor and temperature | 11 Dec | 🔧 | 20 | the 2024-25 node is in (`aav_battery_victron`); the Junctek reader is not |
| T4.5 | Rail monitors | 18 Dec | ⬜ | 0 | |
| T4.6 | Wiring diagram | 18 Dec | 🔧 | 5 | the photos in [car photos](car-photos.md) are a start |
| T4.7 | One-hour soak test | 18 Dec | ⬜ | 0 | |

### 5 · Ego and vehicle control

| # | Task | Due | Status | % | Next |
|---|---|---|---|---:|---|
| T5.1 | Command mux | 6 Nov | ✅ | 100 | |
| T5.2 | Ego v1 | 6 Nov | ✅ | 100 | |
| T5.3 | Speed profile: acceleration, jerk, stop at a distance | 27 Nov | 🔧 | 75 | jerk limit done 22 Oct (`max_jerk` 2.0 m/s³, speeding up only; `sim-check --obstacle` passes). Left: move the stop-at-distance curve from the A-to-B driver into ego, or agree it stays (D4) |
| T5.4 | Turning limits: steering rate, sideways acceleration | 11 Dec | 🧪 | 90 | done 22 Oct with the weight model ([car numbers](car_measurements.md#weight-and-balance)); the Gazebo check "a tight turn slows to the computed speed" |
| T5.5 | Measured parameters | 18 Dec | ⬜ | 0 | after T3.8 |
| T5.6 | Joystick drives | Dec-Jan | 🔧 | 20 | the code exists (`aav_teleop`), untested ([status](status.md)): the turtle and Gazebo first; on stands after Gate 2 |
| T5.7 | Model validation | 12 Feb | ⬜ | 0 | needs recordings |
| T5.8 | Rust port of ego | 19 Mar | ⬜ | 0 | |

### 6 · Localization and calibration

| # | Task | Due | Status | % | Next |
|---|---|---|---|---:|---|
| T6.1 | Sensor positions | 13 Nov | 🔧 | 20 | the sheet, S1-S8 |
| T6.2 | Time sync (chrony) | 27 Nov | 🔧 | 10 | |
| T6.3 | Filters in Gazebo | 11 Dec | 🧪 | 90 | 0.15 m done; the heading jumps at walking pace much smaller (22 Oct: yaw noise, M9), and since 25 Oct a heading from the GPS velocity (`aav_gps_heading`: median 1.6-2.1°); check against the real IMU's gyro bias |
| T6.4 | GPS on UBX with RTK | 11 Dec | 🔧 | 30 | the stock `ublox_gps` driver is in the environment (`ublox_dgnss` is apt-only, the car only); the new receiver (KEFC) |
| T6.5 | IMU calibration (Allan variance) | 18 Dec | ⬜ | 0 | |
| T6.6 | Camera intrinsics | 22 Jan | ⬜ | 0 | |
| T6.7 | Lidar-camera extrinsics | 12 Feb | ⬜ | 0 | |
| T6.8 | Filters on the car | 12 Feb | ⬜ | 0 | |
| T6.9 | Accuracy and drift tests | 26 Feb | ⬜ | 0 | |

### 7 · Drivers and local planning

| # | Task | Due | Status | % | Next |
|---|---|---|---|---:|---|
| T7.1 | Joystick and remote | 13 Nov | 🔧 | 60 | built, untested ([status](status.md)): drive the turtle with the joystick. RC via ROS (`aav_rc`) works in QEMU, off until H9 and H10 |
| T7.2 | Scripted and replay drivers | 4 Dec | 🧪 | 80 | the "same path twice within 0.03 m" check |
| T7.3 | GPS A to B in Gazebo | 18 Dec | 🧪 | 100 | |
| T7.4 | Hand-drawn costmap of the lot | 11 Dec | 🔧 | 20 | choose the lot; `tools/make_map.py` draws it |
| T7.5 | Nav2 in Gazebo | 5 Feb | 🧪 | 90 | the corner problem fixed (M8); allowed to reverse, both controllers reach every lot goal, U-turn included (22 Oct). Left: forward only, stuck facing a wall (TODO 18) |
| T7.6 | Language-model driver | 29 Jan | 🧪 | 90 | |
| T7.7 | GPS A to B on the car | 26 Feb | ⬜ | 0 | 🎯 the year's demonstration |
| T7.8 | Nav2 on the car | 26 Mar | ⬜ | 0 | |
| T7.9 | MPPI | Mar | 🧪 | 70 | the default since 22 Oct; a track of its own: M1-M9 below |

### 8 · Perception

| # | Task | Due | Status | % | Next |
|---|---|---|---|---:|---|
| T8.1 | Record and label 200 frames | 18 Dec | ⬜ | 0 | during the joystick drives |
| T8.2 | Pipeline: decode, undistort, detect, track, publish | 15 Jan | 🔧 | 65 | the detector node exists, RF-DETR Nano by default, its models loadable without ROS (`aav_detector/detectors.py`) for the camera-stream program; the stream itself (`aav_camera_stream`, 24 Oct): decode, undistort, detect, publish, measured on a Mac ([camera pipeline](camera-pipeline.md)), untested on the Orin; ByteTrack doesn't exist yet |
| T8.3 | Detector comparison | 12 Feb | 🔧 | 60 | done on the Mac ([newer models](model-comparison.md)): **RF-DETR Nano chosen** (Ben, 23 Oct). Its Core ML export runs (10 ms on a Mac); its TensorRT engine build is written (`tools/build_engine.py`): run and time it on the Orin |
| T8.4 | Lidar clustering | 29 Jan | 🔧 | 30 | today a height band (`aav_lidar`); add clustering |
| T8.5 | Lidar costmap layer | 19 Feb | 🧪 | 100 | in Gazebo Nav2 stops for it, and plans round it (M5, 22 Oct) |
| T8.6 | Live on the car (stretch) | Mar | 🎯 | 0 | |

### 9 · Heads-up display and operator tools

| # | Task | Due | Status | % | Next |
|---|---|---|---|---:|---|
| T9.1 | Layout | 6 Nov | ✅ | 80 | agree it with the safety drivers |
| T9.2 | Feasibility on the Pi 5 | 13 Nov | 🔧 | 60 | 25 Oct: `aav_hud` builds on 64-bit ARM Linux, and the HUD and the map window draw there (CI's `arm` job, every push; software OpenGL). Next: on the Pi 5 itself, its frame rate on its GPU |
| T9.3 | HUD v1 | 4 Dec | 🧪 | 90 | |
| T9.4 | Map view | 18 Dec | 🧪 | 90 | the real lot's image (T7.4) |
| T9.5 | Faults, battery and health | 18 Dec | 🧪 | 75 | 24 Oct: the HUD's GO / STOP panel says whether the car may move, and the one main reason (the e-stop, the output mux's gate, the sensor watchdog). Next: rail voltages (T4.5) |
| T9.6 | Boot and crash tests | 18 Dec | 🔧 | 50 | on the Pi |
| T9.7 | The language model's plan on screen | 29 Jan | 🔧 | 50 | |
| T9.8 | HUD stop and heartbeat | 29 Jan | 🧪 | 80 | done 23 Oct: `/hud/heartbeat` while the screen draws; the watchdog stops Nav2 without it (car: `watch_hud:=true`). Passes in Gazebo (`sim-check --faults`, `hud stop`) and with the real window on a Mac. 24 Oct (Ben): latched, a dropout after the HUD's first 5 s holds Nav2 until `pixi run watchdog-clear`. Next: on the car's screen computer |

### Safety (cross-cutting)

| # | Task | Status | Next |
|---|---|---|---|
| S-1 | A written test card for every car session | ⬜ | a template in the repo |
| S-2 | Written permission for the lot (November) | ⬜ | |
| S-3 | Geofence: everything outside the test area blocked on the map | 🔧 | the lot's map (T7.4) |
| S-4 | Simulated messages can never reach the car | 🧪 | two layers: stand-ins run `--local`; and (23 Oct) the real backends read only `/control/car/actuator_cmd`, which the output mux fills only while every source is REAL, else a latched stop. Tested: with Gazebo announcing SIM, `spine_bridge` sent no drive frame. Next: the same on the car |
| S-5 | Speed capped at 1 m/s in the firmware and in ego | 🔧 | ego ✅; the firmware's cap with T3.6 |

## The MPPI track (new)

MPPI was a stretch goal for March (T7.9). Ben, 22 Oct: "we do want to put mppi in there
too". It shares the car's model with ego, as the proposal says (section 8.3).

| # | Task | Status | Next |
|---|---|---|---|
| M1 | `controller:=mppi`: one argument switches Nav2's path follower | ✅ 22 Oct | |
| M2 | Its limits from `vehicle.yaml`: turning radius, outline, top and reverse speed | ✅ | its braking limit from ego (done by hand: `ax_min` = ego's -2.0) |
| M3 | Tuned for a car, not a robot, in Gazebo | ✅ 22 Oct | moving within 0.5 s at 0.95 m/s; turns to a goal's heading from 2.5 m out ([why](path-followers.md#making-mppi-drive)). Its U-turn in a 14 m lane: forward only it swings wide and fails; allowed to reverse it makes a three-point turn (22 Oct, [controllers](path-followers.md)) |
| M4 | The comparison: `pixi run controller-compare` | ✅ 22 Oct | six runs overnight; MPPI did at least as well in each. **MPPI is the default** since 22 Oct ([controllers](path-followers.md)) |
| M5 | Steering round an obstacle | ✅ 22 Oct, Gazebo | the planner's costmap has the radar and lidar (`plan_around:=true`); both controllers drove round the box 2.6-3.8 m clear. `sim-check --plan-around` |
| M6 | Its CPU on the Orin | ⬜ | with the load report (T1.7) |
| M7 | MPPI on the car | ⬜ | after Nav2 passes on the car (T7.8); MPPI is the default now |
| M8 | **The corner problem** (both controllers) | ✅ 22 Oct, Gazebo | the planner's costmap inflates 3.0 m gently, and it plans turns 1.25 x wider than full lock: both kept clear of the building. Left: after a wide turn the car can stop facing a wall, and forward only it can't get out (TODO 18) |
| M9 | **The heading jumps** at walking pace (both) | 🧪 | measured over whole drives (22 Oct night): under 6° when slow, up to 25° at speed; the position is up to 0.7 m off, so goals count within 1.0 m. **A heading source since 25 Oct:** the GPS receiver's velocity (`aav_gps_heading`, TODO 13): median 1.6-2.1° at walking pace. RTK or two antennas would do better: the hardware lead decides, with T6.4. Reversing is worse while the antenna is ahead of the rear axle (22 Oct, [why](learn-gps.md#mount-the-antenna-over-the-rear-axle)): on the car it is over the axle already (Ben, 23 Oct). **Much better since 22 Oct, 06:00:** the global EKF's yaw noise 0.06 → 0.01; the lot tour's worst heading per leg 7-16° (was 14-76°) |

## Where the proposal and the repo disagree

For the team to decide; then change one of them.

| # | Proposal | Repo today | Suggest |
|---|---|---|---|
| D1 | Topic names such as `/control/cmd`, `/vehicle/ackermann_cmd`, `/odometry/local` (section 3.3) | `/control/cmd_vel`, `/ego/actuator_cmd`, `/localization/odom/local` ([naming](naming.md)) | change the proposal's table: the repo's names are built and tested |
| D2 | Hall-sensor wheel nodes on Arduino Unos, in Arduino C++ | the rule "Rust for all firmware" (Ben, 10 Oct); the wheel logic in Rust (`wheel-core`) and the Pico 2 wheel encoder | ✅ **decided 23 Oct** by the hardware lead (passed on by Ben): an Arduino reads the sensors and sends CAN frames; the Orin decodes them and publishes. The proposal stands |
| D3 | A sensor mux and an output mux as nodes; `target:=`, `source:=` | built 23 Oct as nodes (`aav_io_mux`). The sensor mux **checks** where each sensor comes from instead of republishing it (no doubled camera or lidar traffic); the launch picks the source | agree "check, not copy" ([why](sources-and-targets.md#why-check-not-copy)), and change the proposal's wording |
| D4 | The stop-at-distance planner inside ego (T5.3) | inside the A-to-B driver (`aav_drive_to_point`) | either; ego is where the proposal promises it |
| D5 | MPPI a stretch goal in March (T7.9) | switchable and compared now | put MPPI in the committed plan, in Gazebo; on the car as the stretch |
| D6 | Ego enforces the lateral acceleration (T5.4) | done 22 Oct, with the car's weight | the proposal can cite it |

## The 15 October numbering

`docs/` still refers to the old items in places:

| Old | Now |
|---|---|
| S1 A → B in Gazebo | T7.3 |
| S2 the spine in the loop | T3.7 |
| S3 Gazebo steering fix | T2.3 |
| S4 ghost obstacles | T2.8 |
| S5 touchscreen keyboard | T9.4 |
| S6 newer models | T8.3, T7.6 |
| S7 Python → Rust | T1.8, T5.8 |
| S8 street routes | (beyond the proposal) |
| S9 CARLA | T2.9 |
| C1 audit | T3.1 |
| C2 measure and weigh | T3.2 |
| C3 the safety timeout | T3.6 |
| C4 closed-loop sensors | T3.5 |
| C5 the Jetson upgrade | T1.3 |
| C6 drivers for the real parts | T6.4, T8.2 |
| C7 install | T1.4, T9.6 |
| C8 stands, then the lot | T3.8, T5.6 |
| C9 A → B on the car | T7.7 |
| C10 a Gazebo copy of the real lot | T7.4 |
