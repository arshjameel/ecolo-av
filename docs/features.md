# Features: what the car's software does, and why

Every feature of the car's software, one row each, grouped by type.
Add a row in the same commit as the feature (`CLAUDE.md`).
"Why" says the reason for the feature, not how it works. "Who" is the person (or team) who built it.

## Status values

| Status | Means |
|---|---|
| planned | agreed, not built yet |
| built | in the repo, with its unit tests |
| sim | works in Gazebo or the turtle (or bench mode) |
| car | has worked on the real car |

## Sensors

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| IMU driver in Rust (`aav_imu_yahboom`) | Our C++ moves to Rust; it gives the 2025-26 C++ driver's answers, bit for bit | Ben | 2026-10-10 | built |
| GPS driver: stock `ublox_gps`, with `nmea` or `ublox_dgnss` as options | A stock driver that installs with Pixi on every computer, while the receiver model is still open (Q18) | Ben | 2026-10-11 | built |
| TI radar driver (`aav_radar_ti`) | The car's radar already worked with it, so we ported it to Jazzy instead of rewriting it | the 2025-26 team, ported by Ben | 2026-10-09 | built |
| Battery monitor (`aav_battery_victron`) | Shows the battery's state, so the team sees it before it runs flat | the 2024-25 team | 2026-10-09 | built |
| Roof lidar: the stock `velodyne` driver | The car's VLP-16 already speaks to this stock driver; it gives Nav2 a view all round the car | Ben | 2026-10-20 | built |
| Wheel speed from CAN (0x200, 0x201; `spine_bridge role:=sensors`) | The hardware lead's Arduino puts the wheel speeds on CAN; the Orin turns them into odometry | Ben | 2026-10-15 | sim |
| [Front camera streams](camera-pipeline.md) (`aav_camera_stream`) | H.264 from the camera Pi is about 10 times smaller than JPEG, so both front cameras fit the network | Ben | 2026-10-24 | built |
| Camera calibrations, and `pixi run calibrate-camera` | A calibrated lens turns pixels into metres, for the stop-sign distance and camera obstacles | the 2024-25 team; Ben | 2026-10-09 | built |

## Localization

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| Two EKFs and GPS to map (robot_localization) | Stock filters replace the 2025-26 odometry: a smooth position for steering, a GPS-anchored one for planning | Ben | 2026-10-09 | sim |
| GPS outlier gate in `ekf_global` | A fix 10 sigma off is refused, so one bad GPS jump can't throw the car's position | Ben | 2026-10-15 | sim |
| Heading from the gyro and wheels | At walking pace GPS made the heading jump up to 76°; the gyro and wheels hold it steady | Ben | 2026-10-22 | sim |
| Heading from the GPS velocity (`aav_gps_heading`) | The receiver's Doppler velocity gives a good heading while moving, with no magnetometer | Ben | 2026-10-25 | sim |
| [AMCL as a trial](trials.md) (stock `nav2_amcl`, `amcl:=true`) | To score a particle filter on the lidar and the map against the global EKF, without letting it steer the car (`tf_broadcast: false`) | Ben | 2026-10-29 | built |
| NDT on a saved lidar map, our own in Rust (`aav_ndt`) | To find the car to centimetres on a map of the lot without RTK, as Autoware does, in code we understand and test on a laptop; scored as a trial before it feeds the EKF | Ben's idea; built by Ben | 2026-10-29 | sim |

## Perception

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| [Object detector](perception.md) (`aav_detector`): RF-DETR Nano, YOLO as an option | RF-DETR saw a stop sign from 20 m (YOLO11n: 10 m), with no false finds and an Apache-2.0 licence | Ben | 2026-10-20 | sim |
| One detector per computer: Core ML on a Mac | A laptop adds a second opinion over WiFi, at 21 ms a picture on a MacBook | Ben | 2026-10-11 | sim |
| Stop-sign watcher | Holds the drive commands so the car stops about 4.7 m before a stop sign | the 2024-25 team, rewritten by Ben | 2026-10-10 | sim |
| Lane finder (`aav_lane_finder`) | Classic OpenCV finds the lane in 0.78 ms a picture, with no model to train | Ben | 2026-10-10 | sim |
| Lane keeping (`aav_lane_geometry`) | Steers back to the lane's centre: on the bench, from 0.3 m off to within 5 cm | the 2025-26 team | 2026-10-10 | sim |
| Radar clustering in Rust (`aav_radar_clusters`) | Turns radar points into objects; our C++ moves to Rust, with the same answers bit for bit | Ben | 2026-10-15 | sim |
| Lidar to a 2D scan (`aav_lidar` `lidar_scan`) | Nav2's costmap and stop zone need a flat scan, without the car's own body in it | Ben | 2026-10-15 | sim |

## Drivers

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| Keyboard driving (`teleop_twist_keyboard`) | Drive the turtle, Gazebo or the car from any laptop, with no joystick | Ben | 2026-10-09 | sim |
| Joystick with a deadman (`aav_teleop`) | The joystick can drive with no autonomy running, and letting go stops the car | Ben | 2026-10-09 | built |
| Scripted drives (`aav_scripted`) | Repeatable speed-up, hold and slow-down runs, for tests and the first drives | Ben | 2026-10-09 | sim |
| [Drive to point: GPS A to B](a-to-b.md) (`aav_drive_to_point`) | The Gate 4 goal: drive to a GPS point and stop within 1 m; in Gazebo it stopped within 0.4 m. Called `aav_goto` until 26 Oct: renamed so the name says the job (action `/drive_to_point`, slot `point`, `pixi run drive-to-point`) | Ben | 2026-10-15 | sim |
| [Plain English](plain-english.md) (`aav_language`) | Say "drive 10 m forward": a local model plans it, and asks before the car moves | Ben | 2026-10-15 | sim |
| RC via ROS (`rc_driver`) and a three-position mode switch | A person on the RC transmitter outranks every program, but still passes through ROS's safety checks | Ben | 2026-10-24 | built |
| [Preflight on stands](actuator-tests.md#the-preflight-on-stands) (`pixi run preflight`, `actuator-test preflight`) | One run before the car goes on the ground: it speeds up, holds, slows, brakes and steers while every sensor is checked, and says PASS or FAIL per line. Refused on the real car until a rear-wheel speed exists (TODO 49b); `--no-drive` checks the sensors alone | Ben | 2026-10-29 | sim |

## Navigation

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| Nav2 set up for a car (`aav_navigation`) | A car can't turn on the spot: SmacPlannerHybrid plans paths it can drive, with no Spin | Ben | 2026-10-09 | sim |
| [MPPI path follower](path-followers.md), Pure Pursuit as an option | MPPI reached the first goal cleanly in 4 of 4 Gazebo runs; Pure Pursuit looped in 2 of 3 | Ben | 2026-10-22 | sim |
| Reversing, with a stop zone behind the car | Backing up allows a three-point turn, instead of getting stuck facing a wall. Off by default on the car (26 Oct) until the wheel sensors tell direction (H5) | Ben | 2026-10-22 | sim |
| Obstacles in the costmap, and a stop zone (`collision_monitor`) | Radar and lidar points stop Nav2 before it hits something: 0.3-0.4 m short of a box on the bench | Ben | 2026-10-11 | sim |
| Plan around obstacles (`plan_around:=true`) | The planner sees the radar and lidar too, so it steers round a box instead of only stopping | Ben | 2026-10-22 | sim |
| [Trial path followers](trials.md) (`aav_track`, `tracker:=`; in sim, swapped live: `ros2 param set /track tracker lqr`) | To see what Stanley, LQR and MPC gain over pure pursuit on the same routes, beside MPPI and in Nav2's own chain, before any of them replaces the default; the live swap compares them on the same drive, watched in the debug HUD. LQR now linearised about the bend with a preview of the bends ahead (it lost the forward U-turn); `mpc_full` plans speed and steering together within the throttle, brake and steering limits, to drive the whole A-to-B itself | Ben | 2026-10-29 | built |
| [Trial planners](controllers.md#planners) (`planner:=` hybrid, lattice, smac2d, navfn or theta_star; the lattice on our own primitives, `pixi run lattice-primitives`) | To see whether another of Nav2's planners plans better or faster than Hybrid-A* on the same routes, scored by `controller-compare --planners`, before any replaces the default. Only hybrid and lattice keep to the turning radius, so the car refuses the other three | Ben | 2026-10-29 | built |
| [Street routes](routes.md) (`aav_routes`, Valhalla) | Turns a destination into GPS waypoints along real streets, for Nav2 to follow | Ben | 2026-10-16 | sim |

## Control

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| ego, the virtual car (`aav_ego`), with Ackermann output | Only ego talks to a backend, and every backend takes the same Ackermann message | Ben | 2026-10-09 | sim |
| Stale-command stop and a speed cap (`max_speed:=`) | The car stops when commands go stale, and stays at 1.0 m/s or less until the tests raise it | Ben | 2026-10-09 | sim |
| Car model and `vehicle.yaml` (`aav_description`) | One file holds every car measurement, so no number is copied or guessed twice | Ben | 2026-10-09 | sim |
| [Measuring map](measure-the-car.md#the-measuring-map) (`measure_map.py`) | A tape route from base_link, each step from the last mark, so the team at the car measures every `vehicle.yaml` number in order, in feet or metres (Ben's idea, 29 Oct) | Ben | 2026-10-29 | built |
| Body model: mass and curve speed | ego knows the car weighs about 181 kg, so it slows for tight turns and heavy loads | Ben | 2026-10-22 | sim |
| Jerk limit | A smoother start for the people in the car; braking is never softened | Ben | 2026-10-22 | sim |
| [Linear model of the car](vehicle-model.md) (`vehicle_model.py`) | Transfer functions and state space, to tune the loops and write the report in SYSC 3600's terms | Ben | 2026-10-24 | built |

## Safety

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| Command mux and e-stop lock (`aav_command_mux`, twist_mux) | One command wins by priority, and the e-stop locks every driver out | Ben | 2026-10-09 | sim |
| Four e-stops | The car's red button stays untouched; the joystick, `pixi run estop` and the HUD add three more | Ben | 2026-10-10 | sim |
| [Sensor watchdog](simulation.md#sensor-failures-on-purpose) (`aav_safety`) | When a sensor fails or freezes, Nav2 stops loudly and says why, instead of driving blind | Ben | 2026-10-15 | sim |
| Frozen-wheel cross-check | Wheels that stop reporting are caught by checking them against the IMU and GPS | Ben | 2026-10-15 | sim |
| HUD heartbeat and latch | The safety drivers rely on the screen, so a frozen HUD holds Nav2 until a person clears it | Ben | 2026-10-23 | sim |
| [Sensor mux and output mux](sources-and-targets.md) (`aav_io_mux`) | Simulated data can never drive the real car, and the spine won't arm while the gate is shut | Ben | 2026-10-23 | sim |
| The bridge won't arm while the safety gate is shut (`spine_bridge`, `/spine/arm`) | Arming said "armed" and did nothing while no commands could pass; now it refuses and says why | Ben | 2026-10-24 | sim |
| Orin-to-ESP32 heartbeat (serial frame version 2) | The ESP32 brakes when the Orin stops, even if the Motor Pi still sends: 111-120 ms in QEMU | Ben | 2026-10-24 | built |
| No-timeout lock (`legacy_uart_bridge`, twist_mux `no_timeout`, priority 60) | Today's ESP32 firmware keeps the last throttle if the Orin or the bridge dies, so on it only people and the actuator tests may drive, never a program | Ben | 2026-10-26 | built |
| Actuator tests stop on an unseen speed or a runaway (`aav_actuator_tests` `safety.unseen`, `runaway`) | On stands the front wheel sensors read 0 while the rear wheels spin, so the speed cap could never fire; now 4 s of driving with no measured speed, or a speed rising while the command falls, stops the test | Ben | 2026-10-29 | built |
| Stop hold (`aav_safety` `stop_hold`, twist_mux `stop_zone`, priority 21) | collision_monitor flickered between stop and go with an obstacle at its zone's edge, and the car crept on; now a stop holds until the zone stays clear for 1 s | Ben | 2026-10-29 | built |
| [Geofence](geofence.md) (`aav_safety` `geofence`, twist_mux `geofence`, priority 51) | Only the point driver's distance and time limits stopped a runaway route, and nothing at all without Nav2 (F6); now every program stops before its stopping distance reaches the fence, or when the position can't be trusted, and people still drive | Ben | 2026-10-29 | sim |
| Stand-ins stay local (`tools/ros.sh --local`) | A pretend car in the turtle, Gazebo or bench mode can never reach the real one | Ben | 2026-10-10 | sim |

## Backends

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| Turtlesim backend (`aav_turtlesim`) | A turtle that steers like the car: the first thing a new member drives | Ben | 2026-10-09 | sim |
| Gazebo backend (`aav_gazebo`) | The car in 3D with Ackermann steering, an IMU and GPS, to test Nav2 before the car | Ben | 2026-10-09 | sim |
| Today's car over UART (`aav_legacy`) | Drives the 2025-26 ESP32 through the Motor Pi, and reads its wheel encoder, before the CAN spine exists | Ben | 2026-10-09 | built |
| Uplink from the ESP32 (`legacy_uart_bridge`) | Brings the car's mode, faults, steering and brake positions and RC sticks back into ROS | Ben | 2026-10-24 | built |
| Speed loop in the UART bridge (`speed_loop.mode: pi`, `speed_control.py`) | Today's ESP32 drives open loop and brakes only standing still, so the car coasts past B; feed-forward plus PI on the wheel speed, out as throttle or brake, stops it where ego planned; a smooth stop (Autoware's design) eases the last of the braking so the car stops without a jolt, then holds it firmly | Ben | 2026-10-29 | built |
| CAN spine bridge (`spine_bridge`): Rust by default, Python as an option | ego's commands become CAN frames, decoded by the firmware's own code at both ends | Ben | 2026-10-09 | sim |
| [CARLA adapter](carla.md) (`aav_carla`) | Realistic camera pictures, once the NVIDIA computer runs CARLA | Ben | 2026-10-15 | built |

## Firmware

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| CAN contract (`aav.dbc`, `can_codec.py`, test vectors) | One file defines every frame, tested from both sides against the same vectors | Ben | 2026-10-09 | sim |
| CAN v0.2.0: wheel-speed frames, a 100 ms timeout, `wheel-core` | Puts the wheel sensors on the bus, and stops the car within 100 ms of lost commands | Ben | 2026-10-15 | sim |
| `spine-core`: the spine's logic | Boots latched, arms only on request and brakes on a command gap, so a glitch never restarts the car | Ben | 2026-10-09 | sim |
| The spine's state machine (`spine-core` `fsm`: lifecycle, safety, authority, motion, confidence) | Each question the spine answers has one small, tested place, so a new safety check adds an event instead of another flag | Arsh | 2026-10-27 | built |
| Spine self-test at boot | Nothing arms until both Jrks and the DAC answer, so a missing device is never driven blind | Arsh | 2026-10-27 | built |
| Hardware test mode (`spine-core`, VEHICLE_STATUS mode 3) | Bring-up moves one actuator at a time from a laptop (proposal 6.5), and the mode can't be left on: a stale request or anything unsafe leaves it, latched; no throttle test without a wheel speed, where its speed exit can't fire | Arsh; Ben (throttle rule) | 2026-10-27 | built |
| Spine device drivers and glue (`jrk-g2`, `ad5311`, `pca9555`, `ibus`, `mcp2515`, `spine-board`, `legacy-link`) | Each device the spine drives, tested against mocks before any board exists | Ben | 2026-10-10 | built |
| Pico 2 spine program (`boards/pico2`) | A spine on an ARM chip, with a 100 ms hardware watchdog and a fail-safe e-stop input | Ben | 2026-10-11 | built |
| ESP32 spine in Rust (`boards/esp32-umod`) | Replaces the C++ on today's control board with no new hardware; tested in QEMU | Ben | 2026-10-15 | built |
| ESP32 actuator firmware in C++ (`aav_umod_esp32`) | It drives today's car, so it stays until its Rust version is proven | the 2025-26 team | 2026-10-09 | car |
| Wheel encoder and status display Pico 2s in Rust | The 2025-26 C++ boards move to Rust, as decided, sending the same packets | Ben | 2026-10-15 | built |
| Speed loop's `smooth` profile | Today's gains overshoot about 15%; this one starts smoother, ready to try on stands | Ben | 2026-10-25 | sim |
| Speed LQI in `spine-core` (`speed_loop:=lqi`, TODO 3) | One signed acceleration for throttle and brake (never both), its gains worked out from the model on a laptop, so it can be scored against today's PI and P-only brake; the PI stays the default | Ben | 2026-10-29 | built |

## Simulation

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| [Gazebo road world](simulation.md) | Lane lines, a stop sign, an obstacle, and the car's camera, radar and roof lidar, to test the whole graph | Ben | 2026-10-11 | sim |
| Actuator model | Lag and limits between ego and Gazebo, so the simulated car answers like a real one | Ben | 2026-10-15 | sim |
| [Bench mode](bench.md) (`aav_bench`, `aav_spine_sim`) | The car's own software on one laptop, with the spine's logic and pretend sensors | Ben | 2026-10-10 | sim |
| Bench GPS that drifts (`aav_bench` `models.GpsError`) | A real receiver's error wanders slowly; fresh noise on every fix made Nav2 weave in a way the car never would (N7) | Ben | 2026-10-29 | built |
| The spine in the loop (`sim spine:=true`) | Gazebo drives through the spine's real logic, so its stops and e-stops get tested | Ben | 2026-10-15 | sim |
| Today's car in the loop (`sim esp32:=true`, `gazebo_esp32`) | Today's car runs the 2025-26 C++ ESP32, not the spine: Gazebo drives through the real UART bridge and that firmware's throttle ramp and brake, so its speed loop is tested before car day | Ben | 2026-10-29 | sim |
| `sim-check` and `bench-check` | One command proves the whole chain drives, stops and keeps its lane | Ben | 2026-10-10 | sim |
| `bench-check --weave` and `--zone-edge` | Measure the weave on a straight Nav2 drive (N7), and prove a box at the stop zone's edge can't let the car creep on | Ben | 2026-10-29 | built |
| Sensor failures on purpose (`pixi run sim-fault`) | Breaks a sensor in Gazebo, to prove the watchdog catches it | Ben | 2026-10-15 | sim |
| [Ghost obstacles](ghost-obstacles.md) (`aav_ghost`) | A pretend person only Nav2 sees, to test stopping with nothing real to hit | Ben | 2026-10-16 | sim |
| Record and replay a drive | A recorded drive runs through the EKFs again, to check them; they match within 0.03 m | Ben | 2026-10-09 | sim |

## HUD

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| HUD back end (Flask, `hud_server`): **archived 25 Oct** to `archive/2026-27-flutter-hud/` | Kept the 2025-26 touchscreen working, on our topics, until the raylib HUD replaced it (Q29) | the 2025-26 team, ported by Ben | 2026-10-10 | sim |
| Flutter HUD app: e-stop button, Cancel, camera: **archived 25 Oct** to `archive/2026-27-flutter-hud/` | The app could never reach its back end; then its e-stop stopped the bench car in 0.3 s | the 2025-26 team, changed by Ben | 2026-10-11 | sim |
| [Raylib HUD in Rust](hud.md) (`aav_hud`), with a SIMULATED banner | One program that is also its ROS node, and nobody mistakes simulated data for the car; it replaced the Flutter app (Q29) | Ben | 2026-10-15 | sim |
| GO / STOP panel | The safety drivers get one answer at a glance: may the car move, and if not, why | Ben | 2026-10-24 | built |
| Map window: offline street map and place search | A big-screen map of Nav2's plan over real streets, and a destination picked by name | Ben | 2026-10-15 | sim |
| Car screens at login | The HUD and the map start on the car's two screens by themselves | Ben | 2026-10-15 | built |
| [Debug HUD](hud.md#the-debug-hud) (`debug_hud`, `pixi run debugdemo`): the lidar's cubes in 3D, kept and fading (the occupancy-network look), every localizer's car (NDT purple, with its score and LOST in the panel), AMCL's particles, NDT's map as wireframes, every sensor from its own topic; keys 1-4 swap the sim's trial tracker; a key legend on screen. Since 29 Oct it opens as the **seeing screen**: the cubes big on the left, NDT's map from above with every localizer's car, and NDT's match (its cells as ellipses, the scan over them, the score); key L brings the sensor panel back | See what the car sees and what it thinks, side by side, to debug localization (NDT, AMCL) and the trackers without RViz; the seeing screen shows off what the lidar sees (Ben, 29 Oct: "another screen to show off the voxels") | Ben | 2026-10-29 | built |
| [Hardware panel](hud.md#the-hardware-panel) (`hw_panel`, `pixi run hw-panel`): every sensor's number from its own node with OK / STALE / NEVER, and very simple buttons (Speed +/-, steering, CENTRE, STOP, hold Space to drive, E-STOP) through twist_mux's buttons slot (45), ego and the spine; speed only on SIM and BENCH (1.0 m/s), off on the real car until TODO 49b, with the reason on screen; an e-stop, a takeover or a source change zeroes it | Test every sensor and work every actuator from one screen with simple buttons (Ben, 29 Oct: "one screen just to test all the sensors and operate all the actuators", "We need very simple buttons"), without going around the car's caps and e-stop | Ben | 2026-10-29 | sim |

## Tools

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| [Switch picker](switches.md) (`pixi run pick`) | Every swappable part (who steers, where-am-I, speed loop) as a plain-words menu that prints, saves and runs the command, so nobody has to remember the switches; `demo` and `debugdemo` take them too | Ben | 2026-10-29 | sim |
| Live graph check (`pixi run check`) | Compares the running graph with `docs/interfaces.yaml`, so a wrong topic shows at once | Ben | 2026-10-09 | sim |
| Network doctor, network map and machine survey | Tells whether a laptop can join the car's network, and records what's on each computer, read-only | Ben | 2026-10-10 | car |
| Read-only [MCP server](mcp.md) | An AI app can read the docs, contracts and live graph, but never move the car | Ben | 2026-10-10 | built |
| Car kit (`pixi run car-kit`) | Installs our Jazzy on the car's ARM computers from a USB stick, with no internet or admin rights | Ben | 2026-10-15 | built |
| [Car-day kit](car-day.md): runbook and preflight | Ten sessions at the car, in order, and a check that the laptop has everything | Ben | 2026-10-24 | built |
| [Actuator tests](actuator-tests.md) (`aav_actuator_tests`) | Measures the throttle, brake and steering on stands, and gives the numbers for `vehicle.yaml`; since 26 Oct each fit is a dead time plus a lag, says what it was measured from and to, and refuses what the data can't resolve | Ben | 2026-10-15 | sim |
| Bandwidth, CAN load and timing tests | Checks each network link has room, and finds late messages and stalls | Ben | 2026-10-23 | sim |
| [Watch from a laptop](foxglove.md) (Foxglove and Lichtblick) | One bridge and three layouts to watch any profile from a laptop | Ben | 2026-10-24 | sim |
| [A-to-B scorer](a-to-b.md) (`pixi run atob-score`) | Numbers for how well the car sped up, held its speed and stopped on B, the same for Gazebo and car runs; a tape-measured stop splits the GPS error from the control error | Ben | 2026-10-26 | built |

## Infrastructure

| Feature | Why | Who | Date | Status |
|---|---|---|---|---|
| One workspace with Pixi and RoboStack, Macs included | ROS 2 Jazzy, Gazebo, Nav2 and Rust on any Linux or Mac, with no admin rights | Ben | 2026-10-09 | sim |
| Cyclone DDS on every computer | Fast DDS delivered messages up to 0.48 s late beside Gazebo; Cyclone never over 0.05 s | Ben | 2026-10-23 | sim |
| Launch profiles and machine files (`aav_bringup`) | One command per profile (turtle, sim, bench, replay, car), and each computer picks its own parts | Ben | 2026-10-09 | sim |
| Topic contract and [topic names by area](naming.md) | Every topic's publisher, subscriber, type and QoS in one place, named so you see where it belongs | Ben | 2026-10-09 | built |
| CI: Linux and 64-bit ARM on every push, a Mac and Gazebo nightly | A break shows on the push that made it, including on the car's ARM computers | Ben | 2026-10-10 | built |
| Docs as HTML and PDF, with the Learn pages | Short pages with tables and pictures, for a dyslexic reader and new team members | Ben | 2026-10-09 | built |
| [Getting-started guide](getting-started.md) | A new laptop goes from nothing to driving the turtle in under 30 minutes (T1.1) | Arsh (arshjameel) | 2026-10-24 | built |
| The previous teams' 27 repos, imported with their history | All the older code and its history in one repo, to read and port from | Arsh (arshjameel) | 2026-10-24 | built |
| One team repo: `AAV-2025-2026/ecolo-av` | The team's own repo, keeping the whole history of both workspaces | Ben | 2026-10-25 | built |
| [CONTRIBUTING.md](../CONTRIBUTING.md) and the merge queue | One way into `main` for everyone: fork, branch, pull request, review, merge queue | Arsh (arshjameel) | 2026-10-25 | built |
