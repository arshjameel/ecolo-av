# Overview: everything in ecolo-av

The whole project on one page: what's in the repo, the stock ROS packages and plugins it
uses, the code it took from the 2025-26 and 2024-25 teams, and what's still missing. Each
section links to the page with the details. Counts checked on 14 November 2026; every
package, one per row: [Packages](packages.md). What works: [Status](status.md).

## In numbers

| What | How many |
|---|---|
| ROS packages | 51: 30 in Python, 21 in CMake (12 of them Rust nodes). Plus two Bevy crates outside colcon: the developer window and Ravensim's cameras |
| Rust firmware crates | 15 (`spine-core`, `aav-inputs`, device drivers, glue, the status lines, the firmware update), and 4 board programs: the spine on today's ESP32 and on a Pico 2, the wheel encoder, the status display |
| The 2025-26 team's firmware boards, in C++ | 3 |
| Launch profiles | 6: `turtle`, `sim`, `car`, `replay`, `bench`, `carla` |
| Computers in the machine files | 6: `orin`, `motor_pi`, `motor_pi_alone`, `localization`, `laptop`, `all` |
| Contracts | 3: the topics, the CAN frames, the car's measurements |
| Tests | `pixi run test`: 1110 on 25 Oct, 1575 on 3 Nov, 0 failures ([Status](status.md)); plus `bench-check` and `sim-check` |
| Tools | about 100 scripts and 45 `pixi run` tasks (89 before 1 Nov) |

## The folders

| Folder | What's in it | More |
|---|---|---|
| `ros/` | the ROS 2 packages, one folder per area | [Packages](packages.md) |
| `firmware/` | the CAN contract, the Rust crates, the spine's Pico 2 program, and the 2025-26 C++ boards | [Firmware](firmware.md), [Rust firmware](rust-firmware.md) |
| `tools/` | tests, the graph check, the doctor, recording, maps, the docs, the mailer, the MCP server | [Infrastructure](infrastructure.md) |
| `docs/` | these pages, the topic contract, and the history | |
| `models/` | the detector models, RF-DETR Nano (the default) and YOLO: `pixi run get models` downloads them (not in git) | [Perception](perception.md) |
| `archive/` | the older teams' repos, as they were, and the 2025-26 Flutter HUD (archived 25 Oct). Reference only: never built | [Older code](older-code.md) |
| `vendor/` | other people's source, to look things up: ROS 2 Jazzy and rclrs. Not ours, and not in git: `pixi run get vendor` downloads it | |
| `pixi.toml`, `pixi.lock` | the whole environment: ROS 2 Jazzy, Rust, compilers | [Infrastructure](infrastructure.md) |

## Our own code

Written for 2026-27. The car's defaults are still mostly configuration around stock ROS
packages, with ego, the muxes, the bridges and the safety locks our own. Since 29 Oct we
also write our own algorithms, each a trial beside its stock default (the second table).

The main ones. Every package: [Packages](packages.md).

| Package | Does | Area (`ros/<area>/`) |
|---|---|---|
| `aav_ego` | ego: one command into a speed and steering angle the car can do | ego |
| `aav_command_mux` | twist_mux's settings: the priorities and the locks | command_mux |
| `aav_io_mux` | the sensor mux and the output mux: simulated data never drives the real car | io_mux |
| `aav_scripted` | scripted drives: speed up, hold, slow down | drivers |
| `aav_drive_to_point` | drive to one GPS point and stop there smoothly | drivers |
| `aav_language` | plain English: a local model plans drive tools, you say yes | drivers |
| `aav_teleop` | the joystick's settings, and its e-stop button | drivers |
| `aav_rc` | the RC transmitter as a driver (priority 95, off until the hardware lead confirms the wire) | drivers |
| `aav_actuator_tests` | the stage 3-4 actuator tests, as a driver (priority 60) | drivers |
| `aav_navigation` | our navigation set up for a car (each node a port of a Nav2 server), and the maps | drivers |
| `aav_routes` | street routes (Valhalla) as GPS waypoints for the navigator | drivers |
| `aav_localization` | the two EKFs and GPS-to-map: settings only | localization |
| `aav_gps_heading_rs` | the heading from the GPS receiver's velocity, for `ekf_global` | localization |
| `aav_description` | the car's model and `vehicle.yaml` | bringup |
| `aav_sensors` | starts the sensor drivers, with their settings. The GPS: `ublox_gps` by default | sources |
| `aav_imu_yahboom` | the IMU driver, in Rust (Ben's rewrite of the 2025-26 C++ one) | sources |
| `aav_camera_stream` | one node per front camera: the camera Pi's stream, decoded, RF-DETR inside | sources |
| `aav_interfaces` | our messages and actions: `VehicleStatus`, `WheelSpeeds`, `Heartbeat`, `DataSource`, `ScriptedDrive`, `DriveToPoint` and more | ecolo_msgs |
| `aav_spine`, `aav_spine_bridge` | the CAN spine: ego's commands as CAN frames (`spine_bridge`: Rust by default, Python with `bridge:=python`) | can_bridge |
| `aav_legacy` | today's car: the 2025-26 ESP32 over UART, through the Motor Pi | can_bridge |
| `aav_safety`, `aav_ghost` | the sensor watchdog, the geofence, the stop hold and the joystick-alive check (twist_mux locks); a pretend person only navigation sees | safety |
| `aav_simulator`, `aav_sim_cameras` | Ravensim in Rust, the default since 8 Nov, and its cameras (Bevy) | sim |
| `aav_turtlesim`, `aav_simulator`, `aav_sim_models`, `aav_carla` | the turtle, Ravensim's car, a CARLA car (untested) | sim |
| `aav_bringup`, `aav_run_info` | the profiles, the machine files (each picks its parts, perception too), the RViz layout; which algorithms a run uses (`/system/algorithms`) | bringup |
| `aav_hud`, `aav_dev_window` | the HUD, the map window, the debug screen and the hardware panel (Rust, raylib); the developer window (Bevy) | hud |
| `aav_spine_sim`, `aav_bench` | bench mode: the spine on a laptop, and pretend sensors | sim |
| `firmware/can`, `spine-core`, `aav-inputs` | the CAN contract, the spine's logic, clean inputs | firmware |
| `jrk-g2`, `ad5311`, `pca9555`, `ibus`, `mcp2515`, `legacy-link`, `spine-board` | the spine's device drivers, the 2025-26 UART link, and the glue to spine-core | firmware |
| `boards/esp32-umod` | the spine on today's ESP32 control board, in Rust (Embassy): builds, never flashed | firmware |
| `boards/pico2` | the spine's board program for a Pico 2 (Embassy): builds, never flashed | firmware |
| `aav_lane_finder` | the lane finder: the camera's picture to `/perception/lane/geometry`, with OpenCV | perception |
| `aav_detector` | RF-DETR Nano (the default) or YOLO (`detector:=yolo`) on the camera's images, and the stop-sign watcher | perception |
| `aav_lidar` | the roof lidar's cloud as a 2D scan for the costmap and the stop zone | perception |

### Trials: ours, beside the stock default

Off unless a switch turns them on; in the simulator only, except where it says
([Switches](switches.md), [Trials](trials.md)).

| Package | Does | Area |
|---|---|---|
| `aav_ndt`, `aav_kiss_icp`, `aav_localization_health_rs` | NDT on a saved lidar map (Rust); KISS-ICP lidar odometry (stock, MIT); health checks that only warn (on by default) | localization |
| `aav_voxels` | the lidar as 0.25 m cubes for the costmap | perception |
| `aav_track`, `aav_mppi_lag` | path followers: pure pursuit, Stanley, LQR, MPC, a lag-aware MPC; Nav2's MPPI in Rust on a model with the car's lags | drivers |
| `aav_costmap`, `aav_planner`, `aav_navigator`, `aav_smoother` | Nav2's costmap, State Lattice planner, navigator and velocity smoother, ported to Rust: the defaults since 17 Nov | drivers |
| `aav_diffusion_planner` | a small model draws the path, a guard checks it, the lattice is the fallback | drivers |
| `aav_behaviour_rs` | the behaviour planner: a speed cap on navigation before a stop sign, an obstacle, the fence and the goal | drivers |

## Stock ROS packages and plugins we use

"Prefer stock" is a ground rule: these do most of the work.

| Layer | Stock package | What it does for us |
|---|---|---|
| Sensors | `imu_filter_madgwick` | turns the IMU's raw readings into an orientation |
| Sensors | `ublox_gps` (KumarRobotics) | the u-blox GPS, in Pixi on every computer: the default (Tentative, Q18) |
| Sensors | `ublox_dgnss` | the 2024-25 choice for the same GPS. It isn't in Pixi, so it runs on the car only: `gps_driver:=ublox_dgnss` |
| Sensors | `nmea_navsat_driver` | any GPS that speaks NMEA: `gps_driver:=nmea` |
| Sensors | `usb_cam` | the cameras, with the 2024-25 calibrations (Linux only) |
| Sensors | `velodyne` | the roof Velodyne VLP-16, on by default on the car; untested with the real lidar |
| Perception | `vision_msgs`, `cv_bridge`, `image_publisher` | detections, pictures, and a picture file as a pretend camera |
| Perception | Roboflow's RF-DETR (the default), Ultralytics YOLO and PyTorch (not ROS) | the detector, in the `ml` environment |
| Description | `robot_state_publisher`, `joint_state_publisher`, `xacro` | the car's model and frames |
| Localization | `robot_localization`: `ekf_node` (twice) and `navsat_transform_node` | where the car is: `ekf_local` to steer, `ekf_global` to plan, GPS into map coordinates |
| Localization | `tf2_ros` | a fixed map-to-odom frame when there's no GPS |
| Localization | AMCL, ours in Rust (`aav_amcl_rs`, since 18 Nov; stock `nav2_amcl` removed 20 Nov); KISS-ICP (not ROS, MIT) | trials beside the EKFs, off by default |
| Drivers | `joy`, `teleop_twist_joy`, `joy_teleop` | the joystick, its deadman, its e-stop button |
| Drivers | `teleop_twist_keyboard` | driving from a laptop's keyboard |
| Messages | `nav2_msgs` | Nav2's action and message types: our navigator, planner and follower serve Nav2's names, so goal senders never changed. Nav2's servers went on 17 Nov ([as it was](history/nav2-as-it-was.md)) |
| The mux | `twist_mux` | picks one command, by priority |
| Backends | `ros2_socketcan` | the CAN bus (Linux only) |
| Backends | `turtlesim` | the turtle (the simulated car is ours: Ravensim; Gazebo until 16 Nov) |
| Watching | `rviz2`, `rqt_graph`, `rqt_tf_tree` | the 3D view, the node graph, the frame tree |
| Watching | `foxglove_bridge` | Foxglove or Lichtblick on a laptop, on the car |
| Recording | `rosbag2`, with MCAP files | `tools/record.sh` and replay |
| Ready, not used yet | `rosbridge_server` | a web page could read ROS through it; nothing uses it since the Flutter HUD was archived (25 Oct) |

### The plugins inside Nav2 (until 17 Nov) and Gazebo (until 16 Nov)

| Where | Plugin | Notes |
|---|---|---|
| Nav2 planner | `SmacPlannerLattice` (the default since 1 Nov); `SmacPlannerHybrid` forward only | plans paths a car can drive: no turning on the spot. `SmacPlanner2D`, NavFn and Theta* in the simulator only |
| Nav2 controller | `MPPI` | follows the path, with the car's Ackermann model. `controller:=rpp` swaps in `RegulatedPurePursuit` ([path followers](path-followers.md)) |
| Nav2 behaviours | `BackUp`, `DriveOnHeading`, `Wait` | no `Spin`: a car can't |
| Nav2 navigator | `NavigateToPose`, `NavigateThroughPoses` | with our car behaviour trees, adapted from Nav2's |
| Nav2 waypoints | `WaitAtWaypoint` | GPS waypoints come through the same server |
| Nav2 checkers | `SimpleGoalChecker`, `PoseProgressChecker` | when a goal counts as reached; when the car counts as stuck (turning counts as moving) |
| Costmaps | `StaticLayer`, `InflationLayer`, `ObstacleLayer` (twice) | the radar's obstacle layer is on where the radar runs; the cameras' layer is off until they're calibrated (Q22) |
| Nav2 safety | `collision_monitor` | stops Nav2 for radar points in a 1 m zone ahead of the bumper. It guards Nav2 only |
| Gazebo | `AckermannSteering`, `JointStatePublisher`, `Imu`, `NavSat`, `Physics`, `SceneBroadcaster`, `UserCommands` | the simulated car, its sensors, and the world |

## Code from the previous teams

Each copy carries a `PROVENANCE.md`: where it came from, and every change since.

| Code | From | Now in | State |
|---|---|---|---|
| The Yahboom IMU driver (C++) | 2025-26 `yahboom-IMU` | `ros/sources/aav_imu_yahboom` | rewritten in Rust (Ben); the C++ went on 11 Oct |
| The TI radar driver | 2025-26 `Nav_Radar` | `ros/sources/aav_radar_ti` | ported; its settings are indoor demos |
| The battery monitor | 2024-25 `Battery-Monitor-ROS` | `ros/sources/aav_battery_victron` | ported |
| Camera calibrations | 2024-25 `cameras` | `aav_sensors/config/cameras` | used by `cameras.launch.py` |
| The HUD app (Flutter), its Flask back end and boot scripts | 2025-26 `User-Interface` | `archive/2026-27-flutter-hud/` | ported to our topics (10-11 Oct), then archived 25 Oct: the raylib HUD replaced it (Q29). Its route maths lives on in `aav_routes/route.py`, its place database in `pixi run get places` |
| The ESP32 actuator firmware (C++) | 2025-26 `AAV_cortex` | `firmware/legacy/aav_umod_esp32` | runs today's car; its WiFi password moved out of git. Its device logic is ported to the Rust drivers |
| The status display (Pico 2) | 2025-26 `AAV_cortex` | `firmware/spine/boards/sentinel-pico2`, crates `sentinel`, `hd44780` | rewritten in Rust, 15 Oct; builds, never flashed; not wired in |
| The wheel encoder (Pico 2) | 2025-26 `Wheel_Encoders` | `firmware/spine/boards/wheel-encoder-pico2`, crate `wheel-encoder` | rewritten in Rust, 15 Oct; builds, never flashed. The car's UDP encoder is off since 23 Oct: wheel speed now comes from the hardware lead's Arduino on CAN |
| Their UART and encoder protocols | 2025-26 `AAV_cortex`, `Wheel_Encoders` | `aav_legacy` | rewritten in Python |
| Lane keeping | 2025-26 `Lane_Detection_ROS2` | `ros/perception/aav_lane_geometry` | brought in: a driver on `/drivers/ai/cmd_vel` |
| Radar clustering (C++) | 2025-26 `Nav_Radar` | `ros/perception/aav_radar_clusters` | rewritten in Rust; the C++ went on 15 Oct |
| A YOLO stop-sign detector | 2024-25 `camera_ros2_ws` | `ros/perception/aav_detector` | rewritten: detections out, and a stop-sign watcher |
| The car's geometry | 2025-26 `Motor_Steering` (`Ackermann.m`) | `vehicle.yaml`, `docs/reference/` | numbers carried over, still guesses |
| The joystick's button map | 2025-26 `joy_teleop_ackerman` | `aav_teleop` | carried over |
| CARLA's axis rules | `carla-ros-bridge` (MIT) | `aav_carla` | rewritten, untested |

Replaced by stock packages instead of copied: the 2025-26 path planners, obstacle
avoidance, odometry and QNX layer. [The older code](older-code.md) has every repo.

## What's missing

| Area | Missing | More |
|---|---|---|
| The spine | a test on a real Pico 2 and a 3.3 V CAN module; the car's chip (Q10–Q13); the CAN wiring. The drivers, the glue and a Pico 2 program are written | [Rust firmware](rust-firmware.md) |
| Rust drivers | the W5500 and the display, if the board keeps them | [Rust firmware](rust-firmware.md) |
| Sensors | the camera models (Q22); the GPS model (Q18); the magnetometer in real units; a first run with the real lidar | [Status](status.md) |
| Perception | the 2024-25 team's `best.pt`; the Orin's TensorRT engine, built on the Orin after its reflash | [Perception](perception.md) |
| The car | most measurements in `vehicle.yaml` (Q1: the wheelbase and track are in, 1 Nov); the steering calibration (Q2); our full suite on its computers (13 Nov: only the localization update is installed) | [Car numbers](car_measurements.md) |
| Navigation | the real test site's map (Q21); N7 (the EKF's GPS heading jumps); the radar's real mount. Reversing on the car: on by default again since 1 Nov, where the lidar sees behind; until the wheels tell direction (H5) the speed's sign is the last command's | [Costmaps](costmaps.md) |
| Drivers | a first real joystick test; an e-stop message that can't get lost (N2) | [Starting points](team.md) |
| The HUD | the build for the car's Pi; its tile server and OSRM route data. The map database downloads with `pixi run get places` | [HUD](hud.md) |
| Simulation | CARLA: adapter ready, never run (needs the NVIDIA computer). Ravensim's cameras need a GPU. Bench mode works: [Bench mode](bench.md) | [CARLA](carla.md) |
| The team | owners (Q33), the leaked secrets changed (Q34), backups before any reflash | [Work](work.md) |
