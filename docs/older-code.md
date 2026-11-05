# The older code: what we have, and what's missing

The previous teams left 32 repos in two public GitHub orgs: `AAV-2025-2026` (the 2025-26
team, 19 repos) and `Carleton-AAV-Capstone` (the 2024-25 team, 13 repos). This page says
what each one is, what's already in this repo ("`aav`" below: ecolo-av since 25 Oct), what's worth
pulling in next, and what no repo provides. Surveyed on 10 October 2026; since 24 Oct, 27 of
the repos are in `archive/`, with their history.

### Earlier teams (found 15 October)

| Where | Team | What |
|---|---|---|
| `Carleton-AAV` (org, made Sept 2022) | 2022-23 | 2 public repos: `Legacy-AAV-Control-Code` (the 2021-22 team's Arduino scooter code, "most likely no longer of use") and `AI-Development-Scripts` (Colab notebooks for vision models) |
| `KeshwarmenMulliah/Carleton_AAV_UWB_Localisation` | 2022-23 | UWB radio localisation scripts, one person's repo |
| `CollinDang24/AAV-Wheel-Encoders` | 2025-26 | a personal copy of the wheel encoder repo (6 Oct 2026) |

No 2023-24 org has been found. A repo with more documentation is said to exist: it may
be private (an org's private repos don't show to non-members), or not on GitHub.

## In one picture

| | Repos |
|---|---|
| Already in `aav` (all or part) | AAV_cortex, Motor_Steering (its numbers), User-Interface, Wheel_Encoders, yahboom-IMU, Lane_Detection_ROS2, Nav_Radar; from 2024-25: Battery-Monitor-ROS, camera_ros2_ws, camera_Yolo_detection, cameras |
| Replaced by stock ROS packages | Navigation_PathPlanning_ObstacleAvoidance, Navigation-LocalPathGenerator-rf_cpp, Obstacle_Avoidance_logic, Odometry, Real-Time-OS, joy_teleop_ackerman |
| Looked at, then dropped (15 Oct) | Navigation-LocalPathGenerator's OSRM parser (on GitHub, main @ 1486f69; `street_router` does that job now); the HUD's place database (`pixi run get places` downloads it; back in use from 15 Oct: the map window's place search reads it); Motor-Control's `FinalRev` (below); AAV_dashboard (its README is now `docs/reference/2025-26-camera-system`) |
| Empty | Navigation-BehaviorPlanning |
| Not surveyed yet | three 2025-26 repos and seven 2024-25 repos (the tables below) |

## The 2025-26 repos, one by one

| Repo | What it is | Language, hardware | State | In `aav`? | Pull in next? |
|---|---|---|---|---|---|
| AAV_cortex | the ESP32 actuator firmware (RC override, the Pi's UART), a Pico 2 status display, a Pi serial bridge | C++ (PlatformIO, Pico SDK), Python ROS 2; ESP32, Pico 2 | working, on branch `6-pico2-aav-firmware` | partly: `firmware/legacy/`, and its UART protocol in `aav_legacy` | the Pi-side display script, if the status display stays |
| Motor-Control | the 2024-25 micro-ROS motor firmware: a fork | C++, micro-ROS; ESP32 | inactive since 2025 | no | no. What it taught us: branch `FinalRev` (4166ec0) reads the steering's position from the **steering Jrk G2's own feedback**, `jrk_steer.getScaledFeedback()`, 0-4095 mapped to -1..+1 (`src/esp32_ros_pcb/uRos_fns/uRos_fns.cpp:187`). So the Jrk may already have a position sensor wired (team-todo.md B6, audit D6, D7) |
| Motor_Steering | a Pi speed controller and UART bridge, an ESP32 add-on, sims, MATLAB steering geometry | Python ROS 2, C++, MATLAB; Pi 5, ESP32 | prototype | partly: its geometry in `docs/reference/` and `vehicle.yaml` | its 250 ms link timeout idea, for today's ESP32 (but braking, not releasing) |
| Navigation-BehaviorPlanning | a README title | none | empty | no | no |
| Navigation-LocalPathGenerator | follows an OSRM route from GPS: the current step, a local path, a speed | Python ROS 2 | prototype; nothing feeds it | no | no: `street_router` turns a street route into Nav2 GPS waypoints ([Street routes](routes.md)) |
| Navigation-LocalPathGenerator-rf_cpp | a C++ rewrite of the above | C++17 | prototype | no | no: Nav2 does this |
| Navigation_PathPlanning_ObstacleAvoidance | a GPS-goal grid planner (A*, D* Lite) with lidar obstacles | C++, PCL; Orin | prototype, with a crash bug | no | no: Nav2 does this |
| Obstacle_Avoidance_logic | a lidar costmap and braking-distance e-stop | one C++ file | can't build: no package files | no | no: Nav2's obstacle layer and collision monitor |
| Odometry | merges a SLAM pose and the IMU into `/odom` | Python ROS 2 | prototype; nothing publishes its input | no | no: robot_localization does this |
| Real-Time-OS | a QNX layer: command picking, a SQLite logger, body control, a QNX image for a Pi 4 | C, C++; Pi 4 on QNX 8 | prototype: its own README lists the watchdog, e-stop and CAN as not done | no | no: twist_mux, the spine's watchdog and `ros2 bag` cover it ("No QNX" is Tentative, Q23) |
| User-Interface | the in-car HUD: a Flutter app, a Flask ROS node, offline routing and place search | Dart/Flutter, Python ROS 2; the UI Pi | a working demo, on mock data | partly: ported, then archived 25 Oct to `archive/2026-27-flutter-hud/` (the app; `maincar` as `aav_hud`); its route maths lives on in `aav_routes/route.py` | **its place database `ottawa.db` (51 MB)**: the map window's place search reads it |
| Wheel_Encoders | a Pico 2 that counts wheel spokes and sends them over Ethernet | Arduino C++; Pico 2, W5500 | working demo | yes: rewritten in Rust (`firmware/spine/boards/wheel-encoder-pico2`, 15 Oct), and `aav_legacy` | its 3D-printed case files |
| joy_teleop_ackerman | joystick to Ackermann commands, with deadman buttons | Python ROS 2 | working | no: stock teleop replaced it; its button map lives on in `aav_teleop` | no |
| yahboom-IMU | the Yahboom IMU driver | C++ ROS 2; Pi 4 | working | yes, rewritten in Rust: `ros/sources/aav_imu_yahboom` (the C++ copy went on 11 Oct) | no |

The five not cloned here:

| Repo | What GitHub says | Status |
|---|---|---|
| Lane_Detection_ROS2 | none | the controller is in `aav`: `ros/perception/aav_lane_geometry`, a driver on `/drivers/ai/cmd_vel`. The lane finder isn't in the repo |
| Nav_Radar | none | in `aav`: `aav_radar_ti`; the clustering, rewritten in Rust, as `aav_radar_clusters` |
| AAV_dashboard | the **camera system's README** (main @ 5a26e15): CSI cameras → MediaMTX RTSP on the camera Pis (port 8554) → the Jetson's `camera_decode` (NVDEC) → `/camera/cam1/image_raw`, `cam2` → YOLOv8 on TensorRT → `/aav/stop_sign_*`; domain 10. The code itself is only on the Jetson, `~/Desktop/aav/Camera`. Plus a small web dashboard (ports 8080, 9090) | the README: `docs/reference/2025-26-camera-system` (15 Oct). The code: copy on Friday |
| Speed | an older copy of the wheel-encoder Pico program, `Wheel_Encoders.cpp` (master @ a520646) | no: Wheel_Encoders is the newer one, now in Rust |
| Navigation-GlobalMapRouter---Abandoned--- | none | abandoned |

## The 2024-25 repos

| Repo | What GitHub says | In `aav`? |
|---|---|---|
| GNSS | GNSS code for the car (u-blox, with RTK corrections) | surveyed in depth; replaced by the stock u-blox driver |
| Battery-Monitor-ROS | battery monitor ROS code | yes: `ros/sources/aav_battery_victron` |
| camera_ros2_ws | Python | yes: rewritten as `ros/perception/aav_detector` |
| camera_Yolo_detection | Python, 72 MB | no: its training script and `data.yaml` were in `aav_detector/training` until 26 Oct, unused. The trained `best.pt` isn't in it |
| cameras | publishes camera feeds to ROS 2 | its 4 calibration files: `aav_sensors/config/cameras`, used by `cameras.launch.py` |
| AutoCtrl_uRos | C++ | Motor-Control is a fork of it |
| emergency-stop | a browser e-stop | not surveyed: it may answer Q6 |
| Braking | C++ | not surveyed |
| Simulation | Python (CARLA work) | not surveyed |
| Power-Management-Documentation-2024-25 | power documentation | not surveyed: it may answer Q8 |
| AAV_board | none | not surveyed |
| AAV_ROS_Interfaces | common ROS interfaces | not surveyed |
| ROS2_test | a test repo | no |

## What no repo provides

| Area | Missing | Where `aav` stands |
|---|---|---|
| Sensors | a GPS driver that anything runs; a lidar driver; camera streaming and the stop-sign detector (both only on the car's Orin); steering feedback (only 2024-25's `FinalRev`); a fine enough wheel speed; magnetometer calibration | GPS: the stock driver, model unknown (Q18). Lidar: the stock `velodyne` driver (20 Oct). Cameras: `aav_camera_stream` (24 Oct). Both untested on the car |
| Localization | a URDF, TF, a working estimator | [[works]] the EKFs and `navsat_transform`, in sim |
| Navigation | any planner whose output reached the motors; the HUD's route into GPS waypoints; lidar or radar costmap inputs | [[works]] Nav2 in Gazebo |
| Drivers | anything beyond RC and the joystick | [[works]] scripted, keyboard; the joystick is [[untested]] |
| Ego | a command layer with limits and a timeout; calibrations from radians to the actuators' counts | [[works]] ego; the calibrations are TODO |
| The spine | any CAN code; a watchdog that brakes; feedback frames; the hardware e-stop in code | [[works]] the logic (`spine-core`); the board programs (`boards/pico2`, `boards/esp32-umod`) are [[untested]]: never flashed |
| HUD | real data: it only ever showed mock data | [[works]] our raylib HUD and map window read real topics, on a Mac; [[untested]] on the car. Their app was ported, then archived 25 Oct (Q29) |
| Simulation | a whole-car simulator | [[works]] Gazebo |

!!! warning "Back up the car before any reflash"
  Some work exists only on the car's computers: the stop-sign detector, `camera_decode`,
  the three nodes in the 2025-26 team's rqt_graph photo, and the OSRM map data. Image the
  Orin and the Pis first.

## Red flags in the old repos

The repos are public. Deleting a file doesn't remove it from the history.

| What | Where (no values here) | What to do |
|---|---|---|
| A GitHub personal access token | User-Interface: `Project_Documents/UI_notes.txt` | revoke it (Q34) |
| Two Firebase service-account private keys | User-Interface: 7 JSON files under `Database/` and `MainCar/` | revoke them (Q34) |
| The team WiFi password | AAV_cortex and Motor-Control: several firmware files | change it (Q34) |
| Build output and binaries | User-Interface (`node_modules`, 24,438 files), AAV_cortex, Real-Time-OS (a 33 MB QNX image), Motor_Steering, joy_teleop_ackerman | don't copy them into `aav` |

`aav` and its history contain none of these secrets.

## How to read the older code safely

The "In `aav`?" columns above already map each repo to its module here; each module's
card is in [Every module](modules.md). To read the originals:

| Do | Why |
|---|---|
| Read them in `archive/<repo>/` | the 27 repos came in with their history (24 Oct). Read-only: never build, fix or commit in them |
| Search them all: `grep -r <word> archive/` | every old repo at once; colcon skips the folder |
| Or on GitHub (`AAV-2025-2026`, `Carleton-AAV-Capstone`) | the originals, with every branch |
| Check the branch first: `git -C <repo> branch -a` | the working code is often not on `main` (AAV_cortex: `6-pico2-aav-firmware`; Motor-Control: `FinalRev`) |
| Start from the `PROVENANCE.md` of the module that came from it | it names the repo, branch and commit we copied, and every change since |
| When a search hits a token, key or password, close the file | write down only where it is and what kind (the "Red flags" table). Never paste it into notes, chat or an AI app |
| Don't run the old scripts or flash the old firmware from the clones | some reach the car's WiFi or flash a board; the car's own copies are what Friday's [audit](audit-friday.md) records |
| Expect the old topic names (`/cmd_vel`, `/gps/fix`, ...) | they're Humble-era; ours start with their area ([Topic areas](naming.md)) |

## Bringing more code in

| Rule | Why |
|---|---|
| Copy only source, never build output | the old repos committed `build/`, `install/`, `log/` and `node_modules` |
| Give it a `PROVENANCE.md`: the repo, branch and commit, and every change since | so anyone can trace it back |
| Prefer a stock ROS package when one does the job | most of the old bugs were in home-made versions of stock packages |
| Big data (like the 51 MB `ottawa.db`) gets a download step, not git | GitHub warns at 50 MB and refuses files over 100 MB |
| Check for secrets before the first commit | the old repos leaked three kinds |
