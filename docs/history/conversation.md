# How we got here: questions and answers

A condensed record of one long Claude Code session in which Ben (2026–27 Carleton AAV capstone team) asked Claude about the inherited code and the new ROS 2 design: every question Ben asked, with the answer given. Dates: 2026-10-09, and the night into 10 October.

Reading notes:
- **"Previous team"** means the 2025–26 team (org `AAV-2025-2026`, cloned into `/home/ben/aav/`). The 2024–25 team's org is `Carleton-AAV-Capstone`. Years are written out because early answers called 2024–25 "last year".
- Findings about the inherited code come from reading it. Nothing was run on the car.
- **"The design doc"** is the Claude Doc *AAV 2026–27 Autonomy Stack — Architecture & Plan* (https://claude.ai/code/artifact/0aa669cb-16cc-4896-b076-c17547e3c4b8).
- **Revised:** marks an answer that a later answer changed. The text shows the latest version.
- Secret values are never reproduced, only their location and type.

## Contents

- [The inherited repos and what we found](#the-inherited-repos-and-what-we-found)
- [Running ROS 2: ament, Linux versions, the Orin and Pixi](#running-ros-2-ament-linux-versions-the-orin-and-pixi)
- [Rust in ROS 2 and ros2-rust](#rust-in-ros-2-and-ros2-rust)
- [The target architecture](#the-target-architecture)
- [The design doc](#the-design-doc)
- [common_interfaces PR #233 and useful stock packages](#common_interfaces-pr-233-and-useful-stock-packages)
- [Rust on the boards and bare-metal sensors](#rust-on-the-boards-and-bare-metal-sensors)
- [The camera streaming app](#the-camera-streaming-app)
- [Which GitHub org is which, and who can see it](#which-github-org-is-which-and-who-can-see-it)
- [The rqt_graph photo](#the-rqt_graph-photo)
- [Pins: firmware or ROS](#pins-firmware-or-ros)
- [The full graph and directory structure](#the-full-graph-and-directory-structure)
- [Languages](#languages)
- [Debounce reviews](#debounce-reviews)
- [Building the workspace](#building-the-workspace)
- [Jazzy, packages, Gazebo vs CARLA, RViz and rqt](#jazzy-packages-gazebo-vs-carla-rviz-and-rqt)
- [Documentation requests and the hardware list](#documentation-requests-and-the-hardware-list)
- [Overnight into 10 October: Jazzy, the Mac and the car](#overnight-into-10-october-jazzy-the-mac-and-the-car)
- [10 October, evening: chips, Rust, Embassy and tools](#10-october-evening-chips-rust-embassy-and-tools)
- [11 October: the night's results, and what we can try at home](#11-october-the-nights-results-and-what-we-can-try-at-home)
- [15 October: topic names by area](#15-october-topic-names-by-area)
- [Open threads](#open-threads)

## The inherited repos and what we found

### Here is the git repo (github.com/orgs/AAV-2025-2026). Can you pull everything into this project and let's see what we have?

- **The clone:** all 19 repos are public and were cloned in full (every branch, 642 MB) into `/home/ben/aav/`. The last pushes were in April 2026.
- **Activity:** most active are User-Interface (172 commits, 12 contributors), Real-Time-OS (64), Navigation_PathPlanning_ObstacleAvoidance (34) and Motor_Steering (33, plus 11 of the 18 open issues).
- **Dead or stub:** Navigation-BehaviorPlanning (empty), Navigation-GlobalMapRouter (abandoned), Speed (a Pico stub), Motor-Control (a 2024–25 fork, untouched since Jan 2025).
- **Verdict:** separate prototypes, not a vehicle stack. Only manual driving (RC or joystick) reaches the motors.
- **Intended chain:** sensors → Jetson AGX Orin (plans) → QNX Pi (picks the winning command) → Motor Pi → ESP32 (actuators), plus a UI Pi. Mostly ROS 2 Humble.
- **Secrets in public repos:**
  - a GitHub personal access token in `User-Interface/Project_Documents/UI_notes.txt` (line 3);
  - two Firebase Admin SDK private keys in 7 JSON files under `User-Interface/Database/`, `MainCar/Database/` and `MainCar/UI/`;
  - the team WiFi name and password in AAV_cortex (branch `6-pico2-aav-firmware`) and Motor-Control.

  Deleting the files doesn't remove them from history: revoke the token and keys, and change the password.
- **What doesn't connect** (✓ = re-checked in source):
  - The motor code listens on `/cmd_drive` or `/driveData`, but navigation publishes elsewhere.
  - The QNX command processor is connected at neither end.
  - Motor_Steering and AAV_cortex disagree on serial format, units, pins and which way is forward.
  - ✓ The wheel encoders send to port 10111, but the Motor Pi listens on 5000.
  - ✓ The UI and QNX structs (`User-Interface/ros2_ws/src/rtos/src/message_structure.hpp` vs `Real-Time-OS/BodyControlModule/src/MessageStructures.hpp`) swap the speed/location codes, and swap latitude and longitude.
  - The display only shows mock data.
  - The IMU publishes `/imu/data_raw`, but consumers want `/imu/data`.
  - GPS is expected on four topic names and published on none.
  - Two `/local_costmap` publishers use different frames.
  - Two `/velodyne_points` subscribers expect different types.
  - Nothing reads the radar.
- **Safety:**
  - ✓ The UI clears the stop-sign e-stop every 0.5 s (`navigation_system.cpp:387-403`, `ui_flask.py:210`).
  - ✓ The planner segfaults once more than 20% of the path is blocked: `route_request_pub_` is used at `:964` but never created.
  - AAV_cortex's ESP32 has no command timeout.
  - Motor_Steering's "safe stop" releases the brake, and its forward/reverse is inverted.
  - The firmware update page has no password.
  - QNX forwards nothing if its logger wasn't started first.
- **Housekeeping:**
  - 96% of User-Interface's files are left-over `node_modules` (321 MB).
  - Repos commit ROS `build/` and `log/` folders (Lane_Detection: 500 of its 511 files).
  - A public repo holds a 35 MB QNX boot image; check the licence.
- **Unmerged work:** AAV_cortex `6-pico2-aav-firmware` (the real ESP32 firmware), Motor-Control `FinalRev`, AAV_dashboard `NEW_Dashboard_bypassing_ros2`, and User-Interface `split-display` (the only boot service file). Copy commits from Real-Time-OS `safety_watchdog`, but don't merge it: it deletes the vehicle-state module.
- **Missing from the org:** the stop-sign detector, the LiDAR driver and SLAM, the GPS driver, the camera streaming setup, and any startup config.
- **Appears to work:**
  - the RC override, and the joystick → ESP32 path (with the branch firmware);
  - the UI demo on mock data;
  - the QNX processor and logger, on a VM;
  - the encoder Pico;
  - the IMU and TI radar drivers.

> **Revised:** this and some later answers (and parts of the design doc) called the 2024–25 code "last year's". For this team, last year is 2025–26.

### Can you review each AAV package (not the Rust part): what it is, what it does, what hardware it uses and what it publishes on ROS 2? (Ben attached the team's architecture diagram.)

- **The 2025–26 car:**
  - pick a destination on an in-car touchscreen, routed over an offline OSRM map;
  - plan around LiDAR and radar obstacles, and stop for stop signs detected on the Jetson;
  - QNX to arbitrate commands;
  - an ESP32 driving throttle, brake and steering, with RC override.

  Claude read 28 ROS 2 packages (22 unique). Nothing runs end to end.
- **Computers:**
  - Jetson AGX Orin (JetPack 6);
  - Pi 4 with QNX 8;
  - UI Pi: Ubuntu 22.04, two 800×480 screens, probably 192.168.1.104;
  - Motor Pi: 192.168.1.193;
  - Pi 5 running MediaMTX;
  - a radar Pi.
- **Microcontrollers:** a SparkFun MicroMod ESP32 on a custom board, and two Pico 2s: the wheel encoder (with a WIZnet W5500) and a status display.
- **I2C actuators:** Pololu Jrk G2 for steering (0x05) and brake (0x04), a 10-bit throttle DAC (0x0C), and a PCA9555 for direction (0x20).
- **Sensors:**
  - Velodyne LiDAR (inferred from topic names);
  - TI IWR6843AOP radar (USB);
  - Delphi ESR 2.5 radar (CAN via an Arduino bridge; code missing);
  - Yahboom 10-axis IMU (USB);
  - two cameras;
  - GPS (model unknown);
  - two IR sensors on an 8-spoke wheel.
- **Human input:** a USB joystick and a FlySky FS-iA6B receiver (iBUS).
- **From code constants:** about 1.4 m × 2 m, a 1.3–1.57 m wheelbase, about ±0.5 rad of steering.
- **Notable topics:**
  - **Joystick:** publishes `/driveData` (AckermannDrive scaled to ±1, not SI units) to a serial bridge (`/dev/ttyAMA0`, 115200).
  - **Motor_Steering chain:** encoder UDP (port 5000) → `/vehicle/speed_mps` → `speed_pid` (`/cmd_drive` → `/ackermann_cmd`, with raw throttle/brake in the acceleration field) → serial.
  - **2024–25 ESP32 firmware (`FinalRev`):** runs micro-ROS and publishes `/carla/ego_vehicle/ackermann_cmd`.
  - **IMU:** `/imu/data_raw` (no orientation) and `/imu/mag` (raw counts).
  - **Radar:** obstacles go to `/navigation_objects`, which nothing reads.
  - **`obstacle_avoidance`:** publishes `/cmd_vel` as AckermannDriveStamped.
  - **Domains:** the dashboard uses `ROS_DOMAIN_ID` 10 and the UI uses 0.
  - **Never published by anything:** `/velodyne_points`, camera images, detections, GPS (`/aav/gps/fix`, `/gps`, `/gnss/fix`, `/gnss/velocity`), `/imu/data`, `/odometry`.
  - **Also:** a copied `ackermann_msgs` that would shadow the real one, and a speed-profile node in `aav_longitudinal_sim` that is effectively a scripted driver.

| Diagram box | What exists | Verdict |
|---|---|---|
| wheel+steer | encoder over UDP, speed estimator; no steering feedback or CAN | Adapt |
| IMU | `yahboom_imu_cpp` | Reuse; add `imu_filter_madgwick` |
| vehicle URDF | none, no TF at all | Missing |
| GPS/RTK | no driver | Missing; see 2024–25 `GNSS` |
| lidar, cams | stock drivers, not in repos; TI radar driver works | Use stock drivers |
| ekf_local/global | buggy `odometry_node` | Replace with `robot_localization` |
| costmap_2d | two conflicting publishers | Replace with `nav2_costmap_2d` |
| telemetry | no `/tf` | Missing |
| joystick + HUD | `joy_teleop_ackerman`; Flutter HUD (offline OSRM, 600k Ottawa addresses, mock data only) | Adapt |
| remote / scripted | video only / a speed-profile node | Partial / trivial |
| Nav2 | custom A*/D* Lite planner, OSRM follower | Replace; OSRM can feed waypoints |
| twist_mux | QNX selection, never connected | Replace; RC stays top priority in hardware |
| /cmd_vel | five Ackermann command topics, each with different units | New convention: the biggest fix |
| ego | `speed_pid` | Adapt; Twist→steering still to write |
| real car | ESP32 over I2C, two incompatible serial stacks | ESP32 has CAN built in; add a transceiver |
| CARLA / Gazebo | 2024–25 firmware drove CARLA / a brake plugin | Recover / minor |

- **Keep:** the IMU and TI radar drivers, the encoder hardware and speed estimator, the ESP32 actuator code and the HUD.
- **Replace with stock packages:** their estimator, costmaps, planner, mux and joystick code. Most of the bugs live there.
- **Write first:** a URDF with a TF tree, and a one-page interface contract (REP-103 units, REP-105 `map → odom → base_link`). Never writing one down is why nothing connects.

> **Revised:** the joystick verdict was first "adapt it to send Twist on `/cmd_vel/joy`, or use `teleop_twist_joy`". It settled on `teleop_twist_joy` publishing into `twist_mux`. Ben later set the rule that the joystick is a driver and never drives the wheels directly.

## Running ROS 2: ament, Linux versions, the Orin and Pixi

### What is ament?

- **What it is:** ROS 2's build system. It turns a folder into a package that other packages and `ros2 run` can find (a pun on ROS 1's *catkin*).
- **In Cargo terms:**
  - `package.xml` ≈ `Cargo.toml`;
  - ament (`ament_cmake`, `ament_python`, `ament_cargo`) ≈ `cargo build` for one crate;
  - colcon ≈ `cargo build --workspace`, across languages.
- **The team's packages:** 14 use `ament_cmake` and 14 use `ament_python`. Typical use:

  ```bash
  source /opt/ros/humble/setup.bash; colcon build; source install/setup.bash; ros2 run yahboom_imu_cpp imu_node
  ```
- **Don't commit `build/`, `install/` or `log/`.** They're machine-specific colcon output, with `/home/vaasu/...` paths and x86 binaries that won't run on ARM.
- **This machine:** AlmaLinux 9.7 with Rust 1.92, but no ROS 2 and no Docker or Podman.

> **Revised:** the first fix offered was Podman with `ros:humble`. Later answers recommended Jazzy everywhere, and Pixi (no admin) on this machine.

### Can you run ROS 2 on a regular Linux machine? Do we need a specific Ubuntu version? Can we build it ourselves on Red Hat or another platform?

Yes. The Ubuntu-version rule only applies to the official binary packages.

| Release | Supported until | Ubuntu (Tier 1) | Red Hat (Tier 2) |
|---|---|---|---|
| Humble (car today) | May 2027 | 22.04 | RHEL 8 |
| Jazzy | May 2029 | 24.04 | RHEL 9 |
| Kilted | Nov 2026 | 24.04 | RHEL 9 |
| Lyrical (May 2026) | ~May 2031 | 26.04 | RHEL 10 |

- **Other platforms:** Windows gets official packages. macOS, Debian and others are Tier 3: you build ROS yourself, with community support only.
- **Ben's machine** (AlmaLinux 9.7, a RHEL 9 rebuild, no admin rights) matches Jazzy. Options, easiest first:
  1. **Pixi/RoboStack:** no admin needed. Community-supported, and proposed as a co-official method since Jan 2026.
  2. **Official Jazzy RPMs:** IT enables EPEL, CRB and the ROS repo. First check that Nav2, robot_localization and twist_mux are built for RHEL.
  3. **Podman:** an admin installs it once.
  4. **Build from source** with colcon. 16 cores and 125 GB is plenty, but it needs admin for the dev libraries, plus building Nav2, robot_localization and Gazebo too. The most fragile option.
- **All machines on the car must run the same release.** Mixing Humble and Jazzy is unsupported, with crashes and misread messages reported.
- **Recommendation: Jazzy everywhere.**
  - Humble ends in May 2027, which would leave next year's team unsupported.
  - Lyrical needs Ubuntu 26.04, which the Jetson can't run.
  - The old nodes need few changes.
- **CARLA caveat:** Jazzy nodes stalled on CARLA's ROS 2 topics because of a middleware version mismatch (issue #9551). Upgrading CARLA's copy of the middleware fixed it.

### So we can upgrade the Orin to Jazzy? ("JetPack 7.2... test it before committing"?)

- **Yes, per NVIDIA's own notes.** JetPack 7.2 (June 2026; now 7.2.1, Aug 2026) supports the AGX Orin 64 GB, 32 GB and Industrial on Ubuntu 24.04. Jazzy installs normally, and Isaac ROS 4.6 is on Jazzy.
- **It's a wipe, not an upgrade:**
  1. **Back up first.** The Orin likely holds the only copy of the YOLO detector and the camera/LiDAR setup. Copy `/home/aavjetson` and the workspaces; better, take a full image with `l4t_backup_restore.sh` from the JetPack 6 kit.
  2. **Check the hardware.** A third-party carrier board needs its vendor's 7.2 package. Module 699-13701-0005-502 reportedly fails to flash, while -501 works. `cat /etc/nv_tegra_release` shows the current version.
  3. **Flash** with SDK Manager from an x86 Ubuntu 22.04/24.04 PC (not AlmaLinux), or use the dev kit's USB installer. Accept the firmware update.
  4. **Rebuild:**
     - regenerate the TensorRT engines;
     - PyTorch needs CUDA 13 builds;
     - Python moves from 3.10 to 3.12;
     - third-party drivers may lag (Stereolabs ZED, as of June).
- **User reports on the 64 GB** (not NVIDIA advisories):
  - display stuck at 640×480;
  - failed boots;
  - missing GPU frequency controls after a cold start;
  - black screen after a USB install.

  Rolling back means reflashing JetPack 6.2.
- **Timing:** do it early in term, and move the Pis to 24.04 + Jazzy at the same time (the QNX Pi has no ROS). Claude offered a backup checklist and script.

> **Revised:** "one forum user had display problems" became several user reports once checked.

### What is Pixi?

- **What it is:** a per-project package manager, written in Rust, that works like Cargo:
  - `pixi.toml` and `pixi.lock`;
  - the environment lives in `.pixi/`;
  - `pixi run` or `pixi shell` to use it;
  - packages come from the conda-forge and RoboStack channels.
- **Why it fits:** no admin needed, and it brings its own libraries, so Jazzy runs on AlmaLinux 9, Macs and Windows.
- **RoboStack's Jazzy has:**
  - ROS 2 desktop, and Nav2 with costmap_2d;
  - robot_localization, twist_mux, teleop_twist_joy;
  - the IMU filter and the Velodyne driver;
  - the Ackermann, vision and radar messages;
  - PCL, RViz and the new Gazebo.
- **Missing:** Gazebo Classic (dropped from Jazzy) and rclrs (a crate).

```bash
curl -fsSL https://pixi.sh/install.sh | sh
pixi init aav_ws -c robostack-jazzy -c conda-forge && cd aav_ws
pixi add ros-jazzy-desktop colcon-common-extensions compilers cmake
pixi run rviz2
```

- **Limits:**
  - community-supported;
  - a few GB;
  - don't mix it with a system ROS install in one terminal;
  - dev machines only. The Jetson and Pis use Ubuntu packages, and the Jetson's CUDA and TensorRT come from NVIDIA.

### Should we download the ROS 2 source into this directory too, in case we want to search it?

- **Done:** `/home/ben/aav/ros2-src/` (570 MB, Jazzy, no history).
  - `core/` holds 108 repos, including rclcpp, rclpy, common_interfaces, tf2 and launch.
  - `ecosystem/` holds 13: Nav2, robot_localization, twist_mux, teleop_twist_joy, the joystick driver, the IMU filter, Velodyne, the Gazebo bridge, the Ackermann, vision and radar messages, the PCL tools, and xacro.
- **To search team code only:** `grep -r --exclude-dir=ros2-src --exclude-dir=ros2-rust`

## Rust in ROS 2 and ros2-rust

### Maybe pull github.com/ros2-rust too?

- **Cloned:** 9 repos into `/home/ben/aav/ros2-rust/` (14 MB).
- **What it is:** rclrs, the Rust counterpart of rclcpp and rclpy.
  - A Rust package is a `Cargo.toml` plus a `package.xml` with `ament_cargo`.
  - colcon builds it through `colcon-cargo` and `colcon-ros-cargo`.
  - Rust nodes talk to C++ and Python nodes over normal topics.
- **Status:** v0.8.0 (2026-09-23) supports Humble, Jazzy, Kilted and Rolling, with pub/sub, services, actions, timers, parameters, QoS and zero-copy. There's no API stability guarantee.
- **Fit:** small, safety-relevant bridges. Some inherited bugs are the kind Rust prevents: a `memcpy` without a length check, and a −1 stored in an unsigned size.
- **But:** it doesn't fix integration, which is the real problem, and it adds a fifth language to C, C++, Python and Dart. Don't rewrite working code.

### What do you think of the ros2-rust code?

- **Verdict:** well made and maintained. Fine for a few self-contained nodes, but not the main language.
- **Strengths:**
  - an idiomatic API (a "worker" holds state shared with callbacks);
  - typed parameters;
  - active since 2017, with 62 contributors and CI on Humble and Jazzy;
  - its generator is now in Jazzy's core list;
  - all 19 thread-safety overrides carry a written justification.
- **Weaknesses:**
  - the 0.x API still breaks;
  - the default executor idles at ~28% CPU (issue #666);
  - Ctrl-C doesn't shut nodes down cleanly;
  - no lifecycle nodes;
  - `colcon test` skips Rust tests, and every package rebuilds its dependencies;
  - large messages are copied;
  - only ~43% of 259 `unsafe` blocks are documented;
  - 3–5 core people.
- **Also:** Nav2 plugins are C++ only, and fixes sent upstream need an `Assisted-by:` line disclosing AI use.

> **Revised:** this named `ego` and the CAN spine bridge as Rust candidates. The current recommendation is Python for all ROS nodes, and Rust for the spine firmware (see [Languages](#languages)).

### Wait, what? Rust is now part of Jazzy?

- **Half true.** The Rust message types are part of Jazzy; the node library (rclrs) isn't.
- **How:** the ros2-rust lead added the Rust generator to ROS 2's default generators (ros2/rosidl_core PR #12). It shipped as an official Jazzy update on 23 Feb 2026, and is also in Kilted and Rolling, but not Humble.
- **Verified:** the official `ros-jazzy-sensor-msgs` (built 11 Sept 2026) contains a crate at `/opt/ros/jazzy/share/sensor_msgs/rust/`. Your own message packages get one through colcon, and `ros-env` finds them (`use ros_env::sensor_msgs::msg::Imu;`).
- **Not in Jazzy:** rclrs comes from crates.io (`rclrs = "0.8"`), and its build plugins from pip. The official node languages remain C++ and Python.
- **Unchecked:** whether RoboStack's Jazzy includes the Rust code.

### So should I delete the stuff related to that random ROS GitHub repo?

- **Keep it,** though deleting it is harmless. It's the project that maintains Rust support for ROS 2.
- Jazzy absorbed only the generator, which `ros2-src/core/ros2-rust/rosidl_rust` already duplicates. rclrs, `ros-env`, the build plugins and the examples exist only in `ros2-rust/`.
- At 14 MB it's reference-only, since cargo fetches rclrs anyway. Keep it until the team decides what role Rust has.

### But it is the right project?

- **Yes:** the main Rust effort for ROS 2, and the only one ROS has adopted.
  - Core maintainer ahcorde merged its generator: the development version in Oct 2025, Jazzy in Feb 2026.
  - Its lead, Esteve Fernandez, maintains core Jazzy packages, including rclpy.
- **Alternatives, by GitHub stars:**
  - rclrs: 1,533;
  - r2r: 439, a legitimate alternative;
  - ros2-client: 127, pure-Rust networking;
  - rclrust: 58, archived in 2023.
- **Caveat:** still community-run, and pre-1.0.

### Will the ros2-rust package and the examples work with Jazzy?

- **Yes:**
  - Jazzy is in rclrs's CI matrix, alongside Humble, Kilted, Lyrical and Rolling.
  - Its Jazzy bindings were refreshed on 12 Sept 2026.
  - The last full run passed on Jazzy on 29 Sept 2026: build, `clippy` with warnings as errors, and unit tests.
  - Lint compiles every example. Unit tests cover all except `minimal_pub_sub`, `minimal_client_service` and `rust_pubsub`.
  - The 6 Oct failures were in CI's ROS install step, before any Rust compiled.
- **Not proven:** on the car, or under Pixi. The idle-CPU and Ctrl-C issues remain.
- Trying it through Pixi was offered. Ben: "Not yet."

## The target architecture

### Not yet: I'm still working out where we are, what we have and what we should have. How should we organize this (by hardware, or sensors / localization / drivers / mux / ego / CARLA / actuators)? What publishes on which topics, and do we use actions or services? Do we upgrade to Jazzy?

Ben's additions:
- Actuators also sense: for example, they report the steering angle.
- Use the available modules (costmap_2d, Nav2, local and global EKF) plus the existing code.
- The costmap "will be black at first".
- Ego uses Ackermann and publishes to the actuators, CARLA and maybe turtlesim.
- Test the hardware with the joystick separately from the costmap and scripted drives.
- **Phases:** GPS A→B or a scripted drive (speed up, hold, slow down) → empty costmap → pre-rendered costmap → perception next year.
- The EKF outputs feed the autonomous drivers and the HUD.

**The diagram, as Claude read it:** sensors (wheel+steer, IMU, URDF, GPS/RTK, lidar, cams) → ekf_local, ekf_global, costmap_2d → telemetry → drivers (joystick + HUD, remote, scripted, Nav2, AI) → twist_mux → `/cmd_vel` → ego → real car (CAN spine), CARLA or Gazebo.

Claude sees only this conversation, not Ben's other Claude chats. The answer, "a starting point to argue with":

- **Organize by diagram layer, not by hardware.** One repo is one colcon workspace. Hardware details stay in the sensor and backend packages, so every other layer runs unchanged on the car, CARLA or turtlesim.
  - **We write:** ego, the spine bridge, the scripted driver and two adapters.
  - **Everything else:** configuration of stock packages that exist for Jazzy.
- **Topics:**
  - **IMU:** `/imu/data_raw` + `/imu/mag` → Madgwick → `/imu/data`.
  - **GPS:** `/gps/fix`.
  - **Spine:** `/vehicle/odom` (speed and turn rate from wheel speed and the measured steering angle) and `/vehicle/status` (steering, brake, throttle, mode, faults). This is where actuator feedback goes.
  - **EKFs:** ekf_local → `/odometry/filtered` + `odom→base_link`; ekf_global → `/odometry/global` + `map→odom`.
  - **Commands:** drivers → `/cmd_vel/joy|scripted|nav` (later `remote`, `replay`, `ai`) → twist_mux → `/cmd_vel` → ego → `/ego/actuator_cmd` (AckermannDriveStamped) → the spine, or a simulator adapter.
  - **E-stop:** `/estop` (Bool) locks the mux and reaches the spine.
- **Who reads the EKFs:**
  - the scripted driver reads `/odometry/filtered`;
  - Nav2 and the HUD read both;
  - replay needs neither: it's `ros2 bag play` remapped to `/cmd_vel/replay`.
- **Format catch:** twist_mux defaults to TwistStamped, while Nav2 and the joystick node default to plain Twist, so nothing would pass the mux. Use TwistStamped everywhere: `enable_stamped_cmd_vel: true` in Nav2, `publish_stamped_twist: true` in teleop_twist_joy. The stamp also lets ego reject stale commands.
- **Interfaces:**
  - **topics** for continuous data;
  - **actions** for long, cancellable goals: Nav2 goals, and the scripted drive (e.g., "accelerate to 3 m/s, hold 10 s, stop");
  - **services** for quick requests: arming the spine, setting the GPS reference;
  - **YAML parameters** for limits, gains and addresses.
- **Launch profiles:**
  - `hw_test`: spine, joystick, mux, ego;
  - `actuator_test`: commands sent straight to `/ego/actuator_cmd`;
  - `sensors`;
  - `autonomy`.

  Each takes `backend:=real|carla|turtlesim`. The joystick is top priority, behind a hold-to-drive button, and the RC sits above everything, in hardware. Turtlesim works as a plumbing test (ω = v·tan(δ)/L), but has no GPS or IMU.
- **Phases:**
  - 0, joystick test: spine bridge and CAN firmware, ego, URDF.
  - 1a, scripted drive: needs no localization, so do it before 1b.
  - 1b, GPS A→B: IMU, GPS, EKFs, Nav2 GPS waypoints. Mostly configuration.
  - 2, empty costmap: the same setup as 1b, so the two merge.
  - 3, pre-rendered costmap: aligned to a GPS reference point.
  - 4, perception, next year.
- **Nav2 for a car:**
  - SmacPlannerHybrid with Regulated Pure Pursuit (`use_rotate_to_heading: false`), or MPPI's Ackermann model;
  - remove spin-in-place;
  - in ROS maps black means blocked, so an empty costmap is all **white**.
- **Jazzy: yes.** Humble ends in May 2027, the Orin is covered by JetPack 7.2, and Nav2 on Jazzy has GPS waypoint following. Back up the Jetson first.
- **Reuse:**
  - the Yahboom driver;
  - the speed and wheel-speed logic and the ESP32 actuator code, moved into the spine (fixing the brake-releasing safe stop);
  - later: the HUD via `rosbridge_suite`, OSRM, and the TI radar.
- **Questions for the team:**
  - Does the spine take targets, or raw throttle and brake?
  - Is QNX needed?
  - Which GPS/RTK receiver?

> **Revised:** the diagram's plain Twist on `/cmd_vel` became TwistStamped on every `/cmd_vel*` topic (Proposed in the design doc). This answer's language and simulator advice also changed later; see [Languages](#languages) and [Jazzy, packages, Gazebo vs CARLA, RViz and rqt](#jazzy-packages-gazebo-vs-carla-rviz-and-rqt).

## The design doc

### I don't think we're using QNX now. Yes, write it up, and we need all the questions. For the firmware, do the ROS nodes consume firmware? I think the spine is currently a Pi feeding the wheels directly; we're breaking that connection and building the correct thing. (Also: "only using GPS but not sure the model".)

The answer is the design doc (link above). Beyond the previous answer, it says:

- **"ROS nodes don't consume firmware."** Firmware runs on a microcontroller, with no OS and usually no ROS. A ROS node on a Linux computer translates between topics and wire messages; here that's `aav_spine`, over CAN.
  - **Today:** the Motor Pi sends serial (115200) to the ESP32, which drives the Jrk G2s and the DAC over I2C. There's no CAN.
  - **Target:** the Orin talks CAN directly to the spine microcontroller (actuators, wheel encoder, steering feedback), and the Motor Pi leaves the command path.
- **How to connect:**
  - **Recommended:** a bridge node plus our own CAN frames defined in a DBC file, with `ros2_socketcan` moving the raw frames.
  - **micro-ROS** (as in the 2024–25 firmware): needs an agent process, usually runs over serial or WiFi, and ties the firmware to the ROS version.
  - **`ros2_control`** with `bicycle_steering_controller`: a possible later refactor.
- **The spine takes targets** (m/s and rad) and runs the speed loop itself, from the wheel encoder. The Jrk G2 holds steering position, and `speed_pid` can be ported in.
- **Draft CAN map** (classic CAN at 500 kbit/s, kept in `firmware/can/aav.dbc`):
  - 0x010: e-stop, on change and at 10 Hz;
  - 0x100: drive command (speed, steering, limits, armed, counter, checksum), 50 Hz;
  - 0x180: feedback (speed, steering angle, wheel ticks), 50 Hz;
  - 0x181: status (mode, armed, e-stop, throttle, brake, faults), 10 Hz.
- **The firmware alone must:**
  - stop on a ~200 ms watchdog (ten missed frames): zero throttle, brake, hold steering;
  - treat safe stop as braking;
  - rank hardware e-stop > RC > computer;
  - ignore the computer until armed;
  - enforce its own limits;
  - check a rolling counter and checksum;
  - report at fixed rates;
  - drop the open update page.
- **CAN hardware:**
  - **Orin:** two CAN controllers but no transceiver. Add a 3.3 V one, such as the Waveshare SN65HVD230, on the 40-pin header (CAN0 = pins 29/31), or use USB-CAN.
  - **ESP32:** needs only a transceiver.
  - **Pi:** needs an MCP2515 HAT or a USB adapter.
  - **The bus:** twisted pair with 120 Ω at each end (about 60 Ω across CANH–CANL with the power off).
  - **Encoder:** wire it to the spine (the Pico has no CAN).
  - **Bench testing:** `candump`/`cansend`, and cantools or SavvyCAN.
- **Decision status:**
  - **Decided:** driver → twist_mux → ego → one backend; ego outputs AckermannDriveStamped; robot_localization, Nav2, nav2_costmap_2d and twist_mux; joystick driving without autonomy; a CAN spine replaces the Pi's serial link.
  - **Tentative:** no QNX; GPS model unknown.
  - **Proposed:** Jazzy everywhere; TwistStamped; one repo; car-safe Nav2; languages; RC as the top override, in hardware.
- **Contract:**
  - `/gps/fix` needs a timestamp, the `gps_link` frame and a covariance; the 2024–25 GPS node set none of these.
  - Convert the IMU magnetometer to tesla.
  - Remap twist_mux's `cmd_vel_out` to `/cmd_vel`.
  - Only `aav_spine` touches `/from_can_bus` and `/to_can_bus`.
  - REP-103 units: positive steering turns left.
  - `base_link` sits at the rear-axle centre; `map` is east-north-up.
  - **ego:**
    - speed comes from the forward twist (negative = reverse);
    - steering = atan(L·ω/v);
    - always fills in the acceleration and steering-rate limits;
    - clamps;
    - stops if `/cmd_vel` is older than about 0.3 s, because twist_mux never sends a stop;
    - publishes at about 50 Hz;
    - never carries raw throttle or brake.
  - **Actions and services:** `scripted_drive`, `follow_gps_waypoints`, `navigate_to_pose`, `datum`, `spine/arm`.
  - **Rules:**
    - one publisher per topic, except `/estop` and the mux inputs;
    - real timestamps;
    - chrony on every computer;
    - one `ROS_DOMAIN_ID`.
- **Testing:**
  - **twist_mux priorities:** `/estop` lock 255, joystick 100 (0.5 s timeout), remote 90, autonomous drivers 10.
  - **Releasing hold-to-drive** sends one stop, then hands back after 0.5 s; a pause lock is the alternative.
  - **Ladder:** turtlesim → CARLA → car on stands → on the ground at walking pace.
  - **Every session:**
    - test the e-stop and RC, and stop ego on purpose;
    - new profiles start on stands;
    - someone away from the laptop holds the stops;
    - start near 1 m/s;
    - `ros2 bag record` every run.
  - **CI:** ego and scripted-profile tests, a headless `hw_test` against turtlesim, and DBC checks on both sides.
- **Localization:**
  - ekf_local (`world_frame: odom`) fuses wheel speed and the IMU turn rate; ekf_global (`world_frame: map`) adds `/odometry/gps`.
  - Settings:
    - `two_d_mode: true`;
    - navsat_transform's input remapped to `/odometry/global`;
    - `magnetic_declination_radians` for Ottawa (about 12° west; take the exact figure from NOAA);
    - `yaw_offset`, `zero_altitude` and `publish_filtered_gps`;
    - for phase 3, `wait_for_datum` and `datum` at a surveyed point.
  - **Heading** comes from the magnetometer, which motors and steel disturb: calibrate it, or use a two-antenna GPS.
  - **Accuracy:** plain GPS is good to metres, RTK to centimetres.
- **Nav2:**
  - SmacPlannerHybrid's `minimum_turning_radius`: about 2.4–2.9 m from the old constants, with `DUBIN` (`REEDS_SHEPP` adds reverse).
  - Regulated Pure Pursuit with `allow_reversing: false`; MPPI later.
  - Drop Spin and BackUp from the behaviour tree.
  - With no lidar yet, skip collision_monitor and remap `cmd_vel_smoothed` to `/cmd_vel/nav`.
- **Costmaps:**
  - phases 1b–2: a rolling window with only inflation and no map (as in Nav2's GPS tutorial);
  - phase 3: adds a `map_server` static layer (an image plus a YAML file with resolution and origin);
  - phase 4: adds obstacle layers.
  - White is free, black is blocked, grey is unknown.
  - **GPS A→B:** a lat/lon list → `follow_gps_waypoints` → `fromLL` → `/cmd_vel/nav`. OSRM could produce the list.
- **Inherited:**
  - The 2024–25 `GNSS` repo reads a u-blox receiver (UBX, `/dev/ttyACM0`) with RTK from the public rtk2go caster. Use the stock u-blox driver plus `ntrip_client` instead.
  - Start the spine from AAV_cortex `6-pico2-aav-firmware`, the RC-tested version.
- **Next steps:**
  1. secrets, backups, measuring the car and photographing labels, and a team meeting on the Proposed decisions and owners;
  2. then the repo + CI, Jazzy everywhere, ego + turtlesim with a real joystick, the DBC bench test, and the URDF;
  3. then phase 0 on stands.
- **Open questions Q1–Q34:**
  - **The car (Q1–Q9),** including which computers stay. Suggested: the Orin, the spine microcontroller and the HUD screen.
  - **Spine and firmware (Q10–Q17),** including what the spine accepts. Suggested: targets.
  - **Sensors (Q18–Q22).** For Q18, the 2024–25 code handles `ESF-INS`, which only u-blox sensor-fusion receivers such as the ZED-F9R send.
  - **Software and process (Q23–Q34):** QNX, the reflash, the org, languages, steering at standstill, joystick handover, HUD, remote driving, CARLA, test rules, owners, secrets.

> **Revised:** QNX dropped. The doc lists "No QNX" as Tentative (Q23): twist_mux takes over arbitration, and hard real-time safety moves to the spine firmware and the hardware e-stop. A doc line saying collision_monitor could run "with no sources" was also corrected, because it needs observation sources.

## common_interfaces PR #233 and useful stock packages

### I'm looking at github.com/ros2/common_interfaces/pull/233 (the Jazzy docs). Is this something we can use?

- **What it is:** `geometry_msgs/VelocityStamped`, a velocity plus `body_frame_id` and `reference_frame_id`. With those, tf2 can transform it, which it can't do with TwistStamped.
- **Status:** #233 closed unmerged. Its replacement, #240, merged in April 2024, so Jazzy ships it. Jazzy also added `VelocityWithCovarianceStamped` in June 2026.
- **For us:** keep TwistStamped on `/cmd_vel*`.
  - twist_mux, teleop_twist_joy and Nav2 all use it (twist_mux's `VelocityStampedTopicHandle` carries TwistStamped).
  - robot_localization takes Odometry and TwistWithCovarianceStamped.
  - It may matter in phase 4, for other objects' velocities.

### Any modules that will help get us there faster, like robot_localization, Nav2, etc.?

All are in Jazzy. The doc already had robot_localization, Nav2, twist_mux, teleop_twist_joy, the IMU filter, ros2_socketcan and rosbridge.

| Package | Gives us | Phase |
|---|---|---|
| nav2_gps_waypoint_follower_demo | ready configs: both EKFs with GPS, Nav2 without a map | 1b–2 |
| nav2_loopback_sim | tests Nav2 settings without a simulator | 1b–3 |
| Gazebo Harmonic (`ros_gz`) | Ackermann, GPS, IMU; much lighter than CARLA | 1–3 |
| ublox_dgnss / ublox | u-blox drivers; ublox_dgnss has the NTRIP client | 1b |
| joy_teleop | joystick buttons → arm, scripted drive, e-stop | 0–1 |
| rqt_robot_steering | slider for velocity commands | 0 |
| autoware_vehicle_msgs | VelocityReport, SteeringReport, ControlModeReport (may replace our VehicleStatus) | 0 |
| robot_upstart | starts the software at boot | 0 |
| foxglove_bridge, mapviz, rviz_satellite, plotjuggler_ros | GPS tracks on satellite imagery, tuning plots | all |
| diagnostics | standard health reporting | 0+ |
| slam_toolbox | a lidar map of the test site | 3 |
| nav2_route | routes with per-segment speed limits; could replace OSRM | later |
| gscam, camera_ros, ffmpeg_image_transport | camera video into ROS | 4 |

Autoware expects HD maps and lidar localization, far more than GPS A→B needs. Borrow its message types.

## Rust on the boards and bare-metal sensors

### Can Rust cross-compile to the ESP32, the Pi, the Orin (whatever that is) or Arduino?

- **The Orin** is the car's main computer: an NVIDIA Jetson AGX Orin, a 64-bit ARM computer running a version of Ubuntu, with an NVIDIA GPU for AI and vision.
- **Orin, Pi 4/5:** yes. `cargo zigbuild` cross-compiles without Docker, but Rust ROS nodes link against the ROS libraries, so build those on the board (the Orin is fast enough).
- **Current ESP32 (Xtensa):** yes, via `espup`. esp-hal reached 1.0 in Oct 2025, but its CAN (TWAI) driver is marked unstable.
- **ESP32-C3/C6 (RISC-V):** standard Rust, with built-in USB debugging.
- **Pico 2:** `embassy-rp` or `rp235x-hal`; neither is 1.0.
- **Arduino Uno/Nano:** partly. avr-hal needs nightly Rust, and the chips are tiny. ARM Arduinos are fine.

### Should we consider bare metal with Rust to get all the sensors working top-notch?

- **Sensors: no.** They run their own firmware, with maintained ROS drivers. Sensor quality comes from timestamps, URDF mounting positions, IMU and compass calibration, and realistic EKF noise settings; bare metal doesn't help any of these.
- **Spine: maybe.** Timing and safety matter there, and it's being rewritten for CAN and the watchdog anyway. Rust with embassy fits, and one CAN-message crate shared by the firmware and the Orin's bridge means the two sides can't disagree.
- **Costs of Rust on the spine:**
  - Espressif's toolchain;
  - the unstable CAN driver;
  - the RC-tested actuator code is C++;
  - EEs mostly know Arduino C++.
- The firmware owner decides (Q10). If that's Ben: a Rust spine, and stock packages elsewhere.

## The camera streaming app

### The sensors run Ubuntu with ROS, like the camera feeds, plus some other app for streaming video. What's it called?

- **It's MediaMTX:**
  1. Two cameras feed a Pi 5 running MediaMTX, which streams over RTSP and WebRTC.
  2. The Flutter HUD plays the WebRTC stream directly, with no ROS.
  3. On the Orin, `camera_decode` pulls the stream, hardware-decodes it, and publishes `/camera/cam1/image_raw` and `/camera/cam2/image_raw` for YOLO.
- **`camera_decode`'s code isn't in any repo** (README only). It probably lives only on the Orin, so back it up.
- **The sensors don't run Ubuntu;** the small computers beside them do. In the new design the IMU and GPS plug into the Orin over USB and the lidar over Ethernet, so only the cameras keep a Pi.
- **Keep MediaMTX** for low-latency HUD and remote-driving video.

## Which GitHub org is which, and who can see it

### Is this (github.com/orgs/Carleton-AAV-Capstone) the same capstone I'm working on?

- **Yes:** the same capstone and car, from earlier. Each team made its own org:
  - 2024–25 `Carleton-AAV-Capstone`: Sept 2024, 13 repos, last updated April 2025.
  - 2025–26 `AAV-2025-2026`: Sept 2025, 19 repos, last updated April 2026. This is the one cloned.
  - 2026–27: none. `AAV-2026-2027` doesn't exist.
- **Evidence:**
  - Motor-Control forks the 2024–25 `AutoCtrl_uRos` (copied Nov 2025).
  - Both years' ESP32 firmware drove CARLA.
  - The 2025–26 code expects a GPS driver that the 2024–25 `GNSS` repo has.
  - The 2025–26 route planner starts at Carleton by default.
- **Repos that may fill gaps:**
  - `emergency-stop`
  - `Simulation`
  - `camera_Yolo_detection` (maybe the missing detector)
  - `Braking`
  - `Power-Management-Documentation-2024-25`

  Claude offered to clone all 13.

### Is Carleton-AAV-Capstone the same code we're using? I guess ours is github.com/orgs/AAV-2025-2026. Is ours private?

- **No: it's 2024–25, two years back.** The org history:
  - `Carleton-AAV`: 2022–23, 2 repos, including older autonomous-scooter code;
  - 2023–24: none found;
  - `Carleton-AAV-Capstone`: 2024–25, 13 repos;
  - `AAV-2025-2026`: 2025–26, 19 repos.
- **Little carried over** (compared file by file):
  - Motor-Control is an unchanged copy of `AutoCtrl_uRos`, and the 2025–26 team never committed to it.
  - Motor_Steering copied three message packages from it.
  - Nothing else matches, apart from ROS template files.
- **`AAV-2025-2026` is the 2025–26 team's org** (the inherited code), not the 2026–27 team's. No new org exists yet (Q25). Suggestion: one permanent org handed on each year, with the supervisor as an owner.
- **Nothing is private.** All 19 and all 13 repos are public; private ones would be invisible from outside. So the secrets are public too (Q34), and the Motor-Control WiFi password is in both orgs.
- **GitHub's free plan:** private repos get no branch protection and only 2,000 CI minutes a month. Public repos get both, but then no secret can ever be committed.
- Claude offered to replace "last year's" in the design doc with explicit years, and to add this history.

> **Revised:** the previous answer said the 2025–26 team copied the 2024–25 firmware "and kept building on it". The file-by-file comparison found Motor-Control unchanged.

## The rqt_graph photo

### This is what they currently have for ROS (rqt_graph photo). So we have a lot of work to do.

- **What's there:** no driving, sensors, position estimate or `/tf`. Every data topic is connected at one end only:
  - `/e_stop_subscriber` listens on `/trigger_estop`, which nothing sends to.
  - `/e_stop_publisher` sends on `/e_stop`, which nothing listens to.
  - `/influxdb_subscriber` logs four CARLA topics (GPS, IMU, speed, drive command) and `/battery_status` to InfluxDB, but none of them is published.
  - The rest is tooling: `rqt_gui`, `_ros2cli_daemon`, `/rosout`, `/parameter_events`.
- **The names are the 2024–25 team's:**
  - Their `emergency-stop` web page triggers if pressed, or if any browser drops; a good fail-safe idea.
  - Only a CARLA test node reacted to it. The ESP32 never listened, so it never reached the real car.
  - `Battery-Monitor-ROS` reads a Victron monitor over serial.
- **The three running nodes' code isn't in any public repo from either team** (all branches were checked). Copy it off before reflashing.
- **Limits of the photo:** rqt_graph shows only the current network and domain, and the photo is cut off on the left (one more oval).
- **The gap is smaller than it looks.** Phase 0 is about seven nodes: `joy`, `teleop_twist_joy`, `twist_mux`, `ego`, `aav_spine`, `ros2_socketcan`, `robot_state_publisher`. Only `ego`, `aav_spine` and the spine firmware are ours to write.
- **Next step:** run this on each computer and send the output, for a "running today" section:

```bash
printenv | grep -E 'ROS_DISTRO|ROS_DOMAIN_ID|RMW_IMPLEMENTATION'
ros2 node list; ros2 topic list -t
systemctl list-units --type=service --all | grep -i -E 'ros|carla|influx|estop|mediamtx'
docker ps -a 2>/dev/null
ps -eo pid,args | grep -i -E 'e_stop|influx|ros2' | grep -v grep
```

## Pins: firmware or ROS

### AAV_cortex has a lot of stuff relating to pins. Is that firmware, which the ROS nodes then consume?

- **Yes, it's firmware.** `hardware_configs.h` (on `6-pico2-aav-firmware`) sets:
  - the ESP32 pins for the RC receiver and I2C;
  - the I2C addresses: steering 0x05, brake 0x04, throttle 0x0C;
  - the RC channels;
  - calibration: actuator ranges, the throttle ramp and its cap.
- **ROS never sees it.** The path is pins ↔ firmware ↔ CAN ↔ `spine_bridge` ↔ topics. ROS knows only device names and addresses (`can0`, `/dev/ttyUSB0`, IPs), in one config file per computer.
- **The rest of AAV_cortex:** `main` holds hardware tests (reference only), and `AAV_serial_bridge` is replaced by `spine_bridge`.
- **Port the steering calibration carefully (Q2):**
  - the code steers with 0–3500 centred on 1950, using constants labelled as brake values;
  - an unused set says centre 500, while its comment says about 1000.

## The full graph and directory structure

### Let's build the full graph and directory structure and see how everything looks.

The design doc gained a section, *The ROS graph*. We write three nodes (ego, spine_bridge, scripted driver); the rest are configured stock packages.

- **The phase 0 drawing** ("ROS stops at the CAN bus; the pins belong to the firmware"):
  - joystick → `joy_node` → `teleop_twist_joy_node` → `twist_mux` → `ego` → `spine_bridge` ↔ `ros2_socketcan` ↔ `can0`;
  - → the spine firmware (watchdog, arming, speed loop, limits);
  - → steering (Jrk G2, 0x05), brake (Jrk G2, 0x04), throttle (DAC, 0x0C), FlySky iBUS on GPIO 32, and the wheel encoder (planned to be wired to the spine);
  - plus `robot_state_publisher` → `/tf_static`.
- **The phases 0–3 drawing** shows every node with what it reads (←) and publishes (→). It adds the IMU and GPS drivers, Madgwick, navsat_transform, both EKFs, Nav2, and `rosbridge_websocket` for the HUD.

```
aav/                         one git repo = one colcon workspace
├── src/
│   ├── aav_interfaces/      msg/VehicleStatus.msg, action/ScriptedDrive.action
│   ├── aav_description/     urdf/aav.urdf.xacro
│   ├── aav_bringup/         launch/{actuator_test,hw_test,sensors,autonomy}.launch.py, config/orin.yaml
│   ├── sensors/             aav_sensors (config), yahboom_imu_cpp (reused C++)
│   ├── localization/        aav_localization (ekf_local, ekf_global, navsat yaml)
│   ├── drivers/             aav_teleop, aav_scripted, aav_navigation (nav2.yaml, behavior_trees/car.xml, maps/)
│   ├── mux/aav_mux/         twist_mux.yaml
│   ├── ego/aav_ego/
│   └── backends/            aav_spine (spine_bridge, can_codec), aav_carla, aav_turtlesim
├── firmware/                can/aav.dbc, spine/
├── hud/  tools/  docs/
└── .github/workflows/ci.yml build and test on Jazzy for every PR
```

- **Python packages** also hold `package.xml`, `setup.py`, `setup.cfg` and `resource/<name>`.
- **Config-only packages** hold `package.xml`, a short `CMakeLists.txt`, `config/` and `launch/`.

> **Revised:** `firmware/spine/` was C++ PlatformIO firmware here (pins in `include/board_config.h`). It became a Cargo project once Rust was chosen. The build also added `backends/aav_legacy/` (an interim bridge to today's firmware) and `firmware/legacy/` (the inherited C++ firmware).

## Languages

### Should we be all C++, Python or Rust?

- **Python for every ROS node we write.** They're glue running at 50 Hz or less, and the hard timing lives in the firmware. Python is quickest to write, and the easiest for this year's and next year's teams.
- **Why not C++ for nodes:** it only pays off for speed or for Nav2-style plugins, and we need neither.
- **Why not Rust for nodes:** it needs rclrs, which isn't in ROS's official release, so there's extra setup and help is thin.
- **One language everywhere** would mean C++, the only one fully supported by both ROS and the existing firmware, at the cost of slower ROS work.
- The design doc's language row, Rust risk and Q26 were updated to match.

> **Revised:** this was previously "C++ for ego and the spine bridge (Rust an option), Python for the scripted driver, adapters and launch files, C/C++ for firmware". It's now "Python for every ROS node we write".

### I think I would rather Rust over C++. What do you think? (asked later, during the build)

- **Yes, for the new spine firmware:**
  - The ESP32's built-in CAN controller is supported in Rust.
  - The logic (watchdog, modes, limits, speed loop, CAN frames) sits in a small library, tested on a laptop with `cargo test` before it touches the car.
  - The inherited C++ firmware stays in `firmware/legacy/` as reference, and as a fallback until the CAN spine works.
- **ROS nodes stay Python:** rclrs isn't in the Jazzy binaries, and the nodes are simple. The only C++ left is the reused IMU driver.

> **Revised:** the spine firmware went from "C++, or Rust if its owner prefers" to Rust (`firmware/spine/`, crate `spine-core`).

## Debounce reviews

### I saw some debounce code, and I recall writing an advanced debounce. Is this debounce good?

Two routines were found, and neither is Ben's: there are no commits under Ben's name in either org.

**The AAV_cortex buttons** (`AAV_sentinel/AAV_sentinal_pico.cpp` on `6-pico2-aav-firmware`, copied into three test programs) use a 200 ms lockout after the first falling edge.
- **Good:**
  - it responds instantly;
  - `now - last` survives the 71-minute timer wrap;
  - the ISR only sets a flag.
- **Bugs:**
  - Release bounce counts as a second press if the button was held more than 200 ms, so a pin/unpin toggle flips twice.
  - Any spike counts as a press.

  It's only on the status display.

**The Wheel_Encoders code** polls two IR sensors every 1 ms, counts a spoke when A turns on, then ignores A for 10 ms. That lockout is fine: it caps counting at 100 spokes/s, about 42 km/h with the assumed 0.30 m wheel. It calls the movement reverse only if B turned on within the previous 20 ms. The problems:
- **Slow reverse counts as forward.** A spoke takes about 120 ms to pass at walking pace. Check B's state at the moment A turns on instead.
- **Speed is far too coarse.** 8 counts per turn, 0.12 m per count, reported every 100 ms: speed moves in ~1.2 m/s steps, so walking pace reads as 0 or 1.2 m/s.
- **The lockout trusts noise.**

**Better:**
- **Buttons:** sample every 1–5 ms, and accept a level only after it has held about 10–20 ms. Report both press and release (Jack Ganssle's guide).
- **Encoder:**
  - no lockout;
  - read both sensors as a pair and count every change, for 4× the counts;
  - require 2–3 stable readings;
  - compute speed from the time between counts;
  - on the Pico 2, let PIO do the counting (the official quadrature example).

  This moves into the spine firmware.

> **Revised:** this answer called the knob's rotary-encoder code sloppy. The next answer corrected that: its decoding is done well.

### I copy-pasted some 3313 code into it; I think that was my embedded class? ... Never mind, it's not. ... Maybe the C sample code.

- **No debounce in either year's repos came from sample code** (every 2025–26 branch and the 2024–25 main branches were searched). Both pieces are 2025–26 additions:
  - the knob and buttons by shabesa, 2026-02-27 ("added KY-040 pot test code");
  - the encoder by Collin-Dang, 2026-03-24 ("Converting to Arduino").

  The 200 ms lockout is the usual Pico-tutorial snippet.
- **Correction:** the knob decoder (`AAV_cortex/potentiometer_test/potentiometer_test.cpp:57`) checks each change of the two pins against a table, so bounce nets out (+1, −1, +1 = one step) with no timer. Its real problems:
  - `beep()` waits with interrupts switched off;
  - `enc_delta` is overwritten instead of added to.
- **For the spine:** reuse the table for the wheel encoder. It fixes the low-speed direction bug and gives 4 counts per spoke, but only if the two IR sensors sit a quarter of the spoke spacing apart.

### I added a simplied.c too. I know I wrote a bunch of code related to embedded systems.

Ben's files: `/home/ben/aav/simplied.c`, `c_code_example/` and `SYSC3313/`. `c_code_example/button.h` is Ben's advanced debounce, and it uses the right method: it accepts a new state only after 5 identical polls and reports releases too, which fixes both AAV_cortex bugs.

| Debounce | Phantom press on release | Ignores glitches | Other problem |
|---|---|---|---|
| AAV_cortex (200 ms lockout) | yes, if held > 200 ms | no | — |
| `simplied.c` (10 ms wait in the interrupt) | yes | no | bounces during the wait retrigger it |
| `button.h` (5 steady polls) | no | yes | window far too short as used |

- **Fix `button.h`'s window.** `main.c` polls with no timer, about 30,000 times a second by `TICKS_PER_S`, so 5 polls is ~0.17 ms, against a typical 1–6 ms of bounce. The fix:
  - have a 1 ms timer interrupt set a flag;
  - call `button_poll()` when it's set;
  - raise `DEBOUNCE_TICKS` to 10–20.

  `simplied.c` already sets up Timer_A: with the 32,768 Hz ACLK, `TA0CCR0 = 32` gives about 1 ms.
- **`simplied.c:164`:** the handler clears `P1IFG` then spins. Bounces set the flag again, so the handler runs twice: button 1's start/stop toggles back, and releases count as presses. The unused `debounce()` wouldn't help either: disabling `P1IE` doesn't stop `P1IFG` latching, and it returns true when the pin is high, which means "released" on these active-low buttons.
- **Other bugs in `c_code_example`** (the source of the `lab2-2-extra` Keil build):
  - **The LEDs never turn on** (`main.c:109`). It registers the toggle functions instead of `*_power_on_exec`, which the linker dropped, so `CURRENT_POWER` never becomes `POWER_ON`.
  - **RGB mode is unreachable** (`fsm.h:89`). The entry function comes from the next state's row, so RGB runs `red_selected_init`. Give each entry its next state, and each state one entry and one exit function.
  - **The timer fires once** (`main.c:124`). `while (TICKS--)` leaves `TICKS` at 0xFFFFFFFF, so the next event is about 40 hours away. The 1 ms tick fixes it.
- **For the car:** this layout (hardware layer, event queue, state machine, fixed-tick polling) suits the spine.
  - CAN frames and watchdog timeouts would be the events; RC override, autonomous and e-stop would be the states.
  - Debounce any switch the way `button.h` does.
  - Size arrays at compile time instead of using `calloc`.
  - Make the queue interrupt-safe.
- **The SYSC3313 labs** (threads, semaphores, message queues, sockets, periodic tasks) are the same ideas on Linux. In ROS they become nodes, topics and timers.

## Building the workspace

### Let's start building the structure and moving everything to where it belongs, for every part of the diagram.

Ben's brief, over several messages:
- **Data flow:**
  - sensors are publishing nodes;
  - the local EKF feeds the global EKF;
  - the URDF and car measurements go "somewhere in here" (part of localization?). How do we view and use them?
- **Firmware:**
  - firmware goes in its own directory, as drivers consumed by higher levels; C code might get a Python wrapper or run as C/C++;
  - define the data infrastructure and the basic firmware interface, so the firmware people can poll the hardware.
- **Command path:** ego is fed by a mux fed by drivers, and actuators publish and receive.
- **Approach:**
  - assume the code works, but check for better ROS modules;
  - publish to Nav2's spec;
  - the joystick is a driver: it publishes and doesn't drive the wheels;
  - get the node graph, file locations and topics/actions/services right;
  - prefer ROS open source, then this code, then stubs.
- **Scope this year:** a costmap drive, replay and GPS.
- **Models:** ego is Ackermann, even though Nav2 assumes something like a unicycle.
- Then document where we are, and save our thoughts in a CLAUDE.md-type file.

**The plan:**
1. read the design doc's tree and contract;
2. survey the inherited code for each part of the diagram;
3. build the Jazzy workspace with the code moved in;
4. check the node graph statically;
5. document the status.

**Already settled:**
- **The joystick:** `teleop_twist_joy` → `/cmd_vel/joy` → twist_mux. Only ego reaches a backend.
- **The firmware interface:** the CAN frames in `firmware/can/aav.dbc`, handled only by `aav_spine`.
- **Ackermann:** car-aware Nav2 plugins, plus ego's Twist→Ackermann conversion.
- **Measurements:** the URDF → `robot_state_publisher` → `/tf_static`, checked in RViz.

**Progress reported** (in `/home/ben/aav/aav/`; still running when this was written):
- **Done and tested:**
  - the folder tree;
  - the `aav_interfaces` messages and the CAN map;
  - the CAN codec: its CRC passes its standard check value, and nine test vectors were generated;
  - the Rust firmware core in `firmware/spine/`;
  - ego's maths: all 14 tests pass;
  - the CAN spine bridge, plus an interim bridge to today's firmware in `backends/aav_legacy/` (9 tests pass);
  - the turtlesim, Gazebo and CARLA adapters, and the scripted driver (6 tests pass);
  - one measurements file, `ros/bringup/aav_description/config/vehicle.yaml`, feeding the URDF, ego, Nav2 and the bridges.
- **Inherited code:**
  - copied in;
  - the legacy firmware's WiFi name and password moved from `main.cpp` into an untracked `secrets.h`, without printing the values;
  - a copied battery monitor checked: its `SyntaxError` came only from the local Python 3.9, since `VEDirect.py` uses `match`, which works on Jazzy's Python 3.12.
- **Maps:** `tools/make_map.py` generates Nav2 maps: an empty one for phase 2, and an example test lot for phase 3.
- **Still running:** a check of the Jazzy package parameters.

## Jazzy, packages, Gazebo vs CARLA, RViz and rqt

### We are using Jazzy, correct?

- Yes. Everything targets ROS 2 Jazzy on Ubuntu 24.04.

### What is a package, and how do we apply it?

- **What it is:** the unit ROS builds and installs, like a Cargo crate. It's a folder with a `package.xml` (name and dependencies, like `Cargo.toml`) plus `setup.py` (Python) or `CMakeLists.txt` (C++). `colcon build` builds them all.
- **Our code** (ego, the spine bridge, the scripted driver) is one package each.
- **Stock packages** (robot_localization, Nav2, twist_mux) install with `apt`. We only write their YAML and launch files, in small config packages like `aav_localization`.

### What's the difference between Gazebo and CARLA? What should we use? What about RViz and rqt?

- **Gazebo** (Harmonic pairs with Jazzy): a general robot simulator with physics and simulated IMU, GPS and lidar. It's light enough for a laptop, and has an Ackermann steering plugin.
- **CARLA:** an Unreal Engine self-driving simulator (towns, traffic, weather). It needs a strong GPU, and Jazzy support for its ROS 2 bridge was still being confirmed.
- **This year** (costmap drive, replay, GPS): Gazebo, plus turtlesim for quick logic checks. CARLA next year, for perception.
- **RViz:** the 3D view of the car model built from the measurements, its frames, odometry, paths and costmaps. It's how to check the measurements.
- **rqt:** small tools: rqt_graph, rqt_tf_tree, rqt_console, rqt_plot, rqt_robot_steering.
- **Foxglove** does most of both in a browser, which works well over WiFi.

> **Revised:** the doc's test ladder put CARLA second. The latest pick is Gazebo this year, and CARLA next year.

### Would it be possible to get CARLA 5 running too? Do they have the digital twin for Ottawa? (Ben, 15 Oct: "I think we want carla 5 not 4")

- **Yes, on the NVIDIA computer, not a Mac.** CARLA 0.10 (Unreal Engine 5.5, Dec 2024) is still the only UE5 release; it wants an RTX 3070 or better with 16 GB of video memory, Ubuntu 22.04 or Windows 11. No Mac build, no Mac Python client.
- **How it connects:** our own `carla_adapter`, through CARLA's Python client (TCP), not CARLA's ROS 2 link, which stalls on Jazzy (issue 9551), and not carla-ros-bridge (no Jazzy release; last commit 2022). The same adapter runs 0.10 and 0.9.16.
- **What UE5 lacks today:** only Town10 and Mine01, daylight only, no OpenDRIVE or OpenStreetMap import, no Digital Twin Tool.
- **Ottawa:** no ready-made map. We build Carleton's from OpenStreetMap, round our navsat datum, so CARLA's GPS lines up with Nav2 and the map window (`pixi run get-carla-osm`, then `osm:=`). That needs CARLA 0.9.16 (Unreal Engine 4) until UE5 imports maps.
- **State:** built and tested without a server; never run. [CARLA](../carla.md).

## Documentation requests and the hardware list

### We might want to port the code to Jazzy too, since there may be better implementations there. Add this to the docs.

- Done on 10 October: everything builds and runs on Jazzy, and every config key was checked against the running Jazzy nodes. The fixes are in `docs/history/audit-2026-10-10-*.md`; [Status](../status.md) says what works.

> **Revised:** this was "not answered yet"; the port was done overnight.

### We need documentation of everything we've talked about and decided

The request covered:
- the software in use, and the Jazzy and Ubuntu versions;
- compiling from source, so a Mac can join the network;
- the supported languages;
- running the simulator;
- the ROS modules used;
- the costmap's real-world dimensions;
- whether the global costmap means a real road map;
- each file's format, with examples such as an empty costmap;
- everything wired to the diagram.

Ben's notes file (`/home/ben/aav/text`, lines 22–27) adds the URDF, what upgrading everything to Jazzy takes, and "how do we build on different platforms... compile on that platform or just know the target? Pi on Ubuntu and an ESP32 — how do we target the nodes?"

Done on 10 October: the pages in `docs/html/` and the PDF (`pixi run docs`) cover each point. The short answers, from the sections above:
- **Versions:** Jazzy on Ubuntu 24.04.
- **Macs:** macOS is Tier 3, but Pixi runs Jazzy on Macs.
- **Languages:** Python nodes, Rust spine firmware, the C++ IMU driver.
- **The global costmap:** a rolling window with no map in phases 1b–2, and a drawn, GPS-pinned map of the test site in phase 3.
- **Formats:** map image + YAML, YAML parameters, `.launch.py`, xacro URDF, DBC.
- **Building per platform:** plain Rust cross-compiles with `cargo zigbuild`, Rust ROS nodes build on the board, and the ESP32 uses `espup`. Per-computer settings live in `aav_bringup` (e.g., `config/orin.yaml`).

### We have a bunch of Pi nodes, two ESP32s I believe, and an Orin in the centre. We can run stuff anywhere as needed.

- Not answered yet. Related: Q9 suggests keeping only the Orin, the spine microcontroller and the HUD screen. In the new design, only the cameras keep a Pi.

### Everything we talk about should be in docs, so we know how we got there; your answers scrolled by too fast.

- This file is that record. The design doc holds the decisions.

## Overnight into 10 October: Jazzy, the Mac and the car

### Is `aav_turtlesim` different from the regular turtlesim?

- No, it's the stock turtlesim. Our `turtlesim_adapter` steers it with the car's bicycle model, so like the car, it can't turn on the spot. ([Run it](../run.md))

### We talked about Python for the nodes and C++ for the rest. Is that still the case? Should everything be Rust? Which languages are used now?

- **Now:** Python for every ROS node we write, Rust for the spine firmware, and C++ only for the reused IMU driver. Nav2, the EKFs and twist_mux are C++, but they're stock: we only configure them.
- **All Rust?** Not yet. Nav2 and robot_localization stay C++ whatever we pick, launch files stay Python, and `rclrs` is a community library still at 0.x. Port a node where Rust pays off, such as ego, once the car drives.
- **Rust works here:** `examples/rust-listener` (rclrs 0.8 from crates.io) builds with plain `cargo` and printed ego's commands live. ([Languages](../languages.md))

### Can it compile on a Mac without Homebrew?

- Yes. Pixi installs ROS 2 Jazzy from RoboStack, with no Homebrew and no admin rights. The same `pixi.toml` resolves for Apple Silicon and Intel Macs (macOS 14 or newer). The one Linux-only line found, a serial speed in the IMU driver, is fixed. Not yet tried on a real Mac. ([Your Mac](../mac.md))

### Can you get it all compiled on my machine? If not, why not? What are we missing?

- All 19 packages build on oirp-spss (AlmaLinux 9, no admin) with Pixi, and `pixi run test` passes.
- Missing: the GPS driver `ublox_dgnss`, which Pixi doesn't have (B4, a team decision), and foxglove_bridge, which needs a newer glibc than AlmaLinux 9 has. ([Status](../status.md))

### What is ROS, how do we extend it, and how do we use it?

- ROS 2 is a communication system (nodes talking over topics, services and actions), plus a library of ready-made modules such as Nav2 and robot_localization. We configure the modules, and write small nodes only where nothing stock fits: ego, the scripted driver, the bridges. ([ROS 2](../ros2.md))

### Could cargo pull the Rust messaging layer, as part of the normal build?

- Yes. `rclrs` and `ros-env` come from crates.io, and the message crates for every package, ours included, ship in the Jazzy install. ([Languages](../languages.md))

### The code sits on a Linux machine, and I can't show its windows on my Mac. Compile it for AlmaLinux, or pull it to the Mac?

- Both. It builds and runs headless on oirp-spss, and the same repository goes to the Mac as a git bundle (`/home/ben/aav/aav.bundle`). The Mac runs it with Pixi; Ubuntu in a UTM virtual machine is the fallback. ([Your Mac](../mac.md))

### For now, how do I finish this on a Mac, or at least a VM, and join the car's network, to drive the car and see everything from my MacBook?

- **Set up:** install Pixi, clone the bundle, then `pixi install` and `pixi run build`.
- **Join:** the car's WiFi, the team's `ROS_DOMAIN_ID`, then `pixi run doctor`, which says what to fix.
- **Drive and watch:** `pixi run keyboard` and `pixi run rviz`. Lichtblick watches with no ROS on the Mac at all.
- **First, the car:** it must run this stack on Jazzy. Today it runs the 2025–26 software on Humble, and the two don't mix. Pixi can put Jazzy on its Ubuntu 22.04 without a reflash. ([Platforms](../platforms.md))

### The chat at claude.ai (fe09d020…) has the final design: the sensors publish, the local EKF injects, then the global EKF reads the local one and the GPS.

- Claude can't open claude.ai chats, so the key parts need pasting in.
- The dual EKF described is how it's built: `ekf_local` fuses the wheels and the IMU (odom → base_link), and `ekf_global` adds GPS through navsat_transform (map → odom). In effect the global one builds on the local one. It runs in Gazebo. ([How it fits](../how-it-fits.md))

### We have numbers to validate. Can we build a Gazebo test world?

- Built: `aav_gazebo` with two worlds, `empty` and `test_lot`. In the empty world, Nav2 reached goals 8 m and 9.5 m away. The car's numbers in `vehicle.yaml` are still guesses until someone measures the car. ([Simulation](../simulation.md))

## 10 October, evening: chips, Rust, Embassy and tools

### Does the Gazebo sim have ego in the middle? Are we still downloading? What happened to auto-compact at 50%? (16:34)

- **Ego:** yes. In `pixi run sim`, every driver goes through twist_mux and ego, then `gazebo_adapter` and Gazebo's Ackermann steering. Only the step after ego changes between Gazebo, the bench and the car.
- **The download:** finished at 16:13, merged and tested.
- **Auto-compact:** a Claude Code setting in each computer's `~/.claude/settings.json`; only the server had it. Claude copied the server's file to the Mac.

### I'll put you on the network with the hardware at some point, to audit it yourself. (16:42)

- The plan: read-only first. Find the devices, take each computer's inventory over SSH, copy what only the Orin has, compare with the contract, then report. Nothing that drives, arms, flashes or reboots without Ben's OK.
- Done beforehand: the stand-ins (`turtle`, `sim`, `bench`, `view`) stay on their own computer, so a pretend car can't reach the real one ([Simulation](../simulation.md)).

### Can the bare-metal chips (ESP32, Pico 2, STM32, Arduino) still join ROS 2, if we don't use rust-ros? (16:55)

- **Yes, and they already do.** The chip never runs ROS. It talks over a plain wire, and a small ROS node on a Linux computer translates: UART to `legacy_uart_bridge`, UDP to `encoder_udp`, and CAN to `spine_bridge` next. ([Firmware](../firmware.md), with a picture.)
- **rclrs can't run on those chips anyway:** it needs a full operating system.
- **micro-ROS** puts a ROS library in C on the chip, but still needs an "agent" on Linux: a bridge either way. The 2024-25 team's Motor-Control used it; the 2025-26 team moved to the UART bridge.

### For the hardware interfaces, I'd like Rust instead of C or C++. Is that possible? (17:03)

- **Yes: it's already the rule** (Ben, 10 Oct). The spine's device drivers are written in Rust: the Jrk G2s, the throttle DAC, the expander and the RC receiver (98 tests).
- **On the microcontrollers:** `no_std` Rust, against embedded-hal 1.0.
- **On Linux:** a Rust ROS node with rclrs works for a device on USB or serial; Ben's `aav_imu_yahboom` is one. Pins on a Pi's header work too, through `rppal` or `linux-embedded-hal`, with the same drivers.
- **Still C or C++:** the stock ROS drivers we reuse (the GPS, the cameras, the radar), and the 2025-26 boards until each one's Rust version replaces it.

### Does Rust compile for Arduino, the ESP32, the Pico, a Pi 4 and a Pi 5? We still want ROS 2; we just want the low-level interfaces handled safely. (17:03)

- **Pi 4, Pi 5 and the Orin:** yes, ordinary Rust on Linux. They run ROS 2 nodes, in Python or in Rust.
- **ESP32:** yes. Today's (Xtensa) needs Espressif's `espup` toolchain; the C3 and C6 (RISC-V) use standard Rust.
- **Pico 2:** yes. All seven firmware crates already build for it in `pixi run test`.
- **Arduino:** the ARM boards, yes. The Uno and Nano (AVR) only partly: nightly Rust, and tiny chips.
- **The split:** ROS 2 runs on the Linux computers; the chips handle the low-level parts in Rust and talk to ROS through bridges.

### Embassy sounds interesting. Where does it fit, with Linux staying on the Pi and the Orin? The sensors and actuators should poll the hardware in Rust, and the consumers should be ROS 2 nodes. (17:03)

- **Embassy is for microcontrollers only:** it's an async framework for chips with no operating system (Pico 2, STM32, ESP32). On Linux, Rust uses threads or Tokio instead.
- **Where it fits:** the spine (the RC receiver, the e-stop, `spine-core` every 10 ms, the actuators, CAN), the wheel encoder, the status display, and any new sensor board.
- **Ben's idea is the design:** an Embassy board polls at a steady rate, sends each reading over CAN or Ethernet (UDP), and a bridge node publishes it as a standard ROS message. robot_localization subscribes to the topic, like any other.
- **Which wire:** CAN for the spine (short messages that must get through); UDP over Ethernet for sensor boards; USB serial for devices that plug into a computer.
- **Next step, when a new board is designed:** one shared Rust crate per wire format, as `spine-core` does for CAN, so the firmware and the bridge can't disagree.

### Should we create a vendor directory for ros2-src and ros2-rust, with their examples? (17:03)

- **Yes, done:** `vendor/` in `aav`. Only the lists are in git: `vendor/ros2-src.repos` (121 repositories) and `vendor/ros2-rust.repos` (9), each at an exact commit. `pixi run vendor` downloads them again.
- `vendor/COLCON_IGNORE` keeps them out of the build, and `tools/test.sh` checks it.

### Can we add an MCP server to this project, for Claude or another LLM? (17:03)

- **Yes.** A small program that an AI app starts on this computer, offering tools it can call: read the docs and the contract, list the live graph, read one message, run the tests.
- **Safety first:** read-only, on this computer only. Nothing that publishes to the car. Later, bench-only actions, through the `--local` stand-ins.
- **Built (10-10, 20:3x):** `tools/mcp_server.py`, with 9 tools. See [AI apps (MCP)](../mcp.md).

### Put a CLAUDE.md in the root that explains what everything is and what's ours, so we don't waste time on old code. (17:03)

- **Done:** `~/Code/aav/CLAUDE.md`. It maps every 2025-26 folder to what replaced it in `aav`, lists the other folders, and repeats the rules: work only in `aav`, never copy the leaked secrets.

### "Shall I add the slim per-machine environments (Orin, Motor Pi, laptop) to pixi.toml?" What does that actually mean? (17:03)

- **Today:** one big Pixi environment, about 4 GB: ROS 2 desktop, Gazebo, Nav2, RViz, Rust. Every computer gets all of it.
- **Slim environments:** one per computer, with only the packages its machine file runs. The Orin skips Gazebo and RViz; the Motor Pi gets little more than Python ROS and the two legacy bridges; a laptop gets the viewing tools.
- **When:** when the first car computer gets Pixi (Phase 0). On Macs nothing changes.

### Add the first-day steps to a README in aav/aav. (17:03)

- **Done:** `README.md` has the six steps, how long each takes, and the same steps as commands.

### The scp seems done, but what about the last file? (17:03)

- It didn't finish by itself: Claude stopped it at 16:13, in the middle of a file. It was copying the server's Linux build output, which a Mac can't use.
- The half-copied file is in `~/Code/aav/aav-linux-build-output/`, with the rest of that output. Everything that matters came by rsync and was checked: 0 files missing, 0 different.

### Where does the code live, and how are commits credited? (17:18 to 17:51)

- **On GitHub:** `git@github.com:zander1976/aav.git`, private (Ben, Q25). CI runs `pixi run test` on Linux for every push, and on a Mac every night.
- **A school project:** commit messages give the intention, with no attribution lines. The co-author line came off all 108 commits before the first push (20:24). The README's Credits section names the previous teams, the open-source projects, and Claude as the AI assistant.
- **Before the push:** a scan of all 100 commits found no secrets. `~/aav`, the Linux build output and the old bundle were deleted, and Ben's Rust IMU driver was reviewed and committed.

### When I say CUDA on a Mac, I mean MLX or Metal, not CUDA. (17:54)

- **Done:** the detector's `device` setting now means "the GPU this computer has". `auto`, `gpu`, `cuda`, `metal` and `mps` all pick CUDA on the Orin, and Metal on a Mac; `cuda` on a Mac used to fail. Checked on Ben's Mac with its own PyTorch. ([Perception](../perception.md))
- **MLX** isn't needed for YOLO: Metal, through PyTorch or Core ML, is the Mac's fast path. It could join the `ml` environment later, for training or other models.

### Is `best.pt` the weights? And what's the picture with the stop sign? (18:04)

- **`best.pt`:** yes, a trained YOLO model's weights, in a PyTorch file. Training saves `best.pt` (the round that scored best on the test pictures) and `last.pt`. This one is the 2024-25 team's: one label, `Stop_Sign`, trained on their own pictures. It was on one student's computer (`~/Yolo/Capstone/runs/detect/train7/weights/best.pt`) and never went into a repo.
- **Not needed to start:** the stock model knows stop signs. When `best.pt` turns up, it drops in: our detector reads `Stop_Sign` and `stop sign` as the same label. (`models/README.md`)
- **The picture** (`docs/img/bench-stop-sign.png`) is a drawing, not a photo. The bench's pretend camera draws the road and the sign from where the model car is, and the stock YOLO model found the sign: "stop sign", 90% sure. `bench-check --perception` saved it when the car stopped, about 5 m before the sign. The Perception page now explains it in a table.
- **The rest of that line:** the server work is done (Ben's IMU work and three launch edits, committed from the Mac: dfaec15). The three firmware decisions still wait on the team: the steering on an e-stop, the brake after boot, and a brake on the RC gear switch.

### "Missing: the spine's board code, Rust drivers, a GPS driver inside Pixi, …": what is this? (18:08)

- **Where it's from:** the 13:59 overview email's one-line summary of what still stands between us and a car that drives itself. The Overview page keeps the current list, under "What's missing".
- **The spine's board code:** the spine's logic is written and tested on a laptop; the program for a real chip waits on the chip (Q10–Q13) and the CAN wiring.
- **Rust drivers:** mostly done since that email (10 Oct, 98 tests): the Jrk G2s, the throttle DAC, the PCA9555 and iBUS. Left: the W5500 and the display, if the board keeps them.
- **A GPS driver inside Pixi:** `ublox_dgnss` isn't in RoboStack, so it would run on the car only. Keep it, or switch to `ublox` (Q18).
- **The car's real measurements:** `vehicle.yaml`'s numbers are 2025-26 guesses (Q1), and the steering isn't calibrated (Q2).
- **The test site's map:** Nav2 uses an empty map today; the site is Q21, then `tools/make_map.py` draws it.
- **The HUD's port:** the back end is ported; the app needs small changes and its 51 MB map database (Q29).
- **Fixed on the way:** the Overview still said Ben's Rust IMU driver wasn't committed. It now lists it with our own code, and its numbers match today's run: 25 packages, 199 `colcon test` tests, 24 tasks.

### "Please continue." (20:19)

- **The push:** done at 20:24. First the co-author line came off all 108 commits: only the messages changed, and every commit kept its files, author and date. The 4 commit IDs the docs quote were updated, and a new scan of all 109 commits found no secrets. `main` is on GitHub at `zander1976/aav`, and CI runs on every push.
- **The MCP server:** built, with 9 read-only tools, a `.mcp.json` for Claude Code, and its page, [AI apps (MCP)](../mcp.md). Its smoke test starts it the way an app does, and reads a live topic.
- **The board survey and the emails:** the survey restarted after the session limit. The emails wait for the VPN.

### "What was that question? It doesn't seem to make sense. I said yes." (20:48)

- **The "go" question (17:54):** Ben had said yes to the strip (17:18) and asked for the push (17:51). An interruption just before the strip made me stop and ask for a "go". One question too many: from now on, an interruption with a different question doesn't cancel a yes.
- **Not a pop-up:** no new Claude Code session asked about the `aav` server, the Mac's firewall is off, and the Mac's normal SSH agent asks nothing.

### Is "the spine" from what I said about martial arts? The car already has an e-stop: don't break it. (20:51 to 20:53)

- **The name:** a nickname from the design doc (9 Oct), not from Ben's theory. The computers are the brain, the motors the muscles, and the board between them the spine, with one reflex: it brakes by itself if the computers go quiet for 200 ms. The usual industry names are "drive-by-wire controller" and "vehicle control unit" (VCU). Renaming is cheap until the board code exists: Ben's call.
- **The e-stop (Decided, Ben):** the car's existing e-stop stays exactly as it is. The spine may read it, but never sits in its path.
- **Found so far:** today's ESP32 code reads no e-stop button, so it's wired outside the code. The 2025-26 status display shows "!!! E-STOP !!!" when told to, over I2C; what tells it is still unknown. Q6 (what kind, what it cuts, the wiring) waits on Ben.

### I haven't had an email for a long time. That PDF is amazing! (20:54)

- **The emails are queued, not lost:** Carleton's mail server only takes them on campus or through the VPN. A sender keeps trying, and they go out within a minute of the VPN coming on.
- **The PDF:** 28 pages, with the new MCP page. `pixi run docs` rebuilds it.

### CARLA 4 or 5 for Jazzy? Do we even need it, with Gazebo and turtlesim? (20:54)

- **"4 or 5" are its two Unreal Engine versions:** CARLA 0.9.16 (Sept 2025) uses Unreal Engine 4, CARLA 0.10 (Dec 2024) Unreal Engine 5. Its built-in ROS 2 link stalls on Jazzy, and the fix was closed without merging (26 May 2026). Neither runs on a Mac.
- **Parked (recommended):** Gazebo and bench mode already run the whole car graph. CARLA mainly adds realistic streets for testing cameras. It could come back on the NVIDIA Linux machine (Q31). ([Simulation](../simulation.md))

### Get the firmware requirements into vendor/, so everybody can build out of the box. (20:54)

- **Ben's yes covered three questions:** the Jrk G2 driver and the iBUS parser (already written, 98 tests), a spare Pico 2 to test the spine on a desk (still open), and an ARM chip, a Pico 2 or an STM32, instead of the ESP32 (Tentative until the team confirms Q10–Q13).
- **Done:** a `firmware` environment with PlatformIO, its downloads in `vendor/platformio/` (about 1.5 GB). `pixi run -e firmware legacy-esp32` builds today's ESP32 firmware. It needed two fixes: the build looked for the framework in a folder named after one student's computer. From a clean checkout on Ben's Mac: 48 s, RAM 14.3%, flash 63.4%.
- **Later:** the tools to flash an ARM chip (`probe-rs`, a UF2 converter) aren't on conda-forge; they come with the first board program.
- **For the chip choice:** an STM32 has CAN built in; a Pico 2 needs an add-on CAN chip, such as an MCP2515.

### Do we need NVIDIA's Isaac ROS for inference? Does it work on Jazzy? Is there a Mac equivalent, with Metal? (20:55)

- **Not for inference:** our detector already uses TensorRT on the Orin, through Ultralytics.
- **Jazzy:** Isaac ROS 4.6 (18 Aug 2026) is the last release for Jazzy, on the Orin with JetPack 7.2. Isaac ROS 5.0 (21 Sept 2026) moved to ROS 2 Lyrical.
- **Later, maybe:** nvblox (a costmap from cameras), visual SLAM, stereo depth. In NVIDIA's Docker containers, not Pixi.
- **Mac:** no equivalent, since it needs CUDA. On a Mac our own nodes do the job, with PyTorch on Metal or Core ML. ([Perception](../perception.md))

### What do the Picos, the actuator controllers and the Raspberry Pi do? Are they all on a board with the ESP32? (20:55)

- **The ESP32** is the actuator controller: steering, brake, throttle, forward/reverse, RC override. Serial to the Motor Pi; I2C to the two Jrk G2s (steering 0x05, brake 0x04), the throttle DAC and the expander.
- **The Pico 2s:** one counts wheel spokes and sends them over its own Ethernet chip; the "sentinel" is a status display and buzzer on I2C (0x42), which nothing talks to yet.
- **The Motor Pi** runs the ROS bridge to the ESP32.
- **One board?** Probably a carrier board the ESP32 plugs into, but the code can't show the circuit board. A photo would settle it. The board survey that was looking for one stopped unfinished at 20:48, when a new message came in.

### The firmware and car environments: do we still need them? Maybe put them in vendor? (20:55)

- **The `firmware` environment:** yes, while the 2025-26 boards run the car. Done; its downloads go in `vendor/`.
- **The car environments** (the Orin, the Pis): yes, later, once the car's computers run Jazzy (Phase 0).
- **Espressif's installer:** no longer needed: the spine is going to an ARM chip, and PlatformIO brings its own ESP32 tools.

### There's a Linux machine with an NVIDIA card. The weights are likely there too. (20:55)

- **Recorded:** that answers question 4 of the ML email (a Linux PC for training). It isn't the design doc's Q4, the throttle, which is still open.
- **`best.pt`:** probably on that machine, with the training pictures (`~/Yolo/Capstone/yolo_data`). Open: which machine it is, and access to it.

### Video on the Orin or my M4 Max? Open models better than the students'? Both models on the network, ML and lidar into a costmap, then Nav2: is that right? (20:55)

- **On the Mac: yes.** Our detector already runs there on Metal, 7.7 ms a frame. Over WiFi, the cameras should also publish compressed images.
- **Better models: probably.** The students trained one label on their own pictures. To try: YOLO11, RT-DETR, RF-DETR; YOLO-World, Grounding DINO, OWLv2 (named in words); YOLOPv2, TwinLiteNet (lanes); Depth Anything V2 (distance). The fair test: one recording of the car's cameras, every model on it.
- **Several models at once: yes.** Each reads the same camera topic and publishes its own result.
- **The pipeline is right:** points from the lidar or radar, detections from the cameras; a detection needs a distance before it's a point on the ground; Nav2's costmap marks the points, and Nav2 drives around them. Stop signs stay out of the costmap: they lock the mux. Today the car has the radar; the lidar comes next year.

### The "Choices I made for you" table, pasted with no note. (20:55)

- **Read as no change:** every choice stays as it was. Ben can name any one to change.

### "go", for the 17:54 question: carry on with the MCP server, push, or undo? (21:13)

- **Both were already done:** the push at 20:24, after the strip, and every commit since. GitHub's `main` matches this Mac's (`71ad715`). The MCP server is built and committed (`839e35f`).
- **Nothing to undo:** the `pixi.toml` and `pixi.lock` changes stay, with the server.

### The scp's half-copied last file: are we missing code? Check the server. (21:29, 21:41)

- **No code is missing.** The file at 0% was `vulkan_static_assertions.hpp`: a header of Vulkan, a graphics library, in the server's Linux Pixi environment (`.pixi/envs/default/include/vulkan/`). It's conda-forge's download, not anyone's code; the server's copy is intact, and Macs don't use Vulkan.
- **Checked on oirp-spss, read-only (21:30):**

| On the server | On the Mac |
|---|---|
| Our repo's commits, up to 04:23 | all in our history: the same files as `e8a9aa5`, with 83 commits since |
| Ben's 12 uncommitted files (the Rust IMU driver, three launch and package edits) | byte for byte in `dfaec15`; one test script improved since, in `db67f08` |
| Edits after 16:00 | none |
| Files git ignores, beyond build output | only the docs PDF and a linter cache, both made by tools |
| The 19 old team repos | every branch and commit the same |
| The ROS 2 source (130 repos) | the same commits, in `aav/vendor/` |
| `SYSC3313`, `c_code_example`, `simplied.c`, `text` | the same checksums |
| `aav.bundle` | the 04:23 bundle; its commit is in our history |

- **Only on the Mac:** the folder's `CLAUDE.md`, and a local branch `cleanup/gitignore-2026-10-10` in 14 old clones (13:27). Each has one commit that stops tracking build output and editor settings: no source code, never pushed. Their `main` branches are untouched.

### Can we change all the C++ to Rust? (21:32)

- **Yes, for our own C++.** Stock ROS packages (Nav2, robot_localization, twist_mux) stay as they are: ROS maintains them.

| Our C++ | Lines | Plan |
|---|---|---|
| `yahboom_imu_cpp` | 251 | Ben's Rust twin becomes the default; then the C++ goes |
| `aav_object_detection` (radar clustering) | 1,694 | the one node that runs becomes a Rust ROS node; the unlaunched demos go |
| Today's ESP32 board | 686 | replaced by the spine (Rust, Embassy) |
| The wheel encoder's Pico 2 | 269 | a Rust twin (Embassy, the same UDP packets) |
| The status display's Pico 2 | 1,043 | a Rust twin, or dropped: nothing talks to it |
| The HUD's Linux launcher | 154 | stays: Flutter generates it |

- **When:** right after tonight's five parallel pieces merge, since they edit the same launch files and the radar.

### No micro-ROS: Rust and Embassy on the boards, talking to a real ROS node. (21:34)

- **Decided (Ben):** no micro-ROS. Each board runs Rust and Embassy, and talks plain CAN, UART or UDP to a ROS node on Linux, which publishes. That's already the spine's design: spine → CAN → `spine_bridge`.
- **Changed 11 Oct (Ben, 11:38):** Tentative now. No micro-ROS for now, and we think again once we see the hardware.

### "cuda means Metal" seems weird: one ML node on the Orin and one on my laptop. Which weights for stop signs, perception and distance? (21:34)

- **Both at once:** each machine runs its own detector, under its own name, and publishes its own detections; the stop-sign watcher listens to both. The `cuda`-means-Metal alias goes: `auto` (the default) picks each machine's GPU, and the other choices are the real names (`cuda`, `mps`, `cpu`). Next, after the five pieces merge.
- **Weights:** YOLO11, the stock one (AGPL-3.0; "stop sign" is a COCO class); RF-DETR (Apache-2.0; beats YOLO11 on COCO, and runs on Jetsons); NVIDIA's TrafficCamNet and DashCamNet (cars, people, bikes, and "road sign" for any sign); for a sharper stop-sign model, fine-tune on the Mapillary Traffic Sign Dataset (free for academic research).
- **Distance:** a stop sign's known size (Ontario's come in 600, 750 and 1200 mm: distance = focal length × width ÷ the box's width in pixels); the flat-road maths for other objects; Depth Anything V2 Small, metric (Apache-2.0), from one camera.
- **Recommended:** YOLO11 or RF-DETR, the known-size trick for stop signs, and the flat-road maths for the rest.

### Where are we? Python, Rust, C++... Lots of mock drivers: are there real ones? (21:34)

- **Code:** Python 11,641 lines (our ROS nodes, tools, tests), Rust 6,417 (the spine, Ben's IMU driver), C and C++ 4,097 (inherited), Dart 2,082 (the HUD app).
- **Real drivers:** every device has one: the IMU (C++, and Ben's Rust twin), the GPS (the stock u-blox, NMEA or `ublox_dgnss`), the camera (`usb_cam`), the radar (`ti_ros2_driver`), the battery, the wheel encoder and the actuators (today's Pico and ESP32, through `aav_legacy`), the spine (`spine_bridge`), the joystick. The mocks are the bench's stand-ins. None of the real ones has run on the car yet: it runs Humble.

### How do I run the full suite and look at the graph in rqt_graph? (21:34)

- `pixi run bench` in one terminal (the whole car, with stand-ins), then `pixi run graph` in another; in rqt_graph pick "Nodes/Topics (all)" and refresh. `pixi run bench rviz:=true` adds RViz; `pixi run graph-picture` saves a picture.

### Everything will be Jazzy, not yet but it will be. (21:34)

- **Yes:** Decided on 9 Oct. First Pixi's Jazzy on the car's Ubuntu 22.04, then the Orin's reflash to JetPack 7.2 (Q24).

### "The spine may read the e-stop, but never sits in its path": what does it mean? I just want it to work. (21:37)

- **It means the red button always stops the car, whatever our software does.** We don't rewire it. The spine only listens, through a spare contact on the button, and tells ROS and the HUD it's pressed. The button's wiring doesn't run through our board, so if the board crashes or loses power, the button still stops the car.
- The docs now say it that way, with a picture ([Firmware](../firmware.md)).

### "A free Pico 2 for the bench: still needs an answer": what does that mean? (21:43)

- **Jargon, sorry:** "the bench" means a desk, and "free" means spare. The car's two Pico 2s are busy (the wheel encoder, the status display).
- **Why:** the spine's program for a Pico 2 only compiles until it runs on a real one. A spare Pico 2 on a desk is its first real test, with nothing that can move the car. It needs a Pico 2 (about $7), an MCP2515 CAN module (about $10, since the Pico 2 has no CAN) and a USB cable.

> **Revised (11 Oct):** a **3.3 V** CAN module, for example an MCP2515 with an SN65HVD230. The common module, with a TJA1050, runs at 5 V and doesn't suit a Pico 2's 3.3 V pins.

### TensorRT on the Orin, Core ML on the MacBook Pro, and whatever ROS tool helps. (21:43)

- **Decided (Ben).** Each machine runs its own detector, with its own model file:

| Machine | Model file | Runs on | The ROS node |
|---|---|---|---|
| The Orin | `.engine` (TensorRT), built on the Orin itself | its GPU | ours, `aav_yolo`, with JetPack's TensorRT. Later, if it needs more speed: Isaac ROS's `isaac_ros_yolov8` or `isaac_ros_rtdetr` |
| The MacBook Pro | `.mlpackage` (Core ML) | its GPU and Neural Engine | ours, `aav_yolo`: 4.3 ms a frame for the model, measured on Ben's Mac |

- **No ROS package runs Core ML:** our detector node is that tool, through Ultralytics and coremltools. Both machines publish the same detection messages, so nothing downstream cares which one saw the sign.
- **Next, with one detector per machine:** the machine files pick the model file; tasks export the `.mlpackage` (on a Mac) and the `.engine` (on the Orin, after the reflash, Q24).

### What was the Vulkan code doing? High-level first, then port, then make every file native. Real drivers everywhere. Ackermann, not bicycle. A fully working car. (21:45)

- **Vulkan:** a graphics library, the Linux counterpart of Metal. `vulkan_static_assertions.hpp` never runs: it checks, while a program compiles, that Vulkan's C++ and C types agree. Linux's Pixi has it for Qt (RViz, rqt), Gazebo's renderer and ffmpeg; none of our code uses it.
- **The order (Decided, Ben):**

| Step | What |
|---|---|
| 1. The high-level pieces | tonight's five parallel pieces: the spine board, the lane finder, obstacles, the car launch, the HUD |
| 2. Ackermann, done properly | reversing (Reeds-Shepp), both front wheels' real angles, the steering motor's real curve (Q2) |
| 3. Port the C++ to Rust | the IMU, the radar clustering, the encoder and display Picos |
| 4. Every file native | Rust that reads like Rust, not C in Rust; standard conventions in every language, checked by `pixi run test` |
| 5. Fully working | one check that drives everything end to end on the bench; then the car, once it runs Jazzy |

- **Ackermann:** Nav2 already plans for a car (Hybrid-A*, the 2.86 m turning radius), its controller never spins in place, and Gazebo steers with its Ackermann system. "Bicycle" is only the maths for the path: two front wheels treated as one in the middle, which gives the same path when the car doesn't skid, and is how ROS's Ackermann message defines its steering angle. Step 2 adds what's missing.
- **Real code everywhere:** every device already has a real driver; after step 1, every remaining stub gets replaced.
- **The old folders** stay as a reference: `aav` merged what mattered, and anything missing can be found there.

### The helpers stopped: restart them? Keep working; I'm going back to bed. (21:52 to 01:38)

- **Why they stopped:** every new message from Ben interrupts the session, and that stops the helpers running in the background too (20:48 and 21:29). At 23:30 all five also hit the plan's usage limit, which reset at 01:10.
- **Ben's answer (21:52):** restart them, and after any stop, restart them from where they got to, without asking again. Each commits after every working step, so a stop loses little.
- **01:40:** all five restarted from their branches. Ben: "pick the recommended option and continue working".

## 11 October: the night's results, and what we can try at home

### "We can only do so much from home so lets focus on stuff that makes sence to work on." (03:00, sent again 10:56)

Ben's order: the full graph with drivers, ego and the simulators, all set up; then the C++ to Rust (Python later); then the Mac on the network with ML on it, and Nav2; then the drivers in Rust, ready to try on a board; and raylib for the GUI.

- **Recorded** in `TODO.md` ("11 Oct: what we can try at home") and on the Decisions page.
- **Three helpers started at 11:18,** each in its own worktree: the full graph in Gazebo (a road, a camera and a radar on the car, perception and lane keeping in `pixi run sim`); Ackermann done properly (reversing, both wheels' angles, the steering curve); and the C++ to Rust (the IMU driver, the radar clustering).
- **The laptop's detector** is on in `laptop.yaml`, reading the camera's compressed pictures (Core ML on a Mac).

### "Ohh and for the gui lets maybe think about raylib?" (03:00)

- **Claude's take: yes, worth trying.** One Rust program (the `raylib` crate 6.0 and rclrs) can be the HUD and its ROS node at once, instead of three programs that must agree (the Flutter app, the Flask back end, Socket.IO). raylib draws on a Pi's screen with no desktop (its DRM mode), and on Macs.
- **The cost:** few ready-made widgets. The map, an on-screen keyboard and the destination search are ours to draw.
- **Other options:** keep Flutter (works on the bench; no Flutter download for the Pi's ARM Linux); egui (pure Rust, full widgets, needs a desktop on the Pi); Slint (made for car dashboards; check its licence).
- **Proposed:** the driving screen first (speed, mode, the e-stop button, the camera), the Flutter app kept until raylib's does everything; the map and the search decided with Q29. Waits on Ben.
- **Decided (Ben, 11:58): yes.**

### "how did last night go. Did you have a bunch of emails quueed." (10:56)

- **No emails were waiting.** The three queued for the VPN went out at 21:16, and one more at 21:42. Nothing was written after that: the overnight summary went out at 11:01.
- **The night:** the helpers stopped at 21:29 (Ben's messages; the 21:42 email wrongly said they were running) and restarted at 21:53; the usage limit stopped them at 23:30; they restarted at 01:10 and again at 01:38. Four branches merged between 02:07 and 02:43. At 03:00 the usage limit stopped everything until 10:56, in the middle of the fifth merge.
- **Since 10:56:** the fifth merge (obstacles) finished and was pushed (11:05). On `main`, on a quiet Mac, the bench checks passed, the stop sign and lane keeping too. The obstacle check failed; see the next answer.
- **Correction** (from the 21:43 answer): the Pico 2's CAN module must be a 3.3 V one.

### Why did bench-check --obstacle fail on main? (11:20, Claude's own finding)

- **Alone:** the stop zone stopped the car 0.34 m before the box. Then a 0.8 s stall of the whole computer (every topic paused at once) latched the spine, as its 200 ms watchdog should, and nothing armed it again, so Nav2 never finished.
- **After the stop-sign drive:** the car drove on past the box. A recorded rerun shows why it can: the car weaves up to 1.5 m off its line (N7: the global EKF's heading jumps with each GPS fix), so it can pass beside a 1 m box. In the rerun it reached the box 1 m off its line, and the stop zone just caught the box's edge.
- **The busy computer:** macOS's code checks (`syspolicyd`, XProtect) used about half a core, checking each program the helpers built.
- **The fix in the check:** a box as wide as the lane, and a re-arm after a latch, as a safety driver would (the lane check already does). The stop zone must still be what stops the car.
- **Still open:** N7 itself, and collision_monitor flickering between stop and go at the zone's edge.

### "No micro-ROS - lets rethink thinks later after we see the hardware" (11:38)

- **Changed:** micro-ROS goes from Decided to Tentative. For now the boards keep talking plain CAN, UART or UDP to a ROS node on Linux; we choose again once we've seen the car's chips and wiring (Q10–Q13).
- **Nothing is lost by waiting:** the spine's logic lives in `spine-core` either way. Only the layer that talks to Linux would change.
- **Changed on:** the Decisions page, `CLAUDE.md`, the firmware and languages pages, and the chips picture.

### "I guess i should be able to run the hud on my screen on my mac and drive the car with my mac, and core ml ai driver on the mac too." (11:38)

Ben: "All the perception stack maybe, what part does what. When sitting at the car? maybe the orin can do the processing". Saved as an idea in `TODO.md` ("Ideas to keep").

**The rule that decides it:** the car must stay safe with the Mac gone, because WiFi drops. So the car keeps everything that stops it; the Mac adds a screen, hands, and a second GPU.

| Part | Where | Why |
|---|---|---|
| The spine bridge, ego, the mux, the e-stop | the car, always | the car must stop by itself when the WiFi drops |
| The sensors, localization, Nav2, collision_monitor | the car | they need every reading, on time |
| The radar clusters, the camera obstacles | the car | they feed collision_monitor and the costmap: safety |
| The detector (YOLO) | both: the Orin's (TensorRT) and your Mac's (Core ML), each under its own name | works on the bench: the stop-sign watcher hears both, and the car doesn't need the Mac's |
| Lane keeping | the car; your Mac to try | from the Mac, its steering crosses the WiFi. A drop stops the car: safe, but jerky |
| The HUD | your Mac's screen | raylib's HUD would run on a Mac as it is. The Flutter app needs CocoaPods first |
| Driving by hand | your Mac: the joystick (priority 100) or the keyboard (90) | works today, on the bench |
| Watching and recording: RViz, Foxglove, rqt_graph, `ros2 bag` | your Mac | |

- **"When sitting at the car?"** The Orin does what the car needs to drive; your Mac watches, drives by hand, and tries new models first. A model moves to the Orin once it works.
- **What it waits on:** the car on Jazzy (Q24); the HUD on a Mac (raylib, or CocoaPods for Flutter); the car's WiFi, measured; and your Mac's clock in step with the Orin's (chrony, [Platforms](../platforms.md) step 5).

### "gh repo edit OWNER/REPO --visibility public --accept-visibility-change-consequences" (11:38)

- **Not run yet:** making the repo public can't be undone (anyone can copy it at once), and `CLAUDE.md` says private (Q25). Claude checked the history first, and asked Ben.
- **The check, all 167 commits on GitHub (`main` is its only branch):** no tokens, keys or passwords; `secrets.h.example` has placeholders only; none of the old repos' WiFi values.
- **What would become public:** Ben's two email addresses on every commit (rewriting the history would need a force-push); Ben's messages, word for word, in `docs/history/`; the server's name (`oirp-spss`) and Carleton's mail relay; the file paths of the old repos' leaked secrets (`docs/older-code.md`, not yet revoked: Q34); a 2025-26 student's own email address, in the radar driver's package files (inherited); and the 2025-26 code, whose repos have no licence. The repo has no licence file either, and 24 of our 26 packages say `TODO` for theirs.

### "Raylib for the HUD: ... yes". "any other questions?" A release statement, in point form. (11:58)

- **Raylib: Decided.** The HUD becomes one Rust program, raylib and rclrs, that is also its ROS node; the Flutter app stays until it matches. The next free helper, after the board drivers.
- **The release statement:** a new docs page, [Release notes](../release-notes.md): every project merged in or replaced, the models, the firmware, the Rust ports, Macs, Core ML and MCP, ego and the Ackermann model, the EKFs, the drivers, bridges and simulators, the tests and the docs, and what's left for the first pass.
- **"streedview?":** nothing by that name exists. The closest: the HUD's offline map and place search, its camera page, the bench's drawn road, Gazebo's road (in progress). Asked Ben what he meant.
- **Claude's questions for Ben:** the repo's visibility and a licence; what "street view" means; when someone can get to the car; a spare Pico 2 and a 3.3 V CAN module; the brake after boot; whether to copy today's decisions into the design doc.
- **The helpers:** Ben's message stopped all three at 11:58; they restarted at 12:00 from their branches.

### "Now, is a good time to start wrapping everything up." (12:27)

- **Done:** `main` pushed with its tests green. The three helpers (stopped by the message) are saved on their branches, each with a last "Work in progress" commit, pushed to GitHub too. A leftover ROS daemon was stopped.
- **Friday 16 Oct:** the hardware audit, read-only first; then driving with W A S D, the car or the turtle.
- **The final write-up:** emailed, with the release notes' list.

### "Can we get that in pdf or something so I can share it. Its a serious document." Think of it as an advertisement for this version. (13:28 to 13:30)

- **Rewritten as a release announcement:** "Release 0.1: the first pass". Highlights first (one workspace, one command path through ego, AI on every GPU with Core ML and TensorRT, Nav2 for a car, the CAN spine in Rust, bench mode, Macs, the MCP server), then a section per area, the numbers, what's next, and the credits.
- **Its own PDF:** `pixi run docs` now also makes `docs/html/aav-release-notes.pdf`: a cover with the pitch, six number tiles and the contents, then 7 pages. Emailed to Ben.
- **Kept honest:** nothing has run on the car yet ("Next: the real car"); TensorRT is ready for the Orin's reflash, not yet run; the full test suite was confirmed on the Mac.

## 15 October: topic names by area

### "The drivers all need to be in a /drivers/keybaord, etc type stuff as an example." (00:30)

- **Decided, then done:** every topic now starts with its area,
  `/<area>/<device or node>/<thing>`: `/sensors/gps/fix`, `/localization/odom/local`,
  `/drivers/joy/cmd_vel`, `/control/cmd_vel`, `/safety/estop`. The table is in
  [Topic areas](../naming.md).
- **The drivers:** each one publishes `/drivers/<name>/cmd_vel`. The joystick's nodes and
  the keyboard run in the namespaces `/drivers/joy` and `/drivers/keyboard`, so their
  names fall into place without remaps. twist_mux's `remote` slot was the keyboard, so it
  is called `keyboard` now.
- **Kept:** ROS's global names (`/tf`, `/clock`...), Nav2's own, navsat_transform's
  services (Nav2 calls `/fromLL` by its absolute name) and the HUD's HTTP routes.
- **Checked:** `tools/test.sh`, `pixi run test`, the turtle, Gazebo and bench mode with
  their checks. A test keeps every new topic in an area.

## 15 October: driving in plain English

### "Myabe we can load our own weights with qwen or llama or whatever is the best that allows us to type in plan engish what we want to do. like drive 100m forward. and have the model produce react commands."

- **Built:** `aav_language` ([Plain English](../plain-english.md)). A drive tool server
  (MCP over stdio, separate from the read-only one) with high-level tools only, and a
  ReAct agent on a local MLX model. It prints the plan and drives only after a typed yes.
- **The rules are in the server**, not the model: 1 m/s, 200 m per command, reverse
  0.5 m/s and 10 m, the car's tightest turn, no move while the e-stop is on (it reads
  twist_mux's own report), never clears it, no real car without `--car`, every call logged.
- **The model, by a fair test:** five local models, 23 requests (7 should be refused).
  Qwen3-4B-Instruct-2507 is the default: 21 of 23, 0.9 s a step, 3.3 GB. Ben's
  Llama-3.2-1B planned none of the 12 moves.
- **The test changed the design:** turns take "left"/"right" words (models got the sign
  wrong); the plan flags any number the person never typed (models shrank "250 m" to 200).
- **Ran:** Gazebo, "drive 10 m forward" = 10.02 m; "turn left 90 degrees with a 5 m
  radius" = 87.6°. Ctrl-C stopped a 30 m drive at 11.7 m; a drive while e-stopped was refused.
- **Credit:** the design follows Ben's SYSC 4415 A3 ReAct agent and his `mcpsearch`.

## 23 October: the sensor mux and the output mux

### "go" for T1.5 / D3 / S-4: simulated data must never reach the car.

- **Built:** `aav_io_mux`. The sensor mux (`source:=`) checks, from the ROS graph and
  `/system/source`, that each sensor comes from the one source asked for. The output mux
  (`target:=`) sends ego's command to `/control/<target>/actuator_cmd`; the real car's
  only while every source is REAL, else a latched, silent stop.
- **Choice made:** check, don't copy. Republishing the cameras and point clouds would
  double their traffic; a launch starts only the chosen source, and the check catches a
  stray publisher (two on one topic). Written down in
  [Sources and targets](../sources-and-targets.md).
- **One launch:** `stack.launch.py target:=turtle|gazebo|bench|car|carla`; the pixi tasks
  are unchanged.

### "Should the control board refuse to arm while the safety gate is shut?" Yes. (24 Oct)

- **Built:** `/spine/arm` (both `spine_bridge`s) refuses unless the output mux said `OPEN`
  for the bridge's own target within 1.5 s, and says why. The bench and Gazebo arm
  through `arm_when_open`, which waits for `OPEN`. `legacy_uart_bridge` has no arm step
  ([Arming waits for OPEN](../sources-and-targets.md#arming-waits-for-open)).

## 1 November: Betsi's measurements, and the lattice as the default

### "Can we fix her pull request?" Yes.

- **Fixed for Betsi** (PR #18): tests for her health checks, `tools/test.sh`'s two
  missing tasks, `interfaces.md`, then main merged in. CI green; Arsh approved and merged.
- **Her numbers are PROVISIONAL** in `vehicle.yaml`: the track (one average of front
  0.65 m and rear 0.85 m) and the wheel diameter (0.36 m; it read 0.42 m loaded). Both
  are car-day items (TODO 60, 61).

### "do all the ideas ... but car day." The lattice becomes the default.

- **Decided (Ben):** the State Lattice is the default planner where the car reverses
  (sim, bench, CARLA). The car keeps Hybrid-A* until H5: forward only, the lattice lost
  the U-turn (29 Oct).
- **Also built:** the cubes in RViz; health in `pixi run check`; on branches, the cubes
  and health on the debug screen, and KISS-ICP into `ekf_local` (`kiss_ekf:=true`).

## Open threads

- **Car day (1 Nov):** roll the car to measure the wheel diameter; check the front and rear track; then the lattice forward only (TODO 60-62).
- **D3, the muxes (23 Oct):** built and tested in every profile; for the team: agree
  "check, not copy", and whether a bench with a real spine board and pretend sensors needs
  a deliberate exception (today the output mux refuses it). On the car next.
- **CARLA (15 Oct, Q31 reopened):** the adapter and Carleton's map download are ready and tested without a server. Left: run it on the NVIDIA computer (0.10 for Town10, 0.9.16 for Carleton's streets), and check the IMU's roll and pitch signs, the radar's speed sign and the GPS line-up ([CARLA](../carla.md)).
- **Plain English (15 Oct):** works in Gazebo and the turtle; untested on the bench and
  the car. Before the car: the scripted driver on stands, steering feedback, a safety
  driver on the joystick.

- **Topic areas (15 Oct):** done and on `main`. Bags recorded before then hold the old names: play them with `--remap`. The raylib HUD's window wasn't opened in this check (no screen for the session); its names come from `config/hud.yaml`.
- **Secrets (Q34):** revoke the token and keys, and change the WiFi password. The owners are unknown.
- **oirp-spss:** its copy of `aav` stops at 04:23, with the old commit IDs. Make a fresh clone from GitHub there; nothing on the server is missing from GitHub (checked 21:30).
- **The five overnight pieces (10-11):** all merged and pushed (02:07–11:05). Now Ben's 11 October order: the full graph (with Ackermann), the C++ to Rust, the Mac with ML, Nav2, the board drivers, raylib. Three helpers started at 11:18.
- **The obstacle check:** fixed in the check (a lane-wide box, a re-arm after a latch); N7 and collision_monitor's flicker are still open.
- **Raylib for the HUD:** Decided (Ben, 11:58). The next free helper, after the boards one.
- **Paused 11 Oct, saved on branches:** `wt/sim-full` (nearly ready: its docs page is left), `wt/ackermann` (reversing in Nav2), `wt/rust-ports` (the radar clustering in Rust). See `TODO.md`, "Where we stopped".
- **Friday 16 Oct:** the hardware audit; then W A S D driving.
- **micro-ROS:** Tentative since 11 Oct: none for now; think again once we see the hardware.
- **Your Mac as the operator station:** an idea, saved (11 Oct): `TODO.md`, "Ideas to keep".
- **The repo public?** Checked 11 Oct; waits on Ben's yes.
- **One detector per machine:** `yolo_detector_<machine>`, each with its own model file (done 11 Oct). The device names without aliases come with the obstacles merge.
- **Backups:** image the Orin and the Pis before any reflash. They hold the detector, `camera_decode`, the three rqt_graph nodes and `/home/aavjetson`. A backup checklist and script were offered.
- **The car on Jazzy:** it runs Humble today, so no Jazzy laptop can join it. Try Pixi on its Ubuntu 22.04 first, then reflash the Orin to JetPack 7.2 (Q24).
- **Ben's Mac:** works, 10-10: build, tests, the turtle, Gazebo headless, Nav2, the Rust listener, the docs. Our version, `aav/aav`, is complete on the Mac since 16:13, with every commit from the `~/aav` clone merged in, and it builds and tests green. Gazebo's window works on the Mac: Ben saw the screenshots, and in `pixi run sim` every driver goes through twist_mux and ego (16:34). Left: look at RViz and rqt_graph.
- **Bench mode and perception (10-10 afternoon):** the whole car graph runs on the Mac with stand-ins (`pixi run bench`, `bench-check` 7 of 7); YOLO stops the car at stop signs on the bench; the spine's device drivers are in Rust; the HUD's back end is on our topics. Done 11 Oct: Ben's server work is in `dfaec15`, perception is parts of `car.launch.py`, and we wrote our own lane finder. Waiting on the team: the spine's chip, three firmware decisions (one constant each now), `best.pt`.
- **Claude on the car's network (Ben, 10-10 16:42):** Ben will put Claude on the network with the hardware, to audit it directly. The plan is read-only first: find the devices, take each computer's inventory, copy what only the Orin has, and compare with our contract. Done beforehand: the stand-ins stay on their own computer (`tools/ros.sh --local`).
- **`.gitignore` and `vendor/`:** both done: the fuller `.gitignore` (593db7f), and `vendor/` (10-10 evening: the lists in git, the source out of it). Pruning the app's snapshots from `.git` waits on Ben.
- **The MCP server (Ben, 10-10 17:03):** built that evening, read-only. Open: bench-only actions through the stand-ins, if the team wants them; nothing that reaches the car.
- **Slim per-machine Pixi environments:** for the Orin, the Motor Pi and laptops, matching the machine files. Waits until the first car computer gets Pixi (Phase 0).
- **Running today:** the inventory commands haven't been run on the car's computers.
- **The board survey** (the old repos, for the board's wiring, photos and the e-stop): stopped unfinished at 20:48, with no report. It runs again only if Ben asks.
- **2024–25 repos:** cloning and surveying `emergency-stop`, `Simulation`, `camera_Yolo_detection`, `Braking` and `Power-Management-Documentation-2024-25` was offered, not done. Only `GNSS` has been read in depth.
- **`ros2-rust/`:** keep or delete is undecided; the recommendation is keep.
- **The claude.ai chat (fe09d020…):** Ben to paste the key parts.
- **CARLA:** parked this year (10-10). It comes back on the NVIDIA Linux machine if it ever supports Jazzy (Q31).
- **The NVIDIA Linux machine:** which one, and access to it, to find `best.pt`.
- **The board:** a photo of the ESP32 board, and a spare Pico 2 with a 3.3 V MCP2515 module, to test the spine on a desk.
- **Team decisions:**
  - the team's `ROS_DOMAIN_ID`;
  - the spine arming rules (Proposed);
  - the GPS driver: `ublox` is the default since 11 Oct (Tentative); keep it, or go back to `ublox_dgnss` (B4, with Q18);
  - the brake after boot: `IDLE_BRAKE` is 0, so the brake lets go; the 2025-26 firmware kept it on;
  - the `/estop` QoS in the mux (N2), respawning device nodes (N3), and Nav2's lookahead (N7);
  - every other Proposed row;
  - QNX (Q23);
  - the GPS model and corrections (Q18, Q19);
  - the spine owner and inputs (Q10, Q12);
  - a new org or not (Q25);
  - steering at standstill (Q27);
  - joystick handover (Q28);
  - the rest of Q1–Q34.
- **Hardware checks:**
  - the conflicting steering calibrations (Q2);
  - whether the 8-spoke encoder can hold walking pace;
  - the IR sensor spacing needed for quadrature;
  - the Orin module part number, before JetPack 7.2.
