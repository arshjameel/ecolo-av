# What each package does

Every ROS package in `ros/`, area by area (`ros/<area>/<package>`), with what it reads
and writes. Then the firmware, the HUD and the examples. 37 packages build today (the
Flask HUD went to the archive on 25 Oct), and none is parked.

## At a glance

| Package | Area | Does | Status | Origin |
|---|---|---|---|---|
| `aav_sensors` | sources | starts the IMU, GPS, lidar, radar, battery and USB camera drivers, with their configs | [[untested]] needs the devices | new |
| `aav_camera_stream` | sources | one node per front camera: the camera Pi's H.264 stream, decoded, with RF-DETR inside | [[works]] on a Mac; [[untested]] on the Orin | new |
| `aav_imu_yahboom` | sources | the Yahboom IMU over USB serial (Rust) | [[untested]] needs the device; the C++ driver's answers, bit for bit | new: Ben's rewrite of the 2025-26 `yahboom-IMU` C++ driver |
| `aav_battery_victron` | sources | the Victron battery monitor | [[untested]] | 2024-25 `Battery-Monitor-ROS` |
| `aav_radar_ti` | sources | the TI mmWave radar, as point clouds | [[untested]] | 2025-26 `Nav_Radar` |
| `aav_description` | bringup | the car's model (URDF) and `vehicle.yaml` | [[works]] the numbers are guesses | new |
| `aav_localization` | localization | the two EKFs and `navsat_transform`: configuration only | [[works]] turtle, Gazebo and the bench | new |
| `aav_gps_heading` | localization | `gps_heading`: the heading from the GPS receiver's velocity, for `ekf_global` | [[works]] Gazebo | new |
| `aav_teleop` | drivers | joystick → `/drivers/joy/cmd_vel`, plus an e-stop button | [[untested]] | new; the button map from 2025-26 `joy_teleop_ackerman` |
| `aav_rc` | drivers | the RC transmitter, on "RC via ROS", as a driver (priority 95, proposed) | built 24 Oct; off until the hardware lead confirms the wire (H9) | new |
| `aav_actuator_tests` | drivers | the stage 3-4 tests, as a driver (priority 60) | [[works]] Gazebo; [[untested]] on the car | new |
| `aav_scripted` | drivers | scripted drives: speed up, hold, slow down | [[works]] turtle and the bench | new |
| `aav_drive_to_point` | drivers | drive to one GPS or map point B, and stop there smoothly | [[works]] in Gazebo (15 Oct); [[untested]] on the car | new |
| `aav_language` | drivers | plain English: a local model plans drive tools, you say yes | [[works]] Gazebo and the turtle | new |
| `aav_navigation` | drivers | Nav2, set up for a car, and the maps | [[works]] goals reached in Gazebo | new |
| `aav_routes` | drivers | street routes (Valhalla) as GPS waypoints for Nav2 | [[works]] in Gazebo | new |
| `aav_command_mux` | command_mux | twist_mux: the priorities, the e-stop lock and the other locks | [[works]] | new |
| `aav_io_mux` | io_mux | the sensor mux and the output mux: simulated data never drives the real car | [[works]] turtle, Gazebo and the bench; [[untested]] on the car | new |
| `aav_ego` | ego | one command → speed and steering the car can do | [[works]] | new |
| `aav_spine` | can_bridge | the CAN spine: ego's commands as CAN frames. Its Python `spine_bridge`, the config and the launch file | [[works]] on the bench, against `spine_sim`; no spine board yet | new |
| `aav_spine_bridge` | can_bridge | the same `spine_bridge` in Rust, on the firmware's own CAN code (`spine-core`); the default since 23 Oct (`bridge:=python` for the other) | [[works]] on the bench, against `spine_sim`; no spine board yet | new: a port of `aav_spine`'s node (T1.8) |
| `aav_legacy` | can_bridge | today's car: the 2025-26 ESP32 over UART, through the Motor Pi | [[untested]] on the car | new, for the 2025-26 protocols |
| `aav_turtlesim` | sim | turtlesim, steered like the car | [[works]] | new |
| `aav_gazebo` | sim | Gazebo's car behind the real car's topics | [[works]] headless | new |
| `aav_carla` | sim | a CARLA adapter (CARLA's Python client) | [[untested]] never run against a server | new |
| `aav_spine_sim` | sim | the spine on a laptop: the firmware's logic on a model car (Rust) | [[works]] | new |
| `aav_bench` | sim | pretend sensors: IMU, GPS, battery, camera, radar, lidar | [[works]] | new |
| `aav_safety` | safety | the sensor watchdog: a twist_mux lock (30) when a sensor is stale, NaN, frozen or jumps | [[works]] Gazebo and the bench | new |
| `aav_ghost` | safety | ghost obstacles: a pretend person only Nav2 sees | [[works]] in Gazebo | new |
| `aav_interfaces` | ecolo_msgs | our 9 messages (such as `VehicleStatus`, `DataSource`, `Heartbeat`) and 2 actions (`ScriptedDrive`, `DriveToPoint`) | [[works]] builds | new |
| `aav_bringup` | bringup | the six profiles, `stack.launch.py`, Foxglove, the machine files, the RViz layouts | [[works]] turtle, sim, replay, bench | new |
| `aav_detector` | perception | RF-DETR (the default) or YOLO on the camera's images, and the stop-sign watcher | [[works]] on the bench | 2024-25 `camera_ros2_ws` |
| `aav_radar_clusters` | perception | radar clustering (DBSCAN), with a box round each object (Rust) | [[works]] on the bench and in Gazebo | new: a rewrite of the 2025-26 `Nav_Radar` C++ node |
| `aav_lidar` | perception | `lidar_scan`: the roof lidar's cloud as a 2D scan for Nav2 | [[works]] in Gazebo and the bench | new |
| `aav_lane_finder` | perception | the lane lines in the camera's pictures, as `/perception/lane/geometry` (OpenCV) | [[works]] on the bench | new |
| `aav_lane_geometry` | perception | lane keeping: a driver on `/drivers/ai/cmd_vel` | [[works]] on the bench, with `aav_lane_finder` | 2025-26 `Lane_Detection_ROS2` |
| `aav_hud` | hud | the in-car screens in Rust (raylib): the HUD (node `hud`) and the map window (node `map`) | [[works]] on a Mac: `--demo`, fed topics, bench mode. [[untested]] on a Pi and on the car | new |

## Sensors

| Package | Reads | Publishes | Notes |
|---|---|---|---|
| `aav_sensors` | | | `sensors.launch.py imu:= gps:= lidar:= radar:= battery:= gps_driver:=`; configs in `config/` |
| `aav_imu_yahboom`: `imu_node` | `/dev/aav/imu` | `/sensors/imu/data_raw`, `/sensors/imu/mag` | `/sensors/imu/mag` is in raw counts, not tesla, so it's unused |
| stock `imu_filter_madgwick` | `/sensors/imu/data_raw` | `/sensors/imu/data` | magnetometer off |
| stock `ublox_gps` (KumarRobotics' `ublox`) | `/dev/aav/gps` | `/sensors/gps/fix` | the default GPS driver, in Pixi too. Leaves the receiver's settings as they are. Tentative until Q18 |
| stock `ublox_dgnss` | a u-blox F9P on USB | `/sensors/gps/fix` | `gps_driver:=ublox_dgnss`. Not in Pixi: the car only |
| `aav_radar_ti` | `/dev/aav/radar_cmd`, `/dev/aav/radar_data` | `/sensors/radar/points` | its configs are indoor demos |
| `aav_battery_victron` | `/dev/aav/battery` | `/vehicle/battery` | |
| `aav_camera_stream`: `camera_stream_<camera>` | the camera Pi's RTSP stream (MediaMTX) | `/perception/detections` | the car's two front cameras; RF-DETR inside each node, so no raw pictures on the network ([camera pipeline](camera-pipeline.md)) |
| stock `velodyne` driver | the roof VLP-16's UDP packets | `/sensors/lidar/points` | `lidar:=true` (on by default on the car); untested with the real lidar |
| stock `usb_cam` (`cameras.launch.py`) | `/dev/aav/camera_left`, `_right` | `/sensors/camera/front_left/image_raw`, `/sensors/camera/front_left/camera_info` | a USB camera only; Linux only; the 2024-25 calibrations |
| stock `nmea_navsat_driver` (`gps_nmea.launch.py`) | `/dev/aav/gps` | `/sensors/gps/fix` | `gps_driver:=nmea`: any NMEA receiver |

## Description and localization

| Package | Reads | Publishes | Notes |
|---|---|---|---|
| `aav_description` | `vehicle.yaml`, `/vehicle/joint_states` | `/robot_description`, `/tf`, `/tf_static` | `pixi run view` shows the model |
| `aav_localization`: `ekf_local` | `/vehicle/odom`, `/sensors/imu/data` | `/localization/odom/local`, odom → base_link | smooth; use it to steer |
| `aav_localization`: `ekf_global` | the same, plus `/localization/gps/odom` and `/localization/gps/heading` | `/localization/odom/global`, map → odom | stays near the truth; use it to plan |
| `aav_localization`: `navsat_transform` | `/sensors/gps/fix`, `/sensors/imu/data`, `/localization/odom/global` | `/localization/gps/odom`, `/localization/gps/fix` | the datum in `navsat.yaml` is a placeholder near Carleton |

## Drivers, the mux and ego

| Package | Reads | Publishes | Notes |
|---|---|---|---|
| `aav_teleop` | `/drivers/joy/joy` | `/drivers/joy/cmd_vel` (while the trigger is held), `/safety/estop` | the button numbers are still TODO |
| keyboard (`tools/keyboard.sh`) | your keys | `/drivers/keyboard/cmd_vel` | `pixi run keyboard` |
| `aav_actuator_tests` | `/vehicle/odom`, `/vehicle/steering`, `/throttle`, `/brake`, `/wheels`, the IMU, `/localization/odom/local`, GPS, `/safety/estop`, `/control/cmd_vel`, `/system/source` | `/drivers/tests/cmd_vel` | `pixi run actuator-test`: the stage 3-4 tests; prints `vehicle.yaml` lines ([Actuator tests](actuator-tests.md)) |
| `aav_scripted` | `/localization/odom/local` | `/drivers/scripted/cmd_vel` | the action `/scripted_drive` |
| `aav_drive_to_point` | `/localization/odom/global`, `/sensors/gps/fix`, `/safety/estop`, the watchdog's lock, `/system/source`, `/control/cmd_vel` | `/drivers/point/cmd_vel` | the action `/drive_to_point`; calls `/fromLL` for a lat/lon B. `pixi run drive-to-point` ([A to B](a-to-b.md)) |
| `aav_language` | your words; odometry, GPS, `/diagnostics`, `/vehicle/status` | `/safety/estop` (true only) | calls `/scripted_drive` and `/follow_gps_waypoints`; never publishes a velocity |
| `aav_navigation` | `/localization/odom/local`, `/map`, `/goal_pose` | `/drivers/nav/cmd_vel`, `/plan` | SmacPlannerHybrid and MPPI (or Regulated Pure Pursuit); GPS waypoints through `/follow_gps_waypoints` |
| `aav_command_mux` (twist_mux) | every `/drivers/<driver>/cmd_vel`, `/safety/estop` | `/control/cmd_vel` | priorities: [Build a driver](howto-driver.md) |
| `aav_ego` | `/control/cmd_vel` | `/ego/actuator_cmd`, 50 times a second | 1.0 m/s top speed, ±0.5 rad, brakes after 0.3 s without a command |
| `aav_io_mux` | `/ego/actuator_cmd`, `/system/source`, who publishes each sensor | `/control/<target>/actuator_cmd`, `/system/sensor_mux/state`, `/safety/output_mux/state` | the real car's commands only while every source is REAL: [Sources and targets](sources-and-targets.md) |

## Perception

[Perception](perception.md) has the details.

| Package | Reads | Publishes | Notes |
|---|---|---|---|
| `aav_detector`: `object_detector` | `/sensors/camera/front_left/image_raw` | `/perception/detections`, `/perception/detections/image` | `detector:=rfdetr` (the default: a `.pth`, or its `rf-detr-*.mlpackage` or `.engine`) or `yolo` (a `.pt`, `.mlpackage` or `.engine`); needs the `ml` environment, or JetPack's PyTorch and TensorRT |
| `aav_detector`: `stop_sign_watcher` | `/perception/detections` | `/perception/stop_sign` | twist_mux's stop-sign lock, priority 30 |
| `aav_radar_clusters`: `radar_clusters` | `/sensors/radar/points` | `/perception/radar/clusters`, `/perception/radar/cluster_boxes` | DBSCAN, in Rust; eps 0.8 m, 15 points (5 on the bench and in Gazebo) |
| `aav_lane_finder`: `lane_finder` | `/sensors/camera/front_left/image_raw`, `/sensors/camera/front_left/camera_info` | `/perception/lane/geometry`, `/perception/lane/image` | a bird's-eye view, sliding windows, a curve per line; the camera's place from `vehicle.yaml` |
| `aav_lane_geometry`: `control_interface` | `/perception/lane/geometry` | `/drivers/ai/cmd_vel` | steers back to the lane's centre, measured 3 m ahead |

## Backends

Each backend reads ego's commands from the output mux (`/control/<target>/actuator_cmd`) and reports the car's speed on `/vehicle/odom`.
Only one runs at a time.

| Package | Plays the car with | Also |
|---|---|---|
| `aav_spine` (`aav_spine_bridge`: the node, in Rust) | CAN frames on `/can/tx` (through ros2_socketcan, Linux only) | `/vehicle/status`; the service `/spine/arm`; stops sending drive frames when ego goes quiet for 0.1 s |
| `aav_legacy` | UART frames to today's ESP32 (`/dev/ttyAMA0`): 13 bytes today, or version 2 (17 bytes) with the output mux's heartbeat | `encoder_udp`: the 2025-26 wheel encoder's UDP packets into `/vehicle/odom` (off on the car since 23 Oct: the wheels are on CAN) |
| `aav_turtlesim` | `/turtle1/cmd_vel` | |
| `aav_gazebo` | `/gazebo/cmd_vel` | turns Gazebo's IMU and GPS into `/sensors/imu/data_raw` and `/sensors/gps/fix` |
| `aav_carla` | CARLA's Python client (TCP to the server) | the same sensor topics as Gazebo; untested ([CARLA](carla.md)) |

## Bench mode

[Bench mode](bench.md) runs the car's software with these stand-ins for the hardware.

| Package | Stands in for | Notes |
|---|---|---|
| `aav_spine_sim`: `spine_sim` | the spine board and the actuators | answers spine_bridge's CAN frames with spine-core's own logic; publishes `/vehicle/bench/truth` |
| `aav_bench` | the IMU, GPS, battery and camera drivers | `mock_imu`, `mock_gps`, `mock_battery`, `mock_camera`; the world is `config/bench.yaml` |

## Interfaces and bringup

| Package | Holds |
|---|---|
| `aav_interfaces` | 9 messages: `VehicleStatus` (mode, armed, e-stop, speed, steering, throttle, brake, fault bits), `ActuatorState`, `DataSource`, `SensorSource`, `SourceCheck`, `Heartbeat`, `RcSticks`, `SpineOutputs`, `WheelSpeeds`; 2 actions: `ScriptedDrive`, `DriveToPoint` |
| `aav_bringup` | the profiles `turtle`, `sim`, `bench`, `car`, `carla`, `replay`; `stack.launch.py target:=`; `foxglove.launch.py`; the machine files `orin`, `motor_pi`, `laptop`, `all`, which pick the car's parts, perception too; the RViz layouts |

## Firmware

colcon ignores `firmware/`. It's built with cargo, PlatformIO or the Pico SDK.

| Part | What it is | Status |
|---|---|---|
| `can/aav.dbc` | the CAN contract: six frames between the Orin, the spine and the two wheel nodes | v0.2.0 (the KEFC plan), merged 15 Oct |
| `spine/crates/spine-core` | the spine's logic in Rust: modes, arming, the 100 ms watchdog, the speed loop, the encoder | [[works]] 84 tests, and the bench |
| `spine/crates/wheel-core` | the front wheel nodes' logic in Rust: pulses to speed, the WHEEL_SPEED frame | [[works]] 9 tests, and the bench; no board yet |
| `spine/crates/aav-inputs` | debounced switches, an e-stop that trips at once, counting with a lockout | [[works]] 9 tests |
| `spine/crates/jrk-g2` | the Pololu Jrk G2 (steering, brake) over I2C | [[works]] 14 tests, against mocks |
| `spine/crates/ad5311` | the throttle DAC over I2C | [[works]] 10 tests, against mocks |
| `spine/crates/pca9555` | the I/O expander (direction) over I2C | [[works]] 6 tests, against mocks |
| `spine/crates/ibus` | the FlySky RC receiver's protocol | [[works]] 24 tests |
| `spine/crates/mcp2515` | the MCP2515 CAN controller over SPI, since a Pico 2 has no CAN | [[works]] 34 tests, against mocks |
| `spine/crates/legacy-link` | the 2025-26 UART link: the Motor-Pi's 13-byte frames and the C++'s targets; version 2 with the Orin's heartbeat | [[works]] 72 tests, and against the C++ itself |
| `spine/crates/spine-board` | the glue: spine-core's outputs to the devices, the devices to its inputs, the `Board` trait, the `CommandLink` trait (CAN or UART) | [[works]] 79 tests, against mocks |
| `spine/boards/esp32-umod` | the spine on today's ESP32 control board (Embassy): the C++'s pins and devices; the Motor-Pi's UART or CAN | [[untested]] builds, never flashed |
| `spine/boards/pico2` | the spine's board program for a Pico 2 (Embassy): it implements `Board`, with a 100 ms hardware watchdog | [[untested]] builds, never flashed. The car's chip is Q10-Q13 |
| `spine/crates/wheel-encoder` | the spoke sensors (shared with the spine) and the wheel encoder's UDP packet | [[works]] 12 tests, with examples `aav_legacy` checks too |
| `spine/crates/sentinel` | the status display's logic: its I2C packet, 13 LCD pages, the knob, the beeps | [[works]] 40 tests |
| `spine/crates/hd44780` | a 16x2 character LCD on six GPIO pins | [[works]] 5 tests, against a fake |
| `spine/boards/wheel-encoder-pico2` | the wheel encoder in Rust: spokes to the Motor Pi over UDP, through a W5500 | [[untested]] builds, never flashed |
| `spine/boards/sentinel-pico2` | the status display in Rust: an I2C device at 0x42 | [[untested]] builds, never flashed; not wired in |
| `legacy/aav_umod_esp32` | the 2025-26 ESP32 that drives today's actuators | runs the car today: no timeout, an update page with no password. Stays until `boards/esp32-umod` is proven on the car |

## The HUD

| Part | What it is |
|---|---|
| `ros/hud/aav_hud` | the HUD in Rust (raylib and rclrs): the driving screen, the e-stop button, GPS and topic health (`pixi run hud --demo`); and the map window (`pixi run map --demo`). [HUD](hud.md) |
| `archive/2026-27-flutter-hud/` | the 2025-26 Flutter app and its Flask back end, archived 25 Oct (Q29). Not built |
