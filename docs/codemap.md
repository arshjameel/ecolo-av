# The code map

Where the code is: **read this to find code; don't search the tree.** One line
per package, by area. Written by `tools/codemap.py` (`tools/test.sh` checks it is
current): after adding, moving or removing a package, run
`python3 tools/codemap.py > docs/codemap.md`.

| Column | Means |
|---|---|
| Logic | the maths, no ROS: a Rust crate, or the Python modules |
| Node | the ROS process: a `*-node` crate, or a Python file with `main()` |
| Tests, Config | the folders, in the package |

## ros/sources

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_battery_victron](../ros/sources/aav_battery_victron/README.md) | Python | Reads the Victron battery monitor's VE.Direct serial text and publishes it as sensor_msgs/BatteryState on /... | `aav_battery_victron/(3 modules)` | `aav_battery_victron/battery_monitor.py` | `test/` | - |
| [aav_camera_stream](../ros/sources/aav_camera_stream/README.md) | Python | One node per camera | `aav_camera_stream/(7 modules)` | `aav_camera_stream/camera_stream.py` | `test/` | `config/`, `launch/` |
| [aav_camera_stream_rs](../ros/sources/aav_camera_stream_rs/README.md) | Rust | camera_stream in Rust (rclrs) | `crates/stream-core` | `crates/stream-node` | `test/` | - |
| [aav_imu_yahboom](../ros/sources/aav_imu_yahboom/README.md) | Rust | imu_node in Rust (rclrs) | `crates/yahboom-protocol` | `crates/imu-node` | - | - |
| [aav_radar_ti](../ros/sources/aav_radar_ti/README.md) | Python | Read uart data from ti iwr6843aopevm mmwave sensor and publish the data in ros2 env | `aav_radar_ti/(3 modules)` | `aav_radar_ti/mock_radar_publisher.py`, `aav_radar_ti/pc_publisher.py` | `test/` | - |
| [aav_sensors](../ros/sources/aav_sensors/README.md) | launch, config | Sensor configuration and launch | - | - | - | `config/`, `launch/` |

## ros/localization

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_amcl_rs](../ros/localization/aav_amcl_rs/README.md) | Rust | AMCL in Rust (rclrs), in stock nav2_amcl's place (amcl_impl:=rust, the default) | `crates/amcl-core` | `crates/amcl-node` | `test/` | - |
| [aav_gps_heading_rs](../ros/localization/aav_gps_heading_rs/README.md) | Rust | gps_heading in Rust (rclrs) | `crates/gps-heading-core` | `crates/gps-heading-node` | `test/` | - |
| [aav_kinematic_icp](../ros/localization/aav_kinematic_icp/README.md) | Rust | Lidar odometry that knows it is on a car (Kinematic-ICP's idea, in Rust | `crates/kinematic-icp` | `crates/kinematic-icp-node` | `test/` | `config/`, `launch/` |
| [aav_kiss_icp](../ros/localization/aav_kiss_icp/README.md) | Python | kiss_icp | `aav_kiss_icp/(2 modules)` | `aav_kiss_icp/kiss_icp_node.py`, `aav_kiss_icp/worker.py` | `test/` | `config/`, `launch/` |
| [aav_lane_pose](../ros/localization/aav_lane_pose/README.md) | Python | lane_pose | `aav_lane_pose/lane_fix.py` | `aav_lane_pose/lane_pose.py` | `test/` | - |
| [aav_localization](../ros/localization/aav_localization/README.md) | launch, config | robot_localization configuration | - | - | - | `config/`, `launch/` |
| [aav_localization_health_rs](../ros/localization/aav_localization_health_rs/README.md) | Rust | localization_health in Rust (rclrs) | `crates/health-core` | `crates/health-node` | `test/` | `config/` |
| [aav_ndt](../ros/localization/aav_ndt/README.md) | Rust | NDT in Rust (rclrs), a trial | `crates/ndt-core` | `crates/ndt-node` | `test/` | `config/`, `launch/` |
| [aav_pose_check](../ros/localization/aav_pose_check/README.md) | Rust | pose_check (Rust, rclrs), a challenger (TODO 210, pose_source:=ndt) | `crates/pose-check-core` | `crates/pose-check-node` | `test/` | `config/` |

## ros/perception

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_detector](../ros/perception/aav_detector/README.md) | Python | The object detector | `aav_detector/(6 modules)` | `aav_detector/camera_obstacles.py`, `aav_detector/detector.py`, `aav_detector/stop_sign_watcher.py` | `test/` | `config/`, `launch/` |
| [aav_detector_rs](../ros/perception/aav_detector_rs/README.md) | Rust | The object detector in Rust (rclrs) | `crates/detector-core`, `crates/onnx-session` | `crates/detector-node` | - | - |
| [aav_lane_finder](../ros/perception/aav_lane_finder/README.md) | Python | The lane finder | `aav_lane_finder/(2 modules)` | `aav_lane_finder/lane_finder.py` | `test/` | `config/`, `launch/` |
| [aav_lane_finder_learned](../ros/perception/aav_lane_finder_learned/README.md) | Rust | The learned lane_finder (TODO 262b), a challenger (lane_finder_impl:=learned) | `crates/ufld-core` | `crates/ufld-node` | `test/` | - |
| [aav_lane_finder_rs](../ros/perception/aav_lane_finder_rs/README.md) | Rust | lane_finder in Rust (rclrs) | `crates/lane-core` | `crates/lane-node` | `test/` | - |
| [aav_lane_geometry](../ros/perception/aav_lane_geometry/README.md) | messages | The LaneGeometry message | - | - | - | - |
| [aav_lidar](../ros/perception/aav_lidar/README.md) | Python | The roof lidar for Nav2 | `aav_lidar/(2 modules)` | `aav_lidar/lidar_scan.py` | `test/` | - |
| [aav_lidar_rs](../ros/perception/aav_lidar_rs/README.md) | Rust | lidar_scan in Rust (rclrs) | `crates/lidar-scan` | `crates/lidar-scan-node` | `test/` | - |
| [aav_radar_clusters](../ros/perception/aav_radar_clusters/README.md) | Rust | radar_clusters in Rust (rclrs) | `crates/radar-clustering` | `crates/clusters-node` | `test/` | `launch/` |
| [aav_stop_sign_rs](../ros/perception/aav_stop_sign_rs/README.md) | Rust | stop_sign_watcher in Rust (rclrs) | `crates/stop-sign-core` | `crates/stop-sign-node` | `test/` | - |
| [aav_things_rs](../ros/perception/aav_things_rs/README.md) | Rust | things in Rust (rclrs) | `crates/things-core` | `crates/things-node` | `test/` | `config/` |
| [aav_voxels](../ros/perception/aav_voxels/README.md) | Python | A trial beside aav_lidar's flat scan | `aav_voxels/cubes.py` | `aav_voxels/voxels_node.py` | `test/` | - |
| [aav_voxels_rs](../ros/perception/aav_voxels_rs/README.md) | Rust | voxels in Rust (rclrs) | `crates/voxels` | `crates/voxels-node` | `test/` | - |

## ros/drivers

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_actuator_tests](../ros/drivers/aav_actuator_tests/README.md) | Python | The actuator tests (docs/tests.md stages 3 and 4) | `aav_actuator_tests/(9 modules)` | `aav_actuator_tests/actuator_test.py`, `aav_actuator_tests/car_test_node.py`, `aav_actuator_tests/hallway.py` | `test/` | - |
| [aav_behaviour_rs](../ros/drivers/aav_behaviour_rs/README.md) | Rust | The behaviour planner in Rust (rclrs) | `crates/behaviour-core` | `crates/behaviour-node` | `test/` | - |
| [aav_costmap](../ros/drivers/aav_costmap/README.md) | Rust | Our own costmap in Rust (rclrs) | `crates/costmap` | `crates/costmap-node` | - | - |
| [aav_diffusion_planner](../ros/drivers/aav_diffusion_planner/README.md) | Python | diffusion_planner | `aav_diffusion_planner/(4 modules)` | `aav_diffusion_planner/planner_node.py`, `aav_diffusion_planner/worker.py` | `test/` | - |
| [aav_diffusion_rs](../ros/drivers/aav_diffusion_rs/README.md) | Rust | The diffusion planner's model worker in Rust | `crates/diffusion-core` | - | - | - |
| [aav_drive_to_point](../ros/drivers/aav_drive_to_point/README.md) | Python | Drive-to-point driver | `aav_drive_to_point/(5 modules)` | `aav_drive_to_point/point_cli.py`, `aav_drive_to_point/point_driver.py` | `test/` | `config/`, `launch/` |
| [aav_drive_to_point_rs](../ros/drivers/aav_drive_to_point_rs/README.md) | Rust | point_driver and drive_to_point in Rust (rclrs) | `crates/point-core` | `crates/point-node` | - | - |
| [aav_language](../ros/drivers/aav_language/README.md) | Python | Plain-English driving | `aav_language/(9 modules)` | `aav_language/agent.py`, `aav_language/drive_server.py`, `aav_language/run_eval.py` | `test/` | - |
| [aav_mpc_lag](../ros/drivers/aav_mpc_lag/README.md) | Rust | mpc_lag_rs in Rust (rclrs) | `crates/mpc-lag` | `crates/mpc-node` | - | `config/` |
| [aav_mppi_lag](../ros/drivers/aav_mppi_lag/README.md) | Rust | mppi_lag in Rust (rclrs) | `crates/mppi-lag` | `crates/mppi-node` | - | `config/` |
| [aav_navigation](../ros/drivers/aav_navigation/README.md) | launch, config | Navigation as a driver | - | - | `test/` | `config/`, `launch/` |
| [aav_navigator](../ros/drivers/aav_navigator/README.md) | Rust | Our own navigator in Rust (rclrs) | `crates/navigator` | `crates/navigator-node` | - | - |
| [aav_planner](../ros/drivers/aav_planner/README.md) | Rust | Our own path planner in Rust (rclrs) | `crates/lattice-planner` | `crates/planner-node` | - | - |
| [aav_rc](../ros/drivers/aav_rc/README.md) | Python | The RC transmitter as a ROS driver (TODO 48) | `aav_rc/rc_drive.py` | `aav_rc/rc_driver.py` | `test/` | `launch/` |
| [aav_rc_rs](../ros/drivers/aav_rc_rs/README.md) | Rust | rc_driver in Rust (rclrs) | `crates/rc-core` | `crates/rc-node` | - | - |
| [aav_routes](../ros/drivers/aav_routes/README.md) | Python | Street routes, for real streets later (the lot first) | `aav_routes/(3 modules)` | `aav_routes/street_router.py` | `test/` | `config/`, `launch/` |
| [aav_scripted](../ros/drivers/aav_scripted/README.md) | Python | Scripted driver | `aav_scripted/speed_profile.py` | `aav_scripted/scripted_driver.py` | `test/` | `config/`, `launch/` |
| [aav_smoother](../ros/drivers/aav_smoother/README.md) | Rust | Our own velocity smoother in Rust (rclrs), for a car | `crates/car-smoother` | `crates/smoother-node` | - | - |
| [aav_teleop](../ros/drivers/aav_teleop/README.md) | launch, config | Human drivers | - | - | `test/` | `config/`, `launch/` |
| [aav_track](../ros/drivers/aav_track/README.md) | Python | Trial path followers (pure pursuit, Stanley, LQR, MPC) that stand in for mppi_lag | `aav_track/(8 modules)` | `aav_track/track_node.py` | `test/` | `config/` |

## ros/command_mux

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_command_mux](../ros/command_mux/aav_command_mux/README.md) | launch, config | Configuration and launch for twist_mux | - | - | `test/` | `config/`, `launch/` |
| [aav_mux_state](../ros/command_mux/aav_mux_state/README.md) | Rust | command_mux_state (Rust, rclrs) | `crates/mux-state-core` | `crates/mux-state-node` | - | - |

## ros/ego

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_ego](../ros/ego/aav_ego/README.md) | Python | ego | `aav_ego/(9 modules)` | `aav_ego/ego_node.py`, `aav_ego/look_ahead_node.py` | `test/` | `config/`, `launch/` |
| [aav_ego_rs](../ros/ego/aav_ego_rs/README.md) | Rust | ego and look_ahead in Rust (rclrs), one process with both nodes | `crates/ego-core` | `crates/ego-node` | `test/` | - |

## ros/io_mux

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_io_mux](../ros/io_mux/aav_io_mux/README.md) | Python | The muxes at the two edges of the stack | `aav_io_mux/(4 modules)` | `aav_io_mux/output_mux.py`, `aav_io_mux/sensor_mux.py` | `test/` | `launch/` |
| [aav_io_mux_rs](../ros/io_mux/aav_io_mux_rs/README.md) | Rust | The sensor mux and the output mux in Rust (rclrs) | `crates/io-mux-core` | `crates/io-mux-node` | `test/` | - |

## ros/can_bridge

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_legacy](../ros/can_bridge/aav_legacy/README.md) | Python | Interim backend for the 2025-26 hardware | `aav_legacy/(5 modules)` | `aav_legacy/encoder_udp.py`, `aav_legacy/legacy_uart_bridge.py`, `aav_legacy/steering_limits.py` | `test/` | `config/`, `launch/` |
| [aav_legacy_rs](../ros/can_bridge/aav_legacy_rs/README.md) | Rust | legacy_uart_bridge in Rust (rclrs) | `crates/legacy-bridge` | `crates/legacy-node` | - | - |
| [aav_spine](../ros/can_bridge/aav_spine/README.md) | Python | Backend for the real car on the CAN spine | `aav_spine/(4 modules)` | `aav_spine/arm_when_open.py`, `aav_spine/spine_bridge.py` | `test/` | `config/`, `launch/` |
| [aav_spine_bridge](../ros/can_bridge/aav_spine_bridge/README.md) | Rust | spine_bridge in Rust (rclrs) | `crates/spine-bridge` | `crates/bridge-node` | - | - |

## ros/safety

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_ghost](../ros/safety/aav_ghost/README.md) | Python | Ghost obstacles | `aav_ghost/ghost.py` | `aav_ghost/ghost_cli.py`, `aav_ghost/ghost_obstacle.py` | `test/` | `config/` |
| [aav_joy_alive_rs](../ros/safety/aav_joy_alive_rs/README.md) | Rust | joy_alive in Rust (rclrs) | `crates/joy-alive-core` | `crates/joy-alive-node` | - | - |
| [aav_safety](../ros/safety/aav_safety/README.md) | Python | The sensor watchdog | `aav_safety/(6 modules)` | - | `test/` | `config/`, `launch/` |
| [aav_safety_rs](../ros/safety/aav_safety_rs/README.md) | Rust | The geofence, the stop hold and the goal leash in Rust (rclrs) | `crates/safety-core` | `crates/safety-node` | `test/` | - |
| [aav_self_test_rs](../ros/safety/aav_self_test_rs/README.md) | Rust | The boot self-test in Rust (rclrs), TODO 255 | `crates/self-test-core` | `crates/self-test-node` | `test/` | `launch/` |
| [aav_stop_zone](../ros/safety/aav_stop_zone/README.md) | Rust | The stop zone in Rust (rclrs) | `crates/stop-zone` | `crates/stop-zone-node` | - | - |
| [aav_watchdog_rs](../ros/safety/aav_watchdog_rs/README.md) | Rust | The sensor watchdog in Rust (rclrs) | `crates/watchdog-core` | `crates/watchdog-node` | `test/` | - |

## ros/sim

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_bench](../ros/sim/aav_bench/README.md) | Python | Bench mode's pretend sensors | `aav_bench/(4 modules)` | - | `test/` | `config/` |
| [aav_carla](../ros/sim/aav_carla/README.md) | Python | CARLA (0.10 on Unreal Engine 5, or 0.9.16) as the car, behind the same topics as the real one, through CARL... | `aav_carla/(5 modules)` | `aav_carla/carla_adapter.py`, `aav_carla/georef.py` | `test/` | `launch/` |
| [aav_sim_cameras](../ros/sim/aav_sim_cameras/README.md) | Rust | The simulator's cameras | `crates/camera-core` | - | `tests/` | `config/` |
| [aav_sim_models](../ros/sim/aav_sim_models/README.md) | Python | Ravensim's Python side (Ravensim | `aav_sim_models/(12 modules)` | `aav_sim_models/sim_adapter.py`, `aav_sim_models/sim_camera_relay.py`, `aav_sim_models/sim_esp32.py`, `aav_sim_models/sim_gps.py`, `aav_sim_models/sim_imu.py`, `aav_sim_models/sim_lidar_relay.py`, `aav_sim_models/sim_radar.py`, `aav_sim_models/truth_localizer.py` | `test/` | `launch/` |
| [aav_simulator](../ros/sim/aav_simulator/README.md) | Rust | Ravensim, our own simulator, in Rust (rclrs) | `crates/sim-core` | `crates/sim-node` | - | - |
| [aav_spine_sim](../ros/sim/aav_spine_sim/README.md) | Rust | spine_sim, for bench mode | `crates/bench-car`, `crates/spine-sim` | - | - | - |
| [aav_turtlesim](../ros/sim/aav_turtlesim/README.md) | Python | Backend that drives turtlesim like a car (bicycle model) and reports its speed as /vehicle/odom | - | `aav_turtlesim/turtlesim_adapter.py` | - | `launch/` |

## ros/hud

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_dev_window](../ros/hud/aav_dev_window/README.md) | Rust | The developer window, as a test of Bevy | `src` | - | - | `config/` |
| [aav_hud](../ros/hud/aav_hud/README.md) | Rust | hud | `crates/debug-app`, `crates/debug-core`, `crates/hud-app`, `crates/hud-core`, `crates/map-app`, `crates/map-core`, `crates/panel-app`, `crates/panel-core` | - | - | `config/`, `launch/` |

## ros/bringup

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_bringup](../ros/bringup/aav_bringup/README.md) | launch, config | Start the whole stack | - | - | `test/` | `config/`, `launch/` |
| [aav_description](../ros/bringup/aav_description/README.md) | launch, config | The car's measurements (config/vehicle.yaml), its URDF, launch files to publish and view its frames, and a... | `aav_description/(8 modules)` | - | `test/` | `config/`, `launch/` |
| [aav_run_info](../ros/bringup/aav_run_info/README.md) | Python | run_info | `aav_run_info/algorithms.py` | `aav_run_info/run_info.py` | `test/` | - |

## ros/ecolo_msgs

| Package | Language | What | Logic | Node | Tests | Config |
|---|---|---|---|---|---|---|
| [aav_interfaces](../ros/ecolo_msgs/aav_interfaces/README.md) | messages | AAV message and action definitions | - | - | - | - |

## firmware/spine/crates (Rust, `no_std` where it runs on a board)

| Crate | What |
|---|---|
| `aav-inputs` | Clean digital inputs for every board |
| `ad5311` | The throttle DAC |
| `adaptive-brake` | The adaptive brake |
| `app-update` | The checks around a firmware update over WiFi (the ESP32's wifi-update) |
| `hd44780` | A character LCD with an HD44780 controller (16x2, 20x4), on six GPIO pins in 4-bit mode, write only |
| `ibus` | The FlySky iBUS protocol from the RC receiver |
| `jrk-g2` | Pololu Jrk G2 motor controller over I2C |
| `legacy-link` | The 2025-26 UART link from the Motor-Pi to the ESP32 |
| `mcp2515` | The Microchip MCP2515 CAN controller over SPI |
| `pca9555` | The PCA9555 16-bit I/O expander on I2C |
| `sentinel` | The status display's logic (the 2025-26 AAV_sentinel Pico 2) |
| `spine-board` | The glue between spine-core and the car's devices, for any board |
| `spine-core` | Hardware-independent spine logic |
| `status-lines` | The ESP32's JSON status lines (firmware/status) |
| `wheel-core` | The wheel nodes' logic, for any board |
| `wheel-encoder` | The wheel's two spoke sensors, counted the 2025-26 way, and the wheel encoder's UDP packet to the Motor Pi |

## Elsewhere

| Folder | Holds |
|---|---|
| `firmware/can/` | `aav.dbc`, the CAN contract, and its shared test vectors |
| `firmware/legacy/` | the 2025-26 ESP32 firmware, kept until the Rust one is flashed |
| `firmware/status/`, `firmware/udp/` | the boards' status lines; the wheel encoder's UDP vectors |
| `tools/` | scripts and checks (`pixi run ...`); their tests in `tools/tests/` |
| `docs/` | the guide; `interfaces.yaml` is the topic contract |
| `maps/`, `models/` | maps and trained models (most made, not kept in git) |
