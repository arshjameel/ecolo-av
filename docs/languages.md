# Python and Rust (and the C++ we don't write)

New ROS nodes are Rust (Ben, 16 Nov: "lets remove python were possible. I trust rust more
then python"); the older ones are Python, and the firmware and hardware drivers are Rust. The C++ we
inherited is now Rust (Ben, 10 and 15 Oct): none of our ROS code is C++. Only today's
ESP32 firmware still is, and the hardware lead's wheel-sensor Arduino (Ben, 23 Oct). ROS 2 lets each program pick its own language, because
programs only meet through messages.

## How languages fit into ROS 2

![How the languages stack up](img/languages.svg)

- Every node is its own program, so a Python node and a C++ node talk without knowing it.
- Each `.msg` file is generated into Python, C++ and Rust, so every language sees the
  same types.
- The client libraries all wrap one C core, `rcl`, so they behave the same.

## What we use, and where

| Code | Language | Why |
|---|---|---|
| New ROS nodes | Rust (`rclrs`) | Ben, 16 Nov: "I trust rust more then python". Exceptions: ML on the GPU (train in PyTorch; inference later through ONNX), launch files, tools and tests |
| ego + look_ahead, behaviour, the sensor watchdog, localization_health | Rust (`rclrs`): `aav_ego_rs`, `aav_behaviour_rs`, `aav_watchdog_rs`, `aav_localization_health_rs` | Ports of the Python nodes, the same answers. The default on the car and in the simulator since 16 Nov (Ben); `*_impl:=python` falls back until a car day proves them |
| The sensor mux and the output mux, `aav_io_mux_rs` | Rust (`rclrs`) | Ports of `aav_io_mux`'s nodes (Ben, 17 Nov: "Rust tomorrow"): the same answers, step by step. The default; `io_mux_impl:=python` falls back |
| The point driver and its client, `aav_drive_to_point_rs`; arm_when_open (`aav_spine_bridge`) | Rust (`rclrs`) | Ports of `aav_drive_to_point`'s two nodes and `aav_spine`'s `arm_when_open` (18 Nov): the same answers, drive for drive. The default; `drive_to_point_impl:=python`, `arm_when_open_impl:=python` fall back |
| The ESP32's UART bridge and rc_driver, `aav_legacy_rs`, `aav_rc_rs` | Rust (`rclrs`) | Ports of `legacy_uart_bridge` and `rc_driver` (Ben, 17 Nov: "Rust tomorrow"): the same frames, byte for byte, packed by the firmware's own `legacy-link`. The default; `legacy_impl:=python`, `rc_impl:=python` fall back |
| `joy_alive`, the stop-sign watcher, `aav_joy_alive_rs`, `aav_stop_sign_rs` | Rust (`rclrs`) | Ports (Ben, 17 Nov: "I don't think we want to do this again later"): the same answers on shared vectors. The default; `stop_sign_impl:=python` falls back; the Python joy_alive removed 20 Nov |
| The geofence, the stop hold, the goal leash, `aav_safety_rs` | Rust (`rclrs`) | Ports of `aav_safety`'s three nodes (batch 7, 19 Nov): the Python's answers on shared vectors, to the bit. The only ones; the Python nodes removed 20 Nov |
| The GPS heading, `aav_gps_heading_rs` | Rust (`rclrs`) | A port of `aav_gps_heading` (batch 7, 19 Nov): `course.py`'s answers to the bit on 4,000 cases. The default; the Python removed 20 Nov |
| The lane finder, `aav_lane_finder_rs` | Rust (`rclrs`), no OpenCV | A port of `aav_lane_finder` (18 Nov): the Python's lanes on 45 pictures; 3.7 % of a core against 17.4 %. The default; `lane_finder_impl:=python` falls back |
| The camera stream, `aav_camera_stream_rs` | Rust (`rclrs`) | A port of `aav_camera_stream` (18 Nov), the default with no detector; with RF-DETR or YOLO the Python runs (the models stay Python). `camera_stream_impl:=python` falls back |
| AMCL, `aav_amcl_rs` | Rust (`rclrs`) | A port of Nav2's `nav2_amcl`, its last node (18 Nov, TODO 231): a tie with stock on the lot. The only one; stock `nav2_amcl` removed 20 Nov (TODO 232) |
| Our older ROS nodes: drivers, bridges, adapters | Python | Glue that runs 50 times a second or less. Ported to Rust where possible (Ben, 16 Nov) |
| Launch files | Python | That's how ROS 2 launch works |
| Stock packages: the EKFs, twist_mux (Nav2 until 17 Nov) | C++ | Not ours: we only configure them |
| The IMU driver, `aav_imu_yahboom` | Rust (`rclrs`) | Ben's rewrite of the 2025-26 C++ driver, which it replaced on 11 Oct |
| The radar clustering, `aav_radar_clusters` | Rust (`rclrs`) | A rewrite of the 2025-26 C++ node, which it replaced on 15 Oct |
| The lidar's cubes, `aav_voxels_rs` | Rust (`rclrs`) | A port of `aav_voxels`' Python node (TODO 185, 15 Nov): the same cubes, 8 times faster. Picked with `voxels_impl:=rust`; the Python stays the default until the Rust one has run on the car |
| The flat scan and the things, `aav_lidar_rs`, `aav_things_rs` | Rust (`rclrs`) | Ports of `aav_lidar`'s and `aav_things`' Python nodes (TODO 185, 17 Nov): the same scan and things, at a third and a fifth of the CPU. The default in the simulator and on the car; `lidar_impl:=python` falls back; the Python things removed 20 Nov |
| The CAN bridge, `aav_spine_bridge` | Rust (`rclrs`) | A port of `aav_spine`'s Python node (T1.8), on the firmware's own CAN code. The default since 23 Oct; the Python one stays until the Rust one has run on the car (no record of that yet) |
| The spine firmware, `firmware/spine` | Rust, `no_std` | Memory-safe, and its logic is tested on a laptop |
| The in-car HUD and the map window, `aav_hud` | Rust (`rclrs`, raylib) | One program that is also its ROS node (Ben, 11 Oct). They stay raylib, on the Pi (Ben, 5 Nov) |
| Screens on a laptop or desktop: the developer window (`aav_dev_window`), our simulator's cameras | Rust (Bevy) | Bevy only off the car: the Orin runs no graphics and no simulator (Ben, 5 Nov) |
| Our navigation (costmap, planner, navigator, smoother, MPPI with lag, stop zone) and our simulator | Rust (`rclrs`) | Where Rust pays off; each replaced Nav2's or Gazebo's part once it won |
| Bench mode's spine, `aav_spine_sim` | Rust (`rclrs`) | Runs `spine-core` itself on the CAN frames |
| Today's ESP32 firmware | C++ (Arduino, PlatformIO) | Inherited. Its Rust version (`boards/esp32-umod`) replaces it once flashed and proven: [Firmware](firmware.md) says where that stands |
| The wheel sensors' Arduino | C++ (Arduino) | The hardware lead's board: it sends CAN 0x200/0x201 (the exception in CLAUDE.md) |
| The wheel encoder and status display Pico 2s | Rust, Embassy | Rewrites of the 2025-26 C++, 15 Oct; not yet flashed |

## Is C++ too risky?

Memory bugs and crashes are a fair worry with C++. In this stack:
- The C++ we run is almost all stock packages that thousands of robots use. We wrote none
  of it.
- Our own code is Python and Rust, and both are memory-safe. The inherited C++ is now
  Rust; only today's ESP32 firmware is still C++, until its Rust version replaces it (and the
  hardware lead's Arduino).
- Each node is a separate process. A crash takes down one node, not the whole system.
- Safety never rests on one program: ego's timeout and the spine's watchdog stop the car
  if a node dies.

## What if everything were Rust?

| You'd get | You'd give up |
|---|---|
| Memory safety and speed in every node | Official support: `rclpy` and `rclcpp` are official; `rclrs` is a community project |
| One language from the firmware up | Examples: most ROS answers and tutorials are Python or C++ |
| Cargo, and its tooling | Simple builds: colcon needs a cargo plugin, and builds get slower |
| | Nothing in robot_localization or twist_mux: they stay C++, and we only configure them |
| | Launch files: they stay Python |
| | An easy hand-over: the 2027-28 team has to know Rust |

Where Rust stands (written October 2026):
- **Message crates ship with Jazzy.** Our Pixi install already has them, for example
  `share/sensor_msgs/rust/`.
- **`rclrs`**, the library for writing nodes, comes from crates.io and is still 0.x.
- **It works here.** The IMU driver (`aav_imu_yahboom`), the radar clustering, the CAN
  bridge (`aav_spine_bridge`) and the HUD are `rclrs` nodes, built by colcon. A first
  example, `examples/rust-listener`, showed it in October; it was removed on 26 Oct,
  once these real nodes showed the same.

**After adding a message type: `library not found for -l<package>__rosidl_typesupport_c`.**
Seen 14 Nov, when `RoadAhead` was added. rclrs's build script finds the libraries' folders
once and keeps them until `ROS_DISTRO` changes; `ros-env` looks again whenever the messages
change. So an old build folder links a message package without its folder. A fresh build
(CI) never sees it. On your laptop, clear rclrs's kept answer and build again:

```sh
rm -rf build/*/target/release/build/rclrs-* build/*/target/release/.fingerprint/rclrs-*
pixi run build
```

## Launch overrides: `namespace='/'` on every Rust node

**The trap (found 18 Nov).** rclrs 0.8 reads a parameter file's `/**` entries first, then
the entry under the node's own name, whatever order the files came in. rclpy and rclcpp
take the files in order. launch_ros writes a launch dict under `/**` when the Node has no
namespace. So with a config YAML (`legacy_uart_bridge:`) and a launch dict:

| Who started | The YAML says | Launch says | Python uses | Rust used |
|---|---|---|---|---|
| `legacy_uart_bridge` | `port: /dev/ttyAMA0` | `port:=socket://...` | launch's | **the YAML's** |

**The fix.** Give the Node `namespace='/'` (the same names and topics). launch then writes
its dicts under `/legacy_uart_bridge` too, and the later file wins in both languages.
`tools/tests/test_rust_launch_params.py` fails a Rust Node with several parameter sources
and no namespace; `aav_bringup`'s `test_rust_overrides.py` starts ego, the legacy bridge and
the output mux in both languages and reads the values back.

**Also:** rclrs matches only `/**` and the exact name. A YAML key such as `/**/joy_alive`
reaches the Python node and **not** the Rust one: name a Rust node in full.

What the audit found (18 Nov, every launch profile run without starting anything):

| Node (Rust) | Key in both | YAML | Launch | Was it lost? | Now |
|---|---|---|---|---|---|
| `legacy_uart_bridge` | `max_steering_angle` | 0.5 (legacy.yaml) | vehicle.yaml's | yes, hidden: both 0.5 today; a new measurement would never have reached Rust | `namespace='/'` |
| `legacy_uart_bridge` | `port`, `frame_version` | legacy.yaml | new `port:=`, `frame_version:=` | (no launch argument before) | tested both languages |
| `hud` | `max_speed` | hud.yaml | ego's cap, `max_speed:=` | yes, with `max_speed:=0.3`: the redline kept hud.yaml's 1.0 | `namespace='/'` |
| `ego` (rust), `point_driver` | - | ego.yaml as a dict / one file per node | | no | already safe |
| `ndt`, `lane_finder` | - | | | no: fixed before (5 and 18 Nov) | `namespace='/'` |
| spine_bridge, lidar_scan, voxels, things, mppi_lag, mpc_lag_rs, velocity_smoother, localization_health, sensor_watchdog, imu_node, behaviour, rc_driver, spine_sim | none today | | | no | `namespace='/'`, so a future key can't be lost |
| `joy_alive` | its whole YAML | `/**/joy_alive:` | | would have been: rclrs skips wildcards (its built-in values equal the file's) | fixed 18 Nov: `/drivers/joy/joy_alive:` |

## Rust on the Pi and the ESP32

| Target | Rust there | A ROS node? |
|---|---|---|
| Orin, Raspberry Pi (Linux, ARM) | `aarch64-unknown-linux-gnu` | Yes, with `rclrs` |
| ESP32 (bare metal) | `no_std` with `esp-hal` | No: today it talks UART to `legacy_uart_bridge` on the Motor Pi (`actuators:=uart`, the default); CAN to `spine_bridge` with `actuators:=can`. No micro-ROS for now: we think again once we see the hardware (Ben, 11 Oct) |
| Raspberry Pi Pico 2 | `no_std`, `thumbv8m.main-none-eabihf` | No, the same as the ESP32 |

## The plan

| When | What |
|---|---|
| Now (16 Nov) | Rust for new ROS nodes, the firmware (`spine-core`, tested on a laptop) and the four ports above, the car's defaults; Python for ML, launch files, tools and tests |
| Done | the first `rclrs` nodes: the IMU, the radar clustering, the CAN bridge, the HUD; since 5 Nov our own navigation (Nav2's parts ported) and our simulator |
| Later | port the other Python nodes where possible (Ben, 16 Nov) |
