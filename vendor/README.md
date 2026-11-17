# vendor/: other people's source, to look things up

Not ours, and not in git. Only this README and the lists are committed.
`pixi run vendor` downloads what the lists name, at the exact commits.

| List | Lands in | What's in it | Size |
|---|---|---|---|
| `ros2-src.repos` | `vendor/ros2-src/` | ROS 2 Jazzy's own source (`core/`), and the stock packages we use (`ecosystem/`): Nav2, robot_localization, ros_gz, twist_mux and more | about 550 MB |
| `ros2-rust.repos` | `vendor/ros2-rust/` | rclrs (ROS in Rust), its examples, and its build tools | about 13 MB |
| `gazebo.repos` | `vendor/gazebo/` | Gazebo Harmonic, the simulator `pixi run sim` ran until 16 Nov (Ravensim since), kept to read: 18 libraries at the releases `pixi.lock` installs, plus `ros_gz` and the DART physics engine | about 745 MB |

PlatformIO keeps its downloads here too, in `vendor/platformio/` (about 1.5 GB): the
compilers and framework for today's C++ boards. `pixi run -e firmware legacy-esp32`
fills it.

Espressif's QEMU lives here too, in `vendor/qemu/` (about 22 MB): the ESP32 emulator
(GPL-2.0, a tool only, never linked into our code) and its libgcrypt (LGPL-2.1+, from
conda-forge). `pixi run esp-qemu-tools` fills it, checked by the published SHA-256s; macOS
on Apple silicon only so far. `pixi run esp-qemu-check` uses it (docs/esp32-bench.md).

## Rules

- **Look, don't touch.** Never edit, build or commit anything in here. Our code lives in
  `ros/`, `firmware/` and `tools/`.
- **colcon never builds it:** `vendor/COLCON_IGNORE` keeps it out of `pixi run build`.
- **git never tracks it:** `.gitignore` keeps only this README, the lists and
  `COLCON_IGNORE`.
- **Searching it:** most search tools skip folders that git ignores. Use
  `grep -r <word> vendor/`, or `rg -u <word> vendor/`.

## Gazebo: where each part of our simulator lives

To chase a Gazebo bug, read the library that does that job. Each repo is at the release we
run; the comment above it in `gazebo.repos` names the release tag.

| Our simulator uses | Library | Where in `vendor/gazebo/` |
|---|---|---|
| the car's steering and drive (`AckermannSteering`) | gz-sim | `gz-sim/src/systems/ackermann_steering/` |
| odometry, joint states | gz-sim | `gz-sim/src/systems/odometry_publisher/`, `joint_state_publisher/` |
| the IMU and the GPS (`Imu`, `NavSat`) | gz-sim and gz-sensors | `gz-sim/src/systems/imu/`, `navsat/`; `gz-sensors/src/ImuSensor.cc`, `NavSatSensor.cc` |
| the lidar (`gpu_lidar`) and the cameras | gz-sensors, drawn by gz-rendering | `gz-sensors/src/GpuLidarSensor.cc`, `CameraSensor.cc`; `gz-rendering/` |
| contact, friction, the wheels on the ground | gz-physics, DART | `gz-sim/src/systems/physics/`; `gz-physics/dartsim/`; `dart/` |
| reading the world and the URDF | sdformat | `sdformat/` |
| messages between Gazebo and ROS (`parameter_bridge`) | ros_gz | `ros_gz/ros_gz_bridge/` |
| starting Gazebo from a launch file | ros_gz | `ros_gz/ros_gz_sim/` |
| messages and transport inside Gazebo | gz-msgs, gz-transport | `gz-msgs/`, `gz-transport/` |
| the window, the menus | gz-gui | `gz-gui/` |

Search it with `rg -u <word> vendor/gazebo/`. Gazebo was removed on 16 Nov; our side of it
became `ros/sim/aav_sim_models/` (Ravensim's Python half), and the URDF's Gazebo files were
deleted ([Gazebo, as it was](../docs/history/gazebo-as-it-was.md)).

**Found a real Gazebo bug?** Gazebo's guide is https://gazebosim.org/docs/latest/contributing/.
Report it on that library's GitHub issues, with its release tag and a small world that
shows it. To try a fix, clone the library somewhere else and build it there: never build
inside `vendor/`. `gazebo-tooling/gazebodistro` lists every library of each Gazebo
release (`collection-harmonic.yaml`), and `gazebo-tooling/release-tools` is how Gazebo
itself builds and releases them.

## Commands

| Command | Does |
|---|---|
| `pixi run vendor` | downloads what's missing, at the pinned commits; leaves the rest alone |
| `pixi run esp-qemu-tools` | downloads Espressif's QEMU and its libgcrypt into `vendor/qemu/` |
| `python3 tools/vendor.py --check` | says what's there, what's missing, and what's at another commit |
| `python3 tools/vendor.py --freeze ros2-src` | pins what's checked out now, to update a list |
