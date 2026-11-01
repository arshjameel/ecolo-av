# aav_hud

## Overview

The car's screens, in Rust with raylib. Each is one program that is also its ROS
node (rclrs):

- `hud`, the in-car screen: speed, mode, the e-stop button, GO / STOP, steering,
  throttle and brake, GPS, topic health, the stop-sign alert and the camera;
- `map`, the big screen's map window: Nav2's map over an offline street map, the car,
  the plan, the goal, the distance left, and a place search;
- `debug_hud`, the debug screen, which opens as the **seeing screen**: the lidar's cubes
  in 3D (kept and fading), where each localizer puts the car (NDT too), AMCL's particles,
  NDT's map, Nav2's plan; on the right, NDT's map from above with every car, and NDT's
  match (its cells, the scan, the score). Key L swaps the right side for every sensor
  from its own topic. It only listens;
- `hw_panel`, the **hardware screen**: every sensor's number from its own node, and very
  simple buttons for the actuators (Speed +/-, steering, CENTRE, STOP, GO held, E-STOP),
  through twist_mux's `buttons` slot (45). Speed only on SIM and BENCH.

It was called `aav_hud_raylib` until 25 Oct, when the 2025-26 Flutter app and its Flask
back end (then `aav_hud`) went to `archive/2026-27-flutter-hud/` and this package took
the name.

A Cargo workspace that colcon builds through `CMakeLists.txt`. More in docs/hud.md and
docs/car-screens.md.

## Interface

### Crates

| Crate | Holds |
|---|---|
| [`hud-core`](./crates/hud-core/README.md) | the HUD's logic: no ROS, no raylib |
| [`map-core`](./crates/map-core/README.md) | the map window's logic: no ROS, no raylib |
| `crates/hud-app` | binary `hud`: the window (`ui.rs`) and the node (`ros.rs`, feature `ros`) |
| `crates/map-app` | binary `map`: the same shape, plus the street map and place search |
| [`debug-core`](./crates/debug-core/README.md) | the debug HUD's logic: cubes (kept, fading), frames, poses at a stamp, NDT's status and map, the keys, the camera; no ROS, no raylib |
| `crates/debug-app` | binary `debug_hud`: the window (`ui.rs`, instanced cubes, the seeing screen's top-downs), the node (`ros.rs`), `--demo` |
| [`panel-core`](./crates/panel-core/README.md) | the hardware panel's logic: the buttons, where they may drive (`gate`), every sensor's number, `safety.py`'s rules ported; no ROS, no raylib |
| `crates/panel-app` | binary `hw_panel`: the window (`ui.rs`), the node (`ros.rs`), `--demo`, `--self-test` |

### Node `hud` (topic names from `config/hud.yaml`)

| Direction | Parameter: default topic | Type |
|---|---|---|
| sub | `vehicle_status_topic`: `/vehicle/status` | aav_interfaces/VehicleStatus |
| sub | `gps_topic`: `/sensors/gps/fix`; `gps_filtered_topic`: `/localization/gps/fix` | sensor_msgs/NavSatFix |
| sub | `stop_sign_topic`: `/perception/stop_sign` | std_msgs/Bool |
| sub | `battery_topic`: `/vehicle/battery` | sensor_msgs/BatteryState |
| sub | `camera_topic`: `/sensors/camera/front_left/image_raw` | sensor_msgs/Image |
| sub | `source_topic`: `/system/source` (latched) | aav_interfaces/DataSource |
| sub | `watchdog_topic`: `/safety/sensor_watchdog/state` | std_msgs/String |
| sub | `output_mux_topic`: `/safety/output_mux/state` | std_msgs/String |
| pub | `estop_topic`: `/safety/estop` (only `true`) | std_msgs/Bool |
| pub | `heartbeat_topic`: `/hud/heartbeat` (10 Hz while it draws) | std_msgs/UInt64 |

Parameter `stale_after` (1.0 s): a quieter topic shows STALE.

### Node `map` (topic names from `config/map.yaml`)

| Direction | Parameter: default topic | Type |
|---|---|---|
| sub | `map_topic`: `/map` (latched) | nav_msgs/OccupancyGrid |
| sub | `odom_topic`: `/localization/odom/global` | nav_msgs/Odometry |
| sub | `plan_topic`: `/plan`; `street_route_topic`: `/drivers/nav/route/path` | nav_msgs/Path |
| sub, pub | `goal_topic`: `/goal_pose` (published after DRIVE HERE, then CONFIRM) | geometry_msgs/PoseStamped |
| sub | `source_topic`: `/system/source` | aav_interfaces/DataSource |
| sub | `watchdog_topic`: `/safety/sensor_watchdog/state` | std_msgs/String |

Other parameters: `stale_after` (1.0 s), `plan_stale_after` (3.0 s; `plan_kept_for`, 25 s, within
`final_approach`, 8 m, of the plan's end: Nav2 keeps its path there), `arrive_within`
(1.0 m), `street_map` (`maps/streets/ottawa.pmtiles`), `places` (`maps/places/ottawa.db`).

### Node `debug_hud` (topic names from `config/debug_hud.yaml`)

| Direction | Parameter: default topic | Type |
|---|---|---|
| sub | `lidar_topic`: `/sensors/lidar/points` (reliable, keep last 1) | sensor_msgs/PointCloud2 |
| sub | `local_odom_topic`: `/localization/odom/local`; `global_odom_topic`: `/localization/odom/global`; `truth_topic`: `/vehicle/gazebo/truth`; `wheel_odom_topic`: `/vehicle/odom` | nav_msgs/Odometry |
| sub | `amcl_topic`: `/localization/amcl/pose`; `ndt_pose_topic`: `/localization/ndt/pose` (purple) | geometry_msgs/PoseWithCovarianceStamped |
| sub | `amcl_particles_topic`: `/localization/amcl/particles` (best effort; small orange arrows) | nav2_msgs/ParticleCloud |
| sub | `ndt_status_topic`: `/localization/ndt/status` (the panel's NDT section) | diagnostic_msgs/DiagnosticStatus |
| sub | `plan_topic`: `/plan` | nav_msgs/Path |
| sub | `tf_topic`: `/tf`; `tf_static_topic`: `/tf_static` (latched) | tf2_msgs/TFMessage |
| sub | `imu_topic`: `/sensors/imu/data`; `gps_topic`: `/sensors/gps/fix` | sensor_msgs/Imu, NavSatFix |
| sub | `wheels_topic`: `/vehicle/wheels`; `vehicle_status_topic`: `/vehicle/status` | aav_interfaces/WheelSpeeds, VehicleStatus |
| sub | `track_status_topic`: `/drivers/nav/track/status` | diagnostic_msgs/DiagnosticStatus |
| sub | `source_topic`: `/system/source` (latched) | aav_interfaces/DataSource |
| call | `track_parameters_service`: `/track/set_parameters` (keys 1-4: `tracker`; only while `/system/source` says SIM) | rcl_interfaces/SetParameters |

Other parameters: `odom_frame`, `base_frame`, `map_frame`; `stale_after` (1.0 s),
`cube_size` (0.2 m), `max_cubes` (20,000), `pose_window` (1.0 s), `ground_height` (0.25 m),
`keep_cubes_for` (2.0 s: key K's cubes fade over it), `ndt_map` (`maps/ndt/test_lot.pcd`,
relative to where it starts; key M) and `ndt_map_cell` (1.0 m).

### Node `hw_panel` (topic names from `config/hw_panel.yaml`)

| Direction | Parameter: default topic | Type |
|---|---|---|
| pub | `buttons_topic`: `/drivers/buttons/cmd_vel` (only while GO is held, then 0.5 s of zeros; 10 Hz, only while the window draws) | geometry_msgs/TwistStamped |
| pub, sub | `estop_topic`: `/safety/estop` (sends only `true`, 5 times; a true heard zeroes the buttons) | std_msgs/Bool |
| sub | `control_topic`: `/control/cmd_vel` (someone else's command for 0.3 s: a takeover) | geometry_msgs/TwistStamped |
| sub | `geofence_lock_topic`: `/safety/geofence/lock`; `no_timeout_lock_topic`: `/safety/legacy_uart_bridge/lock` (latched) | std_msgs/Bool |
| sub | `source_topic`: `/system/source` (latched) | aav_interfaces/DataSource |
| sub | `vehicle_status_topic`: `/vehicle/status`; `steering_topic`, `throttle_topic`, `brake_topic`: `/vehicle/steering`, `/vehicle/throttle`, `/vehicle/brake`; `wheels_topic`: `/vehicle/wheels` | aav_interfaces/VehicleStatus, ActuatorState, WheelSpeeds |
| sub | `imu_topic`: `/sensors/imu/data`; `gps_topic`: `/sensors/gps/fix`; `battery_topic`: `/vehicle/battery` | sensor_msgs/Imu, NavSatFix, BatteryState |
| sub | `lidar_topic`, `radar_topic`: `/sensors/lidar/points`, `/sensors/radar/points`; `camera_topic`: `/sensors/camera/front_left/image_raw` (counted only: Hz; best effort, keep last 1) | sensor_msgs/PointCloud2, Image |

Other parameters: `stale_after` (1.0 s), `speed_step` (0.1 m/s), `steer_step` (0.05 rad),
`creep` (0.1 m/s, CENTRE), `sim_cap` (1.0 m/s), `stands_cap` (0.3 m/s).

### Launch and commands

| Command | Starts |
|---|---|
| `pixi run hud [fullscreen:=true] [monitor:=HDMI-1]` | `hud.launch.py` |
| `pixi run map [fullscreen:=true] [monitor:=HDMI-1] [streets:=] [places:=]` | `map.launch.py`, with `vehicle.yaml` and `navsat.yaml` |
| `cargo run -p hud-app --no-default-features -- --demo` | the HUD with pretend data, no ROS |
| `cargo run -p map-app --no-default-features -- --demo` | the map with pretend data |
| `pixi run debug-hud`; `pixi run debugdemo` | `debug_hud.launch.py`; `demo` plus the debug HUD |
| `cargo run --release -p debug-app --no-default-features -- --demo [--bench-cubes 20000]` | the debug HUD with pretend data; the drawing test |
| `pixi run hw-panel [stands:=true] [fullscreen:=true]`; `pixi run hw-panel-demo` | `hw_panel.launch.py` (localhost-only); the panel with a pretend car |
| `hw_panel --self-test 0.3:8` | for checks: Speed + to 0.3 m/s, GO held 8 s, through the same buttons and rules; prints asked and seen |

`sim.launch.py hud:=true` and `map:=true` start them too. On the car,
`tools/car_screen.sh` starts them at boot; `car.launch.py` does not.

## Invariants

- Topic names live only in `config/*.yaml`, never in Rust. They are built in as the
  defaults, so `--demo` shows the same names.
- raylib owns the main thread (macOS needs it); rclrs spins on a thread called `ros`
  and sends news over a channel.
- The e-stop button only ever publishes `true`: five, 0.1 s apart, reliable, from the
  ROS thread's 10 ms timer, so a slow frame can't delay it.
- A heartbeat goes out only while the drawn-frame count rises: it proves the screen
  and the e-stop timer both run. The sensor watchdog stops Nav2 without it.
- `/system/source` and `/map` are read transient local, so a late window still gets them.
- `map` publishes a goal only after DRIVE HERE then CONFIRM, and refuses one off Nav2's
  map or with no car position.
- The logic lives in `hud-core`, `map-core`, `debug-core` and `panel-core`, tested without ROS or a screen.
- `debug_hud` publishes nothing and never touches an actuator. Its one call swaps a
  **simulated** trial tracker (keys 1-4), and only while `/system/source` says SIM.
- `hw_panel` is the only screen that drives, and only through twist_mux (slot 45), ego
  and the spine. The car moves only while GO is held; a frozen window stops it (the
  command goes out only while the frame count rises). An e-stop, a takeover or a source
  change zeroes the buttons; GO and Speed + must be pressed again. Speed buttons: SIM and
  BENCH only (1.0 m/s); off on the real car, on stands too until TODO 49b (the reason is
  on screen).
- `debug_hud`'s clouds never go through the shared queue: a newest-only slot to the
  `cubes` thread, then a newest-only slot to the screen. Each is placed with the local
  EKF's pose at its own stamp.
- Kept cubes age by the clouds' stamps, never the window's clock: when the lidar stops,
  they stop fading (and the lidar row says STALE).

## Side Effects

- Opens a window: it needs a screen, and fails without one.
- Publishes `/safety/estop`, `/hud/heartbeat` and `/goal_pose` (a goal drives the car);
  `hw_panel` publishes `/drivers/buttons/cmd_vel` (the buttons drive the car).
- `map` reads the PMTiles and SQLite files from disk; `debug_hud` reads `ndt_map` (a PCD)
  once at start-up, if it is there.
- On Linux the build needs X11 and OpenGL headers; without them CMake skips this
  package with a warning.

## Verification

```sh
cd ros/hud/aav_hud && cargo test --locked -p hud-core -p map-core -p debug-core -p panel-core
```

The nodes themselves, with ROS (after `pixi run build`; each on its own domain and topics;
`LIBRARY_PATH` because ros-env links every message package in the workspace):

```sh
pixi run bash -c 'source install/setup.bash && export ROS_DOMAIN_ID=94 \
  LIBRARY_PATH=$(ls -d $PWD/install/*/lib | tr "\n" ":")$CONDA_PREFIX/lib \
  && cd ros/hud/aav_hud && cargo test --release -p hud-app -p debug-app -p panel-app'
```

The drawing test (the design's pass mark: 20,000 cubes at 30 fps or more, release build,
a team Mac): `pixi run debug-hud-demo --bench-cubes 20000` prints the frame rate over 10 s.
Measured 29 Oct on Ben's Mac: **358 fps** (kept cubes, NDT's map and AMCL's particles on).

`tools/test.sh` runs the logic's tests; `pixi run test` runs everything else.
`colcon build` needs Rust 1.88 or newer (`pixi run build`).
