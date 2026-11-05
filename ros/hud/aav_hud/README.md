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
  NDT's map, Nav2's plan, Nav2's own cubes (the voxels trial's, red wireframes, key V);
  on the right, NDT's map from above with every car, NDT's match (its cells, the scan,
  the score), Nav2's cubes' status and the localization health checks. Key L swaps
  the right side for every sensor from its own topic. It only listens;
- `hw_panel`, the **hardware screen**: every sensor's number from its own node, and very
  simple buttons for the actuators (Speed +/-, steering, CENTRE, STOP, GO held, E-STOP),
  through twist_mux's `buttons` slot (45). Speed only on SIM and BENCH. Its second page
  (Tab), **STEERING CAL**, finds the steering actuator's ends and centre (limp or jog,
  mark, equal limits, save) and checks them lock to lock, stopping on resistance.

It was called `aav_hud_raylib` until 25 Oct, when the 2025-26 Flutter app and its Flask
back end (then `aav_hud`) went to `archive/2026-27-flutter-hud/` and this package took
the name.

A Cargo workspace that colcon builds through `CMakeLists.txt`. More in docs/hud.md and
docs/car-screens.md.

**Colours (1 Nov; Initial D 2 Nov):** all four screens share one theme
(`hud_core::theme`), each chosen by `theme:` in its own yaml: `initial_d` (black, white
and Carleton's red, sharp: **the default on every screen**, Ben, 2 Nov), `mocha` (dark),
`latte` (light, daytime). `calm` still loads, as a copy of Mocha: no Calm theme is
coming (Ben, 3 Nov). Mocha and Latte are [Catppuccin](https://catppuccin.com)'s palettes, MIT
licence. In the glass themes panels are frosted glass (`hud_app::paint`); the map blurs
what's behind its boxes (`map-app/src/frost.rs`, `blur`); the debug screen's legends
are glass too. Initial D draws the same boxes black with thin white edges and sharp
corners (`Style::corner`). The localizers keep their colours in every theme (local
blue, global green, AMCL orange, NDT purple). docs/hud.md, "Themes".

## Interface

### Crates

| Crate | Holds |
|---|---|
| [`hud-core`](./crates/hud-core/README.md) | the HUD's logic: no ROS, no raylib |
| [`map-core`](./crates/map-core/README.md) | the map window's logic: no ROS, no raylib |
| `crates/hud-app` | binary `hud`: the window (`ui.rs`) and the node (`ros.rs`, feature `ros`); its library: the window's monitor (`window`) and drawing with a theme (`paint`: glass, the background), shared with `map` |
| `crates/map-app` | binary `map`: the same shape, plus the street map, place search and the blur behind the glass (`frost.rs`) |
| [`debug-core`](./crates/debug-core/README.md) | the debug HUD's logic: cubes (kept, fading), frames, poses at a stamp, NDT's status and map, the keys, the camera; no ROS, no raylib |
| `crates/debug-app` | binary `debug_hud`: the window (`ui.rs`, instanced cubes, the seeing screen's top-downs), the node (`ros.rs`), `--demo` |
| [`panel-core`](./crates/panel-core/README.md) | the hardware panel's logic: the buttons, where they may drive (`gate`), every sensor's number, `safety.py`'s rules ported, the steering calibration page (`calibration`, `steering_limits`, `sweep`); no ROS, no raylib |
| `crates/panel-app` | binary `hw_panel`: the window (`ui.rs`, the calibration page in `ui/calibration.rs`), the node (`ros.rs`), `--demo`, `--self-test`, `--cal-self-test` (`cal_script.rs`) |

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

Parameters `stale_after` (1.0 s): a quieter topic shows STALE; `theme` (`initial_d`, `mocha`, `latte`, `calm`).

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
(1.0 m), `street_map` (`maps/streets/ottawa.pmtiles`), `places` (`maps/places/ottawa.db`),
`theme` (`initial_d`), `blur` (true: the map blurred behind the glass), `car_marker`
(`outline`; `robot`: the car as the little robot, the easter egg).

### Node `debug_hud` (topic names from `config/debug_hud.yaml`)

| Direction | Parameter: default topic | Type |
|---|---|---|
| sub | `lidar_topic`: `/sensors/lidar/points` (reliable, keep last 1) | sensor_msgs/PointCloud2 |
| sub | `voxels_cubes_topic`: `/perception/voxels/cubes` (reliable, keep last 1, base_link; red wireframes, key V) | sensor_msgs/PointCloud2 |
| sub | `voxels_status_topic`: `/perception/voxels/status` (Nav2's cubes section: road fit, count, ms) | diagnostic_msgs/DiagnosticStatus |
| sub | `health_topic`: `/localization/health` (OK, or WARN and each check) | std_msgs/String |
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
relative to where it starts; key M), `ndt_map_cell` (1.0 m), `voxels_cube_size` (0.25 m),
`health_stale_after` (3.0 s: the health checks come once a second), `theme` (`initial_d`),
`easter_egg` (true: the little robot's eyes peek when all is quiet) and `easter_egg_after` (60 s).

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
| sub | `steering_raw_topic`: `/vehicle/steering_actuator/raw` (the calibration page: counts, the firmware's state) | aav_interfaces/ActuatorRaw |
| sub | `calibration_line_topic`: `/vehicle/steering_actuator/calibration` (the bridge's line) | std_msgs/String |
| pub | `steering_calibration_topic`: `/drivers/buttons/steering_calibration` (the calibration page's limp or jog, 10 Hz while it asks and the window draws; END: MODE_OFF 0.5 s) | aav_interfaces/SteeringCalibration |

Other parameters: `stale_after` (1.0 s), `speed_step` (0.1 m/s), `steer_step` (0.05 rad),
`creep` (0.1 m/s, CENTRE), `sim_cap` (1.0 m/s), `stands_cap` (0.3 m/s), `theme` (`initial_d`);
the calibration page's `jog_step` (25 counts), `jog_min`, `jog_max` (400, 3500),
`sweep_rate` (100 counts/s), `sweep_max_error` (150 counts), `sweep_stall` (0.6 s);
`easter_egg` (true: the little robot peeks when all is quiet, and drives across at
start-up and after a check passes) and `easter_egg_after` (60 s).
SAVE writes `~/.local/share/aav/steering-calibration/steering-<time>.txt` (a person copies
it; never vehicle.yaml).

### Launch and commands

| Command | Starts |
|---|---|
| `pixi run hud [fullscreen:=true] [monitor:=HDMI-1]` | `hud.launch.py` |
| `pixi run map [fullscreen:=true] [monitor:=HDMI-1] [streets:=] [places:=]` | `map.launch.py`, with `vehicle.yaml` and `navsat.yaml` |
| `cargo run -p hud-app --no-default-features -- --demo` | the HUD with pretend data, no ROS |
| `cargo run -p map-app --no-default-features -- --demo [--bench 6]` | the map with pretend data; `--bench` prints the frame rate (the blur's test) |
| `pixi run debug-hud`; `pixi run demo --debug` | `debug_hud.launch.py` (passes `--vehicle` and `--own-box`, the car's own box from `vehicle.self_box`); `demo` plus the debug HUD |
| `cargo run --release -p debug-app --no-default-features -- --demo [--bench-cubes 20000]` | the debug HUD with pretend data; the drawing test |
| `pixi run hw-panel [stands:=true] [fullscreen:=true]`; `pixi run hw-panel --demo` | `hw_panel.launch.py` (localhost-only); the panel with a pretend car |
| `hw_panel --self-test 0.3:8` | for checks: Speed + to 0.3 m/s, GO held 8 s, through the same buttons and rules; prints asked and seen |
| `map`, `hw_panel`, `debug_hud` `--demo --robot-now` | the little robot (the easter egg) at once, for screenshots (docs/hud.md, "The little robot") |
| `hw_panel --demo --page calibration [--demo-bind 2800] [--cal-self-test]` | the steering calibration page with a pretend steering actuator; the self-test limps, marks, holds and runs the lock-to-lock check (with a stiff spot it must stop) |

`sim.launch.py hud:=true` and `map:=true` start them too. On the car,
`tools/car_screen.sh` starts them at boot; `car.launch.py` does not.

## Invariants

- Colours live only in `hud_core::theme`, never in the drawing code. Its test checks
  every text and status pair for contrast (WCAG: 4.5 : 1 words, 3 : 1 shapes).
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
- `hw_panel`'s calibration page never drives (GO is off there) and asks nothing while it
  isn't allowed (an e-stop, the car moving, the speed or the position unseen); leaving
  the page ends its mode. Jog targets stay in `jog_min`..`jog_max`, the check's inside
  the measured limits; the check stops and holds on resistance. It never writes
  vehicle.yaml or the firmware: SAVE writes a file for a person to copy.
- `debug_hud`'s clouds never go through the shared queue: a newest-only slot to the
  `cubes` thread, then a newest-only slot to the screen. Each is placed with the local
  EKF's pose at its own stamp. Nav2's cubes go the same way, through their own slots
  and their own thread (`nav2_cubes`).
- Kept cubes age by the clouds' stamps, never the window's clock: when the lidar stops,
  they stop fading (and the lidar row says STALE).
- The little robot (the easter egg, a tribute to WALL-E) never covers a button, the
  e-stop or a line of words, never shows while the car moves or anything warns, and
  hides on any input or change. On the map it sits on the car's true outline and heading.

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
a team Mac): `pixi run debug-hud --demo --bench-cubes 20000` prints the frame rate over 10 s.
Measured 29 Oct on Ben's Mac: **358 fps** (kept cubes, NDT's map and AMCL's particles on).

`tools/test.sh` runs the logic's tests; `pixi run test` runs everything else.
`colcon build` needs Rust 1.88 or newer (`pixi run build`).
