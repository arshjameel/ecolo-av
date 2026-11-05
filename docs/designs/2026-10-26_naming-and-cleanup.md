# Naming and cleanup: what we use, what to call it, where Python fits

Audit of 26 Oct 2026, for Ben's questions: "what is aav_goto", "remove stuff we aren't
using", "meaningful names, some organization", "is Python good for the jobs it's doing",
"do we use aav_lane_finder?". Read-only: nothing was changed.

**How "used" was checked:** by `grep`, not by the docs. That means the launch files
(`ros/bringup/aav_bringup/launch/*.py`), the machine files (`config/machines/*.yaml`),
`pixi.toml`, `tools/test.sh`, the CI workflow (`.github/workflows/test.yml`) and the imports.

| Class | Meaning |
|---|---|
| **in use** | a profile, a machine file, a task or CI starts it by default |
| **optional** | behind a flag or a task, and there's a reason to keep it |
| **dormant** | nothing starts it, and there's no near-term plan |
| **dead** | something else has replaced it |

## The short answers

| Question | Answer |
|---|---|
| What is `aav_goto`? | The **A-to-B driver**. It drives straight to one GPS or map point and stops on it. My pick for its new name is **`aav_drive_to_point`** (section 3) |
| Do we use `aav_lane_finder`? | **No, not by default.** The `lane` part is `false` on every machine, and `lane:=false` is the default in `sim` and `bench`. Only `bench-check --lane` and `sim-check --lane` run it, by hand. CI doesn't. It's a decided feature (`decisions.md`), so it stays **optional** |
| Is Python right? | **Yes, for every node today.** The stalls we measured came from Fast DDS, not from Python (TODO 11, 32). Watch 4 nodes on the Orin. The real safety gap is in the firmware: today's ESP32 has no command timeout (team-todo B2). Rewriting the bridge in another language doesn't fix that |

## 1. Inventory: is each thing used?

### ROS packages (37)

| Package | Does | Started by | Class |
|---|---|---|---|
| `aav_bringup` | the launch profiles, machine files, RViz layouts | every `pixi run` profile | in use |
| `aav_description` | `vehicle.yaml`, the URDF, `vehicle.py` | every profile | in use |
| `aav_legacy` `legacy_uart_bridge` | ego's commands to today's ESP32, over UART | `car` part `legacy_uart` (`motor_pi.yaml`, `all.yaml`) | in use |
| `aav_legacy` `encoder_udp` | the 2025-26 Pico's UDP wheel packets | part `legacy_encoder`: `false` in all 3 machine files | **dead** (plan D2: the Arduino on CAN) |
| `aav_spine` | `bridges.py` (every launch file imports it), `spine.launch.py`, `arm_when_open`, `can_codec.py`, and the Python `spine_bridge` | the `car` part `spine`; bench; `sim spine:=true` | in use (its Python bridge node: optional) |
| `aav_spine_bridge` | `spine_bridge` in Rust, the default (`bridges.DEFAULT = 'rust'`) | `car` part `spine`, bench, `sim spine:=true` | in use |
| `aav_mux` | twist_mux's config: driver slots, locks | every profile | in use |
| `aav_actuator_tests` | stage 3-4 actuator tests | `pixi run actuator-test` | in use (on demand) |
| `aav_goto` | A-to-B: drive to one point, stop on it | `car` part `goto` (orin, all), `sim`, `bench` | in use |
| `aav_language` | plain-English driving: MCP drive server, local-model agent | `-e mcp drive-server`, `-e llm talk` | optional |
| `aav_navigation` | Nav2's config, maps, behaviour trees | part `navigation`, `sim`, `bench`, `carla`, `replay` | in use |
| `aav_rc` | the RC transmitter through ROS | `legacy.launch.py rc:=true` (the default) on the Motor Pi | in use |
| `aav_routes` | street routes from Valhalla, as Nav2 GPS waypoints | only by hand (`ros2 launch aav_routes`); `-e routes router` | optional (plan S8) |
| `aav_scripted` | one speed profile: speed up, hold, stop | every profile, part `scripted` | in use |
| `aav_teleop` | joystick, keyboard, e-stop button | `laptop.yaml`, `sim`, `bench`, `turtle` | in use |
| `aav_interfaces` | our messages and actions | built by all | in use |
| `aav_ego` | Twist to Ackermann, within limits | every profile | in use |
| `aav_hud` | the HUD and the map window (Rust) | `pixi run hud`/`map`/`demo`, `tools/screens/` | in use |
| `aav_io_mux` | `sensor_mux`, `output_mux`: one data source, the real car only on real data | every profile | in use |
| `aav_gps_heading` | heading from the GPS velocity | `localization.launch.py gps_heading:=true` (default) | in use |
| `aav_localization` | the two EKFs and navsat_transform | every profile | in use |
| `aav_detector` | RF-DETR (default) or YOLO, `stop_sign_watcher`, `camera_obstacles` | `stop_signs` (orin), `detector` (laptop); `perception:=true` | in use (YOLO path: optional) |
| `aav_detector/training/` | 2 inherited files from the 2024-25 YOLO repo | nothing imports them | **dead** |
| `aav_lane_finder` | OpenCV lane lines | `lane:=true` only; part `lane: false` everywhere | optional |
| `aav_lane_geometry` | 2025-26 lane keeping (`control_interface`) and `LaneGeometry.msg` | `lane:=true\|fake` only | optional (the message is needed by `aav_lane_finder`) |
| `aav_lidar` | the VLP-16's cloud as a 2D scan for Nav2 | `navigation.launch.py lidar:=true` (on in orin, all, sim) | in use |
| `aav_radar_clusters` | DBSCAN on the radar (Rust) | bench, sim; part `radar_clusters: false` on the car | in use (sim, bench) |
| `aav_ghost` | a pretend person in Nav2's view | `ghost:=true`, `pixi run ghost` | optional (plan T2.8) |
| `aav_safety` | the sensor watchdog lock | every profile except turtle | in use |
| `aav_bench` | bench mode's mock sensors | `bench` | in use |
| `aav_carla` | CARLA as the car | `pixi run carla-car`; never run against a server | dormant (plan T2.9, a stretch goal) |
| `aav_gazebo` | Gazebo worlds and adapters | `sim`, CI's `sim-check --obstacle` | in use |
| `aav_spine_sim` | `spine-core` on CAN frames, a model car (Rust) | bench, `sim spine:=true` | in use |
| `aav_turtlesim` | turtlesim as the car | `turtle` | in use |
| `aav_camera_stream` | RTSP H.264 to images, plus a detector each | part `camera_stream` (orin) | in use (untested on the Orin) |
| `aav_imu_yahboom` | the IMU driver (Rust) | part `imu` (orin, all) | in use |
| `aav_sensors` | the sensors' launch and config | the `car` parts; bench lidar | in use |
| `battery_monitor` | the Victron over serial | part `battery: false` everywhere; the bench uses `mock_battery` | dormant |
| `ti_ros2_driver` `pcl_pub` | the TI radar over UART | part `radar: false` everywhere | dormant (team-todo C5 needs it) |
| `ti_ros2_driver`: `obstacle_extractor`, `mock_radar_publisher`, `benchmark.py`, `plotheatmap.py`, 5 launch files | the 2025-26 team's extras | nothing outside `archive/` | **dead** (replaced by `aav_radar_clusters` and `mock_radar`) |

### Firmware (`firmware/spine/`)

| Item | Does | Used by | Class |
|---|---|---|---|
| `boards/esp32-umod` | Rust for today's ESP32 board (UART, or CAN) | `test.sh`, `esp-*` tasks, `car-day-build`, QEMU check | optional: the fix for B2, waiting to be flashed |
| `boards/pico2` | the spine on a Pico 2 + MCP2515 | `test.sh` builds it; `pico spine build`/`pico spine flash` | dormant: the board choice (Q13) is the hardware lead's |
| `boards/wheel-encoder-pico2` | spokes to UDP via a W5500 | `test.sh` builds it; `pico wheel build`/`pico wheel flash` | **dead** (plan D2: the Arduino on CAN, decided 23 Oct) |
| `boards/sentinel-pico2` | the status display | `test.sh`; `sentinel-*` tasks | dormant: TODO "once we know the display is on the car" |
| `spine-core`, `spine-board`, `aav-inputs`, `ibus`, `jrk-g2`, `ad5311`, `pca9555` | the spine's logic and drivers | esp32-umod, pico2, `aav_spine_sim` | in use |
| `legacy-link` | the 13-byte UART link | esp32-umod, spine-board | in use |
| `wheel-core` | the WHEEL_SPEED frame | `aav_spine_sim`'s `bench-car` | in use (bench) |
| `wheel-encoder` | spoke counting (spine-board uses it) + the UDP packet | spine-board; the wheel-encoder board | in use (the UDP half: dead) |
| `mcp2515` | the CAN chip for a Pico 2 | pico2 only | dormant, with pico2 |
| `sentinel`, `hd44780` | the display's logic and LCD | sentinel-pico2 only | dormant, with it |
| `firmware/udp/` | the UDP wheel packet's vectors | `wheel-encoder`, `aav_legacy` tests | **dead**, with the encoder |
| `firmware/legacy/aav_umod_esp32` | the C++ the car runs today | `legacy-esp32`, `car-day-build` | in use |

### Examples

| Item | Used by | Class |
|---|---|---|
| `examples/rust-listener` | only `pixi run rust-listener`. No test, no CI. Four real rclrs nodes now show the same thing | **dead** |

### `tools/` scripts (71)

Every script is reachable: from a task, another script, a test, CI or a launch file. No
tool reads `archive/`. A few still name retired things (below).

| Class | Scripts |
|---|---|
| in use, run by a task, a test or CI | `test.sh`, `lint.sh`, `ros.sh`, `check_graph.py`, `bench_check.py`, `sim_check.py`, `gz_crash.py` (sim_check imports it), `ci_annotate.py`, `build_docs.py`, `gen_interfaces_md.py`, `graph_picture.py`, `layout_check.py`, `car_kit.sh`, `car_install.sh`, `car_day_check.py`, `car_day_build.sh`, `actuator_test.sh`, `goto.sh`, `keyboard.sh`, `demo_all.sh`, `sim_fault.sh`, `gz_window.sh`, `software_gl.sh`, `controller_compare.py`, `heading_check.py`, `timing_test.py`, `stalls.py`, `stream_check.py`, `calibrate_camera.py`, `camera_calibration_cv5.py`, `doctor.py`, `net_map.sh`, `net_map.py`, `bandwidth.py`, `topic_load.py`, `survey_car.sh`, `machine_survey.sh`, `keychain_askpass.sh`, `vendor.py`, `get_models.sh`, `get_mediamtx.sh`, `get_hud_map.sh`, `get_street_map.sh`, `get_routes.sh`, `router.sh`, `elf2uf2.py`, `esp32.sh`, `esp32_qemu.py`, `legacy_firmware.sh`, `atob_score.py`, `drive_metrics.py`, `can_load.py`, `export_model.py`, `vehicle_model.py`, `state_space_demo.py`, `mcp_server.py`, `mcp_parts.py`, `car_screen.sh`, `car_screens_install.sh` (screens/ units), `cyclonedds.xml` |
| in use, run by hand (documented) | `car_listen.sh`, `record.sh`, `replay_commands.sh`, `build_engine.py` (on the Orin), `fake_vlp16.py`, `make_map.py`, `progress_mail.py` (Ben's only) |
| one-off generators and analysis | `make_stop_sign.py`, `sim_frames.py`, `compare_detectors.py`, `gazebo_steering.py` |
| dormant | `get_carla_osm.sh` (with CARLA) |
| names retired things | `graph_picture.py` (`encoder_udp` in a regex), `car_listen.sh` (encoder_udp's UDP port), `net_map.py` ("the wheel-encoder Pico?"), `get_hud_map.sh` (pulls `ottawa.db` from the 2025-26 Flutter repo on GitHub, pinned: fine) |

### `pixi run` tasks (78, with the feature tasks)

| Class | Tasks |
|---|---|
| in use: CI | `build`, `test`, `sim-check` (`--obstacle`), `car-kit`, `-e mcp` (smoke test) |
| in use: daily | `lint`, `turtle`, `keyboard`, `sim`, `bench`, `bench-check`, `check`, `car`, `view`, `rviz`, `estop`, `clear estop`, `clear gate`, `clear watchdog`, `doctor`, `fastdds shm clean`, `docs`, `hud`, `hud --demo`, `map`, `map --demo`, `demo`, `graph`, `tools/ros.sh python3 tools/graph_picture.py`, `measure bandwidth`, `python3 tools/layout_check.py`, `ghost`, `sim-fault`, `tools/ros.sh tools/gz_window.sh` |
| in use: car day | `car-day-check`, `car-day-build`, `actuator-test`, `goto`, `measure atob`, `calibrate-camera`, `bash tools/survey_car.sh`, `bash tools/net_map.sh`, `legacy-esp32`, `esp tools`, `esp build`, `esp qemu-tools`, `esp qemu-check`, `detect`, `laptop-bench`, `python3 tools/export_model.py --format coreml` |
| analysis | `python3 tools/vehicle_model.py`, `python3 tools/state_space_demo.py`, `measure controllers`, `tools/ros.sh --local python3 tools/heading_check.py`, `measure timing`, `measure stream`, `python3 tools/can_load.py` |
| downloads | `get vendor`, `get models`, `get mediamtx`, `get-hud-map`, `get street-map` |
| optional | `mcp`, `drive-server`, `talk`, `tools/ros.sh python3 -m aav_language.run_eval`, `get routes`, `router`, `esp flash`, `pico spine tools` |
| dormant | `carla-car`, `get carla-osm`, `pico spine build`, `pico spine flash`, `pico sentinel build`, `pico sentinel flash` |
| **dead** | `rust-listener`, `pico wheel build`, `pico wheel flash` |

## 2. Removal candidates

Rule: anything `docs/plan.md`, `TODO.md` or `docs/designs/` still needs is **keep**.

| Item | Why we think it's unused | What depends on it | Risk | Recommendation |
|---|---|---|---|---|
| `examples/rust-listener` + task | only its own task; the IMU, radar, spine bridge and HUD are real rclrs examples | CLAUDE.md line 70, README (2 rows), `docs/languages.md`, `docs/infrastructure.md` | none | **remove now**; point CLAUDE.md at `aav_imu_yahboom` |
| `aav_detector/training/` | 2 inherited files; nothing imports them; `best.pt` was never in them | `docs/older-code.md` row | none | **remove now** (the source is on GitHub, as its PROVENANCE says) |
| `ti_ros2_driver` extras: `obstacle_extractor`, `mock_radar_publisher`, `benchmark.py`, `plotheatmap.py`, 5 `6843aop_*`/`mock_*` launch files | no launch outside `archive/` uses them; `aav_radar_clusters` and `mock_radar` replace them | `setup.py` entry points, PROVENANCE | low | **remove now**; keep `pcl_pub`, `TI.py`, `cfg/` (C5 needs them). Update PROVENANCE |
| `aav_lane_geometry` `control_test.launch.py`, `geometry_logger.py` | dev-only; the drift report found the launch file broken on a Mac | none | none | **remove now** |
| `encoder_udp`, part `legacy_encoder`, `firmware/udp/`, `boards/wheel-encoder-pico2`, `pico wheel build`/`pico wheel flash`, the UDP half of `wheel-encoder` | plan D2: the hardware lead's Arduino sends wheel speed on CAN (23 Oct); `false` in every machine file; the Pico wasn't seen on 21 Oct | `car.launch.py` PARTS, `test_car_launch.py`, `legacy.launch.py`, `test.sh` board list, `aav_language/safety.py`, ~25 docs | low, but TODO line 205 still says "flash them" | **move to `archive/`** once Q5 closes (the Arduino's frames seen on the car). Edit TODO 205 to the sentinel only. Keep `wheel-encoder`'s spoke counting: `spine-board` uses it |
| Python `spine_bridge` (`spine_bridge.py`, `output_gate.py`, `wheel_speed.py`) | Rust is the default since 23 Oct | `bridge:=python`, `bench-check --python-bridge`, `aav_io_mux`'s `test_real_backend_guard.py` | medium: the fallback if the Rust one fails on the car | **keep** until the Rust one has run on the car ("kept until", `bridges.py`). Then remove the node. Keep `can_codec.py`: `gen_test_vectors.py` and the DBC rule need it |
| `aav_lane_finder`, `aav_lane_geometry` | off everywhere by default | decisions.md line 44, features.md, `sim-check`/`bench-check --lane` | — | **keep** (optional, decided) |
| YOLO path in `aav_detector` | RF-DETR is the default everywhere | plan.md line 61, team-todo D3, `compare_detectors.py`, `get models`, Ultralytics in `-e ml` | AGPL-3.0 licence (`model-comparison.md`) | **keep** for now. Drop it, and Ultralytics, if TODO 417's `best.pt` is never found |
| `aav_carla`, `get carla-osm` | never run against CARLA | plan T2.9 (stretch) | — | **keep** (dormant) |
| `boards/pico2`, `mcp2515` | never flashed; the board is open | Q13, the hardware lead's choice | — | **keep** (it's cheap: `test.sh` builds it) |
| `boards/sentinel-pico2`, `sentinel`, `hd44780` | nothing sends it packets | TODO "once we know the display is on the car" | — | **keep** (dormant) |
| `battery_monitor` | `battery: false` everywhere | a sensor that's on the car | — | **keep** |

## 3. Names

### `aav_goto` first

It is the **A-to-B driver**: drive the straight line to point B, stop on B, smoothly.
Unlike Nav2, it has no map and no obstacle avoidance; it has its own stopping law. "goto"
names a keyword, not a job.

| | A. `aav_drive_to_point` | B. `aav_a_to_b` | C. `aav_stop_at_point` |
|---|---|---|---|
| Node | `point_driver` | `a_to_b_driver` | `stop_at_driver` |
| twist_mux slot, topic | `point`, `/drivers/point/cmd_vel` | `atob`, `/drivers/atob/cmd_vel` | `stop_at`, `/drivers/stop_at/cmd_vel` |
| Action, type | `/drive_to_point`, `DriveToPoint` | `/a_to_b`, `AToB` | `/stop_at_point`, `StopAtPoint` |
| Task | `pixi run drive-to-point` | `pixi run atob` (beside `measure atob`) | `pixi run stop-at` |
| For | says the job in plain words; matches `/scripted_drive`'s verb style | matches `docs/a-to-b.md`, `measure atob` and plan D4's "the A-to-B driver" | names what makes it different: the stop |
| Against | longest | "A to B" is jargon, and Nav2 goes A to B too | odd as a slot name; hides the driving |

**Pick: A, `aav_drive_to_point`.** A new reader understands it with no doc. Keep "A to B"
as the name of the demo and of `measure atob`.

**What it touches** (79 files mention `goto`, outside `archive/` and the HTML):

| Where | Change |
|---|---|
| package | folder, `package.xml`, `setup.py`, `setup.cfg`, `resource/`, module `aav_goto/` → `aav_drive_to_point/`, `config/goto.yaml`, `launch/goto.launch.py` |
| contract (`docs/interfaces.yaml`, then `interfaces.md`) | topic `/drivers/goto/cmd_vel`, action `/goto`, node `goto_driver` in 6 `sub:` lists, the `/fromLL` note |
| `aav_interfaces` | `GoTo.action` → `DriveToPoint.action`, CMakeLists |
| twist_mux | slot `goto` (priority 25) in `aav_mux/config/twist_mux.yaml` |
| launch | `car.launch.py` (PARTS, include), `sim.launch.py`, `bench.launch.py`, machine files `orin.yaml`, `all.yaml` (key `goto`) |
| tests | `test_sim_launch.py`, the package's tests, `test.sh`'s list |
| tools | `goto.sh`, `atob_score.py` (`COMMANDS`, `FEEDBACK = '/goto/_action/feedback'`), `sim_check.py`, `record.sh`, `drive_metrics.py`, `build_docs.py`, `tools/README.md` |
| pixi, README | the task, its "Every command" row |
| docs | `a-to-b.md`, `naming.md`, `features.md`, `status.md`, `how-it-fits.md`, about 20 more; the three graph SVGs |

**Cost:** a day with tests. **Contract: yes, team-visible** (a topic, an action, its
type, a slot). Bags recorded before the change keep `/goto/...`, so `measure atob` needs
the old feedback name as a fallback. Do it before the car's first A-to-B run, so no car
bag has the old name.

### Other names

| Now | Problem | Better | Touches | Contract? | Worth it? |
|---|---|---|---|---|---|
| `aav_lane_geometry` + slot `ai` | named after its message; it's the lane-keeping **driver**; "ai" is a promise, and lane keeping isn't AI | `aav_lane_keeping` (in `ros/drivers/`), node `lane_keeper`, slot `lane`: `/drivers/lane/cmd_vel`, `/drivers/lane/status`, `/drivers/lane/state`. `LaneGeometry.msg` moves to `aav_interfaces` | `aav_lane_finder` (its import), twist_mux, sensor_watchdog's "AI slot", interfaces.yaml, naming.md, launch files, PROVENANCE | **yes** | yes: the next thing after `aav_goto` |
| `aav_legacy`, part `legacy_uart`, node `legacy_uart_bridge` | "legacy" will be wrong: the Rust `esp32-umod` speaks the same UART link by default | `aav_esp32_uart`, part `esp32_uart`, node `esp32_uart_bridge` | car.launch, motor_pi/all.yaml, graph_picture, esp32_qemu.py, car_listen.sh, interfaces.yaml `sub:` lists, docs | node name only (no topic) | yes, together with the encoder removal |
| `aav_mux` | which mux? There are three | `aav_command_mux` (its folder's name) | `package.xml` deps, 6 launch includes, docs | no | yes, cheap |
| `aav_io_mux` | neither node muxes: one checks sources, one gates ego's output | `aav_source_gate`, folder `ros/source_gate`; **keep** the node names (`/system/sensor_mux/state`, `/safety/output_mux/*`, `clear gate` are the contract) | 5 launch includes, `bench_check.py`, `stack.launch.py` import, docs | no | maybe: lower value |
| `aav_spine` vs `aav_spine_bridge` | two packages, one node | no rename. Once the Python node goes, `aav_spine` = launch, config, `can_codec`, `arm_when_open`; say so in its README | — | no | — |
| `ti_ros2_driver`, `battery_monitor` | break `aav_<kind>_<maker>` | `aav_radar_ti`, `aav_battery_victron` (already in `conventions.md`, "proposed") | sensors.launch, PROVENANCE, interfaces.yaml | no | yes, cheap |
| `aav_bench` | it's only the mock sensors | `aav_bench_sensors` | bench.launch, sensors.launch | no | low |
| `aav_scripted`, `aav_ghost` | fine: the docs call them "scripted drive" and "ghost obstacles" everywhere | keep | — | — | no |
| `ros/ecolo_msgs/` | holds `aav_interfaces` | `ros/interfaces/` | folder only | no | cheap |
| `ros/can_bridge/` | also holds the UART bridge | `ros/car_bridge/` | folder, `ros/README.md` | no | cheap |
| `get_hud_map.sh`, `get-hud-map` | gets the place-search database for the **map window**, not the HUD | `get_places.sh`, `get places` | pixi, README, docs | no | yes |
| `car_listen.sh` | "listen" suggests it opens ports; it never does | `car_devices.sh` | car_install, car_kit, test.sh, docs | no | low |
| `demo_all.sh` | the task is `demo` | `demo.sh` | pixi, README | no | low |
| `carla-car` | every other profile task is its plain name | `carla` | pixi, docs | no | low |
| `bash tools/survey_car.sh` | the script is `survey_car.sh` | `survey-car` | pixi, README, docs | no | low |

Do the renames one per pull request, each updating `features.md` and `interfaces.yaml` first.

## 4. Organization

The `ros/<area>/` layout (Arsh, 25 Oct) is right: the areas follow the picture and the topic
areas. Four fixes:

| Fix | Why |
|---|---|
| Move lane keeping to `ros/drivers/` (`aav_lane_keeping`) | it publishes `/drivers/*/cmd_vel`. `ros/drivers/README.md` already has to explain why it lives in perception |
| `LaneGeometry.msg` into `aav_interfaces` | then a perception package doesn't depend on a driver package for a message |
| Folder names that match their packages: `ros/interfaces/`, `ros/car_bridge/` | `ecolo_msgs` and `can_bridge` mislead |
| Optional: `aav_ghost` to `ros/sim/` | it's a test aid. Its `/safety/...` names can stay |

Nothing to merge. `aav_detector` holds three nodes (detector, stop-sign watcher, camera
obstacles) that share their box maths: fine as one package.

## 5. Is Python right for each job?

### The evidence

| Source | What it says |
|---|---|
| TODO 11 and 32, `timing_test.py` (23 Oct, 10 min) | the stalls came from **the messaging, Fast DDS**. A bare 50 Hz Python publisher never slipped more than 0.03 s. With Cyclone: worst IMU gap 0.06-0.08 s, 0 stalls, 0 watchdog stops |
| `sources-and-targets.md` (23 Oct) | the output mux's extra Python hop didn't change the command gaps (median 19-21 ms) |
| `vehicle-model.md` | the lateral loop has 84 ms of delay today, and could take 0.55 s more at 1 m/s. A Python hop costs a few ms |
| `mac.md` | rclpy's `MultiThreadedExecutor` spins a whole core. `scripted_driver` and `goto_driver` moved off it. **`street_router` still uses it** |
| `status.md` | `lidar_scan` "under load can be late" (20 Oct, a busy Mac) |
| `camera-pipeline.md` | `camera_stream` + its decoder: 29-32% of a core per camera, on a Mac |

Already in Rust: the IMU driver, radar clustering, the CAN bridge (default), `spine_sim`,
the HUD and map, all the firmware.

### Node by node

| Node | Rate, work per tick | Control or safety path? | Deadline | Verdict |
|---|---|---|---|---|
| `ego` | 50 Hz, a few lines of maths | **yes** | 20 ms tick; stale-command stop | fine |
| `output_mux` | each ego command, a compare; 2 Hz check | **yes** | pass-through | fine (measured) |
| `sensor_mux` | a publisher check, low rate | no | — | fine |
| `legacy_uart_bridge` | 50 Hz frame out, 100 Hz serial read | **yes** | `cmd_timeout` 0.3 s | fine. See the box below |
| `sensor_watchdog` | 10 Hz checks; a callback per sensor message | safety lock (software) | 1 s fail-safe | watch its CPU on the Orin |
| `rc_driver` | 50 Hz | yes (RC via ROS) | 0.5 s mux timeout | fine |
| `goto_driver`, `scripted_driver` | 50 Hz, pure pursuit or a profile; one thread | yes, as drivers | 0.5 s mux timeout | fine |
| `actuator_test` | 50 Hz, logging | test only | — | fine |
| `stop_sign_watcher` | 10 Hz | a lock | — | fine |
| `gps_heading` | each GPS fix (5-10 Hz) | localization input | — | fine |
| `lidar_scan` | 10 Hz, ~29,000 points, numpy | **yes**: Nav2's stop zone | 100 ms | **watch**: it was late on a busy Mac. Measure on the Orin. If late, Rust (as `aav_radar_clusters`) |
| `camera_stream` (+ detector) | 10-15 fps, decode, undistort, inference | perception | ~100 ms | **watch**: the heavy work is in GStreamer, OpenCV and TensorRT, not Python. Measure 2 cameras on the Orin |
| `detector` | 10 fps, inference | perception | — | fine: PyTorch or TensorRT does the work |
| `camera_obstacles` | 4 ms a picture | Nav2 (off) | — | fine |
| `lane_finder` | 0.78 ms a picture (1.2 ms in the node) | off | — | fine |
| `control_interface` | 10 Hz | off | — | fine |
| `street_router` | events, HTTP | no | — | fine, but **fix its executor** (single-threaded, as `scripted_driver`): it subscribes to the GPS fix, so it may spin a core |
| `ghost_obstacle` | 10 Hz | test aid | — | fine |
| `spine_bridge` (Python) | 50 Hz | yes | — | replaced by Rust; remove later |
| `encoder_udp` | 50 Hz | — | — | dead |
| `pcl_pub` (TI radar) | ~10-30 Hz serial parsing | Nav2 (off) | — | **watch** when the radar goes on (C5) |
| `battery_monitor` | ~1 Hz serial | no | — | fine |
| `gazebo_*`, `carla_adapter`, `turtlesim_adapter`, bench mocks | 10-50 Hz | simulation only | — | fine. TODO 10's split already fixed the late Gazebo sensors |
| `drive_server`, `agent` | per request | no | — | fine |
| `arm_when_open` | one shot | no | — | fine |

> **The real gap is not Python.** Today the car runs the C++ ESP32 firmware with frame
> version 1, which has **no command timeout**: "if the bridge dies, the last throttle
> stays on" (`aav_legacy` README). A Rust bridge can die too. The fix is in the
> firmware: team-todo **B2** (flash `esp32-umod`, or add a timeout to the C++). Hard
> real-time safety must live there, as the spine's 100 ms watchdog does.

### Verdict

- **Python is right for these jobs**: glue at 50 Hz or less, maths in pure modules, heavy
  work handed to C and C++ libraries (numpy, OpenCV, TensorRT, GStreamer).
- **Move nothing to Rust now.** No node has a measured reason. `ego` in Rust stays a
  "later, once the car drives" idea (`languages.md`).
- **Measure on the Orin** before car day: `lidar_scan`, `camera_stream` with two
  cameras, `sensor_watchdog`. On the Motor Pi: `legacy_uart_bridge`'s frame jitter.
  `measure timing` runs on Gazebo, so on the car use `measure bandwidth` and a `top` snapshot per
  node. A node over half a core, or with gaps past its deadline, gets a Rust port, on the
  pattern of `aav_radar_clusters`.
