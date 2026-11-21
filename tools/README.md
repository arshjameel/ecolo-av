# tools

The team's scripts: tests and checks, the docs, the car and car day, the simulator,
the network, downloads, firmware helpers, analysis, the MCP server and Ben's mailer.
Most are run by a `pixi run` task (`pixi.toml`); README.md's "Every command" lists them
all. Each script's own header or docstring says how to use it.

## Overview

| Kind | Language | ROS? |
|---|---|---|
| a checker or runner (`*_check.py`, `*.sh`) | Python 3 or bash 3.2 | through `tools/ros.sh` |
| the maths behind one (`topic_load.py`, `stalls.py`, `drive_metrics.py`) | Python 3, standard library | no: tested in `tools/tests/` |
| the MCP server's parts (`mcp_parts.py`) | Python 3 and PyYAML | no: tested in `tools/tests/` |

`tools/ros.sh <command>` runs a command with the workspace sourced, on Cyclone DDS
(`AAV_RMW=fastdds` for Fast DDS). Cyclone reads `tools/cyclonedds.xml`, then this
computer's own `~/.config/aav/cyclonedds.xml` if it has one (later settings win). `tools/ros.sh --local` keeps ROS discovery on this
computer.

The Flutter HUD's scripts (`flutter.sh`, `get_flutter.sh`, `hud_test.sh`,
`hud_fake_backend.py`) went to `archive/2026-27-flutter-hud/tools/` on 25 Oct.

Folders: [foxglove/](./foxglove/README.md) (the Foxglove/Lichtblick layouts),
[screens/](./screens/README.md) (the car screens' systemd units),
[systemd/](./systemd/README.md) (our stack at boot, per car computer),
[udev/](./udev/README.md) (stable device names), `tests/` (the tools' own tests).

## Interface

The `Task` column is `pixi run <task>`. "-" means no task: run the script itself, as
`pixi run python3 tools/<script>.py` or `pixi run bash tools/<script>.sh`. "- (ros.sh)" needs
ROS: `pixi run tools/ros.sh python3 tools/<script>.py`; "- (ros.sh --local)" starts Ravensim,
so add `--local`: `pixi run tools/ros.sh --local python3 tools/<script>.py`.

### Tests and checks

| Script | Does | Task |
|---|---|---|
| `test.sh` | every test that needs no ROS: Python units, Rust, interfaces.md, stand-ins stay local | inside `test` |
| `lint.sh` | ament_flake8 and ament_pep257 on our Python | `lint` |
| `release_charts/` | the 0.4 guide's charts (`charts.py <out dir>`) and its screenshot crops (`crop.py`); run by hand with matplotlib and Pillow | by hand |
| `check_graph.py` | the running graph against `docs/interfaces.yaml`; then `/localization/health`'s line (warn only) | `check <profile>` |
| `bench_check.py` | starts bench mode, tests the whole graph: contract, drive, e-stops | `bench-check` |
| `sim_check.py` | Ravensim's road world headless, the whole graph, a drive (`--voxels`: the costmap on the lidar's cubes) | `sim-check` |
| `drop_in_check.py`, `drop_in.py` | is what's running a true drop-in: each stream's rate and frame against the contract's `drop_in` rules (`drop_in.py`: the logic, no ROS) | `drop-in` |
| `layout_check.py` | is every topic in `foxglove/` served by a bridge? | - |
| `ci_annotate.py` | a failed test log into GitHub annotations | (CI) |

### Docs

| Script | Does | Task |
|---|---|---|
| `build_docs.py` | `docs/*.md` into HTML and the PDFs | `docs` |
| `gen_interfaces_md.py` | `docs/interfaces.yaml` into `docs/interfaces.md`; `--check` | - (test.sh checks) |
| `codemap.py` | `docs/codemap.md`, the code map: one line per package; `--check` | - (test.sh checks) |
| `graph_picture.py` | the running graph as `docs/img/graph.svg` and `.png` | - (ros.sh) |
| `control_figures.py` | the figures of `docs/control-explained.md` (feedforward, PI, LQR, the MPC's horizon and tube, MPPI, the Kalman filter) into `docs/img/control/`, on vehicle-model.md's guesses (matplotlib, scipy, no ROS) | - |
| `teach_figures.py` | the teaching guide's diagrams (AMCL, NDT, ICP, A*, MPPI...) into `docs/img/teach/` (matplotlib, no ROS) | - |

### The car and car day

| Script | Does | Task |
|---|---|---|
| `car_kit.sh` | makes `car-kit/`: our Jazzy for the ARM computers, offline, on a stick | `car-kit` |
| `car_install.sh` | installs from that kit on a car computer (`--dry-run`, `--screens`) | - (on the car) |
| `car_setup.sh` | one car computer, by role: backup, upgrade, install, services at boot, check. A dry run unless `--apply` and the role typed (`docs/car-setup.md`) | - (on the car) |
| `car_listen.sh` | read-only: which devices are here, could each driver open its own | - (on the car) |
| `car_screen.sh` | starts the HUD or the map full screen on its monitor | (screens/ units) |
| `car_screens_install.sh` | installs `screens/` into `~/.config` | - |
| `car_day_check.py` | is this laptop ready for car day? OK, MISSING, REMIND | `car-day-check` |
| `car_day_build.sh` | ESP32 firmware and QEMU check into `car-day/esp32/`; docs PDF into `docs/html/` | `car-day-build` |
| `machine_survey.sh` | read-only survey of one car computer, over ssh | - |
| `survey_car.sh` | runs that on every machine in a hosts file | - |
| `keychain_askpass.sh` | ssh's password helper: the login from the Mac's Keychain | (survey_car.sh) |
| `actuator_test.sh` | the actuator tests, stages 3-4 (`--sim`, `--stands`), and the preflight | `actuator-test` |
| `clear.sh` | a person clears the e-stop or a safety latch | `clear <estop, gate, geofence, watchdog>` |
| `drive_to_point.sh` | drive to one point and stop (`--sim`) | `drive-to-point` |
| `keyboard.sh` | teleop_twist_keyboard into `/drivers/keyboard/cmd_vel` (`--sim`, `--local`) | `keyboard` |
| `record.sh` | records a drive into `bags/` (MCAP): inputs, commands, and what the stack saw and decided (the front camera as JPEG, the locks, the road ahead); what the Motor Pi wrote for `link_check.py`. Run it through pixi (`pixi run tools/record.sh`): pixi sets Cyclone's participant limit | - |
| `record_drive.py` | a mapping drive (`record.sh --mapping`): every raw sensor at full rate, MCAP + zstd in split files; checks first (disk, rates, stamps, clocks, frames), a summary after; `--sim` in Ravensim, then its NDT map (`--sim-box X Y`: a parked car for that drive only) (`docs/recording.md`). The maths: `recording.py` (no ROS) | `record-drive` |
| `score_drive.py` | one scorecard for any recorded drive, sim or car: time, distance, top speed, time over the road ahead's limit, passes beyond its end, metres off the plan, holds, locks, a person driving (`--json`) | - (ros.sh) |
| `replay_commands.sh` | plays a bag's commands into `/drivers/replay/cmd_vel` (no flags, never local) | - |
| `build_engine.py` | the Orin's TensorRT model, on the Orin (JetPack's Python) | - |
| `calibrate_camera.py` | calibrate a streamed camera; print the board | `calibrate-camera` |
| `camera_calibration_cv5.py` | ROS's camera_calibration with OpenCV 5's corners | (calibrate_camera.py) |

### Sim and stand-ins

| Script | Does | Task |
|---|---|---|
| `ros.sh` | the workspace sourced; `--local` for stand-ins | (most tasks) |
| `dev_window.sh` | the developer window (Bevy): pretend data, `--live` beside a sim here, or `--watch` | `dev-window` |
| `sim_cameras.sh` | our simulator's cameras (Bevy, offscreen): the timing check, `--bench`, `--save`; `--ros` publishes the camera topics, here only | `sim-cameras` |
| `screen.sh` | one of the four raylib screens, live or with pretend data (`--demo`) | `hud`, `map`, `debug-hud`, `hw-panel` |
| `ravensim.sh` | Ravensim and the developer window live on it, one process group; sim switches pass through (`world:=campus` adds its RavenTiles and lanes) | `ravensim` |
| `demo_all.sh` | Ravensim, RViz, the HUD and the map window in one go (`--debug`: the debug HUD too) | `demo` |
| `pick.py` | a plain-words menu of the sim's switches (route, follower, where-am-I, speed loop): prints, saves and runs the command (`docs/switches.md`) | `pick` |
| `sim_fault.sh` | make a simulated sensor fail on purpose, or mend it | `sim-fault` |
| `software_gl.sh` | Mesa's software renderer with no GPU: Vulkan (lavapipe) for Ravensim's cameras, which then run in lockstep (it exports `AAV_SOFTWARE_GL=1`); `--force` even beside a GPU, a Mac too | - (put in front) |
| `fake_vlp16.py` | pretends to be the VLP-16 lidar: its UDP packets | - |
| `controller_compare.py` | the path follower (`mppi_lag`) against the trial trackers, planners, AMCL, NDT and KISS-ICP (`--trackers`, `--planners`, `--runs`, `--amcl`, `--ndt` (`--ndt-map`: another map), `--ndt-ekf`, `--ndt-leads`, `--gps-off`, `--gps-fault` (stop, jump, drift), `--voxels`, `--kiss-icp`), same routes | `measure controllers` |
| `diffusion.sh` | the diffusion planner trial's family: `data`, `train`, `score` (`models/diffusion/`) | `diffusion <data, train, score>` |
| `diffusion_data.py` | the teachers' paths (the lattice, Hybrid-A*) on random maps: our global costmap and planner, as the stack runs them, through `aav_planner`'s `teach` example (no ROS, no simulator; Nav2's servers until 17 Nov, TODO 242) | `diffusion data` |
| `lattice_primitives.py` | the State Lattice planner's primitives for our car, by Nav2's generator (`--check`: still right for `vehicle.yaml`?) | - (needs `pixi run get vendor`) |
| `lattice_smooth.py` | the lattice's smooth moves (`lattice_moves:=smooth`, `smooth_fast`): curvature no faster than the steering turns; our own generator, no ROS (`--check`) | - |
| `heading_check.py` | the global EKF's heading against Ravensim's truth | - (ros.sh --local) |
| `campus_world.py` | Carleton from OpenStreetMap: a Ravensim world (SDF), its RavenTiles, one map of the whole campus (Nav2's PGM format), and NDT's guessed map (tested without ROS) | `campus` |
| `ndt_map.py` | NDT's map of the test lot: drive the mapping route, or build from a bag, placed by a topic or a TUM file (`--poses-file`) (its maths without ROS) | `ndt-map` |
| `map_drive.py` | a map from a recorded drive, no ROS (the slam env): KISS-SLAM, the GPS fit, NDT's map, a 2D map, chunks, scores in Ravensim; `--merge` two drives or more, their 2D maps too (`docs/maps-from-drives.md`). The maths: `slam_map.py` | `map-drive` |
| `map_area.py` | one area's map from several drives, no ROS (the slam env): `map-drive` each, ICP onto the first, occupancy across passes (parked cars out, what one drive saw labelled `seen_in`), chunks, one report (TODO 259, 260). The maths: `passes.py` | `map-area` |
| `twin_world.py` | a Ravensim world from a labelled map (TODO 253): buildings, trees, poles, parked cars left out; `campus_world.py`'s meshes (its maths tested without ROS) | `twin-world` |
| `timing_test.py` | every topic's gaps and late messages, parked and driving | `measure timing` |
| `stream_check.py` | the camera stream end to end, without the car (`--impl rust\|python`: which node; CPU and memory of each program) | `measure stream` |
| `sim_frames.py` | camera pictures of the sim's stop sign, for the detectors | - |
| `make_stop_sign.py` | the road world's stop-sign mesh and picture | - |
| `make_map.py` | a map (PGM + YAML, Nav2's format) from a short spec | - |

### Network

| Script | Does | Task |
|---|---|---|
| `doctor.py` | can this computer join the car's ROS network? OK, WARN, FIX | `doctor` |
| `net_map.sh`, `net_map.py` | every machine on this network, and the ROS domains heard | - |
| `bandwidth.py` | every topic's Hz, size and MB/s, biggest first | `measure bandwidth` |
| `cyclonedds.xml` | Cyclone DDS settings (`CYCLONEDDS_URI` in pixi.toml) | (every ROS task) |

### Downloads

Each lands outside git, checksum pinned where the script says.

| Script | Gets | Task |
|---|---|---|
| `get.sh` | picks one of the downloads below, or `all` | `get <what>` |
| `vendor.py` | ROS 2 and ros2-rust source into `vendor/` | `get vendor` |
| `get_models.sh` | RF-DETR Nano and YOLO11n into `models/` | `get models` |
| `get_mediamtx.sh` | MediaMTX into `vendor/mediamtx` | `get mediamtx` |
| `get_places.sh` | `ottawa.db` (places) into `maps/places/`, for the map window's search | `get places` |
| `get_street_map.sh` | Ottawa's vector tiles into `maps/streets/` | `get street-map` |
| `get_carla_osm.sh` | Carleton's OpenStreetMap data into `maps/carla/` | `get carla-osm` |
| `get_routes.sh` | Ottawa's OSM and Valhalla's graph into `maps/routes/` | `get routes` |
| `router.sh` | starts Valhalla on 127.0.0.1:8002 | `-e routes router` |

### Firmware helpers

| Script | Does | Task |
|---|---|---|
| `pico.sh` | our Rust Pico 2 boards: install probe-rs, build the UF2, flash through a probe | `pico <spine, wheel, sentinel> <tools, build, flash>` |
| `elf2uf2.py` | a Pico 2 program into a UF2 file | (pico.sh build) |
| `esp32.sh` | the ESP32 board's Rust: tools, build, flash, check, QEMU, the first check, the guided walk | `esp <tools, build, flash, first-check, walk, qemu-tools, qemu-check>` |
| `esp32_qemu.py` | the ESP32 firmware in QEMU: heartbeat, uplink, RC modes; `--idle` boots it and only records its log | (`esp qemu-check`, `esp first-check --qemu`) |
| `esp32_first_check.py` | after the first flash: reads the ESP32's log on the USB cable for 30 s (or `--file`), text or JSON, and prints PASS/FAIL per check, then SAFE TO GO ON or STOP. Only reads | `esp first-check [port]` |
| `esp32_walk.py` | walk me through it: the whole first flash (`docs/esp32-bench.md`), one step at a time, bold and spoken (`say`; `--quiet`): the copy of the C++, the build, the flash once `flash` is typed, the first check row by row, the checks on stands, the roll back; a record in `bench-logs/`. `--dry-run` runs nothing | `esp walk [port]` |
| `esp32_walk_steps.py` | its steps, answers and spoken sentences (no hardware: tested in `tests/`) | - |
| `esp32_status_sim.py` | a pretend ESP32 with `wifi-status` on 127.0.0.1:8765: its page, `/status.json`, `/health`, `/config`, `/scan` and the WebSocket of status lines (`firmware/status`), moving; read only, standard library | - |
| `legacy_firmware.sh` | today's C++ ESP32 firmware, with PlatformIO | `-e firmware legacy-esp32` |

### Data and analysis

| Script | Does | Task |
|---|---|---|
| `topic_load.py` | the numbers behind `measure bandwidth` (no ROS) | - |
| `stalls.py` | the numbers behind `measure timing` (no ROS) | - |
| `drive_metrics.py` | the numbers behind `measure controllers` (no ROS) | - |
| `lane_score.py` | the numbers behind lane keeping (`measure lane` went with the lane followers, 19 Nov, TODO 264); run alone on `measure controllers`' `results.json`: did each run keep right, by the lane map (off centre, in the lane, over the left line and how often, past the kerb) | - |
| `scorecard.py` | the numbers behind `score_drive.py` (no ROS) | - |
| `slam_map.py` | the numbers behind `map-drive`: poses flattened, UTM and the GPS fit, the 2D map, map errors, merging (no ROS) | - |
| `passes.py` | the numbers behind `map-area`: occupancy across passes, what is dynamic, what only one drive saw, the merged 2D map (numpy, no ROS) | - |
| `recording.py` | the numbers behind `record-drive`: what it records, GB an hour, the checks, the summary and its verdicts (no ROS) | - |
| `measure.sh` | picks one of the measurements below and runs it (the simulator ones `--local`) | `measure <controllers, timing, stream, atob, lane, bandwidth, diffusion, onnx>` |
| `link_check.py` | did the Motor Pi's frames reach the ESP32? A bag (or the live topics) against the ESP32's status lines: PASS, FAIL, NOT TESTED (its maths without ROS) | `link-check` |
| `atob_score.py` | scores A-to-B runs (bag or CSV): speed up, hold, stop on B; EKF vs truth or tape | `measure atob` |
| `can_load.py` | the CAN bus load from `firmware/can/aav.dbc` | - |
| `compare_detectors.py` | YOLO11, YOLO26 and RF-DETR on the same pictures | - |
| `export_model.py` | a detector to Core ML (Mac) or ONNX (Orin, and the Rust detector) | - (`pixi run -e ml python3 tools/export_model.py --format coreml`) |
| `onnx_measure.py` | every ONNX model in `models/` (the detector, the diffusion planner, the learned lane finder; their FP16 twins): Core ML against the CPU and PyTorch, its share of the graph, what fell back, ms; PASS/WARN/FAIL, `--json` ([ONNX](../docs/onnx.md)) | `measure onnx` |
| `lane_data.py` | the learned lane finder's pictures (Ravensim, `sim_cameras --poses`) and labels from the lane map, into `models/lane-data/` (its maths without ROS, tested) | - (`pixi run -e ml python3 tools/lane_data.py`) |
| `lane_train.py` | trains the learned lane finder on those (PyTorch, `mps`), scores it on held-out pictures, exports `models/lane-ufld.onnx` and its FP16 twin | - (`pixi run -e ml python3 tools/lane_train.py`) |
| `label_drive.py` | labels after a drive (TODO 252): OWLv2's boxes onto the lidar points, into `labels.yaml`; `--sim-drive` makes a drive from Ravensim; scored against its truth. The maths: `label_core.py` (no ROS, tested) | `label-drive` (ml env) |
| `label_train.py` | our small point segmenter from the reviewed labels: mps, held-out scores, ONNX on the CPU and Core ML | `label-train` (ml env) |
| `onnx_check.py` | the ONNX detector's boxes against PyTorch's, and each runtime's ms a picture (`--time`); `--write-fixtures` for `detector-core`'s tests ([ONNX](../docs/onnx.md)) | - (`pixi run -e ml python3 tools/onnx_check.py`) |
| `step_plots.py` | the actuator tests' runs as graphs, one per test: asked, ego, the car (speed, steering, pedals), the IMU (smoothed), the distance; several sessions drawn over each other, the sim's and the car's (car day 21 Nov) | - |
| `sim_moves.py` | two baseline moves against a running sim, logged as the actuator tests log: `pedal` (the stick through ego's pedal, to reverse) and `three` (3 m and stop, the scripted drive) | - (ros.sh --local) |
| `vehicle_model.py` | the car's linear model: poles, steps, margins | - |
| `state_space_demo.py` | the state-space teaching example | - |
| `measure_map.py` | the measuring map: tape route, pictures, readings into `vehicle.yaml` (no ROS) | - |

### MCP server

| Script | Does | Task |
|---|---|---|
| `mcp_server.py` | the read-only MCP server: docs, contract, live graph (`docs/mcp.md`) | `-e mcp mcp` |
| `mcp_parts.py` | its parts that need neither ROS nor MCP | - |

### Mailer

| Script | Does | Task |
|---|---|---|
| `progress_mail.py` | emails Ben a progress page. **Ben's own sessions only** | - |

## Invariants

- **Stand-ins stay local.** turtle, sim, bench, view, demo, ravensim and their checks run through
  `tools/ros.sh --local`; so do `hw-panel`, `sim-cameras --ros` and the simulator measurements, inside `screen.sh`
  and `measure.sh`. `test.sh` checks the list; give any new stand-in task the same.
- **A family's script lists its choices** when run with no word or a wrong one, and
  exits 2 (`clear.sh`, `diffusion.sh`, `get.sh`, `measure.sh`, `pico.sh`, `screen.sh`, `esp32.sh`).
- `actuator_test.sh`, `drive_to_point.sh`: only `--sim` stays local. On the car they ask you to
  type a word first. `actuator_test` caps its own speed (1 m/s; 0.3 m/s on `--stands`);
  `drive_to_point.sh` has no cap of its own and relies on ego's `max_speed` (1 m/s by default).
- **Portable for Macs**: bash 3.2, no GNU-only flags, no Linux-only serial speeds.
- **Logic without ROS** (`topic_load.py`, `stalls.py`, `drive_metrics.py`, `scorecard.py`, `mcp_parts.py`)
  has its own tests in `tools/tests/`.
- **`mcp_server.py` only reads**: no call that writes to ROS, no file outside git
  (`tests/test_mcp_parts.py`).
- **`progress_mail.py` is Ben's**: only Ben sends email. Helping anyone else, don't
  send, set up or queue mail; write the update in the repo.
- **No secrets in git.** Car logins live in the Mac's Keychain (`keychain_askpass.sh`);
  mail settings in `~/.config/aav/mail.env`; the legacy firmware's WiFi in its untracked
  `secrets.h`.
- Surveys and checks change nothing: `car_listen.sh` never opens a port (that can reset
  an ESP32), `machine_survey.sh`, `net_map.py`, `doctor.py`, `car_day_check.py` only read.
- Each test run on a shared machine: its own `ROS_DOMAIN_ID` (at most 232), and stop it
  by process group, never `pkill` by node name.
- A new `pixi run` task gets a row in README.md's "Every command".

## Side Effects

| Scripts | Touch |
|---|---|
| checks, sim, stand-ins | start ROS and Ravensim processes on this computer |
| `actuator_test.sh`, `drive_to_point.sh` without `--sim`; `keyboard.sh` without `--sim` or `--local` | **move the real car** |
| `replay_commands.sh` (always: it has no `--sim`) | **moves whatever shares the ROS domain**: on the car's network, the car |
| `get.sh`, `get_*.sh`, `vendor.py`, `car_kit.sh` | download into `vendor/` (`car_kit.sh` caches in `vendor/car-kit-cache`), `models/`, `maps/`, `car-kit/` |
| `car_day_build.sh` | writes `car-day/esp32/` and `docs/html/` |
| `build_docs.py`, `graph_picture.py`, `teach_figures.py`, `control_figures.py`, `vehicle_model.py`, `state_space_demo.py`, `measure_map.py` | write `docs/html/`, `docs/img/` (`measure_map.py` also its table in `docs/measure-the-car.md`) |
| `make_stop_sign.py`, `make_map.py` | write the stop sign's files in `ros/sim/aav_sim_models/models/`; a map's PGM and YAML |
| `map_drive.py` | writes `<drive>/map/` (or `--out`); runs `cargo` for the chunks |
| `map_area.py` | writes `maps/<area>/` (or `--out`), each drive's map in its `drives/`; runs `cargo` for the chunks |
| `label_drive.py`, `label_train.py`, `twin_world.py` | write `<drive>/labels/` and the map folder's `labels.yaml` (`--sim-drive`: a whole drive folder, running `sim_cameras` and `cargo` for `scan_poses`); `models/label-seg.*`; `maps/twins/<name>/` |
| `record.sh`, `record_drive.py` | write `bags/`; `record_drive.py` also runs `chronyc` over ssh (keys only) and, with `--sim`, Ravensim on this computer |
| `esp32.sh flash`, `esp32.sh walk` (once `flash` is typed), `pico.sh <board> flash`, `legacy_firmware.sh -t upload` | write a board's flash (bench only) |
| `esp32_walk.py` | writes `bench-logs/`: the record, the first check's log, the copy of the C++ (a secret inside) |
| `legacy_firmware.sh` | with no `src/secrets.h`, copies `secrets.h.example` there (placeholders, untracked) |
| `car_install.sh`, `car_screens_install.sh` | write `~/aav-jazzy`, `~/.config` on a car computer |
| `car_setup.sh --apply` | sudo: apt upgrades, chrony, udev, netplan, linger; `~/aav-backup`, `~/.config` on a car computer |
| `net_map.py`, `doctor.py`, surveys | ping, ssh, listen on the network; read only |
| `progress_mail.py` | sends email over SMTP |

## Verification

```sh
python3 -m pytest -q -p no:cacheprovider tools/tests   # the tools' own tests
tools/test.sh                                          # those, and everything else without ROS
pixi run test                                          # all of it, colcon tests too
```
