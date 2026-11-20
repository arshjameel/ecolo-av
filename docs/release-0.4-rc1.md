# 0.4-rc1 on the car: Wednesday 18 Nov

One page for the team meeting: what is in it, how to put it on the car, what to check
first, and what nobody has tried on the car yet.

!!! warning "Beta, and almost all of it untried on the car"
    The car drove on 13 Nov with **only the localization update** installed. Everything
    below "Untested on the car" has run in Ravensim, bench mode, QEMU or unit tests only.


**Flashing the ESP32 and upgrading the Motor Pi:** [the runbook](esp32-motor-pi-runbook.md) (Senura, 17 Nov), in order, with the gates and the way back.
Rehearsed the night before on a Mac, the ESP32 in QEMU: [the rehearsal](rehearsal-2026-11-18.md), with a morning checklist.

## What's in it

The whole stack on one page: [What we built](what-we-built.md). In short:

| Area | In 0.4-rc1 |
|---|---|
| Driving | our own stack (`stack:=ours`): costmap, planner, navigator, MPPI, smoother, stop zone. Nav2 deleted 17 Nov |
| Simulator | Ravensim, ours. Gazebo deleted 16 Nov |
| Languages | six nodes ran in Rust by default on 17 Nov; by 18 Nov 16 `*_impl:=` switches default to Rust ([Languages](languages.md)); their Python twins stay, one switch away |
| The league | no champion changed on 16 Nov ([its final configuration](league-results-2026-11-16.md#final-configuration)) |

The release in one page, one command per job: [0.4-rc1: start here](release-0.4.md).

## See it first, in Ravensim

A new laptop first: [Getting started](getting-started.md), steps 1-3 (`pixi install`,
then `pixi run build`). Then:

| To see | Run |
|---|---|
| Ravensim and the developer window (scene, console, lidar, map, keys) | `pixi run ravensim` (`world:=campus` for Carleton, after `pixi run campus` once). The first time, the window takes 2-5 min to build, with no output; the sim starts at once. Ctrl-C stops both |
| that it all works, headless | `pixi run sim-check` (about 2 min; every line PASS, then a table of ✅) |

## Install it on the car, step by step

Don't copy the steps from here: follow these pages in order.

| # | Step | Where |
|---|---|---|
| 1 | Make the kit on a Mac with internet: the models (`pixi run get models`), the Orin's ONNX, then `pixi run car-kit` (about 1 min, 2.3 GB with the models); copy `car-kit/` to a stick | [Car day](car-day.md#session-0-before-leaving-at-home), A0 steps 1-5; [Car install](car-install.md), section 1 |
| 2 | Back up every car computer first | [Car day](car-day.md), A2 |
| 3 | On the car: `car_install.sh --dry-run` (checks only) | [Car install](car-install.md), section 2 |
| 4 | Install: `car_install.sh`, about 5 min a computer | [Car install](car-install.md), section 3 |
| 5 | Look, read-only: `car_listen.sh` | [Car install](car-install.md), section 4 |
| 6 | Our stack on the Orin and the Motor Pi | [Car day](car-day.md), A5 (4a-4f) |

Undo: `rm -rf ~/aav-jazzy`. Nothing else changes, no sudo. (Installed with `--screens`? First
`bash ~/aav-jazzy/aav/tools/car_screens_install.sh uninstall`: [Car install](car-install.md#undo).)

## Check first on the car

| # | Check | You should see | If not |
|---|---|---|---|
| 1 | which language runs ([car day 4f](car-day.md#4f-the-rust-nodes-are-the-ones-running)) | `languages: ego_impl:=rust ... things_impl:=rust` in `car.launch.py`'s first lines | a Rust package didn't build: `~/aav-jazzy/build.log` |
| 2 | a Rust node misbehaves | - | e-stop, stop the launch, start again with that node's fall-back: `~/aav-jazzy/aav.sh car orin -- ego_impl:=python` (the `--` is needed) |
| 3 | the clocks | `chronyc tracking` on every computer | [Car day](car-day.md#the-clocks) |

The fall-backs, one per node (the others stay Rust):

| Node | Fall back to Python |
|---|---|
| ego + look_ahead | `ego_impl:=python` |
| behaviour | none: the Python removed 20 Nov (`save/python-behaviour`) |
| sensor watchdog | none: the Python removed 20 Nov (`save/python-safety-nodes`) |
| localization_health | none: the Python removed 20 Nov (`save/python-localization-health`) |
| lidar_scan | `lidar_impl:=python` |
| things | none: the Python removed 20 Nov (`save/python-things`) |
| sensor mux + output mux (the ESP32's heartbeat) | `io_mux_impl:=python` |
| legacy_uart_bridge (the ESP32's UART, the Motor Pi) | `legacy_impl:=python` |
| rc_driver (RC via ROS) | `rc_impl:=python` |
| lane_finder (lane keeping, the lane pose) | `lane_finder_impl:=python` |
| joy_alive (the joystick's check, between joy_node and teleop) | none: the Python removed 20 Nov (`save/python-safety-nodes`) |
| stop_sign_watcher (twist_mux's stop-sign lock) | `stop_sign_impl:=python` |
| point_driver (drive to a point; the Orin) and its client | `drive_to_point_impl:=python`; `pixi run drive-to-point --python ...` |
| arm_when_open (bench and Ravensim's spine only) | `arm_when_open_impl:=python` |
| camera_stream (each camera Pi stream; Rust only with `detector: none`: with RF-DETR the Python runs anyway) | `camera_stream_impl:=python` |

## Untested on the car (everything since 13 Nov)

| Item | Since | Tried in |
|---|---|---|
| Our own stack as the only stack (Nav2 gone) | 14-17 Nov | Ravensim, the league |
| The Rust defaults above | 16-17 Nov | Ravensim, unit tests |
| `path:=lane`: the plan keeps the right lane | 16 Nov | Ravensim (University Drive) |
| The behaviour planner and the stopping law | 14 Nov | Ravensim |
| Map chunks (RavenTiles) for NDT, the costmap, the things | 16 Nov | Ravensim (campus) |
| The watchdog's OVERSPEED latch | 16 Nov | Ravensim, unit tests |
| `sensor_mux`'s start-up grace (TODO 234) | 17 Nov | Ravensim, unit tests |
| The Rust ESP32 firmware | - | QEMU only; never flashed |
| The kit on the Orin and the Motor Pi | - | CI's ARM runner only |

## The seven hardware blockers

The hardware lead decides each; we help. Details: `docs/designs/2026-11-04_weak-links.md`,
section 1, and `TODO.md` 158.

| # | Blocker | What clears it |
|---|---|---|
| 1 | The no-timeout lock, while the car runs the 2025-26 C++ ESP32 | the Rust ESP32 flashed ([car day A8](car-day.md#session-6-the-esp32)) |
| 2 | COASTING until the stop is measured | test 4.3, then a decel in `vehicle.yaml` |
| 3 | The brake doesn't follow a wish | the `pi` brake loop, unless `open` stops within 1 m |
| 4 | Wheel speed not yet seen on the car | the Arduino's CAN frames seen (TODO 60) |
| 5 | Clocks across the computers | `chronyc tracking` on each; `pixi run record-drive --check` asks every car computer it can reach by ssh ([Recording](recording.md)) |
| 6 | Steering pulls right; its ends and centre unknown | measure them (TODO 67) |
| 7 | No way to score "1 m" | a tape mark at B (proposed) |

## Checked on 17 Nov (Ben's Mac)

Every `pixi run sim-check` mode in a row on Ravensim, main at `aaf3ab80`, each on its
own ROS domain (160-177, this computer only), the load under 8 at each start.

| Mode | Result | Note |
|---|---|---|
| the road world (no flag) | ✅ pass | |
| `--obstacle` | ✅ pass | |
| `--lidar-obstacle` | ✅ pass | |
| `--kerb` | ✅ pass | |
| `--movers` | ✅ pass | |
| `--faults` | ✅ pass | |
| `--actuator-faults` | ✅ pass | |
| `--three-point` | ✅ pass | 5 changes of direction, 0.44 m from a wall |
| `--behaviour` | ✅ pass, after a fix | failed once: the check now turns the things off (below) |
| `-e ml --perception` | ✅ pass | stood still 4.3 m before the stop sign |
| `tools/software_gl.sh --force ... --obstacle` (lockstep cameras) | ✅ pass | |

| Also | Result |
|---|---|
| CI for `aaf3ab80`: linux, arm, car-kit, car-kit-22.04 | ✅ all green |
| `pixi run car-kit` on the Mac | ✅ 68 s, 1.9 GB; 804 Rust crates, every `_rs` package resolves offline |
| the ESP32 firmware in QEMU | ✅ pass; now runs once more if only host-speed timings fail |

## Known issues

| Issue | What it means | Where |
|---|---|---|
| With the things on (`stack:=ours`), the behaviour slows only for things that **move** | a still wall the planner can't see is stopped by the stop zone alone (0.80 m short, from 1.1 m/s), with no slow-down first. The planner normally sees it and plans round | `aav_behaviour_rs` README; `sim-check --behaviour` now runs with `things:=false` |
| `aav_detector_rs` (ONNX in Rust) builds on macOS only | on Linux (the Orin) `-DDETECTOR_RS=ON` builds it; the Python detector is the default | [ONNX](onnx.md) |
| The kit needs the models | build the kit from a checkout with `models/` filled (`pixi run get models`), or the Orin has no detector | [Car install](car-install.md) |
| The QEMU check is host-speed sensitive | at load 11-32 its uplink rate and stop times slipped; it now retries once, only for timings | `tools/esp32_qemu.py` |
| Sway | in the sim, ours sways as Nav2 on the lot (0.021 against 0.018 m/s²) and less on the campus (0.099 against 0.131), 15 Nov afternoon; untested on the car (an earlier note here said 2 times: that was the morning's run) | [0.4-rc1](release-0.4.md) |
