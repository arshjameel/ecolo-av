# Infrastructure: how the project is put together

For whoever puts the whole thing together. [How it fits](how-it-fits.md) explains the
car's software: sensors, drivers, the mux, ego, backends. This page explains everything
around it: the repo, the environment, the contracts, the profiles, the computers, the
tests, and how a change goes from an idea to the car.

## The pieces at a glance

| Piece | What it is | Where |
|---|---|---|
| The car's software | ROS 2 Jazzy packages, one folder per area | `ros/` |
| The car's firmware | the CAN contract, the Rust spine logic and its Pico 2 program, the 2025-26 boards' code | `firmware/` |
| The environment | ROS, Gazebo, compilers, Python and Rust, all from Pixi | `pixi.toml`, `pixi.lock` |
| The tools | tests, the graph check, the network doctor, recording, docs, the mailer, the MCP server | `tools/` |
| The docs | short markdown pages, built into web pages and one PDF | `docs/` |
| The rules | names, units, secrets, git, testing | `CLAUDE.md` |
| The decisions | Decided, Tentative, Proposed, and Q1–Q34 | the design doc: summary in [Decisions](decisions.md) |

## The repo, folder by folder

```text
ecolo-av/
  CLAUDE.md           the rules, for people and Claude sessions alike
  README.md, TODO.md  the front page; the current work list
  pixi.toml           the environment, and the one-word tasks (pixi run <task>)
  pixi.lock           the exact version of every package: committed, so every laptop matches
  ros/                ROS 2 packages, one folder per area (colcon builds these)
    bringup/            aav_bringup (the profiles: turtle, sim, bench, car, carla, replay)
                        and aav_description (the URDF and vehicle.yaml, the car's measurements)
    ecolo_msgs/         aav_interfaces: our own messages and actions
    sources/            drivers that turn devices into standard messages: IMU, GPS, lidar, cameras, radar
    localization/       the two EKFs and navsat_transform, and the GPS heading
    perception/         the detector (RF-DETR, YOLO), radar clustering, lanes, the lidar
    drivers/            everything that proposes a command: teleop, RC, actuator tests, scripted,
                        drive to point, plain English, Nav2, street routes
    command_mux/        twist_mux: picks one command
    io_mux/             the sensor mux and the output mux: real or simulated, never mixed
    ego/                the virtual car: one command into speed and steering
    can_bridge/         the real car: the spine bridge (CAN) and today's ESP32 link
    sim/                whatever plays the car instead: Gazebo, turtlesim, CARLA, bench mode
    safety/             the sensor watchdog and the ghost obstacles
    hud/                the in-car screens: the raylib HUD and the map window
  firmware/           microcontroller code (colcon ignores it)
    can/                aav.dbc, the CAN contract, and its test vectors
    spine/              Rust: spine-core (the spine's logic), aav-inputs, the device drivers,
                        and boards/pico2, the spine's board program
    legacy/             the 2025-26 ESP32 and Pico code that runs the car today
  models/             trained model files, not in git: pixi run get models
  tools/              scripts: see the table below
  docs/               these pages; docs/history/ keeps the Q&A, Ben's requests and the audits
  archive/            reference only, never built: the older teams' repos, the Flutter HUD
```

## The three contracts

Three files say how the parts must talk. Change the contract first, then the code, and
the checks tell you if the two disagree.

| Contract | Holds | Checked by | After you change it |
|---|---|---|---|
| `docs/interfaces.yaml` | every topic: its type, who publishes it, who reads it, and in which profiles it must be live | `pixi run check <profile>` on a running system; `pixi run test` checks that the Topics page matches | `pixi run python tools/gen_interfaces_md.py > docs/interfaces.md` |
| `firmware/can/aav.dbc` | every CAN frame between ROS and the spine | test vectors, shared by the Python codec and the Rust tests | update `can_codec.py` and `protocol.rs`, run `firmware/can/gen_test_vectors.py`, run both test suites |
| `ros/bringup/aav_description/config/vehicle.yaml` | every measurement of the car | its unit test; ego, the URDF, Nav2 and the adapters all read it | measure the car: [Car numbers](car_measurements.md) |

## The environment: Pixi

Pixi installs ROS 2 Jazzy and every tool into the repo's own `.pixi` folder, from
RoboStack and conda-forge. No admin rights, and the same versions on every computer,
because `pixi.lock` pins them all.

| Platform in `pixi.toml` | Who uses it |
|---|---|
| `linux-64` | Linux laptops and oirp-spss |
| `linux-aarch64` | the car's ARM computers (Orin, Pis), to run Jazzy before a reflash |
| `osx-arm64`, `osx-64` | Macs, Apple Silicon and Intel (macOS 14 or newer) |

| Command | Does |
|---|---|
| `pixi install` | downloads and installs everything in `pixi.lock` (about 7 GB the first time) |
| `pixi run <task>` | runs a task inside the environment: see the next table |
| `pixi run <any command>` | runs any command inside it, e.g. `pixi run tools/ros.sh ros2 topic list` |
| `pixi shell` | a terminal inside it; then `source install/setup.zsh` (Mac) or `setup.bash` (Linux) |
| `pixi run -e ml <task>` | the same, in the `ml` environment: PyTorch, RF-DETR and YOLO on top of everything else |

| Environment | Holds | Who uses it |
|---|---|---|
| `default` | ROS 2 Jazzy, Gazebo, Nav2, Rust, the compilers | everyone |
| `ml` | the default, plus PyTorch, RF-DETR, Ultralytics (YOLO) and, on Macs, Core ML tools | whoever runs the detector: `pixi run detect`, `pixi run -e ml bench perception:=true`. Linux PCs and Macs only |
| `docs` | Markdown and WeasyPrint | `pixi run docs` |
| `mcp` | the default, plus the MCP Python SDK | an AI app, through `.mcp.json`: [AI apps (MCP)](mcp.md) |
| `firmware` | Python and PlatformIO, no ROS | today's C++ boards: `pixi run -e firmware legacy-esp32` |
| `llm` | `mcp`, plus MLX (Macs with Apple silicon) | plain-English driving: `pixi run -e llm talk` ([Plain English](plain-english.md)) |
| `routes` | Valhalla, no ROS | the street router: `pixi run -e routes router` ([Routes](routes.md)) |
| `gst` | GStreamer, no ROS | the camera stream's fast decoder ([camera pipeline](camera-pipeline.md)) |
| `calib` | ROS's camera_calibration | `pixi run calibrate-camera` ([camera calibration](camera-calibration.md)) |

!!! note "Macs only: GNU coreutils"
  On a Mac, macOS's System Integrity Protection deletes `DYLD_LIBRARY_PATH` whenever
  `/bin/sh` or `/usr/bin/env` starts, and ROS needs that setting to load our own messages.
  So the Mac environment adds GNU `coreutils` for `colcon test`, and scripts that run ROS
  commands go through `tools/ros.sh`. Details: [Your Mac](mac.md).

## The one-word tasks

All defined in `pixi.toml`, and run from the repo's top folder: `pixi run <task>`.
Every one, with what it does: README's [Every command](../README.md#every-command).

## The scripts in tools/

| Script | Purpose | How to run it |
|---|---|---|
| `test.sh` | the tests that don't need ROS: Python logic, Rust spine, the Topics page | inside `pixi run test` |
| `lint.sh` | `ament_flake8` and `ament_pep257` on our Python | `pixi run lint` |
| `ros.sh` | runs a command with the workspace sourced | used by most tasks |
| `keyboard.sh` | teleop_twist_keyboard into `/drivers/keyboard/cmd_vel` | `pixi run keyboard` |
| `check_graph.py` | compares a running system with `docs/interfaces.yaml` | `pixi run check <profile>` |
| `gen_interfaces_md.py` | makes the Topics page from the contract | see the contracts table |
| `doctor.py` | network checks for joining the car | `pixi run doctor` |
| `bandwidth.py`, `topic_load.py` | measures every topic's rate and load; the numbers (no ROS) | `pixi run measure bandwidth` |
| `can_load.py` | the CAN bus load from the DBC's frames and rates (no ROS) | `pixi run python3 tools/can_load.py` |
| `record.sh` | records a drive into `bags/` | `pixi run tools/record.sh <name>` |
| `replay_commands.sh` | plays a recorded drive's commands back into the mux | `pixi run tools/ros.sh tools/replay_commands.sh <bag>` |
| `make_map.py` | draws a Nav2 map from a short spec | `pixi run python tools/make_map.py <spec>` |
| `build_docs.py` | builds the docs pages and the PDF | `pixi run docs` |
| `progress_mail.py` | emails a blue-and-white progress report: Ben only | see its docstring |
| `bench_check.py` | starts bench mode and tests the whole graph | `pixi run bench-check` |
| `graph_picture.py` | draws the running graph, grouped by layer | `pixi run tools/ros.sh python3 tools/graph_picture.py` |
| `mcp_server.py`, `mcp_parts.py` | the read-only MCP server for AI apps, and its parts | an AI app, through `.mcp.json` |
| `legacy_firmware.sh` | builds today's ESP32 firmware with PlatformIO | `pixi run -e firmware legacy-esp32` |
| `tests/` | the tools' own tests: the MCP server's parts and its smoke test, the UF2 converter | inside `pixi run test` |
| `get_models.sh` | downloads the stock RF-DETR Nano and YOLO models, checksums pinned | `pixi run get models` |
| `build_engine.py` | builds the Orin's TensorRT model, RF-DETR Nano by default, on the Orin itself (JetPack's Python) | `python3 tools/build_engine.py` on the Orin |
| `export_model.py` | exports a detector model to Core ML or ONNX (RF-DETR through rfdetr, YOLO through Ultralytics) | `pixi run -e ml python3 tools/export_model.py --format coreml` |
| `get_places.sh` | downloads the place database into `maps/places/`, for the map window's search, checksum pinned | `pixi run get places` |
| `get_street_map.sh` | cuts Ottawa out of Protomaps' world street map, checksums pinned | `pixi run get street-map` |
| `elf2uf2.py` | turns the Pico 2 program into a UF2 file | `pixi run pico spine build` |
| `vendor.py` | downloads the source in `vendor/*.repos` | `pixi run get vendor` |
| `udev/99-aav.rules` | stable device names on the car (`/dev/aav/imu`, ...) | install on the car's Linux computers |

## Launch profiles: the same nodes, a different car

Everything above the backend is the same in every profile. Only what plays the car
changes.

| Profile | Start it with | The car is | Runs |
|---|---|---|---|
| `turtle` | `pixi run turtle` | the turtlesim turtle | mux, ego, scripted driver, the car's model, `ekf_local` |
| `sim` | `pixi run sim` | Gazebo | the whole stack: both EKFs, `navsat_transform`, Nav2 |
| `car` | `pixi run car <machine>` | the real car | the parts its machine file switches on |
| `carla` | `pixi run carla-car host:=<the CARLA computer>` | a CARLA car (untested against a server) | the same as `sim` ([CARLA](carla.md)) |
| `replay` | `ros2 launch aav_bringup replay.launch.py`, with a bag playing | a recorded drive | localization (and Nav2) on real sensor data |
| `bench` | `pixi run bench` | a model car, driven by the spine's own firmware logic | the car's whole graph: the spine bridge, both EKFs with GPS, Nav2, and pretend sensors |

## The computers, and what runs where

The car runs one ROS graph across several computers. Each one starts `pixi run car
<machine>`, and `ros/bringup/aav_bringup/config/machines/<machine>.yaml` switches its parts on.

| Machine | Runs | Today |
|---|---|---|
| `orin` (Jetson AGX Orin) | the car's model, mux, ego, scripted and point drivers, IMU, GPS, the roof lidar, the two camera streams (RF-DETR inside), the stop-sign watcher, localization, Nav2, the CAN bus (wheels), Foxglove | Ubuntu 22.04 and Humble: it must move to Jazzy first (Q24) |
| `motor_pi` | the legacy bridge to today's ESP32 (UART); the 2025-26 UDP encoder is off since 23 Oct | Humble |
| `laptop` | the safety driver's joystick, and its own detector (`-e ml`) | any laptop with Pixi |
| `all` | everything on one computer: for a bench test | |
| the spine | a microcontroller on CAN: throttle, brake, steering, watchdog | a Pico 2 program builds, never flashed; the car's chip is Q10–Q17 |

The development computers:

| Computer | Use |
|---|---|
| oirp-spss | Ben's AlmaLinux workstation: builds and tests headless (no screen) |
| Ben's Mac | development, rqt_graph, Gazebo, RViz; one day, driving and watching the car |
| team laptops | the same, with Pixi: [Your first day](first-day.md) |

## How a change goes from an idea to the car

| # | Step | Where |
|---|---|---|
| 1 | Change the contract first, if the change touches one | the contracts table above |
| 2 | Write the logic in a plain module, with tests | `ros/<area>/<package>/` |
| 3 | Keep the node thin, with a docstring of what it reads and writes | the same package |
| 4 | Register it: a launch profile, the mux (drivers), `tools/test.sh` (tests) | [Build a driver](howto-driver.md) |
| 5 | `pixi run test`, then the turtle, then `pixi run check sim` in Gazebo | [Simulation](simulation.md) |
| 6 | Update the docs page, then `pixi run docs` | `docs/` |
| 7 | Commit small, each commit passing the tests; inherited code gets a `PROVENANCE.md` line | `git` |
| 8 | On the car: stands first, then the ground at walking pace, with a safety driver | the test ladder |

## Sharing the code

The repo is on GitHub: `git@github.com:AAV-2025-2026/ecolo-av.git`, public: anyone can read it.
Changes reach `main` only through a pull request from your own fork and the merge queue:
[Contributing](../CONTRIBUTING.md) has the steps.

| Task | Command |
|---|---|
| Get it, take newer work, share yours | fork, clone, branch, pull request: [Contributing](../CONTRIBUTING.md) |
| Hand it to a laptop with no GitHub access | `git bundle create ../aav.bundle HEAD main` (keep `HEAD`, or a clone checks out nothing), then `git clone aav.bundle aav` there |

CI (`.github/workflows/test.yml`) runs `pixi run test` on Linux for every push to `main`,
every pull request and every merge-queue entry, and on a Mac every night. The results are
on the repo's Actions tab.

**In your fork too.** Turn Actions on once in your fork (its Actions tab, then "I
understand my workflows, go ahead and enable them"). From then on every push to a branch
there runs `linux`, `arm` and `car-kit`, so you see green or red before you open the pull
request. The nightly `mac` and `sim` runs stay in the team's repo.

| Job | Computer | When | What |
|---|---|---|---|
| `linux` | Ubuntu 24.04, x86_64 | every push to `main`, every pull request, the merge queue, a push to a fork's branch | `pixi run test`, the MCP server |
| `arm` | Ubuntu 24.04, **64-bit ARM**, like the Orin and the Pis | the same | 12 min: `pixi run test`; the HUD and the map window built as ARM programs, and each drawn once on a virtual screen (picture: the artifact `arm-screens`) |
| `mac` | macOS 15, Apple Silicon | every night (team's repo) | `pixi run test`, the MCP server |
| `car-kit` | Ubuntu 24.04, 64-bit ARM | the same as `linux` | `pixi run car-kit`, then `car_install.sh --all` with **no network** (its own network namespace), as on the car ([car install](car-install.md)): 6 min |
| `car-kit-22.04` | Ubuntu **22.04**, 64-bit ARM: the car's own OS (JetPack 6) | the same | the same kit, installed offline on the car's Ubuntu. New 26 Oct: allowed to fail until it has passed a few times |
| `sim` | Ubuntu 24.04, x86_64 | every night (team's repo) | `sim-check --obstacle` in Gazebo, below |

Every job also runs from the Actions tab (Run workflow).

Every night it also runs `sim-check --obstacle` (Gazebo, headless) on Linux: the job
`sim`. New on 22 Oct and still on trial: GitHub's computers have no GPU, so the cameras
render in software (Mesa's llvmpipe, through `tools/software_gl.sh`), and a failure there
doesn't fail the night yet (`continue-on-error`).

### Reading a CI failure without signing in

A job's log needs a GitHub sign-in. Its **annotations** don't: when the test step fails,
`tools/ci_annotate.py` prints one annotation per failing test, with its error message.

| Step | Open this (`curl -s` works too) |
|---|---|
| 1. Find the job | `https://api.github.com/repos/AAV-2025-2026/ecolo-av/commits/<sha>/check-runs`: note the `linux` job's `id` |
| 2. Read its failures | `https://api.github.com/repos/AAV-2025-2026/ecolo-av/check-runs/<id>/annotations` |

Without a sign-in GitHub allows 60 of these calls an hour, so don't poll fast. A step keeps
at most 10 annotations: 9 failing tests, then the rest by name, or the log's last 40 lines.

The `sim` job's annotations (`ci_annotate.py --sim`): each `FAIL` line of sim-check, then
the launch log's last errors (Gazebo's crash, a node that died). Its whole logs (sim-check's
output, the launch log, every node's ROS log) are kept 14 days as the artifact `sim-logs`.
Anyone can list artifacts
(`https://api.github.com/repos/AAV-2025-2026/ecolo-av/actions/artifacts`), but downloading one
needs a sign-in: on the run's page, or `gh run download <run id> -n sim-logs`.

!!! note "PowerShell on the Linux runner"
  GitHub's Ubuntu runners set `PSModulePath`, which makes colcon set each package's
  environment through PowerShell; our own packages then go missing from later builds. The
  workflow empties `PSModulePath`, so colcon uses `sh`, as on a laptop.

!!! warning "What never goes in git"
  Build output (`build/`, `install/`, `log/`, `.pixi/`, `target/`), recordings (`bags/`),
  and secrets. `.gitignore` lists them. Downloaded third-party source, such as ROS's own
  code, lives in `vendor/`, which git doesn't track: `pixi run get vendor`.

## Where to find what happened

| Want | Look in |
|---|---|
| Why something is the way it is | `docs/history/conversation.md`: the Q&A record, by topic, with its open threads |
| Ben's requests, word for word | `docs/history/messages-*.md` |
| The audits of the inherited code, the firmware and the live system | `docs/history/audit-2026-10-10-*.md` |
| What's next | `TODO.md` and [Work](work.md) |
| What changed, and why | `git log` |
