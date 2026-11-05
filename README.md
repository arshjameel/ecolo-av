# ecolo-av: the AAV 2026–27 autonomy stack

Single code base for the Ecolo autonomous vehicle (SYSC 4907, Carleton University).
Goal for Gate 4 (26 Feb 2027): drive from GPS point A to B at 1 m/s and stop within 1 m.

**New here? Start with [docs/getting-started.md](docs/getting-started.md).**
**Making a change? Read [CONTRIBUTING.md](CONTRIBUTING.md): fork, branch, pull request, merge queue.**

The ROS 2 workspace and spine firmware for Carleton's autonomous vehicle capstone, built
by the 2026–27 team. Target: **ROS 2 Jazzy on Ubuntu 24.04** on the car's computers (the
Jetson AGX Orin and the Raspberry Pis); Pixi on Macs and other Linux machines.

**Sensors become telemetry, drivers become one command, and ego moves the car.**

Every page is in [`docs/`](docs/). The ones people ask for most:

| Page | What it's for |
|---|---|
| [The plan](docs/plan.md) | what's left to build, in order, by the gates |
| [Features](docs/features.md) | what the software does, why, and who built it |
| [Git](docs/git.md) | worktrees, conflicts during a rebase, handy commands; the flow is in [CONTRIBUTING.md](CONTRIBUTING.md) |
| [Car day](docs/car-day.md) | the runbook at the car: `pixi run car-day-build`, then `pixi run car-day-check` until it says 0 missing |
| [Every test](docs/tests.md) | every test, in order: simulator, bench, stands, lot, the whole drive |
| [Measure the car](docs/measure-the-car.md) | the sheet for whoever measures the car |
| [Bottom-up](docs/start-up.md) | each layer from the device up, start-up, and testing each layer |
| [Sim and the car](docs/sim-and-car.md), [Full simulation](docs/full-simulation.md) | what's simulated, what's real, and what's missing |
| [Maps and position](docs/maps-and-position.md), [A to B](docs/a-to-b.md) | the EKFs and OpenStreetMap; driving from A to B |
| [Perception options](docs/perception-and-hud-map.md) | which model for which job, the lidar, a map on the HUD |

## Read the docs

Open **`docs/html/index.html`** in a browser: short pages with pictures. There's also a
single PDF, built by `pixi run docs` into `docs/html/aav-docs.pdf`.

The pages are made from the markdown in `docs/`. Edit those, then run `pixi run docs`.

## Your first day

New to the team? Do these in order, on a Mac or on Linux.
[Your first day](docs/first-day.md) walks through each step, with what you should see.

| # | Do this | How long |
|---|---|---|
| 1 | Mac only: `xcode-select --install` | a few minutes, once |
| 2 | Install Pixi: the `curl` line below, then open a new terminal | under a minute, once |
| 3 | Get the code: fork `AAV-2025-2026/ecolo-av` on GitHub, then clone your fork ([CONTRIBUTING.md](CONTRIBUTING.md)) | a minute |
| 4 | `pixi install` | the first time, a few minutes: about 7 GB |
| 5 | `pixi run build`, then `pixi run test` | a few minutes the first time |
| 6 | `pixi run turtle`, or `pixi run sim` | ready |

The same steps, as commands:

```bash
xcode-select --install                        # 1. Mac only: Apple's git and compiler
curl -fsSL https://pixi.sh/install.sh | sh    # 2. Pixi; then open a new terminal
git clone git@github.com:<you>/ecolo-av.git ~/ecolo-av && cd ~/ecolo-av   # 3. your fork
git remote add upstream git@github.com:AAV-2025-2026/ecolo-av.git       #    the team's repo
pixi install                                  # 4. ROS 2 Jazzy and every dependency
pixi run build && pixi run test               # 5. our packages, then every test
pixi run turtle                               # 6. terminal 1: a turtle that steers like the car
pixi run keyboard --local                     #    terminal 2: hold i to drive, u or o to turn
```

When something fails: [docs/run.md](docs/run.md).

## Set things up (once)

After [Your first day](#your-first-day), add only the parts you need:

| To use | Run once | More |
|---|---|---|
| The object detector (RF-DETR, YOLO) | `pixi run get models` | [Perception](docs/perception.md) |
| The map window's street map and place search | `pixi run get street-map`, `pixi run get places` | [HUD](docs/hud.md) |
| Street routes (Valhalla) | `pixi run get routes` | [Street routes](docs/routes.md) |
| KISS-ICP lidar odometry (a trial, `kiss_icp:=true`) | `pixi install --locked -e kiss` (Linux, Apple-silicon Macs) | [Trials](docs/trials.md) |
| The camera stream check | `pixi run get mediamtx` | [Camera pipeline](docs/camera-pipeline.md) |
| CARLA on Carleton's streets | `pixi run get carla-osm` | [CARLA](docs/carla.md) |
| Flashing a Pico 2 with a debug probe | `pixi run pico spine tools` | [Rust firmware](docs/rust-firmware.md) |
| The Rust ESP32 firmware | `pixi run esp tools`, and `pixi run esp qemu-tools` for the QEMU check | [ESP32 first flash](docs/esp32-bench.md) |
| ROS 2 and rclrs source, to look things up | `pixi run get vendor` (560 MB) | `vendor/README.md` |
| A Mac on the car's network | `pixi run doctor`, then do what it says | [Your Mac](docs/mac.md) |
| The car's computers (offline, no sudo) | `pixi run car-kit`, copy `car-kit/` to a USB stick | [From a USB stick](docs/car-install.md) |
| Claude Code | start it in this folder: `CLAUDE.md` and the read-only MCP server (`.mcp.json`) load by themselves | [AI apps](docs/mcp.md) |
| Designs with the Elephant-Goldfish plugin | `claude plugin marketplace add google/elephant-goldfish-model && claude plugin install egm@egm` | `CLAUDE.md`, "Elephant-Goldfish" |

## Every command

Each one is `pixi run <command>`, from this folder. "This computer only" means it never
reaches a real car on the same network. A family is one command and a word after it
(`pixi run get models`): run it with no word and it lists its choices. The full list:
`pixi task list`.

### Every day

| Command | What it does |
|---|---|
| `build` | compiles our ROS packages |
| `test` | every test and the style check, on this computer only |
| `lint` | checks our Python against ROS's style rules |
| `docs` | rebuilds the web pages and the PDFs in `docs/html/` |

### Try it, without the car (this computer only)

| Command | What it does |
|---|---|
| `turtle` | turtlesim plays the car |
| `keyboard` | drive from the keyboard, in a second terminal: `keyboard --local` with `turtle` or `bench`, `keyboard --sim` with `sim`. With no flag it drives whatever shares the ROS domain: on the car's network, the car |
| `sim` | Gazebo plays the car (`world:=road`, `hud:=true`, `sim_speed:=3`...) |
| `demo` | the whole simulated car at once: Gazebo, RViz, the HUD and the map window. `demo --debug` adds the debug HUD (the lidar's cubes, NDT, where each localizer puts the car); `demo --fullscreen` |
| `pick` | a menu of the switches (who steers, where-am-I, speed loop), so you needn't remember them: prints the command, saves it, runs it; `pick last` runs it again ([Switches](docs/switches.md)) |
| `bench` | the real car's software, with stand-ins for the hardware |
| `view` | the car's model in RViz |
| `carla-car` | CARLA plays the car: `carla-car host:=<the CARLA computer>` |
| `drive-to-point` | drive to one point and stop smoothly: `drive-to-point --sim --ahead 20` |
| `ghost` | a pretend person in Nav2's view, to test stopping: `ghost ahead 10` |
| `sim-fault` | make a Gazebo sensor fail on purpose: `sim-fault gps stop`, then `sim-fault gps ok` |

### The screens

Add `--demo` to any of the four for pretend data: no ROS, no car, no build.

| Command | What it does |
|---|---|
| `hud` | the in-car screen (`fullscreen:=true`) |
| `map` | the big screen's map window |
| `debug-hud` | the debug screen: the lidar's cubes, the localizers, every sensor |
| `hw-panel` | the hardware panel: every sensor's number, and simple buttons (hold Space to drive) for Gazebo or the bench, beside `sim` or `demo --debug` (this computer only) |
| `rviz` | RViz with the driving layout: map, costmaps, path, car |
| `graph` | a live picture of the nodes and topics |

### Check that it works

| Command | What it does |
|---|---|
| `check` | the running graph against the contract: `check turtle` (or `sim`, `road`, `spine`, `car`, `replay`, `bench`) |
| `bench-check` | starts bench mode and tests the whole graph: contract, drive, e-stops (2 min) |
| `sim-check` | starts Gazebo's road world headless and tests the whole graph in it (2 min) |
| `measure` | `measure controllers`: Pure Pursuit against MPPI (and the trials: docs/trials.md) on the same routes; `measure timing`: are messages on time?; `measure stream`: the camera stream end to end; `measure atob <bag>`: scores an A-to-B run; `measure bandwidth`: every topic's MB/s |
| `diffusion` | the diffusion planner trial (`planner:=diffusion`): `diffusion data` (the lattice's and Hybrid-A*'s paths on random maps, no Gazebo), `diffusion train` (the model, scored by its guard, into `models/diffusion/planner.onnx`; `pixi install --locked -e learn` first), `diffusion score` |
| `ndt-map` | NDT's map of Gazebo's test lot: `drive` the mapping route headless, or `build` from a bag; into `maps/ndt/` |

### At the car

| Command | What it does |
|---|---|
| `car` | the real car: `car orin` (or `motor_pi`, `laptop`, `all`) |
| `estop` | block every driver (the car brakes) |
| `clear` | a person clears a latch: `clear estop`, `clear gate` (the output mux's STOP), `clear geofence` (car back inside), `clear watchdog` (the sensor watchdog's faults) |
| `actuator-test` | the actuator tests, on stands and in a lot: `actuator-test --list`. The preflight on stands, every sensor checked: `actuator-test preflight --stands` |
| `calibrate-camera` | calibrate a streamed camera: `calibrate-camera board` prints the checkerboard |
| `doctor` | can this computer join the car's ROS network? Says what to fix |

### Before car day

| Command | What it does |
|---|---|
| `car-day-check` | is this laptop ready? Run it until it says 0 missing |
| `car-day-build` | builds what the laptop takes to the car (the ESP32 firmware, the C++ way back...) |
| `car-kit` | makes `car-kit/`, to install our Jazzy on the car's computers from a USB stick |
| `get` | downloads what git doesn't keep: `get vendor`, `models`, `mediamtx`, `places`, `street-map`, `carla-osm`, `routes`, or `all` |

### Firmware

Flashing writes a board: on the bench only.

| Command | What it does |
|---|---|
| `pico` | our Rust Pico 2 boards: `pico <spine, wheel or sentinel> <tools, build or flash>`. `build` makes the UF2 file to drag on; `flash` goes through a debug probe (`tools` installs probe-rs once) |
| `esp` | the Rust firmware for today's ESP32 board: `esp tools` (once), `esp build`, `esp flash`, `esp qemu-tools` (once), `esp qemu-check` (does it brake within 0.2 s when the heartbeat stops?) |
| `-e firmware legacy-esp32` | builds today's C++ ESP32 firmware with PlatformIO (`-t upload` flashes it) |

### Perception, AI and maps

| Command | What it does |
|---|---|
| `detect` | the object detector on a picture or the cameras: `detect picture:=<file>` |
| `laptop-bench` | your laptop's own detector against bench mode |
| `talk` | tell the car what to do in plain English; a local model plans it (turtle, sim or bench only) |
| `-e mcp mcp`, `-e mcp drive-server` | the MCP servers for Claude: read-only, and the drive tools. Claude starts them |
| `-e routes router` | the street router on 127.0.0.1:8002 (needs `get routes`) |

### Without a task

Rare jobs and teaching tools are scripts you run yourself, such as
`pixi run python3 tools/vehicle_model.py`. [tools/README.md](tools/README.md) lists every
script and how to run it. Some you may want:

| Command | What it does |
|---|---|
| `pixi run python3 tools/vehicle_model.py` | the car as a linear model: poles, step responses, Bode margins |
| `pixi run python3 tools/state_space_demo.py` | the state-space primer's simulator |
| `pixi run python3 tools/ndt_demo.py` | NDT and ICP from above: the pictures in Learn: localization |
| `pixi run python3 tools/measure_map.py` | the measuring map: the tape route round the car, readings into `vehicle.yaml` |
| `pixi run python3 tools/lattice_primitives.py` | the State Lattice planner's moves for our car (`--check`: still right for `vehicle.yaml`?) |
| `pixi run tools/ros.sh --local python3 tools/heading_check.py` | the global EKF's heading against Gazebo's truth (this computer only) |
| `pixi run python3 tools/can_load.py` | how busy the CAN bus is, frame by frame |
| `pixi run tools/ros.sh tools/gz_window.sh` | Gazebo's window again, for a simulator already running |
| `pixi run bash tools/net_map.sh` | maps the network you're on: machines, ports, ROS domains. Read-only |
| `pixi run fastdds shm clean` | removes Fast DDS files left by ROS programs that died |

## What's where

| Folder | What's in it |
|---|---|
| [`ros/`](ros/README.md) | ROS 2 packages, one folder per area: sources (sensors), localization, perception, drivers, command_mux, io_mux, ego, can_bridge (the real car), sim, safety, hud; bringup has the launch profiles and the car description; ecolo_msgs has the team's own message and action types |
| [`firmware/`](firmware/README.md) | the CAN contract (`can/aav.dbc`), the wheel encoder's UDP packet vectors (`udp/`), the spine's Rust logic and device drivers (`spine/`), and the inherited C++ firmware (`legacy/`) |
| [`tools/`](tools/README.md) | tests, keyboard, graph check, the network doctor, recording, the docs builder, the progress mailer, the MCP server for AI apps |
| `docs/` | the docs (markdown sources, the HTML in `docs/html/`), the topic contract, and the history |
| [`models/`](models/README.md) | the detector models: RF-DETR Nano (the default) and YOLO11n, downloaded by `pixi run get models`, and their exports (`.onnx`, `.mlpackage`). Not in git |
| [`vendor/`](vendor/README.md) | the ROS 2 and ros2-rust source, for looking things up (`pixi run get vendor`); also the downloaded tools: MediaMTX, PlatformIO, QEMU, `cargo-tools`, `esp`, `car-kit-cache`. Not ours, and not in git |
| `archive/` | the 2025-26 and 2024-25 teams' repos, as they were (in git), and our port of their Flutter HUD (`2026-27-flutter-hud/`, put away 25 Oct). Reference only: never build or fix them (`docs/older-code.md`) |
| `.github/` | the CI workflow (`workflows/test.yml`) and the pull request template |

Each folder's README follows the same shape (Overview, Interface, Invariants, Side
Effects, Verification), and links down to the next level.

## Global invariants

The rules every part keeps. The full list is in [CLAUDE.md](CLAUDE.md).

| Rule | What it means |
|---|---|
| Contracts first | topics: `docs/interfaces.yaml`; CAN: `firmware/can/aav.dbc`; car measurements: `vehicle.yaml`. Change the contract, then the code |
| One command path | drivers → twist_mux → ego → output mux → **exactly one** backend |
| Units and frames | REP-103 and REP-105: metres, radians, seconds; x forward, y left; positive steering = left |
| Topic names | `/<area>/<device or node>/<thing>` |
| Maths out of ROS | logic modules have no ROS imports and their own tests |
| Safety | the red button always stops the car; stale commands stop it; nothing clears an e-stop on its own; a person outranks the computer |
| Firmware owns the hardware | pins, I2C addresses and calibration stay in firmware; ROS never sees a pin |
| No secrets in git | write down where a secret is and its type, never its value |

## Global side effects

- **On the car**: moves the actuators (steering, brake, throttle) through CAN or the
  ESP32's UART; opens the sensors' serial, USB, UDP and RTSP ports.
- **Stand-ins** (turtle, sim, bench): stay on this computer (`tools/ros.sh --local`).
- **Downloads** go into `vendor/`, `models/`, `maps/` and `car-kit/`, all outside git.

## System verification

- Run: `tools/test.sh` before every commit (no ROS needed).
- Run: `pixi run test` before every push (adds the build, `colcon test` and the style check).
- Run: `pixi run check <turtle|sim|road|spine|car|replay|bench>` against a running system.

## Status

Everything builds on Jazzy, on Linux and on Macs, and every test passes. The turtle,
Gazebo and bench mode (the whole car's software, with stand-ins for the hardware) all
run. Nothing has run on the car yet. What works, what's untested and what's missing:
[docs/status.md](docs/status.md).

Decisions and open questions (**Q1–Q34**) are in the team design doc, *AAV 2026–27
Autonomy Stack — Architecture & Plan* (a Claude Doc). Conventions are in
[CLAUDE.md](CLAUDE.md).

## Credits

| Who | What |
|---|---|
| The 2026-27 AAV capstone team | this design, the new code, and every decision: see [Decisions](docs/decisions.md) |
| The 2025-26 team (GitHub org `AAV-2025-2026`) | the ESP32 actuator firmware, the Pico 2 wheel encoder, the IMU driver, the radar driver and its clustering, lane keeping, the HUD app, and the steering geometry |
| The 2024-25 team (`Carleton-AAV-Capstone`) | the battery monitor, the camera calibrations, and the first YOLO stop-sign detector |
| Open-source projects | ROS 2 Jazzy, Nav2, robot_localization, twist_mux, imu_tools, Gazebo, RoboStack and Pixi, rclrs (ros2-rust), Roboflow's RF-DETR (the car's object detector), Ultralytics YOLO, NVIDIA TensorRT, Apple's Core ML Tools, and Rust's embedded-hal, Catppuccin's colour palettes (the screens' themes, MIT) |

## Where the code came from

New code was written for this design. Code reused from the 2025-26 team (GitHub org
`AAV-2025-2026`) and the 2024-25 team (`Carleton-AAV-Capstone`) carries a
`PROVENANCE.md`: the source repository, branch and commit, plus every change since.
Credentials committed to those public repositories were not copied. They still need to
be rotated (Q34).
