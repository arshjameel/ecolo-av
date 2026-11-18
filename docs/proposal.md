# The proposal: what we plan to do this year, and what's already done

> **A snapshot from 15 October 2026.** What's true now: [Status](status.md) and [The plan](plan.md).
>
> **Nav2 and Gazebo, named below, are gone**: Gazebo on 16 Nov, Nav2 on 17 Nov. Today: Ravensim and our stack (`stack:=ours`). What each was: [Nav2, as it was](history/nav2-as-it-was.md), [Gazebo, as it was](history/gazebo-as-it-was.md).

Written 15 October 2026, for the team (Ben: "a file called proposal that lists
everything we have talked about that we think we can get working"). This is the **draft**
for the written proposal: every part of the car, what's built, what's left, and how sure
we are. Details live in the pages linked from each row.

> **The short version**
> - **The goal:** the 2025-26 car drives itself from one GPS point to another, safely, with a person watching on two screens.
> - **Already built and tested in simulation:** the whole software stack on ROS 2 Jazzy. Sensors, position (EKFs), Nav2, ego, the safety path, perception, two screens, plain-English driving. Linux and Mac.
> - **Not yet done:** almost everything **on the real car**. Measuring it, the sensors it's missing for closed-loop control, the Jetson upgrade, and real drives.
> - **Status words:** ✅ done and tested · 🧪 built, tested only in simulation or on the bench · 🔧 to build, confident · 🎯 stretch goal.

## 1. The system, in one picture

```text
 SENSORS             POSITION            DECIDE                 ACT                    CAR
 GPS, IMU,      →   EKF local    →   Nav2 (paths,     →   twist_mux → ego    →   spine (CAN) → throttle,
 wheel speed,       EKF global       obstacles)            (one command,          steering, brake
 cameras, radar,    navsat           drivers: keyboard,     Ackermann,            e-stop always wins
 lidar                               joystick, plain        speed limits)
                                     English, search box
        └──────────── everything also runs in Gazebo (the full car) and on a bench ────────────┘
 SCREENS: the instrument cluster (small) · the map with Ottawa streets and search (big) · Lichtblick
```

## 2. What's done, by part

| Part | What we have | Status | Read |
|---|---|---|---|
| **The graph** | one contract for every topic and node (`docs/interfaces.yaml`), checked against the running system; topic names by area (`/sensors/`, `/vehicle/`, `/safety/`...) | ✅ | [interfaces](interfaces.md), [naming](naming.md) |
| **Car model** | the car's shape, sensors and frames from one file (`vehicle.yaml`); zero point under the rear axle | ✅ the numbers are **guesses** until measured | [measure the car](measure-the-car.md) |
| **Ackermann** | the steering maths (speed + turn → wheel angle), with its own tests | ✅ | [conventions](conventions.md) |
| **ego** | turns every driver's command into one Ackermann command, with speed caps (1 m/s on the car; faster only in sim) and one reverse speed (0.5 m/s) | ✅ | |
| **twist_mux** | picks which driver is in charge (joystick > keyboard > Nav2...) and stops stale ones | ✅ | |
| **EKFs** | robot_localization: local (wheels + IMU) and global (+ GPS) | 🧪 sim: within 0.15 m of the truth; replay: 0.03 m | [maps and position](maps-and-position.md) |
| **Maps** | Nav2 maps for an empty world and a test lot; GPS datum ties them to the real world | 🧪 | |
| **Nav2** | paths, obstacles (radar, camera, lidar), collision monitor that stops the car; may reverse (on the car with `reversing:=true`, off by default until the wheels tell direction; only where the lidar sees behind it) | 🧪 sim: goals reached, stops 0.32-0.46 m before a box; with the lidar alone, 0.16-0.55 m before a post; backs up 5 m | |
| **Sensor watchdog** | stops Nav2 when the wheels, IMU, GPS, an EKF or the camera go quiet, NaN or frozen, the GPS jumps, or the wheels disagree with the IMU and GPS; fail-safe if the watchdog itself dies; says why on the HUD and the map window. Faults on purpose in Gazebo (`pixi run sim-fault`) | 🧪 sim: all 13 faults stop the car | [sensor failures](simulation.md#sensor-failures-on-purpose) |
| **Simulation** | Gazebo: the whole car, 3 cameras, radar, lidar, GPS, IMU, wheel speed, an actuator model, a SIMULATED banner; one command opens everything | 🧪 | [full simulation](full-simulation.md) |
| **Bench** | the real CAN messages, the spine's logic and both e-stops, without the car | ✅ `bench-check` | |
| **Perception** | RF-DETR Nano by default (YOLO11 with `detector:=yolo`; since 20 Oct), lane keeping, radar clustering (Rust) | 🧪 bench and sim | [perception](perception-and-hud-map.md) |
| **Core ML and TensorRT** | the same model (RF-DETR Nano since 24 Oct) exported for the Mac (Core ML) and the Jetson Orin (TensorRT, built on the Orin) | 🧪 Mac done; Orin after its upgrade | [perception](perception.md) |
| **Instrument cluster (small screen)** | raylib HUD in Rust: speed, mode, gear, e-stop button, camera, GPS, topic health; full screen on its own monitor at boot (ready to install) | 🧪 | [HUD](hud.md), [two screens](car-screens.md) |
| **Map (big screen)** | Nav2's map over offline Ottawa streets (OpenStreetMap), the car, the path, distance and time left; **search a place and send the car there**; full screen at boot, like the HUD | 🧪 | [HUD](hud.md), [two screens](car-screens.md) |
| **Plain-English driving** | a local language model (Qwen3, on the Mac with MLX) plans "drive 10 m forward"; you type yes | 🧪 sim: drove 10.02 m | [plain English](plain-english.md) |
| **Rust port** | firmware in Rust: the spine (on today's ESP32 control board too, over its UART or CAN), the wheel encoder, the status display, the CAN wheel nodes' logic; the IMU driver and radar clustering in Rust | 🧪 321 tests in `firmware/spine`; builds for the Pico 2 and the ESP32; never flashed | [Rust firmware](rust-firmware.md) |
| **Linux and Mac** | one `pixi.toml`: the same commands on Ubuntu, AlmaLinux and macOS, no admin rights; nightly Mac CI | ✅ | [Mac](mac.md) |
| **Watching** | Lichtblick / Foxglove (`sim ... foxglove:=true`), RViz, recording and replaying drives | ✅ | |
| **Testing** | 516 tests (Python, Rust, launch files), graph checks, `sim-check`, `bench-check`, the actuator tests; GitHub CI on every push to main | ✅ | [tests](tests.md) |
| **Simulation stays on the laptop** | `pixi run sim`, `turtle`, `bench` never reach a real car on the same network (fixed 15 Oct: they used to) | ✅ | [run](run.md) |
| **The car audit** | tools to map the car's network and survey every computer, read-only | ✅ ready for 16 Oct | [Friday](history/friday-todo-2026-10-16.md) |

## 3. What's left, and how sure we are

| # | Work | Why | Needs the car? | How sure |
|---|---|---|---|---|
| 1 | **The audit**: every computer, what runs, what's plugged in, what exists only there (the stop-sign model, `best.pt`) | we can't plan the car without it | yes | 🔧 16 Oct |
| 2 | **Measure the car**: sensor positions, wheelbase, steering angles, weight (scales) | the model, Nav2 and the sim all use these | yes | 🔧 |
| 3 | **Back up and upgrade the Jetson Orin** to a current JetPack, then ROS 2 Jazzy on every car computer | one ROS version everywhere; TensorRT for YOLO | yes | 🔧 |
| 4 | **The missing sensors for closed loop** (below) | today the car is driven **open loop**: we send commands but can't see what the actuators did | yes | 🔧 |
| 5 | **The safety timeout** in the actuator board: brake if commands stop | today a lost cable keeps the car going | yes | 🔧 must come first |
| 6 | **Actuator tests**: speed up, brake, steering sweep, circles; numbers into the sim. The script is built and proven in Gazebo (15 Oct: `pixi run actuator-test`) | the sim behaves like the real car; the car knows how hard it can brake | yes | 🔧 |
| 7 | **GPS-to-GPS movement**: from A to B by GPS in an empty lot, with a smooth stop at B. The driver is built and proven in Gazebo (15 Oct: `aav_drive_to_point`, `pixi run drive-to-point`, a-to-b.md) | the main goal | yes | 🔧 on the car after the brake test (4.3) |
| 8 | **The spine in the loop** in Gazebo ✅ 15 Oct: `pixi run sim spine:=true`; `sim-check --spine` passes (commands lost: full brake in 0.2 s, latched; both e-stops; disarmed: no motion). Sensor failures on purpose and the lidar into Nav2: done 15 Oct too | test the safety logic without the car | no | ✅ in Gazebo |
| 9 | **The real lidar driver** (likely a Velodyne) and the real cameras into perception | more of the world seen | yes | 🔧 |
| 10 | **Newer models**: YOLO26 or RF-DETR for perception; Qwen3.5, Gemma 4 or Granite for plain English, on the Orin too | better and freer models since 2024 | no | 🔧 test, then pick |
| 11 | **Touchscreen keyboard** for the map's search | type a destination in the car | no | 🔧 |
| 12 | **OSRM road routes**: follow real streets, not just a lot | driving beyond a parking lot | yes | 🎯 |
| 13 | **Flash the Rust firmware** on the real boards, replacing the 2025-26 C++ | one language, tested logic | yes | 🎯 board by board |
| 14 | **A Gazebo copy of the real test lot** | practise the exact drive at home | no | 🎯 |

### The missing sensors for closed loop

| Actuator | We command it | We need to read back | Plan |
|---|---|---|---|
| **Steering** | yes | the real wheel angle | the Jrk motor controller already reports its position (the 2024-25 firmware read it); wire it into `/vehicle/steering` |
| **Throttle** | yes | wheel speed: is it moving as asked? | the wheel encoder exists (one wheel); check it's reliable. Or a Hall sensor on each front wheel (the KEFC plan, **the hardware lead to decide**): CAN v0.2.0 and `spine_bridge` already read them into `/vehicle/wheels` and `/vehicle/odom` (bench, 15 Oct) |
| **Brake** | yes | brake position or pressure | a position sensor on the brake actuator: to choose after the audit |
| **E-stop** | the red button | its state | the spine reads it; never in its path |

The topics for all four already exist (`/vehicle/steering`, `/throttle`, `/brake`, `/wheels`); the simulator fills them today, and the bench fills `/wheels` from the CAN wheel frames.

## 4. The order

```text
 Oct: audit · measure · safety timeout     Nov: Jetson upgrade · Jazzy on the car · actuator tests
        │                                         │
        ▼                                         ▼
 Dec: closed-loop sensors · real numbers into the sim · drives on stands
        │
        ▼
 Winter: empty-lot drives · GPS A to B · smooth stop      Stretch: OSRM streets · Rust firmware on the car
```

| Phase | Done when |
|---|---|
| 1. Know the car | the audit is written up; every measurement is in `vehicle.yaml` |
| 2. Safe to move | the timeout works (test 3.9); both e-stops stop the car |
| 3. Closed loop | steering, speed and brake read back, and match the commands |
| 4. Drives itself | A to B by GPS in an empty lot, stopping within 1 m of B, with a safety driver |

The tests for each step, in order: [Every test](tests.md).

## 4b. The build plan (15 Oct)

The same plan as [The plan](plan.md), agreed with Ben on 15 Oct: what's left, split into software we
can build now (S) and work that needs the car or the hardware lead (C). Hardware choices are
the hardware lead's.

### The path to the goal

```text
 now (software)          Friday           hardware lead        on the car
 S1 A→B in Gazebo ──┐    C1 audit    ──►  C3 timeout     ──►  C8 stands → lot ──► C9 A→B 🎯
 S2 spine in loop ──┼──► C2 measure  ──►  C4 sensors
 S3 steering fix  ──┘                     C5 Jetson, Jazzy, C6 drivers, C7 install
```

### Software we can build now (no car)

| # | Build | Why | Size | Status |
|---|---|---|---|---|
| S1 | **GPS A → B with a smooth stop**: speed by the distance left to B, slowing at half the measured braking g | the main goal (tests.md 5.2), proven in Gazebo first | 1 day | ✅ in Gazebo 15 Oct: `aav_drive_to_point`, `pixi run drive-to-point`; every stop within 0.4 m of B, at 1 and 3 m/s (`sim-check --drive-to-point`). On the car: C9 |
| S2 | **The spine in the loop** in Gazebo: ego → CAN → the spine's safety logic → the actuator model → Gazebo | timeouts and e-stops tested the way the car runs them | 1-2 days | ✅ 15 Oct: `pixi run sim spine:=true`, `pixi run sim-check --spine` passes |
| S3 | **Gazebo steering fix**: full lock turns 0.43 rad, not 0.5 | the sim should steer like the car | small | ✅ done 15 Oct: 0.484 rad at full lock (Gazebo's `steering_limit` from `vehicle.py`; the last 3% is tyre slip) |
| S4 | **Ghost obstacles**: a pretend person in Nav2's map while the real car drives | test stopping on the car with nothing real to hit | 1 day | ✅ in Gazebo 16 Oct: `aav_ghost`, `pixi run ghost ahead 10`, `sim:=... ghost:=true`; `sim-check --ghost` stopped 0.21 m before it ([docs](ghost-obstacles.md)). On the car: untested |
| S5 | **Touchscreen keyboard** for the map's search | type a destination in the car | small | ✅ 16 Oct: tap the search box, a 12 x 5 keyboard with é è à ç, keys 60 px or more ([docs](hud.md)). On the car's touchscreen: untested |
| S6 | **Newer models**, on the same pictures and tasks: YOLO26 / RF-DETR vs YOLO11; Qwen3.5 / Gemma 4 / Granite vs Qwen3 | better or freer models | medium | ✅ compared on the Mac 16 Oct ([docs](model-comparison.md)): RF-DETR Nano (Apache-2.0) saw the stop sign from 20 m, YOLO11n 10 m; YOLO26 = YOLO11. Gemma 4 E4B 23/23 vs Qwen3-4B 21/23. Defaults unchanged: the team decides after an Orin run |
| S7 | **Python → Rust**: ego and the spine bridge (team-todo.md D7) | needs Ben's go-ahead | large | |
| S8 | **OSRM street routes** | only for driving on real streets | medium | ✅ in Gazebo 16 Oct: Valhalla (MIT) offline, `aav_routes` street_router; a street route drove to 0.5 m of its end through Nav2 ([docs](routes.md)). For real streets later, the lot first |
| S9 | **CARLA (the Unreal Engine 5 version)** next to Gazebo: photo-real cameras, traffic, towns | Ben asked 15 Oct (reopens Q31). No Mac version: needs Linux or Windows with an NVIDIA RTX card (16 GB of video memory or more). [CARLA](carla.md): Town10 on 0.10 (UE5); Carleton's streets from OpenStreetMap on 0.9.16 (UE5 can't import maps yet) | medium | **research done, adapter built and tested without a server (15 Oct); never run: needs the NVIDIA machine** |

### Needs the car, or the hardware lead

| # | Work | Who decides | Blocks |
|---|---|---|---|
| C1 | **The audit** (16 Oct): every computer, `best.pt`, camera code, the screens ([Friday 16 Oct: the to-do list](history/friday-todo-2026-10-16.md)) | team | everything below |
| C2 | **Measure and weigh** the car → `vehicle.yaml` ([Measure the car](measure-the-car.md)) | team | an accurate sim |
| C3 | ⚠️ **The safety timeout**: flash the Rust ESP32 or patch today's board (team-todo.md B2) | **hardware lead** | any drive on the ground |
| C4 | **Closed-loop sensors**: steering angle from the Jrk, wheel speed (CAN boards or not), brake position | **hardware lead** | closed-loop control |
| C5 | **Back up and upgrade the Jetson**, then Jazzy on every car computer | team | our software on the car |
| C6 | **Drivers for the real parts**: the lidar (probably a Velodyne), the cameras from the Pis, the GPS | after C1 | full perception |
| C7 | **Install**: the car kit, the two screens at boot ([docs/car-screens.md](car-screens.md)) | after C5 | |
| C8 | **Tests on stands, then in the lot**: `pixi run actuator-test` ([docs/actuator-tests.md](actuator-tests.md)) | after C3 | real numbers |
| C9 | **GPS A → B on the car** | after C3-C8 | 🎯 the goal |
| C10 | A Gazebo copy of the real test lot | the lot chosen | |

## 5. Risks

| Risk | What we do |
|---|---|
| The car's hardware isn't what the old repos say | the audit first; nothing changes until it's written up |
| No timeout today: a lost cable keeps the car going | fixed before any drive (team-todo.md B2) |
| Upgrading the Jetson loses the only copy of something | back up the whole disk first |
| Leaked passwords and keys in the old repos | changed, never copied (Q34) |
| Our GitHub repo is **public**, on purpose (Ben, 15 Oct) | no secrets ever go in it: only where the old ones are (Q34) |
| GPS isn't accurate enough near buildings | an RTK fix, and the EKF with the wheels and IMU |
| The sim lies, because its numbers are guesses | measure, then test on stands before the ground |

## 6. Ideas we've talked about (not yet in the plan)

| Idea | Note |
|---|---|
| Ghost obstacles: a simulated person in Nav2 while the real car drives | safe way to test stopping |
| The car's pictures labelled for our own stop-sign model | only if the stock one fails |
| CARLA instead of Gazebo | no: next to it, for the cameras (S9; Q31 reopened 15 Oct). Adapter ready, needs the NVIDIA machine |
| An AI assistant that reads the live graph (MCP, read-only) | built: `docs/mcp.md` |

**Where decisions live:** the design doc (Q1-Q34), `docs/decisions.md`, and `docs/history/`.
