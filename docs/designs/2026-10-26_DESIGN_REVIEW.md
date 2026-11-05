# Design review: the design doc against the repo (26 Oct 2026)

The design doc, *AAV 2026–27 Autonomy Stack — Architecture & Plan* (Claude Doc, revision
61), was written on 9 October. This page checks it against the repo as of 26 October.

> **The short version**
> - The doc's **architecture still holds**: drivers → twist_mux → ego → one backend.
> - Almost every **detail has moved on**: folders, topic names, the path follower, the
>   detector, the DDS, the actuator path, the tests, the git workflow.
> - **The big gap is the brake.** On today's car, a wanted slow-down never reaches the
>   brake. Neither the doc nor the CAN contract has a way to send one.
> - **Nothing has run on the car yet.** Gate 1 (30 Oct) needs measurements; none exist.
> - Of the 34 open questions: 3 answered, 4 built in code but not agreed, 8 partly
>   answered, 19 still open.

## How this was checked

| What | How |
|---|---|
| The design doc | exported whole as Markdown (493 lines, all text; its 4 drawings not readable) |
| The repo's own story | read `docs/status.md`, `plan.md`, `decisions.md`, `features.md`, `audit-questions.md`, `car-day.md`, `CONTRIBUTING.md`, `TODO.md` (open items), `designs/concepts_brief.md`, the 25 Oct drift report |
| The code | grep and read: `aav.dbc`, `car.launch.py`, `bridges.py`, `vehicle.py`, `twist_mux.yaml`, `ego.yaml`, `ackermann.py`, `legacy_protocol.py`, `navsat.yaml`, `ublox_gps.yaml`, `spine.rs`, the BT XMLs, `test.yml`, `pixi.toml` |
| GitHub | the public API: repo settings, issues and pull requests |
| Git | authors since 9 Oct; branches not merged into `main` |
| Not done | no tests run; nothing launched. Claims below that say "works" quote `status.md` |

---

## 1. What changed since 9 Oct

### Summary and target architecture

| The doc says | True now | Shown by |
|---|---|---|
| ego → one backend directly | ego → **output mux** → one backend; a **sensor mux** checks every source is real | `ros/io_mux/`, `interfaces.yaml` `/control/car/actuator_cmd` |
| "Nothing runs end to end" | turtle, Gazebo and bench run end to end; **the car: still nothing** | `status.md` |
| We write ego, the spine bridge, scripted, adapters, aav_legacy | also: `aav_goto`, `aav_routes`, `aav_language`, `aav_actuator_tests`, `aav_rc`, `aav_gps_heading`, `aav_safety`, `aav_ghost`, `aav_hud`, perception, 37 packages | `ros/*/`, `docs/packages.md` |
| First milestone: GPS A→B with Nav2 | A→B is **`aav_goto`** (own pure pursuit + a braking curve); Nav2 follows street routes and lot goals | `ros/drivers/aav_goto/`, `docs/a-to-b.md` |

### Decisions

| The doc says | True now | Shown by |
|---|---|---|
| Fast DDS on every computer | **Cyclone DDS** (23 Oct): Fast DDS delivered up to 0.48 s late | `pixi.toml` `RMW_IMPLEMENTATION`, `decisions.md` |
| Python for every node; C++ only in the IMU driver | Rust for firmware, drivers and ports; **no C++ of our own**. Rust nodes: IMU, radar clusters, spine bridge (default), HUD | `decisions.md` #8, `bridges.py` `DEFAULT = 'rust'` |
| Car-safe Nav2: RPP first, MPPI later | **MPPI is the default** (22 Oct); RPP is `controller:=rpp`; Spin removed | `vehicle.py` `DEFAULT_CONTROLLER = 'mppi'`, `behavior_trees/*_car.xml` |
| Forward only (`DUBIN`, no BackUp) | **Nav2 may reverse** at 0.5 m/s, on by default on the car | `car.launch.py` `reversing` default `true` |
| Folders by diagram layer | **by area**: `ros/<area>/` (Arsh, 25 Oct) | `ros/README.md` |
| A CAN spine replaces the serial link | **Tentative:** today's car drives over the Motor Pi's **UART** (`actuators:=uart`, default); the Orin's CAN bus is **sensors only** | `bridges.py` `DEFAULT_ACTUATORS = 'uart'` |
| HUD: Flutter, reuse later | **raylib HUD** in Rust (`aav_hud`); Flutter archived 25 Oct | `ros/hud/aav_hud/Cargo.toml`, `archive/2026-27-flutter-hud/` |
| (missing) | Detector **RF-DETR Nano** (TensorRT on the Orin), YOLO as an option | `detector.launch.py` default `rfdetr` |
| (missing) | Cameras: **MediaMTX** H.264 on the Pi → `aav_camera_stream` on the Orin | `decisions.md` 23 Oct, `ros/sources/aav_camera_stream/` |
| (missing) | Goals count within 1.0 m and 0.75 rad; ego jerk limit 2.0 m/s³ | `decisions.md` 22 Oct, `ego.yaml` |
| (missing) | HUD heartbeat, latched on dropout; GO/STOP panel; arming waits for OPEN | `decisions.md` 24 Oct |

### Code organisation

| The doc says | True now | Shown by |
|---|---|---|
| `aav/src/<layer>/` tree | `ros/{sources, localization, perception, drivers, command_mux, ego, io_mux, can_bridge, sim, safety, hud, bringup, ecolo_msgs}` | `ls ros` |
| `aav_interfaces` at top level | `ros/ecolo_msgs/aav_interfaces/` | `ls ros/ecolo_msgs` |
| `yahboom_imu_cpp` | `aav_imu_yahboom` (Rust) | `ros/sources/` |
| `hud/maincar`, `hud/flutter_pi` | archived; `ros/hud/aav_hud` | `archive/` |
| spine-core, 47 tests, "board code to come" | 13 crates, 4 boards (`esp32-umod`, `pico2`, `wheel-encoder-pico2`, `sentinel-pico2`), **none flashed** | `firmware/spine/`, `status.md` |
| `rosdep install` | Pixi + RoboStack; `pixi run car-kit` for the car offline | `pixi.toml` |
| CI waits for the org (Q25) | CI on x86_64, **ARM** and an offline car-kit job, every push; Mac and Gazebo nightly | `.github/workflows/test.yml` |
| `/home/ben/aav/aav` | `~/Code/ecolo-av`, repo `AAV-2025-2026/ecolo-av` | `CLAUDE.md` |

### Interface contract

Every topic now starts with its area (Ben, 15 Oct; `docs/naming.md`).

| The doc says | Now |
|---|---|
| `/joy` | `/drivers/joy/joy` |
| `/imu/data_raw`, `/imu/mag`, `/imu/data` | `/sensors/imu/...` |
| `/gps/fix` | `/sensors/gps/fix` (+ `fix_velocity`, `vel`) |
| `/odometry/local`, `/odometry/global` | `/localization/odom/local`, `/localization/odom/global` |
| `/odometry/gps`, `/gps/filtered` | `/localization/gps/odom`, `/localization/gps/fix` |
| `/cmd_vel/joy`, `/cmd_vel/remote`... | `/drivers/<driver>/cmd_vel`: joy, rc, keyboard, tests, scripted, replay, goto, nav, ai |
| `/estop` | `/safety/estop` |
| `/cmd_vel` | `/control/cmd_vel` |
| `/ego/actuator_cmd` → backends | → output mux → `/control/<target>/actuator_cmd` + `/control/<target>/heartbeat` |
| `/from_can_bus`, `/to_can_bus` | `/can/rx`, `/can/tx` |
| `/vehicle/odom`, `/vehicle/status` | same, plus `/vehicle/steering`, `throttle`, `brake`, `rc`, `wheels`, `battery` |
| (missing) | lidar, cameras, perception, `/safety/*` locks, `/system/source`, `/hud/heartbeat` |
| Actions: `follow_gps_waypoints` for A→B | `/goto` (`aav_interfaces/GoTo`) for A→B; `follow_gps_waypoints` for street routes and `aav_language` |
| Frames: `imu_link`, `gps_link` | plus `radar_link`, three camera links, `lidar_link`, `chassis` |

### What ego does

| The doc says | Now | Shown by |
|---|---|---|
| Stale after ~0.3 s; 50 Hz | same: `cmd_timeout: 0.3`, `rate_hz: 50` | `ego.yaml` |
| Clamp speed and steering | also: jerk 2.0 m/s³, steering rate 0.6 rad/s, lateral accel 2.0 m/s², payload, a body model | `ego.yaml`, `dynamics.py` |
| Steering at standstill is Q27 | ego **holds** the steering below 0.05 m/s | `ackermann.py:75` |

### The ROS graph

The doc says we write three nodes. We now write about 25 nodes (see `docs/packages.md`).
Its graph drawings miss the muxes, the safety locks, perception and the HUD.

### How ROS talks to firmware (CAN)

| The doc says | Now | Shown by |
|---|---|---|
| Draft map: 0x010, 0x100, 0x180, 0x181 | **v0.2.0**: those four, plus **WHEEL_SPEED_L/R 0x200, 0x201** (100 Hz); CRC-8/SAE-J1850 and a 4-bit counter on every frame | `firmware/can/aav.dbc` |
| Watchdog ~200 ms | **100 ms**, then the next 10 ms tick: 111-120 ms in QEMU | `plan.md` Gate 2 |
| Wheel encoder wired to the spine | the **hardware lead's Arduino** sends wheel speed on CAN (Arduino C++) | `decisions.md` 23 Oct |
| Today: Pi serial at 115200 → ESP32 | still true, and it is **the default** path; a v2 serial frame (17 bytes, CRC, Orin heartbeat) waits for the Rust firmware | `legacy.yaml` `frame_version: 1` |
| Option A recommended | built; micro-ROS Tentative "no for now" | `decisions.md` |
| Remove the open update page | the Rust firmware has none; the C++ on the car still does (S4) | `audit-questions.md` |

### Testing and simulation

| The doc says | Now | Shown by |
|---|---|---|
| Profiles: turtle, sim, car, replay | plus `bench`, `carla`, `stack.launch.py` (`target:=`, `source:=`), `foxglove` | `ros/bringup/aav_bringup/launch/` |
| twist_mux: remote 90, scripted 50, replay 40, nav 20, ai 10 | plus **rc 95**, **tests 60**, **goto 25**; locks: estop 255, **stop_sign 30**, **sensors 30** | `twist_mux.yaml` |
| 43 + 47 + 77 tests | **1110 tests**, 37 packages (25 Oct) | `status.md` |
| (missing) | Gazebo road world, spine in the loop, sensor faults on purpose, ghost obstacles, Foxglove layouts, car-day kit | `status.md` |

### Roadmap

The doc's drawing has 5 phases and 4 gates. `plan.md` now uses the proposal's 5 dated
gates: Know the car (30 Oct), Safe to move (27 Nov), Closed loop (18 Dec), Drives itself
(26 Feb), Handover (2 Apr), plus tasks T1.1-T9.8 and the MPPI track M1-M9.

### Localization and navigation

| The doc says | Now | Shown by |
|---|---|---|
| Heading from the magnetometer, declination ~12° | **no magnetometer**: `start_yaw` by hand, then the **GPS velocity heading** (`aav_gps_heading`) | `ros/localization/README.md` |
| Set declination and `yaw_offset` | kept at 0 (datum mode) | `navsat.yaml` |
| Datum at a surveyed point | a **placeholder near Carleton** | `navsat.yaml` "TODO: the test site" |
| RPP with `allow_reversing: false` | MPPI default; reversing allowed | above |
| collision_monitor waits for a lidar | the **VLP-16** feeds it (stock `velodyne`) | `status.md` |
| Rolling global costmap only | plus radar, lidar and camera layers; `plan_around` | `status.md` |
| OSRM gives waypoints | **Valhalla** (offline), `aav_routes` | `aav_routes/package.xml` |

### What we inherit

All previous repos are in `archive/` (read only), surveyed in `docs/older-code.md`. The
four 2024-25 repos the doc listed "to read next" have been surveyed there. The Flutter
HUD moved from "reuse later" to archived. The stop-sign detector was replaced (RF-DETR).

### Risks

New risks the doc lacks: the default actuator path has no timeout; Orin load; one main
contributor; reversing without direction sensing. Details in section 2.

### Next steps

| Doc step | Now |
|---|---|
| 1 Secrets (Q34) | no record that anything was rotated |
| 2 Image the Orin and Pis | irreplaceable files copied off; **full image not done** (T1.3 10%) |
| 3 Inventory the car | two audits (16 and 21 Oct); **nothing measured** |
| 4 Team meeting on Proposed items | not recorded; every Proposed item is still Proposed |
| 5-8 Repo, Jazzy, ego + turtle, scripted | done |
| 9 CAN map | done (v0.2.0); bench test of real firmware not done |
| 10 URDF | built from guesses |
| 11-12 On the car | not started |

---

## 2. What's missing

Things the goal ("controlled speed-up, drive consistently, stop on a point", GPS A to B)
needs that neither the doc nor the repo covers, or covers only as a question.

### Software

| # | Missing | Why it matters | Evidence | Owner |
|---|---|---|---|---|
| W1 | **A way for a wanted slow-down to reach the brake** | Today the brake comes on only below `stop_speed`, at a fixed level; ego's acceleration is dropped. The car coasts to B. DRIVE_CMD has only `accel_limit`, which limits speeding up | `legacy_protocol.py:85-95`; `aav.dbc` DRIVE_CMD; `concepts_brief.md` §1 | software + hardware lead (contract change) |
| W2 | **GPS at 1 Hz** | At 1 m/s a fix every metre; the stop needs better than 1 m. `config_on_startup: false`, so the rate is set by hand in u-center | `ublox_gps.yaml:24` | hardware lead (receiver), software (config) |
| W3 | **A real datum and a start-pose routine** | `navsat.yaml` has a placeholder datum; the start heading is typed in (`start_yaw`). A→B on the car needs a surveyed origin and a marked start | `navsat.yaml`; `a-to-b.md` §start | team (site), software |
| W4 | **An A-to-B scorer** | `measure controllers` only runs Nav2 (`mppi`, `rpp`), never `aav_goto`. No metric for stop error, overshoot, hold error | `concepts_brief.md` §5; `tools/controller_compare.py` | software |
| W5 | **Time sync on the car** | No chrony setup anywhere; the EKFs and TF reject stamps from a wrong clock | `plan.md` T6.2 (10%); `audit-questions.md` W6 | software |
| W6 | **Start-up and restart of the stack on the car** | Only the screens have a systemd unit. The ROS stack starts by hand; who restarts a dead driver is open (N3) | `tools/screens/aav-map.service`; `decisions.md` "Respawning device nodes" | software |
| W7 | **An Orin load budget** | MPPI, two RF-DETR streams, lidar and EKFs on one Orin. The lidar stop was 0.13 m late under load on a Mac | `status.md` lidar row; `plan.md` M6, T1.7 | software |
| W8 | **Running on the car's OS as it is** | The car is Ubuntu 22.04 / JetPack 6; CI tests only 24.04. The TensorRT engine is unbuilt | `status.md` ARM row; `audit-questions.md` W7 | software |
| W9 | **A team `ROS_DOMAIN_ID`** | The kit uses 42; the 2025-26 software uses 0. Never decided | `decisions.md` "Also waiting" | team |

### Safety

| # | Missing | Why it matters | Evidence | Owner |
|---|---|---|---|---|
| F1 | **An interlock that keeps programs off the no-timeout path** | The default backend is the C++ ESP32: no command timeout, no e-stop input. `legacy_uart_bridge` only logs a warning on `frame_version 1`. Nothing stops Nav2, `goto` or `aav_language` driving through it | `legacy_uart_bridge.py:100`; `legacy.yaml`; `car-day.md` session 9 allows "the C++" | software (build), team (rule) |
| F2 | **Reversing on by default, without direction sensing** | Wheel speed's sign comes from the last command (H5). Is reverse safe while rolling (Q4)? The roof lidar can't see low objects close behind | `car.launch.py` `reversing` default `true`; `audit-questions.md` H5, D8 | team decision |
| F3 | **What holds the car once it stops at B** | `IDLE_BRAKE = 0.0` releases the brake after a reset; brake on power loss is unknown (S2) | `spine.rs:65`; drift report #2 | hardware lead + team |
| F4 | **A rule for AI drivers on the real car** | `aav_language` checks only the e-stop lock, not `stop_sign` or `sensors` | drift report #3, #4 | team |
| F5 | **A written session card, lot permission, a named safety driver** | S-1, S-2, Q32 all ⬜. Gate 2 needs them | `plan.md` Safety | team |
| F6 | **A geofence that doesn't depend on Nav2** | `goto` drives on GPS with no map. Only its distance and time limits stop a runaway route | `plan.md` S-3; drift report (`goto` limits) | software |
| F7 | **A firmware speed cap on today's path** | ego caps 1 m/s; the C++ ESP32 has none; spine-core's `direct` mode has no reverse cap | `plan.md` S-5; drift report #1 | software |

### Hardware integration: questions for the hardware lead

These are **questions**, not decisions. Most are already in `docs/audit-questions.md`;
the last three are new.

| # | Question | Unblocks |
|---|---|---|
| H1, H9 | Is the Motor Pi–ESP32 link UART, which port? Is ESP32 TX1 wired to the Pi's RX? | `legacy_uart_bridge`, the uplink (steering and brake read back) |
| H2 | Does the ESP32 read steering angle and brake position? | Gate 3, steering fits |
| H4, H5 | Which Arduino and CAN chip; does it send 0x200/0x201 as in `aav.dbc`; can it tell direction? | wheel speed, reversing |
| C1 | Is U7 fitted and wired? SparkFun's schematic shows the CAN pads not connected | whether the CAN spine plan works on this board |
| C2 | How many holes in the brake rotors? | the slowest speed we can read (8 holes ≈ 0.24 m/s) |
| S1, S2 | What does each e-stop cut? What does the brake do without power? | the safe state |
| D6 | The Jrk G2s' settings, including their own timeout | a second stop layer |
| P1, P2 | 5 V converter ratings; what powers the Orin | brown-outs that look like software crashes |
| new | Can the brake be commanded proportionally (travel vs force), and how fast? | W1 |
| new | Can the receiver run at 5 Hz, and is RTK planned this year? | W2, the stop error |
| new | When an Arduino wheel sensor fails, what does it send: nothing, zeros, a flag? | the frozen-wheel check |

### Testing and evidence

| # | Missing | Why it matters | Evidence | Owner |
|---|---|---|---|---|
| E1 | **Car measurements before Gate 1 (30 Oct)** | Every number in `vehicle.yaml` is a guess; every Gazebo result checks the stack against those guesses | `plan.md` Gate 1; `audit-questions.md` §6 | team |
| E2 | **Gate 2's number vs the firmware** | The gate says "brakes within 100 ms"; QEMU measures 111-120 ms. Fix the wording or the firmware | `plan.md` Gate 2 | team |
| E3 | **Ground truth for stops** | GPS can't judge its own stop. Tape from a chalk mark at B, 5 runs | `concepts_brief.md` §4 | team |
| E4 | **Fits that name their signals** | Brake and throttle commands log as NaN, so `fit_lag` returns nothing; no dead time | `concepts_brief.md` §3 | software |
| E5 | **A sim-vs-real report** | T2.7 at 0%: nothing shows Gazebo predicts the car | `plan.md` T2.7 | software |
| E6 | **A place for car data** | Where bags, run sheets and the firmware tag (`meta.yaml`) live, and for how long | `concepts_brief.md` decision 1; `tools/record.sh` | team |
| E7 | **The joystick, ever** | Never tested, even in the turtle; it is the safety driver's tool | `status.md` Joystick; `plan.md` T5.6 | software |

### Hand-over to the 2027-28 team

| # | Missing | Why it matters | Evidence | Owner |
|---|---|---|---|---|
| O1 | **More than one builder** | Since 9 Oct: 703 of 747 commits by Ben, 44 by Arsh; 82 feature rows say "Ben" | `git log --since=2026-10-09`; `features.md` | team |
| O2 | **Car logins anyone can reach** | They are in Ben's Mac's Keychain; usernames in a file on his Mac | `car-day.md:13, 43` | team (a shared vault) |
| O3 | **One source of truth for decisions** | `decisions.md` says "the design doc wins", but the doc is 16 days stale and lives outside git | `decisions.md:105-108` | team |
| O4 | **A GitHub org the next team controls** | The repo is in `AAV-2025-2026`; who are its admins next year? | `CLAUDE.md` Git | team |
| O5 | **The Gate 5 scorecard** | doesn't exist | `plan.md` Gate 5 | team |
| O6 | **Unmerged work** | `wt/goals-steering` still holds a WIP commit (goal circling, TODO 17) | `git log main..wt/goals-steering` | software |

### Team process

| # | Missing | Why it matters | Evidence | Owner |
|---|---|---|---|---|
| P2 | **The Proposed items ratified** | TwistStamped, car-safe Nav2, RC on top, latched spine boot, plus D1-D6: all still Proposed | `decisions.md` Proposed; `plan.md` D1-D6 | team |
| P3 | **Owners (Q10, Q33)** | No package or task has a named owner | `decisions.md` | team |
| P4 | **An issue tracker in use** | `plan.md` says the T numbers "work in the issue tracker"; GitHub has 0 issues | GitHub API: 4 PRs, 0 issues | team |
| P5 | **The hardware lead's answers, first hand** | Hardware answers arrive "passed on by Ben"; the H list has no answer column | `decisions.md` 23 Oct; `audit-questions.md` | hardware lead |

---

## 3. Open questions Q1–Q34

| Q | Topic | Doc (9 Oct) | Now | Blocking |
|---|---|---|---|---|
| Q1 | Dimensions | open | **open**; sheet ready (`measure-the-car.md`) | a day at the car; Gate 1 |
| Q2 | Steering feedback | open | **open** (H2, N9) | hardware lead |
| Q3 | Brake | open | **open** (S2) | hardware lead, on stands |
| Q4 | Throttle, reverse | open | **open** (D8) | hardware lead |
| Q5 | Wheel encoder | open | **partly**: Arduino, CAN 0x200/0x201 (23 Oct); holes, direction open (C2, H4, H5) | hardware lead |
| Q6 | E-stop | open | **partly**: it exists, we don't rewire it (10 Oct); what it cuts open (S1) | hardware lead |
| Q7 | RC override | open | **partly**: 3-position switch Proposed, RC via ROS at 95 (24 Oct) | H10, team |
| Q8 | Power | open | **open** (P1, P2; T4.x 4%) | hardware lead |
| Q9 | Which computers | open | **open**; today's Motor Pi stays in the path | hardware lead, team |
| Q10 | Spine owner | open | **open** | team |
| Q11 | Starting firmware | open | **partly**: Rust `esp32-umod` built, C++ on the car; flashed commit unknown (D1) | car day session 6 |
| Q12 | What the spine accepts | open, suggest targets | **built as targets** (DRIVE_CMD speed + angle); not ratified; no brake field (W1) | team + hardware lead |
| Q13 | Board | open | **open**; C1 may rule out the current board's CAN | hardware lead |
| Q14 | Orin CAN side | open | **open** (H7) | hardware lead |
| Q15 | Bus | open | **partly**: 500 kbit/s in `aav.dbc`; wiring open (H3, H7) | hardware lead |
| Q16 | Watchdog | open | **built**: 100 ms, brake, steering held (`spine.rs`); not ratified, not flashed | team; Gate 2 |
| Q17 | Wireless updates | open | **partly**: Rust firmware has none; C++ still open (S4) | flashing the Rust |
| Q18 | GPS model | open | **open**; `ublox_gps` default (Tentative); ZED-F9R suspected (N1) | hardware lead |
| Q19 | Corrections | open | **open** (N2) | hardware lead |
| Q20 | IMU | open | **partly**: Yahboom kept (Rust driver); no magnetometer used; mount open (N3) | car day |
| Q21 | Test site | open | **open**; datum is a placeholder | team, permission (S-2) |
| Q22 | Lidar, cameras | open | **partly**: VLP-16 on the roof; Pi cameras on MediaMTX; camera models open | car day |
| Q23 | QNX | open | **open** (Tentative: no QNX) | supervisor |
| Q24 | Jazzy on the Orin | open | **open**: still JetPack 6 / 22.04 (W7); T1.3 10% | full disk image first |
| Q25 | Repo | open | **answered**: `AAV-2025-2026/ecolo-av`, public (25 Oct) | — |
| Q26 | Languages | open | **answered**: Decided #8 (10 Oct) | — |
| Q27 | Steering at standstill | open | **built**: ego holds below 0.05 m/s; not ratified | team |
| Q28 | Joystick handover | open | **built** as the doc's default (back after 0.5 s); not ratified | team |
| Q29 | HUD | open | **answered**: raylib HUD; Flutter archived; which computer drives the screens open (W10) | car day |
| Q30 | Remote driving | open | **open**; Foxglove is watch-only on the car | team |
| Q31 | CARLA | open | **reopened**: adapter ready, never run | the NVIDIA machine |
| Q32 | Test rules | open | **open** (S-1, S5) | team |
| Q33 | Owners | open | **open** | team |
| Q34 | Secrets | open | **open**: no record of rotation | account owners |

Count: answered 3 (Q25, Q26, Q29); built but not ratified 4 (Q12, Q16, Q27, Q28);
partly answered 8; still open 19.

---

## 4. Edits for the design doc

**[T]** = needs a team decision first. Unmarked = a fact fix.

### Summary
1. Replace "Where we are: nothing runs end to end" with "Runs end to end in turtlesim, Gazebo and bench mode; nothing has run on the car yet."
2. In "What we write", add `aav_goto`, the muxes, `aav_safety`, `aav_hud`, perception, the Rust ports.
3. Replace "GPS A→B with Nav2" with "GPS A→B with `aav_goto`; Nav2 for street routes and lot goals."

### Target architecture
4. Redraw: add the output mux and sensor mux after ego, the safety locks on twist_mux, perception, the HUD.

### Decisions
5. "Both use Fast DDS" → "All use Cyclone DDS (23 Oct)."
6. Languages row → Decided #8 text: Rust for firmware, drivers and ports; no C++ of our own.
7. "One repo... folders by diagram layer" → "by area, `ros/<area>/` (Arsh, 25 Oct)."
8. Nav2 row → "MPPI default (22 Oct), RPP as `controller:=rpp`; Spin removed; reversing allowed." **[T]** ratify.
9. CAN spine row → add "Tentative: today the car drives over UART (`actuators:=uart`); CAN reads sensors only."
10. Add rows: RF-DETR Nano + TensorRT; MediaMTX camera stream; wheel speed via the hardware lead's Arduino; goal tolerance 1.0 m; raylib HUD; GPS antenna over the rear axle.
11. **[T]** Move TwistStamped, RC top override and the latched spine boot from Proposed to Decided, or say why not.

### Code organisation
12. Replace the whole tree with `ros/README.md`'s table and `ls firmware/spine`.
13. "spine-core, 47 tests; board code to come" → "13 crates, 4 boards, none flashed."
14. "`rosdep install`" → "Pixi; `pixi run car-kit` for the car offline."
15. "A CI job waits for the org" → "CI on x86_64, ARM and car-kit every push; Mac and Gazebo nightly."

### Interface contract
16. Replace the topics table with the area names (section 1 above), or point at `docs/interfaces.yaml` and keep only the 10 main rows.
17. Actions: add `/goto` (`aav_interfaces/GoTo`); say `follow_gps_waypoints` serves routes and `aav_language`.
18. "What ego does": add jerk, steering-rate and lateral limits; "below 0.05 m/s the steering holds".
19. Frames: add `lidar_link`, camera links, `radar_link`.

### The ROS graph
20. "We write three of the nodes" → "We write about 25; see `docs/packages.md`." Redraw both graphs.

### How ROS talks to firmware
21. Draft CAN map → `aav.dbc` v0.2.0: add 0x200/0x201 and the CRC-8 + counter.
22. Watchdog "about 200 ms" → "100 ms, then the next 10 ms tick (111-120 ms in QEMU)."
23. "Wheel encoder wired to the spine" → "the hardware lead's Arduino sends WHEEL_SPEED on CAN."
24. Add "Today's wiring (23 Oct)": UART actuators, CAN sensors only, v2 serial frame waiting.
25. **[T]** Add a brake or signed-deceleration field to DRIVE_CMD and to `to_legacy` (W1). Contract change: team + hardware lead.

### Testing and simulation
26. Profiles: add `bench`, `carla`, `stack.launch.py target:= source:=`, `foxglove`.
27. twist_mux table → `twist_mux.yaml`: add rc 95, tests 60, goto 25, stop_sign and sensors locks at 30.
28. Test counts → "1110 tests, 37 packages (25 Oct)."
29. **[T]** Rules for the real car: add "no program drives through firmware with no timeout" (F1) and "reversing off until H5 and Q4 are answered" (F2).

### Roadmap
30. Replace the 5-phase drawing with `plan.md`'s 5 dated gates.
31. **[T]** Gate 2: "within 100 ms" → match the firmware's 111-120 ms, or tighten the firmware.

### Localization and navigation
32. Heading paragraph → "No magnetometer: `start_yaw`, then GPS velocity heading (`aav_gps_heading`)."
33. Delete "set `magnetic_declination_radians` and `yaw_offset`"; they stay 0 in datum mode.
34. Nav2: MPPI default, reversing on, collision_monitor fed by the VLP-16.
35. "How GPS A→B runs" → `/goto` action, `aav_goto` line follower and braking curve; Valhalla routes via `follow_gps_waypoints`.
36. Add: GPS runs at 1 Hz today; the datum is a placeholder.

### What we inherit
37. Add "All repos are in `archive/`, surveyed in `docs/older-code.md`." Mark the Flutter HUD archived; mark the 2024-25 repos read.

### Risks
38. Add: default path has no timeout (F1); Orin load (W7); one main contributor (O1); reversing without direction (F2); GPS at 1 Hz vs a 1 m stop.
39. "Rust, if used" → Rust is used: nodes on rclrs from crates.io; keep the note on support.

### Open questions
40. Mark Q25, Q26, Q29 answered; Q5, Q6, Q7, Q11, Q15, Q17, Q20, Q22 partly; Q12, Q16, Q27, Q28 built but not ratified **[T]**.

### Next steps
41. Strike steps 5-8 and 10 as done; keep 1-4; add "measure the car by 30 Oct" and "flash the Rust ESP32 on the bench".

### Sources
42. Add Cyclone DDS, MPPI, RF-DETR, MediaMTX, Valhalla, velodyne and Foxglove links.

### Process
43. **[T]** Pick one master: either `decisions.md` in git wins, or keep the doc current. Today `decisions.md` defers to a stale doc.
