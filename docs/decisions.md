# Decisions and open questions

What the team has decided, what's still open, and the 34 numbered questions. The source
is the team design doc, *AAV 2026–27 Autonomy Stack — Architecture & Plan* (a Claude Doc,
revision 61 on 10 October 2026). This page is a short copy, so you don't have to hunt.
When the two differ, the design doc wins.

## How to read the labels

| Label | Means | What you do |
|---|---|---|
| **Decided** | the team agreed | build on it; change it only after a team discussion |
| **Tentative** | the current direction, until a question is answered | build on it, but keep it easy to change |
| **Proposed** | a suggestion waiting for the team | accept or change it at the next team meeting |

Answer a question by its number, for example "Q12: targets". The design doc records the
answer.

## Decided

| # | Decision | Why |
|---|---|---|
| 1 | Commands follow one path: each driver → `twist_mux` → `ego` → one backend | Only ego talks to a backend; no driver reaches the actuators. Teleop (keyboard, remote, joystick) is a driver too (Ben, 9 Oct) |
| 2 | ego outputs Ackermann: `AckermannDriveStamped` | The real car, Gazebo, turtlesim and CARLA all take the same message, so a driver can't tell which one is running |
| 3 | Stock ROS packages: robot_localization (two EKFs), Nav2, nav2_costmap_2d, twist_mux | They replace the previous team's own estimator, planner, costmaps and command picking, where most of its bugs were |
| 4 | The joystick can drive the car with no autonomy running | Done with launch profiles |
| 5 | A CAN spine replaces the serial link from the Motor Pi to the ESP32 (today's car drives over UART: see 23 Oct) | Which signals cross the bus is still open: Q12 |
| 6 | ROS 2 Jazzy on every computer | Humble support ends in May 2027, Jazzy's in May 2029. ROS can't mix versions, so every computer moves together (Ben, 9 Oct) |
| 7 | One repo and one workspace, with folders by layer of the picture (since 25 Oct, by area: `ros/<area>/`, below) | One place to build, test and document everything |
| 8 | Languages: Python for the ROS nodes we write; Rust for the firmware, the drivers and the ports of the inherited C++; no C++ of our own (Ben, 10 Oct) | Our nodes run at 50 Hz or less, and the hard timing lives in the firmware. The spine logic is already Rust (`spine-core`) |
| 9 | Develop on Macs too, with Pixi and RoboStack | The same Jazzy on macOS and any Linux. The car's computers move to Ubuntu 24.04 (today 22.04: Q24); all use Cyclone DDS (23 Oct, below) |

## Tentative

| Decision | Why | Waits on |
|---|---|---|
| No QNX | `twist_mux` takes over QNX's job of picking which command wins. Hard real-time safety moves into the spine firmware and the hardware e-stop | Q23 |
| GPS gives the global position; the model is unknown | Whether it uses RTK corrections decides how precise GPS A→B can be | Q18, Q19 |
| Bare metal waits: only the spine firmware runs without an operating system | Ben, 9 Oct: "Maybe we will wait on this." | |
| The spine runs on an ARM chip, a Pico 2 or an STM32, not the ESP32 | Ben, 10 Oct: yes. It keeps the whole toolchain in Pixi. An STM32 has CAN built in; a Pico 2 needs an add-on CAN chip. A Pico 2 program exists since 11 Oct (`boards/pico2`). (The chip is the hardware lead's choice: Q10-Q13) | Q10–Q13 |
| The GPS driver: the stock `ublox_gps` by default; `nmea` and `ublox_dgnss` stay as `gps_driver:=` options | It's in Pixi on every computer. `ublox_dgnss`, the 2024-25 choice, is apt-only, so it runs on the car only | Q18 |
| The HUD's e-stop button only sets the e-stop. Clearing it stays a person's deliberate act, at the joystick or a laptop | A stray tap on the screen can't restart the car | Q29 |
| ~~Flutter 3.35.7, downloaded into `vendor/`~~ **Superseded 25 Oct:** the Flutter app is archived, and its download task is gone | Pixi has no Flutter. 3.38 and later rewrite the 2025-26 team's `pubspec.lock` | Q29 (answered) |
| Lane keeping: a classic OpenCV lane finder publishes `/perception/lane/geometry`; the 2025-26 lane keeping follows it | No model to train, and 0.78 ms a picture. A model can replace the finder later: the topic stays the same | Q22 |
| No micro-ROS for now: each board runs Rust and Embassy, and talks plain CAN, UART or UDP to a ROS node on Linux, which publishes. To think again once we see the car's hardware | Ben, 10 Oct: "hardware with embassy and rust talking to an actual node running ros". 11 Oct: "lets rethink thinks later after we see the hardware" | seeing the hardware: Q10–Q13 |

## Proposed: for the next team meeting

| Proposal | Why |
|---|---|
| `TwistStamped` on every command topic (`/drivers/*/cmd_vel`, `/control/cmd_vel`) | `twist_mux` expects it, and the timestamp lets ego drop stale commands. The code already does this |
| A car-safe Nav2: SmacPlannerHybrid with Regulated Pure Pursuit (or MPPI's Ackermann model), and no Spin in the behaviour tree | A car can't turn on the spot. The config already does this. (MPPI is the default since 22 Oct: see below) |
| The RC transmitter stays the top-priority override, in hardware | It works today, and doesn't depend on any computer |
| The spine boots latched, and arms only on a 0 → 1 edge; a driver fault stops the car | A reboot or a glitch never restarts the car by itself. It's in `spine-core` since 10 Oct |
| **The RC mode switch, three positions (TODO 48, 24 Oct):** CH6 (index 5, the 2025-26 `RC_EN_CH`) **low = computer, middle = RC via ROS, high = RC direct**; thresholds below 1301, 1301-1700, above 1700 (`ibus::RcMap::THREE_POSITION`). Needs a three-position switch on CH6 (the transmitter's aux-channel menu); a two-position switch still gives computer and RC direct. The team and the hardware lead decide; CH6's switch type and a free channel: question H10 ([audit questions](audit-questions.md)) | One switch, in the order of "how much the computer does". RC direct stays the fallback that needs no computer. Moving the switch from low to high passes RC via ROS, which drives only through ego's limits. Today's firmware builds keep the 2025-26 reading (>1500 = RC); only `--features uplink` reads three positions. Losing the RC in either RC position brakes (fault 256) |
| **RC via ROS at twist_mux priority 95** (TODO 48): below the joystick's deadman (100), above the keyboard (90) | A person with the RC is a person, so above every program. The joystick's holder sits in the car, so the deadman still wins. The ESP32 can't tell who in ROS made a command, so `rc_driver` sends zeros too while on RC via ROS, to keep Nav2 out |
| **The speed loop's `smooth` profile, to try on stands (TODO 27, 25 Oct):** a setpoint weight (b 0.75), kp 1.0, ki 0.25, the integral's limit 1.0 with clamping. **Not the default:** the firmware keeps today's gains until the stands; switch with `SpeedLoopConfig::smooth()` on a board, `speed_loop:=smooth` in sim and bench ([Tuning the speed loop](vehicle-model.md#tuning-the-speed-loop)) | Today's loop overshoots about 15% (0.57 m/s for 0.5) because of the PI's zero, not windup. The weight moves the zero and leaves the poles and margins alone. On the model: no overshoot at the guessed throttle, none at a 5x drag, 8% at a 5x weaker throttle (today 17%, and 29% short of 2 m/s); phase margin 42° or more up to a 2x stronger throttle, its lag doubled, 110 ms late. The real gains still come from the measured a_max and drag (TODO 52b) |
| A linear state-space model of the car, for tuning and the report (24 Oct): speed (first order, the spine's PI around it), steering (second order), the path errors (the bicycle, linearised), the delays; `aav_ego` `vehicle_model.py`, `pixi run vehicle-model` | Ben asked for "a matrix control thing ... maybe second order system" (24 Oct), in SYSC 3600's terms (its professor supervises the capstone). It shows what kinematics can't: the speed loop settles in about 13 s; the delays cost the lateral loop phase margin (52° → 31° at 1 m/s in the worst case); Gazebo steers 0.1 s slower than `vehicle.yaml`. It never drives the car; it is refit from the actuator tests ([The car as a linear model](vehicle-model.md)) |
| **No program drives through firmware with no command timeout (F1, 26 Oct):** with `frame_version: 1` (the 2025-26 C++ ESP32), `legacy_uart_bridge` says true on `/safety/legacy_uart_bridge/lock`, twist_mux's `no_timeout` lock at **priority 60**, timeout 0. `allow_programs_without_timeout: true` lifts it, on a supervised stand only | If the bridge or the Orin dies, that firmware keeps the last throttle. A person at the controls can react; a program can't. 60 holds scripted, replay, point, Nav2 and AI; people and the actuator tests (60: a person runs them, types a confirmation) still drive. Timeout 0, so the spine, sim, bench and turtle never see it |

## Ground rules

Rules for people and Claude sessions alike. They live in `CLAUDE.md`.

| Rule | In short |
|---|---|
| Units and frames | metres, radians, seconds; x forward, y left, z up; positive steering = left; `map → odom → base_link`, with `base_link` on the ground under the rear axle |
| Prefer stock | stock ROS packages first, then the previous teams' code, then a stub |
| Keep the maths out of ROS | a node's logic lives in a plain module, with its own tests |
| No pins in ROS | pins, I2C addresses and calibration stay in the firmware |
| Inherited code | keeps a `PROVENANCE.md`: where it came from, and every change since |
| Secrets | never in git, logs or docs: write down only where a secret is |
| Git | `main` lives on GitHub, in `AAV-2025-2026/ecolo-av` (public), and CI tests every push; small commits, each passing the tests, with messages that give the intention |
| Docs | short pages, sections, tables and pictures; Ben is dyslexic |
| Port first | move everything to Jazzy as it is, then consider rewriting parts in Rust |
| Simulators | Gazebo and bench mode every day; CARLA for the cameras, once the NVIDIA computer runs it (Q31, reopened 15 Oct) |

## Decided on 10 October, on the Mac

| Decision | By |
|---|---|
| `aav/aav` is our version of the project. The other folders are the previous teams' code, or downloaded source. *Since 25 Oct: `~/Code/ecolo-av`, everything at its root* | Ben |
| `aav` will become the merger of everything, in one organized project | Ben |
| Macs get GNU `coreutils` in their Pixi environment, so `colcon test` works on a Mac | a technical fix: see [Your Mac](mac.md) |
| Firmware and hardware drivers in Rust, not C or C++, even where the ROS nodes stay in Python. The 2025-26 C++ keeps running until each board's Rust version replaces it | Ben: see [Rust on the hardware](rust-firmware.md) |
| Bring perception in now, not later: the YOLO detector, the camera calibrations, radar clustering, lane keeping | Ben: "Lets merge in all of this too" |
| Flesh out the whole graph down to the sensors, and stub in what the hardware and firmware don't have yet: bench mode | Ben: "get as much done as we can and stub in the rest" |
| PyTorch and YOLO live in a separate `ml` environment, so the default one stays small | a technical choice: see [Perception](perception.md) |
| Camera images are subscribed reliably, not best effort | measured on the Mac: best effort lost every image |
| The code lives in `git@github.com:zander1976/aav.git`, public (Q25; Ben, 15 Oct). **Moved 25 Oct** into the team's `AAV-2025-2026/ecolo-av`, with its whole history (Ben: "I want to keep my complete history and merge my code into there") | Ben |
| CI runs `pixi run test` on Linux for every push, and on a Mac every night | Ben |
| Commit messages give the intention, with no attribution lines. The README's Credits section names the previous teams, the open-source projects and the AI assistant | Ben: "This is a school project" |
| The stand-ins (`turtle`, `sim`, `bench`, `view`, and the keyboard with `--sim` or `--local`) stay on the computer they run on, so a pretend car can never reach the real one | a safety choice, before the audit on the car's network: see [Simulation](simulation.md) |
| The car's red e-stop button always stops the car, whatever our software does. We don't rewire it; the spine only listens, through a spare contact, to report it and to stay stopped afterwards | Ben: "this already has an estop so we don't want to break that"; "I just want it to work" |
| Our C++ moves to Rust: the IMU driver (Ben's twin), the radar clustering (a Rust ROS node), the encoder and display Picos (Embassy); the spine replaces the ESP32. Stock ROS packages stay | Ben, 10 Oct: "Can we change all the c++ to rust?" |
| ML: a detector on each machine with a GPU (the Orin, a laptop), each under its own name; no device aliases | Ben, 10 Oct: "the orin be one node and/or my laptop be another node" |
| The order of work: the high-level pieces first, then Ackermann done properly (reversing, both wheels' angles, the steering curve), then the C++ ported to Rust, then every file rewritten to its language's own conventions, until the whole car works | Ben, 10 Oct: "Lets try to get a fully functioning thing going on" |
| Inference: TensorRT on the Orin (a `.engine` built there), Core ML on the MacBook Pro (a `.mlpackage`), both in our `aav_detector` node. Isaac ROS's TensorRT detectors are the Orin's upgrade path | Ben, 10 Oct: "TensorRT on the Orin and CoreML ... on the macbook pro" |
| A read-only MCP server lets Claude or another AI app read the docs, the contracts and the live graph. It never writes to ROS, reads only files in git, and has its own `mcp` environment | Ben: "Yes, read-only": see [AI apps (MCP)](mcp.md) |
| Today's C++ boards build with PlatformIO, in a `firmware` environment. Its downloads go into `vendor/platformio/` | Ben: "We want everybody to be able to build this out of the box" |
| CARLA next to Gazebo, through our own adapter (CARLA's Python client), not its ROS 2 link | reopened by Ben 15 Oct ("I think we want carla 5 not 4"): see Q31 and [CARLA](carla.md) |
| The team's Linux machine with an NVIDIA card trains models. The 2024-25 `best.pt` is probably on it | Ben |

## Decided on 11 October

| Decision | By |
|---|---|
| Work on what we can try at home, in this order: the full graph with the drivers, ego and the simulators; our C++ to Rust (Python later); the Mac on the network with ML on it, and Nav2; the drivers in Rust, ready to try on a board; then the GUI | Ben, 03:00: "We can only do so much from home so lets focus on stuff that makes sence to work on" |
| micro-ROS goes back from Decided to Tentative: no micro-ROS for now, and we think again once we see the hardware | Ben, 11:38: "No micro-ROS - lets rethink thinks later after we see the hardware" |
| The HUD becomes one Rust program, raylib and rclrs, that is also its ROS node: the driving screen first (speed, mode, the e-stop button, the camera). The Flutter app stays until the new one matches it; the map and the destination search wait on Q29 (since: the raylib HUD replaced it, 15 Oct; archived 25 Oct) | Ben, 11:58: "Raylib for the HUD: yes, no, or try it first? yes" |

## Decided on 15 October

| Decision | By |
|---|---|
| Topic names by area: `/<area>/<device or node>/<thing>`, with the areas `/sensors/`, `/vehicle/`, `/can/`, `/localization/`, `/perception/`, `/drivers/`, `/control/`, `/ego/`, `/safety/`, `/hud/`, `/system/` ([Topic areas](naming.md)). Each driver gets its own: `/drivers/keyboard/cmd_vel`, `/drivers/joy/cmd_vel` | Ben, 00:30: "The drivers all need to be in a /drivers/keybaord, etc type stuff as an example." |

## Decided on 22 and 23 October

What we changed, and why.

| Decision | Why | By |
|---|---|---|
| **Cyclone DDS on every computer**, the Macs and the car's. Pixi sets it; `AAV_RMW=fastdds` goes back for one run | Fast DDS delivered messages up to **0.48 s late** beside Gazebo, while their sender was on time; Cyclone never over **0.05 s** (25 min each, 22 Oct). The late messages had made sensors look dead and Nav2 refuse goals. `sim-check` passes on Cyclone, the rear camera at its full 10 Hz. Both are free and open source, and fully supported in Jazzy. To check at the car: Cyclone uses one network card ([Your Mac](mac.md#why-we-use-cyclone-dds)) | Ben, 23 Oct: "yes lets do it" |
| **MPPI is Nav2's default path follower**; Regulated Pure Pursuit stays as `controller:=rpp` | In every lot run in Gazebo it did at least as well. It reached the first goal cleanly in 4 of 4 runs; Pure Pursuit looped there in 2 of 3, as it aims at a point and arrives facing the wrong way ([Path followers](path-followers.md)) | Ben, 22 Oct: "I thought we decided to go mppi" |
| **The car may back up** (0.5 m/s at most), where the lidar watches behind it (the rear camera later); collision_monitor then guards a box behind the rear bumper. **Off by default on the car since 26 Oct** (`reversing:=true` to turn it on): the wheel sensors can't tell direction yet (H5); on again once H5 is answered | Driving forward only, a U-turn in a narrow lane works about half the time and can leave the car facing a wall. Allowed to reverse, it makes a three-point turn and reaches every goal | Ben, 22 Oct: "Yes the car can reverse" |
| **The GPS antenna is over the rear axle** (`vehicle.yaml`: x = 0) | It's already there on the car. That is the best spot: our position software corrects for the antenna's offset using the car's heading, so with no offset a heading error can't move the GPS position. In Gazebo, reversing's heading error halved ([Learn: GPS](learn-gps.md#mount-the-antenna-over-the-rear-axle)) | Ben, 23 Oct: "its already there" |
| **The wheel sensors are read by an Arduino**, which sends CAN frames; the Orin reads the bus and publishes them in ROS | The hardware side's plan. The frames are in `aav.dbc` already (0x200, 0x201), and the Orin's side decodes them, so the Arduino only has to send them. An exception to "Rust for all firmware": it's the hardware lead's board | the hardware lead (passed on by Ben, 23 Oct) |
| *Tentative, until the hardware lead confirms:* **today's car is driven through the Motor Pi's UART**, and **the Orin reads its CAN bus for sensors only** (`car.launch.py actuators:=uart`, the default; `spine_bridge role:=sensors`). The CAN spine stays one argument away (`actuators:=can`) | Ben's picture of the wiring: the actuators on the ESP32, with the Motor Pi as its network interface; the CAN bus straight to the Orin, with the wheel sensors. A `role` on `spine_bridge`, not a new node: the wheel decoding is already there and tested, and the sensors role has no way to send ([Today's wiring](can-plan.md#todays-wiring-23-october)) | Ben, 23 Oct: "the motor controllers are on the esp32 with a pi as its network interface" |
| **The cameras stream to the Orin with MediaMTX, compressed (H.264) on the Pi**; the Orin unpacks them with NVIDIA's video hardware, then OpenCV and the object detector (RF-DETR Nano, in TensorRT) run on them, in one program per camera | The camera Pi already runs MediaMTX with no ROS, and stays as it is. H.264 is about 10 times smaller than JPEG; raw pictures (83 MB/s each) never cross the network. People can watch the same stream in a browser. The cost, a little more delay (5-10 cm at 1 m/s), is to measure. `camera_ros` (ROS on the Pi) stays the fallback ([TODO 37b](../TODO.md)) | Ben, 23 Oct: "mediamtx on the pi to the orin seems like the best choice" |
| **TensorRT runs the detector on the Orin; OpenCV is whatever JetPack ships** (4.x if that's it) | TensorRT is NVIDIA's own and the fastest on the Orin. OpenCV only undistorts and finds lanes, so its version matters little, and JetPack's needs no build of our own | Ben, 23 Oct: "tensorrt is definately the best choice. if htat is opencv 4 then thats what we use" |
| **The detector is RF-DETR Nano**, not YOLO | In our comparison it saw the stop sign from 20 m (YOLO11n: 10 m) and found more people, with no false finds. Its licence is Apache-2.0; YOLO's is AGPL-3.0. Its cost: bigger weights, not yet timed on the Orin ([newer models](model-comparison.md)) | Ben, 23 Oct: "I thought we were skipping yolo and using the other one" |
| **Goals count within 1.0 m and 0.75 rad** | GPS without RTK is good to about a metre. At 0.5 m the car chased GPS noise and looped round goals it had reached | 22 Oct, from the Gazebo runs |
| **ego eases into speed**: a jerk limit of 2.0 m/s³, speeding up only | A smoother start for the people in the car. A stop is never softened: braking starts at once | 22 Oct, the proposal's T5.3 |
| **The global EKF trusts the gyro and wheels for heading**; GPS only nudges it (yaw noise 0.06 → 0.01) | At walking pace its heading jumped up to 76°; now the worst per leg is 7-16° in Gazebo. To check with the real IMU | 22 Oct, from the Gazebo runs |

## Decided on 24 October

| Decision | Why | By |
|---|---|---|
| **The detector package is `aav_detector`**, was `aav_yolo`; its node is `object_detector` (`object_detector_<machine>` on the car), was `yolo_detector`. Topics, parameters and `detector:=yolo` stay | It runs RF-DETR Nano by default now, YOLO only as an option, so a name with "yolo" in it misled. `detector:=yolo` keeps its name: it names the model family | Ben, 24 Oct: "aav detector yes" |
| **A HUD that drops out holds Nav2 until a person clears it** (latched, `hud LOST`). One dropout within 5 s of its first beat is forgiven; any other latches, even once it beats again. A person checks the screen, then `pixi run watchdog-clear` (the same service as a GPS jump) | The safety drivers rely on the screen. A HUD that crashed once may crash again; driving on by itself after systemd restarts it hides that. A window's slow first frames shouldn't need a person, so start-up gets one free dropout. The watchdog's own stalls never latch it ([HUD](hud.md#the-heartbeat)) | Ben, 24 Oct |
| **The heartbeat starts on the Orin and ends on the ESP32; the Pi only relays it** (TODO 50). The output mux adds 1 to a counter for each command it passes on (`/control/<target>/heartbeat`); `legacy_uart_bridge` copies it into a **version 2 serial frame** (17 bytes: `0xA5`, version 2, the three values, the counter, armed and e-stop flags, CRC-8/SAE-J1850); the ESP32 brakes, latched, when it hasn't changed for 100 ms. The 13-byte frame stays as a fallback (`--features link-v1`, `frame_version: 1`); the car's `legacy.yaml` keeps 1 until the Rust firmware is flashed | The Pi could be cut off from the Orin and still send frames, so a heartbeat the Pi makes proves nothing. The counter shows the last node on the Orin is alive **and** its gate is open. The CRC is the CAN plan's, so the team learns one. The C++ on the car reads only version 1, and could misread version 2 ([the heartbeat](rust-firmware.md#the-heartbeat-version-2-of-the-serial-frame-24-oct)) | Ben, 24 Oct |
| **The HUD shows whether the car may move**: GO, WAIT, STOP or AUTO STOP, a colour, an icon and the one main reason (the e-stop, the output mux's gate, the sensor watchdog) | The safety drivers need one answer at a glance, short words, colour and a shape, not a log line ([HUD](hud.md#may-the-car-move-the-go-stop-panel)) | Ben, 24 Oct |
| **`spine_bridge` refuses to arm while the safety gate is shut**: `/spine/arm` says no unless the output mux said `OPEN` for the bridge's own target in the last 1.5 s, and says why. Both bridges, Rust and Python; disarming always works. The bench and Gazebo arm through `arm_when_open`, which waits for `OPEN` | While the gate is shut, the bridge gets no commands, so an armed spine only latched again at once: arming said "armed" and did nothing. Now the answer is honest, and names the reason (a STOP's cause, a quiet mux). `legacy_uart_bridge` has no arm step ([Arming waits for OPEN](sources-and-targets.md#arming-waits-for-open)) | Ben, 24 Oct |
| **The muxes err shut on discovery's order**: a node on `/system/source` not heard yet holds the gate; an announcement counts before its node shows in the graph. **Test runs take a ROS domain nobody uses** | A guard test's SIM case let frames through once: another run on the same random domain (cross-talk, seen the same day in a Gazebo check). Across processes the graph can show an announcer 30 ms before its message: small, but it erred open ([Sources and targets](sources-and-targets.md#what-it-cant-catch)) | 24 Oct, from a test that failed on main |

## Decided on 25 October

| Decision | Why | By |
|---|---|---|
| **CI tests on 64-bit ARM Linux too, on every push**: `pixi run test` and the HUD drawn once (job `arm`), and the car kit installed with no network (job `car-kit`), on GitHub's free ARM runner | The Orin and the Pis are ARM; nothing had built there before the car day. 12 and 7 min beside the Linux job, so every push, not nightly: a break shows on the push that made it ([CI](infrastructure.md#sharing-the-code)) | Ben's go-ahead, 24 Oct: everything that can be done without the car |
| **Tests never use the network** (`pixi run test` and `tools/test.sh` set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), and **colcon tests one package at a time** | On GitHub's Mac a test on the network never ended: Cyclone DDS's multicast reader can't be stopped there (TODO 42). On one computer and one domain, two packages' tests side by side heard each other. One at a time costs 14 s | the same |
| **The global EKF takes the heading from the GPS receiver's velocity** while the car moves (`gps_heading`, `aav_gps_heading`; `gps_heading:=false` leaves it out). Gazebo's GPS now has a velocity too | The receiver measures its velocity from the Doppler shift, good to 0.05 m/s while the position wanders by half a metre: its direction is the heading (turned round when reversing). In Gazebo, 4 runs each at walking pace: heading off by a median 1.6-2.1° (was 2.2-2.8), worst 8.5-13.6° (was 11.4-27.1), forward and reversing alike. The magnetometer (motors), lidar matching (not installed, and no north) and robot_localization's own settings (no effect with a datum) were looked at and not taken ([Learn: localization](learn-localization.md#where-the-heading-comes-from-todo-13)) | Ben's go-ahead, 24 Oct (TODO 13) |
| **Every README has a CLAUDE.md beside it** that reads it in (`@README.md`); EGM's designs go in `docs/designs/` | The Elephant-Goldfish plugin (egm.dev) works from each folder's README, and Claude Code only loads a folder's CLAUDE.md. A test keeps them paired | Ben, 25 Oct |
| **Only Ben sends email** | The repo is public and students run their own Claude sessions in it; the mailer is Ben's channel | Ben, 25 Oct |
| **Docs live in `docs/`, under plain names**; the root keeps README, CLAUDE.md, CONTRIBUTING.md and TODO.md | Fourteen READ-ME, PLAN, PROPOSAL and TODO pages at the root were hard to find and named oddly. Their information moved into `docs/`; the dated pages (the proposal, the Friday audit) carry a snapshot banner | Ben, 25 Oct |
| **A features list, `docs/features.md`**: each feature's type, why, who and status, updated in the feature's own commit | The team's report must say what each person built and why; a rule in CLAUDE.md and a checklist on pull requests keep it current | Ben, 25 Oct |
| **ROS packages live in `ros/<area>/`**: sources, localization, perception, drivers, command_mux, ego, io_mux, can_bridge, sim, safety, hud, bringup, ecolo_msgs (was `src/<layer>/`) | Each folder matches a box in the picture, so you find a package where its data flows | Arsh, 25 Oct |
| **The Flutter HUD and its Flask back end are archived** (`archive/2026-27-flutter-hud/`); the raylib package takes the name `aav_hud`; the route maths moves to `aav_routes` | Q29: the raylib HUD replaced the app on 15 Oct. Nothing built, tested or started the old one any more | Ben, 25 Oct |
| **[CONTRIBUTING.md](../CONTRIBUTING.md) is the git rule**: fork, branch, pull request, one approval and green CI, then the merge queue | One way into `main` for everyone, maintainers included | Arsh, 25 Oct |

## Decided on 29 October

| Decision | Why | By |
|---|---|---|
| **Nav2 keeps its path for the last 8 m to a goal** (`navigate_to_pose_car*.xml`): it replans every second elsewhere, but near the goal only if the path is blocked, for another goal, or 20 s old. 8 m = `vehicle.final_approach`, 2 of the planner's turning radii | Replanning there made the car circle goals: a new path from a little behind on the last bend is a 21-25 m loop for a car, and each new path resets Nav2's goal checker. Gazebo, first leg, 12 runs each way: loop plans in 10 before, none after; 3 loops driven before, none after ([Path followers](path-followers.md#circling-a-goal-found-and-fixed-29-october-todo-17)) | TODO 17, from the Gazebo runs |

## Open questions

The design doc numbers 34 questions, and doesn't rank them. **Bold** marks the ones that
look like they block phase 0 or the first drives: a suggestion, for the team to change.
The design doc's next steps take Q1–Q9, Q18, Q20 and Q22 to the car with a tape measure.

### The car

| Q | Topic | The question |
|---|---|---|
| **Q1** | Dimensions | Wheelbase, track, overall size, steering limit, top speed? The old code disagrees: 1.3–1.57 m wheelbase |
| **Q2** | Steering feedback | Is the steering's position sensor wired to its Jrk G2? How do its counts map to the wheel angle? |
| **Q3** | Brake | How far does it travel, how hard does it brake, and what does it do if power is lost? That defines the safe state |
| **Q4** | Throttle | Which motor controller does the DAC drive? Does it brake on its own? Is reverse safe while moving? |
| Q5 | Wheel encoder | Which wheel, how many counts per turn, and can it wire straight to the spine? **Partly answered:** the wheel sensors go through the hardware lead's Arduino, which sends CAN 0x200/0x201 (23 Oct); one board or two is question H4 |
| **Q6** | E-stop | The car already has one (Ben, 10 Oct). Still open: what kind it is, what it cuts, and how it's wired |
| Q7 | RC override | Does the FlySky stay the top override? How does the spine know RC has taken over? |
| Q8 | Power | Battery voltage, the computers' supplies, and what the actuators do when a computer loses power |
| Q9 | Which computers stay | Orin, UI Pi, Motor Pi, radar Pi, camera Pi, QNX Pi? Suggested: the Orin, the spine and the HUD screen |

### The spine and the actuators

| Q | Topic | The question |
|---|---|---|
| **Q10** | Owner | Who owns the spine firmware and the CAN frame map? |
| Q11 | Starting firmware | Which firmware is on the ESP32 now? Build on AAV_cortex's, Motor_Steering's add-on, or the 2024-25 `FinalRev`? |
| **Q12** | What the spine accepts | Target speed and steering angle (the speed loop in the firmware), or raw throttle and brake? Suggested: targets |
| Q13 | Board | Add a CAN transceiver to today's ESP32 board, or a new board? Who changes the PCB? |
| Q14 | The Orin's side | Its built-in CAN with a transceiver, or a USB-CAN adapter? |
| Q15 | The bus | Bitrate, connectors, cable route, terminators, and what else shares it |
| Q16 | Watchdog | How long without a command before it acts, how hard it brakes, and what the steering does |
| Q17 | Wireless updates | Keep them? If so, how are they authenticated? |

### Sensors and localization

| Q | Topic | The question |
|---|---|---|
| **Q18** | GPS model | Which receiver is on the car? RTK or plain GPS? One antenna or two? It also picks the ROS driver |
| Q19 | Corrections | If RTK: which correction source, and is there internet at the test site? |
| Q20 | IMU | Keep the Yahboom on USB? Where is it mounted? Is its magnetometer usable on the car? |
| **Q21** | Test site | Where do we drive, and who gives permission? A surveyed point there becomes the map origin |
| Q22 | Lidar and cameras | Which models are on the car? Not needed until phase 4, but the URDF and the power budget need them. **Partly answered:** a Velodyne VLP-16 on the roof (16 Oct audit); both front ribbon cameras on one Pi 5; the camera models still open |

### Software and process

| Q | Topic | The question |
|---|---|---|
| Q23 | QNX | Is it definitely dropped? Is it a course or sponsor requirement? |
| **Q24** | Jazzy on the car | Who images and reflashes the Orin, and when? Is there a spare machine to try JetPack 7.2 first? Until then, Pixi can run Jazzy on the car's Ubuntu 22.04 |
| Q25 | Repo | A new 2026-27 GitHub organization, or the existing one? **Answered by Ben, 10 Oct: `zander1976/aav`. Public: Ben, 15 Oct ("Public is what we want for github")**. 25 Oct: merged into `AAV-2025-2026/ecolo-av`, the team's repo, history kept |
| Q26 | Languages | Python for all our ROS nodes? C++ or Rust for the spine firmware? **Answered: Decided #8 (Ben, 10 Oct)** |
| Q27 | Steering at standstill | A speed-and-turn-rate command can't steer at zero speed. Should the joystick turn the wheels while stopped? |
| Q28 | Joystick handover | When the hold-to-drive button is released, go back to the autonomous driver, or stay stopped? |
| Q29 | HUD | Keep the Flutter app? Which screen runs it? Does it send GPS destinations this year? **Answered:** the raylib HUD replaced the app (Ben, 15 Oct); the app and its back end were archived on 25 Oct (Ben). Destinations: the map window's place search. Still open: which computer drives the screens ([Team to-do](team-todo.md)) |
| Q30 | Remote driving | In scope this year? Over which link, with what latency limit? |
| Q31 | CARLA | **Reopened by Ben 15 Oct: research done, adapter ready, needs the NVIDIA machine.** `carla_adapter` uses CARLA's Python client, so it runs with CARLA 0.10 (Unreal Engine 5) and 0.9.16 alike, without the ROS 2 link that stalls on Jazzy. Town10 on 0.10 for the cameras; Carleton's streets (from OpenStreetMap) only on 0.9.16 for now, as 0.10 can't import maps yet. Never run against a server: no Mac version ([CARLA](carla.md)). Was: parked, 10 Oct |
| **Q32** | Test rules | Who is the safety driver, what's the speed cap, what's on the pre-drive checklist, who signs it off? |
| **Q33** | Owners | Who owns each package? |
| **Q34** | Secrets | Have the leaked GitHub token, Firebase keys and WiFi password been revoked or changed? Who controls those accounts? |

## Also waiting on the team

From tonight's audits and the Mac work. They're small, but someone has to say yes.

| Item | The choice |
|---|---|
| `ROS_DOMAIN_ID` | one number for the car and every laptop: 0–101 (safe on Linux; Fast DDS breaks above 232) |
| GPS driver | `ublox_gps` is the default now (Tentative, above); keep it, or go back to `ublox_dgnss` (the car only). With Q18 |
| `/safety/estop` reliability (N2) | send it repeatedly, or make the mux hear it reliably |
| Respawning device nodes (N3) | restart a sensor driver when it dies? |
| Nav2's lookahead (N7) | its turn command flips sign about once a second in the sim |
| Package renames | the inherited packages' new names, in [Conventions](conventions.md) |
| A `vendor/` folder | keep downloaded source out of git, with a list to fetch it again |
| Stop signs as a twist_mux lock | priority 30: it holds Nav2 and the AI slot, never a person or a test script; 4 s stop, then 10 s to drive past. Built and working on the bench (Proposed) |
| The steering on an e-stop | hold where it is (`ESTOP_STEERING = Hold` in `spine-core`), or centre it (the 2025-26 firmware) |
| The brake after boot | `IDLE_BRAKE = 0.0` lets it go once the e-stop clears; the 2025-26 firmware kept it full on (1.0). ⚠️ Keeping it on may be safer on a slope |
| The RC gear switch's brake position | `RC_GEAR_BRAKE = SpeedZero`: the speed loop brakes to a stop. `RcInput` now has a brake field, so the old full brake is one constant away |
