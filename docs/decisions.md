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
| 3 | Stock ROS packages: robot_localization (two EKFs), Nav2, nav2_costmap_2d, twist_mux (Nav2 and its costmaps replaced by our Rust ports, 17 Nov: below) | They replace the previous team's own estimator, planner, costmaps and command picking, where most of its bugs were |
| 4 | The joystick can drive the car with no autonomy running | Done with launch profiles |
| 5 | A CAN spine replaces the serial link from the Motor Pi to the ESP32 (today's car drives over UART: see 23 Oct) | Which signals cross the bus is still open: Q12 |
| 6 | ROS 2 Jazzy on every computer | Humble support ends in May 2027, Jazzy's in May 2029. ROS can't mix versions, so every computer moves together (Ben, 9 Oct) |
| 7 | One repo and one workspace, with folders by layer of the picture (since 25 Oct, by area: `ros/<area>/`, below) | One place to build, test and document everything |
| 8 | Languages: Python for the ROS nodes we write (since 16 Nov: Rust for new ones, below); Rust for the firmware, the drivers and the ports of the inherited C++; no C++ of our own (Ben, 10 Oct) | Our nodes run at 50 Hz or less, and the hard timing lives in the firmware. The spine logic is already Rust (`spine-core`) |
| 9 | Develop on Macs too, with Pixi and RoboStack | The same Jazzy on macOS and any Linux. The car's computers move to Ubuntu 24.04 (today 22.04: Q24); all use Cyclone DDS (23 Oct, below) |

## Tentative

| Decision | Why | Waits on |
|---|---|---|
| No QNX | `twist_mux` takes over QNX's job of picking which command wins. Hard real-time safety moves into the spine firmware and the hardware e-stop | Q23 |
| GPS gives the global position; the model is unknown | Whether it uses RTK corrections decides how precise GPS A→B can be. Ben, 5 Nov: no RTK; NDT against a map of the lot is to replace the GPS's offset (a trial, not on the car) | Q18, Q19 |
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
| A car-safe Nav2: SmacPlannerHybrid with Regulated Pure Pursuit (or MPPI's Ackermann model), and no Spin in the behaviour tree | A car can't turn on the spot. The config already does this. (MPPI is the default follower since 22 Oct, the State Lattice the default planner since 1 Nov: see below. Since 17 Nov our ports do the same: no Spin, the lattice, `mppi_lag`) |
| The RC transmitter stays the top-priority override, in hardware | It works today, and doesn't depend on any computer |
| The spine boots latched, and arms only on a 0 → 1 edge; a driver fault stops the car | A reboot or a glitch never restarts the car by itself. It's in `spine-core` since 10 Oct |
| **The RC mode switch, three positions (TODO 48, 24 Oct):** CH6 (index 5, the 2025-26 `RC_EN_CH`) **low = computer, middle = RC via ROS, high = RC direct**; thresholds below 1301, 1301-1700, above 1700 (`ibus::RcMap::THREE_POSITION`). Needs a three-position switch on CH6 (the transmitter's aux-channel menu); a two-position switch still gives computer and RC direct. The team and the hardware lead decide; CH6's switch type and a free channel: question H10 ([audit questions](audit-questions.md)) | One switch, in the order of "how much the computer does". RC direct stays the fallback that needs no computer. Moving the switch from low to high passes RC via ROS, which drives only through ego's limits. Today's firmware builds keep the 2025-26 reading (>1500 = RC); only `--features uplink` reads three positions. Losing the RC in either RC position brakes (fault 256) |
| **RC via ROS at twist_mux priority 95** (TODO 48): below the joystick's deadman (100), above the keyboard (90) | A person with the RC is a person, so above every program. The joystick's holder sits in the car, so the deadman still wins. The ESP32 can't tell who in ROS made a command, so `rc_driver` sends zeros too while on RC via ROS, to keep Nav2 out |
| **The speed loop's `smooth` profile, to try on stands (TODO 27, 25 Oct):** a setpoint weight (b 0.75), kp 1.0, ki 0.25, the integral's limit 1.0 with clamping. **Not the default:** the firmware keeps today's gains until the stands; switch with `SpeedLoopConfig::smooth()` on a board, `speed_loop:=smooth` in sim and bench ([Tuning the speed loop](vehicle-model.md#tuning-the-speed-loop)) | Today's loop overshoots about 15% (0.57 m/s for 0.5) because of the PI's zero, not windup. The weight moves the zero and leaves the poles and margins alone. On the model: no overshoot at the guessed throttle, none at a 5x drag, 8% at a 5x weaker throttle (today 17%, and 29% short of 2 m/s); phase margin 42° or more up to a 2x stronger throttle, its lag doubled, 110 ms late. The real gains still come from the measured a_max and drag (TODO 52b) |
| A linear state-space model of the car, for tuning and the report (24 Oct): speed (first order, the spine's PI around it), steering (second order), the path errors (the bicycle, linearised), the delays; `aav_ego` `vehicle_model.py`, `pixi run python3 tools/vehicle_model.py` | Ben asked for "a matrix control thing ... maybe second order system" (24 Oct), in SYSC 3600's terms (its professor supervises the capstone). It shows what kinematics can't: the speed loop settles in about 13 s; the delays cost the lateral loop phase margin (52° → 31° at 1 m/s in the worst case); Gazebo steers 0.1 s slower than `vehicle.yaml`. It never drives the car; it is refit from the actuator tests ([The car as a linear model](vehicle-model.md)) |
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
| Simulators | our own simulator (the default since 8 Nov) and bench mode every day; Gazebo until 16 Nov; CARLA for the cameras, once the NVIDIA computer runs it (Q31, reopened 15 Oct) |
| Trials | a new idea is its own package beside the default, chosen by a switch, scored on the same routes; it becomes the default only once it wins (Ben, 29 Oct, 5 Nov) |

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
| **MPPI is Nav2's default path follower**; Regulated Pure Pursuit stays as `controller:=rpp` (both went with Nav2 on 17 Nov; our `mppi_lag` follows) | In every lot run in Gazebo it did at least as well. It reached the first goal cleanly in 4 of 4 runs; Pure Pursuit looped there in 2 of 3, as it aims at a point and arrives facing the wrong way ([Path followers](path-followers.md)) | Ben, 22 Oct: "I thought we decided to go mppi" |
| **The car may back up** (0.5 m/s at most), where the lidar watches behind it (the rear camera later); collision_monitor then guards a box behind the rear bumper. **On the car by default again since 1 Nov** (Ben: "reverse and uturn should be in"; off 26 Oct-1 Nov): until the wheels tell direction (H5), the speed's sign is the last command's; the car's planner is then the lattice | Driving forward only, a U-turn in a narrow lane works about half the time and can leave the car facing a wall. Allowed to reverse, it makes a three-point turn and reaches every goal | Ben, 22 Oct: "Yes the car can reverse" |
| **The GPS antenna is over the rear axle** (`vehicle.yaml`: x = 0) | It's already there on the car. That is the best spot: our position software corrects for the antenna's offset using the car's heading, so with no offset a heading error can't move the GPS position. In Gazebo, reversing's heading error halved ([Learn: GPS](learn-gps.md#mount-the-antenna-over-the-rear-axle)) | Ben, 23 Oct: "its already there" |
| **The wheel sensors are read by an Arduino**, which sends CAN frames; the Orin reads the bus and publishes them in ROS | The hardware side's plan. The frames are in `aav.dbc` already (0x200, 0x201), and the Orin's side decodes them, so the Arduino only has to send them. An exception to "Rust for all firmware": it's the hardware lead's board | the hardware lead (passed on by Ben, 23 Oct) |
| *Tentative, until the hardware lead confirms:* **today's car is driven through the Motor Pi's UART**, and **the Orin reads its CAN bus for sensors only** (`car.launch.py actuators:=uart`, the default; `spine_bridge role:=sensors`). The CAN spine stays one argument away (`actuators:=can`) | Ben's picture of the wiring: the actuators on the ESP32, with the Motor Pi as its network interface; the CAN bus straight to the Orin, with the wheel sensors. A `role` on `spine_bridge`, not a new node: the wheel decoding is already there and tested, and the sensors role has no way to send ([Today's wiring](can-plan.md#todays-wiring-23-october)) | Ben, 23 Oct: "the motor controllers are on the esp32 with a pi as its network interface". Seen on 6 Nov: the Motor Pi's bridge holds the ESP32's UART (`history/audit-2026-11-06-car.md`); the wire itself isn't checked |
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
| **A HUD that drops out holds navigation until a person clears it** (latched, `hud LOST`). One dropout within 5 s of its first beat is forgiven; any other latches, even once it beats again. A person checks the screen, then `pixi run clear watchdog` (the same service as a GPS jump) | The safety drivers rely on the screen. A HUD that crashed once may crash again; driving on by itself after systemd restarts it hides that. A window's slow first frames shouldn't need a person, so start-up gets one free dropout. The watchdog's own stalls never latch it ([HUD](hud.md#the-heartbeat)) | Ben, 24 Oct |
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
| **Nav2 keeps its path for the last 8 m to a goal** (`navigate_to_pose_car*.xml`; our navigator does the same since): it replans every second elsewhere, but near the goal only if the path is blocked, for another goal, or 20 s old. 8 m = `vehicle.final_approach`, 2 of the planner's turning radii | Replanning there made the car circle goals: a new path from a little behind on the last bend is a 21-25 m loop for a car, and each new path resets Nav2's goal checker. Gazebo, first leg, 12 runs each way: loop plans in 10 before, none after; 3 loops driven before, none after ([Path followers](path-followers.md#circling-a-goal-found-and-fixed-29-october-todo-17)) | TODO 17, from the Gazebo runs |

## Decided from 26 October to 14 November

Short; the words are in `docs/history/messages-*.md`, the work in `TODO.md`.

| When | Decision | By |
|---|---|---|
| 26 Oct | `aav_goto` becomes `aav_drive_to_point` (action `/drive_to_point`, slot `point`): the name says the job | Ben |
| 29 Oct | **Trials beside defaults:** new nodes or plugins run beside the main ones and replace them only if they win ([Trials](trials.md)) | Ben: "test and then replace the main ones if they work" |
| 1 Nov | **The State Lattice is the default planner** where the car may reverse, the car too; Hybrid-A* forward only. **Reversing and the U-turn are on, on the car** (TODO 59, 62) | Ben: "reverse and uturn should be in" |
| 1 Nov | **0.2 = sim-complete, car-ready; 0.3 = it drove.** Car day is split: Day A destroys nothing, Day B upgrades; every idea is kept for later | Ben |
| 1 Nov | Fewer `pixi run` commands (89 → 45), families with one word after, no aliases | Ben: "Less is more" |
| 2 Nov | Initial D red and black on all four screens; the diffusion planner in 0.2 as a trial, off by default, behind its guard | Ben |
| 3 Nov | **No default switched for 0.2:** NDT-EKF was closer (0.018 m against 0.174 m) but needs the lot's map; the cubes were slower and once drove into a post ([Switches](switches.md)) | from the Gazebo runs |
| 3 Nov | One screen per concept: **Drive** and **Hardware** ("just the stuff we control"); the HUD and the map stay in the car | Ben |
| 4 Nov | Picked from the [weak links](designs/2026-11-04_weak-links.md): the clocks, the fence and the site file (TODO 100); the brake follows a wish (101); a behaviour state machine (102); research A, the car identifies itself (104). The tracker (103) paused 5 Nov; shadow mode (108) not picked | Ben |
| 5 Nov | **Our own algorithms:** for each area a new best hybrid from papers, known algorithms and current code, how the car stops first. Stock stays the default until ours wins; every algorithm stays. "One car" is for down the road | Ben |
| 5 Nov | **Everything is a drop-in:** Gazebo, our simulator and the real sensors give the same topics; `pixi run drop-in` checks | Ben: "just swap out and test" |
| 5 Nov | **Screens by computer:** the HUD and the street map stay raylib, on the Pi; laptop screens can be Bevy; the Orin runs no graphics and no simulator | Ben |
| 5 Nov | A drive only suggests map changes; a person accepts them. No RTK (above) | Ben |
| 7 Nov | **0.3:** one car model shared by ego, the simulators and the Rust drivers; algorithms that know the lag (MPPI in Rust, the lag-aware MPC); the behaviour planner and the stop-point check; directions on the campus | Ben |
| 7 Nov | **Gazebo merged into Rust:** our own simulator, every sensor, beside Gazebo until it wins | Ben: "merge in everything from gazebo so its standalone in rust" |
| 7 Nov | **Diffusion on the car only as a trial:** `diffusion_trial:=true`, 0.5 m/s at most, the guard and every lock as always | Ben: "We want diffusion" |
| 7 Nov | **The Rust ESP32 keeps every C++ behaviour at a minimum** ([parity](esp32-cpp-parity.md)); its WiFi pages as JSON, read only, off by default | Ben |
| 7 Nov | **One forward speed limit, 1.0 m/s, for ego and the spine**, with a test that fails if they differ; the spine's stop rules: every lost input is a hard stop (T3.6) | Arsh |
| 8 Nov | **Our own simulator is the default** (`simulator:=own`); `simulator:=gazebo` for computers with no GPU (until 16 Nov: the camera lockstep took that job) | Ben, after the head-to-head runs |
| 8 Nov | **Off Nav2 one piece at a time** (costmap, planner, navigator, smoother), each beside Nav2 until it wins; **Nav2 stays the default and on the car** (until 17 Nov: below) | Ben |
| 8 Nov | **On the car, 0.3 keeps the defaults:** the new pieces are off or simulator-only ([Rolling out](rollout.md)) | Ben |
| 8 Nov | The adaptive brake, a sim trial: brake harder when the car slows too little, never past the brake's safe maximum | Ben |
| 14 Nov | **The lane map is a plain `lanes.yaml`** (W5), the campus's own format: each road's centre line, width, lanes, one way. Lanelet2 (RoboStack 1.2.3, BSD-3-Clause) was tried first: it installs, but on a Mac its Python modules are `.dylib` files that don't import, and loading them by hand hangs (a second libpython). Not added to `pixi.toml`; try again with a newer build | Ben, W5: "try it; else grow `lanes.yaml`" |
| 14 Nov | **The release is 0.4-rc1**, not 1.0: "We are still in beta stages". Must-haves: reverse, and no oscillation | Ben |
| 14 Nov | **Three levels with fixed map chunks**: the route planner (zoomed out), the path planner (middle), the local planner (close up), the behaviour planner across all three; the saved voxel map as the background for tracking (TODO 175) | Ben |
| 14 Nov | **Behaviour is a feature type of its own** | Ben |
| 14 Nov | The sensor watchdog **warns only** about our own nodes; respawn and the age checks stop the car | Ben |
| 14 Nov | Our simulator's wheels get **noise and slip, seeded, off by default** | Ben |
| 14 Nov | **Steering-rate-aware lattice moves**, a trial beside today's lattice (no oscillation) | Ben |
| 14 Nov | The smooth moves are made for **1 m/s** (`smooth`, ego's top speed) and **2.5 m/s** (`smooth_fast`, the campus), at 1/1.25 of ego's 0.6 rad/s; with them the lattice's **smoother is off** (its Dubins ends put the jumps back) | Ben |
| 14 Nov | **Hybrid-A\* in our Rust planner now**, another option beside the lattice | Ben |
| 14 Nov | **We run our own stack; Nav2 and Gazebo leave**: "I would like to remove nav2 and gazeboo ... run everythign ourselves in rust and/or python". `stack:=ours` and our simulator become the default now; Nav2 and Gazebo stay only as yardsticks until 0.4-rc1 is scored, then are deleted. robot_localization, twist_mux and teleop_twist_* stay for now. CLAUDE.md's rules changed to match | Ben |
| 15 Nov | **No freeze on new challengers** (the whole-car design's W6: no): "We want to build the best version ... that might inlcude reading papers or merging ideas together that we already have" | Ben |
| 15 Nov | **The order**: the 0.4-rc1 list first (the sway, 170, 174, 182-185), then the whole-car design's gaps (the planner inside the road ahead's edges, the soft edge to pass, following at a gap, the things' critic), then 155 and 156 | Ben |
| 15 Nov | **Lanelet2 in Rust** (our own reader, TODO 175) now, after the sway work | Ben |
| 15 Nov | The old `wip/` branches (7 Nov): checked against `next`, anything useful taken, then archived | Ben |
| 15 Nov | **Map correction stays `newest`**; `slow` (TODO 186) is a challenger: it won offline, not yet in the simulator (later that day it won in the sim: the `ours` stack takes it, below) | Ben's rule (5 Nov), applied at night |
| 15 Nov | **Our planner prices a change of gear** (`cusp_penalty: 4.0`, TODO 174): a three-point turn in a 6.5 m street went from 9 changes of direction to 4 (2 runs), the lot and campus tours no worse. Nav2's planner ignores it; 0 gives Nav2's way back | Ben's rule (5 Nov): it won on the same routes |
| 15 Nov | **The radar clears our costmap inside its view** where it sees nothing (TODO 193), so a thing taken away is forgotten | Ben's night orders |
| 15 Nov | **32 lattice headings: no win**; 16 stays (TODO 190) | Ben's night orders |
| 15 Nov | **D2, lane costs**: the one-shape design's proposal taken (a ramp, lowest at the right lane's centre, against `path:=lane`); the go-ahead to finish Fable's designs and the rc1 list | Ben: "lets try to get all of these things finished and merged and tested" |
| 15 Nov | **Everything goes into the next release (0.4-rc1)**: nothing is held back for a later one | Ben: "Nothing is going into a future rekease we want everything in the next release" |
| 15 Nov | **One folder on Ben's Mac**, `~/Code/ecolo-av` on `main`: the night's worktrees (`ecolo-next`, `ecolo-night`) merged and removed; the old `~/Code/aav` and `aav-wt` deleted (its one uncommitted edit kept in `docs/history/leftovers/`) | Ben |
| 15 Nov | The fork's branches (`one-shape`, `review-13nov`, `setup-localization`, `whole-car`) merged into its `main`, then deleted; their last commits: 0f0a1094, b7ed1837, 566ef0c0, 726bff44 | Ben |
| 15 Nov | **A laptop driver that drops off loses its goals** (TODO 116): the car's goal leash cancels the language driver's goals after 1 s without its beat, a normal stop, not an e-stop (in line with 5 Nov: a laptop dropping off is no reason to brake hard on the road; screens still never stop the real car) | Ben ("keep working through the list"; the cautious choice taken) |
| 15 Nov | **The pedal in ego** for people (joy, rc, keyboard): the stick's position sets how hard ego slows (let go: 1.0 m/s²; full back: 2.0 m/s²), then reverses at 0. Computer drivers keep plain speeds; the AI may join as a challenger | Ben (asked "Everyone", and "a similar concept for the ai too? what do you think?"; advised: people only, since a planner's own slowing must be followed at once, or the car rolls past its stop line) |
| 15 Nov | **The sim's `ours` stack takes the three sway fixes** (TODO 168): `plan_smoothing:=strong` (path `lattice_rs_strong`), and `map_correction:=slow` with `mppi_lag_steer_gamma:=0.05` (steer `mppi_lag_calm`). Scored in the sim, 2 runs each: sway at Nav2's on the lot (0.021 against 0.018 m/s²) and under it on the campus (0.099 against 0.131), every leg, closer to the path. The champions, the car and `stack:=stock` are unchanged; each old setting stays a switch ([why](history/mppi-lag-log.md#the-sway-that-was-left)) | Ben's rule (5 Nov): a challenger that wins on the same routes |
| 16 Nov | **The sim's `ours` plans the lane's centre** (`path:=lane`, TODO 148): 3 clean runs each of the `university` tour, no worse on any column than as now or the ramp (9/9 legs each, the same 512 s; sway 0.055 against 0.087-0.090 m/s², never over the centre line against 61 % and 1 %). Off the lane map it is the lattice, as before. The champions, the car and `stack:=stock` are unchanged ([why](own-planner.md#the-swap-16-nov-3-clean-runs-each)) | Ben's rule (5 Nov): a challenger that wins on the same routes; the swap asked for in TODO 148 |
| 16 Nov | **The sim's `ours` builds its global costmap from map chunks** (`costmap:=ours_chunks`, TODO 175): only the chunks round the car and along the route, each read and inflated on its own core (rayon); a goal off them planned in steps. Lot, campus and University Drive, 2 runs each: 20/20 legs as the whole map, the same time and sway, the costmap 474 to 145 MB. Nav2's costmap, `costmap:=ours`, the champions and the car are unchanged ([why](own-costmap.md#from-map-chunks-on-every-core-16-nov-todo-175-195)) | Ben: "Lets turn on chunking for sure"; no worse on legs and driving |
| 16 Nov | **`ours` passes something parked on the lane** (TODO 208): only the blocked stretch moves left, the least that clears it plus 0.5 m, never past the kerb, then back; the planner's lidar layer marks to 18 m (was 12). University Drive with a 1 m box, 3 runs each: over the centre line 4.8 s against 13.2, back in 9 m against 18.5, no stops; a person crossing still waited for; no worse with no obstacles. `lane_pass:=false` keeps the old way ([why](own-planner.md#passing-something-parked-16-nov)) | Ben: "stick to the right side of the lane unless we need to for obstical avoidance" |
| 16 Nov | **Rust is the default on the car too** for ego + look_ahead, behaviour, the sensor watchdog and localization_health (`aav_ego_rs`, `aav_behaviour_rs`, `aav_watchdog_rs`, `aav_localization_health_rs`). The Python stays installed, one switch each (`*_impl:=python`), until a car day proves the Rust ([car day, 4f](car-day.md#4f-the-rust-nodes-are-the-ones-running)) | Ben: "watchdog in rust sure"; the same answers as the Python, in the simulator first |
| 16 Nov | **New ROS nodes are Rust** (rclrs), replacing decision 8's "Python for the ROS nodes we write". Still Python: ML on the GPU (train in PyTorch; inference later through ONNX), launch files, tools and tests | Ben: "lets remove python were possible. I trust rust more then python" |

## Decided on 17 November

| Decision | Why | By |
|---|---|---|
| **Gazebo deleted; our simulator is Ravensim** (16 Nov, TODO 184, 183): `simulator:=ravensim`, the only one (`own` works for one release); its chunked maps are RavenTiles. Gazebo's shared nodes and models moved to `aav_sim_models`; its topics are `/vehicle/sim/*`. Without a GPU the cameras draw in software and the clock waits for each picture (camera lockstep). CARLA stays. Last scores: [Gazebo, as it was](history/gazebo-as-it-was.md) | Ravensim passed every `sim-check` leg Gazebo did (8 Nov); Gazebo's last job was the cameras with no GPU, and lockstep does that now | Ben |
| **Nav2 deleted; ours everywhere** (TODO 184): the simulator, the bench and the car run our navigator, planner, costmaps, `mppi_lag`, smoother, stop zone and map server. Kept: `nav2_msgs`, and `nav2_amcl` for the AMCL trial ([Nav2, as it was](history/nav2-as-it-was.md)) | Ben, 16 Nov: "Now". Every Nav2 server had its port, scored on 16 Nov ([the league](league-results-2026-11-16.md)) | Ben |
| **Our stop zone is the stop zone** (`aav_stop_zone`, TODO 182) | Ben, 16 Nov: "shadow run, then swap". In shadow on every obstacle check, 3 rounds, it stopped on the same command all 24 times Nav2's did, never later | Ben |
| **The EKFs take the car's command, to slow only** (robot_localization's control term, TODO 207) | With the wheels quiet the EKFs ran on at their last speed, and `map -> odom` jumped 9-10 m when the wheels came back; `sim-check --faults` 14 of 14, three runs in a row | Ben |

## Decided on 18-19 November

The whole story, to explain to someone else: [What we decided, 18-19 Nov](decisions-2026-11-19.md).

| Decision | Why | By |
|---|---|---|
| **Keep every commit**: the fork is never reset, rebased or force-pushed; protection rules and save tags; real merges only | the team's queue squashed 1,062 of Ben's commits; the fork holds the full history | Ben |
| **The testing plan**, phases 0 and A to G, each automated ([plan](testing-plan.md)) | Ben's prof: "a full automated testing system" | Ben |
| **NDT leads, the GPS checks** (`pose_source:=ndt`, a challenger, TODO 210) | the GPS was failing; with NDT leading the car held its route with the GPS stopped, jumped or drifting | Ben |
| **Maps from our own drives** (KISS-SLAM, offline), cleaned in a map editor | 2.3 cm against the EKF's 33 cm; a person has the last word | Ben |
| **No Nav2 yardstick; scores tracked, not gates** (TODO 167) | "We can get worse if we think there is room to improve"; safety checks stay pass/fail | Ben |
| **One ONNX file per model**: Core ML on the Mac, TensorRT on the Orin (JetPack 7.2+); moved up (TODO 243) | one export for every computer; proven on the Mac before the car | Ben |
| **Lanes are perception; the costmap prices them; navigation drives.** The separate lane-follower drivers (the 2025-26 PID and ours) are removed once the camera's lanes are in the lanes layer (TODO 264). "Every algorithm stays" now means: not too early; strip what we know won't work (Ben, 19 Nov) | Ben, 19 Nov: "drop there code all together". One driver (MPPI) with its lag and the obstacles, not a second steering loop beside it | Ben |

## Proposed in November: Ben's to decide

Designs, **nothing of them built**.

| Design | What it proposes | Waits on |
|---|---|---|
| [One shape](designs/2026-11-13_one-shape.md) (13 Nov; reviewed: go, with changes) | nine levels, one world model, one boss (the behaviour planner), the league (`stack:=`), 36 switches down to 8, lane lines as soft costs | D1-D7 (TODO 147-152) |
| [The whole car](designs/2026-11-14_the-whole-car.md) (14 Nov) | every design merged: one picture of the world, one car model, one scorecard, and what's missing | W1-W7 (TODO 153-160) |
| [The hardware debug plan](designs/2026-11-07_esp32-debug-link_brief.md) (7 Nov) | debug one part of the car at a time; the order chosen by Ben | the hardware people: a USB cable, the current limits, the return wire (TODO 126) |
| Where the CBF safety filter goes (K1), a live shadow lock (S1), reversing on long drives (TODO 122) | see each design | Ben |

## Open questions

The design doc numbers 34 questions, and doesn't rank them. **Bold** marks the ones that
look like they block phase 0 or the first drives: a suggestion, for the team to change.
The design doc's next steps take Q1–Q9, Q18, Q20 and Q22 to the car with a tape measure.

### The car

| Q | Topic | The question |
|---|---|---|
| **Q1** | Dimensions | Wheelbase, track, overall size, steering limit, top speed? The old code disagrees: 1.3–1.57 m wheelbase. **Partly answered** (Betsi, 1 Nov): wheelbase 1.561 m, track 0.65 m front and 0.85 m rear, the lidar's and GPS's heights. The steering limit, the weight and the wheel diameter are still open |
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
| Q16 | Watchdog | How long without a command before it acts, how hard it brakes, and what the steering does. **Built as:** 100 ms, full brake, latched (CAN v0.2.0, 15 Oct; the Rust ESP32's heartbeat, 24 Oct). The steering's part is still open (below) |
| Q17 | Wireless updates | Keep them? If so, how are they authenticated? **Built, off by default** (7 Nov): only while parked, with a password, a checked image; never in the car-day build |

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
| The follower's lookahead (N7; Nav2's then) | its turn command flips sign about once a second in the sim. Still open: "the wiggle" (TODO 124, 149); the single-goal tree's replanning at 1 Hz is the suspect |
| Package renames | the inherited packages' new names, in [Conventions](conventions.md) |
| A `vendor/` folder | done: downloaded source stays out of git, `pixi run get vendor` fetches it again |
| Stop signs as a twist_mux lock | priority 30: it holds navigation and the AI slot, never a person or a test script; 4 s stop, then 10 s to drive past. Built and working on the bench (Proposed) |
| The steering on an e-stop | hold where it is (`ESTOP_STEERING = Hold` in `spine-core`), or centre it (the 2025-26 firmware) |
| The brake after boot | `IDLE_BRAKE = 0.0` lets it go once the e-stop clears; the 2025-26 firmware kept it full on (1.0). ⚠️ Keeping it on may be safer on a slope |
| The RC gear switch's brake position | `RC_GEAR_BRAKE = SpeedZero`: the speed loop brakes to a stop. `RcInput` now has a brake field, so the old full brake is one constant away |
