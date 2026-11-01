# Glossary

Every acronym and term in these pages, in plain words, from A to Z.

**You'll learn:**
- what each word means, in one line;
- where on the car or in the code you meet it.

Can't find a word? Search this page (Ctrl+F or ⌘F). Missing one? Add a row, in order.

## Numbers and A to C

| Term | Plain words | Where you meet it |
|---|---|---|
| 120 Ω terminator | a resistor at each end of a CAN bus; without both, frames get garbled | the CAN Pal and the last wheel node |
| A* | a classic way to find the shortest path on a grid | the 2025-26 planner, now replaced by Nav2 |
| AAV | Accessible Autonomous Vehicle: Carleton's capstone car | everywhere |
| Ackermann | how a car steers: the front wheels turn, the back ones don't, so it can't spin on the spot | ego's output |
| `AckermannDriveStamped` | a ROS message: a speed and a steering angle, with a time stamp | `/ego/actuator_cmd` |
| action | a ROS request that takes a while and reports progress, like "drive to this point" | `/navigate_to_pose`, `/scripted_drive` |
| actuator | a motor that moves something: the steering, the brake | the moving parts |
| area | the first part of a topic name: `/sensors/`, `/drivers/`, `/control/`... | [Topic areas](naming.md) |
| arm | to tell the spine it may drive; it ignores drive frames until armed | the service `/spine/arm` |
| backend | the one program that plays the car: the real car, Gazebo or the turtle | `ros/can_bridge/` and `ros/sim/` |
| bag | a recording of ROS topics, to play back later | `tools/record.sh`, `bags/` |
| `base_link` | the car's own frame: on the ground, under the middle of the rear axle | TF |
| bench mode | the car's real software on one computer, with pretend boards and sensors | `pixi run bench`, [Bench mode](bench.md) |
| bicycle model | the maths that turns a speed and turn rate into a steering angle | ego, `ackermann.py` |
| bringup | the launch files that start a whole profile | `ros/bringup/aav_bringup` |
| `bt_navigator` | Nav2's boss: runs a behaviour tree that plans, follows and recovers | Nav2 |
| CAN | Controller Area Network: two twisted wires made for cars; short messages that arrive on time | the KEFC bus |
| CAN Pal | a small CAN transceiver board on the Orin's 40-pin header | Orin #1 (KEFC) |
| CANable | a USB-to-CAN adapter for a laptop, to watch or test a bus | the bench test of CAN |
| `candump` | a Linux command that prints every frame on a CAN bus | `candump can0` |
| CARLA | a photo-real driving simulator, on Unreal Engine; next to Gazebo, for the cameras (Q31, [CARLA](carla.md)) | `ros/sim/aav_carla` |
| checksum, CRC | a few bits worked out from a message, so a garbled one is caught | every CAN frame (CRC-8) |
| CI | Continuous Integration: GitHub runs the tests on every pull request and push to `main` | `.github/workflows/test.yml` |
| colcon | the tool that builds a ROS workspace | `pixi run build` |
| `collision_monitor` | a Nav2 node that sends zero speed when something enters its stop zone | `/safety/collision_monitor/state` |
| costmap | a grid on the ground: how bad it is to drive in each cell | [Costmaps](costmaps.md) |
| counter | a number that goes up by one in every frame, so a frozen sender is caught | every CAN frame |

## D to H

| Term | Plain words | Where you meet it |
|---|---|---|
| D* Lite | a path planner that repairs its plan as the map changes | the 2025-26 planner, now replaced |
| DAC | Digital-to-Analog Converter: turns a number into a voltage | the throttle |
| datum | the one GPS point where the map's (0, 0) sits | `navsat.yaml` |
| DBC | a text file that lists every CAN frame and signal | `firmware/can/aav.dbc` |
| DDS | the network layer under ROS 2 that finds nodes and carries messages | `ROS_DOMAIN_ID` |
| deadman | a button you must hold to drive; let go and the car stops | the joystick's trigger |
| DIN rail | a metal rail that boxes clip onto | the Pis at the windscreen |
| driver (ROS) | anything that proposes a command: joystick, keyboard, Nav2... | `/drivers/<name>/cmd_vel` |
| driver (device) | the code that talks to one piece of hardware | `ros/sources/` |
| e-stop | emergency stop | `/safety/estop`, the red button, the Kar-Tech remote |
| ego | our node that turns the chosen twist into a speed and steering angle, within limits | `ros/ego/aav_ego` |
| EKF | Extended Kalman Filter: mixes noisy sensors into one best guess of position | `ekf_local`, `ekf_global` |
| Embassy | a Rust framework for chips with no operating system | the spine firmware |
| `embedded-hal` | a Rust standard way to talk to pins, I2C and SPI on any chip | `firmware/spine` |
| encoder | a sensor that counts wheel turns | the Pico 2 today; Hall sensors later |
| ESP32 | a small, cheap microcontroller with WiFi | the control board |
| ESP32-CAN-X2 | an ESP32-S3 board with CAN built in | proposed for the wheel nodes (KEFC); today the hardware lead's Arduino does that job |
| Ethernet | the network cable a laptop uses: lots of data, many computers | the switch |
| firmware | the program on a microcontroller | `firmware/` |
| FlySky | the RC transmitter and receiver a person can take over with | iBUS on the ESP32 |
| Foxglove | a web app to watch ROS topics, like RViz | `foxglove_bridge` |
| frame (CAN) | one CAN message: an ID and up to 8 bytes | 0x100 `DRIVE_CMD` |
| frame (TF) | a coordinate system, like `map` or `base_link` | TF |
| Gazebo | a 3D physics simulator | `pixi run sim` |
| GNSS | Global Navigation Satellite System: GPS and its cousins | the roof antenna |
| GPS | satellite position: latitude and longitude | `/sensors/gps/fix` |
| Hall sensor | a sensor that notices metal or a magnet passing | the front brake rotors (KEFC) |
| HUD | Head-Up Display: the in-car screen (and, on the big monitor, the map window) | [HUD](hud.md) |
| Humble | the ROS 2 version on the car today; support ends May 2027 | the car |

## I to N

| Term | Plain words | Where you meet it |
|---|---|---|
| I2C | a two-wire bus between chips on one board | the ESP32 to the Jrks and the DAC |
| iBUS | the FlySky receiver's serial format | the ESP32 |
| IMU | Inertial Measurement Unit: measures turning and acceleration | the Yahboom, `/sensors/imu/data` |
| inflation | a soft band of cost around each obstacle in a costmap | Nav2 |
| Jazzy | the ROS 2 version we use; support ends May 2029 | everywhere |
| JetPack | NVIDIA's Ubuntu for the Jetson boards | the Orin (Q24) |
| Jetson AGX Orin | NVIDIA's small computer with a big GPU | Orin #1, Orin #2 |
| Jrk G2 | a Pololu motor driver with feedback: "go to position X" | steering, brake |
| Kar-Tech | the wireless e-stop on the car | the safety layer |
| KEFC request | Ben's October 2026 hardware request: the CAN bus, the wheel nodes, a second Orin, a rear camera | [CAN plan](can-plan.md) |
| KTC-100 | a linear position sensor (a potentiometer) | the steering, new (KEFC) |
| launch file | a Python file that starts many nodes with their settings | `ros/*/*/launch/` |
| lidar | a laser scanner: 3D points around the car | the roof Velodyne VLP-16, stock `velodyne` driver, `/sensors/lidar/points` |
| lock (twist_mux) | a `true`/`false` topic that blocks lower drivers | `/safety/estop`, `/perception/stop_sign` |
| Madgwick | a filter that turns raw IMU data into an orientation | `imu_filter_madgwick` |
| `map` | the fixed frame on the earth, pinned to the datum | TF |
| MCAP | the file format of our bags | `tools/record.sh` |
| MCP | Model Context Protocol: lets an AI app read the project | [AI apps (MCP)](mcp.md) |
| micro-ROS | ROS on a microcontroller; we don't use it for now | `CLAUDE.md` |
| microcontroller | a small chip that runs one program, no operating system | ESP32, Pico 2 |
| mock | a pretend sensor or board, for tests | `mock_imu`, `spine_sim` |
| Motor-Pi | the Raspberry Pi 4 that drives the ESP32 today, over UART | the 2025-26 path |
| MPPI | Model-predictive path integral: Nav2's controller on our car (since 22 Oct). It tries thousands of futures through a model of the car. `controller:=rpp` for Regulated Pure Pursuit instead | [path followers](path-followers.md) |
| Nav2 | ROS 2's navigation stack: plans a path and follows it | `ros/drivers/aav_navigation` |
| `NavSatFix` | a ROS message: latitude, longitude, altitude | `/sensors/gps/fix` |
| `navsat_transform` | turns GPS fixes into map x and y, and back | `/localization/gps/odom` |
| NMEA | a plain-text GPS format | `gps_driver:=nmea` |
| node | one ROS program with one job | `ros2 node list` |

## O to S

| Term | Plain words | Where you meet it |
|---|---|---|
| `odom` | the frame where the car started today: smooth, drifts | TF |
| odometry | how far and how fast the car moved, from its own sensors | `/vehicle/odom` |
| open loop | sending a command with no measurement back | today's throttle |
| OSM | OpenStreetMap: a free map of the world | the map window, street routes, CARLA's campus |
| OSRM | a road router that uses OSM | the 2025-26 HUD's route (archived 25 Oct); `street_router` can ask one too |
| Pico 2 | a Raspberry Pi microcontroller board (RP2350) | the wheel encoder; a spine program |
| Pixi | installs ROS 2 and every tool without admin rights | `pixi run ...` |
| profile | a set of nodes to start: turtle, sim, bench, car, replay, carla | `ros/bringup/aav_bringup/launch/` |
| `PROVENANCE.md` | a file that says where inherited code came from | each inherited package |
| Q1–Q34 | the team's numbered open questions | [Decisions](decisions.md) |
| QNX | a real-time operating system the 2025-26 team used; replaced by twist_mux | the QNX-Pi |
| QoS | Quality of Service: how a topic delivers (reliable or best effort) | `docs/interfaces.yaml` |
| radar | radio waves that find objects ahead | the TI radar, `/sensors/radar/points` |
| raylib | a small graphics library; our Rust HUD uses it | `ros/hud/aav_hud` |
| RC | Radio Control: a hand-held transmitter | the FlySky override |
| RF-DETR | a neural network that finds objects in a picture; RF-DETR Nano is our default detector | `aav_detector`, `aav_camera_stream` |
| Regulated Pure Pursuit | a Nav2 controller: steers toward a point ahead on the path. Ours until 22 Oct; `controller:=rpp` | [path followers](path-followers.md) |
| REP-103, REP-105 | ROS's rules for units and frames | `CLAUDE.md` |
| RoboStack | ROS 2 packaged for Pixi and conda | `pixi.toml` |
| ROS 2 | Robot Operating System: programs (nodes) that talk over topics | everywhere |
| `ROS_DOMAIN_ID` | a number, 0–101 (safe on Linux; Fast DDS breaks above 232); only computers with the same one see each other | `pixi run doctor` |
| RTK | a GPS correction that gives centimetre accuracy | the RTK-Pi |
| rqt_graph | a live picture of nodes and topics | `pixi run graph` |
| Rust | the language for our firmware and hardware drivers | `firmware/spine` |
| RViz | ROS's 3D viewer | `pixi run rviz` |
| service | a ROS request with one quick answer | `/spine/arm`, `/fromLL` |
| SmacPlannerHybrid | Nav2's planner that only plans turns a car can make | `nav2.yaml` |
| SocketCAN | Linux's way to use a CAN bus like a network | `can0`, `ros2_socketcan` |
| spine | the board that turns CAN commands into steering, brake and throttle | `firmware/spine`, `aav_spine` |
| stamped | a message with a time stamp and frame | `TwistStamped` |
| switch | joins many Ethernet cables into one network | TP-Link TL-SG116E |

## T to Z

| Term | Plain words | Where you meet it |
|---|---|---|
| TF | the tree of frames, and how they move against each other | `/tf`, `map → odom → base_link` |
| topic | a named channel; nodes publish to it and subscribe to it | `ros2 topic list` |
| transceiver | the chip between a microcontroller and the CAN wires | U7 on the control board |
| turtlesim | ROS's teaching turtle; our cheapest stand-in for the car | `pixi run turtle` |
| twist | a speed (m/s) and a turn rate (rad/s) | every driver |
| `TwistStamped` | a twist with a time stamp | `/drivers/*/cmd_vel`, `/control/cmd_vel` |
| `twist_mux` | picks one command from all the drivers, by priority | `ros/command_mux/aav_command_mux` |
| UART | a simple serial line between two chips | the Motor-Pi to the ESP32 |
| u-blox ZED-F9R | the GPS chip on the car | `ublox_gps` |
| UDP | a quick network message with no delivery check | the wheel encoder today |
| URDF | a file that describes the car's shape and where each sensor sits | `ros/bringup/aav_description/urdf` |
| USB | the plug on any computer | the IMU, the GPS board |
| `vehicle.yaml` | the only file with the car's measurements | `ros/bringup/aav_description/config` |
| W5500 | an Ethernet chip for small boards | the wheel encoder today |
| watchdog | a timer that acts if a program stops checking in | the spine, the ESP32 |
| wheel node | a small CAN board that reads one wheel's speed | 0x200 left, 0x201 right (KEFC) |
| YOLO | a fast neural network that finds objects in a picture | `aav_detector` with `detector:=yolo` |

## Try it

```bash
pixi run turtle    # terminal 1: start something to look at
pixi run graph     # terminal 2: nodes and topics, as a picture
pixi run tools/ros.sh --local ros2 topic list    # every topic
pixi run tools/ros.sh --local ros2 node list     # every node
```

## Where this lives in the code

| Word family | Where |
|---|---|
| Topics, services, actions, frames | `docs/interfaces.yaml` |
| CAN frames and signals | `firmware/can/aav.dbc` |
| The car's parts and sizes | `ros/bringup/aav_description/config/vehicle.yaml` |
| Decisions and Q numbers | `docs/decisions.md` |
