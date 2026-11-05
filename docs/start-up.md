# How it all fits: bottom-up, and what happens at start-up

Written 15 October 2026, checked 25 October. Read [Sim and the car](sim-and-car.md)
and [Maps and position](maps-and-position.md) first, or after: they go
deeper on two parts of this page.

> **The short version**
> - The car is **layers**. Each one only talks to the layer above and the one below.
> - Every layer can be tested on its own: the **turtle**, the **bench**, **Gazebo**, then the **car**.
> - At start-up, each computer runs **one command**. Its machine file says which parts run there. They find each other on the network by themselves.
> - Nothing moves until a **driver** asks, the **mux** allows it, and the **backend** is **armed**.
> - **None of this has run on the real car yet.** The start-up below is the design, tested in simulation and on the bench.

## 1. The layers, bottom to top

```text
 8  People      HUD screen, laptop (Foxglove, RViz), e-stop buttons
 7  Choose      twist_mux picks ONE driver → ego turns it into speed + steering angle
 6  Drivers     keyboard, joystick, scripted drive, Nav2, AI (lane keeping), replay
 5  Perception  RF-DETR (cameras), radar clusters, lidar scan, lane finder → costmap, stop sign
 4  Where am I  ekf_local, ekf_global, navsat_transform
 3  Backend     ONE of: legacy (UART, today), spine (CAN, later), Gazebo, bench, turtle
 2  Sensors     drivers for camera, radar, lidar, IMU, GPS, and the wheels (CAN)
 1  Boards      ESP32 control board, wheel nodes, Jrk G2s, the GPS and IMU chips
 0  Wires       power, Ethernet, CAN, USB, UART
```

| # | Layer | Talks up with | Runs on (planned) | Test it with |
|---|---|---|---|---|
| 1 | Boards | CAN frames, UART bytes | the boards | unit tests on a laptop (`spine-core`, `wheel-core`); the bench's spine simulator |
| 2 | Sensors | `/sensors/...` topics | the Pi or Orin the device plugs into | Gazebo (all sensors); the car |
| 3 | Backend | `/vehicle/status` up (and `/vehicle/odom` in the stand-ins); `/control/<target>/actuator_cmd` down (from the output mux) | Motor-Pi (today, UART); Orin #1 (the CAN spine, later) | turtle, bench, Gazebo |
| 4 | Where am I | TF `map → odom → base_link`, `/localization/...` | Orin #1 | turtle (local only), bench, Gazebo, replay a bag |
| 5 | Perception | `/perception/...` | Orin #2 (later); Orin #1 or a laptop today | bench (`--perception`), Gazebo road world |
| 6 | Drivers | `/drivers/<name>/cmd_vel` | Orin; laptop for keyboard/joystick | turtle, bench, Gazebo |
| 7 | Choose | `/control/cmd_vel` → `/ego/actuator_cmd` | Orin #1 | turtle (fastest) |
| 8 | People | `/safety/estop`, reads everything | dash Pi (HUD), laptops | `pixi run hud --demo`, bench |

## 2. Each part of the car, from the device up

| Device | Wire | Driver (ours or stock) | Topic it makes | Who reads it | Status |
|---|---|---|---|---|---|
| Front cameras L and R | ribbon, both on one Pi 5 | MediaMTX on the Pi streams H.264; on the Orin, `aav_camera_stream` decodes each, with RF-DETR inside | `/perception/detections` (no raw pictures on the network) | stop-sign watcher, sensor watchdog | ✅ on a Mac; ❌ car |
| mmWave radar (TI) | USB | `aav_radar_ti` (inherited) | `/sensors/radar/points` | radar clusters, Nav2's local costmap, collision_monitor | ✅ bench, Gazebo; ❌ car |
| Lidar (roof Velodyne VLP-16) | Ethernet | the stock `velodyne` driver, on by default on the car | `/sensors/lidar/points` | Nav2's costmap, collision_monitor | ✅ Gazebo, bench, a pretend VLP-16; ❌ the real lidar |
| IMU (Yahboom) | USB | `aav_imu_yahboom` (Rust) + Madgwick filter | `/sensors/imu/data` | both EKFs | ✅ Gazebo; ❌ car |
| GPS (ZED-F9R) | USB to the RTK-Pi | `ublox_gps` (stock) | `/sensors/gps/fix` | navsat_transform, HUD | ✅ Gazebo; ❌ car |
| Wheel speed, today | the hardware lead's Arduino on the Orin's CAN (0x200/0x201) | `spine_bridge role:=sensors` | `/vehicle/wheels`, `/vehicle/odom` | both EKFs | ✅ fed wheel frames by hand; ❌ car. The 2025-26 UDP encoder is off since 23 Oct |
| Steering | Jrk G2 + KTC-100 pot (KEFC) | the control board | `/vehicle/status` (angle) | EKF (later), HUD | ❌ no sensor today (open loop) |
| Brake | Jrk G2 | the control board | `/vehicle/status` (brake %) | HUD | ❌ |
| Throttle | DAC → motor controller | the control board | `/vehicle/status` (throttle %) | HUD | ❌ |

The commands go the other way: `/ego/actuator_cmd` (speed + steering angle) → backend →
board. The board runs the speed loop; each Jrk G2 runs a position loop.

## 3. What happens when the car starts and joins the network

This is the design. It runs this way on the bench and in Gazebo, but never on the car yet.

### Each computer runs one command

```bash
pixi run car orin        # on Orin #1
pixi run car motor_pi    # on the Motor-Pi (today's drive path)
pixi run -e ml car laptop   # on a laptop at the car: joystick, RF-DETR on its GPU
```

Its **machine file** (`ros/bringup/aav_bringup/config/machines/<name>.yaml`) lists the parts that run there:

| Machine | Parts that run there |
|---|---|
| `orin` | car model, mux, ego, scripted, drive to point, IMU, GPS, lidar, the two camera streams (RF-DETR inside), stop-sign watcher, localization, Nav2, the CAN bus (wheels), Foxglove |
| `motor_pi` | the legacy backend: UART to the ESP32 |
| `laptop` | joystick, RF-DETR on the laptop's GPU, compressed camera pictures |
| `all` | everything on one computer (the bench) |

A part that can't run there (no device, no model file, no PyTorch) is **skipped with a warning**, not a crash.

### The start-up, step by step

| # | What happens | Watch it |
|---|---|---|
| 1 | Power on. Each Pi and the Orin boot Linux | ping each one |
| 2 | Each runs its `pixi run car <machine>` (by hand today; at boot later) | its terminal |
| 3 | **Discovery:** every ROS program says "hello" on the network (DDS multicast). Same `ROS_DOMAIN_ID` = same car. In a few seconds every program knows every other | `ros2 node list` |
| 4 | The car model publishes the fixed frames: `base_link → imu_link, gps_link, camera..., lidar_link` | `ros2 run tf2_tools view_frames` |
| 5 | Sensor drivers open their devices and start publishing | `ros2 topic hz /sensors/gps/fix` |
| 6 | `ekf_local` starts as soon as wheel speed and the IMU arrive. `ekf_global` waits for GPS and the datum | `ros2 topic echo /localization/odom/local` |
| 7 | Nav2 starts its servers in order (lifecycle), loads the map, and waits for a goal | `ros2 lifecycle get /bt_navigator` |
| 8 | twist_mux is **silent**: no driver is talking | `ros2 topic hz /control/cmd_vel` = nothing |
| 9 | ego sends "speed 0" 50 times a second, always. The output mux passes it on, with its heartbeat, only once every source is REAL (about 4 s after start) | `ros2 topic echo /safety/output_mux/state` = OPEN |
| 10 | Today's car (`actuators:=uart`) has no arm step: the output mux's OPEN is the gate. The CAN spine (`actuators:=can`) starts **not armed** | `/vehicle/status` armed = false |
| 11 | CAN spine only: a person arms it on purpose, once the gate is OPEN: `ros2 service call /spine/arm std_srvs/srv/SetBool "{data: true}"` | armed = true |
| 12 | Now a driver can move the car: hold the joystick trigger, send a scripted drive, or give Nav2 a goal | the HUD |

### A laptop joining

| # | Do | Why |
|---|---|---|
| 1 | Join the car's WiFi | |
| 2 | `pixi run doctor` | checks the domain ID, discovery and firewall, and says what to fix |
| 3 | Make sure `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` | `pixi run sim` and friends use LOCALHOST on purpose, so a laptop's simulator never meets the car |
| 4 | Watch: Foxglove (port 8765 on the Orin), RViz, `pixi run hud` | |

### What stops it

| Stop | How | Acts on |
|---|---|---|
| Kar-Tech wireless e-stop, kill switch | hardware | power: overrides everything |
| HUD button, `pixi run estop` | `/safety/estop` = true | the mux blocks every driver; the backend stops |
| The computer freezes | ego's 50 Hz stops | the board's timeout brakes: 100 ms in the Rust firmware (the spine, and the Rust ESP32: proven in QEMU, not yet flashed). Today's C++ ESP32 has none (team-todo.md B2) |
| An obstacle | collision_monitor | Nav2's command, before the mux |
| The person drives | any higher-priority driver | the mux picks them over Nav2 |

## 4. How to test each layer, cheapest first

| Rung | Command | Has | Good for |
|---|---|---|---|
| 1. Unit tests | `tools/test.sh` (`pixi run test` adds the ROS tests) | no ROS | maths, CAN frames, firmware logic |
| 2. Turtle | `pixi run turtle` | a 2D turtle as the car | drivers, mux, ego, scripted drive |
| 3. Bench | `pixi run bench`, `pixi run bench-check` | pretend sensors, the real CAN frames, the spine's own logic | the car's software path, e-stops, Nav2, stop sign |
| 4. Gazebo | `pixi run sim world:=road` (or `world:=test_lot`), every sensor on | 3D physics, every sensor | localization, Nav2, perception, A to B |
| 5. Replay | `ros2 bag play` | a recorded drive | tuning the EKFs against real data |
| 6. CARLA | adapter ready, untested | photoreal towns, Carleton's streets | needs the NVIDIA computer (Q31; docs/carla.md) |
| 7. Car on stands | `pixi run car orin` + `motor_pi` | the real boards, wheels in the air | signs, limits, the drive path |
| 8. Car | the same, on the ground | everything | with a safety driver |

### Top-down: "I want to..."

| I want to | Start at |
|---|---|
| see the whole thing move | `pixi run sim world:=road`, then `pixi run keyboard --sim` |
| check that a change broke nothing | `pixi run test`, then `pixi run bench-check` |
| try a new driver | the turtle |
| try perception | `pixi run -e ml sim-check --perception` |
| see what the car sees | `pixi run hud` next to the bench or Gazebo |
| drive A to B | [A to B](a-to-b.md) |
| choose models, or put a map on the HUD | [Perception options](perception-and-hud-map.md) |

## 5. The gaps, in one list

| Gap | Blocks | Next step |
|---|---|---|
| Our code has never run on the car | everything on the car | the audits were 16 and 21 Oct; next, Jazzy or Pixi on the Orin (Q24; [car day](car-day.md)) |
| The camera streams and the lidar driver are untested on the car | cameras and lidar on the car | the car day; RF-DETR on the Orin waits for the reflash (Q24) |
| No steering sensor | closed-loop steering | the KTC-100 (KEFC) |
| Wheel speed: the Arduino's CAN frames never seen yet | good odometry | the car day: `candump can0` on the Orin |
| The datum is a placeholder | GPS goals | measure it at the test site |
| No start-at-boot | a car that's ready when it's switched on | systemd services, once the stack runs on the car |

More detail: [Learn: the car, box by box](learn-car.md), [A command's journey](learn-journey.md),
[Every module](modules.md), [Status](status.md).
