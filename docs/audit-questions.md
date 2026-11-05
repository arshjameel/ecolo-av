# Friday's questions: what only the car can tell us

> **Sections 1-7 are a snapshot from 16 October 2026.** The audits ran on 16 and 21 Oct ([16 Oct](history/audit-2026-10-16-car.md), [21 Oct](history/audit-2026-10-21-car.md)). What's true now: [Status](status.md). **Section 8 (H1-H10) is live:** questions for the hardware lead.

Every hardware question the code is waiting on, in one list for Friday 16 October. Each
row says how to answer it and what the answer unblocks. Write the answers into
`docs/history/audit-2026-10-16-car.md` (no passwords or keys).

**You'll use it with:** [The hardware audit, step by step](audit-friday.md), and
[Friday 16 Oct](history/friday-2026-10-16.md) for the plan of the day.

The ⚠️ rows matter most: they change what we buy, or whether it's safe to drive.

## 1. Safety

| # | Question | How to answer | Unblocks |
|---|---|---|---|
| S1 ⚠️ | What does the Kar-Tech wireless e-stop cut: drive power, the 12 V rail, or everything? And the red mushroom button? (Q6) | press each with the car on stands, watch the boards' LEDs and the motor | the safe state; the e-stop input pin for the Rust firmware (it has no pin yet) |
| S2 ⚠️ | What does the brake do when its power is cut: hold, or release? (Q3) | on stands: brake applied, then cut the Jrk's power | the safe state: "stopped, not just unpowered" |
| S3 ⚠️ | Today's ESP32 firmware has **no command timeout**. Does anything else stop the car if the Motor-Pi freezes? | read the Motor-Pi's software; ask the team | whether driving through the legacy path is safe at all |
| S4 | Is the ESP32's web update page reachable from the car's WiFi? | from a laptop on the WiFi: the ESP32's IP in a browser | close it (the Rust firmware has none) |
| S5 | Who is the safety driver, and what is the speed cap for the first drive? (Q32) | the team decides | the test plan |

## 2. The drive path (today)

| # | Question | How to answer | Unblocks |
|---|---|---|---|
| D1 | Which commit of the ESP32 firmware is flashed? | the version line on the USB log, or the 2025-26 team | is our `aav_legacy` speaking the same protocol? |
| D2 | What runs on the Motor-Pi, and how does it start (systemd, cron, by hand)? (answered 21 Oct: the Pi called CORTEX, .120, runs `aav_serial_bridge` as a service) | `systemctl list-units`, `crontab -l`, `ps aux` | stopping it cleanly, and starting it again |
| D3 | Which serial port goes to the ESP32? Our bridge assumes `/dev/ttyAMA0` at 115200 | `ls -l /dev/serial*`, the Motor-Pi's own code | `aav_legacy` on the Motor-Pi |
| D4 | Is the Motor-Pi's IP `192.168.1.193`? The wheel-encoder Pico sends there, port 10111 | `ip -br addr` on the Motor-Pi | wheel speed reaching `aav_legacy` |
| D5 | Is the Motor-Pi's OS 64-bit? | `uname -m` says `aarch64` | our Pixi environment (needs 64-bit) |
| D6 | The Jrk G2s' settings: input mode, limits, feedback, timeout | `jrk2cmd --status` and `--get-settings` over USB, if the Pololu tools are there | the steering and brake mapping (Q2); the "Jrk command timeout" second layer |
| D7 | How many wires does each actuator have? 5-6 = built-in position feedback | look, photograph | whether the KTC-100 pot is needed (KEFC) |
| D8 | Which motor controller, and its maximum current? Does it brake on its own? Is reverse safe while rolling? (Q4) | its label, its manual | the battery shunt (KEFC); the throttle safe state |

## 3. CAN (the KEFC plan)

| # | Question | How to answer | Unblocks |
|---|---|---|---|
| C1 ⚠️ | Is the CAN chip U7 fitted, and is it wired to the ESP32? SparkFun's MicroMod ESP32 schematic shows the **CAN pads not connected** | look at U7; trace (or beep) U7's TX/RX pins to the MicroMod connector | **whether the KEFC plan works with this board**: decide before ordering |
| C2 | Are the front brake rotors steel, and how many holes? | a magnet; count | the Hall sensors; the slowest speed we can read (8 holes ≈ 0.24 m/s) |
| C3 | Is there room and 12 V near each front wheel for a wheel node? | look | the wheel nodes' mounting |

## 4. Sensors

| # | Question | How to answer | Unblocks |
|---|---|---|---|
| N1 | GPS: is the ZED-F9R on the RTK-Pi's USB? Does it give fixes now? | `ls /dev/ttyACM*`; the 2025-26 GPS node's topic; `ubxtool` if installed | "is there GPS data?"; our `ublox_gps` driver (Q18) |
| N2 | Where do RTK corrections come from: an NTRIP account, a base station, or none? | the RTK-Pi's config; ask | centimetre GPS; the KEFC base-station line |
| N3 | The IMU: which computer is it plugged into? How is it turned on the car? | follow the cable; photograph | `orin.yaml`'s TODO; `vehicle.yaml`'s IMU angles |
| N4 | The cameras: which model? Which Pi each? CSI ribbon (looks like it) or USB? (answered in part 21 Oct: MediaMTX on the camera Pis, no ROS) | the Pis' `libcamera-hello --list-cameras`, `v4l2-ctl --list-devices` | the camera stream: MediaMTX on the Pi, `aav_camera_stream` on the Orin (decided 23 Oct) |
| N5 | Is the rear camera (orange housing) connected to anything? | follow the ribbon | the KEFC's Cam3 line |
| N6 ⚠️ | The lidar: make, model, IP, and which computer reads it (answered 16 Oct: a Velodyne VLP-16 at .201, read by the Jetson; we run the stock `velodyne` driver) | the label on its base; the router's client list; `ping` | a driver: Velodyne, Ouster and RoboSense drivers are in Pixi for the car; Hesai's isn't |
| N7 | The radar: is a TI mmWave radar on the car? Where, and on which USB? | look; `ls /dev/ttyUSB*` | `aav_radar_ti` |
| N8 | The battery monitor (KM110F): any wired data port? | look for a port; its manual | drop the KEFC battery lines, or read it in ROS |
| N9 | The steering's centre: with the wheels straight, what does the Jrk read? | `jrk2cmd --status` | steering offset; drift in the scripted drive |

## 5. Computers and network (we look at this together)

| # | Question | How to answer | Unblocks |
|---|---|---|---|
| W1 | The subnet, and each machine's IP and MAC | `pixi run bash tools/net_map.sh` on a laptop; the router's client list; `ip -br addr` on each | a network map; `vehicle` machine files |
| W2 | Which `ROS_DOMAIN_ID` does the 2025-26 software use? (answered 21 Oct: 0, Humble) | `pixi run bash tools/net_map.sh` (the ROS column); `printenv | grep ROS` on each | picking ours so the two never mix (our kit uses 42) |
| W3 ⚠️ | Does multicast pass between the machines? ROS 2 finds its peers with it | `ros2 multicast receive` on one, `ros2 multicast send` on another, both ways, and over WiFi | ROS working across machines at all |
| W4 | Is IGMP snooping on in the TP-Link switch? It can drop ROS's multicast | the switch's web page | W3 |
| W5 | Does the car's network reach the internet? | `ping 1.1.1.1` from the Orin | online install, or the offline kit |
| W6 | Are the clocks in sync? | `date` on each, side by side; `chronyc tracking` | TF and the EKFs reject messages from a wrong clock |
| W7 | The Orin: JetPack version, module part number, free disk (answered 21 Oct: AGX Orin dev kit, JetPack 6, L4T R36.4.7) | `cat /etc/nv_tegra_release`; the label; `df -h` | the reflash plan (Q24); the kit (13 GB free to install) |
| W8 | Free disk and RAM on each Pi | `df -h`, `free -h` | our environment on each one |
| W9 | What are ES-PI and QNX-PI, and are they in the driving path? | log in; `systemctl list-units`; ask | the network map; the start-up order |
| W10 | Which computer drives the dash's two screens? Their output names, resolutions, X11 or Wayland? | `xrandr --listmonitors`, `wlr-randr` or `kmsprint`; `echo $XDG_SESSION_TYPE` | the monitors in `screens.env`, for the HUD and the map at boot (docs/car-screens.md) |

## 6. Measurements for `vehicle.yaml` (Q1)

Every number in `ros/bringup/aav_description/config/vehicle.yaml` is a guess. Bring a tape measure.

| # | Measure | Today's guess |
|---|---|---|
| M1 | wheelbase: rear axle to front axle | 1.57 m |
| M2 | track: left to right wheel centres | 0.77 m |
| M3 | wheel diameter, and width | 0.30 m, 0.10 m |
| M4 | length, width, height; rear bumper to rear axle | 2.0, 1.0, 1.5 m; 0.20 m |
| M5 | the steering limit: wheel angle at full lock, each way | 0.50 rad (about 29°) |
| M6 | the GPS antenna, IMU, radar, each camera and the lidar: x forward, y left, z up from the ground under the rear axle | see `vehicle.yaml` |
| M7 | mass: 4 bathroom scales, one under each wheel, add them up (or one scale, a wheel at a time, the others on blocks of the same height) | 181 kg, Ben's estimate (about 400 lb) |

How to measure each, as a sheet to fill in: [Measure the car](measure-the-car.md).

## 7. Power

| # | Question | How to answer | Unblocks |
|---|---|---|---|
| P1 | Each 12 V → 5 V converter: its rating, and what it feeds. The dash one showed 3.48 A | the label; follow the wires | a Pi 5 wants 5 A; brown-outs look like software crashes |
| P2 | What powers the Orin, and from which fuse? | follow the cable | the KEFC power plan |

## 8. The car's wiring, for the hardware lead (23 Oct)

Ben's picture of the wiring (23 Oct), and what our code now assumes:
[Today's wiring](can-plan.md#todays-wiring-23-october). **Not confirmed**: the hardware
lead decides, and these answers may change it.

| # | Question | Why it matters |
|---|---|---|
| H1 ⚠️ | Is the Motor Pi to ESP32 link serial (UART)? Which port, which speed? Is it the Pi called CORTEX (.120), which ran `uart_bridge` on 21 Oct? | `legacy_uart_bridge` assumes `/dev/ttyAMA0` at 115200 (D3); `motor_pi.yaml` |
| H2 | Does the ESP32 read the steering angle? And the brake position? How (the Jrk G2s' feedback, a pot)? | `/vehicle/steering`'s measured angle, a better `/vehicle/odom` in turns (a yaw rate), and checking the actuators. The Rust firmware reads both Jrks' scaled feedback, and flags a Jrk with no feedback set up as unknown; the uplink (H9) sends it up. D6's `--get-settings` shows each Jrk's feedback mode |
| H3 | What else is on the CAN bus, besides the wheel sensors? Their IDs and rates? | `aav.dbc`: new frames, and the bus load ([Bandwidth](bandwidth.md)). Their IDs must not clash with 0x010-0x201 |
| H4 | The wheel sensors' Arduino: which board and CAN chip? One Arduino for both wheels, or one each? Does it send `aav.dbc` 0x200 and 0x201 as they are (CRC-8, counter, 100 Hz)? | the Orin decodes only those frames; a wrong checksum is dropped and counted |
| H5 | Can the wheel sensors tell direction (two sensors per wheel, quadrature)? | `direction_known`: today the Orin signs the speed from the last command, which is wrong when the car is pushed or rolls back |
| H6 | Is the 2025-26 wheel encoder (the Pico 2, UDP to the Motor Pi) still on the car? | it is off in `motor_pi.yaml`; turn it on only where the CAN wheel sensors aren't |
| H7 | The CAN bus: 500 kbit/s? Terminated at both ends? Which interface on the Orin (`can0`, the 40-pin header's CAN, a USB adapter)? | `spine.launch.py interface:=`, and `ip link set can0 up type can bitrate 500000` |
| H8 | Will the ESP32 ever move onto CAN (the KEFC plan), or stay on the Motor Pi? | `actuators:=can` stays ready for it; until then `uart` |
| H9 ⚠️ | Is the ESP32's **TX1 (GPIO17, MicroMod pad 17)** wired to the Motor Pi's RX (`/dev/ttyAMA0`'s RX, GPIO15 on a Pi)? The 2025-26 C++ set it up as TX and never sent anything, so the wire may be missing. 3.3 V both sides, a common ground? | the **uplink** (TODO 50, 48): mode, faults, the counter's echo, the Jrks' positions, the RC sticks. Built and tested in the emulator, but off (`--features uplink`, `uplink: true` in `legacy.yaml`) until this is yes ([the bench](esp32-bench.md#the-uplink-and-the-rc-modes-on-the-bench)) |
| H10 | The RC transmitter: is CH6 (the 2025-26 `RC_EN_CH`, index 5) a two- or a three-position switch? If two, can a three-position switch be put on CH6 in the aux-channel menu, or is another channel free (CH7-CH10)? What are the receiver's failsafe values? (Q7) | the proposed three modes: low computer, middle **RC via ROS**, high RC direct ([decisions](decisions.md), Proposed). The channel and its thresholds are one line in the firmware (`ibus::RcMap::THREE_POSITION`) |
