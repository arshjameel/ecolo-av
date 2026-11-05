# Bandwidth: how much data each sensor sends (23 October)

Ben asked (23 Oct): "Do we need to think about the bandwidth of all the sensors like in
hertz." **Yes.** This page is the budget: every data stream, how big, which wire it
crosses, and whether that wire has room.

!!! tip "The answer in one line"
    **The cameras are the only danger.** One raw 1080p camera (93 MB/s) fills most of a
    gigabit cable; two don't fit. Everything else together is under 2 MB/s. Send
    pictures **compressed** over the network, and keep the lidar's points on the Orin.

How sure each number is:

| Mark | Means |
|---|---|
| **measured** | `pixi run measure bandwidth` in Gazebo on Ben's Mac, 23 Oct |
| **config** | from our launch files or `firmware/can/aav.dbc` |
| **audit** | seen on the car, at the 16 Oct and [21 Oct](history/audit-2026-10-21-car.md) audits |
| **guess** | worked out, not seen. Measure it on the car |

## The car's network

```text
  laptops ~~~ WiFi (802.11n: ~20-50 Mbit/s real, shared) ~~~ router
                                                               |
  CAM1-PI (Pi camera) ----1G--+                                | 1G
  CAM2-PI (USB camera) ---1G--+                                |
  LIDAR (VLP-16) -------100M--+---- TP-Link TL-SG116E ---------+
  LOCALIZATION-PI --------1G--+     16 ports x 1 Gbit/s
  DASH (HUD screen) ------1G--+                                |
  MOTOR-PI ---------------1G--+                                | 1G
                                                             ORIN
       IMU, GPS, radar: USB serial into a Pi (or the Orin)     |
                                                               | CAN 500 kbit/s
                                          spine (ESP32), wheel-sensor Arduino
```

1G = gigabit Ethernet: 125 MB/s each way, about 110 MB/s real (the VLP-16's own port is
100 Mbit/s). The switch can carry every port at full speed at once, so the limit is
**each cable**, mostly the Orin's.

## The budget: one row per stream

On the car, as planned (Q22 picks the cameras; the rates are ours or the 21 Oct audit's).

| Stream | Topic | Hz | Size | MB/s | Mbit/s | Made on | Crosses | Read by | How sure |
|---|---|---:|---:|---:|---:|---|---|---|---|
| Front camera, **raw** 1920x1080 RGB | `/sensors/camera/front_left/image_raw` | 15 | 6.2 MB | **93** | **746** | its camera's computer | Ethernet, if a Pi runs the camera | detector (Orin), HUD (dash), lane finder | config (`cameras.launch.py`) |
| The same, **JPEG** | `.../image_raw/compressed` | 15 | ~300 kB | 4.5 | 36 | the same | Ethernet; WiFi to a laptop's detector | laptop detector, viewers | guess (JPEG size) |
| Second front camera | `/sensors/camera/front_right/...` | 15 | as above | 93 raw, 4.5 JPEG | 746, 36 | | | nothing yet | config |
| Rear camera | `/sensors/camera/rear/...` | 15? | as above? | 93 raw, 4.5 JPEG | | | | camera_obstacles (later) | guess (model unknown) |
| **All three, raw** | | | | **280** | **2,240** | | | | |
| Ben's example: 1280x720 RGB, 30 Hz | | 30 | 2.8 MB | 83 | 664 | | | | arithmetic |
| Today's CAM2-PI stream (not ROS) | RTSP `cam1` | 30 | H.264 | 0.06 | 0.5 | CAM2-PI | Ethernet | nothing in ROS | audit (16 Oct) |
| Lidar packets (VLP-16) | UDP 2368, then `/sensors/lidar/packets` | 754 pkt/s; 10 msg/s | 1.2 kB a packet | 0.94 | 7.5 | the lidar | Ethernet, lidar to Orin | lidar_driver | Velodyne's data sheet |
| Lidar points | `/sensors/lidar/points` | 10 | ~0.7 MB | ~7 | ~56 | Orin | stays on the Orin | lidar_scan | guess (~30,000 points x 22 B) |
| Lidar, flattened for Nav2 | `/sensors/lidar/scan` | 10 | 2.9 kB | 0.03 | 0.2 | Orin | stays | costmap, collision_monitor | measured |
| Radar (TI IWR6843AOP) | `/sensors/radar/points` | 10 | <5 kB | <0.05 | <0.4 | the Pi it's plugged into | USB serial, then Ethernet | radar_clusters, Nav2 | guess (its serial port tops out at 0.09 MB/s) |
| IMU (Yahboom) | `/sensors/imu/data_raw`, `/data` | 200 | 324 B | 0.13 (both) | 1.0 | LOCALIZATION-PI today | USB serial, then Ethernet | the filter, the EKFs | audit (rate), measured (size) |
| GPS (ZED-F9R) | `/sensors/gps/fix` | 10 | 128 B | 0.001 | 0.01 | LOCALIZATION-PI today | USB, then Ethernet | navsat_transform | audit (rate), measured (size) |
| Wheel speeds | CAN 0x200, 0x201, then `/vehicle/wheels` | 100 each | 8 B a frame; 48 B | 0.005 | 0.04 | the wheel Arduino | CAN, then stays on the Orin | the EKFs, the watchdog | config (DBC) |
| Drive command | `/ego/actuator_cmd`, then CAN 0x100 | 50 | 48 B; 8 B | 0.002 | 0.02 | Orin | CAN (Ethernet to the Motor Pi today) | the spine | config, measured |
| Twist command | `/control/cmd_vel` | 20 | 76 B | 0.002 | 0.01 | Orin | stays | ego | measured |
| Odometry, both EKFs | `/vehicle/odom`, `/localization/odom/*` | 50, 30, 30 | 724 B | 0.08 | 0.6 | Orin | stays | Nav2, the HUD's map | measured |
| TF | `/tf` | 75 | 243 B | 0.02 | 0.15 | Orin | to every viewer | everyone | measured |
| Local costmap | `/local_costmap/costmap` | 2 | 40 kB | 0.08 | 0.6 | Orin | only to a viewer | RViz, Foxglove | measured (200 x 200 cells) |
| Global costmap | `/global_costmap/costmap` | 1 | 640 kB | 0.64 | 5.1 | Orin | only to a viewer | RViz, Foxglove | measured (Gazebo's road map; grows with the map) |
| Nav2's plan | `/plan` | 1 | 4-7 kB | 0.007 | 0.06 | Orin | to the map screen | map window | measured |
| Foxglove to a laptop | WebSocket, port 8765 (8766 while the 2025-26 bridge holds 8765) | | | **what you open** | | Orin | **WiFi** | Lichtblick, Foxglove | measured (Gazebo, 24 Oct): our three layouts at once, 0.17 MB/s ([Watch from a laptop](foxglove.md)) |

Without the cameras, the whole car is about **2 MB/s (16 Mbit/s)**, the lidar's packets half
of it.

## CAN: the bus has plenty of room

From `firmware/can/aav.dbc` v0.2.0, at its 500 kbit/s, counting the worst-case bit
stuffing (an 8-byte frame is 111 bits, up to 135 with stuffing). `pixi run python3 tools/can_load.py` works
it out from the DBC, so it stays right when the contract changes:

| ID | Frame | From | Bytes | Hz | Bits each | kbit/s | Bus % |
|---|---|---|---:|---:|---:|---:|---:|
| 0x010 | ESTOP_CMD | ORIN | 2 | 10 | 75 | 0.8 | 0.1 |
| 0x100 | DRIVE_CMD | ORIN | 8 | 50 | 135 | 6.8 | 1.4 |
| 0x180 | VEHICLE_FEEDBACK | SPINE | 8 | 50 | 135 | 6.8 | 1.4 |
| 0x181 | VEHICLE_STATUS | SPINE | 8 | 10 | 135 | 1.4 | 0.3 |
| 0x200 | WHEEL_SPEED_L | WHEEL_L | 8 | 100 | 135 | 13.5 | 2.7 |
| 0x201 | WHEEL_SPEED_R | WHEEL_R | 8 | 100 | 135 | 13.5 | 2.7 |
| | **TOTAL** | | | **320** | | **42.6** | **8.5** |

| Question | Answer |
|---|---|
| How busy? | **8.5%** at worst; about 7% without stuffing |
| The rule of thumb | keep a CAN bus under 30-50%, so low-priority frames don't wait |
| Room left | about **800 more 8-byte frames a second** before 30% |
| The wheel sensors | Ben, 23 Oct: an Arduino sends 0x200 and 0x201 to the Orin. Their 200 frames a second are the biggest share (5.4%), and fine. Even 500 Hz each would only reach about 30% |
| The Orin's side | SocketCAN turns each frame into a ROS message on `/can/rx`: 260 a second with the CAN spine (200 with today's wiring: the wheels only), about 40 B each, 0.01 MB/s. Small: the Rust `spine_bridge` (the default) decodes them |

## The links, one by one

| Link | Its limit | Planned load | Verdict |
|---|---|---|---|
| **Orin's Ethernet port** | 1 Gbit/s, ~110 MB/s | lidar 0.94 + 3 JPEG cameras 13.5 + the rest 0.5 = **~15 MB/s (14%)** | ✅ with JPEG. ❌ with raw pictures from the Pis: 2 cameras = 187 MB/s, **170%** |
| A camera Pi's port | 1 Gbit/s | one raw 1080p camera: 93 MB/s (**85%**) | ❌ raw: no room for bursts. ✅ JPEG: 4% |
| The dash's port | 1 Gbit/s | the raylib HUD reads the **raw** front camera: 93 MB/s | ❌ today's HUD code, and a Pi would struggle to draw that many bytes. Give it a small or compressed picture |
| A Pi's **USB 2.0** | 480 Mbit/s, ~35-40 MB/s real | a USB camera at 1080p, 15 Hz: 62 MB/s uncompressed (YUYV) | ❌ uncompressed. ✅ as MJPEG, which is why `cameras.launch.py` asks for MJPEG |
| A Pi 5's USB 3.0 | 5 Gbit/s | | ✅ room for a raw camera |
| USB serial (IMU, GPS, radar) | at most ~0.09 MB/s each, at 921,600 baud | under 0.2 MB/s together | ✅ |
| **WiFi** to laptops | 802.11n (the router: a Linksys EA3500 in the photos): 300-450 Mbit/s on paper, **~20-50 Mbit/s real**, shared, less in a metal car | what each laptop opens | ⚠️ see below |
| The lidar's port | 100 Mbit/s | 7.5 Mbit/s | ✅ 7.5% |
| CAN | 500 kbit/s | 42.6 kbit/s | ✅ 8.5% |
| Inside one computer | memory, several GB/s | raw cameras into the detector on the Orin: 93 MB/s each | ✅ but CPU: see Cyclone below |

### WiFi: the laptops

Plan for about **20 Mbit/s (2.5 MB/s)** for all laptops together (a guess: measure it on the
car). Watching over Foxglove or Lichtblick (port 8765) costs only what you open:

| A laptop opens | MB/s | Mbit/s | Fits? |
|---|---:|---:|---|
| Status, TF, odometry, plan, the lidar's scan, the local costmap | ~0.2 | ~1.6 | ✅ |
| One camera, JPEG, 1080p at 15 Hz | 4.5 | 36 | ❌ |
| The same at 640x360, 5 Hz | ~0.15 | ~1.2 | ✅ |
| The lidar's full points | ~7 | ~56 | ❌ use the scan |
| A raw camera | 93 | 746 | ❌ never |
| The laptop's own detector (`laptop.yaml`, `compressed: true`): one JPEG camera, 15 Hz | 4.5 | 36 | ⚠️ too much for WiFi: plug the laptop into the switch (it has free ports), or lower the rate |

## What to do about the big ones

| # | Do this | Why |
|---|---|---|
| 1 | **Compress pictures on the camera's own computer**, and send only `/compressed` over the network | 93 MB/s becomes about 4.5. The USB camera already sends MJPEG: keep it as JPEG instead of unpacking it to RGB and packing it again, if the driver can (check `usb_cam` and `camera_ros`, Q22) |
| 2 | **Or plug the cameras into the Orin** | then raw pictures never touch the network. The Orin decodes JPEG in hardware (NVJPEG) |
| 3 | **Keep the lidar's points on the Orin** | the packets (0.94 MB/s) come in once; the points (~7 MB/s) and the scan stay there |
| 4 | **Lower rates for people** | a screen needs 5 Hz, not 15. Foxglove and Lichtblick can subscribe to a throttled copy (`topic_tools throttle`) |
| 5 | **Give the HUD a small picture** | the raylib HUD reads the raw front camera. Feed it a 640x360 copy at 5 Hz (3.5 MB/s raw, ~0.15 MB/s JPEG), or teach the HUD JPEG |
| 6 | **Same computer: one process** | where a camera and its reader share the Orin, run them in one process (composable nodes), so the picture is passed, not copied |
| 7 | **Viewers subscribe to one costmap**, not every layer | each Nav2 layer is its own 40-640 kB topic. RViz or Foxglove opening all of them is 5 MB/s |
| 8 | **Measure on the car** with `pixi run measure bandwidth` | every "guess" above becomes a number |

### Cyclone DDS, the default from 23 Oct (Ben)

| Setting | Why |
|---|---|
| **Only the wired network** (Cyclone's `NetworkInterface` set to the Ethernet port) | the localization Pi is also on Carleton's WiFi, and its ROS traffic goes out on both (21 Oct audit). On the car, ROS belongs on the switch |
| **Bigger socket buffers** for big messages (Linux `net.core.rmem_max`, and Cyclone's `SocketReceiveBufferSize`, about 10 MB) | a 1 MB message arrives as hundreds of UDP packets; with a small buffer some are dropped and sent again, and the picture is late |
| No shared memory unless set up | Cyclone sends to the same computer over loopback UDP (its shared memory needs Iceoryx). A raw camera into a detector on the same Orin is then a copy through the kernel: fine for bandwidth, costs CPU. Point 6 above avoids it |
| Sensors at high rates **best effort** | the IMU at 200 Hz needs no resends: a late reading is worthless. Pictures stay reliable (our rule: best effort loses big messages) |

## Measured in Gazebo (23 Oct)

`pixi run measure bandwidth --local --seconds 30` beside `pixi run sim world:=road gui:=false
rviz:=false` (Cyclone, its own ROS domain and Gazebo partition), parked; then again while
Nav2 drove 40 m. Gazebo ran at 0.92-0.98 x real time, so its 10 Hz sensors read 9.2-9.8 Hz.

**These are Gazebo's numbers, not the car's.** Gazebo's cameras are 640x360 (a third of
the car's), and its lidar is a 16-beam stand-in.

| Topic | Hz | Size | MB/s | Mbit/s |
|---|---:|---:|---:|---:|
| `/sensors/lidar/points` | 9.2 | 922 kB | 8.5 | 68 |
| `/sensors/camera/front_left/image_raw` | 9.2 | 691 kB | 6.4 | 51 |
| `/sensors/camera/front_right/image_raw` | 9.2 | 691 kB | 6.4 | 51 |
| `/sensors/camera/rear/image_raw` | 9.2 | 691 kB | 6.4 | 51 |
| `/local_costmap/static_layer` (and `_raw`) | 1.5 | 640 kB | 1.0 each | 7.9 each |
| `/global_costmap/costmap` (and 7 more layers) | 0.5 | 640 kB | 0.32 each | 2.6 each |
| `/joint_states` | 877 | 364 B | 0.32 | 2.6 |
| `/local_costmap/costmap` (and 5 more layers) | 1.5 | 40 kB | 0.06 each | 0.5 each |
| `/gazebo/radar/points` (Gazebo's own) | 9.2 | 6.0 kB | 0.06 | 0.5 |
| `/vehicle/wheels` | 869 | 48 B | 0.04 | 0.3 |
| `/vehicle/odom` | 46 | 724 B | 0.03 | 0.3 |
| `/sensors/lidar/scan` | 9.2 | 2.9 kB | 0.03 | 0.2 |
| `/tf` | 74 | 243 B | 0.02 | 0.1 |
| `/sensors/imu/data_raw` | 46 | 324 B | 0.01 | 0.1 |
| `/ego/actuator_cmd` | 46 | 48 B | 0.002 | 0.02 |
| `/sensors/gps/fix` | 4.6 | 128 B | <0.001 | 0.005 |
| **All 56 topics that sent** | **3,660** | | **33** | **266** |

While driving: the same, plus `/control/cmd_vel` and Nav2's three command topics at 20 Hz
(76 B), and `/plan` at 1 Hz (4-5 kB): **35 MB/s** in all.

What Gazebo taught us:

| Seen | Means |
|---|---|
| 3 cameras and a lidar are 93% of the bytes | the same shape as the car, at a smaller size |
| `/joint_states` and `/vehicle/wheels` at ~880 Hz | Gazebo publishes the wheels every physics step. Gazebo only: the car's wheels are 100 Hz |
| Nav2's costmap layers add up to 5 MB/s | only because the meter subscribed to all 14. Nothing is sent to nobody |
| Nothing late: worst gaps 0.1-0.25 s at 10 Hz | with Cyclone, on a Mac also running other simulators |

## Measure it yourself

```bash
pixi run measure bandwidth --local                  # beside pixi run sim, on this computer
pixi run measure bandwidth --seconds 30 --markdown  # on the car's network: a table for this page
pixi run measure bandwidth --include /sensors       # only some topics (a pattern; repeat it)
pixi run measure bandwidth --exclude image_raw      # leave out the raw pictures
pixi run python3 tools/can_load.py                           # the CAN bus, from the DBC
```

| Column | Means |
|---|---|
| Hz | messages a second over the whole window, by the wall clock |
| size, max | the message as sent (serialized), average and largest |
| MB/s, Mbit/s | Hz x size. 1 MB = 1,000,000 bytes; Mbit = MB x 8, as links are sold |
| worst gap s | the longest wait between two messages: late or bursty data shows here |
| from | the publishing nodes, so you know which computer makes it |

!!! warning "Measuring costs bandwidth too"
    The meter subscribes to every topic, raw pictures too. On the car, run it **on the
    Orin** (where the big topics already arrive), or `--exclude image_raw` from a laptop
    on WiFi.

The code: `tools/bandwidth.py` (the ROS part), `tools/topic_load.py` and
`tools/can_load.py` (the numbers, no ROS; tests in `tools/tests/`).
