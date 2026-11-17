# Sensors from zero

Every sensor on our car: what it measures, how often, which ROS topic and frame it uses,
what it's good and bad at, which driver reads it, and whether Gazebo can pretend to be it.

**You'll learn:**
- the sensors a self-driving car needs, and why it needs more than one;
- each of ours: the numbers, the topic, the frame, the driver, the status;
- which ones the simulator makes, and which it doesn't;
- how to check that a sensor is really talking.

!!! warning "Nothing here has run on the car yet"
    Our drivers work in Gazebo and on the bench. On the real car, every sensor below is
    [[untested]]. The audits of 16 and 21 Oct found what's fitted
    ([16 Oct](history/audit-2026-10-16-car.md), [21 Oct](history/audit-2026-10-21-car.md)).

## 1. Why so many sensors?

| Question | Answered by |
|---|---|
| How fast am I going? | wheel speed |
| How fast am I turning? | the IMU's gyro, the steering angle |
| Where am I on the earth? | GPS |
| What's in front of me? | radar, cameras, lidar |
| What's the road like? | cameras (lanes, signs) |

Each sensor fails in its own way. Together they cover each other's gaps
([Learn: localization](learn-localization.md)).

## 2. All of them, on one table

| Sensor | Measures | Rate | Topic | Message | Frame | In Gazebo? |
|---|---|---|---|---|---|---|
| Wheel speed | forward speed | 50 Hz (sim); 100 Hz CAN frames on the car (`aav.dbc`) | `/vehicle/odom` | `nav_msgs/Odometry` | `base_link` | **yes**, on `main` |
| Steering angle | front wheel angle | 50 Hz (in CAN feedback) | `/vehicle/status`, `/vehicle/joint_states` | `VehicleStatus`, `JointState` | the steering joints | yes (status) |
| IMU (Yahboom) | turn rate, acceleration | 200 Hz device; 50 Hz sim | `/sensors/imu/data_raw` → `/sensors/imu/data` | `sensor_msgs/Imu` | `imu_link` | **yes**, on `main` |
| GPS (ZED-F9R) | latitude, longitude | 1 Hz factory; 5 Hz sim | `/sensors/gps/fix` | `sensor_msgs/NavSatFix` | `gps_link` | **yes**, on `main` |
| Radar (TI mmWave) | points and their speed | 10 Hz | `/sensors/radar/points` | `sensor_msgs/PointCloud2` | `radar_link` | **yes**, in the road world |
| Front cameras (one Pi 5) | pictures | 15 fps, 1280 × 720, H.264 | `/perception/detections` (the pictures stay in `camera_stream`; `image_raw` only with the lane part) | `vision_msgs/Detection2DArray` | `camera_left_link`, `camera_right_link` | **yes**, in the road world (front left only) |
| Lidar (roof, VLP-16) | 3D points all round | 10 Hz | `/sensors/lidar/points` | `sensor_msgs/PointCloud2` | `lidar_link` | a generic 16-beam one, on by default |
| Battery (Victron) | voltage, charge | 1 Hz (bench) | `/vehicle/battery` | `sensor_msgs/BatteryState` | - | no |

Every frame's position on the car is in `ros/bringup/aav_description/config/vehicle.yaml`.

## 3. Wheel speed

| | |
|---|---|
| **Measures** | how fast the wheels turn → forward speed |
| **Today** | an Arduino (the hardware lead's) reads the wheel sensors and sends CAN frames 0x200 and 0x201 straight to the Orin (Ben, 23 Oct; the hardware lead confirms: [Today's wiring](can-plan.md#todays-wiring-23-october)) |
| **The 2025-26 encoder** | an optical encoder (8 spokes) on a Pico 2, sending UDP packets every 0.1 s to `encoder_udp` (`aav_legacy`). It was reported fouled; off in our machine files since 23 Oct |
| **The KEFC plan** | Hall sensors reading the holes in the two front brake rotors, each on an ESP32-CAN-X2 wheel node, the same frames at 100 Hz (Proposed) |
| **Good at** | smooth, fast, never jumps |
| **Bad at** | wheel slip; a wrong wheel size (ours, 0.30 m, is a guess) adds up |
| **Driver** | `spine_bridge role:=sensors` on the Orin (CAN); `encoder_udp` for the 2025-26 encoder |
| **Gazebo** | yes: `sim_adapter` reads the car model's motion |

## 4. Steering angle

| | |
|---|---|
| **Measures** | the angle of the front wheels |
| **Today** | **nothing**: the steering is open loop |
| **After KEFC** | a KTC-100 linear pot, read by the steering Jrk G2, sent in `VEHICLE_FEEDBACK` (0x180) |
| **Good at** | the turn rate, from speed × tan(angle) ÷ wheelbase |
| **Bad at** | needs calibrating: which pot reading is straight ahead? |
| **Driver** | `spine_bridge` |

## 5. IMU: Yahboom 10-axis

| | |
|---|---|
| **Measures** | gyro: turn rate on 3 axes. Accelerometer: acceleration with gravity. Magnetometer: the earth's field |
| **Rate** | 200 Hz (set on the device; the 2025-26 README) |
| **Plugs in** | USB serial, 115200 baud, named `/dev/aav/imu` by our udev rule |
| **Good at** | fast, smooth turn rate |
| **Bad at** | the gyro drifts slowly; the magnetometer is fooled by the motor and steel. **We don't use the magnetometer yet** |
| **Driver** | `aav_imu_yahboom` (Ben's Rust rewrite of the 2025-26 C++ driver), then the stock `imu_filter_madgwick` |
| **Used by** | both EKFs: the **yaw rate** only |
| **Gazebo** | yes, 50 Hz, with noise 0.02 rad/s and 0.2 m/s² |

## 6. GPS: u-blox ZED-F9R

| | |
|---|---|
| **Measures** | latitude, longitude, altitude, time |
| **Rate** | 1 Hz from the factory; faster once set up with u-center |
| **Good at** | never drifts; centimetres with RTK |
| **Bad at** | slow; metres off without RTK; nothing indoors; jumps near buildings |
| **Driver** | `ublox_gps` (default; Tentative, Q18), or `nmea_navsat_driver`, or `ublox_dgnss` |
| **On the car** | a SparkFun board on the ceiling panel, USB to a Pi; its antenna lead is taped |
| **Gazebo** | yes, 5 Hz, about 0.5 m noise |

All about it: [Learn: GPS](learn-gps.md).

## 7. Radar: TI mmWave

| | |
|---|---|
| **Measures** | points (x, y, z) and each point's speed toward or away from the radar |
| **Rate, range** | 10 Hz; 8.4 m (`aav_radar_ti` `cfg/base.cfg`) |
| **Good at** | works in the dark, rain and fog; measures speed directly |
| **Bad at** | few points; poor height; clutter close up. Publishes best effort: subscribe best effort |
| **Driver** | `aav_radar_ti` (from the 2025-26 `Nav_Radar`), then `radar_clusters` (`aav_radar_clusters`, Rust) groups the points |
| **Used by** | the local costmap's obstacle layer, and `collision_monitor` |
| **Gazebo** | yes, in the road world: a lidar shaped like the radar (`/vehicle/sim/radar/points`, turned into ours by `sim_radar`) |
| **Bench** | `mock_radar` draws points on the bench's boxes |

## 8. Cameras: one Pi 5 at the windscreen

| | |
|---|---|
| **Measures** | colour pictures |
| **On the car** | two ribbon cameras on one Pi 5 (no ROS there): MediaMTX streams each as H.264, 15 fps at 1280 × 720 |
| **Good at** | the only sensor that reads lanes, signs and lights |
| **Bad at** | no distance by itself; glare, dark; big messages (so the pictures stay inside the Orin) |
| **Driver** | `aav_camera_stream` on the Orin: decodes, undistorts, runs RF-DETR inside each stream ([Camera pipeline](camera-pipeline.md)). Not calibrated yet ([how](camera-calibration.md)). `usb_cam` (`cameras.launch.py`) is only for a USB camera |
| **Used by** | `aav_detector` (stop signs, objects), `aav_lane_finder`, the HUD |
| **Gazebo** | yes, in the road world: 640 x 360, 10 Hz, the bench camera's lens |
| **Bench** | `mock_camera` draws the bench road |
| **KEFC** | adds a rear camera (Cam3). The QNX Pi's camera doesn't work today: no rear camera for now |

## 9. Lidar: Velodyne VLP-16 on the roof

| | |
|---|---|
| **Measures** | 3D points all round, from laser pulses |
| **Rate** | 10 turns a second |
| **Good at** | exact distances, day or night |
| **Bad at** | rain and snow; expensive; lots of data; blind close to the car, low down ([Perception](perception.md#the-roof-lidar)) |
| **On the car** | a VLP-16 on a printed tripod, 192.168.1.201, sending UDP to the Jetson (the 16 Oct audit) |
| **Driver** | the stock `velodyne` package (`aav_sensors lidar:=true`), then `lidar_scan` (`aav_lidar`) for Nav2. On by default on the car. Tested against `tools/fake_vlp16.py`; [[untested]] with the real lidar |
| **Used by** | the local costmap's lidar layer, and `collision_monitor` |
| **Gazebo** | a generic 16-beam one, on by default (`lidar:=false` to drop it) |
| **Bench** | `pixi run bench lidar:=true`: `mock_lidar` sends VLP-16 packets of the bench world |

## Try it

| You want to | Run |
|---|---|
| Gazebo's sensors | `pixi run sim` |
| Every sensor topic that's live | `pixi run tools/ros.sh ros2 topic list` |
| How often the IMU talks (expect 50 Hz in Gazebo) | `pixi run tools/ros.sh ros2 topic hz /sensors/imu/data` |
| One wheel-speed message | `pixi run tools/ros.sh ros2 topic echo --once /vehicle/odom` |
| The bench's radar and camera too | `pixi run bench`, then `pixi run tools/ros.sh ros2 topic hz /sensors/radar/points` |
| Where each sensor sits on the car | `pixi run view` (RViz with the car model) |
| The live graph against the contract | `pixi run check bench` (with the bench running) |

## Where this lives in the code

| What | Where |
|---|---|
| Starting the drivers | `ros/sources/aav_sensors/launch/sensors.launch.py`, `cameras.launch.py` |
| Driver settings | `ros/sources/aav_sensors/config/` (`imu.yaml`, `imu_filter.yaml`, `ublox_gps.yaml`, `cameras/`) |
| IMU driver | `ros/sources/aav_imu_yahboom` |
| Radar driver | `ros/sources/aav_radar_ti` |
| Battery | `ros/sources/aav_battery_victron` |
| Wheel speed | `ros/can_bridge/aav_spine_bridge` (Rust, the default) or `aav_spine` (Python): `spine_bridge role:=sensors`; `ros/can_bridge/aav_legacy` (the 2025-26 encoder) |
| The front cameras | `ros/sources/aav_camera_stream` |
| The lidar's scan for Nav2 | `ros/perception/aav_lidar` |
| Sensor positions | `ros/bringup/aav_description/config/vehicle.yaml` |
| Gazebo's sensors | `ros/bringup/aav_description/urdf/aav.gazebo.xacro`, `ros/sim/aav_sim_models` |
| The bench's pretend sensors | `ros/sim/aav_bench/aav_bench/sensors.py` |
| Every topic | `docs/interfaces.yaml`, shown as [Topics](interfaces.md) |
