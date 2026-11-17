# GPS from zero

GPS tells the car where it is on the earth, as a latitude and longitude. This page
explains how that works, how good it is, and how our car turns it into metres on a map.

**You'll learn:**
- what GNSS is, and why "GPS" is only part of it;
- fix types, and what RTK float and RTK fixed mean;
- what our receiver, the u-blox ZED-F9R, can do;
- every field of a ROS `NavSatFix` message;
- the datum, and how `navsat_transform` turns degrees into metres;
- how to check that GPS data is really arriving;
- the bench's GPS noise problem (N7 in `TODO.md`).

This page goes deeper than [Maps and position](maps-and-position.md), which
shows the big picture. Read that one first if you haven't.

## 1. GNSS: satellites and clocks

| Word | Means |
|---|---|
| **GNSS** | any satellite positioning system. "GPS" is the American one |
| The four systems | GPS (USA), GLONASS (Russia), Galileo (Europe), BeiDou (China). Modern receivers use all four |
| How it works | each satellite sends the time. The receiver works out how far away each one is from how late the signal arrives |
| How many satellites | at least **4**: three for the position, one for the receiver's own clock error |
| What it gives | latitude, longitude, altitude and very exact time |

### What spoils it

| Problem | Effect |
|---|---|
| The sky is blocked (buildings, trees, a garage) | fewer satellites; no fix indoors |
| Signals bounce off walls (multipath) | a position a few metres off, near buildings |
| The air (ionosphere) slows the signal | metres of error, mostly removed by dual-band receivers |
| A poor antenna cable | weak signal. Our ZED-F9R's antenna lead is held on with tape ([Car photos](car-photos.md)) |

## 2. Fix types: how good is this fix?

| Fix | Typical error | Needs |
|---|---|---|
| No fix | - | fewer than 4 satellites |
| 2D | 5-10 m | 3 satellites: no altitude |
| 3D | **2-5 m** | 4+ satellites. What a phone gets |
| SBAS / DGPS | about 1 m | correction signals from a satellite or a station |
| **RTK float** | 10-50 cm | RTK corrections, still being worked out |
| **RTK fixed** | **1-2 cm** | RTK corrections, fully worked out |

### RTK in one picture

```text
  satellites ───────────┬────────────────────┐
                        ▼                    ▼
   base station (fixed, known spot)     our car (rover)
        │  "here's the error I see right now"  ▲
        └──── corrections (RTCM) over radio or the internet ────┘
```

- A **base station** sits on a known spot. It sees the same errors as the car.
- It sends **corrections** (RTCM messages), often through an internet service (NTRIP).
- The car's receiver removes the shared errors.
- **Float:** the maths hasn't settled yet. **Fixed:** it has. Fixed can take tens of
  seconds after the corrections start.

!!! note "Where do our corrections come from?"
    Not known yet. The switch has a port labelled **RTK-PI**, so the 2025-26 car probably
    had a corrections setup. Still to find out at the car ([Audit questions](audit-questions.md)).

## 3. Our receiver: u-blox ZED-F9R

| Feature | What it means for us |
|---|---|
| **Dual band** (L1 + L2) | removes most of the air's error |
| **RTK** | centimetres, once it gets corrections |
| **A built-in IMU** (gyro + accelerometer) | **dead reckoning**: it keeps a position going for a while in a tunnel or under trees |
| Can read wheel speed | better dead reckoning. Not wired on our car |
| USB | it shows up on the Pi as `/dev/ttyACM*`; our udev rule names it `/dev/aav/gps` |
| Rate | 1 fix a second from the factory. Faster is set once, with u-blox's u-center |

On the car today: a SparkFun ZED-F9R board on the ceiling panel, USB-C into a Pi. The
KEFC plan replaces it with a board that has a screw-on antenna connector.

!!! warning "Never tried with our code"
    Our GPS driver (`ublox_gps`) has **never run** with this receiver. The first step at
    the car: is there any data at all?

### The driver: three choices

`sensors.launch.py gps_driver:=...` picks one. Each publishes `/sensors/gps/fix`.

| Choice | Package | Notes |
|---|---|---|
| `ublox` (default) | `ublox_gps` (KumarRobotics) | in Pixi. Uses the receiver **as it is set up**: changes nothing that lasts |
| `nmea` | `nmea_navsat_driver` | any receiver that speaks NMEA text |
| `ublox_dgnss` | `ublox_dgnss` | the 2024-25 team's choice. apt only, not in Pixi |

Which one stays is open question **Q18** (Tentative: `ublox`).

## 4. The message: sensor_msgs/NavSatFix

| Field | Example | Means |
|---|---|---|
| `header.stamp` | | when the fix was taken |
| `header.frame_id` | `gps_link` | **the antenna**, not the box. Its spot on the car is in `vehicle.yaml`: over the rear axle (x = 0), 1.50 m up (a guess until measured) |
| `status.status` | 0 | -1 no fix, 0 fix, 1 SBAS, 2 GBAS (ground-based corrections) |
| `status.service` | 1 | which systems: 1 GPS, 2 GLONASS, 4 BeiDou, 8 Galileo (added together) |
| `latitude` | 45.3860 | degrees, + north |
| `longitude` | -75.6960 | degrees, + east (so Ottawa is negative) |
| `altitude` | | metres above the WGS-84 ellipsoid, not sea level |
| `position_covariance` | 9 numbers | how unsure, in m², east-north-up. The middle diagonal ones matter |
| `position_covariance_type` | 2 | 0 unknown, 1 approximate, 2 diagonal known, 3 full |

**Reading the covariance:** the square root is the error in metres.
`position_covariance[0] = 0.25` means about **0.5 m** east-west (1 sigma).

**Degrees to metres, near Ottawa:**

| Change | Is about |
|---|---|
| 0.00001° of latitude | 1.1 m north |
| 0.00001° of longitude | 0.79 m east (less, because we're at 45° north) |

## 5. The datum: where degrees meet metres

"Datum" has two meanings. Both matter.

| Meaning | Ours |
|---|---|
| **The earth's shape** that latitude and longitude are measured on | WGS-84, what every GPS uses |
| **robot_localization's datum**: the one point where the `map` frame is (0, 0) | 45.3860 N, -75.6960: a **placeholder** near Carleton (`navsat.yaml`) |

From the datum:
- x points **east**, y points **north**, in metres;
- every map, every Nav2 goal and every GPS waypoint uses this frame;
- set it to the test site before the car drives there.

## 6. navsat_transform: degrees in, metres out

```text
/sensors/gps/fix  ─┐
/sensors/imu/data ─┼─► navsat_transform ─► /localization/gps/odom  (x, y in map) ─► ekf_global
/localization/odom/global ─┘                └► /localization/gps/fix  (the filtered position, as lat/lon)
```

| Setting (`navsat.yaml`) | Ours | Why |
|---|---|---|
| `wait_for_datum` | true | use our datum, not the first fix |
| `datum` | [45.3860, -75.6960, 0.0] | the placeholder above |
| `zero_altitude` | true | a flat map: ignore height |
| `delay` | 3.0 s | wait for the EKF before starting |
| `magnetic_declination_radians`, `yaw_offset` | 0.0 | must be 0 with a datum. No magnetometer yet |
| `frequency` | 30 Hz | |

It also:
- removes the antenna's offset, using TF `base_link → gps_link`: that's why the antenna's
  spot in `vehicle.yaml` matters (and where it is mounted: below);
- serves `/fromLL` (lat/lon → x, y) and `/toLL` (back). A street route reaches Nav2
  through `/fromLL` (`street_router`, [Street routes](routes.md)).

### Mount the antenna over the rear axle

To remove the offset, `navsat_transform` turns it by the car's heading **as the global EKF
believes it** (`map → base_link`). A heading error then moves the GPS position sideways,
and the EKF corrects its heading from that. Driving forward, the two agree. Reversing,
they pull the wrong way, and the heading gets worse.

Measured in Gazebo (22 Oct; two runs of each; 15 s legs at 0.5 m/s, forward and back):

| Antenna | Heading off, reversing (median) | Forward (median) | Worst |
|---|---:|---:|---:|
| 0.8 m ahead of the rear axle (the old guess, before 23 Oct) | 7.5-12.8° | 3.7-6.7° | 39-54° |
| **over the rear axle** | 4.6-6.6° | 5.1-7.5° | 26-31° |

So the antenna belongs **above the rear axle** (any height): it is there already
(`vehicle.yaml`, 23 Oct; measure it to be sure). Then the
offset doesn't turn with the heading, and reversing is as good as forward. A heading
source helps too: since 25 Oct the receiver's own velocity gives one while the car moves
(`gps_heading`, [Learn: localization](learn-localization.md#where-the-heading-comes-from-todo-13)).

## 7. GPS in Gazebo and on the bench

| | Gazebo (`pixi run sim`) | Bench (`pixi run bench`) | The car |
|---|---|---|---|
| Made by | Gazebo's `navsat` sensor on `gps_link` | `mock_gps` | `ublox_gps` |
| Path | `/aav/navsat` → `/vehicle/sim/navsat` → `sim_gps` → `/sensors/gps/fix` | straight to `/sensors/gps/fix` | straight to `/sensors/gps/fix` |
| Rate | 5 Hz | 5 Hz | 1 Hz (factory) |
| Noise | 4.5e-6° (about 0.5 m) per fix | 0.5 m per fix | unknown until we log it |
| Velocity (`/sensors/gps/fix_velocity`) | `sim_gps`: the car's true motion + 0.05 m/s noise | none | `ublox_gps`, from the Doppler shift |
| Status | always `STATUS_FIX` | always `STATUS_FIX` | depends on the sky |
| Origin | the world's `spherical_coordinates` = our datum | the datum | the earth |

### The bench's GPS problem: N7

| | Bench GPS today | A real receiver |
|---|---|---|
| Error | **fresh** random noise on every fix | **drifts slowly**: wrong by the same metre for a while |
| Looks like | dots jumping around the truth, 5 times a second | a smooth line, a little off to one side |
| Effect on `ekf_global` | the map correction jiggles, so the car **weaves** a little | smooth, but offset |

So tuning `ekf_global` on the bench tunes it for the wrong kind of noise. The fix in
`TODO.md` (N7): model the bench's GPS error as a slow drift, then tune `ekf_global` for
the real receiver, once we have logs from it (Q18).

## Try it

| You want to | Run |
|---|---|
| A GPS in Gazebo | `pixi run sim` |
| How often fixes come (expect 5 Hz) | `pixi run tools/ros.sh ros2 topic hz /sensors/gps/fix` |
| One fix, every field | `pixi run tools/ros.sh ros2 topic echo --once /sensors/gps/fix` |
| The same fix after `navsat_transform`, in metres | `pixi run tools/ros.sh ros2 topic echo --once /localization/gps/odom` |
| A lat/lon as map x, y | `pixi run tools/ros.sh ros2 service call /fromLL robot_localization/srv/FromLL "{ll_point: {latitude: 45.3861, longitude: -75.6959}}"` |
| The real receiver (on the Pi it's plugged into) | `ls -l /dev/aav/gps`, then `pixi run tools/ros.sh ros2 launch aav_sensors sensors.launch.py imu:=false` |

The `/fromLL` example is about 11 m north and 8 m east of the datum, so expect x near 8
and y near 11.

!!! note "Nothing on the topic?"
    `ublox_dgnss` publishes best effort. Add `--qos-reliability best_effort` to
    `ros2 topic echo`. Indoors, a real receiver has no fix at all: go outside.

## Where this lives in the code

| What | Where |
|---|---|
| GPS driver settings | `ros/sources/aav_sensors/config/ublox_gps.yaml` |
| Which driver starts | `ros/sources/aav_sensors/launch/sensors.launch.py` (`gps_driver:=`) |
| The datum, navsat_transform | `ros/localization/aav_localization/config/navsat.yaml` |
| The antenna's spot on the car | `ros/bringup/aav_description/config/vehicle.yaml` (`gps_link`) |
| Gazebo's GPS sensor | `ros/bringup/aav_description/urdf/aav.gazebo.xacro` (`navsat`) |
| Gazebo → our topic name | `ros/sim/aav_sim_models/aav_sim_models/sim_gps.py` |
| The bench's pretend GPS | `ros/sim/aav_bench/aav_bench/sensors.py` (`MockGps`) |
| The device name `/dev/aav/gps` | `tools/udev/99-aav.rules` |
| The contract | `docs/interfaces.yaml`: `/sensors/gps/fix`, `/localization/gps/*` |
