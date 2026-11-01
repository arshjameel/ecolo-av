# Localization from zero

"Localization" means: **where is the car, which way is it facing, and how fast is it
going?** No one sensor answers that well. This page shows how our car mixes several
weak answers into one good one.

**You'll learn:**
- odometry, and why it drifts;
- what an IMU measures;
- fusion, and the idea behind a Kalman filter, with one tiny example in numbers;
- robot_localization, the stock package we use;
- the frames `map`, `odom` and `base_link` (REP-105), and TF;
- exactly what our two EKFs read, from their config files;
- how to look at all of it on your own computer.

[Maps and position](maps-and-position.md) has the big picture of the two EKFs
and the map. This page explains **why** it works.

[How the car knows its speed](learn-speed.md) builds on this page.

## 1. Odometry: counting your steps

**Odometry** = working out where you are from how you've moved.

```text
start ──► 1.0 m/s for 2 s, turning 0.1 rad/s ──► "I'm about 2 m ahead, a bit to the left"
```

| Our odometry source | Measures | Topic |
|---|---|---|
| Wheel speed (today: the hardware lead's Arduino, on CAN 0x200/0x201) | forward speed | `/vehicle/odom` |
| The steering angle (KEFC: the KTC-100 pot, through the Jrk) | turn rate = speed × tan(angle) ÷ wheelbase | `/vehicle/odom` |
| The IMU's gyro | turn rate | `/sensors/imu/data` |

### Why it drifts

Every small error is **added up** forever.

| Error | After 100 m |
|---|---|
| Wheel size off by 1 % (ours is a guess: 0.30 m) | 1 m too far or too short |
| Heading off by 1° | **1.7 m** to the side |
| Heading off by 5° | 8.7 m to the side |

Heading errors hurt most. That's why GPS is needed to stay true over a long drive.

## 2. The IMU

An **IMU** (inertial measurement unit) is a chip that feels motion.

| Part | Measures | Good | Bad |
|---|---|---|---|
| Gyro | turn rate (rad/s), on 3 axes | fast and smooth | a small bias: the heading creeps |
| Accelerometer | acceleration (m/s²), with gravity | shows tilt | noisy; twice-integrated it's useless for position |
| Magnetometer | the earth's magnetic field | an absolute heading | fooled by the motor and steel. **Not used by us yet** |

Ours: a Yahboom 10-axis IMU on USB. `imu_filter_madgwick` turns its raw data into a
relative orientation. The EKFs use only its **yaw rate**: how fast the car turns.

## 3. Fusion: two weak answers make one good one

| Source | Strong at | Weak at |
|---|---|---|
| Wheels + IMU | smooth, fast (50+ Hz), short-term | drift over long distances |
| GPS | never drifts | noisy, slow (1-5 Hz), can jump |

**Fusion** keeps the best of each: smooth from the wheels and IMU, true from GPS.

## 4. The Kalman filter, in one tiny example

A Kalman filter keeps two things for every number it tracks:
1. a **best guess**;
2. **how unsure** it is (the variance: the error squared).

It repeats two steps.

| Step | What happens | Uncertainty |
|---|---|---|
| **Predict** | move the guess with the odometry | grows |
| **Update** | a measurement arrives: move the guess toward it | shrinks |

### One dimension, in numbers

The car drives along a straight road. Position in metres.

**Before:** the wheels say **10.0 m**. After a long drive, they're unsure: variance
**4** (about ±2 m).

**A GPS fix arrives:** it says **12.0 m**, variance **1** (about ±1 m).

| Step | Sum | Result |
|---|---|---|
| How much to trust GPS: the **gain** K | 4 ÷ (4 + 1) | **0.8** |
| New guess | 10.0 + 0.8 × (12.0 − 10.0) | **11.6 m** |
| New variance | (1 − 0.8) × 4 | **0.8** (about ±0.9 m) |

- The answer sits **closer to GPS**, because GPS was less unsure.
- The new variance (0.8) is **smaller than either** (4 or 1). Two answers beat one.
- Then the car drives on: predict adds the movement, and the variance grows again
  until the next fix.

An **EKF** (extended Kalman filter) does the same with many numbers at once (x, y,
heading, speeds...), and handles turning, where the maths isn't a straight line.

## 5. robot_localization: the stock EKF

We don't write our own filter. `robot_localization` (version 3.8.3, Jazzy) gives us:

| Node | Job |
|---|---|
| `ekf_node` | the filter. We run it **twice**: `ekf_local` and `ekf_global` |
| `navsat_transform_node` | GPS degrees → map metres ([Learn: GPS](learn-gps.md)) |

Each input has a list of 15 true/false values: which parts of the message to use.

```text
x, y, z,   roll, pitch, yaw,   vx, vy, vz,   vroll, vpitch, vyaw,   ax, ay, az
```

## 6. Frames: map, odom, base_link (REP-105)

![Frames on our car, from above, and the TF tree](img/frames.svg)

| Frame | Is | Moves? | Set by |
|---|---|---|---|
| `map` | pinned to the earth at the datum. x east, y north | never | - |
| `odom` | where the car started today | can **hop** a little when GPS corrects it | `ekf_global` publishes `map → odom` |
| `base_link` | the car: on the ground, under the middle of the rear axle. x forward, y left | smoothly | `ekf_local` publishes `odom → base_link` |
| `imu_link`, `gps_link`, `radar_link`, `camera_*_link` | each sensor | fixed to the car | `robot_state_publisher`, from `vehicle.yaml` |

REP-105 is the ROS rule for these names. REP-103 sets the units and directions:
metres, radians, x forward, y left, z up.

### TF: the frame tree

**TF** is ROS's service for "where is frame A, seen from frame B, at time t?"

| Idea | Means |
|---|---|
| A tree | each frame has **one** parent. `map → odom → base_link → gps_link` |
| One publisher per link | two nodes publishing `odom → base_link` would fight. That's why `imu_filter_madgwick` has `publish_tf: false` |
| `/tf` | links that move (the EKFs), many times a second |
| `/tf_static` | links that never move (sensor mounts), sent once |
| Chaining | ask for `map → gps_link`; TF multiplies the links in between |

## 7. Our two EKFs: their real inputs

From `ros/localization/aav_localization/config/`. Both run at 30 Hz, in 2D
(`two_d_mode: true`: no height, roll or pitch).

| Input | What it gives | `ekf_local` uses | `ekf_global` uses |
|---|---|---|---|
| `/vehicle/odom` (the backend) | wheel speed, turn rate | vx, vy, vyaw | vx, vy, vyaw |
| `/sensors/imu/data` | the gyro | vyaw | vyaw |
| `/localization/gps/odom` (navsat_transform) | GPS as map x, y | - | **x, y** |
| `/localization/gps/heading` (gps_heading) | the heading, from the GPS receiver's velocity, while the car moves | - | **yaw** |

| | `ekf_local` | `ekf_global` |
|---|---|---|
| `world_frame` | `odom` | `map` |
| Publishes | `/localization/odom/local` | `/localization/odom/global` |
| TF | `odom → base_link` | `map → odom` |
| Used by | Nav2's controller and local costmap | Nav2's planner and global costmap; navsat_transform |
| Starts at | (0, 0), sure of it | `start_x`, `start_y`, `start_yaw` from the launch file; unsure (yaw ±18°) |

Four choices worth knowing:

| Choice | Why |
|---|---|
| **vy = 0 is fused** | a car can't slide sideways. Telling the filter so keeps it honest |
| **The IMU gives the turn rate, never the heading** | no magnetometer, so the IMU's heading is only relative. The start heading comes from `start_yaw`; once the car moves, the GPS receiver's own velocity says which way it points (below) |
| **The heading trusts the turn rate** | yaw process noise 0.01, not the stock 0.06: the gyro and wheels give the turn rate well, so the heading follows them and GPS only nudges it. With 0.06 it wandered up to ±25° at walking pace (below) |
| **Each backend sets its own covariance** | the CAN spine works out the turn rate from the measured steering. Today's wheel sensors can't (no steering angle on the bus), so `spine_bridge role:=sensors` sends a huge covariance (1e3) for it, and the EKF ignores that part |

With `gps:=false`, `ekf_global` doesn't run, and a fixed `map → odom` (zero) stands in.

### How well it works

| Where | Result |
|---|---|
| Gazebo | both EKFs within 0.2 m, parked |
| Gazebo, driving (22 Oct) | the global EKF's heading, 3 min forward and back at 0.5-1 m/s: median 2.6° off, worst 14° (yaw noise 0.06: median 6.2-6.7°, worst 39-54°). From a start 29° wrong: right within 10 s. With a gyro biased 0.6°/s: median 3.8° |
| Gazebo, driving (25 Oct) | with the heading from the GPS velocity (below), 5 Hz GPS, 4 runs each: median **1.6-2.1°** off (was 2.2-2.8), 95 % of the time under 4.9-6.0° (was 7.2-9.1), worst 8.5-13.6° (was 11.4-27.1), forward and reversing alike |
| Gazebo, harder (25 Oct, 2 runs each) | a gyro biased 0.6°/s: median 1.5-1.9° (was 3.3-4.3). Started 30° wrong: under 5° after 5.4-6.1 s (was 7.8-10.0), 3 s of it parked |
| Bench | 0.03-0.3 m from the truth |
| The car | never run yet |

### Where the heading comes from (TODO 13)

One GPS antenna can't tell which way the car points while it stands still. Once it moves,
three things can:

```text
  GPS fixes, one after another      the receiver's own velocity      a compass
     •  •   •  • ─▶ (0.5 m noise:     ──▶ (Doppler: 0.05 m/s,          🧭 (the motors
     slow to settle)                   so 6° at 0.5 m/s, 3° at 1 m/s)  bend it)
```

The global EKF had only the first. **Now it has the second too**: `gps_heading`
(`aav_gps_heading`) turns the receiver's velocity (`/sensors/gps/fix_velocity`, from
`ublox_gps`) into a heading on `/localization/gps/heading`:

| Step | What it does |
|---|---|
| 1 | direction of the velocity = where the antenna goes |
| 2 | reversing (the wheels say so): turn it round 180° |
| 3 | an antenna off the rear axle swings out in a turn: take that out (0 today: it's over the axle) |
| 4 | slower than 0.3 m/s, or the GPS's speed and the wheels' disagree by 0.3 m/s: say nothing |
| 5 | how sure: the velocity's noise across the travel ÷ the speed, plus 2° |

robot_localization can't do this alone: it turns a world-frame velocity into the car's
frame with its own heading, so a wrong heading never shows.

What we looked at (`pixi run heading-check`: Gazebo, 3.5 min forward and back at
0.5-1 m/s, against the truth):

| Option | Verdict |
|---|---|
| **(a) The GPS velocity's direction** | **kept**: every number better (table above). No stock package does it; ours is small: `gps_heading.py`, its sums in `course.py`, tested without ROS |
| (b) The magnetometer, through `imu_filter_madgwick` (`use_mag`) | not now: the Yahboom's `/sensors/imu/mag` is raw counts, not tesla; it needs a calibration away from steel, and the motors' current moves it while driving; Ottawa's declination is 12.5° W. Gazebo can't show the motors' effect, so a sim test would flatter it |
| (c) Lidar scan matching | not in our environment (no KISS-ICP, slam_toolbox, rf2o); a new dependency. It gives turn, not north: it helps the local EKF, not this |
| (d) Settings: `use_odometry_yaw`, `yaw_offset`, `differential` | no effect with a datum: `navsat_transform` only uses them to set the map frame, which the datum fixes. `differential` on GPS would throw away its position |

**The receiver's rate matters.** The car's receiver sends one fix a second from the
factory; Gazebo's sends five. At 1 fix a second (3 runs without, 2 with): median
2.2-3.1° (was 2.8-5.4), worst 10.1-16.9° (was 10.4-21.2). Five a second is a receiver
setting (u-center, once), for whoever sets it up (Q18).

**What's left**, beyond software. Built in software; the hardware lead decides:

| Upgrade | Heading | Standing still? |
|---|---|---|
| Today: one antenna, velocity + gyro | 1.6-2.1° median, worst ~14°, in Gazebo | no: the gyro carries it |
| RTK (the ZED-F9R can, with a base station or NTRIP) | position to 1-2 cm, so the fixes' own heading gets ~25 times better | no |
| Two antennas (a moving base, e.g. two ZED-F9P), 1 m apart | about 0.4° | **yes** |
| The ZED-F9R's own fusion (its built-in IMU, `publish.esf` in `ublox_gps.yaml`) | to measure: it needs a calibration drive | after calibrating |

Real GPS errors drift slowly, not fresh each fix as in Gazebo ([Learn: GPS](learn-gps.md)):
that hurts the heading from fixes more than the heading from velocity, so on the car the
gain should be at least as big. To check with the first drive's bag.

## 8. Next: matching the lidar to a map (NDT and ICP)

Not built yet: a research pick ([research picks](designs/2026-10-28_research-picks.md),
pick 2). GPS gets the car to within a metre or two. To get to centimetres without RTK,
match what the lidar sees now against a map of the lot.

### NDT: blobs

**NDT** (Normal Distributions Transform) keeps a blob per square, not every point.

![NDT in four steps: the map, its blobs, a scan off by 1.4 m and 9°, and the scan slid onto the blobs](img/learn-ndt.png)

| Step | What happens |
|---|---|
| 1 | Drive the lot once and keep the lidar's points: the map |
| 2 | Cut the map into squares (cubes, in 3D). In each, keep only the points' centre and spread: a blob |
| 3 | A new scan arrives. Dead reckoning has drifted, so it lands in the wrong place |
| 4 | Slide and turn the scan until its points sit inside the blobs. That slide is the car's error |

Big squares first, then smaller: big ones see far, small ones are precise.

### ICP: nearest points

**ICP** (Iterative Closest Point) keeps every map point.

![ICP: pair each scan point with its nearest map point, move to close the pairs, repeat](img/learn-icp.png)

| Step | What happens |
|---|---|
| 1 | Pair each scan point with the nearest map point (the black lines) |
| 2 | Find the one slide and turn that closes the pairs best |
| 3 | Repeat. The pairs get better each time |

Look at the posts on the left: some pairs are wrong at first. ICP still gets there
because most pairs are right; start it too far off and it locks onto the wrong post.

### Which to use

| | NDT | ICP |
|---|---|---|
| The map holds | one blob per square: small | every point: big |
| Starting far off | copes better (big squares first) | needs a close start |
| Speed | fast, the same for any map | slower as the map grows |
| Used by | Autoware (`ndt_scan_matcher`) to find the car in a map | KISS-ICP, to **make** the map (scan after scan) |

So: **make** the map with KISS-ICP, **find** the car in it with NDT, and give that
pose to `ekf_global` beside GPS.

Not Tesla's way: Tesla has no lidar. Its "occupancy network" turns camera images into
3D cubes, each marked filled or empty. That looks like NDT's squares, but it doesn't
match a scan to a map.

`pixi run ndt-demo` draws both pictures with real NDT and ICP (`tools/ndt_demo.py`,
numpy only). Each ends within about 2 cm.

From the lidar's points to cubes, the costmap and NDT, step by step:
[Learn: lidar](learn-lidar.md).

## Try it

| You want to see | Run |
|---|---|
| Start Gazebo, with both EKFs | `pixi run sim` |
| The frame tree, and who publishes each link | `pixi run tools/ros.sh ros2 run rqt_tf_tree rqt_tf_tree` |
| The car on the map, live | `pixi run tools/ros.sh ros2 run tf2_ros tf2_echo map base_link` |
| The GPS correction alone | `pixi run tools/ros.sh ros2 run tf2_ros tf2_echo map odom` |
| The local estimate | `pixi run tools/ros.sh ros2 topic echo --once /localization/odom/local` |
| Every setting the filter really uses | `pixi run tools/ros.sh ros2 param dump /ekf_global` |
| Drive, then watch the numbers change | `pixi run keyboard --sim` |

Drive in a circle with the keyboard, and watch `tf2_echo map odom`: it stays small.
That's the GPS soaking up the drift.

## Where this lives in the code

| What | Where |
|---|---|
| The EKFs' settings | `ros/localization/aav_localization/config/ekf_local.yaml`, `ekf_global.yaml` |
| GPS to metres | `ros/localization/aav_localization/config/navsat.yaml` |
| How they start, and topic names | `ros/localization/aav_localization/launch/localization.launch.py` |
| The IMU filter | `ros/sources/aav_sensors/config/imu_filter.yaml` |
| The heading from the GPS velocity | `ros/localization/aav_gps_heading` (`course.py`: the sums, no ROS) |
| Measure the heading in Gazebo | `pixi run heading-check` (`tools/heading_check.py`) |
| Sensor positions on the car | `ros/bringup/aav_description/config/vehicle.yaml` |
| Wheel speed into `/vehicle/odom` | `aav_spine/spine_bridge.py`, `aav_legacy` `encoder_udp.py`, `aav_gazebo/gazebo_adapter.py` |
| Why the EKFs meet in TF, not on a topic | [Topic areas](naming.md) |
| Frames in the contract | `docs/interfaces.yaml`, `frames:` |
