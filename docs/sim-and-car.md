# Simulation and the real car: how far we've got

Written 15 October 2026, checked 25 October and 13 November. **This page:** what is
simulated, what is real, and which layer swaps. How to run it: [Simulation](simulation.md);
the plan behind it: [Full simulation](full-simulation.md); the default simulator since
8 Nov: [Our own simulator](own-simulator.md). Where it says Gazebo, read either simulator:
`pixi run sim` now starts ours, and `simulator:=gazebo` starts Gazebo. It answers three
questions:

1. How far did we get with Gazebo?
2. Does it really produce sensor data?
3. How does any of it relate to the real car?

> **The short version**
> - **Gazebo works**, and it **does produce sensor data**: an IMU, a GPS, wheel speed, a camera, a radar and a lidar, all on `main`.
> - The three cameras, the radar and the lidar are in the **road world** and the **parking lot**: `pixi run sim world:=road` or `world:=test_lot` (all on by default since 15 Oct; `lidar:=false` drops the lidar).
> - **The car's lidar** is a roof Velodyne VLP-16: the stock `velodyne` driver, on by default on the car, untested with the real lidar.
> - **Almost none of our code has run on the real car.** On 13 Nov the car drove with only the localization update installed ([Status](status.md)): no recording or numbers in the repo yet. Everything else "works" in simulation or on the bench only.
> - The point of simulation: the **same nodes** run in all three places. Only the bottom layer changes.

## 1. The big idea: swap the bottom layer

Most of the software doesn't know whether it's driving a real car or a pretend one.

```text
            ┌──────────────────────────────────────────────┐
  THE SAME  │ drivers → twist_mux → ego                    │
  EVERYWHERE│ EKFs, navsat_transform, Nav2, perception, HUD│
            └──────────────────────┬───────────────────────┘
                     /ego/actuator_cmd  ↓      ↑  /vehicle/odom, /imu, /sensors/gps/fix ...
            ┌──────────────┬────────────────┬────────────────┐
  SWAPPED   │  Gazebo      │  The bench     │  The real car  │
            │ (simulator)  │ (stand-ins)    │ (hardware)     │
            └──────────────┴────────────────┴────────────────┘
```

- Everything **above** the line is the same code, with the same settings.
- Only the **bottom box** changes. We call it the **backend**, and exactly one runs at a time.
- So if Nav2 drives well in Gazebo, the same Nav2 should drive the car. The only differences are the real sensors' noise and the real motors.

## 2. The three bottom boxes

| | Gazebo | The bench | The real car |
|---|---|---|---|
| What it is | a 3D physics simulator | stand-in programs: no physics, no 3D | the car |
| Command | `pixi run sim simulator:=gazebo` (`pixi run sim` alone: our own simulator, the same topics) | `pixi run bench` | `pixi run car <machine>`: [[untested]] as a whole |
| Who plays the car | Gazebo's Ackermann car model | `aav_spine_sim`: the spine's own firmware logic, on a model car | the boards and motors |
| Sensors come from | Gazebo's sensor models | small "mock" nodes that pretend | the real devices |
| CAN frames | no | **yes**: the same frames the real spine will send | yes, after the KEFC work |
| Best for | Nav2, localization, driving in a world | the car's own software path, e-stops, CAN | the truth |
| Status | ✅ works on Linux and on the Mac | ✅ works, with its own checks | ❌ never run |

## 3. Does Gazebo produce sensor data? Yes

### In every world

| Sensor | Gazebo makes it | The car's software sees it as | Rate | Status |
|---|---|---|---|---|
| IMU | `/aav/imu` | `/sensors/imu/data_raw` → filter → `/sensors/imu/data` | 50 Hz | ✅ |
| GPS | `/aav/navsat` | `/sensors/gps/fix` | 5 Hz | ✅ |
| Wheel speed | from the car model's motion | `/vehicle/odom` | 50 Hz | ✅ |
| Car status | from the car model | `/vehicle/status` | 50 Hz | ✅ |

**How it gets there:**

```text
Gazebo sensor → ros_gz_bridge → /gazebo/... → one node per sensor → the real drivers' topic names
```

`sim_adapter` (the car), `sim_imu`, `sim_gps` and `sim_radar` rename everything to the **same names the real drivers use** (`/sensors/gps/fix`, `/sensors/imu/data_raw`, `/vehicle/odom`). The software above can't tell the difference. One process each, as each sensor on the car has its own driver.

![The simulated car in Gazebo](img/gazebo-car.png)

### In the road world (`pixi run sim world:=road`)

Merged from the branch `wt/sim-full` on 15 Oct. Rates measured on Ben's Mac, on simulated time.

| Added | The car's software sees it as | Rate | Status |
|---|---|---|---|
| A road: lane lines, a stop sign, a box in the road (`obstacle:=true`) | | | ✅ |
| Three cameras: two on top looking forward, one at the back looking back | `/sensors/camera/front_left/image_raw`, `front_right/...`, `rear/...` | 10 Hz | ✅ (15 Oct) |
| A radar on the grill (a lidar shaped like it) | `/sensors/radar/points` (via `/vehicle/sim/radar/points`) | 10 Hz | ✅ |
| A 16-beam lidar on the roof | `/sensors/lidar/points` | 10 Hz | ✅ on by default (15 Oct): all sensors together pass on a quiet Mac. Earlier failures came from other programs competing |

![What the three simulated cameras see](img/sim-cameras.png)

- **Cameras and lidars on a Mac:** they render without a window, and didn't crash.
- **Checked by** `pixi run sim-check`: every check passes, on 15 Oct.
- **Details:** [Simulation](simulation.md): every sensor's noise, the lidar, what isn't simulated.

### Not simulated at all

| Thing | Why not |
|---|---|
| The real lidar | Gazebo's is a generic 16-beam one, not a VLP-16's exact pattern |
| The magnetometer | the EKFs don't use it |
| The battery | nothing to simulate yet |
| The real CAN boards | that's the bench's job, not Gazebo's |

## 4. "Works" means: in simulation or on the bench

| Part | Gazebo | Bench | Real car |
|---|---|---|---|
| twist_mux, ego (the command path) | ✅ | ✅ | ❌ never run |
| Both EKFs + navsat_transform | ✅ within 0.2 m parked | ✅ 0.03-0.3 m from the truth | ❌ |
| Nav2 drives to a goal | ✅ 8 m and 9.5 m goals | ✅ 40 m past a stop sign | ❌ |
| Stop sign → the car stops | ✅ (11 Oct) | ✅ | ❌ |
| Obstacle stop (collision_monitor) | ✅ 0.74 m before the box | ✅ | ❌ |
| The spine and CAN frames | | ✅ against the spine simulator | ❌ the board was never flashed |
| GPS driver (`ublox_gps`) | | | ❌ never run with the receiver |
| IMU driver | | | ❌ needs the device |
| The raylib HUD | ✅ as a ROS node, with test topics | ✅ next to bench mode | ❌ not on the car's screen yet |

## 5. Why none of it has run on the car yet

| Blocker | What it means | Who or when |
|---|---|---|
| **The car runs ROS 2 Humble; we use Jazzy** | the two can't talk on one network (Q24) | reflash the Orin, or run our Jazzy through Pixi on its Ubuntu 22.04 (`pixi run car-kit`) |
| **What runs on each computer** | surveyed in the audits of 16 and 21 Oct (`docs/history/audit-2026-10-16-car.md`, `audit-2026-10-21-car.md`) | done |
| **The drive path is still the 2025-26 one** | Motor-Pi + the C++ ESP32 over UART. Our `aav_legacy` backend speaks it, but has never been tried | the [car day](car-day.md) |
| **The CAN bus has only sensors** | the wheel sensors' Arduino on the Orin's CAN; no CAN spine yet | the hardware lead |

## 6. Today vs after the KEFC changes

| Layer | Today on the car | Our code today | After KEFC |
|---|---|---|---|
| Computer | Orin #1 (Humble) | runs on Jazzy | Orin #1 (control, EKF) + Orin #2 (perception, Nav2) |
| Drive path | Motor-Pi → the C++ ESP32 (UART) | `aav_legacy` backend (`actuators:=uart`), untested; the Rust ESP32 firmware's 100 ms timeout is proven in QEMU, not yet flashed | Orin → CAN → ESP32 control board (`aav_spine`, `actuators:=can`) |
| Wheel speed | the hardware lead's Arduino on the Orin's CAN (0x200/0x201) | `spine_bridge role:=sensors`, untested on the car; the 2025-26 UDP encoder is off since 23 Oct | the same frames |
| Steering feedback | none (open loop) | `measured_steering_angle` waits for it | KTC-100 pot → the Jrk G2 |
| GPS | ZED-F9R on a taped u.FL lead | `ublox_gps` driver, untested | new ZED-F9R board, screw-on antenna |
| Lidar | a Velodyne VLP-16 on the roof (16 Oct audit) | the stock `velodyne` driver, on by default; untested with the real lidar | read on Orin #2 |
| Cameras | 2 front ribbon cameras on one Pi 5 | MediaMTX → `aav_camera_stream` on the Orin, RF-DETR inside; untested on the car | + a rear camera |

## 7. The plan, in order

1. **Done 15 Oct:** the Gazebo camera, radar and lidar, merged into `main`.
2. **Done 16 and 21 Oct:** the audits found what really runs.
3. **Next ([car day](car-day.md)):** get our stack onto the car, either Jazzy on the Orin or Pixi, and listen first: `ros2 topic hz /sensors/gps/fix`.
4. **Then:** drive on stands through `aav_legacy`, before any CAN work.
5. **Then:** the KEFC parts, the CAN bench with a CANable, one board at a time.

## 8. Try it yourself

| You want to | Run |
|---|---|
| See the simulated car | `pixi run sim` (our simulator, no window: watch in RViz); Gazebo's window: `pixi run sim simulator:=gazebo` |
| See its sensor data | in another terminal: `pixi run tools/ros.sh ros2 topic hz /sensors/gps/fix` |
| Look at one GPS message | `pixi run tools/ros.sh ros2 topic echo --once /sensors/gps/fix` |
| Drive it | `pixi run keyboard --sim` |
| Check the whole graph | `pixi run check sim` |
| The road world, with cameras, radar and lidar | `pixi run sim world:=road` |
| The parking lot, with every sensor | `pixi run sim world:=test_lot` |
| Test the road world end to end | `pixi run sim-check` (2 minutes, no window) |
| The bench instead | `pixi run bench`, then `pixi run bench-check` |
| The HUD on top of it | `pixi run hud` |

More detail: [Simulation](simulation.md), [Bench mode](bench.md), [Status](status.md).
