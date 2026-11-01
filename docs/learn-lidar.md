# From lidar points to cubes, maps and where we are

Ben's guess (29 Oct): "the lidar returns some kind of point map and we have an NDT and
occupation network that reads that data and produces something cube related."

**Close.** The lidar gives a cloud of dots. From those dots we can make **cubes** (what is
around us), a **flat map for Nav2** (where we may drive), **objects** (what each thing is),
and with a saved map, **where we are** (NDT). This page takes them one at a time.

**You'll learn:**
- what one lidar point is, and what our VLP-16 sends;
- cubes (voxels): filled, free, unknown;
- why ours aren't Tesla's "occupancy network";
- how cubes flatten into Nav2's costmap, and what runs today;
- joining cubes into named objects;
- NDT, KISS-ICP and AMCL: finding the car on a map;
- what's built, what's planned, what's only an idea.

The maths of NDT and ICP, with pictures, is in
[Learn: localization §8](learn-localization.md#8-next-matching-the-lidar-to-a-map-ndt-and-icp).
Costmaps in more depth: [Costmaps](costmaps.md).

## 1. What a lidar does

A **lidar** fires a laser pulse and times the echo. Light is fast, but the clock is faster.

```text
  lidar ──── pulse ────►  wall
        ◄─── echo ─────
  echo after 66.7 ns  ──►  distance = speed of light × time ÷ 2 = 10 m
```

It knows the beam's direction too, so each echo becomes **one point** in 3D.

| A point has | Means | Example |
|---|---|---|
| `x`, `y`, `z` | where it is, in metres (x forward, y left, z up) | 5.13, −1.07, 0.62 |
| `intensity` | how bright the echo was: paint and signs are bright | 0-255 |
| `ring` | which of the 16 lasers saw it | 0-15 |
| `time` | when, within the turn | |

## 2. Our lidar: the VLP-16

A Velodyne VLP-16 on a printed tripod on the roof ([Learn: sensors §9](learn-sensors.md#9-lidar-velodyne-vlp-16-on-the-roof)).

| | Value |
|---|---|
| Lasers | **16**, stacked from 15° down to 15° up, every 2° |
| Spin | 10 turns a second (10 Hz) |
| Points | 1800 per laser per turn: **28,800 a turn** |
| Range | up to 100 m; a few cm accurate |
| Topic | `/sensors/lidar/points` (`sensor_msgs/PointCloud2`, about 0.9 MB a cloud) |
| Driver | the stock `velodyne` package, in `lidar_link` |

Seen from the side, the 16 lasers fan out like this:

```text
                     ╱  +15°
  lidar (1.8 m up) ●──────  0°
                     ╲  −15°   the lowest ring hits the road 6.7 m away
  ═══════════════════════════════════════  road
```

**Two things that follow, in numbers:**

| Fact | Sum | So |
|---|---|---|
| The lowest ring meets the road at | 1.8 m ÷ tan 15° = **6.7 m** | it can't see the road closer than that |
| It meets a 1 m tall box at | (1.8 − 1.0) ÷ tan 15° = **3.0 m** | closer, low things are the radar's job |
| Rings are 2° apart; at 20 m that's | 20 × tan 2° = **0.7 m** apart | a small thing far away can slip between rings |

!!! warning "Never seen on our car"
    The driver has run against `tools/fake_vlp16.py` and in Gazebo, never with the real
    VLP-16. Where it sits (`lidar_link`, 0.9 m forward, 1.8 m up) is a guess to measure.

## 3. Cubes (voxels)

A cloud is 28,800 loose dots. Hard to ask "is there something here?" So cut the space
round the car into **cubes**, 20 cm on a side. A 3D pixel is called a **voxel**.

Each cube is one of three:

| Cube | Means | How we know |
|---|---|---|
| **Filled** | something is there | a point landed in it |
| **Free** | nothing is there | a laser passed **through** it to a point further on |
| **Unknown** | we can't tell | no laser went through it (behind a car, under the lowest ring) |

```text
  side view, one laser:

  lidar ●  .  .  .  .  .  ■  ?  ?  ?
           free cubes     filled   unknown (hidden behind it)
```

**Worked example:** a point at x = 5.13, y = −1.07, z = 0.62 m.

| Axis | Point ÷ 0.2 m | Round down | The cube spans |
|---|---|---|---|
| x | 25.65 | 25 | 5.0 to 5.2 m |
| y | −5.35 | −6 | −1.2 to −1.0 m |
| z | 3.1 | 3 | 0.6 to 0.8 m |

So cube (25, −6, 3) is **filled**.

**How many cubes?** 20 m × 20 m round the car, 2 m high: 100 × 100 × 10 = **100,000
cubes**, against 28,800 points a turn. Most cubes are free or unknown. Cubes not seen for
a while **fade** back to unknown, so a person who walked away doesn't stay.

!!! note "Planned, not built"
    No code makes cubes yet. The plan (TODO, "Tesla-style cubes on our HUD"): a voxel grid
    in Rust in `hud-core`, no ROS, its own tests; Gazebo first.

## 4. Not Tesla's "occupancy network"

Tesla's car has no lidar. Its **occupancy network** is a learned neural network: it looks
at eight cameras and **guesses** which cubes are filled.

| | Tesla | Ours (planned) |
|---|---|---|
| Input | camera pictures | lidar points |
| How | a trained neural network | plain geometry: a point in a cube = filled |
| Needs | millions of training clips | nothing to train |
| Distance | estimated | measured, to a few cm |
| Sees colour, signs | yes | no: that's the cameras' job (section 6) |

The picture looks the same: cubes round the car, coloured by height. The way it's made
is not. So we call ours "cubes" or a "voxel grid", not an occupancy network.

## 5. From cubes to Nav2's flat map

Nav2 plans on a **2D costmap**: a grid seen from above, 10 cm cells, each free or
blocked ([Learn: Nav2](learn-nav2.md)). To get there, look **down each column** of cubes.

```text
  roof + margin  2.0 m ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄  above: a branch, a bridge. Ignore
                       │ ■ │   │   │
                       │ ■ │   │   │      a filled cube in the band
                       │ ■ │   │   │      ──► this cell is BLOCKED
  clearance      0.3 m ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄
                       │   │ ■ │   │      below: the road, a kerb. Ignore
  road           0.0 m ═══════════════════
```

| A column's filled cube is | The cell is | Example |
|---|---|---|
| between 0.3 m and 2.0 m | **blocked** | a person, a car, a post |
| below 0.3 m | free | the road, a 15 cm kerb |
| above 2.0 m | free | a tree branch the car fits under |

The band comes from `vehicle.yaml`: the body is 1.5 m tall, plus a 0.5 m margin
(`aav_description` `obstacle_heights`). Both are guesses until measured.

### What runs today: `lidar_scan`, no cubes

Today there are no cubes. `lidar_scan` (`ros/perception/aav_lidar`) does the same job a
simpler way: in each of **720 directions** (every 0.5°), it keeps the **nearest point
inside the band**.

| | |
|---|---|
| In | `/sensors/lidar/points`: 28,800 points, 0.9 MB |
| Out | `/sensors/lidar/scan`: 720 distances, about 3 kB |
| Read by | the local costmap's `lidar_layer`, and `collision_monitor`'s stop zone |

The local costmap (`nav2.yaml`) stacks its layers:

| Layer | From |
|---|---|
| `static_layer` | the saved map, if there is one |
| `obstacle_layer` | the radar |
| `camera_layer` | the cameras (off until they're calibrated) |
| `lidar_layer` | **the lidar's scan**: out to 12 m, the newest scan only |
| `ghost_layer` | the pretend person, for tests ([Ghost obstacles](ghost-obstacles.md)) |
| `inflation_layer` | grows each blocked cell by 2.0 m, so the car keeps clear |

A flat scan loses one thing: a table with an empty space under it looks like a wall.
That's fine for a car. Nav2's own 3D `VoxelLayer` is there if 2D ever misses something.

## 6. Cubes into objects, with names

Cubes say **something** is there. Not **what**. Two steps (TODO, 29 Oct; **planned**):

**Step 1: join touching filled cubes into one object.** The radar already does this with
its points (`aav_radar_clusters`, DBSCAN).

```text
   ■■         ■            two objects:
   ■■■        ■            one 0.6 m wide, one 0.2 m wide and 2 m tall
   ■■         ■
```

**Step 2: give it a name.** The camera's detector (RF-DETR) draws boxes round things it
knows: "stop sign", "person", "car" (`/perception/detections`). If an object's cubes fall
inside a box, it gets the box's name, and the cubes give its distance.

```text
   camera picture             lidar cubes, drawn on the picture
   ┌───────────────┐          ┌───────────────┐
   │   ┌─────┐     │          │   ┌─────┐     │
   │   │STOP │     │   ──►    │   │ ■■  │     │   "stop sign, 14 m"
   │   └─────┘     │          │   └─────┘     │
   └───────────────┘          └───────────────┘
```

| Needs | Why |
|---|---|
| **Camera-to-lidar calibration** | to know where each cube lands on the picture. Research pick 4: `direct_visual_lidar_calibration` |
| Matching times | the camera and the lidar must see the same moment |
| Gazebo first | there every pose is exact, so a mistake is in the code, not the calibration |

## 7. Where are we? NDT on a saved map

GPS is good to 1-3 m. A lidar matched to a **saved map** of the lot is good to centimetres
([research picks](designs/2026-10-28_research-picks.md), pick 2; **planned**).

**NDT** (Normal Distributions Transform) keeps the map as cubes too, but each cube holds a
**bell curve**: the centre of its points, and how spread out they are.

```text
  a lamp post's cube:  points ∙∙∙∙  ──►  centre (12.0, 3.0), spread 0.1 m
                                         a scan point near the centre scores high,
                                         one far off scores nothing
```

| Scan point's distance from the centre | Score, e^(−d² ÷ 2σ²), σ = 0.1 m |
|---|---|
| 0.05 m | 0.88 |
| 0.30 m | 0.01 |

**Slide and turn the scan** until its points score highest. That slide is how far the car
really is from where it thought.

```text
  /sensors/lidar/points ──► NDT, against the saved map ──► a pose (x, y, yaw)
                                                          └──► ekf_global, beside GPS
```

`pixi run ndt-demo` shows it on a made-up lot: a scan off by 1.4 m and 9° ends within about
2 cm. Step by step, with pictures: [Learn: localization §8](learn-localization.md#8-next-matching-the-lidar-to-a-map-ndt-and-icp).

## 8. Making the map: KISS-ICP

NDT needs a map first. **KISS-ICP** makes one (planned):

| Step | What |
|---|---|
| 1. Record | drive the lot slowly; record the lidar and the IMU (`tools/record.sh`) |
| 2. Map | run KISS-ICP on the recording: it chains scan to scan by ICP, and keeps the points |
| 3. Save | the point map of the lot |
| 4. Use | NDT matches each new scan against it (section 7) |

KISS-ICP's own scan-to-scan motion could also help `ekf_local` at low speed, where the
wheels' clicks come slowly ([Learn: speed](learn-speed.md)).

**The catch:** a bare lot gives the lidar little to match. Kerbs, posts and walls help.

## 9. A trial alternative: AMCL on a 2D map

**AMCL** (Adaptive Monte Carlo Localization) is Nav2's stock way to find a robot on a 2D
map. It scatters hundreds of guesses ("particles") and keeps the ones whose view matches
the lidar's 2D scan.

| | NDT (planned) | AMCL (a trial design) |
|---|---|---|
| Map | 3D points | 2D grid, seen from above |
| Reads | the full cloud | `/sensors/lidar/scan` |
| Software | not chosen: look at stock NDT localizers first | stock `nav2_amcl` |
| First test | a recording of the real lot | Gazebo's `test_lot` world |

The trials design (`docs/designs/2026-10-29_trials.md`, on branch `trials`, Tentative) runs
AMCL **beside** `ekf_global`: its pose goes out on `/localization/amcl/pose`, it never
takes over `map → odom`, and it's scored against the EKF on the same drives. Not built yet.

## 10. Seeing it all: the debug HUD

The debug HUD (TODO, 29 Oct; **planned**) puts what the car thinks on one screen:

| It draws | From |
|---|---|
| the cubes, coloured by height | the voxel grid (section 3) |
| the cubes on the camera picture | the calibration (section 6) |
| "stop sign, 14 m" | RF-DETR boxes + cubes |
| where the scan sits on the saved map | NDT (section 7) |

It reads each sensor's own topic, not what comes out of the muxes, so a fault shows where
it starts. Built on the raylib HUD (`ros/hud/aav_hud`).

## 11. Today, planned, idea

| Piece | State | Where |
|---|---|---|
| VLP-16 → `/sensors/lidar/points` | ✅ built; Gazebo and a fake lidar; never the real one | `aav_sensors` `sensors.launch.py` |
| Points → 2D scan → costmap and `collision_monitor` | ✅ built; Gazebo | `aav_lidar` `lidar_scan`, `nav2.yaml` |
| Cubes from the lidar | planned (TODO: cubes on the HUD) | `hud-core`, Rust |
| Cubes drawn on the camera picture | planned; on the car, needs calibration | `picture.rs`, research pick 4 |
| Cubes into named objects | planned (TODO 29 Oct) | |
| A map of the lot with KISS-ICP | planned (research pick 2) | needs a recording of the lot |
| NDT pose into `ekf_global` | planned (research pick 2) | |
| AMCL beside the EKF | a trial design, Tentative | branch `trials` |
| The debug HUD | planned (TODO 29 Oct) | |
| Nav2's 3D `VoxelLayer` | an idea: only if 2D misses something | |
| A small model that names cubes from the camera crop | an idea | |

## Where this lives in the code

| What | Where |
|---|---|
| The lidar's topics | `docs/interfaces.yaml`: `/sensors/lidar/points`, `/sensors/lidar/scan` |
| The driver | `ros/sources/aav_sensors/launch/sensors.launch.py` (`lidar:=true`) |
| Points into a scan | `ros/perception/aav_lidar` (`scan.py`: the sums, no ROS) |
| The lowest rings and body roll | `aav_lidar` `ground.py` |
| The costmap's layers | `ros/drivers/aav_navigation/config/nav2.yaml` |
| Where the lidar sits; the height band | `vehicle.yaml` `lidar_link`; `aav_description` `vehicle.py` |
| Joining radar points | `ros/perception/aav_radar_clusters` |
| NDT and ICP, drawn | `tools/ndt_demo.py` (`pixi run ndt-demo`) |
| The roof lidar end to end | [Perception](perception.md#the-roof-lidar) |
