# Maps and position: from the wheels to OpenStreetMap

Written 15 October 2026. It answers:

1. What does the local EKF give you?
2. How does it become a position on a real map, with GPS?
3. Where does OpenStreetMap come in?
4. How do costmaps relate to all of it?

> **The short version**
> - **Local EKF:** "how I've moved since I started". Smooth, but it slowly drifts.
> - **Global EKF:** the same, plus GPS. "Where I am on the earth". True, but it can jump a little.
> - The **map frame** is a flat grid in metres, pinned to one GPS point (the **datum**). That's how x/y and latitude/longitude meet.
> - **OpenStreetMap is only for people and routes:** the HUD's map window (streets, place search) and the street router (Valhalla). Navigation never reads it.
> - **Costmaps** are grids of "how bad is it to drive here". The **global** one is big and coarse (for the whole route). The **local** one is small, fine and live (for what's in front of the car right now).

## 1. Your picture, corrected a little

You said: *"local EKF gives odometry, then you put it on a map and mix it with GPS."*

Close. The mixing with GPS is done by **a second EKF**, not on top of the first one:

```text
 wheels + IMU ──────────────► ekf_local  ──► "odom" position  (smooth, drifts)
 wheels + IMU + GPS(x,y) ───► ekf_global ──► "map" position   (true, may hop)
                    ▲
 GPS lat/lon ─► navsat_transform ─► GPS as x,y in metres
```

- Both EKFs read the raw sensors. Neither reads the other's topic.
- They meet in **TF**, the tree of coordinate frames: `map → odom → base_link`.

## 2. The three frames

| Frame | Think of it as | Who sets it | Good for | Bad at |
|---|---|---|---|---|
| `base_link` | the car itself: on the ground, under the rear axle | the car model | "1 m in front of me" | - |
| `odom` | where the car **started**, today | `ekf_local` | smooth motion, steering | drifts: after 1 km it may be metres out |
| `map` | a fixed grid on the earth | `ekf_global` + GPS | "where am I", plans, goals | can hop when a GPS fix arrives |

A picture of the chain:

```text
   map ──(ekf_global: the GPS correction)──► odom ──(ekf_local: the smooth motion)──► base_link
```

- `ekf_local` says how far the car is from where it started: `odom → base_link`.
- `ekf_global` says how far "where it started" is from the true spot: `map → odom`. That small correction soaks up the drift.
- Chain them and you get the car on the map, still smooth.

## 3. How x/y meets latitude/longitude: the datum

The `map` frame is flat and in metres, but GPS gives latitude and longitude. They're joined at **one point**, the datum.

| Setting | Value | File |
|---|---|---|
| Datum | 45.3860 N, -75.6960 (near Carleton: a **placeholder**) | `ros/localization/aav_localization/config/navsat.yaml` |
| Map (0, 0) | = the datum | |
| x axis | east | |
| y axis | north | |

- `navsat_transform` turns each GPS fix into x/y metres from the datum.
- It also answers on request: `/fromLL` (lat/lon → x/y) and `/toLL` (x/y → lat/lon).
- That's how a GPS goal becomes a navigation goal.

## 4. Where OpenStreetMap comes in

OpenStreetMap (OSM) is a free map of the world's roads and places. **We use it for people
and for routes**, never inside navigation. Everything works offline: the data are files.

| Feature | Uses | Where | In git? |
|---|---|---|---|
| Draw the street map | Ottawa's streets as vector tiles (Protomaps, 40 MB) | the HUD's map window, under the navigation map ([HUD](hud.md#the-street-map-under-it)) | no: `pixi run get street-map` |
| Search "Tim Hortons" | `ottawa.db`: 606,628 named places from OSM (51 MB) | the HUD's map window ([search](hud.md#search-for-a-place-and-drive-there)) | no: `pixi run get places` |
| Plan a street route | Valhalla (a road router), Ottawa's OSM data | `street_router` (`aav_routes`), on this computer ([Street routes](routes.md)) | no: `pixi run get routes` |

A place reaches the car in one of two ways:

```text
 map window: pick a place ─► DRIVE HERE, CONFIRM ─► /goal_pose (inside the navigation map only)

 street_router: a destination ─► Valhalla: a street route (lat/lon points)
     ─► a person calls /drivers/nav/route/follow ─► the navigator: FollowGPSWaypoints
     ─► /fromLL: each point to map x/y ─► the navigator drives through them
```

> ⚠️ **The planner doesn't know where the road is** (except with `path:=lane`, the lane map). The route's points follow the road, but the planner
> only sees the points, not the road. Between two points it plans across its costmap,
> which today is empty (all free). Staying in the lane is the lane keeper's job
> (`aav_lane_finder`, `aav_lane_geometry`). A lane layer in the costmap would be the next step.

Both ways worked in Gazebo (16 Oct), with Nav2; neither has run on the car.

## 5. Costmaps: the fine-grained part

A **costmap** is a grid laid on the ground. Each cell holds a cost:

| Cost | Means |
|---|---|
| 0 | free: drive here |
| low to high | near something: OK, but further away is better |
| lethal | a wall or an obstacle: never |
| unknown | not seen yet |

**Inflation** adds a soft band of cost around every obstacle, as wide as the car (2.0 m for us), so a plan for the car's **centre** also keeps its **corners** clear.

### Our two costmaps

| Costmap | Question it answers | Frame |
|---|---|---|
| Global | "Which way to the goal?" (the planner) | `map` (fixed on the earth) |
| Local | "What's right in front of me?" (the controller) | `odom` (smooth: never jumps) |

Why the local one is in `odom`: if it were in `map`, every GPS hop would shift the obstacles under the car.

Sizes, cells, layers and sensors: [Costmaps](costmaps.md#the-two-costmaps).

## 6. All the scales, big to small

| Scale | Thing | Cell or step | Job |
|---|---|---|---|
| kilometres | OpenStreetMap + Valhalla (`street_router`) | road segments | "which streets to take" |
| tens of metres | GPS waypoints | one point every few tens of metres | breadcrumbs for the navigator |
| 200 m | global costmap | 25 cm | a path the car can drive |
| 20 m | local costmap | 10 cm | dodge what the sensors see |
| a few metres | the stop zone (`aav_stop_zone`) | a box just ahead | last-moment stop, whatever navigation thinks |
| centimetres | lane keeping | camera pixels | stay in the lane |

## 7. What works, and what's missing

| Part | Status |
|---|---|
| `ekf_local`, `ekf_global`, `navsat_transform` | ✅ in the simulator and on the bench. ❌ never on the car |
| Navigation with both costmaps | ✅ in the simulator and on the bench |
| Radar and cameras in the local costmap | ✅ on the bench |
| Lidar in the local costmap | ✅ the roof VLP-16 through the stock `velodyne` driver, tested against a fake lidar; untested with the real one |
| The real datum | ❌ a placeholder: set it at the test site |
| A street route → the navigator | ✅ in the simulator ([Street routes](routes.md)); untested on the car |
| A map of the real test site | ❌ only `empty` and an example lot |
| Navigation knowing about roads and lanes | 🔧 the lane map and `path:=lane` (16 Nov, in `stack:=ours`) |
| A failing GPS stops mattering | 🔧 `pose_source:=ndt` ([below](#8-ndt-leads-the-gps-checks)): a challenger, sim only |

## 8. NDT leads, the GPS checks

**A challenger (TODO 210), `pose_source:=ndt`.** Ben, 18 Nov: "The gps is seeming to fail".
Today `ekf_global` takes the GPS. With `pose_source:=ndt` it takes **NDT** (the lidar on a
saved map), and the GPS only **checks** it ([the design](designs/2026-11-19_ndt-leads.md)).

```text
 NDT's pose ──┐                    ┌─ NDT trusted ──────────▶ NDT's pose ─▶ ekf_global
 GPS fix ─────┼─▶ pose_check ──────┼─ else GPS trusted ─────▶ GPS's fix ─▶ ekf_global
 AMCL pose ───┘  (aav_pose_check)  └─ else HOLD ─▶ the sensor watchdog stops the car
                        └─ NDT lost ─▶ start NDT again from AMCL, else the GPS
```

| When | `ekf_global` takes | The car |
|---|---|---|
| NDT tracks | NDT only; the GPS is compared, never used | drives |
| NDT and the GPS disagree (5 s, past both spreads and 1.5 m) | NDT, if its score is at its floor or better; else the GPS | drives; the status names the wrong one |
| NDT lost | the GPS, if healthy; NDT restarts from AMCL or the GPS | drives |
| neither trusted | nothing | **stops** (HOLD) |

### Scored in Ravensim (19 Nov, the lot, 3 runs each)

The GPS fault goes on once the EKF has its first fix; the watchdog and the geofence on;
nobody clears a latch. "EKF off" = where `ekf_global` put the car, against the truth.

| GPS | Today (`gps`): legs | EKF off, mean / worst | NDT leads (`ndt`): legs | EKF off, mean / worst |
|---|---|---|---|---|
| clean | 12/12 | 0.10 / 0.29 m | 12/12 | **0.01 / 0.15 m** |
| stops | 3/12: held (STALE, geofence LOST) | 0.04 / 0.06 m | **12/12** | 0.01 / 0.15 m |
| jumps 20 m | 3/12: held (JUMP, latched) | 0.04 / 0.06 m | **12/12** | 0.01 / 0.14 m |
| drifts 5 m in a minute | 12/12, **on a pose 5 m wrong** | 3.61 / 5.19 m | **12/12** | **0.01 / 0.12 m** |

- **Today** stops safely when the GPS stops or jumps, but drives on a wrong pose when it
  drifts: nothing notices.
- **NDT leading** drives every leg in every case. The drift is named ("GPS 5.0 m off NDT,
  judged wrong; NDT carries on") and never reaches the pose.
- The geofence kept working: it reads the fused pose, and "a fix arrived" from whichever
  source `ekf_global` takes.
- First try (trust score 2.6, Gazebo's): NDT's 5 s median dipped under it, the drifting
  GPS was believed, and the pose went 3.3 m off. Now the trust score is NDT's own floor
  (2.3): a right NDT scored 2.0-4.0 there, one 5 m off 0-2.0 (TODO 121).

**Untested:** the car (no lidar map of the real lot yet: `car.launch.py` refuses it).

## 9. Try it

| You want to see | Run |
|---|---|
| The two EKFs and the frames | `pixi run sim`, then `pixi run tools/ros.sh ros2 run rqt_tf_tree rqt_tf_tree` |
| The car on the map, live | `pixi run tools/ros.sh ros2 run tf2_ros tf2_echo map base_link` |
| A GPS point as map x/y | `pixi run tools/ros.sh ros2 service call /fromLL robot_localization/srv/FromLL "{ll_point: {latitude: 45.3861, longitude: -75.6959}}"` |
| The costmaps | in RViz (it opens with `pixi run sim`): add `/local_costmap/costmap` and `/global_costmap/costmap` |

More: [Costmaps](costmaps.md), [How it fits](how-it-fits.md), [HUD](hud.md).
