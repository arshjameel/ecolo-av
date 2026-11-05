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
> - **OpenStreetMap is only for people and routes:** the HUD's map window (streets, place search) and the street router (Valhalla). Nav2 never reads it.
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
- That's how a GPS goal becomes a Nav2 goal.

## 4. Where OpenStreetMap comes in

OpenStreetMap (OSM) is a free map of the world's roads and places. **We use it for people
and for routes**, never inside Nav2. Everything works offline: the data are files.

| Feature | Uses | Where | In git? |
|---|---|---|---|
| Draw the street map | Ottawa's streets as vector tiles (Protomaps, 40 MB) | the HUD's map window, under Nav2's map ([HUD](hud.md#the-street-map-under-it)) | no: `pixi run get street-map` |
| Search "Tim Hortons" | `ottawa.db`: 606,628 named places from OSM (51 MB) | the HUD's map window ([search](hud.md#search-for-a-place-and-drive-there)) | no: `pixi run get places` |
| Plan a street route | Valhalla (a road router), Ottawa's OSM data | `street_router` (`aav_routes`), on this computer ([Street routes](routes.md)) | no: `pixi run get routes` |

A place reaches the car in one of two ways:

```text
 map window: pick a place ─► DRIVE HERE, CONFIRM ─► /goal_pose (inside Nav2's map only)

 street_router: a destination ─► Valhalla: a street route (lat/lon points)
     ─► a person calls /drivers/nav/route/follow ─► Nav2: FollowGPSWaypoints
     ─► /fromLL: each point to map x/y ─► Nav2 drives point to point
```

> ⚠️ **Nav2 doesn't know where the road is.** The route's points follow the road, but Nav2
> only sees the points, not the road. Between two points it plans across its costmap,
> which today is empty (all free). Staying in the lane is the lane keeper's job
> (`aav_lane_finder`, `aav_lane_geometry`). A lane layer in the costmap would be the next step.

Both ways work in Gazebo (16 Oct); neither has run on the car.

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
| tens of metres | GPS waypoints | one point every few tens of metres | breadcrumbs for Nav2 |
| 200 m | global costmap | 25 cm | a path the car can drive |
| 20 m | local costmap | 10 cm | dodge what the sensors see |
| a few metres | collision_monitor | its stop zone | last-moment stop, whatever Nav2 thinks |
| centimetres | lane keeping | camera pixels | stay in the lane |

## 7. What works, and what's missing

| Part | Status |
|---|---|
| `ekf_local`, `ekf_global`, `navsat_transform` | ✅ in Gazebo and on the bench. ❌ never on the car |
| Nav2 with both costmaps | ✅ in Gazebo and on the bench |
| Radar and cameras in the local costmap | ✅ on the bench |
| Lidar in the local costmap | ✅ the roof VLP-16 through the stock `velodyne` driver, tested against a fake lidar; untested with the real one |
| The real datum | ❌ a placeholder: set it at the test site |
| A street route → Nav2 | ✅ in Gazebo ([Street routes](routes.md)); untested on the car |
| A map of the real test site | ❌ only `empty` and an example lot |
| Nav2 knowing about roads and lanes | ❌ the next big step |

## 8. Try it

| You want to see | Run |
|---|---|
| The two EKFs and the frames | `pixi run sim`, then `pixi run tools/ros.sh ros2 run rqt_tf_tree rqt_tf_tree` |
| The car on the map, live | `pixi run tools/ros.sh ros2 run tf2_ros tf2_echo map base_link` |
| A GPS point as map x/y | `pixi run tools/ros.sh ros2 service call /fromLL robot_localization/srv/FromLL "{ll_point: {latitude: 45.3861, longitude: -75.6959}}"` |
| The costmaps | in RViz (it opens with `pixi run sim`): add `/local_costmap/costmap` and `/global_costmap/costmap` |

More: [Costmaps](costmaps.md), [How it fits](how-it-fits.md), [HUD](hud.md).
