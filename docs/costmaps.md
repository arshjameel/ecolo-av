# Maps and costmaps

A **costmap** is a grid laid on the ground. Every cell holds a cost:
- free (0);
- near something (inflated, 1–252);
- inside something (lethal, 253–254);
- unknown (255).

Nav2's planner searches the grid for a path the car can drive. Its controller follows that
path and slows down near high-cost cells. `nav2_costmap_2d` builds the grids from layers.
The settings are in `ros/drivers/aav_navigation/config/nav2.yaml`.

## The two costmaps

| | Local costmap | Global costmap |
|---|---|---|
| Used by | the controller (MPPI), each step | the planner (SmacPlannerHybrid), once per path |
| Frame | `odom`: smooth, moves with the car | `map`: fixed to the ground by GPS |
| Size | 20 m x 20 m rolling window, centred on the car | the size of the map that `map_server` loads |
| Cell size | 0.10 m (200 x 200 cells) | 0.25 m |
| Updates | 5 Hz | 1 Hz |
| Layers | static (the map), the radar, the cameras, the lidar, inflation | static (the map), the radar, the lidar, inflation |
| Sensors | the radar and lidar where they run; the cameras when switched on (below) | the radar and lidar where they run (since 22 Oct): the planner plans round what they see. `plan_around:=false` for the map only |
| Inflation | 2.0 m, cost scaling 3.0 | 3.0 m, cost scaling 1.0: gentler and wider, so paths keep to the middle of a lane |

A 20 m window means 10 m ahead and behind, about 5 s of driving at 2 m/s, the most this
car is meant to do this year. Make it larger only if the car gets faster.

- **Footprint:** the car's outline, made from `body` in `vehicle.yaml` by the launch
  file. Nav2 checks that it doesn't enter lethal cells.
- **Inflation:** at least the car's circumscribed radius, 1.93 m. This keeps paths away
  from walls rather than scraping them. The planner's is wider (22 Oct): with 2.0 m its
  path passed a building's corner 0.86 m away, and the collision monitor stopped the car
  there ([Path followers](path-followers.md)).

## Obstacles from the sensors

The local costmap has four sensor layers. Each is a stock Nav2 `ObstacleLayer`, switched on
by `navigation.launch.py`.

| Layer | Reads | On | Marks | Clears |
|---|---|---|---|---|
| `obstacle_layer` | `/sensors/radar/points` (in `radar_link`) | `radar:=true`: the bench always; the car where the machine file that runs `navigation` also runs `radar`; never Gazebo (its car has no radar) | each point 0.3-8.4 m from the radar, -0.5 to 2 m high | the cells between the radar and each point |
| `camera_layer` | `/perception/camera_obstacles`, `/perception/camera_view` | `camera_obstacles:=true`: the bench's argument, or the car's `camera_obstacles` part next to `navigation`. Off until the cameras are calibrated (Q22) | where each detected object stands on the road, out to 12 m | everything the camera sees, out to 12 m, before it marks |
| `lidar_layer` | `/sensors/lidar/scan` (from `lidar_scan`, `aav_lidar`) | where the lidar runs: on by default on the car (the Orin) and in Gazebo; `lidar:=true` on the bench | points 0.3 m or more above the road ([below](#body-roll-and-the-lidar-todo-52e)), out to 12 m | along each beam, out to 15 m |
| `ghost_layer` | `/safety/ghost/scan` | `ghost:=true` only ([Ghost obstacles](ghost-obstacles.md)) | the pretend person | only old ghosts |

Why the radar's numbers:
- **8.4 m:** the TI radar's range (`aav_radar_ti` `cfg/base.cfg`).
- **-0.5 to 2 m high:** a radar sees no ground to reject, and its height is poor, so low
  points stay. Above 2 m is over the car (1.5 m tall).
- **Sparse:** a few points per object. Each 5 Hz update uses the last 0.2 s: both radar
  frames, not one.

The camera layer is its own, so its clearing never wipes what the radar marked.
[Perception](perception.md) says how a box becomes a point on the road.

### collision_monitor

Where the radar or the lidar runs, Nav2's command passes through `collision_monitor` on
its way to twist_mux:

```
controller -> velocity_smoother -> /control/cmd_vel_smoothed -> collision_monitor -> /drivers/nav/cmd_vel -> twist_mux
```

| Setting | Value |
|---|---|
| Stop zone | from the front bumper to 1 m ahead, 0.2 m wider than the body each side: made from `vehicle.yaml` by the launch file |
| Why 1 m | at 1 m/s, ego stops the car in 0.25 m; the radar, mux, ego and brakes add about 0.4 m; the rest is for the radar's noise |
| Stops for | 3 points in the zone, from the radar or the lidar (one stray point mustn't stop the car) |
| Also stops | when a sensor it reads is silent for 1 s, or its TF is late |
| Reads | the sensors' own points, not the costmap: it still works if the costmap goes wrong |

- It only guards **Nav2**. A person, a test script or the AI slot reach twist_mux without
  it.
- After a stop it sends zeros for 2 s, then nothing; twist_mux then drops Nav2's slot and
  ego holds the car.
- `collision_monitor:=false` leaves it out, for a test.

### What the car does at an obstacle

On the bench (`pixi run bench-check --obstacle`, 11 October): a box as wide as the lane, 15 m
ahead, Nav2 driving to a goal 30 m away. (A 1 m box first; but the car can weave a metre or
more off its line, N7 below, and once it passed beside the box.)

| Step | What happened |
|---|---|
| The radar sees the box | 8.4 m out; the costmap marks it |
| About 1 m before it | both stop the car, within 0.15-0.7 s of each other: `collision_monitor`'s stop zone (radar points) and Nav2's controller (the costmap). Either can be first |
| The car stands still | 0.42 m before the box, from 1.02 m/s (0.53 m in a second run). With the lane-wide box on `main`: 0.44 m, and 0.26 m with the car starting 8° off the road |
| The box is taken away | `collision_monitor` lets go at once; Nav2's recovery clears the costmap and the car drives on to the goal |

!!! note "Since 22 Oct, Nav2 steers round it"
  The table above is with `plan_around:=false`: the planner sees only the map, so the car
  stops and waits (the stop zone's tests run like that). By default the global costmap
  has the radar and lidar too, so the planner plans round the box where there's room: in
  Gazebo the car passed it 3.9 m clear and never stopped (`pixi run sim-check
  --plan-around`). Where the map leaves no room round it, the car stops as before. On
  the bench the map is empty, so the planner would leave the road: `bench-check
  --obstacle` runs with `plan_around:=false`.

### Try it

| To see | Run |
|---|---|
| The radar and the stop zone | `pixi run bench obstacle:=true rviz:=true`, then a 2D Goal Pose past the box (15 m east) |
| The whole check | `pixi run bench-check --obstacle` (about 4 minutes) |
| A camera detection | `pixi run bench camera_obstacles:=true`, then publish a pretend person (below) |

A pretend person, 0.5 m wide, standing 7 m ahead of the bench camera:

```bash
ros2 topic pub -r 10 /perception/detections vision_msgs/msg/Detection2DArray \
  "{header: {stamp: now, frame_id: camera_left_link}, detections: [{bbox: {center: {position: {x: 320.0, y: 188.8}}, size_x: 24.5, size_y: 100.0}, results: [{hypothesis: {class_id: person, score: 0.9}}]}]}"
```

**You should see** points at x = 8.8 m (1.8 m to the camera, then 7 m) on
`/perception/camera_obstacles`, and lethal cells there in the local costmap. Change
`person` to `stop sign`: nothing.

### Limits

- **What the radar stops seeing stays marked.** A radar only clears along its points, so
  a box taken away stays in the costmap until Nav2's recovery clears it (it does, when the
  car is stuck).
- **The camera assumes a flat road**, and ignores the lens's distortion. At 12 m, a camera
  tilted 1 degree more than `vehicle.yaml` says puts the point 2 m off.
- **A box cut off by the bottom of the picture** is placed at the nearest road the camera
  sees (2.3 m ahead on the bench): the object is at least that close.
- **The numbers are guesses** until the car is measured: the radar's mount (Q1-Q5) and the
  cameras' (Q22).

### Body roll and the lidar (TODO 52e)

`lidar_scan` keeps points **0.3 m or more above the road**, as if the body were level. In a
turn the body leans out on its springs. On the outside of the turn the road then rises in
the car's frame, by about *distance out × tan(roll)*. Far enough out, the road looks like
an obstacle.

```text
   level:   lidar ●───────────────────────  road at 0 m: ignored
   rolled:  lidar ●─────────╲               the road 10 m out sits 0.23 m "up"
                              ╲___ road     (1.33°): still under 0.3 m
```

The highest road point the lidar sees within the costmap's **12 m** (VLP-16 on the roof,
1.8 m up; `aav_lidar` `ground.py`):

| Roll | When | Highest road point | An obstacle? |
|---|---|---|---|
| 0.23° | 1 m/s, full lock (today) | 0.04 m | no |
| 1.33° | ego's limit: 2 m/s² sideways, with the springs' overshoot | 0.23 m, 10 m out | no |
| **1.46°** | **the limit** | 0.30 m | the edge |
| 1.95° | 3 m/s on full lock, if ego didn't slow down | 0.39 m, 11.5 m out | **yes** |

**The verdict: safe up to 4 m/s.** ego slows the car in turns to 2 m/s² sideways
(`max_lateral_accel`), which holds the roll to 1.33° at most, even on a sudden full lock.
At 5 m/s a sudden full lock still overshoots: the wheels turn faster than ego can brake
(2.4° of roll). The springs are guesses: softer than about 7400 N/m a corner and the margin
is gone. `tools/tests/test_lidar_roll.py` checks all of this together, and fails if
someone raises `max_lateral_accel` or the 12 m, or lowers the 0.3 m.

Gazebo has no suspension, so it can't show this. The scan itself (out to 30 m) has road
in it from about 0.6° of roll, but nothing marks the costmap past 12 m.

**Don't level the cloud with the IMU's roll.** `imu_filter_madgwick` finds "down" from
the accelerometer, and in a turn the sideways push looks like tilt: 2 m/s² reads as
11.5° of roll, about 9 times the real 1.3°. Levelling would need that push taken out
(speed × turn rate) first.

## What each phase uses

| Phase | Map | What it tests |
|---|---|---|
| 0, 1 (joystick, scripted) | none | Nav2 isn't running |
| 2: Nav2 on an empty costmap | `maps/empty.yaml`: 200 m x 200 m, all free, 0.25 m cells, centred on the datum | planning and following within the turning limits, GPS goals |
| 3: Nav2 on a pre-rendered costmap | your test site, drawn as a map, e.g. `maps/example_lot.yaml` (an 80 m x 60 m made-up lot) | planning around known walls and buildings |
| perception | phase 2 or 3, plus the radar (and the cameras, after Q22) | the obstacle layers and `collision_monitor`: on the bench now |

## Map files

A Nav2 map is an image (PGM or PNG) plus a YAML file that says how to read it.
`ros/drivers/aav_navigation/maps/README.md` covers the format line by line. In short:
- one pixel is one cell;
- white is free, black is blocked, grey is unknown;
- `resolution` is metres per pixel;
- `origin` is the map-frame position of the bottom-left pixel.

Don't paint maps by hand. Write a short spec and generate them:

```bash
tools/make_map.py ros/drivers/aav_navigation/maps/example_lot.spec.yaml
```

A spec lists:
- the size in metres east and north;
- the origin;
- blocked rectangles and free rectangles.

`tools/make_map.py --help` (or the top of the file) shows the format. It needs only
Python and PyYAML, not ROS.

## Your first map, in ten minutes

1. Open `ros/drivers/aav_navigation/maps/example_lot.spec.yaml`, and add a box under
   `blocked:`:

    ```yaml
      - [15, -3, 18, 3]       # a box across the drive, to plan around
    ```

2. Make the map again:

    ```bash
    pixi run python tools/make_map.py ros/drivers/aav_navigation/maps/example_lot.spec.yaml
    pixi run build
    ```

   It says `wrote example_lot.pgm/.yaml: 320 x 240 px, 80 x 60 m`.
3. Start Gazebo with the test lot: `pixi run sim world:=test_lot`. On a Mac, add
   `gui:=false`, then run `pixi run gz-window` and `pixi run rviz` in two more terminals.
4. In RViz, send a **2D Goal Pose** at about (30, 0): 30 m straight ahead, past the box.

**You should see** the planned path bend around your box. In the test on 10 October it
passed at y = 4.4 m, beside a box that ends at y = 3 m.

5. Put the map back: `git checkout ros/drivers/aav_navigation/maps/`.

!!! note "Two things this teaches"
  - The box is only in the map, not in Gazebo's world. Nav2 avoids it because the map
    says so: that's how phase 3 works.
  - Leave room in front of an obstacle. The car turns no tighter than 2.86 m, and the
    costmap keeps 2 m of clearance, so a box 5 m ahead can't be swerved around. (The
    test tried it: Nav2 reported "collision ahead".)

!!! warning "Known problem, 10 October: the car doesn't finish this drive"
  In Gazebo the car follows the curve around the box, but overshoots the goal and ends
  about 6 m off. Nav2 then can't plan a way back, because the car only drives forward,
  so the goal fails. It's open work for the navigation team (N7, the lookahead). A
  straight 8 m goal on the empty map succeeds.

### N7: why Nav2's turn command flips (11 October)

In Gazebo, on the empty map, each run on a fresh simulator. This was Regulated Pure
Pursuit, before MPPI became the default (22 Oct; [Path followers](path-followers.md)):

| Turn-sign flips a second | Straight goals (8 m, 25 m) | The curve to (15, 8) |
|---|---|---|
| Lookahead 1.5 m (then) | 0.86-0.99 | 0.48-0.71 |
| Lookahead 3 m (tried) | 0.85-1.44 | 0.25-0.52 |

- **Not the lookahead:** a longer one didn't help on straight drives, so it stayed 1.5 m.
  Both reached every straight goal; each failed the curve once.
- **The GPS:** the flips come in steps, five times a second, with the fixes. One step in
  ten turns the global EKF's heading (`map -> odom`) by more than 4 degrees, and the whole
  path swings with it.
- **Tried, not kept:** with `ekf_global` trusting GPS less, one step in ten turned it by
  more than 0.6 degrees, and the flips fell to 0.06-0.5 a second; but one goal then
  overshot and looped for 45 s.
- **Next:** tune `ekf_global` against the real receiver's noise (Q18).

## Where the map sits on the Earth

The map frame is pinned to the ground by the **datum** in
`ros/localization/aav_localization/config/navsat.yaml`:
- map (0, 0) is the datum's latitude and longitude;
- x points east and y points north.

So a point 20 m east and 5 m north of the datum is (20, 5) on the map, in Nav2 goals and
in RViz. The datum in the file now is a placeholder near Carleton.

To draw the real test site:
1. Pick the datum: a corner of the lot, or the start spot. Survey it with the GPS
   (average a few minutes of fixes) and put its latitude and longitude in `navsat.yaml`.
2. Measure the drivable area in metres east and north of it, with a tape, or by measuring
   on satellite imagery from the datum.
3. Write the spec: the size, then the blocked rectangles (buildings, curbs, parked cars).
4. Build the map and drive phase 3 in Gazebo first (`world:=test_lot` pairs with
   `example_lot`).

## A costmap is not a road map

The global costmap only says which cells may be driven on. Roads, junctions and routing
across a city are a separate kind of map: a road graph. Two tools handle that:
- **A street router** (Valhalla, or OSRM). It turns a destination into a route of
  latitude/longitude points. Those points become a `follow_gps_waypoints` goal for
  Nav2. Our offline router, and the node that turns its routes into waypoints
  (`street_router`): [Street routes](routes.md).
- **Nav2's Route Server** (`nav2_route`). It's released for Jazzy, in Nav2 1.3.13 as of
  October 2026, and follows a hand-made graph of lanes or paths. Consider it if the test site becomes a set of fixed lanes.

For this year's goals (an empty costmap, then a pre-rendered test site) a grid map plus
GPS waypoints is enough.
