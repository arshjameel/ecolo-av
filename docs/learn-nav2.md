# Navigation from zero

**Navigation** turns a goal into driving: plan a path, then steer along it. On our car it
is **just one more driver**: it proposes a speed and a turn rate, like the joystick does.

Our stack (`stack:=ours`) does it, every part in Rust. Each part is a port of a part of
**Nav2**, ROS's stock navigation, which drove the simulated car from 10 Oct to 17 Nov
([Nav2, as it was](history/nav2-as-it-was.md)). The ideas below are Nav2's; the code is ours.

**You'll learn:**
- the parts: goal, navigator, planner, path follower, costmaps, smoother, stop zone;
- how a lattice planner finds paths a car can really drive;
- how pure pursuit steers, with one example in numbers;
- what happens when things go wrong: recoveries and the stop zone;
- how GPS waypoints become goals;
- our settings, and where they live.

Costmaps have their own page: [Costmaps](costmaps.md). The big picture of maps and
GPS is in [Maps and position](maps-and-position.md).

## The picture

```text
goal ─► navigator ─► planner ─► path follower ─► smoother ─► stop zone ─► /drivers/nav/cmd_vel
        (replan,     (lattice)   (mppi_lag)                  (radar,        ─► twist_mux ─► ego
         recover)        └── costmaps ──┘                     lidar)
```

| Part | Job | Ours (package) |
|---|---|---|
| **Goal** | where to go: a pose in the `map` frame | RViz's 2D Goal Pose, an action call, or GPS waypoints from `street_router` or the language tools |
| **Navigator** | the boss: plan, follow, replan, recover | `aav_navigator` ([page](own-navigator.md)) |
| **Planner** | finds a path from here to the goal | `aav_planner`: the State Lattice; Hybrid-A* with `planner_search:=hybrid` ([page](own-planner.md)) |
| **Path follower** | steers along that path, 20 Hz | `aav_mppi_lag`: MPPI that knows the actuators' lag ([page](mppi-lag.md)); trials with `tracker:=` ([Trials](trials.md)) |
| **Costmaps** | what is in the way | `aav_costmap` ([page](own-costmap.md)) |
| **Smoother** | caps speed, acceleration and the steering's rate | `aav_smoother` ([page](own-smoother.md)) |
| **Stop zone** | last-moment stop for radar or lidar points ahead | `aav_stop_zone`, a safety lock |
| **Out** | `/drivers/nav/cmd_vel` → `twist_mux` (priority 20) → ego | |

Navigation never talks to the wheels. ego turns its turn rate into a steering angle
([Learn: control](learn-control.md)).

## 1. The planner: a lattice of car moves

**A\*** searches a grid for the cheapest way from A to B. A plain grid search gives
paths with sharp corners: fine for a robot that spins on the spot, not for a car.

A **lattice planner** searches in (x, y, **heading**), and only joins moves a car can
make: straight on, and arcs no tighter than the car's turning radius (**2.87 m**, from
`vehicle.yaml`). The moves are made once, ahead of time, into a file
(`config/lattice_ackermann*.json`). So every path it returns is one our car can follow.

**Hybrid-A\*** (`planner_search:=hybrid`) is the same idea, with the arcs worked out as
it searches.

| Setting (`nav2.yaml`'s lattice block) | Ours | Means |
|---|---|---|
| `allow_reverse_expansion` | true with `reversing:=true` | forward and back ([Reversing](#reversing)) |
| `reverse_penalty` | 2.0 | a reverse metre costs twice a forward one |
| `cusp_penalty` | 4 m | each change of gear costs as much as 4 m of driving ([own planner](own-planner.md)) |
| headings | 16 (32 with `lattice_headings:=32`) | the directions a path may point in |

## 2. The path follower

Our follower is **MPPI**: it tries 2000 futures through a model of the car, and picks a
blend of the best ([MPPI with lag](mppi-lag.md)). **Pure pursuit** is the easiest way in,
and `tracker:=pure_pursuit` runs it ([Trials](trials.md)).

**Pure pursuit** is how you steer a bike: look at a point on the path a little ahead (the
**carrot**), and turn so you'd arc through it.

```text
            carrot ●   (on the path, the lookahead distance ahead)
                  ╱
                 ╱  arc through it
           car ▲
```

The arc's curvature = 2 × (how far the carrot is to the side) ÷ (lookahead)².

**Example:** the carrot is 2.0 m ahead and 0.4 m to the left.

| Step | Sum | Result |
|---|---|---|
| Curvature | 2 × 0.4 ÷ 2.0² | 0.2 per m (a 5 m circle) |
| Turn rate at 1.0 m/s | 1.0 × 0.2 | 0.2 rad/s → `/drivers/nav/cmd_vel` |
| ego's steering angle | atan(1.57 × 0.2 ÷ 1.0) | **0.304 rad** |

**Done** when within 1.0 m and 0.75 rad (43°) of the goal: no tighter than GPS without RTK
can tell (22 Oct; tighter, the car looped to fix misses it couldn't see). **Stuck** if in
10 s it neither moves 0.5 m nor turns 0.5 rad (29°): rocking back and forth in a
three-point turn still counts.

## 3. Costmaps, in one line each

- **Global:** for the planner, in `map`, the map file's size.
- **Local:** for the follower, in `odom`, 20 m × 20 m around the car.

Sizes, layers, sensors and inflation: [Costmaps](costmaps.md#the-two-costmaps).

## 4. The navigator: the boss

The navigator is one state machine ([our navigator](own-navigator.md)). It does what
Nav2's behaviour trees did for a car:

```text
goal ─► plan ─► follow ─► plan again, at most every second
          │        │        (not in the last 8 m: keep the path, unless blocked or 20 s old)
          ▼        ▼
   clear global  clear local      ◀── a failure a recovery can fix
          └────────┴─► in turn: clear both ─► wait 5 s ─► back up 1 m (reversing only)
                       up to 6 times, then give up
```

| Made for a car | Why |
|---|---|
| **No spin** | a car can't turn on the spot |
| **Back up only with reversing** | 1 m at 0.25 m/s, after the wait; only when something sees behind the car |
| **Keep the path for the last 8 m** | a path planned a few metres from the goal is often a loop for a car: the car circled goals ([TODO 17](path-followers.md#circling-a-goal-found-and-fixed-29-october-todo-17)) |

## Reversing

One switch, `reversing:=`. **On everywhere: Ravensim, the bench, and the car since
1 Nov** (Ben: "reverse and uturn should be in"). The wheel sensors can't tell direction
yet (H5), so the speed's sign is the last command's: wrong only if the car is pushed or
rolls back ([tests](tests.md), 4.25). On the car it works only where something sees
behind it: the roof lidar (the radar faces forward; the rear camera, later). Without one,
`car.launch.py` keeps navigation forward only and says why (`vehicle.blind_behind`).

| | `reversing:=false` | `reversing:=true` (the default everywhere) |
|---|---|---|
| Planner | forward only | forward and back; a reverse metre costs 2x, a change of gear 4 m |
| MPPI's `vx_min` | 0 | -0.5 m/s |
| Smoother's lowest speed | 0 | -0.5 m/s |
| Navigator's recoveries | no back-up | back up 1 m at 0.25 m/s |
| Stop zone | the box ahead | the box ahead going forward, a box behind reversing ([5](#5-the-stop-zone-the-last-word)) |

The 0.5 m/s is `max_reverse_speed` in `vehicle.yaml`: the one reverse speed for the whole car.
ego and the spine (`MAX_REVERSE_SPEED` in `spine-core`, also for the RC transmitter) cap
reversing at the same number, whoever drives.

| Where it is set | How |
|---|---|
| `vehicle.yaml` | `max_reverse_speed: 0.50` |
| `aav_description/vehicle.py` | `nav2_rewrites(...)` and `nav2_tuned(...)` write it into a copy of `nav2.yaml`, the settings our nodes read |
| `car.launch.py`, `sim.launch.py`, `bench.launch.py` | `reversing:=true` by default in each; on the car it needs the lidar running, or it stays forward only |
| Ravensim check | `pixi run sim-check --rear-obstacle`: backing up, the car stops for a post behind it |
| Tests | `test_vehicle.py` (the values, and that each rewrite lands on a real key) |

People can always reverse (joystick, keyboard, RC): this switch is navigation's only.

## 5. The stop zone: the last word

`aav_stop_zone` sits between the smoother and `twist_mux`, and reads the **radar's and the
lidar's own points**, not the costmap. So it still stops the car if the planner, the
follower or the costmap goes wrong. It is a port of Nav2's `collision_monitor`, which it
replaced on 17 Nov after a shadow run ([the swap](history/nav2-as-it-was.md#the-swap-of-the-stop-zone)).

| Setting | Ours |
|---|---|
| Stop zone | going forward (or standing): a box from the front bumper to 1 m ahead. Reversing: a box from the rear bumper to 1 m behind. Each 0.2 m wider than the car each side. The command's speed picks the box |
| Behind the car | only the lidar sees there (all round), not the radar. The rear camera's obstacles: later (TODO 25) |
| Points needed | 3 (one stray point mustn't stop the car) |
| Radar or lidar silent for | 1.0 s → stop |
| Guards | **only navigation**. The joystick and keyboard reach `twist_mux` without it |

It runs with the radar or the lidar: on the bench, on the car, and in Ravensim.

When it stops the car backing up, the follower may pull forward and try again; each
try stops at the box. Nothing is hit, but the car rocks until the way is clear.

## 6. GPS waypoints

```text
street route (lat/lon points) ─► follow_gps_waypoints ─► each to map x, y (the datum) ─► one goal after another
```

- `street_router` and the language tools send the route; the navigator serves
  `/follow_gps_waypoints` (`nav2_msgs/action/FollowGPSWaypoints`: Nav2's name and message,
  so no sender changed).
- It turns each point into map x, y from the datum ([Learn: GPS](learn-gps.md)).
- The planner doesn't know where the road is: it plans across its costmap between points.
  Staying in the lane is the lane planner's (`path:=lane`) and the lane keeper's job.

## How well it works

| Where | Result |
|---|---|
| Ravensim | the league's scores ([The league](league.md)) |
| Bench | the whole car graph with stand-ins (`pixi run bench-check`) |
| The car | never run yet |
| A street route → GPS waypoints | ✅ in the simulator ([Street routes](routes.md)); untested on the car |

## Try it

| You want to | Run |
|---|---|
| Start Ravensim with our stack and RViz | `pixi run sim` |
| Send a goal by clicking | in RViz: **2D Goal Pose**, then click and drag on the map |
| Send a goal by command, 8 m ahead | `pixi run tools/ros.sh ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0}, orientation: {w: 1.0}}}}"` |
| Send one GPS waypoint (about 8 m east, 11 m north) | `pixi run tools/ros.sh ros2 action send_goal /follow_gps_waypoints nav2_msgs/action/FollowGPSWaypoints "{gps_poses: [{position: {latitude: 45.3861, longitude: -75.6959}, orientation: {w: 1.0}}]}"` |
| See the plan | `pixi run tools/ros.sh ros2 topic echo --once /plan` |
| See navigation's command | `pixi run tools/ros.sh ros2 topic echo /drivers/nav/cmd_vel` |
| Watch the stop zone stop the car | `pixi run sim-check --obstacle` |

## Where this lives in the code

| What | Where |
|---|---|
| The settings our nodes read (in Nav2's layout) | `ros/drivers/aav_navigation/config/nav2.yaml` |
| Which nodes start; radar and lidar switches | `ros/drivers/aav_navigation/launch/navigation.launch.py` |
| Maps | `ros/drivers/aav_navigation/maps/` (`empty`, `example_lot`) |
| Footprint, turning radius, stop zone from the car's size | `ros/bringup/aav_description/aav_description/vehicle.py` |
| Navigation's place among the drivers | `ros/command_mux/aav_command_mux/config/twist_mux.yaml` |
| Costmaps in detail; draw a map | [Costmaps](costmaps.md) |
