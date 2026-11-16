# aav_behaviour

A trial, off by default: the behaviour states, and a speed cap on Nav2 from the stopping
law. It knows **why** the car should slow, and slows it. It **can't stop** it: a lock or
Nav2's goal check makes every final stop. The page: `docs/stopping.md`; the design:
`docs/designs/2026-11-05_behaviour-and-speed.md`.

```
 DRIVE ─► APPROACH ─► STOPPED ─► WAIT ─► GO ─► DRIVE
  (no cap)  (cap falls)  (lock on,  (lock on)  (cap rises)
                         still)        │
                                       └─► ASK (a person is told)
```

## Overview

| Stop for | `left` (m) | Final stop |
|---|---|---|
| a stop sign | camera range to the sign − the watcher's 4.7 m | the `stop_sign` lock |
| an obstacle in the lane | lidar scan: nearest point ahead − bumper − stop zone | collision_monitor, then the `stop_zone` lock |
| the fence | along `/plan` to where depth < `near_depth` | the geofence lock |
| the goal | along `/plan` to its end | Nav2's goal check |

The cap is `v_cap(left)` (`aav_description` `stop_law.py`), the lowest that binds;
0.0 (no limit) or between the floor (0.3 m/s) and Nav2's top speed, never in between.

```sh
pixi run sim world:=road perception:=true behaviour:=true
ros2 topic echo /drivers/nav/behaviour/state
```

## Interface

**`behaviour`** (`behaviour_node.py`), started by `aav_navigation`'s
`navigation.launch.py behaviour:=true` (`sim.launch.py behaviour:=true` passes it on).
That also sets controller_server's `speed_limit_topic` to `/drivers/nav/speed_limit`.

| In / out | Topic | Type |
|---|---|---|
| in | `/perception/stop_sign/distance` | `geometry_msgs/PointStamped` (camera frame, kept in `odom` by `/tf`) |
| in | `/perception/stop_sign`, `/safety/stop_hold/lock`, `/safety/geofence/lock`, `/safety/sensor_watchdog/lock`, `/safety/estop` | `std_msgs/Bool`: the locks |
| in | `/safety/geofence/state`, `/safety/geofence/fence` | `std_msgs/String` (NEAR), `geometry_msgs/PolygonStamped` |
| in | `/plan`, `/localization/odom/local`, `/localization/odom/global` | `nav_msgs/Path`, `nav_msgs/Odometry` |
| in | `/drivers/point/plan` | `nav_msgs/Path` (latched): a drive to a point's line; while it runs it is the plan (`active_plan`) |
| in | `/sensors/lidar/scan` | `sensor_msgs/LaserScan` (not with `voxels:=true`) |
| in | `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` |
| out | `/drivers/nav/speed_limit` | `nav2_msgs/SpeedLimit`, 20 Hz, absolute m/s |
| out | `/drivers/nav/behaviour/state` | `diagnostic_msgs/DiagnosticStatus`, 5 Hz: state, reason, cap, left, warnings |
| in | `/world/things` | `aav_interfaces/Things` (odom), with `things`: what moves ahead |
| out | `/world/road_ahead` | `aav_interfaces/RoadAhead` (map), 10 Hz: `/plan`'s next 40 m from the car, its stops, its end, the speed limit by the stopping law ([the whole car](../../../docs/designs/2026-11-14_the-whole-car.md#the-road-ahead)). With `lane_map`: the line is the car's lane's centre along the route, with its edges |

Parameters (from `vehicle.behaviour_params`: nav2.yaml, vehicle.yaml, ego.yaml):

| Name | Default | Means |
|---|---|---|
| `base_speed` | MPPI's `vx_max` (or `sim_speed`) | the top cap |
| `floor`, `accel`, `fall` | 0.3 m/s, 0.5 m/s², 2.0 m/s² | lowest cap; rise in GO; fastest fall |
| `ask_after` | 30 s | a non-sign lock this long: ASK |
| `release_hold` | 1 s | an obstacle must stay gone this long before APPROACH lets go of it (TODO 189: no flapping at the lane's edge); slowing is never held |
| `max_deceleration`, `max_jerk`, `speed_loop` | ego's; the launch's `speed_loop:=` (`default`) | the stopping law's numbers; the speed loop the car runs (TODO 163) |
| `front`, `half_width`, `stop_gap` | the body and the stop zone | the lane ahead |
| `stop_distance_m`, `near_depth`, `plan_timeout` | 4.7 m, 2.5 m, 25 s | the to-pose tree keeps a path up to 20 s near the goal |
| `road_rate_hz`, `road_length`, `road_spacing` | 10 Hz, 40 m, 0.5 m | the road ahead: how often, how far, how fine |
| `lane_map` | empty | a `lanes.yaml` (`aav_description` `lane_map.py`): the road ahead's line on the right lane's centre, with its hard and soft edges. Empty: the line is `/plan`'s, edges NaN. `sim.launch.py lane_map:=world` gives the world's own |
| `things`, `things_timeout` | false, 1.0 s | true (`navigation.launch.py things:=true`): read `/world/things` (`aav_things`); an obstacle stop only for a thing that moves (0.3 m/s or more), in the lane or walking into it before the car gets there (`Crossing`, TODO 194); a parked thing is the planner's to go round (TODO 162). The stop zone's lock still stops the car for anything. A list older than `things_timeout`: the scan's rule, where the scan runs |

| Module | No ROS? | Holds |
|---|---|---|
| `behaviour.py` | yes | `Behaviour.step(World) -> Decision`, `Settings`, `Stop`; `active_plan`, `path_left`, `along_until`, `lane_ahead`, `diagnostic` |
| `behaviour_node.py` | no | the node; the road ahead's maths is `aav_description`'s `road_ahead.py`, the lane's is `lane_map.py` |

## Invariants

- Only ever slows Nav2. Never clears a lock, a latch or the e-stop; no `cmd_vel`.
- A cap is 0.0 or in [floor, base]: Nav2 reads 0.0 as **no limit**.
- The sign's cap never rises in APPROACH; any cap falls no faster than `fall`.
- Not built yet (the design's later steps): the `patient` progress checker; ASK's reply
  (`/hud/ask/answer`); the trial trackers reading the cap; the car (refused in `open`).

## Side Effects

- Publishes the two topics above. No files, no hardware.

## Verification

```sh
cd ros/drivers/aav_behaviour && python3 -m pytest -q   # every transition (tools/test.sh)
pixi run sim-check --behaviour                         # Gazebo: sign, goal, obstacle
```
