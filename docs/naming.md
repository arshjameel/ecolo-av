# Topic names by area (Done, 15 October)

Ben, 14 Oct: "Maybe we need more structure like /localization/<command> and
/drivers/<command> /ego/<command> ... Something that isolates messages into their area."

**Status: Done (15 Oct).** Decided by Ben on 15 Oct ("The drivers all need to be in a
/drivers/keybaord, etc type stuff as an example."), and renamed everywhere on the branch
`wt/topic-areas`, now on `main`. `tools/tests/test_topic_areas.py` keeps it that way.

## The rule

The first part of every name is the **area**: the layer of the picture it comes from. Most
match a folder under `ros/` (the `/sensors/` topics come from `ros/sources/`), plus
`/system/` for the whole run.

```text
/<area>/<device or node>/<thing>
```

| Area | Holds | Who publishes |
|---|---|---|
| `/sensors/` | raw sensor data | the drivers on the Pis and the Orins |
| `/vehicle/` | what the car says about itself | the backend (spine, legacy, Gazebo, bench) |
| `/can/` | raw CAN frames | the socketcan bridge |
| `/localization/` | where the car is | the EKFs, navsat_transform |
| `/perception/` | what the car sees | the detectors, the lane finder, radar clusters |
| `/drivers/` | one command per driver | joystick, RC, keyboard, tests, scripted, replay, go-to, Nav2, AI |
| `/control/` | the chosen command; ego's command for one target, `/control/<target>/actuator_cmd` | twist_mux, the smoother, the output mux (23 Oct) |
| `/ego/` | the command for the wheels | ego |
| `/safety/` | e-stop, collision monitor, the sensor watchdog's lock, the output mux's state, ghost obstacles | the HUD button, collision_monitor, sensor_watchdog, output_mux, ghost_obstacle |
| `/hud/` | what only the screen needs | the HUD |
| `/system/` | what kind of run this is: `/system/source` (real, sim, bench, replay); the sensor mux's check, `/system/sensor_mux/state` | the backends, the bench, a replay (added 15 Oct, `full-simulation.md`); the sensor mux (23 Oct) |

Global names stay global, because every ROS tool expects them: `/tf`, `/tf_static`,
`/clock`, `/robot_description`, `/joint_states`, `/diagnostics`, `/rosout`,
`/parameter_events`. Nav2 keeps its own names for now: `/map`, `/plan`, `/goal_pose`, its
actions, `/cmd_vel_nav` and the costmaps' topics.

## How the drivers get their names

| Driver | How | Topics |
|---|---|---|
| joystick | `joy_node`, `joy_alive`, `teleop_twist_joy_node`, `joy_teleop` run in the namespace `/drivers/joy` | `/drivers/joy/joy`, `/drivers/joy/joy_checked`, `/drivers/joy/alive`, `/drivers/joy/cmd_vel` |
| RC | `rc_driver` (`aav_rc`) | `/drivers/rc/cmd_vel` |
| keyboard | `tools/keyboard.sh` runs `teleop_twist_keyboard` in the namespace `/drivers/keyboard` | `/drivers/keyboard/cmd_vel` |
| tests | `actuator_test` publishes `drivers/tests/cmd_vel` | `/drivers/tests/cmd_vel` |
| scripted | `scripted_driver` publishes `drivers/scripted/cmd_vel` | `/drivers/scripted/cmd_vel` |
| drive to point | `point_driver` (`aav_drive_to_point`) | `/drivers/point/cmd_vel` |
| Nav2 | `velocity_smoother` or `collision_monitor`, remapped | `/drivers/nav/cmd_vel` |
| AI | `control_interface` (lane keeping) | `/drivers/ai/cmd_vel` |
| replay | `tools/replay_commands.sh` | `/drivers/replay/cmd_vel` |

A namespace makes the relative names fall into place, with no remaps. The joystick's
config files name their nodes `/**/joy_node` and so on, so they match in any namespace.

## Old name → new name

| Was | Now |
|---|---|
| `/gps/fix` | `/sensors/gps/fix` |
| `/gps/fix_velocity` | `/sensors/gps/fix_velocity` |
| `/gps/vel`, `/gps/time_reference` (NMEA) | `/sensors/gps/vel`, `/sensors/gps/time_reference` |
| `/imu/data_raw`, `/imu/mag` | `/sensors/imu/data_raw`, `/sensors/imu/mag` |
| `/imu/data` (after Madgwick) | `/sensors/imu/data` |
| `/camera/image_raw` (and `/compressed`) | `/sensors/camera/front_left/image_raw` (and `/compressed`) |
| `/camera/camera_info` | `/sensors/camera/front_left/camera_info` |
| `/camera/right/...` | `/sensors/camera/front_right/...` |
| (none yet) | `/sensors/camera/rear/...` |
| (none yet: the lidar) | `/sensors/lidar/points` |
| `/radar/points` | `/sensors/radar/points` |
| `/battery_status` | `/vehicle/battery` |
| `/battery_monitor_heartbeat` | `/vehicle/battery/heartbeat` |
| `/vehicle/odom`, `/vehicle/status`, `/vehicle/joint_states` | unchanged |
| (new: each wheel's speed: Gazebo, and the CAN wheel nodes) | `/vehicle/wheels` (one topic for both) |
| `/bench/truth`, `/bench/hw_estop`, `/bench/driver_faults` | `/vehicle/bench/truth`, `/vehicle/bench/hw_estop`, `/vehicle/bench/driver_faults` |
| `/to_can_bus`, `/from_can_bus` | `/can/tx`, `/can/rx` |
| `/odometry/local` | `/localization/odom/local` |
| `/odometry/global` | `/localization/odom/global` |
| `/odometry/gps` | `/localization/gps/odom` |
| `/gps/filtered` | `/localization/gps/fix` |
| `/radar/clusters`, `/radar/cluster_boxes` | `/perception/radar/clusters`, `/perception/radar/cluster_boxes` |
| `/lane_geometry`, `/lane_geometry/image` | `/perception/lane/geometry`, `/perception/lane/image` |
| `/lane_geometry_status`, `/vehicle_state` (lane keeping's) | `/drivers/ai/lane_status`, `/drivers/ai/lane_state` |
| `/perception/stop` | `/perception/stop_sign` |
| `/perception/detections`, `/perception/camera_obstacles`, `/perception/camera_view` | unchanged |
| `/joy` | `/drivers/joy/joy` |
| `/cmd_vel/joy`, `/cmd_vel/scripted`, `/cmd_vel/replay`, `/cmd_vel/nav`, `/cmd_vel/ai` | `/drivers/joy/cmd_vel`, `/drivers/scripted/cmd_vel`, ... |
| `/cmd_vel/remote` (it was the keyboard) | `/drivers/keyboard/cmd_vel` (twist_mux's slot `keyboard`) |
| `/cmd_vel` (twist_mux out) | `/control/cmd_vel` |
| `/cmd_vel_smoothed` | `/control/cmd_vel_smoothed` |
| `/ego/actuator_cmd` | unchanged; since 23 Oct the backends read `/control/<target>/actuator_cmd`, from the output mux ([Sources and targets](sources-and-targets.md)) |
| `/estop` | `/safety/estop` |
| `/collision_monitor_state` | `/safety/collision_monitor/state` |
| `/collision_monitor/stop_zone`, `.../collision_points_marker` | `/safety/collision_monitor/stop_zone`, `.../collision_points_marker` |

Changed from the proposal, once the code showed it:

- **`/drivers/joy/joy`, not `/drivers/joy/raw`:** it is what the namespace gives `joy_node`
  by itself, with no remap.
- **Lane keeping's two text topics go under `/drivers/ai/`:** `control_interface`, the AI
  slot's driver, publishes them, not the lane finder.
- **The bench's own topics go under `/vehicle/bench/`:** the bench stands in for the car.

## Left as they were

| Name | Why |
|---|---|
| `/fromLL`, `/toLL`, `/datum` (navsat_transform's services) | Nav2's `waypoint_follower` calls `/fromLL` by its absolute name. Moving them needs a remap on two nodes, one of them inside Nav2: not a clean remap |
| `/spine/arm`, `/scripted_drive`, `/ekf_*/set_pose` | services and actions, not topics: the rule is for topics |
| `/gazebo/cmd_vel`, `/gazebo/odom`, `/gazebo/imu`, `/gazebo/navsat` | Gazebo's own side of `gazebo_adapter` and its sensor nodes, like `/turtle1/...` for turtlesim; not in the contract |
| `/map`, `/plan`, `/goal_pose`, `/cmd_vel_nav`, the costmaps | Nav2's own, for now |
| Bags recorded before 15 Oct | they hold the old names: play them with `--remap` (for example `/cmd_vel:=/control/cmd_vel`) |

## Why the two EKFs don't talk

The local EKF doesn't feed the global one, and that is on purpose: robot_localization's
standard setup. Each one fuses the raw sensors itself.

| | `ekf_local` | `ekf_global` |
|---|---|---|
| Uses | wheel speed, IMU | wheel speed, IMU, **GPS** |
| Gives | a smooth position that never jumps | a true position that may jump |
| Publishes | `odom → base_link` | `map → odom` |
| Who uses it | Nav2's controller (steering) | Nav2's planner, the HUD's map |

If the global EKF used the local one's output, the same wheel and IMU data would count
twice, and it would be too sure of itself. Feeding both from the raw sensors avoids that.

They do meet, but in **TF**, not on a topic: `ekf_global` publishes only the correction
`map → odom`, worked out on top of `ekf_local`'s `odom → base_link`. So `rqt_graph` shows
no line between them; `ros2 run rqt_tf_tree rqt_tf_tree` shows the chain. The launch
file's first line said "the local EKF feeds the global EKF"; it now says this (14 Oct).
