# aav_description

## Overview

The car's body: its measurements, its URDF and its frames. `config/vehicle.yaml` is
**the only place car measurements live**. Every other package reads them from here,
through the Python helper `aav_description.vehicle`.

## Interface

### Launch files

| File | Starts |
|---|---|
| `description.launch.py [sim:=] [use_sim_time:=] [lidar:=] [joint_state_publisher:=true]` | `robot_state_publisher` from the URDF; `joint_state_publisher`, which merges `/vehicle/joint_states` |
| `view.launch.py` (`pixi run view`) | the model in RViz, with joint sliders |

### Topics

| Topic | Type | Who |
|---|---|---|
| `/robot_description` | std_msgs/String | `robot_state_publisher` publishes |
| `/tf_static`, `/tf` | tf2_msgs/TFMessage | `robot_state_publisher` publishes |
| `/vehicle/joint_states` | sensor_msgs/JointState | `joint_state_publisher` subscribes (from `spine_bridge`) |
| `/joint_states` | sensor_msgs/JointState | `joint_state_publisher` publishes (off in Gazebo) |

### `config/vehicle.yaml`

| Section | Holds |
|---|---|
| `vehicle` | wheelbase, track, wheel size (m), `max_steering_angle` (rad), `max_reverse_speed` (m/s), body outline, mass (kg), centre of mass, tyre grip, encoder counts, `speed_wheels` (which wheels `/vehicle/odom` measures: front today, TODO 49b) |
| `actuators` | the actuator model for Gazebo: accel, brake, lags, steering rate. Also read by ego (`max_accel`, `max_brake`, via `ego_params`), `aav_drive_to_point`'s launch (`max_brake`) and `carla_adapter` (`max_accel`). **Every value is a guess** |
| `sensors` | each sensor's mount (`xyz` m, `rpy` rad) from `base_link`: IMU, GPS, radar, three cameras, lidar |

Many values are still `TODO measure` (docs/car_measurements.md).

### `aav_description.vehicle` (no ROS)

| Function | Returns |
|---|---|
| `config_path()` | where the installed `vehicle.yaml` is (ament_index) |
| `load(path=None)` | the YAML file as a dict |
| `min_turning_radius(v)` | `base_link`'s radius at full lock (m) |
| `wheel_angles(v, centre)`, `centre_angle(v, inner, outer)` | Ackermann: each front wheel from the bicycle angle, and back |
| `gazebo_steering_limit(v)` | the limit Gazebo's AckermannSteering needs to reach `max_steering_angle` |
| `footprint(v)`, `stop_zone_points(v)`, `rear_stop_zone_points(v)` | Nav2's footprint and collision_monitor zones |
| `footprint_points(v)`, `polygon(points)` | the body outline as corners; corners as Nav2's polygon string |
| `blind_behind(reversing, **sensors)`, `reverse_limit(v, reversing)` | may Nav2 back up, and how fast |
| `nav2_rewrites(v, reversing)`, `nav2_tuned(...)` | Nav2's settings from the car |
| `plan_turning_radius(v)`, `s_bend_length(radius, offset)`, `final_approach(v)` | the planner's radius; the room ahead an S-bend needs; how near the goal Nav2 keeps its path (TODO 17) |
| `ego_params(cfg)`, `spine_params(v)` | ego's and `spine_bridge`'s parameters |
| `check_planner(name, real)`, `lattice_problems(meta, v, resolution)` | a planner's name, checked (only `CAR_PLANNERS` with `real`); what's wrong with a lattice primitives file for this car and map |
| `check_tracker(name, reversing)`, `track_params(nav2, cfg, ego, max_speed)` | a trial tracker's name, checked; `aav_track`'s parameters from nav2.yaml, vehicle.yaml and ego.yaml |
| `obstacle_heights(v)`, `centre_of_mass(...)` | the lidar's height band; centre of mass from the scales |

Constants: `STOP_ZONE_AHEAD`, `STOP_ZONE_BEHIND`, `STOP_ZONE_MARGIN` (m),
`SEES_BEHIND`, `PLAN_TURN_MARGIN`, `GROUND_CLEARANCE`, `OVERHEAD_MARGIN` (m), and
`CONTROLLERS = ('rpp', 'mppi')` with `DEFAULT_CONTROLLER = 'mppi'` (the default of
`sim.launch.py`'s `controller`); `TRACKERS = ('none', 'pure_pursuit', 'stanley', 'lqr',
'mpc', 'mpc_full')` with `DEFAULT_TRACKER = 'none'` (`tracker:=`, `docs/trials.md`), kept apart from
`CONTROLLERS`; `PLANNERS = ('hybrid', 'lattice', 'smac2d', 'navfn', 'theta_star')` with
`DEFAULT_PLANNER = 'hybrid'`, `CAR_PLANNERS = ('hybrid', 'lattice')` (the ones that keep to
the turning radius) and `PLANNER_BLOCKS` (each one's block in `nav2.yaml`).

### URDF (`urdf/`)

`aav.urdf.xacro` builds `base_link`, `chassis`, four wheels with two steering joints,
and one link per sensor in `vehicle.yaml`. `aav.gazebo.xacro` and
`aav.sensors.gazebo.xacro` add Gazebo's plugins and sensors when `sim:=true`.

## Invariants

- One file holds every measurement. Never copy a number into another package.
- REP-103 and REP-105: metres, radians; x forward, y left, z up; positive steering = left.
- `base_link` is on the ground under the centre of the rear axle. Measure from there.
- `max_steering_angle` is the virtual centre wheel (bicycle model), as ego and Nav2 use.
- `firmware/spine`'s `spine-core` keeps copies of the steering limit and reverse speed.
  `test_vehicle.py` checks they match: change both together.
- `vehicle.py` has no ROS imports, except `config_path()` (ament_index).

## Side Effects

- `vehicle.py` reads `config/vehicle.yaml` from disk.
- The launch files start `robot_state_publisher`, `joint_state_publisher` and RViz.

## Verification

```sh
pixi run build && colcon test --packages-select aav_description && colcon test-result --verbose
```

`pixi run test` runs every package's tests. To see a change: `pixi run view`.
