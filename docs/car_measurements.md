# Car measurements and the URDF

The car's measurements live in one file, `ros/bringup/aav_description/config/vehicle.yaml`. Every
other part reads them from there, so a number only ever changes in one place.

| Who reads it | What it takes |
|---|---|
| `urdf/aav.urdf.xacro`, then `robot_state_publisher`, then `/tf_static` | body, wheels, and every sensor's mount (the frames) |
| `aav_ego` (ego.launch.py) | wheelbase, steering limit, reverse speed; and its body: mass, track, centre of mass, tyre grip, the throttle's and brake's accelerations |
| `aav_spine` (spine.launch.py) | wheelbase, track |
| `aav_legacy` (legacy.launch.py) | wheel diameter, encoder counts per revolution |
| `aav_navigation` (navigation.launch.py) | footprint (both costmaps), minimum turning radius (SmacPlannerHybrid), reverse speed |
| `aav_turtlesim`, `aav_gazebo` | wheelbase, and in Gazebo the whole model |
| `aav_drive_to_point`, `carla_adapter` | `actuators.max_brake`; `actuators.max_accel` |
| `firmware/spine` (`spine-core`) | **copies** of `max_steering_angle` and `max_reverse_speed` (it can't read the file). Change both together: `test_vehicle.py` checks they match |

`aav_description/aav_description/vehicle.py` does the derived maths: turning radius =
wheelbase / tan(max steering angle), and the footprint polygon from the body. Its tests
(`ros/bringup/aav_description/test/test_vehicle.py`) run without ROS.

## Every value is still a guess

Each `TODO` in `vehicle.yaml` came from the previous teams' code, with the file and line
cited next to it, or is a guess. Two examples: the wheelbase (1.57 m) comes from a MATLAB
script, and the old code's steering limit varies from 0.45 to 0.55 rad. Measure the car,
replace the number and delete the `TODO`.

## The frames

REP-103 and REP-105 conventions:
- metres and radians;
- x forward, y left, z up;
- angles positive counter-clockwise, so a positive steering angle turns left.

| Frame | Where |
|---|---|
| `base_link` | On the ground, under the centre of the rear axle. Everything is measured from here. |
| `chassis` | The body box, from `body.length/width/height` and `rear_overhang` |
| `imu_link`, `gps_link`, `radar_link`, `camera_left_link`, `camera_right_link`, `camera_rear_link`, `lidar_link` | `sensors.<name>.xyz` (m) and `rpy` (rad), relative to `base_link` |
| `*_steering_joint`, `*_wheel_joint` | Front wheels steer about z; all wheels spin about y |

Why it matters:
- `navsat_transform` uses `base_link -> gps_link` to remove the antenna offset from every
  GPS fix, so measure the antenna itself, not the receiver box. It is mounted over the rear
  axle (Ben, 23 Oct), which is best: a heading error can't move the GPS position, and
  reversing keeps its heading ([Learn: GPS](learn-gps.md#mount-the-antenna-over-the-rear-axle)).
- The EKFs use `imu_link` to rotate IMU readings into the car's frame. If the IMU is
  mounted turned or upside down, its `rpy` must say so, or yaw comes out backwards.

## How to measure

The tape-measure route, step by step, with pictures: [the measuring map](measure-the-car.md#the-measuring-map)
(`pixi run measure-map`; `--readings` turns chained readings into these numbers).

1. Park on level ground with the wheels straight.
2. Mark `base_link`: plumb down from the centre of the rear axle and chalk the spot.
3. Measure each item with a tape:
   - **wheelbase:** rear axle centre to front axle centre;
   - **track:** left wheel centre to right wheel centre;
   - **wheel diameter:** across the tyre, loaded;
   - **body:** bumper to bumper, side to side, ground to the top, and the rear overhang
     behind the rear axle;
   - **each sensor:** x forward and y left of the chalk mark, z up from the ground.
4. **Steering limit:** turn to full lock and measure the angle of the inside and outside
   front wheels. The value in the file is the "virtual" centre wheel. A good
   approximation is atan(wheelbase / R), where R is the turning radius to the middle of
   the rear axle. Measure R by driving a full circle at full lock at walking pace.
5. **Mass and centre of mass:** four bathroom scales, then the rear lifted
   (measure-the-car.md, M7 and M7b). See "Weight and balance" below.

## Weight and balance

Ego knows the car's weight (`aav_ego/dynamics.py`). Most of it is guessed today.

| `vehicle.yaml` | Today | Measured by |
|---|---|---|
| `mass` | 181 kg (Ben: about 400 lb) | M7: the four scales added up |
| `centre_of_mass.x` | 0.70 m ahead of the rear axle (a guess) | M7: wheelbase × front ÷ total |
| `centre_of_mass.height` | 0.55 m (a guess) | M7b: the rear lifted, the front weighed again |
| `tyre_grip` | 0.7 g (dry asphalt: 0.7-0.9) | test 4.3: full brake; if the wheels skid, the deceleration in g |
| `actuators.max_accel`, `max_brake` | 1.5 and 3.0 m/s², empty | tests 4.1 and 4.3; 4.9 and 4.10 again with a person aboard |

What ego does with them:

| The load (people, cargo) changes | The load doesn't change |
|---|---|
| how hard the car can speed up: the same motor, more mass | how fast it may take a turn: sliding and tipping don't depend on the mass |
| how hard it can brake, so how far it needs to stop | |
| the energy in a crash: ½ m v² | |

- **Speeding up:** ego never asks for more than the motor can give the loaded car:
  `max_accel × mass ÷ (mass + payload)`.
- **Turns:** ego slows the car so the sideways acceleration stays under 2 m/s², or under
  half of what slides or tips the car if that is lower. It keeps the turn the driver asked
  for and lowers the speed, so the car stays on its path. At full lock (2.87 m) that is
  2.4 m/s: it only matters above the car's 1 m/s, so today in the simulator. Checked in
  Gazebo (22 Oct, `sim_speed:=3`): asked 3 m/s at full lock, ego allowed 2.40 m/s and the
  car held 2.37-2.43 m/s, 1.9-2.0 m/s² sideways.
- **Braking:** never softened. If the load means the brakes can't stop the car as fast as
  ego plans (`max_deceleration`), ego warns at start-up and when the payload changes.
- **Payload** is a parameter (ego.yaml, 0 kg): `ros2 param set /ego payload 80` when
  someone gets in.

Tipping matters on this car: a 77 cm track is narrow for a 1.5 m tall car. The car tips
when the sideways acceleration reaches g × (track ÷ 2) ÷ (height of the centre of mass):
6.9 m/s² at 0.55 m high, but 3.8 m/s² at 1 m. Ego keeps to half of that.

From M7 and M7b (any unit, as long as all four are the same):

```python
from aav_description import vehicle
v = vehicle.load()['vehicle']
x, height = vehicle.centre_of_mass(v, front=FL + FR, rear=RL + RR,
                                   front_lifted=FL2 + FR2, lift=0.25)   # lift in m
```

## Look at it

You need Jazzy installed and the workspace built: see [Run it](run.md).

```bash
ros2 launch aav_description view.launch.py
```

RViz opens with the car model and every frame's axes (red x forward, green y left, blue
z up). Each sensor is an orange box. A slider window moves the steering and wheel joints.

The shapes are only for looking at, and every size comes from `vehicle.yaml`: a red tub
over the wheels, a cabin with a white roof and a seat, a bumper and headlights at the
front, and a white mark on each wheel so you can see it turn. Physics and Nav2 use only
the collision shapes: the body's outline between the bumpers, and the four wheels.

![The car model in Gazebo](img/gazebo-car.png)

After editing `vehicle.yaml`, rebuild and relaunch:

```bash
colcon build --packages-select aav_description --symlink-install
```

With `--symlink-install`, later edits to `vehicle.yaml` only need a relaunch.

Two other ways to inspect the model:
- Print the generated URDF: `xacro ros/bringup/aav_description/urdf/aav.urdf.xacro`.
- Draw the frame tree as a diagram: `ros2 run rqt_tf_tree rqt_tf_tree`.

## In Gazebo

`aav.gazebo.xacro` adds the simulation parts when `sim:=true`:
- the Ackermann steering system (gz-sim 8);
- IMU and NavSat sensors;
- the joint-state publisher.

Gazebo builds the simulated car from the same `vehicle.yaml`, so the simulated car turns
like the real one. Its odometry frames are bridged but its TF is not:
`robot_localization` owns `odom -> base_link`, exactly as on the car.
