# aav_lane_finder

## Overview

Finds the lane lines in the front left camera's pictures, with classic OpenCV and no
model. It publishes `/perception/lane/geometry`, which our costmap draws as lane costs
(`lane_source:=camera`, TODO 264) and `aav_lane_pose` reads as a sideways pose.

For each picture:

1. warp it to a bird's-eye view of a flat road (`birds_eye.py`);
2. mask the paint: brighter than the road on both sides, and narrow;
3. climb each line with sliding windows;
4. fit y = a x² + b x + c per line, in `base_link` metres;
5. work out the car's offset from the lane's centre, the lane width and each radius;
6. say each line's paint: yellow (little blue against its red and green) or white
   (`paint_of`, `yellow_ratio`). A grey camera sees every line white.

A model can replace it, behind the same topic: the learned finder,
[aav_lane_finder_learned](../aav_lane_finder_learned/README.md) (`lane_finder_impl:=learned`,
a challenger).

**Its Rust twin runs by default** since 18 Nov: [aav_lane_finder_rs](../aav_lane_finder_rs/README.md),
the same node, topics, parameters and lanes, without OpenCV, at a fifth of the CPU.
`lane_finder_impl:=python` (this launch file, `sim.launch.py`, `car.launch.py`,
`bench.launch.py`) runs this package's node instead.

## Interface

### Node `lane_finder`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/camera/front_left/image_raw` | `sensor_msgs/Image`, reliable, depth 1 |
| in | `/sensors/camera/front_left/camera_info` | `sensor_msgs/CameraInfo`: focal length, centre, distortion |
| out | `/perception/lane/geometry` | `aav_lane_geometry/LaneGeometry`, one per picture, `base_link`, the picture's stamp |
| out | `/perception/lane/image` (only with `debug_image`) | `sensor_msgs/Image`: the bird's-eye view and what was found |

### Parameters

| Parameter | Default | Means |
|---|---|---|
| `camera_xyz`, `camera_rpy` | from `vehicle.yaml` (launch); without it [1.80, 0.29, 1.20], [0, 0, 0] | the camera's place on the car: m, rad |
| `wheelbase` | from `vehicle.yaml` (launch); without it 1.57 | m, for `steering_hint` |
| `yellow_ratio` | 0.75 | a line is yellow when its blue is below this share of the lower of its red and green |
| `focal_px` | 343 px | only until CameraInfo arrives |
| `use_camera_info` | true | false: always use `focal_px` |
| `x_min`, `x_max`, `y_max`, `resolution` | 4 m, 16 m, 4 m, 0.05 m | the patch of road the view covers |
| `paint_contrast`, `paint_width` | 40 grey levels, 0.30 m | what counts as paint |
| `lane_width` | 3.5 m | assumed until both lines are seen |
| `lookahead` | 3.0 m | where the offset is measured, ahead of `base_link` |
| `min_confidence` | 0.3 | below it, a line isn't detected |
| `debug_image` | false | publish `/perception/lane/image` |

Every field of `Settings` in `lane_lines.py` is also a parameter (`windows`,
`window_margin`, `min_pixels`, `min_span`, `fit_scale`, `min_lane_width`, `max_lane_width`).

### Launch and modules

- `launch/lane_finder.launch.py [lane_finder_impl:=rust]`: the node, Rust or Python, with
  `config/lane_finder.yaml` and the camera's place from `aav_description`'s `vehicle.yaml`.
  `bench.launch.py lane:=true` starts it with lane keeping.

| Module | No ROS? | Holds |
|---|---|---|
| `birds_eye.py` | yes | `Lens`, `Mount`, `Grid`, `BirdsEye`, `project`, `rotation` |
| `lane_lines.py` | yes | `Settings`, `Line`, `Lane`, `LaneFinder`, `paint_mask`, `follow`, `fit_line`, `lane_from`, `paint_of`, `draw` |
| `lane_finder.py` | no | the node; `message()` turns a `Lane` into `LaneGeometry` |

`test/scenes.py` draws the made-up lanes the tests (and `aav_lane_finder_rs`'s vectors) use:
solid or dashed (`dash=`), straight or bent, either line missing, a line yellow (`yellow=`).

## Invariants

- `birds_eye.py` and `lane_lines.py` have no ROS imports, and their own tests.
- REP-103: x forward, y left, metres.
- `vehicle_offset_m` is + when the car is left of the lane's centre, measured `lookahead`
  m ahead (a PID on the offset at the rear axle weaves).
- A radius of 9999 m means straight, or not seen. Over 500 m counts as straight
  (`curve_direction` 0).
- The camera's place and the wheelbase come from `vehicle.yaml` when the launch file
  starts it. Run any other way, the node uses the fallbacks written in its code (above).
- CameraInfo is used only when it matches the picture's size; else `focal_px`.
- A bad picture is logged and skipped; it never stops the node.

## Side Effects

- Publishes the topics above. Logs which lines it sees, and its speed every 10 s.

## Verification

```sh
cd ros/perception/aav_lane_finder && python3 -m pytest -q      # the logic, no ROS
pixi run build && colcon test --packages-select aav_lane_finder && colcon test-result --verbose
```

`tools/test.sh` runs the first; `pixi run test` runs everything. In Ravensim:
`pixi run sim lanes:=camera`.
