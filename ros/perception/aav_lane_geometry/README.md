# aav_lane_geometry

## Overview

The `LaneGeometry` message, from the 2025-26 team (`Lane_Detection_ROS2`; see
`PROVENANCE.md`). [aav_lane_finder](../aav_lane_finder/README.md) (and its Rust twin)
publishes it on `/perception/lane/geometry`; our costmap draws its lines as lane costs
(`lanes:=camera`, TODO 264) and `aav_lane_pose` reads it as a sideways pose.

```
lane_finder ─► /perception/lane/geometry ─► costmap (the lanes layer) ─► planner
                                        └─► lane_pose (a trial)
```

**The lane follower is gone** (19 Nov, TODO 264; Ben: "drop there code all together. We
use the perception system to identify the lines and then we bake it into the cost map").
The 2025-26 `control_interface`, its Rust port (`aav_lane_follower_rs`), pure pursuit,
Stanley and the fake lane were removed; they are in git tag `save/lane-followers`, their
scores in [Lane followers, as they were](../../../docs/history/lane-followers.md).

## Interface

### Message `msg/LaneGeometry.msg`

| Field | Type | Means |
|---|---|---|
| `header` | `std_msgs/Header` | the picture's stamp; `base_link` |
| `left_curvature_m`, `right_curvature_m`, `mean_curvature_m` | float32 | radius, m (9999 = straight or not seen) |
| `curve_direction` | float32 | which way the road bends; 0 = straight |
| `vehicle_offset_m` | float32 | m from the lane's centre, + = car left of it |
| `lane_width_m` | float32 | m |
| `left_poly_a/b/c`, `right_poly_a/b/c` | float32 | y = a x² + b x + c (m) |
| `left_confidence`, `right_confidence` | float32 | 0 to 1 |
| `left_lane_detected`, `right_lane_detected`, `both_lanes_detected` | bool | |
| `steering_hint` | float32 | a pure-pursuit wheel angle to the lane's centre (rad) |
| `left_paint`, `right_paint` | uint8 | `PAINT_UNKNOWN` (0, not detected), `PAINT_WHITE` (1), `PAINT_YELLOW` (2): from the line's colour (TODO 264) |

## Invariants

- Keep the old fields: add new ones at the end, so nothing downstream breaks.
- Lane width, offsets and radii are metres; `base_link`, x forward, y left.
- Inherited code: record every change in `PROVENANCE.md`.

## Side Effects

- None: a message package.

## Verification

```sh
pixi run build && ros2 interface show aav_lane_geometry/msg/LaneGeometry
```
