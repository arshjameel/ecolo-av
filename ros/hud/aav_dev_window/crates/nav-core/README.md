# nav-core

## Overview

The top-down navigation view (`pixi run nav-view`, and the developer window's map panel),
without the window: no Bevy, no egui, no ROS, so `tools/test.sh` tests it on any laptop.
It is what RViz's displays did for us, as plain maths
([Screens](../../../../../docs/hud.md#rviz-and-ours)).

```text
a message's parts ──▶ Outline, StopZone, Trail, Body ──▶ points on the map ──▶ the panel draws
the panel's zoom  ──▶ ScaleBar, grid_step, grid_lines
```

## Interface

| Module | Holds | Replaces in RViz |
|---|---|---|
| `trail` | `Trail`: where the car has been; a point each 0.2 m, the newest 4,000; a jump starts it again | the Odometry display's arrows |
| `marker` | `outline(MarkerParts)`: a Marker's cylinder, sphere, cube or line strip as a line on the ground; `Shape` and `Action` are enums; `Frame` (map, odom, base_link) and `Frame::place` | the Marker display |
| `zone` | `StopZone`: the stop zone's box and what it does (`ZoneAction`: CLEAR, STOP, SLOW...), silent after 2 s | the Polygon display |
| `body` | `Body::outline`: the car's body on the map; `quad`: a turned grid's four corners (the local costmap, in odom) | the RobotModel and Footprint displays, the Map display for a turned grid |
| `scale` | `ScaleBar::fit` (a round length that fits), `grid_step`, `grid_lines` | the Grid display, the Measure tool |

## Invariants

- No engine and no ROS: only `map-core`'s `Point` and `Pose`.
- Frames are named, never guessed: a frame it doesn't know is not drawn.

## Verification

```sh
cd ros/hud/aav_dev_window && cargo test -p nav-core     # tools/test.sh runs it
```
