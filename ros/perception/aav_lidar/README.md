# aav_lidar

## Overview

The roof lidar for Nav2. `lidar_scan` flattens the lidar's cloud (about 0.9 MB) into a 2D
scan around `base_link` (a few kB): for each direction, the nearest point between the
road and just over the roof. That is what the car would hit.

Nav2's local costmap (`lidar_layer`) and `collision_monitor` read the scan.
`navigation.launch.py lidar:=true` starts it, with its parameters from `nav2.yaml`.

Why not the stock `pointcloud_to_laserscan`? It subscribes best effort, and on a Mac (UDP,
no shared memory) no 0.9 MB cloud ever arrives that way (`docs/mac.md`).

## Interface

### Node `lidar_scan`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/lidar/points` | `sensor_msgs/PointCloud2`, reliable, depth 1 |
| out | `/sensors/lidar/scan` | `sensor_msgs/LaserScan`, best effort, `base_link`, the cloud's stamp |

The node uses relative names (`sensors/lidar/...`); in the root namespace they are the
absolute ones above.

### Parameters

| Parameter | Default | Means |
|---|---|---|
| `min_height`, `max_height` | 0.3 m, 2.0 m | points between these heights above the road count (`navigation.launch.py` sets them from `vehicle.yaml`) |
| `angle_min`, `angle_max`, `angle_increment` | -π, π, 0.5° | rad: 720 directions |
| `range_min`, `range_max` | 0.2 m, 30 m | from `base_link` |
| `scan_time` | 0.1 s | |

From `vehicle.yaml` (through `aav_description`): the lidar's place (`lidar_link`) and the
car's own outline, plus 0.3 m behind and beside it and 0.1 m ahead of the bumper.

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `scan.py` | yes | `ScanShape`, `rotation`, `to_base`, `flatten`; `body_box` (the car's own outline plus its margins, `SELF_MARGIN`, `FRONT_MARGIN`: `aav_voxels` uses it too) |
| `ground.py` | yes | body roll against the lowest beams: `ground_hit`, `worst_ground_height`, `max_safe_roll`, `VLP16_ELEVATIONS`. Sums only; the node doesn't use it (TODO 52e) |
| `lidar_scan.py` | no | the node |

There is no launch or config folder: `aav_navigation` starts and configures it.

## Invariants

- `scan.py` and `ground.py` have no ROS imports, and their own tests.
- Output is in `base_link` (REP-105): x forward, y left, on the road under the rear axle.
- Each range is the nearest point between `min_height` and `max_height`; `inf` where none.
- Points inside the car's own outline never count.
- The cloud is read reliable, depth 1: a late cloud is dropped for the newest.
- The body is taken as level. In a hard turn the road on the outside can rise above
  `min_height` (see `ground.py`). Gazebo has no suspension, so it can't show this.

## Side Effects

- Publishes `/sensors/lidar/scan`. Reads `vehicle.yaml` once at start.

## Verification

```sh
cd ros/perception/aav_lidar && python3 -m pytest -q        # the logic, no ROS
pixi run build && colcon test --packages-select aav_lidar && colcon test-result --verbose
```

`tools/test.sh` runs the first; `pixi run test` runs everything. In Gazebo:
`pixi run sim-check --lidar-obstacle`.
