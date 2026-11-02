# aav_voxels

## Overview

A trial beside `aav_lidar`'s flat scan (Ben, 30 Oct: "this is a car not a needle").
`voxels` reads the roof lidar's cloud once and publishes **0.25 m cubes**: what stands
between the road and just over the roof. Nav2's local costmap and `collision_monitor`
read the cubes instead of the scan when `voxels:=true`.

```
/sensors/lidar/points ──► voxels ──► /perception/voxels/cubes  ──► costmap (mark), collision_monitor
                            ├──────► /perception/voxels/ground ──► costmap (clear)
                            └──────► /perception/voxels/status
```

The road's height is fitted to every cloud, so a car leaning in a turn doesn't see the
road as an obstacle. The design and why not Nav2's VoxelLayer or STVL:
[`docs/designs/2026-10-30_voxels.md`](../../../docs/designs/2026-10-30_voxels.md).

## Interface

### Node `voxels`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/lidar/points` | `sensor_msgs/PointCloud2`, reliable, depth 1 |
| out | `/perception/voxels/cubes` | `PointCloud2`, `base_link`, x y z: each cube's centre |
| out | `/perception/voxels/ground` | `PointCloud2`, `base_link`: each flat square of road it saw |
| out | `/perception/voxels/status` | `diagnostic_msgs/DiagnosticStatus`: `ms`, `cubes`, `ground_cubes`, `points`, `fitted`, `roll_deg`, `pitch_deg`, `clouds_dropped` |

### Parameters (`aav_navigation` `config/nav2.yaml`, `voxels:`)

| Parameter | Default | Means |
|---|---|---|
| `size` | 0.25 m | one cube's edge |
| `min_height`, `max_height` | 0.3 m, 2.0 m | above the fitted road (the launch file sets them from `vehicle.yaml`) |
| `range_max` | 30 m | flat from `base_link` |

From `vehicle.yaml`: the lidar's place (`lidar_link`) and the car's outline
(`aav_lidar scan.body_box`: its points never count).

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `cubes.py` | yes | `CubeShape`, `Ground`, `fit_ground`, `cube_centres`, `cubes` |
| `voxels_node.py` | no | the node |

`test/lot.py` makes a 16-beam lidar turn round a car on a flat lot, leaning or not.

## Invariants

- `cubes.py` has no ROS imports, and its own tests.
- Output in `base_link` (REP-105), stamped with the cloud's stamp.
- A fit steeper than 6°, more than 0.3 m off, or from fewer than 20 squares is not
  believed: the road is taken as flat, and the status says WARN.
- Points inside the car's own outline never count.
- Off by default: nothing reads the cubes unless `voxels:=true`.

## Side Effects

- Publishes the three topics above. Reads `vehicle.yaml` once at start.

## Verification

```sh
cd ros/perception/aav_voxels && python3 -m pytest -q        # the logic, no ROS
pixi run build && colcon test --packages-select aav_voxels && colcon test-result --verbose
```

In Gazebo: `pixi run sim-check --lidar-obstacle --voxels`; `pixi run controller-compare
--voxels --tours lot`. Results: `docs/trials.md`.
