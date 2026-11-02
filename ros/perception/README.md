# ros/perception

## Overview

Pictures, lidar and radar become things the car can act on: objects, stop signs, lane
lines, a flat scan and obstacle points. Drivers and Nav2 read these; perception never
commands the car itself, except through the stop-sign lock and the lane follower's AI slot.

## Subdirectories & Boundaries

| Package | What it does | Main output |
|---|---|---|
| [aav_detector](./aav_detector/README.md) | object detection (RF-DETR Nano; YOLO as an option), the stop-sign watcher, camera obstacles | `/perception/detections`, `/perception/stop_sign`, `/perception/camera_obstacles` |
| [aav_lane_finder](./aav_lane_finder/README.md) | classic OpenCV lane finder on the front left camera | `/perception/lane/geometry` |
| [aav_lane_geometry](./aav_lane_geometry/README.md) | the 2025-26 team's lane keeping: the message, a PID follower, a fake lane | `/drivers/ai/cmd_vel` |
| [aav_lidar](./aav_lidar/README.md) | flattens the lidar's cloud into a 720-ray scan for Nav2 | `/sensors/lidar/scan` |
| [aav_voxels](./aav_voxels/README.md) | a trial (`voxels:=true`): the lidar's cloud as 0.25 m cubes over the fitted road, for Nav2 instead of the scan | `/perception/voxels/cubes`, `/perception/voxels/ground` |
| [aav_radar_clusters](./aav_radar_clusters/README.md) | Rust DBSCAN on the radar points | `/perception/radar/clusters` |

## Invariants

- Logic modules have no ROS imports and their own tests.
- The stop-sign lock (twist_mux priority 30) holds Nav2, the point driver and the AI slot only, never a person.
- The lane follower drives only the AI slot (priority 10): every other driver outranks it.
- Camera and lidar positions come from `vehicle.yaml`, through the launch files. Run
  without its launch file, the lane finder falls back to positions written in its code.

## Side Effects

- Publishes the topics above; the detector loads a model from `models/` and uses the GPU.
- `aav_lane_geometry`'s logger writes CSV files to `~/lane_logs`.

## Verification

- Run: `pixi run test` (or `colcon test --packages-select <pkg>` after `pixi run build`).
