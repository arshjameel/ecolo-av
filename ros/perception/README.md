# ros/perception

## Overview

Pictures, lidar and radar become things the car can act on: objects, stop signs, lane
lines, a flat scan and obstacle points. Drivers and navigation read these; perception never
commands the car itself, except through the stop-sign lock.

## Subdirectories & Boundaries

| Package | What it does | Main output |
|---|---|---|
| [aav_detector](./aav_detector/README.md) | object detection (RF-DETR Nano; YOLO as an option), the stop-sign watcher, camera obstacles | `/perception/detections`, `/perception/stop_sign`, `/perception/camera_obstacles` |
| [aav_stop_sign_rs](./aav_stop_sign_rs/README.md) | `stop_sign_watcher` in Rust, the default since 17 Nov (`stop_sign_impl:=python` falls back): the same lock and point | `/perception/stop_sign`, `/perception/stop_sign/distance` |
| [aav_detector_rs](./aav_detector_rs/README.md) | the detector in Rust, a challenger (`detector_impl:=rust`): RF-DETR's ONNX through ONNX Runtime, Core ML on a Mac | the same as `aav_detector`'s `detector` |
| [aav_lane_finder](./aav_lane_finder/README.md) | classic OpenCV lane finder on the front left camera (Python: the fall-back, `lane_finder_impl:=python`); its launch file starts either | `/perception/lane/geometry` |
| [aav_lane_finder_rs](./aav_lane_finder_rs/README.md) | the same `lane_finder` in Rust, no OpenCV, the default since 18 Nov: the same lanes | the same as `aav_lane_finder` |
| [aav_lane_finder_learned](./aav_lane_finder_learned/README.md) | the learned lane finder, a challenger (`lane_finder_impl:=learned`): our UFLD-v2-style model, trained on Ravensim's pictures, through ONNX Runtime (Core ML on a Mac) | the same as `aav_lane_finder` |
| [aav_lane_geometry](./aav_lane_geometry/README.md) | the `LaneGeometry` message (the 2025-26 team's; their lane follower removed 19 Nov, TODO 264) | the message |
| [aav_lidar](./aav_lidar/README.md) | flattens the lidar's cloud into a 720-ray scan for the costmap and the stop zone | `/sensors/lidar/scan` |
| [aav_voxels](./aav_voxels/README.md) | a trial (`voxels:=true`): the lidar's cloud as 0.25 m cubes over the fitted road, for the costmap instead of the scan | `/perception/voxels/cubes`, `/perception/voxels/ground` |
| [aav_voxels_rs](./aav_voxels_rs/README.md) | the same `voxels` node in Rust, a drop-in (`voxels_impl:=rust`): the same cubes, 8 times faster | the same as `aav_voxels` |
| [aav_lidar_rs](./aav_lidar_rs/README.md) | the same `lidar_scan` in Rust, the default since 17 Nov (`lidar_impl:=python` falls back): the same ranges | the same as `aav_lidar` |
| [aav_things_rs](./aav_things_rs/README.md) | a trial (`things:=true`): the cubes grouped into things, each tracked by a Kalman filter (the radar's speed too) and remembered a few seconds unseen; Rust (the Python removed 20 Nov) | `/world/things` |
| [aav_radar_clusters](./aav_radar_clusters/README.md) | Rust DBSCAN on the radar points | `/perception/radar/clusters` |

## Invariants

- Logic modules have no ROS imports and their own tests.
- The stop-sign lock (twist_mux priority 30) holds navigation, the point driver and the AI slot only, never a person.
- Camera and lidar positions come from `vehicle.yaml`, through the launch files. Run
  without its launch file, the lane finder falls back to positions written in its code.

## Side Effects

- Publishes the topics above; the detector loads a model from `models/` and uses the GPU.

## Verification

- Run: `pixi run test` (or `colcon test --packages-select <pkg>` after `pixi run build`).
