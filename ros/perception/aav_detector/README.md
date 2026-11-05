# aav_detector

## Overview

Finds objects in the camera pictures, and uses what it finds. Three nodes:

| Node | Does | Needs PyTorch? |
|---|---|---|
| `detector` (runs as `object_detector`) | a model finds boxes in each picture: RF-DETR Nano by default, YOLO as an option | yes (the `ml` environment) |
| `stop_sign_watcher` | holds the autonomous drivers at a stop sign, through a twist_mux lock | no |
| `camera_obstacles` | puts each box on the road as points, for Nav2's local costmap | no |

It began as the 2024-25 team's `yolo_ros2` (see `PROVENANCE.md`). The model comparison
is in `docs/model-comparison.md`.

## Interface

### Topics

| Node | In / out | Topic | Type |
|---|---|---|---|
| detector | in | `image` (launch: `/sensors/camera/front_left/image_raw`) | `sensor_msgs/Image`, reliable, depth 1 |
| detector | in | `<image>/compressed`, with `compressed:=true` | `sensor_msgs/CompressedImage` |
| detector | in | each topic in `extra_images` (the rear camera) | `sensor_msgs/Image` |
| detector | out | `/perception/detections` | `vision_msgs/Detection2DArray`, the picture's header |
| detector | out | `/perception/detections/image` | `sensor_msgs/Image`, boxes drawn on |
| stop_sign_watcher | in | `/perception/detections` | `vision_msgs/Detection2DArray` |
| stop_sign_watcher | in | `camera_info_topics` (front_left, front_right, rear) | `sensor_msgs/CameraInfo` |
| stop_sign_watcher | out | `/perception/stop_sign`, 10 Hz | `std_msgs/Bool` |
| camera_obstacles | in | `/perception/detections` | `vision_msgs/Detection2DArray` |
| camera_obstacles | in | `camera_info_topics` (front_left, rear) | `sensor_msgs/CameraInfo` |
| camera_obstacles | in | TF `base_link` to each camera frame | |
| camera_obstacles | out | `/perception/camera_obstacles` | `sensor_msgs/PointCloud2`, `base_link`, z = 0 |
| camera_obstacles | out | `/perception/camera_view` | `sensor_msgs/PointCloud2`: the far edge the cameras see |

### Parameters

| Node | Parameter | Default | Means |
|---|---|---|---|
| detector | `detector` | `rfdetr` | `rfdetr` or `yolo` |
| detector | `model` | empty | the model file; empty = `models/rf-detr-nano.pth` or `models/yolo11n.pt` |
| detector | `device` | `auto` | `auto`, `cuda`, `cuda:N`, `mps` or `cpu` (PyTorch only) |
| detector | `confidence` | 0.5 | lowest score kept, 0 to 1 |
| detector | `labels` | empty | labels to keep, comma-separated; empty keeps all |
| detector | `image_size`, `publish_image`, `compressed`, `extra_images` | 640, true, false, empty | |
| stop_sign_watcher | `stop_distance_m` | 4.7 m | a sign this close stops the car |
| stop_sign_watcher | `sign_height_m`, `focal_px` | 0.75 m, 343 px | `focal_px` only for a camera with no CameraInfo |
| stop_sign_watcher | `min_score`, `hold_s`, `ignore_s` | 0.5, 4 s, 10 s | |
| camera_obstacles | `classes`, `min_score` | COCO road users, 0.5 | stop signs never count |
| camera_obstacles | `max_range`, `view_range`, `step`, `max_age` | 12 m, 12 m, 0.1 m, 1 s | `config/camera_obstacles.yaml` |

### Launch and modules

- `launch/detector.launch.py` (`pixi run detect`): the detector as a process; `picture:=<file>`
  adds an `image_publisher` test camera. Arguments: `detector`, `model`, `device`,
  `confidence`, `labels`, `extra_images`, `compressed`, plus `image`, `picture`,
  `node_name`, `use_sim_time`. Not `image_size` or `publish_image`: set those as node parameters.
- `config/camera_obstacles.yaml`: camera_obstacles' settings.

| Module | No ROS? | Holds |
|---|---|---|
| `boxes.py` | yes | `Box`, `family`, `check_detector`, `parse_labels`, `boxes_from_corners`, `corners`, `pick_device`, `normal_label` |
| `detectors.py` | yes | `load_detector()`; `Yolo`, `RfdetrTorch`, `RfdetrCoreml`, `RfdetrTensorrt`, each with `detect(bgr, confidence, labels)` |
| `detr.py` | yes | RF-DETR pre- and post-processing for exported models (NumPy only) |
| `ground.py` | yes | pinhole camera over a flat road: `Lens`, `Mount`, `box_points`, `view_edge`, `counts` |
| `recent.py` | yes | `Recent`: each camera's newest points, dropped after `max_age` |
| `stop_sign.py` | yes | `StopRule`, `StopSignWatcher`: CLEAR, STOPPING, PASSING |
| `detector.py`, `stop_sign_watcher.py`, `camera_obstacles.py` | no | the nodes |

## Invariants

- The logic modules have no ROS imports, and their own tests.
- The stop sign lock is twist_mux priority 30. It holds the point driver (25), Nav2 (20) and the AI slot
  (10), never a person or a test script. It can only stop the car.
- The lock never expires on silence: if `stop_sign_watcher` dies while on, the car stays stopped.
- Stop rule: distance = `focal_px` x `sign_height_m` / box height. Lock on for `hold_s`,
  then signs ignored for `ignore_s`.
- Stop signs never become camera obstacles.
- camera_obstacles publishes every camera's newest points together, each time; a camera
  silent longer than `max_age` drops out. Points carry the picture's stamp.
- Pictures are subscribed reliable, depth 1: best effort loses whole pictures over UDP.
- camera_obstacles is off by default until the cameras are calibrated (Q22).

## Side Effects

- Publishes the topics above.
- detector: loads a model file from `models/` (relative to the repo root) and runs it on
  the GPU (CUDA, Metal, Core ML or TensorRT) or CPU.
- Logs a frame rate every 10 s.

## Verification

```sh
cd ros/perception/aav_detector && python3 -m pytest -q     # the logic, no ROS
pixi run build && colcon test --packages-select aav_detector && colcon test-result --verbose
```

`tools/test.sh` runs the first; `pixi run test` runs everything. To try the model:
`pixi run get models`, then `pixi run detect picture:=ros/perception/aav_detector/test/stop_sign.jpg`.
