# Provenance

Copied on 2026-10-09 from github.com/Carleton-AAV-Capstone/camera_ros2_ws @ default branch (downloaded 2026-10-09), src/yolo_ros2.

Files, unchanged unless listed under Changes:

- `package.xml`
- `setup.cfg`
- `setup.py`
- `yolo_ros2/__init__.py`
- `yolo_ros2/yolo_inference.py`

`training/` came from a second repo, `Carleton-AAV-Capstone/camera_Yolo_detection`
(`data.yaml`, `testing_torch.py`, unchanged). Removed on 2026-10-26: nothing used it.

## Changes

- 2026-10-10: `yolo_ros2/yolo_inference.py` `.warn(` -> `.warning(` (deprecated in Jazzy).
- 2026-10-10: brought in from parked as `aav_yolo` (the name in docs/conventions.md).
  - The package, its folder and its module are renamed; `yolo_inference.py` is now
    `aav_yolo/detector.py`, and its executable `detector`.
  - `detector.py` is rewritten. It keeps the idea: camera images in, YOLO, results out.
    Changed:
    - the model file is a parameter, instead of `~/Yolo/Capstone/runs/detect/train7/weights/best.pt`;
    - it publishes `vision_msgs/Detection2DArray`, instead of only an annotated image;
    - the annotated image is still there, on `detections/image`, but no window opens
      (`cv2.imshow` is gone);
    - the confidence is a parameter, 0.5 by default. The old code used 0.81, tuned for its
      own model;
    - it keeps only the labels asked for, and picks the GPU: CUDA, else Metal, else the CPU.
  - New: `aav_yolo/boxes.py` (the logic, with tests), `launch/detector.launch.py`,
    `test/`, and the missing `resource/aav_yolo` marker.
  - `package.xml` and `setup.py` are rewritten with the real dependencies.
- 2026-10-10: `detector.py` can read the JPEG pictures (`compressed:=true`, for a laptop
  on WiFi). The device setting takes PyTorch's own names (`auto`, `cuda`, `cuda:N`, `mps`,
  `cpu`): the aliases (`gpu`, `metal`, `cuda` meaning Metal on a Mac) are gone, and a
  device the computer lacks is a clear error. The launch file takes the node's `name`, so
  each computer's detector has its own.
- 2026-10-11: new, our own: `aav_yolo/ground.py` (a pinhole camera over a flat road: a
  box's bottom edge to points on the road, with tests in `test/test_ground.py`) and the
  `camera_obstacles` node (`aav_yolo/camera_obstacles.py`, `config/camera_obstacles.yaml`),
  which publishes them for Nav2's local costmap. `setup.py` installs `config/`;
  `package.xml` adds `sensor_msgs_py` and `tf2_ros_py`.
- 2026-10-11: `launch/detector.launch.py` takes `use_sim_time` (false by default), so
  the detector runs on Gazebo's clock in `sim.launch.py perception:=true`.
- 2026-10-11: our own `aav_yolo/stop_sign.py`: `min_height_px` 55 by default, was 72. In
  Gazebo's road world a sign 2 m right of the lane left the picture before its box was
  72 px tall whenever Nav2 drove the car 0.5-0.7 m left of its line, so the car drove on.

## Topic names by area (2026-10-15)

The topics move to their areas (docs/naming.md, Decided by Ben on 15 Oct). Only names
change, never the logic:

- `aav_yolo/detector.py`, `aav_yolo/camera_obstacles.py`, `aav_yolo/stop_sign_watcher.py`,
  `config/camera_obstacles.yaml`, `launch/detector.launch.py`: `/camera/...` ->
  `/sensors/camera/front_left/...`, `/perception/stop` -> `/perception/stop_sign`.
- 2026-10-20: RF-DETR, and a switch between it and YOLO (Ben: "Lets switch to RF-DETR").
  `detector.py` takes a `detector` parameter, `rfdetr` (the default) or `yolo`; `model`
  may be empty, for the detector's own file (`models/rf-detr-nano.pth`,
  `models/yolo11n.pt`). The two run behind one `detect()`; the picture's boxes are drawn
  by our own `draw()` for both, instead of Ultralytics' `result.plot()`. New in
  `boxes.py`, with tests: `check_detector`, `rfdetr_size`, `boxes_from_corners` and
  `corners`; `pick_device` treats `.pth` as PyTorch. The launch file takes `detector:=`,
  and its `model` default is empty. The package, the node (`yolo_detector`) and the
  topics keep their names.
- 2026-10-22: the rear camera, so camera_obstacles can see behind a reversing car (Ben:
  "Yes the car can reverse"). `detector.py` and `launch/detector.launch.py` take
  `extra_images`: more camera topics for the same model. `camera_obstacles.py` publishes
  every camera's newest points together, each time (new, our own: `aav_yolo/recent.py`,
  tested in `test/test_recent.py`), and `config/camera_obstacles.yaml` lists the rear
  camera's CameraInfo and `max_age`.
- 2026-10-24: RF-DETR Nano is the car's detector (Ben, 23 Oct: "I thought we were skipping
  yolo and using the other one"), in TensorRT on the Orin. The models move out of
  `detector.py` into `aav_yolo/detectors.py` (new, our own, no ROS): `load_detector()`
  and one class per runtime, each with `detect(bgr, confidence, labels)`, so a camera
  stream's own program can load them too. New runtimes for RF-DETR's exports: Core ML
  (`.mlpackage`) and TensorRT (`.engine`); their pre- and post-processing is
  `aav_yolo/detr.py` (new, our own, NumPy only), which mirrors rfdetr 1.11's private
  `export/_runtime`. `boxes.py`: `family()` tells an RF-DETR export from a YOLO one by
  its name (`rf-detr-nano.engine`), and `check_detector` uses it. Tests:
  `test/test_detr.py`, `test/test_detectors.py`, more in `test/test_boxes.py`.
- 2026-10-24: the package `aav_yolo` is renamed `aav_detector`, its module and
  `resource/` marker with it, and the node `yolo_detector` is `object_detector`
  (`object_detector_<machine>` on the car). It runs RF-DETR Nano by default, so "yolo" in
  the name misled (Ben, 24 Oct: "aav detector yes"). Only names change: the
  executables, parameters, topics and `detector:=yolo` stay. The paths above that say
  `aav_yolo/` are now `aav_detector/`.
- 2026-10-24: `stop_sign.py` (our own): the rule is a distance, not a box height.
  `StopRule.stop_distance_m` (4.7 m: the old 55 px with Gazebo's and the bench's lens,
  343 px), `sign_height_m` (0.75), `focal_px` (343, for a camera with no CameraInfo);
  `min_height_px` became a method, from the camera's focal length. `stop_sign_watcher`
  keeps each camera's fy from its CameraInfo (`camera_info_topics`, by frame_id) and
  passes it with the boxes, so the same 4.7 m holds for the 1280x720 ribbon cameras once
  calibrated. Its `min_height_px` parameter is gone. Tests: `test/test_stop_sign.py`.
- 2026-10-25: the description in `package.xml` and `setup.py` says what the package is
  now: the object detector (RF-DETR Nano by default, YOLO with `detector:=yolo`), the
  stop-sign watcher and camera_obstacles. It still said a YOLO detector with the stock
  YOLO11 model.
- 2026-11-22: `aav_detector/stop_sign_watcher.py` (the node) removed, with its
  `console_scripts` entry (Ben: "lets start removing the python fall back after being
  tested enough times"; git tag `save/python-fallbacks-1`). `aav_stop_sign_rs`'s Rust node,
  the default since 17 Nov, is the only one. `stop_sign.py` and its tests stay: the Rust is
  checked against their answers.
