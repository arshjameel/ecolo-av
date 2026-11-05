# aav_camera_stream

One node per camera. Each pulls the camera Pi's RTSP H.264 stream (MediaMTX), decodes
it, undistorts it with OpenCV, and runs `aav_detector`'s object detector on it, all in
one process. Raw pictures never cross the network or DDS.

The whole picture: `docs/camera-pipeline.md`. Calibration: `docs/camera-calibration.md`.

## Overview

```
camera Pi: MediaMTX ─RTSP H.264─> decoder ─> undistort ─> detector ─> /perception/detections
                                 (child)     (OpenCV)    (aav_detector)
```

- Only the newest picture is worked on. A slow detector drops frames instead of
  falling behind.
- `pi/` holds the camera Pi's MediaMTX settings and how to install them:
  [pi/README.md](./pi/README.md).
- `config/cameras.yaml` lists each camera's frame and calibration file. The ribbon
  cameras have no calibration yet (Q22).

## Interface

### Launch: `launch/camera_stream.launch.py`

```sh
ros2 launch aav_camera_stream camera_stream.launch.py host:=<camera Pi> [decoder:=...]
```

Arguments: `cameras` (front_left,front_right), `host`, `port` (8554), `decoder`,
`pipeline`, `program`, `hwaccel`, `transport` (tcp), `width`, `height`, `fps` (1280, 720,
15), `delay` (s), `detector`, `model`, `device`, `confidence`, `labels`,
`preview_period` (s), `preview_width` (320), `publish_image` (false),
`publish_camera_info` (true), `source`, `time_code`, `use_sim_time` (false). Each camera runs as `camera_stream_<name>`, with `python3 -m`, so
`pixi run -e ml` gives it PyTorch.

### Node: `camera_stream` (`aav_camera_stream/camera_stream.py`)

Subscribes to nothing.

| Publishes | Type | Notes |
|---|---|---|
| `/perception/detections` | `vision_msgs/Detection2DArray` | boxes in the undistorted picture's pixels; frame the camera's |
| `/sensors/camera/<camera>/camera_info` | `sensor_msgs/CameraInfo` | the undistorted lens, with each detection; only when the camera has a calibration (none has yet, so none today) |
| `/perception/camera/<camera>/preview/compressed` | `sensor_msgs/CompressedImage` | small JPEG with boxes, every `preview_period` s |
| `/sensors/camera/<camera>/image_raw` | `sensor_msgs/Image` | only with `publish_image` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz: rates, drops, times, delays |
| `/system/source` | `aav_interfaces/DataSource` | latched: this camera and its source |

Main parameters: `camera`, `url`, `frame_id`, `decoder` (auto, ffmpeg, gst-nvidia,
gst-soft, gst-mac, opencv), `transport` (tcp or udp), `calibration`, `alpha`, `delay`
(s), `detector` (rfdetr, yolo, none), `model`, `device`, `confidence`, `labels`,
`image_size`, `preview_period`, `preview_width`, `publish_image`,
`publish_camera_info`, `time_code`, `source` (real, sim, bench, replay).

### Modules (no ROS imports, except `camera_stream.py`)

| Module | Holds |
|---|---|
| `decoders.py` | one decoder per machine: `choose`, `command`, `ffmpeg_command`, `gst_command`, `PipeSource`, `OpenCvSource` |
| `reader.py` | `Reader`: a thread that keeps the decoder running, and restarts it with a growing pause |
| `latest.py` | `Latest` (newest frame only), `Rate`, `Delays`, `stamp_ns` |
| `lens.py` | `Lens`, `parse_ini`, `parse_yaml`, `scaled`, `Undistorter` |
| `models.py` | the detector hook: `make`, `NoModel` (models are `aav_detector`'s) |
| `calibration.py` | the checkerboard (`Board`, `board_svg`), `calibrator_command`, `install` |
| `timecode.py` | a time code drawn into pictures, to measure the delay: `stamp`, `read` |

## Invariants

- Logic modules have no ROS imports and have their own tests.
- A picture's stamp is its arrival minus `delay`, the measured camera-to-here delay.
- Boxes and `camera_info` describe the undistorted picture.
- A calibration of another size is scaled; one of another shape is refused.
- Only the lens (K, D) is used, never a stereo pair's rectification.
- `decoder:=auto` never picks `gst-nvidia`. It gives `gst-mac` on a Mac, `gst-soft` on
  Linux, and `ffmpeg` if GStreamer can't decode. The Orin gets `gst-nvidia` from its
  machine file (`aav_bringup`'s `config/machines/orin.yaml`), through `car.launch.py`.

## Side Effects

- Starts a decoder child process (ffmpeg or `gst-launch-1.0`), except with `opencv`.
- The launch file writes a temporary parameters file per camera, deleted at exit.
- Opens an RTSP connection to the camera Pi (TCP by default).
- Loads a detector model (PyTorch, Core ML or TensorRT) onto CPU or GPU.
- `calibration.install()` writes `config/calibrations/<camera>.yaml` and edits
  `config/cameras.yaml` (through `pixi run calibrate-camera`).

## Verification

```sh
pixi run build && colcon test --packages-select aav_camera_stream && colcon test-result --verbose
```

`pixi run test` runs everything. End to end without the car: `pixi run get mediamtx`
once, then `pixi run measure stream`.
