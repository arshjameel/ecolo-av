# aav_camera_stream_rs

## Overview

`camera_stream` in Rust (rclrs): a **drop-in** for `aav_camera_stream`'s Python node,
**the default** since Ben, 18 Nov ("lets port them to rust"). The same node name,
topics, QoS, parameters and diagnostics. `camera_stream_impl:=python` falls back.

```
camera Pi: MediaMTX ─RTSP H.264─> decoder ─> undistort ─> /perception/detections (empty)
                                 (child)    (stream-core)  /sensors/camera/<c>/image_raw ...
```

| Runs in Rust | Stays in the Python node |
|---|---|
| the decoders (ffmpeg, gst-nvidia, gst-soft, gst-mac), as child programs: the same command lines | `decoder:=opencv` (OpenCV in the process) |
| `decoder:=auto`'s choice, `gst-inspect` checks, Pixi's paths kept from JetPack's GStreamer | the detector (`rfdetr`, `yolo`: PyTorch, Core ML, TensorRT) |
| newest frame only, restarts with a growing pause, rates, delays, stamps | the checkerboard tools (`calibration.py`, `pixi run calibrate-camera`) |
| calibration files, scaling, undistortion (OpenCV's maths, ported), `camera_info` | |
| the time code, the preview JPEG, `/diagnostics`, `/system/source` | |

**Which one runs** (`camera_stream.launch.py`, `aav_description` `vehicle.camera_stream_runs`):

| `camera_stream_impl` | `detector` | `decoder` | Runs |
|---|---|---|---|
| `rust` (default) | `none` | not `opencv` | **Rust** |
| `rust` (default) | `rfdetr` or `yolo` (the car's `orin.yaml` today) | any | Python, and the log says why |
| `rust` | any | `opencv` | Python, and the log says why |
| `python` | any | any | Python |

The Rust node started by hand with a detector or `opencv` stops, saying
`camera_stream_impl:=python`.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/stream-core` | `decoders`, `pipe`, `reader`, `latest`, `lens`, `timecode`, `preview`, `models` | no: `cargo test -p stream-core` |
| `crates/stream-node` | the node, binary `camera_stream` | yes (rclrs 0.8, `ros-env` 0.3) |

colcon builds it through `CMakeLists.txt` (`cargo build --release --locked`).
`Cargo.lock` is in git: the car kit vendors its crates for the offline ARM build.

### Node `camera_stream` (runs as `camera_stream_<camera>`)

| Out | Type | Notes |
|---|---|---|
| `/perception/detections` | `vision_msgs/Detection2DArray` | one per picture, no boxes; the sensor watchdog watches it |
| `/sensors/camera/<camera>/camera_info` | `sensor_msgs/CameraInfo` | the undistorted lens, with each picture (with a calibration) |
| `/perception/camera/<camera>/preview/compressed` | `sensor_msgs/CompressedImage` | JPEG, `preview_width` wide, every `preview_period` s |
| `/sensors/camera/<camera>/image_raw` | `sensor_msgs/Image` | bgr8, with `publish_image` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz, the Python's keys and formats |
| `/system/source` | `aav_interfaces/DataSource` | latched |

Parameters: the Python's, with the same defaults. `model`, `device`, `confidence`,
`labels`, `image_size` are declared and not used (no model).

## Invariants

- The same names, topics, messages and QoS as the Python node: nothing downstream can tell.
- The logic lives in `stream-core`, with no ROS and no OpenCV, and its own tests.
- `gen_vectors.py` runs the Python modules; `tests/vectors.rs` must give their answers:
  command lines, choices, frames, rates, delays, stamps, time codes, lenses, undistorted
  pictures (floats to 1e-12; pictures grey level by grey level).
- The decoder runs in its own process group, stopped by group (SIGTERM, then SIGKILL).

## Side Effects

- Starts a decoder child program (ffmpeg or `gst-launch-1.0`); opens the RTSP stream.
- Reads the calibration file, if given.

## Verification

```sh
cd ros/sources/aav_camera_stream_rs && cargo test -p stream-core      # no ROS
pixi run python ros/sources/aav_camera_stream_rs/gen_vectors.py        # after a Python change
pixi run build && colcon test --packages-select aav_camera_stream_rs aav_camera_stream
```

`test/test_camera_stream_node.py` runs both nodes on a GStreamer test pattern;
`aav_camera_stream`'s `test/test_launch_reaches_node.py` checks the launch file's values
reach either node. End to end with MediaMTX: `pixi run measure stream --detector none
--impl rust` (or `--impl python`).
