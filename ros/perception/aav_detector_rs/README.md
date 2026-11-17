# aav_detector_rs

## Overview

The object detector in Rust (rclrs): a **drop-in challenger** for `aav_detector`'s Python
node. RF-DETR Nano, exported to ONNX, run by ONNX Runtime through the `ort` crate (Ben,
16 Nov, TODO 243). The same node name, parameters and topics. The Python stays the
default. Why, and the numbers: [docs/onnx.md](../../../docs/onnx.md).

```
image (the front camera) ──► detector (Rust) ──► /perception/detections
                                  └────────────► /perception/detections/image (boxes outlined)
```

Pick it with `detector_impl:=rust` (`detector.launch.py`; `sim.launch.py` with
`perception:=true`). It reads `models/rf-detr-nano.onnx` (not in git: `tools/export_model.py
--format onnx`).

| Where | Runs on | Tested |
|---|---|---|
| a Mac | Core ML's provider: 802 of 808 nodes, on the GPU | [[works]] same boxes as the Python; Ravensim's stop sign |
| any computer | the CPU provider | [[works]] same boxes as the Python |
| the Orin | TensorRT's provider (`-DDETECTOR_FEATURES=tensorrt`) | [[untested]] the Rust compiles; never linked or run |

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/detector-core` | resize, normalise, decode, labels, outlines; with `--features ort`, the model | no: `cargo test` on any laptop |
| `crates/detector-node` | the node, binary `detector` | yes (rclrs 0.8, `ros-env` 0.3) |

colcon builds it through `CMakeLists.txt` (`cargo build --release --locked`; Core ML is in
on a Mac). The first build downloads ONNX Runtime (pyke's build, for this computer).

### Node `detector` (runs as `object_detector`)

| In / out | Topic | Type |
|---|---|---|
| in | `image` (launch: `/sensors/camera/front_left/image_raw`) | `sensor_msgs/Image`, rgb8 or bgr8, reliable, depth 1 |
| in | each topic in `extra_images` | the same |
| out | `/perception/detections` | `vision_msgs/Detection2DArray`, the picture's header |
| out | `/perception/detections/image` | `sensor_msgs/Image`: boxes outlined in red, no text |

| Parameter | Default | Means |
|---|---|---|
| `detector` | `rfdetr` | `rfdetr` only (YOLO is the Python's) |
| `model` | empty | an `.onnx`; empty = `models/rf-detr-nano.onnx`, from the repo's root |
| `device` | `auto` | `auto`, `cpu`, `coreml` (or `mps`), `cuda`, `tensorrt` |
| `confidence`, `labels`, `publish_image`, `extra_images` | 0.5, empty, true, empty | as the Python's |
| `compressed` | false | true stops the node: JPEG pictures need the Python |

## Invariants

- The same name, topics and messages as the Python node: nothing downstream can tell.
- The maths lives in `detector-core`, with no ROS, and its own tests.
- Only the providers asked for run: `ort`'s own default (Core ML on every Mac, even for
  the CPU) is switched off.

## Side Effects

- Loads `models/rf-detr-nano.onnx`; Core ML and TensorRT keep what they compile in
  `models/rf-detr-nano.cache/` (outside git). Core ML's first start: about 30 s; then 6 s.
- Logs the frame rate every 10 s.

## Verification

```sh
cargo test -p detector-core                                   # the maths (tools/test.sh)
cargo test -p detector-core --features ort,coreml             # + the Python's boxes
cargo run --release -p detector-core --features ort,coreml --example timing
pixi run -e ml sim-check --perception --with detector_impl:=rust
```
