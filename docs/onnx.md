# ONNX models from Rust

Train in PyTorch; run the model as **ONNX**, from Rust, with ONNX Runtime (Ben, 16 Nov,
TODO 243). First on a Mac through Core ML; on the Orin later through TensorRT.

The first model done end to end: the **RF-DETR Nano detector** (the stop signs). The
Rust node is `aav_detector_rs`, a challenger beside the Python one.

```
PyTorch (.pth) ──export──► ONNX (.onnx) ──► ONNX Runtime ──► Core ML    (a Mac: GPU)
   training                 one file          (ort crate)  ├► CPU        (anywhere)
                                                           └► TensorRT  (the Orin: untested)
```

## Why

| Today (Python) | With ONNX from Rust |
|---|---|
| one export per runtime: `.mlpackage` (Mac), `.engine` (Orin) | one `.onnx`; the runtime picks the hardware |
| PyTorch and rfdetr in the `ml` environment, 2 GB | one Rust binary; ONNX Runtime linked in |
| a Python node | a Rust node, as the rest of our hot paths (`aav_voxels_rs`, `aav_ego_rs`) |

## How

| Step | Command |
|---|---|
| 1. the model | `pixi run get models` (gets `models/rf-detr-nano.pth`) |
| 2. export | `pixi run -e ml python3 tools/export_model.py --format onnx` |
| 3. check it | `pixi run -e ml python3 tools/onnx_check.py --time 30` |
| 4. run it | `pixi run -e ml sim world:=road perception:=true detector_impl:=rust` |

The ONNX file:

| | |
|---|---|
| file | `models/rf-detr-nano.onnx`, 108 MB, **not in git** (`models/` is ignored) |
| made by | rfdetr 1.11's `model.export()`, PyTorch 2.13, in 5 s |
| opset | 17 (ai.onnx), IR 8; 1,424 nodes (808 after ONNX Runtime's clean-up) |
| input | `input`: float32 (1, 3, 384, 384): RGB, 0-1, ImageNet mean and spread |
| outputs | `dets` (1, 300, 4): centre x, centre y, width, height, 0-1; `labels` (1, 300, 91): logits, COCO's numbers |

## Same boxes?

Seven saved pictures: Ravensim's front camera at 5, 10, 14 and 18 m from the stop sign
(simple and pretty graphics), and a photo. Every box has a twin of the same label.

| Against PyTorch | Worst IoU | Worst score gap |
|---|---|---|
| Python, ONNX Runtime, CPU | 1.0000 | 0.0000 |
| Python, ONNX Runtime, Core ML | 1.0000 | 0.0000 |
| Python, Core ML (`.mlpackage`, today's Mac path) | 1.0000 | 0.0000 |
| **Rust**, ONNX Runtime, CPU (against the Python's) | 0.9999 | 0.0000 |
| **Rust**, ONNX Runtime, Core ML (against the Python's) | 0.9999 | 0.0000 |

The test asks for IoU ≥ 0.95 and scores within 0.02 (`cargo test --features ort,coreml`).

!!! warning "ort's default adds Core ML"
    `ort` 2.0.0-rc.15, on any Mac, quietly adds Core ML in its older, half-precision
    format, even to a "CPU" session: boxes then moved by up to 2 % (IoU 0.978). We switch
    that off (`AutoDevicePolicy::PreferCPU`): only the provider asked for runs.

## How fast?

Ben's MacBook (M4 Max), 17 Nov; 640 x 360 pictures; each time includes the resize and
the decode; median of 150, after 5 warm-up runs. Other work was running (load 3-5).

| Runs | ms per picture |
|---|---|
| Python, PyTorch on the GPU (`mps`): today's default | 14.7 |
| Python, Core ML `.mlpackage` (coremltools) | 11.0 |
| Python, ONNX Runtime, Core ML | 12.8 |
| Python, ONNX Runtime, CPU | 30.9 |
| **Rust, ONNX Runtime, Core ML** | **8.0** (90 % under 8.4) |
| Rust, ONNX Runtime, CPU | 31.3 |

Starting up, Rust: the CPU 0.1 s; Core ML 29 s the first time (it compiles the model), then
5.6 s (its compiled model kept in `models/rf-detr-nano.cache/`).

### Where Core ML runs it

| Core ML's units | ms per picture | So |
|---|---|---|
| all (what we ask) | 8.0 | |
| CPU and GPU | 9.2 | ≈ all: **the GPU does the work** |
| CPU and Neural Engine | 28.1 | ≈ the CPU alone: the Neural Engine takes none of it |
| CPU only | 28.0 | |

ONNX Runtime's log: Core ML takes **802 of the 808 nodes**, in 2 pieces; the CPU runs the
other 6 (shape steps). The Neural Engine works in half precision; this model is full
precision, so Core ML keeps it on the GPU. A half-precision export might reach the Neural
Engine: [[later]].

## In the simulator

`pixi run -e ml sim-check --perception`, Ravensim's road world, 17 Nov: all 10 checks pass
with either detector. The node's own log (a whole picture: in, model, boxes, the picture
with boxes out), while Ravensim draws its cameras on the same GPU:

| Detector | First saw the sign | Locked (score) | Stood still | ms a picture in the sim |
|---|---|---|---|---|
| Python, PyTorch on the GPU (today) | x = 5.8 m | 18.5 m (0.93) | 4.3 m before it | 24-29 |
| Rust, `detector_impl:=rust`, Core ML | x = 6.1 m | 18.5 m (0.92) | 4.3 m before it | 18-19 |

Its first start there took 34 s (Core ML compiling; a model at a new path compiles again).

## Untested: the Orin

| What | State |
|---|---|
| The Rust with `--features tensorrt` (and `cuda`) | [[untested]] compiles (`cargo clippy`) on the Mac; never linked or run |
| An ONNX Runtime with TensorRT for the Orin | [[untested]] `ort` downloads pyke's ARM Linux build with CUDA 13 and TensorRT: it may fit JetPack 7 (CUDA 13, Q24), not JetPack 6 (CUDA 12) |

Steps for a car day (the Orin):

| # | Do | Expect |
|---|---|---|
| 1 | copy `models/rf-detr-nano.onnx` to the Orin's `models/` | 108 MB |
| 2 | JetPack 7: `colcon build --packages-select aav_detector_rs --cmake-args -DDETECTOR_FEATURES=tensorrt` (downloads pyke's CUDA 13 build). JetPack 6: first get an ONNX Runtime with TensorRT for it (Jetson Zoo, or build it), then the same with `ORT_LIB_LOCATION=<its folder>` | builds |
| 3 | `ros2 launch aav_detector detector.launch.py detector_impl:=rust device:=tensorrt picture:=ros/perception/aav_detector/test/stop_sign.jpg` | a stop sign on `/perception/detections`; the first start builds the engine (minutes), kept in `models/rf-detr-nano.cache/tensorrt` |
| 4 | write down the log's "frames in 10 s, ms each" | against the Python's TensorRT ([car day](car-day.md)) |

## Licences

| Part | Licence |
|---|---|
| ONNX Runtime (Microsoft) | MIT |
| `ort` crate (pyke) 2.0.0-rc.15 | MIT or Apache-2.0 |
| ONNX, onnxscript | Apache-2.0, MIT |
| RF-DETR (Roboflow), the model and its weights | Apache-2.0 |

All in [acknowledgements](acknowledgements.md).

## Next

| Model | When |
|---|---|
| the diffusion planner | after the detector wins on the car |
| the lane finder (once it is a model) | later |
