# ONNX models from Rust

Train in PyTorch; run the model as **ONNX**, from Rust, with ONNX Runtime (Ben, 16 Nov,
TODO 243). First on a Mac through Core ML; on the Orin later through TensorRT.

Three models, end to end, each a challenger beside the default:

| Model | From Rust | Switch |
|---|---|---|
| the **RF-DETR Nano detector** (the stop signs) | the node `aav_detector_rs` | `detector_impl:=rust` |
| the **diffusion planner** (its model; the guard stays) | the worker `aav_diffusion_rs` | `diffusion_impl:=rust` |
| the **learned lane finder** (TODO 262b, ours, trained here) | the node `aav_lane_finder_learned` | `lane_finder_impl:=learned` |

Ben, 19 Nov: "if it works on a mac it should work there correct?" Mostly: **one `.onnx`
per model**; what Core ML runs whole is the best guess of what TensorRT will. So every
model is checked here first: `pixi run measure onnx`.

## Every model: `pixi run measure onnx`

Ben's MacBook (M4 Max), 19 Nov, ONNX Runtime 1.31; median of 30 runs, after 5 warm-up
runs; the model alone. "Answers" against the CPU's float32 file.

| Model, file | Provider | Answers | Nodes on the provider | Fell back to the CPU | ms |
|---|---|---|---|---|---|
| detector, FP32 | CPU | the reference | all | - | 25.9 |
| detector, FP32 | Core ML | same (IoU 1.0000) | 802 of 808 (99.3 %), 2 pieces | Expand 2, GatherElements 2, TopK 1, Unsqueeze 1 | 9.6 |
| detector, FP32 | PyTorch (`mps`) | same (IoU 1.0000) | | | 15.8 |
| detector, FP16 | CPU | within (IoU 0.978) | all | - | 52.6 |
| detector, FP16 | Core ML | **out**: 7 boxes lost | 860 of 871, 5 pieces | Cast, Greater, Less, Not... | 9.6 |
| diffusion, FP32 | CPU | the reference | all | - | 63.5 |
| diffusion, FP32 | Core ML | same (0.000 m) | **2,178 of 2,178, 1 piece** | nothing | **9.7** |
| diffusion, FP32 | PyTorch (`mps`) | same (0.000 m) | | | 22.9 |
| diffusion, FP16 | CPU | within (0.080 m, 0.004 rad) | all | - | 66.0 |
| diffusion, FP16 | Core ML | within (0.036 m, 0.002 rad) | 2,204 of 2,204, 1 piece | nothing | 9.4 |
| lanes, FP32 | CPU | the reference | all | - | 11.4 |
| lanes, FP32 | Core ML | same (0.00 px) | **59 of 59, 1 piece** | nothing | **2.4** |
| lanes, FP32 | PyTorch (`mps`) | same (0.00 px) | | | 5.5 |
| lanes, FP16 | CPU | within (0.06 px) | all | - | 10.8 |
| lanes, FP16 | Core ML | within (0.04 px) | **63 of 63, 1 piece** | nothing | **1.3** |

| Verdict | Detector | Diffusion | Lanes |
|---|---|---|---|
| `measure onnx` | **FAIL** (its FP16 on Core ML); FP32 alone: WARN (6 nodes on the CPU) | **WARN** only for the log; every file whole on Core ML, in tolerance | **PASS**: both files whole on Core ML, in tolerance |
| TensorRT's chance (a guess, untested) | good: 6 shape steps on the CPU | good: whole graph, plain ops | good: a ResNet-18 and two dense layers |

The rules (the same for FP16): the detector, every box over 0.5 has a twin, IoU ≥ 0.95,
score within 0.02 (7 test pictures); the diffusion planner, 16 paths on 10 made-up plans,
each point within 0.10 m, each heading within 0.05 rad, 99 % in the same gear; the lanes,
5 Ravensim frames and 30 held-out pictures, 98 % of the rows' "a line here?" the same and
every column within 3 px. PASS: in
tolerance and whole on the provider; WARN: in tolerance, some nodes on the CPU; FAIL: out
of tolerance, or it can't run. `--json FILE` keeps it all.

The same from Rust, any model: `cargo run --release -p onnx-session --features coreml
--example measure -- <model.onnx>` (in `ros/perception/aav_detector_rs`).

## The diffusion planner from Rust

Only the model moved: `aav_diffusion_rs`'s `diffusion_worker` speaks the Python worker's
pipe, so the node, its window, its guard (G1-G8) and the lattice's fallback are unchanged.
`diffusion_impl:=rust diffusion_provider:=auto`.

| | ms a plan (16 paths, 10 steps) | Paths against the CPU's |
|---|---|---|
| Python worker, ONNX Runtime, CPU (today's default) | 63-87 | the reference |
| PyTorch (`mps`) | 22.9 | same (0.000 m) |
| **Rust, Core ML** (all 2,178 nodes, one piece) | **9.7** | same (6e-7 in the window's units) |
| Rust, CPU | 87 | the same session |

To get there the export changed (19 Nov), the same numbers: 16 samples fixed (Core ML
can't build an open number; another number runs in batches), and no ScatterND, Expand,
Flatten, 1D AveragePool or Resize (the 0.3 export went to Core ML in 376 pieces, slower
than the CPU). The model was trained again on this Mac (`pixi run diffusion data`,
`train --epochs 40`, 57 min): 58.6 % of samples pass the guard, 84.7 % of plans have one
(0.3's: 63 %, 87 %).

**Found on the way**: the node asked for the global costmap as "transient local"; ours
(Rust, since 16 Nov) sends it "volatile", so it never arrived and every plan was the
lattice's. Fixed: the node reads it volatile. A first look after the fix (one tour, cut
short): 2 of 4 plans a leg from the model. **Ravensim scoring deferred** (Ben, 19 Nov:
algorithm testing comes after the ports): `pixi run measure controllers --planners
diffusion diffusion_rs --reversing --tours lot --runs 2`.

## FP16

`tools/export_model.py --format onnx --fp16` (the detector; ONNX Runtime's converter) and
`pixi run diffusion export --fp16` (the diffusion planner) write `*_fp16.onnx`: half
precision inside, float32 in and out.

| Core ML's units | detector FP32 | detector FP16 | diffusion FP32 | diffusion FP16 | lanes FP32 | lanes FP16 |
|---|---|---|---|---|---|---|
| all (what we ask) | 9.7 | 9.7 | 9.7 | 9.5 | 2.5 | **1.2** |
| CPU and GPU | 9.8 | 8.9 | 9.7 | 9.4 | 2.4 | 2.1 |
| CPU and **Neural Engine** | 28.8 | **9.6** | 21.7 | 27.2 | 4.8 | **1.2** |
| CPU only | 28.8 | 19.8 | 21.6 | 18.2 | 4.7 | 3.0 |

- **The Neural Engine runs the FP16 detector** (9.6 ms against the CPU's 19.8) but not the
  FP32 one, nor the diffusion planner (no faster than the CPU).
- **The Neural Engine runs the FP16 lane finder** too, whole: 1.2 ms, half the GPU's FP32
  time, within 0.04 px; with all units Core ML picks it itself.
- **On this Mac FP16 buys nothing** for the detector and the planner: with all units Core ML
  picks the GPU, 9-10 ms either way.
- **The detector's FP16 loses boxes on Core ML** (fine on the CPU): keep FP32 on the Mac.
  On the Orin, TensorRT makes its own FP16 from the FP32 file (`trt_fp16_enable`), so the
  FP16 files are only for checking.

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
| The Rust with `--features tensorrt` (and `cuda`): `onnx-session`, the detector, the diffusion worker | [[untested]] compiles (`cargo clippy`) on the Mac; never linked or run |
| An ONNX Runtime with TensorRT for the Orin | [[untested]] `ort` downloads pyke's ARM Linux build with CUDA 13 and TensorRT: it may fit JetPack 7.2 (CUDA 13, Q24), not JetPack 6 (CUDA 12) |
| The same code picks TensorRT | [[untested]] ready: `device:=tensorrt` (detector), `diffusion_provider:=tensorrt` (Rust worker); `auto` picks it when the build has it and no Core ML |
| Its engine cache | [[untested]] ready: FP16 on, the engine and the timing cache kept beside the model: `models/<model>.cache/tensorrt/`. The first start builds them (minutes); a new TensorRT (a JetPack upgrade) builds them again: delete the folder |
| What TensorRT will take | a guess from Core ML: the diffusion model whole (one piece, plain ops), the detector all but 6 shape steps |

Steps for a car day (the Orin):

| # | Do | Expect |
|---|---|---|
| 1 | copy `models/rf-detr-nano.onnx` to the Orin's `models/` | 108 MB |
| 2 | JetPack 7: `colcon build --packages-select aav_detector_rs --cmake-args -DDETECTOR_FEATURES=tensorrt` (downloads pyke's CUDA 13 build). JetPack 6: first get an ONNX Runtime with TensorRT for it (Jetson Zoo, or build it), then the same with `ORT_LIB_LOCATION=<its folder>` | builds |
| 3 | `ros2 launch aav_detector detector.launch.py detector_impl:=rust device:=tensorrt picture:=ros/perception/aav_detector/test/stop_sign.jpg` | a stop sign on `/perception/detections`; the first start builds the engine (minutes), kept in `models/rf-detr-nano.cache/tensorrt` |
| 4 | write down the log's "frames in 10 s, ms each" | against the Python's TensorRT ([car day](car-day.md)) |
| 5 | copy `models/diffusion/planner.onnx`; `colcon build --packages-select aav_diffusion_rs --cmake-args -DDIFFUSION_RS=ON -DDIFFUSION_FEATURES=tensorrt` | builds `diffusion_worker` |
| 6 | `cargo run --release -p diffusion-core --features tensorrt --example bench -- <repo>/models/diffusion/planner.onnx` (in `ros/drivers/aav_diffusion_rs`) | the CPU and TensorRT rows; the budget is 50 ms for the model |
| 7 | the trial, as [car day A12](car-day.md#a12-the-diffusion-planners-trial), with `diffusion_impl:=rust diffusion_provider:=tensorrt` | the status's ms a plan |

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
| the diffusion planner's node itself (the window and the guard) in Rust | once the Rust worker wins in the sim and on the car |
| the detector's FP16 on Core ML (boxes lost) | if the Neural Engine is wanted on a Mac |
| the lane finder (once it is a model) | later |

## Where the Rust detector is built (17 Nov)

| Computer | Built? |
|---|---|
| macOS | yes, by default (ONNX Runtime + Core ML) |
| Linux: CI, the car kit, the Orin | only with `colcon build --cmake-args -DDETECTOR_RS=ON` |

Why: on 17 Nov ort's downloaded ONNX Runtime didn't link with conda's gcc on GitHub's Linux
runners (x86 and ARM), and the car kit installs offline. Until it is proven there, Linux runs
the Python detector; `detector_impl:=rust` stays a challenger.
