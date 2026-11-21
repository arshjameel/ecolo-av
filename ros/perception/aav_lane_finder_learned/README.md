# aav_lane_finder_learned

## Overview

**The learned lane finder** (TODO 262b), a challenger: `lane_finder_impl:=learned`. A
small model of our own, in the style of Ultra-Fast-Lane-Detection v2 (Qin et al., TPAMI
2022, MIT), trained by us on Ravensim's pictures, labelled for free from the lane map.
It publishes the **same** `LaneGeometry` as the classic finder, fitted the same way.

Ben, 19 Nov: "We also need to port the lane following code.or find a better version", and
"I have a fair amount of cores and have neurla net cores too. maybe testing ml stuff
instead". Published weights are out: CULane is non-commercial; BDD100K, TuSimple and
CurveLanes have no clear licence ([the survey](../../../docs/perception.md#a-learned-lane-finder-the-survey-19-nov)).

```
camera picture ─► rows below the horizon, 480 x 144 ─► the model (ONNX Runtime)
                                                           │ 4 lines x 42 rows: which column,
                                                           ▼ and is the line there
/perception/lane/geometry ◄── lane_from ◄── a fit per line ◄── each point onto the road
       (the classic finder's message, lane-core's fit and lane)    (the lens and mount)
```

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/ufld-core` | the crop and scale (Pillow's, to the grey level), decoding, the ground, the lane; the model through `onnx-session` (feature `ort`) | no: `cargo test` |
| `crates/ufld-node` | the node, binary `lane_finder` | yes (rclrs 0.8) |

It shares `lane-core` (`aav_lane_finder_rs`: the lens, the mount, `fit_line`, `lane_from`)
and `onnx-session` (`aav_detector_rs`: the provider, Core ML's cache, the coverage).

### Node `lane_finder` (learned)

The same name, topics and QoS as the classic one ([aav_lane_finder](../aav_lane_finder/README.md)):
the front left camera's `image_raw` and `camera_info` in; `/perception/lane/geometry` out.
`aav_lane_finder`'s launch file starts it with `lane_finder_impl:=learned`, with the same
`lane_finder.yaml` and the camera's place from `vehicle.yaml`. No debug picture.

| Parameter | Default | Means |
|---|---|---|
| `model` | `models/lane-ufld.onnx` | the model; its `.json` beside it says what it was made for |
| `device` | `auto` | `cpu`, `coreml` (a Mac), `cuda`, `tensorrt` (the Orin, untested) |
| `exist_threshold` | 0.5 | a row counts when the model is surer than this that the line is there |
| `min_confidence`, `fit_scale`, `min_span`, `lane_width`, `min_lane_width`, `max_lane_width`, `lookahead`, `wheelbase`, `camera_xyz`, `camera_rpy`, `focal_px`, `use_camera_info` | `lane_finder.yaml`'s | as the classic finder's |

A line's confidence: how close its points lie to the fit (the classic finder's measure) times
the model's mean chance that the line is there.

## The data

`tools/lane_data.py` (no ROS; tested in `tools/tests/test_lane_data.py`):

| Step | What |
|---|---|
| poses | anywhere across the painted roads' tarmac, turned up to 0.25 rad, both ways; none where a crossing road's paint is in view |
| pictures | `sim_cameras --poses` (Bevy, offscreen, the GPU): the front left camera at each pose |
| labels | the lane map's painted lines (the road world's; the campus's white edges and yellow dashed centre, as `campus_world.py` paints them) through the camera's lens and mount: each line's column at 42 rows |
| lines | 4: the car's lane's left and right, and the next line out on each side; white or yellow |
| left out | pictures with grass, not tarmac, just ahead (roads the world draws no tarmac for) |
| held out | the campus's every seventh road (never trained on) and 15 % of the road world |

The set (19 Nov): **7,472 pictures**, 1,412 from the road world and 6,060 from the campus;
1,286 held out. 99 % show the car's two lines, 51 % a neighbour. About 6 minutes on an M4
Max, most of it labelling.

## The model and its scores

`tools/lane_train.py`: ResNet-18 (torchvision's, from random weights: no ImageNet), a
1 x 1 convolution, two layers to 4 lines x 42 rows x 100 columns, and each row's "is it
there". UFLD v2's losses (the column as a class, its expected value, the existence) and a
colour. Augmented: colour, light, blur, noise, mirrored. 40 epochs on the Mac's GPU
(`mps`), about 25 minutes.

The scores on the 1,286 held-out pictures (19 Nov): each line's points on the road, a fit
between 4 and 25 m, against the lane map's line at 5, 10 and 15 m ahead.

| Line | Found (of those in the map) | Error, median (m): 5 / 10 / 15 m ahead | 90th percentile (m) |
|---|---|---|---|
| **the car's left** | **100 %** of 1,277 | **0.012 / 0.021 / 0.035** | 0.18 / 0.25 / 0.35 |
| **the car's right** | **99.9 %** of 1,278 | **0.013 / 0.022 / 0.036** | 0.23 / 0.21 / 0.30 |
| the next one left | 92.9 % of 606 | 0.12 / 0.05 / 0.09 | about 3 (often the wrong line) |
| the next one right | 97.6 % of 613 | 0.12 / 0.05 / 0.08 | about 3 (often the wrong line) |

- Paint colour (white or yellow) right for 95 % of the car's two lines. Since 19 Nov the
  node publishes it as `left_paint` and `right_paint` (the `colour` output, `paints`), so
  the costmap's camera lanes (`lanes:=camera`) can tell the yellow centre line; a model
  exported without the colour head leaves them UNKNOWN. On lane-core's University Drive
  frames: the left line yellow, the right white, in all three (`tests/model.rs`).
- A line found where the map has none: 2.1 % (left neighbour), 12 % (right neighbour); for
  the car's own lines only 9 pictures lack one, and the model found a line in 5 and 5.
- On lane-core's Ravensim frames (University Drive) both lines, a lane 2.9-3.0 m wide; on a
  blank road, none (`tests/model.rs`).
- Training loss falls to almost 0: it learns Ravensim by heart. The held-out roads say it
  still generalises across roads; the car's pictures (TODO 252) will say more.

### On ONNX Runtime (`pixi run measure onnx --models lanes`)

M4 Max, 19 Nov, median of 30 runs, the model alone. 190 MB (FP16: 95 MB). **PASS**.

| File | Provider | Answers vs the CPU | Nodes on the provider | ms |
|---|---|---|---|---|
| FP32 | CPU | the reference | all | 11.4 |
| FP32 | Core ML | the same (0.00 px) | **59 of 59, 1 piece** | **2.4** |
| FP16 | CPU | the same rows, 0.06 px | all | 10.8 |
| FP16 | Core ML | the same rows, 0.04 px | **63 of 63, 1 piece** | **1.3** |
| PyTorch (`mps`) | | the same (0.00 px) | | 5.5 |

| Core ML's units | FP32 | FP16 |
|---|---|---|
| all (what we ask) | 2.5 | **1.2** |
| CPU and GPU | 2.4 | 2.1 |
| CPU and **Neural Engine** | 4.8 | **1.2** |
| CPU only | 4.7 | 3.0 |

**The Neural Engine takes the FP16 model whole** (1.2 ms, within 0.04 px of the CPU's
answers); the FP32 one runs on the GPU. The first export had one `Flatten` that Core ML
handed back to the CPU (2 pieces); a fixed `Reshape` made it one piece.

## Retrain

```sh
cd ros/sim/aav_sim_cameras && pixi run cargo build --release && cd -   # sim_cameras
pixi run campus                                                          # maps/campus, once
pixi run -e ml python3 tools/lane_data.py          # models/lane-data/ (outside git)
pixi run -e ml python3 tools/lane_train.py         # models/lane-ufld.{pt,onnx,json}, _fp16
pixi run measure onnx --models lanes               # CPU against Core ML, the coverage, ms
```

In a worktree, point `lane_data.py` at the main checkout's campus:
`--campus-dir ~/Code/ecolo-av/maps/campus`.

## Invariants

- The same message, frame and stamp as the classic finder; only one runs (`lane_finder_impl:=`).
- The crop and scale give the model Pillow's numbers exactly (`tests/pillow.rs`, from
  `gen_pillow.py`): it was trained on them.
- `ufld-core` has no ROS; `cargo fmt` and `cargo clippy -D warnings` clean.
- The model and its data stay out of git (`models/`).
- REP-103: x forward, y left, metres.

## Not yet

| What | Waits on |
|---|---|
| Driving with it in Ravensim, scored against the classic finder (`measure lane`) | stage 2 |
| The car's pictures (the labelling loop, TODO 252) | recordings from the car |
| Built on Linux (`-DLANE_LEARNED=ON`), TensorRT on the Orin | ONNX Runtime on the car kit, as the Rust detector |
| Hills: the ground is taken to be flat, as the classic finder's | the IMU's pitch |

## Verification

```sh
cd ros/perception/aav_lane_finder_learned
pixi run cargo test -p ufld-core                      # the maths
pixi run cargo test -p ufld-core --features coreml    # and the model, if models/ has it
pixi run build && colcon test --packages-select aav_lane_finder_learned
```
