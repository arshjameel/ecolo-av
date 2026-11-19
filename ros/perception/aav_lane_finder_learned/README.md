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

SCORES

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
