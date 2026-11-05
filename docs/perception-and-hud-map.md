# Perception models, the lidar, and a map on the HUD

> **A snapshot from 15 October 2026.** What's true now: [Status](status.md), [Perception](perception.md), [Camera pipeline](camera-pipeline.md) and [the HUD's map window](hud.md#the-map-window-big-screen).

Written 15 October 2026. **Since then:** the detector is RF-DETR Nano, not YOLO
(Ben, 23 Oct), as TensorRT on the Orin and Core ML on a Mac: see
[Perception](perception.md#models). YOLO stays as `detector:=yolo`. It answers:

1. YOLO, OpenCV, or the previous teams' trained models?
2. What do we do with the lidar? Can we just download a model?
3. Can the HUD show a map?
4. Are there better open-source models, like NVIDIA's per-pixel ones?

> **The short version**
> - **Use stock YOLO for objects** (answered 23 Oct: RF-DETR Nano by default; YOLO with `detector:=yolo`): one model, run as TensorRT on the Orin and Core ML on a Mac. **Keep OpenCV for lane lines** (already done: under 1 ms a frame).
> - **The previous teams' trained model is lost** (`best.pt` was on someone's laptop) and knew one label. Start from stock, and train our own later only if the stock one fails on our recordings.
> - **Lidar: no model to start.** Put its points straight into Nav2's costmap and collision_monitor. Clustering next. A downloaded 3D detector last, and only if needed.
> - **Per-pixel, for where the road is: NVIDIA's CitySemSegFormer** (a ready ONNX file), run with ONNX Runtime, not Isaac ROS: Isaac ROS isn't on Jazzy for the Orin yet (since then: Isaac ROS 4.6 supports the Orin on JetPack 7.2, [Perception](perception.md#isaac-ros-nvidia)).
> - **A map on the HUD: yes.** The raylib HUD can draw offline map tiles with the car's GPS dot on top. (Answered: built 15-16 Oct as [the map window](hud.md#the-map-window-big-screen).)

## 1. Camera perception: which tool for which job

| Job | Tool | Why | Status |
|---|---|---|---|
| Stop signs, people, cars, bikes, traffic lights | **YOLO11 nano**, stock, trained on COCO (80 classes) | one model for many objects; fast; runs on TensorRT and Core ML | ✅ bench, Gazebo road world; ❌ car |
| Lane lines | **OpenCV**, classic (`aav_lane_finder`) | no training; 0.78 ms a frame; easy to debug | ✅ bench |
| How far away is it? | the lidar or radar (see 3), not the camera | a camera alone guesses distance badly | - |

### What the previous teams trained

| Team | Model | Where it is now |
|---|---|---|
| 2024-25 | their own YOLO, `best.pt`, one label, from their own photos | **lost**: their code loaded it from `~/Yolo/Capstone/runs/...` on someone's computer. The training scripts came across; the pictures and weights didn't |
| 2022-23 | Colab notebooks for vision models (`Carleton-AAV/AI-Development-Scripts`) | notebooks only |

We don't know what they started from or how good it was. Stock YOLO11 already knows "stop
sign", so we start there.

### One model, three formats

| Computer | Format | Made by |
|---|---|---|
| Orin | `yolo11n.engine` (TensorRT, half precision) | `python3 tools/build_engine.py`, on the Orin itself |
| Mac | `yolo11n.mlpackage` (Core ML) | `pixi run -e ml python3 tools/export_model.py --format coreml` |
| Anything | `yolo11n.pt` (PyTorch) | `pixi run get models` |

Measured on a Mac: 7.7 ms a frame through PyTorch and Metal, 4.3 ms through Core ML.

### When to train our own

1. Record the car's cameras on a real drive (`tools/record.sh`).
2. Run stock YOLO on the recording. Count what it misses.
3. Only if it misses too much: label a few hundred of our frames and fine-tune YOLO11 from the stock weights. It's the same code, the same three formats.

Other open models worth a fair test on the same recording: RT-DETR and RF-DETR (objects),
YOLO-World (anything named in words, such as "traffic cone"), YOLOPv2 (lanes and drivable
area), Depth Anything V2 (distance from one camera). Check each licence first: Ultralytics'
YOLO is AGPL-3.0.

## 2. The cameras on the car: one thing to fix first

The photos show **ribbon (CSI) cameras** on the Pi 5s. Our camera driver, `usb_cam`, only
reads **USB** cameras. The camera Pis need a libcamera driver instead, such as `camera_ros`.
(Answered: the audits of 16 and 21 Oct found MediaMTX on the camera Pi. Both front ribbon
cameras are on one Pi 5: MediaMTX → `aav_camera_stream` on the Orin, [Camera pipeline](camera-pipeline.md).)

## 3. The lidar: three steps

| Step | What | Model? | Gives | Effort |
|---|---|---|---|---|
| **1. Points into Nav2** | the lidar's point cloud → the local costmap's obstacle layer, and collision_monitor's stop zone | no | "something is there, don't drive into it" | small: configuration, like the radar's |
| **2. Clusters** | group nearby points into objects (DBSCAN, as we already do for the radar) | no | boxes with a position and size | small: reuse `aav_radar_clusters` |
| **3. A 3D detector** | a downloaded model labels the points: car, person, cyclist | yes | labelled 3D boxes | big |

Step 1 is what keeps the car from hitting things, and it needs no model at all. Most
student and research cars stop there.

### "Can we download a random model?"

Yes, but it's step 3, and it comes with catches:

| Model family | Trained on | The catch |
|---|---|---|
| PointPillars | KITTI, nuScenes (car datasets) | trained on a different lidar, mounted differently: expect worse results on ours |
| CenterPoint | nuScenes, Waymo | heavier; wants a big GPU |
| NVIDIA TAO PointPillars | NVIDIA's data | built for TensorRT on Jetson: the best fit for the Orin |

### Camera + lidar together

The usual way: YOLO finds a box in the picture; the lidar points that fall inside that box
give its distance. That needs the camera and lidar **calibrated** to each other (where each
one sits on the car, and the camera's lens). It's a later step, after both run on the car.

### In Gazebo today

`pixi run sim world:=road` (or `world:=test_lot`) simulates a generic 16-beam lidar on the roof, on
`/sensors/lidar/points`. It's on by default (15 Oct): with the Mac quiet, it runs with
YOLO and all three cameras. Step 1 can be built and tested there before the real lidar's model is
known.

## 4. A map on the HUD

### What exists

| Piece | Status |
|---|---|
| The 2025-26 Flutter app's map page (`flutter_map`) | written; it needs a tile server on the dash Pi, which isn't in git (answered: archived 25 Oct, Q29) |
| `ottawa.db`: 606,628 places for the search box | `pixi run get places` |
| OSRM road routes | Docker on the dash Pi; data not in git (answered: [Street routes](routes.md), Valhalla, since) |
| The raylib HUD (ours) | speed, e-stop, GPS panel, camera, topic health. **No map yet** (answered: [the map window](hud.md#the-map-window-big-screen), built since) |

### How the raylib HUD would draw one

```text
 map tiles (256 px squares, offline) ─► raylib textures ─► the map panel
 /localization/gps/fix ─► a dot and an arrow for the car
 the route (lat/lon points) ─► a line over the tiles
```

| Step | Does |
|---|---|
| 1 | Make offline **tiles** of the test area once: a folder of PNGs (zoom 15-19 covers a campus) |
| 2 | The HUD loads the tiles around the car as textures |
| 3 | lat/lon → tile pixel is a standard formula (Web Mercator) |
| 4 | Draw the car's dot, its heading, and the route on top |

- **Offline**, so it works without internet in the car.
- **The rules:** OpenStreetMap's own tile servers forbid bulk downloads. Render our own tiles from OSM data, or use a provider that allows offline use. Show "© OpenStreetMap contributors" on the screen, as the Flutter app does.
- **About a day's work** for a map panel with the car's position. Search and routing come after.

## 5. Better open-source options, including NVIDIA's per-pixel models

Ben, 15 Oct: "if there is better version of stuff that is open source then lets look at
that ... nvidia has some per pixel object detection models."

### Boxes vs per-pixel

| Kind | Answers | Example | Good for |
|---|---|---|---|
| **Detection** (boxes) | "a stop sign, here" | YOLO11, RT-DETR, RF-DETR | signs, people, cars: things to react to |
| **Semantic segmentation** (per pixel) | "every pixel: road, sidewalk, car, person..." | NVIDIA **CitySemSegFormer**, SegFormer, DeepLabV3+ | **where the road is**: the drivable area, for the costmap and lane keeping |
| **Instance segmentation** | per pixel, and which object | YOLO11-seg, Mask2Former | both at once |
| **Drivable area + lanes** | road and lane pixels | TwinLiteNet, YOLOPv2 | a cheap road mask for a small car |

Per-pixel is what our costmap is missing: Nav2 doesn't know where the road is (see
[Maps and position](maps-and-position.md)). A road mask, projected onto
the ground, becomes a costmap layer: off-road = high cost.

### NVIDIA's options

| Option | What | Fits us? |
|---|---|---|
| **CitySemSegFormer** (NGC, TAO) | per-pixel, 19 city classes (road, sidewalk, car, person, rider...). A ready ONNX file; the card says it's ready for commercial use | ✅ **the best first try.** Road mIoU 73.6, sidewalk 46.9 on its own test data: check it on ours |
| **PeopleSemSegNet** (NGC) | per-pixel people | a later add-on |
| **Isaac ROS** (NVIDIA's ROS packages: segmentation, RT-DETR detection, stereo depth, nvblox 3D maps) | open source (Apache 2.0), TensorRT-fast | ⚠️ **not on our Orin with Jazzy yet**: on Orin it's Isaac ROS 3.2 with JetPack 6 and **Humble**; the Jazzy version runs on Jetson Thor. JetPack 7.2 support is "planned", no date |
| **TAO Toolkit** | NVIDIA's fine-tuning tools | for later, if we train on our own pictures |

**So:** use NVIDIA's **models**, not Isaac ROS (yet). A model is just an ONNX file; we run
it ourselves.

### One way to run any model: ONNX Runtime

Most open models export to **ONNX**, one file format. ONNX Runtime (in conda-forge for both
the Mac and the car's ARM computers, version 1.31) runs it on whatever is fastest:

| Computer | ONNX Runtime uses | |
|---|---|---|
| Orin | TensorRT or CUDA (JetPack's build) | NVIDIA's speed, without Isaac ROS |
| Mac | Core ML | the Apple GPU |
| Any | the CPU | slow, but always works |

One node, `segmenter`, with the model file as a setting, could run CitySemSegFormer today
and TwinLiteNet tomorrow, and publish a road mask on `/perception/road/mask` (Proposed: not in `interfaces.yaml`). It could even
be Rust: the `ort` crate wraps ONNX Runtime.

### Licences

| Model | Licence |
|---|---|
| YOLO11 (Ultralytics) | AGPL-3.0: fine for our open project; a problem if anyone ships it closed |
| RT-DETR, RF-DETR | Apache 2.0 |
| CitySemSegFormer | NVIDIA's model licence (commercial use allowed, per the card): read it before redistributing |
| SegFormer (the original code) | NVIDIA's own licence, research use: prefer the TAO model |
| OpenCV | Apache 2.0 |

### The fair test

1. Record the car's cameras on one drive (`tools/record.sh`).
2. Run each model on the **same** recording: YOLO11 vs RT-DETR vs RF-DETR for boxes;
   CitySemSegFormer vs TwinLiteNet for the road.
3. Compare: what each misses, and milliseconds per frame on the Orin and the Mac.
4. Keep the winner; the losers cost nothing.

## 6. What to do, in order

| # | Task | Needs |
|---|---|---|
| 1 | Friday: what cameras, lidar and radar are really on the car, and what drives them | the audit (answered: audits of 16 and 21 Oct) |
| 2 | `camera_ros` instead of `usb_cam` on the camera Pis | step 1 (answered: MediaMTX → `aav_camera_stream` instead) |
| 3 | The lidar's points into Nav2 (step 1), first in Gazebo | nothing (answered: done; the roof VLP-16 through the stock `velodyne` driver, untested with the real lidar) |
| 4 | Record a real drive; run stock YOLO on it; decide whether to train | the cameras on the car |
| 5 | The HUD's map panel with offline tiles | the test area chosen (answered: built as the map window) |
| 6 | Lidar clusters; then camera + lidar calibration | the lidar on the car |
| 7 | A `segmenter` node on ONNX Runtime; try CitySemSegFormer for a road mask, then a road layer in the costmap | can start now, in Gazebo's road world |

## Sources (15 Oct)

- [Isaac ROS 4.0 for Thor](https://forums.developer.nvidia.com/t/nvidia-isaac-ros-4-0-for-thor-has-arrived/352109)
- [Roadmap for Isaac ROS Jazzy on Orin](https://forums.developer.nvidia.com/t/roadmap-for-isaac-ros-jazzy-on-orin-devices/375982)
- [isaac_ros_image_segmentation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation)
- [CitySemSegFormer on NGC](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/tao/models/citysemsegformer)
- [CitySemSegFormer in the TAO docs](https://docs.nvidia.com/tao/tao-toolkit/text/model_zoo/cv_models/citysegformer.html)
