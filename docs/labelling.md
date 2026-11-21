# Labelling after a drive

After each drive, the computer labels **what is around**: trees, buildings, poles, signs,
fences, parked cars. A person fixes the labels in the map editor. Our small model learns
from them. Then a **twin** of the place can be built for Ravensim.

Phases F and G of the [testing plan](testing-plan.md) (TODO 252, 253).

!!! warning "Ravensim only so far"
    Built and run on Ravensim's campus, 19 Nov. Never run on a real drive's pictures.
    The camera-to-lidar calibration on the car (TODO 156) comes first.

## The loop

```text
 a drive ──▶ label-drive ──▶ labels.yaml ──▶ map-edit ──▶ label-train ──▶ our model (ONNX)
 (pictures,   OWLv2 boxes      source:        a person      a point          scored on
  lidar,      onto the lidar   camera         fixes them    segmenter        held-out frames
  poses)      points                              │
                                                  └──────▶ twin-world ──▶ a Ravensim world
```

## The commands

| Command | Does |
|---|---|
| `pixi run label-drive bags/<drive>` | labels into `bags/<drive>/map/labels.yaml`; a report in `bags/<drive>/labels/` |
| `pixi run label-drive --sim-drive bags/<name> --lot P4` | a drive from Ravensim alone, round a campus lot, then its labels |
| `pixi run map-edit bags/<drive>/map` | a person keeps, fixes or deletes them |
| `pixi run label-train <map> [<map>...]` | our small model: `models/label-seg.onnx` and its scores |
| `pixi run twin-world <map>` | a Ravensim world in `maps/twins/<name>/` |
| `pixi run sim world:=maps/twins/<name>/twin.sdf` | drive in it |

`label-drive` and `label-train` run in the `ml` environment (PyTorch on the Mac's GPU).

## Step 1: the detector

**OWLv2** (Google): it finds things named in words, so it knows trees and buildings, which
our car detector (RF-DETR, 80 everyday things) doesn't.

It is asked for: tree, building, car, pole, street light, traffic sign, stop sign,
traffic light, fence, bench, person.

| Word | Label class |
|---|---|
| tree | `tree` |
| building | `building` |
| car | `parked_car` |
| pole, street light, traffic sign, stop sign, traffic light | `pole` |
| fence, bench | `other` |
| person | not mapped: people move |

### Why OWLv2: the licences

We checked the **weights** and their training data, not just the code.

| Model | Code | Weights | Verdict |
|---|---|---|---|
| **OWLv2** (`google/owlv2-base-patch16-ensemble`) | Apache-2.0 | Apache-2.0 | **picked**: allowed, runs on `mps` through Hugging Face transformers (146 ms a picture) |
| Grounding DINO | Apache-2.0 | Apache-2.0 | allowed; a challenger later. Its deformable attention has no Metal kernel |
| YOLO-World | GPL-3.0 | GPL-3.0 | **not used**: copyleft, and our licence isn't settled |
| SAM 2 (masks) | Apache-2.0 | Apache-2.0 | allowed; not needed yet: the box and the lidar's depth do the cutting |

OWLv2 learned from web pictures Google labelled itself (WebLI), then from Objects365 and
Visual Genome. None of the three puts a non-commercial term on the weights. Recheck the
two datasets' terms before any commercial use.

## Step 2: boxes onto lidar points

```text
   picture                                  lidar points in the box
 ┌───────────────┐                          depth:  8 m  ●●●●      ← the tree: kept
 │   ┌──tree──┐  │   project every point          20 m  ●●●●●●●●  ← the building behind
 │   │   ●●   │  │   into the picture
 │   │  ●●●   │  │   (vehicle.yaml's mounts,   the nearest group by depth takes the label
 │   └────────┘  │    camera_info's lens)       (a break of 1.5 m, or 10 %, ends it)
 └───────────────┘
```

- Only points 0.5 m and more above the road (not kerbs or grass) and within 45 m.
- Each frame's labelled points go into **0.5 m map cells**, one vote per frame.
- A cell counts when 2 frames or more agree, with half its votes or more.
- Touching cells of one class become **one label**: the smallest turned box round them, a
  height band, a confidence.
- A "building" under 2.5 m is a fence (`other`). A "tree" under 2 m is dropped (a bush).

## Step 3: labels.yaml

The map editor's format ([The map editor](map-editor.md)):

```yaml
- id: 7
  class: tree
  region: {centre_m: [41.25, -24.5], size_m: [2.5, 2.0], yaw_rad: 0.0}
  band: {min_m: 5.01, max_m: 6.51}
  source: camera
  confidence: 0.2
  seen_in: [sim-lot-p4]
```

| Already in the file | What a new camera label does |
|---|---|
| a person's label, same class, same place | nothing added; the person's gains the drive in `seen_in` |
| an earlier drive's camera label | joined: both drives in `seen_in`, the surer confidence |
| occupancy's box (`source: passes`, class `other`) | names it: it takes the camera's class |
| this same drive's labels (run again) | taken back first: running twice changes nothing |

## Step 4: our small model

A **point segmenter**: each lidar point's class from its height, its range and its
pillar's shape. A small network (10 numbers in, 64, 64, 6 classes out), trained on the
Mac's GPU in about 90 s.

- Trained on the labels as a person left them.
- The last 20 % of each drive's frames are held out: other places.
- Exported to ONNX, and checked on the CPU and Core ML, as `measure onnx` checks.

It's modest on purpose: the loop matters, not the model.

## The proof (Ravensim, 19 Nov)

Two drives made from Ravensim alone: the car put every 2 m round a campus lot, 6 m in
from its edge; three cameras drawn and the lidar cast at each pose. Scored against
Ravensim's own trees, buildings and fences (`campus_world.objects`).

### Labels against Ravensim's truth

| Drive | Pictures | Labels | Right (precision) | Things seen | Found (recall) |
|---|---|---|---|---|---|
| lot P4 (104 poses) | 312 | 11 | 8 (**73 %**) | 12 | 6 (**50 %**) |
| lot P5 (276 poses) | 828 | 22 | 16 (**73 %**) | 13 | 10 (**77 %**) |

Lot P5, by class:

| Class | Labels | Precision | Things | Recall |
|---|---|---|---|---|
| tree | 5 | 80 % | 7 | 57 % |
| building | 8 | 75 % | 5 | 100 % |
| pole | 3 | 0 % | 0 | - (the campus world has no poles: all three are wrong) |
| other (fences) | 6 | 100 % | 1 | 100 % |

"Things seen": those with 20 lidar points or more in the pictures. The lidar sees ±15°,
so most of what OWLv2 boxes is too high or too far: 549 of 6,382 boxes landed on points.

### Our model, on the 76 held-out frames (1.05 million points)

| Class | IoU against the labels | Against the truth: precision / recall | The labels themselves |
|---|---|---|---|
| building | 0.87 | 0.41 / 1.00 | 0.47 / 1.00 |
| tree | 0.40 | 0.97 / 0.41 | 0.97 / 0.28 |
| other | 0.45 | 0.11 / 0.53 | 0.16 / 0.41 |
| pole | 0.34 | no poles in the world | |
| every point right | 97.7 % | 94.2 % | 94.9 % |

- It finds more of the trees than its own labels do (41 % against 28 %).
- Buildings: a label's box is a rectangle, so it takes in what stands next to the wall.
- ONNX: the same class as PyTorch for every point, on the CPU and Core ML (largest
  difference 5e-5); **0.6 ms** for a 17,000-point scan.

### The twin

`twin-world` on lot P5's labels: 8 buildings, 5 trees, 3 poles, 6 blocks (334 triangles).
`sim world:=…/twin.sdf` started: Ravensim read the world, the lidar ran at 10 Hz, the map
server loaded the twin's 2D map, the car stood at the drive's first pose.

## The twin (TODO 253)

| Label | Becomes in Ravensim |
|---|---|
| building | its box, brick or stone, as tall as its NDT points, a roof |
| tree | a trunk and a low-poly crown, as tall and as wide as its points |
| pole | a 15 cm post |
| other | a block, as its band says |
| parked car | **left out**: it may be gone (`--keep-parked` keeps them) |
| seen in under half the drives | **left out**: it moved |

It reuses `campus_world.py`'s meshes, textures and world file. Beside `twin.sdf` it keeps
the map's `map2d.yaml` and `ndt.pcd` (the sim's localisation reads them) and `twin.yaml`
(the start, what went in, what was left out and why).

## Not done yet

| What | Why |
|---|---|
| A real drive | no recording with pictures yet; the car's camera-to-lidar calibration (TODO 156) |
| A recorded Ravensim drive's pictures | `--draw-sim WORLD` draws them at the truth poses; built, not run |
| Kerbs, people | kerbs are under the 0.5 m floor; people move and aren't mapped |
| The twin's pass test | a replayed drive within NDT's error of the real one: stage 2 |
| Poles | the campus world has none, so pole labels can't be scored there |

## Code

| Where | What |
|---|---|
| `tools/label_core.py` | the maths, no ROS, no PyTorch (`tools/tests/test_label_core.py`) |
| `tools/label_drive.py` | the drive, OWLv2, the fusion, the report |
| `tools/label_train.py` | our segmenter: train, score, ONNX |
| `tools/twin_world.py` | the twin (`tools/tests/test_twin_world.py`) |
| `aav_simulator`'s `scan_poses` example | Ravensim's lidar at a list of poses, no ROS |
