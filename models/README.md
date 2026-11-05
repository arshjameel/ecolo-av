# Models

Trained model files for the perception nodes. They stay out of git: only this file is
tracked (see `.gitignore`).

| File | What it is | How to get it |
|---|---|---|
| `rf-detr-nano.pth` | the stock RF-DETR Nano model (Roboflow, Apache-2.0): **the car's detector** (Ben, 23 Oct; `detector:=rfdetr`, the default). It knows the same 80 everyday things | `pixi run get models` (366 MB) |
| `yolo11n.pt` | the stock YOLO11 nano model (`detector:=yolo`). It knows 80 everyday things, "stop sign" among them | `pixi run get models` |
| `best.pt` | the 2024-25 team's own stop-sign model: its trained weights, one label (`Stop_Sign`). Training saves two files, `best.pt` (the round that scored best) and `last.pt`. Their code loaded it from `~/Yolo/Capstone/runs/detect/train7/weights/best.pt`, on one student's computer | ask the team. Then `pixi run detect detector:=yolo model:=models/best.pt labels:="stop sign"` |
| `diffusion/` | the diffusion planner trial's (`planner:=diffusion`, `aav_diffusion_planner`): `data/` (the teachers' paths, one `.npz` a map, and `data/maps/`), `dataset.npz` (the crops), `planner.pt` (the weights), `planner.onnx` (the sampler the node runs, 16 MB), `metrics.json` (training time and the guard's score) | `pixi run diffusion data`, then `pixi run diffusion train` (Ben's Mac, 2 Nov: about 30 min for 45,000 paths, then 23 min to train) |

One model, three formats. Make the other two from the `.pth` (RF-DETR) or `.pt` (YOLO):

| Format | Runs on | Make it with |
|---|---|---|
| `.pth`, `.pt` | anything, with PyTorch | `pixi run get models`, or the training |
| `.mlpackage` | Macs, with Core ML | `pixi run -e ml python3 tools/export_model.py --format coreml [models/<name>.pth or .pt]`, on a Mac: `rf-detr-nano.mlpackage` by default |
| `.engine` | the Orin, with TensorRT | `python3 tools/build_engine.py [models/<name>.pth or .pt]`, on the Orin itself, with JetPack's Python: `rf-detr-nano.engine` by default |
| `.onnx` | only a step to the `.engine` | `tools/build_engine.py` makes it; or `python3 tools/export_model.py --format onnx` with `pip install onnx` |

An exported RF-DETR keeps `rf-detr` in its name: that's how the detector knows it isn't YOLO's.

The Ultralytics models and code are licensed AGPL-3.0; RF-DETR's Nano to Large, Apache-2.0.
