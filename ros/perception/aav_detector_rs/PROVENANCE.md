# Provenance

Ported for `ecolo-av` on 2026-11-17 (TODO 243): a Rust rewrite of our own Python,
`aav_detector` (`ros/perception/aav_detector` at commit `5417d0f2`). Outside code is only
run, not copied: ONNX Runtime and `ort` (see `docs/acknowledgements.md`). The Python stays,
as the default.

| Part | From |
|---|---|
| `crates/detector-core/src/preprocess.rs` | `aav_detector/detr.py`: `preprocess`, ImageNet's mean and spread; the resize is OpenCV's `INTER_LINEAR`, written out |
| `crates/detector-core/src/decode.rs` | `aav_detector/detr.py`: `decode` (itself after rfdetr 1.11's PostProcess, Apache-2.0) |
| `crates/detector-core/src/coco.rs` | rfdetr 1.11's `COCO_CLASSES` (Apache-2.0); `aav_detector/boxes.py`: `normal_label`, `parse_labels` |
| `crates/detector-core/src/draw.rs` | `aav_detector/detector.py`: `draw`, without the text |
| `crates/detector-core/src/model.rs` | `aav_detector/detectors.py`: `RfdetrExported`; ONNX Runtime through `ort` 2.0.0-rc.15 |
| `crates/detector-node` | `aav_detector/detector.py`: the same name, parameters, topics and QoS |
| `crates/detector-core/tests/pictures/` | Ravensim's front camera at 5 to 18 m from the stop sign (17 Nov), and `aav_detector/test/stop_sign.jpg` as PNG |
| `crates/detector-core/tests/python_boxes.json` | `tools/onnx_check.py --write-fixtures`: the Python's boxes |

## Where it differs, on purpose

| | Python | Rust |
|---|---|---|
| models | `.pth`, `.mlpackage`, `.engine`; YOLO too | `.onnx`, RF-DETR only |
| pictures | raw or JPEG (`compressed:=true`) | raw only (rgb8, bgr8) |
| `detections/image` | boxes and their labels, as BGR | boxes only, in the picture's own encoding |
