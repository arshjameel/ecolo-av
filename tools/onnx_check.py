#!/usr/bin/env python3
"""
The ONNX detector's boxes against PyTorch's, and how fast each runtime is here.

usage (in the ml environment: PyTorch, rfdetr, ONNX Runtime, coremltools on a Mac):
    pixi run -e ml python3 tools/onnx_check.py                       # boxes: torch vs ONNX
    pixi run -e ml python3 tools/onnx_check.py --time 30             # and ms per picture
    pixi run -e ml python3 tools/onnx_check.py --write-fixtures      # the Rust tests' answers

The pictures: detector-core's test pictures (Ravensim's front camera, 640 x 360, at 5 to
18 m from the stop sign, and one photo), or any PNGs given with --pictures.

Backends (each picture is BGR from OpenCV, then each backend's own pre-processing):
  torch        RF-DETR's .pth through rfdetr (its predict(): resize, normalise, decode)
  ort-cpu      the .onnx through ONNX Runtime's CPU provider, with detr.py's steps
  ort-coreml   the same, ONNX Runtime's Core ML provider (a Mac's GPU and Neural Engine)
  coreml       the .mlpackage through coremltools (what the detector runs on a Mac today)

Same boxes means: each box over --threshold in one has a box of the same class in the
other with IoU >= 0.95 and a score within 0.02 (a box near the threshold may miss).
docs/onnx.md has the results. --write-fixtures saves ort-cpu's boxes (threshold 0.3) as
detector-core's tests/python_boxes.json: the Rust must find the same.
"""

import argparse
import json
from pathlib import Path
import statistics
import sys
import time

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros' / 'perception' / 'aav_detector'))
from aav_detector.detectors import coco_names, RfdetrTorch, rgb_of  # noqa: E402, I100
from aav_detector.detr import decode, input_size, preprocess, split_outputs  # noqa: E402

CORE = ROOT / 'ros' / 'perception' / 'aav_detector_rs' / 'crates' / 'detector-core'
PICTURES = CORE / 'tests' / 'pictures'
FIXTURES = CORE / 'tests' / 'python_boxes.json'
IOU_MIN, SCORE_GAP = 0.95, 0.02


def iou(a, b) -> float:
    """Return the intersection over union of two boxes, x1, y1, x2, y2."""
    w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = w * h
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def compare(ours, theirs, threshold):
    """
    Return (worst IoU, worst score gap, misses) of `ours` against `theirs`.

    Each is a list of (corners, score, label). A box is missed when no box of its label in
    the other list overlaps it by IOU_MIN; a miss within 0.05 of the threshold is allowed
    (its twin may be just under it), so it isn't counted.
    """
    worst_iou, worst_gap, misses = 1.0, 0.0, 0
    for a, b in ((ours, theirs), (theirs, ours)):
        for corners, score, label in a:
            twins = [(iou(corners, c), s) for c, s, n in b if n == label]
            best = max(twins, default=(0.0, 0.0))
            if best[0] < IOU_MIN:
                misses += score > threshold + 0.05
                continue
            worst_iou = min(worst_iou, best[0])
            worst_gap = max(worst_gap, abs(score - best[1]))
    return worst_iou, worst_gap, misses


class Exported:
    """An exported model (ONNX Runtime or Core ML) with detr.py's steps around it."""

    def __init__(self, name, run, height, width, names):
        self.name, self.run, self.height, self.width = name, run, height, width
        self.names = names

    def boxes(self, bgr, threshold):
        rgb = rgb_of(bgr)
        boxes, logits = split_outputs(self.run(preprocess(rgb, self.height, self.width)))
        corners, scores, classes = decode(boxes, logits, rgb.shape[1], rgb.shape[0],
                                          threshold=threshold)
        return [(c, float(s), self.names.get(int(k), str(int(k))))
                for c, s, k in zip(corners.tolist(), scores, classes)]


def onnx_backend(path, provider, names):
    import onnxruntime as ort
    providers = ([('CoreMLExecutionProvider', {'ModelFormat': 'MLProgram',
                                               'MLComputeUnits': 'ALL'})]
                 if provider == 'coreml' else []) + ['CPUExecutionProvider']
    session = ort.InferenceSession(str(path), providers=providers)
    first = session.get_inputs()[0]
    height, width = input_size(first.shape)
    outputs = [o.name for o in session.get_outputs()]

    def run(picture):
        return dict(zip(outputs, session.run(outputs, {first.name: picture})))
    return Exported(f'ort-{provider}', run, height, width, names)


def coreml_backend(path, names):
    import coremltools as ct
    model = ct.models.MLModel(str(path), compute_units=ct.ComputeUnit.ALL)
    first = model.get_spec().description.input[0]
    height, width = input_size(first.type.multiArrayType.shape)
    return Exported('coreml', lambda p: model.predict({first.name: p}), height, width, names)


class Torch:
    """RF-DETR's own PyTorch path, as the detector runs it."""

    def __init__(self, pth, device):
        self.name = f'torch-{device}'
        self.model = RfdetrTorch(str(pth), device, log=lambda s: None)

    def boxes(self, bgr, threshold):
        found = self.model.rfdetr.predict(rgb_of(bgr), threshold=threshold)
        return [(c, float(s), self.model.names.get(int(k), str(int(k))))
                for c, s, k in zip(found.xyxy.tolist(), found.confidence, found.class_id)]


def timing(backend, pictures, runs):
    """Return the median ms per picture, after 5 warm-up runs, over `runs` runs each."""
    for bgr in pictures[:1] * 5:
        backend.boxes(bgr, 0.5)
    times = []
    for _ in range(runs):
        for bgr in pictures:
            start = time.perf_counter()
            backend.boxes(bgr, 0.5)
            times.append(1000.0 * (time.perf_counter() - start))
    return statistics.median(times)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('--pth', default=str(ROOT / 'models' / 'rf-detr-nano.pth'))
    p.add_argument('--onnx', default=str(ROOT / 'models' / 'rf-detr-nano.onnx'))
    p.add_argument('--mlpackage', default=str(ROOT / 'models' / 'rf-detr-nano.mlpackage'))
    p.add_argument('--pictures', nargs='*', default=sorted(str(f) for f in PICTURES.glob('*.png')))
    p.add_argument('--threshold', type=float, default=0.5)
    p.add_argument('--device', default='mps', help="PyTorch's device: mps, cpu, cuda")
    p.add_argument('--time', type=int, default=0, metavar='RUNS',
                   help='also time each backend: RUNS passes over the pictures')
    p.add_argument('--write-fixtures', action='store_true',
                   help="save ort-cpu's boxes (threshold 0.3) for the Rust tests")
    args = p.parse_args(argv)
    for need in (args.pth, args.onnx):
        if not Path(need).exists():
            print(f'{need} is missing: pixi run get models, then tools/export_model.py '
                  '--format onnx', file=sys.stderr)
            return 2
    names = coco_names()
    pictures = {Path(f).name: cv2.imread(f) for f in args.pictures}
    backends = [Torch(args.pth, args.device), onnx_backend(args.onnx, 'cpu', names)]
    if sys.platform == 'darwin':
        backends.append(onnx_backend(args.onnx, 'coreml', names))
        if Path(args.mlpackage).exists():
            backends.append(coreml_backend(args.mlpackage, names))

    reference = backends[0]
    print(f'| picture | {reference.name}: boxes over {args.threshold} | '
          + ' | '.join(f'{b.name}: worst IoU, score gap, misses' for b in backends[1:]) + ' |')
    print('|---|---|' + '---|' * (len(backends) - 1))
    failed = False
    for name, bgr in pictures.items():
        theirs = reference.boxes(bgr, args.threshold)
        cells = []
        for b in backends[1:]:
            worst_iou, gap, misses = compare(b.boxes(bgr, args.threshold), theirs,
                                             args.threshold)
            failed |= misses > 0 or worst_iou < IOU_MIN or gap > SCORE_GAP
            cells.append(f'{worst_iou:.4f}, {gap:.4f}, {misses}')
        labels = ', '.join(f'{n} {s:.2f}' for _, s, n in theirs) or 'none'
        print(f'| {name} | {labels} | ' + ' | '.join(cells) + ' |')
    print('SAME BOXES' if not failed else 'DIFFERENT BOXES (see above)')

    if args.write_fixtures:
        onnx_cpu = backends[1]
        fixtures = {}
        for name, bgr in pictures.items():
            fixtures[name] = [{'corners': [round(v, 3) for v in c], 'score': round(s, 5),
                               'label': n} for c, s, n in onnx_cpu.boxes(bgr, 0.3)]
        FIXTURES.write_text(json.dumps(fixtures, indent=1) + '\n')
        print(f'{FIXTURES.relative_to(ROOT)}: {sum(map(len, fixtures.values()))} boxes')

    if args.time:
        sim = [bgr for bgr in pictures.values() if bgr.shape[:2] == (360, 640)] \
            or list(pictures.values())
        print(f'\n| backend | ms per picture ({sim[0].shape[1]} x {sim[0].shape[0]}), median |')
        print('|---|---|')
        for b in backends:
            print(f'| {b.name} | {timing(b, sim, args.time):.1f} |')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
