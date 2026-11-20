#!/usr/bin/env python3
"""
Our small model, trained after each drive from the reviewed labels (TODO 252, phase F).

    pixi run label-train bags/<drive>/map [more maps...]   models/label-seg.{onnx,json}
        [--epochs 30] [--held 0.2] [--device mps] [--out models/label-seg]

A point segmenter: each lidar point's class (background, tree, building, parked_car, pole,
other) from its own height and range and its pillar's shape (label_core.point_features: the
same numbers from one scan on the car). A small MLP, 10 -> 64 -> 64 -> 6, on the GPU (mps);
modest on purpose: it's the loop that matters, drive -> labels -> a person's fixes -> a model.

1. For each map folder: its labels.yaml (after a person fixed it in map-edit: whatever is
   in the file is the truth here) and its drive's frames (<drive>/labels/frames.npz, from
   label-drive; an edited map, <drive>/map-edit-1, finds the same drive).
2. Each frame's points labelled by the boxes they fall in (label_core.point_classes).
3. The last --held share of each drive's frames held out (later in the drive: other places).
4. Trained; scored on the held-out frames against the labels, and, on a Ravensim drive,
   against Ravensim's own objects too (label_drive.truth).
5. Exported to ONNX (input `features` N x 10, output `logits` N x 6; the normalisation is in
   the graph); ONNX Runtime on the CPU and Core ML checked against PyTorch, as `measure
   onnx` does: the same classes, the largest difference, ms a scan.

Writes --out .pt, .onnx and .json (features, classes, the scores). No ROS. The maths:
tools/label_core.py (tools/tests/test_label_core.py).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import label_core as lc  # noqa: E402, I100


def drive_of(map_dir: Path) -> Path:
    """Return the drive folder a map folder came from (<drive>/map, <drive>/map-edit-1)."""
    for folder in (map_dir.parent, map_dir):
        if (folder / 'labels' / 'frames.npz').exists():
            return folder
    raise SystemExit(f'{map_dir}: no <drive>/labels/frames.npz beside it (run label-drive)')


def frames_of(drive: Path):
    """Yield (pose, points base_link) for each frame label-drive kept."""
    data = np.load(drive / 'labels' / 'frames.npz')
    ends = np.cumsum(data['sizes'])
    for pose, start, end in zip(data['poses'], ends - data['sizes'], ends):
        yield tuple(pose), data['points'][start:end].astype(float)


def truth_classes(points_map, things):
    """Return each map point's class from Ravensim's objects (inside, and below the top)."""
    out = np.zeros(len(points_map), np.int64)
    for t in things:
        lo, hi = t.polygon.min(0), t.polygon.max(0)
        near = np.flatnonzero(((points_map[:, :2] >= lo) & (points_map[:, :2] <= hi)).all(1))
        if len(near):
            hit = near[lc.inside_polygon(points_map[near, :2], t.polygon)]
            hit = hit[points_map[hit, 2] <= t.height + 0.3]
            out[hit] = lc.SEG_CLASSES.index(t.cls)
    out[points_map[:, 2] < lc.GROUND] = 0
    return out


def dataset(maps, held):
    """Return the train and held-out sets: features, label classes, truth classes or -1."""
    import label_drive as ld
    parts = {'train': [], 'held': []}
    for map_dir in maps:
        drive = drive_of(map_dir)
        labels = lc.read_labels(map_dir / 'labels.yaml')
        info = yaml.safe_load((drive / 'drive.yaml').read_text()) if (
            drive / 'drive.yaml').exists() else {}
        things = ld.truth(info.get('world')) if info.get('world') else []
        frames = list(frames_of(drive))
        if things:                      # only what's near the drive: the campus has 8,500
            xy = np.array([pose[:2] for pose, _ in frames])
            lo, hi = xy.min(0) - lc.MAX_RANGE, xy.max(0) + lc.MAX_RANGE
            things = [t for t in things if (t.polygon.max(0) >= lo).all()
                      and (t.polygon.min(0) <= hi).all()]
        cut = int(round(len(frames) * (1.0 - held)))
        for k, (pose, points) in enumerate(frames):
            points = points[np.hypot(points[:, 0], points[:, 1]) < lc.MAX_RANGE]
            in_map = lc.to_map(points, pose)
            truth = truth_classes(in_map, things) if things else np.full(len(points), -1)
            parts['train' if k < cut else 'held'].append(
                (lc.point_features(points), lc.point_classes(in_map, labels), truth))
        print(f'{map_dir}: {len(labels)} labels, {len(frames)} frames ({len(frames) - cut} '
              f'held out), truth: {len(things)} objects' if things else
              f'{map_dir}: {len(labels)} labels, {len(frames)} frames, no truth')
    return {name: [np.concatenate(c) for c in zip(*rows)] for name, rows in parts.items()}


def make_model(mean, std):
    """Return the MLP, with the features' normalisation inside it."""
    import torch
    from torch import nn

    class Segmenter(nn.Module):
        """10 features -> 6 classes' logits."""

        def __init__(self):
            super().__init__()
            self.register_buffer('mean', torch.tensor(mean, dtype=torch.float32))
            self.register_buffer('std', torch.tensor(std, dtype=torch.float32))
            self.net = nn.Sequential(
                nn.Linear(len(lc.FEATURES), 64), nn.ReLU(),
                nn.Linear(64, 64), nn.ReLU(),
                nn.Linear(64, len(lc.SEG_CLASSES)))

        def forward(self, features):
            return self.net((features - self.mean) / self.std)

    return Segmenter()


def train(model, x, y, device, epochs, batch=8192):
    """Train with class weights (background is most points); return the loss each epoch."""
    import torch
    model = model.to(device)
    counts = np.bincount(y, minlength=len(lc.SEG_CLASSES)).astype(float)
    weight = np.where(counts > 0, 1.0 / np.sqrt(np.maximum(counts, 1.0)), 0.0)
    weight = weight / weight[counts > 0].mean()
    loss_fn = torch.nn.CrossEntropyLoss(weight=torch.tensor(weight, dtype=torch.float32,
                                                            device=device))
    optimiser = torch.optim.Adam(model.parameters(), lr=2e-3)
    xs = torch.tensor(x, device=device)
    ys = torch.tensor(y, device=device)
    losses = []
    for _ in range(epochs):
        order = torch.randperm(len(xs), device=device)
        total = 0.0
        for start in range(0, len(xs), batch):
            b = order[start:start + batch]
            optimiser.zero_grad()
            loss = loss_fn(model(xs[b]), ys[b])
            loss.backward()
            optimiser.step()
            total += float(loss) * len(b)
        losses.append(total / len(xs))
    return model.eval(), losses


def predict(model, x, device):
    """Return the model's class for each row of x."""
    import torch
    with torch.no_grad():
        return model(torch.tensor(x, device=device)).argmax(1).cpu().numpy()


def check_onnx(model, path, x):
    """Return ONNX Runtime's answers against PyTorch's on the CPU and Core ML (one scan)."""
    import onnxruntime as ort
    import torch
    with torch.no_grad():
        ref = model.cpu()(torch.tensor(x)).numpy()
    out = {}
    providers = [('cpu', ['CPUExecutionProvider'])]
    if 'CoreMLExecutionProvider' in ort.get_available_providers():
        providers.append(('coreml', ['CoreMLExecutionProvider', 'CPUExecutionProvider']))
    for name, chain in providers:
        session = ort.InferenceSession(str(path), providers=chain)
        got = session.run(None, {'features': x})[0]
        t0 = time.time()
        for _ in range(20):
            session.run(None, {'features': x})
        out[name] = {'same_class': round(float((got.argmax(1) == ref.argmax(1)).mean()), 4),
                     'max_diff': float(np.abs(got - ref).max()),
                     'ms_a_scan': round((time.time() - t0) / 20 * 1000, 2),
                     'points': len(x)}
    return out


def main():
    """Train, score, export."""
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('maps', type=Path, nargs='+', help='map folders with labels.yaml')
    p.add_argument('--epochs', type=int, default=30)
    p.add_argument('--held', type=float, default=0.2, help="each drive's frames held out")
    p.add_argument('--device', default='mps')
    p.add_argument('--out', type=Path, default=ROOT / 'models' / 'label-seg')
    args = p.parse_args()
    import torch
    torch.manual_seed(1)
    t0 = time.time()
    data = dataset(args.maps, args.held)
    x, y, _ = data['train']
    mean, std = x.mean(0), x.std(0) + 1e-6
    model, losses = train(make_model(mean, std), x, y, args.device, args.epochs)
    hx, hy, ht = data['held']
    guess = predict(model, hx, args.device)
    scores = {'against_labels': lc.segment_scores(hy, guess)}
    if (ht >= 0).all() and len(ht):
        scores['against_truth'] = lc.segment_scores(ht, guess)
        scores['labels_against_truth'] = lc.segment_scores(ht, hy)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    torch.save(model.state_dict(), args.out.with_suffix('.pt'))
    onnx_path = args.out.with_suffix('.onnx')
    model = model.cpu()
    torch.onnx.export(model, (torch.zeros(1, len(lc.FEATURES)),), str(onnx_path),
                      input_names=['features'], output_names=['logits'],
                      dynamic_axes={'features': {0: 'points'}, 'logits': {0: 'points'}},
                      opset_version=17, dynamo=False)
    scan = np.ascontiguousarray(hx[:min(len(hx), 17000)])
    onnx = check_onnx(model, onnx_path, scan)
    meta = {'features': list(lc.FEATURES), 'classes': list(lc.SEG_CLASSES),
            'maps': [str(m) for m in args.maps], 'train_points': int(len(x)),
            'held_points': int(len(hx)), 'epochs': args.epochs,
            'loss': [round(v, 4) for v in losses[::5] + losses[-1:]],
            'scores': scores, 'onnx': onnx, 'seconds': round(time.time() - t0, 1)}
    args.out.with_suffix('.json').write_text(json.dumps(meta, indent=1))
    print(json.dumps({'held_points': meta['held_points'], 'scores': scores, 'onnx': onnx},
                     indent=1))
    print(f'wrote {onnx_path} and its .json, .pt in {meta["seconds"]} s')


if __name__ == '__main__':
    main()
