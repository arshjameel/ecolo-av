#!/usr/bin/env python3
"""
Train the learned lane finder (TODO 262b), score it on held-out pictures, export ONNX.

    pixi run -e ml python3 tools/lane_data.py                  # the pictures, first
    pixi run -e ml python3 tools/lane_train.py [--epochs 40]   # on the Mac's GPU (mps)
    pixi run -e ml python3 tools/lane_train.py --score-only    # score models/lane-ufld.pt

The model is our own small take on Ultra-Fast-Lane-Detection v2 (Z. Qin, P. Zhang, X. Li,
"Ultra Fast Deep Lane Detection With Hybrid Anchor Driven Ordinal Classification", IEEE
TPAMI 2022; their code is MIT): written from the paper's idea, not copied.

  picture (3 x 144 x 480, the rows below the horizon)
    -> ResNet-18 (torchvision's, random start: no ImageNet weights)
    -> 1 x 1 conv to 8 channels -> flatten -> 2048 -> the answers:
  loc     CELLS x R x 4: for each line and each row anchor, which column (classification)
  exist   2 x R x 4: is the line in that row
  colour  2 x 4: white or yellow paint

Losses: cross-entropy on the column (rows with the line only), the column's expectation
against the true one (UFLD v2's mean loss), cross-entropy on exist and on the colour.
Augmented: colour, brightness, blur, noise, and a mirror (left and right lines swap).

The scores, on the held-out pictures (the campus's every seventh road, never trained on,
and 15 % of the road world): each line's points put on the ground (lane_data.ground), a
fit y = a x² + b x + c between 4 and 25 m, and its y against the lane map's at 5, 10 and
15 m ahead; the detection rate (a line in the map, found) and false finds.

Writes models/lane-ufld.pt (PyTorch), models/lane-ufld.onnx and models/lane-ufld.json (what
the node needs: the sizes, the rows, the lens it was made for; and the scores), and the
half-precision twin models/lane-ufld_fp16.onnx (with its .json).
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
import torchvision

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import lane_data as L  # noqa: E402, I100

MODELS = ROOT / 'models'
FIT_X = (4.0, 25.0)        # m ahead: the fit's span for the scores (and the node's default)
MIRROR = [3, 2, 1, 0]      # the lines' slots, mirrored


class LaneNet(nn.Module):
    """UFLD v2's row-anchor head on a ResNet-18. Input: RGB 0..1, N x 3 x H x W."""

    def __init__(self, rows: int, cells: int, lanes: int = 4, input_hw=(L.INPUT_H, L.INPUT_W),
                 mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)):
        super().__init__()
        self.rows, self.cells, self.lanes = rows, cells, lanes
        r = torchvision.models.resnet18(weights=None)
        self.backbone = nn.Sequential(r.conv1, r.bn1, r.relu, r.maxpool,
                                      r.layer1, r.layer2, r.layer3, r.layer4)
        h, w = math.ceil(input_hw[0] / 32), math.ceil(input_hw[1] / 32)
        self.pool = nn.Conv2d(512, 8, 1)
        self.flat = 8 * h * w
        self.n_loc = cells * rows * lanes
        self.n_exist = 2 * rows * lanes
        self.head = nn.Sequential(nn.Linear(self.flat, 2048), nn.ReLU(),
                                  nn.Linear(2048, self.n_loc + self.n_exist + 2 * lanes))
        self.register_buffer('mean', torch.tensor(mean).view(1, 3, 1, 1))
        self.register_buffer('std', torch.tensor(std).view(1, 3, 1, 1))

    def forward(self, x):
        x = (x - self.mean) / self.std
        # A fixed Reshape, not Flatten: Core ML takes it (Flatten fell back to the CPU).
        f = self.pool(self.backbone(x)).reshape(-1, self.flat)
        out = self.head(f)
        n = x.shape[0]
        loc = out[:, :self.n_loc].reshape(n, self.cells, self.rows, self.lanes)
        exist = out[:, self.n_loc:self.n_loc + self.n_exist].reshape(n, 2, self.rows, self.lanes)
        colour = out[:, self.n_loc + self.n_exist:].reshape(n, 2, self.lanes)
        return loc, exist, colour


def targets(cols: np.ndarray, cells: int):
    """Labels in px (N, 4, R, NaN = none) to (cell index, cell float, exist), each N x R x 4."""
    exist = ~np.isnan(cols)
    cell_f = np.where(exist, cols / L.WIDTH * cells - 0.5, 0.0)
    cell_i = np.clip(np.floor(cell_f + 0.5), 0, cells - 1).astype(np.int64)
    return (torch.from_numpy(cell_i.transpose(0, 2, 1).copy()),
            torch.from_numpy(cell_f.transpose(0, 2, 1).astype(np.float32).copy()),
            torch.from_numpy(exist.transpose(0, 2, 1).copy()))


def losses(loc, exist, colour, cell_i, cell_f, has, paint):
    """Return the four losses: UFLD v2's three, and the colour."""
    ce = F.cross_entropy(loc, cell_i, reduction='none')
    w = has.float()
    loc_loss = (ce * w).sum() / w.sum().clamp(min=1)
    p = F.softmax(loc, dim=1)
    idx = torch.arange(loc.shape[1], device=loc.device, dtype=p.dtype).view(1, -1, 1, 1)
    expect = (p * idx).sum(1)
    mean_loss = (F.smooth_l1_loss(expect, cell_f, reduction='none') * w).sum() / w.sum().clamp(
        min=1)
    exist_loss = F.cross_entropy(exist, has.long())
    seen = has.any(1)
    colour_loss = (F.cross_entropy(colour, paint, reduction='none') * seen.float()).sum() / \
        seen.float().sum().clamp(min=1)
    return loc_loss, mean_loss, exist_loss, colour_loss


def augment(x: torch.Tensor, gen: torch.Generator):
    """Colour, brightness, contrast, blur and noise, per picture. x: N x 3 x H x W, 0..1."""
    n = x.shape[0]
    dev = x.device

    def u(lo, hi, *shape):
        return (torch.rand(n, *shape, generator=gen) * (hi - lo) + lo).to(dev)

    grey = x.mean(1, keepdim=True)
    x = grey + (x - grey) * u(0.5, 1.5, 1, 1, 1)                       # saturation
    x = x * u(0.85, 1.15, 3, 1, 1)                                     # colour cast
    m = x.mean((1, 2, 3), keepdim=True)
    x = m + (x - m) * u(0.6, 1.4, 1, 1, 1)                             # contrast
    x = x * u(0.5, 1.4, 1, 1, 1)                                       # brightness
    if torch.rand(1, generator=gen).item() < 0.3:
        x = F.avg_pool2d(x, 3, stride=1, padding=1, count_include_pad=False)
    x = x + torch.randn(x.shape, generator=gen).to(dev) * u(0.0, 0.04, 1, 1, 1)
    return x.clamp(0, 1)


def score(model, images, cols, colour, gt, meta, device, batch=64):
    """Held-out scores: lateral error (m) at GT_X per line, detection and false finds."""
    cam = L.Camera(**{k: (tuple(v) if isinstance(v, list) else v)
                      for k, v in meta['camera'].items()})
    rows = np.asarray(meta['rows'], dtype=float)
    model.eval()
    errors = {k: [[] for _ in L.GT_X] for k in range(4)}
    found = np.zeros(4)
    truth = np.zeros(4)
    false = np.zeros(4)
    absent = np.zeros(4)
    colour_ok = [0, 0]
    with torch.no_grad():
        for s in range(0, len(images), batch):
            x = torch.from_numpy(images[s:s + batch]).to(device).permute(0, 3, 1, 2).float() / 255
            loc, ex, col = (t.cpu().numpy() for t in model(x))
            for j in range(len(x)):
                i = s + j
                px = L.decode(loc[j], ex[j])
                for k in range(4):
                    there = not np.isnan(gt[i, k, 1])
                    pts = L.ground(np.column_stack([px[k], rows]), cam)
                    f = L.fit(pts, *FIT_X)
                    truth[k] += there
                    absent[k] += not there
                    if f is None:
                        continue
                    if not there:
                        false[k] += 1
                        continue
                    found[k] += 1
                    for g, x_ahead in enumerate(L.GT_X):
                        if not np.isnan(gt[i, k, g]):
                            errors[k][g].append(abs(np.polyval(f, x_ahead) - gt[i, k, g]))
                    if k in (1, 2):
                        colour_ok[0] += int(np.argmax(col[j, :, k]) == colour[i, k])
                        colour_ok[1] += 1
    out = {'pictures': int(len(images)), 'lines': {}}
    for k, name in enumerate(L.LANES):
        out['lines'][name] = {
            'detected': round(found[k] / max(truth[k], 1), 3),
            'false_finds': round(false[k] / max(absent[k], 1), 3),
            'in_map': int(truth[k]),
            **{f'error_{int(x)}m': {'median': round(float(np.median(e)), 3) if e else None,
                                    'p90': round(float(np.percentile(e, 90)), 3) if e else None}
               for x, e in zip(L.GT_X, errors[k])}}
    out['colour_right'] = round(colour_ok[0] / max(colour_ok[1], 1), 3)
    return out


def summary(s):
    """Return the scores in a few lines."""
    lines = [f"{s['pictures']} held-out pictures; colour right {s['colour_right'] * 100:.0f} %"]
    for name, v in s['lines'].items():
        errs = '  '.join(f"{x:.0f} m {v[f'error_{int(x)}m']['median']} / "
                         f"{v[f'error_{int(x)}m']['p90']}" for x in L.GT_X)
        lines.append(f"{name:6} found {v['detected'] * 100:5.1f} % of {v['in_map']}, false "
                     f"{v['false_finds'] * 100:4.1f} %; error median / p90 (m): {errs}")
    return '\n'.join(lines)


def export(model, path: Path, meta: dict):
    """Write the model as ONNX (opset 17, one picture, fixed size), its JSON beside it."""
    model = model.cpu().eval()
    dummy = torch.rand(1, 3, meta['input_h'], meta['input_w'])
    torch.onnx.export(model, (dummy,), str(path), input_names=['image'],
                      output_names=['loc', 'exist', 'colour'], opset_version=17, dynamo=False)
    path.with_suffix('.json').write_text(json.dumps(meta, indent=1))
    # Its half-precision twin, for the Neural Engine and TensorRT (measure onnx checks it).
    from export_model import half_precision
    half = half_precision(path)
    half.with_suffix('.json').write_text(json.dumps(meta, indent=1))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('--data', type=Path, default=MODELS / 'lane-data')
    p.add_argument('--out', type=Path, default=MODELS / 'lane-ufld.pt')
    p.add_argument('--epochs', type=int, default=40)
    p.add_argument('--batch', type=int, default=32)
    p.add_argument('--lr', type=float, default=1e-3)
    p.add_argument('--score-only', action='store_true')
    p.add_argument('--seed', type=int, default=1)
    args = p.parse_args(argv)
    torch.manual_seed(args.seed)
    device = 'mps' if torch.backends.mps.is_available() else (
        'cuda' if torch.cuda.is_available() else 'cpu')
    meta = json.loads((args.data / 'meta.json').read_text())
    images = np.load(args.data / 'images.npy', mmap_mode='r')
    lab = np.load(args.data / 'labels.npz')
    cols, colour, gt, held = lab['cols'], lab['colour'], lab['gt_y'], lab['held'].astype(bool)
    rows, cells = len(meta['rows']), meta['cells']
    model = LaneNet(rows, cells, input_hw=(meta['input_h'], meta['input_w'])).to(device)
    test_i = np.flatnonzero(held)
    if not args.score_only:
        train_i = np.flatnonzero(~held)
        print(f'{device}: {len(train_i)} training pictures, {len(test_i)} held out', flush=True)
        x_all = torch.from_numpy(np.ascontiguousarray(images[train_i]))
        cell_i, cell_f, has = targets(cols[train_i], cells)
        paint = torch.from_numpy(colour[train_i])
        mcell_i, mcell_f, mhas = targets(cols[train_i][:, MIRROR, :], cells)
        m_cols = cols[train_i][:, MIRROR, :]
        mcell_f_m = torch.from_numpy(
            np.where(np.isnan(m_cols), 0.0, (L.WIDTH - 1 - m_cols) / L.WIDTH * cells - 0.5)
            .transpose(0, 2, 1).astype(np.float32).copy())
        mcell_i_m = torch.clamp(torch.floor(mcell_f_m + 0.5), 0, cells - 1).long()
        mpaint = paint[:, MIRROR]
        opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
        steps = args.epochs * math.ceil(len(train_i) / args.batch)
        sched = torch.optim.lr_scheduler.OneCycleLR(opt, args.lr, total_steps=steps,
                                                    pct_start=0.05)
        gen = torch.Generator().manual_seed(args.seed)
        t0 = time.time()
        for epoch in range(args.epochs):
            model.train()
            order = torch.randperm(len(train_i), generator=gen)
            total = np.zeros(4)
            for s in range(0, len(order), args.batch):
                b = order[s:s + args.batch]
                x = x_all[b].to(device).permute(0, 3, 1, 2).float() / 255
                mirror = torch.rand(len(b), generator=gen) < 0.5
                ci = torch.where(mirror.view(-1, 1, 1), mcell_i_m[b], cell_i[b])
                cf = torch.where(mirror.view(-1, 1, 1), mcell_f_m[b], cell_f[b])
                hs = torch.where(mirror.view(-1, 1, 1), mhas[b], has[b])
                pt = torch.where(mirror.view(-1, 1), mpaint[b], paint[b])
                x = torch.where(mirror.view(-1, 1, 1, 1).to(device), x.flip(3), x)
                x = augment(x, gen)
                out = model(x)
                parts = losses(*out, ci.to(device), cf.to(device), hs.to(device), pt.to(device))
                loss = parts[0] + 0.1 * parts[1] + parts[2] + 0.1 * parts[3]
                opt.zero_grad()
                loss.backward()
                opt.step()
                sched.step()
                total += [float(v.detach()) for v in parts]
            n = math.ceil(len(order) / args.batch)
            print(f'epoch {epoch + 1}/{args.epochs} {time.time() - t0:.0f} s: loc '
                  f'{total[0] / n:.3f} mean {total[1] / n:.3f} exist {total[2] / n:.3f} '
                  f'colour {total[3] / n:.3f}', flush=True)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        torch.save(model.state_dict(), args.out)
        del mcell_i, mcell_f
    else:
        model.load_state_dict(torch.load(args.out, map_location=device))
    s = score(model, np.ascontiguousarray(images[test_i]), cols[test_i], colour[test_i],
              gt[test_i], meta, device)
    print(summary(s), flush=True)
    meta = {**meta, 'fit_x': list(FIT_X), 'scores': s, 'model': 'lane-ufld (TODO 262b)'}
    export(model, args.out.with_suffix('.onnx'), meta)
    print(f"wrote {args.out}, {args.out.with_suffix('.onnx')}, {args.out.with_suffix('.json')}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
