#!/usr/bin/env python3
"""
Train the diffusion planner on the teachers' paths, score it with the guard, export ONNX.

    pixi run diffusion train                 (dataset, train, score, export: models/diffusion/)
    pixi run diffusion train --epochs 2      (a quick look)
    pixi run diffusion score                 (score the trained model again, no training)

Runs in the learn environment (.pixi/envs/learn: PyTorch, ONNX, ONNX Runtime; no ROS).
In: models/diffusion/data/*.npz (pixi run diffusion data). Out, in models/diffusion/ (not
in git): dataset.npz (the crops, made once), planner.pt (the weights), planner.onnx (the
sampler: crop, cond, noise -> 16 paths), metrics.json (how training went and the score).

The examples: each teacher path the guard accepts, and two more from points along it
(the car replans from where it is, once a second; the rest of a good path is a good path
from there). Mirrored at random while training. Every HOLD-th map is held back: the score
is on those maps only.
"""

import argparse
import glob
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))      # aav_diffusion_planner, beside training/
sys.path.insert(0, HERE)

from aav_diffusion_planner import guard, window  # noqa: E402
import model as net  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
OUT = os.path.join(ROOT, 'models', 'diffusion')
SUFFIXES = 2            # extra examples from points along each path
MIN_LEFT = 4.0          # m: a suffix at least this far from the goal
SAMPLES = 16            # paths a plan
STEPS = 10              # Euler steps a plan


def vehicle_numbers():
    """Return (footprint polygon padded as the costmap pads it, full-lock radius) (m)."""
    import yaml
    with open(os.path.join(ROOT, 'ros', 'bringup', 'aav_description', 'config',
                           'vehicle.yaml')) as f:
        v = yaml.safe_load(f)['vehicle']
    with open(os.path.join(ROOT, 'ros', 'drivers', 'aav_navigation', 'config',
                           'nav2.yaml')) as f:
        pad = yaml.safe_load(f)['global_costmap']['global_costmap']['ros__parameters'][
            'footprint_padding']
    body = v['body']
    rear, half = -body['rear_overhang'] - pad, body['width'] / 2.0 + pad
    front = -body['rear_overhang'] + body['length'] + pad
    lock = v['wheelbase'] / math.tan(v['max_steering_angle'])
    return [[front, half], [front, -half], [rear, -half], [rear, half]], lock


def limits_for(reversing, lock):
    return guard.Limits(min_radius=0.95 * lock, reversing=bool(reversing))


def examples(path, start, goal, rng):
    """Yield (start, path) pairs: the whole path, then SUFFIXES from points along it."""
    yield start, path
    seg = np.hypot(*np.diff(path[:, :2], axis=0).T)
    left = seg.sum() - np.concatenate([[0.0], np.cumsum(seg)])
    can = np.nonzero((left > MIN_LEFT) & (np.arange(len(path)) > 0))[0]
    for i in rng.choice(can, size=min(SUFFIXES, len(can)), replace=False) if len(can) else []:
        yield tuple(path[i]), path[i:]


def build(data_dir, rng):
    """Return the dataset: crops (uint8), conds, targets, test flags, and what was dropped."""
    poly, lock = vehicle_numbers()
    fp = guard.footprint_points(poly)
    crops, conds, targets, test, meta = [], [], [], [], []
    seen = refused = 0
    for f in sorted(glob.glob(os.path.join(data_dir, '*.npz'))):
        d = np.load(f)
        grid = window.Grid(d['costmap'], float(d['origin'][0]), float(d['origin'][1]),
                           float(d['resolution']))
        is_test = os.path.basename(f).startswith('test_')
        for k in range(len(d['plan_ms'])):
            seen += 1
            path = d['path_points'][d['path_start'][k]:d['path_start'][k + 1]]
            start, goal, rev = d['start'][k], d['goal'][k], bool(d['reversing'][k])
            failed, _ = guard.check(path, grid, start, goal, limits_for(rev, lock), fp)
            if failed:
                refused += 1
                continue
            for s, p in examples(path, start, goal, rng):
                win = window.window_for(s, goal)
                crops.append((window.crop(grid, win) * 255).astype(np.uint8))
                conds.append(window.cond(win, s, goal, rev))
                targets.append(window.encode_path(win, p))
                test.append(is_test)
                meta.append((os.path.basename(f), k, str(d['teacher'][k])))
                if is_test:
                    break           # the test keeps whole paths only
    return (np.stack(crops), np.stack(conds), np.stack(targets), np.array(test),
            np.array(meta), {'teacher_paths': seen, 'refused_by_guard': refused})


def mirror(crop, cond, x):
    """Flip left for right: v, sin of the goal and of every heading change sign."""
    crop = torch.flip(crop, dims=[2])
    cond = cond.clone()
    cond[:, [1, 3, 5]] *= -1.0
    x = x.clone()
    x[:, :, [1, 3]] *= -1.0
    return crop, cond, x


def device():
    if torch.backends.mps.is_available():
        return torch.device('mps')
    return torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def train(ds, args, dev):
    crops, conds, targets, test = ds[:4]
    idx = np.nonzero(~test)[0]
    crops_t = torch.from_numpy(crops[idx])
    conds_t = torch.from_numpy(conds[idx])
    targets_t = torch.from_numpy(targets[idx])
    m = net.FlowPlanner().to(dev)
    opt = torch.optim.AdamW(m.parameters(), lr=args.lr, weight_decay=1e-4)
    steps_per = max(1, len(idx) // args.batch)
    total = steps_per * args.epochs
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=args.lr, total_steps=total,
                                                pct_start=0.05)
    gen = torch.Generator().manual_seed(args.seed)
    losses, t0 = [], time.monotonic()
    for epoch in range(args.epochs):
        perm = torch.randperm(len(idx), generator=gen)
        run = 0.0
        for s in range(steps_per):
            b = perm[s * args.batch:(s + 1) * args.batch]
            crop = crops_t[b].to(dev).float().unsqueeze(1) / 255.0
            cond, x1 = conds_t[b].to(dev), targets_t[b].to(dev)
            flip = torch.rand(len(b), device=dev) < 0.5
            if flip.any():
                fc, fd, fx = mirror(crop[flip], cond[flip], x1[flip])
                crop, cond, x1 = crop.clone(), cond.clone(), x1.clone()
                crop[flip], cond[flip], x1[flip] = fc, fd, fx
            x0 = torch.randn_like(x1)
            t = torch.rand(len(b), device=dev)
            xt = (1.0 - t)[:, None, None] * x0 + t[:, None, None] * x1
            loss = torch.mean((m(xt, t, crop, cond) - (x1 - x0)) ** 2)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(m.parameters(), 1.0)
            opt.step()
            sched.step()
            run += loss.item()
        losses.append(run / steps_per)
        print(f'epoch {epoch + 1}/{args.epochs}: loss {losses[-1]:.4f} '
              f'({time.monotonic() - t0:.0f} s)', flush=True)
    return m, {'epochs': args.epochs, 'batch': args.batch, 'train_examples': int(len(idx)),
               'train_seconds': round(time.monotonic() - t0, 1), 'device': str(dev),
               'loss_first': round(losses[0], 4), 'loss_last': round(losses[-1], 4),
               'weights': sum(p.numel() for p in m.parameters())}


def score(sampler_fn, ds, data_dir, limit, seed=0):
    """
    Return the guard's score on the held-back maps: each test example planned once.

    sampler_fn(crop (1,1,S,S) float32, cond (1,8), noise (K,64,5)) -> (K,64,5) numpy.
    """
    crops, conds, _, test, meta = ds[:5]
    poly, lock = vehicle_numbers()
    fp = guard.footprint_points(poly)
    rng = np.random.default_rng(seed)
    idx = np.nonzero(test)[0][:limit]
    grids = {}
    samples_ok = samples = plans_ok = 0
    refusals, ratios, ms = {}, [], []
    for i in idx:
        name, k, _ = meta[i]
        if name not in grids:
            d = np.load(os.path.join(data_dir, name))
            grids[name] = (window.Grid(d['costmap'], float(d['origin'][0]),
                                       float(d['origin'][1]), float(d['resolution'])), d)
        grid, d = grids[name]
        k = int(k)
        start, goal, rev = d['start'][k], d['goal'][k], bool(d['reversing'][k])
        teacher = d['path_points'][d['path_start'][k]:d['path_start'][k + 1]]
        noise = rng.standard_normal((SAMPLES, window.POINTS, window.CHANNELS)).astype(
            np.float32)
        t0 = time.perf_counter()
        out = sampler_fn(crops[i][None, None].astype(np.float32) / 255.0, conds[i][None], noise)
        ms.append(1000.0 * (time.perf_counter() - t0))
        win = window.window_for(start, goal)
        paths, _ = window.decode_paths(win, out)
        lim = limits_for(rev, lock)
        teacher_len = guard.length(teacher)
        passed = []
        for p in paths:
            failed, facts = guard.check(window.smooth(p), grid, start, goal, lim, fp, teacher_len)
            for c in failed:
                refusals[c] = refusals.get(c, 0) + 1
            if not failed:
                passed.append(guard.cost(facts))
                ratios.append(facts['length'] / teacher_len)
        samples += len(paths)
        samples_ok += len(passed)
        plans_ok += bool(passed)
    ms.sort()
    return {'test_plans': int(len(idx)), 'samples_passed': round(samples_ok / samples, 3),
            'plans_with_a_pass': round(plans_ok / len(idx), 3),
            'refusals_by_check': dict(sorted(refusals.items())),
            'passed_length_vs_teacher_mean': round(float(np.mean(ratios)), 3) if ratios else None,
            'sampler_ms_mean': round(float(np.mean(ms)), 1),
            'sampler_ms_worst': round(ms[-1], 1)}


def export(m, path, steps=STEPS):
    sampler = net.Sampler(m.cpu().eval(), steps)
    crop = torch.zeros(1, 1, window.SIZE, window.SIZE)
    cond = torch.zeros(1, window.COND)
    noise = torch.zeros(SAMPLES, window.POINTS, window.CHANNELS)
    torch.onnx.export(sampler, (crop, cond, noise), path, input_names=['crop', 'cond', 'noise'],
                      output_names=['paths'], opset_version=17, dynamo=False,
                      dynamic_axes={'noise': {0: 'k'}, 'paths': {0: 'k'}})


def onnx_fn(path, provider='CPUExecutionProvider'):
    import onnxruntime as ort
    sess = ort.InferenceSession(path, providers=[provider])

    def run(crop, cond, noise):
        return sess.run(['paths'], {'crop': crop, 'cond': cond, 'noise': noise})[0]
    return run


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('what', nargs='?', default='train', choices=('train', 'score'))
    parser.add_argument('--data', default=os.path.join(OUT, 'data'))
    parser.add_argument('--out', default=OUT)
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--batch', type=int, default=256)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--test-plans', type=int, default=300)
    parser.add_argument('--rebuild', action='store_true', help='make dataset.npz again')
    args = parser.parse_args()
    torch.manual_seed(args.seed)
    os.makedirs(args.out, exist_ok=True)
    cache = os.path.join(args.out, 'dataset.npz')
    if os.path.exists(cache) and not args.rebuild:
        z = np.load(cache)
        ds = (z['crops'], z['conds'], z['targets'], z['test'], z['meta'],
              json.loads(str(z['info'])))
    else:
        t0 = time.monotonic()
        ds = build(args.data, np.random.default_rng(args.seed))
        ds[5]['build_seconds'] = round(time.monotonic() - t0, 1)
        np.savez(cache, crops=ds[0], conds=ds[1], targets=ds[2], test=ds[3], meta=ds[4],
                 info=json.dumps(ds[5]))
    info = dict(ds[5], examples=int(len(ds[0])), test_examples=int(ds[3].sum()))
    print(f'dataset: {info}', flush=True)
    metrics_path = os.path.join(args.out, 'metrics.json')
    onnx_path = os.path.join(args.out, 'planner.onnx')
    if args.what == 'train':
        m, trained = train(ds, args, device())
        torch.save(m.state_dict(), os.path.join(args.out, 'planner.pt'))
        export(m, onnx_path)
        metrics = {'dataset': info, 'training': trained}
    else:
        with open(metrics_path) as f:
            metrics = json.load(f)
    metrics['score_onnx_cpu'] = score(onnx_fn(onnx_path), ds, args.data, args.test_plans)
    metrics['onnx_bytes'] = os.path.getsize(onnx_path)
    with open(metrics_path, 'w') as f:
        json.dump(metrics, f, indent=1)
    print(json.dumps(metrics, indent=1))


if __name__ == '__main__':
    main()
