#!/usr/bin/env python3
"""
Every ONNX model in models/ on each provider: the same answers, coverage, ms.

The same answers on Core ML as on the CPU? How much of it does Core ML take? How fast is
each? PASS, WARN or FAIL per model (TODO 243, docs/onnx.md).

    pixi run measure onnx                    (the ml environment: ONNX Runtime, PyTorch)
    pixi run measure onnx --runs 50 --json log/onnx.json
    pixi run measure onnx --models diffusion --no-torch

For each model file there is (the float32 one and its _fp16 twin; a missing one is
SKIP), on each provider this ONNX Runtime has (the CPU; Core ML on a Mac; TensorRT and
CUDA where built in):

  answers   against the CPU's float32 answers (and PyTorch's, where the model has a
            PyTorch twin), within the model's tolerance below
  coverage  the nodes the provider took, and in how many pieces (ONNX Runtime's own log
            line), and which operators fell back to the CPU (its profile of one run).
            Core ML's coverage is the best guess, from a Mac, of whether TensorRT on the
            Orin will run the model whole
  time      the median ms of --runs runs after 5 warm-up runs (the model alone)

  PASS   the answers within tolerance, every node on the provider, in one piece
  WARN   the answers within tolerance, but some nodes on the CPU (or more pieces)
  FAIL   an answer out of tolerance, or the provider couldn't load or run it

The tolerances (the same for FP16 as for float32):
  detector   every box over 0.5 has a twin of its label: IoU >= 0.95, score within 0.02
             (tools/onnx_check.py's rule), on detector-core's 7 test pictures
  diffusion  16 paths on 10 made-up plans (a lot with boxes, training/measure.py's), in
             the map: each point within 0.10 m, each heading within 0.05 rad, and
             99 % of the points in the same gear
  lanes      the learned lane finder (TODO 262b) on lane-core's Ravensim frames and 30
             held-out pictures (models/lane-data): 98 % of the row decisions (a line or
             not) the same, and each line's column within 3 px (half a cell)
"""

import argparse
import contextlib
import json
import os
from pathlib import Path
import platform
import re
import statistics
import sys
import tempfile
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DIFFUSION = ROOT / 'ros' / 'drivers' / 'aav_diffusion_planner'
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(DIFFUSION))
sys.path.insert(0, str(DIFFUSION / 'training'))

PROVIDERS = {'cpu': 'CPUExecutionProvider', 'coreml': 'CoreMLExecutionProvider',
             'cuda': 'CUDAExecutionProvider', 'tensorrt': 'TensorrtExecutionProvider'}
COREML_OPTIONS = {'ModelFormat': 'MLProgram', 'MLComputeUnits': 'ALL',
                  'RequireStaticInputShapes': '1'}
DIFFUSION_POINT_M, DIFFUSION_HEADING, DIFFUSION_GEARS = 0.10, 0.05, 0.99
LANE_ROWS, LANE_PX = 0.98, 3.0
CAPABILITY = re.compile(r'number of partitions supported by \w+: (\d+) number of nodes in '
                        r'the graph: (\d+) number of nodes supported by \w+: (\d+)')


def coverage_from_log(text):
    """Return {'partitions', 'nodes', 'taken'} from ONNX Runtime's log, or None."""
    found = CAPABILITY.findall(text)
    if not found:
        return None
    parts, nodes, taken = (int(v) for v in found[-1])
    return {'partitions': parts, 'nodes': nodes, 'taken': taken}


def cpu_ops(trace):
    """Return {op: count} of the nodes the CPU ran, from ONNX Runtime's profile events."""
    nodes = {}
    for e in trace:
        args = e.get('args', {})
        if e.get('cat') == 'Node' and e.get('name', '').endswith('_kernel_time'):
            nodes[e['name']] = (args.get('provider'), args.get('op_name'))
    ops = {}
    for provider, op in nodes.values():
        if provider == PROVIDERS['cpu']:
            ops[op] = ops.get(op, 0) + 1
    return dict(sorted(ops.items(), key=lambda kv: -kv[1]))


@contextlib.contextmanager
def captured_stderr():
    """Catch what C++ writes to fd 2 (ONNX Runtime's log) into a list's one string."""
    out = []
    sys.stderr.flush()
    saved = os.dup(2)
    with tempfile.TemporaryFile(mode='w+b') as f:
        os.dup2(f.fileno(), 2)
        try:
            yield out
        finally:
            sys.stderr.flush()
            os.dup2(saved, 2)
            os.close(saved)
            f.seek(0)
            out.append(f.read().decode(errors='replace'))


def session(path, provider, profile=None):
    """Return (an ONNX Runtime session on `provider` alone, its coverage, load s)."""
    import onnxruntime as ort
    opts = ort.SessionOptions()
    # The coverage line: a warning when Core ML takes part of the graph, info when all.
    opts.log_severity_level = 3 if provider == 'cpu' else 1
    if profile:
        opts.enable_profiling = True
        opts.profile_file_prefix = profile
    name = PROVIDERS[provider]
    chain = [(name, COREML_OPTIONS if provider == 'coreml' else {})]
    if provider != 'cpu':
        chain.append((PROVIDERS['cpu'], {}))
    start = time.perf_counter()
    with captured_stderr() as log:
        sess = ort.InferenceSession(str(path), sess_options=opts,
                                    providers=[n for n, _ in chain],
                                    provider_options=[o for _, o in chain])
    load = time.perf_counter() - start
    if provider == 'cpu':
        cover = {'partitions': 1, 'nodes': None, 'taken': None}
    else:
        cover = coverage_from_log(log[0])
    return sess, cover, load


def median_ms(fn, runs):
    for _ in range(5):
        fn()
    times = []
    for _ in range(runs):
        start = time.perf_counter()
        fn()
        times.append(1000.0 * (time.perf_counter() - start))
    return statistics.median(times)


# ---- the models -------------------------------------------------------------------------

class Detector:
    """RF-DETR Nano: pictures in, boxes out (tools/onnx_check.py's steps)."""

    name = 'detector'
    files = ('models/rf-detr-nano.onnx', 'models/rf-detr-nano_fp16.onnx')
    rule = 'IoU >= 0.95, score within 0.02'

    def __init__(self):
        import cv2
        import onnx_check
        self.check = onnx_check
        from aav_detector.detectors import coco_names
        self.names = coco_names()
        self.pictures = [cv2.imread(str(f)) for f in sorted(onnx_check.PICTURES.glob('*.png'))]

    def runner(self, sess):
        from aav_detector.detr import input_size, preprocess
        first = sess.get_inputs()[0]
        height, width = input_size(first.shape)
        outputs = [o.name for o in sess.get_outputs()]
        feeds = [{first.name: preprocess(self.check.rgb_of(bgr), height, width)}
                 for bgr in self.pictures]

        def answers():
            return [dict(zip(outputs, sess.run(outputs, f))) for f in feeds]

        return answers, lambda: sess.run(outputs, feeds[0])

    def boxes(self, raw):
        from aav_detector.detr import decode, split_outputs
        out = []
        for bgr, r in zip(self.pictures, raw):
            boxes, logits = split_outputs(r)
            corners, scores, classes = decode(boxes, logits, bgr.shape[1], bgr.shape[0],
                                              threshold=0.5)
            out.append([(c, float(s), self.names.get(int(k), str(int(k))))
                        for c, s, k in zip(corners.tolist(), scores, classes)])
        return out

    def compare(self, raw, reference_boxes):
        """Return (within tolerance?, a short line)."""
        return self.compare_twin(self.boxes(raw), reference_boxes)

    def compare_twin(self, boxes, reference_boxes):
        """As compare, for boxes already decoded (PyTorch's)."""
        worst_iou, worst_gap, misses = 1.0, 0.0, 0
        for ours, theirs in zip(boxes, reference_boxes):
            i, g, m = self.check.compare(ours, theirs, 0.5)
            worst_iou, worst_gap, misses = min(worst_iou, i), max(worst_gap, g), misses + m
        ok = misses == 0 and worst_iou >= self.check.IOU_MIN and worst_gap <= \
            self.check.SCORE_GAP
        return ok, f'IoU {worst_iou:.4f}, score gap {worst_gap:.4f}, misses {misses}'

    def reference(self, raw):
        return self.boxes(raw)

    def torch(self, device):
        """Return (PyTorch's boxes, ms a picture), or None without the .pth."""
        pth = ROOT / 'models' / 'rf-detr-nano.pth'
        if not pth.exists():
            return None
        model = self.check.Torch(pth, device)
        boxes = [model.boxes(bgr, 0.5) for bgr in self.pictures]
        ms = median_ms(lambda: model.boxes(self.pictures[0], 0.5), 10)
        return boxes, ms


class Diffusion:
    """The diffusion planner's sampler: a crop, a cond and noise in, 16 paths out."""

    name = 'diffusion'
    files = ('models/diffusion/planner.onnx', 'models/diffusion/planner_fp16.onnx')
    rule = (f'points within {DIFFUSION_POINT_M} m, headings within {DIFFUSION_HEADING} rad,'
            f' {DIFFUSION_GEARS:.0%} same gear')

    def __init__(self, plans=10):
        import measure as dm
        from aav_diffusion_planner import window
        self.window = window
        rng = np.random.default_rng(7)
        self.feeds, self.windows = [], []
        for grid, start, goal, rev in dm.made_up_plans(plans, seed=3):
            win = window.window_for(start, goal)
            self.windows.append(win)
            self.feeds.append({
                'crop': window.crop(grid, win)[None, None].astype(np.float32),
                'cond': window.cond(win, start, goal, rev)[None],
                'noise': rng.standard_normal((16, window.POINTS, window.CHANNELS)).astype(
                    np.float32)})

    def runner(self, sess):
        def answers():
            return [sess.run(['paths'], f)[0] for f in self.feeds]

        return answers, lambda: sess.run(['paths'], self.feeds[0])

    def reference(self, raw):
        return raw

    def compare(self, raw, reference):
        worst_m, worst_h, same, total = 0.0, 0.0, 0, 0
        for win, a, b in zip(self.windows, raw, reference):
            pa, ga = self.window.decode_paths(win, a)
            pb, gb = self.window.decode_paths(win, b)
            worst_m = max(worst_m, float(np.max(np.hypot(pa[..., 0] - pb[..., 0],
                                                         pa[..., 1] - pb[..., 1]))))
            dh = np.angle(np.exp(1j * (pa[..., 2] - pb[..., 2])))
            worst_h = max(worst_h, float(np.max(np.abs(dh))))
            same += int(np.sum(np.sign(ga) == np.sign(gb)))
            total += ga.size
        share = same / total
        ok = worst_m <= DIFFUSION_POINT_M and worst_h <= DIFFUSION_HEADING and \
            share >= DIFFUSION_GEARS
        return ok, f'{worst_m:.3f} m, {worst_h:.3f} rad, {share:.1%} same gear'

    def compare_twin(self, raw, reference):
        return self.compare(raw, reference)

    def torch(self, device):
        pt = ROOT / 'models' / 'diffusion' / 'planner.pt'
        if not pt.exists():
            return None
        import model as net
        import torch
        m = net.FlowPlanner()
        m.load_state_dict(torch.load(str(pt), map_location='cpu'))
        sampler = net.Sampler(m.eval()).to(device)

        def one(f):
            with torch.no_grad():
                out = sampler(*(torch.from_numpy(f[k]).to(device)
                                for k in ('crop', 'cond', 'noise')))
            return out.cpu().numpy()
        raw = [one(f) for f in self.feeds]
        return raw, median_ms(lambda: one(self.feeds[0]), 10)


class Lanes:
    """The learned lane finder (TODO 262b): a picture in, each line's column at each row out."""

    name = 'lanes'
    files = ('models/lane-ufld.onnx', 'models/lane-ufld_fp16.onnx')
    rule = (f'{LANE_ROWS:.0%} of the rows the same (line or not), columns within {LANE_PX} px')
    pictures = ROOT / 'ros' / 'perception' / 'aav_lane_finder_rs' / 'crates' / 'lane-core' / \
        'tests' / 'pictures'

    def __init__(self, held=30):
        import lane_data
        self.data = lane_data
        files = sorted(self.pictures.glob('ravensim_frame_*.png'))
        self.inputs = [lane_data.crop(f) for f in files]
        held_dir = ROOT / 'models' / 'lane-data'
        if (held_dir / 'labels.npz').exists():
            images = np.load(held_dir / 'images.npy', mmap_mode='r')
            which = np.flatnonzero(np.load(held_dir / 'labels.npz')['held'])[:held]
            self.inputs += [np.asarray(images[i]) for i in which]
        self.feeds = [{'image': (x.transpose(2, 0, 1)[None] / 255.0).astype(np.float32)}
                      for x in self.inputs]

    def runner(self, sess):
        def answers():
            return [sess.run(['loc', 'exist'], f) for f in self.feeds]

        return answers, lambda: sess.run(['loc', 'exist'], self.feeds[0])

    def reference(self, raw):
        return raw

    def columns(self, raw):
        return [self.data.decode(loc[0], exist[0]) for loc, exist in raw]

    def compare(self, raw, reference):
        a, b = self.columns(raw), self.columns(reference)
        same = sum(int(np.sum(np.isnan(x) == np.isnan(y))) for x, y in zip(a, b))
        total = sum(x.size for x in a)
        gaps = [np.abs(x - y)[~np.isnan(x) & ~np.isnan(y)] for x, y in zip(a, b)]
        worst = max((float(g.max()) for g in gaps if g.size), default=0.0)
        share = same / total
        return share >= LANE_ROWS and worst <= LANE_PX, \
            f'{share:.1%} rows the same, columns within {worst:.2f} px'

    def compare_twin(self, raw, reference):
        return self.compare(raw, reference)

    def torch(self, device):
        pt = ROOT / 'models' / 'lane-ufld.pt'
        if not pt.exists():
            return None
        import json as js
        import lane_train
        import torch
        meta = js.loads((ROOT / 'models' / 'lane-ufld.json').read_text())
        m = lane_train.LaneNet(len(meta['rows']), meta['cells'],
                               input_hw=(meta['input_h'], meta['input_w']))
        m.load_state_dict(torch.load(str(pt), map_location='cpu'))
        m = m.eval().to(device)

        def one(f):
            with torch.no_grad():
                loc, exist, _ = m(torch.from_numpy(f['image']).to(device))
            return [loc.cpu().numpy(), exist.cpu().numpy()]
        raw = [one(f) for f in self.feeds]
        return raw, median_ms(lambda: one(self.feeds[0]), 10)


MODELS = {'detector': Detector, 'diffusion': Diffusion, 'lanes': Lanes}


# ---- the measuring ----------------------------------------------------------------------

def providers_here():
    import onnxruntime as ort
    have = ort.get_available_providers()
    return [p for p, name in PROVIDERS.items() if name in have]


def verdict(rows):
    """Return PASS, WARN or FAIL for one model's rows."""
    if any(r.get('error') or r.get('answers_ok') is False for r in rows):
        return 'FAIL'
    for r in rows:
        c = r.get('coverage')
        if r['provider'] != 'cpu' and (not c or c['taken'] != c['nodes'] or
                                       c['partitions'] > 1):
            return 'WARN'
    return 'PASS'


def measure_model(kind, runs, torch_device):
    rows, model = [], None
    reference = None
    for i, rel in enumerate(kind.files):
        path = ROOT / rel
        if not path.exists():
            rows.append({'file': rel, 'skip': 'missing'})
            continue
        model = model or kind()
        for provider in providers_here():
            row = {'file': rel, 'precision': 'fp16' if i else 'fp32', 'provider': provider}
            rows.append(row)
            try:
                sess, cover, load = session(path, provider)
                answers, once = model.runner(sess)
                raw = answers()
                row['load_s'] = round(load, 1)
                row['ms'] = round(median_ms(once, runs), 2)
                row['coverage'] = cover
                with tempfile.TemporaryDirectory() as tmp:
                    prof, _, _ = session(path, provider, profile=os.path.join(tmp, 'p'))
                    model.runner(prof)[1]()
                    with open(prof.end_profiling()) as f:
                        row['cpu_ops'] = {} if provider == 'cpu' else cpu_ops(json.load(f))
            except Exception as e:  # noqa: B902 - a provider that can't is a result
                row['error'] = str(e).splitlines()[0][:200]
                continue
            if reference is None:       # the first: float32 on the CPU
                reference = model.reference(raw)
                row['answers_ok'], row['answers'] = True, 'the reference'
            else:
                row['answers_ok'], row['answers'] = model.compare(raw, reference)
    twin = {}
    if model is not None and torch_device and reference is not None:
        found = model.torch(torch_device)
        if found:
            raw, ms = found
            ok, line = model.compare_twin(raw, reference)
            twin = {'device': torch_device, 'answers_ok': ok, 'answers': line,
                    'ms': round(ms, 2)}
    return rows, twin


def coverage_text(row):
    c = row.get('coverage')
    if row['provider'] == 'cpu':
        return 'all'
    if not c:
        return 'not logged'
    pct = 100.0 * c['taken'] / c['nodes'] if c['nodes'] else 100.0
    return f"{c['taken']} of {c['nodes']} ({pct:.1f} %), {c['partitions']} piece" + \
        ('s' if c['partitions'] != 1 else '')


def report(results):
    lines = [f"ONNX models on {results['machine']}, ONNX Runtime {results['onnxruntime']}, "
             f"median of {results['runs']} runs", '']
    for name, r in results['models'].items():
        lines.append(f"## {name}: {r['verdict']}  ({r['rule']})")
        lines.append('')
        lines.append('| file | provider | answers vs the CPU fp32 | nodes on the provider |'
                     ' fell back to the CPU | ms | load s |')
        lines.append('|---|---|---|---|---|---|---|')
        for row in r['rows']:
            if 'skip' in row:
                lines.append(f"| {row['file']} | - | SKIP: {row['skip']} | | | | |")
                continue
            if 'error' in row:
                lines.append(f"| {row['file']} | {row['provider']} | FAIL: {row['error']} "
                             '| | | | |')
                continue
            ok = 'ok' if row['answers_ok'] else 'OUT'
            ops = ', '.join(f'{o} {n}' for o, n in row.get('cpu_ops', {}).items()) or '-'
            lines.append(f"| {row['file']} | {row['provider']} | {ok}: {row['answers']} | "
                         f"{coverage_text(row)} | {ops} | {row['ms']} | {row['load_s']} |")
        t = r.get('torch')
        if t:
            ok = 'ok' if t['answers_ok'] else 'OUT'
            lines.append(f"| PyTorch ({t['device']}) | - | {ok}: {t['answers']} | | | "
                         f"{t['ms']} | |")
        lines.append('')
    return '\n'.join(lines)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('--models', nargs='+', choices=sorted(MODELS), default=sorted(MODELS))
    p.add_argument('--runs', type=int, default=30, help='timed runs a provider (median)')
    p.add_argument('--torch-device', default='mps' if sys.platform == 'darwin' else 'cpu',
                   help="PyTorch's device for the twins: mps, cpu, cuda")
    p.add_argument('--no-torch', action='store_true', help='skip the PyTorch twins')
    p.add_argument('--json', metavar='FILE', help='also write the results as JSON')
    args = p.parse_args(argv)
    import onnxruntime as ort
    results = {'machine': f'{platform.node()} ({platform.machine()}, {platform.system()})',
               'onnxruntime': ort.__version__, 'runs': args.runs,
               'providers': providers_here(), 'models': {}}
    for name in args.models:
        kind = MODELS[name]
        rows, twin = measure_model(kind, args.runs, None if args.no_torch else
                                   args.torch_device)
        measured = [r for r in rows if 'skip' not in r]
        results['models'][name] = {
            'rule': kind.rule, 'rows': rows, 'torch': twin,
            'verdict': ('SKIP' if not measured else
                        'FAIL' if twin and not twin['answers_ok'] else verdict(measured))}
    print(report(results))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(results, indent=1, default=float) + '\n')
    verdicts = [m['verdict'] for m in results['models'].values()]
    return 1 if 'FAIL' in verdicts else 0


if __name__ == '__main__':
    sys.exit(main())
