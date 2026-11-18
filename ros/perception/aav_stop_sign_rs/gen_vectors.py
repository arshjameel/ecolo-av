#!/usr/bin/env python3
"""
Write stop-sign-core's agreement vectors from aav_detector's stop_sign.py.

Runs of the watcher (boxes near and far, other labels, low scores, the 2024-25 model's
label, lenses or none, quiet ticks) under several rules; and sign_point, biggest_sign,
lidar_sign, normal_label, is_optical and sign_distance_m on random and hand-picked cases.
The Rust (crates/stop-sign-core, tests/vectors.rs) must give the same answers: the states
and booleans the same, the numbers within 1e-12.

Writes ros/perception/aav_stop_sign_rs/crates/stop-sign-core/tests/vectors.json:

  pixi run python ros/perception/aav_stop_sign_rs/gen_vectors.py
"""

from dataclasses import asdict
import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / 'aav_detector')]

from aav_detector.boxes import normal_label  # noqa: E402
from aav_detector.ground import is_optical  # noqa: E402
from aav_detector.stop_sign import (biggest_sign, lidar_sign, sign_distance_m,  # noqa: E402
                                    sign_point, StopRule, StopSignWatcher)

OUT = HERE / 'crates' / 'stop-sign-core' / 'tests' / 'vectors.json'
LABELS = ['stop sign', 'Stop_Sign', 'person', 'stop-sign', 'STOP  sign', 'car', 'stop']


def enc(v):
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def boxes(rng):
    return [[rng.choice(LABELS), rng.choice([0.3, 0.5, 0.9, rng.random()]),
             rng.choice([0.0, 30.0, 55.0, 54.73, 100.0, rng.uniform(0, 200)]),
             rng.uniform(0, 640)] for _ in range(rng.choice([0, 0, 1, 1, 2, 3]))]


def runs():
    rng = random.Random(4)
    rules = [StopRule(), StopRule(stop_distance_m=3.5),
             StopRule(stop_distance_m=6.0, focal_px=950.0, min_score=0.7, hold_s=1.5,
                      ignore_s=3.0),
             StopRule(sign_height_m=0.6, hold_s=0.5, ignore_s=0.0)]
    out = []
    for rule in rules:
        w = StopSignWatcher(rule)
        t = rng.uniform(0, 100)
        steps = []
        for _ in range(600):
            t = round(t + rng.choice([0.05, 0.1, 0.1, 0.5, 2.0]), 6)
            if rng.random() < 0.3:
                steps.append({'op': 'tick', 't': t, 'stop': w.tick(t), 'state': w.state})
                continue
            b = boxes(rng)
            focal = rng.choice([None, None, 343.0, 950.0, 0.0, 1400.0])
            stop = w.update([(lbl, s, h) for lbl, s, h, _ in b], t, focal)
            steps.append({'op': 'update', 't': t, 'boxes': b, 'focal': focal, 'stop': stop,
                          'state': w.state})
        out.append({'rule': asdict(rule), 'steps': steps})
    return out


def points():
    rng = random.Random(5)
    out = []
    for _ in range(200):
        args = [rng.uniform(0, 1280), rng.choice([0.0, rng.uniform(1, 300)]),
                rng.choice([343.0, 950.0, rng.uniform(100, 1500)]),
                rng.choice([None, 0.0, 343.0, rng.uniform(100, 1500)]),
                rng.choice([None, 320.0, rng.uniform(0, 1280)]),
                rng.random() < 0.5, rng.choice([0.75, 0.6])]
        fx, cx = args[3], args[4]
        out.append({'args': args, 'lens': fx is not None and cx is not None,
                    'point': list(sign_point(args[0], args[1], args[2], fx, cx, args[5],
                                             args[6]))})
    return out


def biggest():
    rng = random.Random(6)
    out = []
    for _ in range(150):
        b = boxes(rng) + boxes(rng)
        score = rng.choice([0.5, 0.0, 0.8])
        out.append({'boxes': b, 'min_score': score,
                    'best': biggest_sign([(lbl, s, h, c) for lbl, s, h, c in b], score)})
    return out


def lidar():
    rng = random.Random(7)
    out = []
    for _ in range(150):
        things = [[rng.choice(LABELS + ['unknown']), rng.uniform(-5, 25), rng.uniform(-5, 5)]
                  for _ in range(rng.randrange(0, 6))]
        guess = [rng.uniform(0, 20), rng.uniform(-3, 3)]
        gate = rng.choice([3.0, 1.0, 10.0])
        found = lidar_sign(tuple(guess), [tuple(t) for t in things], gate)
        out.append({'guess': guess, 'things': things, 'gate': gate,
                    'found': None if found is None else list(found)})
    return out


def words():
    names = LABELS + ['  stop_ _sign ', 'Traffic-Light', '']
    frames = ['camera_front_left_optical', 'camera_front_left_optical_frame',
              'camera_left_link', 'optical', 'x_optical_frame_y']
    return {'labels': [[n, normal_label(n)] for n in names],
            'optical': [[f, is_optical(f)] for f in frames],
            'distances': [[h, f, s, sign_distance_m(h, f, s)]
                          for h in (0.0, -1.0, 55.0, 152.0) for f in (343.0, 950.0)
                          for s in (0.75, 0.6)],
            'min_heights': [[asdict(r), f, r.min_height_px(f)]
                            for r in (StopRule(), StopRule(stop_distance_m=3.5))
                            for f in (None, 0.0, 950.0)]}


def build():
    """Return the vectors (NaN, inf as strings): aav_detector's test reruns it."""
    return enc({'note': 'Written by gen_vectors.py from aav_detector stop_sign.py; do not edit.',
                'runs': runs(), 'points': points(), 'biggest': biggest(), 'lidar': lidar(),
                'words': words()})


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(build(), separators=(',', ':')) + '\n')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
