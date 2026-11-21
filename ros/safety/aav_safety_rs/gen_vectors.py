#!/usr/bin/env python3
"""
Write safety-core's agreement vectors from aav_safety's geofence.py, hold.py and leash.py.

The geofence: drives, walks out, jumps, gaps in the pose and in GPS, unsure and NaN poses,
clears, on circles and polygons (a concave one too), with stands, coasting and gps:=false;
its maths (depth, stopping distance, arcs, the stopping paths' points) on random inputs;
and the fences and settings it refuses, with their lines. The stop hold and the leash:
random runs of states, beats and ticks. The Rust (crates/safety-core, tests/vectors.rs)
must give the same answers, states and lines at every event, and the same numbers to the
bit.

Writes ros/safety/aav_safety_rs/crates/safety-core/tests/vectors.json:

  pixi run python ros/safety/aav_safety_rs/gen_vectors.py
"""

import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / 'aav_safety')]

from aav_safety import geofence as gf  # noqa: E402
from aav_safety.hold import StopHold  # noqa: E402
from aav_safety.leash import Leash  # noqa: E402

OUT = HERE / 'crates' / 'safety-core' / 'tests' / 'vectors.json'

FOOTPRINT = [[1.7, 0.5], [1.7, -0.5], [-0.3, -0.5], [-0.3, 0.5]]
FENCES = [
    {'circle': [0.0, 0.0, 30.0]},
    {'circle': [12.5, -7.25, 18.0]},
    {'polygon': [-20.0, -10.0, 40.0, -10.0, 40.0, 10.0, -20.0, 10.0]},
    # An L, concave, drawn clockwise.
    {'polygon': [0.0, 0.0, 0.0, 30.0, 10.0, 30.0, 10.0, 10.0, 30.0, 10.0, 30.0, 0.0]},
    {'polygon': [-15.0, -15.0, 25.0, -12.0, 3.0, 22.0]},
]
MOTIONS = [
    {'reaction_time': 0.6, 'braking': 1.5, 'max_accel': 1.0, 'max_speed': 2.0,
     'max_reverse_speed': 1.0, 'max_curvature': 0.45},
    {'reaction_time': 1.234, 'braking': 0.75, 'max_accel': 0.0, 'max_speed': 3.5,
     'max_reverse_speed': 0.8, 'max_curvature': 0.0},
    {'reaction_time': 0.3, 'braking': 2.5, 'max_accel': 2.0, 'max_speed': 1.0,
     'max_reverse_speed': 1.0, 'max_curvature': 0.3},
]
SETTINGS = [
    {},
    {'margin': 0.5, 'hysteresis': 0.0, 'max_std': 1.5, 'max_age': 0.4, 'gps_max_age': 1.0,
     'settle_time': 0.0, 'jump_margin': 0.5},
]
MODES = [
    {'gps': True, 'coasting': False, 'stands': False},
    {'gps': False, 'coasting': False, 'stands': False},
    {'gps': True, 'coasting': True, 'stands': False},
    {'gps': False, 'coasting': False, 'stands': True},
]


def enc(v):
    """Return the value with NaN and infinities as text: JSON has no word for them."""
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def make_fence(spec):
    if 'circle' in spec:
        return gf.Circle(*spec['circle'])
    return gf.Polygon.from_flat(spec['polygon'])


def pose_list(p):
    return [p.x, p.y, p.yaw, p.speed, p.std]


def run(rng, fence_spec, motion, settings, mode):
    fence = gf.Geofence(FOOTPRINT, make_fence(fence_spec), gf.Motion(**motion),
                        gf.Settings(**settings), **mode)
    events = []
    t = rng.uniform(0.0, 2.0)
    x, y, yaw, speed = rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi), 0.0
    events.append({'op': 'check', 't': t, 'out': list(fence.check(t))})
    gps_quiet_until = -1.0
    pose_quiet_until = -1.0
    for step in range(600):
        t = round(t + 0.05 + rng.uniform(-0.01, 0.01), 6)
        roll = rng.random()
        if roll < 0.004:
            gps_quiet_until = t + rng.choice([1.0, 2.5, 4.0])
        elif roll < 0.007:
            pose_quiet_until = t + rng.choice([0.5, 1.5])
        if step % 150 == 149:
            # Back home, to see every state again (a jump, if settled).
            x, y = rng.uniform(-2, 2), rng.uniform(-2, 2)
        speed = max(-1.0, min(2.5, speed + rng.uniform(-0.3, 0.32)))
        yaw += rng.uniform(-0.08, 0.08)
        x += speed * 0.05 * math.cos(yaw)
        y += speed * 0.05 * math.sin(yaw)
        std = rng.choice([0.05, 0.2, 0.6, 1.2, 2.9, 3.0])
        if rng.random() < 0.01:
            std = rng.choice([3.5, float('nan')])
        px, pyy, pspeed = x, y, speed
        if rng.random() < 0.003:
            px, pyy = x + rng.uniform(-8, 8), y + rng.uniform(-8, 8)     # one wild pose
        if rng.random() < 0.002:
            pspeed = float('nan')
        if t >= pose_quiet_until:
            pose = gf.Pose(px, pyy, yaw, pspeed, std)
            fence.update(t, pose)
            events.append({'op': 'pose', 't': t, 'pose': pose_list(pose)})
        if t >= gps_quiet_until and step % 4 == 0:
            fence.gps_arrived(t)
            events.append({'op': 'gps', 't': t})
        if step % 2 == 0:
            events.append({'op': 'check', 't': t, 'out': list(fence.check(t))})
        if rng.random() < 0.02 or (fence.latched and rng.random() < 0.05):
            events.append({'op': 'clear', 't': t, 'out': list(fence.clear(t))})
    return {'fence': fence_spec, 'motion': motion, 'settings': settings, 'mode': mode,
            'events': events}


def maths(rng):
    out = {'depth': [], 'stopping': [], 'arc': [], 'points': [], 'outline': [], 'hypot': []}
    edges = [0.0, -0.0, 5e-324, 2.2250738585072014e-308, 1e-300, 1e300, 1.7976931348623157e308,
             3.0, 4.0, -7.5, math.inf, -math.inf, float('nan')]
    pairs = [(a, b) for a in edges for b in edges]
    pairs += [(rng.uniform(-50, 50), rng.uniform(-50, 50)) for _ in range(300)]
    out['hypot'] = [{'args': [a, b], 'h': math.hypot(a, b)} for a, b in pairs]
    for spec in FENCES:
        f = make_fence(spec)
        for _ in range(120):
            px, py = rng.uniform(-45, 55), rng.uniform(-45, 45)
            out['depth'].append({'fence': spec, 'p': [px, py], 'd': f.depth(px, py)})
        out['outline'].append({'fence': spec, 'points': [list(p) for p in f.outline()]})
    for _ in range(400):
        args = [rng.uniform(-4, 4), rng.choice([0.0, rng.uniform(0, 2)]),
                rng.choice([0.0, rng.uniform(0, 3)]), rng.uniform(0.2, 4),
                rng.choice([math.inf, rng.uniform(0, 4)])]
        out['stopping'].append({'args': args, 'd': gf.stopping_distance(*args)})
    for _ in range(400):
        args = [rng.uniform(-30, 30), rng.uniform(-30, 30), rng.uniform(-4, 4),
                rng.uniform(-8, 8), rng.choice([0.0, 5e-10, rng.uniform(-0.6, 0.6)])]
        out['arc'].append({'args': args, 'pose': list(gf.along_arc(*args))})
    for motion in MOTIONS:
        fence = gf.Geofence(FOOTPRINT, gf.Circle(0, 0, 30), gf.Motion(**motion))
        for _ in range(8):
            p = gf.Pose(rng.uniform(-10, 10), rng.uniform(-10, 10), rng.uniform(-4, 4),
                        rng.uniform(-1.5, 3.0), 0.1)
            out['points'].append({'motion': motion, 'pose': pose_list(p),
                                  'stopping': list(fence.stopping(p.speed)),
                                  'points': [list(q) for q in fence.points(p)]})
    return out


def refusals():
    """Each constructor's answer: None (taken) or the Python's error line."""
    def attempt(fn):
        try:
            fn()
        except ValueError as e:
            return str(e)
        return None

    nan = float('nan')
    out = []
    for c in ([0.0, 0.0, 30.0], [0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [nan, 0.0, 5.0],
              [0.0, math.inf, 5.0], [0.0, 0.0, math.inf]):
        out.append({'kind': 'circle', 'args': c, 'error': attempt(lambda: gf.Circle(*c))})
    for p in ([0.0, 0.0, 10.0, 0.0, 10.0, 10.0], [0.0, 0.0, 10.0, 0.0], [0.0, 0.0, 1.0],
              [0.0, 0.0, 1.0, 0.0, 2.0, 0.0], [0.0, 0.0, 1.3, 0.0, 1.3, 1.3],
              [0.0, 0.0, 1.5, 0.0, 1.5, 1.5], [0.0, 0.0, nan, 0.0, 5.0, 5.0],
              [0.0, 0.0, 0.1, 0.0, 0.1, 0.1, 0.0, 0.1]):
        out.append({'kind': 'polygon', 'args': p,
                    'error': attempt(lambda: gf.Polygon.from_flat(p))})
    square = [-20.0, -10.0, 40.0, -10.0, 40.0, 10.0, -20.0, 10.0]
    for site_datum, datum in (([45.386, -75.696], [45.386, -75.696, 0.0]),
                              ([45.386, -75.696], [45.38600005, -75.696]),
                              ([45.386, -75.696], [45.3861, -75.696]),
                              ([45.386], [45.386, -75.696]),
                              ([45.0, -75.0], [0.0]),
                              ([45.386, -75.696, 3.0], [45.3, -75.0, 0.0])):
        out.append({'kind': 'site', 'site': 'bench_road', 'corners': square,
                    'site_datum': site_datum, 'datum': datum,
                    'error': attempt(lambda: gf.site_fence('bench_road', square, site_datum,
                                                           datum))})
    good = MOTIONS[0]
    for name, value in (('reaction_time', -0.1), ('braking', 0.0), ('braking', nan),
                        ('max_accel', math.inf), ('max_speed', nan),
                        ('max_curvature', -1.0), ('max_reverse_speed', 0.0)):
        m = dict(good, **{name: value})
        out.append({'kind': 'motion', 'args': m, 'error': attempt(lambda: gf.Motion(**m))})
    for name, value in (('margin', -1.0), ('hysteresis', nan), ('max_std', 0.0),
                        ('max_age', math.inf), ('gps_max_age', -2.0), ('settle_time', 0.0),
                        ('jump_margin', math.inf)):
        s = {name: value}
        out.append({'kind': 'settings', 'args': s, 'error': attempt(lambda: gf.Settings(**s))})
    return out


def hold_runs(rng):
    runs = []
    for clear_time in (1.0, 0.0, 0.35, 2.5):
        hold = StopHold(clear_time)
        t = rng.uniform(0, 1)
        events = []
        for _ in range(1500):
            t = round(t + rng.choice([0.01, 0.05, 0.1, 0.3]), 6)
            if rng.random() < 0.4:
                stop = rng.random() < (0.5 if rng.random() < 0.5 else 0.05)
                hold.update(t, stop)
                events.append({'op': 'update', 't': t, 'stop': stop})
            events.append({'op': 'locked', 't': t, 'locked': hold.locked(t)})
        runs.append({'clear_time': clear_time, 'events': events})
    refused = []
    for clear_time in (float('nan'), math.inf, -0.5, 0.0, 3.0):
        try:
            StopHold(clear_time)
            refused.append({'clear_time': clear_time, 'ok': True})
        except ValueError:
            refused.append({'clear_time': clear_time, 'ok': False})
    return {'runs': runs, 'refused': refused}


def leash_runs(rng):
    runs = []
    for timeout in (1.0, 0.25, 0.0, 3.0):
        leash = Leash(timeout=timeout)
        t = rng.uniform(0, 1)
        events = []
        count = 0
        for _ in range(1500):
            t = round(t + rng.choice([0.05, 0.1, 0.2, 0.5, 1.5]), 6)
            if rng.random() < 0.5:
                count = 0 if rng.random() < 0.08 else count + 1
                leash.beat(count, t)
                events.append({'op': 'beat', 't': t, 'value': count, 'armed': leash.armed})
            events.append({'op': 'check', 't': t, 'trip': leash.check(t), 'armed': leash.armed})
        runs.append({'timeout': timeout, 'events': events})
    return runs


def build():
    """Return the vectors (NaN as 'nan'): aav_safety's test_safety_rs_vectors.py reruns it."""
    rng = random.Random(7)
    runs = []
    for i, fence in enumerate(FENCES):
        for j, motion in enumerate(MOTIONS):
            settings = SETTINGS[(i + j) % len(SETTINGS)]
            mode = MODES[0] if (i + j) % 3 else MODES[1]
            runs.append(run(rng, fence, motion, settings, mode))
    for mode in MODES[2:]:
        runs.append(run(rng, FENCES[0], MOTIONS[0], SETTINGS[0], mode))
    return enc({'note': 'Written by gen_vectors.py from aav_safety geofence.py, hold.py and '
                        'leash.py; do not edit.',
                'footprint': FOOTPRINT,
                'geofence': runs,
                'maths': maths(rng),
                'refusals': refusals(),
                'hold': hold_runs(rng),
                'leash': leash_runs(rng)})


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(build(), separators=(',', ':')) + '\n')
    print(f'wrote {OUT} ({OUT.stat().st_size // 1024} kB)')


if __name__ == '__main__':
    main()
