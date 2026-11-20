#!/usr/bin/env python3
"""
Write gps-heading-core's agreement vectors from aav_gps_heading's course.py.

Random velocities, wheel speeds, turn rates and antenna places, forward and reversing;
slow, disagreeing and NaN inputs; every settings' edge. The Rust (crates/gps-heading-core,
tests/vectors.rs) must give the same heading and variance to the bit, or None alike.

Writes ros/localization/aav_gps_heading_rs/crates/gps-heading-core/tests/vectors.json:

  pixi run python ros/localization/aav_gps_heading_rs/gen_vectors.py
"""

import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / 'aav_gps_heading')]

from aav_gps_heading.course import (CourseSettings, heading, quaternion_z,  # noqa: E402
                                    wrap)

OUT = HERE / 'crates' / 'gps-heading-core' / 'tests' / 'vectors.json'


def enc(v):
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def build():
    """Return the vectors: aav_gps_heading's test_gps_heading_rs_vectors.py reruns it."""
    rng = random.Random(13)
    settings = [CourseSettings(),
                CourseSettings(min_speed=0.01, max_speed_gap=0.05, floor=0.0),
                CourseSettings(min_speed=1.0, max_speed_gap=2.0, floor=math.radians(5.5))]
    cases = []
    for _ in range(4000):
        s = rng.choice(settings)
        yaw = rng.uniform(-math.pi, math.pi)
        speed = rng.choice([rng.uniform(-3, 3), rng.uniform(-0.4, 0.4), 0.0, 1.0])
        turn = rng.choice([0.0, rng.uniform(-0.8, 0.8)])
        antenna = rng.choice([(0.0, 0.0), (1.2, 0.0), (rng.uniform(-1, 2), rng.uniform(-1, 1))])
        ground = abs(speed) + rng.choice([0.0, rng.gauss(0, 0.05), rng.uniform(-0.6, 0.6)])
        direction = yaw + (0.0 if speed >= 0 else math.pi) + rng.gauss(0, 0.03)
        east, north = ground * math.cos(direction), ground * math.sin(direction)
        var_e, var_n = rng.choice([0.0, 0.0025, rng.uniform(0, 0.1), -0.01]), rng.uniform(0, 0.1)
        if rng.random() < 0.02:
            east = rng.choice([float('nan'), math.inf])
        if rng.random() < 0.01:
            turn = float('nan')
        args = [east, north, var_e, var_n, speed, turn, list(antenna)]
        cases.append({'args': args,
                      'settings': [s.min_speed, s.max_speed_gap, s.floor],
                      'out': heading(east, north, var_e, var_n, speed, turn, antenna, s)})
    angles = [rng.uniform(-20, 20) for _ in range(500)] + [
        -math.pi, math.pi, 0.0, -0.0, 3 * math.pi, -3 * math.pi, 2 * math.pi, 1e-300]
    return enc({'note': 'Written by gen_vectors.py from aav_gps_heading course.py; do not '
                        'edit.',
                'default_floor': CourseSettings().floor,
                'floor_deg_default': math.degrees(CourseSettings().floor),
                'heading': cases,
                'wrap': [[a, wrap(a)] for a in angles],
                'quaternion': [[a, list(quaternion_z(a))] for a in angles]})


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(build(), separators=(',', ':')) + '\n')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
