#!/usr/bin/env python3
"""
Write lidar-scan's agreement vectors from aav_lidar's Python scan.py (no ROS).

The Rust (crates/lidar-scan, tests/vectors.rs) must give the same ranges as scan.py's
flatten() on the same float32 points, after the node's cast to float32: every range the
same float32, or (a point whose angle lands within 1e-9 of half way between two
directions) one direction over.

The clouds:
  - aav_voxels' made-up lidar turns (test/lot.py): a post, a wall, parked cars, a lean,
    noise; a turned lidar; a scan of only part of the circle;
  - with --recorded FILE (tools: capture_cloud.py writes it): clouds recorded in our
    simulator, one per line, every 3rd point kept.

Points are float32 in the lidar's frame, as a lidar sends them. Infinity and NaN are
written as the strings 'inf', '-inf' and 'nan'.

  pixi run python ros/perception/aav_lidar_rs/gen_vectors.py [--recorded /tmp/clouds.json]
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy

HERE = Path(__file__).resolve().parent
PERCEPTION = HERE.parent
sys.path[:0] = [str(PERCEPTION / 'aav_lidar'), str(PERCEPTION / 'aav_voxels' / 'test')]
from aav_lidar.scan import flatten, ScanShape, to_base  # noqa: E402, I100
import lot  # noqa: E402

OUT = HERE / 'crates' / 'lidar-scan' / 'tests' / 'vectors.json'
BODY = (-0.5, 1.9, 0.8)
POST = (10.0, 10.5, -0.25, 0.25, 1.8)
WALL = (5.0, 30.0, 8.0, 8.3, 2.5)
CARS = [(x, x + 4.5, y, y + 1.8, 1.5) for x in range(-20, 25, 6) for y in (4.0, -5.8, 9.0)]


def enc(v):
    """Return v with every non-finite float as a string, for strict JSON."""
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def case(name, lidar_points, xyz, rpy, shape):
    """Return one vector: float32 points in the lidar's frame and scan.py's ranges."""
    pts = numpy.asarray(lidar_points, dtype=numpy.float32).reshape(-1, 3)
    ranges = flatten(to_base(pts.astype(float), xyz, rpy), shape).astype(numpy.float32)
    return {'name': name, 'xyz': list(xyz), 'rpy': list(rpy),
            'shape': [shape.min_height, shape.max_height, shape.angle_min, shape.angle_max,
                      shape.angle_increment, shape.range_min, shape.range_max,
                      *shape.body],
            'points': [float(v) for v in pts.ravel()],
            'ranges': [float(r) for r in ranges]}


def made_up():
    """Return the made-up clouds' vectors."""
    level = (list(lot.LIDAR), [0.0, 0.0, 0.0])
    shape = ScanShape(body=BODY)
    rng = numpy.random.default_rng(7)
    out = []
    for name, boxes, roll in [('a post ahead', [POST], 0.0),
                              ('a post and a wall, leaning 2 degrees', [POST, WALL],
                               math.radians(2.0)),
                              ('a car park', CARS, 0.0)]:
        base = lot.turn(boxes, roll=roll)[::4]
        out.append(case(name, base - numpy.asarray(lot.LIDAR), *level, shape))
    base = lot.turn(CARS + [POST])[::4]
    noisy = base - numpy.asarray(lot.LIDAR) + rng.normal(0.0, 0.03, base.shape)
    # Rays that hit nothing: NaN and inf, as a driver sends them.
    noisy[::97] = numpy.nan
    noisy[::89] = numpy.inf
    out.append(case('noise, lost rays', noisy, *level, shape))
    # Random points all round, every height, near and far: the filters' edges.
    pts = rng.uniform([-35, -35, -2.5], [35, 35, 1.0], (6000, 3))
    out.append(case('random points', pts, *level, shape))
    # A turned and tilted lidar.
    turned = ([0.5, 0.2, 1.6], [0.02, -0.05, math.pi / 2])
    out.append(case('a turned lidar', pts, *turned, shape))
    # Only ahead, 1 degree apart, 1 to 12 m.
    part = ScanShape(angle_min=-math.pi / 3, angle_max=math.pi / 3,
                     angle_increment=math.radians(1.0), range_min=1.0, range_max=12.0,
                     min_height=0.2, max_height=1.5, body=BODY)
    out.append(case('part of the circle', pts, *level, part))
    return out


def recorded(path):
    """Return the vectors of clouds recorded in the simulator (capture_cloud.py's file)."""
    with open(path) as f:
        clouds = json.load(f)
    shape = ScanShape(body=tuple(clouds['body']))
    return [case(f'recorded {i}', numpy.asarray(c, dtype=numpy.float32).reshape(-1, 3)[::3],
                 clouds['xyz'], clouds['rpy'], shape)
            for i, c in enumerate(clouds['clouds'])]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--recorded', help='clouds recorded in the simulator (JSON)')
    args = parser.parse_args()
    cases = made_up()
    if args.recorded:
        cases += recorded(args.recorded)
    elif OUT.exists():
        # The recorded clouds already in the file, their ranges worked out again.
        for c in json.loads(OUT.read_text())['cases']:
            if c['name'].startswith('recorded'):
                s = c['shape']
                shape = ScanShape(*s[:7], body=tuple(s[7:]))
                cases.append(case(c['name'], c['points'], c['xyz'], c['rpy'], shape))
    OUT.write_text(json.dumps(enc({'numpy': numpy.__version__, 'cases': cases})))
    print(f'{OUT}: {len(cases)} clouds, {sum(len(c["ranges"]) for c in cases)} ranges')


if __name__ == '__main__':
    main()
