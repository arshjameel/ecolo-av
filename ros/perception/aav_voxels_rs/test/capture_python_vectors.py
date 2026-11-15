#!/usr/bin/env python3
"""
Record what aav_voxels' Python cubes.py makes of some clouds, as test vectors for the Rust.

Purpose: the Rust port (crates/voxels) must give the Python's answers. This plays a few
clouds through cubes.py (no ROS) and writes the clouds and the answers to
crates/voxels/tests/python_vectors.txt, which tests/python_vectors.rs reads.

    python3 ros/perception/aav_voxels_rs/test/capture_python_vectors.py
    python3 ros/perception/aav_voxels_rs/test/capture_python_vectors.py --bench /tmp/cloud.txt

--bench writes one whole lidar turn (aav_voxels' test/lot.py: 16 beams x 1800 steps, a
post and a wall) in the same format, and times the Python on it: to_base and cubes, ms per
cloud. `cargo run --release -p voxels --example timing -- /tmp/cloud.txt` times the Rust.

Each cloud's points are float32 in the lidar's frame, as a lidar sends them: the Python's
answers are worked out from those same float32 values.
"""

import argparse
import math
from pathlib import Path
import struct
import sys
import time

import numpy

HERE = Path(__file__).resolve().parent
PERCEPTION = HERE.parents[1]
# cubes.py and lot.py from the source tree: no ROS, no colcon build needed.
sys.path.insert(0, str(PERCEPTION / 'aav_voxels'))
sys.path.insert(0, str(PERCEPTION / 'aav_voxels' / 'test'))
sys.path.insert(0, str(PERCEPTION / 'aav_lidar'))
from aav_lidar.scan import to_base  # noqa: E402, I100
from aav_voxels.cubes import cubes, CubeShape, Ground  # noqa: E402
import lot  # noqa: E402

OUT = HERE.parent / 'crates' / 'voxels' / 'tests' / 'python_vectors.txt'
BODY = (-0.5, 1.9, 0.8)      # aav_lidar scan.body_box of vehicle.yaml's car (test_cubes.py)
SHAPE = CubeShape(body=BODY)
LEVEL = (lot.LIDAR, (0.0, 0.0, 0.0))
POST = (10.0, 10.5, -0.25, 0.25, 1.8)        # sim_check's post, 10 m ahead
WALL = (5.0, 30.0, 8.0, 8.3, 2.5)            # a wall 8 m to the left
WALLS = [(6.0, 6.3, -20, 20, 3.0), (-20, 20, 6.0, 6.3, 3.0), (-20, 20, -6.3, -6.0, 3.0)]
CARS = [(x, x + 4.5, y, y + 1.8, 1.5) for x in range(-20, 25, 6) for y in (4.0, -5.8, 9.0)]
EVERY = 10       # every 10th step of the turn (2 degrees): small files, the same maths


def turn(boxes=(), roll=0.0, pitch=0.0, noise=0.0, seed=0, every=EVERY):
    """Return a lidar turn in base_link (lot.py), every `every`th step, with range noise."""
    points = lot.turn(boxes, roll=roll, pitch=pitch)[::every]
    if noise:
        points = points + numpy.random.default_rng(seed).normal(0.0, noise, points.shape)
    return points


def grid(xs, ys, step, z):
    """Return points on a grid in base_link, z = z(x, y)."""
    gx, gy = numpy.meshgrid(numpy.arange(*xs, step), numpy.arange(*ys, step))
    gx, gy = gx.ravel(), gy.ravel()
    return numpy.column_stack([gx, gy, z(gx, gy)])


def odds_and_ends():
    """Return a few points: the car's own, the sky, NaN, far away, overhead, a post."""
    return numpy.array([
        [1.75, 0.29, 1.55], [0.0, 0.0, 1.5], [-0.4, 0.7, 1.0],
        [numpy.inf, numpy.inf, numpy.inf], [numpy.nan, 0.0, 0.0], [5.0, numpy.nan, 1.0],
        [-numpy.inf, 2.0, 0.5], [40.0, 0.0, 1.0], [6.0, 0.0, 2.5],
        [3.0, 0.0, 0.5], [3.1, 0.05, 0.9], [3.24, -0.1, 0.3], [-3.0, -3.0, 0.0],
    ])


def random_scene(seed=7):
    """Return random points: many cubes shared by several points, ties and all."""
    rng = numpy.random.default_rng(seed)
    points = rng.uniform((-25.0, -25.0, -0.5), (25.0, 25.0, 3.0), (3000, 3))
    # Points on a 0.05 m lattice: some the same distance from base_link, in one cube.
    return numpy.round(points / 0.05) * 0.05


def clouds():
    """Return (name, points in base_link, (lidar xyz, rpy)) for each cloud."""
    tilted = ((0.49, 0.0, 1.52), (0.01, -0.02, 0.03))
    return [
        ('the lot, a post ahead', turn([POST]), LEVEL),
        ('leaning, a post and a wall, noise', turn([POST, WALL], math.radians(3.0),
                                                   math.radians(2.0), 0.01, 1), LEVEL),
        ('walls all round, leaning', turn(WALLS, roll=math.radians(2.0)), LEVEL),
        ('parked cars and a box', turn(CARS + [(8.0, 9.0, -0.5, 0.5, 1.0)],
                                       roll=math.radians(2.0), noise=0.02, seed=2), LEVEL),
        ('a 10 degree ramp', grid((2.0, 20.0), (-10.0, 10.0), 0.5,
                                  lambda x, y: x * math.tan(math.radians(10))), LEVEL),
        ('a crowned road', grid((2.0, 20.0), (-12.0, 12.0), 0.5,
                                lambda x, y: -0.003 * y ** 2), LEVEL),
        ('odds and ends', odds_and_ends(), LEVEL),
        ('empty', numpy.zeros((0, 3)), LEVEL),
        ('random, a tilted mount', random_scene(), tilted),
    ]


def from_base(points, xyz, rpy):
    """Return float32 points in the lidar's frame, from base_link (the mount's inverse)."""
    from aav_lidar.scan import rotation
    with numpy.errstate(invalid='ignore'):
        lidar = (points - numpy.asarray(xyz)) @ rotation(rpy)
    return lidar.astype(numpy.float32)


def bits(v):
    """Return a float32's bits as 8 hex digits."""
    return struct.unpack('<I', struct.pack('<f', v))[0].to_bytes(4, 'big').hex()


def line(values):
    return ' '.join(repr(float(v)) for v in values)


def record(name, base, mount, out):
    """Write one cloud and what cubes.py makes of it; return the cubes."""
    xyz, rpy = mount
    lidar = from_base(base, xyz, rpy)
    points = to_base(lidar.astype(float).reshape(-1, 3), xyz, rpy)
    found = cubes(points, SHAPE)
    flat = cubes(points, SHAPE, road=Ground())
    road = found.road
    out.append(f'cloud {name}')
    out.append(f'mount {line(xyz)} {line(rpy)}')
    out.append(f'points {len(lidar)}')
    out.extend(' '.join(bits(v) for v in p) for p in lidar)
    out.append(f'road {int(road.fitted)} {road.candidates} {line((road.a, road.b, road.c))}')
    out.append(f'counted {found.points}')
    # Each obstacle cube's point, as its number in the cloud (it is one of the cloud's).
    index = {tuple(p): i for i, p in reversed(list(enumerate(points.tolist())))}
    for label, cloud in (('obstacles', found.obstacles), ('flat_obstacles', flat.obstacles)):
        out.append(f'{label} {len(cloud)}')
        out.extend(str(index[tuple(p)]) for p in cloud.tolist())
    # Each ground square's centre, x and y: its z is the road's height there.
    out.append(f'ground {len(found.ground)}')
    out.extend(line(p) for p in found.ground[:, :2])
    return found


def bench(path):
    """Write one whole lidar turn for the Rust's timing, and time the Python on it."""
    base = lot.turn([POST, WALL])
    out = ['# One whole lidar turn (lot.py: 16 x 1800, a post and a wall), for timing.']
    record('a whole turn', base, LEVEL, out)
    Path(path).write_text('\n'.join(out) + '\n')
    lidar = from_base(base, *LEVEL).astype(float)
    runs = 50
    cubes(to_base(lidar, *LEVEL), SHAPE)            # warm up
    start = time.perf_counter()
    for _ in range(runs):
        cubes(to_base(lidar, *LEVEL), SHAPE)
    ms = (time.perf_counter() - start) * 1000.0 / runs
    print(f'{path}: {len(lidar)} points. Python (numpy {numpy.__version__}): to_base + cubes '
          f'{ms:.2f} ms per cloud, mean of {runs}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('--bench', metavar='FILE',
                        help='write one whole turn here and time the Python on it, instead')
    args = parser.parse_args()
    if args.bench:
        bench(args.bench)
        return
    out = ['# aav_voxels cubes.py on these clouds: test/capture_python_vectors.py made this.',
           f'# numpy {numpy.__version__}. Points: float32 bits, in the lidar frame.',
           f'shape {line((SHAPE.size, SHAPE.min_height, SHAPE.max_height, SHAPE.range_max))}'
           f' {line(BODY)}']
    for name, base, mount in clouds():
        found = record(name, base, mount, out)
        print(f'{name}: fitted {found.road.fitted}, {len(found.obstacles)} cubes, '
              f'{len(found.ground)} ground squares, {found.points} points')
    OUT.write_text('\n'.join(out) + '\n')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
