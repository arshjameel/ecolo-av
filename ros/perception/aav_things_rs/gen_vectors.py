#!/usr/bin/env python3
"""
Write things-core's agreement vectors from aav_things' Python modules (no ROS).

The Rust (crates/things-core, tests/vectors.rs) must give the same answers:

  - runs: cube clouds in base_link, one a lidar turn (10 Hz), each with the car's pose
    in odom and in map; the node's pipeline on each (the saved map's walls dropped,
    groups.py, to odom, tracker.py, the radar's clusters), then the list it would publish
    half a cloud later (expire, at). Every group and every listed thing: the same ids,
    seen counts, coasting and radar flags; places, speeds and outlines within 1e-9 m;
  - the names: random things and boxes through naming.py's pinhole and IoU;
  - the background on its own: random cubes, poses and slack on a made-up map.

The runs: made-up ones (a walker crossing, a car passing, a row of thin posts, parked
boxes, with the radar), and with --recorded FILE the cubes recorded in our simulator
(test/capture_cubes.py writes the file).

Infinity and NaN are written as the strings 'inf', '-inf' and 'nan'.

  pixi run python ros/perception/aav_things_rs/gen_vectors.py [--recorded /tmp/cubes.json]
"""

import argparse
import json
import math
from pathlib import Path
import random
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
PERCEPTION = HERE.parent
sys.path[:0] = [str(PERCEPTION / 'aav_things'), str(PERCEPTION / 'aav_detector')]
from aav_detector.ground import Lens, Mount, rotation  # noqa: E402, I100
from aav_things.background import Background, new_only  # noqa: E402
from aav_things.groups import group, GroupShape, to_frame  # noqa: E402
from aav_things.naming import best_label, Camera, Namer, project, Seen, thing_box  # noqa: E402
from aav_things.tracker import (ground_radial_speed, RadarCluster, Tracker,  # noqa: E402
                                TrackerShape)

OUT = HERE / 'crates' / 'things-core' / 'tests' / 'vectors.json'
SIZE = 0.25


def enc(v):
    """Return v with every non-finite float as a string, for strict JSON."""
    if isinstance(v, (float, np.floating)):
        v = float(v)
        if not math.isfinite(v):
            return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
        return v
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (list, tuple, np.ndarray)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def f32s(values):
    """Return float32 values as the shortest floats that read back as them (small JSON)."""
    return [float(str(v)) for v in np.asarray(values, dtype=np.float32).ravel()]


def clusters_of(rows, pose, car_velocity):
    """Return things_node.py use_clusters' RadarClusters (its maths, no ROS)."""
    points = np.asarray(rows, dtype=float).reshape(-1, 4)
    points = points[~np.isnan(points).any(axis=1)]
    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for number in np.unique(points[:, 3]):
        p = points[points[:, 3] == number]
        rx, ry = p[:, 0].mean(), p[:, 1].mean()
        distance = math.hypot(rx, ry)
        if distance < 0.1:
            continue
        ray = ((c * rx - s * ry) / distance, (s * rx + c * ry) / distance)
        out.append(RadarCluster(
            x=x0 + c * rx - s * ry, y=y0 + s * rx + c * ry, ray_x=ray[0], ray_y=ray[1],
            speed=ground_radial_speed(float(p[:, 2].mean()), ray, car_velocity)))
    return out


def grid_of(spec):
    """Return a Background from a run's map: width, height, resolution, origin, walls."""
    data = np.zeros(spec['width'] * spec['height'], dtype=np.int8)
    data[np.asarray(spec['walls'], dtype=int)] = 100
    return Background.from_grid(data, spec['width'], spec['height'], spec['resolution'],
                                spec['origin'][0], spec['origin'][1], spec['slack'])


def play(run):
    """Return what the Python makes of one run: each frame's groups and published list."""
    shape = TrackerShape(**run.get('tracker', {}))
    tracker = Tracker(shape)
    bg = grid_of(run['map']) if run.get('map') else None
    below = run.get('below')
    out = []
    for frame in run['frames']:
        cubes = np.asarray(frame['cubes'], dtype=np.float32).astype(float).reshape(-1, 3)
        if bg is not None and frame.get('map_pose') is not None and len(cubes):
            cubes = new_only(cubes, frame['map_pose'], bg, below)
        groups = to_frame(group(cubes, GroupShape()), *frame['pose'])
        tracker.step(frame['t'], groups, tuple(frame['pose'][:2]))
        if frame.get('radar'):
            r = frame['radar']
            tracker.radar(r['t'], clusters_of(r['rows'], r['pose'], tuple(r['car_velocity'])))
        now = frame['publish']
        tracker.expire(now, tuple(frame['pose'][:2]))
        listed = [{'id': t.id, 'x': t.x.tolist(), 'length': t.length, 'width': t.width,
                   'height': t.height, 'sigma': t.position_sigma,
                   'confidence': t.confidence(now, shape), 'seen': t.seen,
                   'radar': t.radar, 'coasting': t.coasting, 'last_seen': t.last_seen}
                  for t in tracker.at(now)]
        out.append({'kept': len(cubes),
                    'groups': [[g.x, g.y, g.length, g.width, g.height, g.cubes]
                               for g in groups],
                    'things': listed})
    return out


def box_cubes(x, y, length, width, height, rng, miss=0.0):
    """Return the cube points the voxels node sends for an upright box (base_link)."""
    out = []
    for a in np.arange(x - length / 2 + SIZE / 2, x + length / 2, SIZE):
        for b in np.arange(y - width / 2 + SIZE / 2, y + width / 2, SIZE):
            for c in np.arange(0.3 + SIZE / 2, height, SIZE):
                if rng.random() >= miss:
                    out.append((round(a + rng.uniform(-0.1, 0.1), 3),
                                round(b + rng.uniform(-0.1, 0.1), 3), round(c, 3)))
    return out


def made_up_run(seed, radar):
    """Return a made-up run: the car drives past a walker, a car, posts and parked boxes."""
    rng = random.Random(seed)
    frames = []
    # The saved map: 60 x 40 m at 0.25 m from (-20, -20), a wall along y = 12 (x 0-30).
    width, height, res = 240, 160, 0.25
    walls = [int((12.0 + 20.0) / res) * width + i for i in range(int(20 / res), int(50 / res))]
    for k in range(80):
        t = 100.0 + 0.1 * k
        car = (-5.0 + 1.5 * 0.1 * k, 0.3 * math.sin(0.05 * k), 0.05 * math.sin(0.03 * k))
        # map -> odom: a small offset, as the EKFs keep them.
        map_pose = (car[0] + 0.4, car[1] - 0.2, car[2] + 0.01)
        world = [(12.0, -6.0 + 1.3 * 0.1 * k, 0.5, 0.5, 1.7),        # a walker crossing
                 (-25.0 + 6.0 * 0.1 * k, 6.0, 4.4, 1.8, 1.5),          # a car passing
                 (8.0, -3.5, 1.0, 1.0, 1.0), (20.0, 3.5, 2.0, 1.2, 0.8)]  # parked
        world += [(4.0 + 1.5 * i, 4.5, 0.25, 0.25, 1.2) for i in range(8)]    # thin posts
        world += [(10.0 + 0.5 * i, 12.1, 0.25, 0.25, 0.6) for i in range(30)]  # a kerb on the wall
        cubes = []
        c, s = math.cos(car[2]), math.sin(car[2])
        for (wx, wy, length, wid, h) in world:
            dx, dy = wx - car[0], wy - car[1]
            bx, by = c * dx + s * dy, -s * dx + c * dy
            if math.hypot(bx, by) > 25.0 or rng.random() < 0.15:
                continue
            cubes += box_cubes(bx, by, length, wid, h, rng, miss=0.2)
        frame = {'t': t, 'publish': t + 0.05, 'pose': list(car), 'map_pose': list(map_pose),
                 'cubes': f32s(cubes)}
        if radar:
            # The radar sees the walker and the car: a few points each, with their
            # radial speeds relative to the car (1.5 m/s along x).
            rows = []
            for n, (wx, wy, vx, vy) in enumerate([(12.0, -6.0 + 1.3 * 0.1 * k, 0.0, 1.3),
                                                  (-25.0 + 0.6 * k, 6.0, 6.0, 0.0)]):
                dx, dy = wx - car[0], wy - car[1]
                d = math.hypot(dx, dy)
                ray = (dx / d, dy / d)
                radial = ray[0] * (vx - 1.5) + ray[1] * vy
                for _ in range(rng.randint(1, 9)):
                    px, py = dx + rng.gauss(0, 0.2), dy + rng.gauss(0, 0.2)
                    rows.append([c * px + s * py, -s * px + c * py,
                                 radial + rng.gauss(0, 0.1), float(n + 3)])
            frame['radar'] = {'t': t + 0.02, 'pose': list(car), 'car_velocity': [1.5, 0.0],
                              'rows': rows}
        frames.append(frame)
    return {'name': f'made up {seed}' + (', radar' if radar else ''), 'below': 0.7,
            'map': {'width': width, 'height': height, 'resolution': res,
                    'origin': [-20.0, -20.0], 'slack': 0.25, 'walls': walls},
            'frames': frames}


def posts_run(seed):
    """Return test_things.py's row of thin posts as cubes (TODO 222), no map."""
    rng = random.Random(seed)
    posts = [(5.0 + 1.5 * i, 4.0) for i in range(12)]
    frames = []
    for k in range(100):
        t, car = 0.1 * k + 1.0, (0.2 * k - 5.0, 0.0, 0.0)
        cubes = []
        for px, py in posts:
            if math.hypot(px - car[0], py - car[1]) <= 25 and rng.random() > 0.35:
                cubes += box_cubes(px - car[0] + rng.uniform(-0.15, 0.15),
                                   py + rng.uniform(-0.15, 0.15), 0.25, 0.5, 1.0, rng)
        frames.append({'t': t, 'publish': t + 0.05, 'pose': list(car),
                       'cubes': f32s(cubes)})
    return {'name': f'thin posts {seed}', 'frames': frames}


def names(rng):
    """Return random things and detector boxes through naming.py."""
    cases = []
    for i in range(300):
        q = [rng.gauss(0, 1) for _ in range(4)]
        n = math.sqrt(sum(v * v for v in q))
        optical = rng.random() < 0.5
        mount = Mount((rng.uniform(-1, 3), rng.uniform(-1, 1), rng.uniform(0.5, 2.0)),
                      rotation(*[v / n for v in q]) if i % 3 else
                      (((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)) if optical else
                       ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))),
                      optical)
        lens = Lens(rng.uniform(300, 800), rng.uniform(300, 800), 320.0, 240.0, 640, 480)
        thing = (rng.uniform(1, 20), rng.uniform(-5, 5), rng.uniform(0.2, 3), rng.uniform(0.2, 3),
                 rng.uniform(0.5, 2.5))
        box = thing_box(lens, mount, *thing)
        seen = []
        for j in range(rng.randint(0, 4)):
            u, v = rng.uniform(0, 640), rng.uniform(0, 480)
            if box is not None and rng.random() < 0.5:
                u, v = box[0] + rng.uniform(-30, 30), box[1] + rng.uniform(-30, 30)
            seen.append(Seen(rng.choice(['person', 'car', 'stop sign']), 0.9,
                             (u, v, u + rng.uniform(5, 200), v + rng.uniform(5, 200))))
        namer = Namer()
        namer.cameras['cam'] = Camera(lens, mount, seen, 10.0)
        label = namer.name(i, *thing, 10.1)
        cases.append({'mount': [list(mount.position), [list(r) for r in mount.rotation],
                                optical],
                      'lens': [lens.fx, lens.fy, lens.cx, lens.cy],
                      'thing': list(thing),
                      'point': project(lens, mount, (thing[0], thing[1], thing[4])),
                      'box': box,
                      'seen': [[s.label, *s.box] for s in seen],
                      'strict': best_label(box, seen, 0.3) if box else None,
                      'label': label})
    return cases


def background(rng):
    """Return random cubes, poses and below on a made-up map, and what new_only keeps."""
    width, height, res = 100, 80, 0.25
    walls = sorted({rng.randrange(width * height) for _ in range(400)})
    spec = {'width': width, 'height': height, 'resolution': res, 'origin': [-12.5, -10.0],
            'slack': 0.3, 'walls': walls}
    bg = grid_of(spec)
    cases = []
    for _ in range(20):
        pose = (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi))
        cubes = np.asarray([[rng.uniform(-15, 15), rng.uniform(-15, 15), rng.uniform(0.3, 2)]
                            for _ in range(100)]).round(3)
        below = rng.choice([None, 0.7])
        kept = new_only(cubes, pose, bg, below)
        cases.append({'pose': list(pose), 'below': below, 'cubes': cubes.ravel().tolist(),
                      'kept': kept.ravel().tolist()})
    return {'map': spec, 'cases': cases}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--recorded', help='cubes recorded in the simulator (JSON)')
    args = parser.parse_args()
    runs = [made_up_run(1, radar=True), posts_run(3)]
    if args.recorded:
        with open(args.recorded) as f:
            runs += json.load(f)['runs']
    elif OUT.exists():
        runs += [r for r in json.loads(OUT.read_text())['runs']
                 if r['name'].startswith('recorded')]
    for run in runs:
        run['expected'] = play(run)
    rng = random.Random(11)
    data = {'numpy': np.__version__, 'runs': runs, 'names': names(rng),
            'background': background(rng)}
    OUT.write_text(json.dumps(enc(data)))
    things = sum(len(f['things']) for r in runs for f in r['expected'])
    print(f'{OUT}: {len(runs)} runs, {sum(len(r["frames"]) for r in runs)} clouds, '
          f'{things} things listed; {len(data["names"])} names, '
          f'{len(data["background"]["cases"])} background cases '
          f'({OUT.stat().st_size / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
