#!/usr/bin/env python3
"""
The diffusion planner's examples: our own planner's paths on random maps, no simulator.

    pixi run diffusion data                    (200 maps, 60 pairs each, 4 workers)
    pixi run diffusion data --maps 20 --pairs 10 --workers 1    (a quick look)
    pixi run diffusion data --first 800 --maps 400 --uturn 0.6  (more maps, mostly U-turns)

It copies the lattice and Hybrid-A* (the teachers), as the stack runs them: our global
costmap (aav_costmap: the map, inflated) and our planner (aav_planner), with the settings
navigation.launch.py gives them (vehicle.costmap_params, vehicle.planner_params, from the
tuned nav2.yaml). No ROS: each worker runs aav_planner's `teach` example (its maths
crates, `crates/lattice-planner/examples/teach.rs`), one JSON line a request. For each map
it makes the global costmap, picks random start and goal poses on free cells, and asks a
teacher for a path from one to the other. It keeps the global costmap as the planner saw
it, and each pair's start, goal, reversing, teacher and path. Until 17 Nov the teachers
were Nav2's map_server and planner_server (TODO 242).

Maps: the test lot (maps/example_lot) or an empty walled 80 x 60 m lot, each with 3-14
random boxes and walls; every HOLD_OUT-th map is held back for the test (its name says
so). A map whose npz is there already is skipped, so a stopped run goes on.

Out (not in git): models/diffusion/data/<map>.npz, and maps/ beside them (see
models/README.md). Each npz: costmap (int8, OccupancyGrid units, as
/drivers/nav/costmap/global sends it), origin, resolution, start (K, 3), goal (K, 3),
reversing (K), teacher (K), path_points (sum, 3), path_start (K + 1), plan_ms (K). Pairs a
planner found no path for are not kept.
"""

import argparse
import json
import math
import os
import random
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

import make_map  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

OUT = os.path.join(ROOT, 'models', 'diffusion', 'data')
NAV2 = os.path.join(ROOT, 'ros', 'drivers', 'aav_navigation', 'config', 'nav2.yaml')
LATTICE = os.path.join(ROOT, 'ros', 'drivers', 'aav_navigation', 'config',
                       'lattice_ackermann.json')
LOT_SPEC = os.path.join(ROOT, 'ros', 'drivers', 'aav_navigation', 'maps',
                        'example_lot.spec.yaml')
VEHICLE = os.path.join(ROOT, 'ros', 'bringup', 'aav_description', 'config', 'vehicle.yaml')
PLANNER = os.path.join(ROOT, 'ros', 'drivers', 'aav_planner')
# The teacher, built by `cargo build --release --example teach` (main does it once).
TEACH = os.path.join(PLANNER, 'target', 'release', 'examples', 'teach')
HOLD_OUT = 10           # every 10th map is a test map
# The teachers: our planner's search (lattice or hybrid), and whether each may reverse.
# The names are the ones the data has always had (planner_server's plugins, until 17 Nov).
TEACHERS = {'lattice_rev': ('lattice', True), 'lattice_fwd': ('lattice', False),
            'hybrid_rs': ('hybrid', True), 'hybrid_dubin': ('hybrid', False)}
HYBRID_SHARE = 0.3      # of the pairs, Hybrid-A*; the rest the lattice (the fallback)
DISTANCE = (4.0, 50.0)  # m from start to goal: the lot tour's legs, and its replans
# --uturn: a share of the pairs turn back, as the lot tour's "turn back" leg does (15 m
# behind, facing the other way): the goal 4-25 m away, mostly behind or beside, facing
# back within 35 degrees, and the teacher may reverse. The model drew those worst (3 Nov).
UTURN_DISTANCE = (4.0, 25.0)
UTURN_SPREAD = math.radians(35.0)


def map_spec(i, rng):
    """Return a make_map spec: the lot or an empty lot, with random boxes."""
    with open(LOT_SPEC) as f:
        lot = yaml.safe_load(f)
    base = lot if rng.random() < 0.6 else {**lot, 'blocked': [], 'free': lot['free']}
    blocked = list(base['blocked'])
    for _ in range(rng.randint(3, 14)):
        if rng.random() < 0.25:     # a wall: long and thin
            w, h = (rng.uniform(6, 20), rng.uniform(0.5, 1.5))
        else:                       # a box: a car, a skip, a stall
            w, h = rng.uniform(1.0, 6.0), rng.uniform(1.0, 6.0)
        if rng.random() < 0.5:
            w, h = h, w
        x, y = rng.uniform(-8, 68), rng.uniform(-8, 48)
        blocked.append([round(x, 2), round(y, 2), round(x + w, 2), round(y + h, 2)])
    kind = 'test' if i % HOLD_OUT == HOLD_OUT - 1 else 'train'
    return {**base, 'name': f'{kind}_{i:04d}', 'blocked': blocked, 'amcl': None}


def settings():
    """
    Return (teachers, costmap): each teacher's planner block, and the global costmap's.

    As navigation.launch.py makes them with its defaults, but no sensors (radar and lidar
    off: there are none here), each teacher with its own search and reversing.
    """
    from aav_description import vehicle
    v = vehicle.load(VEHICLE)['vehicle']
    with open(NAV2) as f:
        nav2 = yaml.safe_load(f)
    teachers, costmap = {}, None
    for name, (search, reversing) in TEACHERS.items():
        tuned = vehicle.nav2_tuned(nav2, v, reversing, radar=False, lidar=False,
                                   planner='own', lattice_file=LATTICE)
        teachers[name] = vehicle.planner_params(tuned, v, 'own', search, nav2)
        costmap = vehicle.costmap_params(tuned, v)['global']
    return teachers, costmap


def free_pose(costmap, origin, res, rng, near=None, distance=DISTANCE, uturn=False):
    """
    Return a random (x, y, yaw) on a cell well clear of anything, or None.

    uturn (with near): the goal turned back from near's heading (UTURN_*), behind or beside.
    """
    rows, cols = costmap.shape
    for _ in range(200):
        if near is None:
            r, c = rng.randrange(rows), rng.randrange(cols)
            x, y = origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res
        else:
            if uturn:
                d = rng.uniform(*UTURN_DISTANCE)
                a = near[2] + math.pi + rng.uniform(-0.6 * math.pi, 0.6 * math.pi)
            else:
                d, a = rng.uniform(*distance), rng.uniform(-math.pi, math.pi)
            x, y = near[0] + d * math.cos(a), near[1] + d * math.sin(a)
            c, r = int((x - origin[0]) / res), int((y - origin[1]) / res)
            if not (0 <= r < rows and 0 <= c < cols):
                continue
        # Clear 1.5 m round: the car's outline fits whichever way it faces.
        k = int(math.ceil(1.5 / res))
        patch = costmap[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
        if patch.min() >= 0 and patch.max() < 99:
            if uturn:
                yaw = near[2] + math.pi + rng.uniform(-UTURN_SPREAD, UTURN_SPREAD)
                return (x, y, math.atan2(math.sin(yaw), math.cos(yaw)))
            return (x, y, rng.uniform(-math.pi, math.pi))
    return None


def build_teacher():
    """Build the teach example (release), once; exit if cargo can't."""
    done = subprocess.run(['cargo', 'build', '--release', '--quiet', '-p', 'lattice-planner',
                           '--example', 'teach'], cwd=PLANNER)
    if done.returncode != 0 or not os.path.exists(TEACH):
        sys.exit('could not build the teacher (cargo build -p lattice-planner --example teach)')


class Teacher:
    """One teach process: a JSON line in, a JSON line out."""

    def __init__(self, teachers):
        self.proc = subprocess.Popen([TEACH], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)
        got = self.ask({'teachers': teachers})
        if 'error' in got:
            raise RuntimeError(f'the teachers: {got["error"]}')

    def ask(self, request):
        self.proc.stdin.write(json.dumps(request) + '\n')
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError('the teacher stopped')
        return json.loads(line)

    def costmap(self, map_yaml, block):
        """Return the global costmap of a map file: (int8 grid, origin, resolution), or None."""
        got = self.ask({'map': map_yaml, 'costmap': block})
        if 'error' in got:
            print(f'{map_yaml}: {got["error"]}', flush=True)
            return None
        grid = np.array(got['costmap'], dtype=np.int8).reshape(got['height'], got['width'])
        return grid, tuple(got['origin']), got['resolution']

    def plan(self, teacher, start, goal):
        """Return (poses (N, 3), planning ms) or None."""
        got = self.ask({'plan': teacher, 'start': list(start), 'goal': list(goal)})
        if 'error' in got or len(got['poses']) < 2:
            return None
        return np.array(got['poses']), got['ms']

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=10)


def work(args, shard, shards, teachers, block):
    """One worker: its share of the maps, on its own teach process."""
    rng = random.Random(args.seed * 1000 + shard)
    maps_dir = os.path.join(args.out, 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    mine = [i for i in range(args.first, args.first + args.maps) if i % shards == shard]
    teacher = Teacher(teachers)
    kept = tried = 0
    t0 = time.monotonic()
    try:
        for i in mine:
            spec = map_spec(i, random.Random(args.seed * 100003 + i))
            out_path = os.path.join(args.out, spec['name'] + '.npz')
            if os.path.exists(out_path):
                continue
            make_map.write(spec, maps_dir)
            got = teacher.costmap(os.path.join(maps_dir, spec['name'] + '.yaml'), block)
            if got is None:
                continue
            grid, origin, res = got
            rows = []
            for _ in range(args.pairs):
                uturn = rng.random() < args.uturn
                start = free_pose(grid, origin, res, rng)
                goal = start and free_pose(grid, origin, res, rng, near=start, uturn=uturn)
                if goal is None:
                    continue
                reversing = uturn or rng.random() < 0.5
                hybrid = rng.random() < HYBRID_SHARE
                name = [t for t, (search, rev) in TEACHERS.items()
                        if rev == reversing and (search == 'hybrid') == hybrid][0]
                tried += 1
                got = teacher.plan(name, start, goal)
                if got is not None:
                    rows.append((start, goal, reversing, name) + got)
            kept += len(rows)
            if rows:
                lens = [len(r[4]) for r in rows]
                np.savez_compressed(
                    out_path, costmap=grid, origin=np.array(origin), resolution=res,
                    start=np.array([r[0] for r in rows]), goal=np.array([r[1] for r in rows]),
                    reversing=np.array([r[2] for r in rows]),
                    teacher=np.array([r[3] for r in rows]),
                    path_points=np.concatenate([r[4] for r in rows]),
                    path_start=np.concatenate([[0], np.cumsum(lens)]),
                    plan_ms=np.array([r[5] for r in rows]))
            print(f'worker {shard}: {spec["name"]}: {len(rows)} of {args.pairs} kept '
                  f'({kept} of {tried} so far, {time.monotonic() - t0:.0f} s)', flush=True)
    finally:
        teacher.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--maps', type=int, default=200)
    parser.add_argument('--pairs', type=int, default=60, help='start-goal pairs a map')
    parser.add_argument('--workers', type=int, default=4,
                        help='teach processes, each its share of the maps')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--uturn', type=float, default=0.0,
                        help='share of the pairs that turn back (0-1): the goal behind or '
                             'beside, facing back, reversing allowed (the U-turn)')
    parser.add_argument('--first', type=int, default=0,
                        help="the first map's number (another set of maps beside the "
                             'first: --first 800)')
    parser.add_argument('--out', default=OUT)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    t0 = time.monotonic()
    build_teacher()
    teachers, block = settings()
    # Threads are enough: the planning happens in the teach processes.
    threads = [threading.Thread(target=work, args=(args, i, args.workers, teachers, block))
               for i in range(args.workers)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    files = [f for f in os.listdir(args.out) if f.endswith('.npz')]
    pairs = sum(len(np.load(os.path.join(args.out, f))['plan_ms']) for f in files)
    print(f'{len(files)} maps, {pairs} paths in {args.out}, '
          f'{time.monotonic() - t0:.0f} s')


if __name__ == '__main__':
    main()
