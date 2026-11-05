#!/usr/bin/env python3
"""
The diffusion planner's examples: our own planners' paths on random maps, no Gazebo.

    pixi run diffusion data                    (200 maps, 60 pairs each, 4 workers)
    pixi run diffusion data --maps 20 --pairs 10 --workers 1    (a quick look)

It copies the lattice and Hybrid-A* (the teachers): for each map it starts only Nav2's
map_server and planner_server (the global costmap: the map, inflated), loads the map, picks
random start and goal poses on free cells, and asks each planner for a path from one to
the other (ComputePathToPose, use_start). It keeps the global costmap as the planner saw
it, and each pair's start, goal, reversing, teacher and path.

Maps: the test lot (maps/example_lot) or an empty walled 80 x 60 m lot, each with 3-14
random boxes and walls; every HOLD_OUT-th map is held back for the test (its name says
so). Each worker has its own ROS domain (--domain, then +1 a worker) on this computer only.

Out (not in git): models/diffusion/data/<map>.npz, and maps/ beside them (see
models/README.md). Each npz: costmap (int8, Nav2's OccupancyGrid units), origin,
resolution, start (K, 3), goal (K, 3), reversing (K), teacher (K), path_points (sum, 3),
path_start (K + 1), plan_ms (K). Pairs a planner found no path for are not kept.
"""

import argparse
import math
import os
import random
import signal
import subprocess
import sys
import tempfile
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
HOLD_OUT = 10           # every 10th map is a test map
# The teachers: planner_server's plugins (planner_id), and whether each may reverse.
TEACHERS = {'lattice_rev': True, 'lattice_fwd': False, 'hybrid_rs': True,
            'hybrid_dubin': False}
HYBRID_SHARE = 0.3      # of the pairs, Hybrid-A*; the rest the lattice (the fallback)
DISTANCE = (4.0, 50.0)  # m from start to goal: the lot tour's legs, and its replans
PLAN_TIME = 8.0         # s for an answer
MAP_TIME = 240.0        # s for a map's pairs; longer and the worker starts again (STUCK)
STUCK = 3               # a worker's exit code when it gave up on a stuck map


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


def planner_params(v):
    """Return the parameter file's dict: map_server, planner_server and its costmap."""
    with open(NAV2) as f:
        nav2 = yaml.safe_load(f)
    from aav_description import vehicle
    plans = nav2['planner_server']['ros__parameters']
    radius = round(vehicle.plan_turning_radius(v), 3)
    out = {k: plans[k] for k in ('expected_planner_frequency', 'costmap_update_timeout')}
    out['planner_plugins'] = list(TEACHERS)
    for name, rev in TEACHERS.items():
        if name.startswith('lattice'):
            block = dict(plans['Lattice'], lattice_filepath=LATTICE,
                         allow_reverse_expansion=rev)
        else:
            block = dict(plans['GridBased'], minimum_turning_radius=radius,
                         motion_model_for_search='REEDS_SHEPP' if rev else 'DUBIN')
        out[name] = block
    costmap = dict(nav2['global_costmap']['global_costmap']['ros__parameters'])
    costmap['footprint'] = vehicle.footprint(v)
    return {'planner_server': {'ros__parameters': out},
            'global_costmap': {'global_costmap': {'ros__parameters': costmap}}}


def free_pose(costmap, origin, res, rng, near=None, distance=DISTANCE):
    """Return a random (x, y, yaw) on a cell well clear of anything, or None."""
    rows, cols = costmap.shape
    for _ in range(200):
        if near is None:
            r, c = rng.randrange(rows), rng.randrange(cols)
            x, y = origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res
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
            return (x, y, rng.uniform(-math.pi, math.pi))
    return None


class Asker:
    """The ROS side: load a map, read the costmap, ask a planner."""

    def __init__(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from nav2_msgs.action import ComputePathToPose
        from nav2_msgs.srv import LoadMap
        from nav_msgs.msg import OccupancyGrid
        self.rclpy, self.ComputePathToPose, self.LoadMap = rclpy, ComputePathToPose, LoadMap
        rclpy.init()
        self.node = Node('diffusion_data')
        self.plan = ActionClient(self.node, ComputePathToPose, 'compute_path_to_pose')
        self.load = self.node.create_client(LoadMap, 'map_server/load_map')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap = None
        self.node.create_subscription(OccupancyGrid, 'global_costmap/costmap',
                                      self.on_costmap, qos)

    def on_costmap(self, msg):
        self.costmap = msg

    def wait(self, future, seconds):
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=seconds)
        return future.result() if future.done() else None

    def ready(self, seconds=90.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if (self.plan.wait_for_server(timeout_sec=1.0) and
                    self.load.wait_for_service(timeout_sec=1.0)):
                return True
        return False

    def load_map(self, yaml_path):
        """Load a map; return its costmap (int8 grid, origin, resolution) once it shows."""
        req = self.LoadMap.Request()
        req.map_url = yaml_path
        if self.wait(self.load.call_async(req), 10.0) is None:
            return None
        asked = self.node.get_clock().now().nanoseconds * 1e-9
        end = time.monotonic() + 15.0
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.2)
            m = self.costmap
            # The costmap updates once a second: wait for one made after the map came.
            if m is not None and (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 >
                                  asked + 1.2):
                grid = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
                return (grid, (m.info.origin.position.x, m.info.origin.position.y),
                        m.info.resolution)
        return None

    def ask(self, teacher, start, goal):
        """Return (poses (N, 3), planning ms) or None."""
        g = self.ComputePathToPose.Goal()
        for pose, (x, y, yaw) in ((g.start, start), (g.goal, goal)):
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z, pose.pose.orientation.w = (math.sin(yaw / 2),
                                                                math.cos(yaw / 2))
        g.planner_id, g.use_start = teacher, True
        handle = self.wait(self.plan.send_goal_async(g), 5.0)
        if handle is None or not handle.accepted:
            return None
        done = self.wait(handle.get_result_async(), PLAN_TIME)
        if done is None:
            self.wait(handle.cancel_goal_async(), 2.0)
            return None
        r = done.result
        if r.error_code != 0 or len(r.path.poses) < 2:
            return None
        poses = np.array([(p.pose.position.x, p.pose.position.y,
                           2.0 * math.atan2(p.pose.orientation.z, p.pose.orientation.w))
                          for p in r.path.poses])
        return poses, r.planning_time.sec * 1e3 + r.planning_time.nanosec * 1e-6

    def close(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def start_nodes(first_map, params_path, log):
    """Start map_server, planner_server, a static TF and their lifecycle manager."""
    cmds = [
        ['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map',
         '--child-frame-id', 'base_link'],
        ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '--params-file',
         params_path, '-p', f'yaml_filename:={first_map}'],
        ['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file',
         params_path],
        ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
         '-r', '__node:=lifecycle_manager_data', '-p', 'autostart:=true',
         '-p', 'node_names:=[map_server, planner_server]'],
    ]
    return [subprocess.Popen(c, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True) for c in cmds]


def stop_nodes(procs):
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError):   # gone already
            pass
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


def work(args, shard, shards):
    """One worker: its share of the maps, on its own ROS domain."""
    os.environ['ROS_DOMAIN_ID'] = str(args.domain + shard)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    from aav_description import vehicle
    v = vehicle.load()['vehicle']
    rng = random.Random(args.seed * 1000 + shard)
    maps_dir = os.path.join(args.out, 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    mine = [i for i in range(args.maps) if i % shards == shard]
    specs = []
    for i in mine:
        spec = map_spec(i, random.Random(args.seed * 100003 + i))
        make_map.write(spec, maps_dir)
        specs.append(spec)
    params = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix='diffusion_data_',
                                         delete=False)
    with params:
        yaml.safe_dump(planner_params(v), params)
    log = open(os.path.join(args.out, f'nodes-{shard}.log'), 'w')
    procs = start_nodes(os.path.join(maps_dir, specs[0]['name'] + '.yaml'), params.name, log)
    asker = Asker()
    kept = tried = 0
    t0 = time.monotonic()
    try:
        if not asker.ready():
            print(f'worker {shard}: the planner never came up (see nodes-{shard}.log)')
            return
        for spec in specs:
            # rclpy has hung in rcl_wait now and then (2 Nov, Cyclone on a busy Mac): give
            # each map a time limit; past it, stop the nodes and start this worker again.
            watchdog = threading.Timer(MAP_TIME, lambda: (stop_nodes(procs), os._exit(STUCK)))
            watchdog.daemon = True
            watchdog.start()
            out_path = os.path.join(args.out, spec['name'] + '.npz')
            if os.path.exists(out_path):
                continue
            got = asker.load_map(os.path.join(maps_dir, spec['name'] + '.yaml'))
            if got is None:
                # planner_server's costmap stops updating now and then after a long
                # search: start the nodes again and try the map once more.
                print(f'worker {shard}: {spec["name"]}: no costmap, restarting the nodes',
                      flush=True)
                stop_nodes(procs)
                procs = start_nodes(os.path.join(maps_dir, spec['name'] + '.yaml'),
                                    params.name, log)
                got = asker.ready() and asker.load_map(
                    os.path.join(maps_dir, spec['name'] + '.yaml'))
                if not got:
                    print(f'worker {shard}: {spec["name"]}: still none, skipped', flush=True)
                    continue
            grid, origin, res = got
            rows = []
            for _ in range(args.pairs):
                start = free_pose(grid, origin, res, rng)
                goal = start and free_pose(grid, origin, res, rng, near=start)
                if goal is None:
                    continue
                reversing = rng.random() < 0.5
                hybrid = rng.random() < HYBRID_SHARE
                teacher = [t for t, rev in TEACHERS.items()
                           if rev == reversing and t.startswith('hybrid') == hybrid][0]
                tried += 1
                got = asker.ask(teacher, start, goal)
                if got is not None:
                    rows.append((start, goal, reversing, teacher) + got)
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
            watchdog.cancel()
            print(f'worker {shard}: {spec["name"]}: {len(rows)} of {args.pairs} kept '
                  f'({kept} of {tried} so far, {time.monotonic() - t0:.0f} s)', flush=True)
    finally:
        asker.close()
        stop_nodes(procs)
        os.remove(params.name)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--maps', type=int, default=200)
    parser.add_argument('--pairs', type=int, default=60, help='start-goal pairs a map')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--domain', type=int, default=200,
                        help="the first worker's ROS domain (each next one +1; at most 232)")
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--out', default=OUT)
    parser.add_argument('--shard', type=int, default=None, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.domain + args.workers - 1 > 232:
        sys.exit('--domain: at most 232 for the last worker (Fast DDS breaks above)')
    os.makedirs(args.out, exist_ok=True)
    if args.shard is not None:
        work(args, args.shard, args.workers)
        return
    t0 = time.monotonic()

    def start(i):
        return subprocess.Popen([sys.executable, __file__, *sys.argv[1:], '--shard', str(i)],
                                start_new_session=True)

    kids = [start(i) for i in range(args.workers)]
    try:
        # A worker that gave up on a stuck map starts again (done maps are skipped), up to
        # 5 times each.
        restarts = [0] * args.workers
        while any(k.poll() is None or (k.poll() == STUCK and restarts[i] < 5)
                  for i, k in enumerate(kids)):
            for i, k in enumerate(kids):
                if k.poll() == STUCK and restarts[i] < 5:
                    restarts[i] += 1
                    print(f'worker {i}: stuck on a map; starting it again', flush=True)
                    kids[i] = start(i)
            time.sleep(2.0)
    except KeyboardInterrupt:
        for k in kids:
            os.killpg(k.pid, signal.SIGINT)
        raise
    files = [f for f in os.listdir(args.out) if f.endswith('.npz')]
    pairs = sum(len(np.load(os.path.join(args.out, f))['plan_ms']) for f in files)
    print(f'{len(files)} maps, {pairs} paths in {args.out}, '
          f'{time.monotonic() - t0:.0f} s')


if __name__ == '__main__':
    main()
