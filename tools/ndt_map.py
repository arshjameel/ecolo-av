#!/usr/bin/env python3
"""
NDT's map of Gazebo's test lot: the lidar's clouds, laid down where the car truly was.

    pixi run ndt-map drive              start the test lot headless, drive the mapping route,
                                        record the lidar and the truth, build the map (~6 min)
    pixi run ndt-map build <bag>        build the map from a bag already recorded
    pixi run ndt-map drive --keep-bag   ... and keep the bag (bags/ndt-map-<time>: ~1 MB
                                        a cloud, about 3 GB for the route)

The map goes to maps/ndt/test_lot.pcd (binary PCD: x y z float32, map frame) with
maps/ndt/test_lot.yaml beside it (frame, datum, source bag, git commit, voxel size, the
route). maps/ is not in git: this tool makes it again. The design:
docs/designs/2026-10-29_ndt.md ("The map").

For each cloud: Gazebo's truth is interpolated at the cloud's stamp (position on a
straight line, rotation by slerp; skipped unless there is truth within 20 ms on both
sides), the points go lidar_link -> base_link (vehicle.yaml's lidar_link) -> map; points
that hit nothing, past 45 m, on the car's own body, or below 0.3 m in base_link (the
ground) are dropped; all of it is thinned to one point per 10 cm cube.

The truth is used only here, offline: aav_ndt's node never reads it (aav_gazebo's rule).
The mapping route is not one of controller-compare's tours, and has no obstacle.
Gazebo's lidar noise isn't seeded by us, so two drives give maps a few millimetres apart.

The maths (everything but read_bag and drive) needs no ROS: tools/tests/test_ndt_map.py.
"""

import argparse
import bisect
import datetime
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'maps' / 'ndt' / 'test_lot'
CLOUD_TOPIC = '/sensors/lidar/points'
TRUTH_TOPIC = '/vehicle/gazebo/truth'
VOXEL = 0.10            # m: the map's thinning
MIN_HEIGHT = 0.3        # m above base_link: below is ground
MAX_RANGE = 45.0        # m from base_link, flat
TRUTH_GAP = 0.020       # s: truth needed this close on each side of a cloud
DATUM = [45.3860, -75.6960, 0.0]   # aav_localization config/navsat.yaml: map (0, 0)
# (name, x m, y m, heading deg), map frame: round the lot's edges, away from the scored
# tours' lines (tools/controller_compare.py TOURS), forward only.
ROUTE = [
    ('the south side', 44.0, -4.0, 0.0),
    ('the east lane', 45.0, 42.0, 90.0),
    ('the north side', 5.0, 44.0, 180.0),
    ('the west side', -4.0, 12.0, -90.0),
    ('home', 0.0, 0.0, 0.0),
]


# --- The maths: no ROS -----------------------------------------------------------------

def quat_matrix(q):
    """Return the 3x3 rotation of a unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_matrix(roll, pitch, yaw):
    """Return the rotation of URDF's rpy: roll about x, then pitch about y, then yaw about z."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def slerp(q0, q1, t):
    """Return the unit quaternion t of the way from q0 to q1, the short way round."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
    else:
        theta = math.acos(dot)
        q = (math.sin((1 - t) * theta) * q0 + math.sin(t * theta) * q1) / math.sin(theta)
    return q / np.linalg.norm(q)


class Truth:
    """Gazebo's truth, by time: base_link's position and rotation in the map."""

    def __init__(self, samples):
        """Keep samples, [(t s, (x, y, z), (qx, qy, qz, qw))], in time order."""
        samples = sorted(samples, key=lambda s: s[0])
        self.times = [s[0] for s in samples]
        self.positions = [np.asarray(s[1], float) for s in samples]
        self.rotations = [np.asarray(s[2], float) for s in samples]

    def at(self, t, gap=TRUTH_GAP):
        """Return (position, quaternion) at t, or None without truth within gap both sides."""
        i = bisect.bisect_left(self.times, t)
        if i < len(self.times) and self.times[i] == t:
            return self.positions[i], self.rotations[i]
        if i == 0 or i == len(self.times):
            return None
        t0, t1 = self.times[i - 1], self.times[i]
        if t - t0 > gap or t1 - t > gap:
            return None
        f = (t - t0) / (t1 - t0)
        position = self.positions[i - 1] + f * (self.positions[i] - self.positions[i - 1])
        return position, slerp(self.rotations[i - 1], self.rotations[i], f)


def body_box(body):
    """Return the car's outline in base_link, ((x min, x max), (y min, y max)), m."""
    rear = -body['rear_overhang']
    half = body['width'] / 2.0
    return (rear, rear + body['length']), (-half, half)


def to_map(points, mount, pose, body=None, min_height=MIN_HEIGHT, max_range=MAX_RANGE):
    """
    Return a cloud's useful points in the map frame.

    points: N x 3 in lidar_link. mount: (xyz, rpy) of lidar_link on base_link. pose:
    (position, quaternion) of base_link in the map. Non-finite points, points past
    max_range (flat, from base_link), on the car's body or not above min_height in
    base_link (the ground) are dropped.
    """
    points = np.asarray(points, float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    xyz, rpy = mount
    base = points @ rpy_matrix(*rpy).T + np.asarray(xyz, float)
    keep = (base[:, 2] > min_height) & (np.hypot(base[:, 0], base[:, 1]) <= max_range)
    if body is not None:
        (x0, x1), (y0, y1) = body
        keep &= ~((base[:, 0] >= x0) & (base[:, 0] <= x1) &
                  (base[:, 1] >= y0) & (base[:, 1] <= y1))
    position, rotation = pose
    return base[keep] @ quat_matrix(rotation).T + np.asarray(position, float)


def thin(points, size=VOXEL):
    """Return one point per cube of side size: the mean of the points in it."""
    points = np.asarray(points, float).reshape(-1, 3)
    if len(points) == 0:
        return points
    keys = np.floor(points / size).astype(np.int64)
    _, index, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    index = index.reshape(-1)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, index, points)
    return sums / counts[:, None]


def cloud_xyz(fields, point_step, data, width, height=1, row_step=None, bigendian=False):
    """
    Return a PointCloud2's x, y, z as an N x 3 array, from its bytes.

    fields: [(name, offset, datatype, count)] (PointField: 7 is float32, 8 float64).
    """
    kinds = {7: 'f4', 8: 'f8'}
    found = {name: (offset, kinds.get(datatype)) for name, offset, datatype, count in fields
             if count == 1}
    missing = [n for n in 'xyz' if n not in found or found[n][1] is None]
    if missing:
        raise ValueError(f'the cloud has no float x, y, z: {missing} missing')
    row_step = row_step or point_step * width
    order = '>' if bigendian else '<'
    dtype = np.dtype({'names': list('xyz'),
                      'formats': [order + found[n][1] for n in 'xyz'],
                      'offsets': [found[n][0] for n in 'xyz'],
                      'itemsize': point_step})
    rows = [np.frombuffer(bytes(data), dtype=dtype, count=width, offset=r * row_step)
            for r in range(height)]
    cloud = np.concatenate(rows) if rows else np.zeros(0, dtype=dtype)
    return np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(float)


def write_pcd(path, points):
    """Write points (N x 3, m) as a binary PCD v0.7 of float32 x, y, z."""
    points = np.asarray(points, np.float32).reshape(-1, 3)
    n = len(points)
    header = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n'
              'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.astype('<f4').tobytes())


def read_pcd(path):
    """Return a binary PCD's x, y, z (as write_pcd writes it) as N x 3."""
    with open(path, 'rb') as f:
        blob = f.read()
    end = blob.index(b'DATA binary\n') + len(b'DATA binary\n')
    header = dict(line.split(' ', 1) for line in blob[:end].decode('ascii').splitlines()
                  if line and not line.startswith('#'))
    n = int(header['POINTS'])
    return np.frombuffer(blob[end:], dtype='<f4', count=3 * n).reshape(n, 3).astype(float)


def build(clouds, truth, mount, body=None, voxel=VOXEL):
    """
    Return (the map's points, notes): clouds is [(t, N x 3 in lidar_link)].

    Notes count the clouds used and those skipped for want of truth.
    """
    parts, used, skipped = [], 0, 0
    heights = []
    for t, points in clouds:
        pose = truth.at(t)
        if pose is None:
            skipped += 1
            continue
        heights.append(float(pose[0][2]))
        parts.append(to_map(points, mount, pose, body))
        used += 1
        # Thin as we go, so a long drive never holds every point at once.
        if len(parts) >= 50:
            parts = [thin(np.concatenate(parts), voxel)]
    cloud = thin(np.concatenate(parts), voxel) if parts else np.zeros((0, 3))
    notes = {'clouds_used': used, 'clouds_skipped_no_truth': skipped,
             'truth_z_mean_m': round(float(np.mean(heights)), 4) if heights else None}
    return cloud, notes


def git_commit():
    """Return this checkout's commit, or 'unknown'."""
    out = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=ROOT,
                         capture_output=True, text=True)
    return out.stdout.strip() or 'unknown'


def describe(points, bag, notes, voxel=VOXEL):
    """Return the YAML beside the PCD, as a dict."""
    lo, hi = (points.min(axis=0), points.max(axis=0)) if len(points) else ([0] * 3, [0] * 3)
    return {
        'frame': 'map', 'datum': DATUM, 'source_bag': str(bag), 'git_commit': git_commit(),
        'voxel_m': voxel, 'points': int(len(points)),
        'ground_removed_below_m': MIN_HEIGHT, 'max_range_m': MAX_RANGE,
        'bounds_m': {'min': [round(float(v), 2) for v in lo],
                     'max': [round(float(v), 2) for v in hi]},
        'world': 'test_lot', 'route': [list(leg) for leg in ROUTE],
        'made': datetime.datetime.now().isoformat(timespec='seconds'), **notes,
        'note': "Gazebo's lidar has no sweep (one instant per turn); the car's VLP-16 does",
    }


# --- ROS: the bag and the drive --------------------------------------------------------

def read_bag(bag):
    """Return ([(t, points)], Truth) from a bag with the lidar and Gazebo's truth."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    def messages(topic, kind):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        while reader.has_next():
            _, data, _ = reader.read_next()
            yield deserialize_message(data, kind)

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    truth = Truth([(stamp(m), (m.pose.pose.position.x, m.pose.pose.position.y,
                               m.pose.pose.position.z),
                    (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                     m.pose.pose.orientation.z, m.pose.pose.orientation.w))
                   for m in messages(TRUTH_TOPIC, Odometry)])
    clouds = ((stamp(m), cloud_xyz([(f.name, f.offset, f.datatype, f.count) for f in m.fields],
                                   m.point_step, m.data, m.width, m.height, m.row_step,
                                   m.is_bigendian))
              for m in messages(CLOUD_TOPIC, PointCloud2))
    return clouds, truth


def drive(bag):
    """Start the test lot headless, drive ROUTE with Nav2, record the lidar and truth."""
    import rclpy
    sys.path.insert(0, str(ROOT / 'tools'))
    import controller_compare as cc   # its Recorder, private domain, process-group stop

    domain = cc.private(set())
    log_path = bag.parent / f'{bag.name}-sim.log'
    print(f'== the test lot, headless, on ROS domain {domain} (this computer only)', flush=True)
    with open(log_path, 'w') as log:
        sim = subprocess.Popen(['ros2', 'launch', 'aav_bringup', 'sim.launch.py',
                                'world:=test_lot', 'gui:=false', 'rviz:=false',
                                'lidar:=true', 'reversing:=false'],
                               cwd=ROOT, env=dict(os.environ), stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
        record = None
        rclpy.init()
        rec = cc.Recorder()
        try:
            up = rec.spin_for(cc.UP_TIME, lambda: rec.last_truth is not None and rec.glob)
            if not (up and rec.spin_for(cc.UP_TIME, rec.nav.server_is_ready)):
                raise SystemExit(f'the simulator never came up: see {log_path}')
            record = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(bag), '--topics',
                                       CLOUD_TOPIC, TRUTH_TOPIC, '/clock'],
                                      cwd=ROOT, env=dict(os.environ), stdout=log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            rec.spin_for(cc.SETTLE)
            for name, x, y, heading in ROUTE:
                leg, notes = cc.drive_leg(rec, sim.pid, (x, y, math.radians(heading)), 1.0,
                                          None, None)
                print(f'   {name}: ' + ('reached' if leg['reached'] else 'NOT reached') +
                      ''.join(f'; {n}' for n in notes), flush=True)
        finally:
            rec.destroy_node()
            rclpy.try_shutdown()
            if record is not None:
                os.killpg(record.pid, signal.SIGINT)
                try:
                    record.wait(30)
                except subprocess.TimeoutExpired:
                    cc.stop(record)
            if not cc.stop(sim):
                print(f'   some of process group {sim.pid} would not stop', flush=True)
            sim.wait()
            subprocess.run(['fastdds', 'shm', 'clean'], capture_output=True)
            time.sleep(1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    d = sub.add_parser('drive', help='drive the mapping route in Gazebo, then build')
    d.add_argument('--keep-bag', action='store_true', help='keep the recorded bag')
    b = sub.add_parser('build', help='build the map from a bag')
    b.add_argument('bag', type=Path)
    parser.add_argument('--out', type=Path, default=OUT,
                        help='where the map goes, without .pcd (default maps/ndt/test_lot)')
    parser.add_argument('--voxel', type=float, default=VOXEL, help='m (default 0.1)')
    args = parser.parse_args()

    from aav_description import vehicle
    v = vehicle.load()
    sensor = v['sensors']['lidar_link']
    mount = (sensor['xyz'], sensor['rpy'])
    body = body_box(v['vehicle']['body'])

    if args.command == 'drive':
        (ROOT / 'bags').mkdir(exist_ok=True)
        bag = ROOT / 'bags' / f'ndt-map-{datetime.datetime.now():%Y%m%d-%H%M%S}'
        drive(bag)
    else:
        bag = args.bag
    started = time.monotonic()
    clouds, truth = read_bag(bag)
    points, notes = build(clouds, truth, mount, body, args.voxel)
    if len(points) == 0:
        raise SystemExit(f'{bag}: no cloud had truth around it; nothing to map')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_pcd(args.out.with_suffix('.pcd'), points)
    with open(args.out.with_suffix('.yaml'), 'w') as f:
        yaml.safe_dump(describe(points, bag, notes, args.voxel), f, sort_keys=False)
    print(f'{len(points)} points from {notes["clouds_used"]} clouds '
          f'({notes["clouds_skipped_no_truth"]} without truth) in '
          f'{time.monotonic() - started:.0f} s: {args.out.with_suffix(".pcd")}')
    if args.command == 'drive' and not args.keep_bag:
        shutil.rmtree(bag, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
