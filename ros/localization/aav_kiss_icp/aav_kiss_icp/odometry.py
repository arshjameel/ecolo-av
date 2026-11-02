"""
The maths around KISS-ICP, with no ROS: the cloud in, the odometry out.

Imported by kiss_icp_node.py; tested in test/test_odometry.py.

- cloud_points(): a PointCloud2's bytes into x, y, z (and the per-point time, if the
  cloud has a `time` field: the car's VLP-16 does, Gazebo's lidar doesn't).
- to_base_link(): the scan moved from lidar_link into base_link (the mount from
  vehicle.yaml), so KISS-ICP's pose is the car's own.
- crop(): the car's own body and the ground cut out, and points nearer or further
  than the ranges.
- sweep_times(): the times scaled to 0-1 across the turn, for KISS-ICP's deskew.
- pose_of(), twist_of(): a 4x4 pose into position and quaternion; the last step into a
  twist in base_link.
- find_python(): where the kiss environment's Python is.

Metres, radians, seconds; REP-103 (x forward, y left, z up).
"""

import math
import os

import numpy as np

# PointField datatypes (sensor_msgs/msg/PointField) and their NumPy types.
DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

# The kiss environment's Python, from the repository root (pixi install --locked -e kiss).
KISS_PYTHON = os.path.join('.pixi', 'envs', 'kiss', 'bin', 'python')


def cloud_points(fields, data, point_step, count, big_endian=False):
    """
    Return (points N x 3 float64, times N float64 or None) from a PointCloud2's bytes.

    fields: [(name, offset, datatype)], as PointCloud2's fields. Points with a non-finite
    x, y or z (Gazebo's rays that hit nothing are inf) are left out, with their times.
    """
    order = '>' if big_endian else '<'
    named = {name: (offset, DTYPES[datatype]) for name, offset, datatype in fields
             if datatype in DTYPES}
    missing = [n for n in ('x', 'y', 'z') if n not in named]
    if missing:
        raise ValueError(f'the cloud has no {", ".join(missing)} field')
    wanted = ['x', 'y', 'z'] + (['time'] if 'time' in named else [])
    dtype = np.dtype({'names': wanted,
                      'formats': [order + named[n][1] for n in wanted],
                      'offsets': [named[n][0] for n in wanted],
                      'itemsize': point_step})
    rows = np.frombuffer(data, dtype=dtype, count=count)
    xyz = np.stack([rows[n].astype(np.float64) for n in ('x', 'y', 'z')], axis=1)
    keep = np.isfinite(xyz).all(axis=1)
    times = rows['time'].astype(np.float64)[keep] if 'time' in named else None
    return xyz[keep], times


def rotation(roll, pitch, yaw):
    """Return the 3x3 rotation for roll, pitch, yaw (rad), as URDF's rpy: Rz Ry Rx."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def to_base_link(points, xyz, rpy):
    """Return the points (N x 3, lidar_link) in base_link: the mount at xyz, turned by rpy."""
    return points @ rotation(*rpy).T + np.asarray(xyz, dtype=np.float64)


def crop(points, times, body_min, body_max, min_range, max_range, origin,
         min_height=-float('inf')):
    """
    Return (points, times) without the car's own body or the ground, between the ranges.

    body_min, body_max: (x, y) corners of the car's outline in base_link; a point over
    it, at any height, is the car. The ranges are measured from origin (the lidar).
    min_height (m above base_link): lower points are the ground. On a flat ground the
    lidar's downward beams draw the same circles round the car every scan, and those
    say "you didn't move" (30 Oct, Gazebo: KISS-ICP read 12 m for 21 m driven).
    """
    x, y = points[:, 0], points[:, 1]
    body = ((x >= body_min[0]) & (x <= body_max[0]) &
            (y >= body_min[1]) & (y <= body_max[1]))
    distance = np.linalg.norm(points - np.asarray(origin, dtype=np.float64), axis=1)
    keep = (~body & (distance >= min_range) & (distance <= max_range) &
            (points[:, 2] >= min_height))
    return points[keep], (times[keep] if times is not None else None)


def sweep_times(times):
    """
    Return the per-point times scaled to 0-1 across the turn, or None to deskew nothing.

    None when there are no times or all are the same (Gazebo's lidar: one instant):
    KISS-ICP 1.3 scales them itself again, and equal times give it NaN.
    """
    if times is None or times.size == 0:
        return None
    low, high = float(times.min()), float(times.max())
    if not high > low:
        return None
    return (times - low) / (high - low)


def quaternion(r):
    """Return (x, y, z, w) for a 3x3 rotation matrix."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return tuple(q)


def pose_of(matrix):
    """Return ((x, y, z), (qx, qy, qz, qw)) for a 4x4 pose."""
    m = np.asarray(matrix, dtype=np.float64)
    return tuple(float(v) for v in m[:3, 3]), tuple(float(v) for v in quaternion(m[:3, :3]))


def yaw_of(matrix):
    """Return the heading (rad) of a 4x4 pose."""
    m = np.asarray(matrix, dtype=np.float64)
    return math.atan2(m[1, 0], m[0, 0])


def rotation_vector(r):
    """Return the axis times the angle (rad) of a 3x3 rotation."""
    angle = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0)))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return axis * (angle / (2.0 * math.sin(angle)))


def twist_of(delta, dt):
    """
    Return ((vx, vy, vz), (wx, wy, wz)) in base_link: the last step (4x4) over dt seconds.

    The step is the new pose seen from the last one (KISS-ICP's last_delta). Zeros for a
    dt that isn't positive.
    """
    if not dt > 0.0:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    d = np.asarray(delta, dtype=np.float64)
    v = d[:3, 3] / dt
    w = rotation_vector(d[:3, :3]) / dt
    return tuple(float(x) for x in v), tuple(float(x) for x in w)


def dropped_between(last_stamp, stamp, period):
    """Return how many clouds were missed between two stamps (s), period seconds apart."""
    if last_stamp is None or not period > 0.0:
        return 0
    return max(0, round((stamp - last_stamp) / period) - 1)


def find_python(given='', root=None):
    """
    Return the kiss environment's Python: `given` if set, else <root>/.pixi/envs/kiss.

    root: PIXI_PROJECT_ROOT, else the folder the launch started in (pixi starts it in
    the repository). The path may not exist: the caller says how to install it.
    """
    if given:
        return os.path.abspath(os.path.expanduser(given))
    root = root or os.environ.get('PIXI_PROJECT_ROOT') or os.getcwd()
    return os.path.join(root, KISS_PYTHON)
