"""
The ndt node itself, on a made-up map: a cloud in, a pose and a status out.

It starts the built node on its own ROS domain, on this computer only, with a map of
four walls and two boxes, and a start 0.4 m and 6 degrees off. One cloud of the same
walls, seen from the true pose, must give a pose within 5 cm and 0.5 degrees, stamped
with the cloud's stamp. Needs the workspace built and sourced: `colcon test` runs it.
"""

import math
import os
import signal
import struct
import subprocess
import time

import numpy as np
import pytest

try:
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import rclpy
    from sensor_msgs.msg import PointCloud2, PointField
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

MOUNT = np.array([0.9, 0.0, 1.8])
TRUTH = (2.0, 1.0, 0.1)
START = (2.3, 0.75, 0.1 - math.radians(6.0))


def made_up_lot():
    """Return the map's points: four walls round a 40 x 30 m yard and two parked cars."""
    rng = np.random.default_rng(5)

    def wall(a, b, top):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = int(np.linalg.norm(b - a) / 0.1)
        t = rng.uniform(0.0, 1.0, (n * 8, 1))
        xy = a + t * (b - a)
        return np.c_[xy, rng.uniform(0.4, top, n * 8)]

    def box(x, y, w, h):
        corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)]
        return np.vstack([wall(p, q, 1.5) for p, q in zip(corners, corners[1:])])

    return np.vstack([wall((-20, -15), (20, -15), 2.0), wall((20, -15), (20, 15), 2.0),
                      wall((20, 15), (-20, 15), 2.0), wall((-20, 15), (-20, -15), 2.0),
                      box(8, 4, 4.5, 1.8), box(-7, -6, 1.8, 4.5)])


def write_pcd(path, points):
    n = len(points)
    header = (f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {n}\nHEIGHT 1\nPOINTS {n}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode() + np.asarray(points, '<f4').tobytes())


def scan_from(points, pose):
    """Return the map's points as the lidar at `pose` sees them, in lidar_link."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    d = points - [x, y, 0.0]
    base = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]]
    return base - MOUNT


def cloud(points, stamp):
    msg = PointCloud2()
    msg.header.frame_id = 'lidar_link'
    msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.point_step, msg.row_step = 12, 12 * len(points)
    msg.data = b''.join(struct.pack('<3f', *p) for p in points)
    return msg


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def test_a_cloud_gives_a_pose_and_a_status(tmp_path):
    lot = made_up_lot()
    map_file = tmp_path / 'lot.pcd'
    write_pcd(map_file, lot)
    env = dict(os.environ, ROS_DOMAIN_ID=str(180 + os.getpid() % 40),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    binary = os.path.join(get_package_prefix('aav_ndt'), 'lib', 'aav_ndt', 'ndt')
    node = subprocess.Popen(
        [binary, '--ros-args', '-p', f'map:={map_file}', '-p', f'start_x:={START[0]}',
         '-p', f'start_y:={START[1]}', '-p', f'start_yaw:={START[2]}',
         '-p', 'lidar_xyz:=[0.9, 0.0, 1.8]'],
        env=env, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('ndt_node_test')
        poses, statuses = [], []
        me.create_subscription(PoseWithCovarianceStamped, '/localization/ndt/pose',
                               poses.append, 10)
        me.create_subscription(DiagnosticStatus, '/localization/ndt/status',
                               statuses.append, 10)
        out = me.create_publisher(PointCloud2, '/sensors/lidar/points', 1)
        scan = scan_from(lot[np.hypot(*(lot[:, :2] - TRUTH[:2]).T) < 40.0], TRUTH)
        end = time.monotonic() + 60.0
        while not poses and time.monotonic() < end:
            if node.poll() is not None:
                break
            out.publish(cloud(scan, (100, 500)))
            for _ in range(10):
                rclpy.spin_once(me, timeout_sec=0.05)
        assert poses, ('no pose: ' + (node.stdout.read().decode() if node.poll() is not None
                                      else f'{len(statuses)} statuses'))
        pose = poses[0]
        p = pose.pose.pose
        assert math.hypot(p.position.x - TRUTH[0], p.position.y - TRUTH[1]) < 0.05
        assert abs(math.degrees(yaw_of(p.orientation) - TRUTH[2])) < 0.5
        assert (pose.header.frame_id, pose.header.stamp.sec, pose.header.stamp.nanosec) == (
            'map', 100, 500)
        assert pose.pose.covariance[0] > 0.0 and pose.pose.covariance[14] == 1e3
        for _ in range(20):
            rclpy.spin_once(me, timeout_sec=0.05)
        status = statuses[-1]
        values = {kv.key: kv.value for kv in status.values}
        assert status.name == 'ndt' and status.message.startswith('TRACKING')
        assert status.level == DiagnosticStatus.OK
        assert float(values['score_per_point']) > 2.0 and float(values['match_ms']) >= 0.0
        me.destroy_node()
    finally:
        rclpy.try_shutdown()
        if node.poll() is None:
            os.killpg(node.pid, signal.SIGINT)
            try:
                node.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(node.pid, signal.SIGKILL)
