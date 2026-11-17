"""
The Rust lidar_scan node itself: one lidar turn in, the Python's scan out.

It starts the built node on its own ROS domain (160-199), on this computer only, and
publishes aav_voxels' made-up lidar turn (test/lot.py: a post ahead, a wall on the left),
in the lidar's frame. The scan must be what aav_lidar's scan.py makes of the same points,
with the Python node's frame, stamp, angles and QoS. Needs the workspace built and
sourced: `colcon test` runs it.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest

try:
    from aav_lidar.scan import flatten, ScanShape, to_base
    from ament_index_python.packages import get_package_prefix
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

# aav_voxels' made-up lidar turn, from the source tree (its tests aren't installed).
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'aav_voxels' / 'test'))

BODY = (-0.5, 1.9, 0.8)
POST = (10.0, 10.5, -0.25, 0.25, 1.8)
WALL = (5.0, 30.0, 8.0, 8.3, 2.5)


def environment():
    """Return this test's environment: its own domain, this computer only."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(160 + os.getpid() % 40),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return env


def binary():
    return os.path.join(get_package_prefix('aav_lidar_rs'), 'lib', 'aav_lidar_rs',
                        'lidar_scan')


def test_one_turn_gives_the_pythons_scan():
    import lot
    mount = (list(lot.LIDAR), [0.0, 0.0, 0.0])
    base = lot.turn([POST, WALL])
    lidar = (base - np.asarray(lot.LIDAR)).astype(np.float32)
    shape = ScanShape(body=BODY)
    expected = flatten(to_base(lidar.astype(float), *mount), shape).astype(np.float32)

    node = subprocess.Popen(
        [binary(), '--ros-args', '-p', f'lidar_xyz:={mount[0]}',
         '-p', f'lidar_rpy:={mount[1]}', '-p', f'body:={list(BODY)}'],
        env=environment(), start_new_session=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('lidar_scan_node_test')
        got = []
        me.create_subscription(LaserScan, '/sensors/lidar/scan', got.append,
                               qos_profile_sensor_data)
        out = me.create_publisher(PointCloud2, '/sensors/lidar/points',
                                  QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        header = Header(frame_id='lidar_link')
        header.stamp.sec, header.stamp.nanosec = 100, 500
        msg = point_cloud2.create_cloud_xyz32(header, lidar)
        end = time.monotonic() + 60.0
        while not got and time.monotonic() < end and node.poll() is None:
            out.publish(msg)
            for _ in range(10):
                rclpy.spin_once(me, timeout_sec=0.05)
        assert got, 'nothing back: ' + (node.stdout.read().decode()
                                        if node.poll() is not None else '')
        scan = got[-1]
        assert scan.header.frame_id == 'base_link'
        assert (scan.header.stamp.sec, scan.header.stamp.nanosec) == (100, 500)
        assert len(scan.ranges) == 720
        assert np.array_equal(np.asarray(scan.ranges, dtype=np.float32), expected)
        assert scan.angle_min == pytest.approx(-math.pi, abs=1e-6)
        assert scan.angle_max == pytest.approx(-math.pi + 719 * shape.angle_increment,
                                               abs=1e-6)
        assert scan.angle_increment == pytest.approx(shape.angle_increment, abs=1e-9)
        assert (scan.range_min, scan.scan_time) == pytest.approx((0.2, 0.1))
        assert scan.range_max == pytest.approx(30.0)
        me.destroy_node()
    finally:
        rclpy.try_shutdown()
        if node.poll() is None:
            os.killpg(node.pid, signal.SIGINT)
            try:
                node.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(node.pid, signal.SIGKILL)
    assert node.returncode == 0, 'the node did not stop cleanly on Ctrl-C'


def test_a_missing_mount_stops_the_node():
    done = subprocess.run([binary(), '--ros-args', '-p', 'body:=[-0.5, 1.9, 0.8]'],
                          env=environment(), capture_output=True, timeout=30,
                          start_new_session=True)
    assert done.returncode != 0
    assert b'lidar_xyz' in done.stdout + done.stderr
