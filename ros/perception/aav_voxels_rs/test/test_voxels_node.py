"""
The Rust voxels node itself: one lidar turn in, the Python's cubes and its status out.

It starts the built node on its own ROS domain, on this computer only, and publishes
aav_voxels' made-up lidar turn (test/lot.py: a post ahead, a wall on the left), in the
lidar's frame. The cubes, the ground squares and the status must be what aav_voxels'
cubes.py makes of the same points, with the Python node's frame, stamp and status keys.
Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest

try:
    from aav_lidar.scan import to_base
    from aav_voxels.cubes import cubes, CubeShape
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticStatus
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
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
KEYS = ['ms', 'cubes', 'ground_cubes', 'points', 'fitted', 'roll_deg', 'pitch_deg',
        'clouds_dropped']


def rows(cloud):
    """Return a cloud's x, y, z as a set of float32 triples."""
    xyz = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'))
    return {tuple(p) for p in np.asarray(xyz, dtype=np.float32).reshape(-1, 3).tolist()}


def test_one_turn_gives_the_pythons_cubes():
    import lot
    mount = (list(lot.LIDAR), [0.0, 0.0, 0.0])
    base = lot.turn([POST, WALL], roll=np.radians(2.0))
    lidar = (base - np.asarray(lot.LIDAR)).astype(np.float32)
    expected = cubes(to_base(lidar.astype(float), *mount), CubeShape(body=BODY))

    env = dict(os.environ, ROS_DOMAIN_ID=str(150 + os.getpid() % 30),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    binary = os.path.join(get_package_prefix('aav_voxels_rs'), 'lib', 'aav_voxels_rs',
                          'voxels')
    node = subprocess.Popen(
        [binary, '--ros-args', '-p', f'lidar_xyz:={mount[0]}', '-p', f'lidar_rpy:={mount[1]}',
         '-p', f'body:={list(BODY)}'],
        env=env, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('voxels_node_test')
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        got = {'cubes': [], 'ground': [], 'status': []}
        me.create_subscription(PointCloud2, '/perception/voxels/cubes',
                               got['cubes'].append, newest)
        me.create_subscription(PointCloud2, '/perception/voxels/ground',
                               got['ground'].append, newest)
        me.create_subscription(DiagnosticStatus, '/perception/voxels/status',
                               got['status'].append, 10)
        out = me.create_publisher(PointCloud2, '/sensors/lidar/points', newest)
        header = Header(frame_id='lidar_link')
        header.stamp.sec, header.stamp.nanosec = 100, 500
        msg = point_cloud2.create_cloud_xyz32(header, lidar)
        end = time.monotonic() + 60.0
        while not all(got.values()) and time.monotonic() < end and node.poll() is None:
            out.publish(msg)
            for _ in range(10):
                rclpy.spin_once(me, timeout_sec=0.05)
        assert all(got.values()), ('nothing back: ' + (
            node.stdout.read().decode() if node.poll() is not None
            else str({k: len(v) for k, v in got.items()})))

        cube_cloud, ground_cloud = got['cubes'][-1], got['ground'][-1]
        for c in (cube_cloud, ground_cloud):
            assert c.header.frame_id == 'base_link'
            assert (c.header.stamp.sec, c.header.stamp.nanosec) == (100, 500)
            assert [f.name for f in c.fields] == ['x', 'y', 'z']
        assert rows(cube_cloud) == {tuple(p) for p in
                                    expected.obstacles.astype(np.float32).tolist()}
        ground = np.asarray(sorted(rows(ground_cloud)))
        want = np.asarray(sorted(tuple(p) for p in expected.ground.astype(np.float32).tolist()))
        assert ground.shape == want.shape and np.abs(ground - want).max() < 1e-6

        status = got['status'][-1]
        values = {kv.key: kv.value for kv in status.values}
        assert [kv.key for kv in status.values] == KEYS
        assert (status.name, status.level, status.message) == (
            'voxels', DiagnosticStatus.OK, 'road fitted')
        assert int(values['cubes']) == len(expected.obstacles)
        assert int(values['ground_cubes']) == len(expected.ground)
        assert int(values['points']) == expected.points
        assert values['fitted'] == 'true'
        assert values['roll_deg'] == f'{np.degrees(expected.road.roll):.2f}'
        assert values['pitch_deg'] == f'{np.degrees(expected.road.pitch):.2f}'
        assert float(values['ms']) >= 0.0
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
    env = dict(os.environ, ROS_DOMAIN_ID=str(150 + os.getpid() % 30),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    binary = os.path.join(get_package_prefix('aav_voxels_rs'), 'lib', 'aav_voxels_rs',
                          'voxels')
    done = subprocess.run([binary, '--ros-args', '-p', 'body:=[-0.5, 1.9, 0.8]'], env=env,
                          capture_output=True, timeout=30, start_new_session=True)
    assert done.returncode != 0
    assert b'lidar_xyz' in done.stdout + done.stderr


def test_steps_finds_a_kerb():
    """steps:=true: a 0.15 m kerb 6 m ahead comes out on /perception/voxels/steps."""
    xs, ys = np.meshgrid(np.arange(3.0, 9.0, 0.05), np.arange(-2.0, 2.0, 0.05))
    zs = np.where(xs >= 6.0, 0.15, 0.0)
    points = np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1).astype(np.float32)
    env = dict(os.environ, ROS_DOMAIN_ID=str(150 + os.getpid() % 30),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    binary = os.path.join(get_package_prefix('aav_voxels_rs'), 'lib', 'aav_voxels_rs',
                          'voxels')
    node = subprocess.Popen(
        [binary, '--ros-args', '-p', 'lidar_xyz:=[0.0, 0.0, 0.0]',
         '-p', 'lidar_rpy:=[0.0, 0.0, 0.0]', '-p', f'body:={list(BODY)}', '-p', 'steps:=true'],
        env=env, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('voxels_steps_test')
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        got = {'steps': [], 'status': []}
        me.create_subscription(PointCloud2, '/perception/voxels/steps',
                               got['steps'].append, newest)
        me.create_subscription(DiagnosticStatus, '/perception/voxels/status',
                               got['status'].append, 10)
        out = me.create_publisher(PointCloud2, '/sensors/lidar/points', newest)
        msg = point_cloud2.create_cloud_xyz32(Header(frame_id='lidar_link'), points)
        end = time.monotonic() + 60.0
        while not all(got.values()) and time.monotonic() < end and node.poll() is None:
            out.publish(msg)
            for _ in range(10):
                rclpy.spin_once(me, timeout_sec=0.05)
        assert all(got.values()), ('nothing back: ' + (
            node.stdout.read().decode() if node.poll() is not None
            else str({k: len(v) for k, v in got.items()})))
        steps = np.asarray(sorted(rows(got['steps'][-1])))
        assert got['steps'][-1].header.frame_id == 'base_link'
        assert len(steps) > 20
        assert steps[:, 0].min() >= 5.95 and steps[:, 0].max() <= 6.35
        status = got['status'][-1]
        values = {kv.key: kv.value for kv in status.values}
        assert [kv.key for kv in status.values] == KEYS + ['steps']
        assert int(values['steps']) == len(steps)
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
