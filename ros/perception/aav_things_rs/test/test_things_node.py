"""
The Rust things node itself: cubes and TF in, the Python's things out.

It starts the built node on its own ROS domain (160-199), on this computer only, with
aav_things' things.yaml, and plays a box walking 1 m/s across the car's path, 8 m ahead:
its cubes (base_link) at 10 Hz, and odom <- base_link on /tf at each cloud's stamp. The
same clouds through aav_things' Python modules (groups.py, tracker.py) give the list the
node must publish: one thing, its id, place and speed; the markers' DELETEALL, box, arrow
and words. Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
import signal
import subprocess
import time

import numpy as np
import pytest

try:
    from aav_interfaces.msg import Things
    from aav_things.groups import group, GroupShape, to_frame
    from aav_things.tracker import Tracker
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import TransformStamped
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

SIZE = 0.25


def box(x, y):
    """Return a 0.5 m box's cube centres, 1.5 m tall, in base_link."""
    return np.array([(a, b, c) for a in (x - 0.125, x + 0.125) for b in (y - 0.125, y + 0.125)
                     for c in np.arange(0.425, 1.5, SIZE)], dtype=np.float32)


def environment():
    """Return this test's environment: its own domain, this computer only."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(160 + os.getpid() % 40),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return env


def test_a_walker_is_the_pythons_thing():
    config = os.path.join(get_package_share_directory('aav_things'), 'config', 'things.yaml')
    binary = os.path.join(get_package_prefix('aav_things_rs'), 'lib', 'aav_things_rs',
                          'things')
    node = subprocess.Popen([binary, '--ros-args', '--params-file', config],
                            env=environment(), start_new_session=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('things_node_test')
        got, marks = [], []
        me.create_subscription(Things, '/world/things', got.append, 10)
        me.create_subscription(MarkerArray, '/world/things/markers', marks.append, 10)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        cubes = me.create_publisher(PointCloud2, '/perception/voxels/cubes', newest)
        tf = me.create_publisher(TFMessage, '/tf', 100)
        python = Tracker()
        start, k = time.time(), 0
        end = time.monotonic() + 60.0
        # Wait for the node to hear us, then play 3 s of the walker.
        while (cubes.get_subscription_count() == 0 or tf.get_subscription_count() == 0) \
                and time.monotonic() < end:
            rclpy.spin_once(me, timeout_sec=0.1)
        while k < 30 and node.poll() is None:
            stamp = start + 0.1 * k
            header = Header(frame_id='base_link')
            header.stamp.sec = int(stamp)
            header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
            pose = TransformStamped()
            pose.header.stamp, pose.header.frame_id = header.stamp, 'odom'
            pose.child_frame_id = 'base_link'
            pose.transform.translation.x = 2.0
            pose.transform.rotation.w = 1.0
            tf.publish(TFMessage(transforms=[pose]))
            points = box(8.0, -1.5 + 0.1 * k)
            cubes.publish(point_cloud2.create_cloud_xyz32(header, points))
            python.step(header.stamp.sec + header.stamp.nanosec * 1e-9,
                        to_frame(group(points.astype(float), GroupShape()), 2.0, 0.0, 0.0),
                        (2.0, 0.0))
            k += 1
            stop = time.monotonic() + 0.1
            while time.monotonic() < stop:
                rclpy.spin_once(me, timeout_sec=0.02)
        stop = time.monotonic() + 0.5
        while time.monotonic() < stop:
            rclpy.spin_once(me, timeout_sec=0.05)
        listed = [m for m in got if m.things]
        assert listed, 'no things: ' + (node.stdout.read().decode()
                                        if node.poll() is not None else str(len(got)))
        last = listed[-1]
        assert last.header.frame_id == 'odom'
        (thing,) = last.things
        t = last.header.stamp.sec + last.header.stamp.nanosec * 1e-9
        (want,) = python.at(t)
        assert thing.id == want.id == 1
        assert thing.kind == 'unknown' and not thing.radar
        # The same filter, read at the same time: a hair apart (float32, the stamp).
        assert thing.x == pytest.approx(want.x[0], abs=0.02)
        assert thing.y == pytest.approx(want.x[1], abs=0.02)
        assert thing.vy == pytest.approx(want.x[3], abs=0.02)
        assert thing.vy == pytest.approx(1.0, abs=0.3)
        assert thing.seen == want.seen
        array = marks[-1].markers
        assert array[0].action == Marker.DELETEALL
        assert [m.type for m in array[1:]] == [
            Marker.CUBE, Marker.ARROW, Marker.TEXT_VIEW_FACING]
        assert array[3].text.startswith('1 ') and array[3].text.endswith(' m/s')
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
