"""
The Rust lane_finder itself: pictures and camera_info in, the Python's lane out.

It starts the built node on its own ROS domain (200-219), on this computer only, with
aav_lane_finder's lane_finder.yaml, the bench camera's place and debug_image, and plays a
made-up lane (aav_lane_finder's test/scenes.py) as rgb8 with a CameraInfo, then the same
picture again with no lines. The Python's LaneFinder on the same pictures gives the lane
the node must publish; the debug picture must come too. Needs the workspace built and
sourced: `colcon test` runs it.

aav_lane_finder's modules, scenes and config are read from its source folder beside this
one: that package depends on this one (its launch file starts either), so colcon doesn't
put it in this package's test environment.
"""

import math
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

PYTHON = Path(__file__).resolve().parents[2] / 'aav_lane_finder'
sys.path[:0] = [str(PYTHON), str(PYTHON / 'test')]

try:
    from aav_lane_finder.birds_eye import BirdsEye, Grid, Lens, Mount
    from aav_lane_finder.lane_lines import LaneFinder, Settings
    from aav_lane_geometry.msg import LaneGeometry
    from ament_index_python.packages import get_package_prefix
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

CAMERA = [1.80, 0.29, 1.20]
FOCAL = 343.0


def environment():
    """Return this test's environment: its own domain, this computer only."""
    ros_domain = os.environ.get('AAV_TEST_DOMAIN') or str(200 + os.getpid() % 20)
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain,
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return env


def image_msg(bgr):
    msg = Image(height=bgr.shape[0], width=bgr.shape[1], encoding='rgb8',
                step=3 * bgr.shape[1])
    msg.header.frame_id = 'camera_left_optical_frame'
    msg.header.stamp.sec, msg.header.stamp.nanosec = 12, 345
    msg.data = bgr[:, :, ::-1].tobytes()
    return msg


def info_msg(width, height):
    msg = CameraInfo(width=width, height=height, distortion_model='plumb_bob')
    msg.k = [FOCAL, 0.0, width / 2.0, 0.0, FOCAL, height / 2.0, 0.0, 0.0, 1.0]
    msg.d = [0.0] * 5
    return msg


def test_a_lane_is_the_pythons_lane(tmp_path):
    from scenes import picture   # noqa: I100 - the Python package's own test helper
    config = str(PYTHON / 'config' / 'lane_finder.yaml')
    binary = os.path.join(get_package_prefix('aav_lane_finder_rs'), 'lib',
                          'aav_lane_finder_rs', 'lane_finder')
    # As the launch file adds them; a file for the node by name, as -p loses to one.
    ours = tmp_path / 'ours.yaml'
    ours.write_text('lane_finder:\n  ros__parameters:\n'
                    f'    camera_xyz: {CAMERA}\n    wheelbase: 1.57\n    debug_image: true\n')
    node = subprocess.Popen(
        [binary, '--ros-args', '--params-file', config, '--params-file', str(ours)],
        env=environment(), start_new_session=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('lane_finder_node_test')
        got, debug = [], []
        me.create_subscription(LaneGeometry, '/perception/lane/geometry', got.append, 10)
        me.create_subscription(Image, '/perception/lane/image', debug.append, 2)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        images = me.create_publisher(Image, '/sensors/camera/front_left/image_raw', newest)
        infos = me.create_publisher(CameraInfo, '/sensors/camera/front_left/camera_info',
                                    newest)
        end = time.monotonic() + 60.0
        while (images.get_subscription_count() == 0 or infos.get_subscription_count() == 0) \
                and time.monotonic() < end and node.poll() is None:
            rclpy.spin_once(me, timeout_sec=0.1)
        lane_pic, empty_pic = picture(offset=0.3, radius=80.0), picture(lines=())
        python = LaneFinder(BirdsEye(Lens.pinhole(640, 360, FOCAL), Mount(tuple(CAMERA)),
                                     Grid()), Settings())
        wants = []
        for scene in (lane_pic, empty_pic):
            infos.publish(info_msg(640, 360))
            spin(me, 0.3)
            count = len(got)
            images.publish(image_msg(scene))
            wants.append(python.find(scene))
            stop = time.monotonic() + 10.0
            while len(got) == count and time.monotonic() < stop and node.poll() is None:
                rclpy.spin_once(me, timeout_sec=0.05)
            assert len(got) > count, 'no lane: ' + (
                node.stdout.read().decode() if node.poll() is not None else 'still running')
        spin(me, 0.5)
        lane, none = got[-2], got[-1]
        want = wants[0]
        assert lane.header.frame_id == 'base_link'
        assert (lane.header.stamp.sec, lane.header.stamp.nanosec) == (12, 345)
        assert lane.left_lane_detected and lane.right_lane_detected and lane.both_lanes_detected
        assert lane.vehicle_offset_m == pytest.approx(want.offset, abs=1e-4)
        assert lane.lane_width_m == pytest.approx(want.width, abs=1e-4)
        assert lane.left_poly_c == pytest.approx(want.left.c, abs=1e-4)
        assert lane.right_poly_b == pytest.approx(want.right.b, abs=1e-4)
        assert lane.left_confidence == pytest.approx(want.left_confidence, abs=1e-4)
        assert lane.mean_curvature_m == pytest.approx(want.mean_radius, rel=1e-4)
        assert lane.curve_direction == want.curve_direction == 1.0
        assert lane.steering_hint == pytest.approx(want.steering_hint, abs=1e-4)
        assert not (none.left_lane_detected or none.right_lane_detected)
        assert none.vehicle_offset_m == 0.0
        assert math.isclose(none.mean_curvature_m, 9999.0)
        assert debug, 'no debug picture'
        assert (debug[-1].width, debug[-1].height, debug[-1].encoding) == (320, 480, 'bgr8')
        assert len(debug[-1].data) == 320 * 480 * 3
        assert np.frombuffer(bytes(debug[-1].data), np.uint8).any()
    finally:
        rclpy.try_shutdown()
        os.killpg(node.pid, 2)
        try:
            node.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, 9)
            node.wait()


def spin(me, seconds):
    stop = time.monotonic() + seconds
    while time.monotonic() < stop:
        rclpy.spin_once(me, timeout_sec=0.05)
