"""
The learned lane_finder itself: Ravensim's picture and camera_info in, a LaneGeometry out.

It starts the built node on its own ROS domain (200-219), on this computer only, with
aav_lane_finder's lane_finder.yaml and the car's front left camera (vehicle.yaml), plays
one of lane-core's Ravensim frames as rgb8, then a blank road. The lane must come back in
base_link with the picture's stamp, both lines on their own sides; the blank road, none.

Skips without the model (models/lane-ufld.onnx is outside git: tools/lane_train.py makes
it) and without ROS. Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
from pathlib import Path
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[4]
MODEL = Path(os.environ.get('AAV_LANE_MODEL', ROOT / 'models' / 'lane-ufld.onnx'))
FRAME = ROOT / 'ros' / 'perception' / 'aav_lane_finder_rs' / 'crates' / 'lane-core' / \
    'tests' / 'pictures' / 'ravensim_frame_00.png'
CONFIG = ROOT / 'ros' / 'perception' / 'aav_lane_finder' / 'config' / 'lane_finder.yaml'

try:
    from aav_lane_geometry.msg import LaneGeometry
    from ament_index_python.packages import get_package_prefix
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:
    rclpy = None

pytestmark = [
    pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test'),
    pytest.mark.skipif(not MODEL.exists(), reason=f'no {MODEL} (tools/lane_train.py)'),
]

CAMERA = [1.75, 0.29, 1.55]
FOCAL = 343.0


def environment():
    """Return this test's environment: its own domain, this computer only."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(200 + os.getpid() % 20),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return env


def read_png(path):
    """Return an RGB picture."""
    return cv2.imread(str(path))[:, :, ::-1].copy()


def image_msg(rgb):
    msg = Image(height=rgb.shape[0], width=rgb.shape[1], encoding='rgb8',
                step=3 * rgb.shape[1])
    msg.header.frame_id = 'camera_left_optical_frame'
    msg.header.stamp.sec, msg.header.stamp.nanosec = 12, 345
    msg.data = rgb.tobytes()
    return msg


def info_msg(width, height):
    msg = CameraInfo(width=width, height=height, distortion_model='plumb_bob')
    msg.k = [FOCAL, 0.0, width / 2.0, 0.0, FOCAL, height / 2.0, 0.0, 0.0, 1.0]
    msg.d = [0.0] * 5
    return msg


def spin(me, seconds):
    stop = time.monotonic() + seconds
    while time.monotonic() < stop:
        rclpy.spin_once(me, timeout_sec=0.05)


def test_ravensims_lane_and_a_blank_road(tmp_path):
    binary = os.path.join(get_package_prefix('aav_lane_finder_learned'), 'lib',
                          'aav_lane_finder_learned', 'lane_finder')
    ours = tmp_path / 'ours.yaml'
    ours.write_text('lane_finder:\n  ros__parameters:\n'
                    f'    camera_xyz: {CAMERA}\n    wheelbase: 1.57\n'
                    f'    model: {MODEL}\n    device: cpu\n')
    node = subprocess.Popen(
        [binary, '--ros-args', '--params-file', str(CONFIG), '--params-file', str(ours)],
        env=environment(), start_new_session=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('lane_finder_learned_node_test')
        got = []
        me.create_subscription(LaneGeometry, '/perception/lane/geometry', got.append, 10)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        images = me.create_publisher(Image, '/sensors/camera/front_left/image_raw', newest)
        infos = me.create_publisher(CameraInfo, '/sensors/camera/front_left/camera_info',
                                    newest)
        end = time.monotonic() + 60.0
        while (images.get_subscription_count() == 0 or infos.get_subscription_count() == 0) \
                and time.monotonic() < end and node.poll() is None:
            rclpy.spin_once(me, timeout_sec=0.1)
        frame = read_png(FRAME)
        blank = np.zeros_like(frame)
        blank[:180] = (170, 200, 230)
        blank[180:] = (90, 90, 92)
        for picture in (frame, blank):
            infos.publish(info_msg(picture.shape[1], picture.shape[0]))
            spin(me, 0.3)
            count = len(got)
            images.publish(image_msg(picture))
            stop = time.monotonic() + 20.0
            while len(got) == count and time.monotonic() < stop and node.poll() is None:
                rclpy.spin_once(me, timeout_sec=0.05)
            assert len(got) > count, 'no lane: ' + (
                node.stdout.read().decode() if node.poll() is not None else 'still running')
        lane, none = got[-2], got[-1]
        assert lane.header.frame_id == 'base_link'
        assert (lane.header.stamp.sec, lane.header.stamp.nanosec) == (12, 345)
        assert lane.left_lane_detected and lane.right_lane_detected
        assert lane.left_poly_c > 0.0 > lane.right_poly_c
        assert 2.5 <= lane.lane_width_m <= 5.0
        assert not (none.left_lane_detected or none.right_lane_detected)
    finally:
        rclpy.try_shutdown()
        os.killpg(node.pid, 2)
        try:
            node.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, 9)
            node.wait()
