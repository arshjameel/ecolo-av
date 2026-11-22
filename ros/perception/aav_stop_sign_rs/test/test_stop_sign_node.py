"""
The stop_sign_watcher process: a close stop sign locks, then lets go.

It starts the built node (the Rust binary; aav_detector's Python went on 22 Nov) on its
own ROS domain, on this computer only, with hold_s 1 s, gives it the front camera's lens
(CameraInfo) and plays the detector: a stop sign 100 px tall (2.57 m with that lens).
/perception/stop_sign/distance must say where the sign is (while CLEAR), the lock must go
on, and come off after hold_s while the same sign is still seen (PASSING ignores it).
Then a clean exit on Ctrl-C. Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
import signal
import subprocess
import time

import pytest

try:
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PointStamped
    import rclpy
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import Bool
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

FRAME = 'camera_front_left_link'
PACKAGES = {'rust': 'aav_stop_sign_rs'}


def stop(node):
    if node.poll() is None:
        os.killpg(node.pid, signal.SIGINT)
        try:
            return node.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            node.wait()
    return None


@pytest.mark.parametrize('impl', ['rust'])
def test_a_close_sign_locks_then_lets_go(impl):
    domain = 200 + (os.getpid() + (impl == 'python')) % 20
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    package = PACKAGES[impl]
    exe = os.path.join(get_package_prefix(package), 'lib', package, 'stop_sign_watcher')
    node = subprocess.Popen([exe, '--ros-args', '-p', 'hold_s:=1.0', '-p', 'ignore_s:=30.0'],
                            env=env, start_new_session=True)
    rclpy.init(domain_id=domain)
    try:
        ros = rclpy.create_node('stop_sign_test')
        seen = {'lock': [], 'point': None}
        ros.create_subscription(Bool, '/perception/stop_sign',
                                lambda m: seen['lock'].append((time.monotonic(), m.data)), 10)
        ros.create_subscription(PointStamped, '/perception/stop_sign/distance',
                                lambda m: seen.__setitem__('point', m), 10)
        info = ros.create_publisher(CameraInfo, '/sensors/camera/front_left/camera_info', 10)
        boxes = ros.create_publisher(Detection2DArray, '/perception/detections', 10)

        def send():
            lens = CameraInfo()
            lens.header.frame_id = FRAME
            lens.k = [343.0, 0.0, 320.0, 0.0, 343.0, 180.0, 0.0, 0.0, 1.0]
            info.publish(lens)
            msg = Detection2DArray()
            msg.header.frame_id = FRAME
            msg.header.stamp = ros.get_clock().now().to_msg()
            d = Detection2D()
            d.bbox.size_y = 100.0
            d.bbox.center.position.x = 354.3
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = 'stop sign'
            h.hypothesis.score = 0.9
            d.results.append(h)
            msg.detections.append(d)
            boxes.publish(msg)

        def wait_for(test, seconds, what):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(ros, timeout_sec=0.02)
                if test():
                    return
            raise AssertionError(f'{impl}: {what}: {seen["lock"][-3:]}')

        # Clear at first: the lock heartbeat says false.
        wait_for(lambda: any(not v for _, v in seen['lock']), 20, 'false at first')
        ros.create_timer(0.1, send)
        wait_for(lambda: seen['lock'] and seen['lock'][-1][1], 5, 'the lock on')
        on = time.monotonic()
        # The sign's place, from its height and column: 2.57 m ahead, 0.26 m right.
        wait_for(lambda: seen['point'] is not None, 2, "the sign's point")
        p = seen['point']
        assert p.header.frame_id == FRAME
        ahead = 343.0 * 0.75 / 100.0
        assert abs(p.point.x - ahead) < 1e-6
        assert abs(p.point.y + ahead * 34.3 / 343.0) < 1e-4
        # hold_s later, off, though the sign is still there (PASSING ignores it).
        wait_for(lambda: not seen['lock'][-1][1], 3, 'the lock off after hold_s')
        assert 0.8 < time.monotonic() - on < 2.0
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(ros, timeout_sec=0.02)
        assert not seen['lock'][-1][1]
        ros.destroy_node()
    finally:
        rclpy.shutdown()
        code = stop(node)
    assert code in (0, -signal.SIGINT, None), f'{impl}: exit {code}'
