"""
The Rust gps_heading process itself: wheels and a GPS velocity in, a heading out.

It starts the built binary on its own ROS domain, on this computer only: driving north
at 2 m/s it publishes a heading near pi/2 with the IMU's empty fields marked (-1); slower
than min_speed, nothing. Then a clean exit on Ctrl-C.
Needs the workspace built and sourced: `colcon test` runs it.
"""

import math
import os
import signal
import subprocess
import time

import pytest

try:
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from nav_msgs.msg import Odometry
    import rclpy
    from sensor_msgs.msg import Imu
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')


def test_a_heading_while_moving_and_none_when_slow():
    env = dict(os.environ, ROS_DOMAIN_ID=str(160 + os.getpid() % 10),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    exe = os.path.join(get_package_prefix('aav_gps_heading_rs'), 'lib', 'aav_gps_heading_rs',
                       'gps_heading')
    node = subprocess.Popen([exe], env=env, start_new_session=True)
    rclpy.init()
    try:
        ros = rclpy.create_node('gps_heading_rs_test')
        seen = []
        ros.create_subscription(Imu, '/localization/gps/heading', seen.append, 10)
        odom = ros.create_publisher(Odometry, '/vehicle/odom', 10)
        vel = ros.create_publisher(TwistWithCovarianceStamped, '/sensors/gps/fix_velocity', 10)
        speed = {'v': 2.0}

        def send():
            o = Odometry()
            o.twist.twist.linear.x = speed['v']
            odom.publish(o)
            t = TwistWithCovarianceStamped()
            t.header.stamp = ros.get_clock().now().to_msg()
            t.twist.twist.linear.y = speed['v']
            t.twist.covariance[0] = t.twist.covariance[7] = 0.0025
            vel.publish(t)

        ros.create_timer(0.1, send)
        end = time.monotonic() + 20
        while not seen and time.monotonic() < end:
            rclpy.spin_once(ros, timeout_sec=0.1)
        assert seen, 'no heading'
        imu = seen[-1]
        assert imu.header.frame_id == 'base_link'
        assert abs(2.0 * math.atan2(imu.orientation.z, imu.orientation.w) - math.pi / 2) < 0.05
        assert imu.angular_velocity_covariance[0] == -1.0
        assert imu.linear_acceleration_covariance[0] == -1.0
        speed['v'] = 0.1
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(ros, timeout_sec=0.1)
        seen.clear()
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(ros, timeout_sec=0.1)
        assert not seen, 'a heading while slower than min_speed'
        ros.destroy_node()
    finally:
        rclpy.shutdown()
        if node.poll() is None:
            os.killpg(node.pid, signal.SIGINT)
            try:
                node.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(node.pid, signal.SIGKILL)
                node.wait()
    assert node.returncode in (0, -signal.SIGINT), node.returncode
