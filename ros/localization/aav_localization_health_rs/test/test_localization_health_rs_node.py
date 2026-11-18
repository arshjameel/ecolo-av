"""
The Rust localization_health process itself: a parked car in; the health line out.

It starts the built binary on its own ROS domain, on this computer only, and plays a
parked car (the IMU level, the wheels at 0, ekf_local, and TF map -> odom -> base_link ->
imu_link, gps_link): OK. Then the IMU upside down (imu_mounting WARN), then the IMU quiet
(STALE). /diagnostics has one status per check. Then a clean exit on Ctrl-C.
Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
import signal
import subprocess
import time

import pytest

try:
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

G = 9.80665


def env():
    e = dict(os.environ, ROS_DOMAIN_ID=str(140 + os.getpid() % 30),
             ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: e[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return e


def binary():
    return os.path.join(get_package_prefix('aav_localization_health_rs'), 'lib',
                        'aav_localization_health_rs', 'localization_health')


def stop(node):
    if node.poll() is None:
        os.killpg(node.pid, signal.SIGINT)
        try:
            node.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            node.wait()


def link(ros, parent, child):
    t = TransformStamped()
    t.header.stamp = ros.get_clock().now().to_msg()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.rotation.w = 1.0
    return t


def test_parked_ok_then_upside_down_then_stale():
    node = subprocess.Popen(
        [binary(), '--ros-args', '-p', 'rate_hz:=5.0', '-p', 'still_min:=1.0',
         '-p', 'still_window:=1.0', '-p', 'stale_after:=1.0'],
        env=env(), start_new_session=True)
    rclpy.init()
    try:
        ros = rclpy.create_node('localization_health_rs_test')
        seen = {'health': '', 'diag': []}
        ros.create_subscription(String, '/localization/health',
                                lambda m: seen.__setitem__('health', m.data), 10)
        ros.create_subscription(
            DiagnosticArray, '/diagnostics',
            lambda m: seen.__setitem__('diag', [s.name for s in m.status]), 10)
        imu_pub = ros.create_publisher(Imu, '/sensors/imu/data', 50)
        odom_pub = ros.create_publisher(Odometry, '/vehicle/odom', 50)
        local_pub = ros.create_publisher(Odometry, '/localization/odom/local', 10)
        tf_pub = ros.create_publisher(TFMessage, '/tf', 100)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        static_pub = ros.create_publisher(TFMessage, '/tf_static', latched)
        static_pub.publish(TFMessage(transforms=[link(ros, 'base_link', 'imu_link'),
                                                 link(ros, 'base_link', 'gps_link')]))
        car = {'gz': G, 'imu': True}

        def send():
            now = ros.get_clock().now().to_msg()
            if car['imu']:
                m = Imu()
                m.header.stamp, m.header.frame_id = now, 'imu_link'
                m.linear_acceleration.z = car['gz']
                imu_pub.publish(m)
            for pub in (odom_pub, local_pub):
                o = Odometry()
                o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_link'
                pub.publish(o)
            tf_pub.publish(TFMessage(transforms=[link(ros, 'map', 'odom'),
                                                 link(ros, 'odom', 'base_link')]))

        ros.create_timer(0.02, send)

        def wait_for(test, seconds, what):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(ros, timeout_sec=0.05)
                if test():
                    return
            raise AssertionError(f'{what}: health {seen["health"]!r}')

        wait_for(lambda: seen['health'] == 'OK', 20, 'OK when parked')
        wait_for(lambda: len(seen['diag']) == 8, 5, 'diagnostics')
        assert 'localization_health: gyro_bias' in seen['diag']

        car['gz'] = -G
        wait_for(lambda: 'imu_mounting WARN: upside down' in seen['health'], 10,
                 'upside down')
        car['imu'] = False
        wait_for(lambda: 'gyro_bias STALE: nothing on imu' in seen['health'], 10, 'stale')
        ros.destroy_node()
    finally:
        rclpy.shutdown()
        stop(node)
    assert node.returncode == 0, node.returncode
