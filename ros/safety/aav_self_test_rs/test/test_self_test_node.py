"""
The boot self-test's process: NOT READY, then READY as every input speaks, then NOT READY.

It starts the built binary on its own ROS domain, on this computer only, checking the
watchdog, the geofence, the actuators (/vehicle/status) and the IMU's clock, and plays
them: nothing yet (NOT READY, every check named), all of them (READY after settling),
then the geofence quiet (NOT READY, saying so). Then a clean exit on Ctrl-C.
Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
import signal
import subprocess
import time

import pytest

try:
    from aav_interfaces.msg import VehicleStatus
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticStatus
    import rclpy
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')


def env():
    e = dict(os.environ, ROS_DOMAIN_ID=str(140 + os.getpid() % 30),
             ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: e[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return e


def binary():
    return os.path.join(get_package_prefix('aav_self_test_rs'), 'lib', 'aav_self_test_rs',
                        'boot_self_test')


def stop(node):
    if node.poll() is None:
        os.killpg(node.pid, signal.SIGINT)
        try:
            node.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            node.wait()


def test_not_ready_then_ready_then_not_ready():
    node = subprocess.Popen(
        [binary(), '--ros-args', '-p', 'locks:=[watchdog, geofence]',
         '-p', 'clocks:=[imu]', '-p', 'settle:=1.0', '-p', 'rate_hz:=10.0'],
        env=env(), start_new_session=True)
    rclpy.init()
    try:
        ros = rclpy.create_node('self_test_test')
        seen = {'status': None}
        ros.create_subscription(DiagnosticStatus, '/safety/self_test/status',
                                lambda m: seen.__setitem__('status', m), 10)
        watchdog = ros.create_publisher(String, '/safety/sensor_watchdog/state', 10)
        geofence = ros.create_publisher(String, '/safety/geofence/state', 10)
        vehicle = ros.create_publisher(VehicleStatus, '/vehicle/status', 10)
        imu = ros.create_publisher(Imu, '/sensors/imu/data', 10)
        speak = {'all': False, 'geofence': True}

        def send():
            if not speak['all']:
                return
            watchdog.publish(String(data='OK'))
            if speak['geofence']:
                geofence.publish(String(data='LOST: no GPS yet'))
            vehicle.publish(VehicleStatus())
            m = Imu()
            m.header.stamp = ros.get_clock().now().to_msg()
            imu.publish(m)

        ros.create_timer(0.05, send)

        def wait_for(test, seconds, what):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(ros, timeout_sec=0.05)
                if seen['status'] is not None and test(seen['status']):
                    return
            s = seen['status']
            raise AssertionError(f'{what}: {s.message if s else "nothing heard"}')

        wait_for(lambda s: s.level == DiagnosticStatus.ERROR
                 and s.message.startswith('NOT READY - sensors'), 20, 'NOT READY at first')
        assert [v.key for v in seen['status'].values] == [
            'sensors', 'actuators', 'watchdog', 'geofence', 'clock imu']
        speak['all'] = True
        wait_for(lambda s: s.level == DiagnosticStatus.OK and s.message == 'READY', 10,
                 'READY')
        assert seen['status'].name == 'boot_self_test'
        speak['geofence'] = False
        wait_for(lambda s: s.level == DiagnosticStatus.ERROR and 'geofence' in s.message, 5,
                 'the geofence quiet')
        ros.destroy_node()
    finally:
        rclpy.shutdown()
        stop(node)
    assert node.returncode in (0, -signal.SIGINT), node.returncode
