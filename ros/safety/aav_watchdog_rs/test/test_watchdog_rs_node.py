"""
The Rust sensor watchdog process itself: one GPS input in; the lock, state, clear.

It starts the built binary on its own ROS domain, on this computer only, watching one
GPS topic, and plays: a steady stream (OK, no lock), silence (STALE: locked), the
stream back (OK), a 111 m jump (JUMP: locked, and still locked with the stream steady
again: it latches), then the clear service (OK). Then a clean exit on Ctrl-C.
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
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    import rclpy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

TOPIC = '/test/gps/fix'


def env():
    e = dict(os.environ, ROS_DOMAIN_ID=str(170 + os.getpid() % 30),
             ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: e[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return e


def binary():
    return os.path.join(get_package_prefix('aav_watchdog_rs'), 'lib', 'aav_watchdog_rs',
                        'sensor_watchdog')


def stop(node):
    if node.poll() is None:
        os.killpg(node.pid, signal.SIGINT)
        try:
            node.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            node.wait()


def test_stale_back_a_jump_that_latches_and_a_clear():
    node = subprocess.Popen(
        [binary(), '--ros-args', '-p', 'inputs:=[gps]', '-p', f'gps.topic:={TOPIC}',
         '-p', 'gps.kind:=gps', '-p', 'gps.max_age:=1.0', '-p', 'gps.stop:=true',
         '-p', 'gps.max_jump_speed:=5.0'],
        env=env(), start_new_session=True)
    rclpy.init()
    try:
        ros = rclpy.create_node('watchdog_rs_test')
        seen = {'lock': None, 'state': '', 'diag': []}
        ros.create_subscription(Bool, '/safety/sensor_watchdog/lock',
                                lambda m: seen.__setitem__('lock', m.data), 10)
        ros.create_subscription(String, '/safety/sensor_watchdog/state',
                                lambda m: seen.__setitem__('state', m.data), 10)
        ros.create_subscription(
            DiagnosticArray, '/diagnostics',
            lambda m: seen.__setitem__('diag', [s.name for s in m.status]), 10)
        pub = ros.create_publisher(NavSatFix, TOPIC, 10)
        clear = ros.create_client(Trigger, '/safety/sensor_watchdog/clear')
        where = {'lat': 45.38, 'send': True}

        def send():
            if where['send']:
                m = NavSatFix()
                m.header.stamp = ros.get_clock().now().to_msg()
                m.header.frame_id = 'gps_link'
                m.status.status = NavSatStatus.STATUS_FIX
                m.latitude, m.longitude = where['lat'], -75.69
                pub.publish(m)

        ros.create_timer(0.05, send)

        def wait_for(test, seconds, what):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(ros, timeout_sec=0.05)
                if test():
                    return
            raise AssertionError(f'{what}: lock {seen["lock"]}, state {seen["state"]!r}')

        wait_for(lambda: seen['lock'] is False and seen['state'] == 'OK', 20, 'OK at first')
        wait_for(lambda: any('gps' in n for n in seen['diag']), 5, 'diagnostics')

        where['send'] = False
        wait_for(lambda: seen['lock'] is True and 'gps STALE' in seen['state'], 5, 'stale')
        assert seen['state'].startswith('STOP: ')
        where['send'] = True
        wait_for(lambda: seen['lock'] is False and seen['state'] == 'OK', 5, 'back')

        where['lat'] += 0.001
        wait_for(lambda: seen['lock'] is True and 'gps JUMP' in seen['state'], 5, 'jump')
        # Steady at the new place: a jump latches until a person clears it.
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(ros, timeout_sec=0.05)
        assert seen['lock'] is True and 'gps JUMP' in seen['state'], seen['state']

        assert clear.wait_for_service(timeout_sec=5)
        future = clear.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(ros, future, timeout_sec=5)
        assert future.result() is not None and future.result().success
        wait_for(lambda: seen['lock'] is False and seen['state'] == 'OK', 5, 'cleared')
        ros.destroy_node()
    finally:
        rclpy.shutdown()
        stop(node)
    assert node.returncode in (0, -signal.SIGINT), node.returncode
