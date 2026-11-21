"""
The Rust geofence, stop_hold and goal_leash processes themselves, each on its own domain.

geofence: the fence arrives (transient local); a steady pose and fixes on a remapped fix
topic (as fix_topic does): OK; the fixes stop: LOST; back: OK; the pose 45 m out:
OUTSIDE, a clear refused; back in: still OUTSIDE (latched) until a clear succeeds.
stop_hold: STOP locks; clear for clear_time lets go. goal_leash: beats, then silence,
cancels the goals once. Then a clean exit on Ctrl-C.
Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
import signal
import subprocess
import time

import pytest

try:
    from action_msgs.srv import CancelGoal
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PolygonStamped
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String, UInt64
    from std_srvs.srv import Trigger
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

FIX = '/test/fix'
MOTION = ['reaction_time:=0.5', 'braking:=1.5', 'max_accel:=1.0', 'max_speed:=2.0',
          'max_reverse_speed:=1.0', 'max_curvature:=0.4']


def env(offset):
    e = dict(os.environ, ROS_DOMAIN_ID=str(140 + offset + os.getpid() % 10),
             ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: e[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return e


def start(name, args, offset):
    exe = os.path.join(get_package_prefix('aav_safety_rs'), 'lib', 'aav_safety_rs', name)
    return subprocess.Popen([exe, '--ros-args'] + args, env=env(offset),
                            start_new_session=True)


def stop(node):
    if node.poll() is None:
        os.killpg(node.pid, signal.SIGINT)
        try:
            node.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            node.wait()
    assert node.returncode in (0, -signal.SIGINT), node.returncode


def waiter(ros, seen):
    def wait_for(test, seconds, what):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(ros, timeout_sec=0.05)
            if test():
                return
        raise AssertionError(f'{what}: {seen}')
    return wait_for


def test_geofence():
    args = [a for m in MOTION for a in ('-p', m)]
    node = start('geofence', args + ['-p', 'radius:=30.0', '-p', 'settle_time:=0.5',
                                     '-r', f'/localization/gps/odom:={FIX}'], 0)
    rclpy.init()
    try:
        ros = rclpy.create_node('geofence_rs_test')
        seen = {'lock': None, 'state': '', 'fence': 0}
        ros.create_subscription(Bool, '/safety/geofence/lock',
                                lambda m: seen.__setitem__('lock', m.data), 10)
        ros.create_subscription(String, '/safety/geofence/state',
                                lambda m: seen.__setitem__('state', m.data), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ros.create_subscription(PolygonStamped, '/safety/geofence/fence',
                                lambda m: seen.__setitem__('fence', len(m.polygon.points)),
                                latched)
        pose = ros.create_publisher(Odometry, '/localization/odom/global', 10)
        fix = ros.create_publisher(Odometry, FIX, 10)
        clear = ros.create_client(Trigger, '/safety/geofence/clear')
        where = {'x': 0.0, 'fix': True}

        def send():
            m = Odometry()
            m.header.frame_id = 'map'
            m.pose.pose.position.x = where['x']
            m.pose.pose.orientation.w = 1.0
            m.pose.covariance[0] = m.pose.covariance[7] = 0.01
            pose.publish(m)
            if where['fix']:
                fix.publish(Odometry())

        ros.create_timer(0.05, send)
        wait_for = waiter(ros, seen)

        def call():
            assert clear.wait_for_service(timeout_sec=5)
            future = clear.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(ros, future, timeout_sec=5)
            return future.result()

        wait_for(lambda: seen['fence'] == 36, 20, 'the fence')
        wait_for(lambda: seen['lock'] is False and seen['state'].startswith('OK: '), 20, 'OK')
        where['fix'] = False
        wait_for(lambda: seen['lock'] and seen['state'].startswith('LOST: no GPS'), 5, 'LOST')
        where['fix'] = True
        wait_for(lambda: seen['lock'] is False and seen['state'].startswith('OK: '), 5, 'back')
        where['x'] = 45.0
        wait_for(lambda: seen['lock'] and seen['state'].startswith('OUTSIDE: '), 5, 'OUTSIDE')
        answer = call()
        assert not answer.success and 'outside the fence' in answer.message, answer.message
        where['x'] = 0.0
        wait_for(lambda: 'inside: a person clears it' in seen['state'], 5, 'latched inside')
        assert seen['lock'] is True
        answer = call()
        assert answer.success and answer.message.startswith('cleared OUTSIDE'), answer.message
        wait_for(lambda: seen['lock'] is False and seen['state'].startswith('OK: '), 5, 'cleared')
        ros.destroy_node()
    finally:
        rclpy.shutdown()
        stop(node)


def test_stop_hold():
    node = start('stop_hold', ['-p', 'clear_time:=0.5'], 1)
    rclpy.init()
    try:
        ros = rclpy.create_node('stop_hold_rs_test')
        seen = {'lock': None}
        ros.create_subscription(Bool, '/safety/stop_hold/lock',
                                lambda m: seen.__setitem__('lock', m.data), 10)
        zone = ros.create_publisher(CollisionMonitorState, '/safety/stop_zone/state', 10)
        wait_for = waiter(ros, seen)
        wait_for(lambda: seen['lock'] is False, 20, 'open at first')
        stop_now = CollisionMonitorState(action_type=CollisionMonitorState.STOP)
        wait_for(lambda: zone.publish(stop_now) or seen['lock'] is True, 5, 'held')
        zone.publish(CollisionMonitorState(action_type=CollisionMonitorState.DO_NOTHING))
        started = time.monotonic()
        wait_for(lambda: seen['lock'] is False, 5, 'let go')
        assert time.monotonic() - started >= 0.4
        ros.destroy_node()
    finally:
        rclpy.shutdown()
        stop(node)


def test_goal_leash():
    node = start('goal_leash', ['-p', 'actions:=[/leash_test/drive]', '-p', 'timeout:=0.5'], 2)
    rclpy.init()
    try:
        ros = rclpy.create_node('leash_rs_test')
        cancels = []

        def cancel(request, response):
            cancels.append(request)
            return response

        ros.create_service(CancelGoal, '/leash_test/drive/_action/cancel_goal', cancel)
        beat = ros.create_publisher(UInt64, '/drivers/language/heartbeat',
                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        wait_for = waiter(ros, {'cancels': cancels})
        end = time.monotonic() + 3.0
        count = 0
        while time.monotonic() < end:
            count += 1
            beat.publish(UInt64(data=count))
            rclpy.spin_once(ros, timeout_sec=0.1)
        assert not cancels
        wait_for(lambda: len(cancels) == 1, 5, 'one cancel')
        assert cancels[0].goal_info.goal_id.uuid.tolist() == [0] * 16
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(ros, timeout_sec=0.1)
        assert len(cancels) == 1
        ros.destroy_node()
    finally:
        rclpy.shutdown()
        stop(node)
