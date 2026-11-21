"""
The pose_check node itself: NDT fits and GPS fixes in, the passed poses and its status out.

It starts the built node on its own ROS domain, on this computer only (wall clock). While
NDT tracks, NDT's fits pass to ekf_global and the GPS's don't; once NDT says LOST, the
GPS's pass, NDT is seeded from the GPS, and the status says GPS. Needs the workspace
built and sourced: `colcon test` runs it.
"""

import os
import signal
import subprocess
import time

import pytest

try:
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    import rclpy
    from sensor_msgs.msg import NavSatFix, NavSatStatus
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')


def stamped(msg, me, x, y, var):
    msg.header.stamp = me.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = msg.pose.covariance[7] = var
    msg.pose.covariance[35] = 0.01
    return msg


def ndt_status(word):
    return DiagnosticStatus(name='ndt', message=word,
                            values=[KeyValue(key='score_per_point', value='3.1')])


def test_ndt_leads_then_the_gps_takes_over_when_ndt_is_lost():
    env = dict(os.environ, ROS_DOMAIN_ID=str(180 + os.getpid() % 40),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    binary = os.path.join(get_package_prefix('aav_pose_check'), 'lib', 'aav_pose_check',
                          'pose_check')
    node = subprocess.Popen([binary], env=env, start_new_session=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('pose_check_test')
        got = {'ndt': [], 'gps': [], 'fix': [], 'seed': [], 'status': []}
        for key, kind, topic in (
                ('ndt', PoseWithCovarianceStamped, '/localization/pose_check/ndt_pose'),
                ('gps', Odometry, '/localization/pose_check/gps_odom'),
                ('fix', Odometry, '/localization/pose_check/fix'),
                ('seed', PoseWithCovarianceStamped, '/localization/ndt/initial_pose'),
                ('status', DiagnosticStatus, '/localization/pose_check/status')):
            me.create_subscription(kind, topic, got[key].append, 10)
        ndt = me.create_publisher(PoseWithCovarianceStamped, '/localization/ndt/pose', 10)
        status = me.create_publisher(DiagnosticStatus, '/localization/ndt/status', 10)
        gps = me.create_publisher(Odometry, '/localization/gps/odom', 10)
        fix = me.create_publisher(NavSatFix, '/sensors/gps/fix', 10)
        fused = me.create_publisher(Odometry, '/localization/odom/global', 10)

        def run(seconds, lost):
            end = time.monotonic() + seconds
            k = 0
            while time.monotonic() < end and node.poll() is None:
                status.publish(ndt_status('LOST' if lost else 'TRACKING'))
                if not lost:
                    ndt.publish(stamped(PoseWithCovarianceStamped(), me, 1.0, 2.0, 1e-4))
                fused.publish(stamped(Odometry(), me, 1.0, 2.0, 0.01))
                if k % 2 == 0:
                    fix.publish(NavSatFix(latitude=45.0, longitude=-75.0, status=NavSatStatus(
                        status=NavSatStatus.STATUS_FIX)))
                    gps.publish(stamped(Odometry(), me, 1.2, 1.9, 0.25))
                k += 1
                for _ in range(2):
                    rclpy.spin_once(me, timeout_sec=0.05)

        # Until the node hears us and we hear it (discovery can take seconds).
        end = time.monotonic() + 20.0
        while time.monotonic() < end and node.poll() is None and not (
                ndt.get_subscription_count() and got['status']):
            rclpy.spin_once(me, timeout_sec=0.1)

        def drain(seconds=0.5):
            # Each pose passed is also on fix, a separate topic: let both queues arrive
            # before counting (21 Nov: ARM's CI counted 5358 poses and 5357 fixes).
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(me, timeout_sec=0.05)

        run(4.0, lost=False)
        drain()
        assert got['ndt'], 'no NDT fit passed: ' + (
            node.stdout.read().decode() if node.poll() is not None else '')
        assert not got['gps'], 'the GPS reached the EKF while NDT led'
        assert got['status'][-1].message.startswith('NDT'), got['status'][-1].message
        assert got['status'][-1].level == DiagnosticStatus.OK
        n_fix = len(got['fix'])
        # Each pose passed is also a fix; a queue of 10 may drop one or two of thousands
        # (CI on 21 Nov: 5590 fixes for 5591 poses, even after the drain).
        assert n_fix >= 0.99 * len(got['ndt'])
        run(3.0, lost=True)
        last = got['status'][-1]
        assert last.message.startswith('GPS: NDT lost'), last.message
        assert last.level == DiagnosticStatus.WARN
        assert got['gps'], 'the GPS did not take over'
        assert got['seed'], 'no seed for NDT'
        assert abs(got['seed'][0].pose.pose.position.x - 1.2) < 1e-6
        assert len(got['fix']) > n_fix
        me.destroy_node()
    finally:
        rclpy.try_shutdown()
        if node.poll() is None:
            os.killpg(node.pid, signal.SIGINT)
            try:
                node.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(node.pid, signal.SIGKILL)
