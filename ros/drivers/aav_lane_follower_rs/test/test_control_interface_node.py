"""
control_interface itself, Rust and Python: lane messages in, the same commands out.

Each run starts the built node on its own ROS domain (180-199), on this computer only,
plays a lane with the car 0.3 m left of its centre at 10 Hz, then stops the messages. Both
languages must steer right at the lane-keeping speed, say LANE_KEEP with the same status
line, then slow down (NO_LANE_STOPPING) and stop. The Rust challengers must steer right
too. lane-follow-core's vectors check the numbers exactly; this checks the wiring: names,
topics, QoS, parameters and the timer. Needs the workspace built and sourced:
`colcon test` runs it.
"""

import os
import subprocess
import sys
import time

import pytest

try:
    from aav_lane_geometry.msg import LaneGeometry
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TwistStamped
    import rclpy
    from std_msgs.msg import String
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')


def environment(slot):
    """Return this run's environment: its own domain, this computer only."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(180 + (os.getpid() + slot) % 20),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return env


def command(impl, steering):
    if impl == 'rust':
        binary = os.path.join(get_package_prefix('aav_lane_follower_rs'), 'lib',
                              'aav_lane_follower_rs', 'control_interface')
        return [binary, '--ros-args', '-p', f'steering:={steering}']
    script = os.path.join(get_package_prefix('aav_lane_geometry'), 'lib',
                          'aav_lane_geometry', 'control_interface.py')
    return [sys.executable, script, '--ros-args']


def lane(offset):
    """Return a straight 3.5 m lane, the car `offset` m left of its centre, both lines seen."""
    msg = LaneGeometry(vehicle_offset_m=offset, mean_curvature_m=9999.0, lane_width_m=3.5,
                       left_lane_detected=True, right_lane_detected=True,
                       both_lanes_detected=True)
    msg.left_poly_c, msg.right_poly_c = 1.75 - offset, -1.75 - offset
    msg.header.frame_id = 'base_link'
    return msg


def spin(me, seconds):
    stop = time.monotonic() + seconds
    while time.monotonic() < stop:
        rclpy.spin_once(me, timeout_sec=0.02)


def drive(impl, steering, slot):
    """Return (commands, statuses, states) from one run of the node."""
    node = subprocess.Popen(command(impl, steering), env=environment(slot),
                            start_new_session=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('control_interface_test')
        cmds, statuses, states = [], [], []
        me.create_subscription(TwistStamped, '/drivers/ai/cmd_vel', cmds.append, 10)
        me.create_subscription(String, '/drivers/ai/lane_status',
                               lambda m: statuses.append(m.data), 10)
        me.create_subscription(String, '/drivers/ai/lane_state',
                               lambda m: states.append(m.data), 10)
        lanes = me.create_publisher(LaneGeometry, '/perception/lane/geometry', 10)
        end = time.monotonic() + 60.0
        while (lanes.get_subscription_count() == 0 or me.count_publishers(
                '/drivers/ai/lane_state') == 0) and time.monotonic() < end \
                and node.poll() is None:
            rclpy.spin_once(me, timeout_sec=0.1)
        assert node.poll() is None, node.stdout.read().decode()
        spin(me, 0.5)
        for _ in range(20):
            lanes.publish(lane(0.3))
            spin(me, 0.1)
        keeping = len(cmds)
        spin(me, 5.0)                       # no messages: 2 m/s down at 0.5 m/s², 4 s
        return cmds, keeping, statuses, states
    finally:
        rclpy.try_shutdown()
        os.killpg(node.pid, 2)
        try:
            node.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, 9)
            node.wait()


@pytest.mark.parametrize('impl,steering,slot', [
    ('rust', 'pid', 0), ('python', 'pid', 5), ('rust', 'pure_pursuit', 10),
    ('rust', 'stanley', 15)])
def test_keeps_the_lane_then_stops_when_the_messages_do(impl, steering, slot):
    cmds, keeping, statuses, states = drive(impl, steering, slot)
    assert keeping >= 15, f'{impl}: {keeping} commands while the lane came'
    kept = cmds[2:keeping]
    assert all(c.twist.linear.x == 2.0 for c in kept)
    assert all(c.twist.angular.z < 0.0 for c in kept), [c.twist.angular.z for c in kept]
    assert all(c.header.frame_id == 'base_link' for c in cmds)
    # No lane for stop_wait_s after the start: the watchdog has tripped, so it resumes.
    assert states[:3] == ['RESUMING', 'LANE_KEEP', 'LANE_KEEP'], states
    assert statuses[0] == ('[RESUMING] offset=+0.300m curv=9999.0m speed=2.00m/s '
                           'RESUMING 2.00/2.00')
    assert statuses[1] == ('[LANE_KEEP] offset=+0.300m curv=9999.0m speed=2.00m/s '
                           'LANE_KEEP')
    slowing = [c.twist.linear.x for c in cmds[keeping:]]
    assert slowing and slowing[-1] == 0.0, slowing
    assert all(b <= a for a, b in zip(slowing, slowing[1:])), slowing
