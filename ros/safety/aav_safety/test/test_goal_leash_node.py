"""The goal leash node, in this process: a laptop driver that goes quiet loses its goals."""

import os
import time

import pytest

try:
    from aav_safety.goal_leash_node import GoalLeash
    from action_msgs.srv import CancelGoal
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import UInt64
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='needs ROS and the built workspace')


@pytest.fixture
def graph():
    """Start a private ROS context: the leash, and a pretend action server's cancel service."""
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0')) or 102
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=domain)
    leash = GoalLeash(context=context, parameter_overrides=[
        Parameter('actions', value=['/leash_test/drive']),
        Parameter('timeout', value=0.5)])
    other = rclpy.create_node('leash_test', context=context)
    cancels = []

    def cancel(request, response):
        cancels.append(request)
        return response

    other.create_service(CancelGoal, '/leash_test/drive/_action/cancel_goal', cancel)
    beat = other.create_publisher(UInt64, '/drivers/language/heartbeat',
                                  QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(leash)
    executor.add_node(other)
    yield beat, cancels, executor
    executor.shutdown()
    leash.destroy_node()
    other.destroy_node()
    rclpy.shutdown(context=context)


def spin(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.02)


def beat_for(beat, executor, seconds, start=1):
    for n in range(start, start + int(seconds / 0.1)):
        beat.publish(UInt64(data=n))
        spin(executor, 0.1)


def test_quiet_cancels_every_goal_once(graph):
    beat, cancels, executor = graph
    spin(executor, 1.0)                    # no beat yet: never acts
    assert cancels == []
    beat_for(beat, executor, 1.0)
    assert cancels == []
    spin(executor, 1.5)                    # quiet past 0.5 s
    assert len(cancels) == 1
    goal = cancels[0].goal_info
    assert bytes(goal.goal_id.uuid) == bytes(16)                  # every goal
    assert (goal.stamp.sec, goal.stamp.nanosec) == (0, 0)


def test_goodbye_cancels_nothing(graph):
    beat, cancels, executor = graph
    beat_for(beat, executor, 1.0)
    beat.publish(UInt64(data=0))
    spin(executor, 1.5)
    assert cancels == []
