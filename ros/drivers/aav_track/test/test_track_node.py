"""
The track node with ROS: FollowPath goals in, cmd_vel_nav out, on a pretend car.

Runs only where the workspace is built (colcon test), on a free domain. The pretend car
is the plain bicycle model, moved by the node's own commands, and published as
/localization/odom/local. A path in odom needs no TF.
"""

import math
import os
import time

import pytest

try:
    from aav_io_mux.domains import free_domain
    from aav_track.track_node import Track
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    HAVE_ROS = True
except ImportError:      # tools/test.sh, outside pixi's ROS
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='needs ROS 2 (pixi run test)')


class Car:
    """Moves by the node's commands; publishes its odometry."""

    def __init__(self, node, wheelbase=1.57):
        self.node, self.L = node, wheelbase
        self.x = self.y = self.yaw = self.v = self.w = 0.0
        self.commands = []
        self.last = None
        self.pub = node.create_publisher(Odometry, '/localization/odom/local', 10)
        node.create_subscription(TwistStamped, '/cmd_vel_nav', self.on_cmd, 10)

    def on_cmd(self, msg):
        self.v, self.w = msg.twist.linear.x, msg.twist.angular.z
        self.commands.append((self.v, self.w))

    def step(self):
        # By the wall clock, as the node's timer runs: spin_once returns early when busy.
        now = time.monotonic()
        dt, self.last = (0.0 if self.last is None else now - self.last), now
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = self.v, self.w
        self.pub.publish(msg)


def goal(length, y=0.0):
    g = FollowPath.Goal()
    g.path.header.frame_id = 'odom'
    for i in range(int(length / 0.1) + 1):
        p = PoseStamped()
        p.header.frame_id = 'odom'
        p.pose.position.x, p.pose.position.y = i * 0.1, y
        p.pose.orientation.w = 1.0
        g.path.poses.append(p)
    return g


@pytest.fixture(params=[False])
def ros(request):
    # This computer only, on a domain nobody else is using (CLAUDE.md: shared machines).
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    rclpy.init(domain_id=free_domain())
    overrides = [Parameter('live_swap', value=True)] if request.param else []
    node = Track(parameter_overrides=overrides)
    helper = rclpy.create_node('track_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    car = Car(helper)
    client = ActionClient(helper, FollowPath, '/follow_path')

    def run(until, seconds=30.0, dt=0.02):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not until():
            executor.spin_once(timeout_sec=dt)
            car.step()
        return until()
    try:
        assert run(lambda: client.server_is_ready(), 10.0)
        yield node, car, client, run
    finally:
        node.destroy_node()
        helper.destroy_node()
        rclpy.try_shutdown()


def send(client, run, g):
    future = client.send_goal_async(g)
    assert run(future.done, 5.0) and future.result().accepted
    return future.result()


def test_follows_a_path_to_its_end_and_succeeds(ros):
    _, car, client, run = ros
    handle = send(client, run, goal(8.0, y=0.5))
    result = handle.get_result_async()
    assert run(result.done, 40.0)
    assert result.result().status == 4                 # SUCCEEDED
    assert car.x > 7.0 and abs(car.y - 0.5) < 0.3      # within the goal's 1 m
    assert car.commands[-1] == (0.0, 0.0)              # and it stopped
    assert max(v for v, _ in car.commands) <= 1.0 + 1e-6


def test_a_newer_path_replaces_the_last_without_stopping(ros):
    _, car, client, run = ros
    first = send(client, run, goal(30.0))
    run(lambda: car.v > 0.5, 10.0)
    second = send(client, run, goal(30.0, y=0.2))
    old = first.get_result_async()
    assert run(old.done, 5.0)
    assert old.result().status == 6                    # ABORTED: replaced
    assert car.v > 0.4                                 # no stop between
    cancel = second.cancel_goal_async()
    assert run(cancel.done, 5.0)
    assert run(lambda: car.v == 0.0, 5.0)


def test_an_empty_path_is_refused(ros):
    _, _, client, run = ros
    handle = send(client, run, FollowPath.Goal())
    result = handle.get_result_async()
    assert run(result.done, 5.0)
    assert result.result().result.error_code == FollowPath.Result.INVALID_PATH


def test_the_tracker_is_not_swapped_live_unless_asked(ros):
    node, _, _, _ = ros
    result = node.set_parameters([Parameter('tracker', value='lqr')])[0]
    assert not result.successful and 'live_swap' in result.reason
    assert node.tracker.name == 'pure_pursuit'


@pytest.mark.parametrize('ros', [True], indirect=True)
def test_a_live_swap_mid_path_carries_on(ros):
    node, car, client, run = ros
    handle = send(client, run, goal(30.0))
    assert run(lambda: car.v > 0.5, 10.0)
    hint = node.job.hint
    result = node.set_parameters([Parameter('tracker', value='lqr')])[0]
    assert result.successful
    assert node.tracker.name == 'lqr'
    assert node.tracker.path is node.job.path and node.tracker.hint == hint
    assert node.tracker.lag.angle == node.steer
    bad = node.set_parameters([Parameter('tracker', value='mppi')])[0]
    assert not bad.successful and node.tracker.name == 'lqr'
    # It keeps driving, with no stop, on the new tracker.
    run(lambda: False, 1.0)
    assert car.v > 0.4
    cancel = handle.cancel_goal_async()
    assert run(cancel.done, 5.0)
