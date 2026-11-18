"""
goal_leash: cancel a laptop driver's goals when the laptop goes quiet (TODO 116).

Runs on the car, next to the mux. The language driver (aav_language language_drive) runs
on a laptop and sends goals to drivers on the car; a ROS action carries on when its
client vanishes. While it runs, it beats; when the beats stop for `timeout` s, this
cancels every goal on `actions` once, so the car slows to a stop the driver's own way
(not an e-stop). A goodbye beat (0: it closed cleanly, cancelling its own goals) disarms
it. Nothing happens until a first beat: with no laptop driver, it never acts.

Subscribes  heartbeat_topic   std_msgs/UInt64 (/drivers/language/heartbeat), reliable:
                              1, 2, 3... at 5 Hz while language_drive runs; 0 = goodbye
Calls       <action>/_action/cancel_goal   action_msgs/srv/CancelGoal, for each of
                              `actions` (/scripted_drive, /follow_gps_waypoints): an
                              empty goal id and a zero stamp, ROS's "cancel every goal"

Parameters  heartbeat_topic, actions, timeout (s, 1.0), rate_hz (10). The rule:
            leash.py.
"""

import signal
import time

from aav_safety.leash import Leash
from action_msgs.srv import CancelGoal
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt64


class GoalLeash(Node):

    def __init__(self, **kwargs):
        # kwargs: Node's own (context, parameter_overrides), for test_goal_leash_node.py.
        super().__init__('goal_leash', **kwargs)
        p = self.declare_parameter
        topic = p('heartbeat_topic', '/drivers/language/heartbeat').value
        self.actions = list(p('actions', ['/scripted_drive', '/follow_gps_waypoints']).value)
        self.leash = Leash(timeout=float(p('timeout', 1.0).value))
        rate = float(p('rate_hz', 10.0).value)
        self.cancels = {a: self.create_client(CancelGoal, f'{a}/_action/cancel_goal')
                        for a in self.actions}
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(UInt64, topic, self.on_beat, reliable)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'watching {topic}: {self.leash.timeout:g} s quiet cancels the goals on '
            f'{", ".join(self.actions)}')

    def on_beat(self, msg: UInt64):
        # The steady clock: a laptop's stamps and this machine's clock may differ.
        was = self.leash.armed
        self.leash.beat(msg.data, time.monotonic())
        if self.leash.armed and not was:
            self.get_logger().info('a laptop driver is beating: its goals are on the leash')
        elif was and not self.leash.armed:
            self.get_logger().info('the laptop driver said goodbye')

    def tick(self):
        if not self.leash.check(time.monotonic()):
            return
        self.get_logger().error(
            f'the laptop driver went quiet for {self.leash.timeout:g} s: cancelling the '
            f'goals on {", ".join(self.actions)}; the car slows to a stop')
        for name, client in self.cancels.items():
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())
            else:
                self.get_logger().warning(f'{name}: no action server to cancel')


def main(args=None):
    rclpy.init(args=args)
    node = GoalLeash()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
