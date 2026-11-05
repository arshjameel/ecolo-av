"""
joy_alive: pass the joystick on to teleop_twist_joy only while it is really there and moving.

Runs in the namespace /drivers/joy (aav_teleop teleop.launch.py), between joy_node and
teleop_twist_joy_node.

Subscribes  joy           sensor_msgs/Joy (/drivers/joy/joy): joy_node, 20 Hz while the
                          receiver is in, the last state again and again
Publishes   joy_checked   sensor_msgs/Joy (/drivers/joy/joy_checked): each message as it
                          came, or, while stopped, a released copy (every axis and button
                          0), so teleop_twist_joy sends one zero and then nothing; once more
                          the moment silence trips it
            alive         std_msgs/String (/drivers/joy/alive), at rate_hz: one line for
                          people, OK, WAITING, SILENT or FROZEN, a dash, and why

Parameters  enable_button (0), motion_axes ([1]), motion_threshold (0.05),
            silent_after (s, 0.5), frozen_after (s, 2.0; 0 off), rate_hz (10):
            aav_teleop config/joy_alive.yaml. The rules are in joy_alive.py.

Why: joy_node repeats the last state while the USB receiver is plugged in, so a pad turned
off or out of range could keep the car driving (TODO 74a; docs/car-day.md, A9b).
"""

import signal
import time

from aav_safety.joy_alive import JoyAlive, OK
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoyAliveNode(Node):

    def __init__(self):
        super().__init__('joy_alive')
        p = self.declare_parameter
        self.check = JoyAlive(
            enable_button=p('enable_button', 0).value,
            motion_axes=tuple(p('motion_axes', [1]).value),
            motion_threshold=p('motion_threshold', 0.05).value,
            silent_after=p('silent_after', 0.5).value,
            frozen_after=p('frozen_after', 2.0).value)
        rate = p('rate_hz', 10.0).value
        self.out = self.create_publisher(Joy, 'joy_checked', 10)
        self.state = self.create_publisher(String, 'alive', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.was = ''
        self.create_timer(1.0 / rate, self.tick)

    def released(self):
        axes, buttons = self.check.sizes
        msg = Joy(axes=[0.0] * axes, buttons=[0] * buttons)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joy'
        return msg

    def on_joy(self, msg: Joy):
        # The steady clock: joy_node stamps each repeat afresh, so stamps prove nothing.
        ok = self.check.on_joy(time.monotonic(), msg.axes, msg.buttons)
        self.out.publish(msg if ok else self.released())
        self.report(publish=False)

    def tick(self):
        if self.check.tick(time.monotonic()) and self.check.sizes[1] > 0:
            self.out.publish(self.released())
        self.report()

    def report(self, publish=True):
        line = self.check.state(time.monotonic())
        if publish:
            self.state.publish(String(data=line))
        name = line.split(' ')[0]
        if name != self.was:
            # Two calls: rclpy refuses one call site that logs at two severities.
            if name == OK:
                self.get_logger().info(f'the joystick: {line}')
            else:
                self.get_logger().error(f'the joystick: {line}')
            self.was = name


def main(args=None):
    rclpy.init(args=args)
    node = JoyAliveNode()
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
