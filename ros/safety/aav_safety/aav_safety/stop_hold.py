"""
stop_hold: once collision_monitor stops Nav2, hold the stop until the way stays clear.

Subscribes  safety/collision_monitor/state   nav2_msgs/CollisionMonitorState: Nav2's
                                             collision_monitor, each time its action
                                             changes (STOP, DO_NOTHING...)
Publishes   safety/stop_hold/lock            std_msgs/Bool, at rate_hz: true from a STOP
                                             until the monitor has said something else for
                                             clear_time s without a break (hold.py).
                                             twist_mux's stop_zone lock (priority 21): it
                                             holds Nav2 (20) and the AI slot (10) only. Its
                                             timeout is 0, so this node's silence locks
                                             nothing, and a node that dies while true keeps
                                             the stop

Parameters  clear_time (s, 1.0), rate_hz (10): config/stop_hold.yaml

Why: collision_monitor decides afresh on every command, so an obstacle at its stop zone's
edge flickers it between stop and go, and each go lets the car creep on (TODO, 11 Oct).
navigation.launch.py starts this node wherever it starts collision_monitor.
"""

import signal
import time

from aav_safety.hold import StopHold
from nav2_msgs.msg import CollisionMonitorState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class StopHoldNode(Node):

    def __init__(self):
        super().__init__('stop_hold')
        self.hold = StopHold(self.declare_parameter('clear_time', 1.0).value)
        rate = self.declare_parameter('rate_hz', 10.0).value
        self.lock = self.create_publisher(Bool, '/safety/stop_hold/lock', 10)
        self.create_subscription(CollisionMonitorState, '/safety/collision_monitor/state',
                                 self.on_state, 10)
        self.was = False
        self.create_timer(1.0 / rate, self.tick)

    def on_state(self, msg: CollisionMonitorState):
        # The steady clock: a hold must not end early if sim time jumps.
        self.hold.update(time.monotonic(), msg.action_type == CollisionMonitorState.STOP)
        self.tick()

    def tick(self):
        locked = self.hold.locked(time.monotonic())
        self.lock.publish(Bool(data=locked))
        if locked != self.was:
            self.get_logger().info('holding Nav2 after a stop' if locked else
                                   f'the stop zone stayed clear for {self.hold.clear_time} s: '
                                   'Nav2 may go')
            self.was = locked


def main(args=None):
    rclpy.init(args=args)
    node = StopHoldNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C reaches a launched node twice (the terminal, then launch): ignore the
        # second, as sensor_watchdog does.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
