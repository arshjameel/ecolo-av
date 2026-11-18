"""
output_mux: passes ego's commands to one target, only while every source is that target's.

Subscribes  ego/actuator_cmd          ackermann_msgs/AckermannDriveStamped (ego, 50 Hz)
            system/sensor_mux/state   aav_interfaces/SourceCheck (the sensor mux, 2 Hz)
            system/source             aav_interfaces/DataSource, latched
            hud/<screen>/heartbeat    std_msgs/UInt64: only for each of `screens`
Publishes   control/<target>/actuator_cmd  ackermann_msgs/AckermannDriveStamped: ego's
                                      command, unchanged, while the gate is open
            control/<target>/heartbeat  aav_interfaces/Heartbeat: one for each command passed
                                      on, counter +1 (a byte), so it moves only while OPEN.
                                      The Motor Pi relays it to the ESP32 (TODO 50)
            safety/output_mux/state   std_msgs/String, 2 Hz and on each change: OPEN, WAIT
                                      or STOP, the target, and why
Serves      safety/output_mux/clear   std_srvs/Trigger: a person reopens a latched stop

Parameters  target (car; sim, turtle, carla or bench), settle (2.0 s), stale_after
            (3.0 s), screens (comma-separated names, empty: none; refused for car)

The target decides the source its data must come from: car REAL; sim, turtle and
carla SIM; bench BENCH (provenance.TARGETS). Each backend reads only its own target's
topic, so a simulator's ego can never reach the real car's backend.

The gate (provenance.Gate) opens once the sensor mux says ok for that source, nothing on
/system/source says otherwise, and both have held for `settle` s. Closed, nothing goes
out, and each backend's own timeout brakes the car (spine_bridge 0.1 s, then the spine's
100 ms watchdog; legacy_uart_bridge 0.3 s; Gazebo 0.2 s).

Evidence latches the stop: an announcement of another source, or the sensor mux's
problems. It stays stopped until a person calls /safety/output_mux/clear, which works
only once the evidence has gone. Doubt only holds it shut, and it reopens by itself after
`settle` s of all clear: a sensor mux not heard yet or quiet for `stale_after` s, its
doubts, or a node on /system/source whose announcement hasn't arrived yet. An
announcement heard before its node shows in the graph counts at once
(provenance.Announcers). Its clock is a steady one, not simulated time. This node's
work, without ROS, is muxes.OutputMuxLogic.
"""

import signal
import time

from aav_interfaces.msg import DataSource, Heartbeat, SourceCheck
from aav_io_mux import heartbeat, muxes, provenance
from aav_io_mux.sensor_mux import ANNOUNCEMENTS, announcers_alive, entries_of
from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger


class OutputMux(Node):

    def __init__(self):
        super().__init__('output_mux')
        self.target = self.declare_parameter('target', 'car').value
        topic = provenance.command_topic(self.target)
        self.logic = muxes.OutputMuxLogic(
            self.target, settle=self.declare_parameter('settle', 2.0).value,
            stale_after=self.declare_parameter('stale_after', 3.0).value,
            screens=muxes.screen_names(self.declare_parameter('screens', '').value))
        self.out = self.create_publisher(AckermannDriveStamped, topic, 10)
        self.state_pub = self.create_publisher(String, 'safety/output_mux/state', 10)
        self.heartbeat = self.create_publisher(Heartbeat, heartbeat.heartbeat_topic(self.target),
                                               10)
        self.create_subscription(AckermannDriveStamped, 'ego/actuator_cmd', self.on_cmd, 10)
        self.create_subscription(SourceCheck, 'system/sensor_mux/state', self.on_check, 10)
        self.create_subscription(DataSource, 'system/source', self.on_source, ANNOUNCEMENTS)
        for screen in self.logic.gate.screens:
            self.create_subscription(
                UInt64, f'hud/{screen}/heartbeat',
                lambda _msg, name=screen: self.logic.beat(name, time.monotonic()), 10)
        self.create_service(Trigger, 'safety/output_mux/clear', self.on_clear)
        # Little work beside the commands, so a command never waits long behind it: the
        # graph (who still announces) twice a second, and the state with it.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.5, self.tick_and_publish, clock=steady)
        self.get_logger().info(f'target:={self.target}: ego/actuator_cmd -> {topic}, while '
                               f'every source is '
                               f'{provenance.kind_name(self.logic.gate.expected)}')

    def on_check(self, msg):
        self.say(self.logic.check(time.monotonic(), announcers_alive(self), msg.source,
                                  msg.ok, msg.problems, msg.doubts))

    def on_source(self, msg):
        self.say(self.logic.source(time.monotonic(), announcers_alive(self), msg.announcer,
                                   entries_of(msg)))

    def tick_and_publish(self):
        self.say(self.logic.timer(time.monotonic(), announcers_alive(self)))

    def on_cmd(self, msg):
        counter = self.logic.command(time.monotonic())
        if counter is not None:
            self.out.publish(msg)
            self.heartbeat.publish(Heartbeat(header=msg.header, counter=counter))

    def on_clear(self, request, response):
        response.success, response.message, state = self.logic.clear(
            time.monotonic(), announcers_alive(self))
        # Two call sites: rclpy refuses one that logs at two severities.
        if response.success:
            self.get_logger().warning(f'clear: {response.message}')
        else:
            self.get_logger().error(f'clear: {response.message}')
        self.say(state)
        return response

    def say(self, published):
        """Publish the state line muxes.OutputMuxLogic gives, and log a change."""
        if published is None:
            return
        line, log = published
        if log is not None:
            if log[0] == 'error':
                self.get_logger().error(log[1])
            else:
                self.get_logger().info(log[1])
        self.state_pub.publish(String(data=line))


def main(args=None):
    rclpy.init(args=args)
    node = OutputMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C reaches a launched node twice (terminal and launch), and Jazzy can then
        # raise RCLError ("context is not valid"). Only a live context means a real error.
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
