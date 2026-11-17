"""
sensor_mux: checks that every sensor comes from the one source this run asked for.

Subscribes  system/source            aav_interfaces/DataSource, latched: who says what is
                                     REAL, SIM, BENCH or REPLAY
Publishes   system/sensor_mux/state  aav_interfaces/SourceCheck, 2 Hz: the verdict, read by
                                     the output mux

Parameters  source (real; sim, bench or replay), period (0.5 s), startup (2.0 s: silent
            while discovery settles, so the output mux waits rather than judging an
            unfinished graph), hold (5.0 s: how long a graph doubt lasts before it counts),
            grace (30.0 s from start: a missing announcement counts only after it)

It moves no data. Each launch starts only the chosen source's drivers or stand-ins on the
canonical topics (/sensors/..., /vehicle/odom...); this node checks, from the ROS graph,
that each of those topics has at most one publisher, and that /system/source says it
comes from `source` (provenance.check_sensors). So the pictures and point clouds cost
nothing extra. An announcement counts while its announcer is alive, and before the graph
shows it; a node on /system/source not heard yet is a doubt (provenance.Announcers).

An announcement of another source is a problem at once. What the graph suggests is a
doubt until it has lasted `hold` s (provenance.Hold): a node can be seen publishing
before its announcement arrives, or before its name is known. A doubt an announcement
would settle (a sensor nobody announces, an announcer not heard yet) also waits out the
start-up `grace`: on a busy computer, discovery has shown a simulator's sensors 10 s
before the node announcing them (TODO 234). Meanwhile the output mux holds WAIT, closed.
"""

import signal
import time

from aav_interfaces.msg import DataSource, SensorSource, SourceCheck
from aav_io_mux import provenance
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# /system/source: reliable and transient local, as the announcers publish it. Deep enough
# to keep every announcer's one message.
ANNOUNCEMENTS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
SOURCE_TOPIC = '/system/source'


def entries_of(msg):
    """Return a DataSource message's entries, for provenance."""
    return [provenance.Entry(msg.announcer, s.name, s.topic, s.source) for s in msg.sensors]


def announcers_alive(node):
    """Return the names of the nodes publishing /system/source now."""
    return {i.node_name for i in node.get_publishers_info_by_topic(SOURCE_TOPIC)}


class SensorMux(Node):

    def __init__(self):
        super().__init__('sensor_mux')
        source = self.declare_parameter('source', 'real').value
        self.expected = provenance.source_of(source)
        period = self.declare_parameter('period', 0.5).value
        self.ready_at = time.monotonic() + self.declare_parameter('startup', 2.0).value
        grace = self.declare_parameter('grace', 30.0).value
        self.hold = provenance.Hold(self.declare_parameter('hold', 5.0).value,
                                    grace_until=time.monotonic() + grace)
        self.announcers = provenance.Announcers()
        self.last_problems = None
        self.pub = self.create_publisher(SourceCheck, 'system/sensor_mux/state', 10)
        self.create_subscription(DataSource, 'system/source', self.on_source, ANNOUNCEMENTS)
        # A steady clock: the check runs on simulated time too, before /clock starts.
        self.create_timer(period, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'source:={source}: every sensor must be '
                               f'{provenance.kind_name(self.expected)}')

    def on_source(self, msg):
        self.announcers.hear(msg.announcer, entries_of(msg), time.monotonic())

    def tick(self):
        now = time.monotonic()
        if now < self.ready_at:
            return
        alive = announcers_alive(self)
        publishers = {topic: [i.node_name for i in self.get_publishers_info_by_topic(topic)]
                      for _, topic in provenance.SENSORS}
        found = provenance.check_sensors(
            self.expected, self.announcers.entries(alive, now), publishers,
            unheard=self.announcers.unheard(alive))
        grace = self.hold.in_grace(now, found)
        check = self.hold.judge(now, found)
        msg = SourceCheck(source=check.source, ok=check.ok, problems=list(check.problems),
                          doubts=list(check.doubts))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sensors = [SensorSource(name=s.name, topic=s.topic, source=s.kind)
                       for s in check.sensors]
        self.pub.publish(msg)
        verdict = (check.problems, check.doubts, grace)
        if verdict != self.last_problems:
            want = provenance.kind_name(self.expected)
            if check.problems:
                self.get_logger().error(f'NOT {want}: ' + '; '.join(check.problems))
            elif check.doubts and grace:
                left = self.hold.grace_until - now
                self.get_logger().warning(
                    f'unsure, waiting for /system/source (the output mux holds WAIT; '
                    f'a missing announcement counts in {left:.0f} s): '
                    + '; '.join(check.doubts))
            elif check.doubts:
                self.get_logger().warning(f'unsure (for {self.hold.hold:g} s): '
                                          + '; '.join(check.doubts))
            else:
                found = ', '.join(f'{s.name} {provenance.kind_name(s.kind)}'
                                  for s in check.sensors) or 'no sensors yet'
                self.get_logger().info(f'all {want}: {found}')
            self.last_problems = verdict


def main(args=None):
    rclpy.init(args=args)
    node = SensorMux()
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
