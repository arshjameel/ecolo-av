#!/usr/bin/env python3
"""
Measure every topic of a running system: how often it sends, how big, how many MB/s.

Like `ros2 topic hz` and `ros2 topic bw` for every topic at once, as one table, biggest
first. Run it beside Gazebo now, and on the car later (docs/bandwidth.md):

  pixi run measure bandwidth                    10 s, every topic, a table
  pixi run measure bandwidth --local            the same, for pixi run sim on this computer
  pixi run measure bandwidth --seconds 30 --markdown        a Markdown table, for the docs
  pixi run measure bandwidth --include /sensors --exclude compressed
  ROS_DOMAIN_ID=57 pixi run measure bandwidth --local   a sim on its own domain

It subscribes to each topic as raw bytes (no decoding, so it keeps up with cameras), with
the publishers' reliability (reliable only if all are), always volatile. The size
is the serialized message (CDR); on the network DDS adds a few dozen bytes a packet, and
cuts a big message into many UDP packets.

Measuring costs something too: a subscriber to every camera pulls every picture. On the
car, run it on the computer that already reads the big topics, or --exclude them.

Rates are per real second (the wall clock). Where /clock runs (Gazebo, a bag), it also
prints how fast simulated time ran: at 0.5 x real time, a 10 Hz Gazebo camera sends 5 a
second.

The numbers: tools/topic_load.py (no ROS; tests in tools/tests/test_topic_load.py).
"""

import argparse
import os
import re
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import topic_load  # noqa: E402

ME = 'bandwidth_meter'


def reliability(node, topic):
    """
    Return the reliability to subscribe with: reliable only if every publisher is.

    A best-effort subscriber hears both kinds; a reliable one hears only reliable
    publishers. With no publisher seen yet, best effort, so a late one still matches.
    """
    infos = node.get_publishers_info_by_topic(topic)
    if infos and all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos):
        return ReliabilityPolicy.RELIABLE
    return ReliabilityPolicy.BEST_EFFORT


def publishers(node, topic):
    """Return the names of the nodes publishing `topic`, without ours."""
    return sorted({i.node_name for i in node.get_publishers_info_by_topic(topic)} - {ME})


class Meters:
    """Subscribe to topics as they appear, and count their messages."""

    def __init__(self, node, include, exclude):
        self.node = node
        self.include = include
        self.exclude = exclude
        self.meters = {}
        self.types = {}
        self.skipped = {}
        self.subscriptions = {}
        self.clock = []          # [(wall time, raw /clock message)]: the first and the newest

    def wanted(self, topic):
        """Return True if the command line asks for this topic."""
        if self.include and not any(re.search(p, topic) for p in self.include):
            return False
        return not any(re.search(p, topic) for p in self.exclude)

    def discover(self):
        """Subscribe to every wanted topic not yet subscribed."""
        for topic, types in self.node.get_topic_names_and_types():
            if topic in self.meters or topic in self.skipped or not self.wanted(topic):
                continue
            try:
                kind = get_message(types[0])
            except (AttributeError, ModuleNotFoundError, ValueError) as error:
                why = error.__class__.__name__
                self.skipped[topic] = f'{types[0]}: not installed here ({why})'
                continue
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                             reliability=reliability(self.node, topic),
                             durability=DurabilityPolicy.VOLATILE)
            self.meters[topic] = topic_load.Meter()
            self.types[topic] = types[0]
            self.subscriptions[topic] = self.node.create_subscription(
                kind, topic, self.counter(topic), qos, raw=True)

    def unmatched(self):
        """Return the topics whose publishers haven't all connected to us yet."""
        return sorted(topic for topic, sub in self.subscriptions.items()
                      if sub.get_publisher_count() < self.node.count_publishers(topic))

    def counter(self, topic):
        """Return a callback that counts one topic's messages."""
        def count(data):
            now = time.monotonic()
            self.meters[topic].add(now, len(data))
            if topic == '/clock':
                self.clock[1:] = [(now, data)]
        return count

    def real_time_factor(self):
        """Return simulated seconds per real second over the window, or None without /clock."""
        if len(self.clock) < 2:
            return None

        def seconds(data):
            stamp = deserialize_message(data, Clock).clock
            return stamp.sec + stamp.nanosec * 1e-9
        (wall0, sim0), (wall1, sim1) = self.clock
        return (seconds(sim1) - seconds(sim0)) / (wall1 - wall0) if wall1 > wall0 else None

    def restart(self):
        """Forget what was counted: the window starts now."""
        for topic in self.meters:
            self.meters[topic] = topic_load.Meter()
        self.clock = []


def spin_for(node, seconds):
    """Handle callbacks for `seconds` of wall time."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))


def find_topics(node, meters, warmup, settle=2.0, longest=20.0):
    """
    Subscribe to the topics as discovery finds them, and wait until every one is connected.

    Discovery is gradual, more so with many programs running: a single look can find a
    handful of a sim's 140 topics, and a subscription connects to each publisher a while
    after the topic shows up. Count too early and a 50 Hz topic looks like 14 Hz. So look
    every half second for at least `warmup` s, until no topic is new for `settle` s and
    every publisher is connected; give up waiting after `longest` s.

    Return the topics still not fully connected.
    """
    start = last_new = time.monotonic()
    seen = 0
    while True:
        spin_for(node, 0.5)
        meters.discover()
        now = time.monotonic()
        if len(meters.meters) + len(meters.skipped) > seen:
            seen = len(meters.meters) + len(meters.skipped)
            last_new = now
        waiting = meters.unmatched()
        if now - start >= longest or (now - start >= warmup and now - last_new >= settle
                                      and not waiting):
            return waiting


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--seconds', type=float, default=10.0, help='how long to measure (10)')
    parser.add_argument('--warmup', type=float, default=3.0,
                        help='at least this long (s) to find the topics and connect first; '
                             'longer while new topics keep appearing, up to 20 s (3)')
    parser.add_argument('--include', action='append', default=[],
                        help='only topics matching this pattern (a regular expression; repeat)')
    parser.add_argument('--exclude', action='append', default=[],
                        help='leave out topics matching this pattern (repeat)')
    parser.add_argument('--local', action='store_true',
                        help='a system on this computer only, as pixi run sim starts it '
                             '(ROS discovery on localhost, like tools/ros.sh --local)')
    parser.add_argument('--markdown', action='store_true', help='a Markdown table')
    parser.add_argument('--all', action='store_true', help='also list topics that sent nothing')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be more than 0')

    if args.local:
        # Before rclpy.init(): the middleware reads them once. A LOCALHOST sim and a SUBNET
        # listener find only some of each other's topics.
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        os.environ.pop('ROS_STATIC_PEERS', None)
    rclpy.init()
    node = rclpy.create_node(ME)
    meters = Meters(node, args.include, args.exclude)
    try:
        late = find_topics(node, meters, args.warmup)
        if not meters.meters:
            print('No topics found. Is a system running, on this ROS_DOMAIN_ID?', file=sys.stderr)
            return 1
        meters.restart()
        print(f'Measuring {len(meters.meters)} topics for {args.seconds:g} s...', file=sys.stderr)
        start = time.monotonic()
        spin_for(node, args.seconds)
        window = time.monotonic() - start
        factor = meters.real_time_factor()
        rows = topic_load.summary(meters.meters, window, meters.types,
                                  {t: publishers(node, t) for t in meters.meters})
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print(topic_load.table(rows, markdown=args.markdown, quiet=not args.all))
    quiet = sum(1 for r in rows if r.hz == 0)
    notes = [f'{len(rows)} topics measured for {window:.1f} s of wall time'
             + (f'; {quiet} sent nothing (--all lists them)' if quiet and not args.all else '')]
    if factor is not None:
        notes.append(f'simulated time ran at {factor:.2f} x real time (/clock): '
                     f'a simulated sensor sends that much of its rate per real second')
    if late:
        notes.append(f'not every publisher had connected after 20 s, so these may read low: '
                     f'{", ".join(late)}')
    for topic, why in sorted(meters.skipped.items()):
        notes.append(f'skipped {topic}: {why}')
    print('\n' + '\n'.join(notes))
    return 0


if __name__ == '__main__':
    sys.exit(main())
