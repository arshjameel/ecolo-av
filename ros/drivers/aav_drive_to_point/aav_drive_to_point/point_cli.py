"""
drive_to_point: drive the car to a point and stop there (point_driver's /drive_to_point).

    pixi run drive-to-point --sim --ahead 20                 Gazebo: 20 m ahead of the car
    pixi run drive-to-point --sim --ahead 30 --left 10       ... and 10 m to its left
    pixi run drive-to-point --sim --x 30 --y 5               a map point (x east, y north, m)
    pixi run drive-to-point --lat 45.38630 --lon -75.69580   the real car: a GPS point
    pixi run drive-to-point --lat ... --lon ... --speed 0.5  slower (ego caps it anyway)

Ctrl-C stops the car (it cancels the goal: zero commands). On the real car (/system/source
REAL, or nobody announcing) it shows what it will do and a checklist, and drives only after
you type "drive". --sim keeps ROS on this computer (tools/drive_to_point.sh) and uses
the simulator's clock.

Subscribes  system/source             aav_interfaces/DataSource, latched
            localization/odom/global  nav_msgs/Odometry: for --ahead/--left, and the distance
Calls       /drive_to_point                     aav_interfaces/action/DriveToPoint
"""

import argparse
import math
import signal
import sys
import time

from aav_actuator_tests.safety import Kind, LOT_CHECKLIST, source_kind
from aav_drive_to_point import rules
from aav_interfaces.action import DriveToPoint
from aav_interfaces.msg import DataSource
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

FEEDBACK = DriveToPoint.Feedback
PHASES = {FEEDBACK.PHASE_TURN: 'turning', FEEDBACK.PHASE_ACCELERATE: 'speeding up',
          FEEDBACK.PHASE_HOLD: 'cruising', FEEDBACK.PHASE_DECELERATE: 'slowing',
          FEEDBACK.PHASE_STOPPING: 'stopping'}


def parse(argv):
    ap = argparse.ArgumentParser(prog='drive-to-point', description=__doc__.split('\n')[1])
    ap.add_argument('--lat', type=float, help='degrees north (with --lon)')
    ap.add_argument('--lon', type=float, help='degrees east (with --lat)')
    ap.add_argument('--x', type=float, help='m east of the datum, map frame (with --y)')
    ap.add_argument('--y', type=float, help='m north of the datum, map frame (with --x)')
    ap.add_argument('--ahead', type=float, help="m ahead of the car's nose")
    ap.add_argument('--left', type=float, default=0.0, help='m to its left, with --ahead')
    ap.add_argument('--speed', type=float, default=0.0, help='cruise speed, m/s (0: default)')
    ap.add_argument('--sim', action='store_true', help="Ravensim: this computer, the simulator's clock")
    a = ap.parse_args(argv)
    kinds = [a.lat is not None or a.lon is not None, a.x is not None or a.y is not None,
             a.ahead is not None]
    if sum(kinds) != 1:
        ap.error('give one of: --lat and --lon, --x and --y, or --ahead [--left]')
    if kinds[0] and (a.lat is None or a.lon is None):
        ap.error('--lat needs --lon, and --lon needs --lat')
    if kinds[1] and (a.x is None or a.y is None):
        ap.error('--x needs --y, and --y needs --x')
    return a


class Client(Node):

    def __init__(self, sim):
        super().__init__('point_cli', parameter_overrides=[Parameter('use_sim_time', value=sim)])
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sources, self.odom = {}, None
        self.create_subscription(DataSource, 'system/source',
                                 lambda m: self.sources.update(
                                     {m.announcer: [s.source for s in m.sensors]}), latched)
        self.create_subscription(Odometry, 'localization/odom/global',
                                 lambda m: setattr(self, 'odom', m), 10)
        self.action = ActionClient(self, DriveToPoint, 'drive_to_point')

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until is None

    def pose(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return p.position.x, p.position.y, yaw


def say(text=''):
    print(text, flush=True)


def build_goal(a, pose):
    """Return (the goal, a description of B), or (None, why not)."""
    goal = DriveToPoint.Goal(cruise_speed=float(a.speed))
    if a.lat is not None:
        goal.use_lat_lon, goal.latitude, goal.longitude = True, a.lat, a.lon
        return goal, f'lat {a.lat:.7f}, lon {a.lon:.7f}'
    if a.x is not None:
        goal.x, goal.y = a.x, a.y
    else:
        if pose is None:
            return None, "no /localization/odom/global: --ahead needs the car's position"
        x, y, yaw = pose
        goal.x = x + a.ahead * math.cos(yaw) - a.left * math.sin(yaw)
        goal.y = y + a.ahead * math.sin(yaw) + a.left * math.cos(yaw)
    return goal, f'map x {goal.x:.2f} m, y {goal.y:.2f} m'


def main(argv=None):
    a = parse(sys.argv[1:] if argv is None else argv)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Client(a.sim)
    try:
        return run(node, a)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def run(node, a):
    node.spin_for(3.0, lambda: bool(node.sources) and node.odom is not None)
    kind = source_kind(node.sources)
    if a.sim and kind is not Kind.SIM:
        say(f'REFUSED: --sim, but /system/source says {kind.value}. Start Gazebo first '
            '(pixi run sim ...)')
        return 1
    if not a.sim and kind is Kind.SIM:
        say('REFUSED: Gazebo announces SIM on /system/source: add --sim')
        return 1
    goal, where = build_goal(a, node.pose())
    if goal is None:
        say(f'REFUSED: {where}')
        return 1
    pose = node.pose()
    far = ''
    if pose is not None and not goal.use_lat_lon:
        far = f', {math.hypot(goal.x - pose[0], goal.y - pose[1]):.1f} m from the car'
    say(f'B: {where}{far} ({kind.value})')
    if kind in (Kind.REAL, Kind.UNKNOWN):
        say()
        say('THE REAL CAR. Check, out loud:')
        for item in LOT_CHECKLIST:
            say(f'  [ ] {item}')
        say('  [ ] The straight line to B, and the turn to face it, are clear')
        say('To stop at any time: Ctrl-C here, the e-stop, or the joystick deadman.')
        if not sys.stdin.isatty():
            say('REFUSED: the real car needs a person at the keyboard to confirm')
            return 1
        try:
            typed = input(f'Type "{rules.CONFIRM_WORD}" to start (anything else cancels): ')
        except (EOFError, KeyboardInterrupt):
            typed = None
        if not rules.confirmed(typed):
            say('Cancelled: nothing moved.')
            return 1
        goal.confirmed = True
    if not node.action.wait_for_server(timeout_sec=5.0):
        say('REFUSED: no /drive_to_point action: is point_driver running? (sim, bench and car '
            'start it)')
        return 1

    last = {'t': 0.0}

    def on_feedback(msg):
        f = msg.feedback
        if time.monotonic() - last['t'] >= 0.5:
            last['t'] = time.monotonic()
            say(f'  {PHASES.get(f.phase, f.phase):<11} {f.distance_left:6.1f} m left, '
                f'{f.cross_track:+.2f} m off the line, {f.measured_speed:.2f} m/s '
                f'(asked {f.commanded_speed:.2f})')

    handle = None
    try:
        sent = node.action.send_goal_async(goal, feedback_callback=on_feedback)
        node.spin_for(10.0, sent.done)
        handle = sent.result() if sent.done() else None
        if handle is None or not handle.accepted:
            say('REFUSED: point_driver would not take it (another drive is running?)')
            return 1
        done = handle.get_result_async()
        while not done.done():
            node.spin_for(0.1)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second Ctrl-C can't cut it short
        say('STOPPING: Ctrl-C')
        if handle is not None:
            cancel = handle.cancel_goal_async()
            node.spin_for(5.0, cancel.done)
            done = handle.get_result_async()
            node.spin_for(5.0, done.done)
            if done.done():
                say(done.result().result.message)
        return 130
    r = done.result().result
    say(('DONE: ' if r.success else 'NOT THERE: ') + r.message +
        f'; {r.distance_travelled:.1f} m driven in {r.duration:.1f} s')
    return 0 if r.success else 2


if __name__ == '__main__':
    sys.exit(main())
