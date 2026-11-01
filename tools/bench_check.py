#!/usr/bin/env python3
"""
Bench check: start bench mode, then test the whole car graph on this computer.

    pixi run bench-check          (about 2 minutes; nothing else on the network sees it)
    pixi run -e ml bench-check --perception    (adds the detector and the stop sign: 3 minutes)
    pixi run bench-check --obstacle            (adds a box for the radar: 3 minutes)
    pixi run bench-check --lidar               (adds a box for the lidar alone: 3 minutes)
    pixi run bench-check --lane   (adds lane keeping, in a second bench run: 1 more minute)
    pixi run bench-check --weave              (adds a straight 40 m Nav2 drive: 1 more minute)
    pixi run bench-check --zone-edge          (adds a box at the stop zone's edge: 1 more minute)
    pixi run bench-check --geofence           (a scripted drive at a 12 m geofence, in a second
                                               bench run: 2 more minutes)
    pixi run bench-check --python-bridge       (aav_spine's Python spine_bridge, not the Rust one)

1. Starts bench.launch.py on a private ROS domain, local to this computer.
2. Checks the running graph against docs/interfaces.yaml (check_graph.py --profile bench).
3. Waits for the spine to arm, drives a scripted profile, and compares where the model car
   really is (/vehicle/bench/truth) with where the car's own software thinks it is
   (/localization/odom/global).
4. Presses the e-stop while driving: the car must stop, and the spine must say so.
5. Presses the hardware e-stop the same way.
6. With --perception: Nav2 drives 40 m east, past the bench's stop sign. The car must stop
   before the sign, wait, and go on to the goal. The camera's picture with the detector's boxes, at
   the moment it stopped, is saved to /tmp/aav-bench-stop-sign.png.
7. With --obstacle: a box as wide as the lane appears 15 m ahead, which only the pretend
   radar sees, and Nav2 drives 30 m. collision_monitor's stop zone must stop the car before the
   box; once the box is taken away, the car must go on to the goal.
   With --lidar: bench mode with lidar:=true, and the same again with a box only the
   pretend lidar sees (mock_lidar's VLP-16 packets, read by the stock velodyne driver).
8. With --weave (TODO N7), before the other Nav2 checks: Nav2 drives 40 m straight ahead,
   with nothing in the way. Says how much the car swung from side to side (the weave),
   apart from any slow drift off its line, and how often Nav2's turn command changed sign.
9. With --zone-edge: the car stands still with a box as wide as the lane right at the far
   edge of collision_monitor's stop zone, and Nav2 asks it to drive on. The stop must hold:
   no forward command may reach ego after the first stop, and the car must not creep on.
10. With --geofence: the bench again, its geofence a 12 m circle round the start
   (geofence_radius:=12). A scripted drive at 1 m/s straight ahead: the geofence's lock
   must stop it with every corner of the car inside the circle (by the truth), standing
   in NEAR, and the stop from the lock closing no longer than the stopping distance the
   geofence counts on (RSS, with the spine's numbers). Then the keyboard, a person, backs
   the car 4 m: the lock must not hold it, and must open again (OK).
11. With --lane: the bench again, with lane keeping (lane:=true) and nothing else driving,
   the car 0.3 m left of the road's centre. lane_finder and control_interface must bring it
   back within 5 cm, and keep it there. Its own start-up and graph checks come first.

Prints a table and exits 1 if anything failed. Run it after changing anything between the
drivers and the spine. The launch logs go to /tmp/aav-bench-check.log and, with --lane,
/tmp/aav-bench-check-lane.log. AAV_BENCH_LOG=/tmp/mine.log moves them to /tmp/mine.log and
/tmp/mine-lane.log.
"""

import bisect
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# A private domain and localhost only, set before rclpy starts: the check can't disturb a
# car or another laptop, and nothing out there can disturb it. One nobody on this
# computer uses now (aav_io_mux domains): another run on the same domain would mix in.
if 'AAV_BENCH_DOMAIN' in os.environ:
    os.environ['ROS_DOMAIN_ID'] = os.environ['AAV_BENCH_DOMAIN']
else:
    from aav_io_mux.domains import free_domain
    os.environ['ROS_DOMAIN_ID'] = str(free_domain())
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

from aav_description import vehicle  # noqa: E402
from aav_interfaces.action import ScriptedDrive  # noqa: E402
from aav_interfaces.msg import VehicleStatus  # noqa: E402
from aav_lane_geometry.msg import LaneGeometry  # noqa: E402
from aav_safety import geofence  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav2_msgs.msg import CollisionMonitorState  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, os.path.join(ROOT, 'tools'))
import drive_metrics  # noqa: E402, I100

RESULTS = []

# Lane keeping (--lane): where the car starts, what counts as back on the centre line,
# and how long it has to get there.
LANE_START = 0.3        # m left of the road's centre
LANE_BAND = 0.05        # m either side of the centre
LANE_TIME = 30.0        # s of driving, from when the car starts moving
WEAVE_LIMIT = 0.1       # m RMS: the most the car may swing round its own path (--weave)
LOOKAHEAD = 3.0         # m: where lane_finder measures the offset (its config, lookahead)
GEOFENCE_RADIUS = 12.0  # m: --geofence's fence round the start, short of the bench's sign
with open(os.path.join(ROOT, 'ros', 'sim', 'aav_bench', 'config', 'bench.yaml')) as f:
    ROAD = yaml.safe_load(f)['mock_camera']['ros__parameters']
ROAD_Y = ROAD['road_y']            # m: the centre
LANE_WIDTH = ROAD['lane_width']    # m between the lines: the obstacle check's box spans it


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def result(name, ok, detail):
    RESULTS.append((name, ok, detail))
    print(f'{"PASS" if ok else "FAIL"}  {name}: {detail}', flush=True)


class Probe(Node):

    def __init__(self):
        super().__init__('bench_check')
        self.truth = self.glob = self.status = None
        # Every truth sample and every lane message, from the start: lane keeping starts the
        # moment the spine arms, before its check begins.
        self.track, self.lanes = [], []
        self.create_subscription(Odometry, '/vehicle/bench/truth', self.on_truth, 10)
        self.create_subscription(LaneGeometry, '/perception/lane/geometry', self.on_lane, 10)
        self.create_subscription(Odometry, '/localization/odom/global',
                                 lambda m: setattr(self, 'glob', m), 10)
        self.latches = []   # (stamp, faults) each time the spine stops being armed
        self.create_subscription(VehicleStatus, '/vehicle/status', self.on_status, 10)
        self.estop = self.create_publisher(Bool, '/safety/estop', 10)
        self.hw_estop = self.create_publisher(Bool, '/vehicle/bench/hw_estop', 10)
        self.arm_client = self.create_client(SetBool, '/spine/arm')
        self.drive = ActionClient(self, ScriptedDrive, '/scripted_drive')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.stop = False
        self.picture = None
        self.create_subscription(Bool, '/perception/stop_sign', self.on_stop, 10)
        self.create_subscription(Image, '/perception/detections/image',
                                 lambda m: setattr(self, 'picture', m), 2)
        self.stop_picture = None
        self.monitor = []    # (time, action, polygon) from collision_monitor, as it changes
        self.create_subscription(CollisionMonitorState, '/safety/collision_monitor/state',
                                 lambda m: self.monitor.append(
                                     (time.monotonic(), m.action_type, m.polygon_name)), 10)
        # Nav2's controller's turn command (rad/s), on the clock of the truth's stamps (the
        # bench runs on the computer's clock), for --weave; and the mux's command to ego
        # (m/s), on the steady clock of `monitor`, for --zone-edge.
        self.turns, self.commands = [], []
        self.create_subscription(TwistStamped, '/control/cmd_vel',
                                 lambda m: self.commands.append(
                                     (time.monotonic(), m.twist.linear.x)), 10)
        self.create_subscription(TwistStamped, '/cmd_vel_nav',
                                 lambda m: self.turns.append(
                                     (self.get_clock().now().nanoseconds * 1e-9,
                                      m.twist.angular.z)), 10)
        self.box_params = {s: self.create_client(SetParameters, f'/mock_{s}/set_parameters')
                           for s in ('radar', 'lidar')}
        # The geofence: its state's line, and (time, x, y, speed) by the truth when its
        # lock first closed after fence_on was set to None (--geofence).
        self.fence_state, self.fence_on, self.fence_locked = '', None, None
        self.create_subscription(String, '/safety/geofence/state',
                                 lambda m: setattr(self, 'fence_state', m.data), 10)
        self.create_subscription(Bool, '/safety/geofence/lock', self.on_fence_lock, 10)
        self.keyboard = self.create_publisher(TwistStamped, '/drivers/keyboard/cmd_vel', 10)
        self.output_mux = ''   # the output mux's state: OPEN, WAIT or STOP (aav_io_mux)
        self.create_subscription(String, '/safety/output_mux/state',
                                 lambda m: setattr(self, 'output_mux', m.data), 10)

    def on_truth(self, msg):
        self.truth = msg
        q = msg.pose.pose.orientation
        self.track.append((seconds(msg.header.stamp), msg.pose.pose.position.x,
                           msg.pose.pose.position.y, 2.0 * math.atan2(q.z, q.w),
                           msg.twist.twist.linear.x))

    def on_fence_lock(self, msg):
        if msg.data and not self.fence_locked and self.fence_on is None and self.truth:
            x, y = self.xy(self.truth)
            self.fence_on = (time.monotonic(), x, y, self.truth.twist.twist.linear.x)
        self.fence_locked = msg.data

    def on_status(self, msg):
        if self.status is not None and self.status.armed and not msg.armed:
            self.latches.append((seconds(msg.header.stamp), msg.faults))
        self.status = msg

    def on_lane(self, msg):
        self.lanes.append((seconds(msg.header.stamp), msg.vehicle_offset_m,
                           msg.both_lanes_detected))

    def on_stop(self, msg):
        if msg.data and not self.stop and self.picture is not None:
            self.stop_picture = self.picture   # what the camera saw as the lock went on
            self.stop_time = self.get_clock().now().nanoseconds * 1e-9
        self.stop = msg.data

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until is None

    def xy(self, msg):
        return msg.pose.pose.position.x, msg.pose.pose.position.y

    def arm(self):
        if not self.arm_client.wait_for_service(timeout_sec=20.0):
            return False
        future = self.arm_client.call_async(SetBool.Request(data=True))
        self.spin_for(10.0, lambda: future.done())
        return future.done() and future.result().success

    def arm_and_hold(self, tries=3):
        """Arm, and wait until the spine has stayed armed for 2 s."""
        # Ego's commands reach spine_bridge only once the output mux is open; armed before
        # that, the spine's watchdog latches it again at once. Wait before every try: a
        # node that starts late (mock_camera, say) shuts the mux again for 2 s to settle,
        # and the spine rightly brakes and drops out.
        for _ in range(tries):
            self.spin_for(60.0, lambda: self.output_mux.startswith('OPEN'))
            self.arm()
            self.spin_for(1.0)
            if self.status.armed and not self.spin_for(2.0, lambda: not self.status.armed):
                return True
        return False

    def send_drive(self, speed=1.0, hold=5.0):
        goal = ScriptedDrive.Goal(target_speed=speed, acceleration=0.5, deceleration=1.0,
                                  hold_time=hold)
        if not self.drive.wait_for_server(timeout_sec=20.0):
            return None
        future = self.drive.send_goal_async(goal)
        self.spin_for(10.0, lambda: future.done())
        return future.result() if future.done() else None

    def set_box(self, sensor, obstacle, boxes=None, height=None):
        """Put the bench's boxes where a pretend sensor sees them, or take them away."""
        params = [Parameter(name='obstacle', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=obstacle))]
        if height is not None:
            params.append(Parameter(name='box_height', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=height)))
        if boxes is not None:
            params.append(Parameter(name='boxes', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=boxes)))
        client = self.box_params[sensor]
        if not client.wait_for_service(timeout_sec=10.0):
            return False
        future = client.call_async(SetParameters.Request(parameters=params))
        self.spin_for(10.0, lambda: future.done())
        return future.done() and all(r.successful for r in future.result().results)

    def press(self, publisher, value, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            publisher.publish(Bool(data=value))
            self.spin_for(0.1)


def check_estop(probe, name, publisher):
    """Drive, press `publisher`'s e-stop at 1 m/s, check the stop, release, re-arm."""
    for attempt in range(2):
        if not probe.status.armed:
            probe.arm_and_hold()
        handle = probe.send_drive(hold=8.0)
        if handle is None or not handle.accepted:
            result(name, False, 'the scripted drive was refused')
            return
        probe.spin_for(6.0)
        v_before = probe.truth.twist.twist.linear.x
        if v_before > 0.5 or probe.status.armed:
            break
        # The spine latched before the car got going (a gap in commands on a busy
        # computer), which isn't what this checks: stop the drive, and start again once.
        handle.cancel_goal_async()
        probe.spin_for(2.0)
    probe.press(publisher, True, 3.0)
    v_after = probe.truth.twist.twist.linear.x
    reported = probe.status is not None and probe.status.estop_active
    probe.press(publisher, False, 1.0)
    handle.cancel_goal_async()
    probe.spin_for(2.0)
    ok = v_before > 0.5 and abs(v_after) < 0.05 and reported
    result(name, ok, f'{v_before:.2f} m/s before, {v_after:.2f} m/s 3 s later, '
                     f'spine reported it: {reported}')
    probe.arm_and_hold()


def check_stop_sign(probe):
    """Nav2 drives 40 m east, past the stop sign at x = 25 m: it must stop before it."""
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = 40.0
    goal.pose.pose.orientation.w = 1.0
    if not probe.nav.wait_for_server(timeout_sec=30.0):
        result('stop sign', False, 'no Nav2')
        return
    sent = probe.nav.send_goal_async(goal)
    probe.spin_for(10.0, lambda: sent.done())
    if not sent.done() or not sent.result().accepted:
        result('stop sign', False, 'Nav2 refused the goal')
        return
    done = sent.result().get_result_async()
    stopped_at, stood_still = None, False
    end = time.monotonic() + 150.0
    while time.monotonic() < end and not done.done():
        probe.spin_for(0.1)
        if probe.stop and stopped_at is None:
            stopped_at = probe.truth.pose.pose.position.x
        if probe.stop and abs(probe.truth.twist.twist.linear.x) < 0.05:
            stood_still = True
    x = probe.truth.pose.pose.position.x
    succeeded = done.done() and done.result().status == 4
    ok = stopped_at is not None and stopped_at < 24.0 and stood_still and succeeded and x > 35
    where = f'locked at x = {stopped_at:.1f} m' if stopped_at is not None else 'never locked'
    result('stop sign', ok, f'{where} (the sign is at 25 m), stood still: {stood_still}, '
                            f'Nav2 goal {"reached" if succeeded else "not reached"} at {x:.1f} m')
    if probe.stop_picture is not None:
        try:
            from cv_bridge import CvBridge
            import cv2
            cv2.imwrite('/tmp/aav-bench-stop-sign.png',
                        CvBridge().imgmsg_to_cv2(probe.stop_picture, 'bgr8'))
            stamp = probe.stop_picture.header.stamp
            age = probe.stop_time - (stamp.sec + stamp.nanosec * 1e-9)
            print(f'The camera at the stop: /tmp/aav-bench-stop-sign.png (taken {age:.1f} s '
                  'before the lock went on)', flush=True)
        except Exception as e:  # noqa: B902 - the picture is a bonus
            print(f'could not save the picture: {e}', flush=True)


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def check_obstacle(probe, ahead=15.0, goal=30.0, sensor='radar'):
    """
    Put a box `ahead` m in the lane, seen by `sensor` alone: the stop zone must stop the car.

    The box spans the lane, as a stalled car or a barrier would: Nav2's turn command still
    flips with the GPS (N7), and the car can drift a metre or more off its line, so a narrow
    box tests that drift, not the stop. On a busy computer a gap in commands can latch the
    spine: the check arms it again, as a safety driver would. The stop zone must still be
    what stopped the car.

    For the lidar, the box is 1.5 m tall, as a car or a person: the roof lidar (1.8 m up,
    its lowest laser 15 degrees down) can't see the radar's 1 m box closer than 3 m, and
    the stop zone, 0-1 m past the bumper, is 0.9-1.9 m from it. That blind zone is the
    radar's (docs/perception.md, "The roof lidar").
    """
    name = 'obstacle' if sensor == 'radar' else f'obstacle ({sensor} only)'
    if not probe.arm_and_hold():
        result(name, False, 'the spine would not stay armed')
        return
    body = vehicle.load()['vehicle']['body']
    front = body['length'] - body['rear_overhang']   # base_link to the front bumper
    t = probe.truth
    x0, y0, h = t.pose.pose.position.x, t.pose.pose.position.y, yaw_of(t)
    c, s = math.cos(h), math.sin(h)

    def along():
        p = probe.truth.pose.pose.position
        return (p.x - x0) * c + (p.y - y0) * s

    # A box 1 m deep and as wide as the lane, its near face `ahead` m in front of the bumper.
    bx, by = x0 + (front + ahead + 0.5) * c, y0 + (front + ahead + 0.5) * s
    half = LANE_WIDTH / 2.0
    height = 1.5 if sensor == 'lidar' else None
    if not probe.set_box(sensor, True, [bx - 0.5, by - half, bx + 0.5, by + half], height):
        result(name, False, f'could not place the box (mock_{sensor} parameters)')
        return
    target = NavigateToPose.Goal()
    target.pose.header.frame_id = 'map'
    target.pose.pose.position.x, target.pose.pose.position.y = x0 + goal * c, y0 + goal * s
    orientation = target.pose.pose.orientation
    orientation.z, orientation.w = math.sin(h / 2.0), math.cos(h / 2.0)
    sent = probe.nav.send_goal_async(target)
    probe.spin_for(10.0, lambda: sent.done())
    if not sent.done() or not sent.result().accepted:
        result(name, False, 'Nav2 refused the goal')
        probe.set_box(sensor, False)
        return
    done = sent.result().get_result_async()
    t0, seen = time.monotonic(), len(probe.monitor)
    top, still, gap, latches = 0.0, None, None, 0
    while time.monotonic() - t0 < 150.0 and not done.done():
        probe.spin_for(0.1)
        v = probe.truth.twist.twist.linear.x
        top = max(top, v)
        if probe.status is not None and not probe.status.armed:
            latches += 1
            probe.arm_and_hold()
            continue
        if gap is None and top > 0.5 and abs(v) < 0.02:
            still = still or time.monotonic()
            if time.monotonic() - still > 3.0:      # stood still for 3 s: take the box away
                gap = ahead - along()
                probe.set_box(sensor, False)
        else:
            still = None
    states = probe.monitor[seen:]
    stop_zone = any(a == CollisionMonitorState.STOP and p == 'StopZone' for _, a, p in states)
    reached = done.done() and done.result().status == 4
    passed = along() > front + ahead + 1.0
    ok = gap is not None and 0.0 < gap < 2.0 and stop_zone and reached and passed
    where = f'stood still {gap:.2f} m before the box' if gap is not None else 'never stopped'
    times = 'once' if latches == 1 else f'{latches} times'
    paused = (f'; the spine latched {times} (gaps in the commands: a busy computer?) and was '
              'armed again' if latches else '')
    result(name, ok, f'{where} (top speed {top:.2f} m/s), stop zone fired: {stop_zone}; box '
                     f'taken away, Nav2 goal {"reached" if reached else "not reached"} '
                     f'{along():.1f} m on{paused}')
    probe.set_box(sensor, False)


def nav_goal(probe, x, y, h):
    """Send Nav2 a goal on the map; return its goal handle, or None if refused."""
    target = NavigateToPose.Goal()
    target.pose.header.frame_id = 'map'
    target.pose.pose.position.x, target.pose.pose.position.y = x, y
    orientation = target.pose.pose.orientation
    orientation.z, orientation.w = math.sin(h / 2.0), math.cos(h / 2.0)
    if not probe.nav.wait_for_server(timeout_sec=30.0):
        return None
    sent = probe.nav.send_goal_async(target)
    probe.spin_for(10.0, lambda: sent.done())
    if not sent.done() or not sent.result().accepted:
        return None
    return sent.result()


def drive_rearming(probe, seconds, until=None):
    """
    Spin for `seconds` (or until `until()`), arming the spine again whenever it latches.

    On a busy computer a gap in commands latches the spine, as it should; a safety driver
    would arm it again (as check_obstacle does). Returns how many times that happened.
    """
    end, latches = time.monotonic() + seconds, 0
    while time.monotonic() < end and not (until is not None and until()):
        probe.spin_for(0.1)
        if probe.status is not None and not probe.status.armed:
            latches += 1
            probe.arm_and_hold()
    return latches


def latched_note(latches):
    """Say in a result's detail that the spine latched and was armed again."""
    if not latches:
        return ''
    times = 'once' if latches == 1 else f'{latches} times'
    return (f'; the spine latched {times} (gaps in the commands: a busy computer?) and was '
            'armed again')


def check_weave(probe, goal=40.0, margin=5.0):
    """
    Drive `goal` m straight ahead with Nav2, and measure the weave (TODO N7).

    Measured on the truth, over the straight part (`margin` m in from each end, so the
    start and the stop don't count):
      weave       how far the car's sideways offset swings round its own 4 s average
                  (drive_metrics.weave): side to side, faster than a GPS bias drifts
      off line    the RMS and the most it strays from the straight line that fits its
                  track best: the weave, and slower wander too
      turn flips  how often a second the controller's turn command changes sign (N7's
                  first sign: "flips sign about once a second"), and its RMS
    Runs first, from the start spot facing along the road, before the other Nav2 checks.
    """
    if not probe.arm_and_hold():
        result('weave', False, 'the spine would not stay armed')
        return
    t = probe.truth
    x0, y0, h = t.pose.pose.position.x, t.pose.pose.position.y, yaw_of(t)
    c, s = math.cos(h), math.sin(h)
    since = probe.track[-1][0]
    handle = nav_goal(probe, x0 + goal * c, y0 + goal * s, h)
    if handle is None:
        result('weave', False, 'Nav2 refused the goal')
        return
    done = handle.get_result_async()
    latches = drive_rearming(probe, 120.0, lambda: done.done())
    reached = done.done() and done.result().status == 4
    if not done.done():
        handle.cancel_goal_async()
        probe.spin_for(2.0)
    run = [p for p in probe.track if p[0] >= since
           and margin <= (p[1] - x0) * c + (p[2] - y0) * s <= goal - margin]
    if len(run) < 2:
        result('weave', False, f'too little of the drive recorded ({len(run)} truth samples)'
                               + latched_note(latches))
        return
    t0, t1 = run[0][0], run[-1][0]
    turns = [w for w in probe.turns if t0 <= w[0] <= t1]   # while on the straight part
    swing = drive_metrics.weave(drive_metrics.lateral(run, (x0, y0), (x0 + c, y0 + s)))
    stray = drive_metrics.off_line([(p[1], p[2]) for p in run])
    flips = drive_metrics.sign_flips(turns) / max(t1 - t0, 1e-9)
    turn_rms = drive_metrics.rms([w for _, w in turns])
    ok = reached and swing < WEAVE_LIMIT
    result('weave', ok, f'weave {swing:.3f} m RMS (limit {WEAVE_LIMIT}); off its best line '
                        f'{drive_metrics.rms(stray):.2f} m RMS, {max(map(abs, stray)):.2f} m '
                        f'at most; the turn command changed sign {flips:.2f} times a second, '
                        f'{turn_rms:.3f} rad/s RMS; Nav2 goal '
                        f'{"reached" if reached else "not reached"}' + latched_note(latches))


def check_zone_edge(probe, hold=20.0):
    """
    Put a box at the stop zone's far edge, the car standing still: the stop must hold.

    The box spans the lane, its near face exactly where the stop zone ends (1 m past the
    bumper). The pretend radar's range noise puts some of its points inside, some out, as
    a real one does. Nav2 is asked to drive on through it. Counted over `hold` s:
      monitor   collision_monitor's own stops that let go (its state topic)
      car       forward commands that reached ego (/control/cmd_vel) after the first stop
      crept     the most the car moved toward the box after its first stop (1 s after
                it, so the braking up to the stop doesn't count)
    collision_monitor (Jazzy) decides afresh on every command, so its own state still
    flickers; stop_hold (aav_safety) must keep the flicker from reaching the car.
    """
    if not probe.arm_and_hold():
        result('stop zone edge', False, 'the spine would not stay armed')
        return
    car = vehicle.load()['vehicle']
    front = car['body']['length'] - car['body']['rear_overhang']   # base_link to the bumper
    near = max(x for x, _ in vehicle.stop_zone_points(car))       # the zone's far edge
    t = probe.truth
    x0, y0, h = t.pose.pose.position.x, t.pose.pose.position.y, yaw_of(t)
    c, s = math.cos(h), math.sin(h)
    bx, by = x0 + (near + 0.5) * c, y0 + (near + 0.5) * s
    half = LANE_WIDTH / 2.0
    if not probe.set_box('radar', True, [bx - 0.5, by - half, bx + 0.5, by + half]):
        result('stop zone edge', False, 'could not place the box (mock_radar parameters)')
        return
    probe.spin_for(2.0)
    seen, track_from, sent_from = len(probe.monitor), probe.track[-1][0], len(probe.commands)
    handle = nav_goal(probe, x0 + 30.0 * c, y0 + 30.0 * s, h)
    if handle is None:
        result('stop zone edge', False, 'Nav2 refused the goal')
        probe.set_box('radar', False)
        return
    latches = drive_rearming(probe, hold)
    along = [(p[0], (p[1] - x0) * c + (p[2] - y0) * s) for p in probe.track if p[0] >= track_from]
    # From the state before the goal (collision_monitor publishes only changes) to now.
    states = probe.monitor[max(0, seen - 1):]
    actions = [a for _, a, _ in states]
    flickers = drive_metrics.stop_go(actions, CollisionMonitorState.STOP)
    first = next((w for w, a, _ in states if a == CollisionMonitorState.STOP), None)
    goes = (drive_metrics.starts([cmd for cmd in probe.commands[sent_from:] if cmd[0] > first])
            if first is not None else 0)
    # Creep is what the car moves after the first stop, not its braking up to it: the
    # track's stamps are on the computer's clock (the bench), `first` on the steady one.
    first_wall = first + time.time() - time.monotonic() if first is not None else None
    before = [a for t, a in along if first_wall is None or t <= first_wall + 1.0]
    after = [a for t, a in along if first_wall is not None and t > first_wall + 1.0]
    stopped_at = max(before, default=0.0)
    crept = max(after, default=stopped_at) - stopped_at
    handle.cancel_goal_async()
    probe.set_box('radar', False)
    probe.spin_for(3.0)
    ok = first is not None and goes == 0 and crept < 0.1
    result('stop zone edge', ok, f'over {hold:.0f} s: collision_monitor let its stop go '
                                 f'{flickers} times; {goes} forward starts reached ego after '
                                 f'the first stop; it stopped {stopped_at:.2f} m on, then crept '
                                 f'{crept:.2f} m more toward a box {near - front:.2f} m past '
                                 f'its bumper' + latched_note(latches))


def check_lane_keeping(probe):
    """
    Lane keeping alone: from LANE_START m left of the road's centre, the car must come back.

    control_interface drives from the moment the spine arms, so this reads the track the
    probe has kept since start-up: LANE_TIME seconds of driving, from when the car started
    moving. On a busy computer a gap in commands can latch the spine, which stops the car:
    the check arms it again, as a safety driver would, and its clock only runs while the
    car moves.
    """
    def moved():
        return next((s for s in probe.track if s[4] > 0.1), None)

    if not probe.spin_for(30.0, lambda: moved() is not None):
        result('lane keeping', False, f'the car never moved ({len(probe.lanes)} lane messages)')
        return
    t0, x0, y0 = moved()[:3]

    def driven():
        """Return (seconds of driving so far, sample) for each truth sample since t0."""
        clock, out, last = 0.0, [], None
        for s in probe.track:
            if s[0] >= t0:
                if last is not None and s[4] > 0.05:
                    clock += s[0] - last[0]
                out.append((clock, s))
                last = s
        return out

    give_up = time.monotonic() + 3.0 * LANE_TIME
    while driven()[-1][0] < LANE_TIME and time.monotonic() < give_up:
        probe.spin_for(0.5)
        if not probe.status.armed:
            probe.arm()
    run = [(c, s) for c, s in driven() if c <= LANE_TIME]
    clocks, stamps = [c for c, _ in run], [s[0] for _, s in run]

    def nearest(values, v):
        i = min(bisect.bisect_left(values, v), len(values) - 1)
        return i - 1 if i and v - values[i - 1] < values[i] - v else i

    def at(t):
        """Return the truth sample nearest to time t (s, a stamp)."""
        return run[nearest(stamps, t)][1]

    def after(c):
        """Return the truth sample after c seconds of driving."""
        return run[nearest(clocks, c)][1]

    start = y0 - ROAD_Y
    outside = [c for c, s in run if abs(s[2] - ROAD_Y) > LANE_BAND]
    settled = outside[-1] if outside else 0.0   # s of driving: within the band from then on
    over = max(0.0, max(-math.copysign(1.0, start) * (s[2] - ROAD_Y) for _, s in run))
    last = [s for c, s in run if c >= clocks[-1] - 5.0]
    end = sum(s[2] for s in last) / len(last) - ROAD_Y
    heading = sum(s[3] for s in last) / len(last)
    # lane_finder against the truth: the car's offset LOOKAHEAD m ahead, square to the road.
    errors, both = [], 0
    for t, offset, two in (m for m in probe.lanes if t0 <= m[0] <= stamps[-1]):
        _, _, y, yaw, _ = at(t)
        errors.append(abs(offset - (y - ROAD_Y + LOOKAHEAD * math.sin(yaw)) / math.cos(yaw)))
        both += two
    every = '  '.join(f'{after(k)[2] - ROAD_Y:+.2f}' for k in range(0, int(LANE_TIME) + 1, 3))
    print(f'      the offset (m), every 3 s of driving: {every}', flush=True)
    latches = sum(t0 <= s[0] <= stamps[-1] for s in probe.latches)
    ok = (clocks[-1] >= LANE_TIME - 1.0 and settled <= LANE_TIME - 5.0 and over < 0.10
          and abs(end) < LANE_BAND and bool(errors) and both >= 0.9 * len(errors))
    finder = (f'lane_finder {100 * sum(errors) / len(errors):.1f} cm from the truth on average, '
              f'both lines in {both} of {len(errors)} pictures' if errors else 'no lane messages')
    times = 'once' if latches == 1 else f'{latches} times'
    paused = (f'; the spine latched {times} (gaps in the commands: a busy computer?) and was '
              'armed again' if latches else '')
    result('lane keeping', ok,
           f'{start:+.2f} m to {end:+.2f} m, within {100 * LANE_BAND:.0f} cm after '
           f'{settled:.1f} s and {after(settled)[1] - x0:.1f} m, overshoot {100 * over:.0f} cm, '
           f'heading {heading:+.3f} rad; {finder}{paused}')


def check_contract(probe, env, tag):
    """Check the running graph against docs/interfaces.yaml (check_graph.py --profile bench)."""
    # Up to three tries, 10 s apart: a node's first start after a build can take 20 s
    # on a Mac (it loads PCL and VTK, and macOS checks the new binary). The probe spins
    # meanwhile, so its track of the car has no gaps.
    for attempt in range(3):
        with tempfile.TemporaryFile('w+') as out:
            graph = subprocess.Popen(
                [sys.executable, os.path.join(ROOT, 'tools', 'check_graph.py'), '--profile',
                 'bench'], cwd=ROOT, env=env, stdout=out, stderr=subprocess.DEVNULL, text=True)
            probe.spin_for(60.0, lambda: graph.poll() is not None)
            code = graph.wait()
            out.seek(0)
            fails = [line for line in out.read().splitlines() if line.startswith('FAIL')]
        if code == 0 and not fails:
            break
        probe.spin_for(10.0)
    result(f'graph vs contract{tag}', code == 0 and not fails,
           f'{len(fails)} failures' + (': ' + '; '.join(fails[:3]) if fails else ''))


def check_drive(probe):
    """Drive the scripted profile; compare the distance with the plan, and the EKF with truth."""
    for attempt in range(2):
        if not probe.status.armed:
            # A gap in commands latched the spine (a busy computer): arm it again, or the
            # drive runs out its plan standing still.
            probe.arm_and_hold()
        since = probe.track[-1][0]
        x0, y0 = probe.xy(probe.truth)
        handle = probe.send_drive()
        if handle is None or not handle.accepted:
            result('scripted drive', False, 'refused')
            return
        done = handle.get_result_async()
        probe.spin_for(40.0, lambda: done.done())
        probe.spin_for(3.0)
        x1, y1 = probe.xy(probe.truth)
        travelled = math.hypot(x1 - x0, y1 - y0)
        # The plan is 6.5 m. The spine's speed loop trails a ramp by about a second, so
        # 5-6 m is normal; the range catches a stretched plan (22 m) or no motion at all.
        good = 4.5 < travelled < 8.0
        if good or not any(t >= since for t, _ in probe.latches):
            break
        # The spine latched on the way, which stopped the car: not what this checks.
        # Drive the plan again, once.
    again = ', the second time: the spine latched during the first' if attempt else ''
    result('scripted drive', good,
           f'{travelled:.2f} m (plan: 6.5 m; the speed loop trails it by about 1 s){again}')
    gx, gy = probe.xy(probe.glob)
    error = math.hypot(gx - x1, gy - y1)
    result('global EKF vs truth', error < 1.5, f'{error:.2f} m apart (the GPS drifts 0.5 m)')


def geofence_motion():
    """Return the stopping numbers the bench's geofence uses (spine), as its launch does."""
    def params(package, file, node):
        path = os.path.join(ROOT, 'ros', *package, 'config', file)
        with open(path) as f:
            return yaml.safe_load(f)[node]['ros__parameters']
    own = params(('safety', 'aav_safety'), 'geofence.yaml', 'geofence')
    motion, _ = geofence.motion_from(
        params(('ego', 'aav_ego'), 'ego.yaml', 'ego'), vehicle.load(), 'spine',
        float(own['rate_hz']), float(own['pose_lag']),
        spine=params(('can_bridge', 'aav_spine'), 'spine.yaml', 'spine_bridge'))
    return motion


def check_geofence(probe, radius=GEOFENCE_RADIUS):
    """
    Drive a script at 1 m/s straight at a 12 m fence: it must stop inside; then a person.

    The geofence's lock (51) holds the scripted driver (50): the car must stop with every
    corner inside the circle (by the truth), and the stop measured from the lock against
    the stopping distance the geofence counts on. Then the keyboard (a person, 90) backs
    the car in, which the lock never holds, and the lock must open again.
    """
    corners = vehicle.footprint_points(vehicle.load()['vehicle'])
    probe.spin_for(30.0, lambda: probe.fence_state.startswith('OK'))
    if not probe.fence_state.startswith('OK'):
        result('geofence: ready', False, f'the geofence says {probe.fence_state!r}')
        return
    if not probe.status.armed:
        probe.arm_and_hold()
    x0, y0 = probe.xy(probe.truth)
    handle = probe.send_drive(speed=1.0, hold=40.0)
    if handle is None or not handle.accepted:
        result('geofence: stops a scripted drive', False, 'the scripted drive was refused')
        return
    probe.fence_on = None
    probe.spin_for(60.0, lambda: probe.fence_on is not None)
    if probe.fence_on is None:
        handle.cancel_goal_async()
        result('geofence: stops a scripted drive', False,
               f'the lock never closed (the geofence says {probe.fence_state!r})')
        return
    t_on, x_on, y_on, v_on = probe.fence_on
    probe.spin_for(15.0, lambda: abs(probe.truth.twist.twist.linear.x) < 0.02)
    probe.spin_for(1.0)
    state = probe.fence_state
    x1, y1 = probe.xy(probe.truth)
    q = probe.truth.pose.pose.orientation
    yaw = 2.0 * math.atan2(q.z, q.w)
    worst = max(math.hypot(x1 + fx * math.cos(yaw) - fy * math.sin(yaw) - x0,
                           y1 + fx * math.sin(yaw) + fy * math.cos(yaw) - y0)
                for fx, fy in corners)
    stop = math.hypot(x1 - x_on, y1 - y_on)
    motion = geofence_motion()
    counted = geofence.stopping_distance(v_on, motion.reaction_time, motion.max_accel,
                                         motion.braking, motion.max_speed)
    ok = worst < radius and stop <= counted and state.startswith('NEAR') and \
        probe.status.armed
    result('geofence: stops a scripted drive', ok,
           f'the lock closed at {v_on:.2f} m/s; the car stopped {stop:.2f} m on (the geofence '
           f'counts on {counted:.2f} m), its furthest corner {worst:.2f} m from the start of a '
           f'{radius:g} m fence; state {state.split(":")[0]}')
    handle.cancel_goal_async()
    probe.spin_for(2.0)
    # A person backs it in: never held by the lock.
    xb, yb = probe.xy(probe.truth)
    end = time.monotonic() + 8.0
    while time.monotonic() < end:
        msg = TwistStamped()
        msg.header.stamp = probe.get_clock().now().to_msg()
        msg.twist.linear.x = -0.5
        probe.keyboard.publish(msg)
        probe.spin_for(0.05)
    probe.spin_for(5.0, lambda: probe.fence_state.startswith('OK'))
    back = math.hypot(*(a - b for a, b in zip(probe.xy(probe.truth), (xb, yb))))
    ok = back > 1.0 and probe.fence_state.startswith('OK')
    result('geofence: a person drives it back in', ok,
           f'the keyboard backed it {back:.2f} m; then the geofence says '
           f'{probe.fence_state.split(":")[0]}')


def session(launch_args, checks, log_path, label=''):
    """Start bench.launch.py with these arguments, run the checks on it, and stop it."""
    tag = f' ({label})' if label else ''
    env = dict(os.environ)
    print(f'Starting bench mode{tag} on ROS domain {env["ROS_DOMAIN_ID"]} (local only)...',
          flush=True)
    log = open(log_path, 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'aav_bringup', 'bench.launch.py', 'rviz:=false',
                               *launch_args],
                              cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    rclpy.init()
    probe = Probe()
    try:
        up = probe.spin_for(90.0, lambda: probe.status is not None and probe.glob is not None
                            and probe.truth is not None)
        armed = up and probe.arm_and_hold()
        result(f'start-up{tag}', armed, 'the spine is armed and both EKFs are running' if armed
               else 'no spine status, truth or global odometry within 90 s' if not up
               else 'the spine would not stay armed')
        if armed:
            check_contract(probe, env, tag)
            for check in checks:
                check(probe)
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        log.close()


def main():
    args = sys.argv[1:]
    if '-h' in args or '--help' in args:
        print(__doc__)
        return 0
    perception = '--perception' in args
    checks = [check_drive,
              lambda probe: check_estop(probe, 'software e-stop', probe.estop),
              lambda probe: check_estop(probe, 'hardware e-stop', probe.hw_estop)]
    if '--weave' in args:
        checks.append(check_weave)     # first: from the start spot, along the road
    if perception:
        checks.append(check_stop_sign)
    if '--obstacle' in args:
        checks.append(check_obstacle)
    if '--zone-edge' in args:
        checks.append(check_zone_edge)
    lidar = '--lidar' in args
    if lidar:
        checks.append(lambda probe: check_obstacle(probe, sensor='lidar'))
    # AAV_BENCH_LOG: another place for the logs, so two checks at once keep their own.
    log = os.environ.get('AAV_BENCH_LOG', '/tmp/aav-bench-check.log')
    logs = [log]
    # With --obstacle the planner is blind to the box (it spans the lane, and the bench's map
    # is empty: it would plan off the road round it), so the stop zone must stop the car.
    bridge = ['bridge:=python'] if '--python-bridge' in args else []
    session([f'perception:={str(perception).lower()}', f'lidar:={str(lidar).lower()}']
            + (['plan_around:=false'] if {'--obstacle', '--zone-edge'} & set(args) else [])
            + bridge,
            checks, logs[0])
    if '--geofence' in args:
        # Its own run: a 12 m fence would hold the other checks' drives.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-geofence{ext}')
        session([f'geofence_radius:={GEOFENCE_RADIUS:g}'] + bridge, [check_geofence],
                logs[-1], 'geofence')
    if '--lane' in args:
        # Its own run: control_interface drives whenever it sees the lane, so it would take
        # the wheel between the other checks. Nothing else drives here: no Nav2.
        stem, ext = os.path.splitext(log)
        logs.append(f'{stem}-lane{ext}')
        session(['lane:=true', 'nav:=false', f'start_y:={ROAD_Y + LANE_START}'] + bridge,
                [check_lane_keeping], logs[-1], 'lane')

    print('\n| Check | Result | Detail |\n|---|---|---|')
    for name, ok, detail in RESULTS:
        print(f'| {name} | {"✅" if ok else "❌"} | {detail} |')
    print(f'\nThe bench log: {", ".join(logs)}')
    return 0 if all(ok for _, ok, _ in RESULTS) else 1


if __name__ == '__main__':
    sys.exit(main())
