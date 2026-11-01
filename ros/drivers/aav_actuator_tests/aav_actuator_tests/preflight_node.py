"""
The preflight's part of actuator_test: a counter on every sensor, its refusals, its report.

    pixi run preflight --sim                 Gazebo (this computer only)
    pixi run preflight --bench               bench mode (this computer only)
    pixi run preflight --no-drive            the real car, the sensors only: nothing moves
    pixi run preflight [--skip gps lidar]    the real car on stands (refused until it has a
                                             driven-wheel speed: vehicle.yaml speed_wheels)

`pixi run preflight` is `actuator-test preflight --stands`: actuator_test's Run drives
it (the same checks before, the same stops, the word "preflight" typed on the real car).
This module adds what only the preflight needs:

- before: refuse the real car while /vehicle/odom can't see the driven wheels, while the
  legacy bridge's no-timeout lock is on (the C++ ESP32), or while it runs speed_loop.mode pi;
- while it runs, on stands: the IMU must stay still (preflight.ImuWatch), or it stops;
- after: if the car isn't still within 5 s, NOT STOPPED: it publishes /safety/estop true
  (it may set it, never clear it) and keeps sending zeros;
- the PASS / FAIL lines (preflight.py), into summary.txt and preflight.yaml.

Subscribes  everything actuator_test does, and to count and check them:
            vehicle/odom, vehicle/wheels, vehicle/steering, sensors/imu/data_raw,
            sensors/imu/data, sensors/gps/fix, sensors/lidar/points (PointCloud2),
            sensors/radar/points (PointCloud2), sensors/camera/front_left/image_raw
            (Image, reliable), vehicle/battery (BatteryState): preflight.SENSORS
            sensors/gps/fix_velocity  geometry_msgs/TwistWithCovarianceStamped: Doppler
            safety/legacy_uart_bridge/lock  std_msgs/Bool, latched: the no-timeout lock
Publishes   drivers/tests/cmd_vel    (actuator_test's)
            safety/estop             std_msgs/Bool: true only, when the car won't stop
Calls       (actuator_test's) /ego/get_parameters; the backend's get_parameters, which
            includes legacy_uart_bridge's speed_loop.mode

The logic is in preflight.py and safety.py; this file only connects them to ROS.
"""

import math
import os
import sys
import time

from aav_actuator_tests import preflight as pf
from aav_actuator_tests import procedures as pr
from aav_actuator_tests import safety
from aav_actuator_tests.actuator_test import Aborted, ActuatorTest, Run, write_files
from aav_actuator_tests.analysis import STOPPED
from aav_interfaces.msg import ActuatorState, WheelSpeeds
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import BatteryState, Image, Imu, NavSatFix, PointCloud2
from std_msgs.msg import Bool
import yaml

NO_DRIVE_TIME = 15.0   # s of listening with --no-drive
STOP_TIME = 5.0        # s for the car to be still after the last command
ESTOP_TIME = 10.0      # s of zeros after the e-stop, when it won't stop

TYPES = {'odom': Odometry, 'wheels': WheelSpeeds, 'steering': ActuatorState, 'imu_raw': Imu,
         'imu': Imu, 'gps': NavSatFix, 'lidar': PointCloud2, 'radar': PointCloud2,
         'image': Image, 'battery': BatteryState}


def values(kind, msg):
    """One message's plain numbers, for preflight.value_problems."""
    if kind == 'odom':
        return {'speed': msg.twist.twist.linear.x}
    if kind == 'wheels':
        return {'fl': msg.front_left, 'fr': msg.front_right, 'rl': msg.rear_left,
                'rr': msg.rear_right}
    if kind == 'steering':
        return {'measured': msg.measured}
    if kind in ('imu_raw', 'imu'):
        a, g = msg.linear_acceleration, msg.angular_velocity
        return {'ax': a.x, 'ay': a.y, 'az': a.z, 'gx': g.x, 'gy': g.y, 'gz': g.z}
    if kind == 'gps':
        return {'status': msg.status.status, 'lat': msg.latitude, 'lon': msg.longitude}
    if kind in ('lidar', 'radar'):
        return {'points': msg.width * msg.height}
    if kind == 'image':
        return {'width': msg.width, 'height': msg.height}
    if kind == 'battery':
        return {'voltage': msg.voltage}
    raise ValueError(kind)


def no_drive_steps(s):
    """--no-drive: stand still and listen; zeros only."""
    return [pr.still(), pr.Step(pr.PF_SETTLE, 0.0, until=pr.Until.TIME, value=NO_DRIVE_TIME)]


NO_DRIVE = pr.Test('preflight', '3.P', 3, 'preflight, the sensors only (--no-drive): '
                   'nothing moves', no_drive_steps)


class PreflightNode(ActuatorTest):
    """actuator_test's node, with a counter on every sensor."""

    def __init__(self, sim):
        super().__init__(sim)
        self.samples = {s.name: pf.Samples() for s in pf.SENSORS}
        self.v['gps_speed'] = float('nan')
        self.lock = None   # the legacy bridge's no-timeout lock: None until heard
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for s in pf.SENSORS:
            qos = reliable if s.kind == 'image' else qos_profile_sensor_data
            self.create_subscription(TYPES[s.kind], s.topic.lstrip('/'),
                                     lambda m, s=s: self.on_sample(s, m), qos)
        self.create_subscription(TwistWithCovarianceStamped, 'sensors/gps/fix_velocity',
                                 self.on_doppler, qos_profile_sensor_data)
        self.create_subscription(Bool, 'safety/legacy_uart_bridge/lock', self.on_lock, latched)
        self.estop_pub = self.create_publisher(Bool, 'safety/estop', 10)

    def on_sample(self, sensor, msg):
        self.samples[sensor.name].add(self.seconds(), values(sensor.kind, msg))

    def on_doppler(self, msg):
        t = msg.twist.twist.linear
        self.v['gps_speed'] = math.hypot(t.x, t.y)

    def on_lock(self, msg):
        self.lock = bool(msg.data)

    def estop(self):
        """Set the e-stop (never clear it: only a person does, pixi run estop-clear)."""
        self.estop_pub.publish(Bool(data=True))


class PreflightRun(Run):
    """actuator_test's Run, with the preflight's refusals, live IMU watch and report."""

    def __init__(self, node, args):
        super().__init__(node, args, test=NO_DRIVE if args.no_drive else pr.TESTS['preflight'])
        self.lines = []
        self.end = None
        self.result = 0
        self.watch = pf.ImuWatch()
        self.ground = False

    def prepare(self):
        node, a = self.node, self.args
        node.spin_for(3.0, lambda: bool(node.sources))
        node.spin_for(1.0)   # the other announcers, and the latched lock
        kind = safety.source_kind(node.sources)
        speed_wheels = None
        try:
            from aav_description import vehicle
            speed_wheels = vehicle.load()['vehicle'].get('speed_wheels')
        except (KeyError, OSError):
            pass
        legacy = node.backend_params().get('legacy_uart_bridge', {})
        why = safety.preflight_refusal(kind, not a.no_drive, speed_wheels, node.lock,
                                       legacy.get('speed_loop.mode'))
        if why:
            self.say(f'REFUSED: {why}')
            return 1
        if not a.no_drive:
            self.checklist = safety.PREFLIGHT_CHECKLIST   # Run prints it on the real car
        self.ground = kind in (safety.Kind.SIM, safety.Kind.BENCH)
        return super().prepare()

    def tick(self):
        super().tick()
        if self.ground or self.args.no_drive:
            return
        step = self.runner.step
        v = self.node.v
        if step is None:
            return
        if step.phase in (pf.STILL, pf.SETTLE):
            if step.phase == pf.SETTLE:
                self.watch.rest(v['ax'], v['ay'])
            return
        if self.watch.bias is None and not self.watch.start():
            return   # no IMU at rest: the sensor line fails it; nothing to watch
        why = self.watch.add(self.node.seconds(), v['ax'], v['ay'], v['gz'],
                             step.phase in (pf.LEFT, pf.RIGHT, pf.CENTRE))
        if why:
            raise Aborted(why)

    def still(self):
        speed = self.node.v['speed']
        return math.isfinite(speed) and abs(speed) < STOPPED and self.fresh_odom()

    def stop_car(self):
        """Zeros until still; if not still in 5 s: NOT STOPPED, the e-stop, more zeros."""
        node = self.node
        if not self.driving:
            return
        self.end = node.seconds()
        wheelbase = self.settings.wheelbase
        end = time.monotonic() + STOP_TIME
        still_since = None
        while time.monotonic() < end:
            node.send(0.0, 0.0, wheelbase)
            rclpy.spin_once(node, timeout_sec=0.02)
            if self.still():
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > 0.3:
                    break
            else:
                still_since = None
        if still_since is None:
            self.say('NOT STOPPED: E-STOP NOW. The car is not still 5 s after the stop '
                     'command; publishing /safety/estop true and sending zeros')
            self.result = 6
            end = time.monotonic() + ESTOP_TIME
            while time.monotonic() < end:
                node.estop()
                node.send(0.0, 0.0, wheelbase)
                rclpy.spin_once(node, timeout_sec=0.05)
            self.say('The e-stop stays set: clear it by hand once the car is safe '
                     '(pixi run estop-clear)')
        for _ in range(5):
            node.send(0.0, 0.0, wheelbase)
            time.sleep(0.02)
        self.driving = False
        speed = node.v['speed']
        self.say(f'Stopped: zero commands sent; speed now {speed:.2f} m/s'
                 if math.isfinite(speed) else 'Zero commands sent; the speed is unknown')

    def report(self):
        if not self.rows:
            return
        node = self.node
        node.spin_for(0.5)   # the last messages in flight
        end = self.end or node.seconds()
        still = [(a + self.t0, b + self.t0) for a, b in pf.still_windows(self.rows)]
        lock = self.settings.max_steering
        acts = self.cfg.get('actuators', {})
        centre = float(acts.get('steer_offset', 0.0))
        tolerance = 2.0 * float(acts.get('steer_deadband', 0.005)) + 0.02
        open_loop = str(self.shaping.get('speed_loop', '')).startswith('none: open loop')
        lines = pf.sensor_lines(node.samples, self.t0, end, still, lock, self.ground,
                                self.args.skip)
        if self.args.no_drive:
            lines += [pf.Line(pf.Status.SKIP, 'the drive', '--no-drive: nothing moved')]
        else:
            seen = None
            if not node.has_steering and sys.stdin.isatty() and not self.ground:
                try:
                    text = input('No measured steering angle. Did the front wheels turn '
                                 'left, then right, then straight? [y/n]: ').strip().lower()
                except EOFError:
                    text = ''
                seen = text.startswith('y') if text else None
            lines += pf.motion_lines(self.rows, self.settings.cap, lock, centre, tolerance,
                                     open_loop, seen)
            lines += pf.stands_lines(self.rows, self.ground)
        if self.abort:
            lines.insert(0, pf.Line(pf.Status.FAIL, 'the run', f'cut short: {self.abort}'))
        if self.result == 6:
            lines.insert(0, pf.Line(pf.Status.FAIL, 'the stop', 'NOT STOPPED within 5 s: '
                                    'the e-stop was set'))
        self.lines = lines
        text = pf.table(lines)
        if self.ground:
            text += (f'\nNOTE: {self.kind.value}: the car is on the ground, so the stands '
                     'lines check the opposite (the IMU and the GPS agree with the wheels)')
        elif not self.args.no_drive:
            text += ('\nNOTE: if the wheel speed saw the spinning wheels, the sensor watchdog '
                     'has latched DISAGREE against the still GPS: pixi run watchdog-clear')
        self.say()
        self.say(text)
        write_files(self, text, '')
        with open(os.path.join(self.folder, 'preflight.yaml'), 'w') as f:
            yaml.safe_dump({'passed': pf.passed(lines), 'counts': pf.counts(lines),
                            'skip': list(self.args.skip), 'no_drive': self.args.no_drive,
                            'lines': [{'status': x.status.value, 'check': x.check,
                                       'detail': x.detail} for x in lines]},
                           f, sort_keys=False)
        self.say(f'Saved in {self.folder}')
        if not pf.passed(lines):
            self.result = max(self.result, 5)
