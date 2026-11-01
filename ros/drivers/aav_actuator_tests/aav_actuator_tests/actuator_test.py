"""
actuator_test: docs/tests.md's stage-3 and stage-4 tests, run as a twist_mux driver.

    pixi run actuator-test --list                       the tests
    pixi run actuator-test steer-sweep --sim            in Gazebo (this computer only)
    pixi run actuator-test steer-sweep --stands         the real car, wheels off the ground
    pixi run actuator-test speed-up                     the real car, in an empty lot
    pixi run actuator-test --analyse ~/aav-tests/<run>  a run's report again, from its CSV

It drives through twist_mux's `tests` slot (priority 60): the joystick (100) and the
keyboard (90) always win, and /safety/estop locks it out. It stops the car (zero commands)
on Ctrl-C, on any error, on /safety/estop, when someone else takes over, when the speed,
distance or time leave the test's envelope, and when the speed reading goes stale. On the
real car it asks the person to type the test's name first, and caps the speed at 1 m/s
(0.3 m/s on stands). Each run goes to its own folder: log.csv, summary.txt, meta.yaml,
and with --bag a rosbag. It prints vehicle.yaml lines to paste; it never writes them.
log.csv keeps each value's arrival time (the t_* columns); meta.yaml's shaping says what
shaped the commands: the backend and its firmware, the speed loop's profile, ego's limits.

Subscribes  vehicle/odom             nav_msgs/Odometry: the measured speed
            vehicle/steering         aav_interfaces/ActuatorState
            vehicle/throttle         aav_interfaces/ActuatorState
            vehicle/brake            aav_interfaces/ActuatorState
            vehicle/wheels           aav_interfaces/WheelSpeeds
            sensors/imu/data_raw     sensor_msgs/Imu: forward acceleration, yaw rate
            localization/odom/local  nav_msgs/Odometry: the pose (circles, straight)
            sensors/gps/fix          sensor_msgs/NavSatFix
            safety/estop             std_msgs/Bool: true stops the test
            safety/geofence/state    std_msgs/String: NEAR, OUTSIDE, LOST or JUMP stops a
                                     test on the ground (its slot, 60, is above the
                                     geofence's lock); silent refuses one on the real car
            control/cmd_vel          geometry_msgs/TwistStamped: twist_mux's choice
            ego/actuator_cmd         ackermann_msgs/AckermannDriveStamped: what ego told the
                                     backend (logged: the fits' plant boundary)
            system/source            aav_interfaces/DataSource, latched: REAL or SIM
Publishes   drivers/tests/cmd_vel    geometry_msgs/TwistStamped, 50 Hz while a test runs
Calls       /ego/get_parameters      rcl_interfaces/GetParameters: its limits
            /ego/set_parameters      rcl_interfaces/SetParameters: max_steering_rate, lifted
                                     during steer-rate (3.11) only, then put back
            /<backend>/get_parameters  the backend's speed loop profile, full throttle
                                     speed... (reports.SHAPING_PARAMS), for meta.yaml
Runs        ros2 bag record          with --bag

The logic is in procedures.py (the steps), safety.py (the rules) and reports.py (the
numbers); this file only connects them to ROS.
"""

import argparse
import collections
import csv
import datetime
import math
import os
import signal
import subprocess
import sys
import time
import warnings

from aav_actuator_tests import procedures as pr
from aav_actuator_tests import reports
from aav_actuator_tests import safety
from aav_actuator_tests.analysis import STOPPED
from aav_description import vehicle
from aav_interfaces.msg import ActuatorState, DataSource, WheelSpeeds
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String
import yaml

NAN = float('nan')
RATE_HZ = 50.0
ODOM_STALE = 0.5       # s without /vehicle/odom: stop (the envelope can't be checked)
TAKEOVER_TIME = 0.3    # s of twist_mux passing someone else's command: stop
MUX_SILENT = 2.0       # s of driving with nothing from twist_mux: stop
EGO_PARAMS = ('max_speed', 'max_steering_rate', 'max_steering_angle', 'wheelbase',
              'max_acceleration', 'max_deceleration')
BAG_TOPICS = ('/vehicle/odom', '/vehicle/status', '/vehicle/steering', '/vehicle/throttle',
              '/vehicle/brake', '/vehicle/wheels', '/sensors/imu/data_raw', '/sensors/gps/fix',
              '/localization/odom/local', '/drivers/tests/cmd_vel', '/control/cmd_vel',
              '/ego/actuator_cmd', '/safety/estop', '/system/source', '/tf', '/tf_static')


class Aborted(Exception):
    """A safety rule stopped the test."""


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ActuatorTest(Node):

    def __init__(self, sim):
        super().__init__('actuator_test',
                         parameter_overrides=[Parameter('use_sim_time', value=sim)])
        self.v = dict.fromkeys(reports.COLUMNS, NAN)   # the latest of everything
        self.arrived = {}                              # t_* column -> arrival (s, clock)
        self.odom_time = None
        self.estop = False
        self.geofence = None          # (arrival s on the steady clock, the state's line)
        self.sources = {}
        self.mux = collections.deque(maxlen=200)       # (arrival s, v, w) from twist_mux
        self.sent = collections.deque(maxlen=50)       # our last commands (v, w)
        self.has_steering = False

        sensor = qos_profile_sensor_data
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(TwistStamped, 'drivers/tests/cmd_vel', 10)
        self.create_subscription(Odometry, 'vehicle/odom', self.on_odom, sensor)
        for name in ('steering', 'throttle', 'brake'):
            self.create_subscription(ActuatorState, f'vehicle/{name}',
                                     lambda m, n=name: self.on_actuator(n, m), sensor)
        self.create_subscription(WheelSpeeds, 'vehicle/wheels', self.on_wheels, sensor)
        self.create_subscription(Imu, 'sensors/imu/data_raw', self.on_imu, sensor)
        self.create_subscription(Odometry, 'localization/odom/local', self.on_pose, sensor)
        self.create_subscription(NavSatFix, 'sensors/gps/fix', self.on_fix, sensor)
        self.create_subscription(Bool, 'safety/estop', self.on_estop, 10)
        self.create_subscription(String, 'safety/geofence/state', self.on_geofence, 10)
        self.create_subscription(TwistStamped, 'control/cmd_vel', self.on_mux, 10)
        self.create_subscription(DataSource, 'system/source', self.on_source, latched)
        self.create_subscription(AckermannDriveStamped, 'ego/actuator_cmd', self.on_ego, sensor)
        self.ego = AsyncParameterClient(self, 'ego')

    # --- listening ---------------------------------------------------------------------

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def arrive(self, column):
        """Note when a message arrived, for its t_* column (true times for the fits)."""
        self.arrived[column] = self.seconds()

    def on_odom(self, msg):
        self.v['speed'] = msg.twist.twist.linear.x
        self.odom_time = self.seconds()
        self.arrive('t_speed')

    def on_actuator(self, name, msg):
        cmd_key = {'steering': 'steer_cmd'}.get(name, f'{name}_cmd')
        val_key = {'steering': 'steer'}.get(name, name)
        self.v[cmd_key] = msg.command
        self.v[val_key] = msg.measured
        self.arrive(reports.STAMPS[val_key])
        if name == 'steering' and math.isfinite(msg.measured):
            self.has_steering = True

    def on_wheels(self, msg):
        for k in ('fl', 'fr', 'rl', 'rr'):
            self.v[f'wheel_{k}'] = getattr(msg, {'fl': 'front_left', 'fr': 'front_right',
                                                 'rl': 'rear_left', 'rr': 'rear_right'}[k])
        self.arrive('t_wheels')

    def on_imu(self, msg):
        self.v['ax'] = msg.linear_acceleration.x
        self.v['ay'] = msg.linear_acceleration.y
        self.v['gz'] = msg.angular_velocity.z
        self.arrive('t_imu')

    def on_pose(self, msg):
        p = msg.pose.pose
        self.v['x'], self.v['y'], self.v['yaw'] = p.position.x, p.position.y, yaw_of(p.orientation)
        self.arrive('t_pose')

    def on_fix(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.v['lat'], self.v['lon'] = msg.latitude, msg.longitude
            self.arrive('t_fix')

    def on_ego(self, msg):
        self.v['ego_speed'] = msg.drive.speed
        self.v['ego_steer'] = msg.drive.steering_angle
        self.arrive('t_ego')

    def on_estop(self, msg):
        self.estop = bool(msg.data)

    def on_geofence(self, msg):
        self.geofence = (time.monotonic(), msg.data)

    def geofence_says(self):
        """Return (state, age) of the geofence's last line, (None, None) if never heard."""
        if self.geofence is None:
            return None, None
        return self.geofence[1], time.monotonic() - self.geofence[0]

    def on_mux(self, msg):
        self.mux.append((time.monotonic(), msg.twist.linear.x, msg.twist.angular.z))

    def on_source(self, msg):
        self.sources[msg.announcer] = [s.source for s in msg.sensors]

    # --- talking -----------------------------------------------------------------------

    def send(self, speed, steering, wheelbase):
        w = speed * math.tan(steering) / wheelbase if speed else 0.0
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(speed)
        msg.twist.angular.z = float(w)
        self.pub.publish(msg)
        self.sent.append((msg.twist.linear.x, msg.twist.angular.z))

    def spin_for(self, seconds, until=None):
        """Spin for up to `seconds` of wall-clock time, or until `until()` is true."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if until is not None and until():
                return True
        return False

    def wait(self, future, seconds=3.0):
        self.spin_for(seconds, future.done)
        return future.result() if future.done() else None

    def ego_params(self):
        """Return ego's limits, or {} if ego doesn't answer."""
        if not self.ego.wait_for_services(timeout_sec=3.0):
            return {}
        res = self.wait(self.ego.get_parameters(list(EGO_PARAMS)))
        if res is None:
            return {}
        return {n: parameter_value_to_python(v) for n, v in zip(EGO_PARAMS, res.values)
                if parameter_value_to_python(v) is not None}

    def backend_params(self, seconds=1.0):
        """Return reports.SHAPING_PARAMS' values from each of those nodes that answers."""
        running = set(self.get_node_names())
        out = {}
        for node, names in reports.SHAPING_PARAMS.items():
            if node not in running:
                continue
            client = AsyncParameterClient(self, node)
            if not client.wait_for_services(timeout_sec=seconds):
                continue
            res = self.wait(client.get_parameters(list(names)), seconds)
            if res is None:
                continue
            values = {n: parameter_value_to_python(v) for n, v in zip(names, res.values)}
            out[node] = {n: v for n, v in values.items() if v is not None}
        return out

    def set_ego(self, name, value):
        """Set one of ego's parameters; returns why it failed, or None."""
        res = self.wait(self.ego.set_parameters([Parameter(name, value=float(value))]))
        if res is None:
            return 'ego did not answer'
        r = res.results[0]
        return None if r.successful else r.reason


class Run:
    """One test run: the checks before, the loop, the stop, the files."""

    def __init__(self, node, args, test=None):
        self.node = node
        self.args = args
        self.test = test or pr.TESTS[args.test]
        self.rows = []
        self.driving = False
        self.abort = None
        self.ego_restore = None
        self.bag = None
        self.manual = False
        self.paused = 0.0
        self.shaping = {}
        self.history = collections.deque(maxlen=250)   # (t, our last command, speed)
        self.last_cmd = 0.0
        self.checklist = None                          # None: the stage's own

    def say(self, text=''):
        print(text, flush=True)

    # --- before ------------------------------------------------------------------------

    def prepare(self):
        """Every check before the car moves. Returns an exit code to stop with, or None."""
        a, node = self.args, self.node
        self.cfg = vehicle.load()
        self.ctx = reports.context(self.cfg)
        self.say(f'actuator test {self.test.name} (docs/tests.md {self.test.number}): '
                 f'{self.test.title}')
        self.say('Listening for /system/source ...')
        node.spin_for(3.0, lambda: bool(node.sources))
        node.spin_for(0.5)   # the other announcers
        kind = safety.source_kind(node.sources)
        self.decision = d = safety.decide(kind, a.sim, a.stands, self.test.stage, a.max_speed,
                                          bench=a.bench)
        self.kind = kind
        if not d.allowed:
            self.say(f'REFUSED: {d.reason}')
            return 1
        self.ego = node.ego_params()
        if not self.ego:
            if d.real:
                # The real car: without ego's limits we don't know its speed cap or
                # steering, so we don't move it.
                self.say('REFUSED: ego did not answer (/ego/get_parameters): start the car '
                         "stack first; on the real car we don't drive without ego's limits")
                return 1
            self.say('WARNING: ego did not answer (/ego/get_parameters): is the stack running?')
        self.shaping = reports.shaping(list(node.sources), node.backend_params(), self.ego)
        self.ctx = reports.with_shaping(self.ctx, self.shaping)
        self.say(f"Backend: {self.shaping['backend']}; speed loop: "
                 f"{self.shaping['speed_loop']}")
        wheelbase = float(self.ego.get('wheelbase', self.ctx.wheelbase))
        max_steer = float(self.ego.get('max_steering_angle', self.ctx.max_steering))
        settings = pr.Settings(cap=d.cap, max_steering=max_steer, wheelbase=wheelbase,
                               creep=min(0.2, d.cap), speeds=tuple(a.speeds or ()),
                               angles=tuple(a.angles or ()))
        settings, note = pr.with_ego_limit(settings, self.ego.get('max_speed'))
        if note:
            self.say(f'NOTE: {note}')
        self.settings = settings
        self.plan = pr.plan(self.test, settings, a.max_distance, a.max_time)
        self.envelope = safety.Envelope(safety.speed_limit(settings.cap), self.plan.distance,
                                        self.plan.time)
        self.say(f'Driving: {d.reason} ({kind.value}). Speed cap {settings.cap:g} m/s; '
                 f'stops above {self.envelope.max_speed:.2f} m/s, after '
                 f'{self.envelope.max_distance:.0f} m or {self.envelope.max_time:.0f} s.')
        if d.confirm:
            self.say()
            self.say('THE REAL CAR. Check, out loud:')
            for item in self.checklist or safety.checklist(a.stands):
                self.say(f'  [ ] {item}')
            if not a.stands:
                self.say(f'  [ ] Room for up to {self.envelope.max_distance:.0f} m of driving')
            self.say('To stop at any time: Ctrl-C here, the e-stop, or the joystick deadman.')
            if not sys.stdin.isatty():
                self.say('REFUSED: the real car needs a person at the keyboard to confirm')
                return 1
            try:
                typed = input(f'Type "{self.test.name}" to start (anything else cancels): ')
            except EOFError:
                typed = None
            if not safety.confirmed(typed, self.test.name):
                self.say('Cancelled: nothing moved.')
                return 1
        return self.wait_for_data()

    def wait_for_data(self):
        node = self.node
        self.say('Waiting for the speed (/vehicle/odom) ...')
        if not node.spin_for(10.0, lambda: self.fresh_odom()):
            self.say('REFUSED: no /vehicle/odom: the measured speed is needed to stop safely')
            return 1
        node.spin_for(1.0)   # the rest of the sensors
        if node.estop:
            self.say('REFUSED: /safety/estop is true. Clear it first (pixi run estop-clear)')
            return 1
        why = safety.geofence_stop(*node.geofence_says(), self.args.stands, self.decision.real,
                                   starting=True)
        if why:
            self.say(f'REFUSED: {why}')
            return 1
        if not math.isfinite(node.v['ax']):
            self.say('WARNING: no IMU (/sensors/imu/data_raw): no g figures from it')
        if self.test.stage == 3 and not node.has_steering:
            if sys.stdin.isatty():
                self.manual = True
                self.say('NOTE: no measured steering angle (/vehicle/steering). At each '
                         'measuring point the car stops and asks you for the angle.')
            else:
                self.say('WARNING: no measured steering angle, and nobody to ask')
        return None

    def fresh_odom(self):
        t = self.node.odom_time
        return t is not None and self.node.seconds() - t < ODOM_STALE and t > 0

    # --- driving -----------------------------------------------------------------------

    def drive(self):
        node, a = self.node, self.args
        if self.test.ego_steer_rate and node.ego.services_are_ready() and self.ego:
            old = self.ego.get('max_steering_rate')
            why = node.set_ego('max_steering_rate', self.test.ego_steer_rate)
            if why:
                self.say(f"WARNING: could not lift ego's steering rate ({why}): the result "
                         "may be ego's ramp, not the motor")
            else:
                self.ego_restore = old
                self.shaping['ego_changed'] = {'max_steering_rate': self.test.ego_steer_rate}
                self.say(f"ego's max_steering_rate lifted from {old} to "
                         f'{self.test.ego_steer_rate} rad/s for this test')
        self.folder = run_folder(a.out, self.test.name, self.kind)
        if a.bag:
            self.bag = start_bag(os.path.join(self.folder, 'bag'), a.sim)
        self.runner = pr.Runner(list(self.plan.steps))
        self.t0 = node.seconds()
        self.last = {'t': self.t0, 'wall': time.monotonic(), 'heading': 0.0, 'distance': 0.0,
                     'done': 0, 'grace': time.monotonic() + 1.0, 'other': None,
                     'started': time.monotonic()}
        self.driving = True
        timer = node.create_timer(1.0 / RATE_HZ, self.tick)
        self.say(f'Running: {len(self.plan.steps)} steps. Ctrl-C stops the car.')
        try:
            while self.abort is None and not self.runner.finished:
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() - self.last['wall'] > 3.0:
                    raise Aborted('the clock stopped (the simulator or this computer stalled)')
        finally:
            node.destroy_timer(timer)

    def tick(self):
        node, last = self.node, self.last
        now = node.seconds()
        dt = max(0.0, now - last['t'])
        last['t'], last['wall'] = now, time.monotonic()
        v = node.v
        speed = v['speed'] if math.isfinite(v['speed']) else 0.0
        if math.isfinite(v['gz']):
            last['heading'] += v['gz'] * dt
        last['distance'] += abs(speed) * dt
        elapsed = now - self.t0 - self.paused

        grace = time.monotonic() < last['grace']
        if node.estop:
            raise Aborted('/safety/estop is true')
        why = safety.geofence_stop(*node.geofence_says(), self.args.stands, self.decision.real)
        if why:
            raise Aborted(why)
        if not grace and not self.fresh_odom():
            raise Aborted(f'no /vehicle/odom for {ODOM_STALE} s: the speed is unknown')
        why = self.envelope.breach(speed, last['distance'], elapsed)
        if why:
            raise Aborted(why)
        self.check_mux()
        # Told to drive but nothing measured (front sensors on stands), or a speed that
        # rises while the command falls (a stuck throttle): stop.
        self.history.append((elapsed, self.last_cmd, speed))
        why = safety.unseen(self.history) or safety.runaway(self.history)
        if why:
            raise Aborted(why)

        obs = pr.Obs(elapsed, speed, last['distance'], last['heading'])
        cmd = self.runner.tick(obs)
        self.after_steps()
        if cmd is None:
            return
        speed_cmd = min(cmd.speed, self.settings.cap)
        node.send(speed_cmd, cmd.steering, self.settings.wheelbase)
        self.last_cmd = speed_cmd
        row = dict(v)
        row.update(t=elapsed, phase=cmd.phase, cmd_speed=speed_cmd, cmd_steer=cmd.steering,
                   heading=last['heading'], distance=last['distance'], manual_steer=NAN)
        # Arrival times on the row's clock: t = now - t0 - paused.
        offset = self.t0 + self.paused
        row.update({k: arrived - offset for k, arrived in node.arrived.items()})
        self.rows.append(row)

    def check_mux(self):
        """Stop when twist_mux passes someone else's command, or nothing at all."""
        node, last = self.node, self.last
        wall = time.monotonic()
        recent = [m for m in node.mux if wall - m[0] < 0.2]
        if not node.mux or wall - node.mux[-1][0] > MUX_SILENT:
            if wall - last['started'] > MUX_SILENT:
                raise Aborted('twist_mux passes nothing on /control/cmd_vel: is the tests slot '
                              'in its config (aav_command_mux twist_mux.yaml)? Or a lock (e-stop, '
                              'stop sign) holds it')
            return
        if recent and safety.taken_over(node.sent, recent[-1][1:]):
            if last['other'] is None:
                last['other'] = wall
            elif wall - last['other'] > TAKEOVER_TIME:
                raise Aborted('someone else is driving (a higher priority in twist_mux: the '
                              'joystick or the keyboard)')
        else:
            last['other'] = None

    def after_steps(self):
        """On stands with no steering sensor: stop at each measuring point and ask."""
        done = self.runner.done
        new, self.last['done'] = done[self.last['done']:], len(done)
        for d in new:
            if d.timed_out:
                self.say(f'  step "{d.step.phase}" timed out after {d.end - d.start:.1f} s')
                if d.step.must:
                    raise Aborted(f'step "{d.step.phase}" did not finish in '
                                  f'{d.end - d.start:.0f} s: stopping rather than going on')
            if self.manual and d.step.measure:
                self.ask_angle(d.step)

    def ask_angle(self, step):
        # Nothing is published meanwhile: twist_mux and ego time out and the wheels stop,
        # with the steering held where it is.
        started = time.monotonic()
        try:
            text = input(f'  {step.phase}: the wheel angle in degrees, + left '
                         '(Enter to skip): ')
        except EOFError:
            text = ''
        try:
            angle = math.radians(float(text))
        except ValueError:
            angle = NAN
        for row in reversed(self.rows):
            if row['phase'] == step.phase:
                row['manual_steer'] = angle
                break
        waited = time.monotonic() - started
        self.paused += waited
        self.last['grace'] = time.monotonic() + 1.0
        self.last['started'] = time.monotonic()
        self.history.clear()   # the wheels stopped while nothing was sent: start afresh
        self.last['wall'] = time.monotonic()

    # --- after -------------------------------------------------------------------------

    def stop_car(self):
        """Zero commands until the car stands still (at most 5 s), then a few more."""
        node = self.node
        if not self.driving:
            return
        end = time.monotonic() + 5.0
        still_since = None
        while time.monotonic() < end:
            node.send(0.0, 0.0, self.settings.wheelbase)
            rclpy.spin_once(node, timeout_sec=0.02)
            speed = node.v['speed']
            if math.isfinite(speed) and abs(speed) < STOPPED and self.fresh_odom():
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > 0.3:
                    break
            else:
                still_since = None
        for _ in range(5):
            node.send(0.0, 0.0, self.settings.wheelbase)
            time.sleep(0.02)
        self.driving = False
        speed = node.v['speed']
        self.say(f'Stopped: zero commands sent; speed now {speed:.2f} m/s'
                 if math.isfinite(speed) else 'Stopped: zero commands sent')

    def restore(self):
        if self.ego_restore is not None:
            why = self.node.set_ego('max_steering_rate', self.ego_restore)
            self.say(f"ego's max_steering_rate back to {self.ego_restore} rad/s"
                     if why is None else
                     f"WARNING: ego's max_steering_rate is still lifted ({why}): restart ego")
            self.ego_restore = None
        if self.bag is not None:
            stop_bag(self.bag)
            self.bag = None

    def report(self):
        if not self.rows:
            return
        rep = reports.analyse(self.test.name, self.rows, self.ctx)
        if self.abort:
            rep.notes.insert(0, f'ABORTED: {self.abort}. The numbers cover only what ran, '
                                'so no vehicle.yaml lines: run it again to the end')
            rep.yaml.clear()
        text = reports.table(rep)
        lines = reports.yaml_lines(rep)
        self.say()
        self.say(text)
        if lines:
            self.say()
            self.say(lines)
        write_files(self, text, lines)
        self.say()
        self.say(f'Saved in {self.folder}')


def run_folder(base, test, kind):
    base = os.path.expanduser(base or os.environ.get('AAV_TESTS_DIR', '~/aav-tests'))
    stamp = datetime.datetime.now().strftime('%Y-%m-%d-%H%M%S')
    suffix = '' if kind is safety.Kind.REAL else f'-{kind.value.lower()}'
    folder = os.path.join(base, f'{stamp}-{test}{suffix}')
    os.makedirs(folder, exist_ok=True)
    return folder


def write_files(run, text, lines):
    with open(os.path.join(run.folder, 'log.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(reports.COLUMNS)
        for row in run.rows:
            w.writerow([row['phase'] if k == 'phase' else reports.number(row[k])
                        for k in reports.COLUMNS])
    with open(os.path.join(run.folder, 'summary.txt'), 'w') as f:
        f.write(text + '\n\n' + (lines + '\n' if lines else ''))
    meta = {
        'test': run.test.name, 'number': run.test.number, 'source': run.kind.value,
        'stands': bool(run.args.stands), 'cap': run.settings.cap,
        'envelope': {'max_speed': run.envelope.max_speed,
                     'max_distance': run.envelope.max_distance,
                     'max_time': run.envelope.max_time},
        'aborted': run.abort, 'ego': run.ego, 'shaping': run.shaping,
        'vehicle_yaml': dict(run.ctx.current),
        'wheelbase': run.ctx.wheelbase, 'max_steering': run.ctx.max_steering,
        'gps_x': run.ctx.gps_x,
        'steps': [{'phase': d.step.phase, 'start': d.start, 'end': d.end,
                   'timed_out': d.timed_out} for d in run.runner.done],
    }
    with open(os.path.join(run.folder, 'meta.yaml'), 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)


def start_bag(path, sim):
    cmd = ['ros2', 'bag', 'record', '-o', path] + (['--use-sim-time'] if sim else [])
    cmd += list(BAG_TOPICS)
    # Its own process group, so stopping it can't reach anything else.
    return subprocess.Popen(cmd, start_new_session=True, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


def stop_bag(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def analyse_folder(folder):
    """Make a run's report again from its log.csv and meta.yaml (no ROS needed)."""
    with open(os.path.join(folder, 'meta.yaml')) as f:
        meta = yaml.safe_load(f)
    if meta['test'] == 'preflight':   # its lines need the sensors' counts: kept as written
        with open(os.path.join(folder, 'summary.txt')) as f:
            print(f.read().rstrip())
        return 0
    with open(os.path.join(folder, 'log.csv'), newline='') as f:
        rows = reports.parse_rows(list(csv.DictReader(f)))
    rep = reports.analyse(meta['test'], rows, reports.context_from_meta(meta))
    print(reports.table(rep))
    lines = reports.yaml_lines(rep)
    if lines:
        print()
        print(lines)
    return 0


def list_tests():
    print('Test          #     Where      What')
    for t in pr.TESTS.values():
        where = 'on stands' if t.stage == 3 else 'in a lot'
        print(f'{t.name:<13} {t.number:<5} {where:<10} {t.title}')


def parse(argv):
    p = argparse.ArgumentParser(
        prog='actuator-test',
        description="docs/tests.md's stage-3 and stage-4 tests: drive, log, report.")
    p.add_argument('test', nargs='?', choices=list(pr.TESTS), help='the test to run')
    p.add_argument('--list', action='store_true', help='list the tests')
    where = p.add_mutually_exclusive_group()
    where.add_argument('--sim', action='store_true',
                       help='Gazebo: simulated time, this computer only (pixi run sim first)')
    where.add_argument('--bench', action='store_true',
                       help='bench mode, this computer only (pixi run bench first)')
    p.add_argument('--skip', nargs='+', default=[], metavar='SENSOR',
                   help='preflight: sensors meant to be off (odom, wheels, steering, imu, '
                        'imu_filtered, gps, lidar, radar, camera, battery)')
    p.add_argument('--no-drive', action='store_true',
                   help='preflight: the sensors only, nothing moves (the real car until it '
                        'has a driven-wheel speed)')
    p.add_argument('--stands', action='store_true',
                   help='the wheels are off the ground (stage 3): speed capped at 0.3 m/s')
    p.add_argument('--max-speed', type=float, default=None,
                   help='m/s: the speed cap (default 1.0 on the car, at most 1.0; '
                        '3.0 in Gazebo, at most 5.0)')
    p.add_argument('--speeds', type=float, nargs='+',
                   help='m/s for speed-up, coast, brake, throttle-map')
    p.add_argument('--angles', type=float, nargs='+', help='rad for circles (each both ways)')
    p.add_argument('--max-distance', type=float, help='m: stop beyond this (default: per test)')
    p.add_argument('--max-time', type=float, help='s: stop after this (default: per test)')
    p.add_argument('--out', help='folder for the runs (default ~/aav-tests, or $AAV_TESTS_DIR)')
    p.add_argument('--bag', action='store_true', help='also record a rosbag of the run')
    p.add_argument('--analyse', metavar='RUN_FOLDER',
                   help="make a run's report again from its log.csv (no ROS)")
    args = p.parse_args(argv)
    if not (args.list or args.analyse or args.test):
        p.error('name a test (--list shows them)')
    if args.test != 'preflight' and (args.skip or args.no_drive):
        p.error('--skip and --no-drive are for the preflight only')
    if args.test == 'preflight':
        from aav_actuator_tests.preflight import NAMES
        unknown = sorted(set(args.skip) - set(NAMES))
        if unknown:
            p.error(f'--skip: no sensor {", ".join(unknown)} (pick from {", ".join(NAMES)})')
        if not args.stands:
            p.error('the preflight runs on stands: add --stands (pixi run preflight does)')
        if args.speeds or args.angles or args.max_distance or args.max_time:
            p.error('the preflight has fixed steps: no --speeds, --angles, --max-distance '
                    'or --max-time')
    return args


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    if args.list:
        list_tests()
        return 0
    if args.analyse:
        return analyse_folder(args.analyse)

    # Ctrl-C can land inside the executor, which then leaves a coroutine behind: harmless.
    warnings.filterwarnings('ignore', message="coroutine 'Executor._make_handler")
    # No rclpy signal handler: Ctrl-C must leave ROS working, to send the stop commands.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    if args.test == 'preflight':
        # Its own counters and report (preflight_node.py); the same Run underneath.
        from aav_actuator_tests.preflight_node import PreflightNode, PreflightRun
        node = PreflightNode(args.sim)
        run = PreflightRun(node, args)
    else:
        node = ActuatorTest(args.sim)
        run = Run(node, args)
    code = 0
    try:
        code = run.prepare()
        if code is None:
            run.drive()
            print('Done.')
    except KeyboardInterrupt:
        run.abort = 'Ctrl-C'
        code = 3
    except Aborted as e:
        run.abort = str(e)
        code = 3
    except Exception as e:   # anything at all: stop the car first, then say what it was
        run.abort = f'error: {e!r}'
        code = 4
        raise
    finally:
        # A second Ctrl-C must not cut the stop short.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if run.abort:
            print(f'STOPPING: {run.abort}', flush=True)
        try:
            run.stop_car()
            run.restore()
            run.report()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    # The preflight: 5 when a check failed, 6 when the car would not stop.
    return max(code or 0, getattr(run, 'result', 0))


if __name__ == '__main__':
    sys.exit(main())
