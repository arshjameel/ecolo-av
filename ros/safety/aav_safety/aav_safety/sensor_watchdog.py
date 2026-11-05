"""
sensor_watchdog: stops the automatic drivers when a sensor they need dies, lies or freezes.

Subscribes  each input's topic, as config/sensor_watchdog.yaml says (the launch file picks
            which: inputs), such as
            vehicle/odom              nav_msgs/Odometry     (also: the wheel speed, for
                                                             the camera's frozen check)
            sensors/imu/data          sensor_msgs/Imu
            sensors/gps/fix           sensor_msgs/NavSatFix
            localization/odom/local   nav_msgs/Odometry
            localization/odom/global  nav_msgs/Odometry
            sensors/camera/front_left/image_raw  sensor_msgs/Image (reliable, newest only)
            sensors/radar/points      sensor_msgs/PointCloud2 (best effort)
            perception/detections     vision_msgs/Detection2DArray: a streamed camera
                                      (camera_stream, on the car), one message a
                                      picture; only its own frame_id's count
            hud/heartbeat             std_msgs/UInt64: the HUD's screen is alive (the
                                      frames it has drawn, 10 Hz), where the launch
                                      expects a HUD
            sensors/imu/data          also for the odom input's cross check, if no imu
                                      input watches it
Publishes   safety/sensor_watchdog/lock   std_msgs/Bool, at rate_hz: true while an input
                                          marked stop isn't OK, false otherwise: a
                                          heartbeat. twist_mux's sensors lock (priority
                                          30): it holds Nav2 and the AI slot, never a
                                          person or a test, and also after 1 s without
                                          one (aav_command_mux twist_mux.yaml): a watchdog that
                                          dies or hangs stops them too
            safety/sensor_watchdog/state  std_msgs/String, at rate_hz: OK, or STOP/WARN and
                                          what is wrong, for the HUD and people
            diagnostics                   diagnostic_msgs/DiagnosticArray, once a second:
                                          one status per input (rqt_robot_monitor, Foxglove,
                                          Lichtblick): its state, messages a second, the
                                          longest gap in that second and ever
Serves      safety/sensor_watchdog/clear  std_srvs/Trigger: forget a latched GPS jump,
                                          wheel disagreement or lost HUD (LOST), once a
                                          person has checked (pixi run clear watchdog)

What each check means is in watchdog.py (no ROS there). Each change of state is logged
once: a warning when an input goes wrong, info when it is back.

Parameters  inputs (the names to watch), rate_hz (10), and for each name: topic, kind
            (odom, imu, gps, image, cloud, heartbeat, detections), max_age (s), stop
            (bool), frozen_after (s, 0 off), max_jump_speed (m/s, 0 off), jump_margin
            (m), frame_id (detections: the camera's frame), latch (bool: a dropout after
            its start-up latches LOST, the HUD's), latch_grace (s); for an odom input,
            cross_check (bool: the wheels against the IMU, wheel_check.py), cross_window
            (s), cross_speed_tolerance (m/s), cross_yaw_tolerance (rad/s), cross_persist (s),
            cross_gps_window (s), cross_gps_tolerance (m/s): the GPS's fixes join the check
            where a gps input is watched
"""

import signal
import time
import zlib

from aav_safety.watchdog import NEVER, OK, Rule, Watchdog
from aav_safety.wheel_check import CrossCheck
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, NavSatFix, NavSatStatus, PointCloud2
from std_msgs.msg import Bool, String, UInt64
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

IMU_TOPIC = '/sensors/imu/data'     # what the odom input's cross check reads
TYPES = {'odom': Odometry, 'imu': Imu, 'gps': NavSatFix, 'image': Image, 'cloud': PointCloud2,
         'heartbeat': UInt64, 'detections': Detection2DArray}


def odom_values(msg):
    t = msg.twist.twist
    p = msg.pose.pose.position
    return (t.linear.x, t.linear.y, t.angular.z, p.x, p.y)


def box_values(msg):
    """Every number in the boxes, for the NaN check: a broken detector's output."""
    out = []
    for d in msg.detections:
        b = d.bbox
        out += [b.center.position.x, b.center.position.y, b.size_x, b.size_y]
        out += [r.hypothesis.score for r in d.results]
    return out


def imu_values(msg):
    w, a = msg.angular_velocity, msg.linear_acceleration
    return (w.x, w.y, w.z, a.x, a.y, a.z)


class SensorWatchdog(Node):

    def __init__(self):
        super().__init__('sensor_watchdog')
        names = list(self.declare_parameter('inputs', ['odom']).value)
        rate = self.declare_parameter('rate_hz', 10.0).value
        rules = []
        for name in names:
            def param(key, default, name=name):
                return self.declare_parameter(f'{name}.{key}', default).value
            cross = None
            if bool(param('cross_check', False)):
                d = CrossCheck()
                cross = CrossCheck(
                    window=float(param('cross_window', d.window)),
                    speed_tolerance=float(param('cross_speed_tolerance', d.speed_tolerance)),
                    yaw_tolerance=float(param('cross_yaw_tolerance', d.yaw_tolerance)),
                    persist=float(param('cross_persist', d.persist)),
                    gps_window=float(param('cross_gps_window', d.gps_window)),
                    gps_tolerance=float(param('cross_gps_tolerance', d.gps_tolerance)))
            rules.append(Rule(name, param('topic', ''), param('kind', 'odom'),
                              float(param('max_age', 1.0)), bool(param('stop', True)),
                              float(param('frozen_after', 0.0)),
                              float(param('max_jump_speed', 0.0)),
                              float(param('jump_margin', 5.0)), cross,
                              str(param('frame_id', '')), bool(param('latch', False)),
                              float(param('latch_grace', 5.0))))
        # Five ticks' time without one, on a steady clock: the watchdog itself was stopped,
        # so nothing latches for 3 s after.
        self.dog = Watchdog(rules, stall=max(0.5, 5.0 / rate))
        self.speed = None      # the wheels' newest speed, for the camera's frozen check
        for rule in rules:
            if rule.kind == 'image':
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            elif rule.kind == 'cloud':
                qos = qos_profile_sensor_data
            else:
                qos = 10
            self.create_subscription(TYPES[rule.kind], rule.topic,
                                     lambda msg, r=rule: self.on_message(r, msg), qos)
        if not any(r.topic == '/vehicle/odom' for r in rules):
            self.create_subscription(Odometry, '/vehicle/odom', self.on_wheels, 10)
        if self.dog.cross_checked() and not any(r.topic == IMU_TOPIC for r in rules):
            self.create_subscription(Imu, IMU_TOPIC, self.on_imu, 10)
        self.lock_pub = self.create_publisher(Bool, 'safety/sensor_watchdog/lock', 10)
        self.state_pub = self.create_publisher(String, 'safety/sensor_watchdog/state', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_service(Trigger, 'safety/sensor_watchdog/clear', self.on_clear)
        self.said = {}         # the state each input was last logged in
        self.ticks = 0
        self.diag_every = max(1, int(round(rate)))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('watching ' + ', '.join(
            f'{r.name} ({r.topic}, {r.max_age:g} s, {"stop" if r.stop else "warn"})'
            for r in rules))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_wheels(self, msg):
        self.speed = msg.twist.twist.linear.x

    def on_imu(self, msg):
        self.dog.imu(self.now(), msg.linear_acceleration.x, msg.angular_velocity.z)

    def on_message(self, rule, msg):
        watch = self.dog.watches[rule.name]
        now = self.now()
        if rule.kind == 'odom':
            values = odom_values(msg)
            if rule.topic == '/vehicle/odom':
                self.speed = values[0]
            watch.arrived(now, values, digest=values)
        elif rule.kind == 'imu':
            values = imu_values(msg)
            watch.arrived(now, values, digest=values)
            if rule.topic == IMU_TOPIC:
                self.dog.imu(now, values[3], values[2])
        elif rule.kind == 'gps':
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            has_fix = msg.status.status >= NavSatStatus.STATUS_FIX
            watch.arrived(now, fix=(msg.latitude, msg.longitude), has_fix=has_fix,
                          stamp=stamp)
            if has_fix:
                self.dog.gps(stamp, (msg.latitude, msg.longitude))
        elif rule.kind == 'image':
            # A frozen camera repeats its whole picture: a sample of it is enough.
            watch.arrived(now, digest=zlib.crc32(bytes(memoryview(msg.data)[::997])),
                          speed=self.speed)
        elif rule.kind == 'detections':
            # Every stream shares the topic: only this camera's messages count.
            if rule.counts(msg.header.frame_id):
                watch.arrived(now, box_values(msg))
        elif rule.kind == 'heartbeat':
            # The count must keep rising: the same count again is a screen that stopped.
            watch.arrived(now, digest=msg.data)
        else:
            watch.arrived(now)

    def tick(self):
        now = self.now()
        self.dog.update(now, time.monotonic())
        states = self.dog.states(now)
        locked = any(stops for *_, stops in states)
        self.lock_pub.publish(Bool(data=locked))
        summary = self.dog.summary(now)
        self.state_pub.publish(String(data=summary))
        for name, state, detail, stops in states:
            before = self.said.get(name)
            if before == state:
                continue
            if state == OK:
                if before not in (None, NEVER):
                    self.get_logger().info(f'{name}: OK again')
            elif before is not None or state != NEVER:      # not the quiet start-up
                # One call per severity: rclpy refuses a call site whose severity changes.
                if stops:
                    self.get_logger().error(f'{name} {state}: {detail}. Nav2 and the AI '
                                            'slot stop (twist_mux lock)')
                else:
                    self.get_logger().warning(f'{name} {state}: {detail}')
            self.said[name] = state
        self.ticks += 1
        if self.ticks % self.diag_every == 0:
            self.publish_diagnostics(states)

    def publish_diagnostics(self, states):
        out = DiagnosticArray()
        out.header.stamp = self.get_clock().now().to_msg()
        for name, state, detail, stops in states:
            watch = self.dog.watches[name]
            rule = watch.rule
            level = (DiagnosticStatus.OK if state == OK else
                     DiagnosticStatus.ERROR if stops else DiagnosticStatus.WARN)
            rate, gap, worst_gap = watch.traffic(self.now())
            values = [KeyValue(key='rate_hz', value=f'{rate:.1f}'),
                      KeyValue(key='longest_gap_s', value=f'{gap:.2f}'),
                      KeyValue(key='worst_gap_s', value=f'{worst_gap:.2f}'),
                      KeyValue(key='max_age', value=f'{rule.max_age:g}'),
                      KeyValue(key='stops', value=str(rule.stop).lower())]
            if watch.cross is not None:
                # For tuning the tolerances on the car: how close normal driving comes.
                gaps = watch.cross.gaps
                for i, key in enumerate(('speed_gap', 'turn_gap', 'gps_gap')):
                    values += [KeyValue(key=key, value='' if gaps is None
                                        else f'{gaps[i]:.3f}'),
                               KeyValue(key=f'worst_{key}',
                                        value=f'{watch.cross.worst[i]:.3f}')]
            out.status.append(DiagnosticStatus(
                level=level, name=f'sensor_watchdog: {name}',
                # The rate and the gap in the line itself: a summary panel shows them.
                message=f'{state}: {detail}, {rate:.1f} Hz, longest gap {gap:.2f} s',
                hardware_id=rule.topic, values=values))
        self.diag_pub.publish(out)

    def on_clear(self, request, response):
        self.dog.clear()
        response.success = True
        response.message = ('latched GPS jumps, wheel disagreements and lost inputs (the '
                            'HUD) forgotten')
        self.get_logger().warning('cleared by a person: ' + response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SensorWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C reaches a launched node twice (the terminal, then launch): ignore the
        # second, so it can't interrupt the clean-up with a traceback.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
