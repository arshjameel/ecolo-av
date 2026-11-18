"""
point_driver: drive to one point B (lat/lon or map x/y) and stop there smoothly.

Action      drive_to_point             aav_interfaces/action/DriveToPoint
Subscribes  localization/odom/global   nav_msgs/Odometry: where the car is on the map, its speed
            sensors/gps/fix            sensor_msgs/NavSatFix (best effort): fresh? RTK?
            safety/estop               std_msgs/Bool: true stops the drive
            safety/sensor_watchdog/lock  std_msgs/Bool: true, or silent for watchdog_timeout,
                                       stops the drive (as twist_mux's sensors lock)
            safety/geofence/fence      geometry_msgs/PolygonStamped, transient local: a B
                                       outside it is refused before the car moves
            system/source              aav_interfaces/DataSource, latched: REAL needs a person
                                       to confirm (the goal's `confirmed`)
            control/cmd_vel            geometry_msgs/TwistStamped: twist_mux's choice, to
                                       stop when someone else takes over
Publishes   drivers/point/cmd_vel       geometry_msgs/TwistStamped, at rate_hz while a goal
                                       runs (twist_mux priority 25: below every person,
                                       test and script, above Nav2; the stop-sign and
                                       sensor locks, at 30, hold it)
            drivers/point/plan          nav_msgs/Path (map), latched: the line A -> B while a
                                       drive runs, empty when it ends. The behaviour planner
                                       makes the road ahead from it, so the obstacle check,
                                       the sign and ego's limit hold this driver too (the
                                       whole-car design, R5)
Calls       /fromLL                    robot_localization/srv/FromLL (navsat_transform):
                                       lat/lon to the map frame, as the EKF sees it
            /ego/get_parameters        rcl_interfaces/GetParameters: ego's caps

Try it (or: pixi run drive-to-point --sim --ahead 20):
  ros2 action send_goal /drive_to_point aav_interfaces/action/DriveToPoint \
      "{x: 20.0, y: 0.0}" --feedback

It speeds up gently, holds a cruise speed, and slows down by the distance left at half
the measured braking (vehicle.yaml actuators max_brake), so it stops at B (approach.py).
It steers by pure pursuit: first towards B, then along the straight line to it (line.py).
It stops at once (zero commands; ego's own deceleration) on a cancel, the e-stop, the
watchdog's lock or its silence, a stale position or GPS fix, someone else driving, or the
car standing still though asked to move (rules.py). It also stops, apart from the
geofence and Nav2, when the car leaves the drive's corridor (line.Fence, TODO 187): more
than max_off_line from the line to B, more than max_overrun past B, or further from the
start than the drive could need. It refuses a B over max_distance away, inside the
car's tightest turn, or outside the
geofence (aav_safety, whose lock at 51 holds this driver too).
"""

import collections
import math
import signal
import time

from aav_actuator_tests.safety import source_kind, taken_over
from aav_description import vehicle
from aav_description.car_model import ActuatorConfig
from aav_drive_to_point import rules
from aav_drive_to_point import run as run_mod
from aav_drive_to_point.line import min_radius, Pose
from aav_interfaces.action import DriveToPoint
from aav_interfaces.msg import DataSource
from aav_safety.geofence import Polygon
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PolygonStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool

EGO_PARAMS = ('max_speed', 'max_acceleration', 'max_deceleration', 'max_jerk')
# How it slows into B: approach.py's sqrt(2 a (d - lead v)) (the default), or hybrid, the
# stopping law (aav_description stop_law.py), a trial until it wins.
STOP_LAWS = ('approach', 'hybrid')
TAKEOVER_TIME = 0.3   # s of twist_mux passing someone else's commands: stop


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PointDriver(Node):

    def __init__(self):
        super().__init__('point_driver')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('rate_hz', 50.0),
            ('cruise_speed', 1.0), ('acceleration', 0.5),
            ('max_brake', 3.0), ('brake_fraction', 0.5), ('deceleration', 0.0),
            ('brake_lead', 0.8), ('stop_band', 0.05), ('creep', 0.1),
            ('lookahead_min', 2.0), ('lookahead_time', 1.0), ('lateral_accel', 1.5),
            ('wheelbase', 1.57), ('max_steering_angle', 0.5),
            ('max_distance', 200.0), ('pose_timeout', 0.5), ('gps_timeout', 2.0),
            ('require_gps', True), ('watchdog_timeout', 1.0), ('stuck_time', 5.0),
            ('still_speed', 0.05), ('still_time', 1.0), ('stop_law', 'approach'),
            ('max_off_line', 3.0), ('max_overrun', 2.0))}
        self.p = p
        if p['stop_law'] not in STOP_LAWS:
            raise ValueError(f"stop_law:={p['stop_law']}: pick one of {', '.join(STOP_LAWS)}")
        # hybrid: the stopping law, from the shared car model's brake (vehicle.yaml).
        self.car = (ActuatorConfig.from_vehicle_yaml(vehicle.config_path())
                    if p['stop_law'] == 'hybrid' else None)
        self.limits = rules.Limits(max_distance=p['max_distance'],
                                   pose_timeout=p['pose_timeout'],
                                   gps_timeout=p['gps_timeout'], require_gps=p['require_gps'],
                                   watchdog_timeout=p['watchdog_timeout'])
        self.radius = min_radius(p['wheelbase'], p['max_steering_angle'])
        self.settings = run_mod.Settings(**{k: float(p[k]) for k in (
            'cruise_speed', 'acceleration', 'max_brake', 'brake_fraction', 'deceleration',
            'brake_lead', 'stop_band', 'creep', 'lookahead_min', 'lookahead_time',
            'lateral_accel', 'still_speed', 'stuck_time', 'still_time', 'max_off_line',
            'max_overrun')})
        self.busy = False
        self.pose = None            # (Pose, speed)
        self.pose_time = None       # s, this node's clock
        self.fix_time = None
        self.fix_status = None
        self.estop = False
        self.locked = False
        self.lock_time = None       # s: the watchdog's lock last arrived (its heartbeat)
        self.sources = {}
        self.ours = collections.deque(maxlen=50)   # our last commands (v, w)
        self.other_since = None                    # s: twist_mux passing someone else's

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(TwistStamped, 'drivers/point/cmd_vel', 10)
        self.line_pub = self.create_publisher(Path, 'drivers/point/plan', latched)
        self.publish_line([])
        self.create_subscription(Odometry, 'localization/odom/global', self.on_pose, 10)
        self.create_subscription(NavSatFix, 'sensors/gps/fix', self.on_fix,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, 'safety/estop',
                                 lambda m: setattr(self, 'estop', bool(m.data)), 10)
        self.create_subscription(Bool, 'safety/sensor_watchdog/lock', self.on_lock, 10)
        self.create_subscription(DataSource, 'system/source', self.on_source, latched)
        self.fence = None
        self.create_subscription(PolygonStamped, 'safety/geofence/fence', self.on_fence, latched)
        self.create_subscription(TwistStamped, 'control/cmd_vel', self.on_mux, 10)
        self.from_ll = self.create_client(FromLL, '/fromLL')
        self.ego = AsyncParameterClient(self, 'ego')
        self.server = ActionServer(
            self, DriveToPoint, 'drive_to_point', execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)

    # --- listening ---------------------------------------------------------------------

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        q = msg.pose.pose
        if not all(math.isfinite(v) for v in (q.position.x, q.position.y,
                                              msg.twist.twist.linear.x)):
            return          # a NaN is no position: it goes stale, and the drive stops
        self.pose = (Pose(q.position.x, q.position.y, yaw_of(q.orientation)),
                     msg.twist.twist.linear.x)
        self.pose_time = self.seconds()

    def on_fix(self, msg):
        if (msg.status.status >= NavSatStatus.STATUS_FIX and math.isfinite(msg.latitude)
                and math.isfinite(msg.longitude)):
            self.fix_time = self.seconds()
            self.fix_status = msg.status.status

    def on_lock(self, msg):
        self.locked = bool(msg.data)
        self.lock_time = self.seconds()

    def on_fence(self, msg):
        try:
            self.fence = Polygon([(p.x, p.y) for p in msg.polygon.points])
        except ValueError as e:
            self.get_logger().warning(f'the geofence sent no usable fence: {e}')

    def on_source(self, msg):
        self.sources[msg.announcer] = [s.source for s in msg.sensors]

    def on_mux(self, msg):
        theirs = (msg.twist.linear.x, msg.twist.angular.z)
        if not self.busy or not self.ours or not taken_over(self.ours, theirs):
            self.other_since = None
        elif self.other_since is None:
            self.other_since = self.seconds()

    def now(self):
        t = self.seconds()

        def age(stamp):
            return None if stamp is None else t - stamp
        return rules.Now(pose_age=age(self.pose_time), fix_age=age(self.fix_time),
                         estop=self.estop, locked=self.locked,
                         kind=source_kind(self.sources), lock_age=age(self.lock_time),
                         taken_over=(self.other_since is not None
                                     and t - self.other_since > TAKEOVER_TIME))

    # --- talking -----------------------------------------------------------------------

    def publish(self, v, curvature):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(v * curvature)
        self.pub.publish(msg)
        self.ours.append((msg.twist.linear.x, msg.twist.angular.z))

    def publish_line(self, points):
        """Send the drive's line A -> B (map), or an empty path: no drive."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.line_pub.publish(msg)

    async def within(self, future, seconds):
        """Wait for a future, at most `seconds` of wall-clock time; its result, or None."""
        gate, deadline = Future(), time.monotonic() + seconds

        def check(*_):
            if (future.done() or time.monotonic() > deadline) and not gate.done():
                gate.set_result(None)
        timer = self.create_timer(0.05, check)
        future.add_done_callback(check)
        try:
            await gate
        finally:
            self.destroy_timer(timer)
        return future.result() if future.done() else None

    async def ego_caps(self):
        """Return ego's caps ({} if it doesn't answer)."""
        if not self.ego.services_are_ready():
            return {}
        res = await self.within(self.ego.get_parameters(list(EGO_PARAMS)), 2.0)
        if res is None:
            return {}
        return {n: parameter_value_to_python(v) for n, v in zip(EGO_PARAMS, res.values)
                if isinstance(parameter_value_to_python(v), (int, float))}

    async def target_of(self, goal):
        """Return (B in the map frame, None), or (None, why not)."""
        if not goal.use_lat_lon:
            return (goal.x, goal.y), None
        if not (math.isfinite(goal.latitude) and math.isfinite(goal.longitude)
                and abs(goal.latitude) <= 90 and abs(goal.longitude) <= 180):
            return None, 'latitude and longitude must be on the globe'
        if not self.from_ll.service_is_ready():
            return None, 'no /fromLL (navsat_transform): lat/lon needs the GPS running'
        req = FromLL.Request(ll_point=GeoPoint(latitude=goal.latitude,
                                               longitude=goal.longitude, altitude=0.0))
        res = await self.within(self.from_ll.call_async(req), 3.0)
        if res is None:
            return None, '/fromLL did not answer in 3 s'
        return (res.map_point.x, res.map_point.y), None

    # --- one drive ---------------------------------------------------------------------

    def on_goal(self, goal):
        if self.busy:
            self.get_logger().warning('rejected a goal: another drive to a point is running')
            return GoalResponse.REJECT
        self.busy = True      # at once: a second goal, before this one runs, is refused
        return GoalResponse.ACCEPT

    def plan(self, goal, caps):
        """Return (the Approach, None) for this goal and ego's caps, or (None, why not)."""
        return run_mod.plan(goal.cruise_speed, goal.acceleration, goal.deceleration, caps,
                            self.settings, self.car)

    async def execute(self, handle):
        # A coroutine on one thread, driven by a timer, as aav_scripted's driver: a
        # MultiThreadedExecutor spins a whole core (rclpy 7.1.12).
        try:
            return await self.drive(handle)
        except Exception as e:  # noqa: B902 - stop the car first, then say what it was
            self.publish(0.0, 0.0)
            self.get_logger().error(f'drive to point failed: {e!r}')
            handle.abort()
            return DriveToPoint.Result(success=False, message=f'error: {e!r}')
        finally:
            self.busy = False

    def refuse(self, handle, why, target=(math.nan, math.nan)):
        self.get_logger().warning(f'drive to point refused: {why}')
        handle.abort()
        return DriveToPoint.Result(success=False, message=why, target_x=float(target[0]),
                                   target_y=float(target[1]))

    async def drive(self, handle):
        goal = handle.request
        why = rules.refuse_start(self.now(), self.limits, goal.confirmed)
        if why:
            return self.refuse(handle, why)
        target, why = await self.target_of(goal)
        if why:
            return self.refuse(handle, why)
        caps = await self.ego_caps()
        approach, why = self.plan(goal, caps)
        if why:
            return self.refuse(handle, why, target)
        start, _ = self.pose
        why = rules.refuse_target(start, target, self.limits, self.radius) or \
            rules.refuse_outside(None if self.fence is None else self.fence.depth(*target)) or \
            rules.refuse_start(self.now(), self.limits, goal.confirmed)
        if why:
            return self.refuse(handle, why, target)

        p = self.p
        run = run_mod.Run(target, approach, self.settings, self.limits, self.radius, start,
                          self.seconds())
        self.get_logger().info(
            f'drive to ({target[0]:.2f}, {target[1]:.2f}): {run.distance:.1f} m; cruise '
            f'{approach.cruise:g} m/s, up {approach.accel:g} m/s^2, down {approach.decel:g} '
            f'm/s^2 (max_brake {p["max_brake"]:g} x {p["brake_fraction"]:g})')
        self.ours.clear()
        self.other_since = None

        finished = Future()
        feedback = DriveToPoint.Feedback()

        def tick():
            pose, measured = self.pose
            step = run.tick(self.seconds(), self.now(), pose, measured,
                            handle.is_cancel_requested)
            if step.stopped:
                self.get_logger().warning(f'drive to point stopping: {step.stopped}')
            if step.send is not None:
                self.publish(*step.send)
            if step.feedback is not None:
                feedback.phase = step.feedback.phase
                feedback.distance_left = float(step.feedback.left)
                feedback.cross_track = float(step.feedback.cross)
                feedback.commanded_speed = float(step.feedback.speed)
                feedback.measured_speed = float(measured)
                handle.publish_feedback(feedback)
            if step.finished and not finished.done():
                finished.set_result(None)

        self.publish_line([(start.x, start.y), target])
        timer = self.create_timer(1.0 / p['rate_hz'], tick)
        try:
            await finished
        finally:
            self.destroy_timer(timer)
            self.publish(0.0, 0.0)
            self.publish_line([])

        pose, _ = self.pose
        end = run.outcome(pose, self.fix_status, p['require_gps'], self.seconds())
        result = DriveToPoint.Result(target_x=float(target[0]), target_y=float(target[1]),
                                     distance_left=float(end.left),
                                     tolerance=float(end.tolerance),
                                     distance_travelled=float(end.travelled),
                                     duration=float(end.duration), success=end.success,
                                     message=end.message)
        if end.status == 'cancelled':
            handle.canceled()
        elif end.status == 'succeeded':
            handle.succeed()
        else:
            handle.abort()
        self.get_logger().info(f'drive to point: {result.message}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PointDriver()
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
        if rclpy.ok():
            node.publish(0.0, 0.0)    # a last stop; twist_mux's timeout does the rest
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
