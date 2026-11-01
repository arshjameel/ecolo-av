"""
geofence: stops every program before the car's stopping distance reaches the fence's edge.

Subscribes  localization/odom/global         nav_msgs/Odometry: the global EKF, map frame
                                             (position, heading, forward speed, covariance)
            localization/gps/odom            nav_msgs/Odometry: each GPS fix into the EKF;
                                             only its arrival counts (silent 2 s: LOST)
Publishes   safety/geofence/lock             std_msgs/Bool, at rate_hz: twist_mux's geofence
                                             lock (priority 51): every program (scripted,
                                             replay, point, Nav2, AI), never a person or the
                                             actuator tests. True while STARTING, NEAR,
                                             OUTSIDE, JUMP, LOST or COASTING (geofence.py).
                                             Its timeout is 1 s, so a dead or hung node
                                             holds them too
            safety/geofence/state            std_msgs/String, at rate_hz: the state and why
                                             (the actuator tests stop on it)
            safety/geofence/fence            geometry_msgs/PolygonStamped, map frame,
                                             transient local: the fence (the point driver
                                             refuses a B outside it)
Serves      safety/geofence/clear            std_srvs/Trigger: a person drove the car back
                                             in after OUTSIDE, or understood a JUMP; refused
                                             while it is still out. Only people call it

Parameters  set by geofence.launch.py: the fence (site, or centre_x, centre_y, radius),
            datum, stands, gps, and the stopping numbers (reaction_time, braking,
            max_accel, max_speed, max_reverse_speed, max_curvature, coasting).
            config/geofence.yaml: margin, hysteresis, max_std, max_age, gps_max_age,
            settle_time, jump_margin, rate_hz, sites.<name>.datum and .corners. The car's
            corners come from aav_description's vehicle.yaml.

The fence is fixed (the marked start spot, or a site), so a restarted node finds an
outside car outside again: it holds nothing worth losing. Times are on the steady clock
(arrival), so a jump in sim time can't age a pose or settle it.
Design: docs/designs/2026-10-29_geofence.md; docs/geofence.md.
"""

import math
import signal
import time

from aav_description import vehicle
from aav_safety.geofence import (Circle, Geofence, Motion, OK, Pose, Settings, site_fence,
                                 STANDS)
from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

SETTINGS = ('margin', 'hysteresis', 'max_std', 'max_age', 'gps_max_age', 'settle_time',
            'jump_margin')
MOTION = ('reaction_time', 'braking', 'max_accel', 'max_speed', 'max_reverse_speed',
          'max_curvature')
LOUD = 5.0      # s between the stands mode's reminders


class GeofenceNode(Node):

    def __init__(self):
        super().__init__('geofence')
        defaults = Settings()
        settings = Settings(**{n: float(self.declare_parameter(n, getattr(defaults, n)).value)
                               for n in SETTINGS})
        # No defaults for the stopping numbers: the launch file works them out from the
        # configs, and a node started without them must not guess.
        motion = Motion(**{n: float(self.declare_parameter(n, math.nan).value) for n in MOTION})
        site = self.declare_parameter('site', '').value
        rate = float(self.declare_parameter('rate_hz', 10.0).value)
        stands = bool(self.declare_parameter('stands', False).value)
        if site:
            datum = list(self.declare_parameter('datum', [0.0]).value)
            corners = list(self.declare_parameter(f'sites.{site}.corners', [0.0]).value)
            site_datum = list(self.declare_parameter(f'sites.{site}.datum', [0.0]).value)
            if corners == [0.0]:
                raise ValueError(f'site {site}: no sites.{site}.corners in config/geofence.yaml')
            fence = site_fence(site, corners, site_datum, datum)
            where = f'site {site} (datum {site_datum})'
        else:
            cx = float(self.declare_parameter('centre_x', 0.0).value)
            cy = float(self.declare_parameter('centre_y', 0.0).value)
            radius = float(self.declare_parameter('radius', 30.0).value)
            fence = Circle(cx, cy, radius)
            where = f'a {radius:.0f} m circle round the start spot ({cx:.1f}, {cy:.1f})'
        footprint = vehicle.footprint_points(vehicle.load()['vehicle'])
        self.fence = Geofence(footprint, fence, motion, settings,
                              gps=bool(self.declare_parameter('gps', True).value),
                              coasting=bool(self.declare_parameter('coasting', False).value),
                              stands=stands)
        forward, _ = self.fence.stopping(motion.max_speed)
        self.get_logger().info(
            f'the fence: {where}; stopping {forward:.1f} m from {motion.max_speed:.1f} m/s '
            f'(reaction {motion.reaction_time:.2f} s, braking {motion.braking:.2f} m/s^2) '
            f'plus a {settings.margin:.1f} m margin and the pose spread')
        if stands:
            self.get_logger().error('STANDS MODE: no geofence. The wheels must be off the '
                                    'ground')

        self.lock = self.create_publisher(Bool, '/safety/geofence/lock', 10)
        self.state = self.create_publisher(String, '/safety/geofence/state', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        outline = PolygonStamped()
        outline.header.frame_id = 'map'
        outline.header.stamp = self.get_clock().now().to_msg()
        outline.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in fence.outline()]
        self.create_publisher(PolygonStamped, '/safety/geofence/fence', latched).publish(outline)
        self.create_subscription(Odometry, '/localization/odom/global', self.on_pose, 10)
        self.create_subscription(Odometry, '/localization/gps/odom',
                                 lambda _: self.fence.gps_arrived(time.monotonic()), 10)
        self.create_service(Trigger, '/safety/geofence/clear', self.on_clear)
        self.was = None
        self.reminded = 0.0
        self.create_timer(1.0 / rate, self.tick)

    def on_pose(self, msg: Odometry):
        q = msg.pose.pose.orientation
        cov = msg.pose.covariance
        spread = max(cov[0], cov[7])
        std = math.sqrt(spread) if spread >= 0.0 else math.nan
        self.fence.update(time.monotonic(), Pose(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
            msg.twist.twist.linear.x, std))

    def tick(self):
        now = time.monotonic()
        locked, state, why = self.fence.check(now)
        self.lock.publish(Bool(data=locked))
        self.state.publish(String(data=f'{state}: {why}'))
        log = self.get_logger()
        # One severity per call site: rclpy refuses a logger call that changes it.
        if state != self.was and state == OK:
            log.info(f'geofence {state}: {why}')
            self.was = state
        elif state != self.was:
            log.error(f'geofence {state}: {why}' + ('' if not locked else
                                                    ': every program is held; people may drive'))
            self.was = state
        elif state == STANDS and now - self.reminded > LOUD:
            log.error(f'geofence {state}: {why}')
            self.reminded = now

    def on_clear(self, request, response):
        response.success, response.message = self.fence.clear(time.monotonic())
        if response.success:
            self.get_logger().warning(f'clear: {response.message}')
        else:
            self.get_logger().error(f'clear refused: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
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
