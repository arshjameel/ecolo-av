"""
look_ahead: the squiggly line's numbers, worked out once for every screen.

Subscribes  ego/trace                 nav_msgs/Path, base_link: what the car WILL do (2 s)
            ego/stop_point            geometry_msgs/PoseStamped, base_link: where a full
                                      brake now stops it
            drivers/nav/local_plan    nav_msgs/Path, any frame: what the path follower
                                      WANTS over its own horizon (Nav2's MPPI, mppi_lag)
            plan                      nav_msgs/Path, map: Nav2's whole plan, to its goal
            local_costmap/costmap     nav_msgs/OccupancyGrid, odom: what's in the way
            world/road_ahead          aav_interfaces/RoadAhead, map: the road ahead
                                      (behaviour's), for road_mps, road_m, road_why
            /tf, /tf_static           each of those frames into base_link (the newest)
Publishes   ego/plan_ahead            nav_msgs/Path, base_link, at rate_hz: the faint line,
                                      the local plan while fresh, else Nav2's plan, from
                                      beside the car to plan_ahead_m on (empty: no plan)
            ego/ahead                 diagnostic_msgs/DiagnosticStatus, at rate_hz: level OK
                                      (green) / WARN (amber) / ERROR (red) for how far the
                                      trace strays from the plan; message one line for
                                      people; values gap_m, plan (local, global, none),
                                      stop_m, goal_m, obstacle_m, room_m ('nan':
                                      unknown); front_m, rear_m, half_width_m (the
                                      body from vehicle.yaml, so screens draw the car
                                      and its bumper's stop without a copy); road_mps
                                      (the speed the road ahead allows here), road_m
                                      (metres to its end), road_why (its nearest stop:
                                      sign, obstacle, fence, goal; '' none) - 'nan' and
                                      '' with no fresh road ahead
Serves      nothing; calls nothing.
Parameters  rate_hz (10), plan_ahead_m (10.0), local_plan_timeout (0.5 s), stale_after
            (1.0 s: older trace, stop point, plan or costmap is ignored), gap_warn (0.3 m),
            gap_bad (0.6 m), look_m (15.0: how far ahead an obstacle is looked for), lethal
            (100), base_frame (base_link).

The maths is look_ahead.py (no ROS). The stop point itself is ego's (stop_ahead.py), so
this node, the screens and the stop-point check agree on it. docs/hud.md, "The squiggly
line".
"""

import math
import signal

from aav_description import vehicle
from aav_description.road_ahead_ros import from_msg
from aav_ego import look_ahead as la
from aav_interfaces.msg import RoadAhead
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

LEVELS = {la.OK: DiagnosticStatus.OK, la.WARN: DiagnosticStatus.WARN,
          la.BAD: DiagnosticStatus.ERROR}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Frame2D:
    """A 2D transform: a point in the source frame into the target frame."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.c, self.s = math.cos(yaw), math.sin(yaw)

    def point(self, px, py):
        return (self.x + self.c * px - self.s * py, self.y + self.s * px + self.c * py)

    def pose(self, px, py, pyaw):
        x, y = self.point(px, py)
        return (x, y, pyaw + self.yaw)


class LookAhead(Node):

    def __init__(self, **kwargs):
        super().__init__('look_ahead', **kwargs)
        p = self.declare_parameter
        rate = p('rate_hz', 10.0).value
        self.ahead_m = p('plan_ahead_m', 10.0).value
        self.local_timeout = p('local_plan_timeout', 0.5).value
        self.stale = p('stale_after', 1.0).value
        self.limits = la.Limits(p('gap_warn', 0.3).value, p('gap_bad', 0.6).value)
        self.look_m = p('look_m', 15.0).value
        self.lethal = p('lethal', 100).value
        self.base = p('base_frame', 'base_link').value
        (front, left), _, (rear, _), _ = vehicle.footprint_points(vehicle.load()['vehicle'])
        self.front, self.rear, self.half_width = front, rear, left
        self.latest = {}   # name -> (message, arrival Time)
        for name, kind, topic in (('trace', Path, 'ego/trace'),
                                  ('stop', PoseStamped, 'ego/stop_point'),
                                  ('local', Path, 'drivers/nav/local_plan'),
                                  ('plan', Path, 'plan'),
                                  ('costmap', OccupancyGrid, 'local_costmap/costmap')):
            self.create_subscription(kind, topic, self.keeper(name), 10)
        self.road, self.road_frame = None, 'map'
        self.create_subscription(RoadAhead, 'world/road_ahead', self.on_road, 10)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.plan_pub = self.create_publisher(Path, 'ego/plan_ahead', 10)
        self.ahead_pub = self.create_publisher(DiagnosticStatus, 'ego/ahead', 10)
        self.create_timer(1.0 / rate, self.tick)

    def keeper(self, name):
        def keep(msg):
            self.latest[name] = (msg, self.get_clock().now())
        return keep

    def on_road(self, msg):
        self.road, self.road_frame = from_msg(msg), msg.header.frame_id or 'map'
        self.latest['road'] = (msg, self.get_clock().now())

    def car_in(self, frame):
        """Return base_link's (x, y) in frame, the newest TF; None when unknown."""
        try:
            t = self.tf.lookup_transform(frame, self.base, Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        return t.translation.x, t.translation.y

    def road_here(self):
        """Return the road ahead at the car (look_ahead.road); None when stale or unplaced."""
        if self.fresh('road') is None:
            return None
        car = self.car_in(self.road_frame)
        return None if car is None else la.road(self.road, *car)

    def fresh(self, name, within=None):
        got = self.latest.get(name)
        if got is None:
            return None
        age = (self.get_clock().now() - got[1]).nanoseconds * 1e-9
        return got[0] if age <= (self.stale if within is None else within) else None

    def into_base(self, frame):
        """Return frame -> base_link, the newest TF; None when unknown."""
        if not frame or frame == self.base:
            return Frame2D()
        try:
            t = self.tf.lookup_transform(self.base, frame, Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        return Frame2D(t.translation.x, t.translation.y, yaw_of(t.rotation))

    def path_in_base(self, msg):
        f = self.into_base(msg.header.frame_id)
        if f is None:
            return None
        return [f.point(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

    def grid_in_base(self, msg):
        f = self.into_base(msg.header.frame_id)
        if f is None:
            return None
        o = msg.info.origin
        x, y, yaw = f.pose(o.position.x, o.position.y, yaw_of(o.orientation))
        return la.Grid(msg.info.width, msg.info.height, msg.info.resolution, (x, y, yaw),
                       msg.data)

    def tick(self):
        trace = self.fresh('trace')
        if trace is None:
            return
        trace_xy = [(ps.pose.position.x, ps.pose.position.y) for ps in trace.poses]
        trace_poses = [(ps.pose.position.x, ps.pose.position.y, yaw_of(ps.pose.orientation))
                       for ps in trace.poses]
        whole = self.fresh('plan', within=max(self.stale, 3.0))   # Nav2 replans each second
        global_xy = self.path_in_base(whole) if whole is not None else None
        local = self.fresh('local', within=self.local_timeout)
        local_xy = self.path_in_base(local) if local is not None and local.poses else None
        source, faint = 'none', []
        if local_xy:
            source, faint = 'local', la.crop_ahead(local_xy, self.ahead_m)
        elif global_xy and len(global_xy) >= 2:
            reach = max(self.ahead_m, la.length(trace_xy) + 2.0)
            source, faint = 'global', la.crop_ahead(global_xy, reach)
        gap = la.gap(trace_xy, faint)
        level = la.grade(gap, self.limits)

        stop = self.fresh('stop')
        stop_xy = None if stop is None else (stop.pose.position.x, stop.pose.position.y)
        stop_m = None if stop_xy is None else math.hypot(*stop_xy)
        goal_m = None
        if stop_xy is not None and global_xy and len(global_xy) >= 2:
            goal_m = la.to_goal(global_xy[self.nearest_index(global_xy):], stop_xy)
        obstacle_m = room_m = None
        grid_msg = self.fresh('costmap', within=max(self.stale, 2.0))
        grid = self.grid_in_base(grid_msg) if grid_msg is not None else None
        if grid is not None:
            obstacle_m = la.first_hit(grid, la.extend(trace_poses, self.look_m), self.front,
                                      self.half_width, self.lethal)
            if obstacle_m is not None and stop_m is not None:
                room_m = obstacle_m - stop_m

        out = Path()
        out.header.stamp = trace.header.stamp
        out.header.frame_id = self.base
        for x, y in faint:
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
        self.plan_pub.publish(out)

        def num(v, digits=2):
            return 'nan' if v is None or not math.isfinite(v) else f'{v:.{digits}f}'

        status = DiagnosticStatus(level=LEVELS[level], name='look_ahead',
                                  hardware_id='aav_ego')
        status.message = la.describe(gap, level, stop_m, goal_m, room_m)
        values = [('gap_m', num(gap)), ('plan', source), ('stop_m', num(stop_m)),
                  ('goal_m', num(goal_m)),
                  ('obstacle_m', num(obstacle_m)), ('room_m', num(room_m)),
                  ('front_m', num(self.front)), ('rear_m', num(self.rear)),
                  ('half_width_m', num(self.half_width))]
        road = self.road_here()
        values += [('road_mps', num(None if road is None else road.limit)),
                   ('road_m', num(None if road is None else road.room)),
                   ('road_why', '' if road is None else road.why)]
        status.values = [KeyValue(key=k, value=v) for k, v in values]
        self.ahead_pub.publish(status)

    @staticmethod
    def nearest_index(points):
        return min(range(len(points)), key=lambda i: math.hypot(*points[i]))


def main(args=None):
    rclpy.init(args=args)
    node = LookAhead()
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
