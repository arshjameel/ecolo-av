"""
ghost_obstacle: a pretend person in navigation's view, to test stopping with nothing real to hit.

Publishes   safety/ghost/scan    sensor_msgs/LaserScan, in base_link, at rate_hz: what a
                                 perfect 2D lidar at base_link would see of the ghost (its
                                 near side), inf in every other direction; all inf with no
                                 ghost. Read by the local costmap's ghost_layer and
                                 the stop zone's ghost source (navigation.launch.py
                                 ghost:=true): their own layer and source, so the ghost only
                                 ever adds an obstacle, and never clears or hides a real one
            safety/ghost/marker  visualization_msgs/Marker: the ghost as a cylinder, for
                                 RViz and Lichtblick; DELETE when it goes
Calls       TF: frame -> base_link (where the car is)

Parameters (config/ghost_obstacle.yaml; change them live, or with `pixi run ghost`):
  enabled  false: no ghost (the default). true: place it, as below
  ahead    m. 0 or more: the ghost's near edge this far in front of the car's front bumper,
           where the car is when the ghost is placed; then it stays there. Negative: at x, y
  x, y     m in frame, when ahead is negative
  frame    the fixed frame the ghost stands still in: map
  width, height   m: a person 0.5 x 1.7 (the default); a post 0.3 x 1.2
  rate_hz  how often the scan goes out. Keep it well inside the stop zone's
           source_timeout (1 s): a silent ghost stops Nav2, like a silent radar

Setting enabled, ahead, x, y or frame places the ghost again. The maths is in ghost.py.
"""

import math
import signal

from aav_description import vehicle
from aav_ghost.ghost import ahead_of, Ghost, Pose, ranges, ScanShape
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros
from visualization_msgs.msg import Marker

# Changing one of these puts the ghost in its place again.
PLACING = ('enabled', 'ahead', 'x', 'y', 'frame')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GhostObstacle(Node):

    def __init__(self):
        super().__init__('ghost_obstacle')
        for name, default in (('enabled', False), ('ahead', 10.0), ('x', 0.0), ('y', 0.0),
                              ('frame', 'map'), ('width', 0.5), ('height', 1.7),
                              ('rate_hz', 10.0)):
            self.declare_parameter(name, default)
        body = vehicle.load()['vehicle']['body']
        self.front = float(body['length']) - float(body['rear_overhang'])  # base_link to bumper
        self.shape = ScanShape()
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.scan_pub = self.create_publisher(LaserScan, 'safety/ghost/scan', 10)
        self.marker_pub = self.create_publisher(Marker, 'safety/ghost/marker', 10)
        self.ghost = None          # Ghost, in frame, once placed
        self.placing = True        # place it (again) at the next tick
        self.add_post_set_parameters_callback(self.on_parameters)
        self.create_timer(1.0 / float(self.param('rate_hz')), self.tick)
        self.get_logger().info(
            'ghost_obstacle: no ghost until enabled (pixi run ghost ahead 10)'
            if not self.param('enabled') else 'ghost_obstacle: a ghost, from the parameters')

    def param(self, name):
        return self.get_parameter(name).value

    def on_parameters(self, parameters):
        if any(p.name in PLACING for p in parameters):
            self.placing = True

    def car(self, frame):
        """Return (Pose of base_link in frame, its TF stamp), or (None, a usable stamp)."""
        try:
            t = self.tf.lookup_transform(frame, 'base_link', Time())
        except tf2_ros.TransformException:
            pass
        else:
            p = t.transform.translation
            return Pose(p.x, p.y, yaw_of(t.transform.rotation)), t.header.stamp
        try:
            # No fixed frame yet: an empty scan still needs a time the costmap can use.
            return None, self.tf.lookup_transform('odom', 'base_link', Time()).header.stamp
        except tf2_ros.TransformException:
            return None, self.get_clock().now().to_msg()

    def place(self, pose):
        """Put the ghost where the parameters say; False if it needs the car's pose first."""
        width, height = float(self.param('width')), float(self.param('height'))
        ahead = float(self.param('ahead'))
        if ahead >= 0.0:
            if pose is None:
                return False
            x, y = ahead_of(pose, self.front, ahead, width)
        else:
            x, y = float(self.param('x')), float(self.param('y'))
        self.ghost = Ghost(x, y, width, height)
        self.get_logger().info(
            f'ghost: {width:.2f} m wide, {height:.2f} m tall, at {self.param("frame")} '
            f'x {x:.2f}, y {y:.2f}' + (f' ({ahead:.1f} m ahead of the bumper)'
                                       if ahead >= 0.0 else ''))
        return True

    def tick(self):
        frame = self.param('frame')
        pose, stamp = self.car(frame)
        if not self.param('enabled'):
            if self.ghost is not None:
                self.get_logger().info('ghost: gone')
            self.ghost, self.placing = None, False
        elif self.placing:
            self.placing = not self.place(pose)
        elif self.ghost is not None:
            # A new size keeps the ghost where it stands.
            self.ghost = Ghost(self.ghost.x, self.ghost.y, float(self.param('width')),
                               float(self.param('height')))
        self.publish_scan(pose, stamp)
        self.publish_marker(frame)

    def publish_scan(self, pose, stamp):
        s = self.shape
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_link'
        scan.angle_min = s.angle_min
        scan.angle_increment = s.angle_increment
        scan.angle_max = s.angle_min + (s.count - 1) * s.angle_increment
        scan.scan_time = 1.0 / float(self.param('rate_hz'))
        scan.range_min = s.range_min
        scan.range_max = s.range_max
        if self.ghost is not None and pose is not None:
            scan.ranges = ranges(self.ghost, pose, s).astype(float).tolist()
        else:
            scan.ranges = [math.inf] * s.count
        self.scan_pub.publish(scan)

    def publish_marker(self, frame):
        m = Marker()
        m.header.frame_id = frame       # stamp zero: RViz draws it with the newest TF
        m.ns, m.id = 'ghost', 0
        if self.ghost is None:
            m.action = Marker.DELETE
        else:
            g = self.ghost
            m.type, m.action = Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = g.x, g.y, g.height / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = g.width
            m.scale.z = g.height
            m.color.r, m.color.g, m.color.b, m.color.a = 0.7, 0.3, 1.0, 0.6
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GhostObstacle()
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
