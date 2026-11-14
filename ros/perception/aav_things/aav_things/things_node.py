"""
things: the lidar's cubes as things with an id, a speed and a memory (a first form).

Subscribes  perception/voxels/cubes     sensor_msgs/PointCloud2 (aav_voxels), base_link, the
                                        centre of every 0.25 m cube over the road; reliable,
                                        the newest only
            perception/radar/clusters   sensor_msgs/PointCloud2 (aav_radar_clusters), the
                                        radar's frame: x y z velocity cluster per point; each
                                        cluster's mean radial speed, the car's own taken off,
                                        gives the nearest thing its speed along the ray
                                        (radar:=true only, best effort)
            localization/odom/local     nav_msgs/Odometry: the car's speed (for the radar)
Publishes   world/things                aav_interfaces/Things, odom, 10 Hz: every confirmed
                                        thing, moved on its speed to the stamp
            world/things/markers        visualization_msgs/MarkerArray, odom: a box, a speed
                                        arrow and the id per thing, for RViz and the screens
Uses TF     odom <- base_link at each cloud's stamp; odom <- the radar's frame

The maths is in groups.py (Euclidean grouping) and tracker.py (a constant-velocity Kalman
filter per thing, greedy nearest association, the memory). No stock ROS 2 Jazzy tracker
on RoboStack fits (README.md). Names: every thing is UNKNOWN until camera-to-lidar
calibration (Q22); Tracker.name is the hook.

navigation.launch.py things:=true starts it (with voxels, which makes the cubes), and
sim.launch.py passes it on. docs/designs/2026-11-14_the-whole-car.md, "Seeing: things".
"""

import math
import signal

from aav_interfaces.msg import Thing, Things
from aav_things.groups import group, GroupShape, to_frame
from aav_things.tracker import ground_radial_speed, RadarCluster, Tracker, TrackerShape
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

ODOM = 'odom'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_of(t):
    sec = math.floor(t)
    return TimeMsg(sec=int(sec), nanosec=int((t - sec) * 1e9))


class Things(Node):

    def __init__(self):
        super().__init__('things')
        g, t = GroupShape(), TrackerShape()
        self.group_shape = GroupShape(**{
            n: type(getattr(g, n))(self.declare_parameter(n, getattr(g, n)).value)
            for n in ('size', 'gap', 'min_cubes', 'max_size')})
        self.tracker = Tracker(TrackerShape(**{
            n: type(getattr(t, n))(self.declare_parameter(n, getattr(t, n)).value)
            for n in vars(t)}))
        self.use_radar = bool(self.declare_parameter('radar', False).value)
        rate = float(self.declare_parameter('rate', 10.0).value)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.car = (0.0, 0.0)
        self.car_velocity = (0.0, 0.0)    # m/s, odom
        self.things_pub = self.create_publisher(Things, 'world/things', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'world/things/markers', 10)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, 'perception/voxels/cubes', self.on_cubes, newest)
        self.create_subscription(Odometry, 'localization/odom/local', self.on_odom, 10)
        if self.use_radar:
            self.create_subscription(PointCloud2, 'perception/radar/clusters', self.on_radar,
                                     qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(
            f'things: cubes within {self.group_shape.gap:.2f} m make a thing; kept '
            f'{self.tracker.shape.keep:.0f} s unseen ({self.tracker.shape.keep_blind:.0f} s '
            f'within {self.tracker.shape.blind_range:.1f} m); radar '
            f'{"on" if self.use_radar else "off"}')

    def pose(self, frame, stamp):
        """Return (x, y, yaw) of `frame` in odom at `stamp`, or None."""
        try:
            t = self.tf.lookup_transform(ODOM, frame, Time.from_msg(stamp),
                                         timeout=Duration(seconds=0.05))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'no {ODOM} <- {frame}: {e}', throttle_duration_sec=5.0)
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_of(t.transform.rotation)

    def on_odom(self, msg):
        # The local EKF's twist is in base_link: turn it into odom.
        yaw = yaw_of(msg.pose.pose.orientation)
        v = msg.twist.twist.linear
        self.car_velocity = (v.x * math.cos(yaw) - v.y * math.sin(yaw),
                             v.x * math.sin(yaw) + v.y * math.cos(yaw))

    def on_cubes(self, msg):
        pose = self.pose(msg.header.frame_id, msg.header.stamp)
        if pose is None:
            return
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        found = to_frame(group(numpy.asarray(xyz, dtype=float).reshape(-1, 3),
                               self.group_shape), *pose)
        self.car = pose[:2]
        self.tracker.step(seconds(msg.header.stamp), found, self.car)

    def on_radar(self, msg):
        pose = self.pose(msg.header.frame_id, msg.header.stamp)
        if pose is None or msg.width * msg.height == 0:
            return
        # Mixed types (cluster is uint32): read the structured array, then one column each.
        names = ('x', 'y', 'velocity', 'cluster')
        rows = point_cloud2.read_points(msg, field_names=names, skip_nans=True)
        points = numpy.column_stack([numpy.asarray(rows[n], dtype=float) for n in names])
        x0, y0, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        clusters = []
        for number in numpy.unique(points[:, 3]):
            p = points[points[:, 3] == number]
            rx, ry = p[:, 0].mean(), p[:, 1].mean()
            distance = math.hypot(rx, ry)
            if distance < 0.1:
                continue
            ray = ((c * rx - s * ry) / distance, (s * rx + c * ry) / distance)
            clusters.append(RadarCluster(
                x=x0 + c * rx - s * ry, y=y0 + s * rx + c * ry, ray_x=ray[0], ray_y=ray[1],
                speed=ground_radial_speed(float(p[:, 2].mean()), ray, self.car_velocity)))
        self.tracker.radar(seconds(msg.header.stamp), clusters)

    def publish(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self.tracker.time is None:
            return
        self.tracker.expire(t, self.car)
        shape = self.tracker.shape
        out = Things()
        out.header.stamp = now.to_msg()
        out.header.frame_id = ODOM
        for track in self.tracker.at(t):
            out.things.append(Thing(
                id=track.id, kind=track.kind,
                x=float(track.x[0]), y=float(track.x[1]),
                vx=float(track.x[2]), vy=float(track.x[3]),
                length=float(track.length), width=float(track.width),
                height=float(track.height), position_sigma=float(track.position_sigma),
                confidence=float(track.confidence(t, shape)),
                last_seen=stamp_of(track.last_seen), seen=track.seen,
                radar=track.radar, coasting=track.coasting))
        self.things_pub.publish(out)
        self.markers_pub.publish(markers(out))


def markers(things):
    """A box, a speed arrow and the id for each thing; DELETEALL first."""
    array = MarkerArray(markers=[Marker(header=things.header, action=Marker.DELETEALL)])
    for th in things.things:
        grey = th.coasting
        m = Marker(header=things.header, ns='things', id=th.id * 3, type=Marker.CUBE)
        m.pose.position.x, m.pose.position.y = float(th.x), float(th.y)
        m.pose.position.z = th.height / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y = max(th.length, 0.1), max(th.width, 0.1)
        m.scale.z = max(th.height, 0.1)
        m.color.r, m.color.g, m.color.b = (0.6, 0.6, 0.6) if grey else (1.0, 0.6, 0.0)
        m.color.a = 0.5
        arrow = Marker(header=things.header, ns='things', id=th.id * 3 + 1, type=Marker.ARROW)
        arrow.points = [Point(x=float(th.x), y=float(th.y), z=float(th.height)),
                        Point(x=float(th.x + th.vx), y=float(th.y + th.vy), z=float(th.height))]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.08, 0.2, 0.2
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.2, 0.6, 1.0, 1.0
        text = Marker(header=things.header, ns='things', id=th.id * 3 + 2,
                      type=Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y = float(th.x), float(th.y)
        text.pose.position.z = th.height + 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.4
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'{th.id} {math.hypot(th.vx, th.vy):.1f} m/s' + (' R' if th.radar else '')
        array.markers += [m, arrow, text]
    return array


def main(args=None):
    rclpy.init(args=args)
    node = Things()
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
