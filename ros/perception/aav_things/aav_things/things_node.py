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
            /perception/detections      vision_msgs/Detection2DArray (names true only): the
                                        camera's boxes; a thing whose box in the picture
                                        overlaps one takes its label (naming.py, TODO 192)
            camera_info_topics          sensor_msgs/CameraInfo (names true only): each lens
            /map                        nav_msgs/OccupancyGrid, transient local (background
                                        true only): the saved map; cubes on its walls (with
                                        background_slack m for pose error) are dropped
                                        before grouping (background.py, TODO 175 step 5)
Publishes   world/things                aav_interfaces/Things, odom, 10 Hz: every confirmed
                                        thing, moved on its speed to the stamp
            world/things/markers        visualization_msgs/MarkerArray, odom: a box coloured by
                                        its kind, a speed arrow, and its name, id and speed
                                        ("person 3 1.2 m/s") per thing (looks.py), for RViz
                                        and Foxglove
Uses TF     odom <- base_link at each cloud's stamp; odom <- the radar's frame; with
            background, map <- base_link (the newest: a cloud with none keeps every cube). A cloud
            newer than /tf waits for it (tf_wait.py), never blocking: a hair newer (0.1 s)
            uses the newest transform; one still waiting after 1.5 s is dropped, and the
            drops are said at most once every 5 s, not per cloud

The maths is in groups.py (Euclidean grouping) and tracker.py (a constant-velocity Kalman
filter per thing, greedy nearest association, the memory). No stock ROS 2 Jazzy tracker
on RoboStack fits (README.md). Names (names:=true, naming.py): the lidar says where, the
camera says what; with names off, or on the car before camera calibration (Q22), every
thing is UNKNOWN. Also uses TF base_link <- each camera's frame, and odom <- base_link.

navigation.launch.py things:=true starts it (with voxels, which makes the cubes), and
sim.launch.py passes it on. docs/designs/2026-11-14_the-whole-car.md, "Seeing: things".
"""

import math
import signal

from aav_detector import ground
from aav_interfaces.msg import Thing, Things
from aav_things.background import Background, new_only
from aav_things import looks
from aav_things.groups import group, GroupShape, to_frame
from aav_things.naming import Camera, Namer, Seen
from aav_things.tf_wait import Step, Waiting
from aav_things.tracker import ground_radial_speed, RadarCluster, Tracker, TrackerShape
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

ODOM = 'odom'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_of(t):
    sec = math.floor(t)
    return TimeMsg(sec=int(sec), nanosec=int((t - sec) * 1e9))


class ThingsNode(Node):

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
        self.use_background = bool(self.declare_parameter('background', False).value)
        self.background_slack = float(self.declare_parameter('background_slack', 0.25).value)
        self.background = None
        self.background_dropped = 0
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.car = (0.0, 0.0)
        self.car_velocity = (0.0, 0.0)    # m/s, odom
        # Each stream's clouds waiting for their transform, oldest first.
        self.waiting = {'cubes': Waiting(), 'radar': Waiting()}
        self.behind = 0.0       # s: the last waiting cloud's lead over the newest transform
        self.things_pub = self.create_publisher(Things, 'world/things', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'world/things/markers', 10)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, 'perception/voxels/cubes', self.on_cubes, newest)
        self.create_subscription(Odometry, 'localization/odom/local', self.on_odom, 10)
        if self.use_radar:
            self.create_subscription(PointCloud2, 'perception/radar/clusters', self.on_radar,
                                     qos_profile_sensor_data)
        # Names from the camera's boxes (naming.py, TODO 192): off until it scores.
        self.use_names = bool(self.declare_parameter('names', False).value)
        self.namer = Namer(min_iou=float(self.declare_parameter('name_min_iou', 0.3).value),
                           hold=float(self.declare_parameter('name_hold', 3.0).value))
        self.name_min_score = float(self.declare_parameter('name_min_score', 0.5).value)
        self.lenses = {}
        if self.use_names:
            topics = self.declare_parameter(
                'camera_info_topics',
                [f'/sensors/camera/{c}/camera_info' for c in ('front_left', 'front_right',
                                                               'rear')]).value
            for topic in topics:
                self.create_subscription(CameraInfo, topic, self.on_camera_info, 10)
            self.create_subscription(Detection2DArray, '/perception/detections',
                                     self.on_detections, 10)
            self.tracker.name = self.name_of
        if self.use_background:
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(
            f'things: cubes within {self.group_shape.gap:.2f} m make a thing; kept '
            f'{self.tracker.shape.keep:.0f} s unseen ({self.tracker.shape.keep_blind:.0f} s '
            f'within {self.tracker.shape.blind_range:.1f} m); radar '
            f'{"on" if self.use_radar else "off"}; the saved map as background '
            f'{"on" if self.use_background else "off"}')

    def on_camera_info(self, msg):
        lens = ground.lens_from_k(msg.k, msg.width, msg.height)
        if lens is not None:
            self.lenses[msg.header.frame_id] = lens

    def on_detections(self, msg):
        """Keep each camera's newest boxes, with its lens and its mount on the car."""
        frame = msg.header.frame_id
        lens = self.lenses.get(frame)
        if lens is None:
            return
        try:
            t = self.tf.lookup_transform('base_link', frame, Time())
        except tf2_ros.TransformException:
            return
        p, q = t.transform.translation, t.transform.rotation
        mount = ground.Mount((p.x, p.y, p.z), ground.rotation(q.x, q.y, q.z, q.w),
                             ground.is_optical(frame))
        seen = []
        for d in msg.detections:
            if not d.results:
                continue
            best = max(d.results, key=lambda r: r.hypothesis.score).hypothesis
            if best.score < self.name_min_score:
                continue
            cx, cy = d.bbox.center.position.x, d.bbox.center.position.y
            hw, hh = d.bbox.size_x / 2.0, d.bbox.size_y / 2.0
            seen.append(Seen(best.class_id, best.score, (cx - hw, cy - hh, cx + hw, cy + hh)))
        now = self.get_clock().now().nanoseconds * 1e-9
        self.namer.cameras[frame] = Camera(lens, mount, seen, now)

    def name_of(self, track):
        """Return a track's name from the cameras (Tracker.name's hook)."""
        try:
            t = self.tf.lookup_transform(ODOM, 'base_link', Time())
        except tf2_ros.TransformException:
            return track.kind
        cx, cy = t.transform.translation.x, t.transform.translation.y
        yaw = yaw_of(t.transform.rotation)
        dx, dy = track.x[0] - cx, track.x[1] - cy
        c, s = math.cos(yaw), math.sin(yaw)
        now = self.get_clock().now().nanoseconds * 1e-9
        return self.namer.name(track.id, c * dx + s * dy, -s * dx + c * dy, track.length,
                               track.width, track.height, now)

    def on_map(self, msg):
        info = msg.info
        self.background = Background.from_grid(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, self.background_slack)
        self.get_logger().info(
            f'background: the saved map, {info.width} x {info.height} cells at '
            f'{info.resolution:.2f} m; walls grown {self.background_slack:.2f} m')

    def drop_background(self, points, msg):
        """Return the cubes not on the saved map; all of them without a map or map pose."""
        if self.background is None or len(points) == 0:
            return points
        try:
            t = self.tf.lookup_transform('map', msg.header.frame_id, Time())
        except tf2_ros.TransformException:
            return points
        tr = t.transform.translation
        kept = new_only(points, (tr.x, tr.y, yaw_of(t.transform.rotation)), self.background)
        self.background_dropped += len(points) - len(kept)
        self.get_logger().info(f'background: {self.background_dropped} cubes on the saved map '
                               'so far, not things', throttle_duration_sec=10.0)
        return kept

    def look(self, msg):
        """
        Return (found, newest) for tf_wait.decide().

        found: the transform at msg's stamp is there; newest: the newest transform's time
        (s), or None if there is none.
        """
        frame = msg.header.frame_id
        if self.tf.can_transform(ODOM, frame, Time.from_msg(msg.header.stamp)):
            return True, None
        try:
            newest = seconds(self.tf.lookup_transform(ODOM, frame, Time()).header.stamp)
        except tf2_ros.TransformException:
            return False, None
        self.behind = seconds(msg.header.stamp) - newest
        return False, newest

    def pose(self, msg, step):
        """Return (x, y, yaw) of msg's frame in odom at its stamp, or the newest (NEWEST)."""
        frame = msg.header.frame_id
        at = Time() if step is Step.NEWEST else Time.from_msg(msg.header.stamp)
        try:
            t = self.tf.lookup_transform(ODOM, frame, at)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'no {ODOM} <- {frame}: {e}', throttle_duration_sec=5.0)
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_of(t.transform.rotation)

    def wait(self, stream, msg):
        """Keep a cloud until its transform comes, then use what is ready."""
        self.waiting[stream].add(seconds(msg.header.stamp), msg)
        self.drain()

    def drain(self):
        """Use every waiting cloud whose transform has come, oldest first; say the drops."""
        now = self.get_clock().now().nanoseconds * 1e-9
        for stream, use in (('cubes', self.use_cubes), ('radar', self.use_clusters)):
            waiting = self.waiting[stream]
            for msg, step in waiting.take(lambda _stamp, m: self.look(m), now):
                pose = self.pose(msg, step)
                if pose is not None:
                    use(msg, pose)
            if waiting.dropped:
                self.get_logger().warn(
                    f'{stream}: {waiting.dropped} cloud(s) dropped, no {ODOM} transform at '
                    f'their stamp (the newest was {self.behind:.2f} s behind)',
                    throttle_duration_sec=5.0)
                waiting.dropped = 0

    def on_odom(self, msg):
        # The local EKF's twist is in base_link: turn it into odom.
        yaw = yaw_of(msg.pose.pose.orientation)
        v = msg.twist.twist.linear
        self.car_velocity = (v.x * math.cos(yaw) - v.y * math.sin(yaw),
                             v.x * math.sin(yaw) + v.y * math.cos(yaw))

    def on_cubes(self, msg):
        self.wait('cubes', msg)

    def use_cubes(self, msg, pose):
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        points = self.drop_background(numpy.asarray(xyz, dtype=float).reshape(-1, 3), msg)
        found = to_frame(group(points, self.group_shape), *pose)
        self.car = pose[:2]
        self.tracker.step(seconds(msg.header.stamp), found, self.car)

    def on_radar(self, msg):
        if msg.width * msg.height:
            self.wait('radar', msg)

    def use_clusters(self, msg, pose):
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
        self.drain()
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
    """Return a box, a speed arrow and the id for each thing; DELETEALL first."""
    array = MarkerArray(markers=[Marker(header=things.header, action=Marker.DELETEALL)])
    for th in things.things:
        m = Marker(header=things.header, ns='things', id=th.id * 3, type=Marker.CUBE)
        m.pose.position.x, m.pose.position.y = float(th.x), float(th.y)
        m.pose.position.z = th.height / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y = max(th.length, 0.1), max(th.width, 0.1)
        m.scale.z = max(th.height, 0.1)
        m.color.r, m.color.g, m.color.b = looks.colour(th.kind, th.coasting)
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
        text.text = looks.caption(th.id, th.kind, th.vx, th.vy, th.radar)
        array.markers += [m, arrow, text]
    return array


def main(args=None):
    rclpy.init(args=args)
    node = ThingsNode()
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
