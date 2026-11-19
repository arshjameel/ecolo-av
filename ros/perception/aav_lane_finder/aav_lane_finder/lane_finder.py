"""
lane_finder: the lane lines in the front camera's pictures, as /perception/lane/geometry.

Subscribes  /sensors/camera/front_left/image_raw    sensor_msgs/Image (reliable, the newest
                                                    only)
            /sensors/camera/front_left/camera_info  sensor_msgs/CameraInfo: the lens's focal
                                                    length, centre and distortion
Publishes   /perception/lane/geometry               aav_lane_geometry/LaneGeometry, one per
                                                    picture, in base_link, each line's
                                                    paint (white or yellow) too: the
                                                    costmap's camera lanes and lane_pose
                                                    read it
            /perception/lane/image                  sensor_msgs/Image: the bird's-eye view
                                                    with the paint, the windows and the lines
                                                    found (only with debug_image)

Classic OpenCV, no model: a bird's-eye warp of a flat road, a mask for the paint, sliding
windows and a second-order fit per line (birds_eye.py and lane_lines.py hold the maths).
A model can replace it later, behind the same topic.

The camera's place on the car comes from vehicle.yaml, through the launch file
(camera_xyz, camera_rpy). The lens comes from /sensors/camera/front_left/camera_info;
until it arrives, the
node uses focal_px and the middle of the picture.
"""

from dataclasses import fields
import signal
import time

from aav_lane_finder.birds_eye import BirdsEye, Grid, Lens, Mount
from aav_lane_finder.lane_lines import draw, Lane, LaneFinder, Settings
from aav_lane_geometry.msg import LaneGeometry
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

# Only the newest picture, and reliable: a camera picture is too big for one UDP packet,
# and best effort loses the whole picture when one piece is lost (aav_detector's object detector).
LATEST = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE)
# The distortion models OpenCV's projectPoints knows.
DISTORTIONS = ('plumb_bob', 'rational_polynomial')


class LaneFinderNode(Node):

    def __init__(self):
        super().__init__('lane_finder')
        self.mount = Mount(tuple(self.declare_parameter('camera_xyz', [1.80, 0.29, 1.20]).value),
                           tuple(self.declare_parameter('camera_rpy', [0.0, 0.0, 0.0]).value))
        self.focal = self.declare_parameter('focal_px', 343.0).value
        self.use_info = self.declare_parameter('use_camera_info', True).value
        self.grid = Grid(**{f.name: self.declare_parameter(f.name, getattr(Grid(), f.name)).value
                            for f in fields(Grid)})
        self.settings = Settings(**{f.name: self.declare_parameter(
            f.name, getattr(Settings(), f.name)).value for f in fields(Settings)})
        debug = self.declare_parameter('debug_image', False).value

        self.bridge = CvBridge()
        self.info = None
        self.finder = None
        self.pub = self.create_publisher(LaneGeometry, '/perception/lane/geometry', 10)
        self.debug_pub = (self.create_publisher(Image, '/perception/lane/image', 2)
                          if debug else None)
        self.create_subscription(CameraInfo, '/sensors/camera/front_left/camera_info',
                                 self.on_info, LATEST)
        self.create_subscription(Image, '/sensors/camera/front_left/image_raw',
                                 self.on_image, LATEST)
        self.seen = None
        self.frames, self.busy_s, self.since = 0, 0.0, time.monotonic()
        self.get_logger().info(
            f'camera at {list(self.mount.xyz)} m, turned {list(self.mount.rpy)} rad; looking '
            f'{self.grid.x_min}-{self.grid.x_max} m ahead; the offset {self.settings.lookahead} '
            'm ahead')

    def on_info(self, msg: CameraInfo):
        self.info = msg

    def lens_for(self, width: int, height: int) -> Lens:
        """Return the lens: camera_info's, if it describes pictures of this size."""
        info = self.info
        if not self.use_info or info is None or info.k[0] <= 0.0:
            return Lens.pinhole(width, height, self.focal)
        if (info.width, info.height) != (width, height):
            self.get_logger().warning(
                f'camera_info is for {info.width} x {info.height} pictures, these are '
                f'{width} x {height}: using focal_px', throttle_duration_sec=30.0)
            return Lens.pinhole(width, height, self.focal)
        distortion = ()
        if any(info.d):
            if info.distortion_model in DISTORTIONS:
                distortion = tuple(float(d) for d in info.d)
            else:
                self.get_logger().warning(f'ignoring the {info.distortion_model} distortion',
                                          throttle_duration_sec=30.0)
        return Lens(width, height, float(info.k[0]), float(info.k[4]), float(info.k[2]),
                    float(info.k[5]), distortion)

    def finder_for(self, width: int, height: int) -> LaneFinder:
        lens = self.lens_for(width, height)
        if self.finder is None or self.finder.view.lens != lens:
            width_seen = self.finder.width if self.finder else self.settings.lane_width
            self.finder = LaneFinder(BirdsEye(lens, self.mount, self.grid), self.settings)
            self.finder.width = width_seen
            self.get_logger().info(
                f'lens: {lens.width} x {lens.height}, focal {lens.fx:.1f} px; the road is seen '
                f'from {self.finder.view.nearest_seen():.1f} m ahead')
        return self.finder

    def on_image(self, msg: Image):
        t0 = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            finder = self.finder_for(frame.shape[1], frame.shape[0])
            lane = finder.find(frame)
        except Exception as e:  # noqa: B902 - a bad picture must not stop the node
            self.get_logger().error(f'lane finding failed: {e}', throttle_duration_sec=5.0)
            return
        self.pub.publish(message(lane, msg))
        if self.debug_pub is not None:
            picture = self.bridge.cv2_to_imgmsg(draw(finder.last, lane, self.grid), 'bgr8')
            picture.header = msg.header
            self.debug_pub.publish(picture)
        self.report(lane, time.monotonic() - t0)

    def report(self, lane: Lane, busy: float):
        seen = (lane.left is not None, lane.right is not None)
        if seen != self.seen:
            names = [n for n, s in zip(('left', 'right'), seen) if s]
            self.get_logger().info(f'lines seen: {" and ".join(names) or "none"}')
            self.seen = seen
        self.frames += 1
        self.busy_s += busy
        if time.monotonic() - self.since >= 10.0:
            self.get_logger().info(f'{self.frames} pictures in 10 s, '
                                   f'{1000.0 * self.busy_s / self.frames:.1f} ms each')
            self.frames, self.busy_s, self.since = 0, 0.0, time.monotonic()


def message(lane: Lane, image: Image) -> LaneGeometry:
    """Return the lane as LaneGeometry, stamped with the picture's time, in base_link."""
    msg = LaneGeometry()
    msg.header.stamp = image.header.stamp
    msg.header.frame_id = 'base_link'
    msg.left_curvature_m = lane.left_radius
    msg.right_curvature_m = lane.right_radius
    msg.mean_curvature_m = lane.mean_radius
    msg.curve_direction = lane.curve_direction
    msg.vehicle_offset_m = lane.offset
    msg.lane_width_m = lane.width
    if lane.left:
        msg.left_poly_a, msg.left_poly_b, msg.left_poly_c = lane.left.a, lane.left.b, lane.left.c
    if lane.right:
        msg.right_poly_a, msg.right_poly_b, msg.right_poly_c = (lane.right.a, lane.right.b,
                                                                lane.right.c)
    msg.left_confidence = lane.left_confidence
    msg.right_confidence = lane.right_confidence
    msg.left_lane_detected = lane.left is not None
    msg.right_lane_detected = lane.right is not None
    msg.both_lanes_detected = lane.left is not None and lane.right is not None
    msg.steering_hint = lane.steering_hint
    msg.left_paint = lane.left_paint
    msg.right_paint = lane.right_paint
    return msg


def main():
    rclpy.init()
    node = None
    try:
        node = LaneFinderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C reaches a launched node twice (the terminal, then launch): ignore the
        # second, so it can't interrupt the clean-up with a traceback.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
