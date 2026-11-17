"""
camera_obstacles: the detectors' boxes as points on the road, for the local costmap.

Subscribes  /perception/detections        vision_msgs/Detection2DArray, from every detector
                                          (one per computer with a GPU). Each message's
                                          header.frame_id names its camera
            camera_info_topics            sensor_msgs/CameraInfo, one topic per camera
                                          (default /sensors/camera/front_left/camera_info):
                                          each lens, kept by the frame it names
Publishes   /perception/camera_obstacles  sensor_msgs/PointCloud2 in base_link: where each
                                          counted box's bottom edge meets the road, from
                                          every camera heard in the last max_age s
            /perception/camera_view       sensor_msgs/PointCloud2 in base_link: the far edge
                                          of what those cameras see, on the road. The
                                          costmap clears its camera layer out to it
Reads TF    base_link -> each camera's frame (vehicle.yaml, through the URDF)

Every camera's newest points go out together, each time (recent.py): so a person behind
the car doesn't vanish with every front picture, for the costmap or the stop zone.

Only the listed classes count (people, cyclists, cars...), and stop signs never do: they
lock twist_mux instead (stop_sign_watcher). The maths is in ground.py: a pinhole camera
over a flat road. Off by default until the cameras are calibrated (Q22): bench.launch.py
camera_obstacles:=true starts it, or on the car the machine file's camera_obstacles part,
and either turns on the costmap layer that reads it (navigation.launch.py
camera_obstacles:=true).

Needs no PyTorch: it runs in the default environment.
"""

import signal

from aav_detector import ground
from aav_detector.recent import Recent
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray


class CameraObstacles(Node):

    def __init__(self):
        super().__init__('camera_obstacles')
        self.classes = list(self.declare_parameter('classes', list(ground.ROAD_USERS)).value)
        self.min_score = self.declare_parameter('min_score', 0.5).value
        self.max_range = self.declare_parameter('max_range', 12.0).value
        self.step = self.declare_parameter('step', 0.1).value
        self.view_range = self.declare_parameter('view_range', 12.0).value
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.recent = Recent(self.declare_parameter('max_age', 1.0).value)
        topics = self.declare_parameter('camera_info_topics',
                                        ['/sensors/camera/front_left/camera_info']).value
        self.lenses = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        for topic in topics:
            self.create_subscription(CameraInfo, topic, self.on_camera_info, 10)
        self.obstacles = self.create_publisher(PointCloud2, '/perception/camera_obstacles', 10)
        self.view = self.create_publisher(PointCloud2, '/perception/camera_view', 10)
        self.create_subscription(Detection2DArray, '/perception/detections', self.on_detections,
                                 10)
        dropped = sorted(ground.NEVER & {ground.normal_label(c) for c in self.classes})
        self.get_logger().info(f'counting {", ".join(self.classes)} within {self.max_range} m'
                               + (f'; never {", ".join(dropped)}' if dropped else ''))

    def on_camera_info(self, msg: CameraInfo):
        lens = ground.lens_from_k(msg.k, msg.width, msg.height)
        if lens is None:
            self.get_logger().warning(f'{msg.header.frame_id}: its CameraInfo has no lens '
                                      '(K is 0): calibrate it', throttle_duration_sec=30.0)
            return
        self.lenses[msg.header.frame_id] = lens

    def mount(self, frame: str) -> ground.Mount:
        # The cameras are fixed to the car, so the newest transform is as good as any.
        t = self.tf.lookup_transform(self.base, frame, Time())
        p, q = t.transform.translation, t.transform.rotation
        return ground.Mount((p.x, p.y, p.z), ground.rotation(q.x, q.y, q.z, q.w),
                            ground.is_optical(frame))

    def on_detections(self, msg: Detection2DArray):
        frame = msg.header.frame_id
        lens = self.lenses.get(frame)
        if lens is None:
            self.get_logger().warning(f'no CameraInfo yet for camera {frame!r}: add its topic '
                                      'to camera_info_topics', throttle_duration_sec=10.0)
            return
        try:
            mount = self.mount(frame)
        except TransformException as e:
            self.get_logger().warning(f'no TF from {self.base} to {frame!r}: {e}',
                                      throttle_duration_sec=10.0)
            return
        points = []
        for d in msg.detections:
            if not d.results:
                continue
            best = max(d.results, key=lambda r: r.hypothesis.score).hypothesis
            if best.score < self.min_score or not ground.counts(best.class_id, self.classes):
                continue
            points += ground.box_points(lens, mount, d.bbox.center.position.x,
                                        d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y,
                                        self.step, self.max_range)
        # The picture's time, so the costmap places the points where the car was then.
        header = Header(stamp=msg.header.stamp, frame_id=self.base)
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.recent.update(frame, now, points,
                           ground.view_edge(lens, mount, self.view_range, self.step))
        self.obstacles.publish(point_cloud2.create_cloud_xyz32(
            header, [(x, y, 0.0) for x, y in self.recent.obstacles(now)]))
        self.view.publish(point_cloud2.create_cloud_xyz32(
            header, [(x, y, 0.0) for x, y in self.recent.view(now)]))


def main():
    rclpy.init()
    node = None
    try:
        node = CameraObstacles()
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
