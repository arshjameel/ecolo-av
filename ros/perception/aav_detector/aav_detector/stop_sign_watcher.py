"""
stop_sign_watcher: stop the car at stop signs, through a lock on twist_mux.

Subscribes  /perception/detections  vision_msgs/Detection2DArray (from the detector)
            camera_info_topics      sensor_msgs/CameraInfo, one topic per camera (default
                                    /sensors/camera/<front_left, front_right, rear>/
                                    camera_info): its lens, kept by the frame it names
Publishes   /perception/stop_sign        std_msgs/Bool, 10 Hz: true while the car must stay stopped
            /perception/stop_sign/distance  geometry_msgs/PointStamped: where the tallest
                                    stop sign seen by a forward camera is, in that camera's
                                    frame at the picture's stamp (sign_point: the range from
                                    its height, the bearing from its column and the lens).
                                    Only while CLEAR: the behaviour planner slows for it.
                                    With lidar_distance (off by default), a thing the
                                    things node named stop sign within 3 m of that guess
                                    gives the point instead: the lidar's distance, not the
                                    pixel height's (TODO 192 step 2)
            /world/things           aav_interfaces/Things, odom (lidar_distance only)
Uses TF     odom <-> each forward camera's frame (lidar_distance only)

A sign counts once it is stop_distance_m (4.7) away or closer: its box's height, the
sign's (sign_height_m, 0.75) and the focal length of the camera that saw it, the fy of
the CameraInfo with the detections' frame_id (stop_sign.py). A camera with no CameraInfo
yet (not calibrated) gets focal_px (343: Gazebo's and the bench's lens, 640x360), with a
warning: set focal_px for it, or calibrate it (docs/camera-calibration.md).

twist_mux's stop_sign lock (aav_command_mux, priority 30) holds every driver below it: Nav2 (20)
and the AI slot (10). A person (joystick 100, keyboard 90) and test scripts (50, 40) still
drive. The lock never expires on silence, so if this node dies while it's on, the car stays
stopped. It only ever stops the car; it can't make it move.

Needs no PyTorch: it runs in the default environment.
"""

import signal

from aav_detector.ground import is_optical
from aav_detector.stop_sign import (biggest_sign, CLEAR, lidar_sign, sign_point, StopRule,
                                    StopSignWatcher)
from aav_interfaces.msg import Things
from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped with tf2
import tf2_ros
from vision_msgs.msg import Detection2DArray

CAMERA_INFO = [f'/sensors/camera/{c}/camera_info' for c in ('front_left', 'front_right', 'rear')]


class Watcher(Node):

    def __init__(self):
        super().__init__('stop_sign_watcher')
        p = self.declare_parameter
        rule = StopRule(
            stop_distance_m=p('stop_distance_m', StopRule.stop_distance_m).value,
            focal_px=p('focal_px', StopRule.focal_px).value,
            sign_height_m=p('sign_height_m', StopRule.sign_height_m).value,
            min_score=p('min_score', StopRule.min_score).value,
            hold_s=p('hold_s', StopRule.hold_s).value,
            ignore_s=p('ignore_s', StopRule.ignore_s).value,
        )
        self.watcher = StopSignWatcher(rule)
        self.focal = {}                 # frame_id -> fy (px), from each camera's CameraInfo
        self.lens = {}                  # frame_id -> (fx, cx) (px), for the sign's bearing
        # The forward cameras' frames: a frame naming one of these words. The rear camera's
        # signs face the other way, and must never slow the car.
        self.forward = list(p('forward_words', ['front', 'left', 'right']).value)
        self.backward = list(p('backward_words', ['rear']).value)
        for topic in p('camera_info_topics', CAMERA_INFO).value:
            self.create_subscription(CameraInfo, topic, self.on_camera_info, 10)
        self.get_logger().info(
            f'stops {rule.stop_distance_m:g} m before a {rule.sign_height_m:g} m stop sign: '
            f'{rule.min_height_px():.0f} px tall with focal_px {rule.focal_px:g}, the lens '
            'of a camera without CameraInfo')
        self.pub = self.create_publisher(Bool, '/perception/stop_sign', 10)
        self.point = self.create_publisher(PointStamped, '/perception/stop_sign/distance', 10)
        self.create_subscription(Detection2DArray, '/perception/detections', self.on_boxes, 10)
        self.create_timer(0.1, self.tick)
        self.state = self.watcher.state
        # The lidar's distance to a sign the things node named (TODO 192 step 2).
        self.use_lidar = bool(p('lidar_distance', False).value)
        self.gate_m = float(p('lidar_gate_m', 3.0).value)
        self.things = ('odom', [])          # (frame, [(kind, x, y)]) from /world/things
        if self.use_lidar:
            self.tf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf, self)
            self.create_subscription(Things, '/world/things', self.on_things, 10)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_camera_info(self, msg: CameraInfo):
        fy = float(msg.k[4])
        if float(msg.k[0]) > 0:
            self.lens[msg.header.frame_id] = (float(msg.k[0]), float(msg.k[2]))
        if fy > 0 and self.focal.get(msg.header.frame_id) != fy:
            self.get_logger().info(
                f'{msg.header.frame_id}: focal length {fy:.0f} px: stops for a stop sign '
                f'{self.watcher.rule.min_height_px(fy):.0f} px tall')
            self.focal[msg.header.frame_id] = fy

    def on_boxes(self, msg: Detection2DArray):
        boxes = [(d.results[0].hypothesis.class_id, d.results[0].hypothesis.score, d.bbox.size_y)
                 for d in msg.detections if d.results]
        focal = self.focal.get(msg.header.frame_id)
        if focal is None and boxes:
            self.get_logger().warning(
                f'no CameraInfo for camera {msg.header.frame_id!r}: its stop signs are judged '
                f'with focal_px {self.watcher.rule.focal_px:g}. Calibrate it, or set focal_px',
                throttle_duration_sec=30.0)
        if self.watcher.state == CLEAR and self.facing_forward(msg.header.frame_id):
            self.publish_point(msg, focal)
        self.publish(self.watcher.update(boxes, self.now(), focal))

    def facing_forward(self, frame: str) -> bool:
        return (not any(w in frame for w in self.backward)
                and any(w in frame for w in self.forward))

    def publish_point(self, msg: Detection2DArray, focal):
        sign = biggest_sign(
            ((d.results[0].hypothesis.class_id, d.results[0].hypothesis.score, d.bbox.size_y,
              d.bbox.center.position.x) for d in msg.detections if d.results),
            self.watcher.rule.min_score)
        if sign is None:
            return
        fx, cx = self.lens.get(msg.header.frame_id, (None, None))
        x, y, z = sign_point(sign[0], sign[1], focal or self.watcher.rule.focal_px, fx, cx,
                             is_optical(msg.header.frame_id), self.watcher.rule.sign_height_m)
        point = PointStamped(header=msg.header, point=Point(x=x, y=y, z=z))
        if self.use_lidar:
            point = self.lidar_point(point) or point
        self.point.publish(point)

    def on_things(self, msg: Things):
        self.things = (msg.header.frame_id, [(t.kind, t.x, t.y) for t in msg.things])

    def lidar_point(self, camera: PointStamped):
        """Return the named sign's lidar place in the camera's frame, or None."""
        frame, things = self.things
        if not things:
            return None
        try:
            guess = self.tf.transform(camera, frame)
            found = lidar_sign((guess.point.x, guess.point.y), things, self.gate_m)
            if found is None:
                return None
            guess.point.x, guess.point.y = found
            back = self.tf.transform(guess, camera.header.frame_id)
        except tf2_ros.TransformException:
            return None
        back.header = camera.header
        return back

    def tick(self):
        self.publish(self.watcher.tick(self.now()))

    def publish(self, stop: bool):
        if self.watcher.state != self.state:
            self.state = self.watcher.state
            self.get_logger().info(f'stop sign: {self.state}')
        self.pub.publish(Bool(data=stop))


def main():
    rclpy.init()
    node = None
    try:
        node = Watcher()
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
