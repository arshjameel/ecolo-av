"""
lane_pose: the lane finder as a pose sensor, for the global EKF (a trial, TODO 155).

Subscribes  /perception/lane/geometry   aav_lane_geometry/LaneGeometry: the painted lines
                                        in base_link, one per camera picture (lane_finder)
            TF map -> base_link          where the filter has the car at the picture's time
Publishes   /localization/lane/pose     geometry_msgs/PoseWithCovarianceStamped, map frame,
                                        the picture's stamp: the car's place across its
                                        lane (sure), along it the filter's own (50 m unsure)

Parameters  lane_map (the lanes.yaml, aav_description lane_map.py; needed), and every
            field of lane_fix.Settings by the same name: the gates and the covariance.

Only with localization.launch.py lane_pose:=true (sim.launch.py lane_pose:=true); then
ekf_global fuses its x and y (config/ekf_global_lane.yaml). Off by default: a challenger
beside the GPS alone until it wins (CLAUDE.md, "Our own algorithms"). The maths and the
gates are in lane_fix.py (no ROS). Every 10 s it logs how many pictures each gate refused.
"""

from dataclasses import fields
import math
import signal

from aav_description.lane_map import load
from aav_lane_geometry.msg import LaneGeometry
from aav_lane_pose.lane_fix import LanePose, lines_from, seen_from, Settings
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

GEOMETRY_FIELDS = ('left_poly_a', 'left_poly_b', 'left_poly_c', 'left_lane_detected',
                   'right_poly_a', 'right_poly_b', 'right_poly_c', 'right_lane_detected')


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LanePoseNode(Node):

    def __init__(self):
        super().__init__('lane_pose')
        path = self.declare_parameter('lane_map', '').value
        if not path:
            raise RuntimeError('lane_pose needs lane_map:= (a lanes.yaml)')
        settings = Settings(**{f.name: float(self.declare_parameter(
            f.name, getattr(Settings(), f.name)).value) for f in fields(Settings)})
        self.lanes = LanePose(load(path), settings)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         '/localization/lane/pose', 10)
        self.create_subscription(LaneGeometry, '/perception/lane/geometry', self.on_lane, 10)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'lane_pose: the lane map {path}, '
                               f'{len(self.lanes.lanes.roads)} roads')

    def pose_at(self, stamp):
        """Return the filter's (x, y, yaw) of base_link in the map at `stamp`, or None."""
        for when in (Time.from_msg(stamp), Time()):    # the picture's time, else the newest
            try:
                t = self.tf.lookup_transform('map', 'base_link', when,
                                             timeout=Duration(seconds=0.05))
            except TransformException:
                continue
            p = t.transform.translation
            return p.x, p.y, yaw_of(t.transform.rotation)
        return None

    def on_lane(self, msg: LaneGeometry):
        pose = self.pose_at(msg.header.stamp)
        if pose is None:
            return
        left, right = lines_from({n: getattr(msg, n) for n in GEOMETRY_FIELDS})
        seen = seen_from(left, right, msg.lane_width_m, msg.left_confidence,
                         msg.right_confidence)
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        fix, _ = self.lanes.update(t, pose, seen)
        if fix is None:
            return
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.pose.pose.position.x, out.pose.pose.position.y = fix.x, fix.y
        out.pose.pose.orientation.z = math.sin(fix.yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(fix.yaw / 2.0)
        out.pose.covariance = list(fix.covariance)
        self.pub.publish(out)

    def report(self):
        self.get_logger().info(f'pictures in 10 s: {self.lanes.summary()}')


def main():
    rclpy.init()
    node = None
    try:
        node = LanePoseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
