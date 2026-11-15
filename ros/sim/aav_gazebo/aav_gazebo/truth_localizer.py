"""
truth_localizer: the simulator's truth as the car's pose, in place of the global EKF.

A test, sim only (sim.launch.py truth_pose:=true; TODO 149, the wiggle's first test): with
no GPS steps and no EKF in the map frame, does the car still wiggle? ekf_local still gives
odom -> base_link; this node puts map -> odom on top of it so that map -> base_link is
exactly the truth (pose_frames.py).

Subscribes  /vehicle/sim/truth          nav_msgs/Odometry, map -> base_link
Publishes   /localization/odom/global   nav_msgs/Odometry, map -> base_link: the truth, as
                                        ekf_global would publish it (navsat_transform and
                                        the watchdog read it)
            TF map -> odom              at each truth sample, against ekf_local's odom ->
                                        base_link at the same stamp (else its newest)
Parameters  none
"""

from aav_gazebo.pose_frames import map_to_odom, quaternion_of, yaw_of
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class TruthLocalizer(Node):

    def __init__(self):
        super().__init__('truth_localizer')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/localization/odom/global', 10)
        self.create_subscription(Odometry, '/vehicle/sim/truth', self.on_truth, 10)

    def odom_base(self, stamp):
        """Return ekf_local's odom -> base_link (x, y, yaw) at stamp, else its newest."""
        for when in (Time.from_msg(stamp), Time()):
            try:
                t = self.tf_buffer.lookup_transform('odom', 'base_link', when).transform
            except TransformException:
                continue
            q = t.rotation
            return (t.translation.x, t.translation.y, yaw_of(q.z, q.w, q.x, q.y))
        return None

    def on_truth(self, msg):
        out = Odometry()
        out.header.stamp, out.header.frame_id = msg.header.stamp, 'map'
        out.child_frame_id = 'base_link'
        out.pose.pose = msg.pose.pose
        out.pose.pose.position.z = 0.0
        out.twist = msg.twist
        for i in (0, 7, 14, 21, 28, 35):
            out.pose.covariance[i] = 1e-4
            out.twist.covariance[i] = 1e-4
        self.pub.publish(out)
        odom = self.odom_base(msg.header.stamp)
        if odom is None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        x, y, yaw = map_to_odom((p.x, p.y, yaw_of(q.z, q.w, q.x, q.y)), odom)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id = msg.header.stamp, 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = quaternion_of(yaw)
        self.broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = TruthLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
