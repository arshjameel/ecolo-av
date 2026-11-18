"""
gps_heading: the car's heading from the GPS receiver's velocity, for the global EKF.

Subscribes  sensors/gps/fix_velocity      geometry_msgs/TwistWithCovarianceStamped: the
                                          antenna's velocity, ENU (east, north, up), m/s,
                                          from the receiver's Doppler (ublox_gps; Gazebo's
                                          sim_gps)
            vehicle/odom                  nav_msgs/Odometry: the wheels' signed speed and
                                          the turn rate (which way the car goes, and the
                                          antenna's swing in a turn)
Publishes   localization/gps/heading      sensor_msgs/Imu, in base_link: only an
                                          orientation (the heading, yaw from east,
                                          counter-clockwise) and its variance; no turn rate
                                          or acceleration (covariance -1, REP-145)

Parameters  min_speed (0.3 m/s), max_speed_gap (0.3 m/s), floor_deg (2.0): course.py's
            CourseSettings. Where the antenna sits comes from vehicle.yaml's gps_link.

Nothing is published while the car is slower than min_speed, so a parked car keeps the
heading the gyro carries. The maths is in course.py (no ROS). TODO 13; docs/learn-localization.md.
"""

import math
import signal

from aav_description import vehicle
from aav_gps_heading.course import CourseSettings, heading, quaternion_z
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

LOOSE = 1.0     # rad^2: roll and pitch, which this says nothing about (the EKF is 2D)


class GpsHeading(Node):

    def __init__(self):
        super().__init__('gps_heading')
        default = CourseSettings()
        self.settings = CourseSettings(
            min_speed=float(self.declare_parameter('min_speed', default.min_speed).value),
            max_speed_gap=float(self.declare_parameter('max_speed_gap',
                                                       default.max_speed_gap).value),
            floor=math.radians(float(self.declare_parameter(
                'floor_deg', math.degrees(default.floor)).value)))
        xyz = vehicle.load()['sensors']['gps_link']['xyz']
        self.antenna = (float(xyz[0]), float(xyz[1]))
        self.speed = self.turn = None
        self.pub = self.create_publisher(Imu, 'localization/gps/heading', 10)
        self.create_subscription(Odometry, 'vehicle/odom', self.on_odom, 10)
        self.create_subscription(TwistWithCovarianceStamped, 'sensors/gps/fix_velocity',
                                 self.on_velocity, 10)
        self.get_logger().info(
            f'gps_heading: a heading above {self.settings.min_speed:g} m/s, the antenna at '
            f'{self.antenna} from base_link')

    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x
        self.turn = msg.twist.twist.angular.z

    def on_velocity(self, msg):
        if self.speed is None:
            return
        v, cov = msg.twist.twist.linear, msg.twist.covariance
        out = heading(v.x, v.y, cov[0], cov[7], self.speed, self.turn, self.antenna,
                      self.settings)
        if out is None:
            return
        yaw, variance = out
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = 'base_link'
        q = imu.orientation
        q.x, q.y, q.z, q.w = quaternion_z(yaw)
        imu.orientation_covariance[0] = imu.orientation_covariance[4] = LOOSE
        imu.orientation_covariance[8] = variance
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = GpsHeading()
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
