"""
turtlesim_adapter: drive the turtlesim turtle like a car.

Subscribes  control/turtle/actuator_cmd  ackermann_msgs/AckermannDriveStamped: ego's
                              command, from the output mux (aav_io_mux, target turtle)
            turtle1/pose      turtlesim/Pose
Publishes   turtle1/cmd_vel   geometry_msgs/Twist (bicycle model: w = v tan(d) / L)
            vehicle/odom      nav_msgs/Odometry (twist only), so ekf_local can run
            vehicle/status    aav_interfaces/VehicleStatus (mode COMPUTER, armed), so the
                              HUD shows the speed and steering, as with Gazebo
            vehicle/steering  aav_interfaces/ActuatorState: command = measured (a turtle
                              steers at once)
            system/source     aav_interfaces/DataSource, latched: all of it is SIM

The fastest way to test the joystick, mux, ego, scripted driver and ekf_local
together on a laptop. One turtlesim unit counts as one metre. No command for cmd_timeout
(0.2 s, as sim_adapter) stops the turtle, so a closed output mux stops it as it stops
the other stand-ins.
"""

import math
import signal
import time

from aav_interfaces.msg import ActuatorState, DataSource, SensorSource, VehicleStatus
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from turtlesim.msg import Pose

# /system/source: one message, kept for late subscribers.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TurtlesimAdapter(Node):

    def __init__(self):
        super().__init__('turtlesim_adapter')
        self.wheelbase = self.declare_parameter('wheelbase', 1.57).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.2).value   # s
        self.cmd_at = None          # time.monotonic() of the last command; None = stopped
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, 'vehicle/odom', 10)
        self.status_pub = self.create_publisher(VehicleStatus, 'vehicle/status', 10)
        self.steering = 0.0
        self.create_subscription(AckermannDriveStamped, 'control/turtle/actuator_cmd',
                                 self.on_cmd, 10)
        self.steering_pub = self.create_publisher(ActuatorState, 'vehicle/steering', 10)
        self.create_subscription(Pose, 'turtle1/pose', self.on_pose, 10)
        self.create_timer(0.05, self.check_stale)

        # Say once, latched, that this car is a turtle.
        self.source_pub = self.create_publisher(DataSource, 'system/source', LATCHED)
        announcement = DataSource(announcer=self.get_name())
        announcement.header.stamp = self.get_clock().now().to_msg()
        announcement.sensors = [SensorSource(name=n, topic=t, source=SensorSource.SIM)
                                for n, t in (('vehicle', '/vehicle/status'),
                                             ('odometry', '/vehicle/odom'))]
        self.source_pub.publish(announcement)

    def on_cmd(self, msg):
        self.cmd_at = time.monotonic()
        self.steering = msg.drive.steering_angle
        steer = ActuatorState(command=self.steering, measured=self.steering)
        steer.header.stamp = msg.header.stamp
        steer.header.frame_id = 'base_link'
        self.steering_pub.publish(steer)
        t = Twist()
        t.linear.x = msg.drive.speed
        t.angular.z = msg.drive.speed * math.tan(msg.drive.steering_angle) / self.wheelbase
        self.cmd_pub.publish(t)

    def check_stale(self):
        # A steady clock: the turtle runs on wall time, whatever use_sim_time says.
        if self.cmd_at is not None and time.monotonic() - self.cmd_at > self.cmd_timeout:
            self.cmd_at = None
            self.cmd_pub.publish(Twist())       # one zero: turtlesim then holds still
            self.get_logger().warning(
                f'no command for {self.cmd_timeout} s: stopped the turtle')

    def on_pose(self, pose):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = pose.linear_velocity
        odom.twist.twist.angular.z = pose.angular_velocity
        cov = [0.0] * 36
        cov[0] = cov[35] = 1e-3
        cov[7] = 1e-4
        cov[14] = cov[21] = cov[28] = 1e3
        odom.twist.covariance = cov
        odom.pose.covariance = [1e3 if i % 7 == 0 else 0.0 for i in range(36)]
        self.odom_pub.publish(odom)

        s = VehicleStatus()
        s.header.stamp = odom.header.stamp
        s.header.frame_id = 'base_link'
        s.mode = VehicleStatus.MODE_COMPUTER
        s.armed = True
        s.speed = pose.linear_velocity
        s.steering_angle = self.steering
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C reaches a launched node twice (terminal and launch), and Jazzy can then
        # raise RCLError ("context is not valid"). Only a live context means a real error.
        if rclpy.ok():
            raise
    finally:
        # Ctrl-C reaches a launched node twice (the terminal, then launch): ignore the
        # second, so it can't interrupt the clean-up with a traceback.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
