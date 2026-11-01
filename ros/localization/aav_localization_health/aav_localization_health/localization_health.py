"""
localization_health: warns when localization's inputs or outputs stop being believable.

Subscribes  sensors/imu/data           sensor_msgs/Imu: the gyro's bias parked, gravity
            vehicle/odom               nav_msgs/Odometry: the wheels' speed (parked or not)
            vehicle/wheels             aav_interfaces/WheelSpeeds: the front wheels, where
                                       the backend publishes them
            sensors/gps/fix            sensor_msgs/NavSatFix: fix, covariance
            localization/odom/local    nav_msgs/Odometry: ekf_local's output
            localization/odom/global   nav_msgs/Odometry: ekf_global's output
            tf, tf_static              base_link -> imu_link, gps_link; map -> base_link
Publishes   localization/health        std_msgs/String, at rate_hz: OK, or WARN and every
                                       check that isn't OK
            diagnostics                diagnostic_msgs/DiagnosticArray, at rate_hz: one
                                       status per check, "localization_health: <check>"

Warn only: it never stops the car, never touches the EKFs or their parameters, and
nothing reads /localization/health to drive. What each check means is in health.py (no
ROS there). Each change of state is logged once.

Parameters  rate_hz (1.0), and health.py's Settings by the same names
            (config/localization_health.yaml). The front track and wheelbase come from
            vehicle.yaml.
"""

import signal

from aav_description import vehicle
from aav_interfaces.msg import WheelSpeeds
from aav_localization_health.health import Health, OK, Settings, STALE, summary, WARN
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LEVELS = {OK: DiagnosticStatus.OK, WARN: DiagnosticStatus.WARN, STALE: DiagnosticStatus.STALE}
STATIC = ('imu_link', 'gps_link')


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LocalizationHealth(Node):

    def __init__(self):
        super().__init__('localization_health')
        defaults = Settings()
        settings = Settings(**{name: float(self.declare_parameter(name, value).value)
                               for name, value in vars(defaults).items()})
        rate = float(self.declare_parameter('rate_hz', 1.0).value)
        car = vehicle.load()['vehicle']
        self.health = Health(float(car['wheelbase']), float(car['track']), settings)
        self.levels = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state_pub = self.create_publisher(String, 'localization/health', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Imu, 'sensors/imu/data', self.on_imu, 50)
        self.create_subscription(Odometry, 'vehicle/odom', self.on_odom, 50)
        self.create_subscription(WheelSpeeds, 'vehicle/wheels', self.on_wheels, 50)
        self.create_subscription(NavSatFix, 'sensors/gps/fix', self.on_gps, 10)
        for name in ('local', 'global'):
            self.create_subscription(Odometry, f'localization/odom/{name}',
                                     lambda msg, n=name: self.on_ekf(n, msg), 10)
        self.create_timer(1.0 / rate, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg):
        a = msg.linear_acceleration
        self.health.imu(self.now(), seconds(msg.header.stamp), msg.header.frame_id,
                        msg.angular_velocity.z, (a.x, a.y, a.z))

    def on_odom(self, msg):
        self.health.odom(self.now(), seconds(msg.header.stamp), msg.header.frame_id,
                         msg.child_frame_id, msg.twist.twist.linear.x)

    def on_wheels(self, msg):
        self.health.wheels(self.now(), seconds(msg.header.stamp), msg.header.frame_id,
                           msg.front_left, msg.front_right)

    def on_gps(self, msg):
        self.health.gps_fix(self.now(), seconds(msg.header.stamp), msg.header.frame_id,
                            msg.status.status, msg.position_covariance_type,
                            list(msg.position_covariance))

    def on_ekf(self, name, msg):
        p, t = msg.pose.pose.position, msg.twist.twist
        self.health.ekf(name, self.now(), seconds(msg.header.stamp), msg.header.frame_id,
                        msg.child_frame_id, p.x, p.y, t.linear.x, t.angular.z)

    def look_at_tf(self):
        missing = []
        for frame in STATIC:
            try:
                self.tf_buffer.lookup_transform('base_link', frame, Time())
            except TransformException:
                missing.append(frame)
        try:
            tree = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            age = self.now() - seconds(tree.header.stamp)
        except TransformException:
            age = None
        self.health.tf(missing, age)

    def tick(self):
        self.look_at_tf()
        now_msg = self.get_clock().now().to_msg()
        statuses = self.health.report(self.now())
        self.state_pub.publish(String(data=summary(statuses)))
        array = DiagnosticArray()
        array.header.stamp = now_msg
        for s in statuses:
            array.status.append(DiagnosticStatus(
                level=LEVELS[s.level], name=f'localization_health: {s.name}',
                message=s.message, hardware_id='localization',
                values=[KeyValue(key=k, value=f'{v:.4g}') for k, v in s.values.items()]))
            # One call per severity: rclpy refuses one call site at two severities.
            if self.levels.get(s.name, OK) != s.level and s.level == OK:
                self.get_logger().info(f'{s.name}: {s.level}, {s.message}')
            elif self.levels.get(s.name, OK) != s.level:
                self.get_logger().warning(f'{s.name}: {s.level}, {s.message}')
            self.levels[s.name] = s.level
        self.diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationHealth()
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
