"""
lidar_scan: the roof lidar's cloud as a 2D scan around base_link, for Nav2.

Subscribes  sensors/lidar/points  sensor_msgs/PointCloud2 (the lidar's driver; Gazebo's
                                  bridge in the simulator), reliable, the newest only
Publishes   sensors/lidar/scan    sensor_msgs/LaserScan, best effort, in base_link: for each
                                  direction, the nearest point between min_height and
                                  max_height above the road (scan.py), inf where none

Parameters  min_height, max_height (m above the road; navigation.launch.py sets them from
            vehicle.yaml), angle_min, angle_max, angle_increment (rad), range_min,
            range_max (m from base_link), scan_time (s). Where the lidar sits comes from
            vehicle.yaml's lidar_link, as robot_state_publisher's frames do; the car's own
            outline (its points don't count) from vehicle.yaml's body.

navigation.launch.py lidar:=true starts it, with its parameters from nav2.yaml (lidar_scan:).
The local costmap's lidar_layer and collision_monitor read the scan.
"""

import signal

from aav_description import vehicle
from aav_lidar.scan import body_box, flatten, ScanShape, to_base
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class LidarScan(Node):

    def __init__(self):
        super().__init__('lidar_scan')
        default = ScanShape()
        car = vehicle.load()
        self.shape = ScanShape(**{
            name: float(self.declare_parameter(name, getattr(default, name)).value)
            for name in ('min_height', 'max_height', 'angle_min', 'angle_max',
                         'angle_increment', 'range_min', 'range_max')},
            body=body_box(vehicle.footprint_points(car['vehicle'])))
        self.scan_time = self.declare_parameter('scan_time', 0.1).value
        mount = car['sensors']['lidar_link']
        self.xyz, self.rpy = mount['xyz'], mount['rpy']
        self.pub = self.create_publisher(LaserScan, 'sensors/lidar/scan', qos_profile_sensor_data)
        # Reliable: best effort loses whole clouds where they travel by UDP (docs/mac.md).
        # Depth 1: a late cloud is dropped for the newest.
        self.create_subscription(PointCloud2, 'sensors/lidar/points', self.on_cloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(
            f'lidar_scan: {self.shape.count} directions, obstacles from '
            f'{self.shape.min_height:.2f} to {self.shape.max_height:.2f} m above the road, '
            f'the lidar at {self.xyz} in base_link')

    def on_cloud(self, msg):
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        points = to_base(numpy.asarray(xyz, dtype=float).reshape(-1, 3), self.xyz, self.rpy)
        s = self.shape
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = 'base_link'
        scan.angle_min = s.angle_min
        scan.angle_increment = s.angle_increment
        scan.angle_max = s.angle_min + (s.count - 1) * s.angle_increment
        scan.scan_time = float(self.scan_time)
        scan.range_min = s.range_min
        scan.range_max = s.range_max
        scan.ranges = flatten(points, s).astype(numpy.float32).tolist()
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarScan()
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
