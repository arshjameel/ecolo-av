"""
voxels: the roof lidar's cloud as coarse cubes around base_link, for Nav2 (a trial).

Subscribes  sensors/lidar/points        sensor_msgs/PointCloud2 (the lidar's driver; Gazebo's
                                        bridge in the simulator), reliable, the newest only
Publishes   perception/voxels/cubes     sensor_msgs/PointCloud2, base_link, x y z float32:
                                        the centre of every cube (size, 0.25 m) holding a
                                        point between min_height and max_height above the
                                        road (cubes.py), the cloud's stamp
            perception/voxels/ground    sensor_msgs/PointCloud2, base_link: the centre of
                                        every flat square where the lidar saw the road (free
                                        space: the local costmap clears with it)
            perception/voxels/status    diagnostic_msgs/DiagnosticStatus, every cloud: ms,
                                        cubes, ground_cubes, points, fitted, roll_deg,
                                        pitch_deg, clouds_dropped

Parameters  size (m), min_height, max_height (m above the road; navigation.launch.py sets
            them from vehicle.yaml), range_max (m from base_link). Where the lidar sits
            comes from vehicle.yaml's lidar_link, as lidar_scan's; the car's own outline
            (its points don't count) from vehicle.yaml's body (aav_lidar scan.body_box).

navigation.launch.py voxels:=true (with lidar:=true) starts it, with its parameters from
nav2.yaml (voxels:). The local costmap's lidar_layer and collision_monitor then read the
cubes instead of /sensors/lidar/scan. docs/designs/2026-10-30_voxels.md.
"""

import signal
import time

from aav_description import vehicle
from aav_lidar.scan import body_box, to_base
from aav_voxels.cubes import cubes, CubeShape
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

PERIOD = 0.1     # s: the lidar's 10 Hz, to count the clouds that came while one was cut


class Voxels(Node):

    def __init__(self):
        super().__init__('voxels')
        default = CubeShape()
        car = vehicle.load()
        self.shape = CubeShape(**{
            name: float(self.declare_parameter(name, getattr(default, name)).value)
            for name in ('size', 'min_height', 'max_height', 'range_max')},
            body=body_box(vehicle.footprint_points(car['vehicle'])))
        mount = car['sensors']['lidar_link']
        self.xyz, self.rpy = mount['xyz'], mount['rpy']
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        # Reliable, depth 1: best-effort readers (the costmap's, collision_monitor's) still
        # connect, and a Mac's UDP doesn't lose them.
        self.cubes_pub = self.create_publisher(PointCloud2, 'perception/voxels/cubes', newest)
        self.ground_pub = self.create_publisher(PointCloud2, 'perception/voxels/ground', newest)
        self.status_pub = self.create_publisher(DiagnosticStatus, 'perception/voxels/status', 10)
        # Reliable: best effort loses whole clouds where they travel by UDP (docs/mac.md).
        self.create_subscription(PointCloud2, 'sensors/lidar/points', self.on_cloud, newest)
        self.last_stamp, self.dropped = None, 0
        s = self.shape
        self.get_logger().info(
            f'voxels: {s.size:.2f} m cubes from {s.min_height:.2f} to {s.max_height:.2f} m '
            f'above the fitted road, out to {s.range_max:.0f} m; the lidar at {self.xyz}')

    def on_cloud(self, msg):
        start = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None:
            # Keep-last-1: a gap of n lidar periods is n - 1 clouds dropped (as ndt counts).
            self.dropped += max(0, int(round((stamp - self.last_stamp) / PERIOD)) - 1)
        self.last_stamp = stamp
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        points = to_base(numpy.asarray(xyz, dtype=float).reshape(-1, 3), self.xyz, self.rpy)
        found = cubes(points, self.shape)
        header = Header(stamp=msg.header.stamp, frame_id='base_link')
        self.cubes_pub.publish(point_cloud2.create_cloud_xyz32(header, found.obstacles))
        self.ground_pub.publish(point_cloud2.create_cloud_xyz32(header, found.ground))
        ms = (time.perf_counter() - start) * 1000.0
        road = found.road
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if road.fitted else DiagnosticStatus.WARN,
            name='voxels',
            message='road fitted' if road.fitted else 'road taken as flat (no good fit)')
        status.values = [KeyValue(key=k, value=v) for k, v in (
            ('ms', f'{ms:.2f}'), ('cubes', str(len(found.obstacles))),
            ('ground_cubes', str(len(found.ground))), ('points', str(found.points)),
            ('fitted', str(road.fitted).lower()),
            ('roll_deg', f'{numpy.degrees(road.roll):.2f}'),
            ('pitch_deg', f'{numpy.degrees(road.pitch):.2f}'),
            ('clouds_dropped', str(self.dropped)))]
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = Voxels()
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
