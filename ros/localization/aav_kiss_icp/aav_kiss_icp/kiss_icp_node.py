"""
kiss_icp: lidar odometry by KISS-ICP (MIT, PRBonn), a trial. Never TF.

How far the car moved, from the lidar alone: each scan matched to a small map of the last
few. docs/designs/2026-10-30_kiss-icp.md. Only ekf_local reads it, and only its
twist, with kiss_ekf:=true (aav_localization config/ekf_local_kiss.yaml).

Subscribes  sensors/lidar/points       sensor_msgs/PointCloud2, lidar_link (reliable,
                                       keep last 1: a scan that arrives while one is being
                                       matched replaces the one waiting)
Publishes   localization/kiss_icp/odom    nav_msgs/Odometry: frame kiss_odom (its own,
                                          where base_link was at the first scan), child
                                          base_link; pose, and twist in base_link from the
                                          last step; stamped with the cloud's stamp.
                                          KISS-ICP gives no covariance: the twist's is
                                          twist_linear_sd, twist_angular_sd (measured in
                                          Gazebo, 1 Nov), the pose's a fixed 1
            localization/kiss_icp/status  diagnostic_msgs/DiagnosticStatus after each
                                          scan: OK (TRACKING), or ERROR (no kiss
                                          environment, a cloud without x, y, z, the worker);
                                          values match_ms, points, deskew, clouds,
                                          clouds_dropped, restarts

Parameters  kiss_python (the kiss environment's Python: '' = <repo>/.pixi/envs/kiss),
            lidar_xyz, lidar_rpy (the mount, from vehicle.yaml by the launch file),
            body_min, body_max (the car's outline in base_link, x y), min_height
            (m above base_link: lower is the ground, cut out), min_range,
            max_range, voxel_size (0 = max_range / 100, KISS-ICP's rule),
            max_points_per_voxel, threads, deskew, lidar_period, twist_linear_sd (m/s),
            twist_angular_sd (rad/s). config/kiss_icp.yaml.

KISS-ICP runs in worker.py, in the kiss environment's Python (pixi install --locked -e
kiss), as a child program fed through a pipe (wire.py). The maths is in odometry.py.
"""

import os
import signal
import subprocess

from aav_kiss_icp import odometry, wire
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

FRAME = 'kiss_odom'
# KISS-ICP gives no covariance (K6). The pose's is a fixed guess: it drifts, use the
# twist. The twist's comes from the parameters (K11: measured against Gazebo's truth).
POSE_VARIANCE = 1.0          # m^2, rad^2
WORKER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'worker.py')
# Only what Python needs: none of the default environment's library paths reach the
# kiss environment's Python.
KEEP_ENV = ('HOME', 'PATH', 'LANG', 'LC_ALL', 'TMPDIR', 'USER')


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class KissIcp(Node):

    def __init__(self):
        super().__init__('kiss_icp')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('kiss_python', ''), ('lidar_xyz', [0.0, 0.0, 0.0]), ('lidar_rpy', [0.0, 0.0, 0.0]),
            ('body_min', [0.0, 0.0]), ('body_max', [0.0, 0.0]), ('min_height', 0.3),
            ('min_range', 0.5),
            ('max_range', 100.0), ('voxel_size', 0.0), ('max_points_per_voxel', 20),
            ('threads', 4), ('deskew', True), ('lidar_period', 0.1),
            ('twist_linear_sd', 0.15), ('twist_angular_sd', 0.01))}
        self.mount = ([float(v) for v in p['lidar_xyz']], [float(v) for v in p['lidar_rpy']])
        self.body = ([float(v) for v in p['body_min']], [float(v) for v in p['body_max']])
        self.ranges = (float(p['min_range']), float(p['max_range']))
        self.min_height = float(p['min_height'])
        self.deskew = bool(p['deskew'])
        self.period = float(p['lidar_period'])
        self.twist_variance = (float(p['twist_linear_sd']) ** 2,
                               float(p['twist_angular_sd']) ** 2)
        self.settings = {'min_range': 0.0, 'max_range': self.ranges[1],
                         'voxel_size': float(p['voxel_size']),
                         'max_points_per_voxel': int(p['max_points_per_voxel']),
                         'threads': int(p['threads']), 'deskew': self.deskew}
        self.python = odometry.find_python(p['kiss_python'])
        self.worker = None
        # Each worker starts at the identity: `offset` carries the pose across a restart.
        self.offset = np.eye(4)
        self.last_pose = self.last_raw = np.eye(4)
        self.last_stamp = None
        self.clouds = self.dropped = self.restarts = 0
        self.odom_pub = self.create_publisher(Odometry, 'localization/kiss_icp/odom', 10)
        self.status_pub = self.create_publisher(DiagnosticStatus,
                                                'localization/kiss_icp/status', 10)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, 'sensors/lidar/points', self.on_cloud, qos)
        self.start_worker()
        self.get_logger().info(
            f'kiss_icp: lidar odometry, the mount {self.mount} from base_link, '
            f'{self.ranges[0]:g}-{self.ranges[1]:g} m, above {self.min_height:g} m, '
            f'deskew {self.deskew}, worker {self.python}')

    def start_worker(self):
        if not os.path.isfile(self.python):
            self.worker = None
            self.get_logger().error(
                f'no kiss environment at {self.python}: run `pixi install --locked -e kiss` '
                '(Linux and Apple-silicon Macs only)')
            return False
        env = {k: os.environ[k] for k in KEEP_ENV if k in os.environ}
        self.worker = subprocess.Popen([self.python, '-I', WORKER], stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, env=env)
        self.worker.stdin.write(wire.encode_settings(self.settings))
        self.worker.stdin.flush()
        # The new worker's identity is where the last one left the car.
        self.offset, self.last_raw = self.last_pose, np.eye(4)
        return True

    def stop_worker(self):
        if self.worker is not None:
            try:
                self.worker.stdin.close()
                self.worker.wait(timeout=5.0)
            except (OSError, subprocess.TimeoutExpired):
                self.worker.kill()
            self.worker = None

    def on_cloud(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        try:
            points, times = odometry.cloud_points(
                [(f.name, f.offset, f.datatype) for f in msg.fields], bytes(msg.data),
                msg.point_step, msg.width * msg.height, msg.is_bigendian)
        except ValueError as e:
            self.status(DiagnosticStatus.ERROR, f'ERROR: {e}', {})
            return
        points = odometry.to_base_link(points, *self.mount)
        points, times = odometry.crop(points, times, *self.body, *self.ranges,
                                      origin=self.mount[0], min_height=self.min_height)
        times = odometry.sweep_times(times) if self.deskew else None
        if self.worker is None or self.worker.poll() is not None:
            if self.worker is not None:
                self.restarts += 1
                self.get_logger().error('the KISS-ICP worker died: starting it again')
            if not self.start_worker():
                self.status(DiagnosticStatus.ERROR,
                            'ERROR: no kiss environment (pixi install --locked -e kiss)', {})
                return
        try:
            self.worker.stdin.write(wire.encode_scan(points, times))
            self.worker.stdin.flush()
            raw, ms = wire.read_pose(self.worker.stdout)
        except (OSError, EOFError, ValueError) as e:
            self.get_logger().error(f'the KISS-ICP worker: {e}')
            self.status(DiagnosticStatus.ERROR, f'ERROR: the worker: {e}', {})
            return
        delta = np.linalg.inv(self.last_raw) @ raw
        self.last_raw = raw
        pose = self.offset @ raw
        dt = stamp - self.last_stamp if self.last_stamp is not None else 0.0
        self.dropped += odometry.dropped_between(self.last_stamp, stamp, self.period)
        self.last_stamp, self.last_pose = stamp, pose
        self.clouds += 1
        self.publish(msg.header.stamp, pose, odometry.twist_of(delta, dt))
        self.status(DiagnosticStatus.OK, 'TRACKING', {
            'match_ms': f'{ms:.2f}', 'points': str(len(points)),
            'deskew': str(times is not None).lower(), 'clouds': str(self.clouds),
            'clouds_dropped': str(self.dropped), 'restarts': str(self.restarts)})

    def publish(self, stamp, pose, twist):
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = FRAME
        out.child_frame_id = 'base_link'
        (x, y, z), (qx, qy, qz, qw) = odometry.pose_of(pose)
        p, q = out.pose.pose.position, out.pose.pose.orientation
        p.x, p.y, p.z = x, y, z
        q.x, q.y, q.z, q.w = qx, qy, qz, qw
        (vx, vy, vz), (wx, wy, wz) = twist
        v, w = out.twist.twist.linear, out.twist.twist.angular
        v.x, v.y, v.z = vx, vy, vz
        w.x, w.y, w.z = wx, wy, wz
        for i in range(6):
            out.pose.covariance[7 * i] = POSE_VARIANCE
            out.twist.covariance[7 * i] = self.twist_variance[0 if i < 3 else 1]
        self.odom_pub.publish(out)

    def status(self, level, message, values):
        s = DiagnosticStatus(level=level, name='kiss_icp', message=message,
                             hardware_id='lidar')
        s.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = KissIcp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C reaches a launched node twice (the terminal, then launch): ignore the
        # second, so it can't interrupt the clean-up with a traceback.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.stop_worker()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
