"""
The amcl node itself, with stock's amcl.yaml, on a made-up map: scans in, a pose out.

It starts the built node on its own ROS domain, on this computer only, with a walled
yard and two boxes on /map (latched), a start 0.5 m and 6 degrees off, the odometry
on /localization/odom/local and scans from the truth on /sensors/lidar/scan, as the car
drives 6 m. The last pose on /localization/amcl/pose must be within 0.15 m and 2 degrees
of the truth at its stamp (its scan's), and particles come on
/localization/amcl/particles. Needs the workspace built and sourced: `colcon test` runs it.
"""

import math
import os
import pathlib
import signal
import subprocess
import time

import numpy as np
import pytest

try:
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav2_msgs.msg import ParticleCloud
    from nav_msgs.msg import OccupancyGrid, Odometry
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

# Stock's settings, read from the source tree (aav_navigation depends on this package).
CONFIG = (pathlib.Path(__file__).resolve().parents[3]
          / 'drivers' / 'aav_navigation' / 'config' / 'amcl.yaml')
RES, ORIGIN = 0.1, (-20.0, -15.0)
START_TRUTH = (-6.0, 0.0, 0.0)
GUESS = (-5.6, -0.3, math.radians(6.0))


def made_up_yard():
    """Return a 40 x 30 m grid (0 free, 100 occupied): walls round it and two boxes."""
    grid = np.zeros((300, 400), np.int8)
    grid[:2, :] = grid[-2:, :] = grid[:, :2] = grid[:, -2:] = 100

    def box(x, y, w, h):
        i0, j0 = int((x - ORIGIN[0]) / RES), int((y - ORIGIN[1]) / RES)
        grid[j0:j0 + int(h / RES), i0:i0 + int(w / RES)] = 100

    box(8.0, 4.0, 4.5, 1.8)
    box(-7.0, -6.0, 1.8, 4.5)
    box(-12.0, 6.0, 3.0, 3.0)
    return grid


def scan_from(grid, pose):
    """Return 720 ranges from `pose` (base_link), to the first occupied cell, or inf."""
    x, y, yaw = pose
    angles = -math.pi + np.arange(720) * (2 * math.pi / 720)
    steps = np.arange(1, 1200) * (RES / 4)
    px = x + np.outer(np.cos(yaw + angles), steps)
    py = y + np.outer(np.sin(yaw + angles), steps)
    i = ((px - ORIGIN[0]) / RES).astype(int).clip(0, grid.shape[1] - 1)
    j = ((py - ORIGIN[1]) / RES).astype(int).clip(0, grid.shape[0] - 1)
    hit = grid[j, i] == 100
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    return [float(steps[k]) if k >= 0 else math.inf for k in first]


def stamp(msg, t):
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def test_scans_give_a_pose_near_the_truth():
    assert CONFIG.exists(), CONFIG
    env = dict(os.environ, ROS_DOMAIN_ID=str(150 + os.getpid() % 20),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    binary = os.path.join(get_package_prefix('aav_amcl_rs'), 'lib', 'aav_amcl_rs', 'amcl')
    node = subprocess.Popen(
        [binary, '--ros-args', '-r', '__node:=amcl', '--params-file', str(CONFIG),
         '-p', 'set_initial_pose:=true', '-p', f'initial_pose.x:={GUESS[0]}',
         '-p', f'initial_pose.y:={GUESS[1]}', '-p', f'initial_pose.yaw:={GUESS[2]}',
         '-p', 'seed:=7',
         '-r', 'amcl_pose:=/localization/amcl/pose',
         '-r', 'particle_cloud:=/localization/amcl/particles',
         '-r', 'initialpose:=/localization/amcl/initial_pose'],
        env=env, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('amcl_node_test')
        poses, clouds = [], []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        me.create_subscription(PoseWithCovarianceStamped, '/localization/amcl/pose',
                               poses.append, latched)
        me.create_subscription(ParticleCloud, '/localization/amcl/particles',
                               clouds.append, rclpy.qos.qos_profile_sensor_data)
        grid = made_up_yard()
        map_out = me.create_publisher(OccupancyGrid, '/map', latched)
        odom_out = me.create_publisher(Odometry, '/localization/odom/local', 10)
        scan_out = me.create_publisher(LaserScan, '/sensors/lidar/scan', 10)
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution = RES
        m.info.height, m.info.width = grid.shape
        m.info.origin.position.x, m.info.origin.position.y = ORIGIN
        m.info.origin.orientation.w = 1.0
        m.data = grid.flatten().tolist()
        map_out.publish(m)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(me, timeout_sec=0.02)

        spin(2.0)   # discovery, and the map
        # Drive 6 m east at 0.3 m a step; the odometry starts at its own origin.
        truths = {}   # by stamp (s): AMCL publishes after every second update
        for k in range(21):
            if node.poll() is not None:
                break
            truth = (START_TRUTH[0] + 0.3 * k, START_TRUTH[1], START_TRUTH[2])
            t = 100.0 + 0.5 * k
            truths[t] = truth
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            stamp(odom, t)
            odom.pose.pose.position.x = 0.3 * k
            odom.pose.pose.orientation.w = 1.0
            odom_out.publish(odom)
            scan = LaserScan()
            scan.header.frame_id = 'base_link'
            stamp(scan, t)
            scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 720
            scan.angle_max = scan.angle_min + 719 * scan.angle_increment
            scan.range_min, scan.range_max = 0.2, 30.0
            scan.ranges = scan_from(grid, truth)
            spin(0.1)
            scan_out.publish(scan)
            spin(0.3)
        spin(0.5)
        log = node.stdout.read().decode() if node.poll() is not None else ''
        assert poses, f'no pose; {len(clouds)} particle clouds; {log}'
        assert clouds and len(clouds[-1].particles) >= 500
        pose = poses[-1]
        p = pose.pose.pose
        truth = truths[pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9]
        err = math.hypot(p.position.x - truth[0], p.position.y - truth[1])
        assert err < 0.15, (err, p.position, truth)
        assert abs(math.degrees(yaw_of(p.orientation) - truth[2])) < 2.0
        assert pose.header.frame_id == 'map'
        assert pose.pose.covariance[0] > 0.0 and pose.pose.covariance[35] > 0.0
        me.destroy_node()
    finally:
        rclpy.try_shutdown()
        if node.poll() is None:
            os.killpg(node.pid, signal.SIGINT)
            try:
                node.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(node.pid, signal.SIGKILL)
