"""
The Rust behaviour process itself: the locks, odometry, a plan and a scan in.

Out: the cap, the state and the road ahead.

It starts the built binary on its own ROS domain, on this computer only, with the road
world's lane map, and plays a short drive: a goal along a straight plan (DRIVE, no cap, the
road ahead on the lane's centre with its edges), a thing in the lane 6 m ahead (APPROACH,
the cap falls, the road ahead ends at the obstacle's stop), then the stop zone's lock with
the car standing (WAIT, the floor). Then a clean exit on Ctrl-C.
Needs the workspace built and sourced: `colcon test` runs it.
"""

import math
import os
import signal
import subprocess
import time

import pytest

try:
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from aav_interfaces.msg import RoadAhead
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.msg import SpeedLimit
    from nav_msgs.msg import Odometry, Path
    import rclpy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

FLOOR, BASE = 0.3, 1.5


def env():
    e = dict(os.environ, ROS_DOMAIN_ID=str(150 + os.getpid() % 30),
             ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: e[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return e


def binary():
    return os.path.join(get_package_prefix('aav_behaviour_rs'), 'lib', 'aav_behaviour_rs',
                        'behaviour')


def lane_map():
    # aav_sim_models's road world: one lane along y = 0. Its source copy, if not installed.
    try:
        share = get_package_share_directory('aav_sim_models')
        path = os.path.join(share, 'worlds', 'road.lanes.yaml')
        if os.path.exists(path):
            return path
    except Exception:  # noqa: BLE001 - not built: fall back to the source tree
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, '..', '..', '..', 'sim', 'aav_sim_models', 'worlds',
                        'road.lanes.yaml')


def stop(node):
    if node.poll() is None:
        os.killpg(node.pid, signal.SIGINT)
        try:
            node.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            node.wait()


def test_a_drive_a_thing_ahead_and_a_lock():
    vehicle = os.path.join(get_package_share_directory('aav_description'), 'config',
                           'vehicle.yaml')
    node = subprocess.Popen(
        [binary(), '--ros-args', '-p', f'vehicle_yaml:={vehicle}', '-p', f'base_speed:={BASE}',
         '-p', f'lane_map:={lane_map()}', '-p', 'front:=2.0', '-p', 'half_width:=0.6',
         '-p', 'stop_gap:=1.0'],
        env=env(), start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('behaviour_rs_node_test')
        got = {'limit': [], 'state': [], 'road': []}
        me.create_subscription(SpeedLimit, '/drivers/nav/speed_limit', got['limit'].append, 10)
        me.create_subscription(DiagnosticStatus, '/drivers/nav/behaviour/state',
                               got['state'].append, 10)
        me.create_subscription(RoadAhead, '/world/road_ahead', got['road'].append, 10)
        local = me.create_publisher(Odometry, '/localization/odom/local', 10)
        glob = me.create_publisher(Odometry, '/localization/odom/global', 10)
        plan = me.create_publisher(Path, '/plan', 10)
        status = me.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status', 10)
        scan = me.create_publisher(LaserScan, '/sensors/lidar/scan',
                                   rclpy.qos.qos_profile_sensor_data)
        zone = me.create_publisher(Bool, '/safety/stop_hold/lock', 10)

        path = Path()
        path.header.frame_id = 'map'
        for i in range(61):
            p = PoseStamped()
            p.pose.position.x = 0.5 * i
            path.poses.append(p)
        running = GoalStatusArray()
        running.status_list.append(GoalStatus(status=GoalStatus.STATUS_EXECUTING))

        def send(speed, obstacle=None, locked=False):
            for pub in (local, glob):
                o = Odometry()
                o.pose.pose.orientation.w = 1.0
                o.twist.twist.linear.x = speed
                pub.publish(o)
            plan.publish(path)
            status.publish(running)
            s = LaserScan(angle_min=-0.5, angle_max=0.5, angle_increment=0.25, range_max=40.0)
            s.ranges = [math.inf] * 5
            if obstacle is not None:
                s.ranges[2] = obstacle
            scan.publish(s)
            zone.publish(Bool(data=locked))

        def run(seconds, **kw):
            for key in got:
                got[key].clear()
            end = time.monotonic() + seconds
            while time.monotonic() < end and node.poll() is None:
                send(**kw)
                rclpy.spin_once(me, timeout_sec=0.05)

        def state():
            return {v.key: v.value for v in got['state'][-1].values}

        # Wait for the process, then drive along the plan.
        end = time.monotonic() + 30.0
        while not got['limit'] and time.monotonic() < end and node.poll() is None:
            send(1.0)
            rclpy.spin_once(me, timeout_sec=0.1)
        assert got['limit'], 'nothing back: ' + (
            node.stdout.read().decode() if node.poll() is not None else 'no speed_limit')
        run(2.0, speed=1.0)
        assert state()['state'] == 'DRIVE' and got['limit'][-1].speed_limit == 0.0
        assert not got['limit'][-1].percentage
        road = got['road'][-1]
        assert road.header.frame_id == 'map' and len(road.x) == 61  # 30 m, every 0.5 m
        assert road.end == pytest.approx(30.0, abs=1e-4)          # the goal: /plan's end
        assert [s.reason for s in road.stops] == ['goal'] and road.stops[0].kind == 1
        assert all(abs(y) < 1e-6 for y in road.y)                  # the lane's centre
        assert road.left_hard[0] == pytest.approx(2.25)            # the road world's kerb

        # A thing 6 m ahead: 6 - 2 (bumper) - 1 (stop gap) = 3 m left.
        run(2.0, speed=1.0, obstacle=6.0)
        assert state()['state'] == 'APPROACH' and state()['reason'] == 'obstacle'
        cap = got['limit'][-1].speed_limit
        assert FLOOR <= cap < BASE
        assert got['road'][-1].end == pytest.approx(3.0, abs=1e-4)
        assert got['road'][-1].stops[0].reason == 'obstacle'

        # The stop zone's lock, the car standing: WAIT at the floor.
        run(1.5, speed=0.0, obstacle=2.5, locked=True)
        assert state()['state'] == 'WAIT'
        assert got['limit'][-1].speed_limit == pytest.approx(FLOOR)

        stop(node)
        assert node.returncode == 0, node.stdout.read().decode()
    finally:
        stop(node)
        rclpy.try_shutdown()
