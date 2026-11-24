"""
The Rust ego process itself: a command in, ego's and look_ahead's topics out.

It starts the built binary on its own ROS domain, on this computer only, with both nodes'
parameters in one file (as ego.launch.py ego_impl:=rust writes it), drives it with
/control/cmd_vel, and checks what the Python nodes would send: the Ackermann command
speeding up toward the request and steering by the bicycle model, then braking at
max_deceleration when the command stops; a 21-pose trace and a stop point in base_link;
look_ahead's status with the Python node's keys. Then a clean exit on Ctrl-C.
Needs the workspace built and sourced: `colcon test` runs it.
"""

import math
import os
import signal
import subprocess
import tempfile
import time

import pytest

try:
    from ackermann_msgs.msg import AckermannDriveStamped
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Path
    import rclpy
    import yaml
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

KEYS = ['gap_m', 'plan', 'stop_m', 'goal_m', 'obstacle_m', 'room_m', 'front_m', 'rear_m',
        'half_width_m', 'road_mps', 'road_m', 'road_why']
WHEELBASE = 1.561


def env():
    ros_domain = os.environ.get('AAV_TEST_DOMAIN') or str(180 + os.getpid() % 30)
    e = dict(os.environ, ROS_DOMAIN_ID=ros_domain,
             ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: e[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return e


def binary():
    return os.path.join(get_package_prefix('aav_ego_rs'), 'lib', 'aav_ego_rs', 'ego')


def params(look):
    vehicle = os.path.join(get_package_share_directory('aav_description'), 'config',
                           'vehicle.yaml')
    ego = {'vehicle_yaml': vehicle, 'wheelbase': WHEELBASE, 'max_jerk': 2.0,
           'lag_lead': 'off', 'pedal_drivers': ['joy']}
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        yaml.safe_dump({'/ego': {'ros__parameters': ego},
                        '/look_ahead': {'ros__parameters': look}}, f)
    return f.name


def stop(node):
    if node.poll() is None:
        os.killpg(node.pid, signal.SIGINT)
        try:
            node.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            node.wait()


def test_a_command_drives_and_both_nodes_answer():
    node = subprocess.Popen(
        [binary(), '--ros-args', '--params-file',
         params({'body': [2.0, -0.5, 0.6]})],
        env=env(), start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        me = rclpy.create_node('ego_rs_node_test')
        got = {'cmd': [], 'trace': [], 'stop': [], 'ahead': []}
        me.create_subscription(AckermannDriveStamped, '/ego/actuator_cmd',
                               got['cmd'].append, 10)
        me.create_subscription(Path, '/ego/trace', got['trace'].append, 10)
        me.create_subscription(PoseStamped, '/ego/stop_point', got['stop'].append, 10)
        me.create_subscription(DiagnosticStatus, '/ego/ahead', got['ahead'].append, 10)
        out = me.create_publisher(TwistStamped, '/control/cmd_vel', 10)

        def spin(seconds, send=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end and node.poll() is None:
                if send is not None:
                    msg = TwistStamped()
                    msg.header.stamp = me.get_clock().now().to_msg()
                    msg.twist.linear.x, msg.twist.angular.z = send
                    out.publish(msg)
                rclpy.spin_once(me, timeout_sec=0.05)

        # Wait for the process, then 2.5 s of (0.5 m/s, 0.1 rad/s).
        end = time.monotonic() + 30.0
        while not got['cmd'] and time.monotonic() < end and node.poll() is None:
            rclpy.spin_once(me, timeout_sec=0.1)
        assert got['cmd'], 'nothing back: ' + (
            node.stdout.read().decode() if node.poll() is not None else 'no actuator_cmd')
        got['cmd'].clear()
        spin(2.5, send=(0.5, 0.1))
        driving = got['cmd'][-1]
        assert driving.header.frame_id == 'base_link'
        assert driving.drive.speed == pytest.approx(0.5, abs=0.01)
        assert driving.drive.steering_angle == pytest.approx(
            math.atan(WHEELBASE * 0.1 / 0.5), abs=0.01)
        speeds = [m.drive.speed for m in got['cmd']]
        assert all(b >= a - 1e-6 for a, b in zip(speeds, speeds[1:])), 'speeds up only'

        trace = got['trace'][-1]
        assert trace.header.frame_id == 'base_link' and len(trace.poses) == 21
        assert trace.poses[-1].pose.position.x > 0.5 and trace.poses[-1].pose.position.y > 0.0
        stop_point = got['stop'][-1]
        assert stop_point.header.frame_id == 'base_link'
        assert 0.0 < stop_point.pose.position.x < 1.0

        ahead = got['ahead'][-1]
        values = {kv.key: kv.value for kv in ahead.values}
        assert [kv.key for kv in ahead.values] == KEYS
        assert (ahead.name, ahead.hardware_id) == ('look_ahead', 'aav_ego')
        assert ahead.level == DiagnosticStatus.OK
        assert values['plan'] == 'none' and values['gap_m'] == 'nan'
        assert (values['front_m'], values['rear_m'], values['half_width_m']) == (
            '2.00', '-0.50', '0.60')
        assert ahead.message.startswith('NO PLAN - stops in ')

        # Silence: stop at max_deceleration (2 m/s^2), the wheels held.
        got['cmd'].clear()
        spin(1.0)
        assert got['cmd'][-1].drive.speed == 0.0
        assert got['cmd'][-1].drive.steering_angle == pytest.approx(
            driving.drive.steering_angle, abs=1e-6)
        me.destroy_node()
    finally:
        rclpy.try_shutdown()
        stop(node)
    assert node.returncode == 0, 'the process did not stop cleanly on Ctrl-C'


def test_a_missing_body_stops_look_ahead():
    no_body = params({'rate_hz': 10.0})
    done = subprocess.run([binary(), '--ros-args', '--params-file', no_body], env=env(),
                          capture_output=True, timeout=30, start_new_session=True)
    assert done.returncode != 0
    assert b'body' in done.stdout + done.stderr


def test_ego_only_needs_no_body():
    no_body = params({'rate_hz': 10.0})
    node = subprocess.Popen([binary(), '--ego-only', '--ros-args', '--params-file', no_body],
                            env=env(), start_new_session=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)
    try:
        time.sleep(3.0)
        assert node.poll() is None, node.stdout.read().decode()
    finally:
        stop(node)
    assert node.returncode == 0
