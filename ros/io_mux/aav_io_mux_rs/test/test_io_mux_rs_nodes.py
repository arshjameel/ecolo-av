"""
The Rust sensor mux and output mux processes themselves, for the real car (target car).

It starts both built binaries on their own ROS domain, on this computer only, with an
announcer saying REAL and ego's commands at 20 Hz, and plays:
  - the gate opens (OPEN) and each command goes on, with a heartbeat counting +1;
  - a simulator announces SIM: STOP, latched, no command and no heartbeat goes on, and
    the clear service refuses while it is alive;
  - the simulator goes: still STOP until a person clears; cleared, OPEN again;
then a clean exit on Ctrl-C. Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
import signal
import subprocess
import time

import pytest

try:
    from aav_interfaces.msg import DataSource, Heartbeat, SensorSource
    from ackermann_msgs.msg import AckermannDriveStamped
    from ament_index_python.packages import get_package_prefix
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')


def env():
    ros_domain = os.environ.get('AAV_TEST_DOMAIN') or str(200 + os.getpid() % 20)
    e = dict(os.environ, ROS_DOMAIN_ID=ros_domain,
             ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: e[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return e


def binary(name):
    return os.path.join(get_package_prefix('aav_io_mux_rs'), 'lib', 'aav_io_mux_rs', name)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def announce(node, name, kind):
    pub = node.create_publisher(DataSource, '/system/source', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    msg = DataSource(announcer=name)
    msg.sensors = [SensorSource(name='imu', topic='/sensors/imu/data_raw', source=kind)]
    pub.publish(msg)
    return pub


def test_open_heartbeat_a_sim_latch_and_a_clear():
    e = env()
    muxes = [
        subprocess.Popen([binary('sensor_mux'), '--ros-args', '-p', 'source:=real',
                          '-p', 'startup:=0.0'], env=e, start_new_session=True),
        subprocess.Popen([binary('output_mux'), '--ros-args', '-p', 'target:=car',
                          '-p', 'settle:=0.5'], env=e, start_new_session=True),
    ]
    rclpy.init()
    try:
        car = rclpy.create_node('io_mux_rs_car')
        sim = rclpy.create_node('sim_adapter')
        seen = {'state': '', 'beats': [], 'commands': 0}
        car.create_subscription(String, '/safety/output_mux/state',
                                lambda m: seen.__setitem__('state', m.data), 10)
        car.create_subscription(Heartbeat, '/control/car/heartbeat',
                                lambda m: seen['beats'].append(m.counter), 50)

        def passed(_msg):
            seen['commands'] += 1

        car.create_subscription(AckermannDriveStamped, '/control/car/actuator_cmd', passed, 50)
        keep = [announce(car, 'io_mux_rs_car', 0)]
        cmd = car.create_publisher(AckermannDriveStamped, '/ego/actuator_cmd', 10)
        clear = car.create_client(Trigger, '/safety/output_mux/clear')

        def send():
            m = AckermannDriveStamped()
            m.header.stamp = car.get_clock().now().to_msg()
            m.drive.speed = 0.5
            cmd.publish(m)

        car.create_timer(0.05, send)

        def spin(seconds, test=lambda: False):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(car, timeout_sec=0.02)
                rclpy.spin_once(sim, timeout_sec=0.0)
                if test():
                    return True
            return False

        def call_clear():
            assert clear.wait_for_service(timeout_sec=5)
            future = clear.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(car, future, timeout_sec=5)
            assert future.result() is not None
            return future.result()

        assert spin(30, lambda: seen['state'].startswith('OPEN')), seen['state']
        assert seen['state'] == 'OPEN - car: every source REAL'
        seen['beats'].clear()
        seen['commands'] = 0
        spin(1.0)
        beats = seen['beats']
        assert len(beats) >= 10 and seen['commands'] >= 10, (len(beats), seen['commands'])
        assert all((b - a) % 256 == 1 for a, b in zip(beats, beats[1:])), beats

        keep.append(announce(sim, 'sim_adapter', 1))
        assert spin(10, lambda: seen['state'].startswith('STOP')), seen['state']
        assert 'imu is SIM (sim_adapter)' in seen['state'], seen['state']
        spin(0.3)
        seen['beats'].clear()
        seen['commands'] = 0
        spin(1.0)
        assert seen['beats'] == [] and seen['commands'] == 0
        answer = call_clear()
        assert not answer.success and answer.message.startswith('still wrong: '), answer

        sim.destroy_node()
        sim = rclpy.create_node('bystander')
        # Gone from the graph, but the stop stays latched until a person clears it.
        spin(2.0)
        assert seen['state'].startswith('STOP'), seen['state']
        assert spin(10, lambda: call_clear().success), seen['state']
        assert spin(10, lambda: seen['state'].startswith('OPEN')), seen['state']
        car.destroy_node()
        sim.destroy_node()
    finally:
        rclpy.shutdown()
        for m in muxes:
            stop(m)
    assert [m.returncode for m in muxes] in ([0, 0], [-signal.SIGINT] * 2), \
        [m.returncode for m in muxes]
