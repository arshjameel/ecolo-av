"""
The joystick's whole chain, with each joy_alive: a pretend pad drives ego, and stops.

    pretend pad -> /drivers/joy/joy -> joy_alive -> teleop_twist_joy -> twist_mux -> ego

It starts teleop.launch.py (joy_device:=false: this test plays the pad), the car's
twist_mux (no watchdog or geofence here: their locks off) and ego, on their own ROS domain,
on this computer only. For joy_alive_impl rust (the Python went on 20 Nov): a live hand
drives ego forward
(OK); the same values frozen stop it (FROZEN) and keep it stopped while the trigger stays
held; released and pressed again it drives; then the pad goes silent: ego stops (SILENT).
Needs the workspace built and sourced: `colcon test` runs it.
"""

import os
import signal
import subprocess
import time

import pytest

try:
    from ackermann_msgs.msg import AckermannDriveStamped
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    import rclpy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String
except ImportError:
    rclpy = None

pytestmark = pytest.mark.skipif(rclpy is None, reason='needs ROS sourced: colcon test')

HELD = [1, 0, 0, 0, 0, 0, 0, 0]
RELEASED = [0] * 8


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


def start(env, cmd):
    return subprocess.Popen(cmd, env=env, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


@pytest.mark.parametrize('impl', ['rust'])
def test_a_pretend_pad_drives_ego_then_frozen_and_silent_stop_it(impl):
    domain = 200 + (os.getpid() + (impl == 'python')) % 20
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    # This process too: a test node on another discovery range never meets them.
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    config = os.path.join(get_package_share_directory('aav_command_mux'), 'config')
    mux = os.path.join(get_package_prefix('twist_mux'), 'lib', 'twist_mux', 'twist_mux')
    procs = [
        start(env, ['ros2', 'launch', 'aav_teleop', 'teleop.launch.py', 'joy_device:=false',
                    'estop_button:=false', f'joy_alive_impl:={impl}']),
        start(env, [mux, '--ros-args',
                    '--params-file', os.path.join(config, 'twist_mux.yaml'),
                    '--params-file', os.path.join(config, 'no_sensor_lock.yaml'),
                    '--params-file', os.path.join(config, 'no_geofence_lock.yaml'),
                    '-r', 'cmd_vel_out:=/control/cmd_vel']),
        start(env, ['ros2', 'launch', 'aav_ego', 'ego.launch.py', 'look_ahead:=false']),
    ]
    rclpy.init(domain_id=domain)
    try:
        ros = rclpy.create_node('joy_chain_test')
        seen = {'speed': 0.0, 'alive': ''}
        ros.create_subscription(AckermannDriveStamped, '/ego/actuator_cmd',
                                lambda m: seen.__setitem__('speed', m.drive.speed), 10)
        ros.create_subscription(String, '/drivers/joy/alive',
                                lambda m: seen.__setitem__('alive', m.data), 10)
        pad = ros.create_publisher(Joy, '/drivers/joy/joy', 10)
        play = {'stick': 0.0, 'buttons': RELEASED, 'jitter': True, 'on': True, 'i': 0}

        def send():
            if not play['on']:
                return
            play['i'] += 1
            wobble = 0.004 * ((play['i'] * 7) % 5 - 2) if play['jitter'] else 0.0
            m = Joy()
            m.header.stamp = ros.get_clock().now().to_msg()
            m.axes = [0.0, play['stick'] + wobble, 0.0, 0.0, 0.0, 0.0]
            m.buttons = list(play['buttons'])
            pad.publish(m)

        ros.create_timer(0.05, send)

        def wait_for(test, seconds, what):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(ros, timeout_sec=0.02)
                if test():
                    return time.monotonic()
            raise AssertionError(f'{impl}: {what}: ego {seen["speed"]:.2f} m/s, '
                                 f'joy_alive "{seen["alive"]}"')

        # Released once (WAITING lets go), then a live hand pushes forward: ego drives.
        wait_for(lambda: seen['alive'].startswith('OK'), 40, 'OK after a release')
        play.update(stick=0.5, buttons=HELD)
        wait_for(lambda: seen['speed'] > 0.1, 15, 'a live hand drives')
        assert seen['alive'] == 'OK - driving'

        # Frozen while driving (a dead pad's receiver): FROZEN, and ego stops.
        play.update(jitter=False)
        frozen = time.monotonic()
        tripped = wait_for(lambda: seen['alive'].startswith('FROZEN'), 6, 'FROZEN')
        assert 1.8 < tripped - frozen < 3.5
        wait_for(lambda: abs(seen['speed']) < 1e-3, 5, 'ego stops when frozen')
        # The pad comes back to life with the trigger still held: still stopped.
        play.update(jitter=True)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(ros, timeout_sec=0.02)
        assert seen['alive'].startswith('FROZEN') and abs(seen['speed']) < 1e-3
        # Released, pressed again: it drives.
        play.update(buttons=RELEASED, stick=0.0)
        wait_for(lambda: seen['alive'].startswith('OK'), 3, 'OK after the release')
        play.update(buttons=HELD, stick=0.5)
        wait_for(lambda: seen['speed'] > 0.1, 10, 'drives again')

        # The receiver unplugged while driving: SILENT, and ego stops.
        play.update(on=False)
        wait_for(lambda: seen['alive'].startswith('SILENT'), 3, 'SILENT')
        wait_for(lambda: abs(seen['speed']) < 1e-3, 5, 'ego stops when silent')
        ros.destroy_node()
    finally:
        rclpy.shutdown()
        for proc in procs:
            stop(proc)
