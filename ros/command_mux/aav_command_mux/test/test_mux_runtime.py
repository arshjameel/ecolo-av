import os
import random
import signal
import subprocess
import time

import pytest

try:
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import TwistStamped
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(
    not HAVE_ROS, reason='needs ROS 2 and the built workspace')

SPEED = {
    'joy': 0.10,
    'rc': 0.20,
    'keyboard': 0.30,
    'tests': 0.40,
    'scripted': 0.50,
    'buttons': 0.60,
    'replay': 0.70,
    'point': 0.80,
    'nav': 0.90,
    'ai': 1.00,
}


class Run:
    """A test ROS node and a real twist_mux process."""

    def __init__(self):
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        self.domain = random.randint(150, 230)

        rclpy.init(domain_id=self.domain)

        self.executor = SingleThreadedExecutor()
        self.test = rclpy.create_node('mux_runtime_test')
        self.pubs = {name: self.test.create_publisher(TwistStamped, f'/drivers/{name}/cmd_vel',
                                                      10) for name in SPEED}
        self.out = []
        self.test.create_subscription(TwistStamped, '/control/cmd_vel',
                                      lambda m: self.out.append(m.twist.linear.x), 100)
        self.executor.add_node(self.test)
        self.mux = None

    def start_mux(self):
        """Start twist_mux with the car's config; no watchdog or geofence here: their locks off."""
        config = os.path.join(get_package_share_directory(
            'aav_command_mux'), 'config')
        exe = os.path.join(get_package_prefix('twist_mux'),
                           'lib', 'twist_mux', 'twist_mux')
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain))
        self.mux = subprocess.Popen(
            [exe, '--ros-args', '--params-file', os.path.join(config, 'twist_mux.yaml'),
             '--params-file', os.path.join(config, 'no_sensor_lock.yaml'),
             '--params-file', os.path.join(config, 'no_geofence_lock.yaml'),
             '-r', 'cmd_vel_out:=/control/cmd_vel'],
            env=env, start_new_session=True, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)

    def spin(self, seconds, *drivers):
        """Spin this long, the named drivers each sending at 20 Hz; return what came out."""
        start = len(self.out)
        end, next_send = time.monotonic() + seconds, 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                next_send = time.monotonic() + 0.05
                for name in drivers:
                    msg = TwistStamped()
                    msg.header.stamp = self.test.get_clock().now().to_msg()
                    msg.twist.linear.x = SPEED[name]
                    self.pubs[name].publish(msg)
            self.executor.spin_once(timeout_sec=0.01)
        return {round(v, 2) for v in self.out[start:]}

    def until_mux_passes(self, driver, seconds=15.0):
        """Wait until twist_mux is up and passes this driver's commands."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if SPEED[driver] in self.spin(0.3, driver):
                return True
        return False

    def close(self):
        if self.mux is not None:
            os.killpg(self.mux.pid, signal.SIGINT)
            try:
                self.mux.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(self.mux.pid, signal.SIGKILL)
                self.mux.wait()
        self.test.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()


@pytest.fixture
def run():
    r = Run()

    try:
        r.start_mux()
        assert r.until_mux_passes(
            'joy'), 'twist_mux never passed the joystick'
        r.spin(0.7)
        yield r
    finally:
        r.close()


def test_joystick_overrides_every_source(run):
    for driver, speed in SPEED.items():
        run.spin(0.6)
        output = run.spin(0.3, driver)

        assert output == {speed}, (
            f'{driver} did not reach the mux output; '
            f'expected {speed}, actual {output}'
        )
    run.spin(0.6)
    output = run.spin(1.0, *SPEED)

    assert output == {SPEED['joy']}


def test_silent_source_times_out(run):
    output = run.spin(0.5, 'keyboard', 'scripted')

    assert output == {SPEED['keyboard']}

    output = run.spin(1.0, 'scripted')

    assert output == {SPEED['scripted']}
