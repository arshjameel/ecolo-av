"""
The no-timeout lock on ROS: legacy_uart_bridge says it, the real twist_mux obeys it.

legacy_uart_bridge on a pseudo-terminal, then twist_mux (the stock node, with
aav_command_mux's twist_mux.yaml) started a second later, as on the car (the bridge on the
Motor Pi, the mux on the Orin), all on a private ROS domain on this computer. Each driver
sends its own speed, so /control/cmd_vel shows who got through.

  frame_version 1                 -> scripted (50) and Nav2 (20) held; the joystick (100) and
                                     the actuator tests (60) still drive
  frame_version 1, allow_programs_without_timeout -> scripted drives
  frame_version 2                 -> scripted drives
"""

import fcntl
import os
import random
import signal
import subprocess
import time

import pytest

try:
    from aav_legacy.legacy_uart_bridge import LegacyUartBridge
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import TwistStamped
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='needs ROS 2 and the built workspace')

# Each driver's speed, so the mux's output says who it passed on.
SPEED = {'joy': 0.1, 'tests': 0.2, 'scripted': 0.3, 'nav': 0.5}


class Run:
    """The bridge in this process, twist_mux in its own process group, and a test node."""

    def __init__(self, version, allow=False):
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        self.domain = random.randint(150, 230)
        self.master, self.slave = os.openpty()
        fcntl.fcntl(self.master, fcntl.F_SETFL,
                    fcntl.fcntl(self.master, fcntl.F_GETFL) | os.O_NONBLOCK)
        args = ['--ros-args', '-p', f'legacy_uart_bridge:port:={os.ttyname(self.slave)}',
                '-p', f'legacy_uart_bridge:frame_version:={version}',
                '-p', f'legacy_uart_bridge:allow_programs_without_timeout:={allow}']
        rclpy.init(args=args, domain_id=self.domain)
        self.executor = SingleThreadedExecutor()
        self.bridge = LegacyUartBridge()
        self.test = rclpy.create_node('no_timeout_lock_test')
        self.pubs = {name: self.test.create_publisher(TwistStamped, f'/drivers/{name}/cmd_vel',
                                                      10) for name in SPEED}
        self.out = []
        self.test.create_subscription(TwistStamped, '/control/cmd_vel',
                                      lambda m: self.out.append(m.twist.linear.x), 100)
        for n in (self.bridge, self.test):
            self.executor.add_node(n)
        self.mux = None

    def start_mux(self):
        """Start twist_mux with the car's config; no watchdog or geofence here: their locks off."""
        config = os.path.join(get_package_share_directory('aav_command_mux'), 'config')
        exe = os.path.join(get_package_prefix('twist_mux'), 'lib', 'twist_mux', 'twist_mux')
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
            try:
                os.read(self.master, 4096)    # keep the pseudo-terminal from filling
            except BlockingIOError:
                pass
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
        for n in (self.bridge, self.test):
            n.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        os.close(self.master)
        os.close(self.slave)


def started(version, allow=False):
    r = Run(version, allow)
    r.spin(1.0)          # the bridge's latched lock goes out before the mux exists
    r.start_mux()
    # The joystick shows the mux is up: then a silence below is the lock, not a slow start.
    assert r.until_mux_passes('joy'), 'twist_mux never passed the joystick'
    r.spin(0.7)          # the joystick times out (0.5 s)
    return r


@pytest.fixture
def run(request):
    r = started(**request.param)
    yield r
    r.close()


@pytest.mark.parametrize('run', [{'version': 1}], indirect=True)
def test_no_timeout_holds_every_program_but_not_people_or_the_tests(run):
    run.spin(0.5, 'scripted', 'nav')                  # the lock arrives at 5 Hz
    assert run.spin(1.5, 'scripted', 'nav') == set()  # held: nothing comes out
    assert run.spin(1.0, 'tests', 'scripted') == {SPEED['tests']}
    assert run.spin(1.0, 'joy', 'scripted') == {SPEED['joy']}


@pytest.mark.parametrize('run', [{'version': 1, 'allow': True}, {'version': 2}],
                         indirect=True)
def test_with_a_timeout_or_the_override_programs_drive(run):
    assert SPEED['scripted'] in run.spin(1.5, 'scripted')
