import os
import random
import signal
import subprocess
import tempfile
import time

import pytest

try:
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import TwistStamped
    import rclpy
    from std_msgs.msg import Bool
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
        # command_mux_state (aav_mux_state): who it says is driving, beside twist_mux.
        self.said = []
        self.test.create_subscription(
            DiagnosticStatus, '/control/command_mux/state',
            lambda m: self.said.append({kv.key: kv.value for kv in m.values}), 100)
        self.stop_sign = self.test.create_publisher(Bool, '/perception/stop_sign', 10)
        self.executor.add_node(self.test)
        self.mux = None
        self.state = None

    def start_mux(self):
        """
        Start twist_mux with the car's config; no watchdog or geofence here: their locks off.

        And command_mux_state beside it, given the same files in the same order.
        """
        config = os.path.join(get_package_share_directory(
            'aav_command_mux'), 'config')
        files = [os.path.join(config, f) for f in
                 ('twist_mux.yaml', 'no_sensor_lock.yaml', 'no_geofence_lock.yaml')]
        exe = os.path.join(get_package_prefix('twist_mux'),
                           'lib', 'twist_mux', 'twist_mux')
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain))
        params = [a for f in files for a in ('--params-file', f)]
        self.mux = subprocess.Popen(
            [exe, '--ros-args', *params, '-r', 'cmd_vel_out:=/control/cmd_vel'],
            env=env, start_new_session=True, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        state = os.path.join(get_package_prefix('aav_mux_state'),
                             'lib', 'aav_mux_state', 'command_mux_state')
        # Its log goes to a file, for the failure messages.
        self.state_log = tempfile.TemporaryFile(mode='w+')
        self.state = subprocess.Popen(
            [state, '--ros-args', '-p', f'config_files:=[{",".join(files)}]'],
            env=env, start_new_session=True, stdout=self.state_log,
            stderr=subprocess.STDOUT)

    def last_said(self):
        """Return what command_mux_state said last ({} if nothing yet)."""
        return self.said[-1] if self.said else {}

    def until_said(self, driver, *sending, seconds=3.0):
        """Spin, the named drivers sending, until command_mux_state names `driver`."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.spin(0.1, *sending)
            if self.last_said().get('driver') == driver:
                return True
        return False

    def state_says(self):
        """Return the state's last words and its log, for a failure message."""
        self.state_log.seek(0)
        alive = 'running' if self.state.poll() is None else f'exited {self.state.returncode}'
        return f'said {self.last_said()}; command_mux_state {alive}: {self.state_log.read()}'

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
        for proc in (self.mux, self.state):
            if proc is None or proc.poll() is not None:
                continue
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
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
        # Wait until it comes through (on a loaded machine the first commands take a
        # moment), then check that it alone does.
        assert run.until_mux_passes(driver, 5.0), f'{driver} never reached the mux output'
        output = run.spin(0.3, driver)

        # Nothing but its own commands (that they arrive is the wait's check above).
        assert output <= {speed}, (
            f'{driver} did not reach the mux output; '
            f'expected {speed}, actual {output}'
        )
        # command_mux_state names the same driver twist_mux passed.
        assert run.until_said(driver, driver), f'twist_mux passed {driver}; {run.state_says()}'
    run.spin(0.6)
    # All start at once: the first message to land passes for an instant (a race), so
    # let every one be heard first, then the joystick must win.
    run.spin(0.3, *SPEED)
    output = run.spin(1.0, *SPEED)

    assert output == {SPEED['joy']}


def test_silent_source_times_out(run):
    # The keyboard first, until it comes through; then both: the keyboard must win.
    # (Both starting at once after a silence is a race: the first message to land passes.)
    assert run.until_mux_passes('keyboard', 5.0), 'the keyboard never reached the output'
    output = run.spin(0.5, 'keyboard', 'scripted')

    assert output == {SPEED['keyboard']}

    output = run.spin(1.0, 'scripted')

    assert output == {SPEED['scripted']}


def test_the_state_names_who_twist_mux_passes_and_who_a_lock_holds(run):
    # Navigation alone: twist_mux passes it, and the state says nav.
    assert run.until_mux_passes('nav', 5.0), 'navigation never reached the output'
    assert run.until_said('nav', 'nav'), run.state_says()
    # A stop sign (lock 30) holds navigation (20): nothing passes, the state says why.
    run.stop_sign.publish(Bool(data=True))
    run.spin(0.3, 'nav')
    output = run.spin(0.5, 'nav')
    assert output == set(), f'the stop sign should hold navigation, got {output}'
    assert run.until_said('none', 'nav'), run.state_says()
    said = run.last_said()
    assert said.get('held') == 'nav', said
    assert said.get('locks') == 'stop_sign 30', said
    # The keyboard (90) is above the lock: it drives, and the state says so.
    assert run.until_mux_passes('keyboard', 5.0)
    assert run.until_said('keyboard', 'keyboard', 'nav'), run.state_says()
    assert run.last_said().get('held') == 'nav', run.state_says()
    run.stop_sign.publish(Bool(data=False))
