"""
A launch value reaches the node: each launch started, its parameter read back.

rclrs lets a YAML's entry under the node's own name beat launch's `/**` dicts; every Rust
Node() with several parameter sources therefore has `namespace='/'` (docs/languages.md,
"Launch overrides"; tools/tests/test_rust_launch_params.py checks the source). This test
checks the running processes, for the safety numbers (legacy.launch.py in both
languages; ego and the muxes are Rust only since 22 Nov):

| Launch | Argument | The node's parameter |
|---|---|---|
| ego.launch.py | max_speed:=0.3 | /ego max_speed 0.3, max_reverse_speed 0.3 |
| legacy.launch.py | port:=socket://..., frame_version:=2| port, frame_version (YAML: ttyAMA0, 1) |
| legacy.launch.py | (vehicle.yaml) | /legacy_uart_bridge max_steering_angle, over legacy.yaml's |
| io_mux.launch.py | target:=bench | /output_mux target bench |

Each run: its own ROS_DOMAIN_ID (200-219) and localhost discovery; stopped by its process
group.
"""

import os
import signal
import socket
import subprocess
import time

import pytest

try:
    from aav_description import vehicle
    import rclpy
    from rclpy.parameter import parameter_value_to_python
    from rcl_interfaces.srv import GetParameters
    HAVE_ROS = True
except ImportError:  # pragma: no cover - only without ROS
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='needs ROS 2 (pixi run test)')


def environment():
    """Give this run its own domain, on this computer only."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(200 + os.getpid() % 20),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return env


class Launched:
    """`ros2 launch package file args...`, in its own process group."""

    def __init__(self, package, launch_file, *args):
        self.process = subprocess.Popen(
            ['ros2', 'launch', package, launch_file, *args], env=environment(),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def stop(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()


@pytest.fixture
def ros():
    environment()
    rclpy.init()
    node = rclpy.create_node('rust_overrides_test')
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def params(node, target, names, timeout=30.0):
    """Return {name: value} from `target`'s get_parameters, waiting for it to come up."""
    client = node.create_client(GetParameters, f'{target}/get_parameters')
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.5):
                future = client.call_async(GetParameters.Request(names=list(names)))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                if future.result() is not None:
                    return {n: parameter_value_to_python(v)
                            for n, v in zip(names, future.result().values)}
        raise AssertionError(f'{target} never answered get_parameters in {timeout:g} s')
    finally:
        node.destroy_client(client)


# Both languages: the Python twins must keep their values too.
IMPLS = pytest.mark.parametrize('impl', ['rust', 'python'])
# ego and the muxes: their Python nodes went on 22 Nov.
RUST = pytest.mark.parametrize('impl', ['rust'])


@RUST
def test_ego_max_speed_reaches_ego(ros, impl):
    run = Launched('aav_ego', 'ego.launch.py', f'ego_impl:={impl}', 'look_ahead:=false',
                   'max_speed:=0.3')
    try:
        got = params(ros, '/ego', ['max_speed', 'max_reverse_speed'])
    finally:
        run.stop()
    assert got == {'max_speed': 0.3, 'max_reverse_speed': 0.3}


@IMPLS
def test_legacy_port_frame_and_steering_reach_the_bridge(ros, impl):
    # A pretend ESP32 to connect to: the bridge opens its port at start-up.
    server = socket.socket()
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    port = f'socket://127.0.0.1:{server.getsockname()[1]}'
    run = Launched('aav_legacy', 'legacy.launch.py', f'legacy_impl:={impl}', 'encoder:=false',
                   'rc:=false', f'port:={port}', 'frame_version:=2')
    try:
        got = params(ros, '/legacy_uart_bridge',
                     ['port', 'frame_version', 'max_steering_angle'])
    finally:
        run.stop()
        server.close()
    steering = float(vehicle.load()['vehicle']['max_steering_angle'])
    assert got == {'port': port, 'frame_version': 2, 'max_steering_angle': steering}


@RUST
def test_output_mux_target_reaches_the_mux(ros, impl):
    run = Launched('aav_io_mux', 'io_mux.launch.py', f'io_mux_impl:={impl}', 'target:=bench',
                   'source:=bench')
    try:
        got = params(ros, '/output_mux', ['target'])
    finally:
        run.stop()
    assert got == {'target': 'bench'}
