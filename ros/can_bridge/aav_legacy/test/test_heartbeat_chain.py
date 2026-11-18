"""
The heartbeat chain on ROS (TODO 50): the output mux makes the counter, the bridge relays it.

The real nodes, on a private ROS domain on this computer: sensor_mux (source real),
output_mux (target car) and legacy_uart_bridge (frame_version 2), whose serial port is a
pseudo-terminal this test reads, byte for byte what the ESP32 would get.

  all REAL, ego at 50 Hz       -> the counter moves in the frames, as the mux sent it
  ego stops                    -> frames still come at 50 Hz: stops, the counter frozen
  Gazebo announces SIM as well -> the gate shuts: not one heartbeat, the counter never moves

Each test runs twice (conftest's impl): the Python bridge and muxes in this process, and the
Rust ones (aav_legacy_rs, aav_io_mux_rs: the car's defaults since 17 Nov) as processes:
the Rust-to-Rust heartbeat chain.
"""

import fcntl
import os
import random
import time

from conftest import ros_nodes, RustNode
import pytest

try:
    from aav_interfaces.msg import DataSource, Heartbeat, SensorSource
    from aav_io_mux.output_mux import OutputMux
    from aav_io_mux.sensor_mux import SensorMux
    from aav_legacy.legacy_protocol import FRAME_V2_LEN, SYNC, unpack_frame_v2
    from aav_legacy.legacy_uart_bridge import LegacyUartBridge
    from ackermann_msgs.msg import AckermannDriveStamped
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='needs ROS 2 and the built workspace')

LATCHED = None if not HAVE_ROS else QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def frames_in(data):
    """Return the version 2 frames in a byte stream, and the bytes after the last one."""
    frames, at = [], 0
    while True:
        at = data.find(bytes([SYNC]), at)
        if at < 0 or len(data) - at < FRAME_V2_LEN:
            return frames, data[at:] if at >= 0 else b''
        try:
            frames.append(unpack_frame_v2(data[at:at + FRAME_V2_LEN]))
            at += FRAME_V2_LEN
        except ValueError:
            at += 1


class Run:
    """The muxes, the bridge on a pseudo-terminal, and a test node, spun together."""

    def __init__(self, gazebo=False, impl='python'):
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        self.master, slave = os.openpty()
        port = os.ttyname(slave)
        self.slave = slave
        fcntl.fcntl(self.master, fcntl.F_SETFL,
                    fcntl.fcntl(self.master, fcntl.F_GETFL) | os.O_NONBLOCK)
        args = ['--ros-args', '-p', 'sensor_mux:startup:=0.0', '-p', 'sensor_mux:source:=real',
                '-p', 'output_mux:target:=car', '-p', 'output_mux:settle:=0.5',
                '-p', f'legacy_uart_bridge:port:={port}',
                '-p', 'legacy_uart_bridge:frame_version:=2']
        domain = random.randint(150, 230)
        rclpy.init(args=args, domain_id=domain)
        self.executor = SingleThreadedExecutor()
        if impl == 'rust':
            self.nodes = [RustNode('aav_io_mux_rs', 'sensor_mux', args, domain),
                          RustNode('aav_io_mux_rs', 'output_mux', args, domain),
                          RustNode('aav_legacy_rs', 'legacy_uart_bridge', args, domain)]
        else:
            self.nodes = [SensorMux(), OutputMux(), LegacyUartBridge()]
        self.test = rclpy.create_node('heartbeat_test')
        self.nodes.append(self.test)
        self.heartbeats = []
        self.state = ''
        self.buffer = b''
        self.frames = []
        self.ego = self.test.create_publisher(AckermannDriveStamped, '/ego/actuator_cmd', 10)
        self.test.create_subscription(Heartbeat, '/control/car/heartbeat',
                                      lambda m: self.heartbeats.append(m.counter), 100)
        self.test.create_subscription(String, '/safety/output_mux/state', self.on_state, 10)
        if gazebo:
            self.gazebo = rclpy.create_node('sim_adapter')
            self.nodes.append(self.gazebo)
            pub = self.gazebo.create_publisher(DataSource, '/system/source', LATCHED)
            pub.publish(DataSource(announcer='sim_adapter', sensors=[SensorSource(
                name='camera_front_left', topic='/sensors/camera/front_left/image_raw',
                source=SensorSource.SIM)]))
        for n in ros_nodes(self.nodes):
            self.executor.add_node(n)

    def on_state(self, msg):
        self.state = msg.data

    def read_serial(self):
        try:
            while True:
                chunk = os.read(self.master, 4096)
                if not chunk:
                    break
                self.buffer += chunk
        except BlockingIOError:
            pass
        found, self.buffer = frames_in(self.buffer)
        self.frames.extend(found)

    def spin(self, seconds, drive=False):
        """
        Spin everything for this long, ego sending at 50 Hz if `drive`.

        Return the frames the bridge wrote meanwhile.
        """
        start = len(self.frames)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if drive:
                cmd = AckermannDriveStamped()
                cmd.drive.speed = 0.6
                self.ego.publish(cmd)
            self.executor.spin_once(timeout_sec=0.02)
            self.read_serial()
        return self.frames[start:]

    def close(self):
        for n in self.nodes:
            n.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        os.close(self.master)
        os.close(self.slave)


@pytest.fixture
def run(request, impl):
    r = Run(**getattr(request, 'param', {}), impl=impl)
    yield r
    r.close()


def changes(frames):
    return sum(a.counter != b.counter for a, b in zip(frames, frames[1:]))


def test_the_counter_comes_from_the_mux_and_the_bridge_only_relays_it(run):
    # The gate opens after 0.5 s of all clear. The Rust nodes are processes that must
    # start and find each other first (once 1.5 s wasn't enough under load, 18 Nov), so
    # wait for OPEN, up to 10 s, rather than a fixed time.
    deadline = time.monotonic() + 10.0
    while not run.state.startswith('OPEN') and time.monotonic() < deadline:
        run.spin(0.25, drive=True)
    assert run.state.startswith('OPEN'), run.state
    run.heartbeats.clear()
    driving = run.spin(2.0, drive=True)
    assert len(driving) > 60                # ~50 frames a second
    assert changes(driving) > 40            # it moves while the mux passes commands on
    assert all(f.armed and not f.estop for f in driving[5:])
    # Every counter in a frame is one the mux sent: the bridge makes none up.
    sent = set(run.heartbeats)
    assert {f.counter for f in driving[5:]} <= sent
    assert len(sent) > 60

    # Ego stops: the bridge keeps sending, stops now, and the counter stays put.
    run.spin(0.5)
    quiet = run.spin(1.0)
    # 50 a second on a laptop; GitHub's Mac (a slow VM, one executor for four nodes) wrote
    # 12 and 23 (25 Oct). What matters: it keeps sending, and the counter stays put.
    assert len(quiet) > 10
    assert changes(quiet) == 0
    assert all(not f.armed and f.accel == -1.0 and f.speed == 0.0 for f in quiet)


@pytest.mark.parametrize('run', [{'gazebo': True}], indirect=True)
def test_a_shut_gate_sends_no_heartbeat(run):
    frames = run.spin(3.0, drive=True)
    assert run.state.startswith('STOP'), run.state
    assert run.heartbeats == []
    assert len(frames) > 100
    assert {f.counter for f in frames} == {0}   # never heard one, never made one
    assert not any(f.armed for f in frames)
