"""
The uplink over socket:// keeps up (17 Nov rehearsal).

The bridge on `port: socket://127.0.0.1:<port>`, as Ravensim's sim_esp32 and the QEMU
rehearsal wire it, with this test as the ESP32: an uplink frame every 20 ms for 1.5 s. Every
frame must come out on /vehicle/rc. pyserial's in_waiting says 0 or 1 on socket://, and the
Python bridge once read only that much each 10 ms: it fell behind and the uplink went stale.

Each test runs twice (conftest's impl): the Python bridge in this process, the Rust as a process.
"""

import random
import socket
import time

from conftest import ros_nodes, RustNode
import pytest

try:
    from aav_interfaces.msg import RcSticks
    from aav_legacy.legacy_protocol import MODE_COMPUTER, pack_uplink, SWITCH_RC_VIA_ROS, Uplink
    from aav_legacy.legacy_uart_bridge import LegacyUartBridge
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='needs ROS 2 and the built workspace')


def frame(sequence):
    return pack_uplink(Uplink(sequence=sequence & 0xFF, echo=0, mode=MODE_COMPUTER,
                              switch=SWITCH_RC_VIA_ROS, armed=False, estop=False, faults=0,
                              steering=0.0, brake_position=1.0, throttle_out=0.0,
                              brake_out=1.0, rc_throttle=0.0, rc_steering=0.0,
                              rc_brake=False))


def test_every_uplink_frame_comes_through_on_socket(impl):
    server = socket.socket()
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    server.settimeout(0.0)
    port = server.getsockname()[1]
    args = ['--ros-args', '-p', f'legacy_uart_bridge:port:=socket://127.0.0.1:{port}',
            '-p', 'legacy_uart_bridge:frame_version:=2',
            '-p', 'legacy_uart_bridge:uplink:=true']
    domain = random.randint(150, 230)
    rclpy.init(args=args, domain_id=domain)
    executor = SingleThreadedExecutor()
    nodes = ([RustNode('aav_legacy_rs', 'legacy_uart_bridge', args, domain)]
             if impl == 'rust' else [LegacyUartBridge()])
    test = rclpy.create_node('uplink_socket_test')
    nodes.append(test)
    got = []
    test.create_subscription(RcSticks, '/vehicle/rc', lambda m: got.append(m), 200)
    for n in ros_nodes(nodes):
        executor.add_node(n)
    esp = None
    try:
        deadline = time.monotonic() + 10.0
        while esp is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            try:
                esp, _ = server.accept()
            except BlockingIOError:
                pass
        assert esp is not None, 'the bridge never connected'
        esp.setblocking(False)
        # Warm up (discovery, the Rust process), then count one stretch exactly.
        sent, end, next_frame = 0, time.monotonic() + 1.0, time.monotonic()
        while time.monotonic() < end:
            if time.monotonic() >= next_frame:
                esp.sendall(frame(sent))
                sent += 1
                next_frame += 0.02
            executor.spin_once(timeout_sec=0.002)
            try:
                esp.recv(65536)                  # the bridge's frames: drained, not judged
            except BlockingIOError:
                pass
        got.clear()
        counted, end = 0, time.monotonic() + 1.5
        while time.monotonic() < end:
            if time.monotonic() >= next_frame:
                esp.sendall(frame(sent))
                sent += 1
                counted += 1
                next_frame += 0.02
            executor.spin_once(timeout_sec=0.002)
            try:
                esp.recv(65536)
            except BlockingIOError:
                pass
        end = time.monotonic() + 0.3                 # the last ones arrive
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert counted >= 70
        assert len(got) >= counted - 3, f'{len(got)} of {counted} uplink frames came out'
    finally:
        if esp is not None:
            esp.close()
        server.close()
        for n in nodes:
            n.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
