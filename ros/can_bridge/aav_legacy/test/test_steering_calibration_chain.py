"""
The steering calibration on ROS (1 Nov): the panel's request in, version 3 frames out.

The real legacy_uart_bridge (frame_version 3, uplink true), on a private ROS domain on this
computer, its serial port a pseudo-terminal. This test plays the ESP32 (uplink frames with
the steering actuator's counts) and the hardware panel (/drivers/buttons/steering_calibration
at 10 Hz).

  steering_calibration stands, the panel asks for limp -> frames with limp, brake full,
                                                          no speed; the raw counts and a
                                                          LIMP line come back
  the panel stops asking                               -> frames without it within 0.5 s
  steering_calibration none (the default)               -> never limp; the line says why
"""

import fcntl
import os
import random
import time

import pytest

try:
    from aav_interfaces.msg import ActuatorRaw, SteeringCalibration
    from aav_legacy.legacy_protocol import (CAL_LIMP, CAL_MODE_LIMP, CAL_MODE_OFF,
                                            FRAME_V3_LEN, MODE_STOPPED, pack_uplink,
                                            SWITCH_COMPUTER, SYNC, unpack_frame_v3, Uplink)
    from aav_legacy.legacy_uart_bridge import LegacyUartBridge
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='needs ROS 2 and the built workspace')


def uplink(sequence, counts, calibration):
    return Uplink(sequence=sequence & 0xFF, echo=None, mode=MODE_STOPPED,
                  switch=SWITCH_COMPUTER, armed=False, estop=False, faults=0, steering=0.0,
                  brake_position=1.0, throttle_out=0.0, brake_out=1.0, rc_throttle=0.0,
                  rc_steering=0.0, rc_brake=False, steering_counts=counts, brake_counts=3500,
                  calibration=calibration)


class Run:
    """The bridge on a pseudo-terminal, and a test node playing the panel."""

    def __init__(self, where='stands'):
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        self.master, self.slave = os.openpty()
        port = os.ttyname(self.slave)
        fcntl.fcntl(self.master, fcntl.F_SETFL,
                    fcntl.fcntl(self.master, fcntl.F_GETFL) | os.O_NONBLOCK)
        args = ['--ros-args', '-p', f'legacy_uart_bridge:port:={port}',
                '-p', 'legacy_uart_bridge:frame_version:=3',
                '-p', 'legacy_uart_bridge:uplink:=true',
                '-p', f'legacy_uart_bridge:steering_calibration:={where}']
        rclpy.init(args=args, domain_id=random.randint(150, 230))
        self.executor = SingleThreadedExecutor()
        self.bridge = LegacyUartBridge()
        self.test = rclpy.create_node('steering_calibration_test')
        self.ask = self.test.create_publisher(SteeringCalibration,
                                              '/drivers/buttons/steering_calibration', 10)
        self.raw, self.lines, self.frames = [], [], []
        self.test.create_subscription(ActuatorRaw, '/vehicle/steering_actuator/raw',
                                      lambda m: self.raw.append(m), 100)
        self.test.create_subscription(String, '/vehicle/steering_actuator/calibration',
                                      lambda m: self.lines.append(m.data), 100)
        for n in (self.bridge, self.test):
            self.executor.add_node(n)
        self.buf = bytearray()
        self.sequence = 0
        self.esp32_state = 0

    def spin(self, seconds, mode=None):
        """Spin; the ESP32 sends an uplink every 20 ms, the panel `mode` every 100 ms."""
        end = time.monotonic() + seconds
        next_up = next_ask = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_up:
                os.write(self.master, pack_uplink(uplink(self.sequence, 2100,
                                                         self.esp32_state)))
                self.sequence += 1
                next_up += 0.02
            if mode is not None and now >= next_ask:
                self.ask.publish(SteeringCalibration(mode=mode, target=0))
                next_ask += 0.1
            self.executor.spin_once(timeout_sec=0.005)
            self.read_frames()

    def read_frames(self):
        try:
            while True:
                chunk = os.read(self.master, 4096)
                if not chunk:
                    break
                self.buf += chunk
        except BlockingIOError:
            pass
        while len(self.buf) >= FRAME_V3_LEN:
            if self.buf[0] != SYNC:
                del self.buf[0]
                continue
            frame = unpack_frame_v3(bytes(self.buf[:FRAME_V3_LEN]))
            # The ESP32 would say limp once it saw limp frames.
            self.esp32_state = CAL_LIMP if frame.cal_mode == CAL_MODE_LIMP else 0
            self.frames.append((time.monotonic(), frame))
            del self.buf[:FRAME_V3_LEN]

    def since(self, t):
        return [f for at, f in self.frames if at >= t]

    def close(self):
        for n in (self.bridge, self.test):
            n.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        os.close(self.master)
        os.close(self.slave)


@pytest.fixture
def run(request):
    r = Run(**getattr(request, 'param', {}))
    yield r
    r.close()


def test_the_panels_limp_reaches_the_frames_and_ends_when_it_stops_asking(run):
    run.spin(1.0)                                     # discovery; uplinks only
    run.spin(1.0, mode=SteeringCalibration.MODE_LIMP)
    t = time.monotonic()
    run.spin(0.5, mode=SteeringCalibration.MODE_LIMP)
    frames = run.since(t)
    assert len(frames) > 15                           # 50 Hz
    assert all(f.cal_mode == CAL_MODE_LIMP for f in frames)
    assert all((f.speed, f.accel, f.armed) == (0.0, -1.0, False) for f in frames)
    assert run.raw and run.raw[-1].valid and run.raw[-1].position == 2100
    assert run.raw[-1].calibration == ActuatorRaw.CALIBRATION_LIMP
    assert any(line.startswith('LIMP') for line in run.lines)
    # The panel stops asking: within 0.5 s (and a tick or two) the mode is gone.
    t = time.monotonic()
    run.spin(1.0)
    assert all(f.cal_mode == CAL_MODE_OFF for f in run.since(t + 0.6))


@pytest.mark.parametrize('run', [{'where': 'none'}], indirect=True)
def test_with_steering_calibration_none_it_never_goes_limp(run):
    run.spin(1.0)
    run.spin(1.5, mode=SteeringCalibration.MODE_LIMP)
    assert run.frames and all(f.cal_mode == CAL_MODE_OFF for _, f in run.frames)
    assert any(line.startswith('REFUSED') and 'hardware lead' in line for line in run.lines)
