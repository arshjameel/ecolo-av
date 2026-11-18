"""
sim_esp32: today's ESP32 (the 2025-26 C++ firmware) as a stand-in, in Ravensim.

Only with sim.launch.py esp32:=true. legacy_uart_bridge, the car's own, connects with
port socket://127.0.0.1:<port> instead of the ESP32's serial line; this node takes its
frames through the firmware's logic (esp32_firmware: the ramp, the brake, no timeout) and
hands the outputs to sim_adapter's actuator model (esp32:=true), as the wires do:

  legacy_uart_bridge ──TCP, 13-byte frames──> sim_esp32 ──> sim_adapter ──> Ravensim

Listens     TCP 127.0.0.1:port, one connection at a time: the bridge's bytes
Publishes   vehicle/sim/esp32_outputs  aav_interfaces/SpineOutputs, 50 Hz: the
                              throttle (DAC / 700), brake (the Jrk's travel), direction
                              and steering target the firmware drives now

Parameters  port (47000), max_steering_angle (rad), steering_sign: the bridge's, to read
            back the angle it meant.
"""

import socket

from aav_interfaces.msg import SpineOutputs
from aav_sim_models.esp32_firmware import Esp32Firmware
from aav_sim_models.relay import run
from rclpy.node import Node


class SimEsp32(Node):

    def __init__(self):
        super().__init__('sim_esp32')
        port = self.declare_parameter('port', 47000).value
        self.firmware = Esp32Firmware(
            self.declare_parameter('max_steering_angle', 0.5).value,
            self.declare_parameter('steering_sign', -1.0).value)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', port))
        self.server.listen(1)
        self.server.setblocking(False)
        self.client = None
        self.pub = self.create_publisher(SpineOutputs, 'vehicle/sim/esp32_outputs', 10)
        self.create_timer(0.01, self.read)
        self.create_timer(0.02, self.publish)
        self.get_logger().info(f'the C++ ESP32 firmware, standing in: listening on '
                               f'127.0.0.1:{port} for legacy_uart_bridge')

    def read(self):
        if self.client is None:
            try:
                self.client, _ = self.server.accept()
                self.client.setblocking(False)
                self.get_logger().info('legacy_uart_bridge connected')
            except BlockingIOError:
                return
        try:
            data = self.client.recv(4096)
        except BlockingIOError:
            return
        except OSError:
            data = b''
        if not data:
            # The bridge went away. Like the C++, keep the last outputs: no timeout.
            self.get_logger().warning('legacy_uart_bridge disconnected; the outputs stay '
                                      'as they were (the C++ has no command timeout)')
            self.client.close()
            self.client = None
            return
        self.firmware.feed(data)

    def publish(self):
        out = self.firmware.outputs()
        msg = SpineOutputs(throttle=float(out.throttle), brake=float(out.brake),
                           reverse=out.reverse, steering=float(out.steering))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.pub.publish(msg)

    def destroy_node(self):
        for s in (self.client, self.server):
            if s is not None:
                s.close()
        super().destroy_node()


def main(args=None):
    run(SimEsp32, args)


if __name__ == '__main__':
    main()
