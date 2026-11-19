"""
What Ravensim's sensor nodes share: a fault on purpose, and a clean start and stop.

Each of Ravensim's sensors has a node of its own (sim_imu, sim_gps, sim_radar,
sim_camera_relay), as each sensor on the car has its own driver: one process each, so each
runs on its own core, and a slow one (the radar's cloud) can't make another late (Ben,
22 Oct). sim_adapter keeps the car itself: the actuator model, odometry, the wheels.

Each node takes the parameter fault.<its sensor> (faults.py), set while it runs:

    pixi run sim-fault imu stop      (ros2 param set /sim_imu fault.imu stop)
"""

import signal

from aav_sim_models import faults
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter


class SensorRelay(Node):
    """A node that passes one of Ravensim's sensors on, through that sensor's fault."""

    def __init__(self, name, sensor):
        super().__init__(name)
        self.sensor = sensor
        self.fault = faults.Fault(sensor)
        self.declare_parameter(f'fault.{sensor}', 'ok')
        self.add_on_set_parameters_callback(self.on_fault)

    def on_fault(self, params):
        for p in params:
            if p.name != f'fault.{self.sensor}':
                continue
            mode = p.value if p.type_ == Parameter.Type.STRING else repr(p.value)
            why = faults.check(self.sensor, mode)
            if why is not None:
                return SetParametersResult(successful=False, reason=why)
            self.fault.switch(mode)
            # One call per severity: rclpy refuses a call site whose severity changes.
            if mode == 'ok':
                self.get_logger().info(f'fault cleared: {self.sensor} ok')
            else:
                self.get_logger().warning(f'FAULT ON PURPOSE: {self.sensor} {mode}')
        return SetParametersResult(successful=True)

    def faulty(self, msg, nan=None, jump=None, drift=None, now=None):
        """Return msg through the fault (None: send nothing), stamped as msg."""
        out = self.fault.apply(msg, nan=nan, jump=jump, drift=drift, now=now)
        if out is not None and out is not msg:
            out.header.stamp = msg.header.stamp     # a frozen sensor still looks alive
        return out


def run(node_class, args=None):
    """Start one node and spin it until Ctrl-C or launch stops it."""
    rclpy.init(args=args)
    node = node_class()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C reaches a launched node twice (terminal and launch), and Jazzy can then
        # raise RCLError ("context is not valid"). Only a live context means a real error.
        if rclpy.ok():
            raise
    finally:
        # Ctrl-C reaches a launched node twice (the terminal, then launch): ignore the
        # second, so it can't interrupt the clean-up with a traceback.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
