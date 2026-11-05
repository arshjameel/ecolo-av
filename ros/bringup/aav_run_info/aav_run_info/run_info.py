"""
run_info: which algorithms this run uses, as the launch chose them, for the screens.

Subscribes  nothing
Publishes   system/algorithms   diagnostic_msgs/DiagnosticStatus, reliable, transient local
                                (latched), depth 1: name algorithms, level OK, message one
                                short line (lattice, mppi, voxels...), hardware_id the run
                                (sim world:=road, car), values one row each - run, planner,
                                path follower, obstacles, local EKF, global EKF, NDT,
                                KISS-ICP, AMCL. Again every period seconds, so a screen
                                that joins late sees it at once.
Serves      nothing
Calls       nothing

Parameters  profile, nav, planner, controller, tracker, lidar, radar, voxels, amcl,
            ndt, ndt_ekf, kiss_icp, kiss_ekf: the launch's own choices, already checked
            (sim.launch.py, car.launch.py fill them); period (2.0 s).

It says what the launch asked for, not what is alive: the cockpit's tracker panel may swap
the tracker later in Gazebo (/drivers/nav/track/status says which then). The words are in
algorithms.py (no ROS).
"""

import signal

from aav_run_info.algorithms import Choices, rows, summary, use
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class RunInfo(Node):

    def __init__(self):
        super().__init__('run_info')

        def param(name, default):
            return self.declare_parameter(name, default).value

        flags = {n: bool(param(n, False))
                 for n in ('lidar', 'radar', 'voxels', 'amcl', 'ndt', 'ndt_ekf', 'kiss_icp',
                           'kiss_ekf')}
        choices = Choices(
            profile=str(param('profile', 'sim')), nav=bool(param('nav', True)),
            planner=str(param('planner', 'lattice')),
            controller=str(param('controller', 'mppi')),
            tracker=str(param('tracker', 'none')),
            lidar=flags['lidar'], radar=flags['radar'], voxels=flags['voxels'],
            amcl=flags['amcl'], ndt=use(flags['ndt'], flags['ndt_ekf']),
            kiss_icp=use(flags['kiss_icp'], flags['kiss_ekf']))
        self.msg = DiagnosticStatus(
            level=DiagnosticStatus.OK, name='algorithms', message=summary(choices),
            hardware_id=choices.profile,
            values=[KeyValue(key=k, value=v) for k, v in rows(choices)])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(DiagnosticStatus, 'system/algorithms', latched)
        self.pub.publish(self.msg)
        self.create_timer(float(param('period', 2.0)), lambda: self.pub.publish(self.msg))
        self.get_logger().info(f'run_info: {self.msg.message}')


def main():
    rclpy.init()
    node = RunInfo()
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
