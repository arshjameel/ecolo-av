"""
The sensor mux and the output mux: where the data comes from, and where the commands go.

  ros2 launch aav_io_mux io_mux.launch.py target:=car|sim|bench|turtle|carla
      [source:=real|sim|bench|replay] [io_mux_impl:=rust]

  target       who plays the car: the output mux sends ego's commands to
               /control/<target>/actuator_cmd. Empty: no output mux (a replay drives
               nothing)
  source       where the sensor data must come from; empty: the target's own (car: real;
               sim, turtle, carla: sim; bench: bench). A source the target can't take is
               refused here: the real car only ever runs on real data
  io_mux_impl  rust (aav_io_mux_rs, the default since Ben, 17 Nov: "Rust tomorrow"). This
               package's Python nodes went on 22 Nov (tag save/python-fallbacks-1); its
               rules (provenance.py, muxes.py) and this file stay. Both muxes switch together

Every profile launch file includes this with its own target (turtle, sim, bench, car,
carla), so `pixi run sim` and the rest start both muxes as before.
"""

from aav_description import vehicle
from aav_io_mux.provenance import choose
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    target, source = choose(LaunchConfiguration('target').perform(context).strip(),
                            LaunchConfiguration('source').perform(context).strip())
    package = vehicle.IO_MUX_IMPLS[vehicle.check_io_mux_impl(
        LaunchConfiguration('io_mux_impl').perform(context))]
    actions = [Node(package=package, executable='sensor_mux', name='sensor_mux',
                    parameters=[{'source': source}])]
    if target:
        actions.append(Node(package=package, executable='output_mux', name='output_mux',
                            parameters=[{'target': target,
                                         'screens': LaunchConfiguration('screens')
                                         .perform(context)}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target', default_value='',
                              description='car, sim, bench, turtle or carla: who plays the '
                                          'car (empty: no output mux)'),
        DeclareLaunchArgument('source', default_value='',
                              description="real, sim, bench or replay (empty: the target's)"),
        DeclareLaunchArgument('screens', default_value='',
                              description='screens that must be alive for commands to flow, '
                                          'such as dev_window,hw_panel: a test, never the car'),
        DeclareLaunchArgument('io_mux_impl', default_value=vehicle.DEFAULT_IO_MUX_IMPL,
                              description='rust (aav_io_mux_rs; the Python removed 22 Nov): '
                                          'which sensor mux and output mux run'),
        OpaqueFunction(function=launch_setup),
    ])
