"""
The RC transmitter as a ROS driver (TODO 48): rc_driver, /vehicle/rc -> /drivers/rc/cmd_vel.

  ros2 launch aav_rc rc.launch.py [max_speed:=1.0] [use_sim_time:=false] [rc_impl:=rust]

It drives only while the transmitter's mode switch is on RC via ROS, and /vehicle/rc only
comes with legacy_uart_bridge's uplink on (aav_legacy legacy.launch.py starts this beside
it). The reverse speed, the steering limit and the wheelbase come from
aav_description/config/vehicle.yaml. rc_impl: rust (aav_rc_rs, the default since 17 Nov)
or python (this package's own; the same node name, topic and parameters).
"""

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def rc_params(cfg):
    """Return rc_driver's numbers from the whole of vehicle.yaml."""
    v = cfg['vehicle']
    return {'max_reverse_speed': float(v['max_reverse_speed']),
            'max_steering_angle': float(v['max_steering_angle']),
            'wheelbase': float(v['wheelbase'])}


def driver(context):
    """Return rc_driver, in the language rc_impl picks."""
    impl = vehicle.check_rc_impl(context.launch_configurations.get(
        'rc_impl', vehicle.DEFAULT_RC_IMPL))
    return [Node(
        package=vehicle.RC_IMPLS[impl],
        executable='rc_driver',
        name='rc_driver', namespace='/',
        parameters=[rc_params(vehicle.load()),
                    {'max_speed': ParameterValue(LaunchConfiguration('max_speed'),
                                                 value_type=float),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_speed', default_value='1.0',
                              description='m/s at full stick forward'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rc_impl', default_value=vehicle.DEFAULT_RC_IMPL,
                              description='rc_driver in rust (aav_rc_rs, the default) or '
                                          'python (aav_rc)'),
        OpaqueFunction(function=driver),
    ])
