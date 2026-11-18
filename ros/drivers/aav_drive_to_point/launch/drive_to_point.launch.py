"""
The point driver: an action server (/drive_to_point) that drives to one point and stops there.

  ros2 launch aav_drive_to_point drive_to_point.launch.py [use_sim_time:=false] [watchdog:=true]
      [stop_law:=approach|hybrid] [drive_to_point_impl:=rust]
  pixi run drive-to-point --sim --ahead 20          (the client: tools/drive_to_point.sh)

drive_to_point_impl: rust (aav_drive_to_point_rs, the default since 18 Nov: the same node
name, action, topics, parameters and answers) or python (this package's own node).

The braking (actuators max_brake), the wheelbase and the steering limit come from
aav_description/config/vehicle.yaml; the rest from config/drive_to_point.yaml.
watchdog:=false where no sensor watchdog runs (sim.launch.py watchdog:=false): it then
doesn't wait for its lock.
"""

import os

from aav_description import vehicle
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def point_params(cfg):
    """Return the point driver's numbers from the whole of vehicle.yaml."""
    v = cfg['vehicle']
    return {'max_brake': float(cfg['actuators']['max_brake']),
            'wheelbase': float(v['wheelbase']),
            'max_steering_angle': float(v['max_steering_angle'])}


def launch_setup(context):
    with open(os.path.join(get_package_share_directory('aav_drive_to_point'), 'config',
                           'drive_to_point.yaml')) as f:
        config = yaml.safe_load(f)['point_driver']['ros__parameters']
    watchdog = LaunchConfiguration('watchdog').perform(context).lower()
    if watchdog not in ('true', 'false'):
        raise ValueError(f'watchdog:={watchdog}: use true or false')
    if watchdog == 'false':
        config['watchdog_timeout'] = 0.0
    law = LaunchConfiguration('stop_law').perform(context).strip()
    if law:
        config['stop_law'] = law
    impl = vehicle.check_drive_to_point_impl(context.launch_configurations.get(
        'drive_to_point_impl', vehicle.DEFAULT_DRIVE_TO_POINT_IMPL))
    return [Node(
        package=vehicle.DRIVE_TO_POINT_IMPLS[impl],
        executable='point_driver',
        name='point_driver', namespace='/',
        # drive_to_point.yaml as a dict, not the file: a named entry in a file beats a global
        # parameter set later (sim.launch.py's use_sim_time), as in ego.launch.py.
        parameters=[dict(config), point_params(vehicle.load()),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('watchdog', default_value='true',
                              description="false where no sensor watchdog runs: don't wait "
                                          'for its lock'),
        DeclareLaunchArgument('drive_to_point_impl',
                              default_value=vehicle.DEFAULT_DRIVE_TO_POINT_IMPL,
                              description='rust (aav_drive_to_point_rs, the default) or python '
                                          '(this package): which point_driver runs'),
        DeclareLaunchArgument('stop_law', default_value='',
                              description='approach (the default, drive_to_point.yaml) or '
                                          'hybrid: the stopping law, a trial'),
        OpaqueFunction(function=launch_setup),
    ])
