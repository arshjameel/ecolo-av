"""
ego: /control/cmd_vel (TwistStamped from twist_mux) -> /ego/actuator_cmd (AckermannDriveStamped).

  ros2 launch aav_ego ego.launch.py [use_sim_time:=false] [max_speed:=0.3] [look_ahead:=true]
                                [road_ahead:=false]

road_ahead (false): true holds Nav2's commands to /world/road_ahead's speed limit
(ego_node.py, road_limit.py); sim.launch.py sets it with behaviour:=true, its writer.

look_ahead (true) also starts look_ahead: the squiggly line's numbers for the screens
(/ego/plan_ahead, /ego/ahead; look_ahead_node.py). It only listens and publishes those two.

max_speed (m/s) caps the forward speed below config/ego.yaml's, for a first drive on
stands; empty (the default) keeps ego.yaml's. It can only lower the cap, never raise it.

The wheelbase, the steering limit, the reverse speed and the body (mass, track, centre of
mass, tyre grip, the throttle's and brake's accelerations) come from
aav_description/config/vehicle.yaml; the other limits and the payload from config/ego.yaml.
"""

import os

from aav_description import vehicle
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

with open(os.path.join(get_package_share_directory('aav_ego'), 'config', 'ego.yaml')) as f:
    CONFIG = yaml.safe_load(f)['ego']['ros__parameters']
CONFIG_MAX_SPEED = float(CONFIG['max_speed'])
SIM_MAX_SPEED = 5.0  # m/s: sim_max_speed's ceiling, about 18 km/h


def ego_node(context):
    car = vehicle.load()
    params = [
        # ego.yaml as a dict, not the file: the file names the node (ego:), and a named
        # entry beats the later caps below whenever the launch also sets a global
        # parameter (sim.launch.py's use_sim_time). As dicts, the last one wins.
        dict(CONFIG),
        vehicle.ego_params(car),
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    cap = context.launch_configurations.get('max_speed', '').strip()
    if cap:
        limit = float(cap)
        if not 0.0 < limit <= CONFIG_MAX_SPEED:
            raise ValueError(f'max_speed:={cap} must be above 0 and at most '
                             f'{CONFIG_MAX_SPEED} m/s (config/ego.yaml)')
        params.append({'max_speed': limit})
    sim = context.launch_configurations.get('sim_max_speed', '').strip()
    if sim:
        # Above ego.yaml's cap, so only ever in a simulator: refused on wall-clock time.
        if LaunchConfiguration('use_sim_time').perform(context).lower() != 'true':
            raise ValueError('sim_max_speed:= is for simulators only (use_sim_time:=true)')
        limit = float(sim)
        if not 0.0 < limit <= SIM_MAX_SPEED:
            raise ValueError(f'sim_max_speed:={sim} must be above 0 and at most '
                             f'{SIM_MAX_SPEED} m/s')
        params.append({'max_speed': limit})
    if context.launch_configurations.get('road_ahead', 'false').strip().lower() == 'true':
        params.append({'road_ahead': True})
    nodes = [Node(package='aav_ego', executable='ego', name='ego', parameters=params)]
    if context.launch_configurations.get('look_ahead', 'true').strip().lower() == 'true':
        nodes.append(Node(package='aav_ego', executable='look_ahead', name='look_ahead',
                          parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('max_speed', default_value='',
                              description="m/s: a lower forward speed cap than ego.yaml's"),
        DeclareLaunchArgument('sim_max_speed', default_value='',
                              description='m/s: a higher cap, in a simulator only '
                                          '(sim.launch.py sim_speed:=)'),
        DeclareLaunchArgument('road_ahead', default_value='false',
                              description='true: Nav2 never faster than /world/road_ahead '
                                          'allows (needs the behaviour planner)'),
        DeclareLaunchArgument('look_ahead', default_value='true',
                              description='also start look_ahead (the squiggly line)'),
        OpaqueFunction(function=ego_node),
    ])
