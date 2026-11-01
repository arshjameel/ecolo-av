"""
The geofence: stops every program before the car's stopping distance reaches the fence.

  ros2 launch aav_safety geofence.launch.py backend:=spine|uart|gazebo|gazebo_esp32
      [radius:=30.0] [centre_x:=0.0] [centre_y:=0.0] [site:=] [max_speed:=]
      [gps:=true] [stands:=false] [use_sim_time:=false]

backend: who drives the car, for its command timeout and whether it brakes to a stop
(geofence.py motion_from): spine (spine_bridge, the bench, Gazebo spine:=true), uart
(the 2025-26 ESP32: in speed_loop open it coasts, so every program stays held), gazebo,
gazebo_esp32. No default: a wrong guess would be a wrong stopping distance.
radius, centre_x, centre_y: a circle round the car's marked start spot (the map frame:
car.launch.py's start_x, start_y). site: a polygon in config/geofence.yaml's sites
instead, checked against navsat.yaml's datum. max_speed: a lower cap than ego.yaml's (m/s,
empty: ego.yaml's), as the launch gives ego. gps:=false: no GPS to wait for (only with
stands:=true). stands:=true: the wheels are off the ground, no fence, said loudly.
twist_mux's geofence lock (aav_command_mux config/twist_mux.yaml) reads its
/safety/geofence/lock. docs/geofence.md.
"""

import os

from aav_description import vehicle
from aav_safety.geofence import BACKENDS, motion_from
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def share(package, *path):
    return os.path.join(FindPackageShare(package).find(package), *path)


def params(package, file, node):
    with open(share(package, 'config', file)) as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def flag(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name}:={value}: use true or false')
    return value == 'true'


def node_parameters(context):
    """Return the geofence's parameters from the arguments and the configs."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    backend = arg('backend')
    if backend not in BACKENDS:
        raise ValueError(f'backend:={backend}: pick one of {", ".join(BACKENDS)}')
    radius = float(arg('radius'))
    if not radius > 0.0:
        raise ValueError(f'radius:={radius}: the fence needs a radius > 0 m')
    stands, gps = flag(context, 'stands'), flag(context, 'gps')
    if not gps and not stands:
        raise ValueError('gps:=false without stands:=true: with no GPS the fence can never '
                         'be trusted (every program would stay held). On stands, say so')
    own = params('aav_safety', 'geofence.yaml', 'geofence')
    max_speed = arg('max_speed')
    motion, coasting = motion_from(
        params('aav_ego', 'ego.yaml', 'ego'), vehicle.load(), backend, float(own['rate_hz']),
        float(own['pose_lag']),
        spine=params('aav_spine', 'spine.yaml', 'spine_bridge'),
        legacy=params('aav_legacy', 'legacy.yaml', 'legacy_uart_bridge'),
        max_speed=float(max_speed) if max_speed else None)
    datum = params('aav_localization', 'navsat.yaml', 'navsat_transform')['datum']
    return {'site': arg('site'), 'radius': radius, 'centre_x': float(arg('centre_x')),
            'centre_y': float(arg('centre_y')), 'datum': [float(d) for d in datum],
            'stands': stands, 'gps': gps, 'coasting': coasting,
            'reaction_time': motion.reaction_time, 'braking': motion.braking,
            'max_accel': motion.max_accel, 'max_speed': motion.max_speed,
            'max_reverse_speed': motion.max_reverse_speed,
            'max_curvature': motion.max_curvature,
            'use_sim_time': flag(context, 'use_sim_time')}


def geofence(context):
    # These aren't in the file, so this dict adds them rather than losing to it.
    return [Node(package='aav_safety', executable='geofence', name='geofence',
                 output='screen',
                 parameters=[share('aav_safety', 'config', 'geofence.yaml'),
                             node_parameters(context)])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend',
                              description='spine, uart, gazebo or gazebo_esp32: its timeout '
                                          'and whether it brakes to a stop'),
        DeclareLaunchArgument('radius', default_value='30.0',
                              description="m: a circle round the car's start spot"),
        DeclareLaunchArgument('centre_x', default_value='0.0',
                              description='m, map frame: the start spot (start_x)'),
        DeclareLaunchArgument('centre_y', default_value='0.0',
                              description='m, map frame: the start spot (start_y)'),
        DeclareLaunchArgument('site', default_value='',
                              description="a polygon in config/geofence.yaml's sites, "
                                          'instead of the circle'),
        DeclareLaunchArgument('max_speed', default_value='',
                              description="m/s: ego's cap if lower than ego.yaml's"),
        DeclareLaunchArgument('gps', default_value='true'),
        DeclareLaunchArgument('stands', default_value='false',
                              description='true: wheels off the ground, no fence (loudly)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=geofence),
    ])
