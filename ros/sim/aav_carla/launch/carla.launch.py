"""
CARLA as the car: carla_adapter, and the car's model (TF) on CARLA's clock.

  ros2 launch aav_carla carla.launch.py host:=<the CARLA computer> [port:=2000]
      [town:=] [osm:=<file.osm>] [xodr:=<file.xodr>] [sync:=true] [fixed_delta:=0.05]
      [realtime:=true] [vehicle:=] [spawn:=] [frame:=auto] [control:=ackermann]
      [cameras:=true] [radar:=true] [lidar:=true]

The CARLA server runs first, anywhere this computer can reach on TCP port 2000-2001
(docs/carla.md). carla_adapter needs CARLA's Python client (Linux x86_64 or Windows):
on a Mac it stops with a one-line explanation.

  town:=Town10HD_Opt          load one of the server's maps ('' keeps what it has)
  osm:=maps/carla/carleton.osm  Carleton's roads from OpenStreetMap (pixi run
                              get carla-osm), lined up with our datum. CARLA 0.9.x only
  frame:=auto                 the map frame: CARLA's world when its map is round our datum
                              (a Carleton map), else the spawn point facing east (a town)
  spawn:=3 or spawn:=x,y,yaw  where the car starts (x,y,yaw: our map frame)
  control:=pedals             throttle and brake from our speed loop, instead of CARLA's
                              Ackermann controller

The datum comes from aav_localization's navsat.yaml, the one the EKFs use.
"""

import os

from aav_carla import georef
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# carla_adapter's parameters that come straight from an argument of the same name, with
# their defaults, and how to read them.
TEXT = {'host': '127.0.0.1', 'town': '', 'osm': '', 'xodr': '', 'vehicle': '', 'spawn': '',
        'frame': 'auto', 'control': 'ackermann', 'role_name': 'ego_vehicle'}
NUMBER = {'port': '2000', 'fixed_delta': '0.05', 'timeout': '20.0'}
FLAG = {'sync': 'true', 'realtime': 'true', 'cameras': 'true', 'radar': 'true',
        'lidar': 'true'}


def datum(context):
    path = os.path.join(FindPackageShare('aav_localization').perform(context), 'config',
                        'navsat.yaml')
    with open(path) as f:
        return georef.datum_from_navsat(f.read())


def adapter_parameters(context):
    """Return carla_adapter's parameters from the launch arguments."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    params = {k: arg(k) for k in TEXT}
    params['port'] = int(arg('port'))
    params['fixed_delta'] = float(arg('fixed_delta'))
    params['timeout'] = float(arg('timeout'))
    for k in FLAG:
        value = arg(k).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{k}:={value}: true or false')
        params[k] = value == 'true'
    if params['osm'] and params['xodr']:
        raise ValueError('osm:= or xodr:=, not both')
    for k in ('osm', 'xodr'):
        if params[k]:
            params[k] = os.path.abspath(params[k])
            if not os.path.isfile(params[k]):
                raise ValueError(f'{k}:={params[k]}: no such file')
    if params['frame'] not in ('auto', 'world', 'spawn'):
        raise ValueError(f"frame:={params['frame']}: auto, world or spawn")
    if params['control'] not in ('ackermann', 'pedals'):
        raise ValueError(f"control:={params['control']}: ackermann or pedals")
    georef.parse_spawn(params['spawn'])           # refuse a bad one now, not in the node
    lat, lon, _ = datum(context)
    params['datum_lat'], params['datum_lon'] = lat, lon
    return params


def launch_setup(context):
    params = adapter_parameters(context)
    return [
        # The car's model and TF, on CARLA's clock. joint_state_publisher fills the
        # joints: nobody measures them in CARLA.
        IncludeLaunchDescription(
            os.path.join(FindPackageShare('aav_description').perform(context), 'launch',
                         'description.launch.py'),
            launch_arguments={'use_sim_time': 'true',
                              'lidar': str(params['lidar']).lower()}.items()),
        # The adapter IS the clock (/clock from CARLA's steps), so it runs on wall time.
        Node(package='aav_carla', executable='carla_adapter', name='carla_adapter',
             output='screen', parameters=[params, {'use_sim_time': False}]),
    ]


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in
            {**TEXT, **NUMBER, **FLAG}.items()]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
