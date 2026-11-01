"""
The command mux: every /drivers/<driver>/cmd_vel in, one /control/cmd_vel out (to ego).

  ros2 launch aav_command_mux mux.launch.py geofence_lock:=true|false
      [use_sim_time:=true] [sensor_lock:=true]

See config/twist_mux.yaml for the inputs and their priorities.

sensor_lock:=true (the default) keeps the sensors lock fail-safe: Nav2 and the AI slot are
held unless aav_safety's sensor_watchdog says "false" at least once a second. Start the
watchdog wherever this runs with it (sim, bench and car launch files do). sensor_lock:=false
is for profiles that run no watchdog (the turtle): config/no_sensor_lock.yaml sets the
lock's timeout to 0, so a lock that nobody publishes stays open, as before.

geofence_lock (no default: say it) does the same for aav_safety's geofence. true: every
program is held unless the geofence says "false" at least once a second; start it wherever
this runs with it (car, bench, sim). false (config/no_geofence_lock.yaml, loaded after
twist_mux.yaml and no_sensor_lock.yaml) is for profiles that run no geofence (the turtle,
CARLA). No default, so a new user of this file can't silently get a lock that never opens.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def config(name):
    return PathJoinSubstitution([FindPackageShare('aav_command_mux'), 'config', name])


def flag(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name}:={value}: use true or false')
    return value == 'true'


def mux(context):
    # A second file after the first, both under twist_mux:, so it wins. A dict wouldn't:
    # see the file (and docs/conventions.md, "A launch override that silently loses").
    files = ([config('twist_mux.yaml')]
             + ([] if flag(context, 'sensor_lock') else [config('no_sensor_lock.yaml')])
             + ([] if flag(context, 'geofence_lock') else [config('no_geofence_lock.yaml')]))
    return [Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[*files, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('cmd_vel_out', '/control/cmd_vel')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensor_lock', default_value='true',
                              description="true: Nav2 and the AI slot need sensor_watchdog's "
                                          'heartbeat; false: no watchdog runs (the turtle)'),
        DeclareLaunchArgument('geofence_lock',
                              description="true: every program needs the geofence's "
                                          'heartbeat; false: no geofence runs (the turtle, '
                                          'CARLA). No default'),
        OpaqueFunction(function=mux),
    ])
