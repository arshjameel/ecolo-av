"""
The sensor watchdog: stops Nav2 and the AI slot when a sensor they need dies or lies.

  ros2 launch aav_safety watchdog.launch.py inputs:=odom,imu,gps,local,global
      [use_sim_time:=false] [watchdog_impl:=rust]

inputs: the names in config/sensor_watchdog.yaml to watch, comma-separated; each profile
passes the sensors it has (sim.launch.py, bench.launch.py, car.launch.py with Nav2).
twist_mux's sensors lock (aav_command_mux config/twist_mux.yaml) reads its
/safety/sensor_watchdog/lock. docs/simulation.md ("Sensor failures on purpose").

watchdog_impl (rust): aav_watchdog_rs, the default since Ben decided (16 Nov: "watchdog in
rust sure"). The Python node went on 20 Nov (tag save/python-safety-nodes); this package
keeps its maths, config and launch files.
"""

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def watchdog(context):
    names = [n.strip() for n in LaunchConfiguration('inputs').perform(context).split(',')
             if n.strip()]
    config = FindPackageShare('aav_safety').perform(context) + '/config/sensor_watchdog.yaml'
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    impl = vehicle.check_watchdog_impl(LaunchConfiguration('watchdog_impl').perform(context))
    # inputs isn't in the file, so this dict adds it rather than losing to the file.
    return [Node(package=vehicle.WATCHDOG_IMPLS[impl], executable='sensor_watchdog',
                 name='sensor_watchdog', namespace='/',
                 output='screen',
                 parameters=[config, {'inputs': names, 'use_sim_time': sim_time}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('inputs', default_value='odom,imu,gps,local,global',
                              description='which of config/sensor_watchdog.yaml to watch'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('watchdog_impl', default_value=vehicle.DEFAULT_WATCHDOG_IMPL,
                              description='rust (aav_watchdog_rs; the Python removed '
                                          '20 Nov)'),
        OpaqueFunction(function=watchdog),
    ])
