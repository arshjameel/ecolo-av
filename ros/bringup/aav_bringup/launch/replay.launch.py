"""
Re-run localization (and optionally Nav2) on a recorded drive, on any laptop.

  ros2 launch aav_bringup replay.launch.py [nav:=false] [rviz:=false]
  ros2 bag play <bag> --clock --topics /vehicle/odom /sensors/imu/data_raw \
      /sensors/gps/fix /sensors/gps/fix_velocity /tf_static

A bag from the simulator carries simulated time, so play its own /clock instead of
--clock:
  ros2 bag play <bag> --topics /clock /vehicle/odom /sensors/imu/data_raw \
      /sensors/gps/fix /sensors/gps/fix_velocity /tf_static

The bag (tools/record.sh) supplies the raw inputs and the clock; this file runs the
same IMU filter and EKFs as the car, so a config change can be tried on yesterday's
drive. To replay a drive's COMMANDS into the car instead, see tools/replay_commands.sh.

It also says, latched, on /system/source, that the sensors are a recording (REPLAY), so
the HUD shows a REPLAY banner: `ros2 topic pub` as the node replay_source.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# /system/source: what the bag plays (aav_interfaces/SensorSource REPLAY = 3).
REPLAYED = (('odometry', '/vehicle/odom'), ('imu', '/sensors/imu/data_raw'),
            ('gps', '/sensors/gps/fix'))
ANNOUNCEMENT = ('{announcer: replay_source, sensors: ['
                + ', '.join(f"{{name: {n}, topic: '{t}', source: 3}}" for n, t in REPLAYED)
                + ']}')


def launch_setup(context):
    def share(p):
        return FindPackageShare(p).perform(context)

    def include(package, launch_file, **args):
        return IncludeLaunchDescription(
            os.path.join(share(package), 'launch', launch_file),
            launch_arguments={**args, 'use_sim_time': 'true'}.items())

    actions = [
        include('aav_description', 'description.launch.py'),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick',
             parameters=[os.path.join(share('aav_sensors'), 'config', 'imu_filter.yaml'),
                         {'use_sim_time': True}],
             remappings=[('imu/data_raw', '/sensors/imu/data_raw'),
                         ('imu/data', '/sensors/imu/data')]),
        include('aav_localization', 'localization.launch.py'),
        # The sensor mux checks that only the bag plays the sensors (REPLAY). No output
        # mux: a replay drives nothing.
        IncludeLaunchDescription(
            os.path.join(share('aav_io_mux'), 'launch', 'io_mux.launch.py'),
            launch_arguments={'target': '', 'source': 'replay'}.items()),
        # Latched, and again every 10 s (a late subscriber gets it either way).
        ExecuteProcess(name='replay_source', output='log', cmd=[
            'ros2', 'topic', 'pub', '-n', 'replay_source', '-r', '0.1',
            '--qos-reliability', 'reliable', '--qos-durability', 'transient_local',
            '--qos-depth', '1', '/system/source', 'aav_interfaces/msg/DataSource',
            ANNOUNCEMENT]),
    ]
    if LaunchConfiguration('nav').perform(context).lower() == 'true':
        actions.append(include('aav_navigation', 'navigation.launch.py'))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=['-d', os.path.join(share('aav_bringup'), 'rviz',
                                                          'drive.rviz')],
                            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
