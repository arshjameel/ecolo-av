"""
The whole stack with CARLA as the car: the same nodes and configs as sim.launch.py.

  ros2 launch aav_bringup carla.launch.py host:=<the CARLA computer> [nav:=true]
      [town:=] [osm:=] [spawn:=] [frame:=auto] [control:=ackermann] [sync:=true]
      [cameras:=true] [radar:=true] [lidar:=true] [teleop:=false] [rviz:=false]
      [foxglove:=false] [watchdog:=true] [sim_speed:=] [reversing:=true]
  pixi run carla-car host:=192.168.1.50                  a town, Nav2 and the sensors
  pixi run carla-car host:=127.0.0.1 osm:=maps/carla/carleton.osm   Carleton's roads

Only the backend differs from Gazebo: aav_carla's carla_adapter in place of
gazebo_adapter, so mux, ego, the EKFs, navsat_transform, Nav2, radar_clusters and the
sensor watchdog run with the car's own configs. Every node runs on CARLA's clock.

Where things run (docs/carla.md): the CARLA server on an NVIDIA computer; this launch on
a Linux x86_64 computer with our pixi environment (CARLA has no macOS client), often the
same one. The adapter reaches the server over TCP (host:port), not ROS, so ROS discovery
stays on this computer (pixi run carla-car uses tools/ros.sh --local, like every
stand-in). To watch from a Mac: AAV_DISCOVERY=SUBNET pixi run carla-car ..., on a network
with no real car on the same ROS_DOMAIN_ID.

The map: Nav2 gets the empty map. In a town the car starts at the map origin facing east,
as in Gazebo; on a Carleton map (osm:=) the map frame is CARLA's world, round the datum.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

# Passed to aav_carla's carla.launch.py as they are.
BACKEND = {'host': '127.0.0.1', 'port': '2000', 'town': '', 'osm': '', 'xodr': '',
           'spawn': '', 'frame': 'auto', 'control': 'ackermann', 'vehicle': '',
           'sync': 'true', 'fixed_delta': '0.05', 'realtime': 'true', 'cameras': 'true',
           'radar': 'true', 'lidar': 'true'}
FLAGS = ('nav', 'teleop', 'rviz', 'foxglove', 'watchdog', 'reversing')


def include(context, package, launch_file, **args):
    share = FindPackageShare(package).perform(context)
    return IncludeLaunchDescription(
        os.path.join(share, 'launch', launch_file),
        launch_arguments={k: str(v).lower() if isinstance(v, bool) else str(v)
                          for k, v in args.items()}.items())


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    flag = {n: arg(n).lower() == 'true' for n in FLAGS}
    radar = arg('radar').lower() == 'true'
    lidar = arg('lidar').lower() == 'true'
    nav_share = FindPackageShare('aav_navigation').perform(context)
    actions = [
        SetParameter(name='use_sim_time', value=True),
        include(context, 'aav_carla', 'carla.launch.py', **{k: arg(k) for k in BACKEND}),
        include(context, 'aav_command_mux', 'mux.launch.py', use_sim_time='true',
                # No geofence here yet (CARLA's towns are big): its lock stays open.
                sensor_lock=flag['watchdog'], geofence_lock='false'),
        include(context, 'aav_ego', 'ego.launch.py', use_sim_time='true',
                sim_max_speed=arg('sim_speed')),
        # ego's commands go to carla_adapter only while every source is SIM (aav_io_mux).
        include(context, 'aav_io_mux', 'io_mux.launch.py', target='carla', source='sim'),
        include(context, 'aav_scripted', 'scripted.launch.py', use_sim_time='true'),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick',
             parameters=[os.path.join(FindPackageShare('aav_sensors').perform(context),
                                      'config', 'imu_filter.yaml'), {'use_sim_time': True}],
             remappings=[('imu/data_raw', '/sensors/imu/data_raw'),
                         ('imu/data', '/sensors/imu/data')]),
        include(context, 'aav_localization', 'localization.launch.py', use_sim_time='true'),
    ]
    if radar:
        actions.append(include(context, 'aav_radar_clusters', 'radar_clusters.launch.py',
                               minimum_points='5'))
    if flag['nav']:
        actions.append(include(context, 'aav_navigation', 'navigation.launch.py',
                               use_sim_time='true',
                               map=os.path.join(nav_share, 'maps', 'empty.yaml'),
                               radar=radar, lidar=lidar, collision_monitor=radar or lidar,
                               max_speed=arg('sim_speed'),
                               reversing=str(flag['reversing']).lower()))
    if flag['watchdog']:
        inputs = ['odom', 'imu', 'gps', 'local', 'global'] + (['radar'] if radar else [])
        actions.append(include(context, 'aav_safety', 'watchdog.launch.py',
                               inputs=','.join(inputs), use_sim_time='true'))
    if flag['teleop']:
        actions.append(include(context, 'aav_teleop', 'teleop.launch.py', use_sim_time='true'))
    if flag['foxglove']:
        # This computer only, like the rest (tools/ros.sh --local): from a Mac, through an
        # SSH tunnel (docs/foxglove.md). The front camera's preview, as on the car.
        cameras = arg('cameras').lower() == 'true'
        actions.append(include(context, 'aav_bringup', 'foxglove.launch.py',
                               foxglove_address='127.0.0.1', foxglove_port='8765',
                               use_sim_time='true', foxglove_control='true',
                               foxglove_topics='wifi',
                               foxglove_preview='front_left' if cameras else ''))
    if flag['rviz']:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(FindPackageShare('aav_bringup').perform(context),
                                          'rviz', 'road.rviz')],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in BACKEND.items()]
    args += [
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='true if a joystick is plugged into this computer'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz: needs a screen (the CARLA computer may '
                                          'have none)'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='foxglove_bridge on 127.0.0.1:8765, for Lichtblick '
                                          'or Foxglove (foxglove.launch.py)'),
        DeclareLaunchArgument('watchdog', default_value='true',
                              description='the sensor watchdog (aav_safety)'),
        DeclareLaunchArgument('sim_speed', default_value='',
                              description='m/s, up to 5: a higher top speed for ego and Nav2, '
                                          'in simulators only'),
        DeclareLaunchArgument('reversing', default_value='true',
                              description='Nav2 may drive backwards'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
