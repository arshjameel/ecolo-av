"""
KISS-ICP lidar odometry (aav_kiss_icp): the node kiss_icp, a trial. Never TF.

  ros2 launch aav_kiss_icp kiss_icp.launch.py [kiss_python:=] [use_sim_time:=false]

kiss_python  the kiss environment's Python; empty: <repo>/.pixi/envs/kiss/bin/python
             (pixi install --locked -e kiss, once). Refused if it isn't there: Linux and
             Apple-silicon Macs only (KISS-ICP's wheels)

The lidar's mount (lidar_link) and the car's outline come from aav_description's
vehicle.yaml; the rest from config/kiss_icp.yaml. Topics: docs/interfaces.yaml
(/localization/kiss_icp/*). sim.launch.py kiss_icp:=true includes this.
"""

import os

from aav_description import vehicle
from aav_kiss_icp import odometry
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def car_parameters(v):
    """Return the node's parameters that come from vehicle.yaml: the mount and the body."""
    lidar = v['sensors']['lidar_link']
    body = v['vehicle']['body']
    rear, half = -float(body['rear_overhang']), float(body['width']) / 2.0
    return {
        'lidar_xyz': [float(x) for x in lidar['xyz']],
        'lidar_rpy': [float(x) for x in lidar['rpy']],
        'body_min': [rear, -half],
        'body_max': [rear + float(body['length']), half],
    }


def python_path(given):
    """Return the kiss environment's Python; refuse one that isn't there."""
    path = odometry.find_python(given)
    if not os.path.isfile(path):
        raise ValueError(f'kiss_icp: no kiss environment at {path}. Install it once with '
                         '`pixi install --locked -e kiss` (Linux and Apple-silicon Macs; '
                         'docs/designs/2026-10-30_kiss-icp.md)')
    return path


def nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = FindPackageShare('aav_kiss_icp').perform(context)
    return [Node(package='aav_kiss_icp', executable='kiss_icp', name='kiss_icp',
                 output='screen',
                 parameters=[os.path.join(share, 'config', 'kiss_icp.yaml'),
                             car_parameters(vehicle.load()),
                             {'kiss_python': python_path(arg('kiss_python')),
                              'use_sim_time': arg('use_sim_time').lower() == 'true'}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('kiss_python', default_value='',
                              description="the kiss env's Python (empty: .pixi/envs/kiss)"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
