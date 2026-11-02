"""
NDT on a saved lidar map (aav_ndt, Rust): the node ndt, a trial. Never TF.

  ros2 launch aav_ndt ndt.launch.py [ndt_map:=maps/ndt/test_lot.pcd] [ndt_x:=0.0]
      [ndt_y:=0.0] [ndt_yaw:=0.0] [ndt_from_global:=false] [use_sim_time:=false]

The arguments start with ndt_: included launch files share their arguments, and sim's
own map and start_y mean other things (aav_bringup README).

ndt_map     the PCD file (pixi run ndt-map drive makes the test lot's); a relative path is
            from the folder the launch starts in (pixi starts it in the repo). Refused if
            it isn't there
ndt_x, ndt_y, ndt_yaw  where it starts, map frame (m, rad); sim.launch.py ndt:=true
            passes the car's start, as AMCL's
ndt_from_global  take ekf_global's pose instead (the car)

The lidar's mount (lidar_link) and the car's box (vehicle.self_box) come from aav_description's
vehicle.yaml; the rest from config/ndt.yaml. Topics: docs/interfaces.yaml
(/localization/ndt/*).
"""

import os

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MAP = os.path.join('maps', 'ndt', 'test_lot.pcd')


def car_parameters(v):
    """
    Return the node's parameters that come from vehicle.yaml: the mount and the car's box.

    The box is vehicle.self_box: the body and every sensor on it, 15 cm wider all round, so
    the lidar's noise can't put the car's own roof or cameras into the scan (30 Oct).
    """
    lidar = v['sensors']['lidar_link']
    box = vehicle.self_box(v)
    return {
        'lidar_xyz': [float(x) for x in lidar['xyz']],
        'lidar_rpy': [float(x) for x in lidar['rpy']],
        'body_min': [float(x) for x in box['min']],
        'body_max': [float(x) for x in box['max']],
    }


def map_path(given):
    """Return the map file's absolute path; refuse one that isn't there."""
    path = os.path.abspath(os.path.expanduser(given))
    if not os.path.isfile(path):
        raise ValueError(f"ndt_map:={given}: no such file. Make the test lot's with "
                         '`pixi run ndt-map drive` (docs/designs/2026-10-29_ndt.md)')
    return path


def nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = FindPackageShare('aav_ndt').perform(context)
    start = {'start_x': float(arg('ndt_x')), 'start_y': float(arg('ndt_y')),
             'start_yaw': float(arg('ndt_yaw')),
             'start_from_global': arg('ndt_from_global').lower() == 'true'}
    return [Node(package='aav_ndt', executable='ndt', name='ndt', output='screen',
                 parameters=[os.path.join(share, 'config', 'ndt.yaml'),
                             car_parameters(vehicle.load()), start,
                             {'map': map_path(arg('ndt_map')),
                              'use_sim_time': arg('use_sim_time').lower() == 'true'}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ndt_map', default_value=MAP,
                              description='the PCD map (pixi run ndt-map drive)'),
        DeclareLaunchArgument('ndt_x', default_value='0.0', description='m, map frame'),
        DeclareLaunchArgument('ndt_y', default_value='0.0', description='m, map frame'),
        DeclareLaunchArgument('ndt_yaw', default_value='0.0', description='rad, map frame'),
        DeclareLaunchArgument('ndt_from_global', default_value='false',
                              description="start from ekf_global's pose (the car)"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
