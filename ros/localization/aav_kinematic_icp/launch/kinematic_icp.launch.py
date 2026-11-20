"""
Kinematic-ICP's idea in Rust (aav_kinematic_icp): the node kinematic_icp, never TF.

A challenger to aav_kiss_icp (TODO 216).

  ros2 launch aav_kinematic_icp kinematic_icp.launch.py [use_sim_time:=false]

A drop-in: the same topics as kiss_icp (/localization/kiss_icp/odom and status), so
ekf_local's kiss_ekf:=true file reads it unchanged. sim.launch.py odom_lidar:=kinematic_icp
includes this one instead of kiss_icp.launch.py. The lidar's mount and the car's outline
come from aav_description's vehicle.yaml; the rest from config/kinematic_icp.yaml.
"""

import os

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def car_parameters(v):
    """Return the node's parameters from vehicle.yaml: the mount and the body (as kiss_icp's)."""
    lidar = v['sensors']['lidar_link']
    body = v['vehicle']['body']
    rear, half = -float(body['rear_overhang']), float(body['width']) / 2.0
    return {
        'lidar_xyz': [float(x) for x in lidar['xyz']],
        'lidar_rpy': [float(x) for x in lidar['rpy']],
        'body_min': [rear, -half],
        'body_max': [rear + float(body['length']), half],
    }


def nodes(context):
    share = FindPackageShare('aav_kinematic_icp').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [Node(package='aav_kinematic_icp', executable='kinematic_icp', name='kinematic_icp',
                 namespace='/',
                 output='screen',
                 parameters=[os.path.join(share, 'config', 'kinematic_icp.yaml'),
                             car_parameters(vehicle.load()),
                             {'use_sim_time': use_sim_time}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
