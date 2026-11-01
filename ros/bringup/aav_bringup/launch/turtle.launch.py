"""
Phase 0 on any laptop: turtlesim as the car.

Runs the real mux, ego, scripted driver and ekf_local. No hardware, no Gazebo. The
sensor mux and the output mux (aav_io_mux, target turtle) pass ego's commands to the
turtle while everything is SIM.

  ros2 launch aav_bringup turtle.launch.py [teleop:=false]

Then, in another terminal:
  ros2 action send_goal /scripted_drive aav_interfaces/action/ScriptedDrive --feedback \
    "{target_speed: 1.0, acceleration: 0.5, deceleration: 1.0, hold_time: 3.0, curvature: 0.2}"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, condition=None, **args):
    return IncludeLaunchDescription(
        [FindPackageShare(package), os.sep + os.path.join('launch', launch_file)],
        launch_arguments=args.items(), condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='false',
                              description='true if a joystick is plugged in'),
        include('aav_turtlesim', 'turtlesim.launch.py'),
        # No sensor watchdog here, so no heartbeat for the mux's sensors lock to need.
        # No global EKF either, so no geofence: its lock stays open too.
        include('aav_command_mux', 'mux.launch.py', sensor_lock='false',
                geofence_lock='false'),
        include('aav_ego', 'ego.launch.py'),
        # ego's commands go to the turtle only while everything is SIM (aav_io_mux).
        include('aav_io_mux', 'io_mux.launch.py', target='turtle', source='sim'),
        include('aav_scripted', 'scripted.launch.py'),
        include('aav_description', 'description.launch.py'),
        include('aav_localization', 'localization.launch.py', gps='false'),
        include('aav_teleop', 'teleop.launch.py',
                condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
