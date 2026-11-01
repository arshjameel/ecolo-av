"""
The debug screen (aav_hud): one window, which is also the ROS node `debug_hud`.

  ros2 launch aav_hud debug_hud.launch.py
  ros2 launch aav_hud debug_hud.launch.py fullscreen:=true monitor:=1

What the car sees and what it thinks: the lidar's cubes round the car in 3D, where each
localizer puts the car (EKF local blue, EKF global green, Gazebo's truth white, AMCL
orange), Nav2's plan (yellow), and a panel of every sensor straight from its own topic.
It only listens: it never touches an actuator.

Topic names, frames and limits: config/debug_hud.yaml. The car's size: aav_description's
vehicle.yaml. monitor: as in hud.launch.py, by name or number. It needs a screen.
The whole simulated car with it: pixi run debugdemo (tools/demo_all.sh debug).
Keys: drag turn, right-drag move, wheel zoom, C chase / follow / free, R reset, F11.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def debug_node(context):
    """Make the node; the file, --fullscreen and --monitor are arguments, not parameters."""
    vehicle = PathJoinSubstitution(
        [FindPackageShare('aav_description'), 'config', 'vehicle.yaml'])
    arguments = ['--vehicle', vehicle]
    if IfCondition(LaunchConfiguration('fullscreen')).evaluate(context):
        arguments.append('--fullscreen')
    monitor = LaunchConfiguration('monitor').perform(context).strip()
    if monitor:
        arguments += ['--monitor', monitor]
    return [Node(
        package='aav_hud',
        executable='debug_hud',
        name='debug_hud',
        arguments=arguments,
        parameters=[
            PathJoinSubstitution([FindPackageShare('aav_hud'), 'config', 'debug_hud.yaml']),
        ],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fullscreen', default_value='false',
                              description='Fill the screen (F11 switches too)'),
        DeclareLaunchArgument('monitor', default_value='',
                              description='The monitor, by name or number; '
                                          'empty: wherever the desktop puts it'),
        OpaqueFunction(function=debug_node),
    ])
