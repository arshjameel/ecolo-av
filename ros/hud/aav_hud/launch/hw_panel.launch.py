"""
The hardware panel (aav_hud): one window, which is also the ROS node `hw_panel`.

  ros2 launch aav_hud hw_panel.launch.py
  ros2 launch aav_hud hw_panel.launch.py stands:=true fullscreen:=true monitor:=1

Every sensor's number straight from its own node, and very simple buttons for the
actuators: Speed +/-, the steering arrows, CENTRE, STOP, GO held to drive (Space), and
the E-STOP. The buttons go to twist_mux's buttons slot (45), then ego and the spine; the
speed buttons work only on SIM and BENCH, never the real car on the ground, and on stands
(stands:=true) only once a driven-wheel speed exists (TODO 49b).

Topic names, steps and caps: config/hw_panel.yaml. The car: aav_description's
vehicle.yaml. monitor: as in hud.launch.py, by name or number. It needs a screen.
Design: docs/designs/2026-10-29_two-screens.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def panel_node(context):
    """Make the node; the files, --stands, --fullscreen and --monitor are arguments."""
    vehicle = PathJoinSubstitution(
        [FindPackageShare('aav_description'), 'config', 'vehicle.yaml'])
    arguments = ['--vehicle', vehicle]
    if IfCondition(LaunchConfiguration('stands')).evaluate(context):
        arguments.append('--stands')
    if IfCondition(LaunchConfiguration('fullscreen')).evaluate(context):
        arguments.append('--fullscreen')
    monitor = LaunchConfiguration('monitor').perform(context).strip()
    if monitor:
        arguments += ['--monitor', monitor]
    return [Node(
        package='aav_hud',
        executable='hw_panel',
        name='hw_panel',
        arguments=arguments,
        parameters=[
            PathJoinSubstitution([FindPackageShare('aav_hud'), 'config', 'hw_panel.yaml']),
        ],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stands', default_value='false',
                              description='The wheels are off the ground (the real car: '
                                          'speed buttons still off until TODO 49b)'),
        DeclareLaunchArgument('fullscreen', default_value='false',
                              description='Fill the screen (F11 switches too)'),
        DeclareLaunchArgument('monitor', default_value='',
                              description='The monitor, by name or number; '
                                          'empty: wherever the desktop puts it'),
        OpaqueFunction(function=panel_node),
    ])
