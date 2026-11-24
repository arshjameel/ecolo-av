"""
The RC's rules on a laptop: a gamepad as the FS-i6X, spine_sim's model car, rviz (23 Nov).

  pixi run rc-pad [mode_start:=high|low] [rviz:=true|false]

  gamepad -> game_controller_node -> /drivers/rc_pad/joy -> rc_pad (a pretend receiver)
  rc_pad -> /vehicle/bench/rc_channels (the 14 iBUS channels, µs) -> spine_sim rc:=true
  spine_sim (spine-core, the firmware's own RC rules) -> its model car -> /vehicle/bench/truth

Nothing else runs: no ego, no muxes, no spine_bridge, so the computer never drives. With
the mode switch high (manual, RC direct, the default here) the sticks drive the model car
straight through the firmware. Low (normal) is the right stick's hand-over, but with no
computer to hand to, give leaves the car standing still.

The gamepad: a DualShock 4 by USB or Bluetooth (bench-car's PadLayout::PS4). The left stick
is the pedal (CH3), the right stick stop (down), give (up) and steering (CH4), R1 flips the
mode switch (CH6), L1 held is the radio lost (the failsafe).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('aav_spine_sim')
    return LaunchDescription([
        DeclareLaunchArgument('mode_start', default_value='high',
                              description='where the mode switch (CH6) starts: high '
                                          '(manual, RC direct) or low (normal)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description="rviz: the model car's path, from above"),
        Node(package='joy', executable='game_controller_node', name='rc_pad_joy',
             remappings=[('joy', '/drivers/rc_pad/joy'),
                         ('joy/set_feedback', '/drivers/rc_pad/joy/set_feedback')]),
        Node(package='aav_spine_sim', executable='rc_pad', name='rc_pad', output='screen',
             parameters=[{'mode_start': LaunchConfiguration('mode_start')}]),
        Node(package='aav_spine_sim', executable='spine_sim', name='spine_sim',
             output='screen', parameters=[{'car': 'model', 'rc': True}]),
        Node(package='rviz2', executable='rviz2', name='rc_pad_rviz',
             arguments=['-d', os.path.join(share, 'config', 'rc_pad.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        LogInfo(msg='The car is held for its first second, until the left stick moves. '
                    'Its speed: ros2 topic echo /vehicle/bench/truth --field '
                    'twist.twist.linear.x (pixi run tools/ros.sh --local ...)'),
    ])
