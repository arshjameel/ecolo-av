"""
The big screen's map window (aav_hud): one window, which is also the ROS node `map`.

  ros2 launch aav_hud map.launch.py
  ros2 launch aav_hud map.launch.py fullscreen:=true
  ros2 launch aav_hud map.launch.py fullscreen:=true monitor:=HDMI-2 \
      streets:=/path/ottawa.pmtiles places:=/path/ottawa.db

Nav2's map, the car on it, the plan and the goal, and the distance left, over an
offline street map of Ottawa (pixi run get street-map). Topic names and the street
map's file: config/map.yaml, the one place they're written; streets:= and places:=
override the files (empty: config/map.yaml's). The car's size: aav_description's
vehicle.yaml. Where map (0, 0) is on the Earth: aav_localization's navsat.yaml, the
datum navsat_transform uses. It needs a screen: on a computer without one, it fails.
monitor: as in hud.launch.py, by name or number. On the car, tools/car_screen.sh starts
it at boot (docs/car-screens.md).
Keys: / search for a place (ottawa.db: pixi run get places), then DRIVE HERE and
CONFIRM publish Nav2's goal on /goal_pose; F follow the car / whole map, S streets
on / off, + - or the mouse wheel zoom, drag to move.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def map_node(context):
    """Make the node; the files, --fullscreen and --monitor are arguments, not parameters."""
    vehicle = PathJoinSubstitution(
        [FindPackageShare('aav_description'), 'config', 'vehicle.yaml'])
    navsat = PathJoinSubstitution(
        [FindPackageShare('aav_localization'), 'config', 'navsat.yaml'])
    arguments = ['--vehicle', vehicle, '--navsat', navsat]
    if IfCondition(LaunchConfiguration('fullscreen')).evaluate(context):
        arguments.append('--fullscreen')
    for name, option in (('monitor', '--monitor'), ('streets', '--streets'),
                         ('places', '--places')):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            arguments += [option, value]
    return [Node(
        package='aav_hud',
        executable='map',
        name='map',
        arguments=arguments,
        parameters=[
            PathJoinSubstitution([FindPackageShare('aav_hud'), 'config', 'map.yaml']),
        ],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fullscreen', default_value='false',
                              description='Fill the screen (F11 switches too)'),
        DeclareLaunchArgument('monitor', default_value='',
                              description='The monitor, by name (HDMI-2) or number (1); '
                                          'empty: wherever the desktop puts it'),
        DeclareLaunchArgument('streets', default_value='',
                              description="The street map (PMTiles); empty: map.yaml's"),
        DeclareLaunchArgument('places', default_value='',
                              description="The place database (SQLite); empty: map.yaml's"),
        OpaqueFunction(function=map_node),
    ])
