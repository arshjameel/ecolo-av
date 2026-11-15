"""
street_router: routes as Nav2 GPS waypoints (docs/routes.md).

Streets, from an offline router:

  pixi run -e routes router                          # the router, in another terminal
  ros2 launch aav_routes street_router.launch.py
  ros2 topic pub --once /drivers/nav/route/destination geographic_msgs/msg/GeoPointStamped \
    "{position: {latitude: 45.3983, longitude: -75.6838}}"
  ros2 service call /drivers/nav/route/follow std_srvs/srv/Trigger     # then Nav2 drives it

The campus, from its own road graph (no router to start; pixi run campus once):

  ros2 launch aav_routes street_router.launch.py router:=campus use_sim_time:=true speed:=2.5
  ros2 topic pub --once /drivers/nav/route/place std_msgs/msg/String "{data: Fieldhouse}"

For real streets later; the test lot first. Settings: config/street_router.yaml.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('router', default_value='valhalla',
                              description='valhalla, osrm, or campus (its own road graph)'),
        DeclareLaunchArgument('speed', default_value='1.0',
                              description='m/s: the car, for the time the status line gives'),
        Node(
            package='aav_routes',
            executable='street_router',
            name='street_router',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('aav_routes'), 'config', 'street_router.yaml']),
                {'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'router': LaunchConfiguration('router'),
                 'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float)},
            ],
        ),
    ])
