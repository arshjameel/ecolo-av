"""
Joystick driver: /drivers/joy/joy -> /drivers/joy/cmd_vel, and an e-stop button.

The command is TwistStamped; the button publishes /safety/estop true.

  ros2 launch aav_teleop teleop.launch.py [estop_button:=true] [joy_device:=true]
                                          [use_sim_time:=false] [joy_alive_impl:=rust]

joy_device:=false leaves joy_node out, for a stand-in that publishes /drivers/joy/joy
itself (tools/bench_check.py's frozen pad).

joy_alive (aav_joy_alive_rs) sits between joy_node and teleop_twist_joy: it passes the stick on
as /drivers/joy/joy_checked only while the joystick is really there and moving, and says
why not on /drivers/joy/alive (TODO 74a). The e-stop button reads joy_node directly.
joy_alive_impl: rust (aav_joy_alive_rs, the default since 17 Nov; the Python went on
20 Nov, tag save/python-safety-nodes).

The joystick is a driver like any other: it publishes a velocity command into
twist_mux and never talks to the wheels. Run it on the machine the USB receiver is
plugged into (any machine on the same ROS_DOMAIN_ID).

All four nodes run in the namespace /drivers/joy, so their relative topics land there:
joy_node's joy is /drivers/joy/joy, teleop_twist_joy's cmd_vel is /drivers/joy/cmd_vel.
The stock nodes' config files name them /**/<node>, so they match in any namespace.
joy_alive's names it in full (/drivers/joy/joy_alive): the Rust joy_alive (rclrs) never
matches /**/, so a value there would be silently ignored (found 18 Nov).
"""

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def joy_alive(context):
    """Return joy_alive, in the language joy_alive_impl picks."""
    impl = vehicle.check_joy_alive_impl(context.launch_configurations.get(
        'joy_alive_impl', vehicle.DEFAULT_JOY_ALIVE_IMPL))
    config = PathJoinSubstitution([FindPackageShare('aav_teleop'), 'config'])
    return [Node(
        package=vehicle.JOY_ALIVE_IMPLS[impl],
        executable='joy_alive',
        name='joy_alive',
        namespace='/drivers/joy',
        parameters=[PathJoinSubstitution([config, 'joy_alive.yaml']),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )]


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('aav_teleop'), 'config'])
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('estop_button', default_value='true',
                              description='Also map a button to /safety/estop (joy_teleop)'),
        DeclareLaunchArgument('joy_device', default_value='true',
                              description='Start joy_node (false: a stand-in publishes joy)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joy_alive_impl', default_value=vehicle.DEFAULT_JOY_ALIVE_IMPL,
                              description='joy_alive in rust (aav_joy_alive_rs; the Python '
                                          'removed 20 Nov)'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='/drivers/joy',
            parameters=[PathJoinSubstitution([config, 'joy.yaml']), use_sim_time],
            condition=IfCondition(LaunchConfiguration('joy_device')),
        ),
        OpaqueFunction(function=joy_alive),     # between joy_node and teleop_twist_joy
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            namespace='/drivers/joy',
            parameters=[PathJoinSubstitution([config, 'teleop_twist_joy.yaml']), use_sim_time],
            remappings=[('joy', 'joy_checked')],   # only what joy_alive passes
        ),
        Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            namespace='/drivers/joy',
            parameters=[PathJoinSubstitution([config, 'estop_button.yaml']), use_sim_time],
            condition=IfCondition(LaunchConfiguration('estop_button')),
        ),
    ])
