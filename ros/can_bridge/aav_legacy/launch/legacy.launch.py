"""
Today's car (2025-26 hardware) as a backend, until the CAN spine exists.

  ros2 launch aav_legacy legacy.launch.py [uart:=true] [encoder:=true] [rc:=true]
      [legacy_impl:=rust] [rc_impl:=rust]

  /control/car/actuator_cmd, /safety/estop ──> legacy_uart_bridge ──> frames ──> ESP32
  ESP32 ──uplink (legacy.yaml uplink: true)──> legacy_uart_bridge ──> /vehicle/status,
      /vehicle/steering, /vehicle/brake, /vehicle/rc ──> rc_driver (aav_rc, with rc:=true)
      ──> /drivers/rc/cmd_vel, only while the RC's mode switch is on RC via ROS (TODO 48)
  Pico 2 wheel encoder ──UDP:10111──> encoder_udp ──> /vehicle/odom (speed only)

legacy_impl (rust): legacy_uart_bridge in Rust (aav_legacy_rs), the default since Ben
decided, 17 Nov: "Rust tomorrow". python: this package's own, the same node name, topics,
parameters and frames. rc_impl (rust): rc_driver in Rust (aav_rc_rs); python: aav_rc's.

Run legacy_uart_bridge on the Pi wired to the ESP32's UART, encoder_udp anywhere
on the car's network. READ docs/firmware.md first: the ESP32 firmware has no
command timeout, so if this bridge dies the last throttle stays on.
"""

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def on(context, name):
    """Return True if the launch argument `name` says true."""
    return context.launch_configurations.get(name, 'true').strip().lower() == 'true'


def nodes(context):
    """Return the bridge and rc_driver, each in the language its *_impl picks."""
    config = PathJoinSubstitution([FindPackageShare('aav_legacy'), 'config', 'legacy.yaml'])
    car = vehicle.load()
    v = car['vehicle']
    # The speed loop's car model (speed_loop.mode: pi): vehicle.yaml's actuators:.
    model = {f'speed_loop.{k}': float(car['actuators'][k])
             for k in ('max_accel', 'max_brake', 'drag')}
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    legacy = vehicle.check_legacy_impl(context.launch_configurations.get(
        'legacy_impl', vehicle.DEFAULT_LEGACY_IMPL))
    rc = vehicle.check_rc_impl(context.launch_configurations.get(
        'rc_impl', vehicle.DEFAULT_RC_IMPL))
    actions = []
    if on(context, 'uart'):
        actions.append(Node(
            package=vehicle.LEGACY_IMPLS[legacy],
            executable='legacy_uart_bridge',
            name='legacy_uart_bridge',
            parameters=[config, {'max_steering_angle': float(v['max_steering_angle'])},
                        model, use_sim_time]))
        if on(context, 'rc'):
            actions.append(Node(
                package=vehicle.RC_IMPLS[rc],
                executable='rc_driver',
                name='rc_driver',
                parameters=[{'max_reverse_speed': float(v['max_reverse_speed']),
                             'max_steering_angle': float(v['max_steering_angle']),
                             'wheelbase': float(v['wheelbase'])}, use_sim_time]))
    return actions


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('aav_legacy'), 'config', 'legacy.yaml'])
    v = vehicle.load()['vehicle']
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('uart', default_value='true'),
        DeclareLaunchArgument('encoder', default_value='true'),
        DeclareLaunchArgument('rc', default_value='true',
                              description='rc_driver beside the bridge: it drives only on RC '
                                          'via ROS, with the uplink on'),
        DeclareLaunchArgument('legacy_impl', default_value=vehicle.DEFAULT_LEGACY_IMPL,
                              description='legacy_uart_bridge in rust (aav_legacy_rs, the '
                                          'default) or python (aav_legacy)'),
        DeclareLaunchArgument('rc_impl', default_value=vehicle.DEFAULT_RC_IMPL,
                              description='rc_driver in rust (aav_rc_rs, the default) or '
                                          'python (aav_rc)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=nodes),
        Node(
            package='aav_legacy',
            executable='encoder_udp',
            name='encoder_udp',
            parameters=[config, {'wheel_diameter': float(v['wheel_diameter']),
                                 'counts_per_rev': int(v['encoder_counts_per_rev'])},
                        use_sim_time],
            condition=IfCondition(LaunchConfiguration('encoder')),
        ),
    ])
