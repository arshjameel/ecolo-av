"""
Today's car (2025-26 hardware) as a backend, until the CAN spine exists.

  ros2 launch aav_legacy legacy.launch.py [uart:=true] [encoder:=true] [rc:=true]

  /control/car/actuator_cmd, /safety/estop ──> legacy_uart_bridge ──> frames ──> ESP32
  ESP32 ──uplink (legacy.yaml uplink: true)──> legacy_uart_bridge ──> /vehicle/status,
      /vehicle/steering, /vehicle/brake, /vehicle/rc ──> rc_driver (aav_rc, with rc:=true)
      ──> /drivers/rc/cmd_vel, only while the RC's mode switch is on RC via ROS (TODO 48)
  Pico 2 wheel encoder ──UDP:10111──> encoder_udp ──> /vehicle/odom (speed only)

Run legacy_uart_bridge on the Pi wired to the ESP32's UART, encoder_udp anywhere
on the car's network. READ docs/firmware.md first: the ESP32 firmware has no
command timeout, so if this bridge dies the last throttle stays on.
"""

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('aav_legacy'), 'config', 'legacy.yaml'])
    car = vehicle.load()
    v = car['vehicle']
    # The speed loop's car model (speed_loop.mode: pi): vehicle.yaml's actuators:, as Gazebo's.
    model = {f'speed_loop.{k}': float(car['actuators'][k])
             for k in ('max_accel', 'max_brake', 'drag')}
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('uart', default_value='true'),
        DeclareLaunchArgument('encoder', default_value='true'),
        DeclareLaunchArgument('rc', default_value='true',
                              description='rc_driver beside the bridge: it drives only on RC '
                                          'via ROS, with the uplink on'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='aav_legacy',
            executable='legacy_uart_bridge',
            name='legacy_uart_bridge',
            parameters=[config, {'max_steering_angle': float(v['max_steering_angle'])},
                        model, use_sim_time],
            condition=IfCondition(LaunchConfiguration('uart')),
        ),
        Node(
            package='aav_legacy',
            executable='encoder_udp',
            name='encoder_udp',
            parameters=[config, {'wheel_diameter': float(v['wheel_diameter']),
                                 'counts_per_rev': int(v['encoder_counts_per_rev'])},
                        use_sim_time],
            condition=IfCondition(LaunchConfiguration('encoder')),
        ),
        Node(
            package='aav_rc',
            executable='rc_driver',
            name='rc_driver',
            parameters=[{'max_reverse_speed': float(v['max_reverse_speed']),
                         'max_steering_angle': float(v['max_steering_angle']),
                         'wheelbase': float(v['wheelbase'])}, use_sim_time],
            condition=IfCondition(AndSubstitution(LaunchConfiguration('uart'),
                                                  LaunchConfiguration('rc'))),
        ),
    ])
