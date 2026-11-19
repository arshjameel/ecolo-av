"""
The boot self-test (TODO 255): READY or NOT READY, and why, on /safety/self_test/status.

  ros2 launch aav_self_test_rs self_test.launch.py [self_test_actuators:=status]
      [self_test_locks:=watchdog,geofence,output_gate] [self_test_clocks:=]
      [use_sim_time:=false]

self_test_actuators  status (/vehicle/status: the CAN spine, or the ESP32's uplink), link
                     (the UART bridge writes frames; it can't hear the ESP32) or none
self_test_locks      the locks that must be alive: watchdog, geofence, output_gate,
                     stop_zone (comma separated)
self_test_clocks     the stamped sources whose clocks must agree with this computer's:
                     imu, gps, odom (comma separated; empty: none)

The arguments start with self_test_: included launch files share their arguments.
car.launch.py starts it next to the mux; spine_bridge (require_ready:=true) refuses to arm
until it says READY. It moves nothing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ACTUATORS = ('status', 'link', 'none')
LOCKS = ('watchdog', 'geofence', 'output_gate', 'stop_zone')
CLOCKS = ('imu', 'gps', 'odom')


def names(given, known, what):
    """Return the comma-separated names, checked against `known`."""
    found = [n.strip() for n in given.split(',') if n.strip()]
    unknown = [n for n in found if n not in known]
    if unknown:
        raise ValueError(f'self_test_{what}:={given}: unknown {unknown}; use {", ".join(known)}')
    return found


def node(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    actuators = arg('self_test_actuators')
    if actuators not in ACTUATORS:
        raise ValueError(f'self_test_actuators:={actuators}: use {", ".join(ACTUATORS)}')
    params = {'actuators': actuators,
              'locks': names(arg('self_test_locks'), LOCKS, 'locks'),
              'use_sim_time': arg('use_sim_time').lower() == 'true'}
    clocks = names(arg('self_test_clocks'), CLOCKS, 'clocks')
    if clocks:   # an empty list has no type in ROS: leave the node's default (none)
        params['clocks'] = clocks
    return [Node(package='aav_self_test_rs', executable='boot_self_test',
                 name='boot_self_test', namespace='/', output='screen',
                 parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('self_test_actuators', default_value='status',
                              description='status, link or none'),
        DeclareLaunchArgument('self_test_locks', default_value='watchdog,geofence,output_gate',
                              description='the locks that must be alive'),
        DeclareLaunchArgument('self_test_clocks', default_value='',
                              description='imu, gps, odom: clocks that must agree with ours'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=node),
    ])
