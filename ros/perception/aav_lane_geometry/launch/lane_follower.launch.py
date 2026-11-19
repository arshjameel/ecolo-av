"""
Lane keeping: the steering half, as a driver on twist_mux's AI slot.

  ros2 launch aav_lane_geometry lane_follower.launch.py [fake:=false]
      [lane_follower_impl:=rust] [lane_follower:=pid]

  /perception/lane/geometry (LaneGeometry) -> control_interface -> /drivers/ai/cmd_vel (TwistStamped, priority 10)

The half that finds the lane lines in the camera picture is aav_lane_finder's
lane_finder.launch.py: it publishes /perception/lane/geometry. fake:=true runs the 2025-26 team's
fake_lane_geometry_pub instead (a pretend straight lane, no camera). Every other driver
outranks /drivers/ai/cmd_vel, so a person, a test script or navigation always wins.

lane_follower_impl: rust (aav_lane_follower_rs, the default since 19 Nov: the same node
name, topics, parameters and commands) or python (this package's control_interface.py,
the 2025-26 team's).
lane_follower: how it steers. pid (the 2025-26 PID on the offset, the default), or a Rust
challenger on the fitted lane centre: pure_pursuit or stanley. The challengers read the
lines' fits in metres, so they need lane_finder, not fake:=true (its fits are pixels).
They get the wheelbase, the full lock and the lag (the camera, the steering's dead time
and lag) from aav_description's vehicle.yaml.

The Python script runs through python3, not its `#!/usr/bin/env python3` line: on a Mac,
/usr/bin/env drops the library path that its messages need (docs/mac.md).
"""

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def follower(context):
    """Return control_interface, in the language and with the steering asked for."""
    get = context.launch_configurations.get
    impl = vehicle.check_lane_follower_impl(
        get('lane_follower_impl', vehicle.DEFAULT_LANE_FOLLOWER_IMPL))
    law = vehicle.check_lane_follower(get('lane_follower', vehicle.DEFAULT_LANE_FOLLOWER),
                                      impl)
    package, program = vehicle.LANE_FOLLOWER_IMPLS[impl]
    parameters = []
    if impl == 'rust':
        car = vehicle.load()
        parameters = [{'steering': law,
                       'lag_s': vehicle.lane_follower_lag(car),
                       'wheelbase': float(car['vehicle']['wheelbase']),
                       'max_steering_angle': float(car['vehicle']['max_steering_angle'])}]
    return [Node(package=package, executable=program, name='control_interface',
                 namespace='/', prefix='python3' if impl == 'python' else None,
                 output='screen', parameters=parameters)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false',
                              description='a pretend straight lane on /perception/lane/geometry'),
        DeclareLaunchArgument('lane_follower_impl',
                              default_value=vehicle.DEFAULT_LANE_FOLLOWER_IMPL,
                              description='control_interface in rust (aav_lane_follower_rs, '
                                          'the default) or python (aav_lane_geometry)'),
        DeclareLaunchArgument('lane_follower', default_value=vehicle.DEFAULT_LANE_FOLLOWER,
                              description='how it steers: pid (the 2025-26 PID), or the Rust '
                                          'challengers pure_pursuit, stanley'),
        OpaqueFunction(function=follower),
        Node(package='aav_lane_geometry', executable='fake_lane_geometry_pub.py',
             name='fake_lane_geometry_pub', prefix='python3',
             condition=IfCondition(LaunchConfiguration('fake'))),
    ])
