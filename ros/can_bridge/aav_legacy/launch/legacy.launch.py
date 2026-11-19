"""
Today's car (2025-26 hardware) as a backend, until the CAN spine exists.

  ros2 launch aav_legacy legacy.launch.py [uart:=true] [encoder:=true] [rc:=true]
      [legacy_impl:=rust] [rc_impl:=rust] [port:=] [frame_version:=]
      [adaptive_brake:=false]

  /control/car/actuator_cmd, /safety/estop ──> legacy_uart_bridge ──> frames ──> ESP32
  ESP32 ──uplink (legacy.yaml uplink: true)──> legacy_uart_bridge ──> /vehicle/status,
      /vehicle/steering, /vehicle/brake, /vehicle/rc ──> rc_driver (aav_rc, with rc:=true)
      ──> /drivers/rc/cmd_vel, only while the RC's mode switch is on RC via ROS (TODO 48)
  Pico 2 wheel encoder ──UDP:10111──> encoder_udp ──> /vehicle/odom (speed only)

legacy_impl (rust): legacy_uart_bridge in Rust (aav_legacy_rs), the default since Ben
decided, 17 Nov: "Rust tomorrow". python: this package's own, the same node name, topics,
parameters and frames. rc_impl (rust): rc_driver in Rust (aav_rc_rs); python: aav_rc's.

port, frame_version (empty: legacy.yaml's): the bridge's serial port and frame, for one
run (a bench ESP32 on /dev/ttyUSB0, the Rust firmware's frame_version 2) without editing
legacy.yaml. The bridge has namespace '/', so these and vehicle.yaml's numbers are written
under its own name and beat legacy.yaml in both languages (rclrs lets a file's node name
beat launch's /**: docs/languages.md, "Launch overrides").

adaptive_brake (false): the adaptive brake (TODO 258; aav_description adaptive_brake.py,
the sim's, and its Rust twin, firmware/spine/crates/adaptive-brake) between the bridge's
pi speed loop and the brake actuator: harder when the car slows less than the car model
says, never above vehicle.yaml's brake: safe_max. Needs legacy.yaml's speed_loop.mode pi
(open asks for no brake to adapt): refused otherwise. The stops (stale, e-stop) never pass
through it. Its car model (actuators:) and the brake's limits (brake:) come from vehicle.yaml.

Run legacy_uart_bridge on the Pi wired to the ESP32's UART, encoder_udp anywhere
on the car's network. READ docs/firmware.md first: the ESP32 firmware has no
command timeout, so if this bridge dies the last throttle stays on.
"""

import os

from aav_description import vehicle
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

# The adaptive brake's car model: these of vehicle.yaml's actuators: (uart_relay.ADAPTIVE_CAR;
# the dead times may be missing: not measured yet, so 0, as the car model takes them).
ADAPTIVE_CAR = ('max_brake', 'brake_lag', 'brake_dead_time', 'drag', 'throttle_dead_time',
                'throttle_lag')


def adaptive_brake(context, car):
    """Return the bridge's adaptive brake parameters (off: just adaptive_brake false)."""
    if not on(context, 'adaptive_brake', 'false'):
        return {'adaptive_brake': False}
    path = os.path.join(get_package_share_directory('aav_legacy'), 'config', 'legacy.yaml')
    with open(path) as f:
        mode = yaml.safe_load(f)['legacy_uart_bridge']['ros__parameters']['speed_loop.mode']
    if mode != 'pi':
        raise ValueError(f'adaptive_brake:=true: legacy.yaml has speed_loop.mode {mode}; the '
                         'adaptive brake sits after the pi loop, and open asks for no brake '
                         'to adapt (TODO 258)')
    params = {'adaptive_brake': True}
    params.update({f'adaptive_brake.{k}': float(car['actuators'].get(k, 0.0))
                   for k in ADAPTIVE_CAR})
    params.update({f'brake.{k}': float(v) for k, v in (car.get('brake') or {}).items()})
    return params


def on(context, name, default='true'):
    """Return True if the launch argument `name` says true."""
    return context.launch_configurations.get(name, default).strip().lower() == 'true'


def nodes(context):
    """Return the bridge and rc_driver, each in the language its *_impl picks."""
    config = PathJoinSubstitution([FindPackageShare('aav_legacy'), 'config', 'legacy.yaml'])
    car = vehicle.load()
    v = car['vehicle']
    # The speed loop's car model (speed_loop.mode: pi): vehicle.yaml's actuators:.
    model = {f'speed_loop.{k}': float(car['actuators'][k])
             for k in ('max_accel', 'max_brake', 'drag')}
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    # One run's port and frame, over legacy.yaml's (empty: keep the file's).
    overrides = {}
    port = context.launch_configurations.get('port', '').strip()
    if port:
        overrides['port'] = port
    frame = context.launch_configurations.get('frame_version', '').strip()
    if frame:
        if frame not in ('1', '2', '3'):
            raise ValueError(f'frame_version:={frame}: use 1, 2 or 3 (legacy.yaml)')
        overrides['frame_version'] = int(frame)
    legacy = vehicle.check_legacy_impl(context.launch_configurations.get(
        'legacy_impl', vehicle.DEFAULT_LEGACY_IMPL))
    rc = vehicle.check_rc_impl(context.launch_configurations.get(
        'rc_impl', vehicle.DEFAULT_RC_IMPL))
    actions = []
    if on(context, 'uart'):
        actions.append(Node(
            package=vehicle.LEGACY_IMPLS[legacy],
            executable='legacy_uart_bridge',
            name='legacy_uart_bridge', namespace='/',
            parameters=[config, {'max_steering_angle': float(v['max_steering_angle'])},
                        model, adaptive_brake(context, car), use_sim_time, overrides]))
        if on(context, 'rc'):
            actions.append(Node(
                package=vehicle.RC_IMPLS[rc],
                executable='rc_driver',
                name='rc_driver', namespace='/',
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
        DeclareLaunchArgument('port', default_value='',
                              description="the bridge's serial port (empty: legacy.yaml's)"),
        DeclareLaunchArgument('frame_version', default_value='',
                              description="1, 2 or 3: the bridge's frame (empty: "
                                          "legacy.yaml's)"),
        DeclareLaunchArgument('adaptive_brake', default_value='false',
                              description='true: the adaptive brake after the pi speed loop '
                                          '(TODO 258; needs speed_loop.mode pi). Never above '
                                          "vehicle.yaml's brake: safe_max"),
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
