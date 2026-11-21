"""
The real car's CAN bus: ros2_socketcan + spine_bridge.

  ros2 launch aav_spine spine.launch.py [interface:=can0] [bridge:=rust|python]
      [role:=spine|sensors] [require_ready:=false]

role:=spine (the default): the CAN spine drives the car.
  /control/car/actuator_cmd (the output mux), /safety/estop ──> spine_bridge ──> /can/tx
      ──> socket_can_sender ──> can0
  can0 ──> socket_can_receiver ──> /can/rx ──> spine_bridge ──> /vehicle/odom,
                                                                    /vehicle/status,
                                                                    /vehicle/joint_states
  Service /spine/arm (std_srvs/SetBool): the spine ignores DRIVE_CMD until armed.

role:=sensors: the car is driven another way (today's: legacy_uart_bridge on the Motor
Pi), and this bus has only sensors on it: the wheel nodes (docs/can-plan.md). No sender
starts, and spine_bridge never sends:
  can0 ──> socket_can_receiver ──> /can/rx ──> spine_bridge ──> /vehicle/odom,
                                                                    /vehicle/wheels

require_ready (false): /spine/arm also waits for the boot self-test's READY
(/safety/self_test/status, aav_self_test_rs; TODO 255). car.launch.py sets it true; the
bench and Ravensim's spine leave it false (arm_when_open arms them once the gate opens).

bridge picks spine_bridge's language: rust (aav_spine_bridge, the default) or python (this
package's). The same node either way, with this package's config/spine.yaml.

Bring the interface up first (once per boot, on the Orin):
  sudo ip link set can0 up type can bitrate 500000
The frame layouts are firmware/can/aav.dbc.
"""

from aav_description import vehicle
from aav_spine import bridges
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def bridge(context):
    """spine_bridge, in the language bridge:= picks, in the role role:= picks."""
    v = vehicle.load()['vehicle']
    role = bridges.check_role(LaunchConfiguration('role').perform(context))
    return [Node(
        package=bridges.package(LaunchConfiguration('bridge').perform(context)),
        executable=bridges.EXECUTABLE,
        name='spine_bridge', namespace='/',
        parameters=[
            PathJoinSubstitution([FindPackageShare('aav_spine'), 'config', 'spine.yaml']),
            vehicle.spine_params(v),
            {'use_sim_time': LaunchConfiguration('use_sim_time'), 'role': role,
             'require_ready': LaunchConfiguration('require_ready').perform(context)
             .strip().lower() == 'true'},
        ],
    )]


def bus(context):
    """
    ros2_socketcan: the receiver always; the sender only in role spine.

    Lifecycle nodes; these launch files configure and activate them. Their topics
    (from_can_bus, to_can_bus) move to /can/rx and /can/tx through the launch files' own
    *_topic arguments.
    """
    socketcan = FindPackageShare('ros2_socketcan')
    interface = LaunchConfiguration('interface')
    actions = [IncludeLaunchDescription(
        PathJoinSubstitution([socketcan, 'launch', 'socket_can_receiver.launch.py']),
        launch_arguments={'interface': interface, 'from_can_bus_topic': '/can/rx'}.items())]
    if bridges.check_role(LaunchConfiguration('role').perform(context)) == 'spine':
        actions.append(IncludeLaunchDescription(
            PathJoinSubstitution([socketcan, 'launch', 'socket_can_sender.launch.py']),
            launch_arguments={'interface': interface, 'to_can_bus_topic': '/can/tx'}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bridge', default_value=bridges.DEFAULT,
                              description='spine_bridge in rust or python'),
        DeclareLaunchArgument('role', default_value=bridges.ROLES[0],
                              description='spine: the CAN spine drives the car; sensors: '
                                          'read the wheel nodes only, send nothing'),
        DeclareLaunchArgument('require_ready', default_value='false',
                              description='true: /spine/arm waits for the boot self-test '
                                          'to say READY (the car)'),
        OpaqueFunction(function=bus),
        OpaqueFunction(function=bridge),
    ])
