"""
Localization: two EKFs, each fed by the raw sensors; GPS comes in through navsat_transform.

Neither EKF reads the other's topic. They meet only in TF: ekf_global publishes
map -> odom, worked out on top of ekf_local's odom -> base_link.

  ros2 launch aav_localization localization.launch.py [gps:=true] [gps_heading:=true]
      [health:=true] [start_x:=0.0] [start_y:=0.0] [start_yaw:=0.0] [use_sim_time:=false]

  /vehicle/odom, /sensors/imu/data ──> ekf_local ──> /localization/odom/local, TF odom -> base_link
  /vehicle/odom, /sensors/imu/data, /localization/gps/odom, /localization/gps/heading
      ──> ekf_global ──> /localization/odom/global, TF map -> odom
  /sensors/gps/fix, /localization/odom/global ──> navsat_transform
      ──> /localization/gps/odom, /localization/gps/fix
  /sensors/gps/fix_velocity, /vehicle/odom ──> gps_heading (aav_gps_heading)
      ──> /localization/gps/heading: the heading from the receiver's velocity, while the
      car moves (TODO 13). gps_heading:=false leaves it out

start_x, start_y, start_yaw: where the car is when this starts, in the map frame
(metres east and north of the datum, yaw in radians counter-clockwise from east).
Park on the marked start spot and these are its numbers.

Without GPS (gps:=false: the bench, indoors, phase 0-1) only ekf_local runs and a
static map -> odom identity stands in, so map-frame tools still work.

  everything above, TF ──> localization_health (aav_localization_health)
      ──> /localization/health, /diagnostics: warn-only checks (gyro bias, IMU mounting,
      GPS quality, stamps, frames, TF, front wheels, runaway EKFs). Drives nothing and
      changes nothing above. health:=false leaves it out
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def ekf_services(node_name):
    # Both EKFs create set_pose, enable, reset and toggle with relative names; give each
    # its own so they don't collide on /set_pose.
    return [(s, f'{node_name}/{s}') for s in ('set_pose', 'enable', 'reset', 'toggle')]


def global_ekf(context):
    x, y, yaw = (float(LaunchConfiguration(n).perform(context))
                 for n in ('start_x', 'start_y', 'start_yaw'))
    config = PathJoinSubstitution([FindPackageShare('aav_localization'), 'config'])
    initial_state = [x, y, 0.0, 0.0, 0.0, yaw] + [0.0] * 9
    return [Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        parameters=[PathJoinSubstitution([config, 'ekf_global.yaml']),
                    {'initial_state': initial_state,
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=([('odometry/filtered', '/localization/odom/global')]
                    + ekf_services('ekf_global')),
        condition=IfCondition(LaunchConfiguration('gps')),
    )]


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('aav_localization'), 'config'])
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('gps', default_value='true'),
        DeclareLaunchArgument('gps_heading', default_value='true'),
        DeclareLaunchArgument('health', default_value='true'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            parameters=[PathJoinSubstitution([config, 'ekf_local.yaml']), use_sim_time],
            remappings=([('odometry/filtered', '/localization/odom/local')]
                        + ekf_services('ekf_local')),
        ),
        OpaqueFunction(function=global_ekf),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[PathJoinSubstitution([config, 'navsat.yaml']), use_sim_time],
            remappings=[('imu', '/sensors/imu/data'),
                        ('gps/fix', '/sensors/gps/fix'),
                        ('gps/filtered', '/localization/gps/fix'),
                        ('odometry/gps', '/localization/gps/odom'),
                        ('odometry/filtered', '/localization/odom/global')],
            condition=IfCondition(LaunchConfiguration('gps')),
        ),
        Node(
            package='aav_gps_heading',
            executable='gps_heading',
            name='gps_heading',
            parameters=[use_sim_time],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('gps'), "' == 'true' and '",
                LaunchConfiguration('gps_heading'), "' == 'true'"])),
        ),
        Node(
            package='aav_localization_health',
            executable='localization_health',
            name='localization_health',
            parameters=[PathJoinSubstitution([FindPackageShare('aav_localization_health'),
                                              'config', 'localization_health.yaml']),
                        use_sim_time],
            condition=IfCondition(LaunchConfiguration('health')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_no_gps',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[use_sim_time],
            condition=UnlessCondition(LaunchConfiguration('gps')),
        ),
    ])
