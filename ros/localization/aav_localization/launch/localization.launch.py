"""
Localization: two EKFs, each fed by the raw sensors; GPS comes in through navsat_transform.

Neither EKF reads the other's topic. They meet only in TF: ekf_global publishes
map -> odom, worked out on top of ekf_local's odom -> base_link.

  ros2 launch aav_localization localization.launch.py [gps:=true] [gps_heading:=true]
      [health:=true] [localization_health_impl:=rust] [gps_heading_impl:=rust]
      [ndt_ekf:=false] [kiss_ekf:=false]
      [pose_source:=gps] [pose_trust_score:=]
      [lane_pose:=false] [lane_map:=]
      [start_x:=0.0] [start_y:=0.0]
      [start_yaw:=0.0] [use_sim_time:=false]

  /vehicle/odom, /sensors/imu/data ──> ekf_local ──> /localization/odom/local, TF odom -> base_link
  /vehicle/odom, /sensors/imu/data, /localization/gps/odom, /localization/gps/heading
      ──> ekf_global ──> /localization/odom/global, TF map -> odom
  /sensors/gps/fix, /localization/odom/global ──> navsat_transform
      ──> /localization/gps/odom, /localization/gps/fix
  /sensors/gps/fix_velocity, /vehicle/odom ──> gps_heading (aav_gps_heading_rs)
      ──> /localization/gps/heading: the heading from the receiver's velocity, while the
      car moves (TODO 13). gps_heading:=false leaves it out

Both EKFs also read /control/cmd_vel (twist_mux's command): robot_localization's control
term predicts the speed from it (vehicle.ekf_control_params), so with the wheels silent a
car told to stop is predicted to stop (TODO 207; before, ekf_local kept its last speed and
ran on 15 m while the car stood). The wheels, when they speak, still decide.

global_ekf:=false (a test, sim only: sim.launch.py truth_pose:=true): no ekf_global;
aav_sim_models's truth_localizer publishes /localization/odom/global and map -> odom from the
simulator's truth instead.

ndt_ekf:=true (a trial, docs/trials.md): ekf_global also takes NDT's pose,
/localization/ndt/pose (x, y, yaw), with config/ekf_global_ndt.yaml on top of
ekf_global.yaml. NDT itself is aav_ndt's: sim.launch.py ndt_ekf:=true starts both.

pose_source:=ndt (a challenger, TODO 210; gps, the default, is today's): NDT leads,
the GPS checks. pose_check (aav_pose_check) passes NDT's fits to ekf_global while NDT is
trusted, the GPS's only while it isn't and the GPS is, and HOLDs (an ERROR status the
sensor watchdog stops on) when neither is; ekf_global reads both through it
(config/ekf_global_ndt.yaml, then config/ekf_global_ndt_leads.yaml on top). NDT itself,
the watchdog's inputs and the geofence's fix are the caller's: sim.launch.py
pose_source:=ndt sets them. pose_trust_score: NDT's score per point that wins a
disagreement (empty: pose_check.yaml's, the lot's).

  /localization/ndt/pose, /localization/ndt/status, /localization/gps/odom,
  /sensors/gps/fix, /localization/amcl/pose ──> pose_check
      ──> /localization/pose_check/ndt_pose, /localization/pose_check/gps_odom (ekf_global),
          /localization/pose_check/fix (the geofence), /localization/pose_check/status,
          /localization/ndt/initial_pose (a seed for a lost NDT)

kiss_ekf:=true (a trial, docs/trials.md): ekf_local also takes KISS-ICP's twist,
/localization/kiss_icp/odom (vx, vyaw; never its pose), with config/ekf_local_kiss.yaml
on top of ekf_local.yaml. KISS-ICP itself is aav_kiss_icp's: sim.launch.py
kiss_ekf:=true starts both. Without its odometry, ekf_local runs on the wheels and IMU.

lane_pose:=true (a trial, TODO 155): lane_pose (aav_lane_pose) turns the lane finder's
lines and lane_map:= (a lanes.yaml) into the car's place across its lane, and ekf_global
takes its x and y (config/ekf_global_lane.yaml on top; pose1 after NDT's with ndt_ekf).
The lane finder itself is the caller's: sim.launch.py lane_pose:=true starts it.

  /perception/lane/geometry, TF map -> base_link ──> lane_pose ──> /localization/lane/pose

start_x, start_y, start_yaw: where the car is when this starts, in the map frame
(metres east and north of the datum, yaw in radians counter-clockwise from east).
Park on the marked start spot and these are its numbers.

Without GPS (gps:=false: the bench, indoors, phase 0-1) only ekf_local runs and a
static map -> odom identity stands in, so map-frame tools still work.

  everything above, TF ──> localization_health (aav_localization_health_rs)
      ──> /localization/health, /diagnostics: warn-only checks (gyro bias, IMU mounting,
      GPS quality, stamps, frames, TF, front wheels, runaway EKFs). Drives nothing and
      changes nothing above. health:=false leaves it out.
      localization_health_impl: rust (aav_localization_health_rs), the only one since
      the Python went on 20 Nov
"""

import os

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

POSE_SOURCES = ('gps', 'ndt')


def health_node(context):
    """Start localization_health (localization_health_impl:=), or none."""
    if LaunchConfiguration('health').perform(context).lower() != 'true':
        return []
    impl = vehicle.check_localization_health_impl(
        LaunchConfiguration('localization_health_impl').perform(context))
    return [Node(
        package=vehicle.LOCALIZATION_HEALTH_IMPLS[impl],
        executable='localization_health',
        name='localization_health', namespace='/',
        parameters=[os.path.join(FindPackageShare('aav_localization_health_rs').perform(context),
                                 'config', 'localization_health.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}])]


def gps_heading_node(context):
    """Start gps_heading in Rust or Python (gps_heading_impl:=), with gps and gps_heading."""
    on = all(LaunchConfiguration(n).perform(context).lower() == 'true'
             for n in ('gps', 'gps_heading'))
    if not on:
        return []
    impl = vehicle.check_gps_heading_impl(
        LaunchConfiguration('gps_heading_impl').perform(context))
    return [Node(package=vehicle.GPS_HEADING_IMPLS[impl], executable='gps_heading',
                 name='gps_heading',
                 parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])]


def pose_source(context):
    """Return pose_source:= checked: gps (today's) or ndt (NDT leads, TODO 210)."""
    source = LaunchConfiguration('pose_source').perform(context).strip().lower()
    if source not in POSE_SOURCES:
        raise ValueError(f'pose_source:={source}: use {" or ".join(POSE_SOURCES)}')
    return source


def pose_check_node(context):
    """Start pose_check (aav_pose_check) with pose_source:=ndt, else nothing."""
    if pose_source(context) != 'ndt':
        return []
    params = [os.path.join(FindPackageShare('aav_pose_check').perform(context), 'config',
                           'pose_check.yaml'),
              {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    trust = LaunchConfiguration('pose_trust_score').perform(context).strip()
    if trust:
        params.append({'trust_score': float(trust)})
    return [Node(package='aav_pose_check', executable='pose_check', name='pose_check',
                 namespace='/', parameters=params)]


def ekf_services(node_name):
    # Both EKFs create set_pose, enable, reset and toggle with relative names; give each
    # its own so they don't collide on /set_pose.
    return [(s, f'{node_name}/{s}') for s in ('set_pose', 'enable', 'reset', 'toggle')]


def local_ekf(context):
    config = PathJoinSubstitution([FindPackageShare('aav_localization'), 'config'])
    files = [PathJoinSubstitution([config, 'ekf_local.yaml'])]
    if LaunchConfiguration('kiss_ekf').perform(context).lower() == 'true':
        files.append(PathJoinSubstitution([config, 'ekf_local_kiss.yaml']))
    return [Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        parameters=files + [vehicle.ekf_control_params(vehicle.load()),
                            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=([('odometry/filtered', '/localization/odom/local'),
                     ('cmd_vel', vehicle.EKF_CONTROL_TOPIC)]
                    + ekf_services('ekf_local')),
    )]


def global_ekf(context):
    x, y, yaw = (float(LaunchConfiguration(n).perform(context))
                 for n in ('start_x', 'start_y', 'start_yaw'))
    config = PathJoinSubstitution([FindPackageShare('aav_localization'), 'config'])
    initial_state = [x, y, 0.0, 0.0, 0.0, yaw] + [0.0] * 9
    files = [PathJoinSubstitution([config, 'ekf_global.yaml'])]
    leads = pose_source(context) == 'ndt'
    ndt = LaunchConfiguration('ndt_ekf').perform(context).lower() == 'true' or leads
    if ndt:
        files.append(PathJoinSubstitution([config, 'ekf_global_ndt.yaml']))
    if leads:
        # NDT's pose and the GPS's, both through pose_check (TODO 210).
        files.append(PathJoinSubstitution([config, 'ekf_global_ndt_leads.yaml']))
    if LaunchConfiguration('lane_pose').perform(context).lower() == 'true':
        # robot_localization reads pose0, pose1... in order: after NDT's, the lane's is pose1.
        files.append(lane_pose_params(
            os.path.join(FindPackageShare('aav_localization').perform(context), 'config',
                         'ekf_global_lane.yaml'), 1 if ndt else 0))
    return [Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        parameters=files + [vehicle.ekf_control_params(vehicle.load()),
                            {'initial_state': initial_state,
                             'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=([('odometry/filtered', '/localization/odom/global'),
                     ('cmd_vel', vehicle.EKF_CONTROL_TOPIC)]
                    + ekf_services('ekf_global')),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('gps'), "' == 'true' and '",
            LaunchConfiguration('global_ekf'), "' == 'true'"])),
    )]


def lane_pose_params(path, index):
    """Return ekf_global_lane.yaml's parameters, its pose0 renamed pose<index>."""
    with open(path) as f:
        params = yaml.safe_load(f)['ekf_global']['ros__parameters']
    return {(f'pose{index}' + k[len('pose0'):] if k.startswith('pose0') else k): v
            for k, v in params.items()}


def lane_pose_node(context):
    if LaunchConfiguration('lane_pose').perform(context).lower() != 'true':
        return []
    lane_map = LaunchConfiguration('lane_map').perform(context)
    if not lane_map:
        raise ValueError('lane_pose:=true needs lane_map:= (a lanes.yaml)')
    return [Node(package='aav_lane_pose', executable='lane_pose', name='lane_pose',
                 parameters=[{'lane_map': lane_map,
                              'use_sim_time': LaunchConfiguration('use_sim_time')}])]


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('aav_localization'), 'config'])
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('gps', default_value='true'),
        DeclareLaunchArgument('gps_heading', default_value='true'),
        DeclareLaunchArgument('gps_heading_impl', default_value=vehicle.DEFAULT_GPS_HEADING_IMPL,
                              description='rust (aav_gps_heading_rs): which gps_heading runs '
                                          '(the Python removed 20 Nov)'),
        DeclareLaunchArgument('health', default_value='true'),
        DeclareLaunchArgument('localization_health_impl',
                              default_value=vehicle.DEFAULT_LOCALIZATION_HEALTH_IMPL,
                              description='rust (aav_localization_health_rs): which '
                                          'localization_health runs (the Python removed 20 Nov)'),
        DeclareLaunchArgument('ndt_ekf', default_value='false',
                              description="ekf_global also takes NDT's pose (a trial)"),
        DeclareLaunchArgument('pose_source', default_value='gps',
                              description="gps (today's) or ndt: NDT leads ekf_global's "
                                          'pose, the GPS checks it (pose_check, a '
                                          'challenger, TODO 210)'),
        DeclareLaunchArgument('pose_trust_score', default_value='',
                              description="pose_source:=ndt: NDT's score per point that "
                                          "wins a disagreement (empty: pose_check.yaml's)"),
        DeclareLaunchArgument('kiss_ekf', default_value='false',
                              description="ekf_local also takes KISS-ICP's twist (a trial)"),
        DeclareLaunchArgument('lane_pose', default_value='false',
                              description='ekf_global also takes the lane pose: the lane '
                                          "finder's offset against lane_map (a trial)"),
        DeclareLaunchArgument('lane_map', default_value='',
                              description='the lanes.yaml lane_pose reads (lane_pose:=true)'),
        DeclareLaunchArgument('global_ekf', default_value='true',
                              description='false: no ekf_global; something else gives '
                                          'map -> odom (the sim, truth_pose:=true)'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        OpaqueFunction(function=local_ekf),
        OpaqueFunction(function=global_ekf),
        OpaqueFunction(function=lane_pose_node),
        OpaqueFunction(function=pose_check_node),
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
        OpaqueFunction(function=gps_heading_node),
        OpaqueFunction(function=health_node),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_no_gps',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[use_sim_time],
            condition=UnlessCondition(LaunchConfiguration('gps')),
        ),
    ])
