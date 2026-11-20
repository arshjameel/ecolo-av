"""
Bench mode: the real car's software on one computer, with stand-ins for the hardware.

  ros2 launch aav_bringup bench.launch.py [nav:=true] [teleop:=false|true|stand_in] [rviz:=false]
      [perception:=false] [lane:=false|true] [hud:=false] [arm:=true] [foxglove:=false]
      [start_y:=0.0] [start_yaw:=0.0] [bend_radius:=] [obstacle:=false]
      [camera_obstacles:=false] [reversing:=true] [lidar:=false] [plan_around:=true]
      [bridge:=rust|python] [speed_loop:=default|smooth|lqi|firm] [speed_loop_model:=]
      [geofence_radius:=100.0]
      [stop_sign_impl:=rust|python]
      [geofence_impl:=rust] [stop_hold_impl:=rust] [goal_leash_impl:=rust]
  pixi run bench                            (the same)
  pixi run bench bridge:=python             (aav_spine's Python spine_bridge, not the Rust one)
  pixi run -e ml bench perception:=true     (adds YOLO and the stop-sign watcher)
  pixi run bench lane:=true                 (the lane finder on the pretend camera)
  pixi run bench lidar:=true obstacle:=true (the roof lidar, through its real driver)

Above the hardware, everything is the car's own: spine_bridge and its CAN frames, the
EKFs with GPS, twist_mux, ego, the drivers and navigation. Below it, stand-ins, on the same
topics and frames as the real devices:

  spine_sim                 the spine board and actuators: the firmware's own logic
                            (spine-core) on the CAN frames, driving a model car
  mock_imu, mock_gps,       the sensors, reading where the model car really is
  mock_battery, mock_camera,
  mock_radar                the radar sees the bench world's obstacles (obstacle:=true);
                            radar_clusters (aav_radar_clusters, Rust) clusters its points,
                            and our local costmap marks them (navigation.launch.py
                            radar:=true)
  mock_lidar                with lidar:=true: the roof's VLP-16, as UDP packets of the
                            bench world (its boxes and the ground) to 127.0.0.1:2369.
                            Below the driver: the stock velodyne driver (aav_sensors
                            sensors.launch.py lidar:=true, as on the car) reads them, and
                            our costmap and the stop zone read its points

camera_obstacles:=true adds aav_detector's camera_obstacles (the detectors' boxes as points on
the road, from mock_camera's lens) and the local costmap's camera layer. The bench camera
draws only stop signs, which never count: publish a detection by hand to see it work.

The model car starts at the map origin (the GPS datum), facing east, on the centre line
of a straight road; start_y and start_yaw move it. A stop sign stands 25 m ahead on the
right; obstacle:=true puts a 1 m box in the lane 15 m ahead (aav_bench/config/bench.yaml).
lane:=true runs the lane finder on the pretend camera: /perception/lane/geometry (the
lane follower that steered by it went on 19 Nov, TODO 264: the costmap draws the lanes).
lane_finder_impl:=python runs the Python lane_finder in the Rust one's place (the default).

arm:=true arms the spine after 12 s, once navigation is up, the way `ros2 service call
/spine/arm std_srvs/srv/SetBool "{data: true}"` does: aav_spine's arm_when_open asks
each second until the output mux says OPEN, since spine_bridge refuses to arm while the
safety gate is shut (Ben, 24 Oct). Arming earlier, while everything starts, risks a gap
in commands that latches the spine again: its 200 ms watchdog doing its job.

The geofence (aav_safety) runs as on the car, always: a circle of geofence_radius m round
the start spot (0, start_y) (100 m here, past every bench drive; the car's is 30 m), with
the spine's stopping numbers. twist_mux's geofence lock needs its heartbeat.
bench-check --geofence uses 12 m.
"""

import os

from aav_description import vehicle
from aav_spine import bridges
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def share(package):
    return FindPackageShare(package).find(package)


def include(package, launch_file, **args):
    return IncludeLaunchDescription(
        os.path.join(share(package), 'launch', launch_file),
        launch_arguments={k: str(v) for k, v in args.items()}.items())


def launch_setup(context):
    flag = {n: LaunchConfiguration(n).perform(context).lower() == 'true'
            for n in ('nav', 'rviz', 'perception', 'hud', 'arm', 'foxglove',
                      'obstacle', 'camera_obstacles', 'reversing', 'lidar', 'plan_around')}
    lane = LaunchConfiguration('lane').perform(context).lower()
    if lane not in ('false', 'true'):
        raise ValueError(f'lane:={lane}: use true (the lane finder) or false')
    start = {n: float(LaunchConfiguration(n).perform(context)) for n in ('start_y', 'start_yaw')}
    bend = LaunchConfiguration('bend_radius').perform(context)
    road = [{'bend_radius': float(bend)}] if bend else []
    v = vehicle.load()['vehicle']
    bench_config = os.path.join(share('aav_bench'), 'config', 'bench.yaml')
    with open(os.path.join(share('aav_localization'), 'config', 'navsat.yaml')) as f:
        datum = yaml.safe_load(f)['navsat_transform']['ros__parameters']['datum']

    speed_loop = vehicle.check_speed_loop(LaunchConfiguration('speed_loop').perform(context))
    # What every model of the car assumes: spine_sim's loop unless told (TODO 163).
    model_loop = vehicle.check_speed_loop_model(
        LaunchConfiguration('speed_loop_model').perform(context), speed_loop)
    actions = [
        # The car's own software, as car.launch.py starts it.
        include('aav_description', 'description.launch.py'),
        include('aav_command_mux', 'mux.launch.py', geofence_lock='true'),
        # The models' loop (spine_sim's unless told), for ego's trace (TODO 163).
        include('aav_ego', 'ego.launch.py', speed_loop=model_loop),
        # ego's commands go to spine_bridge (source bench) only while everything is BENCH.
        include('aav_io_mux', 'io_mux.launch.py', target='bench', source='bench'),
        include('aav_scripted', 'scripted.launch.py'),
        include('aav_drive_to_point', 'drive_to_point.launch.py',
                drive_to_point_impl=vehicle.check_drive_to_point_impl(
                    LaunchConfiguration('drive_to_point_impl').perform(context))),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick',
             parameters=[os.path.join(share('aav_sensors'), 'config', 'imu_filter.yaml')],
             remappings=[('imu/data_raw', '/sensors/imu/data_raw'),
                         ('imu/data', '/sensors/imu/data'),
                         ('imu/mag', '/sensors/imu/mag')]),
        include('aav_localization', 'localization.launch.py', gps='true'),
        Node(package=bridges.package(LaunchConfiguration('bridge').perform(context)),
             executable=bridges.EXECUTABLE, name='spine_bridge', namespace='/',
             parameters=[os.path.join(share('aav_spine'), 'config', 'spine.yaml'),
                         vehicle.spine_params(v),
                         # /system/source: the spine is spine_sim, not the real board.
                         {'source': 'bench'}]),

        # The stand-ins for the hardware.
        Node(package='aav_spine_sim', executable='spine_sim', name='spine_sim',
             parameters=[{'wheelbase': float(v['wheelbase']), 'track': float(v['track']),
                          'wheel_radius': float(v['wheel_diameter']) / 2,
                          'max_steering': float(v['max_steering_angle']),
                          'speed_loop': speed_loop,
                          **start}]),
        Node(package='aav_bench', executable='mock_imu', name='mock_imu',
             parameters=[bench_config]),
        Node(package='aav_bench', executable='mock_gps', name='mock_gps',
             parameters=[bench_config, {'datum': [float(d) for d in datum]}]),
        Node(package='aav_bench', executable='mock_battery', name='mock_battery',
             parameters=[bench_config]),
        Node(package='aav_bench', executable='mock_camera', name='mock_camera',
             parameters=[bench_config] + road),
        Node(package='aav_bench', executable='mock_radar', name='mock_radar',
             parameters=[bench_config, {'obstacle': flag['obstacle']}]),
        include('aav_radar_clusters', 'radar_clusters.launch.py', minimum_points='5'),
    ]
    if flag['lidar']:
        # The pretend lidar sends to 2369, not the real one's 2368: the two never mix.
        actions += [
            Node(package='aav_bench', executable='mock_lidar', name='mock_lidar',
                 parameters=[bench_config, {'obstacle': flag['obstacle'], 'port': 2369}]),
            include('aav_sensors', 'sensors.launch.py', imu='false', gps='false',
                    lidar='true', lidar_ip='127.0.0.1', lidar_port='2369'),
        ]
    if flag['nav']:
        # mock_radar always runs, so our local costmap always reads the radar.
        actions.append(include('aav_navigation', 'navigation.launch.py',
                               map=os.path.join(share('aav_navigation'), 'maps', 'empty.yaml'),
                               radar='true', lidar=str(flag['lidar']).lower(),
                               camera_obstacles=flag['camera_obstacles'],
                               reversing=str(flag['reversing']).lower(),
                               plan_around=str(flag['plan_around']).lower(),
                               speed_loop=model_loop,
                               stop_hold_impl=vehicle.check_stop_hold_impl(
                                   LaunchConfiguration('stop_hold_impl').perform(context))))
    # The sensor watchdog guards navigation and the AI slot as on the car, always:
    # the mux's sensors lock needs its heartbeat (aav_command_mux mux.launch.py sensor_lock).
    actions.append(include('aav_safety', 'watchdog.launch.py',
                           inputs='odom,imu,gps,local,global,radar'
                           + (',lidar' if flag['lidar'] else '')))
    # The goal leash (TODO 116): a laptop driver that goes quiet loses its goals, as on the car.
    leash = vehicle.check_goal_leash_impl(LaunchConfiguration('goal_leash_impl').perform(context))
    actions.append(Node(package=vehicle.GOAL_LEASH_IMPLS[leash], executable='goal_leash',
                        name='goal_leash'))
    # The geofence holds every program near its edge, as on the car, always: the mux's
    # geofence lock needs its heartbeat (mux.launch.py geofence_lock).
    actions.append(include('aav_safety', 'geofence.launch.py', backend='spine',
                           radius=LaunchConfiguration('geofence_radius').perform(context),
                           centre_x=0.0, centre_y=start['start_y'], max_speed='',
                           geofence_impl=vehicle.check_geofence_impl(
                               LaunchConfiguration('geofence_impl').perform(context))))
    teleop = LaunchConfiguration('teleop').perform(context).lower()
    if teleop not in ('false', 'true', 'stand_in'):
        raise ValueError(f'teleop:={teleop}: use true (a joystick here), stand_in or false')
    if teleop != 'false':
        # stand_in: the joystick path without joy_node; a script plays the pad on
        # /drivers/joy/joy (bench_check.py's live, frozen and silent pad).
        actions.append(include('aav_teleop', 'teleop.launch.py',
                               joy_device=str(teleop == 'true').lower()))
    if flag['perception']:
        actions += [
            include('aav_detector', 'detector.launch.py', labels='stop sign'),
            Node(package=vehicle.STOP_SIGN_IMPLS[vehicle.check_stop_sign_impl(
                LaunchConfiguration('stop_sign_impl').perform(context))],
                executable='stop_sign_watcher', name='stop_sign_watcher'),
        ]
    if flag['camera_obstacles']:
        actions.append(Node(package='aav_detector', executable='camera_obstacles',
                            name='camera_obstacles',
                            parameters=[os.path.join(share('aav_detector'), 'config',
                                                     'camera_obstacles.yaml')]))
    if lane == 'true':
        actions.append(include('aav_lane_finder', 'lane_finder.launch.py',
                               lane_finder_impl=vehicle.check_lane_finder_impl(
                                   LaunchConfiguration('lane_finder_impl').perform(context))))
    if flag['hud']:
        actions.append(include('aav_hud', 'hud.launch.py'))
    if flag['foxglove']:
        # This computer only, like the rest of the bench (tools/ros.sh --local), with the
        # pretend camera's small preview, as camera_stream makes on the car.
        actions.append(include('aav_bringup', 'foxglove.launch.py',
                               foxglove_address='127.0.0.1', foxglove_port='8765',
                               use_sim_time='false', foxglove_control='true',
                               foxglove_topics='wifi', foxglove_preview='front_left'))
    if flag['rviz']:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(share('aav_bringup'), 'rviz', 'bench.rviz')]))
    if flag['arm']:
        # Once the safety gate is OPEN: spine_bridge refuses before.
        actions.append(TimerAction(period=12.0, actions=[Node(
            package=vehicle.ARM_WHEN_OPEN_IMPLS[vehicle.check_arm_when_open_impl(
                LaunchConfiguration('arm_when_open_impl').perform(context))],
            executable='arm_when_open', name='arm_when_open',
            output='screen')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('nav', default_value='true',
                              description='navigation (ours: the planner, follower, costmaps, '
                                          'stop zone)'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='true if a joystick is plugged into this computer; '
                                          'stand_in: the joystick path, a script plays '
                                          'the pad (no joy_node)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('stop_sign_impl', default_value=vehicle.DEFAULT_STOP_SIGN_IMPL,
                              description='rust (aav_stop_sign_rs) or python (aav_detector): '
                                          'the stop-sign watcher, with perception'),
        DeclareLaunchArgument('perception', default_value='false',
                              description='the object detector and the stop-sign watcher '
                                          '(the ml environment)'),
        DeclareLaunchArgument('geofence_impl', default_value=vehicle.DEFAULT_GEOFENCE_IMPL,
                              description='rust (aav_safety_rs; the Python removed 20 Nov): '
                                          'the geofence'),
        DeclareLaunchArgument('stop_hold_impl', default_value=vehicle.DEFAULT_STOP_HOLD_IMPL,
                              description='rust (aav_safety_rs; the Python removed 20 Nov): '
                                          'stop_hold, with navigation'),
        DeclareLaunchArgument('goal_leash_impl', default_value=vehicle.DEFAULT_GOAL_LEASH_IMPL,
                              description='rust (aav_safety_rs; the Python removed 20 Nov): '
                                          'the goal leash'),
        DeclareLaunchArgument('drive_to_point_impl',
                              default_value=vehicle.DEFAULT_DRIVE_TO_POINT_IMPL,
                              description='rust or python: which point_driver drives to a '
                                          'point (aav_drive_to_point_rs, or aav_drive_to_point)'),
        DeclareLaunchArgument('arm_when_open_impl',
                              default_value=vehicle.DEFAULT_ARM_WHEN_OPEN_IMPL,
                              description='rust or python: which arm_when_open arms the spine '
                                          "(aav_spine_bridge's, or aav_spine's)"),
        DeclareLaunchArgument('lane_finder_impl',
                              default_value=vehicle.DEFAULT_LANE_FINDER_IMPL,
                              description='rust or python: which lane_finder finds the lane '
                                          'lines (aav_lane_finder_rs, or aav_lane_finder)'),
        DeclareLaunchArgument('lane', default_value='false',
                              description='true: the lane finder, on '
                                          '/perception/lane/geometry'),
        DeclareLaunchArgument('hud', default_value='false',
                              description='the HUD window (aav_hud, as pixi run hud); '
                                          'needs a screen'),
        DeclareLaunchArgument('arm', default_value='true',
                              description='arm the spine after 12 s'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='foxglove_bridge on 127.0.0.1:8765, for Lichtblick '
                                          'or Foxglove (foxglove.launch.py)'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description="m: start the car this far left of the road's centre"),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='rad: start the car turned this far left of the road '
                                          '(the EKFs still start facing east)'),
        DeclareLaunchArgument('bend_radius', default_value='',
                              description='m: bend the bench road from 30 m on, + left, - right '
                                          '(empty: bench.yaml, straight)'),
        DeclareLaunchArgument('obstacle', default_value='false',
                              description="a box in the car's lane, 15 m ahead, that the "
                                          'pretend radar sees'),
        DeclareLaunchArgument('lidar', default_value='false',
                              description="the roof lidar: mock_lidar's packets through the "
                                          'stock velodyne driver, into the costmap'),
        DeclareLaunchArgument('camera_obstacles', default_value='false',
                              description="the detectors' boxes into the local costmap"),
        DeclareLaunchArgument('reversing', default_value='true',
                              description='the car may drive backwards, as on the car'),
        DeclareLaunchArgument('plan_around', default_value='true',
                              description='true: the planner plans round what the radar '
                                          'and lidar see; false: the map only, so the car '
                                          'stops for it (bench-check --obstacle)'),
        DeclareLaunchArgument('bridge', default_value=bridges.DEFAULT,
                              description='spine_bridge in rust (aav_spine_bridge) or python '
                                          '(aav_spine)'),
        DeclareLaunchArgument('speed_loop', default_value='default',
                              description="spine_sim's speed loop profile: default (today's "
                                          'gains), smooth (TODO 27), lqi (TODO 3) or '
                                          'firm (TODO 163)'),
        DeclareLaunchArgument('speed_loop_model', default_value='',
                              description='the loop every model of the car assumes; '
                                          "empty: speed_loop's (TODO 163)"),
        DeclareLaunchArgument('geofence_radius', default_value='100.0',
                              description='m: the geofence, a circle round where the car '
                                          'first stands still (the car: 30)'),
        OpaqueFunction(function=launch_setup),
    ])
