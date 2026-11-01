"""
Nav2 as one more driver: goals in, /drivers/nav/cmd_vel (TwistStamped) out to twist_mux.

  ros2 launch aav_navigation navigation.launch.py [map:=<path to map yaml>]
      [radar:=false] [lidar:=false] [collision_monitor:=<radar or lidar>]
      [camera_obstacles:=false] [ghost:=false]
      [reversing:=false] [max_speed:=] [controller:=mppi] [planner:=hybrid]
      [plan_around:=true]
      [tracker:=none] [live_swap:=false] [amcl:=false] [amcl_x:=0.0] [amcl_y:=0.0]
      [amcl_yaw:=0.0] [amcl_map:=]
      [use_sim_time:=false]
      [autostart:=true]

Starts only the Nav2 servers a car needs (not nav2_bringup's full list: no docking or
route server, and AMCL only as a trial, amcl:=true, since robot_localization does
localization):

  map_server        the map (default: maps/empty.yaml, everything free)
  planner_server    SmacPlannerHybrid: drivable paths for a car; planner:= for another
                    (below)
  controller_server MPPI, with its Ackermann model -> cmd_vel_nav; controller:=rpp for
                    Regulated Pure Pursuit instead (docs/path-followers.md)
  behavior_server   wait, backup, drive_on_heading (no spin)
  bt_navigator      navigate_to_pose, navigate_through_poses (behavior_trees/*_car*.xml)
  waypoint_follower follow_waypoints, follow_gps_waypoints (lat/lon via /fromLL)
  velocity_smoother cmd_vel_nav -> /drivers/nav/cmd_vel
                    (or -> /control/cmd_vel_smoothed -> collision_monitor)
  collision_monitor /control/cmd_vel_smoothed -> /drivers/nav/cmd_vel, stopping for radar
                    or lidar points ahead; its state and stop zone under
                    /safety/collision_monitor/
  stop_hold         with collision_monitor (aav_safety): holds its stops until the zone
                    stays clear for 1 s, through twist_mux's stop_zone lock
                    (/safety/stop_hold/lock), so the zone's edge can't flicker it
  lidar_scan        with lidar:=true (aav_lidar): the roof lidar's
                    cloud -> /sensors/lidar/scan (what stands between the road and just
                    over the roof, all the way round), for the costmap and collision_monitor
  lifecycle_manager brings them all up

Trials (docs/trials.md), each beside the default and scored by pixi run controller-compare:
  planner:=hybrid|lattice|smac2d|navfn|theta_star  the planner behind the plugin name
                          GridBased (the behaviour trees never change; docs/controllers.md,
                          Planners). hybrid, the default: SmacPlannerHybrid, Dubins or
                          Reeds-Shepp. lattice: SmacPlannerLattice on our own primitives
                          (config/lattice_ackermann.json, pixi run lattice-primitives), for
                          the car's turning radius and the map's cells (refused if either
                          differs), backwards too with reversing:=true. smac2d, navfn,
                          theta_star: grid paths that ignore the turning radius, so the
                          follower must round their corners; car.launch.py refuses them
  tracker:=pure_pursuit|stanley|lqr|mpc|mpc_full  aav_track's track node follows the path instead
                          of controller_server: it serves follow_path (bt_navigator's
                          FollowPath) and publishes cmd_vel_nav; controller_server keeps
                          running (its costmaps, the trees' clear calls) with its own action
                          renamed follow_path_unused. The same planner, smoother,
                          collision_monitor and twist_mux slot. Forward only: refused with
                          reversing:=true. tracker:=none (the default): controller_server
  live_swap:=true         the tracker can change while it runs (`ros2 param set /track
                          tracker lqr`); sim.launch.py sets it, the car never does
  amcl:=true              stock nav2_amcl on the map and /sensors/lidar/scan (so only with
                          lidar:=true), from amcl_x, amcl_y, amcl_yaw (map frame), with its
                          own lifecycle manager. tf_broadcast false: ekf_global keeps
                          map -> odom; AMCL's pose, /localization/amcl/pose, is only scored
  amcl_map:=<yaml>        AMCL's own map (finer, obstacles as outlines: maps/README.md),
                          from amcl_map_server on /localization/amcl/map; empty: Nav2's /map

Obstacles, from where the sensors are:
  radar:=true             where a radar publishes /sensors/radar/points (the bench; the car
                          with its radar part; Gazebo's road world): the local costmap's
                          obstacle layer reads it, so Nav2 stops for what the radar sees
  lidar:=true             where a lidar publishes /sensors/lidar/points (Gazebo's road and
                          test_lot worlds; the car's VLP-16, where car.launch.py's lidar
                          part runs): lidar_scan flattens it, the local
                          costmap's lidar_layer reads it, and collision_monitor too
  collision_monitor       radar or lidar unless set: it stops the car for radar or lidar
                          points in the stop zone ahead, whatever the costmap says. A
                          silent radar or lidar stops Nav2 too
  camera_obstacles:=true  the local costmap's camera layer reads aav_detector camera_obstacles
                          (the detectors' boxes on the road). Off until the cameras are
                          calibrated (Q22)
  camera_guard:=true      collision_monitor stops for them too, and when they go silent:
                          only where a detector really reads the cameras for people (the
                          car, with the rear camera: car.launch.py sets it)
  ghost:=true             a pretend person (aav_ghost ghost_obstacle, docs/ghost-obstacles.md):
                          its own costmap layer and collision_monitor source, so it only
                          adds. No ghost until someone places one: pixi run ghost ahead 10

Reversing (docs/learn-nav2.md): nothing senses what is behind the car, so it is off unless
asked for:
  reversing:=false        forward only: Dubins paths, no reversing in the controller, the
                          velocity smoother's lowest speed 0, the trees without BackUp
  reversing:=true         Reeds-Shepp paths (forward and back), Regulated Pure Pursuit
                          follows them backwards, down to -max_reverse_speed, and the trees
                          back up 1 m when stuck (behavior_trees/*_car_reversing.xml).
                          Gazebo and the bench turn it on; car.launch.py leaves it off

The car's footprint, turning radius and reverse speed come from
aav_description/config/vehicle.yaml.
Send a goal: RViz "2D Goal Pose" (publishes /goal_pose), or
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 20.0, y: 5.0}}}}"
"""

import atexit
import json
import os
import tempfile

from aav_description import vehicle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import yaml


# The local costmap's parameters, as a whole path for RewrittenYaml: a leaf key such as
# `enabled` would change every key of that name in nav2.yaml.
LOCAL = 'local_costmap.local_costmap.ros__parameters.'


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def tuned_copy(source, v, reversing, max_speed, radar, lidar, ghost=False,
               controller=vehicle.DEFAULT_CONTROLLER, plan_around=True, camera=False,
               planner=vehicle.DEFAULT_PLANNER, lattice_file=None):
    """Return a temporary copy of nav2.yaml, tuned by vehicle.nav2_tuned (no ROS there)."""
    with open(source) as f:
        cfg = vehicle.nav2_tuned(yaml.safe_load(f), v, reversing, max_speed, radar, lidar,
                                 ghost, controller, plan_around, camera, planner,
                                 lattice_file)
    out = tempfile.NamedTemporaryFile('w', prefix='nav2_', suffix='.yaml', delete=False)
    with out:
        yaml.safe_dump(cfg, out)
    # The nodes read it as they start; gone when the launch ends (1000 were left, 22 Oct).
    atexit.register(os.remove, out.name)
    return out.name


def lattice_for(share, v, map_yaml):
    """
    Return the lattice primitives' path, checked against the car and the map.

    SmacPlannerLattice measures its moves in the file's cells: on a map with other cells,
    or for a car that now turns wider, it would plan moves the car can't make. Refuse
    rather than start it so (pixi run lattice-primitives makes a new file).
    """
    path = f'{share}/config/lattice_ackermann.json'
    with open(path) as f:
        meta = json.load(f)['lattice_metadata']
    with open(map_yaml) as f:
        resolution = yaml.safe_load(f)['resolution']
    wrong = vehicle.lattice_problems(meta, v, resolution)
    if wrong:
        raise ValueError(f'planner:=lattice: {path}: ' + '; '.join(wrong) +
                         '. Run pixi run lattice-primitives, or pick another planner')
    return path


def nodes(context):
    share = FindPackageShare('aav_navigation').perform(context)
    use_sim_time = flag(context, 'use_sim_time')
    radar = flag(context, 'radar')
    lidar = flag(context, 'lidar')
    ghost = flag(context, 'ghost')
    # Empty: wherever a sensor (or the ghost) can feed it.
    monitor = LaunchConfiguration('collision_monitor').perform(context).strip().lower()
    camera = flag(context, 'camera_obstacles')
    # Only where a detector feeds them: a silent source stops Nav2.
    guard = camera and flag(context, 'camera_guard')
    collision_monitor = ((radar or lidar or ghost or guard) if monitor == ''
                         else monitor == 'true')
    reversing = flag(context, 'reversing')
    tracker = vehicle.check_tracker(LaunchConfiguration('tracker').perform(context), reversing)
    amcl = flag(context, 'amcl')
    if amcl and not lidar:
        raise ValueError('amcl:=true needs lidar:=true: AMCL reads /sensors/lidar/scan')

    planner = vehicle.check_planner(LaunchConfiguration('planner').perform(context))

    cfg = vehicle.load()
    v = cfg['vehicle']
    map_yaml = LaunchConfiguration('map').perform(context)
    lattice_file = lattice_for(share, v, map_yaml) if planner == 'lattice' else None
    trees = '_car_reversing.xml' if reversing else '_car.xml'
    rewrites = vehicle.nav2_rewrites(v, reversing, planner)
    rewrites.update({
        'yaml_filename': map_yaml,
        'default_nav_to_pose_bt_xml': f'{share}/behavior_trees/navigate_to_pose{trees}',
        'default_nav_through_poses_bt_xml':
            f'{share}/behavior_trees/navigate_through_poses{trees}',
        LOCAL + 'obstacle_layer.enabled': str(radar).lower(),
        LOCAL + 'camera_layer.enabled': str(camera).lower(),
    })
    # A copy of nav2.yaml with the values RewrittenYaml can't write (lists: the velocity
    # smoother's limits, collision_monitor's sources, the costmap's layers), the
    # controller, and max_speed:= (sim.launch.py sim_speed:=). In the file, not as
    # per-node dicts after it: nav2.yaml names its nodes, and a named entry beats a later
    # dict when the launch sets a global parameter (sim.launch.py's use_sim_time).
    speed = LaunchConfiguration('max_speed').perform(context).strip()
    controller = LaunchConfiguration('controller').perform(context).strip().lower()
    source = tuned_copy(f'{share}/config/nav2.yaml', v, reversing,
                        float(speed) if speed else None, radar, lidar, ghost, controller,
                        flag(context, 'plan_around'), guard, planner, lattice_file)
    params = ParameterFile(
        RewrittenYaml(source_file=source, param_rewrites=rewrites, convert_types=True),
        allow_substs=True)

    smoothed = '/control/cmd_vel_smoothed' if collision_monitor else '/drivers/nav/cmd_vel'
    servers = [
        ('nav2_map_server', 'map_server', []),
        ('nav2_planner', 'planner_server', []),
        # With a trial tracker, it keeps running but its FollowPath goes unused: rcl_action
        # resolves action names with the node's remappings, as topics are.
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')] + (
            [('follow_path', 'follow_path_unused')] if tracker != 'none' else [])),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', smoothed)]),
    ]
    if collision_monitor:
        # Its marker topic is private (~/collision_points_marker): move it next to the
        # rest of what it publishes, under /safety/collision_monitor/.
        servers.append(('nav2_collision_monitor', 'collision_monitor', [
            ('/collision_monitor/collision_points_marker',
             '/safety/collision_monitor/collision_points_marker')]))

    # use_sim_time is passed on its own: RewrittenYaml only rewrites keys that already
    # exist, and nav2.yaml has none, so as a rewrite it was silently dropped.
    actions = [
        Node(package=pkg, executable=exe, name=exe, output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}],
             remappings=remaps)
        for pkg, exe, remaps in servers
    ]
    if collision_monitor:
        # Holds collision_monitor's stops (aav_safety stop_hold, twist_mux's stop_zone
        # lock): Jazzy's monitor flickers between stop and go at its zone's edge. Not a
        # lifecycle node: it locks nothing until the monitor first says STOP.
        config = FindPackageShare('aav_safety').perform(context) + '/config/stop_hold.yaml'
        actions.append(Node(package='aav_safety', executable='stop_hold', name='stop_hold',
                            output='screen',
                            parameters=[config, {'use_sim_time': use_sim_time}]))
    if lidar:
        # Not a lifecycle node: it runs from the start, Nav2's servers or not.
        actions.append(Node(package='aav_lidar', executable='lidar_scan', name='lidar_scan',
                            output='screen',
                            parameters=[params, {'use_sim_time': use_sim_time}]))
    if ghost:
        # Not a lifecycle node either. Its own file, then use_sim_time: the file doesn't
        # set it, so the dict adds it rather than losing to the file.
        config = FindPackageShare('aav_ghost').perform(context) + '/config/ghost_obstacle.yaml'
        actions.append(Node(package='aav_ghost', executable='ghost_obstacle',
                            name='ghost_obstacle', output='screen',
                            parameters=[config, {'use_sim_time': use_sim_time}]))
    if tracker != 'none':
        # Not a lifecycle node: it serves follow_path from the start. Nav2's own numbers
        # (the speed, the checks), so every follower is held to the same.
        with open(f'{share}/config/nav2.yaml') as f:
            nav2 = yaml.safe_load(f)
        ego_file = FindPackageShare('aav_ego').perform(context) + '/config/ego.yaml'
        with open(ego_file) as f:
            ego = yaml.safe_load(f)['ego']['ros__parameters']
        own = FindPackageShare('aav_track').perform(context) + '/config/track.yaml'
        actions.append(Node(
            package='aav_track', executable='track', name='track', output='screen',
            parameters=[own, vehicle.track_params(nav2, cfg, ego,
                                                  float(speed) if speed else None),
                        {'tracker': tracker, 'use_sim_time': use_sim_time,
                         'live_swap': IfCondition(
                             LaunchConfiguration('live_swap')).evaluate(context)}]))
    if amcl:
        config = f'{share}/config/amcl.yaml'
        start = {'set_initial_pose': True, 'use_sim_time': use_sim_time}
        for axis in ('x', 'y', 'yaw'):
            start[f'initial_pose.{axis}'] = float(
                LaunchConfiguration(f'amcl_{axis}').perform(context))
        amcl_map = LaunchConfiguration('amcl_map').perform(context)
        managed = ['amcl']
        if amcl_map:
            # AMCL's own map (maps/README.md: finer cells, obstacles as outlines), from a
            # map_server of its own; Nav2's planner and costmaps keep /map.
            start['map_topic'] = '/localization/amcl/map'
            managed.insert(0, 'amcl_map_server')
            actions.append(Node(
                package='nav2_map_server', executable='map_server', name='amcl_map_server',
                output='screen',
                parameters=[{'yaml_filename': amcl_map, 'topic_name': '/localization/amcl/map',
                             'frame_id': 'map', 'use_sim_time': use_sim_time}]))
        actions += [
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                 parameters=[config, start],
                 remappings=[('amcl_pose', '/localization/amcl/pose'),
                             ('particle_cloud', '/localization/amcl/particles'),
                             ('initialpose', '/localization/amcl/initial_pose')]),
            # Its own: Nav2's manager lists only its servers, and a trial that fails to
            # come up mustn't hold Nav2 down.
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'autostart': flag(context, 'autostart'),
                              'node_names': managed}]),
        ]
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': flag(context, 'autostart'),
                     'node_names': [name for _, name, _ in servers]}],
    ))
    return actions


def generate_launch_description():
    empty_map = PathJoinSubstitution([FindPackageShare('aav_navigation'), 'maps', 'empty.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=empty_map,
                              description='Map YAML (see maps/README.md)'),
        DeclareLaunchArgument('radar', default_value='false',
                              description='true where a radar publishes '
                                          '/sensors/radar/points: the '
                                          "local costmap's obstacle layer reads it"),
        DeclareLaunchArgument('lidar', default_value='false',
                              description='true where a lidar publishes '
                                          '/sensors/lidar/points: lidar_scan, the '
                                          "local costmap's lidar_layer and collision_monitor "
                                          'read it'),
        # Empty: radar, lidar or ghost. collision_monitor needs one of them, and stops Nav2
        # when one goes silent.
        DeclareLaunchArgument('collision_monitor', default_value='',
                              description='Insert collision_monitor (needs the radar, '
                                          'the lidar or the ghost); empty: wherever one runs'),
        DeclareLaunchArgument('camera_obstacles', default_value='false',
                              description="true: the local costmap's camera layer reads "
                                          '/perception/camera_obstacles'),
        DeclareLaunchArgument('camera_guard', default_value='false',
                              description='true (with camera_obstacles): collision_monitor '
                                          'stops for the camera obstacles too, and when '
                                          'they go silent'),
        DeclareLaunchArgument('ghost', default_value='false',
                              description='true: a pretend person (aav_ghost), its own '
                                          'costmap layer and collision_monitor source; '
                                          'placed with pixi run ghost'),
        DeclareLaunchArgument('max_speed', default_value='',
                              description="m/s: Nav2's top speed, if not nav2.yaml's 1.0 "
                                          '(sim.launch.py sim_speed:=)'),
        DeclareLaunchArgument('reversing', default_value='false',
                              description='true: Nav2 may drive backwards (blind: nothing '
                                          'senses behind the car). docs/learn-nav2.md'),
        DeclareLaunchArgument('controller', default_value=vehicle.DEFAULT_CONTROLLER,
                              description='mppi or rpp (Regulated Pure Pursuit): what '
                                          'follows the path. docs/path-followers.md'),
        DeclareLaunchArgument('planner', default_value=vehicle.DEFAULT_PLANNER,
                              description=', '.join(vehicle.PLANNERS) + ': the planner '
                                          'behind GridBased. hybrid and lattice keep to the '
                                          "car's turning radius; the others don't (sim and "
                                          'bench only). docs/controllers.md, Planners'),
        DeclareLaunchArgument('plan_around', default_value='true',
                              description='true: the planner sees the radar and lidar too, '
                                          'and plans round what they see; false: the map '
                                          'only, so the car stops for it (the stop tests)'),
        DeclareLaunchArgument('tracker', default_value=vehicle.DEFAULT_TRACKER,
                              description="a trial path follower in controller_server's "
                                          'place (aav_track): ' + ', '.join(vehicle.TRACKERS) +
                                          '. docs/trials.md'),
        DeclareLaunchArgument('live_swap', default_value='false',
                              description='true (sim.launch.py sets it): the tracker can be '
                                          'changed while it runs, `ros2 param set /track '
                                          'tracker lqr`. Never on the car'),
        DeclareLaunchArgument('amcl', default_value='false',
                              description='true (with lidar:=true): stock AMCL as a trial, '
                                          'scored only: /localization/amcl/pose'),
        DeclareLaunchArgument('amcl_x', default_value='0.0',
                              description="m, map frame: AMCL's start"),
        DeclareLaunchArgument('amcl_y', default_value='0.0',
                              description="m, map frame: AMCL's start"),
        DeclareLaunchArgument('amcl_yaw', default_value='0.0',
                              description="rad from east: AMCL's start heading"),
        DeclareLaunchArgument('amcl_map', default_value='',
                              description="empty: AMCL reads Nav2's /map; else a map YAML "
                                          'of its own, on /localization/amcl/map (the '
                                          "test lot's: maps/example_lot_amcl.yaml)"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=nodes),
    ])
