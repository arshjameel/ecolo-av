"""
Nav2 as one more driver: goals in, /drivers/nav/cmd_vel (TwistStamped) out to twist_mux.

  ros2 launch aav_navigation navigation.launch.py [map:=<path to map yaml>]
      [radar:=false] [lidar:=false] [collision_monitor:=<radar or lidar>]
      [camera_obstacles:=false] [ghost:=false] [voxels:=false]
      [reversing:=false] [max_speed:=] [controller:=mppi] [planner:=lattice]
      [diffusion_model:=] [diffusion_provider:=CPUExecutionProvider] [diffusion_record:=]
      [diffusion_python:=] [diffusion_smoothing:=gear]
      [plan_around:=true] [behaviour:=false] [replan_hz:=1.0]
      [tracker:=none] [costmap:=nav2] [navigator:=nav2] [smoother:=nav2] [live_swap:=false]
      [amcl:=false]
      [amcl_x:=0.0] [amcl_y:=0.0] [amcl_yaw:=0.0] [amcl_map:=]
      [use_sim_time:=false]
      [autostart:=true]

Starts only the Nav2 servers a car needs (not nav2_bringup's full list: no docking or
route server, and AMCL only as a trial, amcl:=true, since robot_localization does
localization):

  map_server        the map (default: maps/empty.yaml, everything free)
  planner_server    SmacPlannerLattice: drivable paths for a car; planner:= for another
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

Trials (docs/trials.md), each beside the default and scored by pixi run measure controllers:
  planner:=hybrid|lattice|smac2d|navfn|theta_star  the planner behind the plugin name
                          GridBased (the behaviour trees never change; docs/controllers.md,
                          Planners). hybrid, the default: SmacPlannerHybrid, Dubins or
                          Reeds-Shepp. lattice: SmacPlannerLattice on our own primitives
                          (config/lattice_ackermann.json, from
                          pixi run python3 tools/lattice_primitives.py), for the car's turning
                          radius and the map's cells (refused if either differs),
                          backwards too with reversing:=true.
                          lattice_headings:=32 takes the trial file with 32 headings
                          (lattice_ackermann_32.json). smac2d, navfn,
                          theta_star: grid paths that ignore the turning radius, so the
                          follower must round their corners; car.launch.py refuses them.
                          diffusion (aav_diffusion_planner): a model draws 16 paths, a
                          guard checks them, the lattice's path when none passes or it is
                          late. Its diffusion_planner serves compute_path_to_pose and
                          publishes /plan; planner_server runs the lattice behind it, its
                          action renamed compute_path_to_pose_lattice and its plan
                          /drivers/nav/diffusion/lattice_plan. diffusion_model:= (empty:
                          models/diffusion/planner.onnx), diffusion_provider:= (ONNX
                          Runtime's). car.launch.py refuses it
                          own (aav_planner): our own planner, Nav2's State Lattice ported to
                          Rust (the same primitives file and penalties). Its planner node
                          serves /drivers/nav/planner/compute_path_to_pose and publishes
                          /plan, and .../compute_path_through_poses; bt_navigator's two
                          planner calls are remapped to it. planner_server keeps the lattice
                          beside it, its /plan renamed /drivers/nav/planner/nav2_plan.
                          It plans on the global costmap costmap:= picks. car.launch.py
                          refuses it (docs/own-planner.md)
  tracker:=pure_pursuit|stanley|lqr|mpc|mpc_full|mpc_lag
                          aav_track's track node follows the path instead
                          (tracker:=mppi_lag: aav_mppi_lag's mppi_lag node, Nav2's MPPI
                          with the actuators' lag; docs/mppi-lag.md)
                          of controller_server: it serves follow_path (bt_navigator's
                          FollowPath) and publishes cmd_vel_nav; controller_server keeps
                          running (its costmaps, the trees' clear calls) with its own action
                          renamed follow_path_unused. The same planner, smoother,
                          collision_monitor and twist_mux slot. aav_track's trackers
                          drive forwards only: refused with reversing:=true; mppi_lag
                          reverses as Nav2's MPPI does.
                          tracker:=none (the default): controller_server
  costmap:=own            aav_costmap's costmap node (Rust, a trial): our own local and
                          global costmaps from the same nav2.yaml blocks, on
                          /drivers/nav/costmap/local and /global (and *_raw). Nav2's own
                          costmaps keep running; with tracker:=mppi_lag, mppi_lag reads our
                          local one instead of Nav2's (docs/own-costmap.md)
  navigator:=own          aav_navigator's navigator (Rust, a trial): our own navigator in
                          place of bt_navigator and waypoint_follower. It serves the same four
                          actions under /drivers/nav/navigator/ and /goal_pose, and calls
                          whichever planner, follower and costmap clears run. With
                          planner:=own costmap:=own tracker:=mppi_lag (all ours) no Nav2
                          server runs for navigation: only map_server, the velocity smoother
                          and collision_monitor (docs/own-navigator.md)
  smoother:=own           aav_smoother's car_smoother (Rust, a trial) in velocity_smoother's
                          place, under its name, parameters and topics: Nav2's speed limits,
                          and the turn rate from where the wheels can be (the steering's rate
                          and full lock, vehicle.yaml through the car model; ego's ramp).
                          Not a lifecycle node (docs/own-smoother.md)
  live_swap:=true         the tracker can change while it runs (`ros2 param set /track
                          tracker lqr`); sim.launch.py sets it, the car never does
  amcl:=true              stock nav2_amcl on the map and /sensors/lidar/scan (so only with
                          lidar:=true), from amcl_x, amcl_y, amcl_yaw (map frame), with its
                          own lifecycle manager. tf_broadcast false: ekf_global keeps
                          map -> odom; AMCL's pose, /localization/amcl/pose, is only scored
  voxels:=true            (with lidar:=true) aav_voxels' voxels cuts the cloud into 0.25 m
                          cubes between the fitted road and just over the roof
                          (/perception/voxels/cubes, and the road it saw, .../ground): the
                          local costmap's lidar_layer and collision_monitor read those
                          instead of /sensors/lidar/scan. lidar_scan then runs only for
                          amcl:=true (docs/designs/2026-10-30_voxels.md)
  behaviour:=true         aav_behaviour's behaviour node (a trial, docs/stopping.md): the
                          behaviour states, and a speed cap on Nav2 from the stopping law for
                          a stop sign, an obstacle in the lane, the fence and the goal.
                          controller_server reads /drivers/nav/speed_limit; the locks still
                          make every stop. false (the default): the stock path, no cap
  replan_hz:=0            the navigate-to-pose tree plans once a goal, not every second
                          (a test, the wiggle: TODO 149); any other number, that rate. A
                          copy of the tree, removed when the launch ends
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
               planner='hybrid', lattice_file=None, voxels=False):
    """Return a temporary copy of nav2.yaml, tuned by vehicle.nav2_tuned (no ROS there)."""
    with open(source) as f:
        cfg = vehicle.nav2_tuned(yaml.safe_load(f), v, reversing, max_speed, radar, lidar,
                                 ghost, controller, plan_around, camera, planner,
                                 lattice_file, voxels)
    out = tempfile.NamedTemporaryFile('w', prefix='nav2_', suffix='.yaml', delete=False)
    with out:
        yaml.safe_dump(cfg, out)
    # The nodes read it as they start; gone when the launch ends (1000 were left, 22 Oct).
    atexit.register(os.remove, out.name)
    return out.name


def replanning_copy(tree, hz):
    """Return a temporary copy of a to-pose tree that plans again at hz (0: once a goal)."""
    with open(tree) as f:
        text = vehicle.replan_tree(f.read(), hz)
    out = tempfile.NamedTemporaryFile('w', prefix='tree_', suffix='.xml', delete=False)
    with out:
        out.write(text)
    atexit.register(os.remove, out.name)
    return out.name


def lattice_for(share, v, map_yaml, headings=vehicle.LATTICE_HEADINGS[0]):
    """
    Return the lattice primitives' path, checked against the car and the map.

    SmacPlannerLattice measures its moves in the file's cells: on a map with other cells,
    or for a car that now turns wider, it would plan moves the car can't make. Refuse
    rather than start it so (pixi run python3 tools/lattice_primitives.py makes a new file).
    """
    path = f'{share}/config/{vehicle.lattice_file_name(headings)}'
    with open(path) as f:
        meta = json.load(f)['lattice_metadata']
    with open(map_yaml) as f:
        resolution = yaml.safe_load(f)['resolution']
    wrong = vehicle.lattice_problems(meta, v, resolution)
    if meta.get('num_of_headings') != int(headings):
        wrong.append(f"{meta.get('num_of_headings')} headings, not {headings}")
    if wrong:
        raise ValueError(f'planner:=lattice: {path}: ' + '; '.join(wrong) +
                         '. Run pixi run python3 tools/lattice_primitives.py, '
                         'or pick another planner')
    return path


DIFFUSION_WAIT_MS = 15000   # bt_navigator's wait for diffusion_planner's action (ms)


def diffusion_params(nav2, v, reversing):
    """Return the guard's numbers for diffusion_planner: the outline, turns, the goal."""
    costmap = nav2['global_costmap']['global_costmap']['ros__parameters']
    pad = float(costmap.get('footprint_padding', 0.0))
    outline = [[x + (pad if x > 0 else -pad), y + (pad if y > 0 else -pad)]
               for x, y in vehicle.footprint_points(v)]
    server = nav2['controller_server']['ros__parameters']
    goal = server[server['goal_checker_plugins'][0]]
    return {'footprint': vehicle.polygon(outline), 'reversing': reversing,
            # The teachers' own smoothed paths dip just under full lock (the design).
            'min_radius': 0.95 * vehicle.min_turning_radius(v),
            'goal_xy': float(goal['xy_goal_tolerance']),
            'goal_yaw': float(goal['yaw_goal_tolerance'])}


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
    voxels = flag(context, 'voxels')
    screen_cubes = flag(context, 'screen_cubes')
    if (voxels or screen_cubes) and not lidar:
        raise ValueError('voxels:=true needs lidar:=true: the cubes come from the lidar')
    if amcl and not lidar:
        raise ValueError('amcl:=true needs lidar:=true: AMCL reads /sensors/lidar/scan')

    planner = vehicle.check_planner(LaunchConfiguration('planner').perform(context))

    cfg = vehicle.load()
    v = cfg['vehicle']
    map_yaml = LaunchConfiguration('map').perform(context)
    headings = (LaunchConfiguration('lattice_headings').perform(context).strip() or
                vehicle.LATTICE_HEADINGS[0])
    lattice_file = (lattice_for(share, v, map_yaml, headings)
                    if vehicle.server_planner(planner) == 'lattice' else None)
    trees = '_car_reversing.xml' if reversing else '_car.xml'
    rewrites = vehicle.nav2_rewrites(v, reversing, planner)
    cube_floor = LaunchConfiguration('cube_floor').perform(context).strip()
    if cube_floor:
        rewrites['voxels.ros__parameters.min_height'] = cube_floor
    if not flag(context, 'full_costmap'):
        rewrites['global_costmap.global_costmap.ros__parameters.always_send_full_costmap'] = (
            'false')
    if planner == 'diffusion':
        # Its node is Python and starts its model too: bt_navigator, waiting 1 s for
        # compute_path_to_pose when it loads its trees, gave up before it was there (8 Nov,
        # our own simulator: every goal refused). 15 s; a server that's gone still fails a
        # goal within default_server_timeout.
        rewrites['wait_for_service_timeout'] = str(DIFFUSION_WAIT_MS)
    to_pose = f'{share}/behavior_trees/navigate_to_pose{trees}'
    replan_hz = float(LaunchConfiguration('replan_hz').perform(context) or vehicle.REPLAN_HZ)
    if replan_hz != vehicle.REPLAN_HZ:
        to_pose = replanning_copy(to_pose, replan_hz)
    rewrites.update({
        'yaml_filename': map_yaml,
        'default_nav_to_pose_bt_xml': to_pose,
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
                        flag(context, 'plan_around'), guard, planner, lattice_file, voxels)
    params = ParameterFile(
        RewrittenYaml(source_file=source, param_rewrites=rewrites, convert_types=True),
        allow_substs=True)

    smoothed = '/control/cmd_vel_smoothed' if collision_monitor else '/drivers/nav/cmd_vel'
    # planner:=diffusion: its node answers compute_path_to_pose (bt_navigator's trees ask
    # it) and publishes /plan; planner_server's lattice, renamed, is its fallback.
    diffusion = planner == 'diffusion'
    # planner:=own: aav_planner answers the trees' compute_path_to_pose and publishes /plan.
    own_planner = planner == 'own'
    # costmap:=own: aav_costmap beside Nav2's costmaps; own_local: the follower reads ours.
    costmap = vehicle.check_costmap(LaunchConfiguration('costmap').perform(context))
    own_local = costmap == 'own' and tracker == 'mppi_lag'
    # navigator:=own: aav_navigator in place of bt_navigator and waypoint_follower. Each
    # Nav2 server it no longer needs stays off: planner_server when our planner plans on
    # our global costmap, controller_server (and its local costmap) when mppi_lag reads
    # ours, behavior_server when nothing backs up (forward only) or its costmap is gone.
    navigator = vehicle.check_navigator(LaunchConfiguration('navigator').perform(context))
    own_navigator = navigator == 'own'
    nav2_planner = not (own_navigator and own_planner and costmap == 'own')
    nav2_controller = not (own_navigator and own_local)
    nav2_behaviors = not own_navigator or (reversing and nav2_controller)
    # smoother:=own: aav_smoother's car_smoother in velocity_smoother's place, under its name.
    own_smoother = vehicle.check_smoother(
        LaunchConfiguration('smoother').perform(context)) == 'own'
    servers = [
        ('nav2_map_server', 'map_server', []),
        ('nav2_planner', 'planner_server', [
            ('compute_path_to_pose', 'compute_path_to_pose_lattice'),
            ('plan', '/drivers/nav/diffusion/lattice_plan')] if diffusion else
         [('plan', vehicle.OWN_PLANNER_NAV2_PLAN)] if own_planner else []),
        # With a trial tracker, it keeps running but its FollowPath goes unused: rcl_action
        # resolves action names with the node's remappings, as topics are.
        # MPPI's chosen future (visualize: true in nav2.yaml) is the screens' faint line,
        # /drivers/nav/local_plan (aav_ego look_ahead). Regulated Pure Pursuit has none.
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')] + (
            [('follow_path', 'follow_path_unused')] if tracker != 'none' else
            [('optimal_trajectory', '/drivers/nav/local_plan')])),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        # costmap:=own with tracker:=mppi_lag: the follower reads our local costmap, so the
        # trees' recovery clears ours (aav_costmap serves Nav2's ClearEntireCostmap). The
        # global clear stays Nav2's: the planner still reads Nav2's global costmap.
        # planner:=own: the trees' ComputePathToPose goes to our planner (rcl_action
        # resolves action client names with the node's remappings too).
        ('nav2_bt_navigator', 'bt_navigator',
         ([('local_costmap/clear_entirely_local_costmap',
            '/drivers/nav/costmap/clear_entirely_local')] if own_local else []) +
         ([('compute_path_to_pose', vehicle.OWN_PLANNER_ACTION),
           ('compute_path_through_poses', vehicle.OWN_PLANNER_THROUGH_ACTION)]
          if own_planner else [])),
        ('nav2_waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', smoothed)]),
    ]
    dropped = ({'bt_navigator', 'waypoint_follower'} if own_navigator else set()) | (
        set() if nav2_planner else {'planner_server'}) | (
        set() if nav2_controller else {'controller_server'}) | (
        set() if nav2_behaviors else {'behavior_server'}) | (
        {'velocity_smoother'} if own_smoother else set())
    servers = [s for s in servers if s[1] not in dropped]
    if collision_monitor:
        # Its marker topic is private (~/collision_points_marker): move it next to the
        # rest of what it publishes, under /safety/collision_monitor/.
        servers.append(('nav2_collision_monitor', 'collision_monitor', [
            ('/collision_monitor/collision_points_marker',
             '/safety/collision_monitor/collision_points_marker')]))

    # use_sim_time is passed on its own: RewrittenYaml only rewrites keys that already
    # exist, and nav2.yaml has none, so as a rewrite it was silently dropped.
    behaviour = flag(context, 'behaviour')
    # behaviour:=true: controller_server's speed limit comes from the behaviour node. A dict
    # after the file: nav2.yaml doesn't name speed_limit_topic, so the dict wins.
    own = {'controller_server': {'speed_limit_topic': '/drivers/nav/speed_limit'}} \
        if behaviour else {}
    actions = [
        Node(package=pkg, executable=exe, name=exe, output='screen',
             parameters=[params, {'use_sim_time': use_sim_time, **own.get(exe, {})}],
             remappings=remaps)
        for pkg, exe, remaps in servers
    ]
    if behaviour:
        # Not a lifecycle node; respawns, so a dead one comes back in DRIVE (sending 0.0).
        with open(f'{share}/config/nav2.yaml') as f:
            nav2 = yaml.safe_load(f)
        ego_file = FindPackageShare('aav_ego').perform(context) + '/config/ego.yaml'
        with open(ego_file) as f:
            ego = yaml.safe_load(f)['ego']['ros__parameters']
        actions.append(Node(
            package='aav_behaviour', executable='behaviour', name='behaviour',
            output='screen', respawn=True, respawn_delay=1.0,
            parameters=[vehicle.behaviour_params(nav2, cfg, ego,
                                                 float(speed) if speed else None, controller),
                        {'obstacles': lidar and not voxels, 'use_sim_time': use_sim_time,
                         # Every navigator action: a goal running on any of them.
                         'goal_status_topics': [
                             vehicle.navigator_action(navigator, a) + '/_action/status'
                             for a in vehicle.NAVIGATOR_ACTIONS]}]))
    if collision_monitor:
        # Holds collision_monitor's stops (aav_safety stop_hold, twist_mux's stop_zone
        # lock): Jazzy's monitor flickers between stop and go at its zone's edge. Not a
        # lifecycle node: it locks nothing until the monitor first says STOP.
        config = FindPackageShare('aav_safety').perform(context) + '/config/stop_hold.yaml'
        actions.append(Node(package='aav_safety', executable='stop_hold', name='stop_hold',
                            output='screen',
                            parameters=[config, {'use_sim_time': use_sim_time}]))
    if lidar and (amcl or not voxels):
        # Not a lifecycle node: it runs from the start, Nav2's servers or not. With
        # voxels:=true only AMCL still reads its scan.
        actions.append(Node(package='aav_lidar', executable='lidar_scan', name='lidar_scan',
                            output='screen',
                            parameters=[params, {'use_sim_time': use_sim_time}]))
    if voxels or screen_cubes:
        # The cubes, for the costmap and collision_monitor (voxels:=true), or only for the
        # developer window (screen_cubes:=true: Nav2 stays on the flat scan). Not a
        # lifecycle node.
        actions.append(Node(package='aav_voxels', executable='voxels', name='voxels',
                            output='screen',
                            parameters=[params, {'use_sim_time': use_sim_time}]))
    if ghost:
        # Not a lifecycle node either. Its own file, then use_sim_time: the file doesn't
        # set it, so the dict adds it rather than losing to the file.
        config = FindPackageShare('aav_ghost').perform(context) + '/config/ghost_obstacle.yaml'
        actions.append(Node(package='aav_ghost', executable='ghost_obstacle',
                            name='ghost_obstacle', output='screen',
                            parameters=[config, {'use_sim_time': use_sim_time}]))
    if costmap == 'own':
        # aav_costmap (Rust): our own local and global costmaps, beside Nav2's, from the
        # same tuned nav2.yaml (the same layers and sources). Not a lifecycle node.
        with open(source) as f:
            blocks = vehicle.costmap_params(yaml.safe_load(f), v, radar, camera)
        actions.append(Node(
            package='aav_costmap', executable='costmap', name='costmap', output='screen',
            parameters=[{'local': json.dumps(blocks['local']),
                         'global': json.dumps(blocks['global']),
                         'use_sim_time': use_sim_time}]))
    if own_planner:
        # aav_planner (Rust): Nav2's lattice ported, from the same tuned Lattice block, the
        # car's outline and the global costmap's inflation. Not a lifecycle node.
        with open(source) as f:
            settings = vehicle.planner_params(yaml.safe_load(f), v, costmap)
        topic, updates, latched = vehicle.GLOBAL_COSTMAP_TOPICS[costmap]
        actions.append(Node(
            package='aav_planner', executable='planner', name='planner', output='screen',
            parameters=[{'settings': json.dumps(settings), 'costmap_topic': topic,
                         'costmap_updates_topic': updates,
                         'costmap_latched': str(latched).lower(),
                         'use_sim_time': use_sim_time}]))
    if own_navigator:
        # aav_navigator (Rust): the trees' logic and waypoint_follower's, calling whichever
        # planner, follower and costmaps run. Not a lifecycle node.
        clear_local = ((['/drivers/nav/costmap/clear_entirely_local'] if costmap == 'own'
                        else []) +
                       (['/local_costmap/clear_entirely_local_costmap'] if nav2_controller
                        else []))
        clear_global = ((['/drivers/nav/costmap/clear_entirely_global'] if costmap == 'own'
                         else []) +
                        (['/global_costmap/clear_entirely_global_costmap'] if nav2_planner
                         else []))
        navsat = (FindPackageShare('aav_localization').perform(context) +
                  '/config/navsat.yaml')
        actions.append(Node(
            package='aav_navigator', executable='navigator', name='navigator',
            output='screen',
            parameters=[{
                'settings': json.dumps(vehicle.navigator_params(v, reversing)),
                'planner_action': (vehicle.OWN_PLANNER_ACTION if own_planner
                                   else '/compute_path_to_pose'),
                'planner_through_action': (vehicle.OWN_PLANNER_THROUGH_ACTION if own_planner
                                           else '/compute_path_through_poses'),
                'follow_action': '/follow_path',
                'clear_local': ','.join(clear_local),
                'clear_global': ','.join(clear_global),
                'backup_action': '/backup' if reversing and nav2_behaviors else '',
                'navsat_file': navsat,
                'use_sim_time': use_sim_time}]))
    if own_smoother:
        # aav_smoother (Rust): Nav2's velocity_smoother's block from the same tuned file
        # (its node name), the car from vehicle.yaml through the car model, ego's steering
        # ramp. Not a lifecycle node.
        with open(FindPackageShare('aav_ego').perform(context) + '/config/ego.yaml') as f:
            ego = yaml.safe_load(f)['ego']['ros__parameters']
        actions.append(Node(
            package='aav_smoother', executable='car_smoother', name='velocity_smoother',
            output='screen',
            parameters=[params, vehicle.smoother_params(cfg, ego, vehicle.config_path()),
                        {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', smoothed)]))
    if tracker != 'none':
        # Not a lifecycle node: it serves follow_path from the start. Nav2's own numbers
        # (the speed, the checks), so every follower is held to the same.
        with open(f'{share}/config/nav2.yaml') as f:
            nav2 = yaml.safe_load(f)
        ego_file = FindPackageShare('aav_ego').perform(context) + '/config/ego.yaml'
        with open(ego_file) as f:
            ego = yaml.safe_load(f)['ego']['ros__parameters']
        max_speed = float(speed) if speed else None
        if tracker == 'mppi_lag':
            # aav_mppi_lag (Rust): Nav2's MPPI with the actuators' lag in its model. nav2.yaml's
            # MPPI block and checks, vehicle.yaml's actuators, ego.yaml's ramps.
            own = FindPackageShare('aav_mppi_lag').perform(context) + '/config/mppi_lag.yaml'
            actions.append(Node(
                package='aav_mppi_lag', executable='mppi_lag', name='mppi_lag',
                output='screen',
                parameters=[own, vehicle.mppi_lag_params(nav2, cfg, ego, max_speed,
                                                         reversing),
                            {'use_sim_time': use_sim_time,
                             'costmap_topic': vehicle.COSTMAP_RAW_TOPICS[costmap]}]))
        else:
            own = FindPackageShare('aav_track').perform(context) + '/config/track.yaml'
            actions.append(Node(
                package='aav_track', executable='track', name='track', output='screen',
                parameters=[own, vehicle.track_params(nav2, cfg, ego, max_speed),
                            {'tracker': tracker, 'use_sim_time': use_sim_time,
                             'live_swap': IfCondition(
                                 LaunchConfiguration('live_swap')).evaluate(context)}]))
    if diffusion:
        # Not a lifecycle node: it serves from the start, and falls back to the lattice
        # until the lattice answers. The guard's numbers from the car and Nav2's files.
        with open(f'{share}/config/nav2.yaml') as f:
            nav2 = yaml.safe_load(f)
        actions.append(Node(
            package='aav_diffusion_planner', executable='diffusion_planner',
            name='diffusion_planner', output='screen',
            parameters=[diffusion_params(nav2, v, reversing), {
                'model': LaunchConfiguration('diffusion_model').perform(context),
                'provider': LaunchConfiguration('diffusion_provider').perform(context),
                'record': LaunchConfiguration('diffusion_record').perform(context),
                'learn_python': LaunchConfiguration('diffusion_python').perform(context),
                'smoothing': LaunchConfiguration('diffusion_smoothing').perform(context),
                'use_sim_time': use_sim_time}]))
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
        DeclareLaunchArgument('cube_floor', default_value='',
                              description="m above the road: the cubes' floor (empty: the "
                                          "car's obstacle height)"),
        DeclareLaunchArgument('full_costmap', default_value='true',
                              description='false: the global costmap goes out once, then only '
                                          'its changes. For a map too big to send every second'),
        DeclareLaunchArgument('screen_cubes', default_value='false',
                              description='true (with lidar:=true): run aav_voxels for the '
                                          "developer window; Nav2 doesn't read its cubes"),
        DeclareLaunchArgument('voxels', default_value='false',
                              description='true (with lidar:=true): the trial, aav_voxels: '
                                          "the costmap's lidar_layer and collision_monitor "
                                          'read 0.25 m cubes instead of the flat scan'),
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
        DeclareLaunchArgument('diffusion_model', default_value='',
                              description='with planner:=diffusion: the model (.onnx); '
                                          'empty: models/diffusion/planner.onnx'),
        DeclareLaunchArgument('diffusion_provider', default_value='CPUExecutionProvider',
                              description="with planner:=diffusion: ONNX Runtime's engine "
                                          '(CPUExecutionProvider, CoreMLExecutionProvider, '
                                          'CUDAExecutionProvider, TensorrtExecutionProvider)'),
        DeclareLaunchArgument('diffusion_smoothing', default_value='gear',
                              description="with planner:=diffusion: gear (each gear's run "
                                          'smoothed on its own) or average (2 Nov)'),
        DeclareLaunchArgument('diffusion_python', default_value='',
                              description="with planner:=diffusion: the model worker's "
                                          "Python (empty: Pixi's learn environment)"),
        DeclareLaunchArgument('diffusion_record', default_value='',
                              description="with planner:=diffusion: a folder for each plan's "
                                          'inputs, to replay offline (training/replay.py); '
                                          'empty: none'),
        DeclareLaunchArgument('lattice_headings',
                              default_value=str(vehicle.LATTICE_HEADINGS[0]),
                              description='with planner:=lattice: 16 headings (the default '
                                          'file) or 32 (a trial: '
                                          'config/lattice_ackermann_32.json)'),
        DeclareLaunchArgument('plan_around', default_value='true',
                              description='true: the planner sees the radar and lidar too, '
                                          'and plans round what they see; false: the map '
                                          'only, so the car stops for it (the stop tests)'),
        DeclareLaunchArgument('replan_hz', default_value=str(vehicle.REPLAN_HZ),
                              description=('how often the navigate-to-pose tree plans again '
                                           "(Hz; Nav2's own 1.0); 0: once a goal, a test "
                                           '(the wiggle, TODO 149)')),
        DeclareLaunchArgument('behaviour', default_value='false',
                              description='true: aav_behaviour, the behaviour states and a '
                                          'speed cap on Nav2 (a trial, docs/stopping.md)'),
        DeclareLaunchArgument('costmap', default_value=vehicle.DEFAULT_COSTMAP,
                              description='nav2 (the default) or own: aav_costmap, our own '
                                          "costmaps beside Nav2's, on /drivers/nav/costmap/"
                                          '*; tracker:=mppi_lag then reads its local one. '
                                          'docs/own-costmap.md'),
        DeclareLaunchArgument('navigator', default_value=vehicle.DEFAULT_NAVIGATOR,
                              description='nav2 (the default: bt_navigator and '
                                          'waypoint_follower) or own: aav_navigator, our own, '
                                          'on /drivers/nav/navigator/*; car.launch.py refuses '
                                          'it (docs/own-navigator.md)'),
        DeclareLaunchArgument('smoother', default_value=vehicle.DEFAULT_SMOOTHER,
                              description="nav2 (the default: Nav2's velocity_smoother) or "
                                          "own: aav_smoother's car_smoother in its place, "
                                          'the turn rate from where the wheels can be (sim; '
                                          'car.launch.py always passes nav2: '
                                          'docs/own-smoother.md)'),
        DeclareLaunchArgument('tracker', default_value=vehicle.DEFAULT_TRACKER,
                              description="a trial path follower in controller_server's "
                                          'place (aav_track; mppi_lag: aav_mppi_lag): ' +
                                          ', '.join(vehicle.TRACKERS) +
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
