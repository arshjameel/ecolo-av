"""
Navigation as one more driver: goals in, /drivers/nav/cmd_vel (TwistStamped) out to twist_mux.

  ros2 launch aav_navigation navigation.launch.py [map:=<path to map yaml>]
      [radar:=false] [lidar:=false] [stop_zone:=<radar or lidar>]
      [camera_obstacles:=false] [ghost:=false] [voxels:=false] [voxels_impl:=python]
      [kerbs:=false] [lidar_impl:=rust] [things_impl:=rust] [stop_hold_impl:=rust]
      [reversing:=false] [max_speed:=] [planner:=own] [planner_search:=lattice]
      [diffusion_model:=] [diffusion_provider:=CPUExecutionProvider] [diffusion_record:=]
      [diffusion_python:=] [diffusion_smoothing:=gear] [diffusion_impl:=python]
      [plan_around:=true] [behaviour:=false] [behaviour_impl:=rust] [things:=false]
      [replan_hz:=1.0] [tracker:=mppi_lag] [live_swap:=false]
      [map_correction:=newest] [speed_loop:=default]
      [amcl:=false] [amcl_impl:=rust] [amcl_x:=0.0] [amcl_y:=0.0] [amcl_yaw:=0.0]
      [amcl_map:=]
      [use_sim_time:=false] [autostart:=true]

Our own nodes only (Nav2 deleted, 17 Nov, TODO 184; its scores:
docs/league-results-2026-11-16.md and docs/history/nav2-as-it-was.md). None is a
lifecycle node: each respawns 1 s after it dies, and while one is down the car stops.

  map_server        the map, latched on /map (aav_costmap's, as Nav2's read it)
  costmap           our local and global costmaps (aav_costmap), Nav2's layers ported,
                    on /drivers/nav/costmap/*
  planner           our planner (aav_planner): the State Lattice, or Hybrid-A*
                    (planner_search:=hybrid), on the global costmap; /plan
  navigator         our navigator (aav_navigator): navigate_to_pose,
                    navigate_through_poses, follow_waypoints, follow_gps_waypoints
                    (Nav2's action names, so no goal sender changed)
  mppi_lag          the follower (aav_mppi_lag; tracker:= for another): follow_path ->
                    cmd_vel_nav
  velocity_smoother our smoother (aav_smoother's car_smoother, under that name):
                    cmd_vel_nav -> /control/cmd_vel_smoothed (or /drivers/nav/cmd_vel)
  stop_zone         the stop zone (aav_stop_zone, a safety lock): /control/cmd_vel_smoothed
                    -> /drivers/nav/cmd_vel, zero for points in the box ahead (behind,
                    reversing) or a silent sensor; its state on /safety/stop_zone/state
  stop_hold         with the stop zone (aav_safety_rs): holds its stops until the zone
                    stays clear for 1 s, through twist_mux's stop_zone lock
  lidar_scan        with lidar:=true (aav_lidar_rs): the roof lidar's cloud ->
                    /sensors/lidar/scan, for the costmap and the stop zone;
                    lidar_impl:=python for the Python node (aav_lidar)

The settings are config/nav2.yaml's, in Nav2's layout: our ports read the blocks they were
tuned with (vehicle.nav2_tuned writes the car into a temporary copy; vehicle.rewrite the
values Nav2's RewrittenYaml once wrote).

Switches:
  planner:=diffusion      (aav_diffusion_planner, a trial) a model draws 16 paths, a guard
                          checks them, our planner's path when none passes or it is late.
                          It serves compute_path_to_pose; the navigator calls it.
                          car.launch.py refuses it unless diffusion_trial:=true
  planner_search:=hybrid  our planner runs Nav2's Hybrid-A* ported instead of the lattice,
                          on nav2.yaml's GridBased block (Reeds-Shepp with reversing:=true)
  lattice_headings:=32, lattice_moves:=smooth|smooth_fast, plan_smoothing:=strong
                          the lattice's trial files and its smoother (docs/own-planner.md)
  tracker:=pure_pursuit|stanley|lqr|mpc|mpc_full|mpc_lag|mpc_lag_rs
                          another follower serves follow_path (aav_track's; mpc_lag_rs:
                          aav_mpc_lag). Each reverses with reversing:=true (TODO 174,
                          213)
  mppi_lag_steer_gamma:=0.05  mppi_lag's steering control cost (TODO 168)
  mppi_lag_batch:=1000        mppi_lag's samples a tick (empty: 2000; TODO 152)
  mppi_lag_sampling:=smooth   mppi_lag samples the steering's change (TODO 212)
  mpc_lag_tube:=0             mpc_lag_rs's tube off (empty: 0.3 m, tuned 21 Nov)
  live_swap:=true         aav_track's tracker can change while it runs
  map_correction:=slow    the follower's held map -> odom moves toward the newest slowly
                          (TODO 186)
  speed_loop:=firm        the spine's speed loop profile the follower and the behaviour
                          planner assume (TODO 163)
  costmap:=own, navigator:=own, smoother:=own, controller:=mppi
                          each the only option since Nav2 went; kept for a challenger
  amcl:=true              AMCL, scored only: on the map and /sensors/lidar/scan
                          (lidar:=true); amcl_map:= gives it its own map (amcl_map_server).
                          amcl_impl:=rust (aav_amcl_rs: a plain node; stock nav2_amcl went
                          on 20 Nov, TODO 232); config/amcl.yaml
  voxels:=true            (with lidar:=true) aav_voxels' 0.25 m cubes instead of the scan,
                          for the costmap and the stop zone; voxels_impl:=rust for the
                          Rust node. screen_cubes:=true: the cubes for the screens only
  ground:=patchwork       (with lidar:=true) the cubes over a ground plane a patch,
                          ring by ring, the costmap on them (TODO 215, a challenger)
  kerbs:=true             (with lidar:=true) the Rust voxels node's step finder: the local
                          costmap's kerb_layer and the stop zone's steps source (TODO 119)
  behaviour:=true         the behaviour planner (aav_behaviour_rs):
                          the states and a speed cap, in /world/road_ahead's limits; lane_map:=
                          a lanes.yaml for the road ahead's line
  lane_costs:=, lane_source:=, costmap_chunks:=, local_chunks:=true, lane_route:=,
  lane_pass:=, lane_corners:=, lane_pass_shape:=jerk,
  lane_graph:=            our costmap's lanes and chunks, our planner's lane-centre route
                          (TODO 148, 175; docs/own-costmap.md, docs/own-planner.md)
  things:=true            (with lidar:=true) aav_things_rs: the cubes as tracked things on
                          /world/things; things_background*, things_names: theirs
                          (the Python node went on 20 Nov)
  replan_hz:=0            the navigator plans a to-pose goal once (a test: TODO 149)

Obstacles, from where the sensors are:
  radar:=true             the local costmap's obstacle layer reads /sensors/radar/points
  lidar:=true             lidar_scan flattens /sensors/lidar/points; the local costmap's
                          lidar_layer reads it, and the stop zone too
  stop_zone               empty: wherever the radar, the lidar, the ghost or the camera guard
                          runs. A silent source stops the car too
  camera_obstacles:=true  the local costmap's camera layer reads aav_detector's
                          camera_obstacles (off until Q22); camera_guard:=true, the stop zone
                          too (car.launch.py, with the rear camera)
  ghost:=true             a pretend person (aav_ghost, docs/ghost-obstacles.md): its own
                          costmap layer and stop zone source, so it only adds. No ghost until
                          someone places one: pixi run ghost ahead 10

Reversing: nothing but the lidar senses behind the car, so it is off unless asked for:
  reversing:=true         Reeds-Shepp paths, the follower backs up to -max_reverse_speed, the
                          navigator backs up 1 m when stuck, the stop zone's box behind.
                          The simulator and the bench turn it on; car.launch.py with the lidar

The car's footprint, turning radius and reverse speed come from
aav_description/config/vehicle.yaml.
Send a goal: RViz "2D Goal Pose" (publishes /goal_pose), or
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...
    "{pose: {header: {frame_id: map}, pose: {position: {x: 20.0, y: 5.0}}}}"
"""

import atexit
import json
import os
import tempfile

from aav_description import lane_map, vehicle
from aav_lidar.scan import body_box
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import yaml


# The local costmap's parameters, as a whole path for vehicle.rewrite: a leaf key such as
# `enabled` would change every key of that name in nav2.yaml.
LOCAL = 'local_costmap.local_costmap.ros__parameters.'


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def check_lane_pass_shape(value):
    """Return lane_pass_shape:= checked: 'cosine' (empty too) or 'jerk' (TODO 214)."""
    value = value.strip().lower() or 'cosine'
    if value not in ('cosine', 'jerk'):
        raise ValueError(f'lane_pass_shape:={value}: cosine or jerk')
    return value


def check_lane_corners(value):
    """Return lane_corners:= checked: 'arc' (empty too) or 'clothoid'."""
    value = value.strip().lower() or 'arc'
    if value not in ('clothoid', 'arc'):
        raise ValueError(f'lane_corners:={value}: clothoid or arc')
    return value


def voxels_car(cfg):
    """
    Return the Rust voxels node's car parameters, from vehicle.yaml (cfg: vehicle.load()).

    The Python node reads them itself: where the lidar sits (lidar_link), and the car's own
    outline, whose points never count (aav_lidar scan.body_box: x from, x to, half width).
    """
    lidar = cfg['sensors']['lidar_link']
    return {'lidar_xyz': [float(x) for x in lidar['xyz']],
            'lidar_rpy': [float(x) for x in lidar['rpy']],
            'body': [float(x) for x in body_box(vehicle.footprint_points(cfg['vehicle']))]}


def tuned_copy(source, v, reversing, max_speed, radar, lidar, ghost=False,
               controller=vehicle.DEFAULT_CONTROLLER, plan_around=True, camera=False,
               planner='hybrid', lattice_file=None, voxels=False,
               lattice_moves=vehicle.LATTICE_MOVES[0], lidar_memory=0.0, global_window=0.0,
               plan_smoothing='nav2', kerbs=False):
    """Return a temporary copy of nav2.yaml, tuned by vehicle.nav2_tuned (no ROS there)."""
    with open(source) as f:
        cfg = vehicle.nav2_tuned(yaml.safe_load(f), v, reversing, max_speed, radar, lidar,
                                 ghost, controller, plan_around, camera, planner,
                                 lattice_file, voxels, lattice_moves, kerbs=kerbs)
    vehicle.with_lidar_memory(cfg, lidar_memory)
    vehicle.with_global_window(cfg, global_window)
    vehicle.with_plan_smoothing(cfg, plan_smoothing)
    out = tempfile.NamedTemporaryFile('w', prefix='nav2_', suffix='.yaml', delete=False)
    with out:
        yaml.safe_dump(cfg, out)
    # The nodes read it as they start; gone when the launch ends (1000 were left, 22 Oct).
    atexit.register(os.remove, out.name)
    return out.name


def rewritten_copy(source, rewrites):
    """Return a temporary copy of a tuned file with vehicle.rewrite's values written in."""
    with open(source) as f:
        cfg = vehicle.rewrite(yaml.safe_load(f), rewrites)
    out = tempfile.NamedTemporaryFile('w', prefix='nav2_rewritten_', suffix='.yaml',
                                      delete=False)
    with out:
        yaml.safe_dump(cfg, out)
    atexit.register(os.remove, out.name)
    return out.name


def lattice_for(share, v, map_yaml, headings=vehicle.LATTICE_HEADINGS[0],
                moves=vehicle.LATTICE_MOVES[0]):
    """
    Return the lattice primitives' path, checked against the car and the map.

    SmacPlannerLattice measures its moves in the file's cells: on a map with other cells,
    or for a car that now turns wider, it would plan moves the car can't make. Refuse
    rather than start it so (pixi run python3 tools/lattice_primitives.py makes a new file).
    """
    path = f'{share}/config/{vehicle.lattice_file_name(headings, moves)}'
    with open(path) as f:
        meta = json.load(f)['lattice_metadata']
    with open(map_yaml) as f:
        resolution = yaml.safe_load(f)['resolution']
    wrong = vehicle.lattice_problems(meta, v, resolution)
    if meta.get('num_of_headings') != int(headings):
        wrong.append(f"{meta.get('num_of_headings')} headings, not {headings}")
    if wrong:
        raise ValueError(f'the lattice: {path}: ' + '; '.join(wrong) +
                         '. Run pixi run python3 tools/lattice_primitives.py, '
                         'or pick another planner')
    return path


# s before one of our own nodes (not lifecycle: no bond) starts again after it dies, as
# Nav2's servers did with use_respawn. While it is down the car stops: the costmap's and
# odometry's age checks, twist_mux's timeout on /drivers/nav/cmd_vel.
RESPAWN_DELAY = 1.0
# With planner:=diffusion, our planner's own /plan (the fallback's path) goes here.
DIFFUSION_LATTICE_PLAN = '/drivers/nav/diffusion/lattice_plan'


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
    zone = LaunchConfiguration('stop_zone').perform(context).strip().lower()
    camera = flag(context, 'camera_obstacles')
    # Only where a detector feeds them: a silent source stops the car.
    guard = camera and flag(context, 'camera_guard')
    stop_zone = (radar or lidar or ghost or guard) if zone == '' else zone == 'true'
    reversing = flag(context, 'reversing')
    tracker = vehicle.check_tracker(LaunchConfiguration('tracker').perform(context), reversing)
    # The spine's speed loop profile the car runs: every model of the car here assumes it.
    speed_loop = vehicle.check_speed_loop(LaunchConfiguration('speed_loop').perform(context),
                                          tracker)
    amcl = flag(context, 'amcl')
    amcl_impl = vehicle.check_amcl_impl(LaunchConfiguration('amcl_impl').perform(context))
    voxels = flag(context, 'voxels')
    screen_cubes = flag(context, 'screen_cubes')
    voxels_impl = vehicle.check_voxels_impl(LaunchConfiguration('voxels_impl').perform(context))
    things = flag(context, 'things')
    lidar_impl = vehicle.check_lidar_impl(LaunchConfiguration('lidar_impl').perform(context))
    things_impl = vehicle.check_things_impl(LaunchConfiguration('things_impl').perform(context))
    # The kerbs need the lidar; a world without one just runs without them.
    kerbs = flag(context, 'kerbs') and lidar
    # TODO 215: ground:=patchwork, the ground a plane a patch (Patchwork++'s idea) in the
    # Rust voxels node, so a lower kerb is a cube; the costmap then reads the cubes.
    patchwork = vehicle.check_ground(LaunchConfiguration('ground').perform(context)) == \
        'patchwork' and lidar
    voxels = voxels or patchwork
    if (voxels or screen_cubes or things) and not lidar:
        raise ValueError('voxels:=true (and things:=true) needs lidar:=true: the cubes come '
                         'from the lidar')
    if amcl and not lidar:
        raise ValueError('amcl:=true needs lidar:=true: AMCL reads /sensors/lidar/scan')

    planner = vehicle.check_planner(LaunchConfiguration('planner').perform(context))
    search = vehicle.check_planner_search(
        LaunchConfiguration('planner_search').perform(context), 'own')
    # The switches whose Nav2 options went on 17 Nov: each has ours only now.
    vehicle.check_costmap(LaunchConfiguration('costmap').perform(context))
    vehicle.check_navigator(LaunchConfiguration('navigator').perform(context))
    vehicle.check_smoother(LaunchConfiguration('smoother').perform(context))

    cfg = vehicle.load()
    v = cfg['vehicle']
    map_yaml = LaunchConfiguration('map').perform(context)
    headings = (LaunchConfiguration('lattice_headings').perform(context).strip() or
                vehicle.LATTICE_HEADINGS[0])
    moves = vehicle.check_lattice_moves(
        LaunchConfiguration('lattice_moves').perform(context))
    lattice_file = lattice_for(share, v, map_yaml, headings, moves)
    rewrites = vehicle.nav2_rewrites(v, reversing, planner)
    cube_floor = LaunchConfiguration('cube_floor').perform(context).strip()
    if cube_floor:
        rewrites['voxels.ros__parameters.min_height'] = cube_floor
    rewrites.update({
        LOCAL + 'obstacle_layer.enabled': str(radar).lower(),
        LOCAL + 'camera_layer.enabled': str(camera).lower(),
    })
    # A copy of nav2.yaml tuned for the car (vehicle.nav2_tuned: the sensors, the planner's
    # block, the speeds) and max_speed:= (sim.launch.py sim_speed:=). The file keeps Nav2's
    # layout: our ports read the blocks they were tuned with. Our nodes take their JSON
    # from it; the nodes that read a parameter file by their names (lidar_scan, voxels, the
    # smoother) take its rewritten twin (vehicle.rewrite: the car's footprint, heights...).
    speed = LaunchConfiguration('max_speed').perform(context).strip()
    controller = vehicle.DEFAULT_CONTROLLER
    source = tuned_copy(f'{share}/config/nav2.yaml', v, reversing,
                        float(speed) if speed else None, radar, lidar, ghost, controller,
                        flag(context, 'plan_around'), guard, planner, lattice_file, voxels,
                        moves, float(LaunchConfiguration('lidar_memory').perform(context)),
                        float(LaunchConfiguration('global_window').perform(context)),
                        LaunchConfiguration('plan_smoothing').perform(context), kerbs=kerbs)
    params = ParameterFile(rewritten_copy(source, rewrites), allow_substs=True)
    with open(source) as f:
        tuned = yaml.safe_load(f)
    with open(f'{share}/config/nav2.yaml') as f:
        nav2 = yaml.safe_load(f)
    ego_file = FindPackageShare('aav_ego').perform(context) + '/config/ego.yaml'
    with open(ego_file) as f:
        ego = yaml.safe_load(f)['ego']['ros__parameters']
    max_speed = float(speed) if speed else None

    smoothed = '/control/cmd_vel_smoothed' if stop_zone else '/drivers/nav/cmd_vel'
    # planner:=diffusion: its node answers compute_path_to_pose and publishes /plan; our
    # planner plans beside it, its fallback.
    diffusion = planner == 'diffusion'
    # lane_costs:=<lanes.yaml>: the lanes layer in our global costmap (TODO 148).
    lane_costs = LaunchConfiguration('lane_costs').perform(context).strip()
    # lane_source:=camera|both: its lanes from the lane finder's lines too (TODO 264).
    lane_source = lane_map.check_lane_source(
        LaunchConfiguration('lane_source').perform(context))
    # costmap_chunks:=<folder>: our global costmap from the map's chunks (TODO 175).
    costmap_chunks = LaunchConfiguration('costmap_chunks').perform(context).strip()
    # lane_route:=<lanes.yaml>: the lane-centre planner, inside ours (TODO 148, path:=lane).
    lane_route = LaunchConfiguration('lane_route').perform(context).strip()
    behaviour = flag(context, 'behaviour')

    # The map, latched on /map (aav_costmap's map_server, in Nav2's map_server's place).
    actions = [Node(package='aav_costmap', executable='map_server', name='map_server',
                    output='screen', respawn=True, respawn_delay=RESPAWN_DELAY,
                    parameters=[{'yaml_filename': map_yaml, 'topic_name': '/map',
                                 'frame_id': 'map', 'use_sim_time': use_sim_time}])]
    if behaviour:
        # Not a lifecycle node; respawns, so a dead one comes back in DRIVE (sending 0.0).
        # behaviour_impl:=rust: the Rust port, the same name, topics and parameters; it
        # finds vehicle.yaml on AMENT_PREFIX_PATH itself (vehicle_yaml '').
        impl = vehicle.check_behaviour_impl(
            LaunchConfiguration('behaviour_impl').perform(context))
        actions.append(Node(
            package=vehicle.BEHAVIOUR_IMPLS[impl], executable='behaviour', name='behaviour',
            namespace='/',
            output='screen', respawn=True, respawn_delay=1.0,
            parameters=[vehicle.behaviour_params(nav2, cfg, ego, max_speed, controller,
                                                 speed_loop),
                        {'obstacles': lidar and not voxels, 'use_sim_time': use_sim_time,
                         'lane_map': LaunchConfiguration('lane_map').perform(context),
                         # The things: wait only for what moves, a parked thing is the
                         # planner's to go round (behaviour.py moving_ahead).
                         'things': things,
                         # The kerbs (TODO 219): one on the road ahead slows the car as
                         # an obstacle does, before the stop zone acts.
                         'kerbs': kerbs,
                         # Every navigator action: a goal running on any of them.
                         'goal_status_topics': [
                             vehicle.navigator_action(a) + '/_action/status'
                             for a in vehicle.NAVIGATOR_ACTIONS]}]))
    if stop_zone:
        # The stop zone (aav_stop_zone, Rust; collision_monitor ported, TODO 182): between
        # the smoother and twist_mux, it zeroes the command for points in the box ahead
        # (behind, reversing) or a silent sensor. Its settings and sensors from the tuned
        # file's collision_monitor block. Not a lifecycle node; respawns: while it is down
        # nothing reaches /drivers/nav/cmd_vel, and twist_mux's slot times out.
        actions.append(Node(
            package='aav_stop_zone', executable='stop_zone', name='stop_zone',
            output='screen', respawn=True, respawn_delay=RESPAWN_DELAY,
            parameters=[{'settings': json.dumps(vehicle.stop_zone_params(tuned, cfg)),
                         'cmd_in': '/control/cmd_vel_smoothed',
                         'cmd_out': '/drivers/nav/cmd_vel',
                         'state': '/safety/stop_zone/state',
                         'status': '/safety/stop_zone/status',
                         'box': '/safety/stop_zone/box',
                         'use_sim_time': use_sim_time}]))
        # Holds the stop zone's stops (aav_safety stop_hold, twist_mux's stop_zone lock):
        # a zone flickers between stop and go at its edge. It locks nothing until the zone
        # first says STOP.
        config = FindPackageShare('aav_safety').perform(context) + '/config/stop_hold.yaml'
        # stop_hold_impl: rust (aav_safety_rs; the Python went on 20 Nov).
        hold = vehicle.check_stop_hold_impl(
            LaunchConfiguration('stop_hold_impl').perform(context))
        actions.append(Node(package=vehicle.STOP_HOLD_IMPLS[hold], executable='stop_hold',
                            name='stop_hold', namespace='/', output='screen',
                            parameters=[config, {'use_sim_time': use_sim_time}]))
    if lidar and (amcl or not voxels):
        # Not a lifecycle node: it runs from the start. With voxels:=true only AMCL still
        # reads its scan.
        # lidar_impl:=rust: the Rust port, the same name and topics; it reads no
        # vehicle.yaml, so the lidar's mount and the car's outline come with it.
        car = voxels_car(cfg) if lidar_impl == 'rust' else {}
        actions.append(Node(package=vehicle.LIDAR_IMPLS[lidar_impl], executable='lidar_scan',
                            name='lidar_scan', namespace='/', output='screen',
                            parameters=[params, {'use_sim_time': use_sim_time, **car}]))
    if voxels or screen_cubes or things or kerbs:
        # The cubes, for the costmap and the stop zone (voxels:=true), or only for the
        # developer window (screen_cubes:=true) or the things (things:=true): the costmap
        # then stays on the flat scan.
        # voxels_impl:=rust: the Rust port, the same name and topics; it reads no
        # vehicle.yaml, so the car's numbers come with it.
        # kerbs:=true: only the Rust node finds steps, so it runs whatever voxels_impl says.
        impl = 'rust' if kerbs or patchwork else voxels_impl
        car = voxels_car(cfg) if impl == 'rust' else {}
        if kerbs:
            car['steps'] = True
        if patchwork:
            car['ground'] = 'patchwork'
        actions.append(Node(package=vehicle.VOXELS_IMPLS[impl], executable='voxels',
                            name='voxels', namespace='/', output='screen',
                            parameters=[params, {'use_sim_time': use_sim_time, **car}]))
    if things:
        # The cubes as things, on /world/things (aav_things_rs).
        config = FindPackageShare('aav_things_rs').perform(context) + '/config/things.yaml'
        actions.append(Node(package=vehicle.THINGS_IMPLS[things_impl], executable='things',
                            name='things', namespace='/',
                            output='screen',
                            parameters=[config, {'radar': radar,
                                                 'background': flag(context,
                                                                    'things_background'),
                                                 'background_chunks': LaunchConfiguration(
                                                     'things_background_chunks'
                                                 ).perform(context),
                                                 'background_below': float(
                                                     LaunchConfiguration(
                                                         'things_background_below'
                                                     ).perform(context) or 0.0),
                                                 'names': flag(context, 'things_names'),
                                                 # TODO 217: a motion model per kind.
                                                 'per_kind': vehicle.check_things_tracker(
                                                     LaunchConfiguration('things_tracker')
                                                     .perform(context)) == 'per_kind',
                                                 'use_sim_time': use_sim_time}]))
    if ghost:
        # Its own file, then use_sim_time: the file doesn't set it, so the dict adds it.
        config = FindPackageShare('aav_ghost').perform(context) + '/config/ghost_obstacle.yaml'
        actions.append(Node(package='aav_ghost', executable='ghost_obstacle',
                            name='ghost_obstacle', output='screen',
                            parameters=[config, {'use_sim_time': use_sim_time}]))
    # aav_costmap (Rust): our local and global costmaps, Nav2's layers ported, from the
    # tuned file's blocks (the same layers and sources). Not a lifecycle node.
    blocks = vehicle.costmap_params(tuned, v, radar, camera)
    if lane_costs or lane_source != 'map':
        blocks['global']['lanes'] = lane_map.lanes_block(
            lane_map.load(lane_costs) if lane_costs else None, lane_source)
    if costmap_chunks:
        blocks['global']['chunks'] = vehicle.costmap_chunks_block(costmap_chunks)
        if LaunchConfiguration('local_chunks').perform(context).strip().lower() == 'true':
            # The local costmap's static layer: the 3 x 3 chunks round the car only.
            blocks['local']['chunks'] = vehicle.costmap_chunks_block(costmap_chunks, 0.0, 0)
    actions.append(Node(
        package='aav_costmap', executable='costmap', name='costmap', output='screen',
        respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[{'local': json.dumps(blocks['local']),
                     'global': json.dumps(blocks['global']),
                     'use_sim_time': use_sim_time}]))
    # aav_planner (Rust): Nav2's lattice ported, from the same tuned Lattice block (or
    # Hybrid-A*, planner_search:=hybrid, from nav2.yaml's own Hybrid-A* block), the car's
    # outline and the global costmap's inflation. Not a lifecycle node.
    settings = vehicle.planner_params(tuned, v, 'own', search, nav2)
    if lane_route:
        settings['lanes'] = lane_map.planner_block(
            lane_route, LaunchConfiguration('lane_graph').perform(context))
        settings['lanes']['pass'] = LaunchConfiguration('lane_pass').perform(
            context).strip().lower() != 'false'
        settings['lanes']['corners'] = check_lane_corners(
            LaunchConfiguration('lane_corners').perform(context))
        # TODO 214: the pass a jerk-limited QP at the car's top speed (a challenger).
        if check_lane_pass_shape(LaunchConfiguration('lane_pass_shape').perform(
                context)) == 'jerk':
            settings['lanes']['pass_shape'] = 'jerk'
            settings['lanes']['pass_speed'] = max_speed or 1.0
    topic, updates, latched = vehicle.GLOBAL_COSTMAP_TOPICS['own']
    actions.append(Node(
        package='aav_planner', executable='planner', name='planner', output='screen',
        respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[{'settings': json.dumps(settings), 'costmap_topic': topic,
                     'costmap_updates_topic': updates,
                     'costmap_latched': str(latched).lower(),
                     # IsPathValid, as planner_server served it.
                     'is_path_valid_service': '/is_path_valid',
                     'use_sim_time': use_sim_time}],
        # With the diffusion planner, /plan is the model's: ours, its fallback, on its own.
        remappings=([('/plan', DIFFUSION_LATTICE_PLAN)] if planner == 'diffusion' else [])))
    # aav_navigator (Rust): the behaviour trees' logic and waypoint_follower's, under their
    # action names, calling our planner (or the diffusion planner), follower and costmaps.
    navsat = (FindPackageShare('aav_localization').perform(context) +
              '/config/navsat.yaml')
    replan_hz = float(LaunchConfiguration('replan_hz').perform(context) or vehicle.REPLAN_HZ)
    actions.append(Node(
        package='aav_navigator', executable='navigator', name='navigator',
        output='screen', respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[{
            'settings': json.dumps(vehicle.navigator_params(v, reversing,
                                                            replan_hz=replan_hz)),
            'planner_action': ('/compute_path_to_pose' if diffusion
                               else vehicle.OWN_PLANNER_ACTION),
            'planner_through_action': vehicle.OWN_PLANNER_THROUGH_ACTION,
            'follow_action': '/follow_path',
            'clear_local': '/drivers/nav/costmap/clear_entirely_local',
            'clear_global': '/drivers/nav/costmap/clear_entirely_global',
            'backup_action': '',
            'navsat_file': navsat,
            'use_sim_time': use_sim_time}]))
    # aav_smoother (Rust): Nav2's velocity_smoother's block from the same tuned file (its
    # node name), the car from vehicle.yaml through the car model, ego's steering ramp.
    actions.append(Node(
        package='aav_smoother', executable='car_smoother', name='velocity_smoother', namespace='/',
        output='screen', respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[params, vehicle.smoother_params(cfg, ego, vehicle.config_path()),
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', smoothed)]))
    # The follower serves follow_path. Nav2's own numbers (the speed, the checks), so every
    # follower is held to the same.
    # behaviour:=true with a lane map: the road ahead has edges. mppi_lag's LaneCritic
    # keeps the car between them; aav_track keeps each path's points inside them
    # (TODO 169). Without one, the path as planned.
    lane = behaviour and bool(LaunchConfiguration('lane_map').perform(context).strip())
    # TODO 186: slow, the nodes' own rates; newest, the old way (no slew). A switch of
    # its own, not a rate: the node's file would win over a rate set here.
    correction = LaunchConfiguration('map_correction').perform(context)
    if correction not in ('slow', 'newest'):
        raise ValueError(f'map_correction:={correction}: slow or newest')
    slew = {'map_correction': correction == 'slow'}
    # TODO 168: mppi_lag's control cost on its steering (empty: nav2.yaml's gamma).
    gamma = LaunchConfiguration('mppi_lag_steer_gamma').perform(context).strip()
    if gamma:
        slew['steer_gamma'] = vehicle.check_steer_gamma(gamma)
    # TODO 152: mppi_lag's samples a tick (empty: nav2.yaml's batch_size, 2000).
    batch = LaunchConfiguration('mppi_lag_batch').perform(context).strip()
    if batch:
        slew['batch_size'] = vehicle.check_mppi_batch(batch)
    # TODO 212: mppi_lag samples the steering's change (smooth), a challenger.
    sampling = LaunchConfiguration('mppi_lag_sampling').perform(context).strip()
    if sampling:
        slew['sampling'] = vehicle.check_mppi_lag_sampling(sampling)
    # The behaviour planner's speed cap is in /world/road_ahead's limits (TODO 233).
    if tracker == 'mpc_lag_rs':
        # aav_mpc_lag (Rust): aav_track's mpc_lag ported, a challenger beside it.
        own = FindPackageShare('aav_mpc_lag').perform(context) + '/config/mpc_lag_rs.yaml'
        # The tube (Ben, 21 Nov): no cost for being off the path within this many m.
        tube = LaunchConfiguration('mpc_lag_tube').perform(context).strip()
        extra = {'mpc_lag.tube': vehicle.check_mpc_lag_tube(tube) if tube
                 else vehicle.MPC_LAG_TUBE}
        actions.append(Node(
            package='aav_mpc_lag', executable='mpc_lag_rs', name='mpc_lag_rs', namespace='/',
            output='screen', respawn=True, respawn_delay=RESPAWN_DELAY,
            parameters=[own, vehicle.mpc_lag_rs_params(nav2, cfg, ego, max_speed,
                                                       speed_loop, reversing=reversing),
                        {'use_sim_time': use_sim_time,
                         'map_correction': slew['map_correction'], **extra}]))
    elif tracker == 'mppi_lag':
        # aav_mppi_lag (Rust): Nav2's MPPI with the actuators' lag in its model. nav2.yaml's
        # MPPI block and checks, vehicle.yaml's actuators, ego.yaml's ramps; our local
        # costmap.
        own = FindPackageShare('aav_mppi_lag').perform(context) + '/config/mppi_lag.yaml'
        actions.append(Node(
            package='aav_mppi_lag', executable='mppi_lag', name='mppi_lag', namespace='/',
            output='screen', respawn=True, respawn_delay=RESPAWN_DELAY,
            parameters=[own, vehicle.mppi_lag_params(nav2, cfg, ego, max_speed,
                                                     reversing, lane, speed_loop),
                        {'use_sim_time': use_sim_time,
                         'costmap_topic': vehicle.COSTMAP_RAW_TOPICS['own'], **slew}]))
    else:
        own = FindPackageShare('aav_track').perform(context) + '/config/track.yaml'
        actions.append(Node(
            package='aav_track', executable='track', name='track', output='screen',
            respawn=True, respawn_delay=RESPAWN_DELAY,
            parameters=[own, vehicle.track_params(nav2, cfg, ego, max_speed, speed_loop,
                                                  reversing=reversing),
                        {'tracker': tracker, 'use_sim_time': use_sim_time,
                         'road_ahead_topic': '/world/road_ahead' if lane else '',
                         'live_swap': IfCondition(
                             LaunchConfiguration('live_swap')).evaluate(context),
                         **slew}]))
    if diffusion:
        # Not a lifecycle node: it serves from the start, and falls back to our planner
        # until the model answers. The guard's numbers from the car and the tuned file.
        actions.append(Node(
            package='aav_diffusion_planner', executable='diffusion_planner',
            name='diffusion_planner', output='screen',
            parameters=[diffusion_params(nav2, v, reversing), {
                'model': LaunchConfiguration('diffusion_model').perform(context),
                'provider': LaunchConfiguration('diffusion_provider').perform(context),
                'record': LaunchConfiguration('diffusion_record').perform(context),
                'learn_python': LaunchConfiguration('diffusion_python').perform(context),
                'worker': vehicle.check_diffusion_impl(
                    LaunchConfiguration('diffusion_impl').perform(context)),
                'smoothing': LaunchConfiguration('diffusion_smoothing').perform(context),
                'use_sim_time': use_sim_time}],
            # It reads our global costmap's grid (/drivers/nav/costmap/global) itself.
            remappings=[('compute_path_to_pose_lattice', vehicle.OWN_PLANNER_ACTION)]))
    if amcl:
        # AMCL, scored only (a pose trial; tf_broadcast false). Ours (aav_amcl_rs):
        # stock's parameters, topics and frames, a plain node. Stock nav2_amcl, the last of
        # Nav2, went on 20 Nov (TODO 232, tag save/nav2-amcl).
        config = f'{share}/config/amcl.yaml'
        start = {'set_initial_pose': True, 'use_sim_time': use_sim_time}
        for axis in ('x', 'y', 'yaw'):
            start[f'initial_pose.{axis}'] = float(
                LaunchConfiguration(f'amcl_{axis}').perform(context))
        amcl_map = LaunchConfiguration('amcl_map').perform(context)
        if amcl_map:
            # AMCL's own map (maps/README.md: finer cells, obstacles as outlines), from a
            # map server of its own; the costmaps keep /map.
            start['map_topic'] = '/localization/amcl/map'
            actions.append(Node(
                package='aav_costmap', executable='map_server', name='amcl_map_server',
                output='screen', respawn=True, respawn_delay=RESPAWN_DELAY,
                parameters=[{'yaml_filename': amcl_map, 'topic_name': '/localization/amcl/map',
                             'frame_id': 'map', 'use_sim_time': use_sim_time}]))
        remaps = [('amcl_pose', '/localization/amcl/pose'),
                  ('particle_cloud', '/localization/amcl/particles'),
                  ('initialpose', '/localization/amcl/initial_pose')]
        # Not a lifecycle node: it respawns like our others. namespace='/': so the
        # start (a launch dict) beats amcl.yaml in rclrs too (docs/languages.md).
        actions.append(Node(
            package=vehicle.AMCL_IMPLS[amcl_impl], executable='amcl', name='amcl',
            namespace='/', output='screen', respawn=True, respawn_delay=RESPAWN_DELAY,
            parameters=[config, start], remappings=remaps))
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
                                          "local costmap's lidar_layer and the stop zone "
                                          'read it'),
        # Empty: radar, lidar, ghost or camera guard. The stop zone needs one of them, and
        # stops the car when one goes silent.
        DeclareLaunchArgument('stop_zone', default_value='',
                              description='Insert the stop zone (aav_stop_zone; needs the '
                                          'radar, the lidar, the ghost or the camera guard); '
                                          'empty: wherever one runs'),
        DeclareLaunchArgument('camera_obstacles', default_value='false',
                              description="true: the local costmap's camera layer reads "
                                          '/perception/camera_obstacles'),
        DeclareLaunchArgument('camera_guard', default_value='false',
                              description='true (with camera_obstacles): the stop zone '
                                          'stops for the camera obstacles too, and when '
                                          'they go silent'),
        DeclareLaunchArgument('ghost', default_value='false',
                              description='true: a pretend person (aav_ghost), its own '
                                          'costmap layer and stop zone source; '
                                          'placed with pixi run ghost'),
        DeclareLaunchArgument('lidar_memory', default_value='0.0',
                              description='s the local lidar layer keeps a mark after the '
                                          'lidar loses it (0: the newest scan only; TODO 188)'),
        DeclareLaunchArgument('global_window', default_value='0.0',
                              description='m: the global costmap a square rolling with the '
                                          'car (192: the 3 x 3 chunks; 0: the whole map)'),
        DeclareLaunchArgument('things_tracker', default_value='',
                              description='with things:=true: constant_velocity (empty too: '
                                          "one model for all) or per_kind: a bicycle's noise "
                                          "for a vehicle, a person's for a person, speeds "
                                          're-fitted when found again (TODO 217, a challenger)'),
        DeclareLaunchArgument('things_background', default_value='true',
                              description='with things:=true: low cubes on the saved map '
                                          'are not things (TODO 175 step 5; the default '
                                          'since 16 Nov); false: every cube'),
        DeclareLaunchArgument('things_background_chunks', default_value='',
                              description='with things_background: a folder of the NDT '
                                          "map's chunks (maps/campus/chunks/ndt) as the "
                                          'background instead of /map'),
        DeclareLaunchArgument('things_background_below', default_value='0.7',
                              description='with things_background: m, only cubes lower '
                                          'than this drop (kerbs, grass; not a body). '
                                          'Empty: any height'),
        DeclareLaunchArgument('things_names', default_value='false',
                              description='with things:=true: things take the camera '
                                          "detector's labels (TODO 192)"),
        DeclareLaunchArgument('cube_floor', default_value='',
                              description="m above the road: the cubes' floor (empty: the "
                                          "car's obstacle height)"),
        DeclareLaunchArgument('screen_cubes', default_value='false',
                              description='true (with lidar:=true): run aav_voxels for the '
                                          "developer window; the costmap doesn't read its cubes"),
        DeclareLaunchArgument('voxels', default_value='false',
                              description='true (with lidar:=true): the trial, aav_voxels: '
                                          "the costmap's lidar_layer and collision_monitor "
                                          'read 0.25 m cubes instead of the flat scan'),
        DeclareLaunchArgument('ground', default_value='',
                              description='with lidar:=true: plane (empty too: one road plane '
                                          'a cloud) or patchwork, a plane a patch ring by ring '
                                          'in the Rust voxels node, the costmap on its cubes '
                                          '(TODO 215, a challenger)'),
        DeclareLaunchArgument('kerbs', default_value='false',
                              description='true (with lidar:=true): the Rust voxels node '
                                          'finds kerbs (steps in the ground) and the local '
                                          "costmap's kerb_layer marks them (TODO 119)"),
        DeclareLaunchArgument('behaviour_impl', default_value=vehicle.DEFAULT_BEHAVIOUR_IMPL,
                              description='rust (aav_behaviour_rs; the Python removed 20 Nov): '
                                          'which behaviour node runs, with behaviour:=true'),
        DeclareLaunchArgument('voxels_impl', default_value=vehicle.DEFAULT_VOXELS_IMPL,
                              description='python (aav_voxels) or rust (aav_voxels_rs, the '
                                          'same node in Rust): which voxels node runs, with '
                                          'voxels:=true or screen_cubes:=true'),
        DeclareLaunchArgument('stop_hold_impl', default_value=vehicle.DEFAULT_STOP_HOLD_IMPL,
                              description='rust (aav_safety_rs, the default) or python '
                                          '(aav_safety): which stop_hold runs'),
        DeclareLaunchArgument('lidar_impl', default_value=vehicle.DEFAULT_LIDAR_IMPL,
                              description='python (aav_lidar) or rust (aav_lidar_rs, the same '
                                          'node in Rust): which lidar_scan runs, with '
                                          'lidar:=true'),
        DeclareLaunchArgument('things_impl', default_value=vehicle.DEFAULT_THINGS_IMPL,
                              description='rust (aav_things_rs; the Python removed 20 Nov): '
                                          'which things node runs, with things:=true'),
        DeclareLaunchArgument('max_speed', default_value='',
                              description="m/s: Nav2's top speed, if not nav2.yaml's 1.0 "
                                          '(sim.launch.py sim_speed:=)'),
        DeclareLaunchArgument('reversing', default_value='false',
                              description='true: the car may drive backwards (blind: '
                                          'nothing senses behind the car but the lidar)'),
        DeclareLaunchArgument('controller', default_value=vehicle.DEFAULT_CONTROLLER,
                              description="mppi: nav2.yaml's MPPI block, the one the "
                                          "followers read (Nav2's controllers went, 17 Nov)"),
        DeclareLaunchArgument('planner', default_value=vehicle.DEFAULT_PLANNER,
                              description=', '.join(vehicle.PLANNERS) + ': own '
                                          '(aav_planner, the lattice or Hybrid-A* in Rust), '
                                          'or diffusion (a trial, ours behind it). '
                                          'docs/own-planner.md'),
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
        DeclareLaunchArgument('diffusion_impl', default_value=vehicle.DEFAULT_DIFFUSION_IMPL,
                              description="with planner:=diffusion: the model's worker: "
                                          'python (worker.py) or rust (aav_diffusion_rs, '
                                          'ONNX Runtime from Rust; docs/onnx.md)'),
        DeclareLaunchArgument('diffusion_python', default_value='',
                              description="with planner:=diffusion: the model worker's "
                                          "Python (empty: Pixi's learn environment)"),
        DeclareLaunchArgument('diffusion_record', default_value='',
                              description="with planner:=diffusion: a folder for each plan's "
                                          'inputs, to replay offline (training/replay.py); '
                                          'empty: none'),
        DeclareLaunchArgument('lattice_headings',
                              default_value=str(vehicle.LATTICE_HEADINGS[0]),
                              description='with planner:=own: 16 headings (the default '
                                          'file) or 32 (a trial: '
                                          'config/lattice_ackermann_32.json)'),
        DeclareLaunchArgument('lattice_moves', default_value=vehicle.LATTICE_MOVES[0],
                              description="with planner:=own: nav2, Nav2's "
                                          'moves (the default); smooth or smooth_fast, a '
                                          'trial: curvature no faster than the steering '
                                          'turns at 1 or 2.5 m/s, the smoother off '
                                          '(config/lattice_ackermann_smooth*.json)'),
        DeclareLaunchArgument('mppi_lag_batch', default_value='',
                              description='with tracker:=mppi_lag: its samples a tick '
                                          "(batch_size); empty: nav2.yaml's 2000. 1000, a "
                                          'challenger: half the work a tick (TODO 152)'),
        DeclareLaunchArgument('mpc_lag_tube', default_value='',
                              description='with tracker:=mpc_lag_rs: m each side of the '
                                          'path where being off it costs nothing; empty: '
                                          '0.3 (tuned 21 Nov); 0: off'),
        DeclareLaunchArgument('mppi_lag_sampling', default_value='',
                              description='with tracker:=mppi_lag: correlated (empty too: '
                                          "the steering's noise AR(1)) or smooth (TODO 212, "
                                          "a challenger: noise on the steering's change)"),
        DeclareLaunchArgument('mppi_lag_steer_gamma', default_value='',
                              description="with tracker:=mppi_lag: its steering's control "
                                          "cost weight; empty: nav2.yaml's gamma (0.015, as "
                                          "Nav2's MPPI). 0.05, a challenger: less sampling "
                                          'jitter (TODO 168)'),
        DeclareLaunchArgument('plan_smoothing', default_value='nav2',
                              description="the planner's smoother: nav2, Nav2's weights (the "
                                          'default); strong, a challenger: it also takes out '
                                          "the lattice's staircase on roads at an angle "
                                          '(vehicle.PLAN_SMOOTHINGS, TODO 168)'),
        DeclareLaunchArgument('planner_search', default_value=vehicle.PLANNER_SEARCHES[0],
                              description='with planner:=own: lattice (the default), the '
                                          "State Lattice; or hybrid, a challenger: Nav2's "
                                          "Hybrid-A* in Rust, on nav2.yaml's GridBased "
                                          'block (docs/own-planner.md)'),
        DeclareLaunchArgument('plan_around', default_value='true',
                              description='true: the planner sees the radar and lidar too, '
                                          'and plans round what they see; false: the map '
                                          'only, so the car stops for it (the stop tests)'),
        DeclareLaunchArgument('replan_hz', default_value=str(vehicle.REPLAN_HZ),
                              description=('how often the navigator plans a to-pose goal '
                                           "again (Hz; Nav2's trees' 1.0); 0: once a goal, a "
                                           'test (the wiggle, TODO 149)')),
        DeclareLaunchArgument('behaviour', default_value='false',
                              description='true: aav_behaviour_rs, the behaviour states and a '
                                          'speed cap on Nav2 (a trial, docs/stopping.md)'),
        DeclareLaunchArgument('lane_map', default_value='',
                              description="with behaviour: a lanes.yaml, so the road ahead's "
                                          "line is the lane's centre; empty: /plan's line"),
        DeclareLaunchArgument('lane_costs', default_value='',
                              description='with costmap:=own: a lanes.yaml, drawn into our '
                                          'global costmap as soft costs, lowest at the right '

                                          "lane's centre (TODO 148); empty: off"),
        DeclareLaunchArgument('lane_source', default_value=lane_map.DEFAULT_LANE_SOURCE,
                              description="the lanes layer's lanes: map (lane_costs:='s "
                                          "roads), camera (the lane finder's lines, on "
                                          '/perception/lane/geometry; needs it running) or '
                                          'both (the map wins where it has a road; TODO 264)'),
        DeclareLaunchArgument('costmap_chunks', default_value='',
                              description="with costmap:=own: a folder of the map's chunks "
                                          '(chunks.yaml, fine/); the global costmap holds '
                                          'only those round the car and along the route '
                                          '(TODO 175). Empty: the whole map (/map)'),
        DeclareLaunchArgument('lane_route', default_value='',
                              description='with planner:=own: a lanes.yaml; the path is the '
                                          "right lane's centre along it, the lattice joining "
                                          'the ends (TODO 148, path:=lane); empty: off'),
        DeclareLaunchArgument('lane_pass', default_value='true',
                              description='with lane_route: true, a parked obstacle on the '
                                          'lane is passed by moving only its stretch left; '
                                          "false: the whole path is the lattice's"),
        DeclareLaunchArgument('lane_pass_shape', default_value='',
                              description='with lane_route: a pass round a parked obstacle '
                                          'as a half cosine (empty, cosine: the default) or '
                                          'jerk, a QP holding the lateral jerk (TODO 214, a '
                                          'challenger)'),
        DeclareLaunchArgument('lane_corners', default_value='arc',
                              description="with lane_route: the lane line's corners, "
                                          'arc (circle arcs, the default) or clothoid '
                                          '(continuous curvature, TODO 211, a challenger)'),
        DeclareLaunchArgument('lane_graph', default_value=lane_map.DEFAULT_LANE_GRAPH,
                              description='with lane_route: lanes_yaml (its roads) or '
                                          'lanelet2 (the Lanelet2 map beside it; TODO 175)'),
        DeclareLaunchArgument('local_chunks', default_value='false',
                              description='with costmap_chunks: true, the local costmap'
                                          "'s static layer from the chunks round the car "
                                          'too, not /map (TODO 175)'),
        DeclareLaunchArgument('things', default_value='false',
                              description='true (with lidar:=true): aav_things_rs, the cubes as '
                                          'tracked things on /world/things (voxels runs too)'),
        DeclareLaunchArgument('costmap', default_value=vehicle.DEFAULT_COSTMAP,
                              description='own: aav_costmap, our costmaps, on '
                                          "/drivers/nav/costmap/* (Nav2's went, 17 Nov). "
                                          'docs/own-costmap.md'),
        DeclareLaunchArgument('navigator', default_value=vehicle.DEFAULT_NAVIGATOR,
                              description="own: aav_navigator, under Nav2's action "
                                          'names (bt_navigator went, 17 Nov). '
                                          'docs/own-navigator.md'),
        DeclareLaunchArgument('smoother', default_value=vehicle.DEFAULT_SMOOTHER,
                              description="own: aav_smoother's car_smoother, under "
                                          "velocity_smoother's name: the turn rate from where "
                                          "the wheels can be (Nav2's went, 17 Nov). "
                                          'docs/own-smoother.md'),
        DeclareLaunchArgument('speed_loop', default_value=vehicle.DEFAULT_SPEED_LOOP,
                              description="the spine's speed loop profile the car runs "
                                          '(default, smooth, lqi, firm): the follower and '
                                          "the behaviour planner's models assume it "
                                          '(mpc_lag, mpc_lag_rs: not lqi)'),
        DeclareLaunchArgument('tracker', default_value=vehicle.DEFAULT_TRACKER,
                              description='the path follower (mppi_lag: aav_mppi_lag, the '
                                          'default; mpc_lag_rs: aav_mpc_lag; the rest '
                                          "aav_track's): " +
                                          ', '.join(vehicle.TRACKERS) +
                                          '. docs/trials.md'),
        DeclareLaunchArgument('map_correction', default_value='newest',
                              description='with a tracker: newest, each new path by the '
                                          'newest map -> odom; slow (a challenger, TODO '
                                          '186), the held map -> odom slewed toward it'),
        DeclareLaunchArgument('live_swap', default_value='false',
                              description='true (sim.launch.py sets it): the tracker can be '
                                          'changed while it runs, `ros2 param set /track '
                                          'tracker lqr`. Never on the car'),
        DeclareLaunchArgument('amcl', default_value='false',
                              description='true (with lidar:=true): AMCL as a trial, '
                                          'scored only: /localization/amcl/pose'),
        DeclareLaunchArgument('amcl_impl', default_value=vehicle.DEFAULT_AMCL_IMPL,
                              description='rust: which AMCL (aav_amcl_rs; stock nav2_amcl '
                                          'removed 20 Nov)'),
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
