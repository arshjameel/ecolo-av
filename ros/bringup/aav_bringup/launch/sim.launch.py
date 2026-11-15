"""
The whole stack in a simulator: same nodes and configs as the car, simulated hardware.

  ros2 launch aav_bringup sim.launch.py [stack:=ours|stock|best|car|]
      [world:=empty|test_lot|road|campus] [nav:=true]
      [teleop:=false] [gui:=true] [rviz:=true]
      [perception:=false] [lane:=false|true|fake] [camera_obstacles:=false] [hud:=false]
      [map:=false] [foxglove:=false] [chase_camera:=true] [cameras:=true]
      [obstacle:=false] [start_x:=0.0] [start_y:=0.0] [start_yaw:=0.0] [lidar:=true] [radar:=true]
      [actuator_model:=true] [speed_loop:=default|smooth|lqi] [sim_speed:=]
      [payload:=0] [adaptive_brake:=false]
      [reversing:=true] [controller:=] [planner:=] [tracker:=] [costmap:=] [navigator:=]
      [smoother:=] [behaviour:=]
      [amcl:=false] [voxels:=false]
      [ndt:=false] [ndt_ekf:=false] [ndt_map:=maps/ndt/test_lot.pcd] [kiss_icp:=false]
      [kiss_ekf:=false]
      [plan_around:=true] [things:=false]
      [watchdog:=true] [geofence:=true] [geofence_radius:=]
      [watch_hud:=false] [faults:=false] [spine:=false] [arm:=true]
      [esp32:=false] [esp32_loop:=pi]
      [bridge:=rust|python]
      [ghost:=false] [simulator:=gazebo|own]
  pixi run sim world:=road                         the road: the car's camera and radar
  pixi run sim world:=campus                       Carleton, every level ours
  pixi run sim stack:=stock                        every level Nav2's (the yardstick)
  pixi run sim costmap:=nav2                       Nav2's costmaps, the rest ours

Ours is the default (Ben, 14 Nov: "Ours default now, then delete"): stack:=ours sets the
planner, path follower, costmap, navigator, smoother, behaviour and pose at once
(aav_description's config/stacks.yaml, docs/league.md). Those switches are empty by
default, meaning "the stack's choice"; any value given by hand wins over the stack.
stack:= (empty): no stack, each switch as given, else the champions' (Nav2's). The real
car (car.launch.py) keeps its own choices until ours wins there too.
  pixi run -e ml sim world:=road perception:=true  adds the detector and the stop-sign watcher
  pixi run sim world:=test_lot sim_speed:=3 hud:=true   Gazebo, RViz, Nav2 and the HUD window
  pixi run sim world:=test_lot map:=true           ... and the big screen's map window

world:=empty pairs with the empty map (phase 2); world:=test_lot with
maps/example_lot.yaml (phase 3); world:=road, the bench's road in Gazebo, with the empty
map: chosen automatically. The car starts at the world origin facing east, which is also
the map origin (the datum); start_x, start_y and start_yaw move it, as on the bench.
world:=campus is Carleton (pixi run campus makes it): its map, its NDT guess and a start in lot
P4 come with it, because the datum itself is on a station shelter there.

The road and test_lot worlds render the car's three cameras, its radar stand-in and the
roof lidar (aav_gazebo worlds.has_sensors), so the car's whole graph runs there, with the
bench's argument names:

  the radar           where the world renders it (radar:=false drops it, to test the
                      lidar alone): radar_clusters, and Nav2's obstacle layer and
                      collision_monitor (navigation.launch.py radar:=true)
  perception:=true    the detector (RF-DETR; the ml environment) and stop_sign_watcher: twist_mux's
                      stop-sign lock
  lane:=true          lane keeping: lane_finder and control_interface, on /drivers/ai/cmd_vel.
                      lane:=fake: control_interface on a pretend lane, in any world
  camera_obstacles    the detectors' boxes into Nav2's local costmap
  hud:=true           the HUD window (aav_hud, as pixi run hud): needs a screen
  map:=true           the map window (aav_hud, as pixi run map): Nav2's map, the
                      car, the plan, the distance left. Needs a screen
  foxglove:=true      foxglove_bridge on 127.0.0.1:8765, for Lichtblick or Foxglove on this
                      computer: open ws://localhost:8765 and a layout from tools/foxglove/
                      (foxglove.launch.py: the car's topic list, and the front camera's
                      preview where the world renders it; docs/foxglove.md)
  chase_camera:=false no chase camera: by default a camera behind and above the car
                      (aav_gazebo), for Lichtblick's cockpit layout (tools/foxglove/
                      cockpit.json): /vehicle/sim/chase_camera/compressed
  cameras:=false      no cameras at all (no picture, no chase camera): our simulator then
                      runs on a computer with no GPU, as CI's runners; perception, lane
                      and camera_obstacles are refused
  run_info            always: which algorithms the run uses, from these arguments, on
                      /system/algorithms (aav_run_info), for the cockpit's Algorithms table
  obstacle:=true      the bench's 1 m box in the lane, 15 m ahead (aav_gazebo)
  lidar:=false        no roof lidar. It's on by default (15 Oct): a simulated 16-beam
                      lidar on /sensors/lidar/points, which Nav2 reads as a 2D scan
                      (navigation.launch.py lidar:=true: its local costmap and
                      collision_monitor). Its earlier flakiness on a Mac came from other
                      programs competing
  watch_screens:=dev_window,hw_panel  commands flow only while those screens beat (they
                      send it with --developer): close one and the output mux closes, and
                      with spine:=true the spine's own state machine latches
  sim_speed:=3        a higher top speed (m/s, up to 5), for ego and Nav2, in Gazebo only:
                      the car's 1.0 m/s cap (ego.yaml, nav2.yaml) is for its first drives.
                      ego refuses it outside a simulator
  controller:=rpp     Nav2 follows the path with Regulated Pure Pursuit, not MPPI (the
                      champion), with tracker:=none: pixi run measure controllers drives
                      both (docs/path-followers.md)
  planner:=hybrid     another planner (navigation.launch.py planner:=): own (ours, the
                      default), lattice (Nav2's, stack:=stock) or hybrid keep to the car's
                      turning radius;
                      smac2d, navfn and theta_star don't, trials only. pixi run
                      measure controllers --planners scores them (docs/controllers.md).
                      lattice_headings:=32: the lattice's trial file, 32 headings
                      lattice_moves:=smooth|smooth_fast: the lattice's trial moves,
                      curvature no faster than the steering turns (tools/lattice_smooth.py)
                      planner_search:=hybrid: with planner:=own, ours runs Nav2's
                      Hybrid-A* ported instead of the lattice (a challenger, the league's
                      hybrid_rs)
                      planner:=diffusion: a model draws the path, a guard checks it,
                      the lattice's path when it fails (aav_diffusion_planner, a trial:
                      pixi install --locked -e learn, pixi run diffusion train first);
                      diffusion_model:=<.onnx> another model; diffusion_record:=<folder>
                      each plan's inputs, to replay offline; diffusion_smoothing:=average
                      the 2 Nov smoothing
                      planner:=own: our own planner (aav_planner: Nav2's lattice in
                      Rust) answers the trees, Nav2's lattice beside it
                      (docs/own-planner.md)
  tracker:=lqr        another path follower in controller_server's place (mppi_lag, ours
                      in Rust, is the default; aav_track: pure_pursuit, stanley, lqr or
                      mpc), on the same planner, smoother and collision_monitor.
                      aav_track's drive forwards only: with reversing:=false
                      (docs/trials.md); mppi_lag reverses too. tracker:=none: Nav2's
                      controller_server (controller:=)
  costmap:=nav2       Nav2's costmaps instead of ours (aav_costmap, the default; with
                      tracker:=mppi_lag, mppi_lag reads ours: docs/own-costmap.md)
  navigator:=nav2     bt_navigator and waypoint_follower instead of our own navigator
                      (aav_navigator, the default); with planner:=own costmap:=own
                      tracker:=mppi_lag, no Nav2 server navigates (docs/own-navigator.md)
  behaviour:=false    no behaviour planner (on by default with stack:=ours). On:
                      the behaviour states and a speed cap on Nav2 from the stopping law
                      (aav_behaviour, a trial: docs/stopping.md); it also sends the road ahead
                      (/world/road_ahead), and ego holds Nav2 to it (ego road_ahead:=true)
  lane_map:=world    with behaviour:=true: the road ahead's line is the lane's centre, from
                      the world's lane map (road: aav_gazebo worlds/road.lanes.yaml; campus:
                      maps/campus/lanes.yaml), with its edges; or a lanes.yaml's path.
                      Empty (the default): the line is /plan's
  things:=true        the lidar's cubes as things with an id, a speed and a memory, on
                      /world/things (aav_things; voxels runs for it): only with nav:=true
                      and the lidar. Nothing reads the list yet
  amcl:=true          stock AMCL beside the global EKF, scored only (docs/trials.md): only
                      in world:=test_lot with nav:=true and the lidar, from the start pose
  voxels:=true        Nav2's local costmap and collision_monitor read the lidar as 0.25 m
                      cubes (aav_voxels, a trial: docs/designs/2026-10-30_voxels.md)
                      instead of the flat scan; only with the lidar
  ndt:=true           NDT on the saved lidar map (aav_ndt, Rust), scored only, never TF
                      (docs/designs/2026-10-29_ndt.md): only in world:=test_lot with the
                      lidar, from the start pose, and only once ndt_map (pixi run ndt-map
                      drive) is there
  ndt_ekf:=true       a trial: NDT as above (ndt:=true is implied), and ekf_global takes
                      its pose (x, y, yaw) beside the GPS (aav_localization
                      config/ekf_global_ndt.yaml; docs/trials.md). Default false: the
                      global EKF on GPS alone
  kiss_icp:=true      KISS-ICP lidar odometry (aav_kiss_icp), scored only, never TF or an
                      EKF (docs/designs/2026-10-30_kiss-icp.md): any world with the lidar;
                      needs the kiss environment once (pixi install --locked -e kiss)
  kiss_ekf:=true      a trial: KISS-ICP as above (kiss_icp:=true is implied), and ekf_local
                      takes its twist (vx, vyaw) beside the wheels and the IMU
                      (aav_localization config/ekf_local_kiss.yaml; docs/trials.md).
                      Default false: the local EKF on the wheels and the IMU alone
  truth_pose:=true    a test (the wiggle, TODO 149): the simulator's truth is the car's pose.
                      No ekf_global; aav_gazebo's truth_localizer publishes
                      /localization/odom/global and map -> odom so that map -> base_link
                      is the truth. ekf_local still gives odom -> base_link. Never the car
  replan_hz:=1.0      how often Nav2's navigate-to-pose tree plans again (navigation.launch.py;
                      1.0 as Nav2's own tree). replan_hz:=0: plan once per goal, the wiggle's
                      second test
  plan_around:=false  Nav2's planner sees only the map, not the radar and lidar: the car
                      stops for an obstacle instead of planning round it (the stop tests)
  watchdog:=true      the sensor watchdog (aav_safety): twist_mux's sensors lock stops Nav2
                      and the AI slot when the wheels, the IMU, the GPS or an EKF go quiet,
                      NaN or frozen, or the GPS jumps; and the camera, where something
                      reads it (perception, lane, camera_obstacles, faults). With
                      hud:=true, the HUD too: its /hud/heartbeat stops when its screen
                      freezes or dies
  geofence:=true      the geofence (aav_safety): twist_mux's geofence lock holds every
                      program (scripted, point, Nav2, AI; never a person) before its
                      stopping distance reaches a circle of geofence_radius m (200 here,
                      600 on the campus: past every sim check's drive; the car's is 30)
                      round where the car starts (0, start_y), or while the global pose
                      can't be trusted
  watch_hud:=true     the watchdog watches /hud/heartbeat without the HUD window: something
                      else beats (sim-check --faults stands in for the screen)
  faults:=true        the front left picture through gazebo_adapter, so its fault.camera
                      works (pixi run sim-fault camera freeze); the other faults always do
  spine:=true         the spine in the loop: ego -> spine_bridge -> CAN frames -> spine_sim
                      (the spine's own firmware logic: arming, its 100 ms command
                      watchdog, both e-stops, faults, its speed loop) -> the actuator
                      model -> Gazebo; Gazebo's measurements back as CAN frames ->
                      spine_bridge -> /vehicle/*. arm:=true arms it after 12 s;
                      bridge:=python runs aav_spine's Python spine_bridge, not the Rust one
  esp32:=true         today's car in the loop: ego -> legacy_uart_bridge (the car's own)
                      -> 13-byte frames over local TCP -> gazebo_esp32 (the 2025-26 C++
                      firmware's logic: its throttle ramp, brake, no timeout) -> the
                      actuator model -> Gazebo. esp32_loop:= is the bridge's
                      speed_loop.mode: pi (the default here) or open (the 2025-26 mapping)
  ghost:=true        a pretend person in Nav2's view (aav_ghost), for stopping tests
                      with nothing in Gazebo: pixi run ghost ahead 10, pixi run ghost off.
                      Its own costmap layer and collision_monitor source: it only adds
                      (docs/ghost-obstacles.md)
  actuator_model:=false  the old instant car instead of the actuator model (gazebo_adapter;
                      vehicle.yaml's actuators:), to compare the two
  lag_scale:=2        the actuator model's throttle, brake and steering lags twice
                      vehicle.yaml's, while every driver still reads vehicle.yaml: how a
                      follower copes with a car slower than it was told (1.0: as written).
                      Gazebo's adapter only (simulator:=gazebo)
  simulator:=own      the default (8 Nov): our own Rust simulator (aav_simulator,
                      docs/own-simulator.md) in gz sim's place: the same worlds, topics
                      and rates, the lidar and radar by ray casting, the cameras by Bevy
                      (aav_sim_cameras; needs a GPU), no window.
                      simulator:=gazebo: Gazebo, as before (on a computer with no GPU, with
                      tools/software_gl.sh in front)

The empty and test_lot worlds render nothing, so they run headless anywhere, and the
camera's parts need world:=road. Every node runs on simulated time.
"""

import os

from aav_description import stacks, vehicle
from aav_gazebo import worlds
from aav_spine import bridges
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

MAPS = {'empty': 'empty.yaml', 'test_lot': 'example_lot.yaml', 'road': 'empty.yaml',
        worlds.CAMPUS: os.path.abspath(worlds.CAMPUS_MAP)}
DEFAULT_NDT_MAP = os.path.join('maps', 'ndt', 'test_lot.pcd')
# What each world shows in RViz: the road world adds the camera, the radar and the truth.
LAYOUTS = {'road': 'road.rviz', 'test_lot': 'road.rviz', worlds.CAMPUS: 'road.rviz'}
# The parts that read the camera, so need a world that renders it.
CAMERA_PARTS = ('perception', 'camera_obstacles')


def include(context, package, launch_file, **args):
    share = FindPackageShare(package).perform(context)
    return IncludeLaunchDescription(
        os.path.join(share, 'launch', launch_file),
        launch_arguments={k: str(v).lower() if isinstance(v, bool) else str(v)
                          for k, v in args.items()}.items())


# The switches a stack (stack:=, config/stacks.yaml) may set. Each is declared empty
# (stacks.CHOSEN: "the stack's choice"), so any value given by hand wins; with no stack,
# an empty one takes its fallback here (the champions', Nav2's: as car.launch.py runs).
STACK_FALLBACK = {'controller': vehicle.DEFAULT_CONTROLLER, 'tracker': vehicle.DEFAULT_TRACKER,
                  'planner': vehicle.DEFAULT_PLANNER, 'costmap': vehicle.DEFAULT_COSTMAP,
                  'navigator': vehicle.DEFAULT_NAVIGATOR, 'smoother': vehicle.DEFAULT_SMOOTHER,
                  'behaviour': 'false', 'ndt_ekf': 'false', 'kiss_ekf': 'false',
                  'lattice_moves': vehicle.LATTICE_MOVES[0],
                  'planner_search': vehicle.PLANNER_SEARCHES[0]}


def launch_setup(context):
    def raw(name):
        return LaunchConfiguration(name).perform(context)

    # stack:= sets the switches in STACK_FALLBACK at once; one given by hand still wins.
    stack = raw('stack').strip().lower()
    league = stacks.load()
    picked = stacks.resolve(league, stack, {n: raw(n) for n in STACK_FALLBACK},
                            STACK_FALLBACK)
    print(f'[sim.launch.py] {stacks.describe(league, stack)}', flush=True)

    def arg(name):
        return picked[name] if name in picked else raw(name)

    world = arg('world')
    if world not in MAPS:
        raise ValueError(f'world:={world}: pick one of {sorted(MAPS)}')
    start_x, start_y, start_yaw = arg('start_x'), arg('start_y'), arg('start_yaw')
    ndt_map = arg('ndt_map')
    if world == worlds.CAMPUS:
        # Not on the world's origin unless someone asks: it's on a station shelter.
        worlds.world_file('', world)
        with open(worlds.CAMPUS_PLACE) as place:
            x, y, yaw = worlds.campus_start(place.read())
        if float(start_x) == 0.0 and float(start_y) == 0.0:
            start_x, start_y, start_yaw = str(x), str(y), str(yaw)
        if ndt_map == DEFAULT_NDT_MAP:
            ndt_map = worlds.CAMPUS_NDT
    sim_speed = arg('sim_speed') or (worlds.CAMPUS_SPEED if world == worlds.CAMPUS else '')
    flag = {n: arg(n).lower() == 'true'
            for n in ('nav', 'teleop', 'rviz', 'perception', 'camera_obstacles', 'hud',
                      'map', 'foxglove', 'reversing', 'watchdog', 'faults',
                      'ghost', 'plan_around', 'watch_hud', 'geofence', 'voxels',
                      'screen_cubes', 'behaviour', 'things')}
    lane = arg('lane').lower()
    if lane not in ('false', 'true', 'fake'):
        raise ValueError(f'lane:={lane}: use true (the lane finder), fake or false')
    controller = arg('controller').lower()
    if controller not in vehicle.CONTROLLERS:
        raise ValueError(f'controller:={controller}: pick one of {", ".join(vehicle.CONTROLLERS)}')
    tracker = vehicle.check_tracker(arg('tracker'), flag['reversing'])
    planner = vehicle.check_planner(arg('planner'))
    amcl = arg('amcl').lower() == 'true'
    # ndt_ekf: ekf_global takes NDT's pose, so NDT runs too.
    ndt_ekf = arg('ndt_ekf').lower() == 'true'
    ndt = arg('ndt').lower() == 'true' or ndt_ekf
    truth_pose = arg('truth_pose').lower() == 'true'
    if truth_pose and ndt_ekf:
        raise ValueError('truth_pose:=true has no ekf_global for ndt_ekf:=true to feed')
    # kiss_ekf: ekf_local takes KISS-ICP's twist, so KISS-ICP runs too.
    kiss_ekf = arg('kiss_ekf').lower() == 'true'
    kiss_icp = arg('kiss_icp').lower() == 'true' or kiss_ekf
    gazebo_share = FindPackageShare('aav_gazebo').perform(context)
    simulator = worlds.check_simulator(arg('simulator'))
    if float(arg('lag_scale')) != 1.0 and simulator != 'gazebo':
        # Only Gazebo's adapter scales the lags; our simulator would silently ignore it.
        raise ValueError('lag_scale:= needs simulator:=gazebo (our simulator has no '
                         'lag_scale yet)')
    world_path = worlds.world_file(gazebo_share, world)
    lane_map = worlds.lane_map_file(gazebo_share, world, arg('lane_map').strip())
    # The lidar and radar where the world has them; the cameras only where they're drawn.
    sensors = worlds.has_sensors(world_path)
    cameras = worlds.has_cameras(world_path, simulator) and arg('cameras').lower() == 'true'
    # Each only where the world renders it, as gazebo.launch.py decides.
    radar = sensors and arg('radar').lower() == 'true'
    lidar = sensors and arg('lidar').lower() == 'true'
    wanted = [n for n in CAMERA_PARTS if flag[n]] + (['lane'] if lane == 'true' else [])
    if amcl and not (world == 'test_lot' and flag['nav'] and lidar):
        raise ValueError('amcl:=true needs world:=test_lot nav:=true lidar:=true: the only '
                         'world with a real map, and AMCL reads the lidar')
    if flag['things'] and not (lidar and flag['nav']):
        raise ValueError('things:=true needs the lidar and nav:=true (navigation.launch.py '
                         'starts it, with voxels)')
    if (flag['voxels'] or flag['screen_cubes']) and not lidar:
        raise ValueError('voxels:=true needs the lidar (lidar:=true, world:=road or test_lot): '
                         'the cubes come from it')
    if ndt and not (world in ('test_lot', worlds.CAMPUS) and lidar):
        raise ValueError('ndt:=true (and ndt_ekf:=true) needs world:=test_lot or campus and '
                         'lidar:=true: the only worlds with '
                         'a lidar map (pixi run ndt-map drive), and NDT reads the lidar')
    if ndt and not os.path.isfile(ndt_map):
        raise ValueError(f'ndt:=true: no map at ndt_map:={ndt_map}. Make it with '
                         '`pixi run ndt-map drive`')
    if kiss_icp and not lidar:
        raise ValueError('kiss_icp:=true (and kiss_ekf:=true) needs the lidar: '
                         f'world:={world} lidar:={arg("lidar")} has none '
                         '(world:=test_lot or road, lidar:=true)')
    if wanted and not cameras:
        raise ValueError(f'{", ".join(wanted)}: world:={world} has no camera with '
                         f'simulator:={simulator} cameras:={arg("cameras")}. Use '
                         'world:=road, test_lot or campus, and cameras:=true')
    nav_share = FindPackageShare('aav_navigation').perform(context)
    actions = [
        # Every node on simulated time, those in the included files too.
        SetParameter(name='use_sim_time', value=True),
        include(context, 'aav_gazebo', 'gazebo.launch.py', world=world, gui=arg('gui'),
                obstacle=arg('obstacle'), start_x=start_x, start_y=start_y, start_yaw=start_yaw,
                lidar=arg('lidar'), radar=arg('radar'), chase_camera=arg('chase_camera'),
                cameras=str(cameras).lower(),
                actuator_model=arg('actuator_model'), speed_loop=arg('speed_loop'),
                payload=arg('payload'), adaptive_brake=arg('adaptive_brake'),
                lag_scale=arg('lag_scale'),
                faults=flag['faults'],
                spine=arg('spine'), arm=arg('arm'), bridge=arg('bridge'),
                esp32=arg('esp32'), esp32_loop=arg('esp32_loop'), simulator=simulator),
        include(context, 'aav_command_mux', 'mux.launch.py', use_sim_time='true',
                sensor_lock=flag['watchdog'], geofence_lock=flag['geofence']),
        # behaviour:=true: the road ahead's writer runs, so ego holds Nav2 to it.
        include(context, 'aav_ego', 'ego.launch.py', use_sim_time='true',
                sim_max_speed=sim_speed, road_ahead=str(flag['behaviour']).lower()),
        # ego's commands go to Gazebo (gazebo_adapter, or spine_bridge with spine:=true)
        # only while every source is SIM (aav_io_mux).
        include(context, 'aav_io_mux', 'io_mux.launch.py', target='sim', source='sim',
                **({'screens': arg('watch_screens')} if arg('watch_screens') else {})),
        include(context, 'aav_scripted', 'scripted.launch.py', use_sim_time='true'),
        include(context, 'aav_drive_to_point', 'drive_to_point.launch.py', use_sim_time='true',
                watchdog=arg('watchdog')),
        # Only the IMU filter: Gazebo stands in for the drivers.
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick',
             parameters=[os.path.join(FindPackageShare('aav_sensors').perform(context),
                                      'config', 'imu_filter.yaml'), {'use_sim_time': True}],
             remappings=[('imu/data_raw', '/sensors/imu/data_raw'),
                         ('imu/data', '/sensors/imu/data')]),
        include(context, 'aav_localization', 'localization.launch.py', use_sim_time='true',
                ndt_ekf=str(ndt_ekf).lower(), kiss_ekf=str(kiss_ekf).lower(),
                global_ekf=str(not truth_pose).lower()),
    ]
    if truth_pose:
        # The wiggle's first test: the truth as the pose, in ekf_global's place.
        actions.append(Node(package='aav_gazebo', executable='truth_localizer',
                            name='truth_localizer', parameters=[{'use_sim_time': True}]))
    # What this run uses, in words, for the cockpit's Algorithms table (aav_run_info).
    profile = f'sim world:={world}' + (
        f' simulator:={simulator}' if simulator != worlds.DEFAULT_SIMULATOR else '')
    actions.append(Node(package='aav_run_info', executable='run_info', name='run_info',
                        parameters=[{'use_sim_time': True, 'profile': profile,
                                     'nav': flag['nav'], 'planner': planner,
                                     'controller': controller, 'tracker': tracker,
                                     'lidar': lidar, 'radar': radar,
                                     'voxels': flag['voxels'], 'amcl': amcl, 'ndt': ndt,
                                     'ndt_ekf': ndt_ekf, 'kiss_icp': kiss_icp,
                                     'kiss_ekf': kiss_ekf}]))
    if radar:
        # The radar stand-in, as the bench's: few points per object, so small clusters.
        actions.append(include(context, 'aav_radar_clusters', 'radar_clusters.launch.py',
                               minimum_points='5'))
    if flag['nav']:
        # Where the world renders the radar and the lidar, Nav2's local costmap reads
        # them, and collision_monitor guards Nav2's commands, as on a car with them.
        actions.append(include(context, 'aav_navigation', 'navigation.launch.py',
                               use_sim_time='true',
                               map=os.path.join(nav_share, 'maps', MAPS[world]),
                               radar=radar, lidar=lidar, ghost=flag['ghost'],
                               voxels=flag['voxels'],
                               screen_cubes=flag['screen_cubes'],
                               full_costmap=world != worlds.CAMPUS,
                               cube_floor=(worlds.CAMPUS_CUBE_FLOOR
                                           if world == worlds.CAMPUS and not flag['voxels']
                                           else ''),
                               collision_monitor=radar or lidar or flag['ghost'],
                               camera_obstacles=flag['camera_obstacles'],
                               max_speed=sim_speed,
                               reversing=str(flag['reversing']).lower(),
                               controller=controller, tracker=tracker, planner=planner,
                               costmap=vehicle.check_costmap(arg('costmap')),
                               navigator=vehicle.check_navigator(arg('navigator')),
                               smoother=vehicle.check_smoother(arg('smoother')),
                               lattice_headings=arg('lattice_headings'),
                               lattice_moves=arg('lattice_moves'),
                               planner_search=arg('planner_search'),
                               diffusion_model=arg('diffusion_model'),
                               diffusion_record=arg('diffusion_record'),
                               diffusion_smoothing=arg('diffusion_smoothing'),
                               # A trial tracker may be swapped live here (the debug
                               # HUD's panel shows which): Gazebo only, never the car.
                               live_swap='true',
                               amcl=amcl, amcl_x=start_x, amcl_y=start_y,
                               amcl_yaw=start_yaw,
                               # The test lot's map for AMCL: finer, outlines only
                               # (maps/README.md). AMCL runs only in the test lot.
                               amcl_map=(os.path.join(nav_share, 'maps',
                                                      'example_lot_amcl.yaml')
                                         if amcl else ''),
                               plan_around=str(flag['plan_around']).lower(),
                               replan_hz=arg('replan_hz'),
                               behaviour=str(flag['behaviour']).lower(),
                               things=str(flag['things']).lower(),
                               lane_map=lane_map))
    if ndt:
        # Scored only (measure controllers --ndt), unless ndt_ekf:=true: then ekf_global
        # reads its pose (measure controllers --ndt-ekf scores the difference).
        here = ({'ndt_min_score': worlds.CAMPUS_NDT_MIN_SCORE,
                 'ndt_min_height': worlds.CAMPUS_NDT_FLOOR} if world == worlds.CAMPUS else {})
        actions.append(include(context, 'aav_ndt', 'ndt.launch.py', use_sim_time='true',
                               ndt_map=ndt_map, ndt_x=start_x, ndt_y=start_y,
                               ndt_yaw=start_yaw, **here))
    if kiss_icp:
        # Scored only (measure controllers --kiss-icp), unless kiss_ekf:=true: then
        # ekf_local reads its twist (measure controllers --kiss-ekf scores the difference).
        actions.append(include(context, 'aav_kiss_icp', 'kiss_icp.launch.py',
                               use_sim_time='true'))
    if flag['watchdog']:
        # What this world has, and the camera only where something reads it: watching it
        # costs one more stream of pictures.
        camera = cameras and (flag['perception'] or flag['camera_obstacles'] or lane != 'false'
                              or flag['faults'])
        inputs = (['odom', 'imu', 'gps', 'local', 'global'] + (['camera'] if camera else [])
                  + (['radar'] if radar else [])
                  + (['hud'] if flag['hud'] or flag['watch_hud'] else []))
        actions.append(include(context, 'aav_safety', 'watchdog.launch.py',
                               inputs=','.join(inputs), use_sim_time='true'))
    if flag['geofence']:
        actions.append(include(context, 'aav_safety', 'geofence.launch.py',
                               backend=('spine' if arg('spine').lower() == 'true' else
                                        'gazebo_esp32' if arg('esp32').lower() == 'true'
                                        else 'gazebo'),
                               radius=fence_radius(arg('geofence_radius'), world),
                               centre_x=start_x,
                               centre_y=start_y, max_speed=sim_speed,
                               use_sim_time='true'))
    if flag['teleop']:
        actions.append(include(context, 'aav_teleop', 'teleop.launch.py', use_sim_time='true'))
    if flag['perception']:
        actions += [
            include(context, 'aav_detector', 'detector.launch.py', labels='stop sign',
                    use_sim_time='true'),
            Node(package='aav_detector', executable='stop_sign_watcher', name='stop_sign_watcher',
                 parameters=[{'use_sim_time': True}]),
        ]
    if flag['camera_obstacles']:
        actions.append(Node(package='aav_detector', executable='camera_obstacles',
                            name='camera_obstacles',
                            parameters=[os.path.join(FindPackageShare('aav_detector')
                                                     .perform(context), 'config',
                                                     'camera_obstacles.yaml'),
                                        {'use_sim_time': True}]))
    if lane in ('true', 'fake'):
        actions.append(include(context, 'aav_lane_geometry', 'lane_follower.launch.py',
                               fake=lane == 'fake'))
    if lane == 'true':
        actions.append(include(context, 'aav_lane_finder', 'lane_finder.launch.py'))
    if flag['hud']:
        actions.append(include(context, 'aav_hud', 'hud.launch.py'))
    if flag['map']:
        actions.append(include(context, 'aav_hud', 'map.launch.py'))
    if flag['foxglove']:
        # This computer only, like the rest of the simulator (tools/ros.sh --local), and
        # the front camera's small preview where the world renders cameras, as on the car.
        actions.append(include(context, 'aav_bringup', 'foxglove.launch.py',
                               foxglove_address='127.0.0.1', foxglove_port=8765,
                               use_sim_time=True, foxglove_control=True,
                               foxglove_topics='wifi',
                               foxglove_preview='front_left' if cameras else ''))
    if flag['rviz']:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(FindPackageShare('aav_bringup').perform(context),
                                          'rviz', LAYOUTS.get(world, 'drive.rviz'))],
            parameters=[{'use_sim_time': True}]))
    return actions


# m: the geofence's circle when geofence_radius is empty. On the campus 200 m held the car
# at a junction 197 m from the start, half way round a tour (15 Nov).
FENCE_RADIUS = 200.0
CAMPUS_FENCE_RADIUS = 600.0


def fence_radius(given, world):
    """Return the geofence's radius as text: the one given, else by the world."""
    if given.strip():
        return given.strip()
    return str(CAMPUS_FENCE_RADIUS if world == worlds.CAMPUS else FENCE_RADIUS)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stack', default_value=stacks.DEFAULT,
                              description='the league (config/stacks.yaml): ours (the '
                              'default), stock (Nav2), best or car sets the planner, path '
                              'follower, costmap, navigator, smoother, behaviour and pose '
                              'at once; a switch given by hand still wins. Empty: every '
                              "switch as given, else the champions' (Nav2's)"),
        DeclareLaunchArgument('world', default_value='empty',
                              description='empty, test_lot or road'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='true if a joystick is plugged into this computer'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('perception', default_value='false',
                              description='the object detector and the stop-sign watcher (the ml '
                                          'environment; world:=road)'),
        DeclareLaunchArgument('lane', default_value='false',
                              description='true: lane keeping (lane_finder and '
                                          'control_interface, on /drivers/ai/cmd_vel; '
                                          'world:=road); fake: the same on a pretend lane'),
        DeclareLaunchArgument('camera_obstacles', default_value='false',
                              description="the detectors' boxes into the local costmap "
                                          '(world:=road)'),
        DeclareLaunchArgument('hud', default_value='false',
                              description='the HUD window (aav_hud); needs a screen'),
        DeclareLaunchArgument('map', default_value='false',
                              description="the big screen's map window (aav_hud); "
                                          'needs a screen'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='foxglove_bridge on 127.0.0.1:8765 (Lichtblick)'),
        DeclareLaunchArgument('obstacle', default_value='false',
                              description="a 1 m box in the car's lane, 15 m ahead"),
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='m: start the car this far ahead (east) of the origin'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description="m: start the car this far left of the road's centre"),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='rad: start the car turned this far left of east '
                                          '(the EKFs still start facing east)'),
        DeclareLaunchArgument('chase_camera', default_value='true',
                              description='a camera behind and above the car, for '
                                          "Lichtblick's cockpit layout (world:=road or "
                                          'test_lot)'),
        DeclareLaunchArgument('cameras', default_value='true',
                              description='false: no cameras, so our simulator runs on a '
                              'computer with no GPU (the parts that read them are refused)'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='a simulated 16-beam roof lidar, on '
                                          '/sensors/lidar/points, read by Nav2 '
                                          '(world:=road or test_lot)'),
        DeclareLaunchArgument('radar', default_value='true',
                              description='the radar stand-in (world:=road or test_lot); '
                                          'false: the lidar alone guards Nav2'),
        DeclareLaunchArgument('actuator_model', default_value='true',
                              description="the actuators as vehicle.yaml's actuators: say; "
                                          "false: Gazebo's old instant car"),
        DeclareLaunchArgument('lag_scale', default_value='1.0',
                              description="the actuator model's lags times this (a "
                                          'robustness test; the drivers keep vehicle.yaml)'),
        DeclareLaunchArgument('speed_loop', default_value='default',
                              description="the speed loop's profile: default (today's gains)"
                                          ', smooth (TODO 27) or lqi (TODO 3)'),
        DeclareLaunchArgument('payload', default_value='0',
                              description='kg aboard the modelled car: it speeds up and '
                                          'brakes less (ego is not told)'),
        DeclareLaunchArgument('adaptive_brake', default_value='false',
                              description='true: brake harder when the car slows less than '
                                          'expected, within vehicle.yaml brake: '
                                          '(docs/stopping.md); not with spine or esp32'),
        DeclareLaunchArgument('spine', default_value='false',
                              description='the spine in the loop: spine_bridge, CAN frames '
                                          "and spine_sim (the spine's own logic) before the "
                                          'actuator model'),
        DeclareLaunchArgument('arm', default_value='true',
                              description='with spine:=true: arm the spine after 12 s'),
        DeclareLaunchArgument('esp32', default_value='false',
                              description="today's car in the loop: legacy_uart_bridge and "
                                          'gazebo_esp32, the C++ firmware standing in'),
        DeclareLaunchArgument('esp32_loop', default_value='pi',
                              description="with esp32:=true: the bridge's speed_loop.mode, "
                                          'pi or open'),
        DeclareLaunchArgument('bridge', default_value=bridges.DEFAULT,
                              description='with spine:=true: spine_bridge in rust '
                                          '(aav_spine_bridge) or python (aav_spine)'),
        DeclareLaunchArgument('watchdog', default_value='true',
                              description='the sensor watchdog: stops Nav2 and the AI slot '
                                          'when a sensor they need fails (aav_safety)'),
        DeclareLaunchArgument('geofence', default_value='true',
                              description='the geofence: holds every program near the edge of '
                                          'a circle round the start (aav_safety)'),
        DeclareLaunchArgument('geofence_radius', default_value='',
                              description="m: the geofence's circle; empty: 600 on the "
                                          'campus (its routes go 250 m and more from the '
                                          "start), else 200 (the car's: 30)"),
        DeclareLaunchArgument('watch_hud', default_value='false',
                              description="the watchdog also wants the HUD's heartbeat, "
                                          'with no HUD window (hud:=true watches it anyway)'),
        DeclareLaunchArgument('faults', default_value='false',
                              description='route the front left picture through '
                                          'gazebo_adapter, for fault.camera'),
        DeclareLaunchArgument('watch_screens', default_value='',
                              description='screens whose heartbeat commands need, such as '
                                          'dev_window,hw_panel: lose one and, with '
                                          'spine:=true, the spine latches'),
        DeclareLaunchArgument('screen_cubes', default_value='false',
                              description='true: the cubes for the developer window, while '
                                          'Nav2 stays on the flat scan'),
        DeclareLaunchArgument('things', default_value='false',
                              description='true: the cubes as tracked things on /world/things '
                                          '(aav_things; with nav and the lidar)'),
        DeclareLaunchArgument('voxels', default_value='false',
                              description='true: Nav2 reads the lidar as 0.25 m cubes '
                                          '(aav_voxels, a trial) instead of the flat scan'),
        DeclareLaunchArgument('ghost', default_value='false',
                              description="a pretend person in Nav2's view (aav_ghost); "
                                          'place it with pixi run ghost ahead 10'),
        DeclareLaunchArgument('sim_speed', default_value='',
                              description='m/s, up to 5: a higher top speed for ego and Nav2 '
                                          "than the car's 1.0, in Gazebo only"),
        DeclareLaunchArgument('reversing', default_value='true',
                              description='Nav2 may drive backwards, at up to '
                                          "vehicle.yaml's max_reverse_speed "
                                          '(docs/learn-nav2.md, Reversing)'),
        DeclareLaunchArgument('controller', default_value=stacks.CHOSEN,
                              description='mppi or rpp (Regulated Pure Pursuit): what '
                                          'follows the path in controller_server, with '
                                          'tracker:=none (docs/path-followers.md). Empty: '
                                          "the stack's choice"),
        DeclareLaunchArgument('planner', default_value=stacks.CHOSEN,
                              description=', '.join(vehicle.PLANNERS) + ': the planner. '
                                          "own (ours), hybrid and lattice keep to the car's "
                                          'turning radius; the others are trials '
                                          "(docs/controllers.md). Empty: the stack's choice"),
        DeclareLaunchArgument('diffusion_model', default_value='',
                              description='with planner:=diffusion: the model (.onnx); '
                                          'empty: models/diffusion/planner.onnx. A file '
                                          "that is not there: every plan the lattice's "
                                          '(the fallback test)'),
        DeclareLaunchArgument('diffusion_smoothing', default_value='gear',
                              description="with planner:=diffusion: gear (each gear's run "
                                          'smoothed on its own) or average (2 Nov)'),
        DeclareLaunchArgument('diffusion_record', default_value='',
                              description="with planner:=diffusion: a folder for each plan's "
                                          'inputs, to replay offline (training/replay.py)'),
        DeclareLaunchArgument('lattice_headings',
                              default_value=str(vehicle.LATTICE_HEADINGS[0]),
                              description='with planner:=lattice: 16 headings (the default) '
                                          'or 32 (a trial file)'),
        DeclareLaunchArgument('lattice_moves', default_value=stacks.CHOSEN,
                              description="with planner:=lattice (or own): nav2, Nav2's "
                                          'moves; smooth or smooth_fast, a trial '
                                          "(tools/lattice_smooth.py). Empty: the stack's "
                                          "choice (nav2's moves)"),
        DeclareLaunchArgument('planner_search', default_value=stacks.CHOSEN,
                              description='with planner:=own: lattice, the State Lattice; '
                                          "hybrid, Nav2's Hybrid-A* in Rust (a challenger). "
                                          "Empty: the stack's choice (the lattice)"),
        DeclareLaunchArgument('behaviour', default_value=stacks.CHOSEN,
                              description='true: the behaviour states, the road ahead and '
                                          'a speed cap (aav_behaviour); false: none. '
                                          "Empty: the stack's choice (on in ours)"),
        DeclareLaunchArgument('lane_map', default_value='',
                              description="with behaviour:=true: world (the world's lane "
                                          "map) or a lanes.yaml: the road ahead's line on "
                                          "the lane's centre; empty: /plan's line"),
        DeclareLaunchArgument('navigator', default_value=stacks.CHOSEN,
                              description="own (aav_navigator, in bt_navigator's place; "
                                          'docs/own-navigator.md) or nav2. Empty: the '
                                          "stack's choice"),
        DeclareLaunchArgument('smoother', default_value=stacks.CHOSEN,
                              description="nav2 or own: our own velocity smoother in Nav2's "
                                          'place, for a car (aav_smoother, a trial; '
                                          "docs/own-smoother.md). Empty: the stack's choice"),
        DeclareLaunchArgument('costmap', default_value=stacks.CHOSEN,
                              description='own (aav_costmap; docs/own-costmap.md) or nav2. '
                                          "Empty: the stack's choice"),
        DeclareLaunchArgument('tracker', default_value=stacks.CHOSEN,
                              description="a path follower in controller_server's "
                                          'place: ' + ', '.join(vehicle.TRACKERS) +
                                          " (aav_track's with reversing:=false; "
                                          'docs/trials.md); none: controller_server. Empty: '
                                          "the stack's choice"),
        DeclareLaunchArgument('amcl', default_value='false',
                              description='stock AMCL beside the EKF, scored only '
                                          '(world:=test_lot nav:=true lidar:=true)'),
        DeclareLaunchArgument('ndt', default_value='false',
                              description='NDT on the saved lidar map, scored only '
                                          '(world:=test_lot lidar:=true, ndt_map present)'),
        DeclareLaunchArgument('ndt_ekf', default_value=stacks.CHOSEN,
                              description='a trial: NDT (ndt:=true implied) and its pose '
                                          "into ekf_global, beside the GPS. Empty: the stack's "
                                          'choice (false)'),
        DeclareLaunchArgument('ndt_map', default_value=DEFAULT_NDT_MAP,
                              description="NDT's map (pixi run ndt-map drive); relative to "
                                          'where the launch starts'),
        DeclareLaunchArgument('kiss_icp', default_value='false',
                              description='KISS-ICP lidar odometry, scored only (a world '
                                          'with the lidar; pixi install --locked -e kiss)'),
        DeclareLaunchArgument('kiss_ekf', default_value=stacks.CHOSEN,
                              description='a trial: KISS-ICP (kiss_icp:=true implied) and its '
                                          'twist into ekf_local, beside the wheels and IMU. '
                                          "Empty: the stack's choice (false)"),
        DeclareLaunchArgument('plan_around', default_value='true',
                              description="true: Nav2's planner plans round what the radar "
                                          'and lidar see; false: the map only, so the car '
                                          'stops for it'),
        DeclareLaunchArgument('truth_pose', default_value='false',
                              description="a test: the simulator's truth as the pose, in "
                                          "ekf_global's place (aav_gazebo truth_localizer)"),
        DeclareLaunchArgument('replan_hz', default_value='1.0',
                              description="how often Nav2's navigate-to-pose tree plans "
                                          'again; 0: once per goal (a test)'),
        DeclareLaunchArgument('simulator', default_value=worlds.DEFAULT_SIMULATOR,
                              description="gazebo, or own: our Rust simulator in gz sim's "
                                          'place (aav_simulator, aav_sim_cameras)'),
        OpaqueFunction(function=launch_setup),
    ])
