"""
The whole stack in Gazebo: same nodes and configs as the car, simulated hardware.

  ros2 launch aav_bringup sim.launch.py [world:=empty|test_lot|road] [nav:=true]
      [teleop:=false] [gui:=true] [rviz:=true]
      [perception:=false] [lane:=false|true|fake] [camera_obstacles:=false] [hud:=false]
      [map:=false] [foxglove:=false]
      [obstacle:=false] [start_y:=0.0] [start_yaw:=0.0] [lidar:=true] [radar:=true]
      [actuator_model:=true] [speed_loop:=default|smooth|lqi] [sim_speed:=]
      [reversing:=true] [controller:=mppi] [planner:=hybrid] [tracker:=none]
      [amcl:=false]
      [ndt:=false] [ndt_map:=maps/ndt/test_lot.pcd]
      [plan_around:=true]
      [watchdog:=true] [geofence:=true] [geofence_radius:=200.0]
      [watch_hud:=false] [faults:=false] [spine:=false] [arm:=true]
      [esp32:=false] [esp32_loop:=pi]
      [bridge:=rust|python]
      [ghost:=false]
  pixi run sim world:=road                         the road: the car's camera and radar
  pixi run -e ml sim world:=road perception:=true  adds YOLO and the stop-sign watcher
  pixi run sim world:=test_lot sim_speed:=3 hud:=true   Gazebo, RViz, Nav2 and the HUD window
  pixi run sim world:=test_lot map:=true           ... and the big screen's map window

world:=empty pairs with the empty map (phase 2); world:=test_lot with
maps/example_lot.yaml (phase 3); world:=road, the bench's road in Gazebo, with the empty
map: chosen automatically. The car starts at the world origin facing east, which is also
the map origin (the datum); start_y and start_yaw move it, as on the bench.

The road and test_lot worlds render the car's three cameras, its radar stand-in and the
roof lidar (aav_gazebo worlds.has_sensors), so the car's whole graph runs there, with the
bench's argument names:

  the radar           where the world renders it (radar:=false drops it, to test the
                      lidar alone): radar_clusters, and Nav2's obstacle layer and
                      collision_monitor (navigation.launch.py radar:=true)
  perception:=true    YOLO (the ml environment) and stop_sign_watcher: twist_mux's
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
  obstacle:=true      the bench's 1 m box in the lane, 15 m ahead (aav_gazebo)
  lidar:=false        no roof lidar. It's on by default (15 Oct): a simulated 16-beam
                      lidar on /sensors/lidar/points, which Nav2 reads as a 2D scan
                      (navigation.launch.py lidar:=true: its local costmap and
                      collision_monitor). Its earlier flakiness on a Mac came from other
                      programs competing
  sim_speed:=3        a higher top speed (m/s, up to 5), for ego and Nav2, in Gazebo only:
                      the car's 1.0 m/s cap (ego.yaml, nav2.yaml) is for its first drives.
                      ego refuses it outside a simulator
  controller:=rpp     Nav2 follows the path with Regulated Pure Pursuit, not MPPI (the
                      default): pixi run controller-compare drives both (docs/path-followers.md)
  planner:=lattice    Nav2 plans with another planner (navigation.launch.py planner:=):
                      hybrid (the default) or lattice keep to the car's turning radius;
                      smac2d, navfn and theta_star don't, trials only. pixi run
                      controller-compare --planners scores them (docs/controllers.md)
  tracker:=lqr        a trial path follower (aav_track: pure_pursuit, stanley, lqr or mpc)
                      in controller_server's place, on Nav2's same planner, smoother and
                      collision_monitor. Forward only: with reversing:=false (docs/trials.md)
  amcl:=true          stock AMCL beside the global EKF, scored only (docs/trials.md): only
                      in world:=test_lot with nav:=true and the lidar, from the start pose
  ndt:=true           NDT on the saved lidar map (aav_ndt, Rust), scored only, never TF
                      (docs/designs/2026-10-29_ndt.md): only in world:=test_lot with the
                      lidar, from the start pose, and only once ndt_map (pixi run ndt-map
                      drive) is there
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
                      stopping distance reaches a circle of geofence_radius m (200 here:
                      past every sim check's drive; the car's is 30) round where the car
                      starts (0, start_y), or while the global pose can't be trusted
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

The empty and test_lot worlds render nothing, so they run headless anywhere, and the
camera's parts need world:=road. Every node runs on simulated time.
"""

import os

from aav_description import vehicle
from aav_gazebo import worlds
from aav_spine import bridges
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

MAPS = {'empty': 'empty.yaml', 'test_lot': 'example_lot.yaml', 'road': 'empty.yaml'}
# What each world shows in RViz: the road world adds the camera, the radar and the truth.
LAYOUTS = {'road': 'road.rviz', 'test_lot': 'road.rviz'}
# The parts that read the camera, so need a world that renders it.
CAMERA_PARTS = ('perception', 'camera_obstacles')


def include(context, package, launch_file, **args):
    share = FindPackageShare(package).perform(context)
    return IncludeLaunchDescription(
        os.path.join(share, 'launch', launch_file),
        launch_arguments={k: str(v).lower() if isinstance(v, bool) else str(v)
                          for k, v in args.items()}.items())


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = arg('world')
    if world not in MAPS:
        raise ValueError(f'world:={world}: pick one of {sorted(MAPS)}')
    flag = {n: arg(n).lower() == 'true'
            for n in ('nav', 'teleop', 'rviz', 'perception', 'camera_obstacles', 'hud',
                      'map', 'foxglove', 'reversing', 'watchdog', 'faults',
                      'ghost', 'plan_around', 'watch_hud', 'geofence')}
    lane = arg('lane').lower()
    if lane not in ('false', 'true', 'fake'):
        raise ValueError(f'lane:={lane}: use true (the lane finder), fake or false')
    controller = arg('controller').lower()
    if controller not in vehicle.CONTROLLERS:
        raise ValueError(f'controller:={controller}: pick one of {", ".join(vehicle.CONTROLLERS)}')
    tracker = vehicle.check_tracker(arg('tracker'), flag['reversing'])
    planner = vehicle.check_planner(arg('planner'))
    amcl = arg('amcl').lower() == 'true'
    ndt = arg('ndt').lower() == 'true'
    gazebo_share = FindPackageShare('aav_gazebo').perform(context)
    sensors = worlds.has_sensors(worlds.world_file(gazebo_share, world))
    # Each only where the world renders it, as gazebo.launch.py decides.
    radar = sensors and arg('radar').lower() == 'true'
    lidar = sensors and arg('lidar').lower() == 'true'
    wanted = [n for n in CAMERA_PARTS if flag[n]] + (['lane'] if lane == 'true' else [])
    if amcl and not (world == 'test_lot' and flag['nav'] and lidar):
        raise ValueError('amcl:=true needs world:=test_lot nav:=true lidar:=true: the only '
                         'world with a real map, and AMCL reads the lidar')
    if ndt and not (world == 'test_lot' and lidar):
        raise ValueError('ndt:=true needs world:=test_lot lidar:=true: the only world with '
                         'a lidar map (pixi run ndt-map drive), and NDT reads the lidar')
    if ndt and not os.path.isfile(arg('ndt_map')):
        raise ValueError(f'ndt:=true: no map at ndt_map:={arg("ndt_map")}. Make it with '
                         '`pixi run ndt-map drive`')
    if wanted and not sensors:
        raise ValueError(f'{", ".join(wanted)}: world:={world} has no camera. Use world:=road')
    nav_share = FindPackageShare('aav_navigation').perform(context)
    actions = [
        # Every node on simulated time, those in the included files too.
        SetParameter(name='use_sim_time', value=True),
        include(context, 'aav_gazebo', 'gazebo.launch.py', world=world, gui=arg('gui'),
                obstacle=arg('obstacle'), start_y=arg('start_y'), start_yaw=arg('start_yaw'),
                lidar=arg('lidar'), radar=arg('radar'),
                actuator_model=arg('actuator_model'), speed_loop=arg('speed_loop'),
                faults=flag['faults'],
                spine=arg('spine'), arm=arg('arm'), bridge=arg('bridge'),
                esp32=arg('esp32'), esp32_loop=arg('esp32_loop')),
        include(context, 'aav_command_mux', 'mux.launch.py', use_sim_time='true',
                sensor_lock=flag['watchdog'], geofence_lock=flag['geofence']),
        include(context, 'aav_ego', 'ego.launch.py', use_sim_time='true',
                sim_max_speed=arg('sim_speed')),
        # ego's commands go to Gazebo (gazebo_adapter, or spine_bridge with spine:=true)
        # only while every source is SIM (aav_io_mux).
        include(context, 'aav_io_mux', 'io_mux.launch.py', target='gazebo', source='sim'),
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
        include(context, 'aav_localization', 'localization.launch.py', use_sim_time='true'),
    ]
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
                               collision_monitor=radar or lidar or flag['ghost'],
                               camera_obstacles=flag['camera_obstacles'],
                               max_speed=arg('sim_speed'),
                               reversing=str(flag['reversing']).lower(),
                               controller=controller, tracker=tracker, planner=planner,
                               # A trial tracker may be swapped live here (the debug
                               # HUD's panel shows which): Gazebo only, never the car.
                               live_swap='true',
                               amcl=amcl, amcl_x=0.0, amcl_y=arg('start_y'),
                               amcl_yaw=arg('start_yaw'),
                               # The test lot's map for AMCL: finer, outlines only
                               # (maps/README.md). AMCL runs only in the test lot.
                               amcl_map=(os.path.join(nav_share, 'maps',
                                                      'example_lot_amcl.yaml')
                                         if amcl else ''),
                               plan_around=str(flag['plan_around']).lower()))
    if ndt:
        # Scored only (controller-compare --ndt): nothing reads its pose to drive.
        actions.append(include(context, 'aav_ndt', 'ndt.launch.py', use_sim_time='true',
                               ndt_map=arg('ndt_map'), ndt_x=0.0, ndt_y=arg('start_y'),
                               ndt_yaw=arg('start_yaw')))
    if flag['watchdog']:
        # What this world has, and the camera only where something reads it: watching it
        # costs one more stream of pictures.
        camera = sensors and (flag['perception'] or flag['camera_obstacles'] or lane != 'false'
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
                               radius=arg('geofence_radius'), centre_x='0.0',
                               centre_y=arg('start_y'), max_speed=arg('sim_speed'),
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
                               foxglove_preview='front_left' if sensors else ''))
    if flag['rviz']:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(FindPackageShare('aav_bringup').perform(context),
                                          'rviz', LAYOUTS.get(world, 'drive.rviz'))],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
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
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description="m: start the car this far left of the road's centre"),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='rad: start the car turned this far left of east '
                                          '(the EKFs still start facing east)'),
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
        DeclareLaunchArgument('speed_loop', default_value='default',
                              description="the speed loop's profile: default (today's gains)"
                                          ', smooth (TODO 27) or lqi (TODO 3)'),
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
        DeclareLaunchArgument('geofence_radius', default_value='200.0',
                              description="m: the geofence's circle (the car's: 30)"),
        DeclareLaunchArgument('watch_hud', default_value='false',
                              description="the watchdog also wants the HUD's heartbeat, "
                                          'with no HUD window (hud:=true watches it anyway)'),
        DeclareLaunchArgument('faults', default_value='false',
                              description='route the front left picture through '
                                          'gazebo_adapter, for fault.camera'),
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
        DeclareLaunchArgument('controller', default_value=vehicle.DEFAULT_CONTROLLER,
                              description='mppi or rpp (Regulated Pure Pursuit): what '
                                          "follows Nav2's path (docs/path-followers.md)"),
        DeclareLaunchArgument('planner', default_value=vehicle.DEFAULT_PLANNER,
                              description=', '.join(vehicle.PLANNERS) + ": Nav2's planner. "
                                          "hybrid and lattice keep to the car's turning "
                                          'radius; the others are trials (docs/controllers.md)'),
        DeclareLaunchArgument('tracker', default_value=vehicle.DEFAULT_TRACKER,
                              description="a trial path follower in controller_server's "
                                          'place: ' + ', '.join(vehicle.TRACKERS) +
                                          ' (with reversing:=false; docs/trials.md)'),
        DeclareLaunchArgument('amcl', default_value='false',
                              description='stock AMCL beside the EKF, scored only '
                                          '(world:=test_lot nav:=true lidar:=true)'),
        DeclareLaunchArgument('ndt', default_value='false',
                              description='NDT on the saved lidar map, scored only '
                                          '(world:=test_lot lidar:=true, ndt_map present)'),
        DeclareLaunchArgument('ndt_map', default_value=os.path.join('maps', 'ndt',
                                                                    'test_lot.pcd'),
                              description="NDT's map (pixi run ndt-map drive); relative to "
                                          'where the launch starts'),
        DeclareLaunchArgument('plan_around', default_value='true',
                              description="true: Nav2's planner plans round what the radar "
                                          'and lidar see; false: the map only, so the car '
                                          'stops for it'),
        OpaqueFunction(function=launch_setup),
    ])
