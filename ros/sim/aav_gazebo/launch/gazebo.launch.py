"""
Gazebo Harmonic as the car: world, the car from the URDF, the bridge and the adapter.

  ros2 launch aav_gazebo gazebo.launch.py [world:=empty|test_lot|road|campus] [gui:=true]
      [follow:=true] [obstacle:=false] [start_x:=0.0] [start_y:=0.0] [start_yaw:=0.0] [lidar:=true]
      [radar:=true] [actuator_model:=true] [payload:=0] [adaptive_brake:=false]
      [faults:=false] [spine:=false] [arm:=true]
      [esp32:=false] [esp32_loop:=pi] [chase_camera:=true]
      [bridge:=rust|python] [simulator:=gazebo|own]

  /control/sim/actuator_cmd ──> gazebo_adapter ──> /gazebo/cmd_vel ──bridge──> Ackermann system
                       (the actuator model: vehicle.yaml's actuators:, unless
                        actuator_model:=false)
  Gazebo odometry, IMU, NavSat ──bridge──> gazebo_adapter ──> /vehicle/odom,
                                           /sensors/imu/data_raw, /sensors/gps/fix, /vehicle/status
  Gazebo joints ──bridge──> /joint_states;  Gazebo clock ──bridge──> /clock
  Gazebo's truth ──bridge──> /vehicle/sim/truth (where the car really is, for checks)

spine:=true puts the spine in the loop (docs/full-simulation.md step 4): the car's own
spine_bridge takes ego's commands instead of gazebo_adapter, and spine_sim (car:=gazebo)
runs the spine's firmware logic (spine-core) on its CAN frames, on Gazebo's clock:

  /control/sim/actuator_cmd ──> spine_bridge ──> /can/tx ──> spine_sim (the spine's logic)
  spine_sim ──> /vehicle/sim/spine_outputs ──> gazebo_adapter (the actuator model) ──> Gazebo
  gazebo_adapter ──> /vehicle/sim/wheels, /vehicle/sim/steering ──> spine_sim
  spine_sim ──> /can/rx ──> spine_bridge ──> /vehicle/odom, status, steering, wheels...

esp32:=true puts today's car in the loop instead: the car's own legacy_uart_bridge takes
ego's commands, and gazebo_esp32 stands in for the 2025-26 C++ ESP32 firmware (its
throttle ramp, its brake, no command timeout), over a local TCP socket in place of the
serial wire. esp32_loop:= is the bridge's speed_loop.mode: pi (feed-forward plus PI on
the wheel speed, the default here) or open (the 2025-26 mapping):

  /control/sim/actuator_cmd ──> legacy_uart_bridge ──TCP──> gazebo_esp32 (the firmware)
  gazebo_esp32 ──> /vehicle/sim/esp32_outputs ──> gazebo_adapter (the actuator model)
  gazebo_adapter ──> /vehicle/odom (the wheel speed, as the car's wheel sensors) ──> the bridge

In the sim the bridge's no-timeout lock is lifted (allow_programs_without_timeout), so
programs drive as on a supervised stand: there is no car to run away.

Still one backend: spine_bridge alone reads ego's commands (/control/sim/actuator_cmd,
from the output mux); bridge:= picks its language, rust (aav_spine_bridge, the default)
or python (aav_spine). arm:=true (the default) arms the spine 12 s after start-up, as
the bench does, once the output mux says OPEN (aav_spine's arm_when_open: spine_bridge
refuses while the safety gate is shut); /spine/arm arms it again after a stop. The
hardware e-stop and driver faults are the bench's: /vehicle/bench/hw_estop and
/vehicle/bench/driver_faults.

The road world also renders the car's camera and its radar stand-in, a lidar shaped like
the TI radar (aav_gazebo worlds.has_sensors). A second bridge, gz_road_bridge, carries them:

  camera ──road bridge──> /sensors/camera/front_left/image_raw and camera_info
                          (camera_left_link)
  lidar ──road bridge──> /gazebo/radar/points ──> gazebo_adapter ──> /sensors/radar/points
                                                  (radar_link, as the TI driver's)
  roof lidar ──lidar bridge──> /sensors/lidar/points (lidar_link), with lidar:=true

chase_camera:=true, the default, adds a camera behind and above the car where the world
renders sensors, for Lichtblick's cockpit layout (tools/foxglove/cockpit.json): Gazebo's
picture without Gazebo's window. A third bridge and image_transport's republish:

  chase camera ──chase bridge──> /vehicle/sim/chase_camera/image_raw (640 x 360, 10 Hz)
               ──chase_jpeg────> /vehicle/sim/chase_camera/compressed (JPEG)

lidar:=true, the default, adds a simulated 16-beam roof lidar in a world that renders
sensors; elsewhere it is ignored. lidar:=false drops it, if a busy Mac struggles with its
9 MB/s of clouds (docs/simulation.md). radar:=false leaves out /sensors/radar/points (the
stand-in still renders: gazebo_adapter just doesn't pass it on), to try the lidar alone.

faults:=true routes the front left camera's picture through gazebo_adapter too, so its
fault.camera can freeze or stop it (aav_gazebo faults.py); the other sensors' faults work
without it.

obstacle:=true puts the bench's 1 m box in the car's lane, 15 m ahead. start_y and
start_yaw put the car this far left of the world's origin, turned this far left of east
(the EKFs still start at the origin, facing east, as on the bench).

simulator:=own puts our own Rust simulator (aav_simulator, docs/own-simulator.md) in the
place of gz sim and all its bridges: it reads the same world file and models, and publishes
the bridges' topics (/clock, /gazebo/odom, /gazebo/imu, /gazebo/navsat, /joint_states,
/vehicle/sim/truth, /sensors/lidar/points, /gazebo/radar/points) and takes /gazebo/cmd_vel,
so gazebo_adapter and the sensor nodes above run unchanged. Its cameras are their own
program (aav_sim_cameras, Bevy, offscreen): they draw the same world file where
/vehicle/sim/truth puts the car, on the same camera topics (the chase camera too; with
faults:=true the front left picture goes to gazebo_camera). No window. Ours is the default
since 8 Nov; simulator:=gazebo starts Gazebo (on a computer with no GPU, the only one whose
cameras draw: tools/software_gl.sh).

Everything else (sensors filter, localization, Nav2, mux, ego) runs as on the car,
with use_sim_time:=true. aav_bringup sim.launch.py starts the lot.

On a Mac, Gazebo's window can't run in the same program as the simulator, so it starts
as a second program here: `pixi run sim` works the same on a Mac and on Linux. The
cameras render without a window on a Mac (ogre2 on Metal). On Linux without a window,
the simulator renders with EGL (--headless-rendering). follow:=true points the window's
camera at the car once the window is up.
"""

import atexit
import os
import platform
import sys
import tempfile

from aav_description import vehicle
from aav_gazebo import faults, worlds
from aav_spine import bridges
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

MAC = sys.platform == 'darwin'

# Point the window's camera at the car, close behind it and to its right. Wait for the
# car to appear first: asked too early, the camera ends up looking at nothing. Then keep
# asking until the window answers (it takes a few seconds to start).
CAR = '"aav"'
FOLLOW = ('for i in $(seq 1 60); do gz model --list 2> /dev/null | grep -q -- "- aav$" && '
          'break; sleep 1; done; '
          'for i in $(seq 1 30); do '
          'gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean '
          f"--timeout 2000 --req 'data: {CAR}' > /dev/null 2>&1 && "
          'gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d '
          "--reptype gz.msgs.Boolean --timeout 2000 --req 'x: -3.5, y: -2.5, z: 1.8' "
          '> /dev/null 2>&1 && exit 0; sleep 1; done')


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def spine_in_the_loop(context, v, sim_time):
    """spine_bridge and spine_sim: the car's CAN path, and the spine's own logic."""
    spine_share = FindPackageShare('aav_spine').perform(context)
    actions = [
        Node(package=bridges.package(LaunchConfiguration('bridge').perform(context)),
             executable=bridges.EXECUTABLE, name='spine_bridge',
             parameters=[os.path.join(spine_share, 'config', 'spine.yaml'),
                         vehicle.spine_params(v),
                         # /system/source: everything under it is Gazebo's.
                         {'source': 'sim'}, sim_time]),
        Node(package='aav_spine_sim', executable='spine_sim', name='spine_sim',
             parameters=[{'car': 'gazebo', 'wheelbase': float(v['wheelbase']),
                          'speed_loop': LaunchConfiguration('speed_loop').perform(context),
                          'track': float(v['track']),
                          'wheel_radius': float(v['wheel_diameter']) / 2,
                          'max_steering': float(v['max_steering_angle'])}, sim_time]),
    ]
    if flag(context, 'arm'):
        # Once the safety gate is OPEN: spine_bridge refuses before. On a steady clock,
        # not Gazebo's.
        actions.append(TimerAction(period=12.0, actions=[Node(
            package='aav_spine', executable='arm_when_open', name='arm_when_open',
            output='screen')]))
    return actions


def esp32_in_the_loop(context, car, sim_time):
    """legacy_uart_bridge and gazebo_esp32: today's UART path, and the C++ firmware's logic."""
    legacy_share = FindPackageShare('aav_legacy').perform(context)
    config = os.path.join(legacy_share, 'config', 'legacy.yaml')
    with open(config) as f:
        legacy = yaml.safe_load(f)['legacy_uart_bridge']['ros__parameters']
    v = car['vehicle']
    # One port per ROS domain, so two runs on one machine don't meet.
    port = 47000 + int(os.environ.get('ROS_DOMAIN_ID', '0'))
    steering = {'max_steering_angle': float(v['max_steering_angle']),
                'steering_sign': float(legacy['steering_sign'])}
    model = {f'speed_loop.{k}': float(car['actuators'][k])
             for k in ('max_accel', 'max_brake', 'drag')}
    return [
        Node(package='aav_gazebo', executable='gazebo_esp32', name='gazebo_esp32',
             parameters=[{'port': port}, steering, sim_time]),
        # One dict, legacy.yaml's values first: a parameter file's node-specific values
        # would beat any dict given beside it (launch_ros sends dicts as /**).
        Node(package='aav_legacy', executable='legacy_uart_bridge', name='legacy_uart_bridge',
             parameters=[{**legacy, **steering, **model,
                          'port': f'socket://127.0.0.1:{port}', 'source': 'sim',
                          'frame_version': 1, 'allow_programs_without_timeout': True,
                          'speed_loop.mode': LaunchConfiguration('esp32_loop').perform(context),
                          **sim_time}],
             remappings=[('control/car/actuator_cmd', 'control/sim/actuator_cmd'),
                         ('control/car/heartbeat', 'control/sim/heartbeat')]),
    ]


def chase_camera(share, sim_time):
    """Return the chase camera's bridge, and its JPEG for Lichtblick (republish)."""
    raw = '/vehicle/sim/chase_camera/image_raw'
    jpeg = '/vehicle/sim/chase_camera'
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_chase_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge_chase.yaml')},
                         sim_time]),
        Node(package='image_transport', executable='republish', name='chase_jpeg',
             parameters=[sim_time, {'in_transport': 'raw', 'out_transport': 'compressed'}],
             # Its compressed publisher's name isn't remapped with "out" (as
             # foxglove.launch.py's preview): name both.
             remappings=[('in', raw), ('out', jpeg), ('out/compressed', f'{jpeg}/compressed')]),
    ]


def model_paths(share):
    """model://: aav_gazebo's models first, then wherever Gazebo would look."""
    return [os.path.join(share, 'models')] + [
        p for p in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(os.pathsep) if p]


def own_cameras(share, world, chase, relay):
    """Our simulator's cameras (aav_sim_cameras): the world file, drawn where the car is."""
    cmd = worlds.sim_cameras_command(worlds.repo_root(__file__), world, model_paths(share),
                                     chase=chase,
                                     relay=faults.CAMERA_RELAYED if relay else '')
    env = worlds.sim_cameras_env(os.environ, platform.system())
    return ExecuteProcess(cmd=cmd, name='sim_cameras', output='screen',
                          additional_env=env or None)


def own_simulator(context, share, world, car, lidar, radar):
    """Our Rust simulator, in the place of gz sim, the car's spawn and every bridge."""
    v = car['vehicle']
    mounts = car['sensors']

    def mount(link, part):
        return [float(n) for n in mounts[link][part]]

    models = model_paths(share)
    start = {k: float(LaunchConfiguration(k).perform(context))
             for k in ('start_x', 'start_y', 'start_yaw')}
    x, y = worlds.OBSTACLE
    return Node(package='aav_simulator', executable='simulator', name='simulator',
                output='screen',
                parameters=[{'world': world, 'models': ':'.join(models), **start,
                             'clock_hz': float(LaunchConfiguration('clock_hz').perform(context)),
                             # TODO 179: noisy, slipping wheels, seeded; off by default.
                             'seed': int(LaunchConfiguration('noise_seed').perform(context)),
                             'wheel_noise': float(
                                 LaunchConfiguration('wheel_noise').perform(context)),
                             'wheel_slip': float(
                                 LaunchConfiguration('wheel_slip').perform(context)),
                             'wheelbase': float(v['wheelbase']), 'track': float(v['track']),
                             'wheel_radius': float(v['wheel_diameter']) / 2.0,
                             'max_steering': float(v['max_steering_angle']),
                             'imu_xyz': mount('imu_link', 'xyz'),
                             'imu_rpy': mount('imu_link', 'rpy'),
                             'gps_xyz': mount('gps_link', 'xyz'),
                             'lidar_xyz': mount('lidar_link', 'xyz'),
                             'lidar_rpy': mount('lidar_link', 'rpy'),
                             'radar_xyz': mount('radar_link', 'xyz'),
                             'radar_rpy': mount('radar_link', 'rpy'),
                             'lidar': lidar, 'radar': radar,
                             'obstacle': flag(context, 'obstacle'),
                             'obstacle_x': x, 'obstacle_y': y,
                             'obstacle_model': os.path.join(share, 'models', 'obstacle',
                                                            'model.sdf')}])


def launch_setup(context):
    share = FindPackageShare('aav_gazebo').perform(context)
    world = worlds.world_file(share, LaunchConfiguration('world').perform(context))
    simulator = worlds.check_simulator(LaunchConfiguration('simulator').perform(context))
    own = simulator == 'own'
    sensors = worlds.has_sensors(world)
    # Cameras where the world renders sensors: Gazebo's, or ours (aav_sim_cameras, Bevy).
    # cameras:=false: none at all, so a computer with no GPU runs our simulator (M2).
    cameras = worlds.has_cameras(world, simulator) and flag(context, 'cameras')
    # The lidar only where the world renders it: elsewhere it would publish nothing.
    lidar = sensors and flag(context, 'lidar')
    chase = cameras and flag(context, 'chase_camera')
    radar = sensors and flag(context, 'radar')
    relay = cameras and flag(context, 'faults')
    spine = flag(context, 'spine')
    esp32 = flag(context, 'esp32')
    if spine and esp32:
        raise ValueError('spine:=true and esp32:=true are two backends: pick one')
    if (spine or esp32) and not flag(context, 'actuator_model'):
        raise ValueError('spine:=true and esp32:=true drive the actuator model: '
                         'leave actuator_model:=true')
    if LaunchConfiguration('esp32_loop').perform(context) not in ('pi', 'open'):
        raise ValueError('esp32_loop: pi or open')
    gui = flag(context, 'gui')
    # -r: start running; -s: server only (no window). On a Mac the window is always its
    # own program (below). On Linux with no window, the sensors render through EGL.
    if MAC or not gui:
        gz_args = '-r -s ' + ('--headless-rendering ' if sensors and not MAC else '') + world
    else:
        gz_args = '-r ' + world
    car = vehicle.load()
    v = car['vehicle']
    sim_time = {'use_sim_time': True}
    start = ['-x', LaunchConfiguration('start_x').perform(context),
             '-y', LaunchConfiguration('start_y').perform(context),
             '-Y', LaunchConfiguration('start_yaw').perform(context)]
    if own:
        actions = [own_simulator(context, share, world, car, lidar, radar)]
    else:
        actions = [
            IncludeLaunchDescription(
                os.path.join(FindPackageShare('ros_gz_sim').perform(context), 'launch',
                             'gz_sim.launch.py'),
                launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
            Node(package='ros_gz_sim', executable='create', name='spawn_aav', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'aav', '-z', '0.05',
                            *start]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                 parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')},
                             sim_time]),
        ]
    actions += [
        IncludeLaunchDescription(
            os.path.join(FindPackageShare('aav_description').perform(context), 'launch',
                         'description.launch.py'),
            launch_arguments={'sim': 'true', 'use_sim_time': 'true',
                              'joint_state_publisher': 'false',
                              'lidar': str(lidar).lower(),
                              'chase_camera': str(chase).lower()}.items()),
        Node(package='aav_gazebo', executable='gazebo_adapter', name='gazebo_adapter',
             parameters=[{'wheelbase': float(v['wheelbase']), 'rendered': sensors,
                          'cameras': cameras,
                          'radar': radar, 'lidar': lidar,
                          'wheel_radius': float(v['wheel_diameter']) / 2.0,
                          'actuator_model': flag(context, 'actuator_model'),
                          'speed_loop': LaunchConfiguration('speed_loop').perform(context),
                          'payload': float(LaunchConfiguration('payload').perform(context)),
                          'adaptive_brake': flag(context, 'adaptive_brake'),
                          'lag_scale': float(LaunchConfiguration('lag_scale').perform(context)),
                          'spine': spine, 'esp32': esp32}, sim_time]),
        # Each sensor in a process of its own, as on the car: none waits behind another.
        Node(package='aav_gazebo', executable='gazebo_imu', name='gazebo_imu',
             parameters=[{'imu_x': float(car['sensors']['imu_link']['xyz'][0])}, sim_time]),
        Node(package='aav_gazebo', executable='gazebo_gps', name='gazebo_gps',
             parameters=[{'antenna_x': float(car['sensors']['gps_link']['xyz'][0]),
                          'antenna_y': float(car['sensors']['gps_link']['xyz'][1])},
                         sim_time]),
    ]
    if radar:
        actions.append(Node(package='aav_gazebo', executable='gazebo_radar',
                            name='gazebo_radar', parameters=[sim_time]))
    if relay:
        actions.append(Node(package='aav_gazebo', executable='gazebo_camera',
                            name='gazebo_camera', parameters=[sim_time]))
    if spine:
        actions += spine_in_the_loop(context, v, sim_time)
    if esp32:
        actions += esp32_in_the_loop(context, car, sim_time)
    if gui and MAC and not own:
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g'], name='gz_window',
                                      output='screen'))
    if gui and flag(context, 'follow') and not own:
        actions.append(ExecuteProcess(cmd=[FOLLOW], shell=True, name='follow_car'))
    if cameras and own:
        # Ours: the pictures straight onto the camera topics (the radar's rays are the
        # simulator's own), so no road bridge.
        actions.append(own_cameras(share, world, chase, relay))
    elif cameras:
        bridge = os.path.join(share, 'config', 'bridge_road.yaml')
        if relay:
            # The front left picture to gazebo_camera, which passes it on through its fault.
            with open(bridge) as f:
                entries = faults.relayed(yaml.safe_load(f))
            out = tempfile.NamedTemporaryFile('w', prefix='bridge_road_', suffix='.yaml',
                                              delete=False)
            with out:
                yaml.safe_dump(entries, out)
            atexit.register(os.remove, out.name)    # read as the bridge starts
            bridge = out.name
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_road_bridge',
            parameters=[{'config_file': bridge}, sim_time]))
    if lidar and not own:
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_lidar_bridge',
            parameters=[{'config_file': os.path.join(share, 'config', 'bridge_lidar.yaml')},
                        sim_time]))
    if chase:
        # Ours publishes the chase picture itself: only the JPEG for Lichtblick.
        actions += chase_camera(share, sim_time)[1 if own else 0:]
    if flag(context, 'obstacle') and not own:
        x, y = worlds.OBSTACLE
        actions.append(Node(
            package='ros_gz_sim', executable='create', name='spawn_obstacle', output='screen',
            arguments=['-file', os.path.join(share, 'models', 'obstacle', 'model.sdf'),
                       '-name', 'obstacle', '-x', str(x), '-y', str(y)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description='empty, test_lot, road or campus (pixi run campus)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('follow', default_value='true',
                              description="point the window's camera at the car"),
        DeclareLaunchArgument('obstacle', default_value='false',
                              description="the bench's 1 m box in the car's lane, 15 m ahead"),
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='m: start the car this far ahead (east) of the '
                                          "world's origin"),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description='m: start the car this far left (north) of the '
                                          "world's origin"),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='rad: start the car turned this far left of east'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='a simulated 16-beam roof lidar, on '
                                          '/sensors/lidar/points (world:=road)'),
        DeclareLaunchArgument('cameras', default_value='true',
                              description='false: no cameras (no picture, no chase camera), '
                              'so our simulator runs without a GPU; the lidar and radar stay'),
        DeclareLaunchArgument('chase_camera', default_value='true',
                              description='a camera behind and above the car, for '
                                          "Lichtblick's cockpit layout, as "
                                          '/vehicle/sim/chase_camera/compressed '
                                          '(worlds that render sensors)'),
        DeclareLaunchArgument('radar', default_value='true',
                              description='pass the radar stand-in on, as '
                                          '/sensors/radar/points (world:=road)'),
        DeclareLaunchArgument('actuator_model', default_value='true',
                              description="the actuators as vehicle.yaml's actuators: say "
                                          "(gazebo_adapter); false: Gazebo's old instant car"),
        DeclareLaunchArgument('speed_loop', default_value='default',
                              description="the speed loop's profile: default (today's gains)"
                                          ', smooth (TODO 27) or lqi (TODO 3); in '
                                          'the actuator model, or spine_sim with spine:=true'),
        DeclareLaunchArgument('payload', default_value='0',
                              description='kg aboard the modelled car (people, cargo): it '
                                          'speeds up and brakes less. Live: gazebo_adapter '
                                          'payload'),
        DeclareLaunchArgument('adaptive_brake', default_value='false',
                              description='true: the adaptive brake after the actuator '
                                          "model's speed loop (docs/stopping.md); not with "
                                          'spine or esp32'),
        DeclareLaunchArgument('lag_scale', default_value='1.0',
                              description="the actuator model's throttle, brake and steering "
                                          'lags times this (a robustness test)'),
        DeclareLaunchArgument('faults', default_value='false',
                              description='true: the front left picture goes through '
                                          'gazebo_adapter, so fault.camera can freeze it'),
        DeclareLaunchArgument('spine', default_value='false',
                              description='the spine in the loop: spine_bridge, CAN frames '
                                          "and spine_sim, the spine's own logic, in front "
                                          'of the actuator model'),
        DeclareLaunchArgument('esp32', default_value='false',
                              description="today's car in the loop: legacy_uart_bridge and "
                                          'gazebo_esp32, the C++ firmware standing in'),
        DeclareLaunchArgument('esp32_loop', default_value='pi',
                              description="with esp32:=true: the bridge's speed_loop.mode, "
                                          'pi or open'),
        DeclareLaunchArgument('arm', default_value='true',
                              description='with spine:=true: arm the spine after 12 s'),
        DeclareLaunchArgument('bridge', default_value=bridges.DEFAULT,
                              description='with spine:=true: spine_bridge in rust '
                                          '(aav_spine_bridge) or python (aav_spine)'),
        DeclareLaunchArgument('noise_seed', default_value='1',
                              description='simulator:=own: the sensors noise seed; the same '
                                          'seed, the same streams'),
        DeclareLaunchArgument('wheel_noise', default_value='0',
                              description='simulator:=own: m/s of noise on each wheel speed '
                                          'reading (0: off); the odometry then drifts'),
        DeclareLaunchArgument('wheel_slip', default_value='0',
                              description='simulator:=own: the wheels read this fraction more '
                                          'than the ground moved (0.02: 2 % too far; 0: off)'),
        DeclareLaunchArgument('clock_hz', default_value='1000',
                              description='simulator:=own: /clock a second (1000, as Gazebo). '
                                          'Every node on sim time wakes for each one: 250 '
                                          'is lighter, timers then step in 4 ms'),
        DeclareLaunchArgument('simulator', default_value=worlds.DEFAULT_SIMULATOR,
                              description='own (the default): our Rust simulator '
                                          '(aav_simulator) in the place of gz sim and its '
                                          'bridges, its cameras by Bevy (aav_sim_cameras); '
                                          'or gazebo'),
        OpaqueFunction(function=launch_setup),
    ])
