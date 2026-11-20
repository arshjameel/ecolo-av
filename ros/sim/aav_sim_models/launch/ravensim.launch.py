"""
Ravensim as the car: the world, the car, its sensors, the adapter and the cameras.

  ros2 launch aav_sim_models ravensim.launch.py [world:=empty|test_lot|road|campus]
      [obstacle:=false] [start_x:=0.0] [start_y:=0.0] [start_yaw:=0.0] [lidar:=true]
      [radar:=true] [cameras:=true] [chase_camera:=true] [camera_lockstep:=auto]
      [actuator_model:=true] [actuators:=] [payload:=0] [adaptive_brake:=false]
      [faults:=false] [spine:=false] [arm:=true]
      [esp32:=false] [esp32_loop:=pi] [bridge:=rust|python] [legacy_impl:=rust|python]

Ravensim is our own simulator (aav_simulator, Rust; docs/own-simulator.md). It reads the
world file (worlds/*.sdf, model:// from models/), moves the car, casts the lidar's and
radar's rays, and publishes the car's raw streams; the nodes here turn them into the real
car's topics:

  /control/sim/actuator_cmd ──> sim_adapter ──> /vehicle/sim/cmd_vel ──> simulator (the car)
                       (the actuator model: vehicle.yaml's actuators:, unless
                        actuator_model:=false)
  simulator ──> /vehicle/sim/odom, imu, navsat ──> sim_adapter, sim_imu, sim_gps
                ──> /vehicle/odom, /sensors/imu/data_raw, /sensors/gps/fix, /vehicle/status
  simulator ──> /joint_states, /clock, /vehicle/sim/truth (where the car really is, for checks)
  simulator ──> /vehicle/sim/radar/points ──> sim_radar ──> /sensors/radar/points
  simulator ──> /sensors/lidar/points (lidar_link), with lidar:=true

The cameras are their own program (aav_sim_cameras, Bevy, offscreen): they draw the same
world file where /vehicle/sim/truth puts the car, on the camera topics (the chase camera
too). No window. cameras:=false leaves them out, so a computer with no GPU runs it.

camera_lockstep:=auto (the default) holds the simulated clock at each camera's due time
until its picture is out, when tools/software_gl.sh draws them (Mesa's lavapipe, no GPU:
too slow for real time). The world then runs slower than real time, and every camera gets
its full rate in sim time. true or false sets it either way.

spine:=true puts the spine in the loop (docs/full-simulation.md step 4): the car's own
spine_bridge takes ego's commands instead of sim_adapter, and spine_sim (car:=ravensim)
runs the spine's firmware logic (spine-core) on its CAN frames,
on the simulated clock:

  /control/sim/actuator_cmd ──> spine_bridge ──> /can/tx ──> spine_sim (the spine's logic)
  spine_sim ──> /vehicle/sim/spine_outputs ──> sim_adapter (the actuator model) ──> Ravensim
  sim_adapter ──> /vehicle/sim/wheels, /vehicle/sim/steering ──> spine_sim
  spine_sim ──> /can/rx ──> spine_bridge ──> /vehicle/odom, status, steering, wheels...

esp32:=true puts today's car in the loop instead: the car's own legacy_uart_bridge takes
ego's commands, and sim_esp32 stands in for the 2025-26 C++ ESP32 firmware (its
throttle ramp, its brake, no command timeout), over a local TCP socket in place of the
serial wire. esp32_loop:= is the bridge's speed_loop.mode: pi (feed-forward plus PI on
the wheel speed, the default here) or open (the 2025-26 mapping):

  /control/sim/actuator_cmd ──> legacy_uart_bridge ──TCP──> sim_esp32 (the firmware)
  sim_esp32 ──> /vehicle/sim/esp32_outputs ──> sim_adapter (the actuator model)
  sim_adapter ──> /vehicle/odom (the wheel speed, as the car's wheel sensors) ──> the bridge

In the sim the bridge's no-timeout lock is lifted (allow_programs_without_timeout), so
programs drive as on a supervised stand: there is no car to run away.

Still one backend: spine_bridge alone reads ego's commands (/control/sim/actuator_cmd,
from the output mux); bridge:= picks its language, rust (aav_spine_bridge, the default)
or python (aav_spine). arm:=true (the default) arms the spine 12 s after start-up, as
the bench does, once the output mux says OPEN (aav_spine's arm_when_open: spine_bridge
refuses while the safety gate is shut); /spine/arm arms it again after a stop. The
hardware e-stop and driver faults are the bench's: /vehicle/bench/hw_estop and
/vehicle/bench/driver_faults.

A world renders the sensors (cameras, lidar, radar stand-in) where it says so
(worlds.has_sensors). chase_camera:=true, the default, adds the camera behind and above
the car, for Lichtblick's cockpit layout (tools/foxglove/cockpit.json), and its JPEG:

  sim_cameras ──> /vehicle/sim/chase_camera/image_raw (640 x 360, 10 Hz)
              ──chase_jpeg──> /vehicle/sim/chase_camera/compressed (JPEG)

lidar:=false drops the roof lidar; radar:=false leaves out /sensors/radar/points.

faults:=true routes the front left camera's picture through sim_camera_relay, and the
roof lidar's cloud through sim_lidar_relay (the simulator's /sensors/lidar/points remapped
to /vehicle/sim/lidar/points), so fault.camera and fault.lidar can freeze or stop them
(faults.py); the other sensors' faults work without it.

obstacle:=true puts the bench's 1 m box in the car's lane, 15 m ahead. start_y and
start_yaw put the car this far left of the world's origin, turned this far left of east
(the EKFs still start at the origin, facing east, as on the bench).

Everything else (sensors filter, localization, our planner, mux, ego) runs as on the car,
with use_sim_time:=true. aav_bringup sim.launch.py starts the lot.
"""


import os
import platform

from aav_description import vehicle
from aav_sim_models import faults, worlds
from aav_spine import bridges
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def spine_in_the_loop(context, v, sim_time):
    """spine_bridge and spine_sim: the car's CAN path, and the spine's own logic."""
    spine_share = FindPackageShare('aav_spine').perform(context)
    actions = [
        Node(package=bridges.package(LaunchConfiguration('bridge').perform(context)),
             executable=bridges.EXECUTABLE, name='spine_bridge', namespace='/',
             parameters=[os.path.join(spine_share, 'config', 'spine.yaml'),
                         vehicle.spine_params(v),
                         # /system/source: everything under it is the simulator's.
                         {'source': 'sim'}, sim_time]),
        Node(package='aav_spine_sim', executable='spine_sim', name='spine_sim',
             namespace='/',
             parameters=[{'car': 'ravensim', 'wheelbase': float(v['wheelbase']),
                          'speed_loop': LaunchConfiguration('speed_loop').perform(context),
                          'track': float(v['track']),
                          'wheel_radius': float(v['wheel_diameter']) / 2,
                          'max_steering': float(v['max_steering_angle'])}, sim_time]),
    ]
    if flag(context, 'arm'):
        # Once the safety gate is OPEN: spine_bridge refuses before. On a steady clock,
        # not the simulated one.
        actions.append(TimerAction(period=12.0, actions=[Node(
            package=vehicle.ARM_WHEN_OPEN_IMPLS[vehicle.check_arm_when_open_impl(
                LaunchConfiguration('arm_when_open_impl').perform(context))],
            executable='arm_when_open', name='arm_when_open',
            output='screen')]))
    return actions


def esp32_in_the_loop(context, car, sim_time):
    """legacy_uart_bridge and sim_esp32: today's UART path, and the C++ firmware's logic."""
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
        Node(package='aav_sim_models', executable='sim_esp32', name='sim_esp32',
             parameters=[{'port': port}, steering, sim_time]),
        # One dict, legacy.yaml's values first: a parameter file's node-specific values
        # would beat any dict given beside it (launch_ros sends dicts as /**).
        # legacy_impl:= picks its language: rust (aav_legacy_rs, the default) or python.
        Node(package=vehicle.LEGACY_IMPLS[vehicle.check_legacy_impl(
                 LaunchConfiguration('legacy_impl').perform(context))],
             executable='legacy_uart_bridge', name='legacy_uart_bridge',
             parameters=[{**legacy, **steering, **model,
                          'port': f'socket://127.0.0.1:{port}', 'source': 'sim',
                          'frame_version': 1, 'allow_programs_without_timeout': True,
                          'speed_loop.mode': LaunchConfiguration('esp32_loop').perform(context),
                          **sim_time}],
             remappings=[('control/car/actuator_cmd', 'control/sim/actuator_cmd'),
                         ('control/car/heartbeat', 'control/sim/heartbeat')]),
    ]


def chase_jpeg(sim_time):
    """Return the chase camera's JPEG for Lichtblick (republish)."""
    raw = '/vehicle/sim/chase_camera/image_raw'
    jpeg = '/vehicle/sim/chase_camera'
    return [
        Node(package='image_transport', executable='republish', name='chase_jpeg',
             parameters=[sim_time, {'in_transport': 'raw', 'out_transport': 'compressed'}],
             # Its compressed publisher's name isn't remapped with "out" (as
             # foxglove.launch.py's preview): name both.
             remappings=[('in', raw), ('out', jpeg), ('out/compressed', f'{jpeg}/compressed')]),
    ]


def model_paths(share):
    """model://: aav_sim_models's models first, then GZ_SIM_RESOURCE_PATH (SDF's own)."""
    return [os.path.join(share, 'models')] + [
        p for p in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(os.pathsep) if p]


def cameras_process(share, world, chase, relay, looks):
    """Ravensim's cameras (aav_sim_cameras): the world file, drawn where the car is."""
    cmd = worlds.sim_cameras_command(worlds.repo_root(__file__), world, model_paths(share),
                                     chase=chase,
                                     relay=faults.CAMERA_RELAYED if relay else '',
                                     looks=looks)
    env = worlds.sim_cameras_env(os.environ, platform.system())
    return ExecuteProcess(cmd=cmd, name='sim_cameras', output='screen',
                          additional_env=env or None)


def simulator_node(context, share, world, car, lidar, radar, lockstep, lidar_relay=False):
    """Ravensim itself (aav_simulator): the world, the car, its rays and its clock."""
    v = car['vehicle']
    mounts = car['sensors']

    def mount(link, part):
        return [float(n) for n in mounts[link][part]]

    models = model_paths(share)
    start = {k: float(LaunchConfiguration(k).perform(context))
             for k in ('start_x', 'start_y', 'start_yaw')}
    x, y = worlds.OBSTACLE
    # faults:=true: the lidar's cloud goes to sim_lidar_relay first, which passes it on
    # through fault.lidar (TODO 246).
    remap = [(faults.LIDAR, faults.LIDAR_RELAYED)] if lidar_relay else []
    return Node(package='aav_simulator', executable='simulator', name='simulator',
                output='screen', remappings=remap,
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
                                                            'model.sdf'),
                             # Software rendering: the clock waits for each picture.
                             'camera_lockstep': lockstep}])


def launch_setup(context):
    share = FindPackageShare('aav_sim_models').perform(context)
    world = worlds.world_file(share, LaunchConfiguration('world').perform(context))
    sensors = worlds.has_sensors(world)
    # Cameras where the world renders sensors (aav_sim_cameras, Bevy). cameras:=false:
    # none at all, so a computer with no GPU runs Ravensim (M2).
    cameras = sensors and flag(context, 'cameras')
    lockstep = cameras and worlds.camera_lockstep(
        LaunchConfiguration('camera_lockstep').perform(context), os.environ)
    looks = worlds.graphics(LaunchConfiguration('graphics').perform(context))
    # The lidar only where the world renders it: elsewhere it would publish nothing.
    lidar = sensors and flag(context, 'lidar')
    chase = cameras and flag(context, 'chase_camera')
    radar = sensors and flag(context, 'radar')
    relay = cameras and flag(context, 'faults')
    lidar_relay = lidar and flag(context, 'faults')
    spine = flag(context, 'spine')
    esp32 = flag(context, 'esp32')
    if spine and esp32:
        raise ValueError('spine:=true and esp32:=true are two backends: pick one')
    if (spine or esp32) and not flag(context, 'actuator_model'):
        raise ValueError('spine:=true and esp32:=true drive the actuator model: '
                         'leave actuator_model:=true')
    if LaunchConfiguration('esp32_loop').perform(context) not in ('pi', 'open'):
        raise ValueError('esp32_loop: pi or open')
    car = vehicle.load()
    v = car['vehicle']
    sim_time = {'use_sim_time': True}
    actions = [
        simulator_node(context, share, world, car, lidar, radar, lockstep, lidar_relay),
        IncludeLaunchDescription(
            os.path.join(FindPackageShare('aav_description').perform(context), 'launch',
                         'description.launch.py'),
            launch_arguments={'use_sim_time': 'true',
                              'joint_state_publisher': 'false'}.items()),
        Node(package='aav_sim_models', executable='sim_adapter', name='sim_adapter',
             parameters=[{'wheelbase': float(v['wheelbase']), 'rendered': sensors,
                          'cameras': cameras,
                          'radar': radar, 'lidar': lidar,
                          'wheel_radius': float(v['wheel_diameter']) / 2.0,
                          'actuator_model': flag(context, 'actuator_model'),
                          'speed_loop': LaunchConfiguration('speed_loop').perform(context),
                          'payload': float(LaunchConfiguration('payload').perform(context)),
                          'adaptive_brake': flag(context, 'adaptive_brake'),
                          'lag_scale': float(LaunchConfiguration('lag_scale').perform(context)),
                          'actuators': LaunchConfiguration('actuators').perform(context),
                          'spine': spine, 'esp32': esp32}, sim_time]),
        # Each sensor in a process of its own, as on the car: none waits behind another.
        Node(package='aav_sim_models', executable='sim_imu', name='sim_imu',
             parameters=[{'imu_x': float(car['sensors']['imu_link']['xyz'][0])}, sim_time]),
        Node(package='aav_sim_models', executable='sim_gps', name='sim_gps',
             parameters=[{'antenna_x': float(car['sensors']['gps_link']['xyz'][0]),
                          'antenna_y': float(car['sensors']['gps_link']['xyz'][1])},
                         sim_time]),
    ]
    if radar:
        actions.append(Node(package='aav_sim_models', executable='sim_radar',
                            name='sim_radar', parameters=[sim_time]))
    if relay:
        actions.append(Node(package='aav_sim_models', executable='sim_camera_relay',
                            name='sim_camera_relay', parameters=[sim_time]))
    if lidar_relay:
        actions.append(Node(package='aav_sim_models', executable='sim_lidar_relay',
                            name='sim_lidar_relay', parameters=[sim_time]))
    if spine:
        actions += spine_in_the_loop(context, v, sim_time)
    if esp32:
        actions += esp32_in_the_loop(context, car, sim_time)
    if cameras:
        # The pictures straight onto the camera topics (the radar's rays are the
        # simulator's own).
        actions.append(cameras_process(share, world, chase, relay, looks))
    if chase:
        # The cameras publish the chase picture: only its JPEG for Lichtblick here.
        actions += chase_jpeg(sim_time)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description='empty, test_lot, road or campus (pixi run campus)'),
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
                              'so Ravensim runs without a GPU; the lidar and radar stay'),
        DeclareLaunchArgument('camera_lockstep', default_value='auto',
                              description='hold the clock at each picture until it is out: '
                                          'auto (when tools/software_gl.sh draws them, no '
                                          'GPU), true or false'),
        DeclareLaunchArgument('graphics', default_value='auto',
                              description='how much the cameras draw: pretty (grain, sky, '
                                          'haze), simple (plain colours, as Gazebo), or '
                                          'auto (pretty on a GPU, simple in software)'),
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
                                          '(sim_adapter); false: the old instant car'),
        DeclareLaunchArgument('speed_loop', default_value='default',
                              description="the speed loop's profile: default (today's gains)"
                                          ', smooth (TODO 27) or lqi (TODO 3); in '
                                          'the actuator model, or spine_sim with spine:=true'),
        DeclareLaunchArgument('payload', default_value='0',
                              description='kg aboard the modelled car (people, cargo): it '
                                          'speeds up and brakes less. Live: sim_adapter '
                                          'payload'),
        DeclareLaunchArgument('adaptive_brake', default_value='false',
                              description='true: the adaptive brake after the actuator '
                                          "model's speed loop (docs/stopping.md); not with "
                                          'spine or esp32'),
        DeclareLaunchArgument('actuators', default_value='',
                              description="the car's own actuator numbers over vehicle.yaml's"
                                          ', "throttle_lag=0.3,drag=0.2": a known car to '
                                          'measure (car-test --hallway --sim); the drivers '
                                          'keep vehicle.yaml'),
        DeclareLaunchArgument('lag_scale', default_value='1.0',
                              description="the actuator model's throttle, brake and steering "
                                          'lags times this (a robustness test)'),
        DeclareLaunchArgument('faults', default_value='false',
                              description='true: the front left picture goes through '
                                          'sim_camera_relay, and the lidar through '
                                          'sim_lidar_relay, so fault.camera and fault.lidar '
                                          'can freeze or stop them'),
        DeclareLaunchArgument('spine', default_value='false',
                              description='the spine in the loop: spine_bridge, CAN frames '
                                          "and spine_sim, the spine's own logic, in front "
                                          'of the actuator model'),
        DeclareLaunchArgument('esp32', default_value='false',
                              description="today's car in the loop: legacy_uart_bridge and "
                                          'sim_esp32, the C++ firmware standing in'),
        DeclareLaunchArgument('esp32_loop', default_value='pi',
                              description="with esp32:=true: the bridge's speed_loop.mode, "
                                          'pi or open'),
        DeclareLaunchArgument('arm_when_open_impl',
                              default_value=vehicle.DEFAULT_ARM_WHEN_OPEN_IMPL,
                              description='rust or python: which arm_when_open arms the spine '
                                          "(aav_spine_bridge's, or aav_spine's)"),
        DeclareLaunchArgument('arm', default_value='true',
                              description='with spine:=true: arm the spine after 12 s'),
        DeclareLaunchArgument('legacy_impl', default_value=vehicle.DEFAULT_LEGACY_IMPL,
                              description='with esp32:=true: legacy_uart_bridge in rust '
                                          '(aav_legacy_rs) or python (aav_legacy)'),
        DeclareLaunchArgument('bridge', default_value=bridges.DEFAULT,
                              description='with spine:=true: spine_bridge in rust '
                                          '(aav_spine_bridge) or python (aav_spine)'),
        DeclareLaunchArgument('noise_seed', default_value='1',
                              description='the sensors noise seed; the same '
                                          'seed, the same streams'),
        DeclareLaunchArgument('wheel_noise', default_value='0',
                              description='m/s of noise on each wheel speed '
                                          'reading (0: off); the odometry then drifts'),
        DeclareLaunchArgument('wheel_slip', default_value='0',
                              description='the wheels read this fraction more '
                                          'than the ground moved (0.02: 2 % too far; 0: off)'),
        DeclareLaunchArgument('clock_hz', default_value='1000',
                              description='/clock a second (1000). '
                                          'Every node on sim time wakes for each one: 250 '
                                          'is lighter, timers then step in 4 ms'),
        OpaqueFunction(function=launch_setup),
    ])
