"""
The real car. One launch file for every machine; a machine file picks the parts.

  ros2 launch aav_bringup car.launch.py machine:=orin
      [gps:=true] [gps_driver:=ublox] [map:=<map yaml>] [start_x:=0] [start_y:=0]
      [start_yaw:=0] [max_speed:=0.3]
      [reversing:=false]  (off on the car by default (26 Oct): the wheel sensors can't
                           tell direction yet (H5). true: Nav2 may back up, at 0.5 m/s at
                           most, where the machine runs something that sees behind: the
                           lidar, or camera_obstacles with a detector on the rear camera
                           that keeps people. Without, it stays forward only and says why)
      [planner:=hybrid]   (Nav2's planner: hybrid (SmacPlannerHybrid) or lattice
                           (SmacPlannerLattice, our primitives). Only these two on the car:
                           smac2d, navfn and theta_star ignore the turning radius, and are
                           refused here, with no way round it (vehicle.check_planner real)
      [lidar:=true]       (false: no lidar anywhere, whatever the machine file says)
      [ghost:=false]      (true: a pretend person in Nav2's view, where Nav2 runs, for
                           stopping tests with nothing real to hit: pixi run ghost ahead 10)
      [geofence_radius:=30.0] [geofence_site:=] [stands:=false]
                          (the geofence, beside the mux, always: every program (scripted,
                           replay, point, Nav2, AI; never a person) stops before its
                           stopping distance reaches a circle of geofence_radius m round
                           the marked start spot (start_x, start_y), or while the global
                           pose or the GPS is stale, unsure or jumped. geofence_site: a
                           polygon in aav_safety's config/geofence.yaml instead (else the
                           machine file's geofence: site:; the test lot's is a TODO for
                           the team, once its datum is surveyed). With actuators:=uart in
                           speed_loop open the car coasts: every program stays held.
                           stands:=true: the wheels are off the ground, so NO fence, said
                           loudly; gps:=false needs it. docs/geofence.md)
      [watch_hud:=true]   (the safety drivers rely on the HUD: the sensor watchdog stops
                           Nav2 and the AI slot while its /hud/heartbeat is missing. false:
                           a session with no HUD screen, said in the test card)
      [bridge:=rust]      (spine_bridge's language, where the machine runs the spine: rust,
                           aav_spine_bridge; or python, aav_spine's)
      [actuators:=uart]   (who drives the car: docs/can-plan.md. uart, the car as wired
                           today (Ben, 23 Oct; the hardware lead confirms): the ESP32,
                           through legacy_uart_bridge on the Motor Pi, and the Orin's CAN
                           bus for sensors only (spine_bridge role:=sensors). can: the
                           KEFC plan's CAN spine drives, and legacy_uart doesn't start.
                           Give every machine the same value)

machine: config/machines/<machine>.yaml lists which parts run on that computer
(orin, motor_pi, laptop, all), and the detector's and the camera streams' settings
there. Every machine uses the same ROS_DOMAIN_ID, so the parts find each other over the
network; see docs/platforms.md. The arguments win over the machine file: gps:=false starts
no GPS driver anywhere, and lidar:=false no lidar. A part that can't run here (no
PyTorch, no model file, no usb_cam, no decoder program) is skipped with a warning.

Parts and what they start:
  description     robot_state_publisher (+ joint_state_publisher): the car's frames
  mux, ego        twist_mux -> /control/cmd_vel -> ego -> /ego/actuator_cmd; with the
                  mux, the sensor watchdog (aav_safety): it stops Nav2 and the AI slot when
                  a sensor they need fails, or the HUD's screen (watch_hud), and they stop
                  too if it goes quiet; and the geofence (aav_safety): every program stops
                  before the fence's edge, or with no trusted global pose. With ego,
                  the sensor mux and the output mux
                  (aav_io_mux, target car): ego's commands reach /control/car/actuator_cmd,
                  and so the backends, only while every source is REAL
  teleop          joystick -> /drivers/joy/cmd_vel, e-stop button -> /safety/estop
  scripted        /scripted_drive action -> /drivers/scripted/cmd_vel
  drive_to_point  /drive_to_point action (drive to a GPS point, stop there)
                  -> /drivers/point/cmd_vel
  imu, gps,       sensor drivers (on the machine the device is plugged into);
  radar, battery  gps_driver picks the GPS's: ublox (the default), nmea or ublox_dgnss
  lidar           the roof Velodyne VLP-16, through the stock velodyne driver ->
                  /sensors/lidar/points. On the machine its packets go to: the Jetson
  camera          usb_cam -> /sensors/camera/front_left/image_raw (Linux only)
  camera_stream   the front cameras' H.264 streams from the camera Pi's MediaMTX (RTSP,
                  port 8554): one camera_stream_<camera> node each (aav_camera_stream),
                  which decodes, undistorts and runs its own detector ->
                  /perception/detections, with the machine file's host, decoder and model.
                  Where the mux runs too, the sensor watchdog watches each camera's
                  detections (<camera>_stream). The pictures stay inside: image_raw only
                  with the lane part (publish_image). docs/camera-pipeline.md
  detector        YOLO on /sensors/camera/front_left/image_raw -> /perception/detections, as
                  object_detector_<machine>, with the machine file's model. Not for a camera
                  the camera_stream part streams: its own detector replaces this one
  stop_signs      stop_sign_watcher -> /perception/stop_sign, twist_mux's stop-sign lock (the
                  lock is in aav_command_mux's config). One per car, next to the mux
  radar_clusters  DBSCAN (aav_radar_clusters, Rust) on /sensors/radar/points ->
                  /perception/radar/clusters, /perception/radar/cluster_boxes
  camera_obstacles
                  the detectors' boxes as points on the road -> /perception/camera_obstacles,
                  for Nav2's costmap. Off until the cameras are calibrated (Q22)
  lane            lane keeping: lane_finder on /sensors/camera/front_left/image_raw ->
                  /perception/lane/geometry, and
                  control_interface -> /drivers/ai/cmd_vel (twist_mux's AI slot). Off on every
                  machine: it drives whenever it sees a lane, unless a driver above it does
  localization    ekf_local, ekf_global, navsat_transform
  navigation      Nav2 -> /drivers/nav/cmd_vel. Nav2's local costmap reads the sensors
                  this machine runs: with radar, /sensors/radar/points, and
                  collision_monitor stops Nav2 for radar points just ahead; with
                  camera_obstacles, the camera's points; with lidar, /sensors/lidar/points
                  (and collision_monitor stops Nav2 for lidar points, or a silent lidar)
  spine           the CAN bus, on the machine it is wired to (the Orin): ros2_socketcan +
                  spine_bridge. actuators:=uart: role sensors, the wheel nodes' frames
                  into /vehicle/odom and /vehicle/wheels, nothing sent (no CAN sender
                  starts). actuators:=can: role spine, the backend. Linux only (SocketCAN)
  legacy_uart     backend with actuators:=uart: today's ESP32 over UART (the Pi wired to it)
  legacy_encoder  the 2025-26 wheel encoder over UDP (a Pico 2). Not where the spine part
                  runs: both would publish /vehicle/odom, and the output mux stops for two
  foxglove        foxglove_bridge on port 8765 (foxglove.launch.py), for Foxglove or
                  Lichtblick on a laptop: watch only, and no raw cameras or full lidar
                  over the WiFi (docs/foxglove.md)

The detector runs as `python3 -m aav_detector.detector`, the python3 of the environment that
starts this file: on a laptop, the ml environment's PyTorch (pixi run -e ml car laptop);
on the Orin, JetPack's (docs/perception.md).
"""

import os
import re
import shutil
import subprocess

from aav_description import vehicle
from aav_spine import bridges
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import evaluate_condition_expression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import yaml

PARTS = ('description', 'mux', 'ego', 'teleop', 'scripted', 'drive_to_point', 'imu', 'gps',
         'radar', 'lidar', 'battery', 'camera', 'camera_stream', 'detector', 'stop_signs',
         'radar_clusters',
         'camera_obstacles', 'lane', 'localization', 'navigation', 'spine', 'legacy_uart',
         'legacy_encoder', 'foxglove')
# What a machine file holds: the parts, the detector's settings and the camera streams'.
SECTIONS = ('parts', 'detector', 'camera_stream', 'geofence')
# Each model format: what runs it, besides PyTorch and Ultralytics, and how to make it.
FORMATS = {
    '.pth': ([], 'pixi run get-models'),
    '.pt': ([], 'pixi run get-models'),
    '.mlpackage': (['coremltools'], 'pixi run -e ml export-coreml [its .pth or .pt] (on a Mac)'),
    '.engine': (['tensorrt'], 'python3 tools/build_engine.py [its .pth or .pt] (on this '
                              'computer, with the python3 of JetPack)'),
}
# Prints the modules a python3 lacks. find_spec looks without importing: no time at all.
MISSING = ('import importlib.util as u, sys; '
           'print(" ".join(m for m in sys.argv[1:] if not u.find_spec(m)))')


def launch_file(package, name):
    return os.path.join(get_package_share_directory(package), 'launch', name)


def include(package, name, **args):
    # Pass every argument the included file needs: included files share launch
    # arguments, so one left out may arrive from another file.
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file(package, name)),
        launch_arguments={k: str(v).lower() if isinstance(v, bool) else str(v)
                          for k, v in args.items()}.items())


def installed(package):
    try:
        get_package_share_directory(package)
    except PackageNotFoundError:
        return False
    return True


def skipped(part, why):
    """Warn that a part the machine file asks for can't run here; the rest still starts."""
    get_logger('car.launch.py').warning(f'{part}: not started: {why}')
    return []


def workspace():
    """Return this repository: Pixi's project folder, or the folder holding colcon's install/."""
    if os.environ.get('PIXI_PROJECT_ROOT'):
        return os.environ['PIXI_PROJECT_ROOT']
    head, found, _ = get_package_share_directory('aav_bringup').partition(
        f'{os.sep}install{os.sep}')
    return head if found else os.getcwd()


def load_machine(machine):
    folder = os.path.join(get_package_share_directory('aav_bringup'), 'config', 'machines')
    path = os.path.join(folder, f'{machine}.yaml')
    if not os.path.isfile(path):
        known = sorted(f[:-len('.yaml')] for f in os.listdir(folder) if f.endswith('.yaml'))
        raise RuntimeError(f'machine:={machine}: no such machine file. Pick one of {known}')
    with open(path) as f:
        config = yaml.safe_load(f)
    for what, found, known in (('sections', config, SECTIONS),
                               ('parts', config['parts'], PARTS)):
        unknown = set(found) - set(known)
        if unknown:
            raise RuntimeError(f'{machine}.yaml: unknown {what} {sorted(unknown)}')
    return config


def lacks(modules):
    """Return the python3 on PATH, which the detector runs in, and the modules it lacks."""
    python = shutil.which('python3') or 'python3'
    return python, subprocess.run([python, '-c', MISSING, *modules],
                                  capture_output=True, text=True).stdout.split()


def found(program):
    """Return the program's path if this computer has it (an absolute path, or on PATH)."""
    return shutil.which(program)


def arguments(context, machine, package, name, settings, what, fixed=()):
    """
    Return every argument of a launch file, at its own defaults: to pass them all.

    Passing every one means none can leak in from another file. A machine file setting
    the file doesn't take, or one of `fixed` (the car decides those), is an error.
    """
    declared = (PythonLaunchDescriptionSource(launch_file(package, name))
                .get_launch_description(context).get_launch_arguments())
    unknown = set(settings) - ({a.name for a in declared} - set(fixed))
    if unknown:
        raise RuntimeError(f'{machine}.yaml: {what} takes no {sorted(unknown)}')
    return {a.name: perform_substitutions(context, a.default_value) for a in declared
            if a.default_value is not None}


def check_model(machine, kind, model):
    """
    Return (the model file, made absolute, None), or (None, why it can't run here).

    A detector and a model file that don't go together is an error: the machine file is
    wrong, on every computer.
    """
    from aav_detector.boxes import check_detector   # here: only machines with a detector need it
    try:
        model = check_detector(kind, model)
    except ValueError as e:
        raise RuntimeError(f'{machine}.yaml: {e}')
    model = os.path.join(workspace(), os.path.expanduser(model))
    runtime, make = FORMATS.get(os.path.splitext(model)[1], ([], 'docs/perception.md'))
    library = 'rfdetr' if kind.strip().lower() == 'rfdetr' else 'ultralytics'
    python, missing = lacks(['torch', library, *runtime])
    if missing:
        return None, (f'{python} has no {", ".join(missing)}. On a laptop: '
                      f'pixi run -e ml car {machine}. On the Orin: '
                      "JetPack's PyTorch and TensorRT (docs/perception.md)")
    if not os.path.exists(model):
        return None, f'{model} is missing. Make it with {make}'
    return model, None


def detector(context, machine, settings):
    """Return the detector part, as object_detector_<machine>, or why it can't start."""
    if not isinstance(settings, dict):
        raise RuntimeError(f'{machine}.yaml: give the detector settings, such as '
                           f'"detector: {{detector: rfdetr}}"')
    defaults = arguments(context, machine, 'aav_detector', 'detector.launch.py', settings,
                         'the detector', fixed=('node_name', 'picture'))
    args = {**defaults, 'labels': 'stop sign', **settings}
    model, why = check_model(machine, str(args['detector']), str(args['model']))
    if why:
        return skipped('detector', why)
    args.update(model=model, picture='',
                node_name='object_detector_' + re.sub(r'\W', '_', machine))
    return [include('aav_detector', 'detector.launch.py', **args)]


def image_topic(camera):
    return f'/sensors/camera/{camera}/image_raw'


STREAMED = 'front_left,front_right'     # camera_stream.launch.py's own default


def streamed(settings):
    """Return the cameras a machine file's camera_stream settings name (or the default two)."""
    cameras = settings.get('cameras', STREAMED) if isinstance(settings, dict) else STREAMED
    return [c.strip() for c in str(cameras).split(',') if c.strip()]


def detector_images(settings):
    """Return every camera topic the detector part reads, from its machine file settings."""
    settings = settings if isinstance(settings, dict) else {}
    extra = [t.strip() for t in str(settings.get('extra_images', '')).split(',') if t.strip()]
    return [str(settings.get('image', image_topic('front_left'))), *extra]


def stream_labels(obstacles):
    """
    Return the labels the streams keep: stop signs, and with camera_obstacles its classes.

    The car's detector part keeps stop signs only, for stop_sign_watcher; camera_obstacles
    needs the road users too.
    """
    labels = ['stop sign']
    if obstacles:
        path = os.path.join(get_package_share_directory('aav_detector'), 'config',
                            'camera_obstacles.yaml')
        with open(path) as f:
            classes = yaml.safe_load(f)['camera_obstacles']['ros__parameters']['classes']
        labels += [c for c in classes if c not in labels]
    return ','.join(labels)


# The car decides these: its sources are real, no stream check's time code, no sim clock.
STREAM_FIXED = ('source', 'time_code', 'use_sim_time')


def camera_streams(context, machine, settings, labels, publish_image):
    """
    Return the camera_stream part and the cameras it streams; or skip it, saying why.

    One camera_stream_<camera> node per camera (aav_camera_stream camera_stream.launch.py),
    each pulling rtsp://<host>:8554/<camera> from the camera Pi and running its own
    detector. Skipped, like the detector part, where the decoder program, PyTorch or the
    model file is missing: a stream without its detector would look healthy and see no
    stop signs. detector: none in the machine file runs the streams without, on purpose.
    """
    if not isinstance(settings, dict) or not settings.get('host'):
        raise RuntimeError(f"{machine}.yaml: give the camera streams' settings, with the "
                           "camera Pi's address, such as "
                           '"camera_stream: {host: 192.168.1.108, decoder: gst-nvidia}"')
    defaults = arguments(context, machine, 'aav_camera_stream', 'camera_stream.launch.py',
                         settings, 'camera_stream', fixed=STREAM_FIXED)
    args = {**defaults, 'labels': labels, 'publish_image': publish_image, **settings,
            'source': 'real', 'time_code': False, 'use_sim_time': False}
    cameras = streamed(args)
    if not cameras:
        raise RuntimeError(f'{machine}.yaml: camera_stream streams no cameras')
    args['cameras'] = ','.join(cameras)
    decoder = str(args['decoder'])
    if decoder != 'opencv':
        from aav_camera_stream import decoders   # only machines with the streams need it
        try:
            decoders.check(decoder, str(args['transport']))
            # auto always finds one (at worst Pixi's ffmpeg); the node picks it.
            program = '' if decoder == 'auto' else decoders.command(
                decoder, 'rtsp://pi/camera', 2, 2, str(args['transport']),
                decoders.choose(decoder, str(args['program']), workspace()).program,
                str(args['pipeline']), str(args['hwaccel']))[0]
        except ValueError as e:
            raise RuntimeError(f'{machine}.yaml: camera_stream: {e}')
        if program and not found(program):
            return skipped('camera_stream', f'decoder:={decoder} runs {program}, which this '
                                            "computer lacks (gst-nvidia: the Orin's JetPack "
                                            'GStreamer)'), []
    kind = str(args['detector']).strip().lower()
    if kind != 'none':
        model, why = check_model(machine, kind, str(args['model']))
        if why:
            return skipped('camera_stream', why + '. Or detector: none in camera_stream: '
                                                  'the streams and previews only, no stop '
                                                  'signs'), []
        args['model'] = model
    return [include('aav_camera_stream', 'camera_stream.launch.py', **args)], cameras


def uncalibrated(cameras):
    """Return the streamed cameras with no lens file (aav_camera_stream config/cameras.yaml)."""
    path = os.path.join(get_package_share_directory('aav_camera_stream'), 'config',
                        'cameras.yaml')
    with open(path) as f:
        known = yaml.safe_load(f)['cameras']
    return [c for c in cameras if not (known.get(c) or {}).get('calibration')]


REAR_IMAGE = '/sensors/camera/rear/image_raw'


def watches_behind(settings):
    """Return True if this detector reads the rear camera and keeps people (it sees behind)."""
    if not isinstance(settings, dict):
        return False
    extra = [t.strip() for t in str(settings.get('extra_images', '')).split(',')]
    labels = [t.strip().lower() for t in str(settings.get('labels', 'stop sign')).split(',')
              if t.strip()]
    return REAR_IMAGE in extra and (not labels or 'person' in labels)


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        # As IfCondition reads it: true or 1, false or 0; anything else is an error.
        return evaluate_condition_expression(context, [LaunchConfiguration(name)])

    machine = arg('machine')
    config = load_machine(machine)
    parts = config['parts']
    # Who drives the car (docs/can-plan.md): uart, today's wiring, leaves the CAN bus to
    # the sensors (spine_bridge role sensors); can, the KEFC spine (role spine).
    role = bridges.role_for(arg('actuators'))

    def on(name):
        # A launch argument can switch a part off, wherever the machine file has it.
        return (bool(parts.get(name, False)) and (name != 'gps' or flag('gps'))
                and (name != 'lidar' or flag('lidar')))

    # The camera streams first: what they stream decides the detector part and what the
    # sensor watchdog watches. The detector part reads image_raw, which the streams keep
    # inside: so for a streamed camera the stream's own detector replaces it.
    streams, streaming = [], []
    intended = streamed(config.get('camera_stream')) if on('camera_stream') else []
    if on('camera_stream'):
        if on('camera') and 'front_left' in intended:
            raise RuntimeError(f'{machine}.yaml: the camera part (usb_cam) and camera_stream '
                               'both give front_left: pick one')
        streams, streaming = camera_streams(
            context, machine, config.get('camera_stream'),
            stream_labels(on('camera_obstacles')),
            # The lane finder reads the front left picture, here: no network in between.
            publish_image=on('lane') and 'front_left' in intended)
    run_detector = on('detector')
    if run_detector and intended:
        images = detector_images(config.get('detector'))
        taken = [i for i in images if i in {image_topic(c) for c in intended}]
        if taken == images:
            skipped('detector', 'the camera_stream part runs the detector for '
                                f'{", ".join(intended)}, in its own nodes')
            run_detector = False
        elif taken:
            raise RuntimeError(f'{machine}.yaml: the detector reads {", ".join(taken)}, '
                               'which camera_stream keeps inside: set its image and '
                               'extra_images to the other cameras only')

    actions = []
    if on('description'):
        actions.append(include('aav_description', 'description.launch.py'))
    if on('mux'):
        actions.append(include('aav_command_mux', 'mux.launch.py', geofence_lock='true'))
        # The sensor watchdog, next to the mux: twist_mux's sensors lock stops Nav2 and the
        # AI slot when the wheels, the IMU, the GPS or an EKF fail (aav_safety), and when
        # the watchdog itself goes quiet (the lock's 1 s timeout). Nav2 and the sensors may
        # run on other machines: it watches their topics over the network. The HUD starts
        # on its own (aav-hud.service), wherever the screen is: its heartbeat says it lives.
        # And each camera stream this machine starts, through its detections.
        actions.append(include('aav_safety', 'watchdog.launch.py',
                               inputs='odom,imu,local,global' + (',gps' if flag('gps') else '')
                               + (',hud' if flag('watch_hud') else '')
                               + ''.join(f',{c}_stream' for c in streaming)))
        # The geofence, next to the mux, always (F6, docs/geofence.md): twist_mux's geofence
        # lock holds every program near the fence's edge, while the global pose can't be
        # trusted, and when the geofence itself goes quiet (the lock's 1 s timeout).
        # The fence is fixed (the start spot, or a site), so a restart finds the car
        # where it is. A site: the argument, else the machine file's.
        site = arg('geofence_site') or (config.get('geofence') or {}).get('site') or ''
        actions.append(include('aav_safety', 'geofence.launch.py',
                               backend='uart' if arg('actuators') == 'uart' else 'spine',
                               radius=arg('geofence_radius'), centre_x=arg('start_x'),
                               centre_y=arg('start_y'), site=site,
                               max_speed=arg('max_speed'), gps=str(flag('gps')).lower(),
                               stands=str(flag('stands')).lower()))
    if on('ego'):
        actions.append(include('aav_ego', 'ego.launch.py'))
        # Next to ego: the sensor mux and the output mux. The real backends (spine_bridge,
        # legacy_uart_bridge, on any machine) read only /control/car/actuator_cmd, which
        # carries ego's commands only while every source on the network is REAL.
        actions.append(include('aav_io_mux', 'io_mux.launch.py', target='car', source='real'))
    if on('teleop'):
        actions.append(include('aav_teleop', 'teleop.launch.py'))
    if on('scripted'):
        actions.append(include('aav_scripted', 'scripted.launch.py'))
    if on('drive_to_point'):
        actions.append(include('aav_drive_to_point', 'drive_to_point.launch.py'))
    if any(on(s) for s in ('imu', 'gps', 'radar', 'lidar', 'battery')):
        actions.append(include('aav_sensors', 'sensors.launch.py',
                               imu=on('imu'), gps=on('gps'), radar=on('radar'), lidar=on('lidar'),
                               battery=on('battery'), gps_driver=arg('gps_driver')))
    if on('camera'):
        if installed('usb_cam'):
            actions.append(include('aav_sensors', 'cameras.launch.py',
                                   left='/dev/aav/camera_left', right=''))
        else:
            actions += skipped('camera', 'usb_cam is not installed here (it is Linux only)')
    actions += streams
    if run_detector:
        actions += detector(context, machine, config.get('detector') or {})
    if on('stop_signs'):
        actions.append(Node(package='aav_detector', executable='stop_sign_watcher',
                            name='stop_sign_watcher'))
        # Its distance needs each camera's focal length, from the stream's CameraInfo.
        blind = uncalibrated(streaming) if streaming else []
        if blind:
            get_logger('car.launch.py').warning(
                f'stop_sign_watcher: {", ".join(blind)}: no calibration, so no CameraInfo: '
                "their stop signs are judged with Gazebo's lens (focal_px 343, 640x360), "
                'wrong at 1280x720. Calibrate them: pixi run calibrate-camera')
    if on('radar_clusters'):
        actions.append(include('aav_radar_clusters', 'radar_clusters.launch.py',
                               mock=False, eps=0.8, minimum_points=15, radar_rviz=False))
    if on('camera_obstacles'):
        config_file = os.path.join(get_package_share_directory('aav_detector'), 'config',
                                   'camera_obstacles.yaml')
        parameters = [config_file]
        if streaming:
            # Each stream's lens too: camera_stream publishes it with its detections, once
            # the camera has a calibration. Without one, its boxes can't reach the road.
            with open(config_file) as f:
                topics = yaml.safe_load(f)['camera_obstacles']['ros__parameters'][
                    'camera_info_topics']
            topics += [f'/sensors/camera/{c}/camera_info' for c in streaming
                       if f'/sensors/camera/{c}/camera_info' not in topics]
            parameters.append({'camera_info_topics': topics})
            blind = uncalibrated(streaming)
            if blind:
                get_logger('car.launch.py').warning(
                    f'camera_obstacles: {", ".join(blind)}: no calibration '
                    '(aav_camera_stream config/cameras.yaml), so no lens: those boxes '
                    "won't reach Nav2's costmap")
        actions.append(Node(package='aav_detector', executable='camera_obstacles',
                            name='camera_obstacles', parameters=parameters))
    if on('lane'):
        actions += [include('aav_lane_finder', 'lane_finder.launch.py', debug_image=False),
                    include('aav_lane_geometry', 'lane_follower.launch.py', fake=False)]
    if on('localization'):
        actions.append(include('aav_localization', 'localization.launch.py',
                               gps=flag('gps'), start_x=arg('start_x'),
                               start_y=arg('start_y'), start_yaw=arg('start_yaw')))
    if on('navigation'):
        # Always pass a map: an empty `map` here would override Nav2's own default,
        # because included launch files share launch arguments.
        empty_map = os.path.join(get_package_share_directory('aav_navigation'), 'maps',
                                 'empty.yaml')
        # Nav2 backs up only if something here sees behind the car (Ben, 22 Oct: "Yes the
        # car can reverse"); collision_monitor then watches a box behind the rear bumper.
        reversing = flag('reversing')
        # The rear camera counts once its detector reads it and keeps people (machine file:
        # detector: {extra_images: /sensors/camera/rear/image_raw, labels: ""}).
        rear_camera = (on('camera_obstacles') and run_detector
                       and watches_behind(config.get('detector')))
        blind = vehicle.blind_behind(reversing, radar=on('radar'), lidar=on('lidar'),
                                     rear_camera=rear_camera)
        if blind:
            get_logger('car.launch.py').warning(f'reversing: {blind}')
            reversing = False
        # The local costmap reads the sensors this machine runs. Each is passed, on or
        # off, so none can leak in from another file.
        # Only planners whose paths keep to the turning radius, on real data.
        planner = vehicle.check_planner(arg('planner'), real=True)
        actions.append(include('aav_navigation', 'navigation.launch.py',
                               map=arg('map') or empty_map, radar=on('radar'), planner=planner,
                               lidar=on('lidar'), ghost=flag('ghost'),
                               collision_monitor=(on('radar') or on('lidar') or flag('ghost')
                                                  or rear_camera),
                               camera_obstacles=on('camera_obstacles'),
                               camera_guard=rear_camera,
                               reversing=reversing))
    # Exactly one backend: legacy_uart_bridge with actuators:=uart, the CAN spine with can.
    uart, encoder = on('legacy_uart'), on('legacy_encoder')
    if on('spine'):
        bridges.package(arg('bridge'))   # rust or python: refuse anything else now
        if installed('ros2_socketcan'):
            actions.append(include('aav_spine', 'spine.launch.py', bridge=arg('bridge'),
                                   role=role))
        else:
            actions += skipped('spine', 'ros2_socketcan is not installed here (SocketCAN is '
                                        'Linux only)')
        if encoder:
            skipped('legacy_encoder', 'the CAN wheel nodes give /vehicle/odom here; two '
                                      'publishers would stop the output mux')
            encoder = False
    if uart and role == 'spine':
        skipped('legacy_uart', 'actuators:=can: the CAN spine drives the car, and only one '
                               'backend may')
        uart = False
    if uart or encoder:
        actions.append(include('aav_legacy', 'legacy.launch.py', uart=uart, encoder=encoder))
    if on('foxglove'):
        # On the car's network (every interface: its addresses aren't fixed yet), for
        # watching only: anyone on its WiFi reaches the bridge, so no client may publish,
        # set parameters or call services. The WiFi topic list; camera_stream makes the
        # pictures' previews itself.
        actions.append(include('aav_bringup', 'foxglove.launch.py',
                               foxglove_address='0.0.0.0', foxglove_port=8765,
                               use_sim_time=False, foxglove_control=False,
                               foxglove_topics='wifi', foxglove_preview=''))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('machine', default_value='all',
                              description='config/machines/<machine>.yaml'),
        DeclareLaunchArgument('max_speed', default_value='',
                              description='m/s: cap ego below config/ego.yaml (a first drive on '
                                          'stands: max_speed:=0.3); empty keeps ego.yaml'),
        DeclareLaunchArgument('gps', default_value='true',
                              description='false: no GPS anywhere (odometry only), whatever '
                                          'the machine file says'),
        DeclareLaunchArgument('gps_driver', default_value='ublox',
                              description='ublox (u-blox, Pixi too), nmea (any NMEA receiver) '
                                          'or ublox_dgnss (u-blox F9P, apt only). Tentative '
                                          'until the receiver is known: Q18'),
        DeclareLaunchArgument('map', default_value='',
                              description='Map YAML for Nav2 (default: the empty map)'),
        # The roof's Velodyne VLP-16 (the 16 Oct audit), where the machine file has the
        # lidar part: its driver, and Nav2 reading it. false switches it off everywhere.
        DeclareLaunchArgument('lidar', default_value='true',
                              description='false: no lidar driver, and Nav2 without the '
                                          'lidar, whatever the machine file says'),
        # Off unless asked for: a pretend person, for stopping tests on the real car with
        # nothing real to hit (aav_ghost, docs/ghost-obstacles.md). Where Nav2 runs.
        DeclareLaunchArgument('ghost', default_value='false',
                              description="true: a pretend person in Nav2's view, placed "
                                          'with pixi run ghost ahead 10 (it only adds)'),
        # The car's fence until the test lot's polygon is surveyed (a TODO for the team):
        # 30 m round where it first stood still. Change it per test card, not in passing.
        DeclareLaunchArgument('geofence_radius', default_value='30.0',
                              description='m: the geofence, a circle round where the car '
                                          'first stands still with a good global pose'),
        DeclareLaunchArgument('geofence_site', default_value='',
                              description="a polygon in aav_safety's config/geofence.yaml "
                                          '(map frame) instead of the circle; empty: the '
                                          "machine file's geofence: site:, else the circle"),
        DeclareLaunchArgument('stands', default_value='false',
                              description='true: the wheels are off the ground. The '
                                          'geofence opens its lock (no fence), and says so '
                                          'loudly. Needed with gps:=false'),
        # The safety drivers rely on the HUD (PLAN T9.8): no heartbeat, no Nav2.
        DeclareLaunchArgument('watch_hud', default_value='true',
                              description='true: the sensor watchdog stops Nav2 and the AI '
                                          "slot while the HUD's /hud/heartbeat is missing; "
                                          'false: a session with no HUD screen'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='Heading at start, rad counter-clockwise from east'),
        # Off on the car since 26 Oct (design review F2): the wheel sensors can't tell
        # direction yet (H5), so a reversing car's speed reads forward. On again once H5
        # is answered. true (Ben, 22 Oct: "Yes the car can reverse") works where something
        # sees behind (vehicle.blind_behind). People can always reverse (joystick,
        # keyboard, RC); this is Nav2 only. sim, bench and carla keep their own true.
        DeclareLaunchArgument('reversing', default_value='false',
                              description='false: Nav2 forward only (the default until the '
                                          'wheels tell direction, H5); true: Nav2 may back '
                                          'up, with the lidar watching behind '
                                          '(docs/learn-nav2.md)'),
        DeclareLaunchArgument('planner', default_value=vehicle.DEFAULT_PLANNER,
                              description=', '.join(vehicle.CAR_PLANNERS) + ": Nav2's "
                                          'planner on the car; the grid planners (sim and '
                                          'bench trials) are refused here'),
        DeclareLaunchArgument('bridge', default_value=bridges.DEFAULT,
                              description='spine_bridge in rust (aav_spine_bridge) or python '
                                          '(aav_spine), where the machine runs the spine'),
        # The car's wiring, as Ben described it on 23 Oct (the hardware lead confirms):
        # the ESP32 through the Motor Pi's UART, the Orin's CAN bus for sensors.
        DeclareLaunchArgument('actuators', default_value=bridges.DEFAULT_ACTUATORS,
                              description='uart: legacy_uart_bridge drives (the Motor Pi), '
                                          'the CAN bus is read for sensors only; can: the '
                                          'CAN spine drives. The same on every machine'),
        OpaqueFunction(function=launch_setup),
    ])
