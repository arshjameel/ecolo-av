"""
car.launch.py: the machine files, and the launch arguments that win over them.

Each test runs a launch file's own setup, car.launch.py's or sensors.launch.py's, with
some arguments, and looks at what it would start. Nothing starts: no ROS, no devices.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.conditions import InvalidConditionExpressionError
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


def load(package, name):
    path = os.path.join(get_package_share_directory(package), 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CAR = load('aav_bringup', 'car.launch.py')
SENSORS = load('aav_sensors', 'sensors.launch.py')
MACHINES = sorted(f[:-len('.yaml')] for f in os.listdir(
    os.path.join(get_package_share_directory('aav_bringup'), 'config', 'machines')))


def context_for(launch_file, **args):
    """Return a launch context with the file's defaults, and these arguments on top."""
    context = LaunchContext()
    for declared in launch_file.generate_launch_description().get_launch_arguments():
        context.launch_configurations[declared.name] = perform_substitutions(
            context, declared.default_value)
    context.launch_configurations.update(args)
    return context


def started(launch_file, **args):
    return launch_file.launch_setup(context_for(launch_file, **args))


def included(actions):
    """Map each launch file included to the arguments it is given."""
    found = {}
    for a in actions:
        if isinstance(a, IncludeLaunchDescription):
            # Loading the file expands its path; it also checks that the file loads.
            a.launch_description_source.get_launch_description(LaunchContext())
            name = os.path.basename(a.launch_description_source.location)
            found[name] = dict(a.launch_arguments)
    return found


def nodes(actions):
    return {(a.node_package, a.node_executable) for a in actions if isinstance(a, Node)}


@pytest.mark.parametrize('machine', MACHINES)
def test_every_machine_file_loads_and_starts(machine):
    assert CAR.load_machine(machine)['parts']
    started(CAR, machine=machine)


@pytest.mark.parametrize('machine', ['orin', 'laptop', 'all'])
def test_every_machine_runs_rfdetr(machine):
    # RF-DETR is the car's detector since 20 Oct; YOLO stays one line away in each file.
    # The Orin runs it inside its camera streams, the others in the detector part.
    section = 'camera_stream' if machine == 'orin' else 'detector'
    assert CAR.load_machine(machine)[section]['detector'] == 'rfdetr'
    # The Orin runs it in TensorRT (Ben, 23 Oct); the others its .pth, through PyTorch.
    model = CAR.load_machine(machine)[section].get('model')
    assert model == ('models/rf-detr-nano.engine' if machine == 'orin' else None)


def test_the_detector_needs_its_own_library(monkeypatch, tmp_path):
    asked = []
    monkeypatch.setattr(CAR, 'lacks', lambda modules: (asked.append(modules), ('python3', []))[1])
    context = context_for(CAR, machine='orin')
    rfdetr, yolo = tmp_path / 'rf-detr-nano.pth', tmp_path / 'yolo11n.pt'
    rfdetr.write_text('')
    yolo.write_text('')
    sent = included(CAR.detector(context, 'orin', {'detector': 'rfdetr', 'model': str(rfdetr)}))
    assert sent['detector.launch.py']['detector'] == 'rfdetr'
    sent = included(CAR.detector(context, 'orin', {'detector': 'yolo', 'model': str(yolo)}))
    assert sent['detector.launch.py']['detector'] == 'yolo'
    assert asked == [['torch', 'rfdetr'], ['torch', 'ultralytics']]
    # RF-DETR's TensorRT engine needs TensorRT too.
    engine = tmp_path / 'rf-detr-nano.engine'
    engine.write_text('')
    sent = included(CAR.detector(context, 'orin', {'detector': 'rfdetr', 'model': str(engine)}))
    assert sent['detector.launch.py']['model'] == str(engine)
    assert asked[-1] == ['torch', 'rfdetr', 'tensorrt']


def test_a_model_the_detector_cant_load_is_an_error():
    context = context_for(CAR, machine='orin')
    with pytest.raises(RuntimeError, match='use detector:=yolo'):
        CAR.detector(context, 'orin', {'detector': 'rfdetr', 'model': 'models/yolo11n.engine'})
    with pytest.raises(RuntimeError, match='use rfdetr or yolo'):
        CAR.detector(context, 'orin', {'detector': 'detr'})


def test_gps_false_turns_the_gps_off_where_the_machine_file_has_it():
    assert CAR.load_machine('all')['parts']['gps']
    on = included(started(CAR, machine='all'))
    assert on['sensors.launch.py']['gps'] == 'true'
    assert on['localization.launch.py']['gps'] == 'true'
    off = included(started(CAR, machine='all', gps='false'))
    assert off['sensors.launch.py']['gps'] == 'false'
    assert off['localization.launch.py']['gps'] == 'false'
    assert started(SENSORS, imu='false', gps='false') == []


def test_gps_true_starts_no_gps_where_none_is_plugged_in():
    for machine in ('laptop', 'motor_pi'):
        assert 'sensors.launch.py' not in included(started(CAR, machine=machine, gps='true'))


@pytest.mark.parametrize('value', ['False', '0'])
def test_false_is_read_as_ros_reads_it(value):
    assert included(started(CAR, machine='all', gps=value))['sensors.launch.py']['gps'] == \
        'false'


def test_the_gps_driver_argument_picks_the_driver():
    sent = included(started(CAR, machine='all', gps_driver='nmea'))
    assert sent['sensors.launch.py']['gps_driver'] == 'nmea'
    ublox = started(SENSORS, imu='false', gps_driver='ublox')
    assert nodes(ublox) == {('ublox_gps', 'ublox_gps_node')}
    nmea = included(started(SENSORS, imu='false', gps_driver='nmea'))
    assert nmea['gps_nmea.launch.py']['port'] == '/dev/aav/gps'


def test_ublox_gps_is_the_default():
    assert ('ublox_gps', 'ublox_gps_node') in nodes(started(SENSORS))


def test_the_imu_driver_is_the_rust_one():
    # The C++ driver (yahboom_imu_cpp) is gone, and with it the imu_driver argument.
    assert ('aav_imu_yahboom', 'imu_node') in nodes(started(SENSORS))
    assert 'imu_driver' not in included(started(CAR, machine='all'))['sensors.launch.py']


def test_the_radar_clusters_are_the_rust_node(monkeypatch):
    # The C++ node (aav_object_detection's dbscan) is gone: aav_radar_clusters replaced it.
    monkeypatch.setattr(CAR, 'load_machine', lambda machine: {'parts': {'radar_clusters': True}})
    sent = included(started(CAR, machine='orin'))
    assert sent['radar_clusters.launch.py'] == {
        'mock': 'false', 'eps': '0.8', 'minimum_points': '15', 'radar_rviz': 'false'}
    clusters = load('aav_radar_clusters', 'radar_clusters.launch.py')
    started_nodes = {(a.node_package, a.node_executable)
                     for a in clusters.generate_launch_description().entities
                     if isinstance(a, Node)}
    assert ('aav_radar_clusters', 'radar_clusters') in started_nodes


def test_the_map_and_the_start_pose_reach_their_launch_files():
    sent = included(started(CAR, machine='all', map='/maps/site.yaml', start_x='1.5',
                            start_y='-2.0', start_yaw='0.3'))
    assert sent['navigation.launch.py']['map'] == '/maps/site.yaml'
    assert (sent['localization.launch.py']['start_x'], sent['localization.launch.py']['start_y'],
            sent['localization.launch.py']['start_yaw']) == ('1.5', '-2.0', '0.3')
    assert sent['localization.launch.py']['gps'] == 'true'
    default = included(started(CAR, machine='all'))['navigation.launch.py']['map']
    assert default.endswith(os.path.join('maps', 'empty.yaml'))


def test_the_lidar_runs_where_its_packets_go_and_nav2_reads_it():
    # The Jetson (orin) gets the VLP-16's packets: its driver runs there, and Nav2 there
    # reads it, with collision_monitor watching it.
    sent = included(started(CAR, machine='orin'))
    assert sent['sensors.launch.py']['lidar'] == 'true'
    nav = sent['navigation.launch.py']
    assert (nav['lidar'], nav['collision_monitor']) == ('true', 'true')
    # lidar:=false switches it off everywhere, whatever the machine file says.
    off = included(started(CAR, machine='orin', lidar='false'))
    assert off['sensors.launch.py']['lidar'] == 'false'
    assert (off['navigation.launch.py']['lidar'],
            off['navigation.launch.py']['collision_monitor']) == ('false', 'false')
    # A machine without the part starts no driver.
    assert 'sensors.launch.py' not in included(started(CAR, machine='laptop'))


def test_nav2_reverses_only_when_asked_and_where_the_lidar_sees_behind():
    # Off by default on the car (26 Oct, F2): the wheels can't tell direction yet (H5).
    default = included(started(CAR, machine='orin'))['navigation.launch.py']
    assert default['reversing'] == 'false'
    # reversing:=true (Ben, 22 Oct: "Yes the car can reverse"): the roof lidar watches behind.
    on = included(started(CAR, machine='orin', reversing='true'))['navigation.launch.py']
    assert on['reversing'] == 'true'
    # Without it nothing sees behind (the radar faces forward): forward only.
    blind = included(started(CAR, machine='orin', reversing='true',
                             lidar='false'))['navigation.launch.py']
    assert blind['reversing'] == 'false'


def test_the_rear_camera_sees_behind_only_when_its_detector_reads_it_for_people():
    rear = '/sensors/camera/rear/image_raw'
    assert CAR.watches_behind({'extra_images': rear, 'labels': ''})
    assert CAR.watches_behind({'extra_images': f'/a, {rear}', 'labels': 'person, car'})
    assert not CAR.watches_behind({'extra_images': rear})            # stop signs only
    assert not CAR.watches_behind({'labels': ''})                    # the front camera only
    assert not CAR.watches_behind(None)


def test_the_watchdog_guards_nav2():
    sent = included(started(CAR, machine='all'))
    assert sent['watchdog.launch.py']['inputs'] == 'odom,imu,local,global,gps,hud'
    no_gps = included(started(CAR, machine='all', gps='false'))
    assert 'gps' not in no_gps['watchdog.launch.py']['inputs']
    # The safety drivers rely on the HUD: watched unless the session has no screen.
    no_hud = included(started(CAR, machine='all', watch_hud='false'))
    assert 'hud' not in no_hud['watchdog.launch.py']['inputs'].split(',')


def test_bad_values_stop_the_launch():
    with pytest.raises(RuntimeError, match='no such machine file'):
        started(CAR, machine='nowhere')
    with pytest.raises(InvalidConditionExpressionError):
        started(CAR, machine='all', gps='maybe')
    with pytest.raises(RuntimeError, match='pick one of'):
        started(SENSORS, gps_driver='garmin')


def test_one_stop_sign_watcher_next_to_the_mux():
    for machine in MACHINES:
        parts = CAR.load_machine(machine)['parts']
        watcher = ('aav_detector', 'stop_sign_watcher') in nodes(started(CAR, machine=machine))
        assert watcher == bool(parts.get('mux')), machine


def test_the_geofence_runs_wherever_the_mux_does():
    # The mux's geofence lock is fail-safe too: silence holds every program.
    for machine in MACHINES:
        parts = CAR.load_machine(machine)['parts']
        sent = included(started(CAR, machine=machine))
        assert ('geofence.launch.py' in sent) == bool(parts.get('mux')), machine
        if parts.get('mux'):
            assert sent['mux.launch.py'].get('geofence_lock', 'true') == 'true', machine
            # The car's default fence: a 30 m circle round the marked start spot, and
            # today's backend (the 2025-26 ESP32, which coasts: geofence.py coasts).
            assert sent['geofence.launch.py'] == {
                'backend': 'uart', 'radius': '30.0', 'centre_x': '0.0', 'centre_y': '0.0',
                'site': '', 'max_speed': '', 'gps': 'true', 'stands': 'false'}, machine
    lot = included(started(CAR, machine='all', geofence_site='bench_road', start_x='5',
                           actuators='can', stands='true'))['geofence.launch.py']
    assert (lot['site'], lot['centre_x'], lot['backend'], lot['stands']) == (
        'bench_road', '5', 'spine', 'true')


def test_the_sensor_watchdog_runs_wherever_the_mux_does():
    # The mux's sensors lock needs the watchdog's heartbeat (fail-safe: silence holds Nav2),
    # so the two always start together, whichever machine Nav2 is on.
    for machine in MACHINES:
        parts = CAR.load_machine(machine)['parts']
        sent = included(started(CAR, machine=machine))
        assert ('watchdog.launch.py' in sent) == bool(parts.get('mux')), machine
        if parts.get('mux'):
            assert sent['mux.launch.py'].get('sensor_lock', 'true') == 'true', machine


def test_the_output_mux_runs_next_to_ego_for_the_real_car():
    # The real backends read only /control/car/actuator_cmd: the output mux, target car,
    # must run wherever ego does, and nowhere else (two would both judge the same ego).
    for machine in MACHINES:
        parts = CAR.load_machine(machine)['parts']
        sent = included(started(CAR, machine=machine))
        assert ('io_mux.launch.py' in sent) == bool(parts.get('ego')), machine
        if parts.get('ego'):
            assert sent['io_mux.launch.py'] == {'target': 'car', 'source': 'real'}, machine


def test_lane_keeping_is_off_until_a_machine_file_turns_it_on(monkeypatch):
    # control_interface drives whenever it sees a lane, so no machine file has it on yet.
    for machine in MACHINES:
        assert not CAR.load_machine(machine)['parts'].get('lane'), machine
        assert 'lane_follower.launch.py' not in included(started(CAR, machine=machine))
    monkeypatch.setattr(CAR, 'load_machine', lambda machine: {'parts': {'lane': True}})
    sent = included(started(CAR, machine='orin'))
    assert sent['lane_finder.launch.py'] == {'debug_image': 'false'}
    assert sent['lane_follower.launch.py'] == {'fake': 'false'}


def test_the_detector_is_named_after_its_machine(monkeypatch, tmp_path):
    model = tmp_path / 'yolo11n.engine'
    model.write_text('')
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', []))
    context = context_for(CAR, machine='orin')
    sent = included(CAR.detector(context, 'orin', {'detector': 'yolo', 'model': str(model),
                                                   'confidence': 0.6}))
    assert sent['detector.launch.py']['node_name'] == 'object_detector_orin'
    assert sent['detector.launch.py']['model'] == str(model)
    assert sent['detector.launch.py']['labels'] == 'stop sign'
    assert sent['detector.launch.py']['picture'] == ''
    assert sent['detector.launch.py']['confidence'] == '0.6'
    # The rest are passed too, at the detector's own defaults: none can leak in.
    assert sent['detector.launch.py']['image'] == '/sensors/camera/front_left/image_raw'
    assert sent['detector.launch.py']['device'] == 'auto'


def test_the_detector_is_skipped_without_pytorch_or_its_model(monkeypatch, tmp_path):
    context = context_for(CAR, machine='orin')
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', ['torch']))
    assert CAR.detector(context, 'orin', {}) == []
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', []))
    assert CAR.detector(context, 'orin', {'model': str(tmp_path / 'rf-detr-none.pth')}) == []
    assert CAR.detector(context, 'orin', {'detector': 'yolo',
                                          'model': str(tmp_path / 'none.engine')}) == []


@pytest.mark.parametrize('machine', MACHINES)
def test_each_machine_files_detector_settings_are_known(machine, monkeypatch):
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', ['torch']))
    settings = CAR.load_machine(machine).get('detector') or {}
    CAR.detector(context_for(CAR, machine=machine), machine, settings)
    with pytest.raises(RuntimeError, match='takes no'):
        CAR.detector(context_for(CAR, machine=machine), machine, {'modle': 'x.pt'})


def test_the_spine_bridge_is_the_rust_one_unless_asked():
    # aav_spine_bridge (Rust) took over from aav_spine's Python node, which stays for now.
    spine = load('aav_spine', 'spine.launch.py')
    assert nodes(spine.bridge(context_for(spine))) == {('aav_spine_bridge', 'spine_bridge')}
    assert nodes(spine.bridge(context_for(spine, bridge='python'))) == {
        ('aav_spine', 'spine_bridge')}
    bench = load('aav_bringup', 'bench.launch.py')
    assert ('aav_spine_bridge', 'spine_bridge') in nodes(started(bench))
    assert ('aav_spine', 'spine_bridge') in nodes(started(bench, bridge='python'))


@pytest.fixture
def socketcan(monkeypatch):
    """ros2_socketcan as if installed (it's Linux only), so the spine part starts."""
    real = CAR.installed
    monkeypatch.setattr(CAR, 'installed', lambda p: p == 'ros2_socketcan' or real(p))


def test_the_car_passes_its_bridge_choice_to_the_spine(monkeypatch, socketcan):
    monkeypatch.setattr(CAR, 'load_machine', lambda machine: {'parts': {'spine': True}})
    assert included(started(CAR, machine='orin'))['spine.launch.py']['bridge'] == 'rust'
    sent = included(started(CAR, machine='orin', bridge='python'))['spine.launch.py']
    assert sent['bridge'] == 'python'
    with pytest.raises(ValueError, match='bridge:=cpp'):
        started(CAR, machine='orin', bridge='cpp')


# The car's wiring, as Ben described it on 23 Oct (the hardware lead confirms): the
# actuators on the ESP32, driven through the Motor Pi's UART; the CAN bus straight to the
# Orin, with the wheel sensors on it (docs/can-plan.md).

def test_the_car_as_wired_today(socketcan):
    # The Orin reads its CAN bus for the wheels only...
    orin = included(started(CAR, machine='orin'))
    assert orin['spine.launch.py']['role'] == 'sensors'
    assert 'legacy.launch.py' not in orin
    # ...and the Motor Pi drives the ESP32, with no UDP encoder beside the CAN wheels.
    motor_pi = included(started(CAR, machine='motor_pi'))
    assert motor_pi['legacy.launch.py'] == {'uart': 'true', 'encoder': 'false'}
    assert 'spine.launch.py' not in motor_pi
    # One computer with everything: the same two.
    everything = included(started(CAR, machine='all'))
    assert everything['spine.launch.py']['role'] == 'sensors'
    assert everything['legacy.launch.py'] == {'uart': 'true', 'encoder': 'false'}


def test_actuators_can_makes_the_can_spine_the_one_backend(socketcan):
    assert included(started(CAR, machine='orin', actuators='can'))['spine.launch.py'][
        'role'] == 'spine'
    assert 'legacy.launch.py' not in included(started(CAR, machine='motor_pi',
                                                      actuators='can'))
    everything = included(started(CAR, machine='all', actuators='can'))
    assert everything['spine.launch.py']['role'] == 'spine'
    assert 'legacy.launch.py' not in everything
    with pytest.raises(ValueError, match='actuators:=pwm'):
        started(CAR, machine='orin', actuators='pwm')


def test_the_udp_encoder_never_runs_beside_the_can_wheels(monkeypatch, socketcan):
    # Both would publish /vehicle/odom: the sensor mux would doubt, and the output mux stop.
    monkeypatch.setattr(CAR, 'load_machine', lambda machine: {'parts': {
        'spine': True, 'legacy_uart': True, 'legacy_encoder': True}})
    assert included(started(CAR, machine='orin'))['legacy.launch.py'] == {
        'uart': 'true', 'encoder': 'false'}
    monkeypatch.setattr(CAR, 'load_machine', lambda machine: {'parts': {
        'legacy_uart': True, 'legacy_encoder': True}})
    assert included(started(CAR, machine='orin'))['legacy.launch.py']['encoder'] == 'true'


def test_no_socketcan_skips_the_can_bus(monkeypatch):
    monkeypatch.setattr(CAR, 'installed', lambda p: p != 'ros2_socketcan')
    sent = included(started(CAR, machine='orin'))
    assert 'spine.launch.py' not in sent


def test_the_sensors_role_starts_no_can_sender():
    spine = load('aav_spine', 'spine.launch.py')

    def socketcan_parts(**args):
        # The path as written: ros2_socketcan is Linux only, so it may not resolve here.
        return {part for a in spine.bus(context_for(spine, **args))
                for part in ('receiver', 'sender')
                if f'socket_can_{part}.launch.py' in str(a.launch_description_source.location)}

    assert socketcan_parts(role='sensors') == {'receiver'}
    assert socketcan_parts() == {'receiver', 'sender'}
    for role in ('sensors', 'spine'):
        context = context_for(spine, role=role)
        [node] = spine.bridge(context)
        given = [p for p in evaluate_parameters(context, node._Node__parameters)
                 if isinstance(p, dict)]
        assert any(p.get('role') == role for p in given), role
    with pytest.raises(ValueError, match='role:=drive'):
        spine.bus(context_for(spine, role='drive'))


# The camera streams (aav_camera_stream, docs/camera-pipeline.md): the camera Pi's H.264
# streams to the Orin, one node per front camera, each with its own detector.

@pytest.fixture
def orin_kit(monkeypatch, tmp_path):
    """Make this computer the Orin to be: JetPack's GStreamer and TensorRT, the engine built."""
    (tmp_path / 'models').mkdir()
    (tmp_path / 'models' / 'rf-detr-nano.engine').write_text('')
    monkeypatch.setattr(CAR, 'workspace', lambda: str(tmp_path))
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', []))
    monkeypatch.setattr(CAR, 'found', lambda program: program)
    return tmp_path


def with_parts(monkeypatch, machine='orin', **parts):
    """Return the machine file, with some parts switched on or off."""
    config = CAR.load_machine(machine)
    config = {**config, 'parts': {**config['parts'], **parts}}
    monkeypatch.setattr(CAR, 'load_machine', lambda name: config)
    return config


def test_the_orin_streams_both_front_cameras_with_rfdetr_inside(orin_kit):
    sent = included(started(CAR, machine='orin'))
    stream = sent['camera_stream.launch.py']
    assert stream['host'] == '192.168.1.108'                     # CAM1-PI
    assert stream['cameras'] == 'front_left,front_right'
    assert stream['decoder'] == 'gst-nvidia'
    assert (stream['detector'], stream['model']) == (
        'rfdetr', str(orin_kit / 'models' / 'rf-detr-nano.engine'))
    assert (stream['source'], stream['time_code'], stream['use_sim_time']) == (
        'real', 'false', 'false')
    assert stream['labels'] == 'stop sign'
    assert stream['publish_image'] == 'false'                  # the pictures stay inside
    # The stream's own detector: no object_detector_orin beside it.
    assert 'detector.launch.py' not in sent
    # The watchdog, next to the mux here, watches both through their detections.
    inputs = sent['watchdog.launch.py']['inputs'].split(',')
    assert inputs[-2:] == ['front_left_stream', 'front_right_stream']
    # The stop-sign watcher reads the same /perception/detections.
    assert ('aav_detector', 'stop_sign_watcher') in nodes(started(CAR, machine='orin'))


def test_the_orin_today_skips_the_streams_and_says_why(monkeypatch, tmp_path):
    # No engine yet (it's built on the Orin itself), or no PyTorch here: no streams, so
    # the watchdog doesn't wait for them, and the old detector part doesn't start instead.
    monkeypatch.setattr(CAR, 'workspace', lambda: str(tmp_path))
    monkeypatch.setattr(CAR, 'found', lambda program: program)
    for lacking in ([], ['tensorrt']):
        monkeypatch.setattr(CAR, 'lacks', lambda modules, m=lacking: ('python3', m))
        sent = included(started(CAR, machine='orin'))
        assert 'camera_stream.launch.py' not in sent
        assert 'detector.launch.py' not in sent
        assert '_stream' not in sent['watchdog.launch.py']['inputs']


def test_no_decoder_program_skips_the_streams(orin_kit, monkeypatch):
    asked = []
    monkeypatch.setattr(CAR, 'found', lambda program: asked.append(program))
    assert 'camera_stream.launch.py' not in included(started(CAR, machine='orin'))
    assert asked == ['/usr/bin/gst-launch-1.0']                 # JetPack's own


def test_detector_none_streams_without_pytorch(monkeypatch, tmp_path):
    monkeypatch.setattr(CAR, 'found', lambda program: program)
    monkeypatch.setattr(CAR, 'lacks', lambda modules: ('python3', ['torch']))
    config = with_parts(monkeypatch)
    config['camera_stream'] = {**config['camera_stream'], 'detector': 'none', 'model': ''}
    sent = included(started(CAR, machine='orin'))
    assert sent['camera_stream.launch.py']['detector'] == 'none'
    assert 'front_left_stream' in sent['watchdog.launch.py']['inputs']


def test_the_stream_settings_are_checked(orin_kit, monkeypatch):
    config = with_parts(monkeypatch)
    streams = dict(config['camera_stream'])
    for bad, match in (({'hots': 'pi'}, 'takes no'),
                       ({'source': 'sim'}, 'takes no'),           # the car's own: real
                       ({'decoder': 'nvdec'}, 'decoder:=nvdec'),
                       ({'model': 'models/yolo11n.engine'}, 'use detector:=yolo'),
                       ({'cameras': ' , '}, 'no cameras')):
        config['camera_stream'] = {**streams, **bad}
        with pytest.raises(RuntimeError, match=match):
            started(CAR, machine='orin')
    config['camera_stream'] = {k: v for k, v in streams.items() if k != 'host'}
    with pytest.raises(RuntimeError, match="camera Pi's address"):
        started(CAR, machine='orin')


def test_the_detector_part_never_reads_a_streamed_camera(orin_kit, monkeypatch):
    config = with_parts(monkeypatch, detector=True)
    config['detector'] = {'detector': 'rfdetr'}
    orin_kit.joinpath('models', 'rf-detr-nano.pth').write_text('')
    # Its default image is the front left camera's, which the stream keeps inside.
    assert 'detector.launch.py' not in included(started(CAR, machine='orin'))
    # A camera that isn't streamed: it runs.
    config['detector'] = {'detector': 'rfdetr', 'image': '/sensors/camera/rear/image_raw'}
    assert included(started(CAR, machine='orin'))['detector.launch.py']['image'] == \
        '/sensors/camera/rear/image_raw'
    # Both kinds at once: the machine file must choose.
    config['detector'] = {'detector': 'rfdetr', 'extra_images': '/sensors/camera/rear/image_raw'}
    with pytest.raises(RuntimeError, match='keeps inside'):
        started(CAR, machine='orin')
    # Without the streams the detector part reads front_left as before.
    with_parts(monkeypatch, detector=True, camera_stream=False)['detector'] = {}
    assert 'detector.launch.py' in included(started(CAR, machine='orin'))


def test_usb_cam_and_the_stream_never_both_give_front_left(orin_kit, monkeypatch):
    with_parts(monkeypatch, camera=True)
    with pytest.raises(RuntimeError, match='pick one'):
        started(CAR, machine='orin')


def test_camera_obstacles_gets_the_streams_lenses_and_road_users(orin_kit, monkeypatch):
    with_parts(monkeypatch, camera_obstacles=True)
    actions = started(CAR, machine='orin')
    labels = included(actions)['camera_stream.launch.py']['labels'].split(',')
    assert labels[0] == 'stop sign' and 'person' in labels and 'car' in labels
    [node] = [a for a in actions
              if isinstance(a, Node) and a.node_executable == 'camera_obstacles']
    given = [p for p in evaluate_parameters(context_for(CAR), node._Node__parameters)
             if isinstance(p, dict)]
    topics = [p['camera_info_topics'] for p in given if 'camera_info_topics' in p][-1]
    assert '/sensors/camera/front_left/camera_info' in topics
    assert '/sensors/camera/front_right/camera_info' in topics
    assert '/sensors/camera/rear/camera_info' in topics            # kept from its file


def test_the_lane_finder_gets_the_front_left_picture_from_the_stream(orin_kit, monkeypatch):
    with_parts(monkeypatch, lane=True)
    assert included(started(CAR, machine='orin'))['camera_stream.launch.py'][
        'publish_image'] == 'true'


def test_the_watchdog_has_a_rule_for_every_streamed_camera():
    # car.launch.py asks for <camera>_stream: each must exist, for that camera's frame.
    share = get_package_share_directory
    with open(os.path.join(share('aav_camera_stream'), 'config', 'cameras.yaml')) as f:
        cameras = yaml.safe_load(f)['cameras']
    with open(os.path.join(share('aav_safety'), 'config', 'sensor_watchdog.yaml')) as f:
        rules = yaml.safe_load(f)['sensor_watchdog']['ros__parameters']
    for camera, settings in cameras.items():
        rule = rules[f'{camera}_stream']
        assert (rule['topic'], rule['kind']) == ('/perception/detections', 'detections')
        assert rule['frame_id'] == settings['frame_id'], camera
        assert rule['stop'] is True, camera
