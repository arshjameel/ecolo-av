"""
sim.launch.py and aav_gazebo's gazebo.launch.py: what each world and argument starts.

Each test runs a launch file's own setup with some arguments, and looks at what it would
start. Nothing starts: no ROS, no Gazebo. The defaults must start what `pixi run sim`
always has; the road and test_lot worlds add the car's sensors and the parts that read
them (test_lot renders them since 15 Oct). The lidar is on by default (15 Oct).
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


def load(package, name):
    path = os.path.join(get_package_share_directory(package), 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SIM = load('aav_bringup', 'sim.launch.py')
GAZEBO = load('aav_gazebo', 'gazebo.launch.py')
# What the parts that read the camera are called: they need a world that renders it.
CAMERA_PARTS = [{'perception': 'true'}, {'camera_obstacles': 'true'}, {'lane': 'true'}]


def context_for(launch_file, **args):
    """Return a launch context with the file's defaults, and these arguments on top."""
    context = LaunchContext()
    for declared in launch_file.generate_launch_description().get_launch_arguments():
        if declared.default_value is not None:     # a required argument has none
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


# A Node's name, arguments and parameters: launch_ros keeps them to itself until the
# node starts, so the tests read its own fields.
def node(actions, name):
    """Return the node started directly under this name."""
    return next(a for a in actions if isinstance(a, Node) and a._Node__node_name == name)


def arguments(a):
    return [str(x) for x in a._Node__arguments]


def parameters(a):
    """Return the node's parameters that are given as values (not files), as one dict."""
    found = {}
    for p in evaluate_parameters(LaunchContext(), a._Node__parameters):
        if isinstance(p, dict):
            found.update(p)
    return found


def names(actions):
    """Return the names of the nodes and processes started directly."""
    found = set()
    for a in actions:
        if isinstance(a, Node):
            found.add(a._Node__node_name)
        elif isinstance(a, ExecuteProcess):
            found.add(perform_substitutions(LaunchContext(), a.name))
    return found


def test_the_defaults_start_what_pixi_run_sim_always_has():
    actions = started(SIM)
    sent = included(actions)
    assert set(sent) == {'gazebo.launch.py', 'mux.launch.py', 'ego.launch.py',
                         'io_mux.launch.py', 'scripted.launch.py', 'drive_to_point.launch.py',
                         'localization.launch.py', 'navigation.launch.py',
                         'watchdog.launch.py', 'geofence.launch.py'}
    assert sent['gazebo.launch.py']['world'] == 'empty'
    # ego's commands reach Gazebo through the output mux, only while everything is SIM.
    assert sent['io_mux.launch.py'] == {'target': 'gazebo', 'source': 'sim'}
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['lidar'], nav['collision_monitor'], nav['camera_obstacles']) == (
        'false', 'false', 'false', 'false')
    # The empty world renders no camera, radar or lidar: the watchdog watches the rest.
    assert sent['watchdog.launch.py']['inputs'] == 'odom,imu,gps,local,global'
    assert nav['map'].endswith(os.path.join('maps', 'empty.yaml'))
    assert names(actions) == {'imu_filter_madgwick', 'run_info', 'rviz2'}
    assert arguments(node(actions, 'rviz2'))[1].endswith('drive.rviz')


def test_run_info_says_which_algorithms_the_run_uses():
    # The cockpit's Algorithms table (aav_run_info): the launch's own choices, resolved.
    info = parameters(node(started(SIM), 'run_info'))
    assert (info['profile'], info['planner'], info['controller'], info['tracker']) == (
        'sim world:=empty', 'lattice', 'mppi', 'none')
    assert not any(info[k] for k in ('voxels', 'ndt', 'kiss_icp', 'amcl', 'lidar'))
    info = parameters(node(started(SIM, world='road', voxels='true', tracker='lqr',
                                   reversing='false', kiss_ekf='true'), 'run_info'))
    assert (info['profile'], info['tracker'], info['voxels'], info['lidar']) == (
        'sim world:=road', 'lqr', True, True)
    # kiss_ekf:=true runs KISS-ICP too, and says so.
    assert info['kiss_icp'] and info['kiss_ekf']


def test_every_node_runs_on_simulated_time():
    first = started(SIM)[0]
    assert isinstance(first, SetParameter)
    assert perform_substitutions(LaunchContext(), list(first.name)) == 'use_sim_time'
    assert first.value is True


def test_the_road_world_adds_the_radar_to_nav2_and_its_clusters():
    actions = started(SIM, world='road')
    assert arguments(node(actions, 'rviz2'))[1].endswith('road.rviz')
    sent = included(actions)
    assert sent['radar_clusters.launch.py'] == {'minimum_points': '5'}
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['lidar'], nav['collision_monitor']) == ('true', 'true', 'true')
    assert nav['map'].endswith(os.path.join('maps', 'empty.yaml'))
    # Nothing reads the camera by default, so the watchdog leaves it alone.
    assert sent['watchdog.launch.py']['inputs'] == 'odom,imu,gps,local,global,radar'


def test_the_lidar_alone_guards_nav2_with_the_radar_off():
    sent = included(started(SIM, world='road', radar='false'))
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['lidar'], nav['collision_monitor']) == ('false', 'true', 'true')
    assert 'radar_clusters.launch.py' not in sent
    assert sent['gazebo.launch.py']['radar'] == 'false'
    assert 'radar' not in sent['watchdog.launch.py']['inputs']


def test_faults_route_the_camera_and_the_watchdog_can_be_left_out():
    sent = included(started(SIM, world='road', faults='true'))
    assert sent['gazebo.launch.py']['faults'] == 'true'
    assert 'camera' in sent['watchdog.launch.py']['inputs'].split(',')
    assert 'watchdog.launch.py' not in included(started(SIM, world='road', watchdog='false'))


def test_the_watchdog_wants_the_hud_only_where_one_runs():
    assert 'hud' not in included(started(SIM))['watchdog.launch.py']['inputs'].split(',')
    sent = included(started(SIM, hud='true'))
    assert 'hud.launch.py' in sent
    assert 'hud' in sent['watchdog.launch.py']['inputs'].split(',')
    # sim-check --faults: no window, the check itself beats for it.
    sent = included(started(SIM, watch_hud='true'))
    assert 'hud.launch.py' not in sent
    assert 'hud' in sent['watchdog.launch.py']['inputs'].split(',')


def test_the_mux_needs_the_watchdogs_heartbeat_only_where_it_runs():
    assert included(started(SIM))['mux.launch.py']['sensor_lock'] == 'true'
    # Without the watchdog, nothing would ever open the fail-safe lock: Nav2 couldn't drive.
    assert included(started(SIM, watchdog='false'))['mux.launch.py']['sensor_lock'] == 'false'
    # The point driver stops when the watchdog goes quiet, as the mux holds it: only if one runs.
    point = 'drive_to_point.launch.py'
    assert included(started(SIM))[point]['watchdog'] == 'true'
    assert included(started(SIM, watchdog='false'))[point]['watchdog'] == 'false'


MUX = load('aav_command_mux', 'mux.launch.py')


def mux_files(**args):
    """Return the parameter files mux.launch.py gives twist_mux, in order."""
    context = context_for(MUX, **{'geofence_lock': 'true', **args})
    node = MUX.mux(context)[0]
    return [os.path.basename(str(p))
            for p in evaluate_parameters(context, node._Node__parameters)
            if not isinstance(p, dict)]


def test_the_geofence_runs_with_its_lock_and_a_sim_sized_circle():
    sent = included(started(SIM))
    assert sent['geofence.launch.py'] == {'backend': 'gazebo', 'radius': '200.0',
                                          'centre_x': '0.0', 'centre_y': '0.0',
                                          'max_speed': '', 'use_sim_time': 'true'}
    assert included(started(SIM, spine='true'))['geofence.launch.py']['backend'] == 'spine'
    assert included(started(SIM, esp32='true'))['geofence.launch.py']['backend'] == \
        'gazebo_esp32'
    # Its circle round where the car starts, and ego's own cap.
    sent = included(started(SIM, start_y='0.3', sim_speed='3'))
    assert (sent['geofence.launch.py']['centre_y'], sent['geofence.launch.py']['max_speed']) \
        == ('0.3', '3')
    assert sent['mux.launch.py']['geofence_lock'] == 'true'
    sent = included(started(SIM, geofence_radius='12'))
    assert sent['geofence.launch.py']['radius'] == '12'
    # Without the geofence, nothing would ever open its fail-safe lock: no program could drive.
    sent = included(started(SIM, geofence='false'))
    assert 'geofence.launch.py' not in sent
    assert sent['mux.launch.py']['geofence_lock'] == 'false'


def test_the_mux_geofence_lock_file():
    assert mux_files(geofence_lock='false') == ['twist_mux.yaml', 'no_geofence_lock.yaml']
    assert mux_files(sensor_lock='false', geofence_lock='false') == [
        'twist_mux.yaml', 'no_sensor_lock.yaml', 'no_geofence_lock.yaml']
    with pytest.raises(ValueError, match='geofence_lock'):
        MUX.mux(context_for(MUX, geofence_lock='maybe'))


def test_the_mux_lock_is_fail_safe_unless_told_no_watchdog_runs():
    # twist_mux.yaml's 1 s timeout: silence locks.
    assert mux_files() == ['twist_mux.yaml']
    # After it, under the same node name, so it wins: a dict (/**) would lose to it.
    assert mux_files(sensor_lock='false') == ['twist_mux.yaml', 'no_sensor_lock.yaml']
    with pytest.raises(ValueError, match='sensor_lock'):
        MUX.mux(context_for(MUX, sensor_lock='maybe', geofence_lock='true'))


def test_the_road_worlds_full_graph():
    actions = started(SIM, world='road', perception='true', lane='true',
                      camera_obstacles='true', hud='true', obstacle='true', start_y='0.3')
    sent = included(actions)
    assert sent['detector.launch.py'] == {'labels': 'stop sign', 'use_sim_time': 'true'}
    assert sent['lane_finder.launch.py'] == {}
    assert sent['lane_follower.launch.py'] == {'fake': 'false'}
    assert sent['hud.launch.py'] == {}   # aav_hud's window (15 Oct)
    assert sent['navigation.launch.py']['camera_obstacles'] == 'true'
    assert (sent['gazebo.launch.py']['obstacle'], sent['gazebo.launch.py']['start_y']) == (
        'true', '0.3')
    assert {'stop_sign_watcher', 'camera_obstacles'} <= names(actions)


@pytest.mark.parametrize('part', CAMERA_PARTS)
def test_the_cameras_parts_need_a_world_with_a_camera(part):
    with pytest.raises(ValueError, match='world:=road'):
        started(SIM, world='empty', **part)
    started(SIM, world='test_lot', **part)  # the lot renders the cameras: no error


def test_the_screens_and_lichtblick():
    sent = included(started(SIM, hud='true', map='true'))
    assert sent['hud.launch.py'] == {} and sent['map.launch.py'] == {}
    fox = included(started(SIM, foxglove='true'))['foxglove.launch.py']
    assert fox['foxglove_address'] == '127.0.0.1'  # this computer only: test_foxglove_launch


def test_a_pretend_lane_needs_no_camera():
    assert included(started(SIM, lane='fake'))['lane_follower.launch.py'] == {'fake': 'true'}


def test_bad_values_stop_the_launch():
    with pytest.raises(ValueError, match='pick one of'):
        started(SIM, world='moon')
    with pytest.raises(ValueError, match='lane:=maybe'):
        started(SIM, lane='maybe')
    with pytest.raises(ValueError, match='controller:=dwb: pick one of rpp, mppi'):
        started(SIM, controller='dwb')


def test_gazebo_bridges_the_sensors_only_where_the_world_renders_them():
    actions = started(GAZEBO, world='empty', gui='false')
    assert 'gz_road_bridge' not in names(actions)
    assert parameters(node(actions, 'gazebo_adapter'))['radar'] is False
    # One node per sensor, as on the car; the radar's only where the world renders it.
    assert {'gazebo_imu', 'gazebo_gps'} <= names(actions)
    assert not {'gazebo_radar', 'gazebo_camera'} & names(actions)
    for world in ('road', 'test_lot'):
        actions = started(GAZEBO, world=world, gui='false')
        assert 'gz_road_bridge' in names(actions), world
        assert parameters(node(actions, 'gazebo_adapter'))['radar'] is True, world
        assert 'gazebo_radar' in names(actions), world
        assert parameters(node(actions, 'gz_road_bridge'))['config_file'].endswith(
            'bridge_road.yaml'), world


def test_gazebo_obstacle_and_start_pose():
    actions = started(GAZEBO, world='road', gui='false', obstacle='true', start_y='0.3',
                      start_yaw='0.1')
    spawn = arguments(node(actions, 'spawn_aav'))
    assert spawn[spawn.index('-y') + 1] == '0.3' and spawn[spawn.index('-Y') + 1] == '0.1'
    box = arguments(node(actions, 'spawn_obstacle'))
    assert box[box.index('-file') + 1].endswith(os.path.join('obstacle', 'model.sdf'))
    assert (box[box.index('-x') + 1], box[box.index('-y') + 1]) == ('15.5', '0.0')
    assert 'spawn_obstacle' not in names(started(GAZEBO, world='road', gui='false'))


def test_gazebo_runs_the_simulator_headless_without_a_window():
    gz = included(started(GAZEBO, world='empty', gui='false'))['gz_sim.launch.py']['gz_args']
    assert gz.startswith('-r -s ') and gz.endswith('empty.sdf')
    # Only Linux renders through EGL; a Mac's ogre2 renders headless on Metal as it is.
    road = included(started(GAZEBO, world='road', gui='false'))['gz_sim.launch.py']['gz_args']
    assert ('--headless-rendering' in road) == (not GAZEBO.MAC)
    assert '--headless-rendering' not in gz


def test_the_lidar_only_where_the_world_renders_it():
    road = started(GAZEBO, world='road', gui='false', lidar='true')
    assert included(road)['description.launch.py']['lidar'] == 'true'
    assert parameters(node(road, 'gz_lidar_bridge'))['config_file'].endswith(
        'bridge_lidar.yaml')
    lot = started(GAZEBO, world='test_lot', gui='false', lidar='true')
    assert included(lot)['description.launch.py']['lidar'] == 'true'
    actions = started(GAZEBO, world='empty', gui='false', lidar='true')
    assert included(actions)['description.launch.py']['lidar'] == 'false'
    assert 'gz_lidar_bridge' not in names(actions)


def test_the_lidar_is_on_by_default():
    road = started(GAZEBO, world='road', gui='false')
    assert included(road)['description.launch.py']['lidar'] == 'true'
    assert 'gz_lidar_bridge' in names(road)
    assert included(started(SIM, world='road'))['gazebo.launch.py']['lidar'] == 'true'
    off = started(GAZEBO, world='road', gui='false', lidar='false')
    assert included(off)['description.launch.py']['lidar'] == 'false'
    assert 'gz_lidar_bridge' not in names(off)


def test_the_chase_camera_only_where_the_world_renders_it():
    road = started(GAZEBO, world='road', gui='false')
    assert included(road)['description.launch.py']['chase_camera'] == 'true'
    assert parameters(node(road, 'gz_chase_bridge'))['config_file'].endswith(
        'bridge_chase.yaml')
    jpeg = node(road, 'chase_jpeg')
    assert (jpeg.node_package, jpeg.node_executable) == ('image_transport', 'republish')
    assert included(started(SIM, world='road'))['gazebo.launch.py']['chase_camera'] == 'true'
    for actions in (started(GAZEBO, world='empty', gui='false'),
                    started(GAZEBO, world='road', gui='false', chase_camera='false')):
        assert included(actions)['description.launch.py']['chase_camera'] == 'false'
        assert not {'gz_chase_bridge', 'chase_jpeg'} & names(actions)


def test_the_spine_in_the_loop_runs_the_rust_bridge_unless_asked():
    def bridge(**args):
        actions = started(GAZEBO, world='empty', gui='false', spine='true', **args)
        a = node(actions, 'spine_bridge')
        return a.node_package, a.node_executable

    assert bridge() == ('aav_spine_bridge', 'spine_bridge')
    assert bridge(bridge='python') == ('aav_spine', 'spine_bridge')
    sent = included(started(SIM, spine='true', bridge='python'))['gazebo.launch.py']
    assert sent['bridge'] == 'python'
    with pytest.raises(ValueError, match='bridge:=cpp'):
        bridge(bridge='cpp')


def test_ndt_needs_the_test_lot_the_lidar_and_its_map(tmp_path):
    # A trial, scored only (docs/designs/2026-10-29_ndt.md): off by default.
    assert 'ndt.launch.py' not in included(started(SIM, world='test_lot'))
    map_file = tmp_path / 'test_lot.pcd'
    with pytest.raises(ValueError, match='world:=test_lot'):
        started(SIM, ndt='true', ndt_map=str(map_file))         # the empty world
    with pytest.raises(ValueError, match='lidar:=true'):
        started(SIM, world='test_lot', lidar='false', ndt='true', ndt_map=str(map_file))
    with pytest.raises(ValueError, match='pixi run ndt-map drive'):
        started(SIM, world='test_lot', ndt='true', ndt_map=str(map_file))
    map_file.write_bytes(b'')
    sent = included(started(SIM, world='test_lot', ndt='true', ndt_map=str(map_file),
                            start_y='4.0', start_yaw='1.0'))['ndt.launch.py']
    # The start pose is the car's, as AMCL's; on simulated time.
    assert sent == {'use_sim_time': 'true', 'ndt_map': str(map_file), 'ndt_x': '0.0',
                    'ndt_y': '4.0', 'ndt_yaw': '1.0'}


def test_ndt_ekf_starts_ndt_and_feeds_its_pose_to_the_global_ekf(tmp_path):
    # Off by default: the global EKF on GPS alone.
    assert included(started(SIM, world='test_lot'))['localization.launch.py'][
        'ndt_ekf'] == 'false'
    map_file = tmp_path / 'test_lot.pcd'
    map_file.write_bytes(b'')
    sent = included(started(SIM, world='test_lot', ndt_ekf='true', ndt_map=str(map_file)))
    assert sent['localization.launch.py']['ndt_ekf'] == 'true'
    assert 'ndt.launch.py' in sent                      # ndt:=true is implied
    with pytest.raises(ValueError, match='lidar:=true'):
        started(SIM, world='test_lot', lidar='false', ndt_ekf='true', ndt_map=str(map_file))


LOCALIZATION = load('aav_localization', 'localization.launch.py')


def global_ekf_files(**args):
    """Return the parameter files ekf_global is given, by name."""
    (ekf,) = LOCALIZATION.global_ekf(context_for(LOCALIZATION, **args))
    return [os.path.basename(str(p.evaluate(LaunchContext())))
            for p in ekf._Node__parameters if isinstance(p, ParameterFile)]


def test_the_global_ekf_reads_ndt_only_with_ndt_ekf():
    assert global_ekf_files() == ['ekf_global.yaml']
    assert global_ekf_files(ndt_ekf='true') == ['ekf_global.yaml', 'ekf_global_ndt.yaml']
    path = os.path.join(get_package_share_directory('aav_localization'), 'config',
                        'ekf_global_ndt.yaml')
    with open(path) as f:
        p = yaml.safe_load(f)['ekf_global']['ros__parameters']
    assert p['pose0'] == '/localization/ndt/pose'
    # x, y and yaw only: a flat lot (design N1); never height, roll or pitch.
    assert [i for i, on in enumerate(p['pose0_config']) if on] == [0, 1, 5]
    assert p['pose0_differential'] is False
    assert 0 < p['pose0_rejection_threshold'] < 10
    # NDT's poses come late (the cloud's stamp): put each in at its own time.
    assert p['smooth_lagged_data'] is True and p['history_length'] >= 0.5


def test_kiss_icp_needs_the_lidar_and_is_off_by_default():
    # A trial, scored only (docs/designs/2026-10-30_kiss-icp.md).
    assert 'kiss_icp.launch.py' not in included(started(SIM, world='test_lot'))
    with pytest.raises(ValueError, match=r'kiss_icp:=true .*needs the lidar'):
        started(SIM, kiss_icp='true')                           # the empty world
    with pytest.raises(ValueError, match=r'kiss_icp:=true .*needs the lidar'):
        started(SIM, world='test_lot', lidar='false', kiss_icp='true')
    sent = included(started(SIM, world='test_lot', kiss_icp='true'))['kiss_icp.launch.py']
    assert sent == {'use_sim_time': 'true'}
    assert 'kiss_icp.launch.py' in included(started(SIM, world='road', kiss_icp='true'))


def test_kiss_ekf_starts_kiss_icp_and_feeds_its_twist_to_the_local_ekf():
    # Off by default: the local EKF on the wheels and the IMU alone.
    assert included(started(SIM, world='test_lot'))['localization.launch.py'][
        'kiss_ekf'] == 'false'
    sent = included(started(SIM, world='test_lot', kiss_ekf='true'))
    assert sent['localization.launch.py']['kiss_ekf'] == 'true'
    assert 'kiss_icp.launch.py' in sent                 # kiss_icp:=true is implied
    with pytest.raises(ValueError, match='kiss_ekf:=true'):
        started(SIM, world='test_lot', lidar='false', kiss_ekf='true')


def local_ekf_files(**args):
    """Return the parameter files ekf_local is given, by name."""
    (ekf,) = LOCALIZATION.local_ekf(context_for(LOCALIZATION, **args))
    return [os.path.basename(str(p.evaluate(LaunchContext())))
            for p in ekf._Node__parameters if isinstance(p, ParameterFile)]


def test_the_local_ekf_reads_kiss_icp_only_with_kiss_ekf():
    assert local_ekf_files() == ['ekf_local.yaml']
    assert local_ekf_files(kiss_ekf='true') == ['ekf_local.yaml', 'ekf_local_kiss.yaml']
    path = os.path.join(get_package_share_directory('aav_localization'), 'config',
                        'ekf_local_kiss.yaml')
    with open(path) as f:
        p = yaml.safe_load(f)['ekf_local']['ros__parameters']
    assert p['odom1'] == '/localization/kiss_icp/odom'
    # Its twist only, vx and vyaw: never its pose (it drifts, in its own frame).
    assert [i for i, on in enumerate(p['odom1_config']) if on] == [6, 11]
    assert p['odom1_differential'] is False
    assert 0 < p['odom1_twist_rejection_threshold'] < 10
    # Its twists carry the cloud's stamp: put each in at its own time.
    assert p['smooth_lagged_data'] is True and p['history_length'] >= 0.5
    # Doesn't clash with ekf_local.yaml's own inputs.
    base = os.path.join(get_package_share_directory('aav_localization'), 'config',
                        'ekf_local.yaml')
    with open(base) as f:
        assert 'odom1' not in yaml.safe_load(f)['ekf_local']['ros__parameters']
