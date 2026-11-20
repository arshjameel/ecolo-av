"""
sim.launch.py and aav_sim_models's ravensim.launch.py: what each world and argument starts.

Each test runs a launch file's own setup with some arguments, and looks at what it would
start. Nothing starts: no ROS, no simulator. The defaults must start what `pixi run sim`
always has; the road and test_lot worlds add the car's sensors and the parts that read
them (test_lot renders them since 15 Oct). The lidar is on by default (15 Oct).
"""

import importlib.util
import os

from aav_sim_models import worlds
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
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
RAVENSIM = load('aav_sim_models', 'ravensim.launch.py')
# What the parts that read the camera are called: they need a world that renders it.
CAMERA_PARTS = [{'perception': 'true'}, {'camera_obstacles': 'true'}, {'lane': 'true'}]


@pytest.fixture
def hud_package(monkeypatch):
    """
    Let hud:=true find aav_hud, which aav_bringup doesn't depend on.

    The car's install skips aav_hud (tools/car_install.sh), so aav_bringup may not depend
    on it, and colcon test then leaves it off the package path. Where this workspace
    built it, put it back; where it wasn't built (a car computer), skip.
    """
    try:
        get_package_share_directory('aav_hud')
        return
    except PackageNotFoundError:
        pass
    prefix = os.path.join(os.path.dirname(get_package_prefix('aav_bringup')), 'aav_hud')
    if not os.path.isdir(os.path.join(prefix, 'share', 'aav_hud')):
        pytest.skip('aav_hud is not built here (a car computer skips it)')
    monkeypatch.setenv('AMENT_PREFIX_PATH',
                       os.pathsep.join([prefix, os.environ.get('AMENT_PREFIX_PATH', '')]))


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


# The league (docs/league.md): ours is the simulator's default (Ben, 14 Nov: "Ours default
# now, then delete"; Nav2's stock stack went on 17 Nov); stack:=best is every level's
# champion; a switch given by hand wins over either.
def stack_of(**args):
    """Return what navigation.launch.py and ego are given: the stack's switches, resolved."""
    sent = included(started(SIM, **args))
    nav = sent['navigation.launch.py']
    return {'planner': nav['planner'], 'controller': nav['controller'],
            'tracker': nav['tracker'], 'costmap': nav['costmap'],
            'navigator': nav['navigator'], 'smoother': nav['smoother'],
            'behaviour': nav['behaviour'], 'road_ahead': sent['ego.launch.py']['road_ahead'],
            'ndt_ekf': sent['localization.launch.py']['ndt_ekf'],
            'kiss_ekf': sent['localization.launch.py']['kiss_ekf'],
            'lattice_moves': nav['lattice_moves'],
            'planner_search': nav['planner_search'],
            'plan_smoothing': nav['plan_smoothing'], 'map_correction': nav['map_correction'],
            'mppi_lag_steer_gamma': nav['mppi_lag_steer_gamma']}


OURS = {'planner': 'own', 'controller': 'mppi', 'tracker': 'mppi_lag', 'costmap': 'own',
        'navigator': 'own', 'smoother': 'own', 'behaviour': 'true', 'road_ahead': 'true',
        'ndt_ekf': 'false', 'kiss_ekf': 'false', 'lattice_moves': 'nav2',
        'planner_search': 'lattice', 'plan_smoothing': 'strong', 'map_correction': 'slow',
        'mppi_lag_steer_gamma': '0.05'}
BEST = {'planner': 'own', 'controller': 'mppi', 'tracker': 'mppi_lag', 'costmap': 'own',
        'navigator': 'own', 'smoother': 'own', 'behaviour': 'false', 'road_ahead': 'false',
        'ndt_ekf': 'false', 'kiss_ekf': 'false', 'lattice_moves': 'nav2',
        'planner_search': 'lattice', 'plan_smoothing': 'nav2', 'map_correction': 'newest',
        'mppi_lag_steer_gamma': ''}


def test_the_default_stack_is_ours():
    assert stack_of() == OURS
    assert stack_of(stack='ours') == OURS


def test_stack_best_is_every_levels_champion():
    assert stack_of(stack='best') == BEST
    # No stack at all: each switch's fallback, the champions'.
    assert stack_of(stack='') == BEST
    # Nav2's stack went (17 Nov, TODO 184).
    with pytest.raises(ValueError, match='stock'):
        stack_of(stack='stock')


def test_a_switch_given_by_hand_wins_over_the_stack():
    # One piece a challenger, the rest ours: tracker:=lqr is told apart from "not given".
    assert stack_of(tracker='lqr', reversing='false') == dict(OURS, tracker='lqr')
    assert stack_of(behaviour='false') == dict(OURS, behaviour='false', road_ahead='false')
    # And one piece ours' on the champions' stack.
    assert stack_of(stack='best', behaviour='true') == dict(BEST, behaviour='true',
                                                            road_ahead='true')


def test_ours_plans_the_lanes_centre_where_the_world_has_a_lane_map():
    # TODO 148 (16 Nov): path:=lane is ours' pick. With a lane map (the road world's) our
    # planner gets it; without one (the empty world, the lot) the lattice alone.
    def lane_route(**args):
        return included(started(SIM, **args))['navigation.launch.py']['lane_route']
    assert lane_route(world='road').endswith('road.lanes.yaml')
    assert lane_route() == ''
    assert lane_route(world='road', stack='best') == ''
    assert lane_route(world='road', path='lattice_rs_strong') == ''

    # TODO 214: path:=lane_jerk passes with the jerk-limited QP; the others the cosine.
    def pass_shape(**args):
        return included(started(SIM, **args))['navigation.launch.py']['lane_pass_shape']
    assert pass_shape(world='road', path='lane_jerk') == 'jerk'
    assert pass_shape(world='road') == ''


def test_the_global_costmap_is_made_of_chunks_only_where_there_are_chunks(tmp_path):
    # TODO 175: costmap_chunks:=auto is the campus's chunks (pixi run campus makes them);
    # a world without any (the empty world, the lot) keeps the whole map.
    def chunks(**args):
        return included(started(SIM, **args))['navigation.launch.py']['costmap_chunks']
    assert chunks() == ''
    assert chunks(costmap_chunks='auto') == ''
    assert chunks(world='test_lot', costmap_chunks='auto') == ''
    assert chunks(costmap_chunks='off') == ''
    (tmp_path / 'chunks.yaml').write_text('origin: [0, 0]\nsize: 64\n')
    assert chunks(costmap_chunks=str(tmp_path)) == str(tmp_path)
    assert SIM.campus_costmap_chunks('auto', 'test_lot') == ''
    assert SIM.campus_costmap_chunks('off', 'campus') == ''


def test_ego_waits_for_the_road_ahead_only_where_its_writer_runs():
    # The behaviour node writes /world/road_ahead inside navigation: with nav:=false ego
    # must not wait for it, or it holds the point driver at 0 (TODO 94, found 15 Nov).
    sent = included(started(SIM, nav='false'))
    assert sent['ego.launch.py']['road_ahead'] == 'false'
    assert included(started(SIM))['ego.launch.py']['road_ahead'] == 'true'


def test_the_simulated_cars_speed_loop_reaches_ego_and_the_car():
    # TODO 163: the car model runs it, and ego's trace assumes the same one.
    sent = included(started(SIM, speed_loop='firm'))
    assert sent['ego.launch.py']['speed_loop'] == 'firm'
    assert sent['ravensim.launch.py']['speed_loop'] == 'firm'
    assert included(started(SIM))['ego.launch.py']['speed_loop'] == 'default'
    with pytest.raises(ValueError, match='speed_loop:=fast'):
        started(SIM, speed_loop='fast')
    with pytest.raises(ValueError, match='PI'):
        started(SIM, speed_loop='lqi', tracker='mpc_lag', reversing='false')
    # speed_loop_model:= (TODO 163): the car runs one loop, every model plans with another.
    split = included(started(SIM, speed_loop='firm', speed_loop_model='smooth'))
    assert split['ravensim.launch.py']['speed_loop'] == 'firm'
    assert split['ego.launch.py']['speed_loop'] == 'smooth'
    assert split['navigation.launch.py']['speed_loop'] == 'smooth'
    with pytest.raises(ValueError, match='speed_loop_model:=fast'):
        started(SIM, speed_loop_model='fast')
    # The MPCs refuse the LQI as their model, not as the car's loop.
    with pytest.raises(ValueError, match='PI'):
        started(SIM, speed_loop_model='lqi', tracker='mpc_lag', reversing='false')
    sent = included(started(SIM, speed_loop='lqi', speed_loop_model='smooth',
                            tracker='mpc_lag', reversing='false'))
    assert sent['ravensim.launch.py']['speed_loop'] == 'lqi'


def test_the_stacks_switches_are_declared_empty_so_a_hand_set_one_is_seen():
    declared = {a.name: a.default_value
                for a in SIM.generate_launch_description().get_launch_arguments()}
    for name in SIM.STACK_FALLBACK:
        assert perform_substitutions(LaunchContext(), declared[name]) == '', name
    assert perform_substitutions(LaunchContext(), declared['stack']) == 'ours'
    # The car can reverse (Ben, 14 Nov): still the sim's default.
    assert perform_substitutions(LaunchContext(), declared['reversing']) == 'true'


def test_the_defaults_start_what_pixi_run_sim_always_has():
    actions = started(SIM)
    sent = included(actions)
    assert set(sent) == {'ravensim.launch.py', 'mux.launch.py', 'ego.launch.py',
                         'io_mux.launch.py', 'scripted.launch.py', 'drive_to_point.launch.py',
                         'localization.launch.py', 'navigation.launch.py',
                         'watchdog.launch.py', 'geofence.launch.py'}
    assert sent['ravensim.launch.py']['world'] == 'empty'
    # ego's commands reach Ravensim through the output mux, only while everything is SIM.
    assert sent['io_mux.launch.py'] == {'target': 'sim', 'source': 'sim', 'io_mux_impl': 'rust'}
    back = included(started(SIM, io_mux_impl='Python'))['io_mux.launch.py']
    assert back['io_mux_impl'] == 'python'
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['lidar'], nav['stop_zone'], nav['camera_obstacles']) == (
        'false', 'false', 'false', 'false')
    # The empty world renders no camera, radar or lidar: the watchdog watches the rest.
    inputs = sent['watchdog.launch.py']['inputs'].split(',')
    # The sensors, then our own nodes the stack starts (warn only: TODO 180).
    sensors = [i for i in inputs if not i.startswith(('node_', 'status_'))]
    assert sensors == 'odom,imu,gps,local,global,steering,speed'.split(',')
    assert {'node_ego', 'node_planner', 'node_mppi_lag'} <= set(inputs)
    assert nav['map'].endswith(os.path.join('maps', 'empty.yaml'))
    assert names(actions) == {'goal_leash', 'imu_filter_madgwick', 'run_info', 'rviz2'}
    assert arguments(node(actions, 'rviz2'))[1].endswith('drive.rviz')


def test_run_info_says_which_algorithms_the_run_uses():
    # The cockpit's Algorithms table (aav_run_info): the launch's own choices, resolved.
    info = parameters(node(started(SIM), 'run_info'))
    assert (info['profile'], info['planner'], info['controller'], info['tracker']) == (
        'sim world:=empty', 'own', 'mppi', 'mppi_lag')
    info = parameters(node(started(SIM, stack='best'), 'run_info'))
    assert (info['planner'], info['controller'], info['tracker']) == ('own', 'mppi', 'mppi_lag')
    assert not any(info[k] for k in ('voxels', 'ndt', 'kiss_icp', 'amcl', 'lidar'))
    info = parameters(node(started(SIM, world='road', voxels='true', tracker='lqr',
                                   reversing='false', kiss_ekf='true'), 'run_info'))
    assert (info['profile'], info['tracker'], info['voxels'], info['lidar']) == (
        'sim world:=road', 'lqr', True, True)
    # kiss_ekf:=true runs KISS-ICP too, and says so.
    assert info['kiss_icp'] and info['kiss_ekf']


def test_odom_lidar_picks_kinematic_icp_in_kiss_icps_place():
    # TODO 216: pose:=kinematic_ekf, Kinematic-ICP's idea in Rust; else KISS-ICP.
    sent = included(started(SIM, world='road', pose='kinematic_ekf'))
    assert 'kinematic_icp.launch.py' in sent and 'kiss_icp.launch.py' not in sent
    assert sent['localization.launch.py']['kiss_ekf'] == 'true'
    with pytest.raises(ValueError, match='odom_lidar'):
        started(SIM, world='road', kiss_icp='true', odom_lidar='loam')


def test_every_node_runs_on_simulated_time():
    first = started(SIM)[0]
    assert isinstance(first, SetParameter)
    assert perform_substitutions(LaunchContext(), list(first.name)) == 'use_sim_time'
    assert first.value is True


def test_the_road_world_adds_the_radar_to_navigation_and_its_clusters():
    actions = started(SIM, world='road')
    assert arguments(node(actions, 'rviz2'))[1].endswith('road.rviz')
    sent = included(actions)
    assert sent['radar_clusters.launch.py'] == {'minimum_points': '5'}
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['lidar'], nav['stop_zone']) == ('true', 'true', 'true')
    assert nav['map'].endswith(os.path.join('maps', 'empty.yaml'))
    # Ours' lane pose reads the camera (17 Nov), so the watchdog watches it.
    inputs = sent['watchdog.launch.py']['inputs'].split(',')
    # The sensors, then our own nodes the stack starts (warn only: TODO 180).
    sensors = [i for i in inputs if not i.startswith(('node_', 'status_'))]
    assert sensors == 'odom,imu,gps,local,global,camera,radar,lidar,steering,speed'.split(',')
    # Nothing reads it on the champions' stack (best), so the watchdog leaves it alone.
    stock = included(started(SIM, world='road', stack='best'))['watchdog.launch.py']
    assert 'camera' not in stock['inputs'].split(',')
    assert {'node_ego', 'node_planner', 'node_mppi_lag'} <= set(inputs)


def test_the_lidar_alone_guards_the_car_with_the_radar_off():
    sent = included(started(SIM, world='road', radar='false'))
    nav = sent['navigation.launch.py']
    assert (nav['radar'], nav['lidar'], nav['stop_zone']) == ('false', 'true', 'true')
    assert 'radar_clusters.launch.py' not in sent
    assert sent['ravensim.launch.py']['radar'] == 'false'
    assert 'radar' not in sent['watchdog.launch.py']['inputs']
    assert 'lidar' in sent['watchdog.launch.py']['inputs'].split(',')
    # No lidar, nothing to watch (TODO 246): its silence would stop the car for nothing.
    off = included(started(SIM, world='road', lidar='false'))['watchdog.launch.py']
    assert 'lidar' not in off['inputs'].split(',')


def test_faults_route_the_camera_and_the_watchdog_can_be_left_out():
    sent = included(started(SIM, world='road', faults='true'))
    assert sent['ravensim.launch.py']['faults'] == 'true'
    assert 'camera' in sent['watchdog.launch.py']['inputs'].split(',')
    assert 'watchdog.launch.py' not in included(started(SIM, world='road', watchdog='false'))


def test_the_watchdog_wants_the_hud_only_where_one_runs(hud_package):
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
    # Without the watchdog, nothing would ever open the fail-safe lock: the car couldn't drive.
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
    assert sent['geofence.launch.py'] == {'backend': 'sim', 'radius': '200.0',
                                          'centre_x': '0.0', 'centre_y': '0.0',
                                          'max_speed': '', 'use_sim_time': 'true',
                                          'fix_topic': '/localization/gps/odom',
                                          'geofence_impl': 'rust'}
    assert included(started(SIM, spine='true'))['geofence.launch.py']['backend'] == 'spine'
    assert included(started(SIM, esp32='true'))['geofence.launch.py']['backend'] == \
        'sim_esp32'
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


def test_the_road_worlds_full_graph(hud_package):
    actions = started(SIM, world='road', perception='true', lane='true',
                      camera_obstacles='true', hud='true', obstacle='true', start_y='0.3')
    sent = included(actions)
    assert sent['detector.launch.py'] == {'labels': 'stop sign', 'use_sim_time': 'true',
                                          'detector_impl': 'python'}
    assert sent['lane_finder.launch.py'] == {'lane_finder_impl': 'rust'}
    assert 'lane_follower.launch.py' not in sent   # removed 19 Nov (TODO 264)
    assert sent['hud.launch.py'] == {}   # aav_hud's window (15 Oct)
    assert sent['navigation.launch.py']['camera_obstacles'] == 'true'
    assert (sent['ravensim.launch.py']['obstacle'], sent['ravensim.launch.py']['start_y']) == (
        'true', '0.3')
    assert {'stop_sign_watcher', 'camera_obstacles'} <= names(actions)


def test_detector_impl_picks_the_rust_detector_or_says_what_to_pick():
    # detector_impl:=rust: aav_detector_rs's detector in the Python one's place (TODO 243).
    sent = included(started(SIM, world='road', perception='true', detector_impl='Rust'))
    assert sent['detector.launch.py']['detector_impl'] == 'rust'
    with pytest.raises(ValueError, match='detector_impl:=cpp: pick one of python, rust'):
        started(SIM, world='road', perception='true', detector_impl='cpp')


@pytest.mark.parametrize('part', CAMERA_PARTS)
def test_the_cameras_parts_need_a_world_with_a_camera(part):
    with pytest.raises(ValueError, match='world:=road'):
        started(SIM, world='empty', **part)
    started(SIM, world='test_lot', **part)  # the lot renders the cameras: no error


def test_the_screens_and_lichtblick(hud_package):
    sent = included(started(SIM, hud='true', map='true'))
    assert sent['hud.launch.py'] == {} and sent['map.launch.py'] == {}
    fox = included(started(SIM, foxglove='true'))['foxglove.launch.py']
    assert fox['foxglove_address'] == '127.0.0.1'  # this computer only: test_foxglove_launch


def test_the_lane_followers_are_gone():
    # TODO 264 (19 Nov): the costmap draws the lanes; lane:=fake and the follower went.
    with pytest.raises(ValueError, match='lane:=fake'):
        started(SIM, world='road', lane='fake')
    assert 'lane_follower.launch.py' not in included(started(SIM, world='road', lane='true'))


def test_camera_lanes_start_the_lane_finder_and_reach_the_costmap():
    # lanes:=camera (TODO 264): the lane finder's lines into the global costmap.
    sent = included(started(SIM, world='road', lanes='camera'))
    assert sent['lane_finder.launch.py'] == {'lane_finder_impl': 'rust'}
    nav = sent['navigation.launch.py']
    assert (nav['lane_source'], nav['lane_costs']) == ('camera', '')
    both = included(started(SIM, world='road', lanes='both'))['navigation.launch.py']
    assert both['lane_source'] == 'both' and both['lane_costs'].endswith('road.lanes.yaml')
    # Off (ours' lanes level): the lane source stays the map's (the lane finder may still
    # run, for ours' lane pose).
    off = included(started(SIM, world='road'))
    assert off['navigation.launch.py']['lane_source'] == 'map'
    assert 'lane_finder.launch.py' not in included(started(SIM, world='road', pose='ekf'))
    with pytest.raises(ValueError, match='lane_source:=camera'):
        started(SIM, world='empty', lanes='camera')


def test_bad_values_stop_the_launch():
    with pytest.raises(ValueError, match='pick one of'):
        started(SIM, world='moon')
    with pytest.raises(ValueError, match='lane:=maybe'):
        started(SIM, lane='maybe')
    with pytest.raises(ValueError, match='controller:=dwb: pick one of mppi'):
        started(SIM, controller='dwb')


def cameras_command(actions):
    """Return the cameras' command line (sim_cameras), or None if they don't run."""
    for a in actions:
        if isinstance(a, ExecuteProcess) and a.name and \
                perform_substitutions(LaunchContext(), a.name) == 'sim_cameras':
            return [perform_substitutions(LaunchContext(), c) for c in a.cmd]
    return None


def test_ravensim_runs_the_sensors_only_where_the_world_renders_them():
    actions = started(RAVENSIM, world='empty')
    assert parameters(node(actions, 'sim_adapter'))['radar'] is False
    assert parameters(node(actions, 'simulator'))['lidar'] is False
    # One node per sensor, as on the car; the radar's only where the world renders it.
    assert {'simulator', 'sim_imu', 'sim_gps'} <= names(actions)
    assert not {'sim_radar', 'sim_camera_relay', 'sim_cameras'} & names(actions)
    for world in ('road', 'test_lot'):
        actions = started(RAVENSIM, world=world)
        assert parameters(node(actions, 'sim_adapter'))['radar'] is True, world
        assert 'sim_radar' in names(actions), world
        assert parameters(node(actions, 'simulator'))['world'].endswith(f'{world}.sdf')
        assert cameras_command(actions) is not None, world
    # No Gazebo left: no gz sim, no bridges, no spawning.
    road = names(started(RAVENSIM, world='road'))
    assert not any(n.startswith(('gz_', 'spawn_')) for n in road)


def remaps(a):
    """Return the node's remappings, as (from, to) text."""
    return [tuple(part if isinstance(part, str)
                  else perform_substitutions(LaunchContext(), list(part)) for part in pair)
            for pair in (a._Node__remappings or [])]


def test_faults_route_the_lidar_through_its_relay():
    # TODO 246: fault.lidar can stop or freeze the lidar, only with faults:=true.
    plain = started(RAVENSIM, world='road')
    assert 'sim_lidar_relay' not in names(plain)
    assert remaps(node(plain, 'simulator')) == []
    faulty = started(RAVENSIM, world='road', faults='true')
    assert {'sim_lidar_relay', 'sim_camera_relay'} <= names(faulty)
    assert remaps(node(faulty, 'simulator')) == [('/sensors/lidar/points',
                                                  '/vehicle/sim/lidar/points')]
    # No lidar, no relay.
    no_lidar = started(RAVENSIM, world='road', faults='true', lidar='false')
    assert 'sim_lidar_relay' not in names(no_lidar)


def test_ravensim_obstacle_and_start_pose():
    actions = started(RAVENSIM, world='road', obstacle='true', start_y='0.3', start_yaw='0.1')
    sim = parameters(node(actions, 'simulator'))
    assert (sim['start_y'], sim['start_yaw']) == (0.3, 0.1)
    assert sim['obstacle'] is True
    assert sim['obstacle_model'].endswith(os.path.join('obstacle', 'model.sdf'))
    assert (sim['obstacle_x'], sim['obstacle_y']) == (15.5, 0.0)
    assert parameters(node(started(RAVENSIM, world='road'), 'simulator'))['obstacle'] is False


def test_the_cameras_lockstep_under_software_rendering(monkeypatch):
    monkeypatch.delenv('AAV_SOFTWARE_GL', raising=False)
    assert parameters(node(started(RAVENSIM, world='road'), 'simulator'))[
        'camera_lockstep'] is False
    monkeypatch.setenv('AAV_SOFTWARE_GL', '1')     # tools/software_gl.sh
    assert parameters(node(started(RAVENSIM, world='road'), 'simulator'))[
        'camera_lockstep'] is True
    # Said outright, either way; and never with no cameras to wait for.
    assert parameters(node(started(RAVENSIM, world='road', camera_lockstep='false'),
                           'simulator'))['camera_lockstep'] is False
    assert parameters(node(started(RAVENSIM, world='road', cameras='false'),
                           'simulator'))['camera_lockstep'] is False
    monkeypatch.delenv('AAV_SOFTWARE_GL')
    assert parameters(node(started(RAVENSIM, world='road', camera_lockstep='true'),
                           'simulator'))['camera_lockstep'] is True
    sent = included(started(SIM, world='road', camera_lockstep='true'))['ravensim.launch.py']
    assert sent['camera_lockstep'] == 'true'


def test_the_cameras_graphics_pass_through():
    sent = included(started(SIM, world='road', graphics='simple'))['ravensim.launch.py']
    assert sent['graphics'] == 'simple'
    assert included(started(SIM, world='road'))['ravensim.launch.py']['graphics'] == 'auto'


def test_the_old_simulator_names_are_refused():
    with pytest.raises(ValueError, match='old name'):
        started(SIM, simulator='own')
    with pytest.raises(ValueError, match='Gazebo was removed'):
        started(SIM, simulator='gazebo')


def test_the_lidar_only_where_the_world_renders_it():
    road = started(RAVENSIM, world='road', lidar='true')
    assert parameters(node(road, 'simulator'))['lidar'] is True
    lot = started(RAVENSIM, world='test_lot', lidar='true')
    assert parameters(node(lot, 'simulator'))['lidar'] is True
    actions = started(RAVENSIM, world='empty', lidar='true')
    assert parameters(node(actions, 'simulator'))['lidar'] is False


def test_the_lidar_is_on_by_default():
    road = started(RAVENSIM, world='road')
    assert parameters(node(road, 'simulator'))['lidar'] is True
    assert included(started(SIM, world='road'))['ravensim.launch.py']['lidar'] == 'true'
    off = started(RAVENSIM, world='road', lidar='false')
    assert parameters(node(off, 'simulator'))['lidar'] is False


def test_the_chase_camera_only_where_the_world_renders_it():
    road = started(RAVENSIM, world='road')
    assert '--cameras' not in cameras_command(road)     # all four, the chase camera too
    jpeg = node(road, 'chase_jpeg')
    assert (jpeg.node_package, jpeg.node_executable) == ('image_transport', 'republish')
    assert included(started(SIM, world='road'))['ravensim.launch.py']['chase_camera'] == 'true'
    for actions in (started(RAVENSIM, world='empty'),
                    started(RAVENSIM, world='road', chase_camera='false')):
        assert 'chase_jpeg' not in names(actions)
    cmd = cameras_command(started(RAVENSIM, world='road', chase_camera='false'))
    assert cmd[cmd.index('--cameras') + 1] == 'front_left,front_right,rear'


def test_the_spine_in_the_loop_runs_the_rust_bridge_unless_asked():
    def bridge(**args):
        actions = started(RAVENSIM, world='empty', spine='true', **args)
        a = node(actions, 'spine_bridge')
        return a.node_package, a.node_executable

    assert bridge() == ('aav_spine_bridge', 'spine_bridge')
    assert bridge(bridge='python') == ('aav_spine', 'spine_bridge')
    sent = included(started(SIM, spine='true', bridge='python'))['ravensim.launch.py']
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
                    'ndt_y': '4.0', 'ndt_yaw': '1.0', 'ndt_chunks': ''}


def campus_files(folder):
    made = folder / 'maps' / 'campus'
    made.mkdir(parents=True)
    (made / 'campus.sdf').write_text(
        '<sdf><world><plugin name="ravensim::sensors"/></world></sdf>')
    (made / 'place.yaml').write_text('start: {x: 49.25, y: -70.75, yaw: 0.5}\n')
    (folder / 'maps' / 'ndt').mkdir()
    (folder / 'maps' / 'ndt' / 'campus.pcd').write_bytes(b'')


def test_the_campus_starts_the_car_in_its_lot_with_its_own_maps(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    campus_files(tmp_path)
    sent = included(started(SIM, world='campus', ndt='true'))
    ravensim = sent['ravensim.launch.py']
    assert (ravensim['start_x'], ravensim['start_y'], ravensim['start_yaw']) == (
        '49.25', '-70.75', '0.5')
    ndt = sent['ndt.launch.py']
    assert (ndt['ndt_x'], ndt['ndt_y']) == ('49.25', '-70.75')
    assert ndt['ndt_map'].endswith('campus.pcd') and ndt['ndt_min_score'] == '1.5'
    assert sent['geofence.launch.py']['centre_x'] == '49.25'
    # No chunks made: the whole map.
    assert ndt['ndt_chunks'] == ''


def test_ndt_on_the_campus_takes_its_chunks_when_they_are_made(tmp_path, monkeypatch):
    # TODO 175: scored as good as the whole map (15 Nov), so the default once made.
    monkeypatch.chdir(tmp_path)
    campus_files(tmp_path)
    (tmp_path / 'maps' / 'campus' / 'chunks' / 'ndt').mkdir(parents=True)
    (tmp_path / 'maps' / 'campus' / 'chunks' / 'chunks.yaml').write_text('size: 64\n')
    ndt = included(started(SIM, world='campus', ndt='true'))['ndt.launch.py']
    assert ndt['ndt_chunks'] == os.path.join('maps', 'campus', 'chunks', 'ndt')
    # off: the whole map; another map of the campus: no chunks of it.
    assert included(started(SIM, world='campus', ndt='true', ndt_chunks='off'))[
        'ndt.launch.py']['ndt_chunks'] == ''
    other = tmp_path / 'other.pcd'
    other.write_bytes(b'')
    assert included(started(SIM, world='campus', ndt='true', ndt_map=str(other)))[
        'ndt.launch.py']['ndt_chunks'] == ''


def test_the_campus_start_can_be_moved(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    campus_files(tmp_path)
    ravensim = included(started(SIM, world='campus', start_x='124', start_y='-98'))[
        'ravensim.launch.py']
    assert (ravensim['start_x'], ravensim['start_y']) == ('124', '-98')


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


def test_ndt_leads_only_with_pose_source_ndt(tmp_path):
    # TODO 210, a challenger: off by default (today's GPS-led EKF, the GPS watched as ever).
    sent = included(started(SIM, world='test_lot'))
    assert sent['localization.launch.py']['pose_source'] == 'gps'
    inputs = sent['watchdog.launch.py']['inputs'].split(',')
    assert 'gps' in inputs and 'pose_check' not in inputs
    assert sent['geofence.launch.py']['fix_topic'] == '/localization/gps/odom'
    map_file = tmp_path / 'test_lot.pcd'
    map_file.write_bytes(b'')
    sent = included(started(SIM, world='test_lot', pose_source='ndt', ndt_map=str(map_file)))
    assert 'ndt.launch.py' in sent                      # NDT runs
    assert sent['localization.launch.py']['pose_source'] == 'ndt'
    assert sent['localization.launch.py']['pose_trust_score'] == ''   # the lot: the file's
    # The GPS's checks warn only; pose_check's HOLD stops; the geofence hears either's fix.
    inputs = sent['watchdog.launch.py']['inputs'].split(',')
    assert {'gps_check', 'pose_check'} <= set(inputs) and 'gps' not in inputs
    assert sent['geofence.launch.py']['fix_topic'] == '/localization/pose_check/fix'
    with pytest.raises(ValueError, match='pose_source'):
        started(SIM, world='test_lot', pose_source='lidar')
    with pytest.raises(ValueError, match='truth_pose'):
        started(SIM, world='test_lot', pose_source='ndt', truth_pose='true',
                ndt_map=str(map_file))


def test_ndt_leading_on_the_campus_has_the_campus_trust_score(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    campus_files(tmp_path)
    sent = included(started(SIM, world='campus', pose_source='ndt'))
    assert float(sent['localization.launch.py']['pose_trust_score']) == pytest.approx(
        worlds.CAMPUS_POSE_TRUST)


def test_with_ndt_leading_the_global_ekf_reads_both_through_pose_check():
    assert LOCALIZATION.pose_check_node(context_for(LOCALIZATION)) == []
    assert global_ekf_files(pose_source='ndt') == [
        'ekf_global.yaml', 'ekf_global_ndt.yaml', 'ekf_global_ndt_leads.yaml']
    path = os.path.join(get_package_share_directory('aav_localization'), 'config',
                        'ekf_global_ndt_leads.yaml')
    with open(path) as f:
        p = yaml.safe_load(f)['ekf_global']['ros__parameters']
    assert p == {'pose0': '/localization/pose_check/ndt_pose',
                 'odom1': '/localization/pose_check/gps_odom'}
    context = context_for(LOCALIZATION, pose_source='ndt', pose_trust_score='1.7')
    (check,) = LOCALIZATION.pose_check_node(context)
    assert check._Node__node_executable == 'pose_check'
    values = {}
    for v in evaluate_parameters(context, check._Node__parameters):
        if isinstance(v, dict):
            values.update(v)
    assert values['trust_score'] == 1.7
    with pytest.raises(ValueError, match='pose_source'):
        global_ekf_files(pose_source='both')


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


def test_the_campus_widens_the_fence_by_itself():
    # 200 m held the car at a campus junction 197 m from the start (15 Nov).
    assert SIM.fence_radius('', 'campus') == '600.0'
    assert SIM.fence_radius('', 'road') == '200.0'
    assert SIM.fence_radius(' 350 ', 'campus') == '350'


def test_the_watchdog_watches_only_our_nodes_that_run():
    # TODO 180 (Ben, 14 Nov: warn only): each name a node_* input of sensor_watchdog.yaml.
    ours = SIM.our_nodes(nav=True, costmap='own', planner='own', tracker='mppi_lag',
                         navigator='own', smoother='own', behaviour=True, cubes=True,
                         things=False)
    assert 'node_planner' in ours and 'node_mppi_lag' in ours
    assert 'node_things' not in ours
    trial = SIM.our_nodes(nav=True, costmap='own', planner='diffusion', tracker='lqr',
                          navigator='own', smoother='own', behaviour=False, cubes=False,
                          things=False)
    assert 'node_planner' not in trial and 'node_mppi_lag' not in trial
    # TODO 171 (Ben, 18 Nov: warn only): their status topics too, status_* inputs.
    assert {'status_costmap_local', 'status_costmap_global', 'status_planner',
            'status_navigator', 'status_mppi_lag'} <= set(ours)
    assert 'status_planner' not in trial and 'status_mppi_lag' not in trial
    assert 'status_navigator' in trial
    assert SIM.our_nodes(nav=False, costmap='own', planner='own', tracker='mppi_lag',
                         navigator='own', smoother='own', behaviour=True, cubes=False,
                         things=False) == ['node_ego', 'node_look_ahead']
    config = os.path.join(get_package_share_directory('aav_safety'), 'config',
                          'sensor_watchdog.yaml')
    with open(config) as f:
        rules = yaml.safe_load(f)['sensor_watchdog']['ros__parameters']
    for name in ours:
        kind = 'status' if name.startswith('status_') else 'node'
        assert rules[name]['kind'] == kind and rules[name]['stop'] is False, name


def test_lane_pose_runs_the_lane_finder_and_feeds_the_global_ekf():
    # Ours' pick since 17 Nov (TODO 155), where a camera and a lane map are; the champions'
    # stack (best): the EKF on GPS alone; off without a camera when the stack chose it.
    assert included(started(SIM, world='road'))['localization.launch.py'][
        'lane_pose'] == 'true'
    assert included(started(SIM, world='road', stack='best'))['localization.launch.py'][
        'lane_pose'] == 'false'
    assert included(started(SIM, world='road', cameras='false'))['localization.launch.py'][
        'lane_pose'] == 'false'
    assert included(started(SIM, world='road', pose='ekf'))['localization.launch.py'][
        'lane_pose'] == 'false'
    actions = started(SIM, world='road', lane_pose='true')
    sent = included(actions)
    assert sent['localization.launch.py']['lane_pose'] == 'true'
    # The world's own lane map, though lane_map:= is empty (the behaviour planner's).
    assert sent['localization.launch.py']['lane_map'].endswith('road.lanes.yaml')
    assert sent['lane_finder.launch.py'] == {'lane_finder_impl': 'rust'}
    assert included(started(SIM, world='road', lane_pose='true',
                            lane_finder_impl='python'))['lane_finder.launch.py'] == {
        'lane_finder_impl': 'python'}
    assert 'lane_follower.launch.py' not in sent        # nothing steers by the lane
    with pytest.raises(ValueError, match='lane_pose'):
        started(SIM, world='road', lane_pose='true', cameras='false')
    with pytest.raises(ValueError, match='truth_pose'):
        started(SIM, world='road', lane_pose='true', truth_pose='true')


def global_ekf_dicts(**args):
    """Return the parameters ekf_global is given as values (not files), as one dict."""
    context = context_for(LOCALIZATION, **args)
    (ekf,) = LOCALIZATION.global_ekf(context)
    found = {}
    for p in evaluate_parameters(context, ekf._Node__parameters):
        if isinstance(p, dict):
            found.update(p)
    return found


def test_the_global_ekf_reads_the_lane_pose_only_with_lane_pose():
    assert not any(k.startswith('pose') for k in global_ekf_dicts())
    p = global_ekf_dicts(lane_pose='true', lane_map='/x/lanes.yaml')
    assert p['pose0'] == '/localization/lane/pose'
    # x and y only: the covariance says which way across the lane.
    assert [i for i, on in enumerate(p['pose0_config']) if on] == [0, 1]
    assert 0 < p['pose0_rejection_threshold'] < 10
    assert p['smooth_lagged_data'] is True
    # With NDT's pose too, NDT keeps pose0 and the lane's is pose1.
    both = global_ekf_dicts(lane_pose='true', lane_map='/x/lanes.yaml', ndt_ekf='true')
    assert both['pose1'] == '/localization/lane/pose' and 'pose0' not in both
    (lane,) = LOCALIZATION.lane_pose_node(context_for(LOCALIZATION, lane_pose='true',
                                                      lane_map='/x/lanes.yaml'))
    assert lane._Node__node_executable == 'lane_pose'
    assert LOCALIZATION.lane_pose_node(context_for(LOCALIZATION)) == []
    with pytest.raises(ValueError, match='lane_map'):
        LOCALIZATION.lane_pose_node(context_for(LOCALIZATION, lane_pose='true'))


def test_ours_runs_the_things_so_the_behaviour_can_hold_for_what_moves():
    # Ben, 16 Nov: "if we see a car people or stuff moving around us we should likely just
    # stop": the behaviour's hold reads the things (stacks.yaml's behaviour level).
    assert included(started(SIM, world='road'))['navigation.launch.py']['things'] == 'true'
    assert included(started(SIM, world='road', stack='best'))['navigation.launch.py'][
        'things'] == 'false'
    # The empty world has no lidar: the stack's things are off there, not an error.
    assert included(started(SIM))['navigation.launch.py']['things'] == 'false'


def test_diffusion_impl_picks_the_rust_worker_or_says_what_to_pick():
    # diffusion_impl:=rust: aav_diffusion_rs's worker under the same node (TODO 243).
    sent = included(started(SIM, world='test_lot', planner='diffusion', diffusion_impl='Rust',
                            diffusion_provider='auto'))
    nav = sent['navigation.launch.py']
    assert (nav['diffusion_impl'], nav['diffusion_provider']) == ('rust', 'auto')
    with pytest.raises(ValueError, match='diffusion_impl:=cpp: pick one of python, rust'):
        started(SIM, world='test_lot', planner='diffusion', diffusion_impl='cpp')
