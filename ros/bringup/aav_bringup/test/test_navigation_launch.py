"""
aav_navigation's navigation.launch.py: reversing, sim.launch.py's faster Nav2, the ghost.

Each test runs the launch file's own setup with some arguments, then reads the settings
each Nav2 server would get: the rewritten copy of nav2.yaml, as the server would load it.
Nothing starts: no ROS, no Nav2. The car is forward only; reversing:=true (Gazebo, the
bench) lets Nav2 plan, follow and back up in reverse, at vehicle.yaml's max_reverse_speed.
"""

import importlib.util
import os

from aav_description import vehicle
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
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


NAV = load('aav_navigation', 'navigation.launch.py')
SIM = load('aav_bringup', 'sim.launch.py')
CAR = load('aav_bringup', 'car.launch.py')
BENCH = load('aav_bringup', 'bench.launch.py')
REVERSE = float(vehicle.load()['vehicle']['max_reverse_speed'])


def context_for(launch_file, **args):
    """Return a launch context with the file's defaults, and these arguments on top."""
    context = LaunchContext()
    for declared in launch_file.generate_launch_description().get_launch_arguments():
        context.launch_configurations[declared.name] = perform_substitutions(
            context, declared.default_value)
    context.launch_configurations.update(args)
    return context


def settings(**args):
    """Return {server: its ros__parameters}, from the file each Nav2 server would load."""
    context = context_for(NAV, **{'map': '/maps/empty.yaml', **args})
    found = {}
    for a in NAV.nodes(context):
        if not isinstance(a, Node):
            continue
        name = a._Node__node_name
        for p in a._Node__parameters:
            if isinstance(p, ParameterFile):
                with open(p.evaluate(context)) as f:
                    found[name] = yaml.safe_load(f)[name]['ros__parameters']
    return found


def sent_to_navigation(launch_file, **args):
    """Return the arguments a bringup file gives navigation.launch.py."""
    for a in launch_file.launch_setup(context_for(launch_file, **args)):
        if isinstance(a, IncludeLaunchDescription):
            # Loading the file expands its path.
            a.launch_description_source.get_launch_description(LaunchContext())
            if os.path.basename(a.launch_description_source.location) == 'navigation.launch.py':
                return dict(a.launch_arguments)
    raise AssertionError('navigation.launch.py is not included')


def test_forward_only_by_default():
    s = settings()
    assert s['planner_server']['GridBased']['motion_model_for_search'] == 'DUBIN'
    assert s['controller_server']['FollowPath']['allow_reversing'] is False
    assert s['controller_server']['MPPI']['vx_min'] == 0.0
    assert s['velocity_smoother']['min_velocity'][0] == 0.0
    trees = s['bt_navigator']
    assert trees['default_nav_to_pose_bt_xml'].endswith('navigate_to_pose_car.xml')
    assert trees['default_nav_through_poses_bt_xml'].endswith('navigate_through_poses_car.xml')


def test_reversing_plans_follows_and_backs_up_at_the_cars_reverse_speed():
    s = settings(reversing='true')
    assert s['planner_server']['GridBased']['motion_model_for_search'] == 'REEDS_SHEPP'
    assert s['controller_server']['FollowPath']['allow_reversing'] is True
    assert s['controller_server']['MPPI']['vx_min'] == pytest.approx(-REVERSE)
    assert s['velocity_smoother']['min_velocity'] == pytest.approx([-REVERSE, 0.0, -1.0])
    trees = s['bt_navigator']
    assert trees['default_nav_to_pose_bt_xml'].endswith('navigate_to_pose_car_reversing.xml')
    assert trees['default_nav_through_poses_bt_xml'].endswith(
        'navigate_through_poses_car_reversing.xml')
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        assert os.path.isfile(trees[key]), trees[key]


def test_a_faster_nav2_keeps_reversing_slow():
    s = settings(reversing='true', max_speed='3', controller='rpp')
    assert s['controller_server']['FollowPath']['desired_linear_vel'] == 3.0
    assert s['velocity_smoother']['max_velocity'][0] == 3.0
    assert s['velocity_smoother']['min_velocity'][0] == pytest.approx(-REVERSE)


def test_mppi_unless_regulated_pure_pursuit_is_asked_for():
    rpp = settings(controller='rpp')['controller_server']['FollowPath']
    assert rpp['plugin'].endswith('::RegulatedPurePursuitController')
    assert settings()['controller_server']['FollowPath']['plugin'] == (
        'nav2_mppi_controller::MPPIController')
    s = settings(controller='mppi', reversing='true', max_speed='3')
    follow = s['controller_server']['FollowPath']
    assert follow['plugin'] == 'nav2_mppi_controller::MPPIController'
    # The car's turning circle reaches MPPI's Ackermann model too, through RewrittenYaml.
    v = vehicle.load()['vehicle']
    assert follow['AckermannConstraints']['min_turning_r'] == pytest.approx(
        vehicle.min_turning_radius(v), abs=1e-3)
    assert follow['vx_min'] == pytest.approx(-REVERSE)
    assert follow['vx_max'] == 3.0
    assert follow['enforce_path_inversion'] is True
    assert s['velocity_smoother']['max_velocity'][0] == 3.0
    assert sent_to_navigation(SIM)['controller'] == 'mppi'
    assert sent_to_navigation(SIM, controller='rpp')['controller'] == 'rpp'


def test_who_may_reverse():
    # Gazebo and the bench: yes. The car only with reversing:=true since 26 Oct (the wheels
    # can't tell direction yet, H5), and where the lidar sees behind (test_car_launch.py).
    assert sent_to_navigation(SIM)['reversing'] == 'true'
    assert sent_to_navigation(SIM, reversing='false')['reversing'] == 'false'
    assert sent_to_navigation(SIM, sim_speed='3')['max_speed'] == '3'
    assert sent_to_navigation(BENCH)['reversing'] == 'true'
    assert sent_to_navigation(CAR, machine='all')['reversing'] == 'false'
    assert sent_to_navigation(CAR, machine='all', reversing='true')['reversing'] == 'true'


def whole_file(**args):
    """Return the whole rewritten nav2.yaml the servers load: the costmaps' keys too."""
    context = context_for(NAV, map='/maps/empty.yaml', **args)
    for a in NAV.nodes(context):
        if isinstance(a, Node) and a._Node__node_name == 'controller_server':
            for p in a._Node__parameters:
                if isinstance(p, ParameterFile):
                    with open(p.evaluate(context)) as f:
                        return yaml.safe_load(f)
    raise AssertionError('no controller_server')


def local_costmap(**args):
    return whole_file(**args)['local_costmap']['local_costmap']['ros__parameters']


def ghost_nodes(**args):
    """Return the ghost_obstacle nodes navigation.launch.py starts with these arguments."""
    context = context_for(NAV, map='/maps/empty.yaml', **args)
    return [a for a in NAV.nodes(context)
            if isinstance(a, Node) and a._Node__node_name == 'ghost_obstacle']


def test_no_ghost_unless_asked_for():
    # The radar alone, so collision_monitor runs: and reads no ghost.
    monitor = settings(radar='true')['collision_monitor']
    assert 'ghost' not in monitor['observation_sources']
    assert 'ghost_layer' not in local_costmap()['plugins']
    assert ghost_nodes() == []


def test_the_ghost_adds_its_own_layer_and_source_beside_the_real_sensors():
    s = settings(radar='true', lidar='true', ghost='true')
    assert s['collision_monitor']['observation_sources'] == ['radar', 'lidar', 'ghost']
    assert s['collision_monitor']['ghost']['topic'] == '/safety/ghost/scan'
    local = local_costmap(radar='true', lidar='true', ghost='true')
    assert local['plugins'].index('ghost_layer') < local['plugins'].index('inflation_layer')
    # Its own layer reads only the ghost: its clearing can't touch the radar's or lidar's.
    assert local['ghost_layer']['observation_sources'] == 'ghost'
    assert local['ghost_layer']['ghost']['topic'] == '/safety/ghost/scan'
    assert local['obstacle_layer']['observation_sources'] == 'radar'
    assert local['lidar_layer']['observation_sources'] == 'lidar'
    assert len(ghost_nodes(ghost='true')) == 1


def test_the_ghost_alone_brings_collision_monitor():
    context = context_for(NAV, map='/maps/empty.yaml', ghost='true')
    names = [a._Node__node_name for a in NAV.nodes(context) if isinstance(a, Node)]
    assert 'collision_monitor' in names


def test_who_may_have_a_ghost():
    # Off everywhere unless asked for; the launch argument reaches Nav2.
    assert sent_to_navigation(SIM)['ghost'] == 'false'
    sim = sent_to_navigation(SIM, ghost='true')
    assert (sim['ghost'], sim['collision_monitor']) == ('true', 'true')
    assert sent_to_navigation(CAR, machine='all')['ghost'] == 'false'
    car = sent_to_navigation(CAR, machine='all', ghost='true')
    assert (car['ghost'], car['collision_monitor']) == ('true', 'true')


def bench_lidar_parts(**args):
    """Return (the mock_lidar node's parameters, sensors.launch.py's arguments) from bench."""
    mock, sensors = None, None
    for a in BENCH.launch_setup(context_for(BENCH, **args)):
        if isinstance(a, Node) and a._Node__node_name == 'mock_lidar':
            # Normalised: each name and value is a tuple of substitutions.
            context = LaunchContext()
            mock = {perform_substitutions(context, k): v
                    for k, v in a._Node__parameters[-1].items()}
        elif isinstance(a, IncludeLaunchDescription):
            a.launch_description_source.get_launch_description(LaunchContext())
            if os.path.basename(a.launch_description_source.location) == 'sensors.launch.py':
                sensors = dict(a.launch_arguments)
    return mock, sensors


def test_the_bench_lidar_feeds_the_cars_own_driver_and_nav2():
    assert bench_lidar_parts() == (None, None)
    assert sent_to_navigation(BENCH)['lidar'] == 'false'
    mock, sensors = bench_lidar_parts(lidar='true')
    # mock_lidar sends where the stock driver listens: this computer, not the real 2368.
    assert sensors['lidar'] == 'true' and sensors['lidar_ip'] == '127.0.0.1'
    assert sensors['lidar_port'] == '2369' == str(mock['port'])
    assert sensors['imu'] == sensors['gps'] == 'false'      # the bench's mocks stand there
    assert sent_to_navigation(BENCH, lidar='true')['lidar'] == 'true'


def test_the_camera_guards_the_stop_zone_only_where_asked():
    # camera_obstacles alone: the costmap reads it, collision_monitor doesn't (a bench or
    # Gazebo with no people detector would never move: a silent source stops Nav2).
    costmap_only = whole_file(camera_obstacles='true', radar='true')
    sources = costmap_only['collision_monitor']['ros__parameters']['observation_sources']
    assert 'camera' not in sources
    guarded = whole_file(camera_obstacles='true', camera_guard='true', radar='true')
    assert 'camera' in guarded['collision_monitor']['ros__parameters']['observation_sources']


# --- the trials (docs/trials.md): tracker:= and amcl:= -----------------------------------

def nav_nodes(**args):
    """Return each node navigation.launch.py starts with these arguments, by name."""
    context = context_for(NAV, map='/maps/empty.yaml', **args)
    return {a._Node__node_name: a for a in NAV.nodes(context) if isinstance(a, Node)}


def remaps(node):
    context = LaunchContext()
    return [(perform_substitutions(context, a), perform_substitutions(context, b))
            for a, b in node._Node__remappings]


def given(node):
    """Return the node's parameters given as values (not files), as one dict."""
    found = {}
    for p in evaluate_parameters(LaunchContext(), node._Node__parameters):
        if isinstance(p, dict):
            found.update(p)
    return found


def test_controller_server_follows_unless_a_tracker_is_asked_for():
    nodes = nav_nodes()
    assert 'track' not in nodes
    assert ('follow_path', 'follow_path_unused') not in remaps(nodes['controller_server'])


def test_a_tracker_takes_follow_path_with_nav2s_own_checks():
    nodes = nav_nodes(tracker='lqr', max_speed='3')
    # controller_server keeps running, its action out of bt_navigator's way.
    assert ('follow_path', 'follow_path_unused') in remaps(nodes['controller_server'])
    assert ('cmd_vel', 'cmd_vel_nav') in remaps(nodes['controller_server'])
    p = given(nodes['track'])
    assert p['tracker'] == 'lqr'
    assert p['live_swap'] is False          # only the sim swaps trackers live
    assert given(nav_nodes(tracker='lqr', live_swap='true')['track'])['live_swap'] is True
    assert p['cruise_speed'] == 3.0
    assert p['xy_goal_tolerance'] == 1.0 and p['yaw_goal_tolerance'] == 0.75
    assert p['odom_topic'] == '/localization/odom/local'
    assert p['stamped'] is True
    # Not a lifecycle node: Nav2's manager doesn't wait for it.
    manager = given(nodes['lifecycle_manager_navigation'])
    assert 'track' not in manager['node_names']


def test_bad_trackers_and_reversing_trackers_are_refused():
    with pytest.raises(ValueError, match='pick one of'):
        nav_nodes(tracker='mppi')
    with pytest.raises(ValueError, match='reversing:=false'):
        nav_nodes(tracker='stanley', reversing='true')


def test_amcl_needs_the_lidar_and_never_publishes_map_to_odom():
    with pytest.raises(ValueError, match='lidar:=true'):
        nav_nodes(amcl='true')
    assert 'amcl' not in nav_nodes(lidar='true')
    nodes = nav_nodes(lidar='true', amcl='true', amcl_y='2.5', amcl_yaw='0.3')
    amcl = nodes['amcl']
    assert ('amcl_pose', '/localization/amcl/pose') in remaps(amcl)
    start = given(amcl)
    assert start['set_initial_pose'] is True
    assert (start['initial_pose.x'], start['initial_pose.y'], start['initial_pose.yaw']) == (
        0.0, 2.5, 0.3)
    config = os.path.join(get_package_share_directory('aav_navigation'), 'config', 'amcl.yaml')
    with open(config) as f:
        file = yaml.safe_load(f)['amcl']['ros__parameters']
    assert file['tf_broadcast'] is False
    assert file['scan_topic'] == '/sensors/lidar/scan' and file['base_frame_id'] == 'base_link'
    # Its own manager: Nav2's lists only its servers.
    assert list(given(nodes['lifecycle_manager_localization'])['node_names']) == ['amcl']
    assert 'amcl' not in given(nodes['lifecycle_manager_navigation'])['node_names']
    assert 'amcl_map_server' not in nodes and 'map_topic' not in start   # Nav2's /map
    # What the launch file sets must not be in the file too: under sim.launch.py's global
    # use_sim_time (SetParameter) the file's value wins, and AMCL read Nav2's /map.
    own = given(nav_nodes(lidar='true', amcl='true', amcl_map='/maps/lot_amcl.yaml')['amcl'])
    assert not set(own) & set(file)


def test_amcl_may_have_a_map_of_its_own():
    nodes = nav_nodes(lidar='true', amcl='true', amcl_map='/maps/lot_amcl.yaml')
    served = given(nodes['amcl_map_server'])
    assert served['yaml_filename'] == '/maps/lot_amcl.yaml'
    assert served['topic_name'] == given(nodes['amcl'])['map_topic'] == '/localization/amcl/map'
    # Nav2's map_server keeps /map; AMCL's manager brings its map up first.
    assert list(given(nodes['lifecycle_manager_localization'])['node_names']) == [
        'amcl_map_server', 'amcl']


def test_the_sim_passes_the_trials_on_and_checks_them():
    assert sent_to_navigation(SIM)['tracker'] == 'none'
    assert sent_to_navigation(SIM)['live_swap'] == 'true'
    assert sent_to_navigation(SIM, tracker='mpc', reversing='false')['tracker'] == 'mpc'
    with pytest.raises(ValueError, match='reversing:=false'):
        sent_to_navigation(SIM, tracker='mpc')          # the sim reverses by default
    with pytest.raises(ValueError, match='world:=test_lot'):
        sent_to_navigation(SIM, amcl='true')            # the empty world: no real map
    with pytest.raises(ValueError, match='world:=test_lot'):
        sent_to_navigation(SIM, world='test_lot', lidar='false', amcl='true')
    sent = sent_to_navigation(SIM, world='test_lot', amcl='true', start_y='4.0',
                              start_yaw='1.0')
    assert (sent['amcl'], sent['lidar']) == ('true', 'true')
    assert (sent['amcl_y'], sent['amcl_yaw']) == ('4.0', '1.0')
    assert sent['amcl_map'].endswith('maps/example_lot_amcl.yaml')
    assert os.path.exists(sent['amcl_map'])
    assert sent_to_navigation(SIM, world='test_lot')['amcl_map'] == ''


# planner:= (docs/controllers.md, Planners): each planner behind the one plugin name the
# behaviour trees ask for. Lattice reads the map's file, so these tests give a real one.
EMPTY_MAP = os.path.join(get_package_share_directory('aav_navigation'), 'maps', 'empty.yaml')
PLUGINS = {'hybrid': 'nav2_smac_planner::SmacPlannerHybrid',
           'lattice': 'nav2_smac_planner::SmacPlannerLattice',
           'smac2d': 'nav2_smac_planner::SmacPlanner2D',
           'navfn': 'nav2_navfn_planner::NavfnPlanner',
           'theta_star': 'nav2_theta_star_planner::ThetaStarPlanner'}


def planner_settings(**args):
    return settings(**{'map': EMPTY_MAP, **args})['planner_server']


@pytest.mark.parametrize('planner', vehicle.PLANNERS)
def test_each_planner_runs_as_gridbased_alone(planner):
    s = planner_settings(planner=planner)
    assert s['planner_plugins'] == ['GridBased']
    assert s['GridBased']['plugin'] == PLUGINS[planner]
    # The spare blocks go: the server reads only GridBased.
    assert not set(s) & {'Lattice', 'Smac2D', 'NavFn', 'ThetaStar'}
    # Only Hybrid has a search model: RewrittenYaml would add the key to any other.
    assert ('motion_model_for_search' in s['GridBased']) == (planner == 'hybrid')


def test_hybrid_is_the_default_with_the_cars_planning_radius():
    s = planner_settings()
    v = vehicle.load()['vehicle']
    assert s['GridBased']['plugin'] == PLUGINS['hybrid']
    assert s['GridBased']['minimum_turning_radius'] == pytest.approx(
        vehicle.plan_turning_radius(v), abs=1e-3)
    assert planner_settings(reversing='true')['GridBased']['motion_model_for_search'] == (
        'REEDS_SHEPP')


def test_lattice_uses_our_primitives_forward_only_unless_reversing():
    forward = planner_settings(planner='lattice')['GridBased']
    path = forward['lattice_filepath']
    assert path.endswith(os.path.join('aav_navigation', 'config', 'lattice_ackermann.json'))
    assert os.path.isfile(path)
    assert forward['allow_reverse_expansion'] is False
    assert planner_settings(planner='lattice',
                            reversing='true')['GridBased']['allow_reverse_expansion'] is True


def test_lattice_is_refused_on_a_map_with_other_cells(tmp_path):
    fine = tmp_path / 'fine.yaml'
    fine.write_text('image: fine.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n')
    with pytest.raises(ValueError, match='lattice-primitives'):
        settings(map=str(fine), planner='lattice')


def test_bad_planners_are_refused():
    with pytest.raises(ValueError, match='pick one of'):
        planner_settings(planner='rrt')


def test_the_sim_passes_any_planner_and_the_car_only_car_shaped_ones():
    assert sent_to_navigation(SIM)['planner'] == 'hybrid'
    for planner in vehicle.PLANNERS:
        assert sent_to_navigation(SIM, planner=planner)['planner'] == planner
    assert sent_to_navigation(CAR, machine='all')['planner'] == 'hybrid'
    assert sent_to_navigation(CAR, machine='all', planner='lattice')['planner'] == 'lattice'
    for planner in set(vehicle.PLANNERS) - set(vehicle.CAR_PLANNERS):
        with pytest.raises(ValueError, match='turning radius'):
            sent_to_navigation(CAR, machine='all', planner=planner)
