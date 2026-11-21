"""
aav_navigation's navigation.launch.py: our nodes (Nav2 deleted, 17 Nov), reversing, speed.

Each test runs the launch file's own setup with some arguments, then reads what each node
would get: the rewritten copy of nav2.yaml (the blocks our ports read, in Nav2's layout),
or a node's own parameters. Nothing starts: no ROS. The car is forward only; reversing:=true
(the simulator, the bench) lets it plan, follow and back up in reverse, at vehicle.yaml's
max_reverse_speed.
"""

import importlib.util
import json
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
# The installed empty map: the lattice (the default planner) reads its cells.
EMPTY_MAP = os.path.join(get_package_share_directory('aav_navigation'), 'maps', 'empty.yaml')
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
    """Return {block: its ros__parameters}: the rewritten nav2.yaml the nodes read."""
    return {name: block['ros__parameters'] for name, block in whole_file(**args).items()
            if 'ros__parameters' in block}


def whole_file(**args):
    """Return the whole rewritten nav2.yaml, as the smoother (and lidar_scan) load it."""
    context = context_for(NAV, **{'map': EMPTY_MAP, **args})
    for a in NAV.nodes(context):
        if isinstance(a, Node) and a._Node__node_name == 'velocity_smoother':
            for p in a._Node__parameters:
                if isinstance(p, ParameterFile):
                    with open(p.evaluate(context)) as f:
                        return yaml.safe_load(f)
    raise AssertionError('no velocity_smoother')


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
    assert s['planner_server']['GridBased']['allow_reverse_expansion'] is False
    assert s['controller_server']['FollowPath']['allow_reversing'] is False
    assert s['controller_server']['MPPI']['vx_min'] == 0.0
    assert s['velocity_smoother']['min_velocity'][0] == 0.0
    assert json.loads(given(nav_nodes()['navigator'])['settings'])['backup'] is None


def test_reversing_plans_follows_and_backs_up_at_the_cars_reverse_speed():
    s = settings(reversing='true')
    assert s['planner_server']['GridBased']['allow_reverse_expansion'] is True
    assert s['controller_server']['FollowPath']['allow_reversing'] is True
    assert s['controller_server']['MPPI']['vx_min'] == pytest.approx(-REVERSE)
    assert s['velocity_smoother']['min_velocity'] == pytest.approx([-REVERSE, 0.0, -1.0])
    backup = json.loads(given(nav_nodes(reversing='true')['navigator'])['settings'])['backup']
    assert backup['speed'] == pytest.approx(REVERSE / 2)


def test_a_faster_follower_keeps_reversing_slow():
    s = settings(reversing='true', max_speed='3')
    assert s['controller_server']['FollowPath']['vx_max'] == 3.0
    assert s['velocity_smoother']['max_velocity'][0] == 3.0
    assert s['velocity_smoother']['min_velocity'][0] == pytest.approx(-REVERSE)


def test_mppi_lag_reads_the_mppi_block():
    s = settings(reversing='true', max_speed='3')
    follow = s['controller_server']['FollowPath']
    assert follow['plugin'] == 'nav2_mppi_controller::MPPIController'
    # The car's turning circle reaches the MPPI block too, through vehicle.rewrite.
    v = vehicle.load()['vehicle']
    assert follow['AckermannConstraints']['min_turning_r'] == pytest.approx(
        vehicle.min_turning_radius(v), abs=1e-3)
    assert follow['vx_min'] == pytest.approx(-REVERSE)
    assert follow['vx_max'] == 3.0
    assert follow['enforce_path_inversion'] is True
    assert sent_to_navigation(SIM)['controller'] == 'mppi'
    with pytest.raises(ValueError, match='controller:=rpp'):
        sent_to_navigation(SIM, controller='rpp')        # Regulated Pure Pursuit went


def test_who_may_reverse():
    # Gazebo and the bench: yes. The car too since 1 Nov (Ben: "reverse and uturn should be
    # in"), where the lidar sees behind (test_car_launch.py).
    assert sent_to_navigation(SIM)['reversing'] == 'true'
    assert sent_to_navigation(SIM, reversing='false')['reversing'] == 'false'
    assert sent_to_navigation(SIM, sim_speed='3')['max_speed'] == '3'
    assert sent_to_navigation(BENCH)['reversing'] == 'true'
    assert sent_to_navigation(CAR, machine='all')['reversing'] == 'true'
    assert sent_to_navigation(CAR, machine='all', reversing='false')['reversing'] == 'false'


def local_costmap(**args):
    return whole_file(**args)['local_costmap']['local_costmap']['ros__parameters']


def ghost_nodes(**args):
    """Return the ghost_obstacle nodes navigation.launch.py starts with these arguments."""
    context = context_for(NAV, map=EMPTY_MAP, **args)
    return [a for a in NAV.nodes(context)
            if isinstance(a, Node) and a._Node__node_name == 'ghost_obstacle']


def test_no_ghost_unless_asked_for():
    # The radar alone, so the stop zone runs: and reads no ghost.
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


def test_the_ghost_alone_brings_the_stop_zone():
    context = context_for(NAV, map=EMPTY_MAP, ghost='true')
    names = [a._Node__node_name for a in NAV.nodes(context) if isinstance(a, Node)]
    assert 'stop_zone' in names and 'collision_monitor' not in names


def test_who_may_have_a_ghost():
    # Off everywhere unless asked for; the launch argument reaches Nav2.
    assert sent_to_navigation(SIM)['ghost'] == 'false'
    sim = sent_to_navigation(SIM, ghost='true')
    assert (sim['ghost'], sim['stop_zone']) == ('true', 'true')
    assert sent_to_navigation(CAR, machine='all')['ghost'] == 'false'
    car = sent_to_navigation(CAR, machine='all', ghost='true')
    assert (car['ghost'], car['stop_zone']) == ('true', 'true')


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
    # camera_obstacles alone: the costmap reads it, the stop zone doesn't (a bench or a
    # simulator with no people detector would never move: a silent source stops the car).
    costmap_only = whole_file(camera_obstacles='true', radar='true')
    sources = costmap_only['collision_monitor']['ros__parameters']['observation_sources']
    assert 'camera' not in sources
    guarded = whole_file(camera_obstacles='true', camera_guard='true', radar='true')
    assert 'camera' in guarded['collision_monitor']['ros__parameters']['observation_sources']


# --- the trials (docs/trials.md): tracker:= and amcl:= -----------------------------------

def nav_nodes(**args):
    """Return each node navigation.launch.py starts with these arguments, by name."""
    context = context_for(NAV, **{'map': EMPTY_MAP, **args})
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


def test_mppi_lag_follows_unless_another_tracker_is_asked_for():
    nodes = nav_nodes()
    assert 'mppi_lag' in nodes and 'track' not in nodes
    assert given(nodes['mppi_lag'])['costmap_topic'] == '/drivers/nav/costmap/local_raw'
    for nav2 in ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server',
                 'waypoint_follower', 'collision_monitor', 'lifecycle_manager_navigation'):
        assert nav2 not in nodes, nav2


def test_a_tracker_takes_follow_path_with_nav2s_own_checks():
    nodes = nav_nodes(tracker='lqr', max_speed='3')
    assert 'mppi_lag' not in nodes
    p = given(nodes['track'])
    assert p['tracker'] == 'lqr'
    assert p['live_swap'] is False          # only the sim swaps trackers live
    assert given(nav_nodes(tracker='lqr', live_swap='true')['track'])['live_swap'] is True
    assert p['cruise_speed'] == 3.0
    assert p['xy_goal_tolerance'] == 1.0 and p['yaw_goal_tolerance'] == 0.75
    assert p['odom_topic'] == '/localization/odom/local'
    assert p['stamped'] is True


def test_diffusion_takes_compute_path_to_pose_with_our_planner_behind_it():
    assert 'diffusion_planner' not in nav_nodes()
    nodes = nav_nodes(planner='diffusion', reversing='true')
    # Our planner keeps planning beside it, the fallback; the navigator asks the model.
    assert 'planner' in nodes
    assert ('compute_path_to_pose_lattice', vehicle.OWN_PLANNER_ACTION) in remaps(
        nodes['diffusion_planner'])
    assert len(remaps(nodes['diffusion_planner'])) == 1
    assert given(nodes['navigator'])['planner_action'] == '/compute_path_to_pose'
    assert given(nav_nodes()['navigator'])['planner_action'] == vehicle.OWN_PLANNER_ACTION
    # Ours plans on its own topic then: /plan is the model's.
    assert remaps(nodes['planner']) == [('/plan', '/drivers/nav/diffusion/lattice_plan')]
    assert remaps(nav_nodes()['planner']) == []
    p = given(nodes['diffusion_planner'])
    v = vehicle.load()['vehicle']
    assert p['reversing'] is True
    assert p['min_radius'] == pytest.approx(0.95 * vehicle.min_turning_radius(v))
    assert p['goal_xy'] == 1.0 and p['goal_yaw'] == 0.75
    assert p['model'] == '' and p['provider'] == 'CPUExecutionProvider'
    # The outline, padded as the global costmap pads it (0.05 m).
    front = yaml.safe_load(p['footprint'])[0]
    assert front[0] == pytest.approx(vehicle.footprint_points(v)[0][0] + 0.05, abs=1e-3)


def test_bad_trackers_and_reversing_trackers_are_refused():
    with pytest.raises(ValueError, match='pick one of'):
        nav_nodes(tracker='none')                       # Nav2's controller_server went
    # Every tracker reverses now: mpc_lag_rs too since 20 Nov (TODO 213).
    rs = given(nav_nodes(tracker='mpc_lag_rs', reversing='true')['mpc_lag_rs'])
    assert rs['max_reverse_speed'] > 0
    # aav_track's reverse since 19 Nov (TODO 174), at vehicle.yaml's reverse speed.
    track = given(nav_nodes(tracker='stanley', reversing='true')['track'])
    assert track['max_reverse_speed'] > 0
    assert given(nav_nodes(tracker='stanley')['track'])['max_reverse_speed'] == 0.0


def test_map_correction_newest_turns_the_followers_slew_off():
    # TODO 186: a switch of its own (a rate set here would lose to the node's own file).
    for tracker, name in (('mppi_lag', 'mppi_lag'), ('lqr', 'track')):
        p = given(nav_nodes(tracker=tracker)[name])
        assert p['map_correction'] is False and 'map_correction_speed' not in p
        p = given(nav_nodes(tracker=tracker, map_correction='slow')[name])
        assert p['map_correction'] is True
    with pytest.raises(ValueError, match='slow or newest'):
        nav_nodes(tracker='lqr', map_correction='fast')
    # The sim's stack picks it (15 Nov, TODO 168): ours slow, best newest; by hand wins.
    assert sent_to_navigation(SIM)['map_correction'] == 'slow'
    assert sent_to_navigation(SIM, stack='best')['map_correction'] == 'newest'
    assert sent_to_navigation(SIM, map_correction='newest')['map_correction'] == 'newest'
    assert sent_to_navigation(SIM, stack='best',
                              map_correction='slow')['map_correction'] == 'slow'


def test_every_model_of_the_car_gets_the_speed_loop_asked_for():
    # TODO 163: the follower and the behaviour planner predict the loop the car runs.
    for tracker, name, key in (('mppi_lag', 'mppi_lag', 'speed_loop'),
                               ('mpc_lag', 'track', 'mpc_lag.speed_loop'),
                               ('mpc_lag_rs', 'mpc_lag_rs', 'mpc_lag.speed_loop')):
        assert given(nav_nodes(tracker=tracker)[name])[key] == 'default', tracker
        firm = nav_nodes(tracker=tracker, speed_loop='firm', behaviour='true')
        assert given(firm[name])[key] == 'firm', tracker
        assert given(firm['behaviour'])['speed_loop'] == 'firm'
    # mppi_lag runs the LQI on the shared model; the two MPCs can't, so the launch says so.
    assert given(nav_nodes(tracker='mppi_lag', speed_loop='lqi')['mppi_lag'])[
        'speed_loop'] == 'lqi'
    for tracker in ('mpc_lag', 'mpc_lag_rs'):
        with pytest.raises(ValueError, match='PI'):
            nav_nodes(tracker=tracker, speed_loop='lqi')
    with pytest.raises(ValueError, match='speed_loop:=fast'):
        nav_nodes(tracker='mppi_lag', speed_loop='fast')


def test_the_sim_and_the_bench_pass_their_speed_loop_on():
    assert sent_to_navigation(SIM)['speed_loop'] == 'default'
    assert sent_to_navigation(SIM, speed_loop='firm')['speed_loop'] == 'firm'
    assert sent_to_navigation(BENCH, nav='true', speed_loop='smooth')['speed_loop'] == 'smooth'
    with pytest.raises(ValueError, match='speed_loop:=fast'):
        sent_to_navigation(SIM, speed_loop='fast')
    # speed_loop_model:= (TODO 163): navigation's models take it, not the car's loop.
    assert sent_to_navigation(SIM, speed_loop='firm',
                              speed_loop_model='smooth')['speed_loop'] == 'smooth'
    assert sent_to_navigation(BENCH, nav='true', speed_loop='firm',
                              speed_loop_model='default')['speed_loop'] == 'default'


def test_patchwork_ground_runs_the_rust_voxels_node_with_it():
    # TODO 215: ground:=patchwork, the Rust voxels node with ground patchwork; none: plane.
    assert 'voxels' not in nav_nodes(lidar='true')
    nodes = nav_nodes(lidar='true', ground='patchwork', voxels_impl='python')
    assert nodes['voxels']._Node__package == 'aav_voxels_rs'
    assert given(nodes['voxels'])['ground'] == 'patchwork'
    with pytest.raises(ValueError, match='ground'):
        nav_nodes(lidar='true', ground='ransac')


def test_the_per_kind_tracker_reaches_the_things_node():
    # TODO 217: things_tracker:=per_kind; none: the constant-velocity champion.
    assert given(nav_nodes(lidar='true', things='true')['things'])['per_kind'] is False
    p = given(nav_nodes(lidar='true', things='true', things_tracker='per_kind')['things'])
    assert p['per_kind'] is True
    with pytest.raises(ValueError, match='things_tracker'):
        nav_nodes(lidar='true', things='true', things_tracker='imm')


def test_smooth_sampling_reaches_mppi_lag():
    # TODO 212: mppi_lag_sampling:=smooth, a challenger; none: the node's own (correlated).
    assert 'sampling' not in given(nav_nodes(tracker='mppi_lag')['mppi_lag'])
    # The tube (Ben, 21 Nov): mpc_lag_rs leaves the car alone near the path.
    # Tuned 21 Nov: 0.3 m unless asked; 0 turns it off.
    assert given(nav_nodes(tracker='mpc_lag_rs')['mpc_lag_rs'])['mpc_lag.tube'] == 0.3
    tube = given(nav_nodes(tracker='mpc_lag_rs', mpc_lag_tube='0')['mpc_lag_rs'])
    assert tube['mpc_lag.tube'] == 0.0
    with pytest.raises(ValueError, match='mpc_lag_tube'):
        nav_nodes(tracker='mpc_lag_rs', mpc_lag_tube='wide')
    p = given(nav_nodes(tracker='mppi_lag', mppi_lag_sampling='smooth')['mppi_lag'])
    assert p['sampling'] == 'smooth'
    with pytest.raises(ValueError, match='mppi_lag_sampling'):
        nav_nodes(tracker='mppi_lag', mppi_lag_sampling='jerky')


def test_the_steering_gamma_and_the_strong_smoother_reach_their_nodes():
    # TODO 168: mppi_lag_steer_gamma:= gives mppi_lag its steer_gamma (none: gamma's);
    # plan_smoothing:=strong gives our planner the smoother's weights.
    assert 'steer_gamma' not in given(nav_nodes(tracker='mppi_lag')['mppi_lag'])
    p = given(nav_nodes(tracker='mppi_lag', mppi_lag_steer_gamma='0.05')['mppi_lag'])
    assert p['steer_gamma'] == pytest.approx(0.05)
    with pytest.raises(ValueError, match='mppi_lag_steer_gamma'):
        nav_nodes(tracker='mppi_lag', mppi_lag_steer_gamma='-1')
    s = json.loads(given(nav_nodes(planner='own')['planner'])['settings'])
    assert 'smoother' not in s
    s = json.loads(given(nav_nodes(planner='own', plan_smoothing='strong')['planner'])[
        'settings'])
    assert s['smoother'] == vehicle.PLAN_SMOOTHINGS['strong']
    with pytest.raises(ValueError, match='plan_smoothing'):
        nav_nodes(planner='own', plan_smoothing='soft')


def test_the_behaviour_planner_slows_for_the_kerbs_with_kerbs_on():
    # TODO 219: kerbs:=true (with the lidar) gives the behaviour planner the steps.
    assert given(nav_nodes(behaviour='true', lidar='true')['behaviour'])['kerbs'] is False
    on = given(nav_nodes(behaviour='true', lidar='true', kerbs='true')['behaviour'])
    assert on['kerbs'] is True
    # No lidar, no steps.
    assert given(nav_nodes(behaviour='true', kerbs='true')['behaviour'])['kerbs'] is False


def test_mppi_lag_batch_reaches_mppi_lag():
    # TODO 152: mppi_lag_batch:=1000 gives mppi_lag its batch_size; none: nav2.yaml's.
    assert given(nav_nodes(tracker='mppi_lag')['mppi_lag'])['batch_size'] == 2000
    p = given(nav_nodes(tracker='mppi_lag', mppi_lag_batch='1000')['mppi_lag'])
    assert p['batch_size'] == 1000
    with pytest.raises(ValueError, match='mppi_lag_batch'):
        nav_nodes(tracker='mppi_lag', mppi_lag_batch='0')


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
    # Ours by default (TODO 231): a plain node, so no lifecycle manager at all.
    assert amcl._Node__package == 'aav_amcl_rs'
    assert 'lifecycle_manager_localization' not in nodes
    assert 'lifecycle_manager_navigation' not in nodes
    assert 'amcl_map_server' not in nodes and 'map_topic' not in start   # our /map
    # What the launch file sets must not be in the file too: under sim.launch.py's global
    # use_sim_time (SetParameter) the file's value wins, and AMCL read Nav2's /map.
    own = given(nav_nodes(lidar='true', amcl='true', amcl_map='/maps/lot_amcl.yaml')['amcl'])
    assert not set(own) & set(file)


def test_amcl_may_have_a_map_of_its_own():
    nodes = nav_nodes(lidar='true', amcl='true', amcl_map='/maps/lot_amcl.yaml')
    served = given(nodes['amcl_map_server'])
    assert served['yaml_filename'] == '/maps/lot_amcl.yaml'
    assert served['topic_name'] == given(nodes['amcl'])['map_topic'] == '/localization/amcl/map'
    # Our map server keeps /map; AMCL's is ours too, not a lifecycle node.
    assert nodes['amcl_map_server']._Node__package == 'aav_costmap'


def test_amcl_impl_is_ours_only():
    # Stock nav2_amcl, the last of Nav2, went on 20 Nov (TODO 232, tag save/nav2-amcl).
    for impl in ('stock', 'python'):
        with pytest.raises(ValueError, match='amcl_impl'):
            nav_nodes(lidar='true', amcl='true', amcl_impl=impl)


def test_the_sim_passes_the_trials_on_and_checks_them():
    # Ours by default (stack:=ours): mppi_lag, the champion too (stack:=best).
    assert sent_to_navigation(SIM)['tracker'] == 'mppi_lag'
    assert sent_to_navigation(SIM, stack='best')['tracker'] == 'mppi_lag'
    assert sent_to_navigation(SIM)['live_swap'] == 'true'
    assert sent_to_navigation(SIM, tracker='mpc', reversing='false')['tracker'] == 'mpc'
    # The sim reverses by default; aav_track's trackers do too since 19 Nov (TODO 174).
    assert sent_to_navigation(SIM, tracker='mpc')['tracker'] == 'mpc'
    # mpc_lag_rs reverses too since 20 Nov (TODO 213).
    assert sent_to_navigation(SIM, tracker='mpc_lag_rs')['tracker'] == 'mpc_lag_rs'
    with pytest.raises(ValueError, match='world:=test_lot'):
        sent_to_navigation(SIM, amcl='true')            # the empty world: no real map
    with pytest.raises(ValueError, match='world:=test_lot'):
        sent_to_navigation(SIM, world='test_lot', lidar='false', amcl='true')
    sent = sent_to_navigation(SIM, world='test_lot', amcl='true', start_y='4.0',
                              start_yaw='1.0')
    assert (sent['amcl'], sent['lidar']) == ('true', 'true')
    assert (sent['amcl_y'], sent['amcl_yaw']) == ('4.0', '1.0')
    assert sent['amcl_impl'] == 'rust'
    assert sent['amcl_map'].endswith('maps/example_lot_amcl.yaml')
    assert os.path.exists(sent['amcl_map'])
    assert sent_to_navigation(SIM, world='test_lot')['amcl_map'] == ''


# Our planner (aav_planner): Nav2's lattice ported. Lattice reads the map's file, so these
# tests give a real one.
def test_our_planner_plans_for_the_navigator_on_our_global_costmap():
    nodes = nav_nodes(reversing='true')
    assert given(nodes['navigator'])['planner_action'] == vehicle.OWN_PLANNER_ACTION
    p = given(nodes['planner'])
    s = json.loads(p['settings'])
    assert s['allow_reverse_expansion'] is True
    assert s['lattice_filepath'].endswith('lattice_ackermann.json')
    assert p['costmap_topic'] == '/drivers/nav/costmap/global_raw'
    assert p['costmap_updates_topic'] == ''
    assert p['is_path_valid_service'] == '/is_path_valid'


def test_smooth_moves_reach_our_planner_with_its_smoother_off():
    # lattice_moves:= (tools/lattice_smooth.py): another file, the same for Nav2's and ours.
    s = json.loads(given(nav_nodes(planner='own', lattice_moves='smooth')['planner'])[
        'settings'])
    assert s['lattice_filepath'].endswith('lattice_ackermann_smooth.json')
    assert os.path.exists(s['lattice_filepath'])
    assert s['smooth_path'] is False
    s = json.loads(given(nav_nodes(planner='own')['planner'])['settings'])
    assert s['smooth_path'] is True
    with pytest.raises(ValueError, match='16 headings only'):
        nav_nodes(planner='own', lattice_moves='smooth', lattice_headings='32')


def test_planner_search_hybrid_gives_our_planner_the_hybrid_a_star_block():
    # planner_search:=hybrid (TODO 178): ours runs Hybrid-A* on nav2.yaml's GridBased block.
    for reversing, model in (('false', 'DUBIN'), ('true', 'REEDS_SHEPP')):
        s = json.loads(given(nav_nodes(planner='own', planner_search='hybrid',
                                       reversing=reversing)['planner'])['settings'])
        assert s['search'] == 'hybrid'
        assert s['motion_model_for_search'] == model
        assert s['angle_quantization_bins'] == 72
        assert s['non_straight_penalty'] == 1.2 and s['change_penalty'] == 0.0
        assert 'lattice_filepath' not in s
    # The default stays the lattice; another planner can't take the switch.
    s = json.loads(given(nav_nodes(planner='own')['planner'])['settings'])
    assert 'search' not in s
    with pytest.raises(ValueError, match='pick one of'):
        nav_nodes(planner='lattice')                     # Nav2's lattice went
    with pytest.raises(ValueError, match='pick one of'):
        nav_nodes(planner='own', planner_search='rrt')


def test_our_navigator_serves_nav2s_action_names():
    nodes = nav_nodes()
    assert 'bt_navigator' not in nodes and 'waypoint_follower' not in nodes
    p = given(nodes['navigator'])
    assert p['planner_action'] == vehicle.OWN_PLANNER_ACTION
    assert p['planner_through_action'] == vehicle.OWN_PLANNER_THROUGH_ACTION
    assert p['follow_action'] == '/follow_path'
    assert p['clear_local'] == '/drivers/nav/costmap/clear_entirely_local'
    assert p['clear_global'] == '/drivers/nav/costmap/clear_entirely_global'
    assert p['backup_action'] == ''
    assert p['navsat_file'].endswith('aav_localization/config/navsat.yaml')
    s = json.loads(p['settings'])
    assert (s['backup'], s['retries'], s['waypoints']) == (None, 6, 'pass_through')
    assert s['replan_period'] == pytest.approx(1.0)
    once = json.loads(given(nav_nodes(replan_hz='0')['navigator'])['settings'])
    assert once['replan_period'] == pytest.approx(1.0 / vehicle.REPLAN_ONCE_HZ)
    with pytest.raises(ValueError, match='navigator:='):
        nav_nodes(navigator='nav2')
    # Nav2's names: no goal sender knows the navigator changed.
    assert vehicle.navigator_action('follow_gps_waypoints') == '/follow_gps_waypoints'
    assert vehicle.navigator_action('navigate_to_pose') == '/navigate_to_pose'


def test_the_map_is_ours_latched_from_the_map_file():
    node = nav_nodes()['map_server']
    assert (node._Node__package, node._Node__node_executable) == ('aav_costmap', 'map_server')
    p = given(node)
    assert (p['yaml_filename'], p['topic_name'], p['frame_id']) == (EMPTY_MAP, '/map', 'map')


def test_our_smoother_runs_under_velocity_smoothers_name():
    ours = nav_nodes()['velocity_smoother']
    assert ours._Node__package == 'aav_smoother'
    assert ours._Node__node_executable == 'car_smoother'
    assert remaps(ours) == [('cmd_vel', 'cmd_vel_nav'),
                            ('cmd_vel_smoothed', '/drivers/nav/cmd_vel')]
    p = given(ours)
    cfg = vehicle.load()
    assert p['vehicle_yaml'] == vehicle.config_path()
    assert p['steering_rate'] <= float(cfg['actuators']['steer_rate'])
    assert p['min_speed_for_steering'] > 0.0
    assert any(isinstance(x, ParameterFile) for x in ours._Node__parameters)
    with pytest.raises(ValueError, match='smoother:='):
        nav_nodes(smoother='nav2')
    assert sent_to_navigation(SIM)['smoother'] == 'own'
    assert sent_to_navigation(CAR, machine='all')['smoother'] == 'own'


def test_the_stop_zone_sits_between_the_smoother_and_twist_mux():
    nodes = nav_nodes(radar='true', lidar='true')
    zone = nodes['stop_zone']
    assert zone._Node__package == 'aav_stop_zone'
    p = given(zone)
    assert (p['cmd_in'], p['cmd_out']) == ('/control/cmd_vel_smoothed', '/drivers/nav/cmd_vel')
    assert (p['state'], p['status'], p['box']) == (
        '/safety/stop_zone/state', '/safety/stop_zone/status', '/safety/stop_zone/box')
    assert remaps(nodes['velocity_smoother'])[1] == (
        'cmd_vel_smoothed', '/control/cmd_vel_smoothed')
    s = json.loads(p['settings'])
    assert [x['name'] for x in s['sources']] == ['radar', 'lidar']
    assert s['min_points'] == 3
    # The stop hold keeps its stops (twist_mux's stop_zone lock).
    assert 'stop_hold' in nodes
    # No sensor, no zone: the smoother sends straight to twist_mux.
    assert 'stop_zone' not in nav_nodes()
    assert nav_nodes(stop_zone='false', radar='true').get('stop_zone') is None


def test_the_car_runs_ours():
    sent = sent_to_navigation(CAR, machine='all')
    assert (sent['navigator'], sent['costmap'], sent['smoother'], sent['planner']) == (
        'own', 'own', 'own', 'own')
    assert sent['stop_zone'] == 'true'
    assert sent_to_navigation(CAR, machine='all', reversing='false')['planner'] == 'own'


def test_bad_planners_are_refused():
    with pytest.raises(ValueError, match='pick one of'):
        nav_nodes(planner='rrt')


def test_the_sim_passes_its_planner_and_the_car_only_ours():
    assert sent_to_navigation(SIM)['planner'] == 'own'            # ours, the default
    for planner in vehicle.PLANNERS:
        assert sent_to_navigation(SIM, planner=planner)['planner'] == planner
    for planner in set(vehicle.PLANNERS) - set(vehicle.CAR_PLANNERS):
        with pytest.raises(ValueError, match='turning radius|diffusion_trial'):
            sent_to_navigation(CAR, machine='all', planner=planner)


def test_lattice_is_refused_on_a_map_with_other_cells(tmp_path):
    fine = tmp_path / 'fine.yaml'
    fine.write_text('image: fine.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n')
    with pytest.raises(ValueError, match='python3 tools/lattice_primitives.py'):
        nav_nodes(map=str(fine))


def test_the_car_refuses_diffusion_without_the_trial_switch():
    with pytest.raises(ValueError, match='diffusion_trial:=true'):
        sent_to_navigation(CAR, machine='all', planner='diffusion')
    with pytest.raises(ValueError, match='diffusion_trial:=true'):
        sent_to_navigation(CAR, machine='all', planner='diffusion', diffusion_trial='false')
    # The switch is for the diffusion planner only: no other planner rides on it.
    for planner in set(vehicle.PLANNERS) - {'diffusion'}:
        with pytest.raises(ValueError, match='planner:=diffusion only'):
            sent_to_navigation(CAR, machine='all', planner=planner, diffusion_trial='true')


def test_the_diffusion_trial_holds_the_car_to_its_speed():
    for asked, held in (('', '0.5'), ('0.3', '0.3'), ('0.5', '0.5')):
        context = context_for(CAR, machine='all', planner='diffusion',
                              diffusion_trial='true', max_speed=asked)
        CAR.launch_setup(context)
        # Every included file (ego, the geofence, navigation) reads this one cap.
        assert context.launch_configurations['max_speed'] == held
        sent = sent_to_navigation(CAR, machine='all', planner='diffusion',
                                  diffusion_trial='true', max_speed=asked)
        assert sent['planner'] == 'diffusion' and sent['max_speed'] == held
    # The switch alone picks the diffusion planner.
    assert sent_to_navigation(CAR, machine='all', diffusion_trial='true')['planner'] == (
        'diffusion')
    # Faster than the trial's cap, or nonsense: refused before anything starts.
    for asked in ('0.6', '1.0', '0', '-0.3', 'fast'):
        with pytest.raises(ValueError, match='max_speed'):
            sent_to_navigation(CAR, machine='all', planner='diffusion',
                               diffusion_trial='true', max_speed=asked)


def test_the_scan_unless_voxels_is_asked_for():
    nodes = nav_nodes(lidar='true')
    assert 'lidar_scan' in nodes and 'voxels' not in nodes
    assert local_costmap(lidar='true')['lidar_layer']['observation_sources'] == 'lidar'
    with pytest.raises(ValueError, match='lidar:=true'):
        nav_nodes(voxels='true')                        # no lidar: no cubes


def test_voxels_swaps_the_scan_for_cubes_in_the_costmap_and_the_stop_zone():
    nodes = nav_nodes(lidar='true', voxels='true')
    assert 'voxels' in nodes and 'lidar_scan' not in nodes
    s = settings(radar='true', lidar='true', voxels='true')
    assert s['collision_monitor']['observation_sources'] == ['radar', 'voxels']
    source = s['collision_monitor']['voxels']
    assert (source['type'], source['topic']) == ('pointcloud', '/perception/voxels/cubes')
    low, high = vehicle.obstacle_heights(vehicle.load()['vehicle'])
    # Its stock defaults (0.05 to 0.5 m) would drop every cube over half a metre.
    assert source['min_height'] < 0.0 and source['max_height'] > high + 1.0
    assert (s['voxels']['min_height'], s['voxels']['max_height']) == (
        pytest.approx(low), pytest.approx(high))
    layer = local_costmap(lidar='true', voxels='true')['lidar_layer']
    assert layer['observation_sources'] == 'voxels voxels_ground'
    assert layer['voxels']['topic'] == '/perception/voxels/cubes'
    assert (layer['voxels']['marking'], layer['voxels']['clearing']) == (True, True)
    ground = layer['voxels_ground']
    assert ground['topic'] == '/perception/voxels/ground'
    assert (ground['marking'], ground['clearing']) == (False, True)
    # Every bound lets the cubes through: the layer's own, and each source's (Nav2's
    # per-source defaults, 0 to 0 m and 3 m of raytrace, would drop or cut them).
    for bounds in (layer, layer['voxels'], ground):
        assert bounds['min_obstacle_height'] <= -1.0 and bounds['max_obstacle_height'] >= 3.5
    assert ground['raytrace_max_range'] >= layer['voxels']['obstacle_max_range']


def test_amcl_keeps_its_scan_beside_the_cubes():
    nodes = nav_nodes(lidar='true', voxels='true', amcl='true')
    assert {'voxels', 'lidar_scan', 'amcl'} <= set(nodes)


def test_the_sim_and_the_car_pass_voxels_on():
    assert sent_to_navigation(SIM)['voxels'] == 'false'
    assert sent_to_navigation(SIM, world='road', voxels='true')['voxels'] == 'true'
    with pytest.raises(ValueError, match='needs the lidar'):
        sent_to_navigation(SIM, voxels='true')          # the empty world: no lidar
    with pytest.raises(ValueError, match='needs the lidar'):
        sent_to_navigation(SIM, world='road', voxels='true', lidar='false')
    assert sent_to_navigation(CAR, machine='all')['voxels'] == 'false'
    assert sent_to_navigation(CAR, machine='all', voxels='true')['voxels'] == 'true'
    # No lidar on the car: no cubes either, rather than a launch that refuses.
    car = sent_to_navigation(CAR, machine='all', voxels='true', lidar='false')
    assert car['voxels'] == 'false'


def test_the_python_voxels_unless_rust_is_asked_for():
    for args in ({'voxels': 'true'}, {'screen_cubes': 'true'}):
        python = nav_nodes(lidar='true', **args)['voxels']
        assert python._Node__package == 'aav_voxels'
        assert 'lidar_xyz' not in given(python)       # it reads vehicle.yaml itself
        rust = nav_nodes(lidar='true', voxels_impl='rust', **args)['voxels']
        assert (rust._Node__package, rust._Node__node_executable) == ('aav_voxels_rs', 'voxels')
        # The same nav2.yaml block, and the car's numbers the Python works out for itself.
        assert any(isinstance(x, ParameterFile) for x in rust._Node__parameters)
        cfg = vehicle.load()
        p = given(rust)
        assert list(p['lidar_xyz']) == pytest.approx(cfg['sensors']['lidar_link']['xyz'])
        assert list(p['lidar_rpy']) == pytest.approx(cfg['sensors']['lidar_link']['rpy'])
        v = cfg['vehicle']
        back, front, half = p['body']
        # aav_lidar scan.body_box: the outline, 0.3 m behind and beside, 0.1 m ahead.
        assert back == pytest.approx(-v['body']['rear_overhang'] - 0.3)
        assert front == pytest.approx(v['body']['length'] - v['body']['rear_overhang'] + 0.1)
        assert half == pytest.approx(v['body']['width'] / 2 + 0.3)
    assert 'voxels' not in nav_nodes(lidar='true', voxels_impl='rust')   # no cubes asked for
    with pytest.raises(ValueError, match='voxels_impl'):
        nav_nodes(lidar='true', voxels='true', voxels_impl='c++')


def test_the_sim_and_the_car_pass_voxels_impl_on():
    assert sent_to_navigation(SIM)['voxels_impl'] == 'python'
    assert sent_to_navigation(SIM, world='road', voxels='true',
                              voxels_impl='Rust')['voxels_impl'] == 'rust'
    assert sent_to_navigation(CAR, machine='all')['voxels_impl'] == 'python'
    assert sent_to_navigation(CAR, machine='all', voxels='true',
                              voxels_impl='rust')['voxels_impl'] == 'rust'
    with pytest.raises(ValueError, match='voxels_impl'):
        sent_to_navigation(SIM, voxels_impl='go')


def test_lidar_scan_and_things_in_either_language():
    nodes = nav_nodes(lidar='true', things='true')
    assert nodes['lidar_scan']._Node__package == vehicle.LIDAR_IMPLS[vehicle.DEFAULT_LIDAR_IMPL]
    assert nodes['things']._Node__package == vehicle.THINGS_IMPLS[vehicle.DEFAULT_THINGS_IMPL]
    rust = nav_nodes(lidar='true', things='true', lidar_impl='rust', things_impl='rust')
    scan, things = rust['lidar_scan'], rust['things']
    assert (scan._Node__package, scan._Node__node_executable) == ('aav_lidar_rs', 'lidar_scan')
    assert (things._Node__package, things._Node__node_executable) == ('aav_things_rs', 'things')
    # The Rust scan reads no vehicle.yaml: the lidar's mount and the outline come with it.
    cfg = vehicle.load()
    assert list(given(scan)['lidar_xyz']) == pytest.approx(cfg['sensors']['lidar_link']['xyz'])
    assert len(given(scan)['body']) == 3
    # The things: the same settings either way.
    assert given(things)['background'] == given(nodes['things'])['background']
    python = nav_nodes(lidar='true', lidar_impl='python')['lidar_scan']
    assert python._Node__package == 'aav_lidar' and 'lidar_xyz' not in given(python)
    with pytest.raises(ValueError, match='lidar_impl'):
        nav_nodes(lidar='true', lidar_impl='c++')
    with pytest.raises(ValueError, match='things_impl'):
        nav_nodes(lidar='true', things='true', things_impl='go')


def test_the_sim_and_the_car_pass_lidar_impl_and_things_impl_on():
    for key, default in (('lidar_impl', vehicle.DEFAULT_LIDAR_IMPL),
                         ('things_impl', vehicle.DEFAULT_THINGS_IMPL)):
        assert sent_to_navigation(SIM)[key] == default
        assert sent_to_navigation(SIM, world='road', **{key: 'Rust'})[key] == 'rust'
        assert sent_to_navigation(CAR, machine='all')[key] == default
        if key == 'lidar_impl':   # the Python things went on 20 Nov
            assert sent_to_navigation(CAR, machine='all', **{key: 'python'})[key] == 'python'
        with pytest.raises(ValueError, match=key):
            sent_to_navigation(SIM, **{key: 'go'})


def test_kerbs_run_the_rust_voxels_node_with_its_step_finder():
    assert 'kerb_layer' not in local_costmap(lidar='true')['plugins']
    # No lidar: no kerbs, and no refusal (a world without one runs as before).
    assert 'voxels' not in nav_nodes(kerbs='true')
    nodes = nav_nodes(lidar='true', kerbs='true')
    # The scan stays; the Rust node runs beside it, even if voxels_impl says python.
    assert {'voxels', 'lidar_scan'} <= set(nodes)
    assert nodes['voxels']._Node__package == 'aav_voxels_rs'
    assert given(nodes['voxels'])['steps'] is True
    local = local_costmap(lidar='true', kerbs='true')
    assert 'kerb_layer' in local['plugins']
    assert local['kerb_layer']['steps']['topic'] == '/perception/voxels/steps'
    assert local['lidar_layer']['observation_sources'] == 'lidar'


def test_the_sim_passes_kerbs_on():
    assert sent_to_navigation(SIM, world='road', kerbs='true')['kerbs'] == 'true'
    assert sent_to_navigation(SIM, world='road', kerbs='false')['kerbs'] == 'false'
