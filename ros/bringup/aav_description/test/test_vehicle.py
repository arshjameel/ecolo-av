import json
import math
from pathlib import Path
import re
import shutil
import subprocess

from aav_description import vehicle
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'vehicle.yaml'
ROOT = Path(__file__).resolve().parents[4]
SPINE_RS = ROOT / 'firmware/spine/crates/spine-core/src/spine.rs'
NAV2_YAML = ROOT / 'ros/drivers/aav_navigation/config/nav2.yaml'


def rust_const(path, name):
    """Return the number in `pub const NAME: f32 = <number>;` from a Rust file."""
    found = re.search(rf'pub const {name}: f32 = ([-0-9.e]+);', path.read_text())
    assert found, f'no `pub const {name}: f32 = ...;` in {path}'
    return float(found.group(1))


def test_config_loads_and_has_what_the_launch_files_read():
    cfg = vehicle.load(CONFIG)
    v = cfg['vehicle']
    for key in ('wheelbase', 'track', 'wheel_diameter', 'max_steering_angle', 'body'):
        assert key in v
    for name in ('imu_link', 'gps_link', 'radar_link', 'camera_left_link',
                 'camera_right_link', 'camera_rear_link', 'lidar_link'):
        assert len(cfg['sensors'][name]['xyz']) == 3


def test_turning_radius_is_bicycle_model():
    v = {'wheelbase': 1.57, 'max_steering_angle': 0.5}
    assert math.isclose(vehicle.min_turning_radius(v), 1.57 / math.tan(0.5))
    assert 2.8 < vehicle.min_turning_radius(v) < 2.9


def test_footprint_encloses_base_link_and_is_nav2_string():
    v = {'body': {'length': 2.0, 'width': 1.0, 'rear_overhang': 0.2}}
    pts = vehicle.footprint_points(v)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    assert min(xs) == -0.2 and math.isclose(max(xs), 1.8)
    assert min(ys) == -0.5 and max(ys) == 0.5
    assert vehicle.footprint(v) == \
        '[[1.800, 0.500], [1.800, -0.500], [-0.200, -0.500], [-0.200, 0.500]]'


def test_stop_zone_is_ahead_of_the_bumper_and_wider_than_the_body():
    v = {'body': {'length': 2.0, 'width': 1.0, 'rear_overhang': 0.2}}
    pts = vehicle.stop_zone_points(v, ahead=1.0, margin=0.2)
    xs = sorted({round(p[0], 6) for p in pts})
    ys = sorted({round(p[1], 6) for p in pts})
    assert xs == [1.8, 2.8] and ys == [-0.7, 0.7]
    assert vehicle.polygon(pts) == \
        '[[2.800, 0.700], [2.800, -0.700], [1.800, -0.700], [1.800, 0.700]]'


def test_reversing_stop_zone_is_behind_the_rear_bumper():
    v = {'body': {'length': 2.0, 'width': 1.0, 'rear_overhang': 0.2}}
    pts = vehicle.rear_stop_zone_points(v, behind=1.0, margin=0.2)
    xs = sorted({round(p[0], 6) for p in pts})
    ys = sorted({round(p[1], 6) for p in pts})
    assert xs == [-1.2, -0.2] and ys == [-0.7, 0.7]


def test_nav2_reverses_only_if_something_sees_behind():
    assert vehicle.blind_behind(False) is None                  # forward only: nothing to see
    assert vehicle.blind_behind(True, lidar=True) is None       # the roof lidar, all round
    assert vehicle.blind_behind(True, rear_camera=True) is None  # the rear camera's
    # camera_obstacles alone reads the front camera: it doesn't see behind.
    assert vehicle.blind_behind(True, camera_obstacles=True)
    why = vehicle.blind_behind(True, radar=True, lidar=False)   # the radar faces forward
    assert why and 'forward only' in why


def test_the_camera_obstacles_guard_the_stop_zone_too():
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    monitor = vehicle.nav2_tuned(nav2, v, lidar=True, camera=True)['collision_monitor']
    assert monitor['ros__parameters']['observation_sources'] == ['radar', 'lidar', 'camera']
    assert monitor['ros__parameters']['camera']['topic'] == '/perception/camera_obstacles'
    plain = vehicle.nav2_tuned(nav2, v)['collision_monitor']['ros__parameters']
    assert 'camera' not in plain['observation_sources']


def test_the_stop_zone_turns_round_with_the_command():
    # collision_monitor picks the box by the command's speed: ahead going forward (and
    # standing still), behind reversing. Both cover every turn rate.
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    zone = nav2['collision_monitor']['ros__parameters']['StopZone']
    assert zone['type'] == 'velocity_polygon' and zone['holonomic'] is False
    forward, backward = (zone[name] for name in zone['velocity_polygons'])
    assert forward['linear_min'] == 0.0 < forward['linear_max']
    assert backward['linear_min'] < 0.0 == backward['linear_max']
    for box in (forward, backward):
        assert box['theta_min'] <= -1.0 and box['theta_max'] >= 1.0


def test_nav2_rewrites_keys():
    v = vehicle.load(CONFIG)['vehicle']
    r = vehicle.nav2_rewrites(v)
    assert set(r) == {'footprint', 'minimum_turning_radius', 'min_turning_r',
                      'collision_monitor.ros__parameters.StopZone.forward.points',
                      'collision_monitor.ros__parameters.StopZone.backward.points',
                      'planner_server.ros__parameters.GridBased.motion_model_for_search',
                      'controller_server.ros__parameters.FollowPath.allow_reversing',
                      'controller_server.ros__parameters.MPPI.vx_min',
                      'lidar_scan.ros__parameters.min_height',
                      'lidar_scan.ros__parameters.max_height'}
    # The planner leaves the controller some steering; MPPI knows the car's real lock.
    assert float(r['minimum_turning_radius']) == pytest.approx(
        float(r['min_turning_r']) * vehicle.PLAN_TURN_MARGIN, abs=2e-3)
    # RewrittenYaml can't index into a list: none of these may try.
    assert not any(key.split('.')[-1].isdigit() for key in r)


def test_nav2_forward_only_or_reversing():
    v = {'wheelbase': 1.57, 'max_steering_angle': 0.5, 'max_reverse_speed': 0.4,
         'body': {'length': 2.0, 'width': 1.0, 'height': 1.5, 'rear_overhang': 0.2}}
    forward = vehicle.nav2_rewrites(v)
    back = vehicle.nav2_rewrites(v, reversing=True)
    model = 'planner_server.ros__parameters.GridBased.motion_model_for_search'
    allow = 'controller_server.ros__parameters.FollowPath.allow_reversing'
    vx_min = 'controller_server.ros__parameters.MPPI.vx_min'
    assert (forward[model], forward[allow], forward[vx_min]) == ('DUBIN', 'false', '0.000')
    assert (back[model], back[allow], back[vx_min]) == ('REEDS_SHEPP', 'true', '-0.400')


NAV2_SMALL = {
    'velocity_smoother': {'ros__parameters': {'min_velocity': [-0.5, 0.0, -1.0],
                                              'max_velocity': [1.0, 0.0, 1.0]}},
    'controller_server': {'ros__parameters': {'FollowPath': {'desired_linear_vel': 1.0}}},
    'collision_monitor': {'ros__parameters': {'observation_sources': ['radar']}},
    'local_costmap': {'local_costmap': {'ros__parameters': {
        'plugins': ['static_layer', 'obstacle_layer', 'lidar_layer', 'ghost_layer',
                    'inflation_layer'],
        'obstacle_layer': {'enabled': False}, 'lidar_layer': {}}}},
    'global_costmap': {'global_costmap': {'ros__parameters': {
        'plugins': ['static_layer', 'inflation_layer']}}},
}


def small(v, **args):
    """nav2_tuned on NAV2_SMALL: it has no MPPI block, so Regulated Pure Pursuit."""
    return vehicle.nav2_tuned(NAV2_SMALL, v, controller='rpp', **args)


def test_the_lidar_feeds_the_costmap_and_the_stop_zone_only_when_it_runs():
    v = {'max_reverse_speed': 0.4}

    def picked(**sensors):
        cfg = small(v, **sensors)
        return (cfg['collision_monitor']['ros__parameters']['observation_sources'],
                cfg['local_costmap']['local_costmap']['ros__parameters']['plugins'])

    sources, layers = picked()                       # the default: the radar alone
    assert sources == ['radar'] and 'lidar_layer' not in layers
    sources, layers = picked(radar=True, lidar=True)
    assert sources == ['radar', 'lidar'] and 'lidar_layer' in layers
    sources, layers = picked(radar=False, lidar=True)
    assert sources == ['lidar'] and layers[-1] == 'inflation_layer'


def test_the_ghost_is_its_own_source_and_layer_only_when_asked_for():
    v = {'max_reverse_speed': 0.4}

    def picked(**sensors):
        cfg = small(v, **sensors)
        return (cfg['collision_monitor']['ros__parameters']['observation_sources'],
                cfg['local_costmap']['local_costmap']['ros__parameters']['plugins'])

    sources, layers = picked()
    assert 'ghost' not in sources and 'ghost_layer' not in layers
    # On top of the real sensors: they keep their own layers and sources.
    sources, layers = picked(radar=True, lidar=True, ghost=True)
    assert sources == ['radar', 'lidar', 'ghost']
    assert layers == ['static_layer', 'obstacle_layer', 'lidar_layer', 'ghost_layer',
                      'inflation_layer']
    sources, layers = picked(radar=False, ghost=True)
    assert sources == ['ghost'] and 'lidar_layer' not in layers and 'ghost_layer' in layers


def test_the_lidar_ignores_the_road_and_what_the_car_passes_under():
    low, high = vehicle.obstacle_heights({'body': {'height': 1.5}})
    assert 0.0 < low < 0.5          # over the road and its kerbs
    assert high > 1.5               # over the roof, so nothing the car hits is dropped
    rewrites = vehicle.nav2_rewrites(vehicle.load(CONFIG)['vehicle'])
    assert float(rewrites['lidar_scan.ros__parameters.max_height']) > 1.5


def test_the_smoother_gets_the_reverse_speed_as_a_list():
    v = {'max_reverse_speed': 0.4}
    smoother = 'velocity_smoother'
    forward = small(v)[smoother]['ros__parameters']
    back = small(v, reversing=True)[smoother]['ros__parameters']
    assert forward['min_velocity'] == [0.0, 0.0, -1.0]
    assert back['min_velocity'] == [-0.4, 0.0, -1.0]
    # A copy: the loaded file stays as it was.
    assert NAV2_SMALL[smoother]['ros__parameters']['min_velocity'] == [-0.5, 0.0, -1.0]


def test_max_speed_raises_the_controller_and_the_smoother_together():
    v = {'max_reverse_speed': 0.4}
    same = small(v)
    assert same['controller_server'] == NAV2_SMALL['controller_server']
    fast = small(v, reversing=True, max_speed=3)
    assert fast['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] == 3.0
    smoother = fast['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [3.0, 0.0, 1.0]
    assert smoother['min_velocity'] == [-0.4, 0.0, -1.0]   # reversing stays as slow


def test_mppi_follows_the_path_under_the_name_the_trees_ask_for():
    # The behaviour trees ask for FollowPath: mppi puts a copy of the MPPI block there.
    v = {'max_reverse_speed': 0.4}
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    servers = 'controller_server'
    assert vehicle.DEFAULT_CONTROLLER == 'mppi'
    rpp = vehicle.nav2_tuned(nav2, v, controller='rpp')[servers]['ros__parameters']
    assert rpp['FollowPath'] == nav2[servers]['ros__parameters']['FollowPath']
    assert 'RegulatedPurePursuit' in rpp['FollowPath']['plugin']
    mppi = vehicle.nav2_tuned(nav2, v, reversing=True, max_speed=3, controller='mppi')
    follow = mppi[servers]['ros__parameters']['FollowPath']
    assert follow['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert follow['motion_model'] == 'Ackermann'
    assert (follow['vx_min'], follow['vx_max']) == (-0.4, 3.0)
    assert follow['enforce_path_inversion'] is True     # it reverses where the path does
    assert follow['PathAngleCritic']['mode'] == 2       # ...in the path's own direction
    assert 'PreferForwardCritic' not in follow['critics']   # the planner chose to reverse
    assert 'desired_linear_vel' not in follow
    forward = vehicle.nav2_tuned(nav2, v, controller='mppi')[servers]['ros__parameters']
    assert (forward['FollowPath']['vx_min'], forward['FollowPath']['vx_max']) == (0.0, 1.0)
    assert forward['FollowPath']['enforce_path_inversion'] is False
    assert 'mode' not in forward['FollowPath'].get('PathAngleCritic', {})
    assert 'PreferForwardCritic' in forward['FollowPath']['critics']
    # The MPPI block itself stays as the file has it, where nav2_rewrites expects it.
    assert mppi[servers]['ros__parameters']['MPPI'] == nav2[servers]['ros__parameters']['MPPI']
    with pytest.raises(ValueError, match='pick one of rpp, mppi'):
        vehicle.nav2_tuned(nav2, v, controller='dwb')


def test_the_planner_sees_the_sensors_but_not_the_ghost():
    v = {'max_reverse_speed': 0.4}
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    planner = 'global_costmap'

    def layers(**sensors):
        cfg = vehicle.nav2_tuned(nav2, v, **sensors)[planner][planner]['ros__parameters']
        return cfg['plugins'], cfg

    assert layers(radar=False, lidar=False)[0] == ['static_layer', 'inflation_layer']
    plugins, cfg = layers(radar=True, lidar=True, ghost=True)
    assert plugins == ['static_layer', 'obstacle_layer', 'lidar_layer', 'inflation_layer']
    assert cfg['obstacle_layer']['enabled'] is True          # the local one waits for launch
    assert cfg['lidar_layer']['lidar']['topic'] == '/sensors/lidar/scan'
    assert 'ghost_layer' not in plugins
    assert layers(radar=False, lidar=True)[0] == ['static_layer', 'lidar_layer',
                                                  'inflation_layer']
    # plan_around:=false: the planner sees only the map, for the stop zone's tests.
    assert layers(radar=True, lidar=True, plan_around=False)[0] == ['static_layer',
                                                                    'inflation_layer']


def test_every_rewrite_lands_on_a_key_nav2_yaml_has():
    # RewrittenYaml drops a whole dotted path that isn't in the file, silently.
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    for key in vehicle.nav2_rewrites(v, reversing=True):
        if '.' not in key:
            continue
        node = nav2
        for part in key.split('.'):
            assert part in node, f'{key}: no {part!r} in nav2.yaml'
            node = node[part]
    tuned = vehicle.nav2_tuned(nav2, v, reversing=True, max_speed=2.0)
    smoother = tuned['velocity_smoother']['ros__parameters']
    assert smoother['min_velocity'][0] == -v['max_reverse_speed']
    assert smoother['max_velocity'][0] == 2.0


def test_nav2_yaml_is_forward_only_until_the_launch_file_says():
    # The file's own values are the safe ones: no launch file, no reversing.
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    follow = nav2['controller_server']['ros__parameters']['FollowPath']
    assert follow['allow_reversing'] is False
    assert nav2['controller_server']['ros__parameters']['MPPI']['vx_min'] == 0.0
    assert nav2['velocity_smoother']['ros__parameters']['min_velocity'][0] == 0.0
    grid = nav2['planner_server']['ros__parameters']['GridBased']
    assert grid['motion_model_for_search'] == 'DUBIN'


def test_ego_gets_the_reverse_speed_and_the_body():
    car = vehicle.load(CONFIG)
    v, params = car['vehicle'], vehicle.ego_params(car)
    assert params['max_reverse_speed'] == v['max_reverse_speed']
    assert params['mass'] == v['mass']
    assert params['cg_height'] == v['centre_of_mass']['height']
    assert params['full_throttle_accel'] == car['actuators']['max_accel']
    assert params['full_brake_decel'] == car['actuators']['max_brake']
    assert all(isinstance(value, float) for value in params.values())


def test_the_centre_of_mass_from_the_scales():
    v = {'wheelbase': 1.57, 'wheel_diameter': 0.30}
    # A made-up car: 181 kg, centre of mass 0.70 m ahead of the rear axle, 0.55 m up.
    total, x, h, r, lift = 181.0, 0.70, 0.55, 0.15, 0.25
    front = total * x / 1.57
    assert vehicle.centre_of_mass(v, front, total - front) == (pytest.approx(x), None)
    # Lift the rear: work out the front's new weight from the pitched car's geometry.
    tilt = math.asin(lift / 1.57)
    along, up = (math.cos(tilt), -math.sin(tilt)), (math.sin(tilt), math.cos(tilt))
    cg_ahead = x * along[0] + (h - r) * up[0]        # of the rear axle, level ground
    front_lifted = total * cg_ahead / (1.57 * math.cos(tilt))
    got_x, got_h = vehicle.centre_of_mass(v, front, total - front, front_lifted, lift)
    assert (got_x, got_h) == (pytest.approx(x), pytest.approx(h))


def test_the_firmware_copies_match_vehicle_yaml():
    # spine-core can't read vehicle.yaml, so it keeps copies. Change both together.
    v = vehicle.load(CONFIG)['vehicle']
    assert rust_const(SPINE_RS, 'MAX_STEERING_ANGLE') == v['max_steering_angle']
    assert rust_const(SPINE_RS, 'MAX_REVERSE_SPEED') == v['max_reverse_speed']


CAR = {'wheelbase': 1.57, 'track': 0.77, 'max_steering_angle': 0.5}


def test_the_inner_wheel_turns_further_than_the_centre_and_the_outer_less():
    inner, outer = vehicle.wheel_angles(CAR, 0.5)
    assert outer < 0.5 < inner
    assert math.isclose(inner, math.atan(1.57 / (1.57 / math.tan(0.5) - 0.385)))
    # Ackermann: both wheels point at the same turning centre.
    assert math.isclose(1 / math.tan(outer) - 1 / math.tan(inner), 0.77 / 1.57)
    assert vehicle.wheel_angles(CAR, 0.0) == (0.0, 0.0)
    assert vehicle.wheel_angles(CAR, -0.3) == vehicle.wheel_angles(CAR, 0.3)


def test_m5s_two_angles_give_the_centre_angle_back():
    for centre in (0.1, 0.3, 0.5, 0.6):
        inner, outer = vehicle.wheel_angles(CAR, centre)
        assert math.isclose(vehicle.centre_angle(CAR, inner, outer), centre)
    # 29 degrees each way: the centre is 29 degrees too.
    same = math.radians(29)
    assert math.isclose(vehicle.centre_angle(CAR, same, same), same)
    with pytest.raises(ValueError):
        vehicle.centre_angle(CAR, 0.5, 0.0)


def gazebo_centre_angle(v, steering_limit):
    """Gazebo's AckermannSteering at full lock (gz-sim 8, UpdateVelocity): the centre angle."""
    smallest_radius = v['wheelbase'] / math.sin(steering_limit)
    return math.atan(v['wheelbase'] / smallest_radius)


def test_gazebos_steering_limit_gives_the_centre_angle_at_full_lock():
    # Given max_steering_angle as it is, Gazebo stops 10% short: the 0.433 found by circles.
    assert gazebo_centre_angle(CAR, 0.5) < 0.45
    limit = vehicle.gazebo_steering_limit(CAR)
    assert limit > 0.5
    assert math.isclose(gazebo_centre_angle(CAR, limit), 0.5)
    # Its smallest radius is the bicycle model's: Nav2's minimum turning radius.
    assert math.isclose(CAR['wheelbase'] / math.sin(limit), vehicle.min_turning_radius(CAR))
    # The inner wheel's joint limit in aav.urdf.xacro (1.3 x max) leaves it room.
    v = vehicle.load(CONFIG)['vehicle']
    assert vehicle.wheel_angles(v, v['max_steering_angle'])[0] < 1.3 * v['max_steering_angle']
    with pytest.raises(ValueError):
        vehicle.gazebo_steering_limit({'max_steering_angle': math.radians(50)})


def find_share():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('aav_description')
    except Exception:       # tools/test.sh: no ROS, or not built
        return None


@pytest.mark.skipif(shutil.which('xacro') is None or find_share() is None,
                    reason='needs xacro and the built package')
def test_gazebos_urdf_gets_the_derived_limit_and_refuses_none():
    urdf = Path(find_share()) / 'urdf' / 'aav.urdf.xacro'
    v = vehicle.load(CONFIG)['vehicle']
    limit = f'{vehicle.gazebo_steering_limit(v):.6f}'
    out = subprocess.run(['xacro', str(urdf), 'sim:=true', f'gz_steering_limit:={limit}'],
                         capture_output=True, text=True, check=True).stdout
    assert f'<steering_limit>{limit}</steering_limit>' in out
    missing = subprocess.run(['xacro', str(urdf), 'sim:=true'], capture_output=True)
    assert missing.returncode != 0
    car = subprocess.run(['xacro', str(urdf)], capture_output=True, text=True, check=True)
    assert 'steering_limit' not in car.stdout


EGO_YAML = ROOT / 'ros/ego/aav_ego/config/ego.yaml'


def test_trackers_sit_beside_the_controllers_and_none_is_the_default():
    assert vehicle.DEFAULT_TRACKER == 'none' and 'none' in vehicle.TRACKERS
    # Not in CONTROLLERS: nav2_tuned would take a tracker's name for RPP.
    assert not set(vehicle.TRACKERS) & set(vehicle.CONTROLLERS)
    assert vehicle.check_tracker(' LQR ') == 'lqr'
    assert vehicle.check_tracker('none', reversing=True) == 'none'
    with pytest.raises(ValueError, match='pick one of'):
        vehicle.check_tracker('mppi')
    with pytest.raises(ValueError, match='reversing:=false'):
        vehicle.check_tracker('stanley', reversing=True)


def test_track_params_are_nav2s_own_checks_and_speeds():
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    cfg = vehicle.load(CONFIG)
    ego = yaml.safe_load(EGO_YAML.read_text())['ego']['ros__parameters']
    p = vehicle.track_params(nav2, cfg, ego)
    server = nav2['controller_server']['ros__parameters']
    assert p['xy_goal_tolerance'] == server['general_goal_checker']['xy_goal_tolerance']
    assert p['yaw_goal_tolerance'] == server['general_goal_checker']['yaw_goal_tolerance']
    assert p['progress_time'] == server['progress_checker']['movement_time_allowance']
    assert p['cruise_speed'] == server['FollowPath']['desired_linear_vel']
    assert p['rate_hz'] == server['controller_frequency']
    assert p['odom_topic'] == '/localization/odom/local'
    assert p['stamped'] is True
    assert p['wheelbase'] == cfg['vehicle']['wheelbase']
    # The tighter of ego's ramp and the steering motor.
    assert p['max_steering_rate'] == min(cfg['actuators']['steer_rate'],
                                         ego['max_steering_rate'])
    assert vehicle.track_params(nav2, cfg, ego, max_speed=3.0)['cruise_speed'] == 3.0


def test_an_s_bend_needs_room_ahead():
    # Two quarter-turns side by side: 2 radii across, 2 radii ahead.
    assert math.isclose(vehicle.s_bend_length(3.0, 6.0), 6.0)
    assert vehicle.s_bend_length(3.0, 0.0) == 0.0
    assert math.isclose(vehicle.s_bend_length(3.0, -1.0), vehicle.s_bend_length(3.0, 1.0))
    # Over 2 radii sideways no S-bend does it: a loop.
    assert vehicle.s_bend_length(3.0, 6.1) == math.inf
    # 1 m sideways at the planner's 3.6 m: 3.6 m ahead, more than the goal checker's circle.
    assert 3.5 < vehicle.s_bend_length(3.6, 1.0) < 3.8


def test_the_final_approach_is_two_planned_turning_radii():
    v = vehicle.load(CONFIG)['vehicle']
    r = vehicle.plan_turning_radius(v)
    assert math.isclose(r, vehicle.min_turning_radius(v) * vehicle.PLAN_TURN_MARGIN)
    assert math.isclose(vehicle.final_approach(v), 2.0 * r)
    # The planner gets the same radius.
    assert vehicle.nav2_rewrites(v)['minimum_turning_radius'] == f'{r:.3f}'


LATTICE_JSON = ROOT / 'ros/drivers/aav_navigation/config/lattice_ackermann.json'
SEARCH_MODEL = 'planner_server.ros__parameters.GridBased.motion_model_for_search'


def test_planners_hybrid_by_default_and_only_car_shaped_ones_on_the_car():
    assert vehicle.DEFAULT_PLANNER == 'hybrid'
    assert set(vehicle.CAR_PLANNERS) < set(vehicle.PLANNERS)
    assert set(vehicle.PLANNER_BLOCKS) == set(vehicle.PLANNERS)
    assert vehicle.check_planner(' Lattice ') == 'lattice'
    assert vehicle.check_planner('navfn') == 'navfn'
    assert vehicle.check_planner('lattice', real=True) == 'lattice'
    for name in ('smac2d', 'navfn', 'theta_star'):
        with pytest.raises(ValueError, match='turning radius'):
            vehicle.check_planner(name, real=True)
    with pytest.raises(ValueError, match='pick one of'):
        vehicle.check_planner('rrt')


def test_every_planner_has_its_block_in_nav2_yaml():
    plans = yaml.safe_load(NAV2_YAML.read_text())['planner_server']['ros__parameters']
    for planner, block in vehicle.PLANNER_BLOCKS.items():
        assert 'plugin' in plans[block], planner


@pytest.mark.parametrize('planner', vehicle.PLANNERS)
def test_nav2_tuned_puts_the_planner_under_gridbased(planner):
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    lattice = '/x/lattice.json' if planner == 'lattice' else None
    plans = vehicle.nav2_tuned(nav2, v, planner=planner,
                               lattice_file=lattice)['planner_server']['ros__parameters']
    want = nav2['planner_server']['ros__parameters'][vehicle.PLANNER_BLOCKS[planner]]
    assert plans['GridBased']['plugin'] == want['plugin']
    assert set(plans) & set(vehicle.PLANNER_BLOCKS.values()) == {'GridBased'}
    # The search model is Hybrid's alone: a whole-path rewrite would add it to any other.
    assert (SEARCH_MODEL in vehicle.nav2_rewrites(v, planner=planner)) == (planner == 'hybrid')
    # nav2.yaml itself is left as it was.
    assert 'Lattice' in nav2['planner_server']['ros__parameters']


def test_lattice_needs_its_file_and_reverses_only_with_reversing():
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    with pytest.raises(ValueError, match='lattice_file'):
        vehicle.nav2_tuned(nav2, v, planner='lattice')
    for reversing in (False, True):
        grid = vehicle.nav2_tuned(nav2, v, reversing=reversing, planner='lattice',
                                  lattice_file='/x/l.json')['planner_server'][
                                      'ros__parameters']['GridBased']
        assert grid['lattice_filepath'] == '/x/l.json'
        assert grid['allow_reverse_expansion'] is reversing


def test_the_committed_lattice_fits_the_car():
    v = vehicle.load(CONFIG)['vehicle']
    meta = json.loads(LATTICE_JSON.read_text())['lattice_metadata']
    assert vehicle.lattice_problems(meta, v, 0.25) == []
    wider = dict(v, max_steering_angle=v['max_steering_angle'] * 0.8)
    assert 'turning radius' in vehicle.lattice_problems(meta, wider, 0.25)[0]
    assert 'cells' in vehicle.lattice_problems(meta, v, 0.05)[0]
