import copy
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
EGO_YAML = ROOT / 'ros/ego/aav_ego/config/ego.yaml'


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


def test_self_box_holds_the_body_and_every_sensor_with_a_margin():
    cfg = {'vehicle': {'body': {'length': 2.0, 'width': 1.0, 'rear_overhang': 0.2}},
           'sensors': {'cam': {'xyz': [1.95, 0.3, 1.5]}, 'back': {'xyz': [-0.2, -0.6, 0.9]}}}
    box = vehicle.self_box(cfg, margin=0.1)
    # The camera sticks out 0.15 m ahead, the rear one 0.1 m to the right: both kept in.
    assert box['min'] == pytest.approx([-0.2 - 0.04 - 0.1, -0.6 - 0.04 - 0.1])
    assert box['max'] == pytest.approx([1.95 + 0.04 + 0.1, 0.5 + 0.1])
    bare = vehicle.self_box({'vehicle': cfg['vehicle']}, margin=0.0)
    assert bare == {'min': [-0.2, -0.5], 'max': [pytest.approx(1.8), 0.5]}


def test_the_real_cars_self_box_holds_its_cameras_and_roof_with_room():
    # The ghost of 30 Oct: Gazebo's lidar saw the roof's rear corners (x -0.21, |y| 0.51)
    # and the front cameras (x 1.85) just outside a box flush with the body.
    box = vehicle.self_box(vehicle.load(CONFIG))
    assert box['min'][0] < -0.3 and box['max'][0] > 1.9
    assert box['min'][1] < -0.6 and box['max'][1] > 0.6
    # ... but not so big it hides a person beside the car.
    assert box['max'][0] - box['min'][0] < 3.0 and box['max'][1] - box['min'][1] < 1.6


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
                      'lidar_scan.ros__parameters.max_height',
                      'voxels.ros__parameters.min_height',
                      'voxels.ros__parameters.max_height'}
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
    with pytest.raises(ValueError, match='pick one of rpp, mppi'):  # TUNED_CONTROLLERS
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
    # The planner's copy of the lidar looks further than the follower's (16 Nov).
    assert cfg['lidar_layer']['lidar']['obstacle_max_range'] == vehicle.PLANNER_SEES
    local = vehicle.nav2_tuned(nav2, v, radar=True, lidar=True)['local_costmap'][
        'local_costmap']['ros__parameters']['lidar_layer']['lidar']
    assert local['obstacle_max_range'] < vehicle.PLANNER_SEES
    assert 'ghost_layer' not in plugins
    assert layers(radar=False, lidar=True)[0] == ['static_layer', 'lidar_layer',
                                                  'inflation_layer']
    # plan_around:=false: the planner sees only the map, for the stop zone's tests.
    assert layers(radar=True, lidar=True, plan_around=False)[0] == ['static_layer',
                                                                    'inflation_layer']


def test_voxels_needs_the_lidar_and_reaches_the_planner_too():
    v = {'max_reverse_speed': 0.4}
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    # Without the lidar there are no cubes: nothing changes.
    assert vehicle.nav2_tuned(nav2, v, lidar=False, voxels=True) == vehicle.nav2_tuned(
        nav2, v, lidar=False)
    cfg = vehicle.nav2_tuned(nav2, v, radar=False, lidar=True, voxels=True)
    planner = cfg['global_costmap']['global_costmap']['ros__parameters']['lidar_layer']
    assert planner['observation_sources'] == 'voxels voxels_ground'
    assert cfg['collision_monitor']['ros__parameters']['observation_sources'] == ['voxels']
    # The file itself is left as it was: the scan stays the default.
    local = nav2['local_costmap']['local_costmap']['ros__parameters']['lidar_layer']
    assert local['observation_sources'] == 'lidar'


def test_kerbs_add_a_marking_only_layer_with_the_lidar():
    v = {'max_reverse_speed': 0.4, 'body': {'length': 2.0, 'width': 1.0, 'rear_overhang': 0.2}}
    nav2 = yaml.safe_load(NAV2_YAML.read_text())

    def plugins(**args):
        cfg = vehicle.nav2_tuned(nav2, v, radar=False, **args)
        return cfg, cfg['local_costmap']['local_costmap']['ros__parameters']['plugins']

    assert 'kerb_layer' not in plugins(lidar=True)[1]
    assert 'kerb_layer' not in plugins(lidar=False, kerbs=True)[1]
    cfg, on = plugins(lidar=True, kerbs=True)
    assert on.index('kerb_layer') < on.index('inflation_layer')
    layer = cfg['local_costmap']['local_costmap']['ros__parameters']['kerb_layer']
    assert layer['observation_sources'] == 'steps'
    assert layer['steps']['topic'] == '/perception/voxels/steps'
    assert layer['steps']['marking'] and not layer['steps']['clearing']
    # The stop zone's too (the node keeps them: the lidar loses a kerb before the zone
    # reaches it); not the planner's.
    assert cfg['collision_monitor']['ros__parameters']['observation_sources'] == [
        'lidar', 'steps']
    assert 'steps' not in plugins(lidar=True)[0]['collision_monitor']['ros__parameters'][
        'observation_sources']
    assert 'kerb_layer' not in cfg['global_costmap']['global_costmap']['ros__parameters'][
        'plugins']
    # Our costmap reads it too, from the same file.
    ours = vehicle.costmap_params(cfg, vehicle.load(CONFIG)['vehicle'])['local']
    kerb = [layer for layer in ours['layers'] if layer['name'] == 'kerb_layer']
    assert len(kerb) == 1 and kerb[0]['sources'][0]['clearing'] is False


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


CALIBRATION_RS = ROOT / 'firmware/spine/crates/spine-board/src/calibration.rs'


def test_the_firmware_brake_travel_matches_vehicle_yaml():
    # Like the steering's ends: the firmware maps the brake 0..1 onto released..full
    # (calibration.rs `Brake`), the adaptive brake reads them from vehicle.yaml.
    brake = vehicle.load(CONFIG)['brake']
    block = re.search(r'brake: Brake \{\s*released: (\d+),\s*full: (\d+),',
                      CALIBRATION_RS.read_text())
    assert block, f'no `brake: Brake {{ released, full }}` in {CALIBRATION_RS}'
    assert (float(block.group(1)), float(block.group(2))) == (brake['released'], brake['full'])


def test_the_brake_limits_are_in_order():
    from aav_description.adaptive_brake import BrakeLimits
    limits = BrakeLimits.from_vehicle(vehicle.load(CONFIG))
    # released < touch < safe max <= full (or all reversed, for a brake that pulls).
    assert 0.0 < limits.touch_fraction < limits.safe_max_fraction <= 1.0
    assert limits.max_hold_time > 0.0


def test_ego_and_the_spine_share_one_forward_speed_limit():
    # Every driver respects one limit: ego asks for no more, the spine enforces it, the RC
    # transmitter too. Raise both together, per the test plan.
    ego = yaml.safe_load(EGO_YAML.read_text())['ego']['ros__parameters']
    assert rust_const(SPINE_RS, 'MAX_SPEED') == ego['max_speed']


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


def test_the_inner_wheels_joint_limit_leaves_room():
    # The inner wheel's joint limit in aav.urdf.xacro (1.3 x max) leaves it room.
    v = vehicle.load(CONFIG)['vehicle']
    assert vehicle.wheel_angles(v, v['max_steering_angle'])[0] < 1.3 * v['max_steering_angle']


def find_share():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('aav_description')
    except Exception:       # tools/test.sh: no ROS, or not built
        return None


@pytest.mark.skipif(shutil.which('xacro') is None or find_share() is None,
                    reason='needs xacro and the built package')
def test_the_urdf_has_no_gazebo_left():
    urdf = Path(find_share()) / 'urdf' / 'aav.urdf.xacro'
    car = subprocess.run(['xacro', str(urdf)], capture_output=True, text=True, check=True)
    assert 'gazebo' not in car.stdout and 'steering_limit' not in car.stdout


EGO_YAML = ROOT / 'ros/ego/aav_ego/config/ego.yaml'


def test_trackers_sit_beside_the_controllers_and_mppi_lag_is_the_default():
    # Nav2's controller_server (tracker:=none) went on 17 Nov: mppi_lag is the default.
    assert vehicle.DEFAULT_TRACKER == 'mppi_lag' and 'none' not in vehicle.TRACKERS
    # Not in CONTROLLERS: nav2_tuned would take a tracker's name for a block layout.
    assert not set(vehicle.TRACKERS) & set(vehicle.TUNED_CONTROLLERS)
    assert vehicle.check_tracker(' LQR ') == 'lqr'
    assert vehicle.check_tracker('mppi_lag', reversing=True) == 'mppi_lag'
    with pytest.raises(ValueError, match='pick one of'):
        vehicle.check_tracker('none')
    with pytest.raises(ValueError, match='pick one of'):
        vehicle.check_tracker('mppi')
    # mppi_lag reverses (Ben, 14 Nov); aav_track's trackers too (TODO 174); mpc_lag_rs
    # since 20 Nov (TODO 213).
    assert vehicle.check_tracker('mppi_lag', reversing=True) == 'mppi_lag'
    assert vehicle.check_tracker('stanley', reversing=True) == 'stanley'
    assert vehicle.check_tracker('mpc_lag_rs', reversing=True) == 'mpc_lag_rs'
    assert set(vehicle.FORWARD_ONLY_TRACKERS) == set()


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
    smoother = nav2['velocity_smoother']['ros__parameters']
    assert p['deceleration'] == -smoother['max_decel'][0] > 0
    assert p['odom_topic'] == '/localization/odom/local'
    assert p['stamped'] is True
    assert p['wheelbase'] == cfg['vehicle']['wheelbase']
    # The tighter of ego's ramp and the steering motor.
    assert p['max_steering_rate'] == min(cfg['actuators']['steer_rate'],
                                         ego['max_steering_rate'])
    assert vehicle.track_params(nav2, cfg, ego, max_speed=3.0)['cruise_speed'] == 3.0
    # TODO 174: aav_track backs up at max_reverse_speed with reversing, never without.
    assert p['max_reverse_speed'] == 0.0
    back = vehicle.track_params(nav2, cfg, ego, reversing=True)['max_reverse_speed']
    assert back == cfg['vehicle']['max_reverse_speed'] > 0
    # TODO 213: mpc_lag_rs the same.
    assert vehicle.mpc_lag_rs_params(nav2, cfg, ego)['max_reverse_speed'] == 0.0
    back = vehicle.mpc_lag_rs_params(nav2, cfg, ego, reversing=True)['max_reverse_speed']
    assert back == cfg['vehicle']['max_reverse_speed']


def test_check_speed_loop_takes_spine_cores_names_and_refuses_the_rest():
    from aav_description.car_model import SPEED_LOOP_PROFILES
    assert vehicle.DEFAULT_SPEED_LOOP == 'default'
    for name in SPEED_LOOP_PROFILES:
        assert vehicle.check_speed_loop(name) == name
    assert vehicle.check_speed_loop(' Firm ') == 'firm'
    with pytest.raises(ValueError, match='pick one of'):
        vehicle.check_speed_loop('fast')
    # The two MPCs model the PI only; mppi_lag and the rest take the LQI too.
    for tracker in vehicle.PI_ONLY_TRACKERS:
        with pytest.raises(ValueError, match='PI'):
            vehicle.check_speed_loop('lqi', tracker)
        assert vehicle.check_speed_loop('firm', tracker) == 'firm'
    assert vehicle.check_speed_loop('lqi', 'mppi_lag') == 'lqi'


def test_the_models_loop_is_the_cars_unless_given():
    # speed_loop_model:= (TODO 163): empty follows the car's loop; given, it wins.
    assert vehicle.check_speed_loop_model('', 'firm') == 'firm'
    assert vehicle.check_speed_loop_model('  ', 'default') == 'default'
    assert vehicle.check_speed_loop_model('Smooth', 'firm') == 'smooth'
    with pytest.raises(ValueError, match='speed_loop_model:=fast'):
        vehicle.check_speed_loop_model('fast', 'firm')
    with pytest.raises(ValueError, match='speed_loop:=fast'):
        vehicle.check_speed_loop_model('', 'fast')
    # The MPCs refuse the LQI as a model, even when the car runs it.
    with pytest.raises(ValueError, match='PI'):
        vehicle.check_speed_loop_model('lqi', 'firm', 'mpc_lag')
    assert vehicle.check_speed_loop_model('smooth', 'lqi', 'mpc_lag') == 'smooth'


def test_every_model_of_the_car_gets_the_speed_loop_asked_for():
    # TODO 163: the followers' and the behaviour planner's models predict the simulated
    # car's loop; default unchanged, an unknown name refused.
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    cfg = vehicle.load(CONFIG)
    ego = yaml.safe_load(EGO_YAML.read_text())['ego']['ros__parameters']
    makers = {
        'mpc_lag.speed_loop': (vehicle.track_params, vehicle.mpc_lag_rs_params),
        'speed_loop': (vehicle.mppi_lag_params, vehicle.behaviour_params),
    }
    for key, fns in makers.items():
        for fn in fns:
            assert fn(nav2, cfg, ego)[key] == 'default', fn.__name__
            assert fn(nav2, cfg, ego, speed_loop='firm')[key] == 'firm', fn.__name__
            with pytest.raises(ValueError):
                fn(nav2, cfg, ego, speed_loop='fast')


def test_behaviour_params_from_nav2_the_body_and_ego():
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    cfg = vehicle.load(CONFIG)
    ego = yaml.safe_load(EGO_YAML.read_text())['ego']['ros__parameters']
    server = nav2['controller_server']['ros__parameters']
    p = vehicle.behaviour_params(nav2, cfg, ego)
    assert p['base_speed'] == server['MPPI']['vx_max']
    assert vehicle.behaviour_params(nav2, cfg, ego, controller='rpp')['base_speed'] == \
        server['FollowPath']['desired_linear_vel']
    assert vehicle.behaviour_params(nav2, cfg, ego, max_speed=3.0)['base_speed'] == 3.0
    # The stop zone's own front edge, width and reach.
    zone = vehicle.stop_zone_points(cfg['vehicle'])
    assert p['front'] == pytest.approx(zone[2][0])
    assert p['half_width'] == pytest.approx(zone[0][1])
    assert p['front'] + p['stop_gap'] == pytest.approx(zone[0][0])
    assert p['max_deceleration'] == ego['max_deceleration'] and p['max_jerk'] == ego['max_jerk']


def test_mppi_lag_params_are_nav2s_mppi_and_the_cars_lag():
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    cfg = vehicle.load(CONFIG)
    ego = yaml.safe_load(EGO_YAML.read_text())['ego']['ros__parameters']
    p = vehicle.mppi_lag_params(nav2, cfg, ego)
    server = nav2['controller_server']['ros__parameters']
    mppi = server['MPPI']
    assert 'mppi_lag' in vehicle.TRACKERS
    assert p['critics'] == mppi['critics']
    for name in ('time_steps', 'batch_size', 'temperature', 'gamma', 'vx_std'):
        assert p[name] == mppi[name], name
    assert isinstance(p['time_steps'], int) and isinstance(p['model_dt'], float)
    assert p['model_dt'] == pytest.approx(1.0 / p['rate_hz'])
    assert p['vx_max'] == mppi['vx_max'] and p['vx_min'] == 0.0
    assert p['GoalAngleCritic.threshold_to_consider'] == 2.5
    assert p['PathAlignCritic.use_path_orientations'] is True
    assert p['xy_goal_tolerance'] == server['general_goal_checker']['xy_goal_tolerance']
    # Every actuator number from vehicle.yaml; the unset dead times are 0.
    for key, value in cfg['actuators'].items():
        if key in vehicle.MPPI_LAG_ACTUATORS:
            assert p[f'actuators.{key}'] == value, key
    assert p['actuators.steer_dead_time'] == cfg['actuators'].get('steer_dead_time', 0.0)
    assert p['ego.max_steering_rate'] == ego['max_steering_rate']
    assert len(p['footprint']) == 8
    # controller_server's 0.3 s wait for a current costmap, plus the local one's 2 Hz.
    assert p['costmap_timeout'] == pytest.approx(0.8)
    # sim_speed:= 3: faster, and the path ahead stretched to fit 2.8 s of it.
    fast = vehicle.mppi_lag_params(nav2, cfg, ego, max_speed=3.0)
    assert fast['vx_max'] == 3.0
    assert fast['prune_distance'] >= 3.0 * 2.8
    # Forward only by default: no cusps to stop at, PathAngle's own mode.
    assert p['enforce_path_inversion'] is False and 'PathAngleCritic.mode' not in p
    # Reversing, as nav2_tuned sets Nav2's MPPI up.
    back = vehicle.mppi_lag_params(nav2, cfg, ego, reversing=True)
    tuned = vehicle.nav2_tuned(nav2, cfg['vehicle'], reversing=True, controller='mppi')
    follow = tuned['controller_server']['ros__parameters']['FollowPath']
    assert back['vx_min'] == pytest.approx(-cfg['vehicle']['max_reverse_speed'])
    assert back['vx_min'] == pytest.approx(follow['vx_min'])
    assert back['enforce_path_inversion'] is follow['enforce_path_inversion'] is True
    assert back['critics'] == follow['critics'] and 'PreferForwardCritic' not in back['critics']
    assert back['PathAngleCritic.mode'] == follow['PathAngleCritic']['mode'] == 2
    assert isinstance(back['PathAngleCritic.mode'], int)
    assert back['inversion_xy_tolerance'] == mppi['inversion_xy_tolerance']
    assert back['inversion_yaw_tolerance'] == mppi['inversion_yaw_tolerance']
    # No lane map: Nav2's critics only, the road ahead not read.
    assert 'LaneCritic' not in p['critics'] and p['road_ahead_topic'] == ''
    assert not any(k.startswith('LaneCritic.') for k in p)
    # With one (behaviour:=true lane_map:=...): our LaneCritic too, on the road ahead.
    lane = vehicle.mppi_lag_params(nav2, cfg, ego, lane=True)
    assert lane['critics'] == mppi['critics'] + ['LaneCritic']
    assert lane['road_ahead_topic'] == '/world/road_ahead'
    assert lane['LaneCritic.hard_weight'] > 5 * lane['LaneCritic.soft_weight']
    assert isinstance(lane['LaneCritic.cost_power'], int)
    both = vehicle.mppi_lag_params(nav2, cfg, ego, reversing=True, lane=True)
    assert both['critics'] == back['critics'] + ['LaneCritic']


def test_costmap_params_read_nav2s_costmaps_as_nav2_does():
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    tuned = vehicle.nav2_tuned(nav2, v, radar=False, lidar=True)
    p = vehicle.costmap_params(tuned, v)
    local, plan = p['local'], p['global']
    assert local['rolling'] and local['global_frame'] == 'odom'
    assert (local['width'], local['resolution'], local['update_frequency']) == (20.0, 0.1, 5.0)
    assert local['footprint'] == vehicle.footprint_points(v)
    assert local['inflation'] == {'radius': 2.0, 'scaling': 3.0}
    assert local['transform_tolerance'] == 0.3   # Nav2's default: nav2.yaml sets none
    # The radar's and the cameras' layers are off unless asked for; the lidar's is on.
    names = [layer['name'] for layer in local['layers']]
    assert names == ['lidar_layer']
    lidar = local['layers'][0]['sources'][0]
    assert lidar['topic'] == '/sensors/lidar/scan' and lidar['inf_is_valid'] is True
    assert (lidar['obstacle_max_range'], lidar['raytrace_max_range']) == (12.0, 15.0)
    assert not plan['rolling'] and plan['track_unknown'] and plan['static_layer']
    # The planner sees the lidar too (plan_around, the default).
    assert [layer['name'] for layer in plan['layers']] == ['lidar_layer']
    with_radar = vehicle.costmap_params(vehicle.nav2_tuned(nav2, v, radar=True), v, radar=True)
    radar = with_radar['local']['layers'][0]
    assert radar['name'] == 'obstacle_layer'
    assert radar['sources'][0]['observation_persistence'] == 0.2
    # The radar's view clears what it no longer sees (TODO 193); the lidar's scan has its inf.
    assert radar['sources'][0]['view_angle'] == pytest.approx(math.radians(120.0), abs=1e-3)
    assert lidar['view_angle'] == 0.0
    json.dumps(p)   # one JSON parameter each


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


def test_planners_ours_by_default_and_on_the_car():
    # Nav2's planner_server and its planners went on 17 Nov: ours, forward or reversing.
    assert vehicle.DEFAULT_PLANNER == 'own'
    assert vehicle.default_planner(reversing=True) == 'own'
    assert vehicle.default_planner(reversing=False) == 'own'
    assert vehicle.CAR_PLANNERS == ('own',)
    assert set(vehicle.CAR_PLANNERS) < set(vehicle.PLANNERS)
    assert set(vehicle.PLANNERS) <= set(vehicle.PLANNER_BLOCKS)
    assert vehicle.check_planner(' Own ') == 'own'
    assert vehicle.check_planner('own', real=True) == 'own'
    for gone in ('lattice', 'hybrid', 'smac2d', 'navfn', 'theta_star'):
        with pytest.raises(ValueError, match='pick one of'):
            vehicle.check_planner(gone)
    with pytest.raises(ValueError, match='a trial'):
        vehicle.check_planner('diffusion', real=True)


def test_every_planner_has_its_block_in_nav2_yaml():
    plans = yaml.safe_load(NAV2_YAML.read_text())['planner_server']['ros__parameters']
    for planner, block in vehicle.PLANNER_BLOCKS.items():
        assert 'plugin' in plans[block], planner


@pytest.mark.parametrize('planner', vehicle.PLANNERS)
def test_nav2_tuned_puts_the_planner_under_gridbased(planner):
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    lattice = '/x/lattice.json' if vehicle.server_planner(planner) == 'lattice' else None
    plans = vehicle.nav2_tuned(nav2, v, planner=planner,
                               lattice_file=lattice)['planner_server']['ros__parameters']
    want = nav2['planner_server']['ros__parameters'][vehicle.PLANNER_BLOCKS[planner]]
    assert plans['GridBased']['plugin'] == want['plugin']
    assert set(plans) & set(vehicle.PLANNER_BLOCKS.values()) == {'GridBased'}
    # The search model is Hybrid's alone: a whole-path rewrite would add it to any other.
    assert (SEARCH_MODEL in vehicle.nav2_rewrites(v, planner=planner)) == (planner == 'hybrid')
    # nav2.yaml itself is left as it was.
    assert 'Lattice' in nav2['planner_server']['ros__parameters']


def test_the_diffusion_trial_stands_on_the_lattice():
    # planner:=diffusion: our planner runs the lattice beside it, the fallback.
    assert vehicle.server_planner('diffusion') == 'lattice'
    assert 'diffusion' not in vehicle.CAR_PLANNERS
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    grid = vehicle.nav2_tuned(nav2, v, reversing=True, planner='diffusion',
                              lattice_file='/x/l.json')['planner_server'][
                                  'ros__parameters']['GridBased']
    assert grid['plugin'].endswith('SmacPlannerLattice') and grid['allow_reverse_expansion']


def test_our_planner_reads_the_lattice_block_the_car_and_the_inflation():
    # planner:=own: aav_planner gets Nav2's own Lattice block, so both plan alike.
    assert vehicle.server_planner('own') == 'lattice'
    assert 'own' in vehicle.CAR_PLANNERS
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    for reversing in (False, True):
        tuned = vehicle.nav2_tuned(nav2, v, reversing=reversing, planner='own',
                                   lattice_file='/x/l.json')
        p = vehicle.planner_params(tuned, v)
        lattice = nav2['planner_server']['ros__parameters']['Lattice']
        assert p['lattice_filepath'] == '/x/l.json'
        assert p['allow_reverse_expansion'] is reversing
        for key in ('reverse_penalty', 'change_penalty', 'non_straight_penalty',
                    'cost_penalty', 'retrospective_penalty', 'tolerance',
                    'analytic_expansion_max_length', 'smooth_path'):
            assert p[key] == lattice[key], key
        assert p['footprint'] == vehicle.footprint_points(v)
        inflation = nav2['global_costmap']['global_costmap']['ros__parameters'][
            'inflation_layer']
        assert p['inflation'] == {'radius': inflation['inflation_radius'],
                                  'scaling': inflation['cost_scaling_factor']}
        # planner_server's 1 s wait for a current costmap, plus the global one's 1 Hz.
        assert p['costmap_timeout'] == pytest.approx(2.0)
    assert set(vehicle.GLOBAL_COSTMAP_TOPICS) == set(vehicle.COSTMAPS)


def test_our_hybrid_a_star_reads_nav2s_hybrid_block_the_radius_and_the_cells():
    # planner_search:=hybrid: the GridBased block (Hybrid-A*), not the lattice's, as
    # nav2_rewrites writes it for planner:=hybrid.
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    hybrid = nav2['planner_server']['ros__parameters']['GridBased']
    for reversing, model in ((False, 'DUBIN'), (True, 'REEDS_SHEPP')):
        tuned = vehicle.nav2_tuned(nav2, v, reversing=reversing, planner='own',
                                   lattice_file='/x/l.json')
        p = vehicle.planner_params(tuned, v, search='hybrid', nav2=nav2)
        assert p['search'] == 'hybrid'
        assert p['motion_model_for_search'] == model
        assert p['minimum_turning_radius'] == pytest.approx(vehicle.plan_turning_radius(v),
                                                            abs=1e-3)
        for key in ('reverse_penalty', 'change_penalty', 'non_straight_penalty',
                    'cost_penalty', 'angle_quantization_bins', 'smooth_path', 'tolerance'):
            assert p[key] == hybrid[key], key
        assert 'lattice_filepath' not in p
        assert p['resolution'] == nav2['global_costmap']['global_costmap'][
            'ros__parameters']['resolution']
        assert p['footprint'] == vehicle.footprint_points(v)
    with pytest.raises(ValueError, match='needs nav2.yaml'):
        vehicle.planner_params(tuned, v, search='hybrid')
    assert vehicle.check_planner_search('') == 'lattice'
    with pytest.raises(ValueError, match='planner:=own'):
        vehicle.check_planner_search('hybrid', 'lattice')
    with pytest.raises(ValueError, match='pick one of'):
        vehicle.check_planner_search('rrt')


def test_lattice_takes_its_file_and_reverses_only_with_reversing():
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    for reversing in (False, True):
        grid = vehicle.nav2_tuned(nav2, v, reversing=reversing, planner='own',
                                  lattice_file='/x/l.json')['planner_server'][
                                      'ros__parameters']['GridBased']
        assert grid['lattice_filepath'] == '/x/l.json'
        assert grid['allow_reverse_expansion'] is reversing


def test_smooth_moves_have_their_own_file_and_turn_the_smoother_off():
    assert vehicle.lattice_file_name(16, 'nav2') == 'lattice_ackermann.json'
    assert vehicle.lattice_file_name(16, 'smooth') == 'lattice_ackermann_smooth.json'
    assert vehicle.lattice_file_name(16, '') == 'lattice_ackermann.json'
    assert vehicle.lattice_file_name(16, 'smooth_fast') == 'lattice_ackermann_smooth_fast.json'
    with pytest.raises(ValueError, match='16 headings only'):
        vehicle.lattice_file_name(32, 'smooth')
    with pytest.raises(ValueError, match='pick one of'):
        vehicle.lattice_file_name(16, 'wiggly')
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']

    def grid(moves):
        return vehicle.nav2_tuned(nav2, v, planner='own', lattice_file='/x/l.json',
                                  lattice_moves=moves)['planner_server'][
                                      'ros__parameters']['GridBased']
    # Nav2's moves keep nav2.yaml's smoother; the smooth ones would get its Dubins ends back.
    assert grid('nav2')['smooth_path'] is True
    assert grid('smooth')['smooth_path'] is False
    # Our planner (planner:=own) reads the same block.
    tuned = vehicle.nav2_tuned(nav2, v, planner='own', lattice_file='/x/l.json',
                               lattice_moves='smooth')
    assert vehicle.planner_params(tuned, v)['smooth_path'] is False


def test_the_committed_lattice_fits_the_car():
    v = vehicle.load(CONFIG)['vehicle']
    meta = json.loads(LATTICE_JSON.read_text())['lattice_metadata']
    assert vehicle.lattice_problems(meta, v, 0.25) == []
    wider = dict(v, max_steering_angle=v['max_steering_angle'] * 0.8)
    assert 'turning radius' in vehicle.lattice_problems(meta, wider, 0.25)[0]
    assert 'cells' in vehicle.lattice_problems(meta, v, 0.05)[0]


def test_mppi_keeps_enough_path_for_its_futures_at_a_higher_speed():
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    tuned = vehicle.nav2_tuned(nav2, v, max_speed=2.5, controller='mppi')
    follow = tuned['controller_server']['ros__parameters']['FollowPath']
    ahead = follow['time_steps'] * follow['model_dt'] * follow['vx_max']
    assert follow['vx_max'] == 2.5
    assert follow['prune_distance'] > ahead


def test_the_diffusion_car_trial_needs_its_switch_and_a_low_speed():
    # Refused on the car without the switch; with it, the diffusion planner and only it.
    with pytest.raises(ValueError, match='diffusion_trial'):
        vehicle.check_planner('diffusion', real=True)
    assert vehicle.check_planner('diffusion', real=True, trial=True) == 'diffusion'
    with pytest.raises(ValueError, match='planner:=diffusion only'):
        vehicle.check_planner('own', real=True, trial=True)
    assert vehicle.check_planner('diffusion') == 'diffusion'        # sim and bench, as before
    # The speed: walking pace, the reverse limit, below ego's own cap.
    v = vehicle.load(CONFIG)['vehicle']
    assert vehicle.DIFFUSION_TRIAL_SPEED <= float(v['max_reverse_speed'])
    assert vehicle.diffusion_trial_speed('') == vehicle.DIFFUSION_TRIAL_SPEED
    assert vehicle.diffusion_trial_speed(' 0.3 ') == 0.3
    for bad in ('0.51', '1.0', '0', '-1', 'fast'):
        with pytest.raises(ValueError, match='max_speed'):
            vehicle.diffusion_trial_speed(bad)


def test_lidar_memory_sets_the_local_lidar_sources_only():
    # TODO 188: the stock blind-zone memory, on the lidar's marking sources.
    path = (Path(__file__).resolve().parents[3] / 'drivers' / 'aav_navigation' / 'config'
            / 'nav2.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    layer = cfg['local_costmap']['local_costmap']['ros__parameters']['lidar_layer']
    assert layer['lidar']['observation_persistence'] == 0.0
    vehicle.with_lidar_memory(cfg, 3.0)
    assert layer['lidar']['observation_persistence'] == 3.0
    assert layer['voxels']['observation_persistence'] == 3.0
    assert layer['voxels_ground']['observation_persistence'] == 0.0     # clears only
    with pytest.raises(ValueError):
        vehicle.with_lidar_memory(cfg, -1.0)


def test_a_still_car_sends_a_nearly_certain_zero_and_a_moving_one_unchanged():
    # TODO 191: stops reset the drift.
    cov = [0.0] * 36
    cov[0], cov[7], cov[35] = 0.01, 1e-4, 1e3
    still = vehicle.standstill_covariance(0.0, cov)
    assert [still[i] for i in (0, 7, 35)] == [vehicle.STANDSTILL_VARIANCE] * 3
    assert vehicle.standstill_covariance(0.5, cov) == cov
    assert vehicle.standstill_covariance(math.nan, cov) == cov
    assert cov[35] == 1e3                  # the caller's list is left alone


def test_a_global_window_rolls_the_global_costmap_and_0_leaves_it():
    """TODO 175 step 3: the global costmap as a 192 m square round the car."""
    def cfg():
        return {'global_costmap': {'global_costmap': {'ros__parameters': {
            'rolling_window': False, 'resolution': 0.25}}}}
    block = vehicle.with_global_window(cfg(), 192.0)['global_costmap']['global_costmap'][
        'ros__parameters']
    assert block == {'rolling_window': True, 'width': 192.0, 'height': 192.0,
                     'resolution': 0.25}
    assert vehicle.with_global_window(cfg(), 0.0) == cfg()
    with pytest.raises(ValueError):
        vehicle.with_global_window(cfg(), -1.0)


def test_the_chunks_block_needs_a_chunks_folder(tmp_path):
    """TODO 175 step 3: our global costmap from the map's chunks (costmap_chunks:=)."""
    with pytest.raises(ValueError):
        vehicle.costmap_chunks_block(str(tmp_path))
    (tmp_path / 'chunks.yaml').write_text('origin: [0, 0]\nsize: 64\n')
    block = vehicle.costmap_chunks_block(str(tmp_path))
    assert block == {'dir': str(tmp_path), 'ahead': vehicle.CHUNKS_AHEAD,
                     'margin': vehicle.CHUNKS_MARGIN}
    with pytest.raises(ValueError):
        vehicle.costmap_chunks_block(str(tmp_path), margin=-1)


def test_plan_smoothing_strong_weights_the_smoother_for_both_planners():
    """TODO 168: plan_smoothing:=strong lowers w_data in GridBased's smoother block."""
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    v = vehicle.load(CONFIG)['vehicle']
    tuned = vehicle.nav2_tuned(nav2, v, planner='own', lattice_file='/x/l.json')
    assert 'smoother' not in vehicle.planner_params(tuned, v)      # nav2: Nav2's defaults
    assert vehicle.with_plan_smoothing(copy.deepcopy(tuned), 'nav2') == tuned
    strong = vehicle.with_plan_smoothing(tuned, 'strong')
    block = strong['planner_server']['ros__parameters']['GridBased']['smoother']
    assert block == vehicle.PLAN_SMOOTHINGS['strong']
    assert block['w_data'] < 0.2                                  # Nav2's own w_data
    assert vehicle.planner_params(strong, v)['smoother'] == block  # ours reads it too
    assert vehicle.check_plan_smoothing('') == 'nav2'
    with pytest.raises(ValueError):
        vehicle.check_plan_smoothing('soft')


def test_mppi_lag_steer_gamma_is_a_number_0_or_more():
    assert vehicle.check_steer_gamma('0.05') == pytest.approx(0.05)
    for bad in ('x', '-1', 'nan'):
        with pytest.raises(ValueError):
            vehicle.check_steer_gamma(bad)


def test_mppi_lag_batch_is_a_whole_number_1_or_more():
    # TODO 152: steer:=mppi_lag_1k's 1000 samples a tick.
    assert vehicle.check_mppi_batch('1000') == 1000
    for bad in ('x', '0', '-5', '1.5'):
        with pytest.raises(ValueError, match='mppi_lag_batch'):
            vehicle.check_mppi_batch(bad)


def test_the_stop_zone_reads_the_collision_monitor_block():
    """TODO 182: our stop zone's settings from the tuned nav2.yaml and the car."""
    share = Path(__file__).resolve().parents[3] / 'drivers' / 'aav_navigation' / 'config'
    car = vehicle.load(CONFIG)
    tuned = vehicle.nav2_tuned(yaml.safe_load((share / 'nav2.yaml').read_text()),
                               car['vehicle'], False, None, True, True)
    p = vehicle.stop_zone_params(tuned, car)
    assert [b['name'] for b in p['bands']] == ['forward', 'backward']
    assert p['bands'][0]['points'] == vehicle.stop_zone_points(car['vehicle'])
    assert p['bands'][0]['speed'] == [0.0, 10.0]
    assert (p['min_points'], p['source_timeout'], p['stop_pub_timeout']) == (3, 1.0, 2.0)
    names = [src['name'] for src in p['sources']]
    assert names == ['radar', 'lidar']
    radar = p['sources'][0]
    assert radar['kind'] == 'pointcloud' and radar['min_range'] == 0.2
    assert radar['mount'][:3] == [float(x) for x in car['sensors']['radar_link']['xyz']]
    assert p['sources'][1]['kind'] == 'scan' and p['sources'][1]['mount'] == [0.0] * 4


def test_ego_impl_is_python_or_rust():
    # Rust on the car and in the simulator (Ben, 16 Nov); the Python stays selectable.
    assert vehicle.DEFAULT_EGO_IMPL == vehicle.SIM_DEFAULT_EGO_IMPL == 'rust'
    assert vehicle.check_ego_impl('python') == 'python'
    assert vehicle.check_ego_impl(' Rust ') == 'rust'
    assert vehicle.EGO_IMPLS['rust'] == 'aav_ego_rs'
    with pytest.raises(ValueError, match='ego_impl:=c'):
        vehicle.check_ego_impl('c')


def test_behaviour_impl_is_rust():
    assert vehicle.DEFAULT_BEHAVIOUR_IMPL == vehicle.SIM_DEFAULT_BEHAVIOUR_IMPL == 'rust'
    assert vehicle.check_behaviour_impl(' Rust ') == 'rust'
    assert vehicle.BEHAVIOUR_IMPLS == {'rust': 'aav_behaviour_rs'}
    with pytest.raises(ValueError, match='behaviour_impl:=python'):
        vehicle.check_behaviour_impl('python')
    with pytest.raises(ValueError, match='behaviour_impl:=c'):
        vehicle.check_behaviour_impl('c')


def test_watchdog_impl_is_rust():
    assert vehicle.DEFAULT_WATCHDOG_IMPL == 'rust'
    assert vehicle.check_watchdog_impl(' Rust ') == 'rust'
    assert vehicle.WATCHDOG_IMPLS == {'rust': 'aav_watchdog_rs'}
    with pytest.raises(ValueError, match='watchdog_impl:=python'):
        vehicle.check_watchdog_impl('python')
    with pytest.raises(ValueError, match='watchdog_impl:=c'):
        vehicle.check_watchdog_impl('c')


def test_io_mux_impl_is_python_or_rust():
    assert vehicle.DEFAULT_IO_MUX_IMPL == 'rust'
    assert vehicle.check_io_mux_impl('python') == 'python'
    assert vehicle.check_io_mux_impl(' Rust ') == 'rust'
    assert vehicle.IO_MUX_IMPLS == {'python': 'aav_io_mux', 'rust': 'aav_io_mux_rs'}
    with pytest.raises(ValueError, match='io_mux_impl:=c'):
        vehicle.check_io_mux_impl('c')


def test_legacy_and_rc_impls_are_python_or_rust():
    # Ben, 17 Nov: "Rust tomorrow": the UART bridge and rc_driver; *_impl:=python falls back.
    assert vehicle.DEFAULT_LEGACY_IMPL == vehicle.DEFAULT_RC_IMPL == 'rust'
    assert vehicle.LEGACY_IMPLS == {'python': 'aav_legacy', 'rust': 'aav_legacy_rs'}
    assert vehicle.RC_IMPLS == {'python': 'aav_rc', 'rust': 'aav_rc_rs'}
    assert vehicle.check_legacy_impl(' Python ') == 'python'
    assert vehicle.check_rc_impl('rust') == 'rust'
    with pytest.raises(ValueError, match='legacy_impl:=c'):
        vehicle.check_legacy_impl('c')
    with pytest.raises(ValueError, match='rc_impl:=c'):
        vehicle.check_rc_impl('c')


def test_lane_finder_impl_is_rust_or_python():
    # Ben, 18 Nov: the lane finder in Rust, the default; lane_finder_impl:=python falls back.
    assert vehicle.DEFAULT_LANE_FINDER_IMPL == 'rust'
    assert vehicle.LANE_FINDER_IMPLS == {'python': 'aav_lane_finder',
                                         'rust': 'aav_lane_finder_rs',
                                         'learned': 'aav_lane_finder_learned'}
    assert vehicle.check_lane_finder_impl('learned') == 'learned'
    assert vehicle.check_lane_finder_impl(' Python ') == 'python'
    with pytest.raises(ValueError, match='lane_finder_impl:=c'):
        vehicle.check_lane_finder_impl('c')


def test_camera_stream_impl_is_rust_unless_a_model_runs():
    # Ben, 18 Nov: camera_stream in Rust, the default; camera_stream_impl:=python falls back.
    assert vehicle.DEFAULT_CAMERA_STREAM_IMPL == 'rust'
    assert vehicle.CAMERA_STREAM_IMPLS == {'python': 'aav_camera_stream',
                                           'rust': 'aav_camera_stream_rs'}
    assert vehicle.check_camera_stream_impl(' Rust ') == 'rust'
    with pytest.raises(ValueError, match='camera_stream_impl:=c'):
        vehicle.check_camera_stream_impl('c')
    assert vehicle.camera_stream_runs('rust', 'none', 'gst-nvidia') == ('rust', '')
    assert vehicle.camera_stream_runs('python', 'none', 'ffmpeg') == ('python', '')
    runs, why = vehicle.camera_stream_runs('rust', 'rfdetr', 'gst-nvidia')
    assert runs == 'python' and 'detector:=rfdetr' in why
    runs, why = vehicle.camera_stream_runs('rust', 'None', 'opencv')
    assert runs == 'python' and 'opencv' in why


def test_localization_health_impl_is_python_or_rust():
    assert vehicle.DEFAULT_LOCALIZATION_HEALTH_IMPL == 'rust'
    assert vehicle.SIM_DEFAULT_LOCALIZATION_HEALTH_IMPL == 'rust'
    assert vehicle.check_localization_health_impl(' Rust ') == 'rust'
    assert vehicle.LOCALIZATION_HEALTH_IMPLS == {'rust': 'aav_localization_health_rs'}
    with pytest.raises(ValueError, match='localization_health_impl:=python'):
        vehicle.check_localization_health_impl('python')
    with pytest.raises(ValueError, match='localization_health_impl:=c'):
        vehicle.check_localization_health_impl('c')


def test_ekf_control_term_follows_the_command_within_the_cars_limits():
    cfg = vehicle.load(CONFIG)
    p = vehicle.ekf_control_params(cfg)
    assert p['use_control'] and p['stamped_control']
    assert p['control_config'] == [True, False, False, False, False, False]
    for key in ('control_config', 'acceleration_limits', 'acceleration_gains',
                'deceleration_limits', 'deceleration_gains'):
        assert len(p[key]) == 6, key
    assert p['acceleration_limits'][0] == 0.0    # it only ever slows the prediction
    assert p['deceleration_limits'][0] == cfg['actuators']['max_brake']
    assert 0.0 < p['control_timeout'] <= 1.0


def test_the_navigator_replans_at_replan_hz_and_once_at_zero():
    v = vehicle.load(CONFIG)['vehicle']
    assert vehicle.navigator_params(v, False)['replan_period'] == 1.0 / vehicle.REPLAN_HZ
    assert vehicle.navigator_params(v, False, replan_hz=0.5)['replan_period'] == 2.0
    once = vehicle.navigator_params(v, False, replan_hz=0)['replan_period']
    assert once == 1.0 / vehicle.REPLAN_ONCE_HZ
    with pytest.raises(ValueError, match='0 .plan once. or more'):
        vehicle.navigator_params(v, False, replan_hz=-1)


def test_rewrite_follows_rewrittenyamls_rules():
    # Nav2's RewrittenYaml, ours (no ROS): a leaf name everywhere, a dotted path once.
    cfg = {'a': {'footprint': 'x', 'b': {'footprint': 'y', 'n': 1}}, 'c': [{'footprint': 'z'}]}
    out = vehicle.rewrite(cfg, {'footprint': '[[1, 2]]', 'a.b.n': '2.5', 'a.new.flag': 'true'})
    assert out['a']['footprint'] == out['a']['b']['footprint'] == '[[1, 2]]'
    assert out['c'][0]['footprint'] == '[[1, 2]]'
    assert out['a']['b']['n'] == 2.5 and out['a']['new']['flag'] is True
    assert cfg['a']['footprint'] == 'x'          # a copy


def test_gps_heading_impl_is_rust():
    assert vehicle.DEFAULT_GPS_HEADING_IMPL == 'rust'
    assert vehicle.check_gps_heading_impl(' Rust ') == 'rust'
    with pytest.raises(ValueError, match='gps_heading_impl:=python'):
        vehicle.check_gps_heading_impl('python')
    assert vehicle.GPS_HEADING_IMPLS['rust'] == 'aav_gps_heading_rs'
    with pytest.raises(ValueError, match='gps_heading_impl:=c'):
        vehicle.check_gps_heading_impl('c')


def test_safety_impls_are_rust():
    for name, check, default in (
            ('geofence_impl', vehicle.check_geofence_impl, vehicle.DEFAULT_GEOFENCE_IMPL),
            ('stop_hold_impl', vehicle.check_stop_hold_impl, vehicle.DEFAULT_STOP_HOLD_IMPL),
            ('goal_leash_impl', vehicle.check_goal_leash_impl,
             vehicle.DEFAULT_GOAL_LEASH_IMPL)):
        assert default == 'rust'
        assert check(' Rust ') == 'rust'
        with pytest.raises(ValueError, match=f'{name}:=python'):
            check('python')
    assert vehicle.GEOFENCE_IMPLS == {'rust': 'aav_safety_rs'}


def test_joy_alive_impl_is_rust():
    assert vehicle.DEFAULT_JOY_ALIVE_IMPL == 'rust'
    assert vehicle.check_joy_alive_impl(' Rust ') == 'rust'
    assert vehicle.JOY_ALIVE_IMPLS == {'rust': 'aav_joy_alive_rs'}
    with pytest.raises(ValueError, match='joy_alive_impl:=python'):
        vehicle.check_joy_alive_impl('python')
    with pytest.raises(ValueError, match='joy_alive_impl:=c'):
        vehicle.check_joy_alive_impl('c')


def test_stop_sign_impl_is_python_or_rust():
    assert vehicle.DEFAULT_STOP_SIGN_IMPL == 'rust'
    assert vehicle.check_stop_sign_impl('PYTHON') == 'python'
    assert vehicle.STOP_SIGN_IMPLS == {'python': 'aav_detector', 'rust': 'aav_stop_sign_rs'}
    with pytest.raises(ValueError, match='stop_sign_impl:=c'):
        vehicle.check_stop_sign_impl('c')


def test_drive_to_point_and_arm_when_open_impls_are_python_or_rust():
    assert vehicle.DEFAULT_DRIVE_TO_POINT_IMPL == vehicle.DEFAULT_ARM_WHEN_OPEN_IMPL == 'rust'
    assert vehicle.DRIVE_TO_POINT_IMPLS == {'python': 'aav_drive_to_point',
                                            'rust': 'aav_drive_to_point_rs'}
    assert vehicle.ARM_WHEN_OPEN_IMPLS == {'python': 'aav_spine', 'rust': 'aav_spine_bridge'}
    assert vehicle.check_drive_to_point_impl(' Python ') == 'python'
    assert vehicle.check_arm_when_open_impl('rust') == 'rust'
    with pytest.raises(ValueError, match='drive_to_point_impl:=c'):
        vehicle.check_drive_to_point_impl('c')
    with pytest.raises(ValueError, match='arm_when_open_impl:=c'):
        vehicle.check_arm_when_open_impl('c')
