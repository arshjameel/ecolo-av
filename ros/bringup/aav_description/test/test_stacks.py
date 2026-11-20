"""The league: stacks.yaml checks, each stack's switches, and a hand-set switch winning."""

import os
import re

from aav_description import lane_map, stacks, vehicle
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FILE = os.path.join(HERE, '..', 'config', 'stacks.yaml')
SIM_LAUNCH = os.path.join(HERE, '..', '..', 'aav_bringup', 'launch', 'sim.launch.py')
LEAGUE = stacks.load(FILE)
# Each switch's fallback in sim.launch.py (STACK_FALLBACK): the champions', Nav2's.
DEFAULTS = {'controller': vehicle.DEFAULT_CONTROLLER, 'tracker': vehicle.DEFAULT_TRACKER,
            'planner': vehicle.DEFAULT_PLANNER, 'costmap': vehicle.DEFAULT_COSTMAP,
            'navigator': vehicle.DEFAULT_NAVIGATOR, 'smoother': vehicle.DEFAULT_SMOOTHER,
            'behaviour': 'false', 'ndt_ekf': 'false', 'kiss_ekf': 'false',
            'lattice_moves': vehicle.LATTICE_MOVES[0],
            'planner_search': vehicle.PLANNER_SEARCHES[0],
            'plan_smoothing': 'nav2', 'map_correction': 'newest', 'mppi_lag_steer_gamma': '',
            'mppi_lag_batch': '',
            'mppi_lag_sampling': '', 'lane_pass_shape': '', 'ground': '',
            'odom_lidar': '', 'things_tracker': '',
            'lane_costs': 'false', 'lane_source': lane_map.DEFAULT_LANE_SOURCE,
            'lane_route': 'false', 'costmap_chunks': 'off',
            'kerbs': 'false', 'things': 'false', 'lane_pose': 'false',
            'lane_graph': lane_map.DEFAULT_LANE_GRAPH, 'local_chunks': 'false'}
# What the launch passes when no switch is given: each declared empty, "the stack's choice".
NOT_GIVEN = {name: stacks.CHOSEN for name in DEFAULTS}
VALID = {'controller': vehicle.CONTROLLERS, 'tracker': vehicle.TRACKERS,
         'planner': vehicle.PLANNERS, 'costmap': vehicle.COSTMAPS,
         'navigator': vehicle.NAVIGATORS, 'smoother': vehicle.SMOOTHERS,
         'behaviour': ('true', 'false'), 'ndt_ekf': ('true', 'false'),
         'kiss_ekf': ('true', 'false'), 'lattice_moves': vehicle.LATTICE_MOVES,
         'planner_search': vehicle.PLANNER_SEARCHES,
         'plan_smoothing': tuple(vehicle.PLAN_SMOOTHINGS), 'map_correction': ('newest', 'slow'),
         'mppi_lag_steer_gamma': ('', '0.05'), 'mppi_lag_batch': ('', '1000'),
         'mppi_lag_sampling': ('',) + vehicle.MPPI_LAG_SAMPLINGS,
         'lane_pass_shape': ('', 'cosine', 'jerk'),
         'ground': ('',) + vehicle.GROUNDS,
         'odom_lidar': ('', 'kiss_icp', 'kinematic_icp'),
         'things_tracker': ('',) + vehicle.THINGS_TRACKERS,
         'lane_costs': ('true', 'false'), 'lane_source': lane_map.LANE_SOURCES,
         'lane_route': ('true', 'false'),
         'costmap_chunks': ('auto', 'off'), 'kerbs': ('true', 'false'),
         'things': ('true', 'false'), 'lane_pose': ('true', 'false'),
         'lane_graph': lane_map.LANE_GRAPHS,
         'local_chunks': ('true', 'false')}


def test_the_four_stacks_are_there():
    # stock (every level Nav2's) went with Nav2 (17 Nov, TODO 184).
    assert set(stacks.names(LEAGUE)) == {'best', 'car', 'ours'}


def test_every_option_sets_real_switches_to_real_values():
    for level in LEAGUE['levels'].values():
        for option in level['options'].values():
            for switch, value in option.items():
                assert switch in VALID, switch
                assert str(value) in VALID[switch], (switch, value)


def test_every_switch_is_an_argument_of_sim_launch():
    with open(SIM_LAUNCH) as f:
        declared = set(re.findall(r"DeclareLaunchArgument\(\s*'(\w+)'", f.read()))
    assert set(DEFAULTS) <= declared
    assert 'stack' in declared


def test_best_is_every_champion_and_today_that_is_the_defaults():
    assert stacks.choices(LEAGUE, 'best') == {
        name: level['champion'] for name, level in LEAGUE['levels'].items()}
    # The champions are the fallback: what stack:=best, or no stack at all, runs.
    assert stacks.resolve(LEAGUE, 'best', NOT_GIVEN, DEFAULTS) == DEFAULTS
    assert stacks.resolve(LEAGUE, stacks.NONE, NOT_GIVEN, DEFAULTS) == DEFAULTS


def test_ours_is_the_default_stack():
    # Ben, 14 Nov: "Ours default now, then delete". The car keeps its own (car.launch.py).
    assert stacks.DEFAULT == 'ours'
    out = stacks.resolve(LEAGUE, stacks.DEFAULT, NOT_GIVEN, DEFAULTS)
    assert out == dict(DEFAULTS, **stacks.switches(LEAGUE, 'ours'))
    assert (out['costmap'], out['navigator'], out['planner'], out['tracker']) == (
        'own', 'own', 'own', 'mppi_lag')


def test_no_level_has_a_nav2_option_left():
    # Nav2 deleted (17 Nov, TODO 184): every champion is its port.
    for name, level in LEAGUE['levels'].items():
        for option in level['options'].values():
            assert option.get('costmap', 'own') == 'own', name
            assert option.get('navigator', 'own') == 'own', name
            assert option.get('smoother', 'own') == 'own', name
            assert option.get('tracker', 'mppi_lag') != 'none', name
            assert option.get('planner', 'own') in ('own', 'diffusion'), name
    best = stacks.resolve(LEAGUE, 'best', NOT_GIVEN, DEFAULTS)
    assert (best['costmap'], best['navigator'], best['planner'], best['tracker'],
            best['smoother']) == ('own', 'own', 'own', 'mppi_lag', 'own')


def test_ours_sets_every_level_we_built():
    s = stacks.switches(LEAGUE, 'ours')
    assert (s['costmap'], s['navigator'], s['planner'], s['tracker']) == (
        'own', 'own', 'own', 'mppi_lag')
    assert s['behaviour'] == 'true'
    # The things too: the behaviour planner's hold for anything moving needs them.
    assert s['things'] == 'true'


def test_the_lanes_layer_is_a_challenger_off_in_every_stack():
    # TODO 148: lane_costs stays off until it wins on the scorecard.
    for stack in stacks.names(LEAGUE):
        assert stacks.switches(LEAGUE, stack).get('lane_costs', 'false') == 'false', stack


def test_ours_global_costmap_is_made_of_chunks():
    # TODO 175 (16 Nov): no worse on the lot, campus and University Drive tours, a third of
    # the memory: ours holds only the chunks it needs; the others the whole map.
    # And its local one too (16 Nov): the campus and the box tour, 2 runs each, every leg,
    # the same times, half the node's memory; no costmap of ours then reads /map.
    assert stacks.choices(LEAGUE, 'ours')['costmap'] == 'ours_all_chunks'
    for stack in stacks.names(LEAGUE):
        want = 'auto' if stack == 'ours' else 'off'
        assert stacks.switches(LEAGUE, stack).get('costmap_chunks', 'off') == want, stack
        want = 'true' if stack == 'ours' else 'false'
        assert stacks.switches(LEAGUE, stack).get('local_chunks', 'false') == want, stack


def test_the_lane_centre_planner_is_ours_only():
    # TODO 148: path:=lane (lane_route) won on the scorecard (16 Nov): ours plans the lane's
    # centre; the champion (the car's) and stock don't.
    assert LEAGUE['levels']['path']['options']['lane']['lane_route'] == 'true'
    assert LEAGUE['levels']['path']['options']['lane']['planner'] == 'own'
    assert stacks.choices(LEAGUE, 'ours')['path'] == 'lane'
    for stack in stacks.names(LEAGUE):
        want = 'true' if stack == 'ours' else 'false'
        assert stacks.switches(LEAGUE, stack).get('lane_route', 'false') == want, stack


def test_ours_reverses():
    # Ben, 14 Nov: "The car can reverse." No stack may pick a forward-only follower.
    for stack in stacks.names(LEAGUE):
        vehicle.check_tracker(stacks.switches(LEAGUE, stack)['tracker'], reversing=True)


def test_a_switch_set_by_hand_wins():
    given = dict(NOT_GIVEN, tracker='mpc_lag')
    out = stacks.resolve(LEAGUE, 'ours', given, DEFAULTS)
    assert out['tracker'] == 'mpc_lag'
    assert out['costmap'] == 'own'


def test_a_switch_set_by_hand_to_its_fallback_still_wins():
    # tracker:=mppi_lag is the fallback's value: still told apart from "not given".
    out = stacks.resolve(LEAGUE, 'ours', dict(NOT_GIVEN, tracker='mppi_lag'), DEFAULTS)
    assert (out['tracker'], out['mppi_lag_steer_gamma']) == ('mppi_lag', '0.05')
    out = stacks.resolve(LEAGUE, 'ours', dict(NOT_GIVEN, tracker='lqr'), DEFAULTS)
    assert (out['costmap'], out['tracker']) == ('own', 'lqr')


def test_no_stack_takes_each_switch_as_given_else_its_fallback():
    out = stacks.resolve(LEAGUE, '', dict(NOT_GIVEN, planner='hybrid'), DEFAULTS)
    assert out == dict(DEFAULTS, planner='hybrid')


def test_unknown_names_are_refused():
    with pytest.raises(ValueError, match='pick one of'):
        stacks.switches(LEAGUE, 'fastest')
    bad = {'levels': {'path': {'champion': 'teleport', 'options': {'lattice': {}}}},
           'stacks': {}}
    with pytest.raises(ValueError, match='champion'):
        stacks.check(bad)
    bad = {'levels': {'path': {'champion': 'lattice', 'options': {'lattice': {}}}},
           'stacks': {'x': {'path': 'teleport'}}}
    with pytest.raises(ValueError, match='teleport'):
        stacks.check(bad)


def test_the_car_stack_keeps_to_the_car_planners():
    assert stacks.switches(LEAGUE, 'car')['planner'] in vehicle.CAR_PLANNERS


def test_describe_names_the_stack():
    assert stacks.describe(LEAGUE, 'ours').startswith('stack:=ours - pose ekf')
    assert 'no stack' in stacks.describe(LEAGUE, '')


def test_a_level_switch_picks_its_option_over_the_stacks():
    # path:=hybrid_rs: our planner's Hybrid-A*; ours' strong smoothing goes back to nav2.
    out = stacks.resolve(LEAGUE, 'ours', NOT_GIVEN, DEFAULTS, {'path': 'hybrid_rs'})
    assert (out['planner'], out['planner_search']) == ('own', 'hybrid')
    assert out['plan_smoothing'] == 'nav2'
    assert out['tracker'] == 'mppi_lag'  # the other levels stay the stack's
    out = stacks.resolve(LEAGUE, 'ours', NOT_GIVEN, DEFAULTS, {'steer': 'mppi_lag'})
    assert (out['controller'], out['tracker']) == ('mppi', 'mppi_lag')
    assert (out['map_correction'], out['mppi_lag_steer_gamma']) == ('newest', '')
    out = stacks.resolve(LEAGUE, 'best', NOT_GIVEN, DEFAULTS, {'pose': 'ndt_ekf'})
    assert (out['ndt_ekf'], out['kiss_ekf'], out['costmap']) == ('true', 'false', 'own')


def test_a_switch_by_hand_still_beats_a_level_switch():
    given = dict(NOT_GIVEN, planner_search='lattice')
    out = stacks.resolve(LEAGUE, 'ours', given, DEFAULTS, {'path': 'hybrid_rs'})
    assert (out['planner'], out['planner_search']) == ('own', 'lattice')


def test_a_level_switch_works_with_no_stack_and_empty_means_the_stacks():
    out = stacks.resolve(LEAGUE, '', NOT_GIVEN, DEFAULTS, {'lanes': 'ramp'})
    assert out == dict(DEFAULTS, lane_costs='true')
    empty = {level: stacks.CHOSEN for level in stacks.LEVEL_SWITCHES}
    assert (stacks.resolve(LEAGUE, 'ours', NOT_GIVEN, DEFAULTS, empty)
            == stacks.resolve(LEAGUE, 'ours', NOT_GIVEN, DEFAULTS))


def test_a_level_switch_is_checked():
    with pytest.raises(ValueError, match='pick one of'):
        stacks.resolve(LEAGUE, 'ours', NOT_GIVEN, DEFAULTS, {'path': 'teleport'})
    with pytest.raises(ValueError, match='not a level'):
        stacks.resolve(LEAGUE, 'ours', NOT_GIVEN, DEFAULTS, {'wings': 'on'})


def test_every_level_switch_is_a_level_and_an_argument_of_sim_launch():
    with open(SIM_LAUNCH) as f:
        launch = f.read()
    for level in stacks.LEVEL_SWITCHES:
        assert level in LEAGUE['levels']
    # Declared in one loop over stacks.LEVEL_SWITCHES, and passed to resolve.
    assert 'for level in stacks.LEVEL_SWITCHES' in launch
    assert '*[DeclareLaunchArgument(level,' in launch
    assert 'STACK_FALLBACK, levels)' in launch


def test_describe_says_what_was_picked_by_hand():
    line = stacks.describe(LEAGUE, 'ours', {'path': 'hybrid_rs', 'steer': ''})
    assert 'path hybrid_rs' in line and line.endswith('(by hand: path:=hybrid_rs)')
    assert stacks.describe(LEAGUE, '', {'lanes': 'ramp'}).endswith('; lanes:=ramp')
