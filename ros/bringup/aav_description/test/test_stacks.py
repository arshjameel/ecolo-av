"""The league: stacks.yaml checks, each stack's switches, and a hand-set switch winning."""

import os
import re

from aav_description import stacks, vehicle
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FILE = os.path.join(HERE, '..', 'config', 'stacks.yaml')
SIM_LAUNCH = os.path.join(HERE, '..', '..', 'aav_bringup', 'launch', 'sim.launch.py')
LEAGUE = stacks.load(FILE)
# Each switch's declared default in sim.launch.py, as the launch file passes them.
DEFAULTS = {'controller': vehicle.DEFAULT_CONTROLLER, 'tracker': vehicle.DEFAULT_TRACKER,
            'planner': vehicle.DEFAULT_PLANNER, 'costmap': vehicle.DEFAULT_COSTMAP,
            'navigator': vehicle.DEFAULT_NAVIGATOR, 'smoother': vehicle.DEFAULT_SMOOTHER,
            'behaviour': 'false', 'ndt_ekf': 'false', 'kiss_ekf': 'false',
            'lattice_moves': vehicle.LATTICE_MOVES[0]}
VALID = {'controller': vehicle.CONTROLLERS, 'tracker': vehicle.TRACKERS,
         'planner': vehicle.PLANNERS, 'costmap': vehicle.COSTMAPS,
         'navigator': vehicle.NAVIGATORS, 'smoother': vehicle.SMOOTHERS,
         'behaviour': ('true', 'false'), 'ndt_ekf': ('true', 'false'),
         'kiss_ekf': ('true', 'false'), 'lattice_moves': vehicle.LATTICE_MOVES}


def test_the_four_stacks_are_there():
    assert set(stacks.names(LEAGUE)) == {'best', 'car', 'stock', 'ours'}


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
    # The champions are what sim runs with no switches at all, until a challenger wins.
    assert stacks.resolve(LEAGUE, 'best', DEFAULTS, DEFAULTS) == DEFAULTS


def test_ours_sets_every_level_we_built():
    s = stacks.switches(LEAGUE, 'ours')
    assert (s['costmap'], s['navigator'], s['planner'], s['tracker']) == (
        'own', 'own', 'own', 'mppi_lag')
    assert s['behaviour'] == 'true'


def test_ours_reverses():
    # Ben, 14 Nov: "The car can reverse." No stack may pick a forward-only follower.
    for stack in stacks.names(LEAGUE):
        vehicle.check_tracker(stacks.switches(LEAGUE, stack)['tracker'], reversing=True)


def test_a_switch_set_by_hand_wins():
    given = dict(DEFAULTS, tracker='mpc_lag')
    out = stacks.resolve(LEAGUE, 'ours', given, DEFAULTS)
    assert out['tracker'] == 'mpc_lag'
    assert out['costmap'] == 'own'


def test_no_stack_changes_nothing():
    given = dict(DEFAULTS, planner='hybrid')
    assert stacks.resolve(LEAGUE, '', given, DEFAULTS) == given


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
