"""The switch picker (tools/pick.py): its switches exist, and its questions work."""
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pick  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
SIM = os.path.join(ROOT, 'ros', 'bringup', 'aav_bringup', 'launch', 'sim.launch.py')
VEHICLE = os.path.join(ROOT, 'ros', 'bringup', 'aav_description', 'aav_description',
                       'vehicle.py')
PIXI = os.path.join(ROOT, 'pixi.toml')


def _read(path):
    with open(path) as f:
        return f.read()


def _all_rows():
    return pick.PLANNERS + pick.FOLLOWERS + pick.LOCALIZERS + pick.SPEED_LOOPS + pick.WORLDS


def test_every_switch_is_a_sim_launch_argument():
    declared = set(re.findall(r"DeclareLaunchArgument\('(\w+)'", _read(SIM)))
    for row in _all_rows():
        for arg in row[2]:
            assert arg.split(':=')[0] in declared, arg


def test_followers_match_vehicle_py():
    text = _read(VEHICLE)
    controllers = re.search(r'^CONTROLLERS = \((.*)\)', text, re.M).group(1)
    trackers = re.search(r'^TRACKERS = \((.*)\)', text, re.M).group(1)
    for _, _, args in pick.FOLLOWERS:
        for arg in args:
            name, value = arg.split(':=')
            if name == 'controller':
                assert f"'{value}'" in controllers
            if name == 'tracker':
                assert f"'{value}'" in trackers
    # Every tracker but 'none' is offered.
    offered = {a.split(':=')[1] for _, _, args in pick.FOLLOWERS for a in args
               if a.startswith('tracker:=')}
    assert offered == set(re.findall(r"'(\w+)'", trackers))


def test_programs_are_pixi_tasks():
    text = _read(PIXI)
    for name, _ in pick.PROGRAMS:
        assert re.search(rf'^{name} *=', text, re.M), name


def test_every_row_sets_both_localizers():
    # debugdemo turns AMCL and NDT on; the last value wins, so each row says both.
    for _, _, args in pick.LOCALIZERS:
        names = {a.split(':=')[0] for a in args}
        assert {'ndt', 'amcl'} <= names


def test_enter_takes_the_defaults():
    cmd = pick.questions(read=lambda _: '', say=lambda *_: None)
    assert cmd[:3] == ['pixi', 'run', 'debugdemo']
    assert 'planner:=hybrid' in cmd
    assert 'controller:=mppi' in cmd and 'tracker:=none' in cmd
    assert 'ndt:=false' in cmd and 'amcl:=false' in cmd


def test_sim_with_a_localizer_moves_to_the_test_lot():
    answers = iter(['3', '3', '2', '6', '4', '2'])  # sim, empty, lattice, mpc, both, lqi
    said = []
    cmd = pick.questions(read=lambda _: next(answers), say=said.append)
    assert cmd[2] == 'sim'
    assert 'world:=test_lot' in cmd and 'tracker:=mpc' in cmd and 'speed_loop:=lqi' in cmd
    assert 'planner:=lattice' in cmd
    assert any('switching the world' in s for s in said)


def test_a_wrong_answer_asks_again():
    answers = iter(['9', 'x', '2'])
    row = pick.ask('?', pick.SPEED_LOOPS, read=lambda _: next(answers),
                   say=lambda *_: None)
    assert row[0] == 'lqi'


def test_listing_names_every_choice():
    text = pick.listing()
    for row in _all_rows():
        assert row[0] in text
