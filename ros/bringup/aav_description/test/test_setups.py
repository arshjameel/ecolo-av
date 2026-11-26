"""The simulator's named set-ups: setup:=car, no set-up, and a switch by hand winning."""

import os
import re

from aav_description import setups
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_LAUNCH = os.path.join(HERE, '..', '..', 'aav_bringup', 'launch', 'sim.launch.py')
PIXI = os.path.join(HERE, '..', '..', '..', '..', 'pixi.toml')
NOT_GIVEN = {name: setups.CHOSEN for name in setups.FALLBACK}


def test_no_setup_keeps_the_launch_files_old_defaults():
    assert setups.resolve('', NOT_GIVEN) == {'world': 'empty', 'esp32': 'false'}


def test_car_is_the_road_world_with_the_esp32():
    assert setups.resolve('car', NOT_GIVEN) == {'world': 'road', 'esp32': 'true'}
    assert setups.resolve(' Car ', {}) == {'world': 'road', 'esp32': 'true'}


def test_a_switch_given_by_hand_wins_over_the_setup():
    assert setups.resolve('car', {'world': 'test_lot', 'esp32': ''}) == {
        'world': 'test_lot', 'esp32': 'true'}
    assert setups.resolve('car', {'world': 'empty', 'esp32': 'false'}) == {
        'world': 'empty', 'esp32': 'false'}


def test_an_unknown_setup_is_refused_by_name():
    with pytest.raises(ValueError, match='setup:=moon: pick one of car'):
        setups.resolve('moon', NOT_GIVEN)


def test_every_setup_sets_only_switches_with_a_fallback():
    for name in setups.names():
        assert set(setups.switches(name)) <= set(setups.FALLBACK), name
        assert setups.SETUPS[name]['what'], name


def test_sim_launch_declares_the_setup_switches_empty():
    """A switch declared with a value could not be told apart from one given by hand."""
    with open(SIM_LAUNCH) as f:
        text = f.read()
    for name in setups.FALLBACK:
        assert re.search(rf"DeclareLaunchArgument\('{name}', default_value=setups.CHOSEN",
                         text), name


@pytest.mark.skipif(not os.path.isfile(PIXI), reason='no pixi.toml beside this package')
def test_pixi_run_sim_is_the_virtual_car():
    with open(PIXI) as f:
        line = next(t for t in f if t.startswith('sim = '))
    assert f'setup:={setups.CAR}' in line


def test_describe_says_what_the_setup_gave():
    values = setups.resolve('car', NOT_GIVEN)
    assert setups.describe('car', values) == 'setup:=car: world:=road esp32:=true'
    assert setups.describe('', setups.resolve('', NOT_GIVEN)).startswith('no set-up')
