"""The simulated car's own actuator numbers (sim.launch.py actuators:=, TODO 248)."""

from aav_sim_models.actuator_model import ActuatorConfig
from aav_sim_models.actuator_overrides import KEYS, overridden, parse
import pytest


def test_parse_takes_commas_or_spaces():
    assert parse('throttle_lag=0.3,drag=0.2 brake_dead_time=0.06') == {
        'throttle_lag': 0.3, 'drag': 0.2, 'brake_dead_time': 0.06}
    assert parse('') == {} and parse(None) == {}


def test_overridden_changes_only_what_is_named():
    base = ActuatorConfig()
    car = overridden(base, 'throttle_lag=0.3,steer_dead_time=0.08')
    assert car.throttle_lag == 0.3 and car.steer_dead_time == 0.08
    assert car.max_accel == base.max_accel and car.wheelbase == base.wheelbase
    assert overridden(base, '') is base


@pytest.mark.parametrize('bad', ['throttle_lag', 'wheelbase=2', 'nonsense=1', 'drag=fast',
                                 'drag=-0.1'])
def test_parse_refuses_what_it_cannot_take(bad):
    with pytest.raises(ValueError):
        parse(bad)


def test_the_size_and_load_stay_vehicle_yamls():
    assert 'wheelbase' not in KEYS and 'mass' not in KEYS and 'payload' not in KEYS
    assert 'steer_offset' in KEYS and parse('steer_offset=-0.01') == {'steer_offset': -0.01}
