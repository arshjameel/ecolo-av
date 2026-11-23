"""The car on stands: the actuator model without the car's weight (no ROS)."""

from dataclasses import replace

from aav_sim_models import jacks
from aav_sim_models.actuator_model import (ActuatorConfig, Actuators, Demand,
                                           speed_loop_profile)
import pytest

DT = 0.02


def run(model, speed, seconds):
    return [model.step(speed, 0.0, DT) for _ in range(round(seconds / DT))]


def test_on_stands_the_motor_moves_only_the_free_wheel_mass():
    ground = ActuatorConfig(mass=181.0)
    lifted = jacks.free_wheels(ground, 20.0)
    assert lifted.share == pytest.approx(181.0 / 20.0)
    # Nothing else changes: the lags, the steering and the drag are the same actuators'.
    assert replace(lifted, payload=0.0) == ground


def test_what_is_aboard_does_not_load_wheels_off_the_ground():
    empty = jacks.free_wheels(ActuatorConfig(mass=181.0), 20.0)
    loaded = jacks.free_wheels(ActuatorConfig(mass=181.0, payload=150.0), 20.0)
    assert loaded.share == pytest.approx(empty.share)


@pytest.mark.parametrize('mass', [0.0, -5.0, float('nan'), float('inf'), 'heavy'])
def test_a_free_wheel_mass_must_be_a_weight(mass):
    assert jacks.check_free_wheel_mass(mass)
    with pytest.raises(ValueError):
        jacks.free_wheels(ActuatorConfig(), mass)
    assert jacks.check_free_wheel_mass(jacks.FREE_WHEEL_MASS) is None


def pedals(cfg, throttle, seconds):
    """Return the speed after this throttle (the firmware's path: no speed loop)."""
    model = Actuators(cfg, speed_loop_profile('default'))
    demand = Demand(throttle=throttle, brake=0.0, reverse=False)
    return [model.drive(demand, 0.0, DT) for _ in range(round(seconds / DT))][-1].speed


def test_free_wheels_spin_up_far_faster_at_the_same_throttle():
    ground = ActuatorConfig(mass=181.0)
    lifted = jacks.free_wheels(ground, 20.0)
    on_ground = pedals(ground, 0.2, 1.0)
    on_stands = pedals(lifted, 0.2, 1.0)
    assert on_ground > 0.1
    assert on_stands == pytest.approx(on_ground * lifted.share, rel=0.05)


@pytest.mark.parametrize('profile', ['default', 'smooth'])
def test_the_speed_loop_still_holds_a_stands_speed_on_free_wheels(profile):
    lifted = jacks.free_wheels(ActuatorConfig(mass=181.0), jacks.FREE_WHEEL_MASS)
    model = Actuators(lifted, speed_loop_profile(profile))
    for target in (0.1, 0.2, 0.3):
        tail = [s.speed for s in run(model, target, 4.0)][-50:]
        # The stands test's own mark (wheel-steps): within 25 %, and steady.
        assert max(abs(v - target) for v in tail) < 0.25 * target
        assert max(tail) - min(tail) < 0.02
    stopped = run(model, 0.0, 3.0)[-1]
    assert stopped.speed == 0.0
