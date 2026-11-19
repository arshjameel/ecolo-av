"""The adaptive brake: harder when the car slows less than expected, within the brake's limits."""

from dataclasses import replace
from pathlib import Path

from aav_description import vehicle
from aav_description.adaptive_brake import (AdaptiveBrake, AdaptiveBrakeConfig, BrakeLimits)
from aav_description.car_model import (ActuatorConfig, Actuators, CarState, Command,
                                       SpeedLoopConfig, step)
from aav_description.stop_law import stop_point
import pytest

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'vehicle.yaml'
DT = 0.02
EMPTY = ActuatorConfig()
TOLERANCE = 0.25   # m: tools/stop_points.py's pass band


def stop_from(cfg, speed, ramp, brake=None, settle=8.0):
    """Hold `speed`, then let the set speed fall at `ramp`: metres to standstill, peak brake."""
    car = Actuators(cfg, SpeedLoopConfig(), brake)
    car.state = replace(CarState(), v=speed,
                        throttle_level=min(1.0, cfg.drag * speed / (cfg.max_accel * cfg.share)))
    for _ in range(round(settle / DT)):
        car.step(speed, 0.0, DT)
    x0, target, peak = car.state.x, speed, 0.0
    for _ in range(round(30.0 / DT)):
        target = max(0.0, target - ramp * DT)
        peak = max(peak, car.step(target, 0.0, DT).brake_command)
        if target == 0.0 and abs(car.speed) < 0.01:
            break
    return car.state.x - x0, peak


@pytest.mark.parametrize('speed, ramp', [(1.0, 1.0), (2.0, 1.0), (1.0, 0.5)])
@pytest.mark.parametrize('payload', [0.0, 90.0, 150.0])
def test_with_the_loop_a_loaded_car_stops_where_the_empty_model_says(speed, ramp, payload):
    # The prediction is the empty car's: nobody told the car model about the load.
    predicted, _ = stop_point(EMPTY, speed, ramp, dt=DT)
    loaded = replace(EMPTY, payload=payload)
    with_loop, _ = stop_from(loaded, speed, ramp, AdaptiveBrake(EMPTY))
    assert abs(with_loop - predicted) <= TOLERANCE


def test_without_the_loop_a_loaded_car_overshoots():
    predicted, _ = stop_point(EMPTY, 2.0, 1.0, dt=DT)
    loaded = replace(EMPTY, payload=150.0)
    without, _ = stop_from(loaded, 2.0, 1.0)
    with_loop, _ = stop_from(loaded, 2.0, 1.0, AdaptiveBrake(EMPTY))
    assert without - predicted > TOLERANCE           # why the loop is worth having
    assert abs(with_loop - predicted) < 0.5 * (without - predicted)


def test_the_empty_car_is_left_alone():
    plain, peak = stop_from(EMPTY, 1.5, 1.0)
    looped, looped_peak = stop_from(EMPTY, 1.5, 1.0, AdaptiveBrake(EMPTY))
    assert looped == pytest.approx(plain, abs=0.01)
    assert looped_peak == pytest.approx(peak, abs=0.01)


def test_it_learns_the_share_and_keeps_it_for_the_next_stop():
    loaded = replace(EMPTY, payload=181.0)        # twice the mass: half the braking
    brake = AdaptiveBrake(EMPTY)
    stop_from(loaded, 1.5, 1.0, brake)
    assert brake.effectiveness == pytest.approx(0.5, abs=0.05)
    assert brake.gain == pytest.approx(2.0, abs=0.2)
    predicted, _ = stop_point(EMPTY, 1.5, 1.0, dt=DT)
    again, _ = stop_from(loaded, 1.5, 1.0, brake)
    assert abs(again - predicted) <= TOLERANCE
    brake.reset()
    assert brake.effectiveness == 1.0


def braking(brake, cfg, asked, seconds, v=2.0):
    """Feed a steady asked brake to a car of cfg; return each tick's BrakeOut."""
    state, outs = CarState(v=v), []
    for _ in range(round(seconds / DT)):
        out = brake.update(0.0, asked, state.v, DT)
        outs.append(out)
        state = step(cfg, state, Command(brake=out.brake), DT)
    return outs


def test_never_above_the_safe_max():
    limits = BrakeLimits(safe_max=2000.0)           # (2000 - 400) / 3100 = 0.52
    weak = replace(EMPTY, payload=400.0)
    outs = braking(AdaptiveBrake(EMPTY, limits), weak, 0.4, 1.5, v=3.0)
    assert max(o.brake for o in outs) <= limits.safe_max_fraction + 1e-12
    assert any(o.capped for o in outs)


def test_the_asked_brake_passes_even_above_the_safe_max():
    # The speed loop's own stop is the spine's business: never weakened here.
    limits = BrakeLimits(safe_max=2000.0)
    outs = braking(AdaptiveBrake(EMPTY, limits), replace(EMPTY, payload=200.0), 0.9, 0.5)
    assert all(o.brake == pytest.approx(0.9) for o in outs)


def test_the_extra_rises_no_faster_than_the_rate():
    tuning = AdaptiveBrakeConfig(rise_rate=0.5)
    outs = braking(AdaptiveBrake(EMPTY, tuning=tuning), replace(EMPTY, payload=300.0), 0.3,
                   1.5, v=4.0)
    steps = [b.brake - a.brake for a, b in zip(outs, outs[1:])]
    assert max(steps) <= 0.5 * DT + 1e-9
    assert outs[-1].brake > 0.35


def test_high_force_is_held_only_max_hold_time():
    limits = BrakeLimits(max_hold_time=0.5)
    brake = AdaptiveBrake(EMPTY, limits)
    brake.effectiveness = 0.4                      # already learnt: a heavy load
    outs = braking(brake, replace(EMPTY, payload=270.0), 0.6, 2.0, v=6.0)
    high = [o for o in outs if o.brake >= 0.9 * limits.safe_max_fraction]
    assert high and len(high) * DT <= 0.5 + 2 * DT
    assert outs[-1].spent and outs[-1].brake == pytest.approx(0.6)
    # Let go of the brake, and the next stop may push again.
    brake.update(0.0, 0.0, 1.0, DT)
    assert not brake.spent


def test_no_wheel_speed_no_extra():
    brake = AdaptiveBrake(EMPTY)
    brake.effectiveness = 0.5
    assert brake.update(0.0, 0.4, None, DT).brake == 0.4
    assert brake.update(0.0, 0.4, float('nan'), DT).brake == 0.4


def test_it_learns_nothing_while_the_throttle_pushes():
    brake = AdaptiveBrake(EMPTY)
    for _ in range(100):
        brake.update(0.5, 0.0, 1.0, DT)            # driving: a steady speed, no brake
    assert brake.effectiveness == 1.0


def test_bad_limits_say_why():
    with pytest.raises(ValueError, match='safe_max'):
        BrakeLimits(safe_max=4000.0)
    with pytest.raises(ValueError, match='touch'):
        BrakeLimits(touch=100.0)
    with pytest.raises(ValueError, match='same'):
        BrakeLimits(released=500.0, full=500.0)
    with pytest.raises(ValueError, match='dt'):
        AdaptiveBrake().update(0.0, 0.5, 1.0, 0.0)


def test_a_brake_that_pulls_the_other_way_maps_the_same():
    reversed_ = BrakeLimits(released=3500.0, touch=3100.0, full=400.0, safe_max=700.0)
    assert reversed_.touch_fraction == pytest.approx(400.0 / 3100.0)
    assert reversed_.safe_max_fraction == pytest.approx(2800.0 / 3100.0)


def test_vehicle_yaml_gives_the_limits():
    limits = BrakeLimits.from_vehicle(vehicle.load(CONFIG))
    assert limits == BrakeLimits()     # the defaults are vehicle.yaml's placeholders
    assert BrakeLimits.from_vehicle({'vehicle': {}}) == BrakeLimits()


# The shared examples (TODO 258): the car runs the Rust copy (firmware/spine/crates/
# adaptive-brake, in legacy_uart_bridge), the sim this one; both check the same file.
ROOT = Path(__file__).resolve().parents[4]
VECTORS = ROOT / 'firmware' / 'spine' / 'adaptive_brake_vectors.txt'
GENERATOR = ROOT / 'firmware' / 'spine' / 'gen_adaptive_brake_vectors.py'


def test_the_shared_examples_are_up_to_date():
    import importlib.util
    spec = importlib.util.spec_from_file_location('gen_adaptive_brake_vectors', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert VECTORS.read_text() == module.text(), \
        'run python3 firmware/spine/gen_adaptive_brake_vectors.py'
