import importlib.util
import math
from pathlib import Path

from aav_gazebo.actuator_model import (ActuatorConfig, Actuators, Demand, Ideal, IGNORED, lag,
                                       LQI_MODEL,
                                       speed_loop_profile, SPEED_LOOP_PROFILES, SpeedLoop,
                                       SpeedLoopConfig)
import pytest
import yaml

DT = 0.02   # gazebo_adapter's 50 Hz
VEHICLE_YAML = (Path(__file__).resolve().parents[3] / 'bringup' / 'aav_description' / 'config'
                / 'vehicle.yaml')
ROOT = Path(__file__).resolve().parents[4]
VECTORS = ROOT / 'firmware/spine/speed_loop_vectors.txt'
GENERATOR = ROOT / 'firmware/spine/gen_speed_loop_vectors.py'
SPEED_LOOP_RS = ROOT / 'firmware/spine/crates/spine-core/src/speed_loop.rs'


def run(model, speed, steer, seconds):
    states = [model.step(speed, steer, DT) for _ in range(round(seconds / DT))]
    return states[-1], states


def test_vehicle_yaml_has_every_actuator_number():
    cfg = yaml.safe_load(VEHICLE_YAML.read_text())
    a = ActuatorConfig.from_vehicle(cfg)
    assert set(cfg['actuators']) <= set(vars(a)), 'an unknown key in actuators:'
    assert a.max_steering == cfg['vehicle']['max_steering_angle']
    # The guesses match Gazebo's old limits (docs/full-simulation.md).
    assert a.max_accel == 1.5 and a.max_brake == 3.0


def test_lag_reaches_63_percent_in_one_time_constant():
    v = 0.0
    for _ in range(50):
        v = lag(v, 1.0, 1.0, 0.02)
    assert v == pytest.approx(1.0 - math.exp(-1.0))
    assert lag(0.0, 1.0, 0.0, 0.02) == 1.0


def test_full_throttle_gives_max_accel_less_drag():
    cfg = ActuatorConfig(throttle_lag=0.0, drag=0.0)
    car = Actuators(cfg, SpeedLoopConfig(kp=100.0))   # always full throttle
    s, _ = run(car, 10.0, 0.0, 1.0)
    assert s.throttle == 1.0
    assert s.speed == pytest.approx(1.5, abs=0.01)
    assert s.accel == pytest.approx(1.5, abs=0.01)


def test_drag_balances_throttle_at_a_top_speed():
    cfg = ActuatorConfig(throttle_lag=0.0, drag=0.5)
    car = Actuators(cfg, SpeedLoopConfig(kp=100.0))
    s, _ = run(car, 100.0, 0.0, 30.0)
    assert s.speed == pytest.approx(1.5 / 0.5, rel=0.02)


def test_coasting_slows_by_drag_alone():
    car = Actuators(ActuatorConfig(drag=0.1))
    car.speed = 2.0
    # Throttle and brake at zero: drive the model directly, with a loop that never asks.
    car.loop.update = lambda target, measured, dt: type(
        'D', (), {'throttle': 0.0, 'brake': 0.0, 'reverse': False})()
    s, _ = run(car, 2.0, 0.0, 1.0)
    assert s.speed == pytest.approx(2.0 * math.exp(-0.1), rel=0.01)
    assert s.brake == 0.0 and s.throttle == 0.0


def test_full_brake_stops_at_max_brake_and_never_reverses():
    cfg = ActuatorConfig(brake_lag=0.0, drag=0.0)
    car = Actuators(cfg, SpeedLoopConfig(brake_gain=100.0, hold_brake=1.0))
    car.speed = 3.0
    s, states = run(car, 0.0, 0.0, 0.5)
    assert states[0].accel == pytest.approx(-3.0)
    s, _ = run(car, 0.0, 0.0, 2.0)
    assert s.speed == 0.0
    assert s.brake_command == 1.0


def test_the_brake_takes_time_to_bite():
    car = Actuators(ActuatorConfig(brake_lag=0.2))
    car.speed = 2.0
    first = car.step(0.0, 0.0, DT)
    assert first.brake_command > 0.3
    assert first.brake < 0.2 * first.brake_command


def test_the_speed_loop_settles_on_the_target():
    car = Actuators()
    s, states = run(car, 1.0, 0.0, 30.0)
    assert s.speed == pytest.approx(1.0, abs=0.02)
    assert max(x.speed for x in states) < 1.25        # no wild overshoot
    assert all(x.throttle == 0.0 or x.brake_command == 0.0 for x in states)


def test_below_the_throttle_deadband_the_car_doesnt_move():
    cfg = ActuatorConfig(throttle_deadband=0.5, throttle_lag=0.0)
    car = Actuators(cfg, SpeedLoopConfig(kp=0.1, ki=0.0))   # 0.1 throttle: in the deadband
    s, _ = run(car, 1.0, 0.0, 2.0)
    assert s.throttle == pytest.approx(0.1)
    assert s.speed == 0.0


def test_reverse_waits_for_a_stop_then_backs_up():
    car = Actuators()
    car.speed = 1.0
    s, states = run(car, -0.5, 0.0, 30.0)
    assert states[0].brake_command > 0.0 and states[0].throttle_command == 0.0
    assert s.speed == pytest.approx(-0.5, abs=0.1)


def test_steering_is_rate_limited_then_lags():
    cfg = ActuatorConfig(steer_rate=0.8, steer_lag=0.1, steer_deadband=0.0)
    car = Actuators(cfg)
    s, states = run(car, 0.0, 0.4, 0.3)
    assert car.motor == pytest.approx(0.24, abs=1e-6)    # 0.8 rad/s for 0.3 s
    assert s.steering < car.motor                         # the linkage is behind
    s, _ = run(car, 0.0, 0.4, 2.0)
    assert s.steering == pytest.approx(0.4, abs=1e-3)
    assert s.steer_command == 0.4


def test_steering_stops_at_full_lock():
    car = Actuators(ActuatorConfig(max_steering=0.5))
    s, _ = run(car, 0.0, 2.0, 5.0)
    assert s.steering == pytest.approx(0.5)


def test_the_centre_offset_shows_when_told_straight():
    car = Actuators(ActuatorConfig(steer_offset=0.02))
    s, _ = run(car, 0.0, 0.0, 1.0)
    assert s.steering == pytest.approx(0.02)


def test_small_steering_changes_are_lost_in_the_deadband():
    car = Actuators(ActuatorConfig(steer_deadband=0.01, steer_lag=0.0))
    s, _ = run(car, 0.0, 0.008, 1.0)
    assert s.steering == 0.0
    s, _ = run(car, 0.0, 0.02, 1.0)
    assert s.steering == pytest.approx(0.02)


def test_ideal_keeps_gazebos_old_limits_and_steers_at_once():
    ideal = Ideal(ActuatorConfig(max_steering=0.5))
    s, _ = run(ideal, 3.0, 0.3, 1.0)
    assert s.speed == pytest.approx(1.5)
    assert s.steering == 0.3
    s, _ = run(ideal, 0.0, 0.9, 0.3)
    assert s.speed == pytest.approx(1.5 - 0.9)
    assert s.steering == 0.5
    assert math.isnan(s.throttle)


def test_a_bad_time_step_is_refused():
    with pytest.raises(ValueError):
        Actuators().step(1.0, 0.0, 0.0)


def test_speed_loop_matches_the_spines_rules():
    # The same cases as spine-core's speed_loop.rs tests.
    loop = SpeedLoop()
    d = loop.update(1.0, 0.0, DT)
    assert d.throttle > 0.0 and d.brake == 0.0
    d = SpeedLoop().update(1.0, 2.0, DT)
    assert d.throttle == 0.0 and d.brake > 0.0
    d = SpeedLoop().update(0.0, 0.0, DT)
    assert d.brake == SpeedLoopConfig().hold_brake


def test_the_spines_demand_drives_the_actuators_directly():
    """spine:=true: the spine's throttle and brake skip our speed loop."""
    cfg = ActuatorConfig(throttle_lag=0.0, brake_lag=0.0, drag=0.0)
    car = Actuators(cfg)
    for _ in range(50):     # 1 s at full throttle: max_accel, whatever our speed loop thinks
        s = car.drive(Demand(1.0, 0.0, False), 0.1, DT)
    assert s.speed == pytest.approx(cfg.max_accel, rel=1e-6)
    assert s.throttle_command == 1.0 and s.brake_command == 0.0
    assert s.steering > 0.0, 'toward the steering target'
    for _ in range(50):     # full brake: stops, and never reverses
        s = car.drive(Demand(0.0, 1.0, False), 0.0, DT)
    assert s.speed == 0.0
    for _ in range(25):     # reverse gear, half throttle
        s = car.drive(Demand(0.5, 0.0, True), 0.0, DT)
    assert s.speed < 0.0


def test_drive_refuses_a_zero_step():
    with pytest.raises(ValueError):
        Actuators().drive(Demand(0.0, 0.0, False), 0.0, 0.0)


# --- the speed loop's profiles (TODO 27) ------------------------------------------------

def ramped(cfg, target, seconds=40.0, car=ActuatorConfig()):
    """Drive the model car with ego's 1 m/s^2 ramp up to target; return every speed."""
    model = Actuators(car, cfg)
    speeds, command = [], 0.0
    for _ in range(round(seconds / DT)):
        command = math.copysign(min(abs(target), abs(command) + 1.0 * DT), target)
        speeds.append(model.step(command, 0.0, DT).speed)
    return speeds


@pytest.mark.parametrize('target', [-0.5, 0.5, 1.0, 2.0])
def test_today_overshoots_and_smooth_does_not(target):
    # TODO 27, as Gazebo showed it: 0.5 -> 0.57, 1.0 -> 1.15, 2.0 -> 2.20.
    today = ramped(SPEED_LOOP_PROFILES['default'], target)
    assert max(abs(v) for v in today) > 1.09 * abs(target)
    smooth = ramped(SPEED_LOOP_PROFILES['smooth'], target)
    assert max(abs(v) for v in smooth) < 1.01 * abs(target)
    assert abs(smooth[-1]) == pytest.approx(abs(target), rel=0.01)


def test_smooth_reaches_2_m_s_on_a_throttle_5_times_weaker():
    weak = ActuatorConfig(max_accel=0.3)       # TODO 52b: the throttle's 5x uncertainty
    assert abs(ramped(SPEED_LOOP_PROFILES['default'], 2.0, 60.0, weak)[-1]) < 1.6
    assert ramped(SPEED_LOOP_PROFILES['smooth'], 2.0, 60.0, weak)[-1] == pytest.approx(
        2.0, rel=0.02)


def test_profiles_go_by_name():
    assert speed_loop_profile('default') == SpeedLoopConfig()
    assert speed_loop_profile('smooth').setpoint_weight == 0.75
    with pytest.raises(ValueError):
        speed_loop_profile('fast')
    with pytest.raises(ValueError):
        SpeedLoop(SpeedLoopConfig(anti_windup='wind'))


def test_the_profiles_match_spine_core():
    """The same names and numbers as speed_loop.rs (Default and SpeedLoopConfig::smooth)."""
    import re
    src = SPEED_LOOP_RS.read_text()
    default = src[src.index('impl Default for SpeedLoopConfig'):]
    smooth = src[src.index('pub fn smooth() -> Self'):]
    for block, cfg in ((default, SPEED_LOOP_PROFILES['default']),
                       (smooth, SPEED_LOOP_PROFILES['smooth'])):
        body = block[:block.index('}')]
        for name in ('kp', 'ki', 'integral_limit', 'setpoint_weight'):
            found = re.search(rf'\b{name}: ([\d.]+)', body)
            assert found and float(found.group(1)) == getattr(cfg, name), name
        windup = re.search(r'anti_windup: AntiWindup::(\w+)', body).group(1)
        assert windup.lower() == cfg.anti_windup
    assert set(re.findall(r'SpeedLoopProfile::\w+ => "(\w+)"', src)) == set(SPEED_LOOP_PROFILES)


def test_the_lqi_gains_match_spine_core():
    """LQI_MODEL is speed_loop.rs's LqiConfig::MODEL, number for number."""
    import re
    src = SPEED_LOOP_RS.read_text()
    body = src[src.index('pub const MODEL: LqiConfig'):]
    body = body[:body.index('};')]
    for name, value in vars(LQI_MODEL).items():
        found = re.search(rf'\b{name}: ([\d.]+)', body)
        assert found and float(found.group(1)) == value, name


def test_the_lqi_is_its_own_law_and_the_pi_the_default():
    assert SPEED_LOOP_PROFILES['default'].lqi is None
    assert SPEED_LOOP_PROFILES['lqi'].lqi == LQI_MODEL
    for target in (0.5, 1.0, 2.0):
        speeds = ramped(SPEED_LOOP_PROFILES['lqi'], target)
        assert max(speeds) < 1.02 * target
        assert speeds[-1] == pytest.approx(target, rel=0.01)


def test_the_lqi_never_uses_both_pedals():
    loop = SpeedLoop(SPEED_LOOP_PROFILES['lqi'])
    for k in range(2000):
        d = loop.update(1.0 + 0.5 * math.sin(k / 50), 1.0 + math.sin(k / 37), DT)
        assert d.throttle == 0.0 or d.brake == 0.0


def load_generator():
    spec = importlib.util.spec_from_file_location('gen_speed_loop_vectors', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_shared_examples_are_up_to_date():
    """speed_loop_vectors.txt is what this port gives today (spine-core checks the same)."""
    assert VECTORS.read_text() == load_generator().text(), \
        'run python3 firmware/spine/gen_speed_loop_vectors.py'


def test_the_port_matches_the_shared_examples():
    cases, loop = 0, None
    for line in VECTORS.read_text().splitlines():
        if not line.strip() or line.startswith('#'):
            continue
        if line.startswith('case '):
            loop = SpeedLoop(speed_loop_profile(line.split()[2]))
            cases += 1
            continue
        target, measured, dt, throttle, brake, reverse = line.split()
        d = loop.update(float(target), float(measured), float(dt))
        assert d.throttle == pytest.approx(float(throttle), abs=1e-6)
        assert d.brake == pytest.approx(float(brake), abs=1e-6)
        assert d.reverse == (reverse == '1')
    assert cases >= 8


def test_the_dead_times_are_left_out_on_purpose():
    # Fitted by the actuator tests, used by the linear model; not modelled here yet.
    cfg = yaml.safe_load(VEHICLE_YAML.read_text())
    cfg['actuators']['throttle_dead_time'] = 0.05
    a = ActuatorConfig.from_vehicle(cfg)
    assert not IGNORED & set(vars(a))
    assert a.throttle_lag == cfg['actuators']['throttle_lag']
