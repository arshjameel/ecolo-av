"""
The tests' steps, and the whole chain on a pretend car.

fake_car runs ego's model and Gazebo's actuator model (the same code `pixi run sim` uses),
so these check that each test gets vehicle.yaml's actuator numbers back out.
"""

from dataclasses import replace
import math

from aav_actuator_tests import procedures as pr
from aav_actuator_tests import reports
from aav_ego.ackermann import Limits
from aav_sim_models.actuator_model import ActuatorConfig
import fake_car
import pytest

CFG = ActuatorConfig()   # the same numbers as vehicle.yaml's actuators: today
# The pretend car logs as Gazebo's adapter does: each command with the tick it caused.
CTX = reports.Context(current={'actuators.drag': CFG.drag}, command_lead=fake_car.DT)
SETTINGS = pr.Settings(cap=3.0)


def run(name, settings=SETTINGS, cfg=CFG):
    p = pr.plan(name, settings)
    limits = Limits(max_speed=settings.cap, max_steering_rate=p.test.ego_steer_rate or 0.6)
    rows, runner = fake_car.drive(p.steps, cfg=cfg, limits=limits)
    assert not [d.step.phase for d in runner.done if d.timed_out]
    # The envelope is generous: the run stays inside it.
    assert rows[-1]['distance'] < p.distance and rows[-1]['t'] < p.time
    assert max(abs(r['speed']) for r in rows) <= settings.cap * 1.2 + 0.1
    return reports.analyse(name, rows, CTX)


def obs(t, speed=0.0, distance=0.0, heading=0.0):
    return pr.Obs(t, speed, distance, heading)


def test_runner_walks_the_steps():
    steps = [pr.Step('a', 0.5, until=pr.Until.TIME, value=1.0),
             pr.Step('b', 1.0, until=pr.Until.REACHED, value=0.9, hold=0.5, timeout=5.0),
             pr.Step('c', 0.0, until=pr.Until.STOPPED, hold=0.2)]
    r = pr.Runner(steps)
    assert r.tick(obs(0.0)).phase == 'a'
    assert r.tick(obs(1.0)).phase == 'b'
    assert r.tick(obs(1.5, speed=1.0)).phase == 'b'     # reached, not yet held
    assert r.tick(obs(2.1, speed=1.0)).phase == 'c'     # held 0.6 s
    assert r.tick(obs(2.2, speed=0.0)).speed == 0.0
    assert r.tick(obs(2.5, speed=0.0)) is None
    assert r.finished and [d.step.phase for d in r.done] == ['a', 'b', 'c']


def test_runner_times_out_and_goes_on():
    r = pr.Runner([pr.Step('never', 1.0, until=pr.Until.REACHED, value=5.0, timeout=2.0)])
    r.tick(obs(0.0))
    assert r.tick(obs(2.5, speed=1.0)) is None
    assert r.done[0].timed_out


def test_coasting_follows_the_speed_down():
    r = pr.Runner([pr.Step('coast', pr.COAST, until=pr.Until.BELOW, value=0.3)],
                  coast_offset=0.1)
    assert r.tick(obs(0.0, speed=2.0)).speed == pytest.approx(1.9)
    assert r.tick(obs(1.0, speed=1.0)).speed == pytest.approx(0.9)
    assert r.tick(obs(2.0, speed=0.2)) is None


def test_turned_and_distance():
    r = pr.Runner([pr.Step('arc', 1.0, until=pr.Until.TURNED, value=math.pi),
                   pr.Step('line', 1.0, until=pr.Until.DISTANCE, value=10.0)])
    r.tick(obs(0.0, heading=1.0))
    assert r.tick(obs(1.0, heading=1.0 - math.pi)).phase == 'line'   # turned right counts
    assert r.tick(obs(2.0, distance=5.0)).phase == 'line'
    assert r.tick(obs(3.0, distance=15.1)) is None


def test_every_command_stays_under_the_cap():
    s = pr.Settings(cap=0.3, speeds=(1.0, 2.0), room=9.0)   # room: the hallway tests
    for name in pr.TESTS:
        for st in pr.plan(name, s).steps:
            assert st.speed is pr.COAST or st.speed <= 0.3, (name, st)
            assert abs(st.steering) <= s.max_steering + 1e-9


def test_every_test_ends_with_the_car_stopped():
    for name in pr.TESTS:
        last = pr.plan(name, replace(SETTINGS, room=9.0)).steps[-1]
        assert last.speed == 0.0 and last.until is pr.Until.STOPPED, name


def test_ego_caps_the_settings():
    s, note = pr.with_ego_limit(pr.Settings(cap=3.0), 1.0)
    assert s.cap == 1.0 and 'sim_speed' in note
    assert pr.with_ego_limit(pr.Settings(cap=1.0), 3.0)[0].cap == 1.0


def test_unknown_test():
    with pytest.raises(ValueError):
        pr.plan('donuts', SETTINGS)


# --- the whole chain, on the pretend car ---------------------------------------------------

def test_steer_sweep_on_the_pretend_car():
    rep = run('steer-sweep')
    assert rep.yaml['vehicle.max_steering_angle'][0] == pytest.approx(0.5, abs=0.01)
    assert rep.yaml['actuators.steer_offset'][0] == pytest.approx(0.0, abs=0.005)


def test_steer_rate_on_the_pretend_car():
    rep = run('steer-rate')
    assert rep.yaml['actuators.steer_rate'][0] == pytest.approx(CFG.steer_rate, rel=0.05)
    assert rep.yaml['actuators.steer_lag'][0] == pytest.approx(CFG.steer_lag, abs=0.02)


def test_steer_centre_finds_an_offset_and_the_deadband():
    cfg = replace(CFG, steer_offset=0.01)
    p = pr.plan('steer-centre', SETTINGS)
    rows, _ = fake_car.drive(p.steps, cfg=cfg)
    rep = reports.analyse('steer-centre', rows, CTX)
    assert rep.yaml['actuators.steer_offset'][0] == pytest.approx(0.01, abs=0.001)
    assert rep.yaml['actuators.steer_deadband'][0] == pytest.approx(CFG.steer_deadband,
                                                                    abs=0.0011)


def test_speed_up_on_the_pretend_car():
    rep = run('speed-up')
    assert rep.yaml['actuators.max_accel'][0] == pytest.approx(CFG.max_accel, rel=0.05)
    assert rep.yaml['actuators.throttle_lag'][0] == pytest.approx(CFG.throttle_lag, abs=0.02)
    assert any(r.what == 'time to 3 m/s' and r.value != '-' for r in rep.rows)


def test_coast_on_the_pretend_car():
    rep = run('coast')
    assert rep.yaml['actuators.drag'][0] == pytest.approx(CFG.drag, rel=0.05)


def test_brake_on_the_pretend_car():
    rep = run('brake')
    assert rep.yaml['actuators.max_brake'][0] == pytest.approx(CFG.max_brake, rel=0.05)
    assert rep.yaml['actuators.brake_lag'][0] == pytest.approx(CFG.brake_lag, abs=0.02)


def test_throttle_map_finds_a_deadband():
    cfg = replace(CFG, throttle_deadband=0.1)
    p = pr.plan('throttle-map', SETTINGS)
    rows, _ = fake_car.drive(p.steps, cfg=cfg, limits=Limits(max_speed=3.0))
    rep = reports.analyse('throttle-map', rows, CTX)
    assert rep.yaml['actuators.max_accel'][0] == pytest.approx(CFG.max_accel, rel=0.1)
    assert rep.yaml['actuators.throttle_deadband'][0] == pytest.approx(0.1, abs=0.03)


def test_circles_on_the_pretend_car():
    s = replace(SETTINGS, angles=(0.5,))
    rep = run('circles', s)
    assert rep.yaml['vehicle.max_steering_angle'][0] == pytest.approx(0.5, abs=0.01)


def test_straight_on_the_pretend_car():
    cfg = replace(CFG, steer_offset=0.01)
    p = pr.plan('straight', SETTINGS)
    rows, _ = fake_car.drive(p.steps, cfg=cfg)
    rep = reports.analyse('straight', rows, CTX)
    assert rep.yaml['actuators.steer_offset'][0] == pytest.approx(0.01, abs=0.002)
    drift = next(r for r in rep.rows if r.what.startswith('sideways drift'))
    # 0.01 rad over 10 m: about 10 x 10 x 0.01 / (2 x 1.57) = 0.3 m to the left.
    assert float(drift.value) == pytest.approx(0.32, abs=0.05)


def test_no_sensor_says_so():
    p = pr.plan('steer-sweep', SETTINGS)
    rows, _ = fake_car.drive(p.steps)
    for r in rows:
        r['steer'] = float('nan')
    rep = reports.analyse('steer-sweep', rows, CTX)
    assert not rep.yaml and 'protractor' in rep.notes[0]
    # A person's readings take the sensor's place.
    for r in rows:
        if r['phase'].startswith('sweep '):
            r['manual_steer'] = r['cmd_steer'] * 0.9
    rep = reports.analyse('steer-sweep', rows, CTX)
    assert rep.yaml['vehicle.max_steering_angle'][0] == pytest.approx(0.45)


def test_report_text_and_yaml_lines():
    rep = run('coast')
    text = reports.table(rep)
    assert '4.2 coast' in text and 'drag' in text
    lines = reports.yaml_lines(rep)
    assert lines.splitlines()[1] == 'actuators:'
    assert lines.splitlines()[2].strip().startswith('drag: 0.1')


def test_csv_round_trip():
    row = dict.fromkeys(reports.COLUMNS, float('nan'))
    row.update(t=1.5, phase='coast from 2', speed=1.25)
    cells = {k: (row[k] if k == 'phase' else reports.number(row[k])) for k in reports.COLUMNS}
    back = reports.parse_rows([cells])[0]
    assert back['phase'] == 'coast from 2' and back['speed'] == 1.25
    assert math.isnan(back['steer'])
