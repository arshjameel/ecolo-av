"""
Phase B in the hallway (TODO 248).

The IMU with the wheels, error bars, the model replayed, the hallway's limits and passes,
and, the key evidence, known numbers found again.

fake_hallway drives every hall-* pass on the pretend car (ego's model, the spine's speed
loop, the car model with numbers unlike vehicle.yaml's), with a noisy, biased IMU and
noisy, slipping wheels, and hallway.analyse must find those numbers within its error bars.
"""

from dataclasses import replace
import math
import os
import random

from aav_actuator_tests import car_test as ct
from aav_actuator_tests import errorbars as eb
from aav_actuator_tests import fusion
from aav_actuator_tests import hallway
from aav_actuator_tests import procedures as pr
from aav_actuator_tests import reports
from aav_actuator_tests import safety
from aav_description.car_model import ActuatorConfig
import fake_hallway as fh
import pytest

# --- the IMU and the wheels ----------------------------------------------------------


def test_still_imu_gives_the_offset_and_noise():
    rng = random.Random(1)
    t = [i * 0.01 for i in range(1000)]
    ax = [0.12 + rng.gauss(0.0, 0.1) for _ in t]
    still = fusion.still_imu(t, ax, t, [0.0] * len(t))
    assert still.offset == pytest.approx(0.12, abs=0.01)
    assert still.noise == pytest.approx(0.1, abs=0.01)
    assert still.trusted
    # Moving samples don't count.
    assert fusion.still_imu(t, ax, t, [1.0] * len(t)) is None


def test_slope_from_both_ways_splits_bias_and_slope():
    g = fusion.G
    pitch = 0.02    # rad uphill facing 'there'
    bias = 0.05
    s = fusion.slope_from([('there', bias + g * math.sin(pitch)),
                           ('back', bias - g * math.sin(pitch))])
    assert s.angle == pytest.approx(pitch, abs=1e-6)
    assert s.bias == pytest.approx(bias, abs=1e-9)
    assert s.level is False and 'NOT level' in s.describe()
    assert s.gravity('there') == pytest.approx(g * math.sin(pitch))
    assert s.gravity('back') == pytest.approx(-g * math.sin(pitch))
    flat = fusion.slope_from([('there', 0.05), ('back', 0.05)])
    assert flat.level is True


def test_one_way_cannot_tell_the_slope():
    s = fusion.slope_from([('there', 0.1), ('there', 0.11)])
    assert s.level is None and not math.isfinite(s.angle)
    assert s.gravity('there') == 0.0 and 'one way' in s.describe()


def test_fuse_follows_the_true_speed_better_than_the_wheels():
    rng = random.Random(2)
    dt = 0.01
    t = [i * dt for i in range(800)]
    true_a = [0.8 if 1.0 <= x < 3.0 else (-1.0 if 5.0 <= x < 6.6 else 0.0) for x in t]
    v, true_v = 0.0, []
    for a in true_a:
        v = max(0.0, v + a * dt)
        true_v.append(v)
    ax = [a + 0.07 + rng.gauss(0.0, 0.2) for a in true_a]   # bias 0.07
    wheels = [x + rng.gauss(0.0, 0.05) for x in true_v]
    f = fusion.fuse(t, ax, t, wheels, offset=0.07, imu_noise=0.2, wheel_noise=0.05)
    err_f = math.sqrt(sum((a - b) ** 2 for a, b in zip(f.speed, true_v)) / len(t))
    err_w = math.sqrt(sum((a - b) ** 2 for a, b in zip(wheels, true_v)) / len(t))
    assert err_f < 0.5 * err_w
    assert f.how == 'imu+wheels' and len(f.accel) == len(t)
    assert all(sd > 0 for sd in f.speed_sd)


def test_no_imu_falls_back_to_the_wheels():
    t = [i * 0.02 for i in range(100)]
    v = [0.5 * x for x in t]
    f = fusion.fuse([], [], t, v)
    assert f.how == 'wheels'
    assert f.accel[50] == pytest.approx(0.5, abs=1e-6)


def test_forces_are_mass_times_acceleration():
    f = fusion.forces(200.0, [0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.1],
                      [-1.0] * 6, drag=0.2, smooth=5)
    assert f.drive_peak == pytest.approx(100.0)
    assert f.brake_peak == pytest.approx(200.0)
    assert f.drag_at_1 == pytest.approx(40.0)


# --- error bars -----------------------------------------------------------------------


def test_estimate_gives_mean_sd_and_trust():
    e = eb.estimate([eb.Value(v) for v in (0.30, 0.32, 0.28, 0.31)])
    assert e.mean == pytest.approx(0.3025) and e.n == 4
    assert e.sd == pytest.approx(0.0171, abs=1e-3)
    assert e.lo <= e.mean <= e.hi
    assert e.trusted and e.covers(0.31) and not e.covers(0.5)


def test_one_run_or_a_spread_or_a_problem_is_not_trusted():
    assert 'one run' in eb.estimate([eb.Value(0.3)]).why
    wide = eb.estimate([eb.Value(v) for v in (0.1, 0.5, 0.3)])
    assert not wide.trusted and 'disagree' in wide.why
    flagged = eb.estimate([eb.Value(0.3), eb.Value(0.31, 0.04, 'under the resolution')])
    assert not flagged.trusted and 'resolution' in flagged.why
    assert eb.estimate([eb.Value(float('nan'))]) is None


def test_the_bar_is_never_under_the_resolution_or_the_floor():
    e = eb.estimate([eb.Value(0.2, 0.05), eb.Value(0.2, 0.05)], floor=0.0)
    assert e.sd == 0.0 and e.bar == 0.05
    g = eb.estimate([eb.Value(2.0), eb.Value(2.0)], floor=0.03)
    assert g.bar == pytest.approx(0.06) and g.covers(2.1) and not g.covers(2.2)


# --- the hallway's limits and passes --------------------------------------------------


def test_hallway_limits_follow_the_plan():
    hall, _ = safety.hallway_limits(20.0)
    assert hall.top == safety.REAL_CAP and hall.room == 17.0     # long: the shared cap
    short, _ = safety.hallway_limits(4.0)
    assert 0.2 < short.top < 1.0
    assert short.needed(short.top) <= 4.0 + 0.05
    assert safety.hallway_limits(3.1)[0] is None                  # too short: refused
    assert safety.hallway_limits(10.0, margin=0.5)[0] is None     # a margin under 1 m
    assert safety.hallway_limits(30.0, cap=0.5)[0].top == 0.5


def test_the_geofence_may_say_stands_only_in_the_hallway():
    assert safety.geofence_stop('STANDS: no fence', 0.1, False, True) is not None
    assert safety.geofence_stop('STANDS: no fence', 0.1, False, True, hallway=True) is None
    assert safety.geofence_stop('NEAR: edge', 0.1, False, True, hallway=True) is not None
    assert safety.geofence_stop(None, None, False, True, hallway=True) is not None


def test_a_hallway_test_needs_its_room():
    with pytest.raises(ValueError):
        pr.plan('hall-brake', pr.Settings(cap=1.0))
    plan = pr.plan('hall-brake', pr.Settings(cap=1.0, room=6.0))
    assert plan.distance == 6.0          # the envelope stops it at the room
    assert plan.steps[1].phase == pr.HALL_STILL and plan.steps[1].value == 10.0


@pytest.mark.parametrize('length', [4.5, 6.0, 15.0])
@pytest.mark.parametrize('test', pr.HALL_TESTS)
def test_every_pass_stops_inside_the_room(test, length):
    hall, _ = safety.hallway_limits(length)
    rows, meta = fh.drive_pass(test, replace(ActuatorConfig(), **fh.TRUE), hall, 'there', 3)
    assert meta['max_distance'] < hall.room, (test, length, meta['max_distance'])
    assert all(r['cmd_speed'] <= hall.top + 1e-9 for r in rows)


@pytest.mark.parametrize('test', pr.HALL_TESTS)
def test_no_pass_trips_the_runaway_rule_on_a_lagging_car(test):
    """The plateau (HALL_HOLD) before each fall: the lags carry the speed on a little."""
    hall, _ = safety.hallway_limits(15.0)
    rows, _ = fh.drive_pass(test, replace(ActuatorConfig(), **fh.TRUE), hall, 'there', 7,
                            wheel_noise=0.02, wheel_slip=0.02)
    history = []
    for r in rows:
        history.append((r['t'], r['cmd_speed'], r['speed']))
        history = [h for h in history if r['t'] - h[0] <= 5.0]
        assert safety.runaway(history) is None, (test, r['t'], r['phase'])


def test_the_brake_point_cuts_a_speed_up_short():
    r = pr.Runner([pr.Step('go', 1.0, until=pr.Until.TIME, value=100.0, fenced=True),
                   pr.Step('stop', 0.0, until=pr.Until.TIME, value=1.0)],
                  room=5.0, stop_decel=0.5, reaction=0.3)
    assert r.tick(pr.Obs(0.0, 1.0, 0.0, 0.0)).phase == 'go'
    # 1 m/s needs 0.3 + 1.0 m to stop: at 3.7 m it must brake.
    assert r.tick(pr.Obs(1.0, 1.0, 3.71, 0.0)).phase == 'stop'
    assert r.done[0].cut


def test_coast_follows_a_smoothed_speed():
    r = pr.Runner([pr.Step('c', pr.COAST, until=pr.Until.TIME, value=10.0)], coast_offset=0.1)
    cmds = [r.tick(pr.Obs(i * 0.02, 1.0 + (0.1 if i % 2 else -0.1), 0.0, 0.0)).speed
            for i in range(20)]
    assert max(cmds[-5:]) - min(cmds[-5:]) < 0.03


def test_hall_passes_turn_round_and_repeat():
    passes = ct.hall_passes(('brake', 'coast'), 2, both_ways=True)
    assert [p.kind for p in passes] == ['coast', 'brake', 'coast', 'brake']   # coast first
    assert [p.direction for p in passes] == ['there', 'back', 'there', 'back']
    assert passes[1].folder == '02-hall-brake-back'
    assert {p.direction for p in ct.hall_passes(ct.HALL_KINDS, 2, False)} == {'there'}
    args = ct.hall_args(passes[0], 12.0, 3.0, True, '/tmp/x')
    assert args[:2] == ['hall-coast', '--hallway'] and '--sim' in args


def test_the_hallway_command_line():
    a = ct.parse(['--hallway', '--sim', '--no-manual'])
    assert a.hallway and a.length == ct.SIM_LENGTH
    a = ct.parse(['--hallway', '--length', '9', '--kinds', 'brake', '--repeats', '2'])
    assert a.length == 9 and a.kinds == ['brake'] and not a.sim
    for bad in (['--hallway', '--stands'], ['--hallway', '--kinds', 'donuts'],
                ['--hallway', '--only', 'brake'], ['--hallway', '--sim-actuators', 'drag=1'],
                ['--hallway', '--sim', '--sim-actuators', 'wheelbase=2'],
                ['--ground'], ['--hallway', '--repeats', '0']):
        with pytest.raises(SystemExit):
            ct.parse(bad)
    assert ct.parse(['--analyse', '/tmp']).analyse == '/tmp'
    assert ct.sim_overrides('throttle_lag=0.3, drag=0.2') == {'throttle_lag': 0.3, 'drag': 0.2}
    launch = ct.sim_launch_args({'drag': 0.2}, 0.05, 0.02)
    assert 'actuators:=drag=0.2' in launch and 'wheel_noise:=0.05' in launch
    # No numbers: no actuators:= at all (launch refuses an empty one; 20 Nov's baseline).
    plain = ct.sim_launch_args({}, 0.0, 0.0)
    assert not any(a.startswith('actuators:=') for a in plain)
    assert all(':=' in a and a.split(':=', 1)[1] for a in plain)


# --- the key evidence: known numbers found again ---------------------------------------


@pytest.fixture(scope='module')
def session(tmp_path_factory):
    folder = str(tmp_path_factory.mktemp('hall'))
    fh.write_session(folder)
    s = hallway.analyse(folder)
    hallway.write(s)
    return s


def test_the_fits_find_the_known_numbers_within_their_error_bars(session):
    rows = list(hallway.recovery_rows(session))
    names = {r[0] for r in rows}
    for name in ('drag', 'max_accel', 'max_brake', 'throttle_delay', 'brake_delay',
                 'steer_lag', 'steer_dead_time', 'throttle_lag', 'brake_lag'):
        assert name in names, name
    missed = [(n, t, e.text()) for n, t, e, ok in rows if not ok]
    assert not missed, missed


def test_the_fitted_numbers_differ_from_the_guesses_and_are_trusted(session):
    est = session.estimates
    for k in ('drag', 'max_accel', 'max_brake', 'throttle_delay', 'brake_delay', 'steer_lag'):
        assert est[k].trusted, (k, est[k].why)
    assert est['drag'].mean == pytest.approx(fh.TRUE['drag'], rel=0.15)
    assert est['max_brake'].mean == pytest.approx(fh.TRUE['max_brake'], rel=0.12)


def test_the_fitted_model_matches_the_car_and_the_guesses_do_not(session):
    fitted = [a for _, a, _, _, _ in session.scores]
    guessed = [b for _, _, b, _, _ in session.scores]
    assert all(a is not None and a.passed for a in fitted)
    assert not all(b.passed for b in guessed)


def test_the_files(session):
    folder = session.folder
    for name in ('report.md', 'vehicle-yaml.txt', 'fits.csv', 'compare.csv'):
        assert os.path.exists(os.path.join(folder, name)), name
    assert len([f for f in os.listdir(os.path.join(folder, 'overlays'))
                if f.endswith('.csv')]) == len(session.passes)
    md = open(os.path.join(folder, 'report.md')).read()
    assert 'Recovery' in md and 'Slope' in md and 'F = m a' in md
    lines = hallway.yaml_lines(session)
    assert '  drag: ' in lines and 'NOT TRUSTED' not in lines.split('\n')[3]


def test_only_trusted_numbers_are_pasted(session):
    for name, c in hallway.chosen(session).items():
        line = [ln for ln in hallway.yaml_lines(session).splitlines()
                if ln.lstrip('# ').startswith(name + ':')][0]
        assert line.startswith('#') != c.trusted, line


def test_a_slope_is_found_from_both_ways(tmp_path):
    folder = str(tmp_path)
    fh.write_session(folder, repeats=1, kinds=('coast', 'brake'), slope=0.02)
    s = hallway.analyse(folder)
    assert s.slope.angle == pytest.approx(0.02, abs=0.004)
    assert s.slope.level is False


def test_a_single_pass_reports_itself():
    hall, _ = safety.hallway_limits(12.0)
    rows, meta = fh.drive_pass('hall-steer', replace(ActuatorConfig(), **fh.TRUE), hall,
                               'there', 4)
    rep = reports.analyse('hall-steer', rows, reports.context_from_meta(meta))
    assert any(r.what == 'steer lag' for r in rep.rows)


def test_the_steering_from_the_yaw_rate_alone(tmp_path):
    """No measured angle (the car's uplink has none yet): the IMU's yaw rate gives it."""
    folder = str(tmp_path)
    fh.write_session(folder, repeats=3, kinds=('steer',), steer_sensor=False)
    s = hallway.analyse(folder)
    rec = {n: (t, e, ok) for n, t, e, ok in hallway.recovery_rows(s)}
    true, e, ok = rec['steer_delay']
    assert ok, (true, e.text())
