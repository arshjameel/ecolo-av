"""car-test --stands (car_test.py): its steps, options, judging and report. No ROS."""

import datetime
import os

from aav_actuator_tests import car_test as ct
from aav_actuator_tests import procedures as pr
from aav_actuator_tests import reports
from fake_car import drive
import pytest
import yaml


def names(planned, runs=True):
    return [s.name for s, why in planned if (why is None) == runs]


# --- the steps and the options -------------------------------------------------------


def test_the_steps_in_order_estop_last():
    assert ct.NAMES[:6] == ('preflight', 'sensors', 'steer-sweep', 'steer-rate',
                            'steer-centre', 'wheel-steps')
    assert ct.NAMES[-1] == 'estop'   # it latches: nothing can drive after it
    assert all(s.how is ct.How.FAULT for s in ct.STEPS[6:])


def test_every_car_step_says_what_the_person_does():
    for s in ct.STEPS:
        if s.car is ct.Manner.MANUAL:
            assert s.ask, s.name
        if s.sim is ct.Manner.NONE:
            assert s.why_not, s.name


def test_only_keeps_the_order_and_expands_groups():
    planned = ct.plan(['estop', 'steering'], sim=True, no_manual=False)
    assert names(planned) == ['steer-sweep', 'steer-rate', 'steer-centre', 'estop']
    assert {s.name: w for s, w in planned}['preflight'] == 'not asked for (--only)'


def test_no_manual_on_the_car_skips_every_person_step():
    planned = ct.plan([], sim=False, no_manual=True)
    assert names(planned) == ['preflight', 'sensors', 'steer-sweep', 'steer-rate',
                              'steer-centre', 'wheel-steps']
    assert all('--no-manual' in w for s, w in planned if s.how is ct.How.FAULT)


def test_the_sim_does_the_faults_it_can_and_says_why_not_the_rest():
    planned = {s.name: w for s, w in ct.plan([], sim=True, no_manual=True)}
    # The lidar since TODO 246: its watchdog input, and Ravensim's fault.lidar.
    for name in ('heartbeat', 'gps', 'imu', 'camera', 'lidar', 'estop'):
        assert planned[name] is None, name
    assert 'car only' in planned['pi-link'] and 'car only' in planned['cable']


def test_the_command_line():
    a = ct.parse(['--stands', '--sim', '--no-manual', '--only', 'faults'])
    assert a.stands and a.sim and a.no_manual and a.only == ['faults']
    for bad in ([], ['--ground'], ['--stands', '--ground'], ['--stands', '--only', 'brake']):
        with pytest.raises(SystemExit):
            ct.parse(bad)


def test_report_folder(monkeypatch):
    monkeypatch.delenv('AAV_TESTS_DIR', raising=False)
    when = datetime.datetime(2026, 11, 19, 14, 5, 6)
    assert ct.report_folder('/tmp/x', True, when) == '/tmp/x/2026-11-19-140506-stands-sim'
    assert ct.report_folder('/tmp/x', False, when).endswith('-stands')


# --- the tool steps ------------------------------------------------------------------


def test_any_exit_code_but_0_fails():
    for code in (1, 3, 4, 5, 6, 99):
        assert ct.tool_result(code)[0] is ct.Status.FAIL
    assert 'e-stop' in ct.tool_result(6)[1]
    assert ct.tool_result(0)[0] is ct.Status.PASS


SUMMARY = """Steering sweep
==============
What  Measured
NOTE: something

# Paste into ros/bringup/aav_description/config/vehicle.yaml (check them first):
actuators:
  steer_offset: 0.001              # measured: test 3.10
  steer_rate: 0.6                  # measured: test 3.11
"""


def test_steering_passes_only_with_vehicle_yaml_lines():
    lines = ct.yaml_lines_of(SUMMARY)
    assert lines.startswith('# Paste into') and 'steer_rate' in lines
    status, detail = ct.tool_result(0, lines, needs_yaml=True)
    assert status is ct.Status.PASS and 'steer_offset, steer_rate' in detail
    assert ct.tool_result(0, '', needs_yaml=True)[0] is ct.Status.FAIL
    assert ct.yaml_lines_of('no lines here') == ''


def test_wheel_steps_result_from_its_verdict():
    assert ct.tool_result(0, wheels=(True, 'ok'))[0] is ct.Status.PASS
    assert ct.tool_result(0, wheels=(False, 'no'))[0] is ct.Status.FAIL
    assert ct.tool_result(3, wheels=(True, 'ok'))[0] is ct.Status.FAIL


def test_tool_args():
    pf, sweep = ct.STEPS[0], ct.STEPS[2]
    assert ct.tool_args(pf, False, '/o') == ['preflight', '--stands', '--out', '/o']
    assert ct.tool_args(pf, True, '/o') == ['preflight', '--stands', '--out', '/o', '--sim',
                                            '--skip', 'battery']
    assert ct.tool_args(sweep, True, '/o') == ['steer-sweep', '--stands', '--out', '/o',
                                               '--sim']
    assert '--no-drive' in ct.tool_args(ct.STEPS[1], False, '/o')


def test_only_folder(tmp_path):
    assert ct.only_folder(str(tmp_path / 'none')) is None
    (tmp_path / 'output.log').write_text('')
    assert ct.only_folder(str(tmp_path)) is None
    (tmp_path / '2026-11-19-1-steer-sweep').mkdir()
    assert ct.only_folder(str(tmp_path)).endswith('steer-sweep')


# --- throttle on free wheels ---------------------------------------------------------


def test_wheel_steps_stay_within_the_stands_cap():
    steps = pr.plan('wheel-steps', pr.Settings(cap=0.3)).steps
    speeds = [s.speed for s in steps if s.phase.startswith('wheel step')]
    assert speeds == [0.1, 0.2, 0.3, 0.15]
    assert pr.TESTS['wheel-steps'].stage == 3   # on stands: the car needs --stands


def test_wheel_steps_on_the_pretend_car_follow():
    rows, _ = drive(pr.plan('wheel-steps', pr.Settings(cap=0.3)).steps)
    v = reports.judge_wheel_steps(reports.wheel_step_results(rows))
    assert v.follows and v.matches, v.detail
    rep = reports.analyse('wheel-steps', rows, reports.Context())
    assert rep.notes[0].startswith('FOLLOWS')


def S(*pairs):
    return [reports.WheelStep(c, m) for c, m in pairs]


def test_wheel_steps_judged():
    good = S((0.1, 0.1), (0.2, 0.19), (0.3, 0.29), (0.15, 0.22))   # coasting down
    v = reports.judge_wheel_steps(good)
    assert v.follows and v.matches, v.detail
    stuck = S((0.1, 0.1), (0.2, 0.19), (0.3, 0.29), (0.15, 0.29))  # never slows
    assert not reports.judge_wheel_steps(stuck).follows
    gain = S((0.1, 0.2), (0.2, 0.4), (0.3, 0.6), (0.15, 0.3))   # open loop: twice as fast
    v = reports.judge_wheel_steps(gain)
    assert v.follows and not v.matches and 'open-loop' in v.detail
    flat = S((0.1, 0.2), (0.2, 0.2), (0.3, 0.2), (0.15, 0.2))   # ignores the command
    assert not reports.judge_wheel_steps(flat).follows
    dead = S((0.1, 0.0), (0.2, 0.2), (0.3, 0.3), (0.15, 0.15))  # one step never turned
    assert not reports.judge_wheel_steps(dead).follows
    short = S((0.1, 0.1), (0.2, 0.2))
    assert 'run ended' in reports.judge_wheel_steps(short).detail
    assert not reports.judge_wheel_steps(S((0.1, float('nan')), (0.2, 0.2), (0.3, 0.3),
                                           (0.15, 0.15))).follows


# --- the faults ----------------------------------------------------------------------


def test_the_watchdog_names_its_inputs():
    # As watchdog-core's summary() writes it (seen in Ravensim, 19 Nov).
    line = 'STOP: gps STALE (nothing for 2.1 s (more than 2 s)); imu NAN (a value; x)'
    assert ct.named_inputs(line) == ['gps', 'imu']
    assert ct.named_inputs('STOP - gps STALE (x), imu NAN (y)') == ['gps', 'imu']
    assert ct.named(line, 'gps') and ct.named(line, 'imu')
    assert not ct.named(line, 'camera') and not ct.named(line, 'gp')
    assert not ct.named('WARN: gps STALE (x)', 'gps')   # only a STOP stops
    assert ct.named_inputs('OK') == [] and not ct.named(None, 'gps')


def test_a_sensors_limit_is_its_max_age_and_a_margin():
    inputs = {'gps': {'max_age': 2.0}, 'imu': {'max_age': 0.5}}
    assert ct.limit_for('gps', inputs) == 2.0 + ct.WATCHDOG_MARGIN
    assert ct.limit_for('lidar', inputs) is None
    assert ct.limit_for('hud', inputs) is None


def test_every_sensor_fault_step_has_a_watchdog_input():
    # TODO 246: step 5e found sensor_watchdog.yaml had no lidar input.
    path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'safety', 'aav_safety',
                        'config', 'sensor_watchdog.yaml')
    with open(path) as f:
        inputs = yaml.safe_load(f)['sensor_watchdog']['ros__parameters']
    for step in ct.STEPS:
        if step.sensor:
            assert ct.limit_for(step.sensor, inputs) is not None, step.sensor
            assert inputs[step.sensor]['stop'] is True, step.sensor


def test_judge_fault():
    ok = ct.Seen('its last message', 2.4, 'the sensor watchdog', 3.1)
    assert ct.judge_fault(ok, 3.0, ct.STILL_AFTER)[0] is ct.Status.PASS
    late = ct.Seen('its last message', 3.5, 'the sensor watchdog', 4.0)
    assert ct.judge_fault(late, 3.0, ct.STILL_AFTER)[0] is ct.Status.FAIL
    never = ct.Seen('its last message', None, '', 1.0)
    status, detail = ct.judge_fault(never, 3.0, ct.STILL_AFTER)
    assert status is ct.Status.FAIL and 'nothing said why' in detail
    slow_stop = ct.Seen('x', 1.0, 'w', 1.0 + ct.STILL_AFTER + 0.1)
    assert ct.judge_fault(slow_stop, None, ct.STILL_AFTER)[0] is ct.Status.FAIL
    no_stop = ct.Seen('x', 1.0, 'w', None)
    assert 'NEVER still' in ct.judge_fault(no_stop, None, 3.0)[1]
    # Without a word: only the time to still counts.
    quick = ct.Seen('the e-stop', None, '', 0.8, 'heard: /safety/estop')
    status, detail = ct.judge_fault(quick, None, ct.ESTOP_LIMIT, need_word=False)
    assert status is ct.Status.PASS and 'heard' in detail
    assert ct.judge_fault(ct.Seen('x', None, '', 2.5), None, 2.0,
                          need_word=False)[0] is ct.Status.FAIL


def test_the_orins_link_pulled_on_the_car():
    # Braked, then held while still asked to roll, with the status saying why.
    assert ct.judge_heartbeat_car(0.4, 0.0, False, 1)[0] is ct.Status.PASS
    # Held, but the status says armed and no timeout: not the ESP32's latch.
    assert ct.judge_heartbeat_car(0.4, 0.0, True, 0)[0] is ct.Status.FAIL
    # Not held: the wheels turned again once the link was back.
    assert ct.judge_heartbeat_car(0.4, 0.2, False, 1)[0] is ct.Status.FAIL
    # No status (the uplink off): the speed alone.
    status, detail = ct.judge_heartbeat_car(None, 0.01, None, None)
    assert status is ct.Status.PASS and 'uplink off' in detail
    assert ct.judge_heartbeat_car(0.4, None, None, None)[0] is ct.Status.FAIL


# --- the report ----------------------------------------------------------------------


def results(*statuses):
    return [ct.Result(s, st, f'detail {i}') for i, (s, st) in
            enumerate(zip(ct.STEPS, statuses))]


def test_verdicts():
    assert ct.verdict(results(*[ct.Status.PASS] * len(ct.STEPS))).startswith('PHASE A PASSED')
    some = results(ct.Status.PASS, ct.Status.SKIPPED)
    assert ct.verdict(some).startswith('NO STEP FAILED, but 1')
    bad = results(ct.Status.PASS, ct.Status.FAIL, ct.Status.NOT_RUN)
    assert ct.verdict(bad).startswith('PHASE A FAILED: 1 step failed') and ct.failed(bad)
    assert ct.counts(bad) == {'PASS': 1, 'FAIL': 1, 'SKIPPED': 0, 'NOT RUN': 1}


def test_table_and_markdown():
    rs = results(ct.Status.PASS, ct.Status.FAIL, ct.Status.NOT_RUN)
    text = ct.table(rs)
    assert 'PASS     1   preflight' in text and 'NOT RUN' in text
    md = ct.markdown(rs, 'Ravensim', '2026-11-19 14:05', {'steer-sweep': 'actuators:\n  a: 1'},
                     ['a | note'])
    assert '| 1 | the preflight' in md and '**FAIL**' in md
    assert '```yaml' in md and '- a | note' in md
    assert ct.cell('a | b\nc') == 'a \\| b c'
