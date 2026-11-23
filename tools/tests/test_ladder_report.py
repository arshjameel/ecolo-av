"""A session's report read for a screen (tools/ladder_report.py): no ROS."""
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import ladder  # noqa: E402
import ladder_report as lr  # noqa: E402
import pytest  # noqa: E402
import yaml  # noqa: E402


def log_rows(n=1000):
    """Return a speed step: asked 1 m/s from 2 s, the wheels follow with a lag; then a stop."""
    rows = []
    speed = 0.0
    for i in range(n):
        t = i * 0.02
        asked = 1.0 if 2.0 <= t < 12.0 else 0.0
        speed += (asked - speed) * 0.02 / 0.5
        rows.append({'t': f'{t:g}', 'phase': 'speed up' if asked else 'stop',
                     'cmd_speed': f'{asked:g}', 'speed': f'{speed:g}',
                     'cmd_steer': '0.1', 'steer': '0.09'})
    return rows


def write_session(folder, name='steer-rate'):
    run = folder / name / '2026-11-22-120010-run-sim'
    run.mkdir(parents=True)
    rows = log_rows()
    with open(run / 'log.csv', 'w') as f:
        f.write(','.join(rows[0]) + '\n')
        f.writelines(','.join(r.values()) + '\n' for r in rows)
    (run / 'meta.yaml').write_text(yaml.safe_dump({'test': name}))
    (run / 'summary.txt').write_text(
        '3.11 steering speed\n\n# Paste into vehicle.yaml (check them first):\nactuators:\n'
        '  steer_lag: 0.095                # measured: test 3.11\n'
        '  steer_rate: 0.7909              # measured\n')
    report = {'started': '2026-11-22 12:00:00', 'where': 'a test',
              'counts': {'PASS': 1, 'FAIL': 0, 'SKIPPED': 1, 'NOT RUN': 0},
              'steps': [{'step': name, 'number': '3b', 'status': 'PASS',
                         'detail': 'vehicle.yaml lines: steer_lag, steer_rate',
                         'files': f'{name}/2026-11-22-120010-run-sim'},
                        {'step': 'cable', 'number': '5g', 'status': 'SKIPPED',
                         'detail': 'no cables in Ravensim', 'files': ''}]}
    (folder / 'report.yaml').write_text(yaml.safe_dump(report))


def test_a_graph_is_commanded_against_measured_thinned_for_a_screen():
    g = lr.graph(log_rows(), 'brake')
    assert (g['kind'], g['unit'], g['what']) == ('speed', 'm/s', 'speed')
    assert len(g['t']) == len(g['commanded']) == len(g['measured']) <= lr.GRAPH_POINTS + 1
    assert g['t'][0] == 0.0 and g['t'][-1] == pytest.approx(19.98)   # the ends are kept
    assert max(g['commanded']) == 1.0 and 0.9 < max(g['measured']) <= 1.0
    steer = lr.graph(log_rows(), 'steer-rate')
    assert (steer['kind'], steer['unit']) == ('steering', 'rad')
    assert steer['measured'][0] == 0.09
    assert lr.graph_kind('hall-steer') == 'steering' and lr.graph_kind('circles') == 'speed'


def test_a_log_with_nothing_to_draw_gives_no_graph():
    assert lr.graph([], 'brake') is None
    assert lr.graph([{'t': '0', 'cmd_speed': '', 'speed': 'nan'}] * 5, 'brake') is None
    assert lr.finite('') is None and lr.finite('inf') is None and lr.finite('1.5') == 1.5


def test_a_session_reads_as_steps_with_their_lines_and_graph(tmp_path):
    folder = tmp_path / '2026-11-22-120000-stands-sim'
    write_session(folder)
    s = lr.session(str(folder))
    assert s['found'] and s['counts']['PASS'] == 1 and s['started'] == '2026-11-22 12:00:00'
    first, second = s['steps']
    assert (first['name'], first['number'], first['status']) == ('steer-rate', '3b', 'PASS')
    assert first['yaml'] == ['steer_lag: 0.095', 'steer_rate: 0.7909']
    assert first['graph']['kind'] == 'steering'
    assert second['status'] == 'SKIPPED' and second['graph'] is None and second['yaml'] == []
    assert json.loads(json.dumps(s)) == s           # it is JSON as it stands
    assert 'steer_lag: 0.095' in lr.text(s)


def test_no_session_says_so_in_words(tmp_path):
    assert not lr.session('')['found'] and 'run it once' in lr.session('')['why']
    assert 'no report.yaml' in lr.session(str(tmp_path))['why']
    assert lr.text(lr.session('')) == lr.session('')['why']


def test_a_hallway_sessions_fits_carry_their_error_bars(tmp_path):
    (tmp_path / 'fits.csv').write_text(
        'number,key,unit,mean,sd,n,lo90,hi90,resolution,now,trusted,why\n'
        'max_brake,actuators.max_brake,m/s^2,2.989,0.093,3,2.9,3.1,0,3,True,\n'
        'throttle_lag,actuators.throttle_lag,s,0.053,0.045,3,0.02,0.08,0.04,0.1,False,'
        'the runs disagree\n'
        'rolling,,m/s^2,nan,,0,,,,,False,no runs\n')
    brake, lag, rolling = lr.fits(str(tmp_path))
    assert (brake['value'], brake['sd'], brake['n'], brake['trusted']) == (2.989, 0.093, 3, True)
    assert not lag['trusted'] and lag['why'] == 'the runs disagree' and lag['now'] == 0.1
    assert rolling['value'] is None and rolling['n'] == 0
    assert all(v is None or not isinstance(v, float) or math.isfinite(v)
               for f in (brake, lag, rolling) for v in f.values())


def test_the_single_tests_come_from_the_runners_own_lists():
    from aav_actuator_tests import car_test as ct
    stands = [name for name, _, _ in lr.single_tests('stands')]
    assert stands == [s.name for s in ct.STEPS if s.sim is ct.Manner.AUTO]
    assert 'cable' not in stands and stands[-1] == 'estop'
    assert [n for n, _, _ in lr.single_tests('hallway')] == list(ct.HALL_KINDS)
    assert [n for n, _, _ in lr.single_tests('lot')] == list(ct.LOT_NAMES)
    with pytest.raises(ValueError):
        lr.single_tests('route')


def test_every_single_test_command_is_the_simulators_and_attaches():
    for rung in ladder.load_rungs():
        tests = ladder.single_tests(rung)
        assert bool(tests) == bool(rung.get('tests')), rung['id']
        for t in tests:
            assert ' --sim ' in t['command'] + ' ' and '--attach' in t['command'], t
            assert t['command'].split()[-1] == t['name']
        # A rung's car command never attaches, and no sim command lacks --sim.
        assert '--attach' not in rung['car'] and '--sim' not in rung['car']


def test_the_tool_prints_a_rungs_newest_session(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv('AAV_TESTS_DIR', str(tmp_path))
    monkeypatch.setenv('AAV_LADDER_FILE', str(tmp_path / 'ladder.yaml'))
    write_session(tmp_path / '2026-11-22-120000-stands-sim')
    assert ladder.main(['session', 'stands', '--json']) == 0
    out = json.loads(capsys.readouterr().out)
    assert out['rung'] == 'stands' and out['found'] and not out['recorded']
    assert out['steps'][0]['graph']['kind'] == 'steering'
    assert ladder.main(['session', 'hallway', '--json']) == 0
    assert not json.loads(capsys.readouterr().out)['found']
    assert ladder.main(['run', 'stands', '--only', 'no-such-test']) == 2
    # The recorded one: none yet; then the folder a result names, though a newer run exists.
    assert ladder.main(['session', 'stands', '--json', '--recorded']) == 0
    none = json.loads(capsys.readouterr().out)
    assert not none['found'] and 'no recorded session' in none['why']
    first = str(tmp_path / '2026-11-22-120000-stands-sim')
    ladder.record('stands', 'sim', 'pass', report=first)
    write_session(tmp_path / '2026-11-22-130000-stands-sim')
    assert ladder.main(['session', 'stands', '--json', '--recorded']) == 0
    recorded = json.loads(capsys.readouterr().out)
    assert recorded['folder'] == first and recorded['recorded']
    assert ladder.main(['session', 'stands', '--json']) == 0
    newest = json.loads(capsys.readouterr().out)
    assert newest['folder'].endswith('130000-stands-sim') and not newest['recorded']
