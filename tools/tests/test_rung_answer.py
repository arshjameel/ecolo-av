"""A rung's answer from a run's own files (tools/rung_answer.py): no ROS."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import ladder  # noqa: E402
import pytest  # noqa: E402
import rung_answer as ra  # noqa: E402
import step_response as sr  # noqa: E402
import yaml  # noqa: E402

DT = 0.02
RUNG = ladder.find(ladder.load_rungs(), 'stands')
ENTRY = {'commit': 'abc1234+', 'when': '2026-11-23 18:04'}


def follow(asked, lag, start=0.0):
    out, value = [], start
    for target in asked:
        value += (target - value) * DT / lag
        out.append(value)
    return out


def wheel_log(brake='follows'):
    """Return a wheels test: 0.1, 0.2, 0.3 m/s, then stop; the brake let go, then pressed."""
    asked = [0.0] * 50 + [0.1] * 200 + [0.2] * 200 + [0.3] * 200 + [0.0] * 100
    speed = follow(asked, 0.2)
    brake_cmd = [0.3] * 50 + [0.0] * 600 + [0.3] * 100
    measured = follow(brake_cmd, 0.1, start=0.3)
    rows = []
    for i, values in enumerate(zip(asked, speed, brake_cmd, measured)):
        cells = [f'{i * DT:g}'] + [f'{v:.6g}' for v in values]
        if brake == 'none':
            cells[4] = 'nan'
        rows.append(cells)
    return ['t', 'cmd_speed', 'speed', 'brake_cmd', 'brake'], rows


def steer_log():
    asked = [0.0] * 50 + [0.125] * 125 + [0.25] * 125 + [0.125] * 125 + [-0.125] * 125
    return ['t', 'cmd_steer', 'steer'], [
        [f'{i * DT:g}', f'{a:g}', f'{m:.6g}']
        for i, (a, m) in enumerate(zip(asked, follow(asked, 0.1)))]


FAULTS = {
    'heartbeat': ('5a', 'PASS', 'still 0.33 s after the output mux paused (no commands, no '
                  'heartbeat) (limit 2 s); stopped by the backend'),
    'gps': ('5b', 'PASS', 'the sensor watchdog ("STOP: gps STALE (nothing for 2.1 s (more '
            'than 2 s))") said why 2.11 s after its last message (limit 3 s); still 2.34 s '
            'after its last message, 0.23 s after the word (limit 3 s); the tests outrank '
            "the watchdog's lock (it holds navigation and the AI slot), so the runner stopped "
            'on its word'),
    'cable': ('5g', 'SKIPPED', 'no cables in Ravensim: on the car only'),
    'estop': ('5h', 'PASS', 'still 0.52 s after the e-stop (limit 2 s); heard: /safety/estop'),
}


def write_run(folder, steps=('steer-sweep', 'wheel-steps'), faults=FAULTS, brake='follows',
              status='PASS'):
    folder.mkdir(parents=True)
    listed = []
    for name, number, (header, rows) in (('steer-sweep', '3a', steer_log()),
                                         ('wheel-steps', '4', wheel_log(brake))):
        if name not in steps:
            continue
        run = folder / name / 'run'
        run.mkdir(parents=True)
        (run / 'log.csv').write_text('\n'.join(','.join(r) for r in [header] + rows) + '\n')
        listed.append({'step': name, 'number': number, 'status': status,
                       'detail': 'as it went', 'files': f'{name}/run'})
    listed += [{'step': name, 'number': number, 'status': state, 'detail': detail, 'files': ''}
               for name, (number, state, detail) in faults.items()]
    (folder / 'report.yaml').write_text(yaml.safe_dump({
        'started': '2026-11-23 18:00:55', 'where': 'a test', 'counts': {}, 'steps': listed}))
    return str(folder)


@pytest.fixture
def whole(monkeypatch):
    """Make the run's two logged steps and its faults count as the whole rung."""
    import ladder_report
    names = ['steer-sweep', 'wheel-steps'] + [n for n in FAULTS if n != 'cable']
    monkeypatch.setattr(ladder_report, 'single_tests',
                        lambda kind: [(n, '', '') for n in names])


def test_only_a_rung_with_a_builder_has_an_answer(tmp_path):
    speed = ladder.find(ladder.load_rungs(), 'speed')
    assert ra.answer(speed, str(tmp_path)) is None
    none = ra.answer(RUNG, '')
    assert not none['found'] and 'No run yet' in none['why'] and none['graphs'] == []
    assert ra.save(RUNG, str(tmp_path)) is None


def test_the_answer_is_a_verdict_three_graphs_and_the_faults(tmp_path, whole):
    a = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-180055-stands-sim'), ENTRY)
    assert a['found'] and a['place'] == 'the simulator' and a['number'] == '1'
    assert a['question'] == RUNG['question'] and a['chapter'] == 'The car and its model'
    assert a['commit'] == 'abc1234+' and a['command'] == RUNG['sim']
    assert a['verdict']['state'] == 'PASS'
    assert a['verdict']['words'].startswith(
        'PASS: the wheels follow each speed step up (a step down is not judged); the '
        'steering follows its commands; 3 of 3 faults tried were caught in time (2 stopped '
        'the car by themselves; for 1 the watchdog named the lost sensor and the test '
        'stopped the car).')
    assert 'Not tried here: a cable pulled' in a['verdict']['words']
    assert [g['id'] for g in a['graphs']] == ['wheels', 'steering', 'brake']
    assert 2 <= len(a['graphs']) + 1 <= 4, 'two to four graphs, the faults table among them'
    for g in a['graphs']:
        assert not g['missing'] and len(g['t']) == len(g['asked']) == len(g['measured'])
        assert len(g['t']) <= ra.SCREEN_POINTS + 1 and g['x_label'] == 'time (s)'
        assert '(' in g['y_label'] and 'dashed' in g['asked_words']
        assert all(len(row) == len(g['columns']) for row in g['rows'])
        assert g['from'].startswith('step ') and 'log.csv' in g['from']


def test_every_number_beside_a_graph_is_step_responses_on_the_logs_own_samples(tmp_path, whole):
    folder = write_run(tmp_path / '2026-11-23-180055-stands-sim')
    a = ra.answer(RUNG, folder, ENTRY)
    header, rows = wheel_log()
    t, asked, speed = ([float(r[i]) for r in rows] for i in (0, 1, 2))
    want = sr.responses(t, asked, speed, least=0.02)
    wheels = a['graphs'][0]
    assert [row[0] for row in wheels['rows']] == ['0.1', '0.2', '0.3', '0']
    for row, r in zip(wheels['rows'], want):
        assert row[1:] == [f'{r.delay:.2f} s', f'{r.rise:.2f} s', f'{r.end:.3f} m/s',
                           f'{r.off:+.3f} m/s']
    brake = a['graphs'][2]
    assert [row[0] for row in brake['rows']] == ['0 (let go)', '0.3 (pressed)']
    assert brake['facts'][0][0].startswith('asked to stop from 0.30 m/s')
    assert brake['facts'][0][1].endswith(' s')
    assert 'no pass mark for the brake' in brake['judged']
    steering = a['graphs'][1]
    assert steering['columns'][0] == "over the sweep's 4 steps"
    assert [row[0] for row in steering['rows']] == ['delay', '10 to 90 %',
                                                    'off at the end, either way']
    assert steering['facts'] == [['asked to the left (+), the measured angle went',
                                  'to the left (+): the right way']]


def test_what_the_files_do_not_hold_is_not_measured_never_made_up(tmp_path, whole):
    alone = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-190000-stands-sim',
                                      steps=('steer-sweep',), faults={}))
    wheels, steering, brake = alone['graphs']
    assert wheels['missing'] == ('not measured: this run did not include the step '
                                 'wheel-steps')
    assert wheels['t'] == [] and wheels['rows'] == [] and brake['missing']
    assert not steering['missing']
    assert alone['faults']['missing'] == 'not measured: this run tried no fault'
    assert alone['verdict']['state'] == 'ONE TEST ONLY'
    assert 'does not count' in alone['verdict']['words']
    assert alone['commit'].startswith('not recorded')
    # A car whose brake reports no position: the speed is drawn, the brake is not.
    blind = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-200000-stands-sim',
                                      brake='none'))
    assert not blind['graphs'][0]['missing']
    assert 'has no numbers in brake_cmd and brake' in blind['graphs'][2]['missing']


def test_a_step_that_did_not_get_there_says_so_beside_its_row(tmp_path, whole):
    folder = tmp_path / '2026-11-23-180055-stands-sim'
    write_run(folder)
    log = folder / 'wheel-steps' / 'run' / 'log.csv'
    lines = log.read_text().splitlines()
    for i in range(51, 251):                    # the first step: the wheels never move
        cells = lines[i].split(',')
        cells[2] = '0'
        lines[i] = ','.join(cells)
    log.write_text('\n'.join(lines) + '\n')
    wheels = ra.answer(RUNG, str(folder))['graphs'][0]
    assert wheels['rows'][0][1:3] == ['not measured', 'not measured']
    assert wheels['notes'][0].startswith('0.1: not measured, because it did not move 10 %')


def test_a_failed_step_fails_the_verdict_and_names_it(tmp_path, whole):
    faults = dict(FAULTS, gps=('5b', 'FAIL', 'nothing said why after its last message; '
                                             'NEVER still'))
    a = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-180055-stands-sim', faults=faults))
    assert a['verdict']['state'] == 'FAIL'
    assert a['verdict']['words'].startswith('FAIL: failed: 5b gps.')
    assert 'only 2 of 3 faults tried passed' in a['verdict']['words']
    gps = next(r for r in a['faults']['rows'] if r['name'] == 'gps')
    assert gps['stopped'] == 'NO' and gps['seconds'] == 'not measured'
    wheels = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-190000-stands-sim',
                                       status='FAIL'))
    assert 'The wheels: NOT passed' in wheels['verdict']['words']
    unfinished = dict(FAULTS, estop=('5h', 'NOT RUN', 'stopped before it'))
    b = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-200000-stands-sim',
                                  faults=unfinished))
    assert b['verdict']['state'] == 'NOT FINISHED'


def test_a_sensor_frozen_before_the_fault_is_not_shown_never_counted(tmp_path, whole):
    def camera(same, said):
        return dict(FAULTS, camera=('5d', 'PASS', (
            f'the sensor watchdog ("STOP: camera FROZEN (the same for {same} s)") said why '
            f'{said} s after its last message (limit 2 s); still 1.12 s after its last '
            'message, 0.23 s after the word (limit 3 s); the runner stopped on its word')))
    a = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-180055-stands-sim',
                                  faults=camera(1.1, '0.90')))
    row = next(r for r in a['faults']['rows'] if r['name'] == 'camera')
    assert row['stopped'] == 'not shown' and 'frozen for 1.1 s already' in row['from']
    assert '3 of 4 faults tried were caught in time' in a['verdict']['words']
    assert 'NOT SHOWN: ' + row['fault'] in a['verdict']['words']
    # Frozen since the fault: the watchdog's word is about the fault, so it counts.
    b = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-190055-stands-sim',
                                  faults=camera(1.1, '1.20')))
    row = next(r for r in b['faults']['rows'] if r['name'] == 'camera')
    assert row['stopped'] == 'caught; the test stopped it'
    assert 'NOT SHOWN' not in b['verdict']['words']


def test_the_faults_table_says_yes_or_no_and_in_how_long(tmp_path, whole):
    a = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-180055-stands-sim'))
    rows = {r['name']: r for r in a['faults']['rows']}
    assert (rows['heartbeat']['stopped'], rows['heartbeat']['seconds'],
            rows['heartbeat']['limit']) == ('yes', '0.33 s', '2 s')
    assert rows['heartbeat']['from'] == 'the output mux paused (no commands, no heartbeat)'
    # The car did not stop itself for a lost sensor: the test did, on the watchdog's word.
    assert rows['gps']['stopped'] == 'caught; the test stopped it'
    assert (rows['gps']['seconds'], rows['gps']['limit']) == ('0.23 s', '3 s')
    assert "It said why 2.11 s after the sensor's last message" in rows['gps']['from']
    assert rows['cable']['stopped'] == 'not tried' and 'on the car only' in rows['cable']['from']
    assert rows['estop']['fault'] == 'the e-stop'
    assert all(r['car'] == 'no car run yet' for r in rows.values())


def test_the_runners_own_sentences_read_back_whatever_it_judged():
    """car_test.judge_fault writes the sentence; a change there must not break the table."""
    from aav_actuator_tests import car_test as ct
    _, said = ct.judge_fault(ct.Seen('its last message', 2.11, 'the sensor watchdog', 2.34),
                             3.0, ct.STILL_AFTER)
    assert ra.fault_times(said)['seconds'] == pytest.approx(0.23)
    assert ra.fault_times(said)['limit'] == ct.STILL_AFTER
    _, plain = ct.judge_fault(ct.Seen('the e-stop', None, '', 0.52), None, ct.ESTOP_LIMIT,
                              need_word=False)
    assert ra.fault_times(plain) == {'stopped': True, 'seconds': 0.52,
                                     'limit': ct.ESTOP_LIMIT, 'from': 'the e-stop'}
    _, never = ct.judge_fault(ct.Seen('its last message', None, '', None), 3.0, 3.0)
    assert ra.fault_times(never)['stopped'] is False
    _, car = ct.judge_heartbeat_car(0.41, 0.0, False, 1)
    assert ra.fault_times(car)['seconds'] == pytest.approx(0.41)
    assert ra.fault_times(car)['limit'] is None
    _, unseen = ct.judge_heartbeat_car(None, None, None, None)
    assert ra.fault_times(unseen)['stopped'] is False
    assert ra.fault_times('the person typed skip')['stopped'] is None


def test_the_cars_side_is_a_slot_and_a_car_run_is_named_as_one(tmp_path, whole):
    a = ra.answer(RUNG, write_run(tmp_path / '2026-11-23-180055-stands-sim'))
    assert a['car'] == 'no car run yet'
    assert all(g['car'] == 'no car run yet' for g in a['graphs'])
    assert a['faults']['car'] == 'no car run yet'
    car = ra.answer(RUNG, write_run(tmp_path / '2026-11-24-100000-stands'), ENTRY)
    assert car['place'] == 'the car' and car['command'] == RUNG['car']


def test_the_saved_page_holds_the_question_the_verdict_the_graphs_and_the_numbers(
        tmp_path, whole):
    folder = write_run(tmp_path / '2026-11-23-180055-stands-sim')
    assert ra.answer(RUNG, folder)['report_file'] == ''
    path = ra.save(RUNG, folder, ENTRY)
    assert path == os.path.join(folder, 'rung-1-report.html')
    page = open(path).read()
    a = ra.answer(RUNG, folder, ENTRY)
    assert a['report_file'] == path
    for words in ('Rung 1: On stands: accelerate, brake, turn', RUNG['question'],
                  a['verdict']['words'].split(';')[0], 'abc1234+', '2026-11-23 18:00:55',
                  'The car and its model', 'No car run yet', '<details>'):
        assert words in page, words
    assert RUNG['sim'] in page
    assert page.count('<svg') == 3 and page.count('stroke-dasharray="8 5" points=') == 3
    assert '<script' not in page
    for g in a['graphs']:
        for row in g['rows']:
            for cell in row:
                assert cell.replace("'", '&#x27;') in page, cell
    for r in a['faults']['rows']:
        assert r['seconds'] in page


def test_a_graph_draws_every_kth_sample_and_never_an_average():
    lines = ([float(i) for i in range(1000)], [float(i % 7) for i in range(1000)])
    t, values = ra.thin(lines, 300)
    assert len(t) <= 301 and t[0] == 0 and t[-1] == 999
    assert all(v == float(int(time) % 7) for time, v in zip(t, values))


def test_the_axis_ticks_are_round_numbers():
    assert ra.ticks(0, 0.33, 4) == [0, 0.1, 0.2, 0.3]
    assert ra.ticks(-0.54, 0.54, 4) == [-0.5, 0, 0.5]
    assert ra.ticks(0, 45, 8) == [0, 10, 20, 30, 40]
