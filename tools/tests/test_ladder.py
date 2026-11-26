"""The test ladder's rules (tools/ladder.py): the ten rungs, what opens one, what is kept."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import ladder  # noqa: E402
import pytest  # noqa: E402
import yaml  # noqa: E402


@pytest.fixture
def progress(tmp_path, monkeypatch):
    path = tmp_path / 'ladder.yaml'
    monkeypatch.setenv('AAV_LADDER_FILE', str(path))
    monkeypatch.setenv('AAV_TESTS_DIR', str(tmp_path))
    monkeypatch.setattr(ladder.tempfile, 'gettempdir', lambda: str(tmp_path))
    return path


def test_the_rungs_are_the_plans_ten_each_with_its_question():
    rungs = ladder.load_rungs()
    assert [str(r['number']) for r in rungs] == ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'S']
    for r in rungs:
        assert {'id', 'title', 'question', 'graphs', 'passes', 'chapter', 'built', 'sim',
                'car', 'how'} <= set(r), r['id']
        assert r['question'].strip().endswith('?') and r['graphs'], r['id']
        assert os.path.exists(os.path.join(ladder.ROOT, r['how'])), r['how']
    # The plan's own table (docs/wrap-up.md) names the same rungs.
    with open(os.path.join(ladder.ROOT, 'docs', 'wrap-up.md')) as f:
        plan = f.read()
    for r in rungs:
        assert f"| {r['number']} | {r['title']} |" in plan, r['title']


def test_a_rung_is_found_by_its_id_or_its_number():
    rungs = ladder.load_rungs()
    assert ladder.find(rungs, '1')['id'] == ladder.find(rungs, 'stands')['id'] == 'stands'
    assert ladder.find(rungs, 's')['id'] == ladder.find(rungs, 'S')['id'] == 'sensors'
    assert ladder.find(rungs, 'hallway') is None and ladder.find(rungs, '10') is None


def test_a_rung_not_built_is_never_run_recorded_passed_or_failed(progress, capsys):
    rungs = ladder.load_rungs()
    unbuilt = [r for r in rungs if not ladder.built(r)]
    assert [r['id'] for r in rungs if ladder.built(r)] == ['stands']
    assert all(not r['sim'] and not r['car'] and r['today'] for r in unbuilt)
    for args in (['run', '2'], ['record', 'speed', 'pass'],
                 ['record', 'speed', 'pass', '--override', 'because'],
                 ['record', 'speed', 'fail', '--car']):
        assert ladder.main(args) == 2
        assert 'not built yet' in capsys.readouterr().err
    assert ladder.load_progress() == []
    # Even a line written into the file by hand shows as "not built".
    progress.write_text(yaml.safe_dump({'results': [
        {'rung': 'speed', 'place': 'sim', 'result': 'pass', 'when': '2026-11-23 10:00'}]}))
    row = ladder.table(rungs, ladder.load_progress())[1]
    assert row['sim']['state'] == row['car']['state'] == 'not built'
    assert row['sim']['command'] == '' and row['built'] is False


def test_a_rung_is_shut_until_the_one_below_has_passed(progress):
    rungs = ladder.load_rungs()
    assert ladder.blockers(rungs, [], 'stands', 'sim') == []
    assert ladder.blockers(rungs, [], 'speed', 'sim') == ['rung 1 (stands) has not passed (sim)']
    ladder.record('stands', 'sim', 'pass')
    assert ladder.blockers(rungs, ladder.load_progress(), 'speed', 'sim') == []


def test_the_checks_are_not_rungs_and_never_shut_one(progress, capsys):
    rungs, checks = ladder.load_rungs(), ladder.load_checks()
    assert [c['id'] for c in checks] == ['code', 'stack', 'bench']
    assert not {c['id'] for c in checks} & {r['id'] for r in rungs}
    for check in ('code', 'stack', 'bench'):
        ladder.record(check, 'sim', 'fail')
    results = ladder.load_progress()
    assert ladder.blockers(rungs, results, 'stands', 'sim') == []
    assert ladder.next_rung(rungs, results, 'sim')['id'] == 'stands'
    before = ladder.checks_table(checks, results)
    assert [c['state'] for c in before] == ['fail', 'fail', 'fail']
    assert before[0]['command'] == 'pixi run bash tools/test.sh'
    # A check is run by the ladder, never recorded by hand as a rung.
    with pytest.raises(SystemExit):
        ladder.main(['record', 'code', 'pass'])


def test_results_saved_under_the_old_rungs_names_are_read_forward(progress):
    """The file from before 23 Nov is never rewritten: stands is rung 1, the rest is said."""
    old = [{'rung': name, 'place': 'sim', 'result': result, 'when': f'2026-11-23 18:0{i}',
            'commit': 'abc'}
           for i, (name, result) in enumerate((
               ('code', 'pass'), ('stack', 'pass'), ('bench', 'fail'), ('stands', 'pass'),
               ('hallway', 'fail'), ('hallway', 'pass'), ('lot', 'pass'), ('route', 'pass')))]
    text = yaml.safe_dump({'results': old})
    progress.write_text(text)
    results = ladder.load_progress()
    rows = ladder.table(ladder.load_rungs(), results)
    assert rows[0]['id'] == 'stands' and rows[0]['sim']['state'] == 'pass'
    assert all(r['sim']['state'] == 'not built' for r in rows[1:])
    checks = ladder.checks_table(ladder.load_checks(), results)
    assert [c['state'] for c in checks] == ['pass', 'pass', 'fail']
    note = ladder.old_results(results)
    assert 'hallway (2)' in note and 'lot (1)' in note and 'route (1)' in note
    assert ladder.old_results(results[:4]) == ''
    assert ladder.main([]) == 0 and ladder.main(['--json']) == 0
    assert progress.read_text() == text, 'reading never rewrites the file'


def test_a_pass_in_the_simulator_never_marks_the_car(progress):
    rungs = ladder.load_rungs()
    ladder.record('stands', 'sim', 'pass')
    results = ladder.load_progress()
    assert ladder.passed(results, 'stands', 'sim')
    assert not ladder.passed(results, 'stands', 'car')
    assert ladder.next_rung(rungs, results, 'car')['id'] == 'stands'
    assert ladder.next_rung(rungs, results, 'sim') is None


def test_a_car_rung_needs_its_own_simulator_run(progress):
    rungs = ladder.load_rungs()
    shut = ladder.blockers(rungs, [], 'stands', 'car')
    assert shut == ['rung 1 (stands) has not passed in the simulator']
    ladder.record('stands', 'sim', 'pass')
    assert ladder.blockers(rungs, ladder.load_progress(), 'stands', 'car') == []


def test_the_newest_result_counts_and_the_history_stays(progress):
    ladder.record('stands', 'sim', 'pass')
    ladder.record('stands', 'sim', 'fail', note='a fault did not stop it')
    results = ladder.load_progress()
    assert len(results) == 2
    assert not ladder.passed(results, 'stands', 'sim')
    assert ladder.next_rung(ladder.load_rungs(), results, 'sim')['id'] == 'stands'


def test_a_pass_on_a_shut_rung_needs_an_override_and_it_is_kept(progress):
    assert ladder.main(['record', '1', 'pass', '--car']) == 2
    assert ladder.load_progress() == []
    assert ladder.main(['record', '1', 'pass', '--car', '--override', 'checked by hand']) == 0
    kept = ladder.load_progress()[0]
    assert kept['override'] == 'checked by hand' and kept['rung'] == 'stands'


def test_a_car_test_report_gives_the_rung_the_place_and_the_result(tmp_path):
    for name, counts, want in (
            ('2026-11-20-222702-stands-sim', {'PASS': 12, 'FAIL': 0, 'NOT RUN': 0},
             ('stands', 'sim', 'pass')),
            ('2026-11-21-111500-stands', {'PASS': 5, 'FAIL': 0, 'NOT RUN': 2},
             ('stands', 'car', 'fail'))):
        folder = tmp_path / name
        folder.mkdir()
        (folder / 'report.yaml').write_text(yaml.safe_dump({'counts': counts}))
        assert ladder.from_report(str(folder)) == want
    hallway = tmp_path / '2026-11-21-101500-hallway'
    hallway.mkdir()
    (hallway / 'report.yaml').write_text(yaml.safe_dump({'counts': {'PASS': 5}}))
    with pytest.raises(ValueError, match='not the run of a rung that is built'):
        ladder.from_report(str(hallway))


def test_the_table_has_both_places_and_says_what_blocks(progress):
    rows = ladder.table(ladder.load_rungs(), [])
    assert rows[0]['id'] == 'stands' and rows[0]['number'] == '1'
    assert rows[0]['sim']['state'] == rows[0]['car']['state'] == 'not run'
    assert rows[0]['sim']['blocked_by'] == [] and rows[0]['car']['blocked_by']
    assert rows[0]['question'] == 'Do the actuators follow a command, the right way?'
    assert rows[0]['chapter'] == 'The car and its model'


def test_the_json_keeps_the_shape_the_window_reads(progress, capsys):
    """The window (ladder-core) parses --json: a field renamed here breaks it there."""
    fixture = os.path.join(ladder.ROOT, 'ros', 'hud', 'aav_dev_window', 'crates',
                           'ladder-core', 'tests', 'ladder.json')
    with open(fixture) as f:
        want = json.load(f)
    assert ladder.main(['--json']) == 0
    got = json.loads(capsys.readouterr().out)
    assert set(got) == set(want) == {'file', 'rungs', 'checks', 'note'}
    assert [r['id'] for r in got['rungs']] == [r['id'] for r in want['rungs']]
    for mine, theirs in zip(got['rungs'], want['rungs']):
        assert mine == {**theirs, 'sim': mine['sim'], 'car': mine['car']}, \
            'the fixture is stale: see its comment in ladder-core/src/ladder.rs'
        for place in ladder.PLACES:
            assert set(mine[place]) == set(theirs[place])
            assert mine[place]['state'] in ('pass', 'fail', 'not run', 'not built')
            assert isinstance(mine[place]['blocked_by'], list)
    for mine, theirs in zip(got['checks'], want['checks']):
        assert set(mine) == set(theirs) and mine['id'] == theirs['id']


def test_the_text_lists_the_top_rung_first_and_the_checks_on_one_line(progress, capsys):
    ladder.record('code', 'sim', 'pass', when='2026-11-23 09:00')
    assert ladder.main([]) == 0
    lines = capsys.readouterr().out.splitlines()
    assert lines[1].startswith('9  Rules of the road') and 'not built yet' in lines[1]
    assert lines[9].startswith('1  On stands')
    before = next(line for line in lines if line.startswith('Before you start'))
    assert "the code's own tests: PASS 2026-11-23" in before and 'not run' in before


def test_a_run_finds_the_report_folder_it_made(tmp_path):
    for name in ('2026-11-22-100000-stands-sim', '2026-11-22-120000-stands-sim',
                 '2026-11-22-130000-hallway-sim', '2026-11-22-140000-stands'):
        (tmp_path / name).mkdir()
        (tmp_path / name / 'report.yaml').write_text('counts: {PASS: 1}\n')
    (tmp_path / '2026-11-22-150000-stands-sim').mkdir()        # no report: not a session
    import datetime
    noon = datetime.datetime(2026, 11, 22, 11, 0, 0).timestamp()
    found = ladder.newest_session('stands', noon, str(tmp_path))
    assert found.endswith('2026-11-22-120000-stands-sim')
    late = datetime.datetime(2026, 11, 22, 16, 0, 0).timestamp()
    assert ladder.newest_session('stands', late, str(tmp_path)) == ''
    assert ladder.newest_session('stands', None, str(tmp_path)).endswith('120000-stands-sim')
    assert ladder.newest_session('lot', None, str(tmp_path)) == ''


def test_a_session_where_nothing_passed_is_not_a_pass(tmp_path):
    folder = tmp_path / '2026-11-22-120000-stands-sim'
    folder.mkdir()
    (folder / 'report.yaml').write_text('counts: {PASS: 0, FAIL: 0, SKIPPED: 2}\n')
    assert ladder.from_report(str(folder)) == ('stands', 'sim', 'fail')
    (folder / 'report.yaml').write_text('counts: {PASS: 3, FAIL: 0, SKIPPED: 2}\n')
    assert ladder.from_report(str(folder)) == ('stands', 'sim', 'pass')


def test_a_run_records_the_result_and_where_its_report_page_is(
        progress, tmp_path, monkeypatch, capsys):
    """`run` with a stand-in for the rung's command: the page is saved, and remembered."""
    rungs = ladder.load_rungs()
    folder = tmp_path / '2099-01-01-000000-stands-sim'

    def fake_run(command, **_):
        assert command == rungs[0]['sim']
        folder.mkdir()
        (folder / 'report.yaml').write_text(yaml.safe_dump({
            'started': '2099-01-01 00:00:00', 'where': 'a test', 'counts': {'PASS': 1},
            'steps': [{'step': 'estop', 'number': '5h', 'status': 'PASS', 'files': '',
                       'detail': 'still 0.52 s after the e-stop (limit 2 s)'}]}))

        class Done:
            returncode = 0
        return Done()

    monkeypatch.setattr(ladder.subprocess, 'run', fake_run)
    monkeypatch.setattr(ladder, 'commit', lambda: 'abc1234')
    assert ladder.main(['run', '1']) == 0
    kept = ladder.load_progress()[-1]
    page = str(folder / 'rung-1-report.html')
    assert kept['report'] == str(folder) and kept['report_file'] == page
    assert os.path.isfile(page) and 'abc1234' in open(page).read()
    assert f'Its report: {page}' in capsys.readouterr().out
    assert ladder.table(rungs, ladder.load_progress())[0]['sim']['report_file'] == page
    assert ladder.main(['report', 'stands']) == 0
    assert capsys.readouterr().out.strip() == page


def test_a_second_run_is_refused_while_one_is_going_and_records_nothing(progress, capsys):
    with ladder.only_run() as free:
        assert free
        assert ladder.main(['run', '1']) == 2
        assert ladder.main(['run', '1', '--only', 'estop']) == 2
    assert 'one car, one run at a time' in capsys.readouterr().err
    assert ladder.load_progress() == []
    with ladder.only_run() as again:      # the first is over: the place is free
        assert again


def test_a_run_is_recorded_under_its_own_folder_not_one_started_over_it(tmp_path, progress):
    for name in ('2026-11-24-003107-stands-sim', '2026-11-24-003358-stands-sim'):
        (tmp_path / name).mkdir()
        (tmp_path / name / 'report.yaml').write_text('counts: {}')
    began = ladder.datetime.datetime(2026, 11, 24, 0, 31, 5).timestamp()
    assert ladder.newest_session('stands', began, first=True).endswith('003107-stands-sim')
    assert ladder.newest_session('stands', None).endswith('003358-stands-sim')
