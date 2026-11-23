"""The test ladder's rules (tools/ladder.py): what opens a rung, and what is kept."""
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
    return path


def test_the_rungs_file_reads_and_each_rung_needs_only_rungs_below_it():
    rungs = ladder.load_rungs()
    assert [r['id'] for r in rungs][:2] == ['code', 'stack']
    assert all({'title', 'passes', 'sim', 'car', 'how'} <= set(r) for r in rungs)
    for r in rungs:
        assert os.path.exists(os.path.join(ladder.ROOT, r['how'])), r['how']


def test_a_rung_is_shut_until_the_one_below_has_passed(progress):
    rungs = ladder.load_rungs()
    assert ladder.blockers(rungs, [], 'code', 'sim') == []
    assert ladder.blockers(rungs, [], 'stack', 'sim')
    ladder.record('code', 'sim', 'pass')
    assert ladder.blockers(rungs, ladder.load_progress(), 'stack', 'sim') == []


def test_a_pass_in_the_simulator_never_marks_the_car(progress):
    rungs = ladder.load_rungs()
    for rung in ('code', 'stack', 'bench', 'stands'):
        ladder.record(rung, 'sim', 'pass')
    results = ladder.load_progress()
    assert ladder.passed(results, 'stands', 'sim')
    assert not ladder.passed(results, 'stands', 'car')
    # code and stack have no car side: the car's next rung is the bench.
    assert ladder.next_rung(rungs, results, 'car')['id'] == 'bench'


def test_a_car_rung_needs_its_own_simulator_twin(progress):
    rungs = ladder.load_rungs()
    for rung in ('code', 'stack'):
        ladder.record(rung, 'sim', 'pass')
    ladder.record('bench', 'car', 'pass', override='for the test')
    shut = ladder.blockers(rungs, ladder.load_progress(), 'stands', 'car')
    assert shut == ['stands has not passed in the simulator']


def test_the_newest_result_counts_and_the_history_stays(progress):
    ladder.record('code', 'sim', 'pass')
    ladder.record('code', 'sim', 'fail', note='a test broke')
    results = ladder.load_progress()
    assert len(results) == 2
    assert not ladder.passed(results, 'code', 'sim')
    assert ladder.next_rung(ladder.load_rungs(), results, 'sim')['id'] == 'code'


def test_a_pass_on_a_shut_rung_needs_an_override_and_it_is_kept(progress):
    assert ladder.main(['record', 'stack', 'pass']) == 2
    assert ladder.load_progress() == []
    assert ladder.main(['record', 'stack', 'pass', '--override', 'checked by hand']) == 0
    assert ladder.load_progress()[0]['override'] == 'checked by hand'


def test_a_car_test_report_gives_the_rung_the_place_and_the_result(tmp_path):
    for name, counts, want in (
            ('2026-11-20-222702-stands-sim', {'PASS': 12, 'FAIL': 0, 'NOT RUN': 0},
             ('stands', 'sim', 'pass')),
            ('2026-11-21-101500-hallway', {'PASS': 5, 'FAIL': 1, 'NOT RUN': 0},
             ('hallway', 'car', 'fail')),
            ('2026-11-21-111500-stands', {'PASS': 5, 'FAIL': 0, 'NOT RUN': 2},
             ('stands', 'car', 'fail'))):
        folder = tmp_path / name
        folder.mkdir()
        (folder / 'report.yaml').write_text(yaml.safe_dump({'counts': counts}))
        assert ladder.from_report(str(folder)) == want


def test_the_table_has_both_places_and_says_what_blocks(progress):
    ladder.record('code', 'sim', 'pass')
    rows = ladder.table(ladder.load_rungs(), ladder.load_progress())
    # code has no car side: its simulator pass shows on both sides. Stands has one.
    assert rows[0]['sim']['state'] == 'pass' and rows[0]['car']['state'] == 'pass'
    assert rows[3]['id'] == 'stands' and rows[3]['car']['state'] == 'not run'
    assert rows[1]['sim']['blocked_by'] == [] and rows[2]['sim']['blocked_by']


def test_the_json_keeps_the_shape_the_window_reads(progress, capsys):
    """The window (ladder-core) parses --json: a field renamed here breaks it there."""
    fixture = os.path.join(ladder.ROOT, 'ros', 'hud', 'aav_dev_window', 'crates',
                           'ladder-core', 'tests', 'ladder.json')
    with open(fixture) as f:
        want = json.load(f)
    assert ladder.main(['--json']) == 0
    got = json.loads(capsys.readouterr().out)
    assert set(got) == set(want) == {'file', 'rungs'}
    assert [r['id'] for r in got['rungs']] == [r['id'] for r in want['rungs']]
    for mine, theirs in zip(got['rungs'], want['rungs']):
        assert set(mine) == set(theirs)
        for place in ladder.PLACES:
            assert set(mine[place]) == set(theirs[place])
            assert mine[place]['state'] in ('pass', 'fail', 'not run')
            assert isinstance(mine[place]['blocked_by'], list)


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
