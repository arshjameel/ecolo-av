"""tools/step_plots.py: finding the runs and the running mean (no matplotlib needed)."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import step_plots  # noqa: E402


def test_the_test_name_comes_from_the_run_folder():
    for folder, test in (('2026-11-19-034218-steer-rate-sim', 'steer-rate'),
                         ('2026-11-21-101500-wheel-steps', 'wheel-steps'),
                         ('2026-10-29-104818-preflight-bench', 'preflight')):
        assert step_plots.RUN.match(folder).group('test') == test


def test_the_running_mean_smooths_and_skips_gaps():
    times = [i * 0.1 for i in range(11)]
    values = [0.0, 1.0] * 5 + [0.0]
    out = step_plots.smooth(times, values, 0.5)
    assert all(0.3 < v < 0.7 for v in out[3:-3])
    with_gap = [1.0, math.nan, 1.0]
    assert step_plots.smooth([0.0, 0.1, 0.2], with_gap, 1.0) == [1.0, 1.0, 1.0]


def test_runs_are_found_by_their_log(tmp_path):
    run = tmp_path / 'session' / 'brake' / '2026-11-21-101500-brake'
    run.mkdir(parents=True)
    (run / 'log.csv').write_text('t,phase,speed\n0,go,0\n0.1,go,0.5\n')
    found = step_plots.find_runs(str(tmp_path))
    assert list(found) == ['brake']
    cols = step_plots.read_run(found['brake'][0])
    assert cols['speed'] == [0.0, 0.5] and cols['phase'] == ['go', 'go']


def test_a_hallway_pass_is_found_with_its_run(tmp_path):
    session = tmp_path / 'hall'
    (session / 'overlays').mkdir(parents=True)
    (session / 'overlays' / '01-hall-brake-there.csv').write_text(
        't,measured,wheels,model_fitted,model_vehicle_yaml\n0,0,0,0,0\n')
    run = session / '01-hall-brake-there' / '2026-11-21-101500-hall-brake'
    run.mkdir(parents=True)
    (run / 'log.csv').write_text('t,phase,ego_speed\n0,go,0\n')
    passes = step_plots.hallway_passes(str(session))
    assert [(stem, os.path.basename(log)) for stem, _, log in passes] == [
        ('01-hall-brake-there', '2026-11-21-101500-hall-brake')]
