"""pixi run jacks: what it refuses, and what it prints (no ROS)."""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import jacks  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), '..', '..')


def test_without_sim_it_refuses_before_any_ros(capsys):
    assert jacks.main(['up']) == jacks.NOT_A_SIM
    out = capsys.readouterr().out
    assert out.startswith('REFUSED: --sim is needed') and 'by hand' in out


def test_a_real_car_anywhere_refuses_even_beside_a_simulator():
    sources = {'sim_adapter': [jacks.SIM, jacks.SIM], 'legacy_uart_bridge': [jacks.REAL]}
    why = jacks.why_not(True, sources)
    assert 'REAL' in why and 'legacy_uart_bridge' in why and 'Nothing was sent' in why


def test_no_simulator_the_bench_and_a_replay_refuse():
    assert 'no simulator heard' in jacks.why_not(True, {})
    assert 'no simulator heard' in jacks.why_not(True, {'quiet': []})
    assert 'only Ravensim' in jacks.why_not(True, {'spine_sim': [jacks.BENCH]})
    assert 'only Ravensim' in jacks.why_not(True, {'bag': [jacks.REPLAY]})


def test_a_simulator_alone_is_accepted():
    assert jacks.why_not(True, {'sim_adapter': [jacks.SIM], 'sim_gps': [jacks.SIM]}) is None


def test_the_answer_in_words_and_exit_codes():
    done = jacks.report('up', True, True, True, 'the car is now on stands')
    assert jacks.text(done) == ('Done: the car is now on stands\n'
                                'The simulated car: on stands')
    assert jacks.exit_code(done) == jacks.OK
    refused = jacks.report('down', True, True, False, 'refused: the wheels still spin')
    assert jacks.text(refused).startswith('REFUSED: refused: the wheels still spin')
    assert jacks.exit_code(refused) == jacks.REFUSED
    status = jacks.report('status', True, False)
    assert jacks.text(status) == 'The simulated car: on the ground'
    assert 'unknown' in jacks.text(jacks.report('status', True, None))
    assert json.loads(json.dumps(status))['on_stands'] is False


def test_the_task_runs_local_only():
    """A stand-in's switch never reaches a real car's network (CLAUDE.md, Testing)."""
    with open(os.path.join(ROOT, 'pixi.toml')) as f:
        line = next(x for x in f if x.startswith('jacks = '))
    assert 'tools/ros.sh --local python3 tools/jacks.py' in line
