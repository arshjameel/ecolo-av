"""
The vectors the Rust muxes are checked against hold this package's answers.

aav_io_mux_rs/gen_vectors.py drives muxes.py (and so provenance.py and heartbeat.py)
through every story and random run; io-mux-core's `cargo test` must give the same
answers, step by step. Here: running it again now gives exactly the committed file,
so the Python and the Rust answer the same inputs the same way. After a change to the
Python, rerun gen_vectors.py (and port the change).
"""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
GENERATOR = ROOT / 'ros/io_mux/aav_io_mux_rs/gen_vectors.py'
VECTORS = ROOT / 'ros/io_mux/aav_io_mux_rs/crates/io-mux-core/tests/vectors.json'


def generator():
    spec = importlib.util.spec_from_file_location('io_mux_gen_vectors', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_rust_vectors_are_what_this_package_answers():
    """vectors.json holds this package's answers: rerun gen_vectors.py after a change."""
    assert json.loads(VECTORS.read_text()) == generator().build()


def test_the_vectors_reach_every_state_and_branch():
    runs = json.loads(VECTORS.read_text())['runs']
    steps = [s for r in runs for s in r['steps']]
    lines = {s['published']['line'].split(' ')[0] for s in steps
             if isinstance(s.get('published'), dict)}
    assert lines == {'OPEN', 'WAIT', 'STOP'}
    logs = {s['log'][1].split(':')[0].split(' (')[0] for s in steps if s.get('log')}
    assert {'unsure, waiting for /system/source', 'unsure'} <= logs
    assert any(t.startswith('NOT SIM') for t in logs)
    counters = [s['counter'] for s in steps if s['do'] == 'command' and s['counter'] is not None]
    assert any(a == 255 and b == 0 for a, b in zip(counters, counters[1:]))
    assert {s['success'] for s in steps if s['do'] == 'clear'} == {True, False}
    refusals = {r['error'] for r in runs if r['error']}
    assert 'target:=car takes no screens: a test-only rule' in refusals
    assert any(e.startswith('target:=gazebo') for e in refusals)
    assert any(e.startswith('source:=gazebo') for e in refusals)
