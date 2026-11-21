"""
The vectors the Rust geofence, stop hold and leash are checked against hold this package's answers.

aav_safety_rs/gen_vectors.py drives geofence.py, hold.py and leash.py through runs of
poses, fixes, checks, clears, stops and beats; safety-core's `cargo test` must give the
same answers, event by event and bit by bit. Here: running it again now gives the
committed file, numbers within 1e-9 (20 Nov: Linux's maths library rounds sin and cos
apart from macOS's in the last bits, so an exact compare failed in CI only), everything
else exactly. After a change to any of the three, rerun gen_vectors.py (and port the
change).
"""

import importlib.util
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
GENERATOR = ROOT / 'ros/safety/aav_safety_rs/gen_vectors.py'
VECTORS = ROOT / 'ros/safety/aav_safety_rs/crates/safety-core/tests/vectors.json'


def generator():
    spec = importlib.util.spec_from_file_location('safety_rs_gen_vectors', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def same(a, b, where='', tol=1e-9):
    """Compare two decoded JSON trees: numbers to tol, everything else exactly."""
    if isinstance(a, dict):
        assert isinstance(b, dict) and a.keys() == b.keys(), where
        for k in a:
            same(a[k], b[k], f'{where}.{k}', tol)
    elif isinstance(a, list):
        assert isinstance(b, list) and len(a) == len(b), where
        for i, (x, y) in enumerate(zip(a, b)):
            same(x, y, f'{where}[{i}]', tol)
    elif isinstance(a, float) or isinstance(b, float):
        assert not isinstance(a, bool) and not isinstance(b, bool), where
        assert math.isclose(a, b, rel_tol=tol, abs_tol=tol), f'{where}: {a} != {b}'
    else:
        assert a == b, f'{where}: {a!r} != {b!r}'


def test_the_rust_vectors_are_what_this_package_answers():
    """vectors.json holds the Python's answers: rerun gen_vectors.py after a change."""
    same(json.loads(VECTORS.read_text()), json.loads(json.dumps(generator().build())))


def test_the_vectors_reach_every_state():
    runs = json.loads(VECTORS.read_text())['geofence']
    states = {e['out'][1] for r in runs for e in r['events'] if e['op'] == 'check'}
    assert states == {'STARTING', 'OK', 'NEAR', 'OUTSIDE', 'LOST', 'JUMP', 'COASTING',
                      'STANDS'}
    clears = {e['out'][1].split(' ')[0] for r in runs for e in r['events']
              if e['op'] == 'clear'}
    assert {'nothing', 'no', 'cleared', 'the'} <= clears
