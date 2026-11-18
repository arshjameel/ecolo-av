"""
The vectors the Rust stop_sign_watcher is checked against hold this package's answers.

aav_stop_sign_rs/gen_vectors.py drives stop_sign.py (the watcher, the sign's point, the
biggest sign, the lidar's sign) through random and hand-picked cases; stop-sign-core's
`cargo test` must give the same answers. Here: running it again now gives exactly the
committed file. After a change to stop_sign.py, rerun gen_vectors.py (and port the change).
"""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
GENERATOR = ROOT / 'ros/perception/aav_stop_sign_rs/gen_vectors.py'
VECTORS = ROOT / 'ros/perception/aav_stop_sign_rs/crates/stop-sign-core/tests/vectors.json'


def generator():
    spec = importlib.util.spec_from_file_location('stop_sign_gen_vectors', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_rust_vectors_are_what_this_package_answers():
    """vectors.json holds stop_sign.py's answers: rerun gen_vectors.py after a change."""
    assert json.loads(VECTORS.read_text()) == generator().build()


def test_the_vectors_reach_every_state():
    runs = json.loads(VECTORS.read_text())['runs']
    states = {s['state'] for r in runs for s in r['steps']}
    assert states == {'clear', 'stopping', 'passing'}
