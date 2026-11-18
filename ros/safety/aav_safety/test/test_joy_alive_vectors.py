"""
The vectors the Rust joy_alive is checked against hold this package's answers.

aav_joy_alive_rs/gen_vectors.py drives joy_alive.py through runs of Joy messages and
ticks; joy-alive-core's `cargo test` must give the same answers, event by event. Here:
running it again now gives exactly the committed file. After a change to joy_alive.py,
rerun gen_vectors.py (and port the change).
"""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
GENERATOR = ROOT / 'ros/safety/aav_joy_alive_rs/gen_vectors.py'
VECTORS = ROOT / 'ros/safety/aav_joy_alive_rs/crates/joy-alive-core/tests/vectors.json'


def generator():
    spec = importlib.util.spec_from_file_location('joy_alive_gen_vectors', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_rust_vectors_are_what_this_package_answers():
    """vectors.json holds joy_alive.py's answers: rerun gen_vectors.py after a change."""
    assert json.loads(VECTORS.read_text()) == generator().build()


def test_the_vectors_reach_every_state():
    runs = json.loads(VECTORS.read_text())['runs']
    trips = {e['tripped'] for r in runs for e in r['events']}
    assert trips == {None, 'WAITING', 'SILENT', 'FROZEN'}
    lines = {e['state'].split(' - ')[1].split(' ')[0] for r in runs for e in r['events']
             if e['op'] == 'tick'}
    assert {'driving', 'idle', 'release', 'nothing', 'the'} <= lines
