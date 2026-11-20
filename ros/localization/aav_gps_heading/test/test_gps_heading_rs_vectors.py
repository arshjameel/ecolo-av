"""
The vectors the Rust gps_heading is checked against hold this package's answers.

aav_gps_heading_rs/gen_vectors.py runs course.py on 4,000 velocities, wheel speeds and
antenna places; gps-heading-core's `cargo test` must give the same answers to the bit.
Here: running it again now gives exactly the committed file. After a change to course.py,
rerun gen_vectors.py (and port the change).
"""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
GENERATOR = ROOT / 'ros/localization/aav_gps_heading_rs/gen_vectors.py'
VECTORS = ROOT / 'ros/localization/aav_gps_heading_rs/crates/gps-heading-core/tests/vectors.json'


def generator():
    spec = importlib.util.spec_from_file_location('gps_heading_rs_gen_vectors', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_rust_vectors_are_what_this_package_answers():
    """vectors.json holds course.py's answers: rerun gen_vectors.py after a change."""
    assert json.loads(VECTORS.read_text()) == generator().build()


def test_the_vectors_answer_and_refuse():
    cases = json.loads(VECTORS.read_text())['heading']
    answered = sum(c['out'] is not None for c in cases)
    assert 1000 < answered < len(cases) - 1000
