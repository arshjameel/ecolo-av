"""
The Python still gives the Rust twin's vectors (aav_drive_to_point_rs, point-core).

gen_vectors.py writes crates/point-core/tests/vectors.json from these modules; the Rust
replays it (cargo test -p point-core). This runs the generator again and compares: a change
to approach.py, line.py, drive.py, rules.py or run.py fails here until the vectors are
written again (and the Rust then fails until it matches).
"""

import importlib.util
import json
import math
from pathlib import Path

import pytest

RS = Path(__file__).resolve().parents[2] / 'aav_drive_to_point_rs'
VECTORS = RS / 'crates' / 'point-core' / 'tests' / 'vectors.json'


def same(a, b, where='', tol=1e-12):
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


@pytest.mark.skipif(not VECTORS.exists(), reason='no aav_drive_to_point_rs here')
def test_the_python_gives_the_rust_vectors():
    spec = importlib.util.spec_from_file_location('gen_vectors', RS / 'gen_vectors.py')
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    now = json.loads(json.dumps(gen.enc(gen.build())))
    same(now, json.loads(VECTORS.read_text()))
