"""
Each pytest run inside `pixi run test` is on a ROS domain of its own.

tools/colcon_test.sh tests several packages at once; tools/pytest_plugins/aav_ros_domain.py
gives each pytest run a domain no other process here holds and says which in
AAV_TEST_DOMAIN. A test that picks its own domain (from its process id, at random, or by
free_domain()) could still land on another package's: so it takes AAV_TEST_DOMAIN first.
"""

import os
from pathlib import Path
import re
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
PLUGINS = ROOT / 'tools' / 'pytest_plugins'
CHOOSES = re.compile(r'getpid\(\)|random\.randint\(|free_domain\(\)')


def ros_tests():
    """Return the ROS packages' test files and conftests."""
    files = list(ROOT.glob('ros/**/test/test_*.py')) + list(ROOT.glob('ros/**/test/conftest.py'))
    return sorted(f for f in files if 'build' not in f.parts)


def test_a_test_that_picks_its_own_domain_takes_the_runs_first():
    missed = []
    for path in ros_tests():
        lines = path.read_text().splitlines()
        for i, line in enumerate(lines):
            here = lines[i - 1] + line if i else line
            if CHOOSES.search(line) and 'domain' in here.lower() and \
                    'AAV_TEST_DOMAIN' not in here:
                missed.append(f'{path.relative_to(ROOT)}:{i + 1}')
    assert not missed, f"take os.environ.get('AAV_TEST_DOMAIN') first: {missed}"


HOLD = """
import sys
sys.path.insert(0, sys.argv[1])
import aav_ros_domain
aav_ros_domain.pytest_configure(None)
import os
print(os.environ['ROS_DOMAIN_ID'], os.environ['AAV_TEST_DOMAIN'], flush=True)
sys.stdin.readline()
aav_ros_domain.pytest_unconfigure(None)
"""


def test_two_runs_at_once_get_two_domains():
    pytest.importorskip('domain_coordinator')
    runs = [subprocess.Popen([sys.executable, '-c', HOLD, str(PLUGINS)], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, text=True, env=dict(os.environ))
            for _ in range(3)]
    try:
        seen = [r.stdout.readline().split() for r in runs]
    finally:
        for r in runs:
            r.communicate('\n', timeout=10)
    domains = [int(ros) for ros, test in seen]
    assert all(ros == test for ros, test in seen)
    assert len(set(domains)) == 3, domains
    assert all(1 <= d <= 100 for d in domains), domains
