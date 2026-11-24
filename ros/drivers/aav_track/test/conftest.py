"""The tests import this package, drive_to_point's line and approach, ego's and Gazebo's models."""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2]
for path in (HERE.parent, HERE, SRC / 'drivers' / 'aav_drive_to_point', SRC / 'ego' / 'aav_ego',
             SRC / 'sim' / 'aav_sim_models', SRC / 'bringup' / 'aav_description'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def pytest_configure(config):
    """
    Register `timing`: a test that times the trackers on the wall clock.

    Beside other packages' tests (colcon test runs several at once) the clock runs slow, so
    `pixi run test` runs these on their own, after the rest (tools/colcon_test.sh).
    """
    config.addinivalue_line('markers', 'timing: times a tracker on the wall clock')
