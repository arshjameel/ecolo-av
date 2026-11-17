"""The tests import this package, the actuator tests' rules, ego's and Gazebo's models."""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2]
for path in (HERE.parent, HERE, SRC / 'drivers' / 'aav_actuator_tests', SRC / 'ego' / 'aav_ego',
             SRC / 'sim' / 'aav_sim_models', SRC / 'bringup' / 'aav_description'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
