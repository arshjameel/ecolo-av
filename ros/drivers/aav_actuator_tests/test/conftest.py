"""The tests import this package, ego's model and the car model, from the source tree."""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2]
for path in (HERE.parent, HERE, SRC / 'ego' / 'aav_ego', SRC / 'sim' / 'aav_sim_models',
             SRC / 'bringup' / 'aav_description'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
