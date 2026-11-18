"""The tests import this package and aav_description (the car model), from the source tree."""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2]
for path in (HERE.parent, SRC / 'bringup' / 'aav_description'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
