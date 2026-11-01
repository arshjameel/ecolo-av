"""The tests import this package, drive_to_point's line and approach, ego's and Gazebo's models."""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2]
for path in (HERE.parent, HERE, SRC / 'drivers' / 'aav_drive_to_point', SRC / 'ego' / 'aav_ego',
             SRC / 'sim' / 'aav_gazebo', SRC / 'bringup' / 'aav_description'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
