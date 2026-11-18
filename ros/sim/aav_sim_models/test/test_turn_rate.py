import math

import pytest

# Not pytest.importorskip: ROS's launch_testing plugin turns a module-level skip into a
# skip of the whole folder (CLAUDE.md). The module imports ROS messages, so without ROS
# these tests skip one by one; colcon test runs them.
try:
    from aav_sim_models.sim_adapter import turn_rate
except ImportError:
    turn_rate = None

pytestmark = pytest.mark.skipif(turn_rate is None, reason='needs ROS: run it with colcon test')


def test_straight_and_stopped():
    assert turn_rate(1.0, 0.0, 1.57) == 0.0
    assert turn_rate(0.0, 0.4, 1.57) == 0.0


def test_matches_bicycle_model_both_ways():
    w = turn_rate(2.0, 0.3, 1.57)
    assert math.isclose(w, 2.0 * math.tan(0.3) / 1.57)
    assert math.isclose(turn_rate(-2.0, 0.3, 1.57), -w)   # reversing turns the other way
