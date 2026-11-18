"""The road ahead's limit in ego: who is trimmed, by how much, and what silence means."""

from collections import namedtuple

from aav_description.road_ahead import build
from aav_description.stop_law import from_car_model
from aav_ego.road_limit import allowed, cap
import pytest

CURVE = from_car_model()
Stop = namedtuple('Stop', 'reason left')
STRIP = build([(0.0, 0.0), (100.0, 0.0)], (0.0, 0.0), [Stop('sign', 10.0)], CURVE, 1.0)


def test_no_strip_no_pose_or_a_stale_pose_allow_nothing():
    assert allowed(None, (0.0, 0.0), 0.0, 1.0, CURVE, 1.0) == 0.0
    assert allowed(STRIP, None, 0.0, 1.0, CURVE, 1.0) == 0.0
    assert allowed(STRIP, (0.0, 0.0), 1.5, 1.0, CURVE, 1.0) == 0.0
    assert allowed(STRIP, (0.0, 0.0), float('nan'), 1.0, CURVE, 1.0) == 0.0


def test_far_from_the_stop_the_base_speed_close_to_it_less_at_it_none():
    assert allowed(STRIP, (0.0, 0.0), 0.1, 1.0, CURVE, 1.0) == pytest.approx(1.0)
    near = allowed(STRIP, (9.8, 0.0), 0.1, 1.0, CURVE, 1.0)
    assert 0.0 < near < 1.0
    assert allowed(STRIP, (10.0, 0.0), 0.1, 1.0, CURVE, 1.0) == 0.0
    assert allowed(STRIP, (12.0, 0.0), 0.1, 1.0, CURVE, 1.0) == 0.0


def test_cap_keeps_the_curvature():
    v, w = cap((1.0, 0.4), 0.5)
    assert v == pytest.approx(0.5) and w / v == pytest.approx(0.4)


def test_cap_leaves_slower_reverse_and_stopped_commands_alone():
    assert cap((0.3, 0.1), 0.5) == (0.3, 0.1)
    assert cap((-0.5, 0.2), 0.0) == (-0.5, 0.2)
    assert cap((0.0, 0.0), 0.0) == (0.0, 0.0)


def test_cap_to_nothing_stops_the_car():
    assert cap((1.0, 0.4), 0.0) == (0.0, 0.0)
