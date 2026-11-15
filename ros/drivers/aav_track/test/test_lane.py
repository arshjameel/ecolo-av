"""The lane's edges for the trackers (aav_track/lane.py, TODO 169)."""

import math

from aav_track.lane import keep_in_lane, LinePoint
import pytest

# A lane along x: 2 m of room each side of the line.
LINE = [LinePoint(float(x), 0.0, 0.0, 2.0, 2.0) for x in range(0, 21)]
HALF = 0.5


def test_a_path_inside_the_lane_is_left_alone():
    path = [(float(x), 0.4, 0.0) for x in range(10)]
    assert keep_in_lane(path, LINE, HALF) == path


def test_a_path_over_the_left_edge_comes_in_by_just_enough():
    path = [(5.0, 1.9, 0.1)]
    (x, y, yaw), = keep_in_lane(path, LINE, HALF, margin=0.1)
    assert (x, yaw) == (5.0, 0.1) and y == pytest.approx(2.0 - 0.5 - 0.1)


def test_a_path_over_the_right_edge_comes_in_too():
    (_, y, _), = keep_in_lane([(5.0, -3.0, 0.0)], LINE, HALF, margin=0.1)
    assert y == pytest.approx(-1.4)


def test_unknown_edges_or_far_points_stay_as_planned():
    unknown = [LinePoint(float(x), 0.0, 0.0, math.nan, math.nan) for x in range(10)]
    path = [(5.0, 3.0, 0.0)]
    assert keep_in_lane(path, unknown, HALF) == path
    far = [(5.0, 9.0, 0.0)]
    assert keep_in_lane(far, LINE, HALF) == far


def test_a_lane_narrower_than_the_car_puts_it_on_the_line():
    narrow = [LinePoint(float(x), 0.0, 0.0, 0.4, 0.4) for x in range(10)]
    (_, y, _), = keep_in_lane([(5.0, 0.3, 0.0)], narrow, HALF)
    assert y == pytest.approx(0.0)


def test_a_line_heading_north_moves_points_east_and_west():
    north = [LinePoint(0.0, float(y), math.pi / 2, 1.5, 1.5) for y in range(10)]
    (x, y, _), = keep_in_lane([(-2.0, 5.0, 0.0)], north, HALF, margin=0.0)
    # The line's left is west (-x): 1.5 - 0.5 = 1 m of room that way.
    assert x == pytest.approx(-1.0) and y == pytest.approx(5.0)


def test_no_line_no_change():
    path = [(1.0, 5.0, 0.0)]
    assert keep_in_lane(path, [], HALF) == path
