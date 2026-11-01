"""The path's geometry: the foot, the cross track, the bend, past the ends, into odom."""

import math

from aav_track.path import moved, Path
import pytest
from sim_car import arc, straight


def test_the_foot_and_cross_track_on_a_straight_line():
    p = Path(straight(10.0))
    f = p.project((3.0, 0.5))
    assert f.s == pytest.approx(3.0) and f.cross == pytest.approx(0.5)
    assert p.project((3.0, -0.5)).cross == pytest.approx(-0.5)
    assert p.length == pytest.approx(10.0) and p.left(3.0) == pytest.approx(7.0)


def test_past_either_end_the_line_carries_on():
    p = Path(straight(10.0))
    assert p.project((12.0, 0.2)).s == pytest.approx(12.0)
    assert p.project((-1.0, 0.0)).s == pytest.approx(-1.0)
    assert p.point_at(12.0) == pytest.approx((12.0, 0.0))
    assert p.left(12.0) == pytest.approx(-2.0)


def test_the_bend_of_an_arc_is_one_over_its_radius():
    p = Path(arc(radius=8.0))
    mid = 5.0 + 8.0 * math.pi / 4
    assert p.curvature_at(mid) == pytest.approx(1 / 8.0, rel=0.05)
    assert p.curvature_at(1.0) == pytest.approx(0.0, abs=1e-6)
    assert p.max_curvature(0.0, p.length) == pytest.approx(1 / 8.0, rel=0.05)
    assert Path([(x, -y, -yaw) for x, y, yaw in arc()]).curvature_at(mid) < 0


def test_the_search_goes_forward_from_the_hint_only():
    # Out 10 m and back along a parallel line 1 m left: a point near the start, from a hint
    # on the way back, stays on the way back.
    out = straight(10.0)
    back = [(10.0 - x, 1.0, math.pi) for x, _, _ in straight(10.0)]
    p = Path(out + back)
    early = p.project((1.0, 0.4))
    assert early.s < 2.0
    late = p.project((1.0, 0.6), hint=len(out) + 5)
    assert late.s > 15.0


def test_repeated_points_are_dropped_and_one_point_still_has_a_heading():
    p = Path([(0, 0, 0), (0, 0, 0), (1, 0, 0)])
    assert len(p.points) == 2
    q = Path([(2.0, 3.0, math.pi / 2)])
    assert q.heading_at(0.0) == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        Path([])
    with pytest.raises(ValueError):
        Path([(math.nan, 0.0, 0.0)])


def test_moved_into_odom():
    # odom 2 m east, turned 90 degrees: map's (1, 0) facing east is (2, 1) facing north.
    (x, y, yaw), = moved([(1.0, 0.0, 0.0)], (2.0, 0.0, math.pi / 2))
    assert (x, y, yaw) == pytest.approx((2.0, 1.0, math.pi / 2))
