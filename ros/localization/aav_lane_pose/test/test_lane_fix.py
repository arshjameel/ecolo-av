"""The lane finder as a pose sensor: the lines into an offset, the gates, the pose."""

import math

from aav_description.lane_map import LaneMap, Road
from aav_lane_pose.lane_fix import (across_covariance, LanePose, Line, lines_from, Seen,
                                    seen_from, Settings)
import pytest

# The road world: one lane, 3.5 m between the lines, east along y = 0, x -20 to 100 m.
ROAD = Road('road', ((-20.0, 0.0), (100.0, 0.0)), 4.5, lanes=1)
# A quarter circle, radius 40 m, round (0, 0) from east to north: a two-way street, 7 m.
ARC = Road('arc', tuple((40.0 * math.cos(a), 40.0 * math.sin(a))
                        for a in (math.radians(d) for d in range(0, 91, 3))), 7.0)


def lines_seen(d, psi, width=3.5):
    """Return the left and right lines a car `d` left of the centre, turned `psi`, sees."""
    b = -math.tan(psi)
    return (Line(0.0, b, (width / 2.0 - d) / math.cos(psi)),
            Line(0.0, b, (-width / 2.0 - d) / math.cos(psi)))


@pytest.mark.parametrize('d', [-0.6, 0.0, 0.25])
@pytest.mark.parametrize('psi', [-0.2, 0.0, 0.1])
def test_both_lines_give_the_offset_heading_and_width(d, psi):
    seen = seen_from(*lines_seen(d, psi), 3.5, 0.9, 0.8)
    assert (seen.offset, seen.heading, seen.width) == pytest.approx((d, psi, 3.5))
    assert (seen.lines, seen.confidence) == (2, 0.9)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_one_line_and_the_width_give_the_offset(side):
    left, right = lines_seen(0.3, 0.05)
    seen = seen_from(left if side == 'left' else None, right if side == 'right' else None,
                     3.5, 0.7, 0.7)
    assert (seen.offset, seen.heading, seen.lines) == pytest.approx((0.3, 0.05, 1))


def test_no_line_no_offset():
    assert seen_from(None, None, 3.5, 0.0, 0.0) is None


def test_lines_from_reads_lane_geometry_fields():
    fields = {'left_poly_a': 0.0, 'left_poly_b': 0.1, 'left_poly_c': 1.7,
              'left_lane_detected': True, 'right_poly_a': 0.0, 'right_poly_b': 0.0,
              'right_poly_c': 0.0, 'right_lane_detected': False}
    assert lines_from(fields) == (Line(0.0, 0.1, 1.7), None)


def test_the_covariance_is_sure_across_the_lane_only():
    heading = 0.7
    cov = across_covariance(heading, 0.1, 50.0, 0.05)
    sxx, sxy, syy = cov[0], cov[1], cov[7]
    along = (math.cos(heading), math.sin(heading))
    left = (-math.sin(heading), math.cos(heading))

    def var(u):
        return sxx * u[0] ** 2 + 2 * sxy * u[0] * u[1] + syy * u[1] ** 2

    assert var(left) == pytest.approx(0.01)
    assert var(along) == pytest.approx(2500.0)
    assert cov[1] == cov[6] and cov[35] == pytest.approx(0.0025)


def seen(d, psi=0.0, lines=2, confidence=0.9, width=3.5, curvature=0.0):
    return Seen(d, psi, width, lines, confidence, curvature)


def test_straight_lane_moves_the_pose_across_only():
    lanes = LanePose(LaneMap([ROAD]))
    # The filter has the car at y = 0.5; the camera says 0.2 left of the centre.
    fix, why = lanes.update(0.0, (30.0, 0.5, 0.02), seen(0.2, 0.02))
    assert why == 'ok'
    assert (fix.x, fix.y) == pytest.approx((30.0, 0.2))
    assert fix.yaw == pytest.approx(0.02)
    assert fix.road == 'road'


def test_driving_west_flips_the_offset():
    lanes = LanePose(LaneMap([ROAD]))
    fix, why = lanes.update(0.0, (30.0, -0.1, math.pi), seen(0.2))
    assert why == 'ok'
    # Left of the centre, driving west, is south: y = -0.2.
    assert (fix.x, fix.y) == pytest.approx((30.0, -0.2))


def test_on_a_curve_the_fix_is_square_to_the_lane():
    lanes = LanePose(LaneMap([ARC]))
    a = math.radians(45.0)
    # Going north-west round the arc (counter-clockwise): the right lane is the outer one,
    # 1.5 m out from the centre line (radius 41.5); the car 0.3 m left of it, inward.
    fix, why = lanes.update(0.0, (41.0 * math.cos(a), 41.0 * math.sin(a), a + math.pi / 2),
                            seen(0.3, width=3.0, curvature=1.0 / 41.5))
    assert why == 'ok'
    assert math.hypot(fix.x, fix.y) == pytest.approx(41.2, abs=0.02)
    assert math.atan2(fix.y, fix.x) == pytest.approx(a, abs=1e-3)


@pytest.mark.parametrize('case, why', [
    ({'confidence': 0.2}, 'confidence'),
    ({'psi': 0.5}, 'turned'),
    ({'width': 2.5}, 'width'),
    ({'d': -1.9}, 'innovation'),
    ({'psi': 0.1}, 'heading'),          # the filter has the car square to the lane
    ({'curvature': 0.01}, 'bend'),      # the map's lane is straight
])
def test_gates(case, why):
    lanes = LanePose(LaneMap([ROAD]))
    args = {'d': 0.1}
    args.update(case)
    assert lanes.update(0.0, (30.0, 0.0, 0.0), seen(**args)) == (None, why)


def test_one_line_skips_the_width_gate_and_is_less_sure():
    lanes = LanePose(LaneMap([ROAD]))
    fix, why = lanes.update(0.0, (30.0, 0.0, 0.0), seen(0.1, lines=1, width=2.5))
    assert why == 'ok'
    assert fix.covariance[7] == pytest.approx(0.3 ** 2)


def test_off_every_road_or_near_its_end_is_refused():
    lanes = LanePose(LaneMap([ROAD]))
    assert lanes.update(0.0, (30.0, 8.0, 0.0), seen(0.1)) == (None, 'off road')
    assert lanes.update(0.0, (95.0, 0.0, 0.0), seen(0.1)) == (None, 'junction')   # ends ahead
    assert lanes.update(0.0, (-18.0, 0.0, 0.0), seen(0.1)) == (None, 'junction')  # behind


def test_a_crossing_road_is_a_junction():
    crossing = Road('crossing', ((40.0, -30.0), (40.0, 30.0)), 7.0)
    lanes = LanePose(LaneMap([ROAD, crossing]))
    assert lanes.update(0.0, (32.0, 0.0, 0.0), seen(0.1)) == (None, 'junction')
    assert lanes.update(0.0, (20.0, 0.0, 0.0), seen(0.1))[1] == 'ok'
    assert lanes.update(0.1, (46.0, 0.0, 0.0), seen(0.1)) == (None, 'junction')


def test_a_jump_is_refused_until_the_window_passes():
    lanes = LanePose(LaneMap([ROAD]), Settings(max_jump=0.4, jump_window=1.0))
    assert lanes.update(0.0, (30.0, 0.0, 0.0), seen(0.1))[1] == 'ok'
    assert lanes.update(0.2, (30.5, 0.0, 0.0), seen(0.7)) == (None, 'jump')
    assert lanes.update(0.4, (31.0, 0.0, 0.0), seen(0.2))[1] == 'ok'
    assert lanes.update(1.5, (33.0, 0.0, 0.0), seen(0.7))[1] == 'ok'     # a second later
    assert 'jump 1' in lanes.summary() and lanes.summary() == 'nothing'


def test_the_lines_bend_is_read_from_the_fit():
    left = Line(0.004, 0.0, 1.75)
    right = Line(0.006, 0.0, -1.75)
    assert seen_from(left, right, 3.5, 0.9, 0.9).curvature == pytest.approx(0.01)
    assert seen_from(None, right, 3.5, 0.9, 0.9).curvature == pytest.approx(0.012)


def test_on_a_curve_lines_seen_straight_are_refused():
    lanes = LanePose(LaneMap([ARC]))
    a = math.radians(45.0)
    pose = (41.0 * math.cos(a), 41.0 * math.sin(a), a + math.pi / 2)
    assert lanes.update(0.0, pose, seen(0.3, width=3.0)) == (None, 'bend')
