"""The squiggly line's maths: the plan cropped ahead, the gap, the goal, the obstacle."""

from collections import namedtuple
import math

from aav_ego import look_ahead as la
import pytest

STRAIGHT = [(float(x), 0.0) for x in range(-5, 21)]


def test_crop_starts_beside_the_car_and_runs_ahead():
    plan = la.crop_ahead(STRAIGHT, 6.0)
    assert plan[0] == pytest.approx((0.0, 0.0))
    assert plan[-1] == pytest.approx((6.0, 0.0))
    assert la.length(plan) == pytest.approx(6.0)


def test_crop_of_a_short_plan_ends_at_its_end():
    plan = la.crop_ahead([(-1.0, 0.5), (2.0, 0.5)], 10.0)
    assert plan[0] == pytest.approx((0.0, 0.5))
    assert plan[-1] == pytest.approx((2.0, 0.5))


def test_gap_is_how_far_the_trace_strays():
    trace = [(x * 0.2, 0.01 * x * x) for x in range(11)]   # bends left, 1.0 m at the end
    assert la.gap(trace, STRAIGHT) == pytest.approx(1.0)
    assert la.gap([(1.0, 0.0), (2.0, 0.0)], STRAIGHT) == 0.0
    assert la.gap(trace, []) is None


def test_grades():
    limits = la.Limits(0.3, 0.6)
    assert la.grade(0.1, limits) == la.OK
    assert la.grade(0.4, limits) == la.WARN
    assert la.grade(0.9, limits) == la.BAD
    assert la.grade(None, limits) == la.OK   # no plan: a person drives


def test_to_goal_short_and_past():
    plan = [(0.0, 0.0), (5.0, 0.0)]
    assert la.to_goal(plan, (3.0, 0.0)) == pytest.approx(2.0)
    assert la.to_goal(plan, (6.5, 0.0)) == pytest.approx(-1.5)
    assert la.to_goal(plan, (5.0, 0.0)) == pytest.approx(0.0)
    assert la.to_goal([], (1.0, 0.0)) is None


def test_to_goal_follows_a_bend():
    plan = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)]
    assert la.to_goal(plan, (4.0, 1.0)) == pytest.approx(3.0)


def grid_with_wall(at_x, resolution=0.1, origin=(-5.0, -5.0, 0.0)):
    n = 100
    data = [0] * (n * n)
    for j in range(n):
        for i in range(n):
            x = origin[0] + (i + 0.5) * resolution
            if at_x <= x < at_x + 0.3:
                data[j * n + i] = 100
    return la.Grid(n, n, resolution, origin, data)


def test_grid_cost_and_outside():
    g = grid_with_wall(3.0)
    assert g.cost((3.1, 0.0)) == 100
    assert g.cost((1.0, 0.0)) == 0
    assert g.cost((50.0, 0.0)) == -1


def test_grid_rotated_origin():
    # The grid turned a quarter left: its +x is base_link's +y.
    g = la.Grid(10, 10, 1.0, (0.0, 0.0, math.pi / 2), [0] * 99 + [100])
    assert g.cost((-9.5, 9.5)) == 100


def test_first_hit_counts_from_base_link_to_the_bumper_touching():
    g = grid_with_wall(3.0)
    path = la.extend([(0.0, 0.0, 0.0)], 4.0)
    hit = la.first_hit(g, path, front=1.5, half_width=0.5)
    assert hit == pytest.approx(1.5, abs=0.11)   # the bumper at 1.5 m reaches 3.0 m
    assert la.first_hit(grid_with_wall(30.0), path, 1.5, 0.5) is None


def test_first_hit_sees_with_the_bumpers_corners():
    n = 100
    data = [0] * (n * n)
    # One lethal cell 0.4-0.5 m left, 2.0 m ahead: a bumper 0.47 m each side meets it.
    i, j = int((2.0 + 5.0) / 0.1), int((0.45 + 5.0) / 0.1)
    data[j * n + i] = 100
    g = la.Grid(n, n, 0.1, (-5.0, -5.0, 0.0), data)
    assert la.first_hit(g, la.extend([(0.0, 0.0, 0.0)], 3.0), 1.0, 0.47) is not None
    assert la.first_hit(g, la.extend([(0.0, 0.0, 0.0)], 3.0), 1.0, 0.3) is None


def test_extend_carries_the_last_heading_on():
    path = la.extend([(0.0, 0.0, 0.0), (1.0, 0.0, math.pi / 2)], 3.0)
    assert path[-1][:2] == pytest.approx((1.0, 2.0))


def test_describe():
    line = la.describe(0.12, la.OK, 1.4, 0.8, 3.0)
    assert line.startswith('ON PLAN 0.12 m')
    assert 'stops in 1.4 m' in line and '0.8 m short of the goal' in line
    assert '3.0 m before an obstacle' in line
    assert 'HITS' in la.describe(None, la.OK, 1.4, -0.5, -0.2)
    assert la.describe(None, la.OK, None, None, None).startswith('NO PLAN - stop point unknown')


def test_road_says_the_limit_the_room_and_why():
    from aav_description import road_ahead as ra
    from aav_description.stop_law import from_car_model
    stop = namedtuple('Stop', 'reason left')
    strip = ra.build([(0.0, 0.0), (30.0, 0.0)], (0.0, 0.0), [stop('sign', 20.0)],
                     from_car_model(), 1.0, state='APPROACH', reason='sign')
    at = la.road(strip, 5.0, 0.1)
    assert at.room == pytest.approx(strip.end - 5.0, abs=0.01)
    assert at.why == 'sign'
    assert at.limit == pytest.approx(ra.limit_at(strip, 5.0, 0.1))
    assert la.road(None, 0.0, 0.0) is None
