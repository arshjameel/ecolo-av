"""The road ahead: the strip's line, its stops, its end, and what a reader is allowed."""

from collections import namedtuple
import math

from aav_description.road_ahead import (along_of, build, capped, FINAL, limit_at, resample,
                                        room_at, WAIT)
from aav_description.stop_law import from_car_model
import pytest

CURVE = from_car_model()
BASE = 1.0
STRAIGHT = [(0.0, 0.0), (100.0, 0.0)]      # 100 m along x
# The writer's stops: aav_behaviour_rs's Stop has the same two fields.
Stop = namedtuple('Stop', 'reason left')


def test_resample_starts_at_the_foot_and_stops_at_length():
    line = resample(STRAIGHT, 10.0, 0.3, spacing=0.5, length=40.0)
    assert line[0][:2] == pytest.approx((10.0, 0.0))
    assert line[-1][:2] == pytest.approx((50.0, 0.0))
    assert len(line) == 81
    assert all(h == pytest.approx(0.0) for _, _, h in line)


def test_resample_keeps_the_paths_end_between_points():
    line = resample([(0.0, 0.0), (3.2, 0.0)], 0.0, 0.0, spacing=0.5, length=40.0)
    assert line[-1][:2] == pytest.approx((3.2, 0.0))
    assert line[-2][:2] == pytest.approx((3.0, 0.0))


def test_resample_follows_a_corner():
    line = resample([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], 0.0, 0.0, spacing=1.0)
    assert [p[:2] for p in line] == [pytest.approx(p) for p in
                                     [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]]
    assert line[-1][2] == pytest.approx(math.pi / 2)


def test_no_plan_or_no_pose_is_an_empty_strip_that_allows_nothing():
    for plan, car in (([], (0.0, 0.0)), (STRAIGHT, None), ([(0.0, 0.0)], (0.0, 0.0))):
        strip = build(plan, car, [], CURVE, BASE)
        assert len(strip) == 0 and strip.end == 0.0
        assert limit_at(strip, 0.0, 0.0) == 0.0
    assert limit_at(None, 0.0, 0.0) == 0.0


def test_the_strip_always_ends_in_a_stop():
    strip = build(STRAIGHT, (0.0, 0.0), [], CURVE, BASE)
    assert strip.end == pytest.approx(40.0)
    assert strip.speed_limit[-1] == 0.0
    assert strip.speed_limit[0] == BASE          # 40 m of room: no need to slow yet
    assert all(a >= b for a, b in zip(strip.speed_limit, strip.speed_limit[1:]))


def test_the_nearest_stop_is_the_end_and_kinds_follow_the_reason():
    stops = [Stop('goal', 30.0), Stop('sign', 12.0)]
    strip = build(STRAIGHT, (0.0, 0.0), stops, CURVE, BASE, state='APPROACH', reason='sign')
    assert strip.end == pytest.approx(12.0)
    assert [(s.reason, s.kind) for s in strip.stops] == [('sign', WAIT), ('goal', FINAL)]
    assert limit_at(strip, 12.0, 0.0) == 0.0
    assert limit_at(strip, 11.0, 0.0) == pytest.approx(CURVE.cap(1.0), abs=1e-6)
    assert strip.state == 'APPROACH' and strip.reason == 'sign'


def test_a_stop_behind_the_car_means_stop_now():
    strip = build(STRAIGHT, (0.0, 0.0), [Stop('obstacle', -0.5)], CURVE, BASE)
    assert strip.end == 0.0
    assert limit_at(strip, 0.0, 0.0) == 0.0


def test_a_stop_beyond_the_line_is_left_out():
    strip = build(STRAIGHT, (0.0, 0.0), [Stop('goal', 70.0)], CURVE, BASE)
    assert strip.stops == () and strip.end == pytest.approx(40.0)


def test_the_limit_never_exceeds_base_or_the_stopping_law():
    strip = build(STRAIGHT, (0.0, 0.0), [Stop('sign', 5.0)], CURVE, BASE)
    for i, v in enumerate(strip.speed_limit):
        room = strip.end - i * strip.spacing
        assert v <= BASE
        assert v <= (CURVE.cap(room) if room > 0 else 0.0) + 1e-9


def test_edges_are_not_known_yet():
    strip = build(STRAIGHT, (0.0, 0.0), [], CURVE, BASE)
    assert all(math.isnan(v) for v in strip.left_hard + strip.right_soft)
    assert len(strip.left_hard) == len(strip)


def test_reader_measures_along_and_off_the_line():
    strip = build(STRAIGHT, (0.0, 0.0), [], CURVE, BASE)
    s, off = along_of(strip, 7.3, -0.4)
    assert s == pytest.approx(7.3) and off == pytest.approx(0.4)
    assert room_at(strip, 7.3, -0.4) == pytest.approx(40.0 - 7.3)


def test_silence_stops_the_car_at_the_end_of_the_last_strip():
    """The writer dies: a reader with its own law slows to 0 at the old strip's end."""
    strip = build(STRAIGHT, (0.0, 0.0), [], CURVE, BASE)
    speeds = [limit_at(strip, x, 0.0, CURVE, BASE) for x in (0.0, 30.0, 38.0, 39.9, 40.0, 45.0)]
    assert speeds[0] == BASE
    assert speeds[2] == pytest.approx(min(BASE, CURVE.cap(2.0)))
    assert speeds[3] == pytest.approx(CURVE.cap(0.1)) and speeds[3] < BASE
    assert all(a >= b for a, b in zip(speeds, speeds[1:]))
    assert speeds[4] == 0.0 and speeds[5] == 0.0


def test_the_reader_never_allows_more_than_the_writer():
    strip = build(STRAIGHT, (0.0, 0.0), [Stop('sign', 6.0)], CURVE, BASE)
    for x in (0.0, 2.0, 4.0, 5.9):
        assert limit_at(strip, x, 0.0, CURVE, BASE) <= limit_at(strip, x, 0.0) + 1e-9


def test_bad_inputs_are_refused():
    with pytest.raises(ValueError):
        build(STRAIGHT, (0.0, 0.0), [], CURVE, 0.0)
    with pytest.raises(ValueError):
        resample(STRAIGHT, 0.0, 0.0, spacing=0.0)


def test_the_behaviour_cap_is_in_the_limits():
    # TODO 233: no limit above the cap; 0.0 is no cap.
    strip = build(STRAIGHT, (0.0, 0.0), [], CURVE, BASE)
    assert capped(strip, 0.0) is strip
    held = capped(strip, 0.3)
    assert max(held.speed_limit) == pytest.approx(0.3)
    assert all(h == min(v, 0.3) for h, v in zip(held.speed_limit, strip.speed_limit))
    assert held.speed_limit[-1] == 0.0 and held.end == strip.end
