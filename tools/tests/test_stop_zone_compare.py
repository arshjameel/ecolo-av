"""Our stop zone's shadow against collision_monitor (tools/stop_zone_compare.py)."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stop_zone_compare import at, compare, lags, onsets, stops, travelled  # noqa: E402


def test_the_same_stops_agree_fully():
    nav2 = [(0.0, False), (5.0, True), (9.0, False)]
    ours = [(t / 10.0, 5.0 <= t / 10.0 < 9.0) for t in range(0, 120)]
    c = compare(nav2, ours, 0.0, 12.0)
    assert c.agree == pytest.approx(1.0)
    assert c.delay == pytest.approx(0.0)
    assert (c.missed, c.extra) == (0.0, 0.0)
    assert (c.nav2_stops, c.our_stops) == (1, 1)


def test_a_late_stop_is_missed_time_and_a_delay():
    nav2 = [(0.0, False), (5.0, True)]
    ours = [(0.0, False), (5.3, True)]
    c = compare(nav2, ours, 0.0, 10.0)
    assert c.delay == pytest.approx(0.3)
    assert c.missed == pytest.approx(0.3)
    assert c.extra == 0.0
    assert c.agree == pytest.approx(0.97)


def test_ours_stopping_alone_is_extra_not_missed():
    c = compare([(0.0, False)], [(0.0, False), (2.0, True), (3.0, False)], 0.0, 10.0)
    assert (c.missed, c.extra) == (0.0, pytest.approx(1.0))
    assert c.delay is None


def test_the_state_before_any_message_is_clear():
    assert at([(2.0, True)], 1.0) is False
    assert at([(2.0, True)], 2.0) is True
    assert stops([(0, True), (1, True), (2, False), (3, True)]) == 2


def test_each_of_nav2s_stops_gets_ours_lag():
    nav2 = [(0.0, False), (5.0, True), (6.0, False), (8.0, True), (9.0, False)]
    ours = [(0.0, False), (4.9, True), (6.1, False), (8.05, True), (9.0, False)]
    assert onsets(nav2) == [5.0, 8.0]
    (t1, a), (t2, b) = lags(nav2, ours)
    assert (t1, t2) == (5.0, 8.0)
    assert a == pytest.approx(-0.1)     # ours first
    assert b == pytest.approx(0.05)     # ours a tick later


def test_a_stop_ours_never_made_has_no_lag():
    assert lags([(1.0, True)], [(0.0, False)]) == [(1.0, None)]


def test_travelled_is_signed_by_order():
    track = [(t / 10.0, t / 10.0, 0.0) for t in range(0, 21)]   # 1 m/s along x
    assert travelled(track, 0.5, 1.0) == pytest.approx(0.5)
    assert travelled(track, 1.0, 0.5) == pytest.approx(-0.5)
