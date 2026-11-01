"""StopHold: a stop holds until the way has stayed clear for clear_time."""

from aav_safety.hold import StopHold
import pytest


def test_free_until_the_first_stop():
    hold = StopHold(1.0)
    hold.update(0.0, False)
    assert not hold.locked(0.0)
    assert not hold.locked(100.0)


def test_a_stop_holds_for_clear_time_after_it_ends():
    hold = StopHold(1.0)
    hold.update(0.0, True)
    assert hold.locked(0.0)
    hold.update(2.0, False)
    assert hold.locked(2.5)
    assert hold.locked(2.99)
    assert not hold.locked(3.0)
    assert not hold.locked(10.0)


def test_a_flicker_never_lets_go():
    # Stop and go every 0.2 s, as an obstacle at the zone's edge gives: always locked.
    hold = StopHold(1.0)
    for i in range(50):
        t = i * 0.1
        hold.update(t, i % 4 < 2)
        assert hold.locked(t)


def test_each_new_stop_restarts_the_wait():
    hold = StopHold(1.0)
    hold.update(0.0, True)
    hold.update(0.5, False)
    hold.update(1.2, True)      # stopped again before 1 s of clear
    hold.update(1.3, False)
    assert hold.locked(2.2)
    assert not hold.locked(2.31)


def test_repeated_go_messages_keep_the_first_clear_time():
    hold = StopHold(1.0)
    hold.update(0.0, True)
    hold.update(1.0, False)
    hold.update(1.5, False)     # the same state again must not restart the wait
    assert not hold.locked(2.0)


def test_zero_clear_time_lets_go_at_once_and_bad_times_are_refused():
    hold = StopHold(0.0)
    hold.update(0.0, True)
    assert hold.locked(0.0)
    hold.update(0.1, False)
    assert not hold.locked(0.1)
    for bad in (-1.0, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            StopHold(bad)
