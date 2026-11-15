"""The goal leash (leash.py, TODO 116): an owner that goes quiet loses its goals."""

from aav_safety.leash import GOODBYE, Leash


def test_never_trips_before_a_beat():
    leash = Leash(timeout=1.0)
    assert not leash.armed
    assert not any(leash.check(t) for t in (0.0, 5.0, 100.0))


def test_trips_once_when_the_beats_stop():
    leash = Leash(timeout=1.0)
    for t in (0.0, 0.2, 0.4):
        leash.beat(int(t * 10) + 1, t)
        assert not leash.check(t)
    assert not leash.check(1.4)            # 1.0 s quiet: not yet
    assert leash.check(1.41)               # past it: trips
    assert not leash.check(1.5)            # once
    assert not leash.armed


def test_goodbye_disarms_without_a_trip():
    leash = Leash(timeout=1.0)
    leash.beat(7, 0.0)
    leash.beat(GOODBYE, 0.1)
    assert not leash.check(10.0)


def test_rearms_on_the_next_beat():
    leash = Leash(timeout=1.0)
    leash.beat(1, 0.0)
    assert leash.check(2.0)
    leash.beat(2, 3.0)
    assert not leash.check(3.5)
    assert leash.check(4.5)
