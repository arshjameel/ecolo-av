"""The one speed policy: cruise, slow for a bend ahead, stop at the path's end."""

from aav_track.path import Path
from aav_track.speed import SpeedPolicy
import pytest
from sim_car import arc, straight


def run(policy, path, s=0.0, measured=0.0, ticks=200, dt=0.05, cap=None):
    v = 0.0
    for _ in range(ticks):
        v = policy.step(path, s, v, measured, dt, cap)
        measured = v
    return v


def test_speeds_up_to_cruise_at_the_smoothers_rate():
    p = SpeedPolicy(cruise=1.0, accel=1.0, max_brake=3.0)
    path = Path(straight(50.0))
    assert p.step(path, 0.0, 0.0, 0.0, 0.05) == pytest.approx(0.05)
    assert run(p, path) == pytest.approx(1.0)


def test_slows_for_a_bend_before_it():
    p = SpeedPolicy(cruise=3.0, accel=1.0, max_brake=3.0, lateral_accel=1.0)
    path = Path(arc(radius=4.0, lead=6.0))
    # 2 m before the bend at 3 m/s: sqrt(a r) = 2 m/s at most.
    assert run(p, path, s=4.0, measured=3.0, ticks=1) <= 2.0 + 1e-6


def test_stops_at_the_end_and_obeys_a_trackers_cap():
    p = SpeedPolicy(cruise=1.0, accel=1.0, max_brake=3.0)
    path = Path(straight(10.0))
    assert p.step(path, 10.0, 1.0, 1.0, 0.05) == 0.0
    assert p.step(path, 11.0, 1.0, 1.0, 0.05) == 0.0
    assert run(p, path, cap=0.4) == pytest.approx(0.4)
    assert p.step(path, 9.95, 0.5, 0.5, 0.05) < 0.5       # sqrt(2 x 1.5 x 0.05)
