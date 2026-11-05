"""The guard: each check G1-G8 refuses what it should, and passes a good path."""

import math

from aav_diffusion_planner import guard, window
import numpy as np
import pytest

FOOT = guard.footprint_points([[1.85, 0.55], [1.85, -0.55], [-0.25, -0.55], [-0.25, 0.55]])
LIMITS = guard.Limits(min_radius=2.73, reversing=True)


def grid(value=0):
    return window.Grid(np.full((240, 320), value, np.int8), -10.0, -10.0, 0.25)


def straight(length=20.0, n=64, yaw=0.0):
    s = np.linspace(0.0, length, n)
    return np.stack([s * math.cos(yaw), s * math.sin(yaw), np.full(n, yaw)], axis=1)


def arc(radius, angle, n=64):
    """Return a left turn from (0, 0) facing east, of this radius, through this angle."""
    a = np.linspace(0.0, angle, n)
    return np.stack([radius * np.sin(a), radius * (1 - np.cos(a)), a], axis=1)


def run(path, g=None, limits=LIMITS, lattice=None, start=None, goal=None):
    start = start if start is not None else tuple(path[0])
    goal = goal if goal is not None else tuple(path[-1])
    return guard.check(path, g or grid(), start, goal, limits, FOOT, lattice)


def test_a_straight_path_passes():
    failed, facts = run(straight())
    assert failed == []
    assert facts['length'] == pytest.approx(20.0)
    assert facts['cusps'] == 0 and facts['reverse_m'] == 0.0


def test_footprint_points_cover_the_outline():
    assert FOOT[:, 0].min() == pytest.approx(-0.25) and FOOT[:, 0].max() <= 1.85
    inside = FOOT[(np.abs(FOOT[:, 0] - 0.8) < 0.06) & (np.abs(FOOT[:, 1]) < 0.06)]
    assert len(inside)                  # the middle too, not only the edge


def test_g1_numbers():
    p = straight()
    p[10, 0] = math.nan
    assert run(p)[0] == ['G1']
    jump = straight()
    jump[40:, 1] += 5.0                 # a 5 m step sideways
    assert 'G1' in run(jump, start=tuple(jump[0]))[0]


def test_g2_g3_the_ends():
    p = straight()
    assert 'G2' in run(p, start=(0.0, 0.5, 0.0))[0]
    assert 'G2' in run(p, start=(0.0, 0.0, math.radians(20)))[0]
    assert 'G3' in run(p, goal=(18.5, 0.0, 0.0))[0]
    assert 'G3' in run(p, goal=(20.0, 0.0, 1.0))[0]
    assert run(p, goal=(19.2, 0.3, 0.5))[0] == []      # inside 1.0 m and 0.75 rad


def test_g4_the_outline_on_a_lethal_cell():
    data = np.zeros((240, 320), np.int8)
    # A post 0.4 m left of the path's line: the car's centre misses it, its side doesn't.
    data[int((0.4 + 10) / 0.25), int((10 + 10) / 0.25)] = 100
    failed, _ = run(straight(), window.Grid(data, -10.0, -10.0, 0.25))
    assert failed == ['G4']
    # Unknown cells too, and off the map.
    assert run(straight(), grid(-1))[0] == ['G4']
    assert 'G4' in run(straight(length=80.0, n=200))[0]


def test_g4_inscribed_cells_are_allowed_under_the_outline():
    # Nav2's inscribed cost (99) round a wall: Smac lets the outline over it, so does G4.
    assert run(straight(), grid(99))[0] == []


def test_g5_turns():
    assert run(arc(4.0, math.pi / 2))[0] == []
    assert run(arc(2.0, math.pi / 2))[0] == ['G5']


def test_g6_sliding_sideways():
    p = straight()
    p[:, 2] = math.radians(40)          # facing 40 deg off its travel
    failed = run(p, start=(0.0, 0.0, math.radians(40)), goal=(20.0, 0.0, math.radians(40)))[0]
    assert failed == ['G6']


def reverse_then_forward(back=6.0):
    """Back 'back' m west facing east, then forward 10 m east."""
    a = np.linspace(0.0, -back, 25)
    b = np.linspace(-back, 10.0 - back, 40)[1:]
    xs = np.concatenate([a, b])
    return np.stack([xs, np.zeros_like(xs), np.zeros_like(xs)], axis=1)


def test_g7_reversing():
    p = reverse_then_forward()
    failed, facts = run(p, limits=LIMITS)
    assert failed == [] and facts['cusps'] == 1
    assert facts['reverse_m'] == pytest.approx(6.0, abs=0.3)
    assert run(p, limits=guard.Limits(min_radius=2.73, reversing=False))[0] == ['G7']
    long_back = reverse_then_forward(back=20.0) + [25.0, 0.0, 0.0]   # inside the map
    assert run(long_back, g=grid())[0] == ['G7']


def test_g7_a_wobble_at_the_end_is_not_a_cusp():
    p = straight()
    p = np.vstack([p, [[19.9, 0.0, 0.0]]])     # 0.1 m back at the very end
    failed, facts = run(p, limits=guard.Limits(min_radius=2.73, reversing=False))
    assert 'G7' not in failed and facts['cusps'] == 0


def test_g8_longer_than_the_lattice():
    assert run(straight(), lattice=10.0)[0] == ['G8']
    assert run(straight(), lattice=15.0)[0] == []


def test_choose_takes_the_cheapest_that_passed():
    bad = straight()
    bad[5, 1] = math.inf
    long_way = np.vstack([straight(10.0, 32), straight(10.0, 32) + [10.0, 0.0, 0.0]])
    short = straight(20.0, 64)
    best, facts, refusals, passed = guard.choose(
        [bad, long_way, short], grid(), (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), LIMITS, FOOT)
    assert best in (1, 2) and passed == 2
    assert refusals['G1'] == 1
    best, _, refusals, passed = guard.choose(
        [bad], grid(), (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), LIMITS, FOOT)
    assert best is None and passed == 0


def test_cost_prices_cusps_and_reversing():
    plain = {'length': 20.0, 'cost_mean': 0.0, 'cusps': 0, 'reverse_m': 0.0}
    assert guard.cost(plain) == pytest.approx(20.0)
    assert guard.cost(dict(plain, cusps=1, reverse_m=3.0)) > guard.cost(plain) + 5.0


def test_decide_falls_back_to_the_lattice():
    assert guard.decide(True, True, 40.0, 150.0, True) == ('model', 'a sample passed every check')
    assert guard.decide(True, True, 40.0, 150.0, False)[0] == 'model'
    assert guard.decide(False, False, 0.0, 150.0, True)[0] == 'lattice'
    src, why = guard.decide(True, False, 40.0, 150.0, True)
    assert src == 'lattice' and 'failed the guard' in why
    src, why = guard.decide(True, True, 200.0, 150.0, True)
    assert src == 'lattice' and why.startswith('late')
    assert guard.decide(True, False, 40.0, 150.0, False)[0] == 'none'


def test_gap():
    assert guard.gap(straight(), straight()) == pytest.approx(0.0, abs=0.2)
    assert guard.gap(straight() + [0.0, 1.0, 0.0], straight()) == pytest.approx(1.0, abs=0.01)
