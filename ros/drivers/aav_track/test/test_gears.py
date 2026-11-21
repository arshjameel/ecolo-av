"""
Reversing in aav_track (TODO 174): the path cut at its cusps, and every tracker backing up.

gears.py cuts the path; each backwards stretch goes to the tracker mirrored. On the pretend
car (sim_car.follow_geared, as track_node.py drives it): a straight back, and forward to a
cusp then backwards round a bend.
"""

import math

from aav_track.gears import directions, Gears, mirror_state, stretches
import pytest
from sim_car import back_straight, follow_geared, forward_then_back, tracker

TRACKERS = ['pure_pursuit', 'stanley', 'lqr', 'mpc', 'mpc_full', 'mpc_lag']


def test_a_segment_against_its_pose_is_backwards():
    assert directions([(0, 0, 0), (1, 0, 0), (2, 0, 0)]) == [1, 1]
    assert directions([(0, 0, 0), (-1, 0, 0), (-2, 0, 0)]) == [-1, -1]
    # A repeated point keeps the gear before it.
    assert directions([(0, 0, 0), (-1, 0, 0), (-1, 0, 0)]) == [-1, -1]


def test_the_path_is_cut_at_each_cusp():
    parts = stretches(forward_then_back())
    assert [p.backwards for p in parts] == [False, True]
    # The cusp's pose ends one stretch and starts the next.
    assert parts[0].poses[-1] == parts[1].poses[0]
    assert len(stretches([(0, 0, 0)])) == 1
    three = [(0, 0, 0), (1, 0, 0), (0.5, 0, 0), (0, 0, 0), (1, 0, 0)]
    assert [p.backwards for p in stretches(three)] == [False, True, False]


def test_a_backwards_stretch_is_mirrored_for_the_tracker():
    part = stretches(back_straight(2.0))[0]
    assert part.backwards
    path = part.path()
    # Mirrored: the goal faces the way the path runs (-x), as a forward car would.
    assert abs(abs(path.goal_yaw) - math.pi) < 1e-9
    assert mirror_state(1, 2, 0.0, -0.4, 0.1, 0.2, True) == (1, 2, -math.pi, 0.4, 0.1, -0.2)
    assert mirror_state(1, 2, 0.3, 0.4, 0.1, 0.2, False) == (1, 2, 0.3, 0.4, 0.1, 0.2)


def test_the_next_stretch_waits_for_the_cusp():
    gears = Gears.of(forward_then_back())
    assert not gears.update(left=2.0, v=1.0, now=0.0)
    assert not gears.update(left=0.2, v=0.5, now=0.1)       # too fast to change gear
    assert gears.update(left=0.2, v=0.1, now=0.2)
    assert gears.backwards and gears.last
    assert not gears.update(left=0.0, v=0.0, now=0.3)        # the last: the goal decides


def test_held_short_of_a_cusp_it_changes_gear_after_standing():
    # A lock holds the car 1 m short (the stop zone before a wall in a narrow street).
    gears = Gears.of(forward_then_back())
    assert not gears.update(left=1.0, v=0.0, now=0.0)
    assert not gears.update(left=1.0, v=0.0, now=1.9)
    assert gears.update(left=1.0, v=0.0, now=2.0)
    # Too far short, it never does: the follower's progress check gives up instead.
    far = Gears.of(forward_then_back())
    assert not any(far.update(left=3.0, v=0.0, now=t) for t in range(20))


@pytest.mark.parametrize('name', TRACKERS)
def test_every_tracker_backs_up_a_straight_line(name):
    run = follow_geared(tracker(name), back_straight(8.0), (0.0, 0.0, 0.0), max_time=60.0)
    assert run.reached, name
    assert min(r[9] for r in run.rows) >= -0.5 - 1e-9     # asked within max_reverse_speed
    assert min(r[4] for r in run.rows) >= -0.65           # the speed loop overshoots a little
    assert max(r[9] for r in run.rows) <= 0.0            # never asked forward
    assert max(abs(r[6]) for r in run.rows) < 0.2, name  # on the line


@pytest.mark.parametrize('name', TRACKERS)
def test_every_tracker_goes_through_a_cusp_and_backs_round_a_bend(name):
    run = follow_geared(tracker(name), forward_then_back(), (0.0, 0.0, 0.0), max_time=90.0)
    assert run.reached, name
    forward = [r for r in run.rows if r[8] == 0]
    back = [r for r in run.rows if r[8] == 1]
    assert forward and back
    assert max(r[9] for r in back) <= 0.0 < max(r[9] for r in forward)
    # Close to the bend backwards (the trackers' own error, mirrored).
    assert max(abs(r[6]) for r in back[len(back) // 4:]) < 0.5, name
