"""Nav2's goal and progress checks, as controller_server's plugins do them."""

import math

from aav_track.checks import GoalCheck, ProgressCheck


def test_the_goal_needs_the_place_then_the_heading():
    g = GoalCheck(xy=1.0, yaw=0.75)
    goal = (10.0, 0.0, 0.0)
    assert not g.done(8.5, 0.0, 0.0, goal)
    assert not g.done(9.5, 0.0, 1.0, goal)      # there, facing wrong
    # Stateful: once there, the place stays reached while the heading comes round.
    assert g.done(11.2, 0.0, 0.1, goal)
    g.reset()
    assert not g.done(11.2, 0.0, 0.1, goal)


def test_not_stateful_needs_both_at_once():
    g = GoalCheck(xy=1.0, yaw=0.75, stateful=False)
    goal = (0.0, 0.0, 0.0)
    assert not g.done(0.0, 0.0, 1.0, goal)
    assert not g.done(2.0, 0.0, 0.0, goal)


def test_progress_is_moving_or_turning_within_the_time():
    p = ProgressCheck(radius=0.5, angle=0.5, time=10.0)
    assert not p.stuck(0.0, 0.0, 0.0, 0.0)
    assert not p.stuck(0.3, 0.0, 0.0, 9.0)
    assert p.stuck(0.3, 0.0, 0.0, 10.5)
    assert not p.stuck(0.3, 0.0, 0.6, 11.0)     # turned: progress, the clock starts again
    assert not p.stuck(0.3, 0.0, 0.6, 20.0)
    assert p.stuck(0.3, 0.0, 0.6, 21.5)
    p.reset()
    assert not p.stuck(5.0, 5.0, 0.0, 100.0)
    assert ProgressCheck(0.5, math.inf, 10.0).stuck(0.0, 0.0, 3.0, 0.0) is False
