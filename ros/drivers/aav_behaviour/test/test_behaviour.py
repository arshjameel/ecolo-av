"""The behaviour states, one test per row of the design's event table, and the cap's rules."""

import math

from aav_behaviour.behaviour import (along_until, APPROACH, ASK, Behaviour, diagnostic, DRIVE,
                                     GO, lane_ahead, path_left, Settings, Stop, STOPPED, WAIT,
                                     World)
from aav_description.stop_law import FLOOR, from_car_model
import pytest

CURVE = from_car_model()
FAR = 100.0


def world(now, speed=1.0, locks=None, stops=(), **kw):
    return World(now=now, speed=speed, odom_age=0.05, locks=locks or {}, stops=stops, **kw)


def run(b, worlds):
    return [b.step(w) for w in worlds]


def at_sign(b, now=0.0, left=1.0):
    """Bring a fresh machine into APPROACH for a sign `left` m ahead."""
    d = b.step(world(now, stops=[Stop('sign', left)], sign_age=0.0))
    assert d.state == APPROACH and d.reason == 'sign'
    return d


def test_starts_in_drive_sending_no_limit():
    b = Behaviour(CURVE)
    d = b.step(world(0.0))
    assert (d.state, d.cap) == (DRIVE, 0.0)


def test_a_far_stop_doesnt_bind_and_a_near_one_starts_the_approach():
    b = Behaviour(CURVE)
    assert b.step(world(0.0, stops=[Stop('goal', FAR)], goal_active=True)).state == DRIVE
    d = b.step(world(0.1, stops=[Stop('goal', 1.0)], goal_active=True))
    assert d.state == APPROACH and d.reason == 'goal'


def test_the_cap_is_never_between_zero_and_the_floor_nor_above_the_base():
    b = Behaviour(CURVE, Settings(base=1.0))
    t = 0.0
    for left in [20.0 - 0.1 * i for i in range(260)]:
        t += 0.05
        d = b.step(world(t, speed=0.5, stops=[Stop('sign', left)], sign_age=0.0))
        assert d.cap == 0.0 or FLOOR <= d.cap <= 1.0


def test_the_cap_falls_no_faster_than_the_car_can_slow():
    b = Behaviour(CURVE, Settings(base=3.0, fall=2.0))
    b.step(world(0.0, speed=3.0))
    d = b.step(world(0.05, speed=3.0, stops=[Stop('obstacle', 0.5)]))
    # From no limit (the base, 3 m/s): at most 2 m/s^2 x 0.05 s lower.
    assert d.cap == pytest.approx(3.0 - 0.1)


def test_approach_to_stopped_when_its_lock_comes_and_the_car_is_still():
    b = Behaviour(CURVE)
    at_sign(b)
    d = b.step(world(1.0, speed=0.0, locks={'stop_sign': True},
                     stops=[Stop('sign', 0.1)], sign_age=0.0))
    assert d.state == STOPPED
    # STOPPED: always, next tick, WAIT.
    assert b.step(world(1.1, speed=0.0, locks={'stop_sign': True})).state == WAIT


def test_approach_to_wait_when_another_lock_holds_a_still_car():
    b = Behaviour(CURVE)
    at_sign(b)
    d = b.step(world(1.0, speed=0.0, locks={'sensors': True}, stops=[Stop('sign', 3.0)],
                     sign_age=0.0))
    assert (d.state, d.reason) == (WAIT, 'sensors')


def test_a_sign_passed_with_no_lock_goes_back_to_drive_and_warns():
    b = Behaviour(CURVE)
    at_sign(b)
    d = b.step(world(1.0, stops=[Stop('sign', -1.5)], sign_age=0.0))
    assert d.state == DRIVE and 'sign passed, no lock' in d.warnings


def test_a_sign_lost_for_5_s_goes_back_to_drive_and_warns():
    b = Behaviour(CURVE)
    at_sign(b)
    assert b.step(world(3.0, sign_age=3.0)).state == APPROACH
    d = b.step(world(6.0, sign_age=6.0))
    assert d.state == DRIVE and 'sign lost' in d.warnings


def test_a_goal_that_ends_ends_its_approach():
    b = Behaviour(CURVE)
    b.step(world(0.0, stops=[Stop('goal', 1.0)], goal_active=True))
    assert b.step(world(0.1, goal_active=False)).state == DRIVE


def test_a_new_goal_far_away_ends_the_last_goals_approach():
    # The next goal starts at once: still a goal, but its path is long, so no cap binds.
    b = Behaviour(CURVE)
    b.step(world(0.0, stops=[Stop('goal', 1.0)], goal_active=True))
    assert b.state == APPROACH
    d = b.step(world(0.1, stops=[Stop('goal', 15.0)], goal_active=True))
    assert d.state in (GO, DRIVE)


def test_an_obstacle_that_goes_away_lets_the_car_speed_up_again():
    b = Behaviour(CURVE)
    for i in range(10):
        b.step(world(0.1 * i, stops=[Stop('obstacle', 1.0)]))
    assert b.state == APPROACH and b.cap < 1.0
    d = b.step(world(1.0))
    assert d.state == GO and d.cap >= FLOOR


def test_wait_to_go_when_every_lock_is_off_then_drive_at_the_base():
    b = Behaviour(CURVE, Settings(base=1.0, accel=0.5))
    at_sign(b)
    b.step(world(1.0, speed=0.0, locks={'stop_sign': True}, stops=[Stop('sign', 0.0)],
                 sign_age=0.0))
    b.step(world(1.1, speed=0.0, locks={'stop_sign': True}))
    d = b.step(world(5.0, speed=0.0))
    assert d.state == GO and d.cap >= FLOOR
    caps = [b.step(world(5.0 + 0.1 * i, speed=0.3)).cap for i in range(1, 30)]
    # Rising at 0.5 m/s^2 from the floor, then no limit at the base.
    assert caps[0] < caps[5] and b.state == DRIVE and caps[-1] == 0.0


def test_a_sign_stopped_for_never_brings_the_approach_back():
    # Gazebo, 14 Nov: after the sign's hold, its last point (0.5 m behind the stop) came
    # back as APPROACH(sign) at the floor, until "sign lost" 5 s after that point.
    b = Behaviour(CURVE, Settings(base=1.0, accel=0.5))
    at_sign(b, left=1.5)
    b.step(world(1.0, speed=0.0, locks={'stop_sign': True}, stops=[Stop('sign', -0.5)],
                 sign_age=0.0))
    b.step(world(1.1, speed=0.0, locks={'stop_sign': True}, stops=[Stop('sign', -0.5)],
                 sign_age=0.1))
    assert b.state == WAIT
    # The lock goes off, the old point still fresh: GO, then DRIVE, never APPROACH(sign).
    states = [b.step(world(4.0 + 0.1 * i, speed=0.3, stops=[Stop('sign', -0.5)],
                           sign_age=3.0 + 0.1 * i)).state for i in range(30)]
    assert APPROACH not in states and states[0] == GO and states[-1] == DRIVE
    # A point newer than the stop is a new sign.
    d = b.step(world(8.0, stops=[Stop('sign', 1.0)], sign_age=0.0))
    assert (d.state, d.reason) == (APPROACH, 'sign')


def test_a_long_hold_that_isnt_the_signs_asks_a_person():
    b = Behaviour(CURVE, Settings(ask_after=30.0))
    b.step(world(0.0, speed=0.0, locks={'stop_zone': True}))
    assert b.state == WAIT
    assert b.step(world(29.0, speed=0.0, locks={'stop_zone': True})).state == WAIT
    d = b.step(world(31.0, speed=0.0, locks={'stop_zone': True}))
    assert d.state == ASK and d.cap == FLOOR
    # ASK: every lock off, GO. Silence never makes the car move: the locks do.
    assert b.step(world(40.0, speed=0.0, locks={'stop_zone': True})).state == ASK
    assert b.step(world(41.0, speed=0.0)).state == GO


def test_the_signs_own_hold_never_asks():
    b = Behaviour(CURVE, Settings(ask_after=30.0))
    b.step(world(0.0, speed=0.0, locks={'stop_sign': True}))
    assert b.step(world(60.0, speed=0.0, locks={'stop_sign': True})).state == WAIT


def test_geofence_near_at_a_standstill_asks_at_once():
    b = Behaviour(CURVE)
    b.step(world(0.0, speed=0.0, locks={'geofence': True}, near=True))
    d = b.step(world(0.1, speed=0.0, locks={'geofence': True}, near=True))
    assert d.state == ASK


def test_go_to_wait_on_a_lock_and_go_to_approach_on_a_new_sign():
    b = Behaviour(CURVE)
    b.step(world(0.0, speed=0.0, locks={'stop_zone': True}))
    b.step(world(1.0, speed=0.0))
    assert b.state == GO
    assert b.step(world(1.1, speed=0.2, locks={'stop_zone': True})).state == WAIT
    b.step(world(1.2, speed=0.0))
    assert b.state == GO
    d = b.step(world(1.3, speed=0.3, stops=[Stop('sign', 0.2)], sign_age=0.0))
    assert d.state == APPROACH


def test_any_state_an_aborted_goal_asks_and_the_estop_waits():
    b = Behaviour(CURVE)
    assert b.step(world(0.0, goal_aborted=True)).state == ASK
    b = Behaviour(CURVE)
    at_sign(b)
    d = b.step(world(0.5, locks={'estop': True}))
    assert (d.state, d.reason) == (WAIT, 'e-stop')


def test_stale_odometry_sends_the_floor_and_stays():
    b = Behaviour(CURVE)
    d = b.step(World(now=0.0, speed=1.0, odom_age=0.8))
    assert (d.state, d.cap) == (DRIVE, FLOOR) and d.warnings


def test_a_person_driving_changes_nothing():
    # People aren't capped: the node never sees who drives, and nothing here depends on it.
    a, b = Behaviour(CURVE), Behaviour(CURVE)
    for t in range(5):
        assert a.step(world(t, speed=2.0)) == b.step(world(t, speed=0.1))


def test_the_signs_cap_never_rises_in_approach():
    b = Behaviour(CURVE)
    at_sign(b, left=0.9)
    caps = [b.step(world(0.1 * i, stops=[Stop('sign', left)], sign_age=0.0)).cap
            for i, left in enumerate((0.8, 0.6, 4.0, 0.4), start=1)]
    assert all(later <= earlier for earlier, later in zip(caps, caps[1:]))


def test_path_left_and_along_until():
    path = [(float(x), 0.0) for x in range(11)]
    left, off = path_left(path, 3.2, 0.5)
    assert left == pytest.approx(6.8) and off == pytest.approx(0.5)
    assert along_until(path, 3.0, 0.0, lambda x, y: x >= 7.5) == pytest.approx(5.0)
    assert along_until(path, 3.0, 0.0, lambda x, y: False) == math.inf
    assert path_left([], 0.0, 0.0) == (math.inf, math.inf)


def test_lane_ahead_finds_the_nearest_thing_in_the_lane():
    n = 360
    inc = 2 * math.pi / n
    ranges = [math.inf] * n
    ranges[0] = 8.0                 # straight ahead, 8 m
    ranges[90] = 1.0                # beside the car, left: not in the lane
    ranges[5] = 6.0                 # 5 degrees left at 6 m: y 0.52, in a 0.6 m half lane
    assert lane_ahead(ranges, 0.0, inc, front=2.0, half_width=0.6) == pytest.approx(
        6.0 * math.cos(5 * inc))
    assert lane_ahead([math.inf] * n, 0.0, inc, 2.0, 0.6) == math.inf


def test_the_state_topics_values():
    b = Behaviour(CURVE)
    values = diagnostic(b.step(world(0.0)))
    assert values['state'] == DRIVE and values['cap_mps'] == '0.000'


def test_the_point_drivers_line_outranks_nav2s_plan():
    from aav_behaviour.behaviour import active_plan
    line, plan = [(0.0, 0.0), (20.0, 0.0)], [(0.0, 0.0), (5.0, 5.0)]
    assert active_plan(line, plan, True) == (line, 'point')
    assert active_plan([], plan, True) == (plan, 'nav')
    assert active_plan([], plan, False) == ([], '')
    assert active_plan([(1.0, 1.0)], [], True) == ([], '')      # one point is no line


def test_moving_ahead_waits_for_what_moves_and_leaves_parked_things_to_the_planner():
    from aav_behaviour.behaviour import moving_ahead, Seen
    car = (10.0, 5.0, math.pi / 2)            # facing +y in odom
    walker = Seen(x=10.3, y=15.0, vx=0.8, vy=0.0, length=0.5, width=0.5)
    parked = Seen(x=10.0, y=9.0, vx=0.05, vy=0.0, length=1.0, width=1.0)
    beside = Seen(x=14.0, y=12.0, vx=1.0, vy=0.0, length=0.5, width=0.5)
    behind = Seen(x=10.0, y=2.0, vx=1.0, vy=0.0, length=0.5, width=0.5)
    things = [walker, parked, beside, behind]
    ahead = moving_ahead(things, car, front=1.8, half_width=0.6)
    assert ahead == pytest.approx(9.75)        # the walker, 10 m ahead, less half its length
    assert moving_ahead([parked], car, front=1.8, half_width=0.6) == math.inf
    # Inside the bumper's reach already: it counts from the bumper.
    close = Seen(x=10.0, y=6.6, vx=0.5, vy=0.0, length=0.6, width=0.6)
    assert moving_ahead([close], car, front=1.8, half_width=0.6) == pytest.approx(1.8)
