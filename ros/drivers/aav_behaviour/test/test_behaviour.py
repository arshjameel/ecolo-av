"""The behaviour states, one test per row of the design's event table, and the cap's rules."""

import math

from aav_behaviour.behaviour import (along_until, APPROACH, ASK, Behaviour, diagnostic, DRIVE,
                                     GO, lane_ahead, moving_near, MOVING_REASON, MovingFor,
                                     Nearby, path_left, Seen, Settings, Stop, STOPPED, WAIT,
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
    held = b.cap
    d = b.step(world(1.0))
    assert d.state == APPROACH and d.cap == pytest.approx(held)    # gone: hold the release
    d = b.step(world(2.05))
    assert d.state == GO and d.cap >= FLOOR


def test_an_obstacle_flickering_at_the_lanes_edge_doesnt_flap_the_state():
    # TODO 189: a person crossing dropped in and out of the lane tick by tick.
    b = Behaviour(CURVE)
    states = []
    for i in range(60):
        here = (i // 3) % 2 == 0           # 0.3 s seen, 0.3 s not, over and over
        d = b.step(world(0.1 * i, stops=[Stop('obstacle', 1.0)] if here else []))
        states.append(d.state)
    first = states.index(APPROACH)
    assert all(s == APPROACH for s in states[first:])


def test_a_new_stop_still_binds_while_the_release_is_held():
    b = Behaviour(CURVE)
    for i in range(10):
        b.step(world(0.1 * i, stops=[Stop('obstacle', 1.0)]))
    held = b.step(world(1.0)).cap          # the obstacle gone: holding
    d = b.step(world(1.1, stops=[Stop('goal', 0.5)]))
    assert (d.state, d.reason) == (APPROACH, 'goal') and d.cap <= held


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


def test_moving_ahead_counts_a_person_who_will_walk_into_the_lane_before_the_car_comes():
    # TODO 194: a person 3.4 m right of the lane, 5.6 m ahead, walking left at 0.8 m/s, the
    # car at 0.9 m/s. Only "in the lane now" saw them a metre from the bumper: too late.
    from aav_behaviour.behaviour import Crossing, moving_ahead, Seen
    car = (0.0, 0.0, 0.0)                     # facing +x in odom: left is +y
    walker = Seen(x=5.6, y=-3.4, vx=0.0, vy=0.8, length=0.5, width=0.5)
    ahead = moving_ahead([walker], car, front=1.8, half_width=0.6, speed=0.9)
    assert ahead == pytest.approx(5.35)       # where they will cross, less half their length
    # Without the prediction, they don't count until they are in the lane.
    assert moving_ahead([walker], car, front=1.8, half_width=0.6, speed=0.9,
                        crossing=None) == math.inf
    # Walking away from the lane, or along beside it: not in the way.
    away = Seen(x=5.6, y=-3.4, vx=0.0, vy=-0.8, length=0.5, width=0.5)
    along = Seen(x=5.6, y=-3.4, vx=1.2, vy=0.1, length=0.5, width=0.5)
    assert moving_ahead([away, along], car, front=1.8, half_width=0.6, speed=0.9) == math.inf
    # From the left, walking right: the same.
    left = Seen(x=5.6, y=3.4, vx=0.0, vy=-0.8, length=0.5, width=0.5)
    assert moving_ahead([left], car, front=1.8, half_width=0.6, speed=0.9) == pytest.approx(5.35)
    # Far from the lane and slow to reach it: past the horizon, not yet.
    slow = Seen(x=20.0, y=-8.0, vx=0.0, vy=0.4, length=0.5, width=0.5)
    assert moving_ahead([slow], car, front=1.8, half_width=0.6, speed=0.9) == math.inf
    # They'd come into the lane long after the car has gone by: not in the way.
    late = Seen(x=4.0, y=-2.0, vx=0.0, vy=0.4, length=0.5, width=0.5)
    assert moving_ahead([late], car, front=1.8, half_width=0.6, speed=2.0,
                        crossing=Crossing(slack=1.0)) == math.inf
    # The car standing: within the horizon, they count.
    assert moving_ahead([walker], car, front=1.8, half_width=0.6) == pytest.approx(5.35)


def test_something_moving_near_holds_the_car_until_it_has_gone_release_hold_s():
    b = Behaviour(CURVE, Settings(release_hold=1.0))
    b.step(world(0.0))
    d = b.step(world(0.05, moving=True))
    assert (d.state, d.reason, d.cap) == (WAIT, MOVING_REASON, FLOOR) and b.holding
    # Gone, but not yet 1 s: still held; a flicker back restarts the second.
    assert b.step(world(0.6, speed=0.0)).state == WAIT
    assert b.step(world(0.7, speed=0.0, moving=True)).state == WAIT
    assert b.step(world(1.6, speed=0.0)).state == WAIT and b.holding
    d = b.step(world(1.75, speed=0.0))
    assert (d.state, d.reason) == (GO, MOVING_REASON) and not b.holding


def test_a_hold_for_something_moving_never_outranks_the_estop_and_keeps_a_lock_waiting():
    b = Behaviour(CURVE)
    d = b.step(world(0.0, locks={'estop': True}, moving=True))
    assert (d.state, d.reason) == (WAIT, 'e-stop') and not b.holding
    # The e-stop off, the thing still there: held for it, not GO.
    d = b.step(world(0.1, speed=0.0, moving=True))
    assert (d.state, d.reason) == (WAIT, MOVING_REASON)
    # A lock comes while held: the wait is the lock's; the lock off, the thing near: held.
    d = b.step(world(0.2, speed=0.0, locks={'stop_zone': True}, moving=True))
    assert (d.state, d.reason) == (WAIT, 'stop_zone')
    d = b.step(world(0.3, speed=0.0, moving=True))
    assert (d.state, d.reason) == (WAIT, MOVING_REASON)


def test_ask_waits_for_something_moving_before_it_goes():
    b = Behaviour(CURVE)
    assert b.step(world(0.0, goal_aborted=True)).state == ASK
    d = b.step(world(0.1, speed=0.0, moving=True))
    assert (d.state, d.reason) == (WAIT, MOVING_REASON)


def test_moving_near_holds_for_what_moves_near_or_onto_the_road_and_passes_still_things():
    car = (0.0, 0.0, 0.0)
    road = [(0.5 * i, 0.0) for i in range(80)]

    def near(*things, nearby=Nearby()):
        return moving_near(things, car, 2.0, road, 0.6, nearby)

    # An oncoming car in the other lane, 8 m ahead and 3.5 m over: within 10 m.
    oncoming = Seen(10.0, 3.5, -1.0, 0.0, 4.5, 1.8)
    assert near(oncoming) == pytest.approx(math.hypot(8.0, 3.5) - 0.9)
    # A person walking along the roadside 5 m away, level with the car: within 10 m.
    assert near(Seen(1.0, -5.0, 1.0, 0.0, 0.5, 0.5)) == pytest.approx(4.75)
    # The parked box 6 m ahead, its speed noise under 0.3 m/s: never.
    assert near(Seen(8.0, 0.0, 0.1, 0.1, 1.0, 1.0)) == math.inf
    # Cross traffic 25 m ahead, 6 m to the side, 3 m/s towards the road: on it in 1.5 s.
    cross = Seen(25.0, -6.0, 0.0, 3.0, 4.5, 1.8)
    assert near(cross) == pytest.approx(math.hypot(23.0, 6.0) - 0.9)
    # Going away from the road, or 5 s from it: not yet.
    assert near(Seen(25.0, -6.0, 0.0, -3.0, 4.5, 1.8)) == math.inf
    assert near(Seen(25.0, -16.0, 0.0, 3.0, 4.5, 1.8)) == math.inf
    # No road: only the radius counts.
    assert moving_near([cross], car, 2.0, [], 0.6) == math.inf
    # Behind the car, measured from base_link; a smaller radius.
    assert near(Seen(-4.0, 0.0, 1.0, 0.0, 0.5, 0.5),
                nearby=Nearby(radius=3.0, within=0.0)) == math.inf


def test_a_thing_holds_only_once_it_has_moved_settle_s_and_a_coasting_one_never_starts():
    sure = MovingFor(settle=1.0)
    walker = Seen(5.0, -3.0, 0.0, 0.8, 0.5, 0.5)
    box = Seen(8.0, 0.0, 0.6, 0.0, 1.0, 1.0)
    # A box's burst of speed under 1 s: never; a walker moving on: after 1 s.
    assert sure.update(0.0, [(1, False, walker), (2, False, box)]) == []
    assert sure.update(0.5, [(1, False, walker), (2, False, box)]) == []
    still = Seen(8.0, 0.0, 0.05, 0.0, 1.0, 1.0)
    assert sure.update(0.6, [(1, False, walker), (2, False, still)]) == []
    assert sure.update(1.0, [(1, False, walker), (2, False, box)]) == [walker]
    # Sure, it walks into the blind zone (coasting): still sure. A new track coasting off
    # at speed never starts a run.
    assert sure.update(1.1, [(1, True, walker), (3, True, box)]) == [walker]
    assert sure.update(2.5, [(1, True, walker), (3, True, box)]) == [walker]
    # It stands: gone at once; walks again: 1 s more.
    halt = Seen(5.0, -1.0, 0.0, 0.1, 0.5, 0.5)
    assert sure.update(2.6, [(1, False, halt)]) == []
    assert sure.update(2.7, [(1, False, walker)]) == []
    assert sure.update(3.7, [(1, False, walker)]) == [walker]


def test_a_thing_that_grows_as_it_comes_into_view_never_holds():
    sure = MovingFor(settle=1.0)
    # A building coming into view: 0.8 m, then 3 m, then 6 m, its middle moving 1.5 m/s.
    for now, size in ((0.0, 0.8), (0.5, 3.0), (1.0, 4.5), (1.5, 6.0), (1.9, 6.2)):
        assert sure.update(now, [(9, False, Seen(20.0, 12.0, 1.5, 0.0, size, 1.0))]) == []
    # A person stays near their size: 0.5-0.8 m.
    for now, size in ((0.0, 0.5), (0.5, 0.8)):
        assert sure.update(now, [(1, False, Seen(5.0, -3.0, 0.0, 0.8, size, 0.4))]) == []
    walker = Seen(5.0, -3.0, 0.0, 0.8, 0.6, 0.4)
    assert sure.update(1.0, [(1, False, walker)]) == [walker]


def test_a_stop_line_is_stood_at_then_gone():
    # TODO 220; behaviour-core's stop_lines test, the same case.
    from aav_behaviour.behaviour import StopLines
    plan = [(float(i), 0.0) for i in range(61)]
    lines = StopLines()
    lines.set_lines([(20.0, 0.5), (40.0, 10.0)])    # one on the way, one 10 m off it
    lefts = lines.update(0.0, plan, (5.0, 0.0), 2.0, 2.0)
    assert lefts == pytest.approx([13.0])
    at = (17.8, 0.0)
    assert len(lines.update(1.0, plan, at, 0.0, 2.0)) == 1
    assert len(lines.update(2.9, plan, at, 0.0, 2.0)) == 1
    assert lines.update(3.0, plan, at, 0.0, 2.0) == []
    assert lines.update(4.0, plan, (25.0, 0.0), 2.0, 2.0) == []
    creep = StopLines()
    creep.set_lines([(20.0, 0.0)])
    assert all(len(creep.update(0.1 * i, plan, at, 0.2, 2.0)) == 1 for i in range(100))
    assert creep.update(11.0, plan, None, 0.0, 2.0) == []
