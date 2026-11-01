"""
Every tracker on the pretend car: a straight line, a curve, a step offset; within the limits.

The pretend car (sim_car.py) is ego's model then Gazebo's actuator model, as in pixi run sim.
Each tracker must converge, reach the end, never ask beyond full lock or the steering
rate, and (MPC) plan each tick well inside the 50 ms a 20 Hz tick allows.
"""

import math
import statistics

from aav_track.path import Path
from aav_track.trackers import Limits, make, State
from aav_track.trackers.lqr import LQR
import pytest
from sim_car import arc, follow, LIMITS, MODEL, straight, tracker, uturn, widest

TRACKERS = ['pure_pursuit', 'stanley', 'lqr', 'mpc', 'mpc_full']


def check_limits(run, period=0.05):
    steers = run.steers()
    assert max(abs(s) for s in steers) <= LIMITS.max_steer + 1e-9
    for a, b in zip([0.0] + steers, steers):
        assert abs(b - a) <= LIMITS.max_steer_rate * period + 1e-9


@pytest.mark.parametrize('name', TRACKERS)
@pytest.mark.parametrize('speed', [1.0, 3.0])
def test_a_straight_line_stays_on_it(name, speed):
    run = follow(tracker(name), straight(), (0.0, 0.0, 0.0), cruise=speed)
    assert run.reached
    assert max(run.cross()) < 0.01
    check_limits(run)


@pytest.mark.parametrize('name', TRACKERS)
@pytest.mark.parametrize('speed', [1.0, 3.0])
def test_a_step_offset_converges(name, speed):
    # Starting 1 m left of the line: back on it within 0.05 m by 20 m on, at either speed.
    run = follow(tracker(name), straight(), (0.0, 1.0, 0.0), cruise=speed)
    assert run.reached
    assert max(run.cross(beyond=20.0)) < 0.05
    # It gets there without swinging far past the line.
    assert min(r[6] for r in run.rows) > -0.3
    check_limits(run)


# The worst off-path in the bend, m. Stanley's has no feed-forward for the bend: the front
# axle sits on the path, so base_link cuts the corner (L^2 / 2R, about 0.15 m on an 8 m bend).
BEND = {'pure_pursuit': 0.25, 'stanley': 0.25, 'lqr': 0.1, 'mpc': 0.05, 'mpc_full': 0.1}


@pytest.mark.parametrize('name', TRACKERS)
@pytest.mark.parametrize('speed', [1.0, 3.0])
def test_a_curve_stays_close(name, speed):
    run = follow(tracker(name), arc(radius=8.0), (0.0, 0.0, 0.0), cruise=speed)
    assert run.reached
    assert max(run.cross()) < BEND[name]
    check_limits(run)


def test_the_model_trackers_beat_pure_pursuit_on_the_bend():
    worst = {n: max(follow(tracker(n), arc(radius=6.0), (0, 0, 0), cruise=3.0).cross())
             for n in ('pure_pursuit', 'lqr', 'mpc')}
    assert worst['lqr'] < worst['pure_pursuit'] and worst['mpc'] < worst['pure_pursuit']


def test_lqr_gains_are_scheduled_by_speed():
    lqr = LQR(LIMITS, MODEL)
    slow, fast = lqr.gains(0.5), lqr.gains(5.0)
    assert slow[0] > 0 and slow[1] > 0
    assert not (slow == pytest.approx(fast))
    assert lqr.gains(0.0) == pytest.approx(slow)        # below the table: its first row
    assert lqr.gains(0.625) == pytest.approx((lqr.gains(0.5) + lqr.gains(0.75)) / 2)


def test_lqr_gains_are_scheduled_by_the_bend():
    # About full lock, the steering turns the car 1 / cos^2 faster: other gains, either way.
    lqr = LQR(LIMITS, MODEL)
    straight_on, lock = lqr.gains(1.0, 0.0), lqr.gains(1.0, LIMITS.max_steer)
    assert not (straight_on == pytest.approx(lock, rel=0.01))
    assert lqr.gains(1.0, -LIMITS.max_steer) == pytest.approx(lock)
    assert lqr.gains(1.0, 0.05) == pytest.approx((straight_on + lqr.gains(1.0, 0.1)) / 2)


def test_lqr_previews_a_bend_ahead():
    # A left bend 1 m ahead, at 1 m/s: the preview steers left before it arrives.
    lqr = LQR(LIMITS, MODEL)
    assert len(lqr.preview_gains(1.0)) == 40
    lqr.reset(Path(arc(radius=4.0, lead=1.0)))
    cmd = lqr.step(State(0.0, 0.0, 0.0, 1.0), 0.05)
    assert cmd.debug['feed_forward_rad'] == pytest.approx(0.0)
    assert cmd.debug['preview_rad'] > 0 and cmd.steer > 0


# The U-turn of the test lot's "turn back" leg (29 Oct): Smac's light bulb, forward only,
# in a 14 m lane, at 1 m/s. At the planner's radius (full lock x 1.25: 3.6 m) and at full
# lock itself (2.88 m), where nothing is left to correct with: the worst off the path, m.
UTURN = {'pure_pursuit': (0.2, 0.35), 'stanley': (0.5, 0.6), 'lqr': (0.1, 0.35),
         'mpc': (0.1, 0.4), 'mpc_full': (0.15, 0.25)}
FULL_LOCK = LIMITS.wheelbase / math.tan(LIMITS.max_steer)


@pytest.mark.parametrize('name', TRACKERS)
@pytest.mark.parametrize('lock', [False, True])
def test_a_u_turn_in_a_14_m_lane(name, lock):
    radius = FULL_LOCK if lock else 1.25 * FULL_LOCK
    run = follow(tracker(name), uturn(radius), (0.0, 0.0, 0.0), cruise=1.0)
    assert run.reached
    assert max(run.cross()) < UTURN[name][lock]
    # The body keeps 2 m from both walls (y = +-7 m): room for collision_monitor's zone.
    assert widest(run) < 5.0
    check_limits(run)


def test_lqr_without_its_preview_swings_wider():
    worst = {p: max(follow(tracker('lqr', preview=p), uturn(FULL_LOCK), (0, 0, 0)).cross())
             for p in (0.0, 2.0)}
    assert worst[2.0] < 0.6 * worst[0.0]


def test_mpc_plans_well_inside_a_tick():
    # A step offset: the bounds bind, so the solver runs. Loose, for a busy laptop.
    run = follow(tracker('mpc'), straight(), (0.0, 1.0, 0.0), cruise=3.0)
    mean, worst = statistics.mean(run.step_ms), max(run.step_ms)
    print(f'\nmpc step: {mean:.2f} ms mean, {worst:.1f} ms worst, {len(run.step_ms)} steps')
    assert mean < 10.0 and worst < 50.0


def test_mpc_keeps_its_plan_within_the_bounds():
    mpc = tracker('mpc')
    run = follow(mpc, straight(), (0.0, 2.0, 0.3), cruise=1.0)
    assert run.reached
    plan = mpc.plan
    step = LIMITS.max_steer_rate * mpc.period
    assert max(abs(plan)) <= LIMITS.max_steer + 1e-9
    assert all(abs(b - a) <= step + 1e-9 for a, b in zip(plan, plan[1:]))


def test_bad_names_and_numbers_are_refused():
    with pytest.raises(ValueError, match='no tracker'):
        make('mppi', LIMITS)
    with pytest.raises(ValueError, match='model'):
        make('lqr', LIMITS)
    with pytest.raises(ValueError):
        Limits(wheelbase=1.5, max_steer=2.0, max_steer_rate=0.5)
    with pytest.raises(ValueError):
        tracker('pure_pursuit', lookahead_min=0.0)
    with pytest.raises(RuntimeError):
        tracker('stanley').step(State(0.0, 0.0, 0.0, 1.0), 0.05)


def test_no_steering_wanted_at_the_end_of_a_straight():
    trk = tracker('lqr')
    run = follow(trk, straight(10.0), (0.0, 0.0, 0.0))
    assert run.reached and abs(run.steers()[-1]) < 1e-6
    assert math.isfinite(run.rows[-1][4])


# --- mpc_full: its own speed, within the same limits ---------------------------------------

def check_speed(run, cruise, period=0.05, throttle=1.0, brake=3.0):
    """Check every speed asked: within [0, cruise], changing within throttle and full brake."""
    assert min(run.asked) >= 0.0 and max(run.asked) <= cruise + 1e-9
    for a, b in zip([0.0] + run.asked, run.asked):
        assert -brake * period - 1e-9 <= b - a <= throttle * period + 1e-9


def test_mpc_full_slows_itself_to_swing_the_steering():
    # 2 m/s into a U-turn at full lock: the bend alone allows 1.7 m/s (lateral_accel 1.0);
    # it goes slower than that where the steering swings from one lock to the other.
    run = follow(tracker('mpc_full'), uturn(FULL_LOCK), (0.0, 0.0, 0.0), cruise=2.0)
    assert run.reached
    assert min(run.asked[100:]) < 1.55
    assert max(run.asked[-20:]) > 1.9
    assert widest(run) < 5.0
    check_limits(run)
    check_speed(run, 2.0)


def test_mpc_full_stops_on_the_goal_as_the_policy_does():
    # Never faster than the planned braking (1.5 m/s^2) allows to stop at the path's end.
    run = follow(tracker('mpc_full'), straight(12.0), (0.0, 0.0, 0.0), cruise=3.0)
    assert run.reached
    for row, v in zip(run.rows, run.asked):
        assert v <= math.sqrt(2 * 1.5 * max(12.0 - row[1], 0.0)) + 0.1
    check_speed(run, 3.0)


def test_mpc_full_plans_inside_a_tick():
    # Loose, for a busy laptop.
    run = follow(tracker('mpc_full'), uturn(), (0.0, 0.0, 0.0), cruise=1.0)
    mean = statistics.mean(run.step_ms)
    print(f'\nmpc_full step: {mean:.2f} ms mean, {max(run.step_ms):.1f} ms worst')
    assert mean < 15.0


def test_mpc_full_refuses_bad_numbers():
    with pytest.raises(ValueError):
        tracker('mpc_full', cruise=0.0)
    with pytest.raises(ValueError):
        tracker('mpc_full', horizon=1)


@pytest.mark.parametrize('name', TRACKERS)
def test_seen_late_it_still_settles(name):
    # The car seen 0.2 s late (the EKF, DDS, a busy computer): a 1 m offset still settles
    # without a weave, and the U-turn at the planner's radius is still made in the lane.
    run = follow(tracker(name), straight(), (0.0, 1.0, 0.0), cruise=1.0, delay=0.2)
    assert run.reached and max(run.cross(beyond=20.0)) < 0.05
    run = follow(tracker(name), uturn(1.25 * FULL_LOCK), (0.0, 0.0, 0.0), delay=0.2)
    assert run.reached and widest(run) < 5.0
