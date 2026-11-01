"""When a drive to B may start, when it must stop, and how close is close enough."""

from aav_actuator_tests.safety import Kind
from aav_drive_to_point import rules
from aav_drive_to_point.line import min_radius, Pose

LIMITS = rules.Limits()
GOOD = rules.Now(pose_age=0.1, fix_age=0.3, estop=False, locked=False, kind=Kind.SIM)
R = min_radius(1.57, 0.5)


def now(**changes):
    return rules.Now(**{**GOOD.__dict__, **changes})


def test_a_healthy_simulator_may_start():
    assert rules.refuse_start(GOOD, LIMITS, confirmed=False) is None


def test_the_real_car_needs_a_person_to_confirm():
    for kind in (Kind.REAL, Kind.UNKNOWN):
        why = rules.refuse_start(now(kind=kind), LIMITS, confirmed=False)
        assert why and rules.CONFIRM_WORD in why
        assert rules.refuse_start(now(kind=kind), LIMITS, confirmed=True) is None
    assert rules.refuse_start(now(kind=Kind.BENCH), LIMITS, confirmed=False) is None
    assert 'REPLAY' in rules.refuse_start(now(kind=Kind.REPLAY), LIMITS, confirmed=True)


def test_every_reason_to_stop():
    cases = {
        'e-stop': now(estop=True),
        'watchdog': now(locked=True),
        'no position': now(pose_age=None),
        'position is stale': now(pose_age=0.6),
        'no GPS fix': now(fix_age=None),
        'GPS fix is stale': now(fix_age=2.5),
        'someone else': now(taken_over=True),
        'watchdog is never heard': now(lock_age=None),
        'watchdog is silent': now(lock_age=1.5),
    }
    for words, state in cases.items():
        assert words in rules.stop_reason(state, LIMITS), words
        assert words in rules.refuse_start(state, LIMITS, confirmed=True), words
    assert rules.stop_reason(GOOD, LIMITS) is None


def test_without_gps_only_where_none_runs():
    no_gps = rules.Limits(require_gps=False)
    assert rules.stop_reason(now(fix_age=None), no_gps) is None


def test_targets():
    car = Pose(0.0, 0.0, 0.0)
    assert rules.refuse_target(car, (60.0, 0.0), LIMITS, R) is None
    assert 'over the limit' in rules.refuse_target(car, (150.0, 150.0), LIMITS, R)
    assert 'at B already' in rules.refuse_target(car, (0.2, 0.0), LIMITS, R)
    assert 'tightest turn' in rules.refuse_target(car, (0.0, 2 * R), LIMITS, R)
    assert 'not a number' in rules.refuse_target(car, (float('nan'), 0.0), LIMITS, R)
    near = rules.Limits(max_distance=20.0)
    assert 'over the limit of 20 m' in rules.refuse_target(car, (30.0, 0.0), near, R)


def test_numbers():
    assert rules.check_numbers(0.0, 0.0, 0.0) is None
    assert rules.check_numbers(1.0, 0.5, 1.5) is None
    assert 'cruise_speed' in rules.check_numbers(-1.0, 0.0, 0.0)
    assert 'deceleration' in rules.check_numbers(1.0, 0.5, float('inf'))


def test_tolerance_is_1_m_with_rtk_and_2_m_without():
    assert rules.tolerance(rules.STATUS_GBAS_FIX) == 1.0
    for status in (rules.STATUS_FIX, rules.STATUS_SBAS_FIX, rules.STATUS_NO_FIX, None):
        assert rules.tolerance(status) == 2.0


def test_confirmation_is_the_word_exactly():
    assert rules.confirmed('drive') and rules.confirmed(' drive\n')
    assert not rules.confirmed('yes') and not rules.confirmed(None) and not rules.confirmed('')


def test_no_watchdog_where_none_runs():
    assert rules.stop_reason(now(lock_age=None), rules.Limits(watchdog_timeout=0.0)) is None


def test_standing_still_though_asked_to_move():
    assert rules.not_moving(commanded=0.5, measured=0.0)
    assert not rules.not_moving(commanded=0.5, measured=0.3)
    assert not rules.not_moving(commanded=0.1, measured=0.0)    # slowing to B: fine


def test_a_b_outside_the_geofence_is_refused():
    assert rules.refuse_outside(None) is None          # no fence heard: the lock holds it
    assert rules.refuse_outside(3.0) is None
    assert 'outside the geofence' in rules.refuse_outside(-0.5)
