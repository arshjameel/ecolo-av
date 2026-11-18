"""run.py: the plan from ego's caps, and a drive's ticks: stops, cancels, the end."""

from aav_actuator_tests.safety import Kind
from aav_drive_to_point import rules, run
from aav_drive_to_point.line import min_radius, Pose

RADIUS = min_radius(1.57, 0.5)
FRESH = rules.Now(pose_age=0.01, fix_age=0.05, estop=False, locked=False, kind=Kind.SIM,
                  lock_age=0.1)


def a_run(**settings):
    approach, why = run.plan(0.0, 0.0, 0.0, {'max_speed': 1.5}, run.Settings(**settings))
    assert why is None
    return run.Run((12.0, 0.0), approach, run.Settings(**settings), rules.Limits(), RADIUS,
                   Pose(0.0, 0.0, 0.0), 10.0)


def test_the_plan_takes_ego_caps_and_refuses_bad_numbers():
    approach, _ = run.plan(3.0, 0.0, 0.0, {'max_speed': 1.5, 'max_deceleration': 1.0},
                           run.Settings())
    assert (approach.cruise, approach.accel, approach.decel) == (1.5, 0.5, 1.0)
    # Without ego: the car's 1 m/s.
    assert run.plan(0.0, 0.0, 0.0, {}, run.Settings())[0].cruise == 1.0
    assert run.plan(-1.0, 0.0, 0.0, {}, run.Settings())[1].startswith('cruise_speed')


def test_an_estop_stops_then_five_zeros_finish():
    r = a_run()
    assert r.tick(10.02, FRESH, Pose(0, 0, 0), 0.0, False).send is not None
    estop = rules.Now(**{**vars(FRESH), 'estop': True})
    step = r.tick(10.04, estop, Pose(0, 0, 0), 0.0, False)
    assert step.send is None and 'e-stop' in step.stopped
    steps = [r.tick(10.06 + 0.02 * i, FRESH, Pose(0, 0, 0), 0.0, False) for i in range(5)]
    assert all(s.send == (0.0, 0.0) for s in steps)
    assert [s.finished for s in steps] == [False] * 4 + [True]
    assert r.outcome(Pose(0, 0, 0), 2, True, 10.2).status == 'aborted'


def test_a_cancel_wins_over_a_later_stop():
    r = a_run()
    assert r.tick(10.02, FRESH, Pose(0, 0, 0), 0.0, True) == run.Tick()
    estop = rules.Now(**{**vars(FRESH), 'estop': True})
    assert r.tick(10.04, estop, Pose(0, 0, 0), 0.0, False).stopped is None
    assert r.outcome(Pose(0, 0, 0), None, True, 10.1).message.startswith('cancelled')


def test_a_car_that_never_moves_is_stopped():
    r = a_run(stuck_time=0.5)
    stopped = [r.tick(10.0 + 0.02 * i, FRESH, Pose(0, 0, 0), 0.0, False).stopped
               for i in range(1, 200)]
    assert any(s and 'stood still' in s for s in stopped)
