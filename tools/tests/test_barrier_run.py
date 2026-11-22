"""
The numbers behind barrier-watch (tools/barrier_run.py): the gap, the verdict, the switches.

Outside the capstone's scope (Ben's own research, toward his MASc thesis): docs/barriers.md.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import barrier_run as br  # noqa: E402, I100 - beside the tests' folder

BODY = br.Body(rear=-0.5, front=1.5, half_width=0.5)
RADIUS = 0.25
CENTRE = br.object_centre(BODY, 15.0, 0.0, RADIUS)      # x 16.75, on the line


def drive(points, **flags):
    """Return samples 0.1 s apart through (x, y, v) points, the car heading along x."""
    return [br.Sample(t=0.1 * i, x=x, y=y, yaw=0.0, v=v, **flags)
            for i, (x, y, v) in enumerate(points)]


def test_the_object_stands_ahead_of_the_bumper_and_beside_the_line():
    assert br.object_centre(BODY, 15.0, 0.4, 0.25) == (16.75, 0.4)


def test_the_gap_is_from_the_body_not_from_base_link():
    at = br.Sample(t=0.0, x=0.0, y=0.0, yaw=0.0, v=0.0)
    assert math.isclose(br.gap(BODY, at, CENTRE, RADIUS), 15.0)
    # Beside the car, level with its middle: from the body's side.
    assert math.isclose(br.gap(BODY, at, (0.5, 2.0), RADIUS), 1.25)
    # Off a corner: the diagonal.
    assert math.isclose(br.gap(BODY, at, (2.5, 1.5), RADIUS), math.sqrt(2.0) - RADIUS)
    # The car turned a quarter left: its front now points at +y.
    turned = br.Sample(t=0.0, x=0.0, y=0.0, yaw=math.pi / 2.0, v=0.0)
    assert math.isclose(br.gap(BODY, turned, (0.0, 3.0), RADIUS), 1.25)


def test_inside_the_body_the_gap_is_negative():
    at = br.Sample(t=0.0, x=0.0, y=0.0, yaw=0.0, v=0.0)
    assert br.gap(BODY, at, (1.4, 0.0), RADIUS) < -RADIUS
    assert br.gap(BODY, at, (1.6, 0.0), RADIUS) < 0.0      # the centre out, the edge in


def test_the_run_frame_goes_there_and_back():
    start = (3.0, -2.0, math.pi / 2.0)
    x, y, yaw = br.to_run_frame(3.0, 5.0, math.pi / 2.0, start)
    assert (round(x, 9), round(y, 9), yaw) == (7.0, 0.0, 0.0)
    back = br.from_run_frame(7.0, 0.0, start)
    assert math.isclose(back[0], 3.0, abs_tol=1e-9) and math.isclose(back[1], 5.0)


def test_through_the_object_is_touched():
    v = br.verdict(drive([(x, 0.0, 1.0) for x in range(0, 30)]), BODY, CENTRE, RADIUS)
    assert v.outcome == 'touched' and v.least_gap < 0.0 and v.first == 'nobody'


def test_standing_before_it_is_stopped_and_the_stop_zone_was_first():
    samples = (drive([(x, 0.0, 1.0) for x in range(0, 14)])
               + drive([(14.0, 0.0, 0.5)], zone_stop=True)
               + drive([(14.5, 0.0, 0.0)] * 3, zone_stop=True))
    v = br.verdict(samples, BODY, CENTRE, RADIUS)
    assert v.outcome == 'stopped' and not v.passed
    assert math.isclose(v.least_gap, 0.5) and v.first.startswith('the stop zone')
    assert v.top_speed == 1.0 and v.stood_x == 14.5


def test_round_it_with_no_stop_asked_is_the_steering_alone():
    points = ([(x, 0.0, 3.0) for x in range(0, 10)]
              + [(x, -2.0, 3.0) for x in range(10, 22)]
              + [(x, 0.0, 3.0) for x in range(22, 30)])
    v = br.verdict(drive(points), BODY, CENTRE, RADIUS)
    assert v.outcome == 'went round' and v.passed and v.first.startswith('the steering alone')
    assert not v.zone_stopped and not v.rule_stopped and 'stops asked by: nobody' in v.line()
    assert math.isclose(v.least_gap, 1.5 - RADIUS) and v.widest == 2.0


def test_the_stop_rule_first_when_it_comes_before_the_stop_zone():
    samples = (drive([(x, 0.0, 1.0) for x in range(0, 12)])
               + [br.Sample(t=5.0, x=12.0, y=0.0, yaw=0.0, v=0.8, barrier_stop=True),
                  br.Sample(t=6.0, x=13.0, y=0.0, yaw=0.0, v=0.3, barrier_stop=True,
                            zone_stop=True),
                  br.Sample(t=7.0, x=13.2, y=0.0, yaw=0.0, v=0.0, zone_stop=True)])
    v = br.verdict(samples, BODY, CENTRE, RADIUS)
    assert v.first.startswith("the tracker's stop rule") and v.zone_stopped and v.rule_stopped
    assert 'stops asked by: the stop rule and the stop zone' in v.line()


def test_a_small_sway_is_nobody_acting():
    # The follower sways a little on its own (0.1 to 0.7 m seen in Ravensim): not a swerve.
    points = [(x, -0.6, 1.0) for x in range(0, 30)]
    v = br.verdict(drive(points), BODY, br.object_centre(BODY, 15.0, 3.0, RADIUS), RADIUS)
    assert v.outcome == 'went round' and v.first == 'nobody'


def test_a_stop_before_the_car_drives_is_not_this_drives():
    # The stop zone says stop for a moment while its sensors start (seen in Ravensim).
    samples = (drive([(0.0, 0.0, 0.0)] * 3, zone_stop=True)
               + [br.Sample(t=1.0 + 0.1 * x, x=x, y=0.0, yaw=0.0, v=1.0) for x in range(0, 30)])
    v = br.verdict(samples, BODY, CENTRE, RADIUS)
    assert v.first == 'nobody' and not v.zone_stopped


def test_the_time_the_backing_up_and_how_lost_it_was_are_kept():
    samples = [br.Sample(t=0.0, x=0.0, y=0.0, yaw=0.0, v=0.0),
               br.Sample(t=1.0, x=0.5, y=0.0, yaw=0.0, v=1.0, off=0.1),
               br.Sample(t=2.0, x=1.5, y=0.0, yaw=0.0, v=1.0, off=0.4),
               br.Sample(t=3.0, x=1.0, y=0.0, yaw=0.0, v=-0.5),
               br.Sample(t=4.0, x=1.0, y=0.0, yaw=0.0, v=0.0)]
    v = br.verdict(samples, BODY, CENTRE, RADIUS)
    assert (v.took, v.backed, v.lost) == (3.0, 0.5, 0.05)
    assert 'after 3 s; backed up 0.5 m; localization off by 0.05 m (median)' in v.line()


def test_no_samples_and_a_car_still_rolling():
    assert br.verdict([], BODY, CENTRE, RADIUS).outcome == 'no data'
    rolling = br.verdict(drive([(x, 0.0, 1.0) for x in range(0, 5)]), BODY, CENTRE, RADIUS)
    assert rolling.outcome == 'still moving'


def test_each_mode_has_its_switches():
    plain = br.launch_switches('plain', 1.0)
    assert plain == ['world:=road', 'rviz:=false', 'tracker:=mpc_lag_rs', 'ghost:=true']
    assert br.launch_switches('step', 3.0)[4:] == ['mpc_barriers:=true', 'mpc_barrier_gamma:=1',
                                                   'sim_speed:=3']
    assert br.launch_switches('barrier', 1.0)[4:] == ['mpc_barriers:=true']
    assert 'mpc_barrier_gamma:=0.02' in br.launch_switches('barrier', 1.0, gamma=0.02)


def test_wrong_settings_are_refused():
    for mode, speed, gamma in (('fence', 1.0, None), ('plain', 0.0, None), ('plain', 6.0, None),
                               ('plain', 1.0, 0.05), ('barrier', 1.0, 0.0),
                               ('barrier', 1.0, 1.5)):
        with pytest.raises(ValueError):
            br.launch_switches(mode, speed, gamma)
