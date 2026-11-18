"""
Whole drives on the pretend car (ego -> Gazebo's actuator model): stop at B, brake gently.

The same targets as tools/sim_check.py --drive-to-point: 10, 30 and 60 m, straight, to the side and
behind; at the car's 1 m/s and at Gazebo's sim_speed 3.
"""

import math

from aav_description.stop_law import from_car_model
from aav_drive_to_point.approach import Approach, planned_deceleration, TURN
from aav_drive_to_point.drive import Drive, Steering
from aav_drive_to_point.line import min_radius, Pose
from aav_ego.ackermann import Limits
from aav_sim_models.actuator_model import ActuatorConfig
from fake_car import drive_to
import pytest

CFG = ActuatorConfig()                 # vehicle.yaml's actuators (the GUESSES, 15 Oct)
DECEL = planned_deceleration(CFG.max_brake)
R = min_radius(1.57, CFG.max_steering)
LEAD = 0.8                             # config/drive_to_point.yaml's brake_lead
TARGETS = [(10.0, 0.0), (30.0, 45.0), (60.0, -30.0), (30.0, 150.0)]   # m, degrees left


def go(distance, degrees, cruise, start=Pose(0.0, 0.0, 0.0), curve=None):
    a = start.yaw + math.radians(degrees)
    target = (start.x + distance * math.cos(a), start.y + distance * math.sin(a))
    plan = Approach(cruise=cruise, accel=0.5, decel=DECEL, lead=LEAD, curve=curve)
    run = drive_to(lambda s: Drive(target, plan, Steering(max_curvature=1.0 / R), s),
                   start=start, cfg=CFG,
                   limits=Limits(max_speed=cruise, max_acceleration=1.0, max_deceleration=2.0))
    return target, run


@pytest.mark.parametrize('cruise', [1.0, 3.0])
@pytest.mark.parametrize('distance,degrees', TARGETS)
def test_it_stops_at_b_without_braking_hard(distance, degrees, cruise):
    target, run = go(distance, degrees, cruise)
    _, x, y, yaw, *_ = run.end
    assert math.hypot(target[0] - x, target[1] - y) < 0.3
    # No overshoot: it never passes B by more than a few centimetres.
    assert (x - target[0]) * math.cos(yaw) + (y - target[1]) * math.sin(yaw) < 0.15
    # Never harder than the plan (with a margin for the speed loop), never backwards.
    assert max(d for _, d in run.decelerations()) < DECEL * 1.25
    assert min(r[4] for r in run.rows) > -0.01


@pytest.mark.parametrize('cruise', [1.0, 3.0])
@pytest.mark.parametrize('distance', [10.0, 30.0])
def test_the_stopping_law_stops_at_b_more_gently(distance, cruise):
    # stop_law:=hybrid: the curve, from the same car model's numbers (a 1.0 m/s^2).
    curve = from_car_model(CFG)
    target, run = go(distance, 0.0, cruise, curve=curve)
    _, x, y, yaw, *_ = run.end
    assert math.hypot(target[0] - x, target[1] - y) < 0.3
    assert x - target[0] < 0.15
    # Its braking stays near its own a, below the default law's (1.5 m/s^2).
    assert max(d for _, d in run.decelerations()) < curve.a * 1.25 < DECEL


def test_it_slows_at_about_the_planned_deceleration():
    _, run = go(60.0, 0.0, 3.0)
    slowing = [d for _, d in run.decelerations() if d > 0.3]
    assert slowing and DECEL * 0.6 < max(slowing) <= DECEL * 1.25


def test_behind_it_turns_round_first():
    _, run = go(30.0, 150.0, 1.0)
    phases = [r[6] for r in run.rows]
    assert phases[0] == TURN and TURN not in phases[-10:]


def test_a_start_facing_any_way():
    _, run = go(30.0, 0.0, 1.0, start=Pose(5.0, -3.0, 2.0))
    target = (5.0 + 30.0 * math.cos(2.0), -3.0 + 30.0 * math.sin(2.0))
    assert math.hypot(target[0] - run.end[1], target[1] - run.end[2]) < 0.3
