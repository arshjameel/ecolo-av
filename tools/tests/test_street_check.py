"""The three-point turn's numbers (tools/street_check.py)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import street_check  # noqa: E402

BODY = {'length': 2.0, 'width': 1.0, 'rear_overhang': 0.2}


def test_the_block_closes_the_street_ahead_across_it():
    # TODO 174: facing north, a block 8 m on: its near face at y 8, turned across.
    name, x, y, yaw = street_check.block(0.0, 0.0, math.pi / 2, 8.0)
    assert name == 'wall_block'
    assert x == pytest.approx(0.0, abs=1e-12)
    assert y == pytest.approx(8.0 + street_check.WALL_THICK / 2.0)
    assert yaw == pytest.approx(math.pi)


def test_a_three_point_turn_changes_direction_twice():
    track = [(0, 0, 0, 0, 1.0), (1, 1, 0, 0, 0.05), (2, 1, 0, 0, -0.8), (3, 0, 0, 0, -0.5),
             (4, 0, 0, 0, 0.0), (5, 1, 0, 0, 0.9)]
    assert street_check.turns(track) == 2


def test_creeping_near_zero_is_not_a_change():
    track = [(0, 0, 0, 0, 1.0), (1, 0, 0, 0, -0.05), (2, 0, 0, 0, 1.0)]
    assert street_check.turns(track) == 0


def test_a_car_down_the_middle_has_the_street_less_half_its_width():
    track = [(0, 0.0, 0.0, 0.0, 1.0), (1, 5.0, 0.0, 0.0, 1.0)]
    assert math.isclose(street_check.clearance(track, 0.0, 0.0, 0.0, 3.25, BODY), 2.75)


def test_a_car_across_the_street_counts_its_front_corners():
    # Facing left (north) at the middle: the front bumper is 1.8 m towards the left wall.
    track = [(0, 0.0, 0.0, math.pi / 2, 0.5)]
    assert math.isclose(street_check.clearance(track, 0.0, 0.0, 0.0, 3.25, BODY), 1.45)


def test_a_corner_through_the_wall_is_below_zero():
    track = [(0, 0.0, 2.0, math.pi / 2, 0.5)]
    assert street_check.clearance(track, 0.0, 0.0, 0.0, 3.25, BODY) < 0.0


def test_a_street_along_another_heading():
    # The street runs north; the car stands 1 m east of its middle line, facing north.
    track = [(0, 1.0, 4.0, math.pi / 2, 1.0)]
    room = street_check.clearance(track, 0.0, 0.0, math.pi / 2, 3.25, BODY)
    assert math.isclose(room, 3.25 - 1.5)


def test_the_walls_faces_are_half_the_street_from_its_line():
    spots = street_check.walls(1.0, 2.0, math.pi / 2.0, 1.3, (0.0, 10.0))
    assert [name for name, *_ in spots] == ['wall_l0', 'wall_l1', 'wall_r0', 'wall_r1']
    # Heading north: left is west (-x), each centre 1.3 + 0.15 m off the line.
    (_, lx, ly), (_, _, ly1), (_, rx, _), _ = spots
    assert math.isclose(lx, 1.0 - 1.45) and math.isclose(rx, 1.0 + 1.45)
    assert math.isclose(ly, 2.0) and math.isclose(ly1, 12.0)


def test_plan_cusps_counts_each_new_plan_once():
    there_and_back = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (1.0, 0.1), (0.0, 0.2)]
    straight = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert street_check.plan_cusps([there_and_back, there_and_back, straight]) == '1, 0'
    assert street_check.plan_cusps([]) == ''
