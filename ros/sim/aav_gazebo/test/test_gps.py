import math
import random

from aav_gazebo.gps import antenna_velocity, noisy
import pytest


@pytest.mark.parametrize('yaw', [0.0, 1.0, -2.5])
def test_over_the_rear_axle_the_velocity_points_along_the_car(yaw):
    east, north = antenna_velocity(yaw, 0.5, 0.3, (0.0, 0.0))
    assert math.atan2(north, east) == pytest.approx(yaw)
    assert math.hypot(east, north) == pytest.approx(0.5)


def test_reversing_points_the_other_way():
    east, north = antenna_velocity(0.0, -0.5, 0.0, (0.0, 0.0))
    assert (east, north) == pytest.approx((-0.5, 0.0))


def test_an_antenna_ahead_swings_out_in_a_left_turn():
    # Facing east, turning left: an antenna 0.8 m ahead moves north too.
    east, north = antenna_velocity(0.0, 1.0, 0.5, (0.8, 0.0))
    assert (east, north) == pytest.approx((1.0, 0.4))


def test_a_seeded_noise_repeats():
    # gazebo_gps's seed parameter: the same seed, the same velocity noise, run after run.
    a, b = random.Random(5), random.Random(5)
    first = [noisy(1.0, 0.05, a) for _ in range(5)]
    assert first == [noisy(1.0, 0.05, b) for _ in range(5)]
    assert len(set(first)) == 5
