"""The map -> odom that puts the car on the simulator's truth (aav_sim_models.pose_frames)."""

import math

from aav_sim_models.pose_frames import compose, inverse, map_to_odom, quaternion_of, yaw_of
import pytest


def test_inverse_undoes_a_pose():
    a = (3.0, -2.0, 0.7)
    assert compose(a, inverse(a)) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert compose(inverse(a), a) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_map_to_odom_lands_on_the_truth():
    truth = (120.0, -80.0, -0.6)
    odom_base = (14.0, 3.0, 0.4)      # ekf_local has drifted its own way
    m2o = map_to_odom(truth, odom_base)
    assert compose(m2o, odom_base) == pytest.approx(truth, abs=1e-9)


def test_no_drift_gives_the_start():
    start = (5.0, 2.0, math.pi / 2)
    assert map_to_odom(start, (0.0, 0.0, 0.0)) == pytest.approx(start)


def test_yaw_of_a_quaternion():
    assert yaw_of(math.sin(0.25), math.cos(0.25)) == pytest.approx(0.5)
    assert yaw_of(0.0, 1.0) == 0.0
    assert yaw_of(*quaternion_of(-2.5)) == pytest.approx(-2.5)
