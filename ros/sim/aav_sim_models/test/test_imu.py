import math

from aav_sim_models.imu import accel_from_odometry, body_acceleration, Speedometer
import pytest


def test_the_acceleration_comes_from_the_simulator_unless_asked():
    assert not accel_from_odometry('simulator')
    assert accel_from_odometry('odometry')
    with pytest.raises(ValueError, match='accel_from'):
        accel_from_odometry('wheels')


def test_straight_line_is_the_speed_change():
    assert body_acceleration(1.2, 3.0, 0.0) == (1.2, 0.0)
    assert body_acceleration(-2.5, 3.0, 0.0, x=0.5) == (-2.5, 0.0)


def test_a_steady_left_turn_pulls_left():
    # 1 m/s round a 2.87 m circle (full lock on a 1.57 m wheelbase): r = 0.348 rad/s.
    r = 1.0 / 2.87
    fwd, left = body_acceleration(0.0, 1.0, r)
    assert left == pytest.approx(1.0 / 2.87)          # v^2 / R
    assert fwd == 0.0
    # Ahead of the axle, the point also swings round: a small pull back toward the centre.
    fwd, left = body_acceleration(0.0, 1.0, r, x=0.5)
    assert fwd == pytest.approx(-0.5 * r * r)
    assert math.hypot(fwd, left) == pytest.approx(r * r * math.hypot(2.87, 0.5))


def test_right_turn_and_a_growing_yaw_rate():
    assert body_acceleration(0.0, 2.0, -0.1)[1] == pytest.approx(-0.2)
    assert body_acceleration(0.0, 0.0, 0.0, yaw_accel=0.4, x=0.5)[1] == pytest.approx(0.2)


def test_the_speedometer_differentiates_the_simulators_odometry():
    s = Speedometer()
    assert not s.ready
    s.update(0.0, 10.00)
    assert s.ready and s.accel == 0.0             # one message: no change seen yet
    s.update(0.02, 10.02)                         # 50 Hz, speeding up at 1 m/s^2
    assert s.accel == pytest.approx(1.0)
    s.update(0.02, 10.02)                         # the same stamp again: keep the last
    assert s.accel == pytest.approx(1.0)
    s.update(0.0, 10.04)
    assert (s.speed, s.accel) == (0.0, pytest.approx(-1.0))
    s.update(0.0, 0.5)                            # the world was reset: the clock went back
    assert s.accel == 0.0
