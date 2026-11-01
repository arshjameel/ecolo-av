import math

from aav_localization_health.health import (
    front_wheel_speeds, G, Health, mean_and_sigma, OK, Settings, STALE, summary, WARN)
import pytest

WHEELBASE, TRACK = 1.561, 0.75     # vehicle.yaml, 31 Oct


def by_name(statuses):
    """Return the report as {check: Status}."""
    return {s.name: s for s in statuses}


def parked(h, seconds=8.0, wz=0.0, accel=(0.0, 0.0, G), start=0.0):
    """Feed a parked car at 50 Hz: wheels at 0 m/s, the IMU as given, the local EKF."""
    t = start
    while t < start + seconds:
        h.odom(t, t, 'odom', 'base_link', 0.0)
        h.imu(t, t, 'imu_link', wz, accel)
        h.ekf('local', t, t, 'odom', 'base_link', 0.0, 0.0, 0.0, 0.0)
        t += 0.02
    return t


def test_front_wheels_turn_apart_in_a_bend():
    left, right = front_wheel_speeds(2.0, 0.0, WHEELBASE, TRACK)
    assert left == pytest.approx(2.0) and right == pytest.approx(2.0)
    left, right = front_wheel_speeds(2.0, 0.5, WHEELBASE, TRACK)
    assert right > left                     # a left turn: the right wheel goes further


def test_mean_and_sigma_of_the_mean():
    assert mean_and_sigma([1.0]) == (1.0, 0.0)
    mean, sigma = mean_and_sigma([1.0, 3.0])
    assert mean == 2.0 and sigma == pytest.approx(1.0)


def test_a_healthy_parked_car_is_ok():
    h = Health(WHEELBASE, TRACK)
    now = parked(h)
    report = by_name(h.report(now))
    assert report['gyro_bias'].level == OK
    assert report['imu_mounting'].level == OK
    assert report['gps_quality'].message == 'not on this profile'   # no GPS on the bench
    assert summary(h.report(now)) == OK


def test_gyro_and_gravity_wait_until_parked_long_enough():
    h = Health(WHEELBASE, TRACK)
    now = parked(h, seconds=2.0, wz=math.radians(1.0))
    assert 'judged after' in by_name(h.report(now))['gyro_bias'].message


def test_a_gyro_bias_warns_only_when_it_stands_out():
    h = Health(WHEELBASE, TRACK)
    now = parked(h, wz=math.radians(0.6))
    gyro = by_name(h.report(now))['gyro_bias']
    assert gyro.level == WARN and gyro.values['bias_dps'] == pytest.approx(0.6)
    h = Health(WHEELBASE, TRACK)
    now = parked(h, wz=math.radians(0.1))                   # under gyro_bias_warn
    assert by_name(h.report(now))['gyro_bias'].level == OK


@pytest.mark.parametrize('accel, words', [
    ((0.0, 0.0, -G), 'upside down'),
    ((0.0, 0.0, G / 9.80665), 'gravity reads'),          # wrong units: g, not m/s^2
    ((G * math.sin(0.35), 0.0, G * math.cos(0.35)), 'tilted'),
])
def test_imu_mounting_problems(accel, words):
    h = Health(WHEELBASE, TRACK)
    now = parked(h, accel=accel)
    mount = by_name(h.report(now))['imu_mounting']
    assert mount.level == WARN and words in mount.message


def test_moving_clears_the_parked_samples():
    h = Health(WHEELBASE, TRACK)
    t = parked(h, wz=math.radians(0.6))
    h.odom(t, t, 'odom', 'base_link', 1.0)
    assert not h.parked


@pytest.mark.parametrize('status, cov_type, sigma, level, words', [
    (-1, 2, 1.0, WARN, 'no fix'),
    (0, 0, 1.0, WARN, 'covariance unknown'),
    (0, 2, 9.0, WARN, 'poor fix'),
    (2, 2, 0.001, WARN, 'too sure'),
    (0, 2, 0.5, OK, 'sigma 0.50 m'),
])
def test_gps_quality(status, cov_type, sigma, level, words):
    h = Health(WHEELBASE, TRACK)
    t = parked(h, seconds=0.1)
    cov = [sigma ** 2, 0, 0, 0, sigma ** 2, 0, 0, 0, (2 * sigma) ** 2]
    h.gps_fix(t, t, 'gps_link', status, cov_type, cov)
    gps = by_name(h.report(t))['gps_quality']
    assert gps.level == level and words in gps.message


def test_late_and_future_stamps_warn():
    h = Health(WHEELBASE, TRACK)
    t = parked(h, seconds=0.1)
    h.imu(t, t - 1.0, 'imu_link', 0.0, (0.0, 0.0, G))         # a second behind
    report = by_name(h.report(t))['timestamps']
    assert report.level == WARN and 'imu stamped +1.00 s off' in report.message
    h.imu(t, t + 0.5, 'imu_link', 0.0, (0.0, 0.0, G))         # from the future
    assert 'imu stamped -0.50 s off' in by_name(h.report(t))['timestamps'].message
    h.imu(t, t, 'imu_link', 0.0, (0.0, 0.0, G))
    assert by_name(h.report(t))['timestamps'].level == OK


def test_a_wrong_frame_warns_until_it_is_right():
    h = Health(WHEELBASE, TRACK)
    t = parked(h, seconds=0.1)
    h.odom(t, t, 'odom', 'base_footprint', 0.0)
    frames = by_name(h.report(t))['frames']
    assert frames.level == WARN and 'odom -> base_footprint' in frames.message
    h.odom(t, t, 'odom', 'base_link', 0.0)
    assert by_name(h.report(t))['frames'].level == OK


def test_tf():
    h = Health(WHEELBASE, TRACK)
    t = parked(h, seconds=0.1)
    assert by_name(h.report(t))['tf'].message == 'not looked yet'
    h.tf(['gps_link'], 0.1)
    assert 'gps_link' in by_name(h.report(t))['tf'].message
    h.tf([], None)
    assert by_name(h.report(t))['tf'].level == WARN
    h.tf([], 0.05)
    assert by_name(h.report(t))['tf'].level == OK


def driving(h, seconds, v, wz, left, right, start):
    """Feed a car driving at v and wz, its front wheels reading left and right."""
    t = start
    while t < start + seconds:
        h.odom(t, t, 'odom', 'base_link', v)
        h.imu(t, t, 'imu_link', wz, (0.0, 0.0, G))
        h.ekf('local', t, t, 'odom', 'base_link', v * t, 0.0, v, wz)
        h.wheels(t, t, 'base_link', left, right)
        t += 0.02
    return t


def test_front_wheels_that_agree_with_the_turn_are_ok():
    h = Health(WHEELBASE, TRACK)
    left, right = front_wheel_speeds(2.0, 0.4, WHEELBASE, TRACK)
    t = driving(h, 2.0, 2.0, 0.4, left, right, 0.0)
    assert by_name(h.report(t))['wheels'].level == OK


def test_a_dead_wheel_warns_after_it_persists():
    h = Health(WHEELBASE, TRACK)
    t = driving(h, 0.5, 2.0, 0.0, 2.0, 0.0, 0.0)        # the right one reads 0
    assert by_name(h.report(t))['wheels'].level == OK   # not yet: persist is 1 s
    t = driving(h, 1.0, 2.0, 0.0, 2.0, 0.0, t)
    assert by_name(h.report(t))['wheels'].level == WARN


def test_a_runaway_or_jumping_local_ekf_warns_for_a_while():
    h = Health(WHEELBASE, TRACK)
    t = parked(h, seconds=0.1)
    h.ekf('local', t, t, 'odom', 'base_link', 0.0, 0.0, 9.0, 0.0)          # 9 m/s
    assert 'ekf_local: 9.0 m/s' in by_name(h.report(t))['motion'].message
    assert by_name(h.report(t + 6.0))['motion'].level != WARN              # hold is 5 s
    h = Health(WHEELBASE, TRACK)
    t = parked(h, seconds=0.1)
    h.ekf('local', t + 0.1, t + 0.1, 'odom', 'base_link', 10.0, 0.0, 0.0, 0.0)
    assert 'jumped' in by_name(h.report(t + 0.1))['motion'].message


def test_the_global_ekf_may_jump():
    h = Health(WHEELBASE, TRACK)
    t = parked(h, seconds=0.1)
    h.ekf('global', t, t, 'map', 'base_link', 0.0, 0.0, 0.0, 0.0)
    h.ekf('global', t + 0.1, t + 0.1, 'map', 'base_link', 10.0, 0.0, 0.0, 0.0)
    assert by_name(h.report(t + 0.1))['motion'].level == OK


def test_a_quiet_required_input_is_stale():
    h = Health(WHEELBASE, TRACK)
    assert by_name(h.report(0.0))['gyro_bias'].level == STALE          # no imu yet
    t = parked(h, seconds=0.1)
    report = by_name(h.report(t + Settings().stale_after + 1.0))
    assert report['gyro_bias'].level == STALE and 'nothing on' in report['gyro_bias'].message
    assert summary(h.report(t + 5.0)).startswith(WARN)
