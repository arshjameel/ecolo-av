"""
The preflight's steps and checks, on the pretend car: on the ground, and as if on stands.

fake_car drives the steps through ego's model and Gazebo's actuator model. On the ground
its IMU and GPS follow the car; for stands the same rows get a still IMU and a still GPS.
"""

import math
import random

from aav_actuator_tests import preflight as pf
from aav_actuator_tests import procedures as pr
from aav_ego.ackermann import Limits
import fake_car
import pytest

LAT0, LON0 = 45.3850, -75.6950   # Carleton
CAP = 0.3
SETTINGS = pr.Settings(cap=CAP, max_steering=0.5, creep=0.2)


def with_gps(rows, still=False, seed=1):
    """Give the rows a GPS: where the car is (or stays), 0.3 m of noise."""
    rng = random.Random(seed)
    for r in rows:
        x, y = (0.0, 0.0) if still else (r['x'], r['y'])
        x, y = x + rng.gauss(0, 0.3), y + rng.gauss(0, 0.3)
        r['lat'] = LAT0 + math.degrees(x / pf.EARTH_RADIUS)
        r['lon'] = LON0 + math.degrees(y / (pf.EARTH_RADIUS * math.cos(math.radians(LAT0))))
    return rows


def ground_rows():
    p = pr.plan('preflight', SETTINGS)
    rows, runner = fake_car.drive(p.steps, limits=Limits(max_speed=CAP))
    assert not [d.step.phase for d in runner.done if d.timed_out]
    assert rows[-1]['t'] < p.time and rows[-1]['distance'] < p.distance
    return with_gps(rows)


def stands_rows():
    """Return the same drive, on stands: the wheels spin, the IMU feels nothing, the GPS stays."""
    rng = random.Random(2)
    rows = ground_rows()
    for r in rows:
        r['ax'] = 0.05 + rng.gauss(0, 0.2)   # a bias and noise, no push
        r['ay'] = rng.gauss(0, 0.2)
    return with_gps(rows, still=True)


def by_check(lines):
    return {line.check.split(' (')[0]: line for line in lines}


def test_the_steps_in_order():
    steps = pr.preflight(SETTINGS)
    assert [s.phase for s in steps] == [pf.STILL, pf.SETTLE, pf.SPEED_UP, pf.HOLD, pf.SLOW,
                                        pf.BRAKE, pf.LEFT, pf.RIGHT, pf.CENTRE, 'stop']
    assert max(s.speed for s in steps) == CAP
    # Speeding up and braking must finish: a time-out stops the test.
    assert {s.phase for s in steps if s.must} == {pf.SPEED_UP, pf.BRAKE, 'stop'}
    assert pr.TESTS['preflight'].stage == 3   # a stands test: the car needs --stands


def test_motion_passes_on_the_pretend_car():
    lines = pf.motion_lines(ground_rows(), CAP, 0.5)
    assert all(x.status is pf.Status.PASS for x in lines), pf.table(lines)


def test_no_steering_sensor_asks_the_person():
    rows = ground_rows()
    for r in rows:
        r['steer'] = float('nan')
    assert pf.steer_line(rows, 0.5, seen=None).status is pf.Status.SKIP
    assert pf.steer_line(rows, 0.5, seen=True).status is pf.Status.PASS
    assert pf.steer_line(rows, 0.5, seen=False).status is pf.Status.FAIL


def test_steering_the_wrong_way_fails():
    rows = ground_rows()
    for r in rows:
        r['steer'] = -r['steer']
    assert pf.steer_line(rows, 0.5, seen=None).status is pf.Status.FAIL


def test_wheels_that_never_spin_fail():
    rows = ground_rows()
    for r in rows:
        r['speed'] = 0.0
    lines = by_check(pf.motion_lines(rows, CAP, 0.5))
    assert lines['speeds up'].status is pf.Status.FAIL
    assert lines['holds the speed'].status is pf.Status.FAIL


def test_ground_agrees_and_stands_disagree():
    ground = pf.stands_lines(ground_rows(), ground=True)
    assert all(x.status is pf.Status.PASS for x in ground), pf.table(ground)
    assert all('sim: on the ground' in x.check for x in ground)
    stands = pf.stands_lines(stands_rows(), ground=False)
    assert stands[0].status is pf.Status.PASS, pf.table(stands)
    # Without Doppler a still GPS can't prove the stands: a WARN, never a PASS.
    assert stands[1].status is pf.Status.WARN and 'consistent' in stands[1].detail


def with_doppler(rows):
    for r in rows:
        r['gps_speed'] = abs(r['speed']) + random.Random(int(r['t'] * 50)).gauss(0, 0.02)
    return rows


def test_doppler_decides_the_gps_line():
    stands = stands_rows()
    for r in stands:
        r['gps_speed'] = 0.02
    assert pf.stands_lines(stands, ground=False)[1].status is pf.Status.PASS
    moving = with_doppler(ground_rows())
    assert pf.stands_lines(moving, ground=False)[1].status is pf.Status.FAIL
    assert pf.stands_lines(moving, ground=True)[1].status is pf.Status.PASS


def test_the_wrong_expectation_fails_both_ways():
    # The car moving while "on stands" (it fell off, or isn't lifted) must fail ...
    lines = pf.stands_lines(ground_rows(), ground=False)
    assert lines[0].status is pf.Status.FAIL
    assert lines[1].status is pf.Status.WARN and 'MOVED' in lines[1].detail
    # ... and a still IMU and GPS in the simulator mean the comparison is broken.
    assert all(x.status is pf.Status.FAIL for x in pf.stands_lines(stands_rows(), ground=True))


def test_an_imu_turned_sideways_still_counts():
    rows = ground_rows()
    for r in rows:
        r['ax'], r['ay'] = 0.0, r['ax']
    assert pf.stands_lines(rows, ground=True)[0].status is pf.Status.PASS


def imu(t, a=(0.0, 0.0, pf.G), g=(0.0, 0.0, 0.0), rng=None):
    n = (lambda: rng.gauss(0, 0.02)) if rng else (lambda: 0.0)
    return {'ax': a[0] + n(), 'ay': a[1] + n(), 'az': a[2] + n(),
            'gx': g[0] + n(), 'gy': g[1] + n(), 'gz': g[2] + n()}


def samples(rate, seconds, make, start=0.0):
    s = pf.Samples()
    for i in range(int(rate * seconds)):
        t = start + i / rate
        s.add(t, make(t))
    return s


SENSOR = {s.name: s for s in pf.SENSORS}


def test_a_good_imu_passes():
    rng = random.Random(3)
    s = samples(200, 10, lambda t: imu(t, rng=rng))   # the Yahboom's 200 Hz
    line = pf.sensor_line(SENSOR['imu'], s, 0.0, 10.0, [(0.0, 2.0)], 0.5, False)
    assert line.status is pf.Status.PASS, line
    # The same at Gazebo's 50 Hz: fine for a stand-in, too slow for the car.
    s = samples(50, 10, lambda t: imu(t, rng=rng))
    assert pf.sensor_line(SENSOR['imu'], s, 0.0, 10.0, [(0.0, 2.0)], 0.5, True).status \
        is pf.Status.PASS
    slow = pf.sensor_line(SENSOR['imu'], s, 0.0, 10.0, [(0.0, 2.0)], 0.5, False)
    assert slow.status is pf.Status.FAIL and 'Yahboom' in slow.detail


@pytest.mark.parametrize('make, why', [
    (lambda t: imu(t), 'frozen'),
    (lambda t: imu(t, a=(0.0, 0.0, 0.0), rng=random.Random(int(t * 100))), 'at rest'),
    (lambda t: imu(t, g=(0.0, 0.0, 0.5), rng=random.Random(int(t * 100))), 'turning'),
])
def test_a_bad_imu_fails(make, why):
    line = pf.sensor_line(SENSOR['imu'], samples(200, 10, make), 0.0, 10.0, [(0.0, 2.0)],
                          0.5, False)
    assert line.status is pf.Status.FAIL and why in line.detail, line


def test_slow_silent_and_missing_sensors_fail():
    gps = lambda t: {'status': 0, 'lat': LAT0, 'lon': LON0}   # noqa: E731
    assert pf.sensor_line(SENSOR['gps'], samples(5, 10, gps), 0, 10, [], 0.5, True).status \
        is pf.Status.PASS
    slow = pf.sensor_line(SENSOR['lidar'], samples(2, 10, lambda t: {'points': 100}),
                          0, 10, [], 0.5, True)
    assert slow.status is pf.Status.FAIL and 'below' in slow.detail
    # 10 Hz, but only for the first 3 s of a 10 s run: it died.
    died = pf.sensor_line(SENSOR['odom'], samples(10, 3, lambda t: {'speed': 0.0}),
                          0, 10, [], 0.5, True)
    assert died.status is pf.Status.FAIL and 'silent' in died.detail
    gone = pf.sensor_line(SENSOR['camera'], None, 0, 10, [], 0.5, True)
    assert gone.status is pf.Status.FAIL and '--skip camera' in gone.detail
    nofix = pf.sensor_line(SENSOR['gps'], samples(5, 10, lambda t: {
        'status': -1, 'lat': float('nan'), 'lon': float('nan')}), 0, 10, [], 0.5, True)
    assert nofix.status is pf.Status.FAIL and 'no fix' in nofix.detail


def test_skipped_sensors_are_skipped_not_passed():
    lines = pf.sensor_lines({}, 0, 10, [], 0.5, True, skip=['gps', 'lidar'])
    status = {x.check.split(' ')[0]: x.status for x in lines}
    assert status['gps'] is pf.Status.SKIP and status['lidar'] is pf.Status.SKIP
    assert status['imu'] is pf.Status.FAIL   # nothing else is skipped by itself
    assert not pf.passed(lines)


def test_an_empty_radar_is_fine_an_empty_lidar_is_not():
    empty = lambda t: {'points': 0}   # noqa: E731
    assert pf.sensor_line(SENSOR['radar'], samples(10, 10, empty), 0, 10, [], 0.5, True).status \
        is pf.Status.PASS
    assert pf.sensor_line(SENSOR['lidar'], samples(10, 10, empty), 0, 10, [], 0.5, True).status \
        is pf.Status.FAIL


def test_steering_beyond_full_lock_fails():
    s = samples(20, 10, lambda t: {'measured': 0.9})
    assert pf.sensor_line(SENSOR['steering'], s, 0, 10, [], 0.5, True).status is pf.Status.FAIL


def test_the_table_says_the_verdict():
    good = [pf.Line(pf.Status.PASS, 'a', ''), pf.Line(pf.Status.SKIP, 'b', '')]
    assert 'PREFLIGHT PASSED' in pf.table(good) and pf.passed(good)
    bad = good + [pf.Line(pf.Status.FAIL, 'c', 'why')]
    assert 'PREFLIGHT FAILED: 1 check failed' in pf.table(bad) and not pf.passed(bad)
    assert pf.counts(bad) == {'PASS': 1, 'FAIL': 1, 'SKIP': 1, 'WARN': 0}


def test_still_windows_find_the_waits():
    rows = ground_rows()
    still = pf.still_windows(rows)
    assert still and still[0][0] == 0.0
    assert all(abs(r['speed']) < 0.05 for r in rows if pf.inside(r['t'], still))


def test_open_loop_hold_is_a_finding_not_a_fault():
    rows = ground_rows()
    for r in rows:
        if r['phase'] == pf.HOLD:
            r['speed'] *= 0.5   # the throttle gives half the speed asked
    closed = by_check(pf.motion_lines(rows, CAP, 0.5))['holds the speed']
    opened = by_check(pf.motion_lines(rows, CAP, 0.5, open_loop=True))['holds the speed']
    assert closed.status is pf.Status.FAIL
    assert opened.status is pf.Status.WARN and 'full_throttle_speed' in opened.detail


def test_centre_uses_the_offset():
    rows = ground_rows()
    for r in rows:
        if r['phase'] == pf.CENTRE:
            r['steer'] += 0.05
    assert pf.steer_line(rows, 0.5).status is pf.Status.FAIL
    assert pf.steer_line(rows, 0.5, centre=0.05, tolerance=0.01).status is pf.Status.PASS


def test_the_imu_window_stops_at_95_percent():
    rows = ground_rows()
    w = pf.speed_up_window(rows)
    took = w[-1]['t'] - w[0]['t']
    assert w and took <= pf.IMU_WINDOW
    # It ends at 95% of the command, or after 3 s, whichever comes first.
    assert abs(w[-1]['speed']) >= 0.95 * CAP or took >= pf.IMU_WINDOW - 0.05
    assert all(abs(r['speed']) < 0.95 * CAP for r in w[:-1])


def watch_rows(rows):
    watch = pf.ImuWatch()
    for r in rows:
        if r['phase'] in (pf.STILL, pf.SETTLE):
            watch.rest(r['ax'], r['ay'])
    assert watch.start()
    for r in rows:
        if r['phase'] in (pf.STILL, pf.SETTLE):
            continue
        why = watch.add(r['t'], r['ax'], r['ay'], r['gz'],
                        r['phase'] in (pf.LEFT, pf.RIGHT, pf.CENTRE))
        if why:
            return why
    return None


def test_the_live_imu_watch():
    for r in (rows := stands_rows()):
        r['gz'] = 0.0
    assert watch_rows(rows) is None                      # on stands: quiet
    assert 'off the stands' in watch_rows(ground_rows())  # rolling: stop
    for r in rows:
        if r['phase'] == pf.LEFT:
            r['gz'] = 0.3
    assert 'not on stands' in watch_rows(rows)
    assert not pf.ImuWatch().start()                     # no rest readings: no watch
